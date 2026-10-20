package physics

import (
    "math"
    "math/geom"
)

/// JointKind is what a joint lets turn.
public enum JointKind {
    /// Turns any way, within its Limits if it has them: hips, shoulders,
    /// necks.
    case ball
    /// Turns only about the frame's X axis, between HingeMin and
    /// HingeMax: knees, elbows, doors.
    case hinge
    /// Doesn't turn at all.
    case weld
}

/// Limits bound a ball joint's turn, as B's frame seen from A's: a twist
/// about the frame's Y axis and a swing of that axis, split into its
/// parts about X and about Z (radians). The swing stays inside the
/// ellipse those make, quadrant by quadrant: a cone when they're equal.
public struct Limits {
    public var TwistMin: float32
    public var TwistMax: float32
    public var SwingXMin: float32
    public var SwingXMax: float32
    public var SwingZMin: float32
    public var SwingZMax: float32

    public init(twist: float32, swingX: float32, swingZ: float32) {
        TwistMin = -twist
        TwistMax = twist
        SwingXMin = -swingX
        SwingXMax = swingX
        SwingZMin = -swingZ
        SwingZMax = swingZ
    }

    public init(twistMin: float32, twistMax: float32, swingXMin: float32, swingXMax: float32,
                swingZMin: float32, swingZMax: float32) {
        TwistMin = twistMin
        TwistMax = twistMax
        SwingXMin = swingXMin
        SwingXMax = swingXMax
        SwingZMin = swingZMin
        SwingZMax = swingZMax
    }
}

/// Joint holds a point of one body to a point of another, and limits how
/// they turn about it. Each body sees the joint as a frame in its own
/// space; the joint keeps the two frames' origins together and their
/// relative rotation within the limits.
public final class Joint {
    public let A: Body
    public let B: Body
    /// The joint's frame in A's space, and in B's.
    public var FrameA: geom.Transform
    public var FrameB: geom.Transform
    public var Kind: JointKind
    /// For a ball joint; none lets it turn freely.
    public var Limits: Limits? = nil
    /// For a hinge, radians about the frame's X axis.
    public var HingeMin: float32 = -float32(math.Pi)
    public var HingeMax: float32 = float32(math.Pi)
    /// Meters per newton of give at the joint's point; 0 is rigid.
    public var Compliance: float32 = 0
    /// Slows the bodies' relative turning, per second: a little keeps a
    /// limp chain from flailing.
    public var Damping: float32 = 0
    /// A rotation of B's frame in A's that the joint pulls towards, with
    /// DriveCompliance's give (radians per newton-meter): muscles for an
    /// active ragdoll, a motor for a door.
    public var Drive: geom.Quat? = nil
    public var DriveCompliance: float32 = 0.01
    /// Whether A and B still collide with each other.
    public var Collide = false
    public var Enabled = true

    /// A joint between a and b at `frame`, a place in the world, taken as
    /// each body's frame from where they are now.
    public init(_ kind: JointKind, _ a: Body, _ b: Body, at frame: geom.Transform) {
        Kind = kind
        A = a
        B = b
        FrameA = a.Transform.Inverse() * frame
        FrameB = b.Transform.Inverse() * frame
    }

    /// Relative is B's frame turned from A's, as a rotation in A's frame.
    public var Relative: geom.Quat {
        let qa = A.Rotation * FrameA.Rotation, qb = B.Rotation * FrameB.Rotation
        return (qa.Conjugate() * qb).Normalized()
    }

    /// Anchors is where each body holds the joint, in the world: together,
    /// unless something pulls them apart.
    public var Anchors: (geom.Vec3, geom.Vec3) {
        return (A.Transform.Point(FrameA.Position), B.Transform.Point(FrameB.Position))
    }

    // solve corrects positions and rotations for one substep.
    func solve(_ h: float32) {
        // Keep the turn within bounds first, then the anchors together,
        // so what's left over is a little angle rather than a gap.
        let r = Relative
        var target = r
        switch Kind {
        case .weld:
            target = geom.Quat.Identity
        case .hinge:
            let st = r.SwingTwist(geom.Vec3.UnitX)
            let a = math.Clamp(twistAngle(st.twist, axis: 0), HingeMin, HingeMax)
            target = geom.AxisAngle(geom.Vec3.UnitX, a)
        case .ball:
            if let l = Limits { target = clamp(r, l) }
        }
        correct(r, target, 0, h)
        if let d = Drive { correct(Relative, d, DriveCompliance, h) }

        let ra = A.Rotation.Rotate(FrameA.Position), rb = B.Rotation.Rotate(FrameB.Position)
        let pa = A.Position + ra, pb = B.Position + rb
        _ = positional(A, B, pb - pa, ra, rb, Compliance, h)
    }

    // correct turns the bodies so B's frame, now at r in A's, moves to
    // target (with give).
    func correct(_ r: geom.Quat, _ target: geom.Quat, _ compliance: float32, _ h: float32) {
        let fix = (target * r.Conjugate()).Normalized()
        if math.Abs(fix.W) > 0.9999999 { return }
        // That turn, in the world: about A's frame.
        let qa = A.Rotation * FrameA.Rotation
        let world = rotationVector(fix)
        let d = qa.Rotate(world)
        // B turns by d, A against it.
        angular(A, B, -d, compliance, h)
    }

    // damp slows the relative spin.
    func damp(_ h: float32) {
        if Damping <= 0 { return }
        let rel = B.AngularVelocity - A.AngularVelocity
        let k = math.Min(Damping * h, 1)
        let dw = rel * k
        let n = dw.Length()
        if n < 1e-9 { return }
        let axis = dw / n
        let w = A.angularWeight(axis) + B.angularWeight(axis)
        if w <= 0 { return }
        let p = dw / w
        if A.Dynamic { A.AngularVelocity += A.inverseInertia(p) }
        if B.Dynamic { B.AngularVelocity -= B.inverseInertia(p) }
    }
}

// twistAngle is the signed angle of a rotation about one axis (0 X, 1 Y).
func twistAngle(_ q: geom.Quat, axis: int) -> float32 {
    let c = axis == 0 ? q.X : (axis == 1 ? q.Y : q.Z)
    let s: float32 = q.W < 0 ? -1 : 1
    return 2 * math.Atan2(c * s, q.W * s)
}

// clamp brings a relative rotation within a ball joint's limits.
func clamp(_ r: geom.Quat, _ l: Limits) -> geom.Quat {
    let st = r.SwingTwist(geom.Vec3.UnitY)
    let twist = math.Clamp(twistAngle(st.twist, axis: 1), l.TwistMin, l.TwistMax)
    // The swing as a rotation vector, which has no Y part.
    // Within the ellipse whose half-axes are the limits on the side the
    // swing is on: a cone when they're equal, not a box.
    let v = rotationVector(st.swing)
    let ex = v.X >= 0 ? l.SwingXMax : -l.SwingXMin, ez = v.Z >= 0 ? l.SwingZMax : -l.SwingZMin
    var sx = v.X, sz = v.Z
    if ex <= 1e-6 { sx = 0 }
    if ez <= 1e-6 { sz = 0 }
    let fx = ex > 1e-6 ? sx / ex : 0, fz = ez > 1e-6 ? sz / ez : 0
    let k = fx * fx + fz * fz
    if k > 1 {
        let s = 1 / math.Sqrt(k)
        sx *= s
        sz *= s
    }
    let sv = geom.Vec3(sx, 0, sz)
    let angle = sv.Length()
    let swing = angle > 1e-9 ? geom.AxisAngle(sv / angle, angle) : geom.Quat.Identity
    return (swing * geom.AxisAngle(geom.Vec3.UnitY, twist)).Normalized()
}
