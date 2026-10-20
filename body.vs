// Package physics is rigid-body dynamics: bodies with mass and shape,
// joints between them with angle limits and drives, and contacts with
// friction, stepped by an XPBD solver (Müller et al., "Detailed Rigid
// Body Simulation with Extended Position Based Dynamics", 2020): many
// small substeps, each moving bodies freely and then correcting
// positions and rotations straight onto their constraints. It stays
// stable with stiff joints and long chains, which is what ragdolls need.
//
// Shapes and contacts come from physics/collide. Conventions are
// math/geom's: right-handed, Y up, meters, kilograms, seconds.
package physics

import (
    "math"
    "math/geom"
    "physics/collide"
)

/// BodyKind is how a body moves.
public enum BodyKind {
    /// Moved by gravity, contacts and joints.
    case dynamic
    /// Moved only by the program (set Position, Rotation and the
    /// velocities); pushes dynamic bodies but isn't pushed.
    case kinematic
    /// Never moves: the ground, walls.
    case fixed
}

/// Body is a rigid body.
public final class Body {
    public var Kind: BodyKind
    public var Shape: collide.Shape
    /// The center of mass, in the world.
    public var Position: geom.Vec3
    public var Rotation: geom.Quat
    public var Velocity = geom.Vec3.Zero
    /// Radians per second about each world axis.
    public var AngularVelocity = geom.Vec3.Zero
    /// Kilograms; 0 for kinematic and fixed bodies.
    public private(set) var Mass: float32 = 0
    /// Coulomb friction, 0…1 and up. A contact uses the mean of its two.
    public var Friction: float32 = 0.6
    /// Bounciness, 0…1. A contact uses the larger of its two.
    public var Restitution: float32 = 0.05
    /// The fraction of velocity lost per second, linear and angular.
    public var LinearDamping: float32 = 0.02
    public var AngularDamping: float32 = 0.05
    /// Collision filtering: two bodies collide when each one's Group is in
    /// the other's Mask.
    public var Group: uint32 = 1
    public var Mask: uint32 = 0xFFFFFFFF
    /// A disabled body is left out of the world's steps: it doesn't move,
    /// collide or hold its joints.
    public var Enabled = true
    public var Name: string

    var invMass: float32 = 0
    var invInertia = geom.Vec3.Zero {
        didSet { iiFor = geom.Quat(0, 0, 0, 0) }
    }
    // The inverse inertia turned into the world, as rows of a matrix, and
    // the rotation it was worked out for: rebuilt when the body turns.
    var iiFor = geom.Quat(0, 0, 0, 0)
    var ii0 = geom.Vec3.Zero
    var ii1 = geom.Vec3.Zero
    var ii2 = geom.Vec3.Zero
    var prevPosition = geom.Vec3.Zero
    var prevRotation = geom.Quat.Identity
    var index = -1

    /// A body of `mass` kilograms (ignored unless dynamic), placed at `at`.
    public init(_ kind: BodyKind, _ shape: collide.Shape, mass: float32 = 1, at t: geom.Transform = geom.Transform.Identity,
                name: string = "") {
        Kind = kind
        Shape = shape
        Position = t.Position
        Rotation = t.Rotation
        Name = name
        SetMass(mass)
    }

    /// SetMass sets the mass, and the inertia of the shape solid at that mass.
    public func SetMass(_ m: float32) {
        if Kind != .dynamic || m <= 0 {
            Mass = 0
            invMass = 0
            invInertia = geom.Vec3.Zero
            return
        }
        Mass = m
        invMass = 1 / m
        let i = collide.Inertia(Shape, mass: m)
        invInertia = geom.Vec3(i.X > 0 ? 1 / i.X : 0, i.Y > 0 ? 1 / i.Y : 0, i.Z > 0 ? 1 / i.Z : 0)
    }

    /// Transform is where the body is.
    public var Transform: geom.Transform {
        get { return geom.Transform(position: Position, rotation: Rotation) }
        set {
            Position = newValue.Position
            Rotation = newValue.Rotation.Normalized()
        }
    }

    /// Dynamic is whether forces move it.
    public var Dynamic: bool { return Kind == .dynamic && invMass > 0 }

    /// VelocityAt is the velocity of the body's point p (in the world).
    public func VelocityAt(_ p: geom.Vec3) -> geom.Vec3 {
        return Velocity + AngularVelocity.Cross(p - Position)
    }

    /// ApplyImpulse changes the velocities as an impulse j (kg·m/s) at
    /// point p would.
    public func ApplyImpulse(_ j: geom.Vec3, at p: geom.Vec3) {
        if !Dynamic { return }
        Velocity += j * invMass
        AngularVelocity += inverseInertia((p - Position).Cross(j))
    }

    /// KineticEnergy is ½mv² + ½ωᵀIω, in joules.
    public var KineticEnergy: float32 {
        if !Dynamic { return 0 }
        let l = Rotation.Conjugate().Rotate(AngularVelocity)
        let i = geom.Vec3(invInertia.X > 0 ? 1 / invInertia.X : 0, invInertia.Y > 0 ? 1 / invInertia.Y : 0,
                          invInertia.Z > 0 ? 1 / invInertia.Z : 0)
        return 0.5 * Mass * Velocity.LengthSquared() + 0.5 * (l.X * l.X * i.X + l.Y * l.Y * i.Y + l.Z * l.Z * i.Z)
    }

    // inverseInertia is I⁻¹v with the body's inertia turned into the world.
    // It is R·diag(invInertia)·Rᵀ, kept while the rotation stays: the
    // solver asks it several times a constraint, and two quaternion
    // rotations a time cost more than everything else in a step.
    func inverseInertia(_ v: geom.Vec3) -> geom.Vec3 {
        let q = Rotation
        if q.X != iiFor.X || q.Y != iiFor.Y || q.Z != iiFor.Z || q.W != iiFor.W { worldInertia() }
        return geom.Vec3(ii0.X * v.X + ii0.Y * v.Y + ii0.Z * v.Z,
                         ii1.X * v.X + ii1.Y * v.Y + ii1.Z * v.Z,
                         ii2.X * v.X + ii2.Y * v.Y + ii2.Z * v.Z)
    }

    func worldInertia() {
        let q = Rotation
        let x = q.X, y = q.Y, z = q.Z, w = q.W
        // The rotation matrix's rows.
        let r00 = 1 - 2 * (y * y + z * z), r01 = 2 * (x * y - w * z), r02 = 2 * (x * z + w * y)
        let r10 = 2 * (x * y + w * z), r11 = 1 - 2 * (x * x + z * z), r12 = 2 * (y * z - w * x)
        let r20 = 2 * (x * z - w * y), r21 = 2 * (y * z + w * x), r22 = 1 - 2 * (x * x + y * y)
        let a = invInertia.X, b = invInertia.Y, c = invInertia.Z
        let m00 = r00 * r00 * a + r01 * r01 * b + r02 * r02 * c
        let m01 = r00 * r10 * a + r01 * r11 * b + r02 * r12 * c
        let m02 = r00 * r20 * a + r01 * r21 * b + r02 * r22 * c
        let m11 = r10 * r10 * a + r11 * r11 * b + r12 * r12 * c
        let m12 = r10 * r20 * a + r11 * r21 * b + r12 * r22 * c
        let m22 = r20 * r20 * a + r21 * r21 * b + r22 * r22 * c
        ii0 = geom.Vec3(m00, m01, m02)
        ii1 = geom.Vec3(m01, m11, m12)
        ii2 = geom.Vec3(m02, m12, m22)
        iiFor = q
    }

    // rotate turns the body by a small rotation vector (radians about an
    // axis, as an angular velocity times a time).
    func rotate(_ w: geom.Vec3) {
        let dq = geom.Quat(w.X, w.Y, w.Z, 0) * Rotation
        Rotation = geom.Quat(Rotation.X + 0.5 * dq.X, Rotation.Y + 0.5 * dq.Y, Rotation.Z + 0.5 * dq.Z,
                             Rotation.W + 0.5 * dq.W).Normalized()
    }

    // weight is the generalized inverse mass for a push along n at offset r
    // from the center.
    func weight(_ r: geom.Vec3, _ n: geom.Vec3) -> float32 {
        if !Dynamic { return 0 }
        let rn = r.Cross(n)
        return invMass + rn.Dot(inverseInertia(rn))
    }

    func angularWeight(_ n: geom.Vec3) -> float32 {
        if !Dynamic { return 0 }
        return n.Dot(inverseInertia(n))
    }
}

// ---- XPBD corrections ----

// positional moves a's point (offset ra) by d and b's (offset rb) against
// it, shared by their inverse masses; answers the multiplier λ.
func positional(_ a: Body, _ b: Body, _ d: geom.Vec3, _ ra: geom.Vec3, _ rb: geom.Vec3,
                _ compliance: float32, _ h: float32) -> float32 {
    let c = d.Length()
    if c < 1e-9 { return 0 }
    let n = d / c
    let w = a.weight(ra, n) + b.weight(rb, n) + compliance / (h * h)
    if w <= 0 { return 0 }
    let lambda = c / w
    let p = n * lambda
    if a.Dynamic {
        a.Position += p * a.invMass
        a.rotate(a.inverseInertia(ra.Cross(p)))
    }
    if b.Dynamic {
        b.Position -= p * b.invMass
        b.rotate(b.inverseInertia(rb.Cross(-p)))
    }
    return lambda
}

// angular turns a by the rotation vector dphi and b against it, shared by
// their inverse inertias.
func angular(_ a: Body, _ b: Body, _ dphi: geom.Vec3, _ compliance: float32, _ h: float32) {
    let theta = dphi.Length()
    if theta < 1e-9 { return }
    let n = dphi / theta
    let w = a.angularWeight(n) + b.angularWeight(n) + compliance / (h * h)
    if w <= 0 { return }
    let p = n * (theta / w)
    if a.Dynamic { a.rotate(a.inverseInertia(p)) }
    if b.Dynamic { b.rotate(b.inverseInertia(-p)) }
}

// impulse changes the relative velocity of a's point (ra) against b's
// (rb) by dv.
func impulse(_ a: Body, _ b: Body, _ dv: geom.Vec3, _ ra: geom.Vec3, _ rb: geom.Vec3) {
    let m = dv.Length()
    if m < 1e-9 { return }
    let n = dv / m
    let w = a.weight(ra, n) + b.weight(rb, n)
    if w <= 0 { return }
    let p = dv / w
    if a.Dynamic {
        a.Velocity += p * a.invMass
        a.AngularVelocity += a.inverseInertia(ra.Cross(p))
    }
    if b.Dynamic {
        b.Velocity -= p * b.invMass
        b.AngularVelocity -= b.inverseInertia(rb.Cross(p))
    }
}

// rotationVector is the small rotation vector of a unit quaternion: its
// axis times its angle, the shorter way round.
func rotationVector(_ q: geom.Quat) -> geom.Vec3 {
    let s: float32 = q.W < 0 ? -1 : 1
    let v = geom.Vec3(q.X * s, q.Y * s, q.Z * s)
    let sinHalf = v.Length()
    if sinHalf < 1e-9 { return geom.Vec3.Zero }
    let angle = 2 * math.Atan2(sinHalf, q.W * s)
    return v * (angle / sinHalf)
}
