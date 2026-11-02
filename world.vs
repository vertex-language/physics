package physics

import (
    "math"
    "math/geom"
    "physics/collide"
)

/// World is bodies and joints, stepped together.
public final class World {
    public var Gravity = geom.Vec3(0, -9.81, 0)
    /// Substeps per Step: more is stiffer and steadier, and costs more.
    public var Substeps = 12
    /// Passes over the joints each substep: two keeps limits and anchors
    /// both close when they pull against each other.
    public var JointIterations = 2
    /// Contacts start this far apart, so resting bodies don't chatter.
    public var Margin: float32 = 0.002
    public private(set) var Bodies: [Body] = []
    public private(set) var Joints: [Joint] = []
    /// Contacts found in the last substep of the last step.
    public private(set) var ContactCount = 0
    var ignored: [int64: bool] = [:]

    public init() {}

    /// Add puts a body in the world.
    @discardableResult
    public func Add(b: Body) -> Body {
        b.index = Bodies.count
        Bodies.append(b)
        return b
    }

    /// Add puts a joint in the world; its bodies stop colliding with each
    /// other unless the joint says otherwise.
    @discardableResult
    public func Add(j: Joint) -> Joint {
        Joints.append(j)
        if !j.Collide { Ignore(j.A, j.B) }
        return j
    }

    /// Remove takes a body and its joints out of the world.
    public func Remove(b: Body) {
        Bodies = Bodies.filter { $0 !== b }
        Joints = Joints.filter { $0.A !== b && $0.B !== b }
        for (i, x) in Bodies.enumerated() { x.index = i }
        ignored = [:]
        for j in Joints where !j.Collide { Ignore(j.A, j.B) }
    }

    /// Ignore stops two bodies colliding with each other.
    public func Ignore(a: Body, b: Body) {
        ignored[pairKey(a, b)] = true
    }

    func pairKey(a: Body, b: Body) -> int64 {
        let x = int64(min(a.index, b.index)), y = int64(max(a.index, b.index))
        return x << 32 | y
    }

    /// Step advances the world by dt seconds.
    public func Step(dt: float32) {
        if dt <= 0 { return }
        let n = max(1, Substeps)
        let h = dt / float32(n)
        let live = Bodies.filter { $0.Enabled }
        let joints = Joints.filter { $0.Enabled && $0.A.Enabled && $0.B.Enabled }
        let pairs = candidates(live, dt)
        for _ in 0..<n {
            // Move freely.
            for b in live {
                b.prevPosition = b.Position
                b.prevRotation = b.Rotation
                if b.Dynamic {
                    b.Velocity += Gravity * h
                    b.Position += b.Velocity * h
                    b.rotate(b.AngularVelocity * h)
                } else if b.Kind == .kinematic {
                    b.Position += b.Velocity * h
                    b.rotate(b.AngularVelocity * h)
                }
            }
            // Then onto the constraints.
            for _ in 0..<JointIterations {
                for j in joints { j.solve(h) }
            }
            var contacts: [contact] = []
            for (a, b) in pairs {
                for c in collide.Collide(a.Shape, a.Transform, b.Shape, b.Transform, margin: Margin) {
                    let ra = c.PointA - a.Position, rb = c.PointB - b.Position
                    let pre = (a.VelocityAt(c.PointA) - b.VelocityAt(c.PointB)).Dot(c.Normal)
                    contacts.append(contact(a: a, b: b, normal: c.Normal, localA: a.Rotation.Conjugate().Rotate(ra),
                                            localB: b.Rotation.Conjugate().Rotate(rb), lambda: 0, approach: pre))
                }
            }
            for i in 0..<contacts.count { solveContact(&contacts[i], h) }
            // Velocities from how far things moved.
            for b in live where b.Dynamic {
                b.Velocity = (b.Position - b.prevPosition) / h
                let dq = (b.Rotation * b.prevRotation.Conjugate()).Normalized()
                b.AngularVelocity = rotationVector(dq) / h
                b.Velocity *= math.Max(0, 1 - b.LinearDamping * h)
                b.AngularVelocity *= math.Max(0, 1 - b.AngularDamping * h)
            }
            for i in 0..<contacts.count { contactVelocity(contacts[i], h) }
            for j in joints { j.damp(h) }
            ContactCount = contacts.count
        }
    }

    /// Raycast is the first body a ray meets within maxDistance, how far
    /// along it, and the surface normal there. `skip` leaves bodies out.
    public func Raycast(origin: geom.Vec3, direction: geom.Vec3, maxDistance: float32,
                        skip: (Body) -> bool = { _ in false }) -> (body: Body, distance: float32, normal: geom.Vec3)? {
        let d = direction.Normalized()
        var best: (body: Body, distance: float32, normal: geom.Vec3)? = nil
        for b in Bodies where b.Enabled && !skip(b) {
            if let hit = collide.Raycast(b.Shape, b.Transform, origin: origin, direction: d, maxDistance: best?.distance ?? maxDistance) {
                best = (body: b, distance: hit.distance, normal: hit.normal)
            }
        }
        return best
    }

    // candidates is the pairs of bodies that may touch this step: bounds
    // grown by how far they could move, overlapping, filters passing.
    func candidates(live: [Body], dt: float32) -> [(Body, Body)] {
        var bounds: [geom.AABB] = []
        for b in live {
            let reach = b.Velocity.Length() * dt + Margin + 0.01
            bounds.append(collide.Bounds(b.Shape, b.Transform).Expanded(by: reach))
        }
        var out: [(Body, Body)] = []
        for i in 0..<live.count {
            let a = live[i]
            for k in (i + 1)..<live.count {
                let b = live[k]
                if !a.Dynamic && !b.Dynamic { continue }
                if (a.Group & b.Mask) == 0 || (b.Group & a.Mask) == 0 { continue }
                if !bounds[i].Intersects(bounds[k]) { continue }
                if ignored[pairKey(a, b)] != nil { continue }
                out.append((a, b))
            }
        }
        return out
    }

    func solveContact(c: inout contact, h: float32) {
        let a = c.a, b = c.b
        let ra = a.Rotation.Rotate(c.localA), rb = b.Rotation.Rotate(c.localB)
        let pa = a.Position + ra, pb = b.Position + rb
        // Depth along the normal (positive when overlapping).
        let depth = (pa - pb).Dot(c.normal)
        if depth <= 0 { return }
        c.lambda = positional(a, b, c.normal * (-depth), ra, rb, 0, h)
        // Static friction: undo sliding this substep while the push allows.
        let ra2 = a.Rotation.Rotate(c.localA), rb2 = b.Rotation.Rotate(c.localB)
        let pa2 = a.Position + ra2, pb2 = b.Position + rb2
        let pa0 = a.prevPosition + a.prevRotation.Rotate(c.localA), pb0 = b.prevPosition + b.prevRotation.Rotate(c.localB)
        let dp = (pa2 - pa0) - (pb2 - pb0)
        let dt = dp - c.normal * dp.Dot(c.normal)
        let mu = (a.Friction + b.Friction) * 0.5
        if dt.Length() < mu * depth {
            _ = positional(a, b, -dt, ra2, rb2, 0, h)
        }
    }

    func contactVelocity(c: contact, h: float32) {
        if c.lambda <= 0 { return }
        let a = c.a, b = c.b
        let ra = a.Rotation.Rotate(c.localA), rb = b.Rotation.Rotate(c.localB)
        let v = a.VelocityAt(a.Position + ra) - b.VelocityAt(b.Position + rb)
        let vt = v - c.normal * v.Dot(c.normal)
        // Dynamic friction: an impulse against the sliding, up to µ times
        // the normal impulse (λ/h), and never more than stops it.
        let st = vt.Length()
        if st > 1e-6 {
            let mu = (a.Friction + b.Friction) * 0.5
            let dir = vt / st
            let w = a.weight(ra, dir) + b.weight(rb, dir)
            if w > 0 {
                let j = math.Min(mu * c.lambda / h, st / w)
                impulse(a, b, -dir * (j * w), ra, rb)
            }
        }
        // Restitution, but not for slow touches (they'd jitter).
        var e = math.Max(a.Restitution, b.Restitution)
        if c.approach < 2 * Gravity.Length() * h { e = 0 }
        let vn2 = (a.VelocityAt(a.Position + ra) - b.VelocityAt(b.Position + rb)).Dot(c.normal)
        impulse(a, b, c.normal * (-vn2 - e * math.Max(c.approach, 0)), ra, rb)
    }
}

// contact is one contact while a substep is solved.
struct contact {
    var a: Body
    var b: Body
    /// From a to b.
    var normal: geom.Vec3
    var localA: geom.Vec3
    var localB: geom.Vec3
    var lambda: float32
    /// The speed a and b closed along the normal before the substep.
    var approach: float32
}
