// Package ragdoll lays rigid bodies over a skeleton's bones and joins them
// with limited joints, so an animated body can go limp and fall, tumble
// and come to rest under physics, and its bones be read back to pose the
// mesh. It knows bones only as placements in the world (geom.Transform),
// so it works with any skeleton type; anim's Skeleton.Transforms and
// PoseFrom are the usual way in and out.
package ragdoll

import (
    "math"
    "math/geom"
    "physics"
    "physics/collide"
)

/// Part is one body of a ragdoll: which bone it carries, its shape and
/// mass, and the joint that hangs it from its parent part.
public struct Part {
    public var Name: string
    /// The bone it moves, as an index into the caller's bone list.
    public var Bone: int
    /// The part it hangs from (earlier in the list), or -1 for the root.
    public var Parent: int
    public var Shape: collide.Shape
    public var Mass: float32
    /// Where the shape sits in the bone's space.
    public var Offset: geom.Transform
    public var Joint: physics.JointKind
    /// The joint frame's rotation in the bone's space at rest: its Y axis
    /// is the twist axis, its X axis a hinge's axis.
    public var Frame: geom.Quat
    public var Limits: physics.Limits?
    public var HingeMin: float32
    public var HingeMax: float32

    public init(name: string, bone: int, parent: int, shape: collide.Shape, mass: float32,
                offset: geom.Transform = geom.Transform.Identity, joint: physics.JointKind = .ball,
                frame: geom.Quat = geom.Quat.Identity, limits: physics.Limits? = nil,
                hingeMin: float32 = 0, hingeMax: float32 = 0) {
        Name = name
        Bone = bone
        Parent = parent
        Shape = shape
        Mass = mass
        Offset = offset
        Joint = joint
        Frame = frame
        Limits = limits
        HingeMin = hingeMin
        HingeMax = hingeMax
    }
}

/// Ragdoll is a set of parts in a physics world. It starts inactive: its
/// bodies are disabled until Activate.
public final class Ragdoll {
    public let Parts: [Part]
    public let Bodies: [physics.Body]
    /// One per part; the root's is nil.
    public let Joints: [physics.Joint?]
    public private(set) var Active = false
    let world: physics.World
    /// Seconds the bodies have all been nearly still.
    public private(set) var StillFor: float32 = 0

    /// A ragdoll built in `world` around bones placed at `rest` (world
    /// placements, indexed as the parts' Bone). Bodies that overlap there
    /// (an upper arm and the chest) don't collide with each other; the
    /// rest do, so limbs don't pass through the body.
    public init(world: physics.World, parts: [Part], rest: [geom.Transform], group: uint32 = 2, mask: uint32 = 0xFFFFFFFF) {
        self.world = world
        Parts = parts
        var bodies: [physics.Body] = []
        for p in parts {
            let b = physics.Body(.dynamic, p.Shape, mass: p.Mass, at: rest[p.Bone] * p.Offset, name: p.Name)
            b.Group = group
            b.Mask = mask
            b.Friction = 0.8
            b.Restitution = 0
            b.LinearDamping = 0.05
            b.AngularDamping = 0.6
            b.Enabled = false
            world.Add(b)
            bodies.append(b)
        }
        var joints: [physics.Joint?] = []
        for (i, p) in parts.enumerated() {
            if p.Parent < 0 {
                joints.append(nil)
                continue
            }
            let bone = rest[p.Bone]
            let frame = geom.Transform(position: bone.Position, rotation: (bone.Rotation * p.Frame).Normalized())
            let j = physics.Joint(p.Joint, bodies[p.Parent], bodies[i], at: frame)
            j.Limits = p.Limits
            j.HingeMin = p.HingeMin
            j.HingeMax = p.HingeMax
            j.Damping = 2
            world.Add(j)
            joints.append(j)
        }
        for i in 0..<bodies.count {
            for k in (i + 1)..<bodies.count {
                let a = bodies[i], b = bodies[k]
                if !collide.Collide(a.Shape, a.Transform, b.Shape, b.Transform, margin: 0.01).isEmpty { world.Ignore(a, b) }
            }
        }
        Bodies = bodies
        Joints = joints
    }

    /// Root is the root part's body: the pelvis, usually.
    public var Root: physics.Body { return Bodies[0] }

    /// Match puts the bodies where the bones are, still.
    public func Match(bones: [geom.Transform]) {
        for (i, p) in Parts.enumerated() {
            Bodies[i].Transform = bones[p.Bone] * p.Offset
            Bodies[i].Velocity = geom.Vec3.Zero
            Bodies[i].AngularVelocity = geom.Vec3.Zero
        }
    }

    /// Activate makes the ragdoll take over from the animation: the bodies
    /// start where the bones are, moving as they were moving between
    /// `previous` and `bones` over dt seconds.
    public func Activate(bones: [geom.Transform], previous: [geom.Transform], dt: float32) {
        for (i, p) in Parts.enumerated() {
            let now = bones[p.Bone] * p.Offset
            let then = previous[p.Bone] * p.Offset
            let b = Bodies[i]
            b.Transform = now
            if dt > 0 {
                b.Velocity = (now.Position - then.Position) / dt
                let dq = (now.Rotation * then.Rotation.Conjugate()).Normalized()
                b.AngularVelocity = angularVelocity(dq, dt)
            }
            b.Enabled = true
        }
        StillFor = 0
        Active = true
    }

    /// Deactivate hands the bones back to the animation.
    public func Deactivate() {
        for b in Bodies { b.Enabled = false }
        Active = false
    }

    /// Bones writes where the parts have put their bones (world
    /// placements) into `bones`; bones without a part are left alone.
    public func Bones(bones: inout [geom.Transform]) {
        for (i, p) in Parts.enumerated() {
            bones[p.Bone] = Bodies[i].Transform * p.Offset.Inverse()
        }
    }

    /// Push gives the part nearest `at` an impulse (kg·m/s).
    public func Push(impulse: geom.Vec3, at point: geom.Vec3) {
        var best = 0
        var d: float32 = 1e30
        for (i, b) in Bodies.enumerated() {
            let x = (b.Position - point).LengthSquared()
            if x < d { d = x; best = i }
        }
        Bodies[best].ApplyImpulse(impulse, at: point)
    }

    /// Speed is the fastest any part is moving, m/s.
    public var Speed: float32 {
        var s: float32 = 0
        for b in Bodies { s = math.Max(s, b.Velocity.Length()) }
        return s
    }

    /// Update counts how long the ragdoll has been still (every part
    /// slower than `speed`); call it once a frame after the world steps.
    public func Update(dt: float32, still speed: float32 = 0.25) {
        StillFor = Active && Speed < speed ? StillFor + dt : 0
    }
}

func angularVelocity(dq: geom.Quat, dt: float32) -> geom.Vec3 {
    let s: float32 = dq.W < 0 ? -1 : 1
    let v = geom.Vec3(dq.X * s, dq.Y * s, dq.Z * s)
    let l = v.Length()
    if l < 1e-9 { return geom.Vec3.Zero }
    return v * (2 * math.Atan2(l, dq.W * s) / l / dt)
}
