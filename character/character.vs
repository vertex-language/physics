// Package character is a kinematic character controller: an upright
// capsule that walks, jumps and falls through a world of colliders. It
// isn't simulated (nothing pushes it about, and it doesn't topple); it
// moves where it's asked, slides along what it meets, stands on what's
// flat enough and drops off what isn't, the way players in games move.
package character

import (
    "math"
    "math/geom"
    "physics/collide"
)

/// Controller is one character's body.
public final class Controller {
    /// The middle of the feet: the bottom of the capsule.
    public var Position: geom.Vec3
    public var Velocity = geom.Vec3.Zero
    public var Radius: float32 = 0.3
    public var Height: float32 = 1.8
    public var Gravity = geom.Vec3(0, -9.81, 0)
    /// The steepest ground it stands on, radians from level.
    public var MaxSlope: float32 = 50 * float32(math.Pi) / 180
    /// Upward speed a jump starts with, m/s.
    public var JumpSpeed: float32 = 5
    /// How fast it reaches the speed it's asked for, m/s², on the ground
    /// and in the air.
    public var GroundAcceleration: float32 = 30
    public var AirAcceleration: float32 = 4
    /// Over drops this small it stays on the ground instead of taking off.
    public var SnapDistance: float32 = 0.08

    public private(set) var Grounded = false
    public private(set) var GroundNormal = geom.Vec3.UnitY
    /// The speed it hit the ground at in the last Move, if it landed then;
    /// 0 otherwise.
    public private(set) var Landed: float32 = 0
    /// Whether the last Move started a jump.
    public private(set) var Jumped = false
    /// Seconds since it last stood on something.
    public private(set) var AirTime: float32 = 0
    /// The highest point of the current time in the air: the start of a
    /// fall, for telling how far it fell.
    public private(set) var Apex: float32 = 0

    public init(at position: geom.Vec3) {
        Position = position
        Apex = position.Y
    }

    /// Collider is its capsule, where it is now.
    public var Collider: collide.Collider {
        return collide.Collider(.capsule(radius: Radius, halfHeight: Height / 2 - Radius),
                                geom.Transform(position: Position + geom.Vec3(0, Height / 2, 0)))
    }

    /// Place puts it somewhere, standing still.
    public func Place(p: geom.Vec3, velocity: geom.Vec3 = geom.Vec3.Zero) {
        Position = p
        Velocity = velocity
        Grounded = false
        AirTime = 0
        Apex = p.Y
    }

    /// Move advances it by dt seconds: towards the horizontal velocity
    /// `wish`, jumping if asked and it's on the ground, through the
    /// colliders.
    public func Move(dt: float32, wish: geom.Vec3, jump: bool, colliders: [collide.Collider]) {
        Landed = 0
        Jumped = false
        var left = math.Min(dt, 0.1)
        while left > 1e-6 {
            let h = math.Min(left, 1.0 / 120.0)
            tick(h, wish, jump && !Jumped, colliders)
            left -= h
        }
    }

    func tick(h: float32, wish: geom.Vec3, jump: bool, colliders: [collide.Collider]) {
        let was = Grounded
        if Grounded && jump {
            Velocity.Y = JumpSpeed
            Grounded = false
            Jumped = true
        }
        // Towards the wished-for horizontal velocity.
        let flat = geom.Vec3(Velocity.X, 0, Velocity.Z)
        let want = geom.Vec3(wish.X, 0, wish.Z)
        let diff = want - flat
        let accel = (Grounded ? GroundAcceleration : AirAcceleration) * h
        let d = diff.Length()
        let change = d > accel ? diff * (accel / d) : diff
        Velocity.X += change.X
        Velocity.Z += change.Z
        if Grounded {
            // Along the ground at the asked-for pace over it: up and down
            // slopes the horizontal speed holds, and the climb or descent
            // is whatever the slope makes of it.
            let n = GroundNormal
            Velocity.Y = n.Y > 0.05 ? -(n.X * Velocity.X + n.Z * Velocity.Z) / n.Y : 0
        } else {
            Velocity += Gravity * h
        }
        let falling = Velocity.Y
        Position += Velocity * h

        Grounded = false
        resolve(colliders)
        if !Grounded && was && !Jumped {
            snap(colliders)
        }
        if Grounded {
            if !was { Landed = math.Max(0, -falling) }
            AirTime = 0
            Apex = Position.Y
        } else {
            AirTime += h
            Apex = math.Max(Apex, Position.Y)
        }
    }

    // resolve pushes the capsule out of what it overlaps and takes away
    // the velocity into it.
    func resolve(colliders: [collide.Collider]) {
        let walkable = math.Cos(MaxSlope)
        for _ in 0..<4 {
            var moved = false
            for c in colliders {
                let me = Collider
                for k in collide.Collide(me.Shape, me.Transform, c.Shape, c.Transform) {
                    if k.Depth <= 0 { continue }
                    // k.Normal points from the capsule into the other.
                    let out = -k.Normal
                    Position += out * k.Depth
                    let vn = Velocity.Dot(k.Normal)
                    if vn > 0 { Velocity -= k.Normal * vn }
                    if out.Y >= walkable {
                        Grounded = true
                        GroundNormal = out
                    }
                    moved = true
                }
            }
            if !moved { break }
        }
    }

    // snap keeps it on the ground over small drops: if there's walkable
    // ground within SnapDistance below, it moves down onto it.
    func snap(colliders: [collide.Collider]) {
        let walkable = math.Cos(MaxSlope)
        let me = Collider
        let probe = geom.Transform(position: me.Transform.Position - geom.Vec3(0, SnapDistance, 0))
        var best: float32 = -1
        var normal = geom.Vec3.UnitY
        for c in colliders {
            for k in collide.Collide(me.Shape, probe, c.Shape, c.Transform) where -k.Normal.Y >= walkable {
                if k.Depth > best { best = k.Depth; normal = -k.Normal }
            }
        }
        if best >= 0 {
            Position.Y -= SnapDistance - best
            Grounded = true
            GroundNormal = normal
            Velocity.Y = math.Min(Velocity.Y, 0)
        }
    }
}
