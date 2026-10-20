// physics/collide: boxes moving through a grid of solid cells; convex
// shapes and their contacts. physics: bodies falling, resting, sliding
// and bouncing; joints holding, hinges and ball joints keeping their
// limits; a chain staying together.
package main

import (
    "math"
    "math/geom"
    "mesh/height"
    "physics"
    "physics/character"
    "physics/collide"
    "physics/ragdoll"
)

var failures = 0

func check(_ ok: bool, _ what: string) {
    print(ok ? "ok    \(what)" : "FAIL  \(what)")
    if !ok { failures += 1 }
}

func near(_ a: float32, _ b: float32) -> bool { return math.Abs(a - b) < 0.001 }

func main() -> int32 {
    // A floor at y = 0 and a wall at x = 3, for y 1 and 2.
    let solid: (geom.IVec3) -> bool = { c in c.Y == 0 || (c.X == 3 && (c.Y == 1 || c.Y == 2)) }
    let player = geom.AABB(min: geom.Vec3(0.2, 1.5, 0.2), max: geom.Vec3(0.8, 3.3, 0.8))

    let fall = collide.MoveBox(player, by: geom.Vec3(0, -2, 0), solid: solid)
    check(near(fall.Box.Min.Y, 1) && fall.BlockedY && fall.Grounded, "falling lands on the floor (y \(fall.Box.Min.Y))")

    let standing = fall.Box
    let walk = collide.MoveBox(standing, by: geom.Vec3(5, 0, 0), solid: solid)
    check(near(walk.Box.Max.X, 3) && walk.BlockedX && !walk.BlockedY, "walking stops flush against the wall (x \(walk.Box.Max.X))")

    let slide = collide.MoveBox(standing, by: geom.Vec3(5, 0, 1), solid: solid)
    check(near(slide.Box.Max.X, 3) && near(slide.Moved.Z, 1), "moving into the wall at an angle slides along it")

    let still = collide.MoveBox(standing, by: geom.Vec3(0, -0.5, 0), solid: solid)
    check(near(still.Moved.Y, 0) && still.Grounded, "standing on the floor, gravity moves nothing and stays grounded")

    let jump = collide.MoveBox(standing, by: geom.Vec3(0, 1.2, 0), solid: solid)
    check(near(jump.Moved.Y, 1.2) && !jump.BlockedY, "jumping in the open isn't blocked")

    let ceiling: (geom.IVec3) -> bool = { c in c.Y == 0 || c.Y == 4 }
    let bump = collide.MoveBox(standing, by: geom.Vec3(0, 2, 0), solid: ceiling)
    check(near(bump.Box.Max.Y, 4) && bump.BlockedY && !bump.Grounded, "jumping into a ceiling stops under it")

    check(collide.Overlaps(geom.AABB(min: geom.Vec3(2.5, 1, 0), max: geom.Vec3(3.5, 2, 1)), solid: solid) &&
          !collide.Overlaps(standing, solid: solid), "Overlaps sees a box in the wall, and not one standing on the floor")

    // Negative coordinates: a floor at y = -10.
    let deep: (geom.IVec3) -> bool = { c in c.Y == -10 }
    let low = collide.MoveBox(geom.AABB(min: geom.Vec3(-5.5, -8, -5.5), max: geom.Vec3(-4.9, -6.2, -4.9)),
                              by: geom.Vec3(0, -3, 0), solid: deep)
    check(near(low.Box.Min.Y, -9), "landing works below zero (y \(low.Box.Min.Y))")

    // A fast fall doesn't tunnel through a one-cell floor.
    let fast = collide.MoveBox(player.Translated(geom.Vec3(0, 50, 0)), by: geom.Vec3(0, -100, 0), solid: solid)
    check(near(fast.Box.Min.Y, 1), "a 100-cell fall stops on the floor instead of passing through")

    shapes()
    bodies()
    joints()
    controller()
    limp()
    ground()
    vehicles()

    print(failures == 0 ? "all passed" : "\(failures) failed")
    return failures == 0 ? 0 : 1
}

func nearv(_ a: geom.Vec3, _ b: geom.Vec3, _ eps: float32 = 0.01) -> bool {
    return math.Abs(a.X - b.X) < eps && math.Abs(a.Y - b.Y) < eps && math.Abs(a.Z - b.Z) < eps
}

let deg = float32(math.Pi) / 180

func shapes() {
    let ground = geom.Transform.Identity
    let ball = collide.Collide(.sphere(radius: 0.5), geom.Transform(position: geom.Vec3(0, 0.4, 0)), .plane, ground)
    check(ball.count == 1 && near(ball[0].Depth, 0.1) && nearv(ball[0].Normal, geom.Vec3(0, -1, 0)),
          "a sphere sunk 0.1 into the ground: one contact, normal into the ground")
    let lying = geom.Transform(position: geom.Vec3(0, 0.25, 0), rotation: geom.AxisAngle(geom.Vec3.UnitZ, 90 * deg))
    check(collide.Collide(.capsule(radius: 0.3, halfHeight: 0.5), lying, .plane, ground).count == 2,
          "a capsule lying down touches the ground at both ends")
    check(collide.Collide(.box(halfExtents: geom.Vec3(0.5, 0.5, 0.5)), geom.Transform(position: geom.Vec3(0, 0.49, 0)), .plane, ground).count == 4,
          "a box resting on the ground touches it at four corners")
    let caps = collide.Collide(.capsule(radius: 0.2, halfHeight: 0.5), geom.Transform.Identity,
                               .sphere(radius: 0.2), geom.Transform(position: geom.Vec3(0.3, 0.2, 0)))
    check(caps.count == 1 && near(caps[0].Depth, 0.1) && nearv(caps[0].Normal, geom.Vec3(1, 0, 0)),
          "capsule against sphere: depth and normal from the capsule's core")
    let sb = collide.Collide(.sphere(radius: 0.25), geom.Transform(position: geom.Vec3(0, 0.7, 0)),
                             .box(halfExtents: geom.Vec3(0.5, 0.5, 0.5)), geom.Transform.Identity)
    check(sb.count == 1 && near(sb[0].Depth, 0.05) && nearv(sb[0].Normal, geom.Vec3(0, -1, 0)), "a sphere on a box")
    check(collide.Collide(.sphere(radius: 0.25), geom.Transform(position: geom.Vec3(0, 2, 0)),
                          .box(halfExtents: geom.Vec3.One), geom.Transform.Identity).count == 0, "apart, no contact")
    if let hit = collide.Raycast(.capsule(radius: 0.5, halfHeight: 1), geom.Transform.Identity,
                                 origin: geom.Vec3(-5, 0, 0), direction: geom.Vec3.UnitX, maxDistance: 10) {
        check(near(hit.distance, 4.5) && nearv(hit.normal, geom.Vec3(-1, 0, 0)), "a ray meets a capsule's side")
    } else {
        check(false, "a ray meets a capsule's side")
    }
}

func bodies() {
    let w = physics.World()
    w.Add(physics.Body(.fixed, .plane, name: "ground"))
    let ball = w.Add(physics.Body(.dynamic, .sphere(radius: 0.5), mass: 2, at: geom.Transform(position: geom.Vec3(0, 3, 0))))
    let crate = w.Add(physics.Body(.dynamic, .box(halfExtents: geom.Vec3(0.4, 0.4, 0.4)), mass: 10,
                                   at: geom.Transform(position: geom.Vec3(3, 2, 0), rotation: geom.AxisAngle(geom.Vec3(1, 0, 1), 0.5))))
    let log = w.Add(physics.Body(.dynamic, .capsule(radius: 0.2, halfHeight: 0.6), mass: 5,
                                 at: geom.Transform(position: geom.Vec3(-3, 2, 0), rotation: geom.AxisAngle(geom.Vec3.UnitZ, 1.2))))
    for _ in 0..<240 { w.Step(1.0 / 60) }
    check(near(ball.Position.Y, 0.5 + w.Margin * 0) || math.Abs(ball.Position.Y - 0.5) < 0.01, "a dropped ball comes to rest on the ground (y \(ball.Position.Y))")
    check(math.Abs(crate.Position.Y - 0.4) < 0.02 && crate.Velocity.Length() < 0.05, "a tumbling crate settles flat (y \(crate.Position.Y))")
    let up = log.Rotation.Rotate(geom.Vec3.UnitY)
    check(math.Abs(log.Position.Y - 0.2) < 0.02 && math.Abs(up.Y) < 0.05, "a capsule falls over and lies on its side (y \(log.Position.Y))")

    // Friction: a box sliding at 4 m/s stops within mu·g of where it should.
    let w2 = physics.World()
    w2.Add(physics.Body(.fixed, .plane))
    let slider = w2.Add(physics.Body(.dynamic, .box(halfExtents: geom.Vec3(0.5, 0.25, 0.5)), mass: 4,
                                     at: geom.Transform(position: geom.Vec3(0, 0.25, 0))))
    slider.Friction = 0.5
    slider.LinearDamping = 0
    slider.Velocity = geom.Vec3(4, 0, 0)
    for _ in 0..<180 { w2.Step(1.0 / 60) }
    // v² / (2 µ g) = 16 / 9.81 = 1.63 m
    check(slider.Velocity.Length() < 0.05 && math.Abs(slider.Position.X - 1.63) < 0.25,
          "friction stops a sliding box near v²/2µg (x \(slider.Position.X))")

    // Restitution: a bouncy ball comes back up.
    let w3 = physics.World()
    w3.Add(physics.Body(.fixed, .plane))
    let bouncy = w3.Add(physics.Body(.dynamic, .sphere(radius: 0.2), mass: 1, at: geom.Transform(position: geom.Vec3(0, 2, 0))))
    bouncy.Restitution = 0.8
    bouncy.LinearDamping = 0
    var peak: float32 = 0
    var landed = false
    for _ in 0..<150 {
        w3.Step(1.0 / 60)
        if bouncy.Position.Y < 0.25 { landed = true }
        if landed { peak = math.Max(peak, bouncy.Position.Y) }
    }
    // It should come back to about e² of the 1.8 m drop: 1.15 m, plus the radius.
    check(peak > 0.9 && peak < 1.6, "a ball with restitution 0.8 bounces back to about e² of its height (\(peak))")

    // Energy: a resting body doesn't creep.
    let before = ball.Position
    for _ in 0..<120 { w.Step(1.0 / 60) }
    check(nearv(before, ball.Position, 0.005), "bodies at rest stay put")
}

func joints() {
    // A pendulum: a ball on a hinge, swinging about Z from a fixed point.
    let w = physics.World()
    w.Gravity = geom.Vec3(0, -9.81, 0)
    let pin = w.Add(physics.Body(.fixed, .sphere(radius: 0.05), at: geom.Transform(position: geom.Vec3(0, 3, 0))))
    let bob = w.Add(physics.Body(.dynamic, .sphere(radius: 0.1), mass: 1, at: geom.Transform(position: geom.Vec3(1, 3, 0))))
    // The hinge turns about the frame's X: put X along world Z.
    let frame = geom.Transform(position: geom.Vec3(0, 3, 0), rotation: geom.AxisAngle(geom.Vec3.UnitY, -90 * deg))
    let hinge = w.Add(physics.Joint(.hinge, pin, bob, at: frame))
    var stretch: float32 = 0
    var offPlane: float32 = 0
    var lowest: float32 = 9
    for _ in 0..<120 {
        w.Step(1.0 / 60)
        stretch = math.Max(stretch, math.Abs((bob.Position - pin.Position).Length() - 1))
        offPlane = math.Max(offPlane, math.Abs(bob.Position.Z))
        lowest = math.Min(lowest, bob.Position.Y)
    }
    check(stretch < 0.02 && offPlane < 0.01, "a hinged pendulum keeps its length and its plane (\(stretch), \(offPlane))")
    check(math.Abs(lowest - 2) < 0.03, "and swings down through the bottom (lowest \(lowest))")

    // A limit: a door-like rod on a hinge, level to start, can't swing
    // past 45° below level. (A point mass on a long arm fights its own
    // tiny inertia; a rod is what limbs are.)
    let wl = physics.World()
    let post = wl.Add(physics.Body(.fixed, .sphere(radius: 0.05), at: geom.Transform(position: geom.Vec3(0, 3, 0))))
    let arm = wl.Add(physics.Body(.dynamic, .capsule(radius: 0.05, halfHeight: 0.45), mass: 2,
                                  at: geom.Transform(position: geom.Vec3(0.5, 3, 0), rotation: geom.AxisAngle(geom.Vec3.UnitZ, 90 * deg))))
    let limited = wl.Add(physics.Joint(.hinge, post, arm, at: frame))
    limited.HingeMin = -45 * deg
    limited.HingeMax = 45 * deg
    var deepest: float32 = 0
    for _ in 0..<120 {
        wl.Step(1.0 / 60)
        let d = arm.Position - post.Position
        deepest = math.Max(deepest, math.Atan2(-d.Y, math.Abs(d.X)))
    }
    check(deepest < 46.5 * deg && deepest > 40 * deg, "a hinge limit stops the swing at 45° (\(deepest / deg)°)")
    _ = hinge

    // A ball joint with a 30° cone: a rod hanging from a ceiling, kicked
    // sideways, can't swing further than 30°.
    let w2 = physics.World()
    let ceiling = w2.Add(physics.Body(.fixed, .sphere(radius: 0.05), at: geom.Transform(position: geom.Vec3(0, 3, 0))))
    let rod = w2.Add(physics.Body(.dynamic, .capsule(radius: 0.05, halfHeight: 0.4), mass: 2,
                                  at: geom.Transform(position: geom.Vec3(0, 2.5, 0))))
    let ball = w2.Add(physics.Joint(.ball, ceiling, rod, at: geom.Transform(position: geom.Vec3(0, 3, 0))))
    ball.Limits = physics.Limits(twist: 20 * deg, swingX: 30 * deg, swingZ: 30 * deg)
    rod.Velocity = geom.Vec3(6, 0, 3)
    var widest: float32 = 0
    for _ in 0..<120 {
        w2.Step(1.0 / 60)
        let axis = rod.Rotation.Rotate(geom.Vec3.UnitY)
        widest = math.Max(widest, math.Acos(math.Clamp(axis.Y, -1, 1)))
    }
    check(widest < 33 * deg, "a ball joint's swing limit holds a kicked rod within its cone (\(widest / deg)°)")

    // A chain of ten links hanging from a fixed point stays together.
    let w3 = physics.World()
    w3.Add(physics.Body(.fixed, .plane))
    var last = w3.Add(physics.Body(.fixed, .sphere(radius: 0.05), at: geom.Transform(position: geom.Vec3(0, 6, 0))))
    var links: [physics.Body] = []
    for i in 0..<10 {
        let y = 6 - 0.25 - float32(i) * 0.5
        let link = w3.Add(physics.Body(.dynamic, .capsule(radius: 0.05, halfHeight: 0.2), mass: 1,
                                       at: geom.Transform(position: geom.Vec3(0, y, 0))))
        w3.Add(physics.Joint(.ball, last, link, at: geom.Transform(position: geom.Vec3(0, y + 0.25, 0))))
        links.append(link)
        last = link
    }
    links[9].Velocity = geom.Vec3(8, 0, 0)
    var gap: float32 = 0
    for _ in 0..<180 {
        w3.Step(1.0 / 60)
        for j in w3.Joints {
            let (pa, pb) = j.Anchors
            gap = math.Max(gap, (pa - pb).Length())
        }
    }
    check(gap < 0.02, "a ten-link chain whipped sideways stays joined (worst gap \(gap) m)")
}

func controller() {
    let ground = [collide.Collider(.plane)]
    let c = character.Controller(at: geom.Vec3(0, 3, 0))
    var landed: float32 = 0
    for _ in 0..<120 {
        c.Move(1.0 / 60, wish: geom.Vec3.Zero, jump: false, colliders: ground)
        if c.Landed > 0 { landed = c.Landed }
    }
    check(c.Grounded && math.Abs(c.Position.Y) < 0.001, "a character dropped from 3 m lands and stands on the ground (y \(c.Position.Y))")
    check(math.Abs(landed - math.Sqrt(2 * 9.81 * 3)) < 0.4, "and says how hard it landed (\(landed) m/s)")
    for _ in 0..<60 { c.Move(1.0 / 60, wish: geom.Vec3(4, 0, 0), jump: false, colliders: ground) }
    check(c.Grounded && math.Abs(c.Velocity.X - 4) < 0.01 && c.Position.X > 3, "it runs at the speed it's asked for (\(c.Velocity.X) m/s)")
    var top: float32 = 0
    c.Move(1.0 / 60, wish: geom.Vec3.Zero, jump: true, colliders: ground)
    check(c.Jumped && !c.Grounded, "a jump leaves the ground")
    for _ in 0..<90 {
        c.Move(1.0 / 60, wish: geom.Vec3.Zero, jump: false, colliders: ground)
        top = math.Max(top, c.Position.Y)
    }
    check(math.Abs(top - 25 / (2 * 9.81)) < 0.06 && c.Grounded, "and rises v²/2g before coming down (\(top) m)")
    // A wall: a box it walks into and slides along.
    let walled = ground + [collide.Collider(.box(halfExtents: geom.Vec3(0.5, 2, 5)), geom.Transform(position: geom.Vec3(10, 2, 0)))]
    c.Place(geom.Vec3(8, 0, 0))
    for _ in 0..<120 { c.Move(1.0 / 60, wish: geom.Vec3(3, 0, 1), jump: false, colliders: walled) }
    check(math.Abs(c.Position.X - (9.5 - c.Radius)) < 0.01 && c.Position.Z > 1.5, "it stops at a wall and slides along it")
}

// limp: a two-part ragdoll (a body and a leg) falls and comes to rest.
func limp() {
    let w = physics.World()
    w.Add(physics.Body(.fixed, .plane))
    let rest = [geom.Transform(position: geom.Vec3(0, 3, 0)), geom.Transform(position: geom.Vec3(0, 2.6, 0))]
    let parts = [
        ragdoll.Part("body", bone: 0, parent: -1, shape: .capsule(radius: 0.15, halfHeight: 0.25), mass: 30,
                     offset: geom.Transform(position: geom.Vec3(0, 0.3, 0))),
        ragdoll.Part("leg", bone: 1, parent: 0, shape: .capsule(radius: 0.08, halfHeight: 0.3), mass: 10,
                     offset: geom.Transform(position: geom.Vec3(0, -0.38, 0)), joint: .hinge, hingeMin: 0, hingeMax: 2.5),
    ]
    let r = ragdoll.Ragdoll(w, parts: parts, rest: rest)
    check(!r.Active && !r.Bodies[0].Enabled, "a ragdoll starts inactive")
    var bones = rest
    bones[0].Rotation = geom.AxisAngle(geom.Vec3.UnitZ, 0.3)
    r.Activate(bones, previous: rest, dt: 1.0 / 60)
    for _ in 0..<240 {
        w.Step(1.0 / 60)
        r.Update(1.0 / 60)
    }
    var out = rest
    r.Bones(&out)
    check(r.StillFor > 0.5 && out[0].Position.Y < 0.4, "it falls, lands and lies still (\(r.StillFor) s still, body at \(out[0].Position.Y))")
    let (pa, pb) = r.Joints[1]!.Anchors
    check((pa - pb).Length() < 0.01, "with its joint still joined")
}

// ground: bodies, a character and a ragdoll on heightfields.
func ground() {
    // A ramp rising 0.3 m a meter along X (about 17°), 40 m square.
    var ramp = height.Field(width: 41, depth: 41, spacing: 1, origin: geom.Vec2(-20, -20))
    for j in 0..<41 { for i in 0..<41 { ramp.Set(i, j, float32(i - 20) * 0.3) } }
    // Hills: bumps 1.5 m high, 25 m apart.
    var hills = height.Field(width: 81, depth: 81, spacing: 1, origin: geom.Vec2(-40, -40))
    for j in 0..<81 { for i in 0..<81 {
        let x = float32(i - 40), z = float32(j - 40)
        hills.Set(i, j, 1.5 * math.Sin(x / 4) * math.Cos(z / 4))
    } }

    let down = collide.Raycast(.heightfield(ramp), geom.Transform.Identity, origin: geom.Vec3(5, 10, 0), direction: geom.Vec3(0, -1, 0), maxDistance: 20)
    check(down != nil && abs(down!.distance - 8.5) < 0.02 && down!.normal.X < -0.2, "a ray down hits the ramp, its normal tilted (\(down?.distance ?? -1) m)")

    let w = physics.World()
    w.Add(physics.Body(.fixed, .heightfield(ramp), name: "ramp"))
    let ball = w.Add(physics.Body(.dynamic, .sphere(radius: 0.3), mass: 5, at: geom.Transform(position: geom.Vec3(5, 3, -5))))
    let box = w.Add(physics.Body(.dynamic, .box(halfExtents: geom.Vec3(0.4, 0.2, 0.4)), mass: 20, at: geom.Transform(position: geom.Vec3(5, 2.5, 5))))
    box.Friction = 0.9
    var lowest: float32 = 1
    for _ in 0..<240 {
        w.Step(1.0 / 60)
        lowest = math.Min(lowest, ball.Position.Y - ramp.Sample(ball.Position.X, ball.Position.Z) - 0.3)
    }
    check(ball.Position.X < 3 && lowest > -0.06, "a ball dropped on the ramp rolls down it without sinking in (x \(ball.Position.X), deepest \(lowest) m)")
    let bh = box.Position.Y - ramp.Sample(box.Position.X, box.Position.Z)
    check(box.Velocity.Length() < 0.2 && bh > 0.15 && bh < 0.35, "a rough box comes to rest on it (\(bh) m above, moving \(box.Velocity.Length()) m/s)")
    let ray = w.Raycast(origin: geom.Vec3(-10, 20, 0), direction: geom.Vec3(0, -1, 0), maxDistance: 50)
    check(ray != nil && ray!.body.Kind == .fixed && abs(20 - ray!.distance - ramp.Sample(-10, 0)) < 0.02, "the world's raycast finds the ground")

    // A character walks over the hills, feet on the ground.
    let g = [collide.Collider(.heightfield(hills))]
    let c = character.Controller(at: geom.Vec3(-30, 3, 0))
    var off: float32 = 0
    var grounded = 0, steps = 0
    for k in 0..<480 {
        c.Move(1.0 / 60, wish: geom.Vec3(3, 0, 0), jump: false, colliders: g)
        if k > 60 {
            steps += 1
            if c.Grounded { grounded += 1 }
            off = math.Max(off, abs(c.Position.Y - hills.Sample(c.Position.X, c.Position.Z)))
        }
    }
    check(c.Position.X > -9 && grounded * 100 / steps > 95 && off < 0.12,
          "a character walks over the hills on its feet (to x \(c.Position.X), grounded \(grounded * 100 / steps)%, at most \(off) m off the ground)")
    c.Place(geom.Vec3(-15, 0, 0))
    for _ in 0..<240 { c.Move(1.0 / 60, wish: geom.Vec3(3, 0, 0), jump: false, colliders: [collide.Collider(.heightfield(ramp))]) }
    check(c.Position.X > -4.5 && c.Grounded && abs(c.Position.Y - ramp.Sample(c.Position.X, 0)) < 0.05, "and up a 17° ramp (to x \(c.Position.X))")

    // A two-part ragdoll on the ramp: it lands, slides or not, never sinks.
    let rw = physics.World()
    rw.Add(physics.Body(.fixed, .heightfield(ramp)))
    let rest = [geom.Transform(position: geom.Vec3(0, 3, 0)), geom.Transform(position: geom.Vec3(0, 2.6, 0))]
    let parts = [
        ragdoll.Part("body", bone: 0, parent: -1, shape: .capsule(radius: 0.15, halfHeight: 0.25), mass: 30,
                     offset: geom.Transform(position: geom.Vec3(0, 0.3, 0))),
        ragdoll.Part("leg", bone: 1, parent: 0, shape: .capsule(radius: 0.08, halfHeight: 0.3), mass: 10,
                     offset: geom.Transform(position: geom.Vec3(0, -0.38, 0)), joint: .hinge, hingeMin: 0, hingeMax: 2.5),
    ]
    let r = ragdoll.Ragdoll(rw, parts: parts, rest: rest)
    r.Activate(rest, previous: rest, dt: 1.0 / 60)
    var sunk: float32 = 0
    for _ in 0..<300 {
        rw.Step(1.0 / 60)
        r.Update(1.0 / 60)
        for b in r.Bodies { sunk = math.Min(sunk, b.Position.Y - ramp.Sample(b.Position.X, b.Position.Z)) }
    }
    check(sunk > -0.05, "a ragdoll falls onto the slope and stays on it (centers at worst \(sunk) m into it)")
}
