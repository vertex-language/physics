package main

import (
    "math"
    "math/geom"
    "physics/vehicle"
)

// vehicles drives the sports car on flat ground: it settles on its
// springs and stays put; pulls away and goes through the gears; reaches
// a sports car's top speed; stops from 100 km/h; holds a corner at a
// steady speed; slides when the handbrake pulls it round; and two cars
// that meet are pushed apart.
func vehicles() {
    let dt: float32 = 1.0 / 60

    // At rest: on its springs, level, still.
    let car = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0.05, 0))
    for _ in 0..<180 { car.Step(dt, vehicle.Controls()) }
    var hubs: float32 = 0
    for i in 0..<car.Wheels.count { hubs += car.Matrix.TransformPoint(car.Hub(i)).Y / float32(car.Wheels.count) }
    check(car.Grounded == 4 && math.Abs(hubs - 0.36) < 0.03 && car.Speed < 0.02,
          "a car settles on its four springs, hubs at its wheels' radius (\(hubs) m), and stays still (\(car.Speed) m/s)")
    check(car.Up.Y > 0.999, "level at rest")

    // Flat out from a standstill.
    var t: float32 = 0
    var to100: float32 = 0
    var topGear = 1
    while t < 40 {
        car.Step(dt, vehicle.Controls(throttle: 1))
        t += dt
        if to100 == 0 && car.ForwardSpeed > 27.78 { to100 = t }
        topGear = max(topGear, car.Gear)
    }
    let top = car.ForwardSpeed * 3.6
    check(to100 > 2.5 && to100 < 6.5, "0–100 km/h in \(to100) s")
    check(topGear >= 5, "shifts up through the gears (to \(topGear))")
    check(top > 240 && top < 360, "tops out at \(Int(top)) km/h")
    check(math.Abs(car.Position.X) < 2 && math.Abs(car.Yaw) < 0.05, "straight as it goes (\(car.Position.X) m aside, yaw \(car.Yaw))")

    // Brakes from 100 km/h.
    let b = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0, 0))
    for _ in 0..<60 { b.Step(dt, vehicle.Controls()) }
    b.Velocity = b.Forward * 27.78
    for i in 0..<b.Wheels.count { b.Wheels[i].Spin = 27.78 / b.Wheels[i].Spec.Radius }
    let start = b.Position
    t = 0
    while b.ForwardSpeed > 0.3 && t < 10 {
        b.Step(dt, vehicle.Controls(brake: 1))
        t += dt
    }
    let stop = (b.Position - start).Length()
    check(stop > 22 && stop < 45, "stops from 100 km/h in \(stop) m (\(t) s)")

    // A steady corner: half steering at 15 m/s holds a circle.
    let c = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0, 0))
    for _ in 0..<60 { c.Step(dt, vehicle.Controls()) }
    c.Velocity = c.Forward * 15
    var yawRate: float32 = 0
    for k in 0..<480 {
        // Keep it near 15 m/s.
        let th: float32 = c.ForwardSpeed < 15 ? 0.35 : 0
        c.Step(dt, vehicle.Controls(throttle: th, steer: 0.5))
        if k > 300 { yawRate += c.AngularVelocity.Y / 180 }
    }
    let radius = c.Speed / math.Max(0.01, math.Abs(yawRate))
    check(yawRate > 0.2 && radius > 8 && radius < 80, "steering left turns it left, round a \(Int(radius)) m circle")
    check(c.Up.Y > 0.97 && c.Grounded == 4, "leaning, but all four wheels down")

    // The handbrake at speed with the wheel turned: the tail comes round.
    let d = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0, 0))
    for _ in 0..<60 { d.Step(dt, vehicle.Controls()) }
    d.Velocity = d.Forward * 22
    var slide: float32 = 0
    for _ in 0..<90 {
        d.Step(dt, vehicle.Controls(steer: 0.6, handbrake: true))
        let v = d.Velocity
        let s = math.Abs(math.Atan2(v.Dot(d.Left), math.Max(0.5, v.Dot(d.Forward))))
        slide = math.Max(slide, s)
    }
    var skid: float32 = 0
    for w in d.Wheels { skid = math.Max(skid, w.Skid) }
    check(slide > 0.25, "the handbrake slides the tail out (\(Int(slide * 57.3))° of sideslip)")
    let e = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0, 0))
    for _ in 0..<60 { e.Step(dt, vehicle.Controls()) }
    e.Velocity = e.Forward * 22
    for _ in 0..<90 { e.Step(dt, vehicle.Controls(steer: 0.6)) }
    check(e.Up.Y > 0.9, "and without it, the same turn stays on its wheels")

    // Two cars: one parked across the other's path, 7 m ahead, and the
    // other driven into its side.
    let a = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0, 0))
    let o = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0, 7), yaw: float32(math.Pi) / 2)
    for _ in 0..<60 {
        a.Step(dt, vehicle.Controls())
        o.Step(dt, vehicle.Controls())
    }
    a.Velocity = a.Forward * 10
    var hit: float32 = 0
    for _ in 0..<120 {
        a.Step(dt, vehicle.Controls())
        o.Step(dt, vehicle.Controls())
        hit = math.Max(hit, vehicle.Collide(a, o))
    }
    check(hit > 0, "two cars that meet are pushed apart (impulse \(Int(hit)) N·s)")
    check(o.Velocity.Z > 0.3 || o.Position.Z > 7.2, "the one hit is shoved along (\(o.Velocity.Z) m/s)")
    let gap = (o.Position - a.Position).Length()
    check(gap > 2, "and they don't end up inside each other (\(gap) m apart)")
}
