// Package vehicle is a car: a rigid chassis on raycast suspension, tires
// that grip up to a friction circle and slide past it, an engine with a
// torque curve driving the wheels through an automatic gearbox, brakes, a
// handbrake, steering that tightens less at speed, drag and downforce.
// It knows the ground only as a function that casts a ray and says how
// grippy what it hit is, so it drives on a heightfield, a track mesh or a
// plane alike; and other things only as contacts (Push, Collide).
//
//     let car = vehicle.Car(vehicle.Spec.Sports(), at: geom.Vec3(0, 0, 0))
//     car.Ground = { from, dir, length in vehicle.Flat(from, dir, length) }
//     car.Step(1.0 / 60, vehicle.Controls(throttle: 1))
//
// The chassis is stepped in substeps (Spec.Substeps a step) by
// semi-implicit Euler. Tire forces are demanded, not integrated from
// wheel spin: the engine and brakes ask for a force along the tire, the
// tire gives what its share of the friction circle allows, and what's
// left of the circle holds it sideways; what it can't give is slip (spin
// or lock) that the wheel shows and the sound hears. Arcade cars want it:
// stable at any frame rate, predictable at the limit, and it still
// slides, drifts and spins when pushed.
//
// Axes are math/geom's: Y up, the car facing +Z at rest, +X its left.
package vehicle

import (
    "math"
    "math/geom"
)

/// Hit is where a suspension ray met the ground.
public struct Hit {
    public var Point: geom.Vec3
    public var Normal: geom.Vec3
    /// How far along the ray, meters.
    public var Distance: float32
    /// What the surface does to grip (asphalt 1, grass ~0.6) and how
    /// much it drags at the tires (asphalt ~0, grass more).
    public var Grip: float32 = 1
    public var Drag: float32 = 0
    /// What the surface is, for the caller's sound and marks (0 the main
    /// surface; the meaning of others is the caller's).
    public var Surface: int = 0

    public init(point: geom.Vec3, normal: geom.Vec3, distance: float32, grip: float32 = 1, drag: float32 = 0, surface: int = 0) {
        Point = point
        Normal = normal
        Distance = distance
        Grip = grip
        Drag = drag
        Surface = surface
    }
}

/// Flat is ground: the plane Y = 0, asphalt everywhere. A Ground for
/// tests and empty lots.
public func Flat(from: geom.Vec3, dir: geom.Vec3, length: float32) -> Hit? {
    if dir.Y >= -1e-4 { return nil }
    let t = -from.Y / dir.Y
    if t < 0 || t > length { return nil }
    return Hit(point: from + dir * t, normal: geom.Vec3.UnitY, distance: t)
}

/// WheelSpec is one wheel: where it hangs, how big, its spring.
public struct WheelSpec {
    /// The wheel's center with the spring stretched all the way, in the
    /// car's frame (origin on the ground between the axles at rest; +Z
    /// forward). It rises by Compression from there.
    public var Position: geom.Vec3
    public var Radius: float32 = 0.34
    /// How far the spring can stretch below its mounting, meters.
    public var Travel: float32 = 0.22
    /// Spring rate (N/m) and damping (N·s/m).
    public var Stiffness: float32 = 42000
    public var Damping: float32 = 3600
    public var Steers = false
    public var Drives = false
    /// Its share of the brakes (the axle's wheels add to 1 with the rest).
    public var BrakeShare: float32 = 0.25
    public var Handbrake = false
    /// Grip relative to the tire's (rear tires a little wider).
    public var Grip: float32 = 1

    public init(position: geom.Vec3) { Position = position }
}

/// Spec is a car's build: body, wheels, engine, gears, tires, aero.
public struct Spec {
    public var Mass: float32 = 1450
    /// Half the chassis' box (for inertia and contacts), and its center
    /// in the car's frame.
    public var HalfExtents = geom.Vec3(0.95, 0.6, 2.2)
    public var BoxCenter = geom.Vec3(0, 0.75, 0.05)
    /// The center of mass in the car's frame: low and near the middle.
    public var CenterOfMass = geom.Vec3(0, 0.42, 0.05)
    public var Wheels: [WheelSpec] = []
    /// Engine torque (N·m) at rpm points, evenly from Idle to Redline.
    public var Torque: [float32] = [300, 380, 440, 480, 500, 505, 495, 470, 430]
    public var Idle: float32 = 950
    public var Redline: float32 = 7800
    /// Gear ratios from first, reverse, and the final drive.
    public var Gears: [float32] = [3.3, 2.25, 1.68, 1.3, 1.04, 0.85]
    public var Reverse: float32 = 3.2
    public var FinalDrive: float32 = 3.6
    /// Seconds without drive while a gear changes.
    public var ShiftTime: float32 = 0.16
    /// How much of the engine's torque reaches the wheels.
    public var Efficiency: float32 = 0.88
    /// Engine braking when off the throttle, N·m at the engine.
    public var EngineBrake: float32 = 70
    /// Brakes: the most force they ask of all four tires together, N.
    public var BrakeForce: float32 = 21000
    public var HandbrakeForce: float32 = 9000
    /// Tires: how hard they grip (times the load), the slip angle (rad)
    /// and slip ratio where grip peaks, and how much they keep sliding
    /// past it (0…1).
    public var Friction: float32 = 1.25
    public var PeakAngle: float32 = 0.13
    public var PeakRatio: float32 = 0.11
    public var Sliding: float32 = 0.86
    /// Steering lock (rad) standing still, and how speed takes it away:
    /// lock / (1 + speed / SteerFalloff).
    public var MaxSteer: float32 = 0.58
    public var SteerFalloff: float32 = 22
    /// How fast the wheels turn to the asked-for angle, rad/s.
    public var SteerRate: float32 = 3.2
    /// Anti-roll bars, N/m of difference across an axle.
    public var AntiRoll: float32 = 16000
    /// Air: drag (½ρCdA) and downforce (½ρClA), newtons per (m/s)².
    public var Drag: float32 = 0.36
    public var Downforce: float32 = 0.55
    /// Rolling resistance, N per N of load.
    public var Rolling: float32 = 0.012
    /// Help at the limit, 0…1: holds slides back before they spin, and
    /// steers into them (an arcade car's stability).
    public var Assist: float32 = 0.6
    /// Chassis substeps a step.
    public var Substeps = 4
    /// Extra force a boost gives (N) at full.
    public var BoostForce: float32 = 6500

    public init() {}

    /// Sports is a mid-engined two-seater on 19-inch wheels: 2.8 m
    /// between the axles, rear-wheel drive, about 300 kW.
    public static func Sports() -> Spec {
        var s = Spec()
        let front: float32 = 1.45, rear: float32 = -1.35, half: float32 = 0.86
        for (z, steers) in [(front, true), (rear, false)] {
            for x in [half, -half] {
                // At rest the springs hold a quarter of the car each.
                var w = WheelSpec(geom.Vec3(x, 0.36 - (steers ? 0.089 : 0.077), z))
                w.Radius = 0.36
                w.Steers = steers
                w.Drives = !steers
                w.Handbrake = !steers
                w.BrakeShare = steers ? 0.3 : 0.2
                w.Grip = steers ? 1 : 1.05
                w.Stiffness = steers ? 40000 : 46000
                s.Wheels.append(w)
            }
        }
        return s
    }
}

/// Controls are what a driver asks for, each 0…1 (Steer -1 right … 1 left).
public struct Controls {
    public var Throttle: float32 = 0
    public var Brake: float32 = 0
    public var Steer: float32 = 0
    public var Handbrake = false
    public var Boost: float32 = 0

    public init(throttle: float32 = 0, brake: float32 = 0, steer: float32 = 0, handbrake: bool = false, boost: float32 = 0) {
        Throttle = throttle
        Brake = brake
        Steer = steer
        Handbrake = handbrake
        Boost = boost
    }
}

/// Wheel is one wheel's state.
public struct Wheel {
    public let Spec: WheelSpec
    /// How far the spring is squeezed from full stretch, meters.
    public var Compression: float32 = 0
    public var Steer: float32 = 0
    /// Its roll about the axle (radians, for drawing) and how fast.
    public var Angle: float32 = 0
    public var Spin: float32 = 0
    public var Contact = false
    public var Load: float32 = 0
    /// How hard it's sliding, m/s: sideways and along (spin or lock).
    public var SlipSide: float32 = 0
    public var SlipAlong: float32 = 0
    /// The ground under it, when touching.
    public var Ground: Hit? = nil

    init(s: WheelSpec) { Spec = s }

    /// Skid is how much it's sliding, 0…1, for screeches and marks.
    public var Skid: float32 {
        if !Contact { return 0 }
        return math.Clamp((math.Abs(SlipSide) + math.Abs(SlipAlong) - 1.2) / 6, 0, 1)
    }
}

/// Car is a car in the world.
public final class Car {
    public var Spec: Spec
    /// The car's frame: its origin (on the ground between the axles) and
    /// how it's turned.
    public var Position: geom.Vec3
    public var Rotation: geom.Quat
    /// Of the center of mass, world.
    public var Velocity = geom.Vec3.Zero
    public var AngularVelocity = geom.Vec3.Zero
    public var Wheels: [Wheel] = []
    /// The ground: a ray from a point, along a direction, up to a length.
    public var Ground: (geom.Vec3, geom.Vec3, float32) -> Hit? = { a, d, l in Flat(a, d, l) }
    /// The engine and gearbox: rpm, the gear (1… forward, -1 reverse, 0
    /// neutral while shifting), how much throttle reached it.
    public private(set) var Rpm: float32
    public private(set) var Gear = 1
    public private(set) var Load: float32 = 0
    /// Counts down a gear change.
    public private(set) var Shifting: float32 = 0
    /// Seconds since the last change: the gearbox waits before the next.
    var settled: float32 = 1
    /// What was last asked of it.
    public private(set) var Input = Controls()
    let invInertia: geom.Vec3
    var gearWant = 1

    public init(spec: Spec, at position: geom.Vec3, yaw: float32 = 0) {
        Spec = spec
        Position = position
        Rotation = geom.AxisAngle(geom.Vec3.UnitY, yaw)
        Rpm = spec.Idle
        for w in spec.Wheels { Wheels.append(Wheel(w)) }
        // A solid box's inertia, a little more toward the ends (engine,
        // fuel, people), as 1/I.
        let e = spec.HalfExtents * 2
        let m = spec.Mass
        let ix = m / 12 * (e.Y * e.Y + e.Z * e.Z), iy = m / 12 * (e.X * e.X + e.Z * e.Z) * 1.1, iz = m / 12 * (e.X * e.X + e.Y * e.Y)
        invInertia = geom.Vec3(1 / ix, 1 / iy, 1 / iz)
        // Settle onto its springs.
        for i in 0..<Wheels.count { Wheels[i].Compression = spec.Mass * 9.81 / float32(max(1, spec.Wheels.count)) / Wheels[i].Spec.Stiffness }
    }

    // ---- where things are ----

    /// Forward, Left and Up are the car's axes in the world.
    public var Forward: geom.Vec3 { return Rotation.Rotate(geom.Vec3.UnitZ) }
    public var Left: geom.Vec3 { return Rotation.Rotate(geom.Vec3.UnitX) }
    public var Up: geom.Vec3 { return Rotation.Rotate(geom.Vec3.UnitY) }

    /// Center is the center of mass in the world.
    public var Center: geom.Vec3 { return Position + Rotation.Rotate(Spec.CenterOfMass) }

    /// Speed is how fast it goes, m/s; ForwardSpeed along its nose (negative
    /// backing up).
    public var Speed: float32 { return Velocity.Length() }
    public var ForwardSpeed: float32 { return Velocity.Dot(Forward) }

    /// Yaw is its heading: 0 facing +Z, growing toward +X.
    public var Yaw: float32 {
        let f = Forward
        return math.Atan2(f.X, f.Z)
    }

    /// Matrix places the car's frame in the world.
    public var Matrix: geom.Mat4 { return geom.Translation(Position) * Rotation.Matrix() }

    /// WheelMatrix places wheel i: at its hub, steered, rolled. A wheel
    /// mesh centered on its hub with its axle along X goes here.
    public func WheelMatrix(i: int) -> geom.Mat4 {
        let w = Wheels[i]
        let hub = w.Spec.Position + geom.Vec3(0, w.Compression, 0)
        return Matrix * geom.Translation(hub) * geom.RotationY(w.Steer) * geom.RotationX(w.Angle)
    }

    /// Hub is wheel i's center in the car's frame (the spring's length now).
    public func Hub(i: int) -> geom.Vec3 {
        let w = Wheels[i]
        return w.Spec.Position + geom.Vec3(0, w.Compression, 0)
    }

    /// Corners are the chassis box's eight corners in the world.
    public func Corners() -> [geom.Vec3] {
        var out: [geom.Vec3] = []
        let h = Spec.HalfExtents
        for sx in [float32(-1), 1] {
            for sy in [float32(-1), 1] {
                for sz in [float32(-1), 1] {
                    out.append(Position + Rotation.Rotate(Spec.BoxCenter + geom.Vec3(h.X * sx, h.Y * sy, h.Z * sz)))
                }
            }
        }
        return out
    }

    /// Place puts the car at rest at a point, heading yaw.
    public func Place(p: geom.Vec3, yaw: float32) {
        Position = p
        Rotation = geom.AxisAngle(geom.Vec3.UnitY, yaw)
        Velocity = geom.Vec3.Zero
        AngularVelocity = geom.Vec3.Zero
        for i in 0..<Wheels.count {
            Wheels[i].Spin = 0
            Wheels[i].Compression = Spec.Mass * 9.81 / float32(max(1, Wheels.count)) / Wheels[i].Spec.Stiffness
        }
        Gear = 1
        gearWant = 1
        Rpm = Spec.Idle
    }

    /// Grounded is how many wheels touch.
    public var Grounded: int {
        var n = 0
        for w in Wheels where w.Contact { n += 1 }
        return n
    }

    // ---- the engine ----

    /// EngineTorque is the torque at full throttle at an rpm, N·m.
    public func EngineTorque(rpm: float32) -> float32 {
        let t = Spec.Torque
        if t.isEmpty { return 0 }
        let f = math.Clamp((rpm - Spec.Idle) / (Spec.Redline - Spec.Idle), 0, 1) * float32(t.count - 1)
        let i = min(t.count - 2, int(f))
        if t.count == 1 { return t[0] }
        let k = f - float32(i)
        return t[i] + (t[i + 1] - t[i]) * k
    }

    func ratio(gear: int) -> float32 {
        if gear > 0 { return Spec.Gears[min(gear, Spec.Gears.count) - 1] * Spec.FinalDrive }
        if gear < 0 { return -Spec.Reverse * Spec.FinalDrive }
        return 0
    }

    // ---- stepping ----

    /// Step moves the car on by dt seconds with these controls.
    public func Step(dt: float32, c: Controls) {
        Input = c
        let n = max(1, Spec.Substeps)
        let h = dt / float32(n)
        gearbox(dt, c)
        // Steering: the asked-for angle, less at speed, reached at a rate.
        let lock = Spec.MaxSteer / (1 + math.Abs(ForwardSpeed) / Spec.SteerFalloff)
        for i in 0..<Wheels.count where Wheels[i].Spec.Steers {
            // The wheel on the inside of the turn turns more (Ackermann).
            let side: float32 = Wheels[i].Spec.Position.X * c.Steer > 0 ? 1.08 : 0.93
            let want = c.Steer * lock * side
            let d = math.Clamp(want - Wheels[i].Steer, -Spec.SteerRate * dt, Spec.SteerRate * dt)
            Wheels[i].Steer += d
        }
        for _ in 0..<n { substep(h, c) }
        // Wheels roll at the ground's speed plus their slip.
        for i in 0..<Wheels.count {
            Wheels[i].Angle += Wheels[i].Spin * dt
            if Wheels[i].Angle > 6.2832 { Wheels[i].Angle -= 6.2832 }
            if Wheels[i].Angle < -6.2832 { Wheels[i].Angle += 6.2832 }
        }
    }

    // gearbox picks the gear: up near the redline, down when it lugs,
    // reverse when stopped and braking with no throttle; and works out
    // the rpm from the driven wheels.
    func gearbox(dt: float32, c: Controls) {
        let v = ForwardSpeed
        settled += dt
        if Shifting > 0 {
            Shifting -= dt
            if Shifting <= 0 { Gear = gearWant }
        }
        // Shifts go by the road's speed through the gears, not the tires'
        // (a spinning inside wheel mid-corner isn't a reason to shift).
        let radius = Wheels.isEmpty ? 0.34 : Wheels[Wheels.count - 1].Spec.Radius
        func road(g: int) -> float32 { return math.Abs(v / radius * ratio(g)) * 60 / (2 * float32(math.Pi)) }
        if Shifting <= 0 && settled > 0.7 {
            if Gear > 0 {
                if c.Brake > 0.2 && c.Throttle < 0.05 && v < 0.6 && v > -0.5 { shift(-1) }
                else if road(Gear) > Spec.Redline * 0.93 && Gear < Spec.Gears.count && c.Throttle > 0.1 { shift(Gear + 1) }
                else if Gear > 1 && road(Gear) < Spec.Redline * (c.Throttle > 0.5 ? 0.5 : 0.36) && road(Gear - 1) < Spec.Redline * 0.85 { shift(Gear - 1) }
            } else if Gear < 0 {
                if c.Throttle > 0.2 && v > -0.6 { shift(1) }
            }
        }
        // The engine turns with the road through the gears, a little faster
        // when the driven tires spin up (never more than 2500 rpm over); it
        // never drops below idle.
        var spin: float32 = 0
        var k: float32 = 0
        for w in Wheels where w.Spec.Drives {
            spin += w.Spin
            k += 1
        }
        spin = k > 0 ? spin / k : 0
        let r = ratio(Shifting > 0 ? 0 : Gear)
        let roadRpm = road(Shifting > 0 ? gearWant : Gear)
        let spinRpm = math.Abs(spin * r) * 60 / (2 * float32(math.Pi))
        var target = roadRpm + math.Clamp(spinRpm - roadRpm, 0, 2500)
        if r == 0 { target = math.Max(roadRpm * 0.85, Spec.Idle + c.Throttle * (Spec.Redline - Spec.Idle) * 0.5) }
        // At a standstill the clutch slips: the engine revs with the throttle.
        let launch = Spec.Idle + c.Throttle * Spec.Redline * 0.45
        target = math.Max(target, launch * math.Clamp(1 - math.Abs(v) / 6, 0, 1))
        target = math.Clamp(target, Spec.Idle, Spec.Redline * 1.02)
        Rpm += (target - Rpm) * math.Min(1, dt * 14)
        Load += (c.Throttle - Load) * math.Min(1, dt * 10)
    }

    func shift(to: int) {
        if to == Gear { return }
        gearWant = to
        Gear = 0
        Shifting = Spec.ShiftTime
        settled = 0
    }

    func substep(h: float32, c: Controls) {
        let m = Spec.Mass
        var force = geom.Vec3(0, -9.81 * m, 0)
        var torque = geom.Vec3.Zero
        let com = Center
        let up = Up
        // Drive force at the tires from the engine, shared by the driven
        // wheels in contact by how hard each is pressed down (a limited-
        // slip differential: the light inside wheel in a turn doesn't take
        // it all and spin).
        var driven: float32 = 0
        var drivenLoad: float32 = 0
        for w in Wheels where w.Spec.Drives && w.Contact {
            driven += 1
            drivenLoad += w.Load
        }
        let r = ratio(Shifting > 0 ? 0 : Gear)
        // In reverse the brake pedal drives (below) and the throttle only
        // brakes until it stops and finds first.
        var engine = r <= 0 ? 0 : EngineTorque(Rpm) * c.Throttle * r * Spec.Efficiency
        // The limiter: by the road's speed through this gear.
        let roadRpm = math.Abs(ForwardSpeed / (Wheels.isEmpty ? 0.34 : Wheels[0].Spec.Radius) * r) * 60 / (2 * float32(math.Pi))
        if roadRpm > Spec.Redline { engine = 0 }
        if c.Throttle < 0.05 && r != 0 { engine -= Spec.EngineBrake * r * (ForwardSpeed > 0 ? 1 : -1) * math.Clamp(Rpm / Spec.Redline, 0, 1) }
        // Springs first: each wheel's ray, compression and load.
        var compress = [float32](repeating: 0, count: Wheels.count)
        for i in 0..<Wheels.count {
            // The ray starts a wheel's height above the hub at full
            // squeeze (so a wheel pressed into the ground still finds it)
            // and reaches the tire's bottom at full stretch.
            let s = Wheels[i].Spec
            let from = Position + Rotation.Rotate(s.Position) + up * (s.Travel + s.Radius)
            let length = s.Travel + s.Radius * 2
            if let hit = Ground(from, -up, length) {
                let comp = math.Clamp(length - hit.Distance, 0, s.Travel * 1.6)
                compress[i] = comp
                Wheels[i].Ground = hit
                Wheels[i].Contact = comp > 0
            } else {
                compress[i] = 0
                Wheels[i].Ground = nil
                Wheels[i].Contact = false
            }
        }
        // Anti-roll: pairs across each axle (same Z).
        var roll = [float32](repeating: 0, count: Wheels.count)
        for i in 0..<Wheels.count {
            for j in 0..<Wheels.count where j != i && math.Abs(Wheels[j].Spec.Position.Z - Wheels[i].Spec.Position.Z) < 0.1 {
                roll[i] += (compress[i] - compress[j]) * Spec.AntiRoll
            }
        }
        for i in 0..<Wheels.count {
            let s = Wheels[i].Spec
            let prev = Wheels[i].Compression
            Wheels[i].Compression = math.Min(compress[i], s.Travel * 1.6)
            guard Wheels[i].Contact, let hit = Wheels[i].Ground else {
                // In the air the wheel slows and droops.
                Wheels[i].Spin *= 0.995
                Wheels[i].Load = 0
                Wheels[i].SlipSide = 0
                Wheels[i].SlipAlong = 0
                continue
            }
            let speed = (Wheels[i].Compression - prev) / h
            // A bump stop past full travel.
            let stop = Wheels[i].Compression > s.Travel ? (Wheels[i].Compression - s.Travel) * s.Stiffness * 8 : 0
            var load = s.Stiffness * Wheels[i].Compression + s.Damping * speed + roll[i] + stop
            load = math.Max(0, load)
            Wheels[i].Load = load
            let n = hit.Normal
            let at = hit.Point
            force = force + n * load
            torque = torque + (at - com).Cross(n * load)

            // The tire: its heading on the ground, how the ground moves under it.
            let heading = Rotation.Rotate(geom.Vec3(math.Sin(Wheels[i].Steer), 0, math.Cos(Wheels[i].Steer)))
            var fwd = heading - n * heading.Dot(n)
            fwd = fwd.Length() > 1e-4 ? fwd.Normalized() : Forward
            let side = n.Cross(fwd)
            let v = Velocity + AngularVelocity.Cross(at - com)
            let vf = v.Dot(fwd), vs = v.Dot(side)
            let grip = Spec.Friction * s.Grip * hit.Grip
            let limit = grip * load

            // Along the tire: drive, brakes, rolling resistance, asked for.
            var along: float32 = 0
            if s.Drives && driven > 0 {
                let share = drivenLoad > 1 ? Wheels[i].Load / drivenLoad : 1 / driven
                along += engine / s.Radius * share
            }
            let brake = c.Brake * Spec.BrakeForce * s.BrakeShare + (c.Handbrake && s.Handbrake ? Spec.HandbrakeForce * 0.5 : 0)
            // Reverse: the brake pedal is the throttle backing up.
            var braking = brake
            if Gear < 0 { braking = c.Throttle * Spec.BrakeForce * s.BrakeShare + (c.Handbrake && s.Handbrake ? Spec.HandbrakeForce * 0.5 : 0) }
            if Gear < 0 && s.Drives && driven > 0 {
                let share = drivenLoad > 1 ? Wheels[i].Load / drivenLoad : 1 / driven
                along = -(EngineTorque(Rpm) * c.Brake * Spec.Reverse * Spec.FinalDrive * Spec.Efficiency) / s.Radius * share
            }
            // Braking opposes rolling, no more than would stop it this substep.
            let mShare = m / float32(Wheels.count)
            let stopAlong = mShare * math.Abs(vf) / h
            along -= math.Clamp(vf * 1000, -1, 1) * math.Min(braking, stopAlong)
            along -= vf * load * (Spec.Rolling + hit.Drag) * math.Clamp(math.Abs(vf), 0, 1) / math.Max(math.Abs(vf), 0.1)
            let locked = c.Handbrake && s.Handbrake

            // Sideways: from the slip angle, over the peak's angle, along a
            // curve that peaks and then holds most of its grip.
            let alpha = math.Atan2(vs, math.Max(math.Abs(vf), 3))
            var sideways = -slipCurve(alpha / Spec.PeakAngle, Spec.Sliding) * limit
            // Never more than stops the sideways motion this substep (a
            // car standing on a slope stays put; no jitter at rest).
            let stopSide = mShare * math.Abs(vs) / h
            sideways = math.Clamp(sideways, -stopSide, stopSide)
            if locked {
                // A locked wheel slides: little sideways grip, drag along.
                sideways *= 0.35
                along -= math.Clamp(vf * 1000, -1, 1) * math.Min(limit * 0.8, stopAlong)
            }
            // The friction circle: along first (the driver asked for it),
            // then what's left for sideways; what doesn't fit is slip.
            var slipAlong: float32 = 0
            if math.Abs(along) > limit {
                slipAlong = (math.Abs(along) - limit) / math.Max(limit, 1) * 4 * (along > 0 ? 1 : -1)
                along = limit * Spec.Sliding * (along > 0 ? 1 : -1)
            }
            let left = math.Sqrt(math.Max(0, limit * limit - along * along))
            sideways = math.Clamp(sideways, -left, left)
            let f = fwd * along + side * sideways
            force = force + f
            torque = torque + (at - com).Cross(f)
            Wheels[i].SlipSide = vs
            Wheels[i].SlipAlong = locked ? -vf : slipAlong
            // The wheel turns with the ground, faster when it spins up,
            // not at all when locked.
            let rolling = vf / s.Radius
            let want = locked ? 0 : rolling + slipAlong * 2 / s.Radius
            Wheels[i].Spin += (want - Wheels[i].Spin) * math.Min(1, h * 30)
        }
        // Air: drag against the motion, downforce with speed squared.
        let v2 = Velocity.Dot(Velocity)
        if v2 > 0.01 {
            force = force - Velocity * (Spec.Drag * Velocity.Length())
            let fs = ForwardSpeed
            force = force - up * (Spec.Downforce * fs * fs)
        }
        // Boost pushes along the nose.
        if c.Boost > 0 && Grounded > 0 { force = force + Forward * (Spec.BoostForce * c.Boost) }
        // Help at the limit: past a few degrees of sideslip, a yaw torque
        // that brings the nose back toward where the car is going.
        if Spec.Assist > 0 && Grounded >= 3 && Velocity.Length() > 4 {
            let fwd = Forward
            let vflat = Velocity - up * Velocity.Dot(up)
            let slip = math.Atan2(vflat.Dot(Left), math.Max(0.5, vflat.Dot(fwd)))
            let excess = math.Abs(slip) > 0.12 ? slip - math.Clamp(slip, -0.12, 0.12) : 0
            let yawRate = AngularVelocity.Dot(up)
            let hold: float32 = c.Handbrake ? 0.25 : 1
            torque = torque + up * ((excess * 26000 - yawRate * 2500 * math.Clamp(math.Abs(slip) * 4, 0, 1)) * Spec.Assist * hold)
        }
        // Integrate: velocity, then position; spin about the center of mass.
        Velocity = Velocity + force * (h / m)
        let local = Rotation.Conjugate().Rotate(torque)
        let w = geom.Vec3(local.X * invInertia.X, local.Y * invInertia.Y, local.Z * invInertia.Z)
        AngularVelocity = AngularVelocity + Rotation.Rotate(w) * h
        // A little damping in the air and on the ground.
        AngularVelocity = AngularVelocity * (1 - 0.4 * h)
        let comNew = com + Velocity * h
        let wl = AngularVelocity.Length()
        if wl > 1e-6 {
            Rotation = (geom.AxisAngle(AngularVelocity * (1 / wl), wl * h) * Rotation).Normalized()
        }
        Position = comNew - Rotation.Rotate(Spec.CenterOfMass)
        keepAboveGround()
    }

    // keepAboveGround stops the body sinking through the ground (landing
    // on its roof, a wheel off an edge): each corner below the ground is
    // lifted out and its downward speed taken away.
    func keepAboveGround() {
        for p in Corners() {
            guard let hit = Ground(p + geom.Vec3(0, 2, 0), geom.Vec3(0, -1, 0), 2.5) else { continue }
            let depth = 2 - hit.Distance
            if depth <= 0 { continue }
            Push(at: p, normal: hit.Normal, depth: depth, friction: 0.5, restitution: 0.05)
        }
    }

    // ---- contacts ----

    /// Push resolves the car against something fixed: at a point, the
    /// way out (a unit normal) and how deep. It moves the car out and
    /// takes away its speed into the normal (some bounces back), with
    /// friction along the surface.
    public func Push(at p: geom.Vec3, normal n: geom.Vec3, depth: float32, friction: float32 = 0.3, restitution: float32 = 0.2) {
        if depth <= 0 { return }
        Position = Position + n * depth
        let com = Center
        let rr = p - com
        let v = Velocity + AngularVelocity.Cross(rr)
        let vn = v.Dot(n)
        if vn >= 0 { return }
        let k = invMass + n.Dot(invInertiaWorld(rr.Cross(n)).Cross(rr))
        let jn = -(1 + restitution) * vn / k
        var impulse = n * jn
        let vt = v - n * vn
        let st = vt.Length()
        if st > 1e-4 {
            let t = vt * (1 / st)
            let kt = invMass + t.Dot(invInertiaWorld(rr.Cross(t)).Cross(rr))
            let jt = math.Min(st / kt, friction * jn)
            impulse = impulse - t * jt
        }
        apply(impulse, at: rr)
    }

    var invMass: float32 { return 1 / Spec.Mass }

    // invInertiaWorld is the inverse inertia applied to a world vector.
    func invInertiaWorld(v: geom.Vec3) -> geom.Vec3 {
        let l = Rotation.Conjugate().Rotate(v)
        return Rotation.Rotate(geom.Vec3(l.X * invInertia.X, l.Y * invInertia.Y, l.Z * invInertia.Z))
    }

    func apply(j: geom.Vec3, at rr: geom.Vec3) {
        Velocity = Velocity + j * invMass
        AngularVelocity = AngularVelocity + invInertiaWorld(rr.Cross(j))
    }
}

/// Collide separates two cars whose chassis boxes overlap, seen from
/// above (cars on a road: their boxes as rectangles in X and Z), and
/// trades the impulse between them. Answers the impulse's size (0 when
/// they didn't touch), for sounds.
@discardableResult
public func Collide(a: Car, b: Car, restitution: float32 = 0.25, friction: float32 = 0.3) -> float32 {
    guard let c = overlap(a, b) else { return 0 }
    let (n, depth, p) = c
    // Apart, each by its share of the mass.
    let wa = a.invMass / (a.invMass + b.invMass)
    a.Position = a.Position - n * (depth * wa)
    b.Position = b.Position + n * (depth * (1 - wa))
    let ra = p - a.Center, rb = p - b.Center
    let va = a.Velocity + a.AngularVelocity.Cross(ra)
    let vb = b.Velocity + b.AngularVelocity.Cross(rb)
    let rel = vb - va
    let vn = rel.Dot(n)
    if vn >= 0 { return 0 }
    let k = a.invMass + b.invMass + n.Dot(a.invInertiaWorld(ra.Cross(n)).Cross(ra)) + n.Dot(b.invInertiaWorld(rb.Cross(n)).Cross(rb))
    let jn = -(1 + restitution) * vn / k
    var j = n * jn
    let vt = rel - n * vn
    let st = vt.Length()
    if st > 1e-4 {
        let t = vt * (1 / st)
        let kt = a.invMass + b.invMass + t.Dot(a.invInertiaWorld(ra.Cross(t)).Cross(ra)) + t.Dot(b.invInertiaWorld(rb.Cross(t)).Cross(rb))
        j = j - t * math.Min(st / kt, friction * jn)
    }
    a.apply(j * -1, at: ra)
    b.apply(j, at: rb)
    return jn
}

// overlap is two cars' boxes as rectangles on the ground: the way from a
// to b to push them apart, how far, and a point where they touch.
func overlap(a: Car, b: Car) -> (geom.Vec3, float32, geom.Vec3)? {
    let ca = a.Position + a.Rotation.Rotate(a.Spec.BoxCenter), cb = b.Position + b.Rotation.Rotate(b.Spec.BoxCenter)
    if (cb - ca).Length() > a.Spec.HalfExtents.Length() + b.Spec.HalfExtents.Length() { return nil }
    if math.Abs(cb.Y - ca.Y) > a.Spec.HalfExtents.Y + b.Spec.HalfExtents.Y { return nil }
    func flat(v: geom.Vec3) -> geom.Vec3 {
        let f = geom.Vec3(v.X, 0, v.Z)
        return f.Length() > 1e-5 ? f.Normalized() : geom.Vec3.UnitX
    }
    let axes = [flat(a.Forward), flat(a.Left), flat(b.Forward), flat(b.Left)]
    func extent(c: Car, axis: geom.Vec3) -> float32 {
        let h = c.Spec.HalfExtents
        return math.Abs(flat(c.Left).Dot(axis)) * h.X + math.Abs(flat(c.Forward).Dot(axis)) * h.Z
    }
    var best: float32 = 1e9
    var normal = geom.Vec3.UnitX
    let d = geom.Vec3(cb.X - ca.X, 0, cb.Z - ca.Z)
    for ax in axes {
        let sep = math.Abs(d.Dot(ax))
        let pen = extent(a, ax) + extent(b, ax) - sep
        if pen <= 0 { return nil }
        if pen < best {
            best = pen
            normal = d.Dot(ax) >= 0 ? ax : ax * -1
        }
    }
    // Where they touch: between the two, toward the side a faces b.
    let p = geom.Vec3((ca.X + cb.X) / 2, (ca.Y + cb.Y) / 2, (ca.Z + cb.Z) / 2)
    return (normal, best, p)
}

// slipCurve is a tire's grip at a slip (in units of where it peaks):
// rising through 1 at the peak, then easing to `hold` of it, sign kept.
func slipCurve(s: float32, hold: float32) -> float32 {
    let a = math.Abs(s)
    var g: float32
    if a < 1 {
        g = a * (2 - a)
    } else {
        g = hold + (1 - hold) * math.Exp(-(a - 1) * 1.5)
    }
    return s < 0 ? -g : g
}
