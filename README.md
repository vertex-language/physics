# physics

[![package: vs-package](https://img.shields.io/badge/package-vs--package-f4f4f5?style=flat-square&labelColor=e4e4e7&color=18181b)](https://github.com/vertex-language)
[![license: MIT](https://img.shields.io/badge/license-MIT-f4f4f5?style=flat-square&labelColor=e4e4e7&color=18181b)](LICENSE)

Collision and rigid-body dynamics in pure Vertex: what overlaps and how
far things can move, bodies and joints stepped by an XPBD solver, a
character controller, and ragdolls laid over skeletons.

---

## Packages

| Package | What it is |
| :--- | :--- |
| **`physics/collide`** | Collision detection with no dynamics, usable on its own (picking, line of sight, character controllers). **Boxes in cell grids**: `MoveBox` moves a `geom.AABB` by a motion, Y then X then Z, each clipped flush against the first solid cell (no tunnelling at any speed); `Overlaps`. The grid is a function `(IVec3) -> bool`. **Shapes**: `Shape` (sphere, capsule along Y, box, plane, and `heightfield`: ground from a `mesh/height` field), `Collider` (a shape placed), `Collide` (contacts between two placed shapes: sphere and capsule pairs, either against a box, anything against a plane or a heightfield; box against box not yet), `Bounds`, `Inertia` (of a solid shape at a mass), `Raycast`. Against a heightfield each point tested (a sphere's center, a capsule's ends and middle, a box's corners) meets the plane under it facing the field's normal: right for hills and slopes, not cliffs. |
| **`physics`** | Rigid bodies: `World` (`Add`, `Remove`, `Ignore`, `Step`, `Raycast`), `Body` (dynamic, kinematic or fixed; mass and inertia from its shape; friction, restitution, damping, collision groups; `ApplyImpulse`), `Joint` (ball, hinge or weld between two bodies at a frame; `Limits` on twist and swing, an elliptical cone; hinge limits; compliance; damping; a `Drive` rotation to pull towards, for motors and active ragdolls). Stepped by XPBD (Müller et al. 2020): substeps that move bodies freely, then correct positions and rotations straight onto joints and contacts, with static and dynamic friction and restitution. Stiff joints and long chains stay stable. |
| **`physics/character`** | `Controller`: a kinematic capsule that walks, jumps and falls through colliders. Accelerates towards a wished-for velocity (on the ground and, less, in the air), slides along walls, stands on slopes up to `MaxSlope` and walks them at the horizontal speed asked for (following the ground's tangent), snaps to the ground over small drops, and says when it `Landed` and how hard, `AirTime` and the `Apex` of a fall. |
| **`physics/vehicle`** | `Car`: a rigid chassis on raycast suspension (`Spec`'s `WheelSpec`s: where each hub hangs, radius, travel, spring, damper, steers, drives, brake share, handbrake), anti-roll bars, tires that grip up to a friction circle and slide past it (`PeakAngle`, `Sliding`), an engine torque curve through an automatic gearbox (`Rpm`, `Gear`, shifting time, reverse on the brake at a stop), brakes and a handbrake, steering that tightens less with speed, drag, downforce, a boost, and an `Assist` that holds slides back before they spin. `Step(dt, Controls)`. The ground is a function (`Ground`: a ray in, a `Hit` with its grip and drag out; `Flat` for a plane). `WheelMatrix`, `Corners`, `Skid` per wheel for drawing and sound; `Push` against something fixed, `Collide` two cars (their boxes from above). `Spec.Sports()`: a 1450 kg mid-engined car, about 300 kW. |
| **`physics/ragdoll`** | `Part` (a bone's body: shape, mass, offset, joint kind, frame and limits) and `Ragdoll`: bodies and joints built around bones at rest, `Activate` (take over from the animation, moving as the bones were), `Deactivate`, `Bones` (write the bones back), `Push`, `StillFor` (how long it's been at rest). Knows bones only as `geom.Transform`s, so it fits any skeleton. |

Planned, from `proposed_graphics_v3.md` §7.1: convex hulls, triangle
meshes and heightfields with GJK/EPA and SAT, box against box, a
broadphase tree, sleeping and islands, a deterministic mode;
`physics/articulation`, `physics/soft`,
`physics/planar`.

---

## Example

```vertex
import "math/geom"
import "physics"

let world = physics.World()
world.Add(physics.Body(.fixed, .plane))                                   // the ground
let crate = world.Add(physics.Body(.dynamic, .box(halfExtents: geom.Vec3(0.4, 0.4, 0.4)), mass: 10,
                                   at: geom.Transform(position: geom.Vec3(0, 3, 0))))
for _ in 0..<120 { world.Step(1.0 / 60) }
// crate.Position.Y is 0.4: it fell and rests on the ground
```

---

## Testing

```bash
vsc run check    # boxes in grids; shapes and contacts; bodies resting, sliding, bouncing; hinges,
                 # cones and a whipped chain; the character controller; a ragdoll coming to rest;
                 # heightfields: a ball rolls down a ramp, a box rests on it, raycasts, a character
                 # walks hills and up a ramp, a ragdoll lands on a slope; a car settles, pulls away
                 # through the gears, tops out, brakes, corners, slides on the handbrake, and two
                 # cars that meet are pushed apart
```

---

## License

[MIT](LICENSE)
