package collide

// This file is shapes placed in space: spheres, capsules, boxes, planes
// and heightfields, the contacts between them, their bounds, and their
// mass. It is what bodies, ragdolls and character controllers stand on.

import (
    "math"
    "math/geom"
    "mesh/height"
)

/// Shape is a convex shape in its own space, centered on the origin.
public enum Shape {
    case sphere(radius: float32)
    /// A capsule along local Y: the points within `radius` of the segment
    /// from (0, -halfHeight, 0) to (0, halfHeight, 0).
    case capsule(radius: float32, halfHeight: float32)
    case box(halfExtents: geom.Vec3)
    /// The half-space below local Y = 0: the ground, a wall. Static only.
    case plane
    /// Ground from a heightfield: what's under its surface is solid. Placed
    /// by the field's own origin, moved by the collider's position (not
    /// turned). Static only. Contacts treat the ground under each point as
    /// a plane facing the field's normal there: right for hills and slopes
    /// (up to ~45°), not for cliffs or overhangs.
    case heightfield(height.Field)
}

extension Shape {
    /// Whether it's a heightfield or a plane: ground, not a body's shape.
    var isGround: bool {
        switch self {
        case .plane, .heightfield(_): return true
        default: return false
        }
    }

    /// Whether it's a heightfield.
    var isField: bool {
        if case .heightfield(_) = self { return true }
        return false
    }
}

/// Collider is a shape placed in the world.
public struct Collider {
    public var Shape: Shape
    public var Transform: geom.Transform

    public init(_ shape: Shape, _ transform: geom.Transform = geom.Transform.Identity) {
        self.Shape = shape
        self.Transform = transform
    }
}

/// Contact is where two shapes touch. Normal points from the first shape
/// to the second: moving the second along it, or the first against it, by
/// Depth separates them.
public struct Contact {
    /// The deepest point of the first shape, and of the second.
    public var PointA: geom.Vec3
    public var PointB: geom.Vec3
    public var Normal: geom.Vec3
    /// How far they overlap; 0 or more.
    public var Depth: float32
}

/// Collide is the contacts between two placed shapes: none when they're
/// apart (or closer than `margin` apart, which counts as touching), up to
/// four for a box resting on a plane. Box against box isn't handled yet
/// and gives none.
public func Collide(_ a: Shape, _ ta: geom.Transform, _ b: Shape, _ tb: geom.Transform, margin: float32 = 0) -> [Contact] {
    // Ground first, outside the switch (a tuple switch over shapes that
    // carry a field doesn't lower yet: vsc_TODO #78).
    if a.isField || b.isField {
        if a.isGround && b.isGround { return [] }
        if case .heightfield(let f) = b { return againstGround(a, ta, f, tb.Position, margin) }
        if case .heightfield(let f) = a { return flip(againstGround(b, tb, f, ta.Position, margin)) }
        return []
    }
    switch (a, b) {
    case (.plane, .plane):
        return []
    case (.plane, _):
        return flip(Collide(b, tb, a, ta, margin: margin))
    case (_, .plane):
        return againstPlane(a, ta, tb, margin)
    case (.box, .box):
        return []
    case (.box, _):
        return flip(Collide(b, tb, a, ta, margin: margin))
    case (_, .box(let he)):
        let (p, q, r) = segment(a, ta)
        if let c = segmentBox(p, q, r, he, tb, margin) { return [c] }
        return []
    default:
        let (p0, p1, ra) = segment(a, ta)
        let (q0, q1, rb) = segment(b, tb)
        let st = geom.ClosestBetweenSegments(p0, p1, q0, q1)
        let pa = geom.Lerp(p0, p1, st.s), pb = geom.Lerp(q0, q1, st.t)
        var d = pb - pa
        let dist = d.Length()
        let depth = ra + rb - dist
        if depth < -margin { return [] }
        if dist > 1e-6 {
            d = d / dist
        } else {
            // Centers on top of each other: push apart along any square
            // to the first segment.
            d = (p1 - p0).Cross(geom.Vec3.UnitX).LengthSquared() > 1e-8 ? (p1 - p0).Cross(geom.Vec3.UnitX).Normalized() : geom.Vec3.UnitY
        }
        return [Contact(PointA: pa + d * ra, PointB: pb - d * rb, Normal: d, Depth: math.Max(depth, 0))]
    }
}

/// Bounds is the box around a placed shape (a huge one for a plane).
public func Bounds(_ s: Shape, _ t: geom.Transform) -> geom.AABB {
    switch s {
    case .sphere(let r):
        return geom.AABB(center: t.Position, size: geom.Vec3(splat: r * 2))
    case .capsule(let r, let h):
        let axis = t.Rotation.Rotate(geom.Vec3(0, h, 0))
        let ext = geom.Vec3(math.Abs(axis.X) + r, math.Abs(axis.Y) + r, math.Abs(axis.Z) + r)
        return geom.AABB(min: t.Position - ext, max: t.Position + ext)
    case .box(let he):
        let x = t.Rotation.Rotate(geom.Vec3(he.X, 0, 0)), y = t.Rotation.Rotate(geom.Vec3(0, he.Y, 0)),
            z = t.Rotation.Rotate(geom.Vec3(0, 0, he.Z))
        let ext = geom.Vec3(math.Abs(x.X) + math.Abs(y.X) + math.Abs(z.X), math.Abs(x.Y) + math.Abs(y.Y) + math.Abs(z.Y),
                            math.Abs(x.Z) + math.Abs(y.Z) + math.Abs(z.Z))
        return geom.AABB(min: t.Position - ext, max: t.Position + ext)
    case .plane:
        let big: float32 = 1e9
        return geom.AABB(min: geom.Vec3(splat: -big), max: geom.Vec3(splat: big))
    case .heightfield(let f):
        // Its extent, and every height: finding its lowest and highest
        // points would read the whole field, and the world asks every step.
        let s = f.Size
        let big: float32 = 1e6
        return geom.AABB(min: geom.Vec3(f.Origin.X + t.Position.X, -big, f.Origin.Y + t.Position.Z),
                         max: geom.Vec3(f.Origin.X + s.X + t.Position.X, big, f.Origin.Y + s.Y + t.Position.Z))
    }
}

/// Inertia is the inertia of a solid shape of mass m about its
/// center, along its own axes (the diagonal of its inertia tensor).
public func Inertia(_ s: Shape, mass m: float32) -> geom.Vec3 {
    switch s {
    case .sphere(let r):
        return geom.Vec3(splat: 0.4 * m * r * r)
    case .capsule(let r, let hh):
        // A cylinder and two half spheres, mass shared by volume.
        let h = hh * 2
        let vc = float32(math.Pi) * r * r * h, vs = 4 / 3 * float32(math.Pi) * r * r * r
        let mc = m * vc / (vc + vs), ms = m * vs / (vc + vs)
        let iy = mc * r * r / 2 + ms * 0.4 * r * r
        let ix = mc * (r * r / 4 + h * h / 12) + ms * (0.4 * r * r + h * h / 4 + 3 * h * r / 8)
        return geom.Vec3(ix, iy, ix)
    case .box(let he):
        let x = he.X * 2, y = he.Y * 2, z = he.Z * 2
        return geom.Vec3(m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y))
    case .plane, .heightfield(_):
        return geom.Vec3.Zero
    }
}

/// Raycast is where a ray first meets a placed shape, as the distance
/// along its (unit) direction and the surface normal there; nil when it
/// misses within maxDistance or starts inside.
public func Raycast(_ s: Shape, _ t: geom.Transform, origin: geom.Vec3, direction: geom.Vec3,
                    maxDistance: float32) -> (distance: float32, normal: geom.Vec3)? {
    // In the shape's space.
    let inv = t.Rotation.Conjugate()
    let o = inv.Rotate(origin - t.Position), d = inv.Rotate(direction)
    var hit: float32 = -1
    var n = geom.Vec3.Zero
    switch s {
    case .plane:
        if o.Y <= 0 || d.Y >= 0 { return nil }
        hit = -o.Y / d.Y
        n = geom.Vec3.UnitY
    case .heightfield(let f):
        // In the world, less the collider's position (it isn't turned).
        let from = origin - t.Position
        guard let h = f.Raycast(origin: from, direction: direction, maxDistance: maxDistance) else { return nil }
        let p = from + direction.Normalized() * h
        return (distance: h, normal: f.Normal(p.X, p.Z))
    case .sphere(let r):
        if let h = raySphere(o, d, geom.Vec3.Zero, r) { hit = h; n = (o + d * h).Normalized() }
    case .capsule(let r, let hh):
        // The nearest of the two end spheres and the side.
        var best: float32 = -1
        for c in [geom.Vec3(0, -hh, 0), geom.Vec3(0, hh, 0)] {
            if let h = raySphere(o, d, c, r), best < 0 || h < best { best = h; n = (o + d * h - c).Normalized() }
        }
        let a = d.X * d.X + d.Z * d.Z, b = o.X * d.X + o.Z * d.Z, c = o.X * o.X + o.Z * o.Z - r * r
        if a > 1e-12 && b * b - a * c >= 0 {
            let h = (-b - math.Sqrt(b * b - a * c)) / a
            let y = o.Y + d.Y * h
            if h >= 0 && y >= -hh && y <= hh && (best < 0 || h < best) {
                best = h
                n = geom.Vec3(o.X + d.X * h, 0, o.Z + d.Z * h).Normalized()
            }
        }
        hit = best
    case .box(let he):
        let box = geom.AABB(min: -he, max: he)
        if box.Contains(o) { return nil }
        if let h = geom.Ray(origin: o, direction: d).Intersect(box) {
            hit = h
            let p = o + d * h
            let q = geom.Vec3(p.X / he.X, p.Y / he.Y, p.Z / he.Z)
            let ax = math.Abs(q.X), ay = math.Abs(q.Y), az = math.Abs(q.Z)
            n = ax >= ay && ax >= az ? geom.Vec3(math.CopySign(1, q.X), 0, 0)
                : (ay >= az ? geom.Vec3(0, math.CopySign(1, q.Y), 0) : geom.Vec3(0, 0, math.CopySign(1, q.Z)))
        }
    }
    if hit < 0 || hit > maxDistance { return nil }
    return (distance: hit, normal: t.Rotation.Rotate(n))
}

// ---- pairs ----

// segment is a sphere or capsule as its core segment and radius, in the
// world (a sphere's segment is a point; a box's isn't used).
func segment(_ s: Shape, _ t: geom.Transform) -> (geom.Vec3, geom.Vec3, float32) {
    switch s {
    case .sphere(let r):
        return (t.Position, t.Position, r)
    case .capsule(let r, let h):
        let axis = t.Rotation.Rotate(geom.Vec3(0, h, 0))
        return (t.Position - axis, t.Position + axis, r)
    default:
        return (t.Position, t.Position, 0)
    }
}

func flip(_ cs: [Contact]) -> [Contact] {
    var out: [Contact] = []
    for c in cs { out.append(Contact(PointA: c.PointB, PointB: c.PointA, Normal: -c.Normal, Depth: c.Depth)) }
    return out
}

// againstPlane is a shape against the half-space below the plane's local
// Y = 0. The normal points into the plane (from the shape to it).
func againstPlane(_ s: Shape, _ ts: geom.Transform, _ tp: geom.Transform, _ margin: float32) -> [Contact] {
    let up = tp.Rotation.Rotate(geom.Vec3.UnitY)
    let d0 = up.Dot(tp.Position)
    var out: [Contact] = []
    func test(_ p: geom.Vec3, _ r: float32) {
        let h = up.Dot(p) - d0 - r
        if h <= margin {
            let deepest = p - up * r
            out.append(Contact(PointA: deepest, PointB: deepest - up * h, Normal: -up, Depth: math.Max(-h, 0)))
        }
    }
    switch s {
    case .sphere(let r):
        test(ts.Position, r)
    case .capsule(let r, let hh):
        let axis = ts.Rotation.Rotate(geom.Vec3(0, hh, 0))
        test(ts.Position - axis, r)
        test(ts.Position + axis, r)
    case .box(let he):
        for i in 0..<8 {
            let c = geom.Vec3((i & 1) != 0 ? he.X : -he.X, (i & 2) != 0 ? he.Y : -he.Y, (i & 4) != 0 ? he.Z : -he.Z)
            test(ts.Point(c), 0)
        }
    case .plane, .heightfield(_):
        break
    }
    return out
}

// againstGround is a shape against a heightfield moved by `offset`. Each
// point tested (a sphere's center, a capsule's ends and middle, a box's
// corners) meets the ground as the plane under it facing the field's
// normal. Normals point into the ground (from the shape to it).
func againstGround(_ s: Shape, _ ts: geom.Transform, _ f: height.Field, _ offset: geom.Vec3, _ margin: float32) -> [Contact] {
    var out: [Contact] = []
    func test(_ p: geom.Vec3, _ r: float32) {
        let x = p.X - offset.X, z = p.Z - offset.Z
        if !f.Contains(x, z) { return }
        let ground = geom.Vec3(p.X, f.Sample(x, z) + offset.Y, p.Z)
        let up = f.Normal(x, z)
        let above = (p - ground).Dot(up) - r
        if above <= margin {
            let deepest = p - up * r
            out.append(Contact(PointA: deepest, PointB: deepest - up * above, Normal: -up, Depth: math.Max(-above, 0)))
        }
    }
    switch s {
    case .sphere(let r):
        test(ts.Position, r)
    case .capsule(let r, let hh):
        let axis = ts.Rotation.Rotate(geom.Vec3(0, hh, 0))
        test(ts.Position - axis, r)
        test(ts.Position + axis, r)
        // A capsule lying across a hump touches it in the middle.
        if hh > r { test(ts.Position, r) }
    case .box(let he):
        for i in 0..<8 {
            let c = geom.Vec3((i & 1) != 0 ? he.X : -he.X, (i & 2) != 0 ? he.Y : -he.Y, (i & 4) != 0 ? he.Z : -he.Z)
            test(ts.Point(c), 0)
        }
    case .plane, .heightfield(_):
        break
    }
    return out
}

// segmentBox is a sphere or capsule (segment p–q, radius r) against a box
// of half extents he placed by tb.
func segmentBox(_ p: geom.Vec3, _ q: geom.Vec3, _ r: float32, _ he: geom.Vec3, _ tb: geom.Transform, _ margin: float32) -> Contact? {
    let inv = tb.Inverse()
    let a = inv.Point(p), b = inv.Point(q)
    let box = geom.AABB(min: -he, max: he)
    // The closest pair between a segment and a box: alternate projections,
    // which settle quickly for convex sets.
    var t: float32 = 0.5
    var onBox = box.Closest(geom.Lerp(a, b, t))
    for _ in 0..<6 {
        t = geom.ClosestOnSegment(onBox, a, b)
        onBox = box.Closest(geom.Lerp(a, b, t))
    }
    let s = geom.Lerp(a, b, t)
    var d = onBox - s
    let dist = d.Length()
    var depth: float32
    var n: geom.Vec3
    if dist > 1e-6 {
        n = d / dist
        depth = r - dist
    } else {
        // The segment's point is inside the box: leave by the nearest face.
        let gaps = [he.X - s.X, he.X + s.X, he.Y - s.Y, he.Y + s.Y, he.Z - s.Z, he.Z + s.Z]
        var k = 0
        for i in 1..<6 where gaps[i] < gaps[k] { k = i }
        let dirs = [geom.Vec3(1, 0, 0), geom.Vec3(-1, 0, 0), geom.Vec3(0, 1, 0), geom.Vec3(0, -1, 0), geom.Vec3(0, 0, 1), geom.Vec3(0, 0, -1)]
        // The normal points from the segment into the box: opposite the
        // way out.
        n = -dirs[k]
        depth = r + gaps[k]
        onBox = s + dirs[k] * gaps[k]
        d = n
    }
    if depth < -margin { return nil }
    let pa = s + n * r
    return Contact(PointA: tb.Point(pa), PointB: tb.Point(onBox), Normal: tb.Rotation.Rotate(n), Depth: math.Max(depth, 0))
}

func raySphere(_ o: geom.Vec3, _ d: geom.Vec3, _ c: geom.Vec3, _ r: float32) -> float32? {
    let m = o - c
    let b = m.Dot(d), cc = m.Dot(m) - r * r
    if cc > 0 && b > 0 { return nil }
    let disc = b * b - cc
    if disc < 0 { return nil }
    return math.Max(0, -b - math.Sqrt(disc))
}
