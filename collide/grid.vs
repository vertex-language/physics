// Package collide is collision detection: whether shapes overlap, and how
// far they can move before they would. It has no dynamics (no mass,
// forces or contacts to solve), so editors, picking and character
// controllers can use it on its own.
//
// This file is boxes against a grid of solid unit cells: what a character
// walking a voxel or tile world needs. The grid is a function, so the
// cells can come from a voxel grid, a tile map, or anything else.
package collide

import (
    "math"
    "math/geom"
)

/// Move is the result of moving a box.
public struct Move {
    /// The box where it ended up.
    public var Box: geom.AABB
    /// How far it actually moved.
    public var Moved: geom.Vec3
    /// The axes on which something stopped it.
    public var BlockedX: bool
    public var BlockedY: bool
    public var BlockedZ: bool

    /// Grounded is whether it was moving down and something stopped it.
    public var Grounded: bool
}

/// Gap is the space left between a box and what it stops against, so the
/// next move doesn't start inside a cell because of rounding.
public let Gap: float32 = 0.0001

/// MoveBox moves `box` by `motion` through a grid whose cell is solid where
/// `solid` says so, sliding along what it hits: Y first, then X, then Z,
/// each clipped on its own. A box that starts overlapping a solid cell
/// isn't pushed out; it just doesn't get further in.
public func MoveBox(box: geom.AABB, by motion: geom.Vec3, solid: (geom.IVec3) -> bool) -> Move {
    var b = box
    var moved = geom.Vec3.Zero
    let dy = sweep(b, axis: 1, motion.Y, solid)
    b = b.Translated(geom.Vec3(0, dy, 0))
    moved.Y = dy
    let dx = sweep(b, axis: 0, motion.X, solid)
    b = b.Translated(geom.Vec3(dx, 0, 0))
    moved.X = dx
    let dz = sweep(b, axis: 2, motion.Z, solid)
    b = b.Translated(geom.Vec3(0, 0, dz))
    moved.Z = dz
    let by = dy != motion.Y
    return Move(Box: b, Moved: moved, BlockedX: dx != motion.X, BlockedY: by, BlockedZ: dz != motion.Z,
                Grounded: by && motion.Y < 0)
}

/// Overlaps is whether the box overlaps any solid cell.
public func Overlaps(box: geom.AABB, solid: (geom.IVec3) -> bool) -> bool {
    let cells = box.Cells()
    var y = cells.lo.Y
    while y < cells.hi.Y {
        var z = cells.lo.Z
        while z < cells.hi.Z {
            var x = cells.lo.X
            while x < cells.hi.X {
                if solid(geom.IVec3(x, y, z)) { return true }
                x += 1
            }
            z += 1
        }
        y += 1
    }
    return false
}

// sweep is how far the box can move by `d` along `axis` (0 X, 1 Y, 2 Z).
func sweep(b: geom.AABB, axis: int, d: float32, solid: (geom.IVec3) -> bool) -> float32 {
    if d == 0 { return 0 }
    // The cells the box covers across the other two axes, shrunk a hair
    // so a box flush against a wall doesn't count the wall's cells.
    let lo = b.Min + geom.Vec3(splat: Gap), hi = b.Max - geom.Vec3(splat: Gap)
    let a1 = (axis + 1) % 3, a2 = (axis + 2) % 3
    let from1 = int32(math.Floor(component(lo, a1))), to1 = int32(math.Floor(component(hi, a1)))
    let from2 = int32(math.Floor(component(lo, a2))), to2 = int32(math.Floor(component(hi, a2)))
    let lead = d > 0 ? component(b.Max, axis) : component(b.Min, axis)
    // The layers of cells the leading face passes into.
    let first = d > 0 ? int32(math.Floor(lead + Gap)) : int32(math.Floor(lead - Gap))
    let last = int32(math.Floor(lead + d))
    let step: int32 = d > 0 ? 1 : -1
    var layer = first
    while d > 0 ? layer <= last : layer >= last {
        var i = from1
        while i <= to1 {
            var j = from2
            while j <= to2 {
                if solid(compose(axis, layer, a1, i, a2, j)) {
                    // Stop flush against this layer, less the gap.
                    let wall = d > 0 ? float32(layer) - Gap : float32(layer + 1) + Gap
                    let allowed = wall - lead
                    return d > 0 ? math.Max(0, math.Min(d, allowed)) : math.Min(0, math.Max(d, allowed))
                }
                j += 1
            }
            i += 1
        }
        layer += step
    }
    return d
}

@inlinable func component(v: geom.Vec3, axis: int) -> float32 {
    switch axis {
    case 0: return v.X
    case 1: return v.Y
    default: return v.Z
    }
}

func compose(axis: int, a: int32, axis1: int, b: int32, axis2: int, c: int32) -> geom.IVec3 {
    var v = geom.IVec3.Zero
    for (ax, value) in [(axis, a), (axis1, b), (axis2, c)] {
        switch ax {
        case 0: v.X = value
        case 1: v.Y = value
        default: v.Z = value
        }
    }
    return v
}
