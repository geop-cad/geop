//! Cheap axis-aligned bounding box helpers shared by [`crate::nurb_curve`]
//! and [`crate::nurb_surface`] — see `NurbCurve`/`NurbSurface`'s own `aabb`
//! field doc comment for why one padded-to-3 `[S; 3]` per curve/surface is
//! enough to serve as a full bounding box under interval arithmetic.

use geop_core_math::{scalars::Scalar, vector::Vector};

use crate::nurb_curve::NurbCurve;

/// Axis-aligned bounding box of `control_points`'s dehomogenized Cartesian
/// coordinates (`D - 1` axes), padded to 3 with [`Scalar::ENTIRE`] so both
/// 2-D pcurves (`D = 3`) and 3-D curves/surfaces (`D = 4`) share one result
/// type — callers only ever read the first `D - 1` (see
/// [`aabb_could_overlap`]'s own `c` parameter).
///
/// Each axis's bound is a *single* `S`, not a separate `(min, max)` pair:
/// `S` is itself an interval type here, so folding every control point's
/// coordinate on that axis into one `S` via [`Scalar::union`] already
/// produces exactly that axis's `[min, max]` extent — "3 scalar values" is
/// the whole bounding box.
///
/// Every weight is definitely positive by construction
/// (`NurbCurve`/`NurbSurface::try_new`), so dehomogenizing cannot fail.
pub(crate) fn compute_aabb<S: Scalar, const D: usize>(control_points: &[Vector<S, D>]) -> [S; 3] {
    let mut acc: [Option<S>; 3] = [None; 3];
    for p in control_points {
        // One reciprocal per control point, shared by all its axes.
        let inv_w = S::ONE
            .div(p[D - 1])
            .expect("weights are definitely positive by construction");
        for (axis, slot) in acc.iter_mut().enumerate().take(D - 1) {
            let coord = p[axis].mul(inv_w);
            *slot = Some(match *slot {
                None => coord,
                Some(a) => a.union(coord),
            });
        }
    }
    acc.map(|a| a.unwrap_or(S::ENTIRE))
}

/// True if two cached [`compute_aabb`] boxes could overlap, comparing only
/// the first `c` (the objects' actual Cartesian dimension) of the 3 padded
/// axes. `false` is a hard proof of separation, and cheap: `O(c)` scalar
/// `could_be_equal`s against an already-cached value.
pub(crate) fn aabb_could_overlap<S: Scalar>(a: &[S; 3], b: &[S; 3], c: usize) -> bool {
    (0..c).all(|i| a[i].could_be_equal(b[i]))
}

/// True if a cached [`compute_aabb`] box could contain `point` — the same
/// check as [`aabb_could_overlap`], with `point` standing in for a
/// zero-extent "box" of its own (each of its `C` coordinates is itself an
/// interval already, so no separate point-vs-box case is needed).
pub(crate) fn aabb_could_contain<S: Scalar, const C: usize>(
    aabb: &[S; 3],
    point: &Vector<S, C>,
) -> bool {
    (0..C).all(|i| aabb[i].could_be_equal(point[i]))
}

/// True if `curve` could meet the cached box `aabb` in its first `c` axes.
///
/// For a straight segment — degree one between two control points, which
/// traces the segment between them whatever their weights — whether the
/// segment itself could: far tighter than its own box when it runs
/// diagonally, and a ray cast across a solid to classify a point is a long
/// diagonal segment, whose box overlaps nearly every face's. Each axis
/// confines the segment's parameter to where it lies within the box's
/// extent on that axis; the segment meets the box only if those ranges
/// and `[0, 1]` share a point. For any other curve, whether the two boxes
/// could overlap. `false` is a hard proof of separation either way.
pub(crate) fn curve_could_meet_aabb<S: Scalar, const D: usize>(
    curve: &NurbCurve<S, D>,
    aabb: &[S; 3],
    c: usize,
) -> bool {
    if !aabb_could_overlap(&curve.aabb, aabb, c) {
        return false;
    }
    let [p, q] = curve.control_points[..] else {
        return true;
    };
    if curve.degree != 1 {
        return true;
    }
    let point = |cp: Vector<S, D>, i: usize| cp[i].div(cp[D - 1]);
    let mut range = S::ZERO.union(S::ONE);
    for (i, extent) in aabb.iter().enumerate().take(c) {
        let (Ok(a), Ok(b)) = (point(p, i), point(q, i)) else {
            return true;
        };
        let d = b.sub(a);
        if d.could_be_equal(S::ZERO) {
            // Hardly moving along this axis, the segment's parameter is not
            // confined by it; the boxes overlap on it, checked above.
            continue;
        }
        let (Ok(t0), Ok(t1)) = (extent.lower().sub(a).div(d), extent.upper().sub(a).div(d)) else {
            continue;
        };
        let along = t0.union(t1);
        if !along.could_be_equal(range) {
            return false;
        }
        range = along.intersect(range);
    }
    true
}
