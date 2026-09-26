//! Cheap axis-aligned bounding box helpers shared by [`crate::nurb_curve`]
//! and [`crate::nurb_surface`] — see `NurbCurve`/`NurbSurface`'s own `aabb`
//! field doc comment for why one padded-to-3 `[S; 3]` per curve/surface is
//! enough to serve as a full bounding box under interval arithmetic.

use geop_core_math::{scalars::Scalar, vector::Vector};

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
/// A control point whose weight could be zero can't be safely dehomogenized
/// (division by zero) — that axis falls back to [`Scalar::ENTIRE`]
/// (matches-anything, i.e. that control point contributes no pruning power
/// rather than an unsound one). `NurbCurve`/`NurbSurface::try_new` already
/// reject such control points outright, so this only matters for the
/// handful of constructors that build one directly (derivatives, splits),
/// which don't always route through `try_new`.
pub(crate) fn compute_aabb<S: Scalar, const D: usize>(control_points: &[Vector<S, D>]) -> [S; 3] {
    let mut acc: [Option<S>; 3] = [None; 3];
    for p in control_points {
        // One reciprocal per control point, shared by all its axes.
        let w = p[D - 1];
        let inv_w = if w.could_be_equal(S::ZERO) {
            None
        } else {
            S::ONE.div(w).ok()
        };
        for (axis, slot) in acc.iter_mut().enumerate().take(D - 1) {
            let coord = inv_w.map_or(S::ENTIRE, |inv_w| p[axis].mul(inv_w));
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
/// axes. `false` is a hard proof of separation — as sound as (never a false
/// negative relative to) the GJK convex-hull check it's meant to
/// short-circuit, and far cheaper: `O(c)` scalar `could_be_equal`s against
/// an already-cached value, no hull construction, no simplex search.
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
