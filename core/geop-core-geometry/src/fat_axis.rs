//! Fat-line/fat-plane separating-axis prefilter: a cheap, non-iterative
//! sufficient test for "these two segments definitely don't overlap",
//! tried before falling back to the expensive iterative GJK check.
//!
//! Unlike `aabb`'s world-axis-aligned box (which is cheap but often loose
//! for a diagonal/slanted curve segment), this tests along the segment's
//! *own* natural direction — the chord through its endpoints for a curve,
//! or the plane through 3 corners for a surface patch — which is often a
//! far more effective separating axis for the kind of thin, non-axis-
//! aligned geometry this crate's curves/surfaces actually are.
//!
//! **Soundness**: this is the classical separating axis theorem, applied
//! along one fixed, cheaply-computed candidate axis rather than searched
//! for iteratively like GJK does. By the same convex-hull property
//! `NurbCurve::convex_hull()`'s own doc comment relies on, every point of
//! a curve/surface lies within the convex hull of its control points — so
//! projecting the *other* object's control points onto this object's own
//! axis and comparing against this object's own projected band is a sound
//! (if not exhaustive) overlap test: if the projected ranges don't
//! overlap, the true hulls can't either. It just isn't guaranteed to find
//! a separating axis when one exists (unlike GJK, which always does) — so
//! a "could overlap" result here is never trusted as a final answer, only
//! ever as "fall through to the real check."

use geop_core_math::{scalars::Scalar, vector::Vector};

use crate::{
    nurb_curve::{NurbCurve, dehomogenize},
    nurb_surface::NurbSurface,
};

/// An axis (`origin`, unit `normal`) plus `band` — the union of this
/// object's own control points' signed distances along it. `band` is a
/// single `S`, not a separate `(lo, hi)` pair, for the same reason
/// `aabb::compute_aabb` uses one: `S` is itself an interval type, so
/// folding every point's distance into one `S` via `Scalar::union` already
/// carries the full `[lo, hi]` extent.
struct FatAxis<S: Scalar, const C: usize> {
    origin: Vector<S, C>,
    normal: Vector<S, C>,
    band: S,
}

fn signed_distance<S: Scalar, const C: usize>(axis: &FatAxis<S, C>, point: &Vector<S, C>) -> S {
    point.sub(&axis.origin).prod_dot(&axis.normal)
}

fn fat_axis_from_points<S: Scalar, const C: usize>(
    origin: Vector<S, C>,
    normal: Vector<S, C>,
    control_points: &[Vector<S, C>],
) -> FatAxis<S, C> {
    let mut acc: Option<S> = None;
    for p in control_points {
        let d = p.sub(&origin).prod_dot(&normal);
        acc = Some(match acc {
            None => d,
            Some(a) => a.union(d),
        });
    }
    FatAxis {
        origin,
        normal,
        band: acc.unwrap_or(S::ZERO),
    }
}

/// True if `other_points`, projected onto `axis`, overlaps `axis`'s own
/// band — i.e. this axis does *not* prove separation. `false` is a hard
/// proof of separation (as sound as `ConvexHull::definitely_no_overlap`,
/// just along one fixed axis instead of an iteratively-discovered one).
fn axis_could_overlap<S: Scalar, const C: usize>(
    axis: &FatAxis<S, C>,
    other_points: &[Vector<S, C>],
) -> bool {
    let mut acc: Option<S> = None;
    for p in other_points {
        let d = signed_distance(axis, p);
        acc = Some(match acc {
            None => d,
            Some(a) => a.union(d),
        });
    }
    match acc {
        Some(other_band) => axis.band.could_be_equal(other_band),
        None => true,
    }
}

/// The 2-D fat line through `points[0]`/`points[last]`: axis = that
/// chord, normal = the chord rotated 90°. `None` on coincident endpoints
/// (chord `normalize()` fails) — no axis to test with then.
fn fat_line_2d<S: Scalar>(points: &[Vector<S, 2>]) -> Option<FatAxis<S, 2>> {
    let chord = points[points.len() - 1].sub(&points[0]);
    let perp = Vector::<S, 2>::from_array([chord[1].neg(), chord[0]]);
    let normal = perp.normalize().ok()?;
    Some(fat_axis_from_points(points[0], normal, points))
}

/// The two independent 3-D fat planes for a curve: axis direction = the
/// chord through `points[0]`/`points[last]`; the two perpendiculars come
/// from crossing that direction with an arbitrary helper vector (`[1,0,0]`,
/// falling back to `[0,1,0]` if that's parallel to the chord — both can't
/// be parallel to the same nonzero direction), then crossing again for the
/// second. `None` on coincident endpoints.
fn fat_planes_3d<S: Scalar>(points: &[Vector<S, 3>]) -> Option<(FatAxis<S, 3>, FatAxis<S, 3>)> {
    let chord = points[points.len() - 1].sub(&points[0]);
    let dir = chord.normalize().ok()?;
    let helper = Vector::<S, 3>::from_array([S::ONE, S::ZERO, S::ZERO]);
    let n1_raw = dir.prod_cross(&helper);
    let n1_raw = if n1_raw.norm_sq().could_be_equal(S::ZERO) {
        let helper2 = Vector::<S, 3>::from_array([S::ZERO, S::ONE, S::ZERO]);
        dir.prod_cross(&helper2)
    } else {
        n1_raw
    };
    let n1 = n1_raw.normalize().ok()?;
    let n2 = dir.prod_cross(&n1).normalize().ok()?;
    let origin = points[0];
    Some((
        fat_axis_from_points(origin, n1, points),
        fat_axis_from_points(origin, n2, points),
    ))
}

/// The fat plane through a surface patch's 3 corner control points
/// (indices `[0, (nu-1)*nv, nv-1]`, same corners `contains::surface`'s
/// `corner_extents` uses): axis = that plane, normal = the cross product
/// of its two edges. `band` folds in *every* control point (not just the
/// 3 corners), so a warped/twisted patch correctly gets a wide (still
/// sound) band rather than a falsely tight one. `None` on collinear
/// corners.
fn fat_plane_from_surface_points<S: Scalar>(
    corner_idx: [usize; 3],
    all_points: &[Vector<S, 3>],
) -> Option<FatAxis<S, 3>> {
    let p00 = all_points[corner_idx[0]];
    let pn0 = all_points[corner_idx[1]];
    let p0m = all_points[corner_idx[2]];
    let v1 = pn0.sub(&p00);
    let v2 = p0m.sub(&p00);
    let normal = v1.prod_cross(&v2).normalize().ok()?;
    Some(fat_axis_from_points(p00, normal, all_points))
}

/// Bridges `NurbCurve<S, D>`'s pair of concrete "does my fat axis/axes
/// prove separation from `other_points`" impls (`D=4` → two 3-D fat
/// planes, `D=3` → one 2-D fat line), mirroring `HasConvexHull`'s
/// identical `D`-vs-`C` bridging so `intersection::curve_curve::dfs`,
/// generic over both, can call this via the same
/// `where NurbCurve<S, D>: HasFatAxes<S, C>` pattern it already uses for
/// `HasConvexHull`.
pub trait HasFatAxes<S: Scalar, const C: usize> {
    /// True if this curve's own fat axis/axes prove `other_points` is
    /// separated from it. `false` is never a proof of overlap, only "this
    /// axis didn't resolve it" — the caller must still fall through to
    /// the exact hull/GJK check in that case.
    fn fat_axes_separate(&self, other_points: &[Vector<S, C>]) -> bool;
}

impl<S: Scalar> HasFatAxes<S, 2> for NurbCurve<S, 3> {
    fn fat_axes_separate(&self, other_points: &[Vector<S, 2>]) -> bool {
        let own_points = dehomogenize::<S, 3, 2>(&self.control_points);
        match fat_line_2d(&own_points) {
            Some(axis) => !axis_could_overlap(&axis, other_points),
            None => false,
        }
    }
}

impl<S: Scalar> HasFatAxes<S, 3> for NurbCurve<S, 4> {
    fn fat_axes_separate(&self, other_points: &[Vector<S, 3>]) -> bool {
        let own_points = dehomogenize::<S, 4, 3>(&self.control_points);
        match fat_planes_3d(&own_points) {
            Some((a1, a2)) => {
                !axis_could_overlap(&a1, other_points) || !axis_could_overlap(&a2, other_points)
            }
            None => false,
        }
    }
}

/// The curve-surface variant: does `patch`'s own fat plane (its 3
/// corners) prove `curve_points` is separated from it?
pub(crate) fn surface_fat_plane_separates<S: Scalar>(
    patch: &NurbSurface<S, 4>,
    curve_points: &[Vector<S, 3>],
) -> bool {
    let surface_points = dehomogenize::<S, 4, 3>(&patch.control_points);
    let nu = patch.num_u();
    let nv = patch.num_v();
    let corner_idx = [0, (nu - 1) * nv, nv - 1];
    match fat_plane_from_surface_points(corner_idx, &surface_points) {
        Some(axis) => !axis_could_overlap(&axis, curve_points),
        None => false,
    }
}

/// The other curve-surface direction: does `curve`'s own fat axis/axes
/// prove separation from `surface_points`?
pub(crate) fn curve_fat_axes_separate<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surface_points: &[Vector<S, 3>],
) -> bool {
    curve.fat_axes_separate(surface_points)
}
