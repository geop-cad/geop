//! Revolve a planar profile, given as `(r, z)` points in the half-plane `r
//! >= 0` (whose first and last points must be poles, `r = 0`), 360 degrees
//! around a vertical axis, entirely from euler operations — built one
//! angular quadrant *column* at a time (all `P = profile.len() - 1` row
//! faces of a given 90-degree wedge, before moving to the next wedge),
//! exactly the same strategy [`sphere_octants`](super::sphere::sphere_octants)
//! uses for its own dedicated (exact, doubly-curved) construction, just
//! generalized to an arbitrary profile instead of a fixed 2-segment
//! pole-equator-pole one.
//!
//! One persistent face (`mvfs`'s own, playing the same role `extrude`'s
//! placeholder does) always holds whatever's still left to close off. The
//! very first column (meridian 0, the profile's own `P`-edge chain, straight
//! in the profile direction) is bootstrapped directly from `mvfs`'s vertex.
//! Each of the next 2 columns grows a fresh meridian one angular "beam" arc
//! at a time (one per non-pole row) and immediately closes every row's own
//! degree-(2, 1) quadrant face (`mve` for the beam, `mef` — minting that
//! row's new meridian edge as it goes — for the face) against the *previous*
//! meridian. The 4th and last column closes back onto the very first
//! meridian's own reversed edges instead of growing a new one — only the
//! interior beams (angle 3 `->` 0) are still new, one per non-last row's own
//! closing `mef` — and its very last row needs no `mef` at all: by the time
//! every other row of that column has closed, its own boundary is already
//! sitting complete on the placeholder face, so `replace_face` alone turns
//! it real. No separate degenerate cap is ever needed, unlike a naive
//! row-by-row sweep (which always leaves one extra zero-area face behind at
//! the final pole).
//!
//! A profile row at `r = 0` (only ever the first/last, both required poles)
//! collapses to a single shared vertex — no angular beam is grown for it at
//! all, and its neighboring rows' own meridian edges connect straight to
//! that one shared vertex instead, both euler ops here (`mve`, `mef`) taking
//! existing vertices as-is rather than minting fresh ones.

use crate::common::{
    Profile, arc3, embed_curve, embed_point, end_point, line2, sqrt2_over_2, start_point,
};
use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_part::{Namer, Part};
use geop_core_topology::{CoedgeId, SolidId};

/// Bridges a quadrant face's own `(u, v)` boundary-loop gap at a pole row
/// it touches on its `v = 0` (top) side — the row's two meridian edges meet
/// directly at the shared pole vertex (a real, correctly-shared 3-D vertex,
/// nothing wrong there), but nothing was ever built to represent that
/// row's own zero-length angular span in *parameter* space, so the loop
/// jumps straight from `u = 0` to `u = 1` without a pcurve covering the
/// gap between them — invisible to per-edge/per-coedge validation (every
/// individual coedge's own pcurve is perfectly valid), but fatal to
/// `contains::face::face_contains`'s ray-casting, which relies on the
/// coedges' pcurves alone tracing a fully closed 2-D polygon.
///
/// `at` is the coedge immediately *after* the gap in its current loop (the
/// row's own pre-existing meridian coedge, reused directly in place of a
/// real angular beam since a pole has no angular extent) — the bridge is
/// inserted right before it, via [`Part::add_vertex_coedge`] onto
/// `at.prev` (whatever's on the gap's other side), sitting at the same
/// shared pole vertex the whole way rather than minting any new edge or
/// vertex of its own.
pub(crate) fn close_top_pole_gap<S: Scalar>(part: &mut Part<S>, at: CoedgeId) -> GeopResult<()> {
    let before = part.topology().get_coedge(at)?.prev;
    let pole = part.topology().coedge_end_vertex_id(before)?;
    part.add_vertex_coedge(before, pole, beam_pcurve_top()?)
        .with_context("revolve_at: closing top-pole pcurve gap failed")?;
    Ok(())
}
/// Like [`close_top_pole_gap`], but for a pole row touched on a quadrant's
/// `v = 1` (bottom) side: the gap sits right *after* `at` (the row's own
/// meridian coedge) instead of right before it, so the bridge grows from
/// `at` itself.
pub(crate) fn close_bottom_pole_gap<S: Scalar>(part: &mut Part<S>, at: CoedgeId) -> GeopResult<()> {
    let pole = part.topology().coedge_end_vertex_id(at)?;
    part.add_vertex_coedge(at, pole, beam_pcurve_bottom()?)
        .with_context("revolve_at: closing bottom-pole pcurve gap failed")?;
    Ok(())
}

const N: usize = 4;

/// The pcurve an "old" (already-built, at whichever angle is currently
/// leading) meridian coedge carries, on *whichever* quadrant it ends up
/// on: `u = 1` (the angularly-later side of that quadrant's own arc), `v: 0
/// -> 1` (top row to bottom row). Every meridian coedge here plays this
/// exact role, so one constant works for all of them.
fn meridian_pcurve<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>> {
    line2(
        Vector2::from_array([S::ONE, S::ZERO]),
        Vector2::from_array([S::ONE, S::ONE]),
    )
}
/// The pcurve a `mef`'s own new meridian edge carries, on the quadrant it
/// closes off: `u = 0` (the angularly-earlier side), `v: 1 -> 0` (bottom
/// row back to top row — the boundary loop runs the opposite way around
/// this side, same reason a plain rectangle's own right edge runs
/// top-to-bottom even though its top edge ran left-to-right).
fn meridian_closing_pcurve<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>>
{
    line2(
        Vector2::from_array([S::ZERO, S::ONE]),
        Vector2::from_array([S::ZERO, S::ZERO]),
    )
}
/// The pcurve a row's angular beam carries as the *upper* quadrant's own
/// `v = 1` (bottom) edge: `u: 1 -> 0` (old angle to new).
fn beam_pcurve_bottom<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>> {
    line2(
        Vector2::from_array([S::ONE, S::ONE]),
        Vector2::from_array([S::ZERO, S::ONE]),
    )
}
/// The pcurve the same beam carries (reversed) as the *lower* quadrant's
/// own `v = 0` (top) edge: `u: 0 -> 1` (new angle back to old).
fn beam_pcurve_top<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>> {
    line2(
        Vector2::from_array([S::ZERO, S::ZERO]),
        Vector2::from_array([S::ONE, S::ZERO]),
    )
}

/// An exact 90-degree arc, around `center`, from `p0` to `p1` — one row's
/// own angular "beam" edge between two adjacent quadrant columns.
fn beam_curve<S: Scalar>(
    p0: Vector3<S>,
    p1: Vector3<S>,
    center: Vector3<S>,
    w: S,
) -> GeopResult<NurbCurve3D<S>> {
    let mid = p0.add(&p1).sub(&center);
    arc3(p0, mid, p1, w)
}

/// The quadrant of the surface of revolution swept by the profile curve
/// `curve` (in `(r, z)`) from angle `angle_new` back to `angle_old`, where
/// `dirs[k]` is the in-plane direction of angle `k`.
///
/// The tensor product of `curve` with an exact 90-degree arc: `u` runs along
/// the arc (degree 2, from `angle_new` to `angle_old`), `v` along `curve`
/// (its own degree and knots). A profile control point `(w r, w z, w)` sweeps
/// the arc `P(angle_new), P(angle_new) + P(angle_old) - center, P(angle_old)`
/// with weights `w, w √2/2, w`; at `r = 0` all three collapse onto the axis,
/// so poles need no special case.
fn quadrant_patch<S: Scalar>(
    curve: &NurbCurve2D<S>,
    coordinate_system: &CoordinateSystem<S>,
    dirs: &[Vector3<S>; N],
    angle_new: usize,
    angle_old: usize,
) -> GeopResult<NurbSurface<S, 4>> {
    let (o, axis) = (coordinate_system.origin(), coordinate_system.w());
    let (d_new, d_old) = (dirs[angle_new], dirs[angle_old]);
    let w = sqrt2_over_2::<S>();
    let mid = d_new.add(&d_old);
    let rows = [(d_new, S::ONE), (mid, w), (d_old, S::ONE)];
    let control_points = rows
        .iter()
        .flat_map(|(d, weight)| {
            curve.control_points.iter().map(move |cp| {
                let p = embed_point(cp, o, d, axis);
                Vector4::from_array([
                    p[0].mul(*weight),
                    p[1].mul(*weight),
                    p[2].mul(*weight),
                    p[3].mul(*weight),
                ])
            })
        })
        .collect();
    NurbSurface::try_new(
        2,
        curve.degree,
        control_points,
        vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
        curve.knot_vector.clone(),
    )
}

/// Revolve `profile` — an open chain of curves in the `(r, z)` half-plane
/// `r >= 0`, starting and ending on the axis (`r = 0`), each clamped and on
/// the domain `[0, 1]` — 360 degrees around a vertical axis through `origin`
/// (parallel to the z-axis), producing a closed, manifold solid of exactly
/// `4 * profile.len()` faces (no leftover degenerate one) — `origin` is added
/// to every generated point, so the profile's own `z` values are relative to
/// `origin`'s `z`. See the module doc for the overall column-by-column
/// strategy, and [`revolve_at_oriented`] for how the result is named.
///
/// The profile runs "top-down": walked from its first point to its last,
/// the region it bounds together with the axis lies on its right (e.g.
/// `(0, h) -> (r, h) -> (r, 0) -> (0, 0)` for a cylinder). Walked the other
/// way, the solid comes out inside-out.
///
/// A thin wrapper around [`revolve_at_oriented`] with the identity
/// (z-axis) coordinate system — see that function to revolve around an
/// arbitrary axis (e.g. [`super::cylinder::revolved_cylinder_along_axis`]).
pub fn revolve_at<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    profile: &Profile<S>,
    origin: Vector3<S>,
) -> GeopResult<SolidId> {
    let identity = CoordinateSystem::try_new(
        origin,
        Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
        Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
        Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
    )
    .expect("axis-aligned basis is never degenerate");
    revolve_at_oriented(part, namer, &namer.root(), profile, &identity)
}

/// Like [`revolve_at`], but revolves around `coordinate_system`'s own `w`
/// axis instead of always the ambient z-axis: a profile point `(r, z)` maps
/// to `coordinate_system.to_xyz([r cos, r sin, z])` — `u`/`v` span the
/// equatorial plane (the angle-0 direction and its 90-degree-around-`w`
/// follower, respectively) and `w` is the revolution axis, exactly the
/// same role `extrude`'s own coordinate system's `w` plays as its sweep
/// direction. Every position this builds still ultimately comes from this
/// one `pos`/`centers` pair — the rest of the function (euler operations,
/// pcurves) has no notion of x/y/z at all, so generalizing the axis needed
/// no changes anywhere else.
///
/// Everything built is named after the profile's curves `X` and joints `P`
/// (see [`Profile`]), the angles `a0..a3` at which the meridians lie
/// (`a0` along `u`, `a1` along `v`, ...), and the quadrants `q0..q3` between
/// them (`q0` from `a0` to `a1`, ...), following `geop_core_part`'s scheme:
///
/// | entity | name |
/// |---|---|
/// | the solid | `solid` (usually `N` itself) |
/// | face swept by `X` through quadrant `q` | `N(X,q)` |
/// | meridian edge: `X` at angle `a` | `N(X,a)` |
/// | circular edge swept by `P` through `q` | `N(P,q)` |
/// | vertex of `P` at angle `a` | `N(P,a)` |
/// | vertex of `P` on the axis (a pole) | `N(P)` |
pub fn revolve_at_oriented<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: &str,
    profile: &Profile<S>,
    coordinate_system: &CoordinateSystem<S>,
) -> GeopResult<SolidId> {
    profile.check_names()?;
    if profile.is_closed() {
        return Err(GeopError::new(
            "revolve: the profile must be an open chain with a name for its end joint",
        ));
    }
    let curves = &profile.curves;
    if curves.is_empty() {
        return Err(GeopError::new(
            "revolve: profile must have at least 1 curve",
        ));
    }
    let p_segments = curves.len();
    // Row `i`: the profile's `i`-th vertex, where curve `i` starts.
    let mut rows = curves
        .iter()
        .map(start_point)
        .collect::<GeopResult<Vec<_>>>()?;
    rows.push(end_point(&curves[p_segments - 1])?);
    for i in 0..p_segments - 1 {
        if !end_point(&curves[i])?.could_be_equal(&rows[i + 1]) {
            return Err(GeopError::new(format!(
                "revolve: profile curve {i} does not end where curve {} starts",
                i + 1
            )));
        }
    }
    let m = rows.len();

    let zero = S::ZERO;
    let one = S::ONE;
    let (u, v) = (coordinate_system.u(), coordinate_system.v());
    let dirs = [
        *u,
        *v,
        u.prod_scalar(zero.sub(one)),
        v.prod_scalar(zero.sub(one)),
    ];

    let degenerate: Vec<bool> = rows.iter().map(|p| p[0].could_be_equal(zero)).collect();
    if !degenerate[0] {
        return Err(GeopError::new(
            "revolve: profile must start at r = 0 (a pole)",
        ));
    }
    if !degenerate[m - 1] {
        return Err(GeopError::new(
            "revolve: profile must end at r = 0 (a pole)",
        ));
    }

    // Names, see the table above.
    let face_name = |i: usize, k: usize| namer.name(&[&profile.curve_names[i], &format!("q{k}")]);
    let meridian_name =
        |i: usize, k: usize| namer.name(&[&profile.curve_names[i], &format!("a{k}")]);
    let beam_name =
        |row: usize, k: usize| namer.name(&[&profile.joint_names[row], &format!("q{k}")]);
    let vertex_name = |row: usize, k: usize| {
        if degenerate[row] {
            namer.name(&[&profile.joint_names[row]])
        } else {
            namer.name(&[&profile.joint_names[row], &format!("a{k}")])
        }
    };

    let centers: Vec<Vector3<S>> = rows
        .iter()
        .map(|p| coordinate_system.to_xyz(&Vector3::from_array([zero, zero, p[1]])))
        .collect();
    let pos = |i: usize, k: usize| -> Vector3<S> {
        if degenerate[i] {
            centers[i]
        } else {
            centers[i].add(&dirs[k].prod_scalar(rows[i][0]))
        }
    };
    // Curve `i` of the profile at angle `k`, from row `i` to row `i + 1`.
    let meridian = |i: usize, k: usize| {
        embed_curve(
            &curves[i],
            coordinate_system.origin(),
            &dirs[k],
            coordinate_system.w(),
        )
    };
    let w = sqrt2_over_2::<S>();

    // The placeholder face becomes the last segment's last quadrant, via
    // `replace_face` at the very end.
    let (v0, face_id, solid_id) = part.mvfs(
        centers[0],
        vertex_name(0, 0),
        face_name(p_segments - 1, N - 1),
        solid,
    )?;

    // Bootstrap meridian 0 (angle index 0): the `p_segments`-edge profile
    // chain from `v0` through every other row, straight in 3-D (the user's
    // own profile segments). `anchors[i]` (forward) gets rebuilt as
    // segment `i`'s own "old" meridian every time a new angle is grown;
    // `mirrors[i]` (reversed) stays untouched, needed only once more, to
    // close the very last angle back onto this first one.
    let mut anchors = Vec::with_capacity(p_segments);
    let mut mirrors = Vec::with_capacity(p_segments);
    let (_, a0, r0, _) = part
        .mve_from_vertex(
            face_id,
            v0,
            meridian(0, 0)?,
            meridian_pcurve()?,
            meridian_closing_pcurve()?,
            pos(1, 0),
            vertex_name(1, 0),
            meridian_name(0, 0),
        )
        .with_context("revolve_at: bootstrap segment 0 failed")?;
    anchors.push(a0);
    mirrors.push(r0);
    for i in 1..p_segments {
        let (_, a, r, _) = part
            .mve(
                anchors[i - 1],
                meridian(i, 0)?,
                meridian_pcurve()?,
                meridian_closing_pcurve()?,
                pos(i + 1, 0),
                vertex_name(i + 1, 0),
                meridian_name(i, 0),
            )
            .with_context(with_context!("revolve_at: bootstrap segment {i} failed"))?;
        anchors.push(a);
        mirrors.push(r);
    }

    // Grow meridians 1..N-1 (angle indices 1, 2, 3), closing all
    // `p_segments` quadrant faces of each new column as it's grown: for
    // every non-pole interior row, one new "beam" (angular arc) edge grows
    // that row across to the new angle (`mve`); each segment's own closing
    // `mef` then mints its own new meridian edge (straight, at the new
    // angle) using whichever of its two rows' beams exist, or the row's
    // shared vertex directly (via the *old* meridian coedge) if a row is a
    // pole.
    for k in 0..N - 1 {
        let k1 = k + 1;
        let mut beam_fwd: Vec<Option<CoedgeId>> = vec![None; p_segments + 1];
        let mut beam_rev: Vec<Option<CoedgeId>> = vec![None; p_segments + 1];
        for row in 1..p_segments {
            if !degenerate[row] {
                let (_, bf, br, _) = part
                    .mve(
                        anchors[row - 1],
                        beam_curve(pos(row, k), pos(row, k1), centers[row], w)?,
                        beam_pcurve_bottom()?,
                        beam_pcurve_top()?,
                        pos(row, k1),
                        vertex_name(row, k1),
                        beam_name(row, k),
                    )
                    .with_context(with_context!(
                        "revolve_at: beam at row {row}, angle {k} -> {k1} failed"
                    ))?;
                beam_fwd[row] = Some(bf);
                beam_rev[row] = Some(br);
            }
        }

        let mut new_anchors = Vec::with_capacity(p_segments);
        for i in 0..p_segments {
            let coedge1 = if degenerate[i + 1] {
                anchors[i]
            } else {
                beam_fwd[i + 1].unwrap()
            };
            let coedge2 = if degenerate[i] {
                anchors[i]
            } else {
                beam_rev[i].unwrap()
            };
            let curve = meridian(i, k1)?.reverse();
            let surface = quadrant_patch(&curves[i], coordinate_system, &dirs, k1, k)?;
            let (_, _, _, coedge_backward) = part
                .mef(
                    coedge1,
                    coedge2,
                    curve,
                    meridian_closing_pcurve()?,
                    meridian_pcurve()?,
                    surface,
                    meridian_name(i, k1),
                    face_name(i, k),
                )
                .with_context(with_context!(
                    "revolve_at: closing segment {i}, angle {k} -> {k1} failed"
                ))?;
            if degenerate[i] {
                close_top_pole_gap(part, coedge2)?;
            }
            if degenerate[i + 1] {
                close_bottom_pole_gap(part, coedge1)?;
            }
            new_anchors.push(coedge_backward);
        }
        anchors = new_anchors;
    }

    // Close the last column (angle 3 -> 0) back onto the very first
    // meridian's own mirrors, reusing `anchors`/`mirrors` directly instead
    // of growing anything new on the meridian side — only the interior
    // beams (angle 3 -> 0) are actually new, one per non-last row's own
    // closing `mef`. The very last segment needs no `mef` at all: after
    // all the others close, its own boundary is already exactly what's
    // left on the placeholder face, so it only needs `replace_face` to
    // become real — the same trick `sphere_octants` uses, no separate
    // degenerate cap required.
    let (k, k1) = (N - 1, 0);
    for i in 0..p_segments - 1 {
        let row = i + 1;
        let curve = beam_curve(pos(row, k), pos(row, k1), centers[row], w)?;
        let surface = quadrant_patch(&curves[i], coordinate_system, &dirs, k1, k)?;
        part.mef(
            anchors[i],
            mirrors[i],
            curve,
            beam_pcurve_bottom()?,
            beam_pcurve_top()?,
            surface,
            beam_name(row, k),
            face_name(i, k),
        )
        .with_context(with_context!(
            "revolve_at: closing final beam at row {row} failed"
        ))?;
        if degenerate[i] {
            close_top_pole_gap(part, anchors[i])?;
        }
    }
    let last = p_segments - 1;
    let surface = quadrant_patch(&curves[last], coordinate_system, &dirs, k1, k)?;
    if degenerate[last] {
        close_top_pole_gap(part, anchors[last])?;
    }
    if degenerate[last + 1] {
        close_bottom_pole_gap(part, anchors[last])?;
    }
    part.replace_face(face_id, surface)
        .with_context("revolve_at: final replace_face failed")?;

    Ok(solid_id)
}

/// `revolve_at` around the z-axis itself (`origin = (0, 0, 0)`).
pub fn revolve<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    profile: &Profile<S>,
) -> GeopResult<SolidId> {
    revolve_at(part, namer, profile, Vector3::from_array([S::ZERO; 3]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{arc2, polyline};
    use geop_core_math::for_all_scalars;
    use geop_core_topology::{
        Model,
        validation::{ValidationParameters, validate, validate_manifold},
    };

    /// Revolve `curves` around the z-axis into a fresh part, as operation `r`.
    fn revolved<S: Scalar>(curves: Vec<NurbCurve2D<S>>) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("revolve", "r").unwrap();
        revolve(&mut part, &namer, &Profile::open(curves)).unwrap();
        part.check_names().unwrap();
        part
    }

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    fn assert_valid<S: Scalar>(model: &Model<S>) {
        let params = ValidationParameters::default();
        if let Err(e) = validate(&params, model) {
            panic!("{e:?}");
        }
        if let Err(e) = validate_manifold(&params, model) {
            panic!("{e:?}");
        }
    }

    /// An exact sphere from two quarter arcs: curved profile edges become
    /// doubly curved quadrant patches.
    fn check_sphere_from_arcs_is_valid<S: Scalar>() {
        let w = sqrt2_over_2::<S>();
        let profile = vec![
            arc2(v2(0.0, 1.0), v2(1.0, 1.0), v2(1.0, 0.0), w).unwrap(),
            arc2(v2(1.0, 0.0), v2(1.0, -1.0), v2(0.0, -1.0), w).unwrap(),
        ];
        let part = revolved(profile);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 8);
        // Two poles, and the equator's ring of four vertices named after the
        // joint between the arcs.
        for name in [
            "revolve(r,p0)",
            "revolve(r,p2)",
            "revolve(r,p1,a0)",
            "revolve(r,p1,a3)",
        ] {
            part.vertex_id(name).unwrap();
        }
        for name in ["revolve(r,c0,a0)", "revolve(r,c1,a2)", "revolve(r,p1,q3)"] {
            part.edge_id(name).unwrap();
        }
        part.face_id("revolve(r,c1,q3)").unwrap();
    }
    #[test]
    fn sphere_from_arcs_is_valid() {
        for_all_scalars!(check_sphere_from_arcs_is_valid);
    }

    /// A vase: a line up the side and a cubic spline bulging out, capped by
    /// lines back to the axis.
    fn check_vase_with_spline_is_valid<S: Scalar>() {
        let hom = |x: f64, y: f64| {
            geop_core_math::vector::Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ONE])
        };
        let spline = geop_core_geometry::nurb_curve::NurbCurve::try_new(
            3,
            vec![hom(0.5, 2.0), hom(1.5, 1.5), hom(0.2, 0.7), hom(1.0, 0.0)],
            vec![
                S::ZERO,
                S::ZERO,
                S::ZERO,
                S::ZERO,
                S::ONE,
                S::ONE,
                S::ONE,
                S::ONE,
            ],
        )
        .unwrap();
        let mut profile = polyline(&[v2(0.0, 2.0), v2(0.5, 2.0)]).unwrap();
        profile.push(spline);
        profile.extend(polyline(&[v2(1.0, 0.0), v2(0.0, 0.0)]).unwrap());
        let part = revolved(profile);
        let model = part.topology();
        assert_valid(model);
        assert_eq!(model.faces.len(), 12);
    }
    #[test]
    fn vase_with_spline_is_valid() {
        for_all_scalars!(check_vase_with_spline_is_valid);
    }

    fn check_cone_is_valid<S: Scalar>() {
        let profile = polyline(&[v2::<S>(0.0, 1.0), v2(1.0, 0.0), v2(0.0, 0.0)]).unwrap();
        assert_valid(revolved(profile).topology());
    }
    #[test]
    fn cone_is_valid() {
        for_all_scalars!(check_cone_is_valid);
    }

    fn check_sphere_is_valid<S: Scalar>() {
        let n = 6;
        let points: Vec<Vector2<S>> = (0..=n)
            .map(|k| {
                if k == 0 || k == n {
                    return v2(0.0, if k == 0 { 1.0 } else { -1.0 });
                }
                let t = std::f64::consts::PI * (k as f64) / (n as f64);
                v2(t.sin(), t.cos())
            })
            .collect();
        assert_valid(revolved(polyline(&points).unwrap()).topology());
    }
    #[test]
    fn sphere_is_valid() {
        for_all_scalars!(check_sphere_is_valid);
    }
}
