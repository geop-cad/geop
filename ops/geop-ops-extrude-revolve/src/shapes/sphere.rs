//! A sphere as exactly 8 octant faces: 2 rows (north pole to equator, and
//! equator to south pole) x 4 quadrants, each an exact rational quarter of a
//! hemisphere. No approximation — the surfaces *are* spheres, so a point
//! sampled anywhere on one is exactly `radius` from the centre.
//!
//! Built quadrant-column by quadrant-column rather than row by row, which is
//! what makes the count come out at exactly 8. `mef` always leaves something
//! behind on the face it split from, so a row-by-row construction ends with
//! the starting placeholder still holding an extra, invisible zero-area cap
//! at one pole. Going column-wise makes the *last* quadrant's own closing
//! land back on the placeholder itself, consuming it.
//!
//! That matters beyond tidiness. A leftover placeholder carries
//! [`NurbSurface::everything`], whose every coordinate is `ENTIRE`, so it
//! `could_be_equal`s any point and silently swallows every containment and
//! intersection query aimed at it; and a zero-area face has no interior
//! point to classify, which is precisely what a boolean needs from every
//! face. Both are asserted against in this module's own tests.

use crate::common::{arc3, line2, pt3, sqrt2_over_2};
use geop_core_geometry::nurb_surface::NurbSurface;
use geop_core_math::{
    geop_error::{GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
};
use geop_core_topology::{CoedgeId, SolidId};
use geop_ops::{Namer, Part};

/// Bridges a quadrant face's own `(u, v)` boundary-loop gap at a pole it
/// touches on its `v = 0` side: the face's two meridian edges meet at the
/// shared pole vertex, but nothing represents the pole's zero-length
/// angular span in *parameter* space, so the loop would jump from `u = 0`
/// to `u = 1` without a pcurve covering the gap — fatal to
/// `contains::face::face_contains`, which relies on the pcurves alone
/// tracing a closed polygon. `at` is the coedge right after the gap; the
/// bridge, a degenerate coedge sitting at the pole, goes right before it.
fn close_top_pole_gap<S: Scalar>(part: &mut Part<S>, at: CoedgeId) -> GeopResult<()> {
    let before = part.topology().get_coedge(at)?.prev;
    let pole = part.topology().coedge_end_vertex_id(before)?;
    let pcurve = line2(
        Vector2::from_array([S::ZERO, S::ZERO]),
        Vector2::from_array([S::ONE, S::ZERO]),
    )?;
    part.add_vertex_coedge(before, pole, pcurve)
        .with_context("sphere: closing top-pole pcurve gap failed")?;
    Ok(())
}

/// Like [`close_top_pole_gap`], for a pole on a quadrant's `v = 1` side:
/// the gap sits right *after* `at`, so the bridge grows from `at` itself.
fn close_bottom_pole_gap<S: Scalar>(part: &mut Part<S>, at: CoedgeId) -> GeopResult<()> {
    let pole = part.topology().coedge_end_vertex_id(at)?;
    let pcurve = line2(
        Vector2::from_array([S::ONE, S::ONE]),
        Vector2::from_array([S::ZERO, S::ONE]),
    )?;
    part.add_vertex_coedge(at, pole, pcurve)
        .with_context("sphere: closing bottom-pole pcurve gap failed")?;
    Ok(())
}

// `sphere_quadrant_surface`'s own `u` (equator direction) runs `eq1 -> eq0`
// (`u = 0` at `eq1`, `u = 1` at `eq0`) — backwards from the naive "eq0 is
// u = 0" reading — so that its normal comes out pointing outward, not
// inward; `v` (meridian direction) is unaffected. Every pcurve below whose
// `u` isn't fixed at a single value (or whose fixed value depends on which
// meridian it's on) has that `u` component flipped (`u -> 1 - u`) from
// what the naive reading would suggest, to match.

/// The pcurve a meridian (pole `->` equator) coedge carries, on *whichever*
/// quadrant it ends up on: `u = 1` (its own quadrant's "eq0 side" edge —
/// swapped from the naive "u = 0"), `v: 0 -> 1` (pole to equator). Every
/// meridian coedge here plays this exact role, so one constant works for
/// all of them — including a fresh one's `pcurve_reversed`, since it stays
/// behind on the placeholder face as *next* quadrant's own anchor, needing
/// this same role there too.
fn meridian_pcurve<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>> {
    line2(
        Vector2::from_array([S::ONE, S::ZERO]),
        Vector2::from_array([S::ONE, S::ONE]),
    )
}
/// The pcurve an equator beam carries as a north quadrant's own `v = 1`
/// edge (`u: 1 -> 0`, quadrant `k` to quadrant `k + 1`, swapped from the
/// naive `u: 0 -> 1`).
fn beam_pcurve_north<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>> {
    line2(
        Vector2::from_array([S::ONE, S::ONE]),
        Vector2::from_array([S::ZERO, S::ONE]),
    )
}
/// The pcurve the same beam carries (reversed) as a south quadrant's own
/// `v = 0` edge (`u: 0 -> 1`, quadrant `k + 1` back to quadrant `k`,
/// swapped from the naive `u: 1 -> 0`).
fn beam_pcurve_south<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>> {
    line2(
        Vector2::from_array([S::ZERO, S::ZERO]),
        Vector2::from_array([S::ONE, S::ZERO]),
    )
}
/// The pcurve the meridian `mef` itself closes off with carries, on the new
/// quadrant it belongs to: `u = 0` (its own quadrant's "eq1 side" edge —
/// swapped from the naive `u = 1`), `v: 1 -> 0` (equator back to
/// pole — the boundary loop runs the opposite way around this side, same
/// reason a plain rectangle's own right edge runs top-to-bottom
/// even though its top edge ran left-to-right.
fn meridian_closing_pcurve<S: Scalar>() -> GeopResult<geop_core_geometry::nurb_curve::NurbCurve2D<S>>
{
    line2(
        Vector2::from_array([S::ZERO, S::ONE]),
        Vector2::from_array([S::ZERO, S::ZERO]),
    )
}

/// A degree-(2, 2) patch spanning from `pole` (a degenerate row, `v = 0` if
/// `pole_at_v0` else `v = 1`) to the exact 90-degree equator arc `eq0 ->
/// eq1` (the other row) — doubly-curved (both `u`, the equator direction,
/// *and* `v`, the meridian direction, are exact arcs, unlike a plain
/// pole-to-arc ruled patch, whose straight meridian direction would cut
/// inside the sphere everywhere except right at the pole and the equator):
/// every `(u, v)` sample, not just the boundary, lands exactly `radius`
/// from `center`. A tensor product of two arcs, one of them degenerate: its
/// pole "row" arc has 3 coincident control points, same as
/// `revolve`'s/`row_cps`'s degenerate case.
fn sphere_quadrant_surface<S: Scalar>(
    pole: Vector3<S>,
    eq0: Vector3<S>,
    eq1: Vector3<S>,
    center: Vector3<S>,
    w: S,
    pole_at_v0: bool,
) -> GeopResult<NurbSurface<S, 4>> {
    let eq_mid = eq0.add(&eq1).sub(&center);
    let mer0_mid = pole.add(&eq0).sub(&center);
    let mer1_mid = pole.add(&eq1).sub(&center);
    // NOT `pole + eq_mid - 2*center` (that double-subtracts `center`,
    // invisible only when `center` happens to be the origin): relative to
    // `center`, the correct interior point is `(pole - center) + (eq0 -
    // center) + (eq1 - center)`, translated back by `+ center`.
    let interior = pole.add(&eq0).add(&eq1).sub(&center).sub(&center);
    let m = w.mul(w);

    let wpt =
        |p: Vector3<S>, wt: S| Vector4::from_array([p[0].mul(wt), p[1].mul(wt), p[2].mul(wt), wt]);

    // Each triple is one `u`-slice, `[v = 0, v = 0.5, v = 1]`, assuming the
    // pole sits at `v = 0` — reversed below if it's actually at `v = 1`.
    // `u0`/`u2` (the true `u = 0`/`u = 1` boundary slices) hit the pole as
    // an genuine unweighted corner; `u1` (the interior slice) instead hits
    // it as the *middle* control point of its own (degenerate) `v`-arc,
    // which — like every other arc's own middle control point here — needs
    // weight `w`, not `1`, even though its position also happens to be the
    // pole.
    let mut u0 = [pt3(pole), wpt(mer0_mid, w), pt3(eq0)];
    let mut u1 = [wpt(pole, w), wpt(interior, m), wpt(eq_mid, w)];
    let mut u2 = [pt3(pole), wpt(mer1_mid, w), pt3(eq1)];
    if !pole_at_v0 {
        u0.reverse();
        u1.reverse();
        u2.reverse();
    }

    // `u = 0`/`u = 1` (equator direction) swapped on purpose — `∂S/∂u x
    // ∂S/∂v` with `u`/`v` as built above (equator direction / meridian
    // direction, in that natural order) pointed inward, not outward, and
    // reversing which end of the equator arc is "u = 0" negates the cross
    // product without changing the surface's own shape or touching `v`
    // (the already-verified pole placement above). Every pcurve that
    // depends on `u` is written in this same reversed convention (`u: 0 ->
    // 1` swapped for `u: 1 -> 0` from what "eq0 is u = 0" would naively
    // suggest) to match.
    let knots = vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE];
    let cps = vec![
        u2[0], u2[1], u2[2], u1[0], u1[1], u1[2], u0[0], u0[1], u0[2],
    ];
    NurbSurface::try_new(2, 2, cps, knots.clone(), knots)
}

/// A sphere of `radius` centered at `center`, built as exactly 8 real
/// degree-(2, 2) quadrant faces (no leftover degenerate one) — one column
/// (a north + a south quadrant) at a time, around 4 meridians: `mvfs` plus
/// two `mve`s bootstrap the very first meridian (pole `->` equator `->`
/// pole, i.e. the profile, each half an exact 90-degree arc), then per
/// remaining meridian, one `mve` grows the equator "beam" (also an exact
/// arc) to it and two `mef`s close off that column's north and south
/// quadrant. The very last meridian is the first one again (its own
/// closing `mef`'s new edge *is* the last beam) closing the loop, so its
/// own closing needs only a single `mef` (the north quadrant) — the south
/// one is already exactly the boundary still left on the placeholder face,
/// which just needs `replace_face` to become real.
///
/// Named as the operation `sphere(name)`: the poles `N(north)`/`N(south)`,
/// the equator's vertices `N(equator,a0..a3)` at angles `a0` (`+x`), `a1`
/// (`+y`), ..., its quarter arcs `N(equator,q0..q3)` (`q0` from `a0` to
/// `a1`, ...), the meridian arcs `N(north,a0..)`/`N(south,a0..)` from the
/// equator to each pole, and the eight quadrant faces
/// `N(north,q0..)`/`N(south,q0..)`.
pub fn sphere_solid<S: Scalar>(
    part: &mut Part<S>,
    name: &str,
    center: Vector3<S>,
    radius: S,
) -> GeopResult<SolidId> {
    let namer = Namer::new("sphere", name)?;
    let n = |args: &[&str]| namer.name(args);
    let w = sqrt2_over_2::<S>();
    let north = center.add(&Vector3::from_array([S::ZERO, S::ZERO, radius]));
    let south = center.add(&Vector3::from_array([
        S::ZERO,
        S::ZERO,
        S::ZERO.sub(radius),
    ]));
    let one = S::ONE;
    let zero = S::ZERO;
    let cos_t = [one, zero, zero.sub(one), zero];
    let sin_t = [zero, one, zero, zero.sub(one)];
    let equator = |k: usize| {
        center.add(&Vector3::from_array([
            radius.mul(cos_t[k]),
            radius.mul(sin_t[k]),
            zero,
        ]))
    };
    // An exact 90-degree arc between 2 points on the sphere, around `center`.
    let arc = |p0: Vector3<S>, p1: Vector3<S>| {
        let mid = p0.add(&p1).sub(&center);
        arc3(p0, mid, p1, w)
    };
    let arc_beam = |k: usize, k1: usize| arc(equator(k), equator(k1));
    let north_quadrant = |k: usize, k1: usize| {
        sphere_quadrant_surface(north, equator(k), equator(k1), center, w, true)
    };
    let south_quadrant = |k: usize, k1: usize| {
        sphere_quadrant_surface(south, equator(k), equator(k1), center, w, false)
    };

    // The placeholder face ends up as the last south quadrant.
    let (v_north, face0, solid_id) =
        part.mvfs(north, n(&["north"]), n(&["south", "q3"]), namer.root())?;

    // The profile: north -> equator(0) -> south, meridian 0's own two
    // halves (each an exact 90-degree arc) — `north_anchor`/`south_anchor`
    // always end at the equator point the *next* beam should grow from.
    // `mirror_north0`/`mirror_south0` (meridian 0's *other* two coedges)
    // stay unused until the very last quadrant, which closes back onto
    // them instead of minting a new meridian 4 (= meridian 0).
    let (_, north_anchor0, mirror_north0, _) = part.mve_from_vertex(
        face0,
        v_north,
        arc(north, equator(0))?,
        meridian_pcurve()?,
        meridian_closing_pcurve()?,
        equator(0),
        n(&["equator", "a0"]),
        n(&["north", "a0"]),
    )?;
    let (_, south_anchor0, mirror_south0, _) = part.mve(
        north_anchor0,
        arc(equator(0), south)?,
        meridian_pcurve()?,
        meridian_closing_pcurve()?,
        south,
        n(&["south"]),
        n(&["south", "a0"]),
    )?;

    let mut north_anchor = north_anchor0;
    let mut south_anchor = south_anchor0;
    for k in 0..3 {
        let k1 = k + 1;
        let (q, a1) = (format!("q{k}"), format!("a{k1}"));
        let (_, beam_fwd, beam_rev, _) = part.mve(
            north_anchor,
            arc_beam(k, k1)?,
            beam_pcurve_north()?,
            beam_pcurve_south()?,
            equator(k1),
            n(&["equator", &a1]),
            n(&["equator", &q]),
        )?;

        let (_, _, _, next_north_anchor) = part.mef(
            beam_fwd,
            north_anchor,
            arc(equator(k1), north)?,
            meridian_closing_pcurve()?,
            meridian_pcurve()?,
            north_quadrant(k, k1)?,
            n(&["north", &a1]),
            n(&["north", &q]),
        )?;
        close_top_pole_gap(part, north_anchor)?;
        let (_, _, _, next_south_anchor) = part.mef(
            south_anchor,
            beam_rev,
            arc(south, equator(k1))?,
            meridian_closing_pcurve()?,
            meridian_pcurve()?,
            south_quadrant(k, k1)?,
            n(&["south", &a1]),
            n(&["south", &q]),
        )?;
        close_bottom_pole_gap(part, south_anchor)?;

        north_anchor = next_north_anchor;
        south_anchor = next_south_anchor;
    }

    // The last quadrant's own closing edge *is* the last beam (equator(3)
    // -> equator(0)) — no separate `mve` needed, since `mirror_north0`
    // (meridian 0's own reversed coedge) already reaches back to
    // `equator(0)` on its own. Only the north quadrant needs this `mef` —
    // the south one (`south_anchor`, `mirror_south0`, and this same beam,
    // reversed) is already exactly what's left on the placeholder face.
    part.mef(
        north_anchor,
        mirror_north0,
        arc_beam(3, 0)?,
        beam_pcurve_north()?,
        beam_pcurve_south()?,
        north_quadrant(3, 0)?,
        n(&["equator", "q3"]),
        n(&["north", "q3"]),
    )?;
    close_top_pole_gap(part, north_anchor)?;
    let _ = mirror_south0;

    // The placeholder's own remaining ring is now exactly the south
    // quadrant's boundary — give it the real surface to match.
    close_bottom_pole_gap(part, south_anchor)?;
    part.replace_face(face0, south_quadrant(3, 0)?)?;

    Ok(solid_id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::for_all_scalars;
    use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};

    fn check_offset_sphere_is_valid<S: Scalar>() {
        let mut part = Part::<S>::new();
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        sphere_solid(&mut part, "t1", center, S::from_f64(2.0)).unwrap();
        let model = part.topology();

        let params = ValidationParameters::default();
        if let Err(e) = validate(&params, model) {
            panic!("{e:?}");
        }
        if let Err(e) = validate_manifold(&params, model) {
            panic!("{e:?}");
        }
    }
    #[test]
    fn offset_sphere_is_valid() {
        for_all_scalars!(check_offset_sphere_is_valid);
    }

    fn check_sphere_solid_normals_point_outward<S: Scalar>() {
        let mut part = Part::<S>::new();
        let center = Vector3::from_array([S::ZERO; 3]);
        sphere_solid(&mut part, "t2", center, S::ONE).unwrap();
        let model = part.topology();

        for face in model.faces.values() {
            let (u0, u1) = face.surface.domain_u();
            let (v0, v1) = face.surface.domain_v();
            let mid_u = u0.add(u1.sub(u0).mul(S::from_f64(0.5)));
            let mid_v = v0.add(v1.sub(v0).mul(S::from_f64(0.5)));
            let p = face.surface.evaluate(mid_u, mid_v).unwrap();
            let n = face.surface.normal(mid_u, mid_v).unwrap();
            let outward = p.sub(&center);
            let dot = n.prod_dot(&outward).to_f64();
            assert!(dot > 0.0, "p={p:?}, n={n:?}, dot={dot}");
        }
    }
    #[test]
    fn sphere_solid_normals_point_outward() {
        for_all_scalars!(check_sphere_solid_normals_point_outward);
    }

    fn check_sphere_solid_is_valid<S: Scalar>() {
        let mut part = Part::<S>::new();
        sphere_solid(&mut part, "t3", Vector3::from_array([S::ZERO; 3]), S::ONE).unwrap();
        let model = part.topology();
        assert_eq!(model.faces.len(), 8);

        let params = ValidationParameters::default();
        if let Err(e) = validate(&params, model) {
            panic!("{e:?}");
        }
        if let Err(e) = validate_manifold(&params, model) {
            panic!("{e:?}");
        }
    }
    #[test]
    fn sphere_solid_is_valid() {
        for_all_scalars!(check_sphere_solid_is_valid);
    }

    /// Every face must have a genuine interior — a point strictly inside its
    /// trim — and that point must lie exactly `radius` from the centre.
    ///
    /// This is the assertion that a zero-area face cannot pass. A degenerate
    /// cap left behind at a pole is structurally valid (its loop closes, its
    /// pcurves are continuous) and every other check accepts it; but it has
    /// no interior, so `face_interior_point` cannot find one, and a boolean
    /// classifying faces by an interior sample would fail on it later and far
    /// from the cause. Sampling the interior rather than the domain corners
    /// is what makes it bite: the corners of a degenerate patch still sit on
    /// the sphere.
    fn check_sphere_solid_face_midpoints_are_on_the_sphere<S: Scalar>() {
        let mut part = Part::<S>::new();
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        let radius = S::from_f64(2.0);
        let solid = sphere_solid(&mut part, "t4", center, radius).unwrap();
        let model = part.topology();

        let faces = model.solid_faces(solid).unwrap();
        assert_eq!(faces.len(), 8, "a sphere is exactly 8 octants");

        let radius_sq = radius.mul(radius);
        for face_id in faces {
            let face = model.get_face(face_id).unwrap();
            assert!(
                !face.surface.is_everything(),
                "face {face_id} still carries the `everything` placeholder surface"
            );

            let (u, v) = geop_core_topology::contains::face::face_interior_point(
                model,
                face_id,
                20000,
                S::from_f64(1e-4),
                0x5EED,
            )
            .unwrap_or_else(|e| panic!("face {face_id} has no interior point: {e}"));

            let p = face.surface.evaluate(u, v).unwrap();
            let d_sq = p.sub(&center).norm_sq();
            assert!(
                d_sq.could_be_equal(radius_sq),
                "face {face_id}'s interior point {p:?} is |p - center|^2={d_sq:?} from the centre, expected {radius_sq:?}"
            );
        }
    }
    #[test]
    fn sphere_solid_face_midpoints_are_on_the_sphere() {
        for_all_scalars!(check_sphere_solid_face_midpoints_are_on_the_sphere);
    }

    fn check_sphere_quadrant_surface_is_exact<S: Scalar>() {
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        let radius = S::from_f64(2.0);
        let north = center.add(&Vector3::from_array([S::ZERO, S::ZERO, radius]));
        let south = center.add(&Vector3::from_array([
            S::ZERO,
            S::ZERO,
            S::ZERO.sub(radius),
        ]));
        let one = S::ONE;
        let zero = S::ZERO;
        let cos_t = [one, zero, zero.sub(one), zero];
        let sin_t = [zero, one, zero, zero.sub(one)];
        let equator = |k: usize| {
            center.add(&Vector3::from_array([
                radius.mul(cos_t[k]),
                radius.mul(sin_t[k]),
                zero,
            ]))
        };
        let w = crate::common::sqrt2_over_2::<S>();
        let radius_sq = radius.mul(radius);

        for k in 0..4 {
            let k1 = (k + 1) % 4;
            for (pole, pole_at_v0) in [(north, true), (south, false)] {
                let surface =
                    sphere_quadrant_surface(pole, equator(k), equator(k1), center, w, pole_at_v0)
                        .unwrap();
                for i in 0..=4 {
                    for j in 0..=4 {
                        let u = S::from_ratio(i as i64, 4).unwrap();
                        let v = S::from_ratio(j as i64, 4).unwrap();
                        let p = surface.evaluate(u, v).unwrap();
                        let d_sq = p.sub(&center).norm_sq();
                        let err = d_sq.sub(radius_sq).abs();
                        assert!(
                            !err.definitely_greater(S::from_f64(1e-9)),
                            "k={k}, pole_at_v0={pole_at_v0}, u={u}, v={v}, p={p:?}, |p-center|^2={d_sq}, expected={radius_sq}"
                        );
                    }
                }
            }
        }
    }
    #[test]
    fn sphere_quadrant_surface_is_exact() {
        for_all_scalars!(check_sphere_quadrant_surface_is_exact);
    }

    /// Every edge is an exact meridian (straight line through `center`'s
    /// axis) or an exact 90-degree equator arc, and every quadrant face is
    /// an exact degree-(2, 1) patch built from the same two kinds of curve
    /// — so, sampled anywhere (not just at control points), both must land
    /// exactly `radius` away from `center`.
    fn check_sphere_solid_points_on_sphere<S: Scalar>() {
        let mut part = Part::<S>::new();
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        let radius = S::from_f64(2.0);
        sphere_solid(&mut part, "t5", center, radius).unwrap();
        let model = part.topology();

        let radius_sq = radius.mul(radius);
        let assert_on_sphere = |p: Vector3<S>, ctx: &str| {
            let d_sq = p.sub(&center).norm_sq();
            let err = d_sq.sub(radius_sq).abs();
            assert!(
                !err.definitely_greater(S::from_f64(1e-9)),
                "{ctx}: p={p:?}, |p - center|^2={d_sq}, expected={radius_sq}"
            );
        };

        for edge in model.edges.values() {
            let (t0, t1) = edge.curve.domain();
            for i in 0..=8 {
                let t = t0.add(t1.sub(t0).mul(S::from_ratio(i as i64, 8).unwrap()));
                let p = edge.curve.evaluate(t).unwrap();
                assert_on_sphere(p, "edge sample");
            }
        }

        for face in model.faces.values() {
            let (u0, u1) = face.surface.domain_u();
            let (v0, v1) = face.surface.domain_v();
            for i in 0..=8 {
                for j in 0..=8 {
                    let u = u0.add(u1.sub(u0).mul(S::from_ratio(i as i64, 8).unwrap()));
                    let v = v0.add(v1.sub(v0).mul(S::from_ratio(j as i64, 8).unwrap()));
                    let p = face.surface.evaluate(u, v).unwrap();
                    assert_on_sphere(p, "surface sample");
                }
            }
        }
    }
    #[test]
    fn sphere_solid_points_on_sphere() {
        for_all_scalars!(check_sphere_solid_points_on_sphere);
    }

    fn check_rasterize_topology_sphere<S: Scalar>() {
        let mut part = Part::<S>::new();
        sphere_solid(&mut part, "t6", Vector3::from_array([S::ZERO; 3]), S::ONE).unwrap();
        let model = part.topology();

        let scene = geop_ops_rasterize::debug::rasterize_topology(model, 32).unwrap();
        assert!(!scene.points.is_empty());
        assert!(!scene.lines.is_empty());
        assert!(!scene.triangles_transparent.is_empty());
        assert!(!scene.labels.is_empty());

        std::fs::create_dir_all("outputs").unwrap();
        scene.save_to_file("outputs/sphere_topology.html").unwrap();
    }
    #[test]
    fn rasterize_topology_sphere() {
        for_all_scalars!(check_rasterize_topology_sphere);
    }

    fn check_rasterize_topology_offset_sphere<S: Scalar>() {
        let mut part = Part::<S>::new();
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        sphere_solid(&mut part, "t7", center, S::from_f64(2.0)).unwrap();
        let model = part.topology();

        let scene = geop_ops_rasterize::debug::rasterize_topology(model, 32).unwrap();
        assert!(!scene.points.is_empty());
        assert!(!scene.lines.is_empty());
        assert!(!scene.triangles_transparent.is_empty());
        assert!(!scene.labels.is_empty());

        std::fs::create_dir_all("outputs").unwrap();
        scene
            .save_to_file("outputs/offset_sphere_topology.html")
            .unwrap();
    }
    #[test]
    fn rasterize_topology_offset_sphere() {
        for_all_scalars!(check_rasterize_topology_offset_sphere);
    }
}
