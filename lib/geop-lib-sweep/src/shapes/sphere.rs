//! A sphere as exactly 8 octant faces: 2 rows (north pole to equator, and
//! equator to south pole) x 4 quadrants, each an exact rational quarter of a
//! hemisphere. No approximation — the surfaces *are* spheres, so a point
//! sampled anywhere on one is exactly `radius` from the centre.
//!
//! Each quadrant is a face over the unit square of its parameters, one side
//! of which is collapsed to its pole. A degenerate coedge at the pole vertex
//! runs along that side, so that the loop closes in `(u, v)` too:
//! `contains::face::face_contains` relies on the pcurves alone tracing a
//! closed polygon.

use geop_core_geometry::{
    nurb_curve::{NurbCurve3D, sqrt2_over_2},
    nurb_surface::NurbSurface,
};
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};
use geop_core_topology::{
    Sense, SolidId,
    build::{BodySpec, CoedgeOn, EdgeSpec},
};
use geop_ops_model::{BodyNames, NamedModel, Namer};

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

    // Each triple is one `u`-slice, `[v = 0, v = 0.5, v = 1]`, assuming the
    // pole sits at `v = 0` — reversed below if it's actually at `v = 1`.
    // `u0`/`u2` (the true `u = 0`/`u = 1` boundary slices) hit the pole as
    // an genuine unweighted corner; `u1` (the interior slice) instead hits
    // it as the *middle* control point of its own (degenerate) `v`-arc,
    // which — like every other arc's own middle control point here — needs
    // weight `w`, not `1`, even though its position also happens to be the
    // pole.
    let mut u0 = [
        pole.homogeneous(S::ONE),
        mer0_mid.homogeneous(w),
        eq0.homogeneous(S::ONE),
    ];
    let mut u1 = [
        pole.homogeneous(w),
        interior.homogeneous(m),
        eq_mid.homogeneous(w),
    ];
    let mut u2 = [
        pole.homogeneous(S::ONE),
        mer1_mid.homogeneous(w),
        eq1.homogeneous(S::ONE),
    ];
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
    // (the already-verified pole placement above). `sphere_solid` bounds
    // each face in this same convention: `u = 0` at `eq1`, `u = 1` at
    // `eq0`.
    let knots = vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE];
    let cps = vec![
        u2[0], u2[1], u2[2], u1[0], u1[1], u1[2], u0[0], u0[1], u0[2],
    ];
    NurbSurface::try_new(2, 2, cps, knots.clone(), knots)
}

/// A sphere of `radius` centered at `center`, as exactly 8 degree-(2, 2)
/// quadrant faces between its poles and 4 points of its equator.
///
/// Named as the operation `sphere(name)`: the poles `N(north)`/`N(south)`,
/// the equator's vertices `N(equator,a0..a3)` at angles `a0` (`+x`), `a1`
/// (`+y`), ..., its quarter arcs `N(equator,q0..q3)` (`q0` from `a0` to
/// `a1`, ...), the meridian arcs `N(north,a0..)`/`N(south,a0..)` between
/// the equator and each pole, and the eight quadrant faces
/// `N(north,q0..)`/`N(south,q0..)`.
pub fn sphere_solid<S: Scalar>(
    model: &mut NamedModel<S>,
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
            radius.mul(cos_t[k % 4]),
            radius.mul(sin_t[k % 4]),
            zero,
        ]))
    };
    // An exact 90-degree arc between 2 points on the sphere, around `center`.
    let arc = |p0: Vector3<S>, p1: Vector3<S>| {
        let mid = p0.add(&p1).sub(&center);
        NurbCurve3D::arc(p0, mid, p1, w)
    };

    // The vertices: the poles, then the equator's points. The edges, three
    // per quarter `k`: the equator's arc `eq(k) -> eq(k + 1)`, and the
    // meridians `north -> eq(k)` and `eq(k) -> south`.
    const NORTH: usize = 0;
    const SOUTH: usize = 1;
    let eq = |k: usize| 2 + k % 4;
    let equator_arc = |k: usize| 3 * (k % 4);
    let north_meridian = |k: usize| 3 * (k % 4) + 1;
    let south_meridian = |k: usize| 3 * (k % 4) + 2;

    let mut spec = BodySpec::new(true);
    let mut names = BodyNames {
        vertices: vec![n(&["north"]), n(&["south"])],
        edges: Vec::new(),
        faces: Vec::new(),
        solid: Some(namer.root()),
    };
    spec.vertices = vec![north, south];
    for k in 0..4 {
        let (a, q) = (format!("a{k}"), format!("q{k}"));
        spec.vertices.push(equator(k));
        names.vertices.push(n(&["equator", &a]));
        let edges = [
            (
                arc(equator(k), equator(k + 1))?,
                eq(k),
                eq(k + 1),
                ["equator", &q],
            ),
            (arc(north, equator(k))?, NORTH, eq(k), ["north", &a]),
            (arc(equator(k), south)?, eq(k), SOUTH, ["south", &a]),
        ];
        for (curve, start, end, edge_name) in edges {
            spec.edges.push(EdgeSpec { curve, start, end });
            names.edges.push(n(&edge_name));
        }
    }
    // A quadrant's `u` runs from `eq(k + 1)` to `eq(k)` (see
    // `sphere_quadrant_surface`), its `v` from the pole to the equator in
    // the north, from the equator to the pole in the south.
    for k in 0..4 {
        let q = format!("q{k}");
        let quadrant = |pole, pole_at_v0| {
            sphere_quadrant_surface(pole, equator(k), equator(k + 1), center, w, pole_at_v0)
        };
        // `square_face` gives each edge the sense it runs along its side in.
        let side = |e| CoedgeOn::Edge(e, Sense::Forward);
        let north_face = spec.square_face(
            quadrant(north, true)?,
            [NORTH, NORTH, eq(k), eq(k + 1)],
            [
                CoedgeOn::Vertex(NORTH),
                side(north_meridian(k)),
                side(equator_arc(k)),
                side(north_meridian(k + 1)),
            ],
        )?;
        let south_face = spec.square_face(
            quadrant(south, false)?,
            [eq(k + 1), eq(k), SOUTH, SOUTH],
            [
                side(equator_arc(k)),
                side(south_meridian(k)),
                CoedgeOn::Vertex(SOUTH),
                side(south_meridian(k + 1)),
            ],
        )?;
        spec.faces.extend([north_face, south_face]);
        names.faces.extend([n(&["north", &q]), n(&["south", &q])]);
    }
    spec.shells = vec![(0..spec.faces.len()).collect()];
    let built = model.build_body(spec, names)?;
    Ok(built.solid.expect("built as a solid"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::testing::render;
    use geop_core_math::for_all_scalars;
    use geop_ops_model::testing::assert_model_valid;

    fn check_offset_sphere_is_valid<S: Scalar>() {
        let mut model = NamedModel::<S>::new();
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        sphere_solid(&mut model, "t1", center, S::from_f64(2.0)).unwrap();
        let topology = model.topology();

        assert_model_valid(topology);
        render(topology, 32, "offset_sphere_topology.html");
    }
    #[test]
    fn offset_sphere_is_valid() {
        for_all_scalars!(check_offset_sphere_is_valid);
    }

    fn check_sphere_solid_normals_point_outward<S: Scalar>() {
        let mut model = NamedModel::<S>::new();
        let center = Vector3::from_array([S::ZERO; 3]);
        sphere_solid(&mut model, "t2", center, S::ONE).unwrap();
        let topology = model.topology();

        for face in topology.faces.values() {
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
        let mut model = NamedModel::<S>::new();
        sphere_solid(&mut model, "t3", Vector3::from_array([S::ZERO; 3]), S::ONE).unwrap();
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 8);

        assert_model_valid(topology);
        render(topology, 32, "sphere_topology.html");
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
        let mut model = NamedModel::<S>::new();
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        let radius = S::from_f64(2.0);
        let solid = sphere_solid(&mut model, "t4", center, radius).unwrap();
        let topology = model.topology();

        let faces = topology.solid_faces(solid).unwrap();
        assert_eq!(faces.len(), 8, "a sphere is exactly 8 octants");

        let radius_sq = radius.mul(radius);
        for face_id in faces {
            let face = topology.get_face(face_id).unwrap();
            assert!(
                !face.surface.is_everything(),
                "face {face_id} still carries the `everything` placeholder surface"
            );

            let (u, v) = geop_core_topology::contains::face::face_interior_point(
                topology,
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
        let w = sqrt2_over_2::<S>();
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
        let mut model = NamedModel::<S>::new();
        let center = Vector3::from_array([S::from_f64(1.0), S::from_f64(-0.5), S::from_f64(0.5)]);
        let radius = S::from_f64(2.0);
        sphere_solid(&mut model, "t5", center, radius).unwrap();
        let topology = model.topology();

        let radius_sq = radius.mul(radius);
        let assert_on_sphere = |p: Vector3<S>, ctx: &str| {
            let d_sq = p.sub(&center).norm_sq();
            let err = d_sq.sub(radius_sq).abs();
            assert!(
                !err.definitely_greater(S::from_f64(1e-9)),
                "{ctx}: p={p:?}, |p - center|^2={d_sq}, expected={radius_sq}"
            );
        };

        for edge in topology.edges.values() {
            let (t0, t1) = edge.curve.domain();
            for i in 0..=8 {
                let t = t0.add(t1.sub(t0).mul(S::from_ratio(i as i64, 8).unwrap()));
                let p = edge.curve.evaluate(t).unwrap();
                assert_on_sphere(p, "edge sample");
            }
        }

        for face in topology.faces.values() {
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
}
