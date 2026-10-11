//! Extruding: sweeping a planar profile along a straight [`Path`] (see
//! [`crate::sweep`]) — a solid between two flat caps, or a sheet of walls.

use crate::sweep::{Frame, Path, Span, SweepLoop, sweep_drawn};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
};
use geop_core_topology::build::BuiltBody;
use geop_ops::{NamedModel, Namer};

/// The straight path along `plane.w()` from `from` to `to` (in multiples of
/// `w`): the profile, drawn in the plane's `(u, v)`, at stations `start`
/// and `end`, and one span between them.
pub fn extrusion<S: Scalar>(plane: &CoordinateSystem<S>, from: S, to: S) -> GeopResult<Path<S>> {
    let station = |h: S| Frame {
        origin: plane.origin().add(&plane.w().prod_scalar(h)),
        e1: *plane.u(),
        e2: *plane.v(),
    };
    let along = plane
        .u()
        .prod_cross(plane.v())
        .prod_dot(plane.w())
        .mul(to.sub(from));
    let along_normal = if along.definitely_greater(S::ZERO) {
        true
    } else if along.definitely_less(S::ZERO) {
        false
    } else {
        return Err(GeopError::new(format!(
            "extrusion: the distance from {from:?} to {to:?} could be zero"
        )));
    };
    Ok(Path {
        stations: vec![station(from), station(to)],
        spans: vec![Span::Line],
        closed: false,
        along_normal,
        station_names: vec!["start".into(), "end".into()],
        span_names: vec![None],
    })
}

/// Extrudes `loops` — the first the outer loop, counter-clockwise in
/// `plane`'s `(u, v)`, the rest holes in it, clockwise — from `from` to
/// `to` along `plane.w()` (see [`extrusion`]): into a solid named `solid`,
/// its start cap at `from`, or, without one, into sheets. Either way round
/// the solid comes out with its faces pointing outwards, whichever way the
/// plane is handed.
///
/// Named after the profiles' curves `X` and joints `P` (see
/// [`crate::common::Profile`] and [`sweep`]): the walls `N(X)`, their edges
/// on the caps `N(X,start)` / `N(X,end)`, the edges between them `N(P)`,
/// their vertices `N(P,start)` / `N(P,end)`, the caps `N(start)` /
/// `N(end)`.
pub fn extrude<S: Scalar>(
    model: &mut NamedModel<S>,
    namer: &Namer,
    solid: Option<&str>,
    plane: &CoordinateSystem<S>,
    from: S,
    to: S,
    loops: &[SweepLoop<S>],
) -> GeopResult<BuiltBody> {
    let path = extrusion(plane, from, to)?;
    sweep_drawn(model, namer, &path, loops, solid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::testing::{self, assert_valid_sheets, circle, circle_at, v2};
    use crate::common::{Profile, arc2, polygon, sqrt2_over_2};
    use geop_core_geometry::nurb_curve::NurbCurve2D;
    use geop_core_math::for_all_scalars;
    use geop_core_math::vector::Vector3;
    use geop_core_topology::Model;

    fn unit_square<S: Scalar>() -> Vec<NurbCurve2D<S>> {
        polygon(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 1.0), v2(0.0, 1.0)]).unwrap()
    }

    /// A square hole (traced the *opposite* winding from `unit_square`, so
    /// the material — the region between the outer boundary and this hole
    /// — genuinely has the hole cut out of it), spanning `[x0, x0 + size] x
    /// [y0, y0 + size]`.
    fn square_hole<S: Scalar>(x0: f64, y0: f64, size: f64) -> Vec<NurbCurve2D<S>> {
        polygon(&[
            v2(x0, y0),
            v2(x0, y0 + size),
            v2(x0 + size, y0 + size),
            v2(x0 + size, y0),
        ])
        .unwrap()
    }

    /// Left-handed (`u x v = -w`), which is what `extrude` requires of a CCW
    /// outer polygon for the resulting faces to point outward — see the
    /// module doc. `figure8_profile` builds its own the same way. A
    /// right-handed basis here produces a solid that is entirely inside-out:
    /// structurally perfect, and rejected by
    /// `validation::face_orientation::check_normals_point_outward`.
    fn axis_aligned_coordinate_system<S: Scalar>(origin: Vector3<S>) -> CoordinateSystem<S> {
        let u = Vector3::from_array([S::ONE, S::ZERO, S::ZERO]);
        let v = Vector3::from_array([S::ZERO, S::ONE, S::ZERO]);
        let w = Vector3::from_array([S::ZERO, S::ZERO, S::from_f64(-1.0)]);
        CoordinateSystem::try_new(origin, u, v, w).unwrap()
    }

    /// Extrude into a fresh part, as the single-region operation `e`.
    fn extruded<S: Scalar>(
        cs: &CoordinateSystem<S>,
        outer: Vec<NurbCurve2D<S>>,
        holes: Vec<Vec<NurbCurve2D<S>>>,
    ) -> NamedModel<S> {
        let mut model = NamedModel::<S>::new();
        let namer = Namer::new("extrude", "e").unwrap();
        let loops: Vec<_> = std::iter::once(Profile::closed(outer))
            .chain(
                holes
                    .into_iter()
                    .enumerate()
                    .map(|(k, h)| Profile::closed(h).with_prefix(&format!("h{k}"))),
            )
            .map(SweepLoop::plain)
            .collect();
        extrude(
            &mut model,
            &namer,
            Some(&namer.root()),
            cs,
            S::ZERO,
            S::ONE,
            &loops,
        )
        .unwrap();
        model.check_names().unwrap();
        model
    }

    fn assert_valid<S: Scalar>(model: &Model<S>) {
        testing::assert_valid(model);
        for edge_id in model.edges.keys() {
            assert_eq!(model.coedges_of_edge(*edge_id).len(), 2);
        }
    }

    fn check_extruded_square_is_valid<S: Scalar>() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let model = extruded(&cs, unit_square::<S>(), vec![]);
        let topology = model.topology();
        assert_valid(topology);
        assert_eq!(topology.faces.len(), 6);
        assert_eq!(topology.vertices.len(), 8);
        assert_eq!(topology.edges.len(), 12);
    }
    #[test]
    fn extruded_square_is_valid() {
        for_all_scalars!(check_extruded_square_is_valid);
    }

    fn check_extruded_square_with_hole_is_valid<S: Scalar>() {
        let hole = square_hole::<S>(0.25, 0.25, 0.5);
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let model = extruded(&cs, unit_square::<S>(), vec![hole]);
        let topology = model.topology();
        assert_valid(topology);
        // 6 outer faces (4 sides + top + bottom) + 4 hole side walls; 8
        // outer vertices + 8 hole vertices (4 on the bottom cap, 4 on top).
        assert_eq!(topology.faces.len(), 6 + 4);
        assert_eq!(topology.vertices.len(), 8 + 8);
    }
    #[test]
    fn extruded_square_with_hole_is_valid() {
        for_all_scalars!(check_extruded_square_with_hole_is_valid);
    }

    fn check_extrude_offsets_footprint<S: Scalar>() {
        let origin = Vector3::from_array([S::from_f64(2.0), S::from_f64(-1.0), S::from_f64(0.5)]);
        let cs = axis_aligned_coordinate_system(origin);
        let model = extruded(&cs, unit_square::<S>(), vec![]);
        let topology = model.topology();
        assert_valid(topology);
        // `w` points along `-z` (see `axis_aligned_coordinate_system`), so
        // the profile sits at the origin's own `z` and the far face one unit
        // *below* it.
        for vertex in topology.vertices.values() {
            assert!(
                vertex.point[2].could_be_equal(S::from_f64(0.5))
                    || vertex.point[2].could_be_equal(S::from_f64(-0.5)),
                "vertex at unexpected height: {:?}",
                vertex.point
            );
        }
    }
    #[test]
    fn extrude_offsets_footprint() {
        for_all_scalars!(check_extrude_offsets_footprint);
    }

    /// A profile anywhere in the plane, not just the unit square: the caps
    /// are sized to it.
    fn check_extrude_profile_away_from_origin<S: Scalar>() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let outer = polygon(&[v2(-3.0, 2.0), v2(-1.0, 2.0), v2(-2.0, 5.0)]).unwrap();
        let model = extruded(&cs, outer, vec![]);
        let topology = model.topology();
        assert_valid(topology);
        assert_eq!(topology.faces.len(), 5);
    }
    #[test]
    fn extrude_profile_away_from_origin() {
        for_all_scalars!(check_extrude_profile_away_from_origin);
    }

    /// A disc with a round hole: every wall is a quarter of a cylinder.
    fn check_extruded_ring_is_valid<S: Scalar>() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([S::ZERO; 3]));
        let outer = circle::<S>(2.0);
        let hole = circle_at::<S>(0.3, 0.0, 1.0, true);
        let model = extruded(&cs, outer, vec![hole]);
        let topology = model.topology();
        assert_valid(topology);
        assert_eq!(topology.faces.len(), 2 + 4 + 4);
    }
    #[test]
    fn extruded_ring_is_valid() {
        for_all_scalars!(check_extruded_ring_is_valid);
    }

    /// Both directions off a right-handed plane give valid solids on the
    /// expected side.
    fn check_extrude_both_directions<S: Scalar>() {
        let plane = CoordinateSystem::try_new(
            Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
            Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
        )
        .unwrap();
        let mut outer = polygon(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 1.0)]).unwrap();
        outer.push(arc2(v2(1.0, 1.0), v2(0.0, 1.0), v2(0.0, 0.0), sqrt2_over_2()).unwrap());
        outer.remove(2);
        let namer = Namer::new("extrude", "e").unwrap();
        let outer = SweepLoop::plain(Profile::closed(outer));
        for (distance, far) in [(0.5, 1.5), (-0.5, 0.5)] {
            let mut model = NamedModel::<S>::new();
            extrude(
                &mut model,
                &namer,
                Some(&namer.root()),
                &plane,
                S::ZERO,
                S::from_f64(distance),
                std::slice::from_ref(&outer),
            )
            .unwrap();
            model.check_names().unwrap();
            let topology = model.topology();
            assert_valid(topology);
            assert!(
                topology
                    .vertices
                    .values()
                    .any(|v| v.point[2].could_be_equal(S::from_f64(far)))
            );
            // Whichever way the profile had to be mirrored to extrude it,
            // every name still sits where it says: `p0` at the profile's
            // first point, the end cap `distance` away from the plane.
            let p0_start = model.vertex_id("extrude(e,p0,start)").unwrap();
            let p0_end = model.vertex_id("extrude(e,p0,end)").unwrap();
            let point = |v| topology.get_vertex(v).unwrap().point;
            assert!(point(p0_start).could_be_equal(&Vector3::from_array([
                S::ZERO,
                S::ZERO,
                S::ONE
            ])));
            assert!(point(p0_end)[2].could_be_equal(S::from_f64(far)));
            let end_cap = model.face_id("extrude(e,end)").unwrap();
            assert!(topology.iterate_face_coedges(end_cap).all(|c| {
                topology.coedge_start_vertex(c).unwrap().point[2].could_be_equal(S::from_f64(far))
            }));
        }
    }
    #[test]
    fn extrude_both_directions() {
        for_all_scalars!(check_extrude_both_directions);
    }

    fn xy_plane<S: Scalar>() -> CoordinateSystem<S> {
        CoordinateSystem::world_at(Vector3::zero())
    }

    /// An open chain extrudes into a sheet: a wall per curve, the edges
    /// along its rims and its two ends each used by that one wall alone.
    fn check_open_chain_extrudes_into_a_sheet<S: Scalar>() {
        let mut model = NamedModel::<S>::new();
        let namer = Namer::new("extrude", "e").unwrap();
        let chain = crate::common::polyline(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 2.0)]).unwrap();
        let built = extrude(
            &mut model,
            &namer,
            None,
            &xy_plane(),
            S::ZERO,
            S::ONE,
            &[SweepLoop::plain(Profile::open(chain))],
        )
        .unwrap();
        model.check_names().unwrap();
        assert!(built.solid.is_none());
        let topology = model.topology();
        assert_valid_sheets(topology);
        assert_eq!(topology.faces.len(), 2);
        let free = |e: &geop_core_topology::EdgeId| topology.coedges_of_edge(*e).len() == 1;
        assert_eq!(topology.edges.keys().filter(|e| free(e)).count(), 6);
        assert!(model.edge_id("extrude(e,p1)").is_ok_and(|e| !free(&e)));
    }
    #[test]
    fn open_chain_extrudes_into_a_sheet() {
        for_all_scalars!(check_open_chain_extrudes_into_a_sheet);
    }

    /// Between any two ends, either way round: the start cap at the first.
    fn check_extrude_between_two_ends<S: Scalar>() {
        for (from, to) in [(-0.5, 1.0), (1.0, -0.5), (0.25, 2.0)] {
            let mut model = NamedModel::<S>::new();
            let namer = Namer::new("extrude", "e").unwrap();
            extrude(
                &mut model,
                &namer,
                Some("extrude(e)"),
                &xy_plane(),
                S::from_f64(from),
                S::from_f64(to),
                &[SweepLoop::plain(Profile::closed(unit_square()))],
            )
            .unwrap();
            let topology = model.topology();
            assert_valid(topology);
            let height = |name: &str| {
                topology
                    .get_vertex(model.vertex_id(name).unwrap())
                    .unwrap()
                    .point[2]
            };
            assert!(height("extrude(e,p0,start)").could_be_equal(S::from_f64(from)));
            assert!(height("extrude(e,p2,end)").could_be_equal(S::from_f64(to)));
        }
    }
    #[test]
    fn extrude_between_two_ends() {
        for_all_scalars!(check_extrude_between_two_ends);
    }
}

#[cfg(test)]
mod naming_tests {
    use super::*;
    use crate::common::{Profile, polygon};
    use geop_core_math::scalars::ScalInF64 as S;
    use geop_core_math::vector::{Vector2, Vector3};

    /// Every entity of an extruded square is named after the profile element
    /// it was swept from, and the names hang together topologically: the
    /// side face of `c0` is bounded by `c0`'s two cap edges and the lateral
    /// edges of its two joints.
    #[test]
    fn extruded_square_names_follow_the_profile() {
        let square = polygon(&[
            Vector2::from_array([S::ZERO, S::ZERO]),
            Vector2::from_array([S::ONE, S::ZERO]),
            Vector2::from_array([S::ONE, S::ONE]),
            Vector2::from_array([S::ZERO, S::ONE]),
        ])
        .unwrap();
        let plane = CoordinateSystem::try_new(
            Vector3::zero(),
            Vector3::from_array([S::ONE, S::ZERO, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ONE, S::ZERO]),
            Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
        )
        .unwrap();
        let mut model = NamedModel::<S>::new();
        let namer = Namer::new("extrude", "box").unwrap();
        extrude(
            &mut model,
            &namer,
            Some(&namer.root()),
            &plane,
            S::ZERO,
            S::ONE,
            &[SweepLoop::plain(Profile::closed(square))],
        )
        .unwrap();
        model.check_names().unwrap();

        let description = geop_ops::PartDescription::of(&model).unwrap();
        assert_eq!(description.solids.len(), 1);
        assert_eq!(description.solids["extrude(box)"][0].len(), 6);
        let mut side = description.faces["extrude(box,c0)"].outer.clone();
        side.sort();
        let mut expected: Vec<String> = [
            "extrude(box,c0,end)",
            "extrude(box,c0,start)",
            "extrude(box,p0)",
            "extrude(box,p1)",
        ]
        .iter()
        .map(|e| e.to_string())
        .collect();
        expected.sort();
        let unsigned: Vec<String> = side.iter().map(|e| e[1..].to_string()).collect();
        let mut unsigned = unsigned;
        unsigned.sort();
        assert_eq!(unsigned, expected);
        assert_eq!(
            description.edges["extrude(box,p1)"].start,
            "extrude(box,p1,start)"
        );
        assert_eq!(
            description.edges["extrude(box,p1)"].end,
            "extrude(box,p1,end)"
        );
        let corner = model.vertex_id("extrude(box,p2,end)").unwrap();
        assert!(
            model
                .topology()
                .get_vertex(corner)
                .unwrap()
                .point
                .could_be_equal(&Vector3::from_array([S::ONE; 3]))
        );
    }
}
