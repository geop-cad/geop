//! Extruding: sweeping a planar profile along a straight [`Path`] (see
//! [`crate::sweep`]) — a solid between two flat caps, or a sheet of walls.

use crate::sweep::{Frame, Path, Span, SweepLoop, sweep_drawn};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
};
use geop_core_topology::build::BuiltBody;
use geop_ops_model::{NamedModel, Namer};

/// The straight path along `plane.w()` through `heights` (in multiples of
/// `w`, two or more, each further the same way than the one before): the
/// profile, drawn in the plane's `(u, v)`, at stations `start`, `l1`,
/// `l2`, ..., `end`, and a span between each two, named `q0`, `q1`, ...
/// where there are several.
pub fn extrusion(plane: &CoordinateSystem, heights: &[Scalar]) -> GeopResult<Path> {
    let n = heights.len();
    let ways: Vec<Option<bool>> = heights
        .windows(2)
        .map(|h| h[1].sub(h[0]).definite_sign())
        .collect();
    if n < 2 || ways.iter().any(|way| way.is_none() || *way != ways[0]) {
        return Err(GeopError::new(format!(
            "extrusion: heights {heights:?} that do not each go further one way"
        )));
    }
    let station = |h: Scalar| Frame {
        origin: plane.origin().add(&plane.w().prod_scalar(h)),
        e1: *plane.u(),
        e2: *plane.v(),
    };
    let along = plane
        .u()
        .prod_cross(plane.v())
        .prod_dot(plane.w())
        .mul(heights[n - 1].sub(heights[0]));
    let along_normal = along.definite_sign().ok_or_else(|| {
        GeopError::new(format!(
            "extrusion: which way {heights:?} go along the plane's normal could be either"
        ))
    })?;
    Ok(Path {
        stations: heights.iter().map(|&h| station(h)).collect(),
        spans: vec![Span::Line; n - 1],
        closed: false,
        along_normal,
        station_names: (0..n)
            .map(|s| match s {
                0 => "start".to_string(),
                s if s + 1 == n => "end".to_string(),
                s => format!("l{s}"),
            })
            .collect(),
        span_names: if n == 2 {
            vec![None]
        } else {
            (0..n - 1).map(|j| Some(format!("q{j}"))).collect()
        },
    })
}

/// Extrudes `loops` — the first the outer loop, counter-clockwise in
/// `plane`'s `(u, v)`, the rest holes in it, clockwise — along
/// `plane.w()` through `heights` (see [`extrusion`]): into a solid named
/// `solid`, its start cap at the first height, or, without one, into
/// sheets. Either way round the solid comes out with its faces pointing
/// outwards, whichever way the plane is handed.
///
/// Named after the profiles' curves `X` and joints `P` (see
/// [`Profile`](geop_ops_model::Profile) and [`sweep`]): the walls `N(X)`,
/// their edges on the caps `N(X,start)` / `N(X,end)`, the edges between
/// them `N(P)`, their vertices `N(P,start)` / `N(P,end)`, the caps
/// `N(start)` / `N(end)`. Through more than two heights each span's walls
/// and edges are `N(X,q0)`, `N(P,q0)`, ..., and the edges and vertices at
/// the heights between `N(X,l1)`, `N(P,l1)`, ....
pub fn extrude(
    model: &mut NamedModel,
    namer: &Namer,
    solid: Option<&str>,
    plane: &CoordinateSystem,
    heights: &[Scalar],
    loops: &[SweepLoop],
) -> GeopResult<BuiltBody> {
    let path = extrusion(plane, heights)?;
    sweep_drawn(model, namer, &path, loops, solid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::polygon;
    use crate::shapes::{circle, circle_at, v2};
    use geop_core_geometry::nurb_curve::{NurbCurve2D, sqrt2_over_2};
    use geop_core_math::vector::Vector3;
    use geop_ops_model::Profile;
    use geop_ops_model::testing::{assert_model_valid, assert_model_valid_sheets};

    fn unit_square() -> Vec<NurbCurve2D> {
        polygon(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 1.0), v2(0.0, 1.0)]).unwrap()
    }

    /// A square hole (traced the *opposite* winding from `unit_square`, so
    /// the material — the region between the outer boundary and this hole
    /// — genuinely has the hole cut out of it), spanning `[x0, x0 + size] x
    /// [y0, y0 + size]`.
    fn square_hole(x0: f64, y0: f64, size: f64) -> Vec<NurbCurve2D> {
        polygon(&[
            v2(x0, y0),
            v2(x0, y0 + size),
            v2(x0 + size, y0 + size),
            v2(x0 + size, y0),
        ])
        .unwrap()
    }

    /// Left-handed (`u x v = -w`), as `figure8_profile` builds its own:
    /// `extrude` points the faces outwards either way round, which
    /// `validation::face_orientation::check_normals_point_outward` checks.
    fn axis_aligned_coordinate_system(origin: Vector3) -> CoordinateSystem {
        let u = Vector3::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO]);
        let v = Vector3::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO]);
        let w = Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::from_f64(-1.0)]);
        CoordinateSystem::try_new(origin, u, v, w).unwrap()
    }

    /// Extrude into a fresh part, as the single-region operation `e`.
    fn extruded(
        cs: &CoordinateSystem,
        outer: Vec<NurbCurve2D>,
        holes: Vec<Vec<NurbCurve2D>>,
    ) -> NamedModel {
        let mut model = NamedModel::new();
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
            &[Scalar::ZERO, Scalar::ONE],
            &loops,
        )
        .unwrap();
        model.check_names().unwrap();
        model
    }

    #[test]
    fn extruded_square_is_valid() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([Scalar::ZERO; 3]));
        let model = extruded(&cs, unit_square(), vec![]);
        let topology = model.topology();
        assert_model_valid(topology);
        assert_eq!(topology.faces.len(), 6);
        assert_eq!(topology.vertices.len(), 8);
        assert_eq!(topology.edges.len(), 12);
    }

    /// A curve kinked at a knot inside it is cut there: its wall is two
    /// faces, `c0` and `c0#k1`, an edge where the kink is. A curve only
    /// C1 in its parametrization at a knot, a circle's quarters as one
    /// rational curve, turning on smoothly, is not cut.
    #[test]
    fn a_kinked_curve_is_swept_into_a_face_per_leg() {
        let mut model = NamedModel::new();
        let prism = crate::shapes::creased_prism(&mut model, "p");
        assert_model_valid(model.topology());
        let names: Vec<String> = (model.topology().solid_faces(prism).unwrap().iter())
            .map(|&f| model.name(f).unwrap().to_string())
            .collect();
        assert_eq!(names.len(), 3 + 2, "{names:?}");
        assert!(model.face_id("prism(p,c0)").is_ok() && model.face_id("prism(p,c0#k1)").is_ok());

        let whole = NurbCurve2D::join(&crate::shapes::circle(1.0)).unwrap();
        let (t0, t1) = whole.domain();
        let (a, b) = whole.split(t0.add(t1).div(Scalar::TWO).unwrap()).unwrap();
        let cut = SweepLoop::plain(Profile::closed(vec![a, b]))
            .at_corners()
            .unwrap();
        assert_eq!(cut.profile.curves.len(), 2);
    }

    #[test]
    fn extruded_square_with_hole_is_valid() {
        let hole = square_hole(0.25, 0.25, 0.5);
        let cs = axis_aligned_coordinate_system(Vector3::from_array([Scalar::ZERO; 3]));
        let model = extruded(&cs, unit_square(), vec![hole]);
        let topology = model.topology();
        assert_model_valid(topology);
        // 6 outer faces (4 sides + top + bottom) + 4 hole side walls; 8
        // outer vertices + 8 hole vertices (4 on the bottom cap, 4 on top).
        assert_eq!(topology.faces.len(), 6 + 4);
        assert_eq!(topology.vertices.len(), 8 + 8);
    }

    #[test]
    fn extrude_offsets_footprint() {
        let origin = Vector3::from_array([
            Scalar::from_f64(2.0),
            Scalar::from_f64(-1.0),
            Scalar::from_f64(0.5),
        ]);
        let cs = axis_aligned_coordinate_system(origin);
        let model = extruded(&cs, unit_square(), vec![]);
        let topology = model.topology();
        assert_model_valid(topology);
        // `w` points along `-z` (see `axis_aligned_coordinate_system`), so
        // the profile sits at the origin's own `z` and the far face one unit
        // *below* it.
        for vertex in topology.vertices.values() {
            assert!(
                vertex.point[2].could_be_equal(Scalar::from_f64(0.5))
                    || vertex.point[2].could_be_equal(Scalar::from_f64(-0.5)),
                "vertex at unexpected height: {:?}",
                vertex.point
            );
        }
    }

    /// A profile anywhere in the plane, not just the unit square: the caps
    /// are sized to it.
    #[test]
    fn extrude_profile_away_from_origin() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([Scalar::ZERO; 3]));
        let outer = polygon(&[v2(-3.0, 2.0), v2(-1.0, 2.0), v2(-2.0, 5.0)]).unwrap();
        let model = extruded(&cs, outer, vec![]);
        let topology = model.topology();
        assert_model_valid(topology);
        assert_eq!(topology.faces.len(), 5);
    }

    /// A disc with a round hole: every wall is a quarter of a cylinder.
    #[test]
    fn extruded_ring_is_valid() {
        let cs = axis_aligned_coordinate_system(Vector3::from_array([Scalar::ZERO; 3]));
        let outer = circle(2.0);
        let hole = circle_at(0.3, 0.0, 1.0, true);
        let model = extruded(&cs, outer, vec![hole]);
        let topology = model.topology();
        assert_model_valid(topology);
        assert_eq!(topology.faces.len(), 2 + 4 + 4);
    }

    /// Both directions off a right-handed plane give valid solids on the
    /// expected side.
    #[test]
    fn extrude_both_directions() {
        let plane = CoordinateSystem::try_new(
            Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::ONE]),
            Vector3::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO]),
            Vector3::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO]),
            Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::ONE]),
        )
        .unwrap();
        let mut outer = polygon(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 1.0)]).unwrap();
        outer.push(
            NurbCurve2D::arc(v2(1.0, 1.0), v2(0.0, 1.0), v2(0.0, 0.0), sqrt2_over_2()).unwrap(),
        );
        outer.remove(2);
        let namer = Namer::new("extrude", "e").unwrap();
        let outer = SweepLoop::plain(Profile::closed(outer));
        for (distance, far) in [(0.5, 1.5), (-0.5, 0.5)] {
            let mut model = NamedModel::new();
            extrude(
                &mut model,
                &namer,
                Some(&namer.root()),
                &plane,
                &[Scalar::ZERO, Scalar::from_f64(distance)],
                std::slice::from_ref(&outer),
            )
            .unwrap();
            model.check_names().unwrap();
            let topology = model.topology();
            assert_model_valid(topology);
            assert!(
                topology
                    .vertices
                    .values()
                    .any(|v| v.point[2].could_be_equal(Scalar::from_f64(far)))
            );
            // Whichever way the profile had to be mirrored to extrude it,
            // every name still sits where it says: `p0` at the profile's
            // first point, the end cap `distance` away from the plane.
            let p0_start = model.vertex_id("extrude(e,p0,start)").unwrap();
            let p0_end = model.vertex_id("extrude(e,p0,end)").unwrap();
            let point = |v| topology.get_vertex(v).unwrap().point;
            assert!(point(p0_start).could_be_equal(&Vector3::from_array([
                Scalar::ZERO,
                Scalar::ZERO,
                Scalar::ONE
            ])));
            assert!(point(p0_end)[2].could_be_equal(Scalar::from_f64(far)));
            let end_cap = model.face_id("extrude(e,end)").unwrap();
            assert!(topology.iterate_face_coedges(end_cap).all(|c| {
                topology.coedge_start_vertex(c).unwrap().point[2]
                    .could_be_equal(Scalar::from_f64(far))
            }));
        }
    }

    fn xy_plane() -> CoordinateSystem {
        CoordinateSystem::world_at(Vector3::zero())
    }

    /// An open chain extrudes into a sheet: a wall per curve, the edges
    /// along its rims and its two ends each used by that one wall alone.
    #[test]
    fn open_chain_extrudes_into_a_sheet() {
        let mut model = NamedModel::new();
        let namer = Namer::new("extrude", "e").unwrap();
        let chain = crate::common::polyline(&[v2(0.0, 0.0), v2(1.0, 0.0), v2(1.0, 2.0)]).unwrap();
        let built = extrude(
            &mut model,
            &namer,
            None,
            &xy_plane(),
            &[Scalar::ZERO, Scalar::ONE],
            &[SweepLoop::plain(Profile::open(chain))],
        )
        .unwrap();
        model.check_names().unwrap();
        assert!(built.solid.is_none());
        let topology = model.topology();
        assert_model_valid_sheets(topology);
        assert_eq!(topology.faces.len(), 2);
        let free = |e: &geop_core_topology::EdgeId| topology.coedges_of_edge(*e).len() == 1;
        assert_eq!(topology.edges.keys().filter(|e| free(e)).count(), 6);
        assert!(model.edge_id("extrude(e,p1)").is_ok_and(|e| !free(&e)));
    }

    /// Between any two ends, either way round: the start cap at the first.
    #[test]
    fn extrude_between_two_ends() {
        for (from, to) in [(-0.5, 1.0), (1.0, -0.5), (0.25, 2.0)] {
            let mut model = NamedModel::new();
            let namer = Namer::new("extrude", "e").unwrap();
            extrude(
                &mut model,
                &namer,
                Some("extrude(e)"),
                &xy_plane(),
                &[Scalar::from_f64(from), Scalar::from_f64(to)],
                &[SweepLoop::plain(Profile::closed(unit_square()))],
            )
            .unwrap();
            let topology = model.topology();
            assert_model_valid(topology);
            let height = |name: &str| {
                topology
                    .get_vertex(model.vertex_id(name).unwrap())
                    .unwrap()
                    .point[2]
            };
            assert!(height("extrude(e,p0,start)").could_be_equal(Scalar::from_f64(from)));
            assert!(height("extrude(e,p2,end)").could_be_equal(Scalar::from_f64(to)));
        }
    }
}

#[cfg(test)]
mod naming_tests {
    use super::*;
    use crate::common::polygon;
    use geop_core_math::scalars::Scalar;
    use geop_core_math::vector::{Vector2, Vector3};
    use geop_ops_model::Profile;

    /// Every entity of an extruded square is named after the profile element
    /// it was swept from, and the names hang together topologically: the
    /// side face of `c0` is bounded by `c0`'s two cap edges and the lateral
    /// edges of its two joints.
    #[test]
    fn extruded_square_names_follow_the_profile() {
        let square = polygon(&[
            Vector2::from_array([Scalar::ZERO, Scalar::ZERO]),
            Vector2::from_array([Scalar::ONE, Scalar::ZERO]),
            Vector2::from_array([Scalar::ONE, Scalar::ONE]),
            Vector2::from_array([Scalar::ZERO, Scalar::ONE]),
        ])
        .unwrap();
        let plane = CoordinateSystem::try_new(
            Vector3::zero(),
            Vector3::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO]),
            Vector3::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO]),
            Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::ONE]),
        )
        .unwrap();
        let mut model = NamedModel::new();
        let namer = Namer::new("extrude", "box").unwrap();
        extrude(
            &mut model,
            &namer,
            Some(&namer.root()),
            &plane,
            &[Scalar::ZERO, Scalar::ONE],
            &[SweepLoop::plain(Profile::closed(square))],
        )
        .unwrap();
        model.check_names().unwrap();

        let description = geop_ops_model::PartDescription::of(&model).unwrap();
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
                .could_be_equal(&Vector3::from_array([Scalar::ONE; 3]))
        );
    }
}
