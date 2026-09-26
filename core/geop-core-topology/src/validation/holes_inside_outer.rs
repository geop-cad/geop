use crate::{
    Model,
    boundary::BoundaryType,
    contains::face::{PointClassification, loops_contain},
    validation::ValidationParameters,
};
use geop_core_geometry::contains::surface::surface_could_contain;
use geop_core_math::{geop_error::GeopError, scalars::Scalar, vector::Vector2};

/// Fixed seed for the ray casting. `loops_contain` retries until it finds a
/// ray grazing no vertex, so the classification is seed-independent and a
/// constant keeps validation reproducible run to run.
const SEED: u64 = 0x00C0_FFEE_0BAD_F00D;

/// Checks that every hole of a face lies **inside** that face's outer
/// boundary.
///
/// A face is the material inside its outer loop minus everything inside its
/// holes, so a hole outside the outer loop subtracts nothing and a hole
/// straddling it subtracts a region that was never there. Both make the face
/// geometrically meaningless while leaving every structural invariant intact:
/// the loops are well-formed, their pcurves are continuous, and every coedge
/// points where it should. Nothing else in `validate` looks at how the loops
/// are arranged *relative to each other*, so without this a face split that
/// hands a hole to the wrong side passes silently — which is exactly what
/// `splice_edge_into_face`'s hole reclassification can get wrong.
///
/// Each hole is tested at one point on it. That is sufficient for the
/// failures this is meant to catch: holes and the outer loop cannot cross
/// (`check_edges_disjoint` already rejects that), so a hole is wholly inside
/// or wholly outside, and any point on it decides which.
///
/// Deliberately not part of `validate_fast`: this casts a ray per hole and
/// runs a pairwise curve intersection against every coedge of the outer loop,
/// which is the same cost class as the other geometric checks in the full
/// `validate` rather than the structural ones.
pub fn check_holes_inside_outer<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    for (&face_id, face) in &model.faces {
        if face.holes.is_empty() {
            continue;
        }
        // A face still bounded by a bare vertex has no interior for anything
        // to be inside of; that it also carries holes is `mvr`'s business and
        // not a containment failure.
        let BoundaryType::Loop(outer_anchor) = face.outer else {
            continue;
        };
        // Capped: a ring that never returns to its anchor is
        // `two_way_references`' error to report, not a reason to hang here.
        let cap = model.coedges.len() + 1;
        let outer: Vec<_> = model.iterate_loop_coedges(outer_anchor).take(cap).collect();
        if outer.len() >= cap {
            continue;
        }

        for (index, &hole) in face.holes.iter().enumerate() {
            let uv = match hole_point(model, params, face_id, hole) {
                Ok(Some(uv)) => uv,
                Ok(None) => {
                    errors.push(GeopError::new(format!(
                        "face {face_id}'s hole {index} ({hole:?}) has no point on this face's surface, so it cannot be shown to lie inside the outer boundary"
                    )));
                    continue;
                }
                Err(e) => {
                    errors.push(e.with_context(format!("face {face_id}, hole {index} ({hole:?})")));
                    continue;
                }
            };

            match loops_contain(
                model,
                &face.surface,
                &outer,
                uv,
                params.max_nodes,
                params.min_subdivision_size,
                SEED,
            ) {
                Ok(PointClassification::Inside) => {}
                Ok(other) => {
                    errors.push(GeopError::new(format!(
                        "face {face_id}'s hole {index} ({hole:?}) is at uv {uv:?}, which is {other:?} its outer boundary rather than Inside it — a hole must lie within the region its face bounds"
                    )));
                }
                Err(e) => {
                    errors.push(e.with_context(format!(
                        "face {face_id}, hole {index} ({hole:?}): classifying it against the outer boundary"
                    )));
                }
            }
        }
    }
}

/// A `(u, v)` on `hole`, in the interior of its anchor's pcurve. A `Loop`
/// reads one straight off that pcurve;
/// a bare `Vertex` boundary has no pcurve, so its 3-D point is located on the
/// face's surface instead.
fn hole_point<S: Scalar>(
    model: &Model<S>,
    params: &ValidationParameters<S>,
    face_id: crate::FaceId,
    hole: BoundaryType,
) -> Result<Option<Vector2<S>>, GeopError> {
    match hole {
        BoundaryType::Loop(anchor) => {
            // The *midpoint* of the anchor's pcurve, not its start. A loop's
            // endpoints are precisely where it can legitimately touch other
            // loops at a shared vertex, and a query point sitting on a vertex
            // of the outer loop classifies as `OnVertex` — telling us nothing
            // about which side of it the hole lies on. The curve's interior
            // carries the same information without the coincidence.
            let pcurve = &model.get_coedge(anchor)?.pcurve;
            let (t0, t1) = pcurve.domain();
            let mid = t0.add(t1).div(S::TWO)?.sharpen();
            Ok(Some(pcurve.evaluate(mid)?))
        }
        BoundaryType::Vertex(vertex_id) => {
            let point = model.get_vertex(vertex_id)?.point;
            let surface = &model.get_face(face_id)?.surface;
            Ok(surface_could_contain(
                surface,
                &point,
                params.max_nodes,
                params.min_subdivision_size,
            )?
            .map(|(u, v)| Vector2::from_array([u, v])))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::check_holes_inside_outer;
    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Edge, Face, FaceId, Model, Sense, ShellId, Vertex,
        VertexId, boundary::BoundaryType, validation::ValidationParameters,
    };
    use geop_core_geometry::{
        nurb_curve::{NurbCurve, NurbCurve2D},
        nurb_surface::NurbSurface3D,
    };
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    fn line2<S: Scalar>(a: (f64, f64), b: (f64, f64)) -> NurbCurve2D<S> {
        NurbCurve::try_new(
            1,
            vec![
                Vector3::from_array([S::from_f64(a.0), S::from_f64(a.1), S::ONE]),
                Vector3::from_array([S::from_f64(b.0), S::from_f64(b.1), S::ONE]),
            ],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    /// A flat unit patch in the xy-plane, big enough to hold every fixture
    /// below inside its own `(u, v)` domain.
    fn flat_face<S: Scalar>(model: &mut Model<S>) -> FaceId {
        let p =
            |x: f64, y: f64| Vector4::from_array([S::from_f64(x), S::from_f64(y), S::ZERO, S::ONE]);
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p(0.0, 0.0), p(0.0, 4.0), p(4.0, 0.0), p(4.0, 4.0)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        model.insert_face(Face {
            surface,
            outer: BoundaryType::Vertex(VertexId(0)),
            holes: Vec::new(),
            shell: ShellId(999),
        })
    }

    /// A closed ring of coedges through `points` (in `(u, v)`, which this
    /// surface maps affinely to `(x, y)`), returning its anchor.
    fn ring<S: Scalar>(model: &mut Model<S>, face_id: FaceId, points: &[(f64, f64)]) -> CoedgeId {
        let n = points.len();
        let verts: Vec<VertexId> = points
            .iter()
            .map(|&(u, v)| {
                model.insert_vertex(Vertex {
                    point: Vector3::from_array([
                        S::from_f64(u * 4.0),
                        S::from_f64(v * 4.0),
                        S::ZERO,
                    ]),
                })
            })
            .collect();
        let coedges: Vec<CoedgeId> = (0..n)
            .map(|i| {
                let a = points[i];
                let b = points[(i + 1) % n];
                let edge = model.insert_edge(Edge {
                    curve: NurbCurve::try_new(
                        1,
                        vec![
                            Vector4::from_array([
                                S::from_f64(a.0 * 4.0),
                                S::from_f64(a.1 * 4.0),
                                S::ZERO,
                                S::ONE,
                            ]),
                            Vector4::from_array([
                                S::from_f64(b.0 * 4.0),
                                S::from_f64(b.1 * 4.0),
                                S::ZERO,
                                S::ONE,
                            ]),
                        ],
                        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                    )
                    .unwrap(),
                    start_vertex: verts[i],
                    end_vertex: verts[(i + 1) % n],
                });
                model.insert_coedge(Coedge {
                    geometry: CoedgeGeometry::Edge(edge),
                    sense: Sense::Forward,
                    pcurve: line2(a, b),
                    next: CoedgeId(0),
                    prev: CoedgeId(0),
                    face: face_id,
                })
            })
            .collect();
        for i in 0..n {
            model.coedges.get_mut(&coedges[i]).unwrap().next = coedges[(i + 1) % n];
            model.coedges.get_mut(&coedges[i]).unwrap().prev = coedges[(i + n - 1) % n];
        }
        coedges[0]
    }

    fn params<S: Scalar>() -> ValidationParameters<S> {
        ValidationParameters {
            min_subdivision_size: S::from_f64(1e-3),
            ..ValidationParameters::default()
        }
    }

    const OUTER: [(f64, f64); 4] = [(0.1, 0.1), (0.9, 0.1), (0.9, 0.9), (0.1, 0.9)];

    fn check_hole_inside_outer_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face = flat_face(&mut model);
        let outer = ring::<S>(&mut model, face, &OUTER);
        let hole = ring::<S>(
            &mut model,
            face,
            &[(0.4, 0.4), (0.6, 0.4), (0.6, 0.6), (0.4, 0.6)],
        );
        let f = model.faces.get_mut(&face).unwrap();
        f.outer = BoundaryType::Loop(outer);
        f.holes = vec![BoundaryType::Loop(hole)];

        let mut errors = Vec::new();
        check_holes_inside_outer(&params(), &mut errors, &model);
        assert!(errors.is_empty(), "{errors:?}");
    }
    #[test]
    fn hole_inside_outer_passes() {
        for_all_scalars!(check_hole_inside_outer_passes);
    }

    /// The failure this check exists for: a hole sitting entirely outside the
    /// loop that bounds the face. Every structural invariant still holds —
    /// both rings are well-formed and closed — so nothing else in `validate`
    /// notices.
    fn check_hole_outside_outer_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face = flat_face(&mut model);
        let outer = ring::<S>(&mut model, face, &OUTER);
        let hole = ring::<S>(
            &mut model,
            face,
            &[(0.02, 0.02), (0.06, 0.02), (0.06, 0.06), (0.02, 0.06)],
        );
        let f = model.faces.get_mut(&face).unwrap();
        f.outer = BoundaryType::Loop(outer);
        f.holes = vec![BoundaryType::Loop(hole)];

        let mut errors = Vec::new();
        check_holes_inside_outer(&params(), &mut errors, &model);
        assert_eq!(errors.len(), 1, "{errors:?}");
    }
    #[test]
    fn hole_outside_outer_fails() {
        for_all_scalars!(check_hole_outside_outer_fails);
    }

    /// A face with no holes has nothing to check, including one still bounded
    /// by a bare vertex.
    fn check_face_without_holes_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        let face = flat_face(&mut model);
        let outer = ring::<S>(&mut model, face, &OUTER);
        model.faces.get_mut(&face).unwrap().outer = BoundaryType::Loop(outer);

        let mut errors = Vec::new();
        check_holes_inside_outer(&params(), &mut errors, &model);
        assert!(errors.is_empty(), "{errors:?}");
    }
    #[test]
    fn face_without_holes_passes() {
        for_all_scalars!(check_face_without_holes_passes);
    }
}
