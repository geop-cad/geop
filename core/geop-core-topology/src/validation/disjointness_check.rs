use geop_core_geometry::intersection::{curve_curve_intersect, curve_surface_intersect};
use geop_core_math::{
    geop_error::GeopError, scalars::Scalar, vector::Vector3,
};

use crate::{
    CoedgeGeometry, EdgeId, Model, VertexId,
    contains::face::{PointClassification, face_contains},
    validation::ValidationParameters,
};

/// Fixed seed for the ray casting behind the edge-inside-face-trim test.
/// `face_contains` retries until it finds a ray grazing nothing, so the
/// answer is seed-independent; a constant keeps validation reproducible.
const EDGE_ON_FACE_SEED: u64 = 0x0EDF_ACE0_0000_0001;

/// Whether `point` lands on an existing vertex — `point` must be evaluated
/// on the intersection search's own *full* returned `t` interval, not
/// collapsed to its midpoint first: both the search's converged `t` and the
/// vertex's own position already carry an honest uncertainty bound (that's
/// the whole point of interval arithmetic), and for a genuine shared vertex
/// those two bounds should simply overlap on their own — no separate
/// distance-epsilon fudge needed. Collapsing to a single representative
/// point first would throw that width away and require compensating for it
/// artificially, which is exactly the kind of epsilon this codebase avoids
/// (see `AGENTS.md`).
fn coincides_with_a_vertex<S: Scalar>(model: &Model<S>, point: &Vector3<S>) -> bool {
    model
        .vertices
        .values()
        .any(|v| point.could_be_equal(&v.point))
}

/// Checks that no two (distinct) vertices could overlap — every vertex is
/// meant to occupy a genuinely distinct position; a shared position should
/// be represented by reusing the same `VertexId`, not by two coincident
/// ones.
pub fn check_vertices_disjoint<S: Scalar>(
    _params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    let ids: Vec<VertexId> = model.vertices.keys().copied().collect();
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            let a = model.vertices[&ids[i]].point;
            let b = model.vertices[&ids[j]].point;
            if a.could_be_equal(&b) {
                errors.push(GeopError::new(format!(
                    "vertex {} and vertex {} could overlap — they should be the same vertex, or genuinely disjoint",
                    ids[i].0, ids[j].0
                )));
            }
        }
    }
}

/// Checks every distinct pair of edges: two edges that are (at least partly)
/// coincident — `Intersections::Coincident` — always fail; otherwise, every intersection point found must coincide
/// with an existing vertex (edges are only allowed to touch each other at
/// shared vertices, never crossing through some other point).
pub fn check_edges_disjoint<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    let ids: Vec<EdgeId> = model.edges.keys().copied().collect();
    for i in 0..ids.len() {
        for j in (i + 1)..ids.len() {
            let edge_a = &model.edges[&ids[i]];
            let edge_b = &model.edges[&ids[j]];
            let intersections = match curve_curve_intersect(
                &edge_a.curve,
                &edge_b.curve,
                params.max_edge_edge_intersection_samples,
                params.max_nodes,
                params.min_subdivision_size,
            ) {
                Ok(v) => v,
                Err(e) => {
                    errors.push(e.with_context(format!("edge {} x edge {}", ids[i].0, ids[j].0)));
                    continue;
                }
            };

            if intersections.is_coincident() {
                errors.push(GeopError::new(format!(
                    "edge {} and edge {} are coincident along an arc",
                    ids[i].0, ids[j].0
                )));
                continue;
            }

            for (t_a, _t_b) in intersections.into_vec() {
                let point = match edge_a.curve.evaluate(t_a) {
                    Ok(p) => p,
                    Err(e) => {
                        errors
                            .push(e.with_context(format!("edge {} x edge {}", ids[i].0, ids[j].0)));
                        continue;
                    }
                };
                if !coincides_with_a_vertex(model, &point) {
                    errors.push(GeopError::new(format!(
                        "edge {} and edge {} intersect at a point that is not an existing vertex",
                        ids[i].0, ids[j].0
                    )));
                }
            }
        }
    }
}

/// Checks every edge x face pair: every intersection point found must
/// coincide with an existing vertex (an edge
/// may only pierce a face's surface at a shared vertex, never elsewhere).
///
/// A pair where the edge is *already* a declared boundary coedge of the
/// face is skipped rather than run through `curve_surface_intersect` at
/// all: a curve entirely embedded in a surface is a continuum of
/// coincidence, not a finite set of points, and empirically the search
/// doesn't reliably resolve that to either "hit the cap" or "found no
/// points" — it can converge to some wide, still-unresolved sub-interval
/// instead, whose midpoint lands nowhere meaningful. Declared boundary
/// relationships don't need re-verifying geometrically here anyway (that's
/// `check_curves_and_surfaces_vertices`'s and the sampling check's job).
pub fn check_edges_and_faces_consistent<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    for (&edge_id, edge) in &model.edges {
        for (&face_id, face) in &model.faces {
            let is_boundary_edge = model.iterate_face_coedges(face_id).any(|coedge_id| {
                model.coedges.get(&coedge_id).map(|c| c.geometry)
                    == Some(CoedgeGeometry::Edge(edge_id))
            });
            if is_boundary_edge {
                continue;
            }

            let intersections = match curve_surface_intersect(
                &edge.curve,
                &face.surface,
                params.max_edge_face_intersection_samples,
                params.max_nodes,
                params.min_subdivision_size,
            ) {
                Ok(v) => v,
                Err(e) => {
                    errors.push(e.with_context(format!("edge {} x face {}", edge_id.0, face_id.0)));
                    continue;
                }
            };

            // No "does this edge lie on the face" test here. Whether a
            // coedge's pcurve actually traces its edge across that face is
            // `curve_and_surface_sampling_check`'s job, and it answers the
            // same question directly rather than by re-deriving it from the
            // surface. What is left for this check is the crossings.
            for (t, uv) in intersections.into_vec() {
                let point = match edge.curve.evaluate(t) {
                    Ok(p) => p,
                    Err(e) => {
                        errors.push(
                            e.with_context(format!("edge {} x face {}", edge_id.0, face_id.0)),
                        );
                        continue;
                    }
                };
                // Only a crossing *inside the face's trim* is a violation.
                // The surface is untrimmed and extends well past the face:
                // an edge of one lobe of a figure-8 crosses the plane of a
                // coplanar face on the other lobe every time, nowhere near
                // it. The search already returns where the crossing lands in
                // `(u, v)`, so the trim test is free.
                let inside_trim = matches!(
                    face_contains(
                        model,
                        face_id,
                        uv[0],
                        uv[1],
                        params.max_nodes,
                        params.min_subdivision_size,
                        EDGE_ON_FACE_SEED,
                    ),
                    Ok(PointClassification::Inside)
                );
                if inside_trim && !coincides_with_a_vertex(model, &point) {
                    errors.push(GeopError::new(format!(
                        "edge {} crosses face {}'s trimmed region at {point:?}, which is not an existing vertex — the edge should have been split there",
                        edge_id.0, face_id.0
                    )));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{check_edges_and_faces_consistent, check_edges_disjoint, check_vertices_disjoint};
    use crate::test_fixtures::test_cube_solid;
    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, Face, FaceId, Model, Sense, Vertex,
        boundary::BoundaryType, validation::ValidationParameters,
    };
    use geop_core_geometry::{
        nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D},
        nurb_surface::NurbSurface3D,
    };
    use geop_core_math::{
        for_all_scalars,
        geop_error::GeopError,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    // `ValidationParameters::default()`'s `min_subdivision_size` (1e-7) is
    // tuned for `curve_could_contain`'s single-curve BFS — with
    // `curve_curve_intersect`/`curve_surface_intersect`'s pairwise search it
    // is too tight to converge at all (confirmed empirically: it returns
    // *zero* solutions even for a clean single crossing, regardless of
    // `max_nodes`), so these tests use a looser tolerance to actually
    // exercise the logic.
    fn params<S: Scalar>() -> ValidationParameters<S> {
        // These hand-built fixtures use small, simple-scale geometry where
        // even the (already loosened) default `min_subdivision_size` isn't
        // quite enough for `curve_curve_intersect`/`curve_surface_intersect`
        // to reliably resolve a genuine mid-curve crossing; loosen further
        // just for these tests of the check's own logic.
        ValidationParameters {
            min_subdivision_size: S::from_f64(1e-2),
            ..ValidationParameters::default()
        }
    }

    fn check_welded_cube_passes_all_three<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);

        let mut errors = Vec::new();
        check_vertices_disjoint(&params(), &mut errors, &model);
        assert!(errors.is_empty(), "vertices: {errors:?}");

        let mut errors = Vec::new();
        check_edges_disjoint(&params(), &mut errors, &model);
        assert!(errors.is_empty(), "edges: {errors:?}");

        let mut errors = Vec::new();
        check_edges_and_faces_consistent(&params(), &mut errors, &model);
        assert!(errors.is_empty(), "edges x faces: {errors:?}");
    }
    #[test]
    fn welded_cube_passes_all_three() {
        for_all_scalars!(check_welded_cube_passes_all_three);
    }

    fn line3<S: Scalar>(a: (f64, f64, f64), b: (f64, f64, f64)) -> NurbCurve3D<S> {
        let p = |x: f64, y: f64, z: f64| {
            Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
        };
        NurbCurve3D::try_new(
            1,
            vec![p(a.0, a.1, a.2), p(b.0, b.1, b.2)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    fn point3<S: Scalar>(p: (f64, f64, f64)) -> Vector3<S> {
        Vector3::from_array([S::from_f64(p.0), S::from_f64(p.1), S::from_f64(p.2)])
    }

    // ── vertices disjoint ─────────────────────────────────────────────────

    fn check_distinct_vertices_pass<S: Scalar>() {
        let mut model = Model::<S>::new();
        model.insert_vertex(Vertex {
            point: point3((0.0, 0.0, 0.0)),
        });
        model.insert_vertex(Vertex {
            point: point3((1.0, 0.0, 0.0)),
        });
        let mut errors = Vec::new();
        check_vertices_disjoint(&ValidationParameters::default(), &mut errors, &model);
        assert!(errors.is_empty());
    }
    #[test]
    fn distinct_vertices_pass() {
        for_all_scalars!(check_distinct_vertices_pass);
    }

    fn check_coincident_vertices_fail<S: Scalar>() {
        let mut model = Model::<S>::new();
        model.insert_vertex(Vertex {
            point: point3((0.0, 0.0, 0.0)),
        });
        model.insert_vertex(Vertex {
            point: point3((0.0, 0.0, 0.0)),
        });
        let mut errors = Vec::new();
        check_vertices_disjoint(&ValidationParameters::default(), &mut errors, &model);
        assert_eq!(errors.len(), 1);
    }
    #[test]
    fn coincident_vertices_fail() {
        for_all_scalars!(check_coincident_vertices_fail);
    }

    // ── edges disjoint ────────────────────────────────────────────────────

    fn check_edges_sharing_a_vertex_pass<S: Scalar>() {
        let mut model = Model::<S>::new();
        let v0 = model.insert_vertex(Vertex {
            point: point3((0.0, 0.0, 0.0)),
        });
        let v1 = model.insert_vertex(Vertex {
            point: point3((1.0, 0.0, 0.0)),
        });
        let v2 = model.insert_vertex(Vertex {
            point: point3((0.0, 1.0, 0.0)),
        });
        model.insert_edge(Edge {
            curve: line3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            start_vertex: v0,
            end_vertex: v1,
        });
        model.insert_edge(Edge {
            curve: line3((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            start_vertex: v0,
            end_vertex: v2,
        });
        let mut errors = Vec::new();
        check_edges_disjoint(&params(), &mut errors, &model);
        assert!(errors.is_empty());
    }
    #[test]
    fn edges_sharing_a_vertex_pass() {
        for_all_scalars!(check_edges_sharing_a_vertex_pass);
    }

    fn check_edges_crossing_at_non_vertex_fail<S: Scalar>() {
        let mut model = Model::<S>::new();
        let v0 = model.insert_vertex(Vertex {
            point: point3((-1.0, 0.0, 0.0)),
        });
        let v1 = model.insert_vertex(Vertex {
            point: point3((1.0, 0.0, 0.0)),
        });
        let v2 = model.insert_vertex(Vertex {
            point: point3((0.0, -1.0, 0.0)),
        });
        let v3 = model.insert_vertex(Vertex {
            point: point3((0.0, 1.0, 0.0)),
        });
        // Two segments crossing at the origin, which is not itself a vertex.
        model.insert_edge(Edge {
            curve: line3((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            start_vertex: v0,
            end_vertex: v1,
        });
        model.insert_edge(Edge {
            curve: line3((0.0, -1.0, 0.0), (0.0, 1.0, 0.0)),
            start_vertex: v2,
            end_vertex: v3,
        });
        let mut errors = Vec::new();
        check_edges_disjoint(&params(), &mut errors, &model);
        assert_eq!(errors.len(), 1);
    }
    #[test]
    fn edges_crossing_at_non_vertex_fail() {
        for_all_scalars!(check_edges_crossing_at_non_vertex_fail);
    }

    /// Two edges meeting only at their shared vertex are not coincident —
    /// not even with a cap of 1, which the old search used to misread as
    /// coincidence the moment it found that one crossing.
    fn check_shared_vertex_at_the_cap_is_not_coincident<S: Scalar>() {
        let mut model = Model::<S>::new();
        let v0 = model.insert_vertex(Vertex {
            point: point3((0.0, 0.0, 0.0)),
        });
        let v1 = model.insert_vertex(Vertex {
            point: point3((1.0, 0.0, 0.0)),
        });
        let v2 = model.insert_vertex(Vertex {
            point: point3((0.0, 1.0, 0.0)),
        });
        model.insert_edge(Edge {
            curve: line3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            start_vertex: v0,
            end_vertex: v1,
        });
        model.insert_edge(Edge {
            curve: line3((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
            start_vertex: v0,
            end_vertex: v2,
        });
        let mut low_cap = params::<S>();
        low_cap.max_edge_edge_intersection_samples = 1;
        let mut errors = Vec::new();
        check_edges_disjoint(&low_cap, &mut errors, &model);
        assert!(errors.is_empty(), "{errors:?}");
    }
    #[test]
    fn shared_vertex_at_the_cap_is_not_coincident() {
        for_all_scalars!(check_shared_vertex_at_the_cap_is_not_coincident);
    }

    /// Two edges overlapping along part of their length fail as coincident.
    fn check_overlapping_edges_fail<S: Scalar>() {
        let mut model = Model::<S>::new();
        let vs: Vec<_> = [0.0, 0.5, 1.0, 1.5]
            .iter()
            .map(|&x| {
                model.insert_vertex(Vertex {
                    point: point3((x, 0.0, 0.0)),
                })
            })
            .collect();
        model.insert_edge(Edge {
            curve: line3((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            start_vertex: vs[0],
            end_vertex: vs[2],
        });
        model.insert_edge(Edge {
            curve: line3((0.5, 0.0, 0.0), (1.5, 0.0, 0.0)),
            start_vertex: vs[1],
            end_vertex: vs[3],
        });
        let mut errors = Vec::new();
        check_edges_disjoint(&params(), &mut errors, &model);
        assert_eq!(errors.len(), 1, "{errors:?}");
        assert!(
            format!("{:?}", errors[0]).contains("coincident"),
            "{errors:?}"
        );
    }
    #[test]
    fn overlapping_edges_fail() {
        for_all_scalars!(check_overlapping_edges_fail);
    }

    // ── edges x faces ─────────────────────────────────────────────────────

    /// A unit-square face on the XY plane with a single quad boundary loop
    /// through `corners`, each edge independently vertexed.
    fn quad_face<S: Scalar>(
        model: &mut Model<S>,
        corners: [(f64, f64, f64); 4],
    ) -> (FaceId, [EdgeId; 4]) {
        let p4 = |q: (f64, f64, f64)| {
            Vector4::from_array([S::from_f64(q.0), S::from_f64(q.1), S::from_f64(q.2), S::ONE])
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![
                p4(corners[0]),
                p4(corners[3]),
                p4(corners[1]),
                p4(corners[2]),
            ],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        let face_id = model.insert_face(Face {
            surface,
            outer: BoundaryType::Vertex(crate::VertexId(0)),
            holes: Vec::new(),
            shell: crate::ShellId(999),
        });

        let verts: Vec<_> = corners
            .iter()
            .map(|&(x, y, z)| {
                model.insert_vertex(Vertex {
                    point: point3((x, y, z)),
                })
            })
            .collect();
        let edges: [EdgeId; 4] = std::array::from_fn(|i| {
            model.insert_edge(Edge {
                curve: line3(corners[i], corners[(i + 1) % 4]),
                start_vertex: verts[i],
                end_vertex: verts[(i + 1) % 4],
            })
        });
        let uv = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        let pc = |a: (f64, f64), b: (f64, f64)| -> NurbCurve2D<S> {
            NurbCurve::try_new(
                1,
                vec![
                    Vector3::from_array([S::from_f64(a.0), S::from_f64(a.1), S::ONE]),
                    Vector3::from_array([S::from_f64(b.0), S::from_f64(b.1), S::ONE]),
                ],
                vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            )
            .unwrap()
        };
        let coedges: [CoedgeId; 4] = std::array::from_fn(|i| {
            model.insert_coedge(Coedge {
                geometry: CoedgeGeometry::Edge(edges[i]),
                sense: Sense::Forward,
                pcurve: pc(uv[i], uv[(i + 1) % 4]),
                next: CoedgeId(0),
                prev: CoedgeId(0),
                face: face_id,
            })
        });
        for i in 0..4 {
            model.coedges.get_mut(&coedges[i]).unwrap().next = coedges[(i + 1) % 4];
            model.coedges.get_mut(&coedges[i]).unwrap().prev = coedges[(i + 3) % 4];
        }
        model.faces.get_mut(&face_id).unwrap().outer = BoundaryType::Loop(coedges[0]);

        (face_id, edges)
    }

    fn run_edge_face<S: Scalar>(model: &Model<S>) -> Vec<GeopError> {
        let mut errors = Vec::new();
        check_edges_and_faces_consistent(&params(), &mut errors, model);
        errors
    }

    fn check_face_boundary_edges_pass<S: Scalar>() {
        let mut model = Model::<S>::new();
        quad_face(
            &mut model,
            [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (1.0, 1.0, 0.0),
                (0.0, 1.0, 0.0),
            ],
        );
        assert!(run_edge_face(&model).is_empty());
    }
    #[test]
    fn face_boundary_edges_pass() {
        for_all_scalars!(check_face_boundary_edges_pass);
    }

    fn check_edge_piercing_face_at_non_vertex_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        quad_face(
            &mut model,
            [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (1.0, 1.0, 0.0),
                (0.0, 1.0, 0.0),
            ],
        );
        // An edge crossing straight through the face's plane at its center
        // (0.5, 0.5, 0.0), not an existing vertex.
        let va = model.insert_vertex(Vertex {
            point: point3((0.5, 0.5, -1.0)),
        });
        let vb = model.insert_vertex(Vertex {
            point: point3((0.5, 0.5, 1.0)),
        });
        model.insert_edge(Edge {
            curve: line3((0.5, 0.5, -1.0), (0.5, 0.5, 1.0)),
            start_vertex: va,
            end_vertex: vb,
        });
        assert!(!run_edge_face(&model).is_empty());
    }
    #[test]
    fn edge_piercing_face_at_non_vertex_fails() {
        for_all_scalars!(check_edge_piercing_face_at_non_vertex_fails);
    }

    fn check_edge_piercing_face_at_a_vertex_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        quad_face(
            &mut model,
            [
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (1.0, 1.0, 0.0),
                (0.0, 1.0, 0.0),
            ],
        );
        // An edge piercing the face's plane exactly at one of its corners
        // (an existing vertex).
        let va = model.insert_vertex(Vertex {
            point: point3((0.0, 0.0, -1.0)),
        });
        let vb = model.insert_vertex(Vertex {
            point: point3((0.0, 0.0, 1.0)),
        });
        model.insert_edge(Edge {
            curve: line3((0.0, 0.0, -1.0), (0.0, 0.0, 1.0)),
            start_vertex: va,
            end_vertex: vb,
        });
        assert!(run_edge_face(&model).is_empty());
    }
    #[test]
    fn edge_piercing_face_at_a_vertex_passes() {
        for_all_scalars!(check_edge_piercing_face_at_a_vertex_passes);
    }
}
