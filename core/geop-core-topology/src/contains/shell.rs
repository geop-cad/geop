//! Point-in-solid containment via ray casting against a shell's vertices,
//! edges, and faces in 3-D — the 3-D analog of `super::face::face_contains`.
//!
//! Coincidence with the boundary is checked first (vertex, then edge, then
//! face — each a full pass, so a higher-priority coincidence is never
//! shadowed by iteration order). Otherwise a ray is cast from the query
//! point in a random direction (drawn from a seeded PRNG, see
//! [`super::rng::Rng`]), retried with a fresh direction whenever it grazes a
//! vertex or an edge (both ambiguous to count reliably — an edge is shared
//! by two faces, a vertex by several edges), until one is found whose only
//! crossings are clean face-interior hits. The even/odd parity of that
//! crossing count then gives inside/outside — this needs no consistently
//! oriented face normal, only a clean ray.

use std::collections::HashSet;

use crate::{CoedgeGeometry, EdgeId, Model, ShellId, VertexId};
use geop_core_geometry::{
    contains::{curve::curve_could_contain, surface::surface_could_contain},
    intersection::{curve_curve_intersect, curve_surface_intersect},
    nurb_curve::NurbCurve3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::{
    face::{PointClassification as FaceClassification, face_contains},
    rng::Rng,
};

/// Result of classifying a query point against a shell's boundary.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PointClassification {
    /// The query point coincides with a vertex.
    OnVertex,
    /// The query point lies on an edge, away from its endpoints.
    OnEdge,
    /// The query point lies on a face's interior surface.
    OnFace,
    /// The query point is strictly inside the shell.
    Inside,
    /// The query point is strictly outside the shell.
    Outside,
}

const MAX_RAY_ATTEMPTS: usize = 64;

fn line3<S: Scalar>(a: Vector3<S>, b: Vector3<S>) -> GeopResult<NurbCurve3D<S>> {
    NurbCurve3D::try_new(
        1,
        vec![
            Vector4::from_array([a[0], a[1], a[2], S::ONE]),
            Vector4::from_array([b[0], b[1], b[2], S::ONE]),
        ],
        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
    )
}

/// The distinct vertices and edges referenced by `shell_id`'s faces.
pub(crate) fn shell_vertices_and_edges<S: Scalar>(
    model: &Model<S>,
    shell_id: ShellId,
) -> (Vec<VertexId>, Vec<EdgeId>) {
    let shell = &model.shells[&shell_id];
    let mut vertex_ids = Vec::new();
    let mut edge_ids = Vec::new();
    let mut seen_v = HashSet::new();
    let mut seen_e = HashSet::new();
    for &face_id in &shell.faces {
        for coedge_id in model.iterate_face_coedges(face_id) {
            let coedge = &model.coedges[&coedge_id];
            match coedge.geometry {
                CoedgeGeometry::Edge(edge_id) => {
                    if seen_e.insert(edge_id) {
                        edge_ids.push(edge_id);
                    }
                    let edge = &model.edges[&edge_id];
                    if seen_v.insert(edge.start_vertex) {
                        vertex_ids.push(edge.start_vertex);
                    }
                    if seen_v.insert(edge.end_vertex) {
                        vertex_ids.push(edge.end_vertex);
                    }
                }
                CoedgeGeometry::Vertex(vertex_id) => {
                    if seen_v.insert(vertex_id) {
                        vertex_ids.push(vertex_id);
                    }
                }
            }
        }
    }
    (vertex_ids, edge_ids)
}

/// A ray length guaranteed to clear `vertex_ids`' whole spatial footprint
/// from `point` — long enough that a ray this long finding zero crossings
/// unambiguously means "missed the shell entirely" (i.e. outside), not
/// "wasn't cast far enough".
pub(crate) fn ray_length_for<S: Scalar>(
    model: &Model<S>,
    vertex_ids: &[VertexId],
    point: &Vector3<S>,
) -> GeopResult<S> {
    let mut max_dist_sq = S::ONE;
    for &vertex_id in vertex_ids {
        let d = model.vertices[&vertex_id].point.sub(point).norm_sq();
        if d.could_be_greater(max_dist_sq) {
            max_dist_sq = d;
        }
    }
    Ok(max_dist_sq.sqrt()?.mul(S::from_f64(3.0)).add(S::ONE))
}

/// Casts a single ray from `point` in `direction` against `shell_id`'s
/// vertices/edges/faces (`vertex_ids`/`edge_ids` as returned by
/// [`shell_vertices_and_edges`], `ray_length` by [`ray_length_for`]) and
/// returns its even/odd crossing-parity classification — or `Ok(Err(why))` if
/// this particular direction is degenerate (grazes a vertex or edge, or
/// hits a face right on its own trim boundary) and the caller should retry
/// with a different one. Factored out of [`shell_contains`] so other
/// callers (see `validation`'s manifold check) can cast several different
/// directions from the same point without redoing its per-point setup.
///
/// `seed` seeds the `face_contains` sub-calls used to classify face-interior
/// hits; callers casting multiple rays from one call site should vary it
/// per ray (e.g. by direction or attempt index) so those sub-calls don't all
/// retry along the identical degenerate path.
pub(crate) fn cast_ray<S: Scalar>(
    model: &Model<S>,
    shell_id: ShellId,
    point: Vector3<S>,
    direction: Vector3<S>,
    vertex_ids: &[VertexId],
    edge_ids: &[EdgeId],
    ray_length: S,
    max_nodes: usize,
    epsilon: S,
    seed: u64,
) -> GeopResult<Result<PointClassification, String>> {
    let shell = &model.shells[&shell_id];
    let t_epsilon = epsilon.div(ray_length)?;
    let far = point.add(&direction.prod_scalar(ray_length));
    let ray = line3(point, far)?;

    // A ray grazing a vertex is ambiguous (shared by several edges).
    for &vertex_id in vertex_ids {
        let vp = model.vertices[&vertex_id].point;
        if curve_could_contain(&ray, &vp, max_nodes, epsilon)?.is_some() {
            return Ok(Err(format!(
                "it could pass through vertex {vertex_id} at {vp:?}"
            )));
        }
    }

    // A ray crossing an edge is ambiguous (shared by two faces), and so is a
    // ray lying along one. An `Err` (the search exhausted its node budget) is
    // no answer at all, so it too asks the caller to retry. One point per
    // hit is all this needs, so `max_solutions` is 1: a coincident ray is
    // rejected either way, and a larger cap would only buy samples of it.
    for &edge_id in edge_ids {
        let full_curve = &model.edges[&edge_id].curve;
        let hits = match curve_curve_intersect(&ray, full_curve, 1, max_nodes, epsilon) {
            Ok(hits) => hits,
            Err(e) => {
                return Ok(Err(format!(
                    "its search against edge {edge_id} failed: {e}"
                )));
            }
        };
        if hits.is_coincident() {
            return Ok(Err(format!("it runs along edge {edge_id}")));
        }
        for (t_hit, _mid) in hits.into_vec() {
            if t_hit.definitely_greater(t_epsilon) {
                return Ok(Err(format!("it could cross edge {edge_id} at t={t_hit:?}")));
            }
        }
    }

    let mut count = 0usize;
    for &face_id in &shell.faces {
        let surface = &model.faces[&face_id].surface;
        // A ray lying in a face is ambiguous, and an `Err` (node budget
        // exhausted) is no answer; both ask the caller to retry. Every
        // crossing is needed for the parity count, so the cap is `max_nodes`
        // — the most crossings a search of that budget could ever report.
        let hits = match curve_surface_intersect(&ray, surface, max_nodes, max_nodes, epsilon) {
            Ok(hits) => hits,
            Err(e) => {
                return Ok(Err(format!(
                    "its search against face {face_id} failed: {e}"
                )));
            }
        };
        if hits.is_coincident() {
            return Ok(Err(format!("it lies in face {face_id}'s surface")));
        }
        for (t_hit, uv) in hits.into_vec() {
            if !t_hit.definitely_greater(t_epsilon) {
                continue;
            }
            // Sharpen — `curve_surface_intersect` honestly returns the
            // whole surviving span of its converged leaf, not an
            // arbitrarily narrowed midpoint.
            let (u, v) = (uv[0].midpoint(), uv[1].midpoint());
            let face_seed = seed ^ face_id.0;
            match face_contains(model, face_id, u, v, max_nodes, epsilon, face_seed) {
                Ok(FaceClassification::Inside) => count += 1,
                Ok(FaceClassification::Outside) => {}
                // A hit right on this face's own trim boundary, or an
                // outright error resolving it, is the same ambiguity as a
                // vertex/edge graze one dimension down.
                other => {
                    return Ok(Err(format!(
                        "its hit on face {face_id} at uv=({u:?}, {v:?}) classified as {other:?}"
                    )));
                }
            }
        }
    }
    // Unlike `face_contains` (whose ray is guaranteed to cross the outer
    // trim loop for any query point within the surface's own bounded
    // parameter domain), a shell occupies a bounded region of otherwise
    // unbounded 3-D space: a ray this long that finds zero crossings has,
    // by construction, missed the shell's spatial footprint entirely, which
    // can only happen when the query point is outside — zero is a
    // legitimate (even) count, not degenerate.
    Ok(Ok(if count % 2 == 1 {
        PointClassification::Inside
    } else {
        PointClassification::Outside
    }))
}

/// Classify `point` against `shell_id`'s boundary: [`PointClassification::OnVertex`] /
/// [`PointClassification::OnEdge`] / [`PointClassification::OnFace`] if the
/// query point itself coincides with the boundary, else
/// [`PointClassification::Inside`]/[`PointClassification::Outside`] via ray
/// casting.
///
/// The ray direction is drawn from a seeded PRNG (see [`Rng`]) and retried
/// (up to a bounded number of attempts, see [`cast_ray`]) until one resolves
/// cleanly. The even/odd parity of that crossing count then determines
/// inside/outside, with no dependence on any face's normal orientation
/// (faces in this crate aren't guaranteed to wind consistently).
///
/// `max_nodes` bounds both the BFS containment searches and the DFS
/// intersection searches; `epsilon` is the shared geometric tolerance;
/// `seed` seeds the direction PRNG.
pub fn shell_contains<S: Scalar>(
    model: &Model<S>,
    shell_id: ShellId,
    point: Vector3<S>,
    max_nodes: usize,
    epsilon: S,
    seed: u64,
) -> GeopResult<PointClassification> {
    let shell = &model.shells[&shell_id];
    let (vertex_ids, edge_ids) = shell_vertices_and_edges(model, shell_id);

    // Coincidence pre-check: vertex, then edge, then face — each a full
    // pass, so higher-priority coincidences are deterministic regardless of
    // iteration order.
    for &vertex_id in &vertex_ids {
        if model.vertices[&vertex_id].point.could_be_equal(&point) {
            return Ok(PointClassification::OnVertex);
        }
    }
    for &edge_id in &edge_ids {
        if curve_could_contain(&model.edges[&edge_id].curve, &point, max_nodes, epsilon)?.is_some()
        {
            return Ok(PointClassification::OnEdge);
        }
    }
    for &face_id in &shell.faces {
        // Proximity to the face's *untrimmed* supporting surface is not
        // membership (see AGENTS.md) — a face's surface generally extends
        // well past its own trim loop (e.g. two faces split from one
        // larger one still share that one surface), so a point can sit
        // squarely on the surface while lying nowhere near this face's
        // actual boundary. Require `face_contains` to agree the point is
        // actually within the trim (or on it) before calling it a match —
        // exactly the check `shell_normal_at` already makes for the same
        // reason.
        let Some((u, v)) =
            surface_could_contain(&model.faces[&face_id].surface, &point, max_nodes, epsilon)?
        else {
            continue;
        };
        if !matches!(
            face_contains(model, face_id, u, v, max_nodes, epsilon, seed ^ face_id.0)?,
            FaceClassification::Outside
        ) {
            return Ok(PointClassification::OnFace);
        }
    }

    let ray_length = ray_length_for(model, &vertex_ids, &point)?;

    let mut rng = Rng::new(seed);
    let mut last_rejection = String::new();
    for attempt in 0..MAX_RAY_ATTEMPTS {
        let direction = rng.next_direction3::<S>();
        let attempt_seed = seed ^ (attempt as u64).wrapping_mul(0x9E3779B97F4A7C15);
        match cast_ray(
            model,
            shell_id,
            point,
            direction,
            &vertex_ids,
            &edge_ids,
            ray_length,
            max_nodes,
            epsilon,
            attempt_seed,
        )? {
            Ok(classification) => return Ok(classification),
            Err(reason) => {
                last_rejection = format!("the ray along {direction:?} was rejected: {reason}")
            }
        }
    }
    Err(GeopError::new(format!(
        "shell_contains: could not find a ray direction clear of every vertex and edge after many \
         attempts; the last one was rejected because {last_rejection}"
    )))
}

#[cfg(test)]
mod tests {
    use super::{PointClassification, shell_contains};
    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, Face, FaceId, Model, Sense, Shell, ShellId,
        SolidId, Vertex, VertexId, boundary::BoundaryType,
    };
    use geop_core_geometry::{
        nurb_curve::{NurbCurve, NurbCurve2D, NurbCurve3D},
        nurb_surface::NurbSurface3D,
    };
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    const MAX: usize = 200;
    const EPS: f64 = 1e-3;
    const SEED: u64 = 424_242;

    type P3 = (f64, f64, f64);

    /// A single planar quad face, corners given in trim-loop order
    /// (`p00 -> p10 -> p11 -> p01`), each face independently vertexed/edged
    /// (geometrically watertight, topologically not shared — fine for pure
    /// containment testing).
    fn quad_face<S: Scalar>(
        model: &mut Model<S>,
        shell: ShellId,
        p00: P3,
        p10: P3,
        p11: P3,
        p01: P3,
    ) -> FaceId {
        let p4 = |p: P3| {
            Vector4::from_array([S::from_f64(p.0), S::from_f64(p.1), S::from_f64(p.2), S::ONE])
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p4(p00), p4(p01), p4(p10), p4(p11)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        let face_id = model.insert_face(Face {
            surface,
            outer: BoundaryType::Vertex(crate::VertexId(0)),
            holes: Vec::new(),
            shell,
        });

        let corners = [p00, p10, p11, p01];
        let verts: Vec<VertexId> = corners
            .iter()
            .map(|&(x, y, z)| {
                model.insert_vertex(Vertex {
                    point: Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)]),
                })
            })
            .collect();
        let edges: Vec<EdgeId> = (0..4)
            .map(|i| {
                model.insert_edge(Edge {
                    curve: NurbCurve3D::try_new(
                        1,
                        vec![p4(corners[i]), p4(corners[(i + 1) % 4])],
                        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                    )
                    .unwrap(),
                    start_vertex: verts[i],
                    end_vertex: verts[(i + 1) % 4],
                })
            })
            .collect();
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
        let coedges: Vec<CoedgeId> = (0..4)
            .map(|i| {
                model.insert_coedge(Coedge {
                    geometry: CoedgeGeometry::Edge(edges[i]),
                    sense: Sense::Forward,
                    pcurve: pc(uv[i], uv[(i + 1) % 4]),
                    next: CoedgeId(0),
                    prev: CoedgeId(0),
                    face: face_id,
                })
            })
            .collect();
        for i in 0..4 {
            model.coedges.get_mut(&coedges[i]).unwrap().next = coedges[(i + 1) % 4];
            model.coedges.get_mut(&coedges[i]).unwrap().prev = coedges[(i + 3) % 4];
        }
        model.faces.get_mut(&face_id).unwrap().outer = BoundaryType::Loop(coedges[0]);

        face_id
    }

    /// The unit cube `[0,1]^3`, as six independently-vertexed quad faces
    /// (mixed winding across faces on purpose — containment must not depend
    /// on consistent orientation).
    fn unit_cube<S: Scalar>(model: &mut Model<S>) -> ShellId {
        let shell_id = model.insert_shell(Shell {
            faces: vec![],
            solid: SolidId(999),
        });
        let faces = vec![
            quad_face(
                model,
                shell_id,
                (0., 0., 1.),
                (1., 0., 1.),
                (1., 1., 1.),
                (0., 1., 1.),
            ), // +Z
            quad_face(
                model,
                shell_id,
                (0., 0., 0.),
                (0., 1., 0.),
                (1., 1., 0.),
                (1., 0., 0.),
            ), // -Z (reversed winding)
            quad_face(
                model,
                shell_id,
                (1., 0., 0.),
                (1., 1., 0.),
                (1., 1., 1.),
                (1., 0., 1.),
            ), // +X
            quad_face(
                model,
                shell_id,
                (0., 0., 0.),
                (0., 0., 1.),
                (0., 1., 1.),
                (0., 1., 0.),
            ), // -X (reversed winding)
            quad_face(
                model,
                shell_id,
                (0., 1., 0.),
                (1., 1., 0.),
                (1., 1., 1.),
                (0., 1., 1.),
            ), // +Y
            quad_face(
                model,
                shell_id,
                (0., 0., 0.),
                (1., 0., 0.),
                (1., 0., 1.),
                (0., 0., 1.),
            ), // -Y
        ];
        model.shells.get_mut(&shell_id).unwrap().faces = faces;
        shell_id
    }

    fn check_center_is_inside<S: Scalar>() {
        let mut model = Model::<S>::new();
        let shell_id = unit_cube(&mut model);
        let p = Vector3::from_array([S::from_f64(0.5); 3]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::Inside
        );
    }
    #[test]
    fn center_is_inside() {
        for_all_scalars!(check_center_is_inside);
    }

    fn check_far_point_is_outside<S: Scalar>() {
        let mut model = Model::<S>::new();
        let shell_id = unit_cube(&mut model);
        let p = Vector3::from_array([S::from_f64(-5.0), S::from_f64(0.5), S::from_f64(0.5)]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::Outside
        );
    }
    #[test]
    fn far_point_is_outside() {
        for_all_scalars!(check_far_point_is_outside);
    }

    fn check_point_just_outside_face_is_outside<S: Scalar>() {
        let mut model = Model::<S>::new();
        let shell_id = unit_cube(&mut model);
        let p = Vector3::from_array([S::from_f64(-0.1), S::from_f64(0.5), S::from_f64(0.5)]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::Outside
        );
    }
    #[test]
    fn point_just_outside_face_is_outside() {
        for_all_scalars!(check_point_just_outside_face_is_outside);
    }

    fn check_face_point_is_on_face<S: Scalar>() {
        let mut model = Model::<S>::new();
        let shell_id = unit_cube(&mut model);
        let p = Vector3::from_array([S::ZERO, S::from_f64(0.5), S::from_f64(0.5)]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::OnFace
        );
    }
    #[test]
    fn face_point_is_on_face() {
        for_all_scalars!(check_face_point_is_on_face);
    }

    fn check_edge_point_is_on_edge<S: Scalar>() {
        let mut model = Model::<S>::new();
        let shell_id = unit_cube(&mut model);
        let p = Vector3::from_array([S::ZERO, S::ZERO, S::from_f64(0.5)]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::OnEdge
        );
    }
    #[test]
    fn edge_point_is_on_edge() {
        for_all_scalars!(check_edge_point_is_on_edge);
    }

    fn check_vertex_point_is_on_vertex<S: Scalar>() {
        let mut model = Model::<S>::new();
        let shell_id = unit_cube(&mut model);
        let p = Vector3::from_array([S::ZERO, S::ZERO, S::ZERO]);
        assert_eq!(
            shell_contains(&model, shell_id, p, MAX, S::from_f64(EPS), SEED).unwrap(),
            PointClassification::OnVertex
        );
    }
    #[test]
    fn vertex_point_is_on_vertex() {
        for_all_scalars!(check_vertex_point_is_on_vertex);
    }
}
