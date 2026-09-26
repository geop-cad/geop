use std::collections::HashSet;

use geop_core_math::{geop_error::GeopError, scalars::Scalar};

use crate::{
    CoedgeGeometry, FaceId, Model,
    contains::{
        face::{PointClassification, face_contains},
        rng::Rng,
    },
    validation::ValidationParameters,
};

/// Every pair of faces that already touch in the model — sharing an edge
/// (two coedges of the same edge, one per face) or, more loosely, just a
/// vertex (e.g. diagonal quadrants of a revolved solid's pole, which share
/// no edge but do meet exactly at the pole point). Such faces' underlying
/// surfaces generically only agree along the *infinite* extension of that
/// shared edge's line/curve (or, for a shared vertex alone, somewhere in
/// the vicinity of it) — the alternating-projection search below has no
/// notion of the surfaces' trims while it iterates, so it can easily wander
/// to, and converge on, a point on that extension beyond the actual shared
/// feature: a point that is genuinely outside one face's trim, but not
/// because the faces improperly interpenetrate anywhere — just an artifact
/// of already-expected adjacency. So these pairs are skipped; shared edges
/// are already validated elsewhere (`disjointness_check`, `manifold_check`).
fn adjacent_face_pairs<S: Scalar>(model: &Model<S>) -> HashSet<(FaceId, FaceId)> {
    let mut pairs = HashSet::new();
    let mut mark_all = |faces: &[FaceId]| {
        for i in 0..faces.len() {
            for j in (i + 1)..faces.len() {
                let (a, b) = (faces[i], faces[j]);
                pairs.insert(if a.0 <= b.0 { (a, b) } else { (b, a) });
            }
        }
    };

    for &edge_id in model.edges.keys() {
        let faces: Vec<FaceId> = model
            .coedges_of_edge(edge_id)
            .iter()
            .map(|&coedge_id| model.coedges[&coedge_id].face)
            .collect();
        mark_all(&faces);
    }
    for &vertex_id in model.vertices.keys() {
        let faces: Vec<FaceId> = model
            .coedges
            .values()
            .filter(|c| match c.geometry {
                CoedgeGeometry::Edge(edge_id) => {
                    let edge = &model.edges[&edge_id];
                    edge.start_vertex == vertex_id || edge.end_vertex == vertex_id
                }
                CoedgeGeometry::Vertex(v) => v == vertex_id,
            })
            .map(|c| c.face)
            .collect();
        mark_all(&faces);
    }
    pairs
}

/// For every face pair, tries `params.face_face_sample_count` random
/// (point-on-face-a, point-on-face-b) starting pairs and runs them through
/// alternating-projection Newton iteration (project face a's point onto
/// face b, then face b's new point back onto face a, repeat) to look for an
/// actual crossing point. A pair that converges has surfaces that meet
/// somewhere; that alone is unremarkable — two coplanar faces of one solid
/// share their entire plane. The violation is a converged point that lies
/// strictly *inside both* faces' trims: two faces sharing no edge or vertex
/// yet occupying the same place, which means their intersection is missing
/// from the topology.
///
/// Each round's result is sharpened (`Scalar::midpoint()`) before feeding
/// the next round, so numerical error doesn't compound over many
/// iterations — except the very last round, which is left unsharpened: the
/// convergence check right after needs that residual imprecision, or an
/// otherwise-genuine match can round to just outside `could_be_equal`'s
/// tolerance.
pub fn check_face_face_numerical_intersection<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    let face_ids: Vec<FaceId> = model.faces.keys().copied().collect();
    let adjacent = adjacent_face_pairs(model);

    for i in 0..face_ids.len() {
        for j in (i + 1)..face_ids.len() {
            let face_a_id = face_ids[i];
            let face_b_id = face_ids[j];
            let normalized = if face_a_id.0 <= face_b_id.0 {
                (face_a_id, face_b_id)
            } else {
                (face_b_id, face_a_id)
            };
            if adjacent.contains(&normalized) {
                continue;
            }
            let face_a = &model.faces[&face_a_id];
            let face_b = &model.faces[&face_b_id];

            let (au_lo, au_hi) = face_a.surface.domain_u();
            let (av_lo, av_hi) = face_a.surface.domain_v();
            let (bu_lo, bu_hi) = face_b.surface.domain_u();
            let (bv_lo, bv_hi) = face_b.surface.domain_v();

            let pair_seed = face_a_id.0 ^ face_b_id.0.wrapping_mul(0x9E3779B97F4A7C15);
            let mut rng = Rng::new(pair_seed);

            for sample in 0..params.face_face_sample_count {
                let mut u1 = S::from_f64(rng.next_range(au_lo.to_f64(), au_hi.to_f64()));
                let mut v1 = S::from_f64(rng.next_range(av_lo.to_f64(), av_hi.to_f64()));
                let mut u2 = S::from_f64(rng.next_range(bu_lo.to_f64(), bu_hi.to_f64()));
                let mut v2 = S::from_f64(rng.next_range(bv_lo.to_f64(), bv_hi.to_f64()));

                for round in 0..params.face_face_newton_iterations {
                    let Ok(p1) = face_a.surface.evaluate(u1, v1) else {
                        break;
                    };
                    let Ok((nu2, nv2)) = face_b.surface.project(p1, u2, v2, 1) else {
                        break;
                    };
                    u2 = nu2;
                    v2 = nv2;

                    let Ok(p2) = face_b.surface.evaluate(u2, v2) else {
                        break;
                    };
                    let Ok((nu1, nv1)) = face_a.surface.project(p2, u1, v1, 1) else {
                        break;
                    };
                    u1 = nu1;
                    v1 = nv1;

                    // Sharpen before the next round — except the last one,
                    // whose (still-uncertain) result feeds the convergence
                    // check right below the loop.
                    if round + 1 < params.face_face_newton_iterations {
                        u1 = u1.midpoint();
                        v1 = v1.midpoint();
                        u2 = u2.midpoint();
                        v2 = v2.midpoint();
                    }
                }

                let (Ok(p1), Ok(p2)) = (
                    face_a.surface.evaluate(u1, v1),
                    face_b.surface.evaluate(u2, v2),
                ) else {
                    continue;
                };
                if !p1.could_be_equal(&p2) {
                    // Didn't converge to a shared point from this starting
                    // pair — no claim to check (most face pairs, e.g.
                    // opposite faces of a box, genuinely never meet).
                    continue;
                }

                // The converged point is where the two *untrimmed* surfaces
                // agree. That on its own says nothing: two coplanar faces of
                // one solid share their whole plane, and every projection
                // converges somewhere on it. What matters is whether the
                // point falls inside both faces' *trims* — that is the two
                // faces occupying the same place, which for a pair that
                // shares no edge or vertex is improper interpenetration.
                //
                // Only a strict `Inside` on both counts. A point classified
                // `OnCoedge`/`OnVertex` sits on a boundary, where the
                // classification is genuinely ambiguous numerically and where
                // faces are allowed to touch; requiring more than this turns
                // every tangential contact into a failure.
                let sample_seed = pair_seed ^ (sample as u64).wrapping_mul(0x2545_F491_4F6C_DD1D);
                let mut inside_both = true;
                for (face_id, u, v) in [(face_a_id, u1, v1), (face_b_id, u2, v2)] {
                    match face_contains(
                        model,
                        face_id,
                        u,
                        v,
                        params.max_nodes,
                        params.min_subdivision_size,
                        sample_seed,
                    ) {
                        Ok(PointClassification::Inside) => {}
                        Ok(_) => inside_both = false,
                        Err(e) => {
                            inside_both = false;
                            errors.push(e.with_context(format!(
                                "face {} x face {}, checking whether the converged point is on face {}",
                                face_a_id.0, face_b_id.0, face_id.0
                            )));
                        }
                    }
                }
                if inside_both {
                    errors.push(GeopError::new(format!(
                        "face {} (shell {}) and face {} (shell {}) share no edge or vertex, yet both contain the point {p1:?} strictly inside their trims (uv=({u1:?}, {v1:?}) and ({u2:?}, {v2:?})) — the two faces occupy the same place, so their intersection is missing from the topology",
                        face_a_id.0, face_a.shell, face_b_id.0, face_b.shell
                    )));
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::check_face_face_numerical_intersection;
    use crate::test_fixtures::test_cube_solid;
    use crate::{
        Coedge, CoedgeGeometry, CoedgeId, Face, FaceId, Model, Sense, ShellId, Vertex,
        boundary::BoundaryType, validation::ValidationParameters,
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

    fn run<S: Scalar>(model: &Model<S>) -> Vec<geop_core_math::geop_error::GeopError> {
        let mut errors = Vec::new();
        check_face_face_numerical_intersection(
            &ValidationParameters::default(),
            &mut errors,
            model,
        );
        errors
    }

    fn check_welded_cube_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        assert!(run(&model).is_empty());
    }
    #[test]
    fn welded_cube_passes() {
        for_all_scalars!(check_welded_cube_passes);
    }

    /// A face trimmed to the quad `(u,v)` corners `q0 -> q1 -> q2 -> q3`, on
    /// a shared big planar surface spanning physical `(0,0,0)..(2,2,0)`
    /// (`(u,v) == (x/2, y/2)`).
    fn quarter_face<S: Scalar>(
        model: &mut Model<S>,
        q0: (f64, f64),
        q1: (f64, f64),
        q2: (f64, f64),
        q3: (f64, f64),
    ) -> FaceId {
        let p =
            |x: f64, y: f64| Vector4::from_array([S::from_f64(x), S::from_f64(y), S::ZERO, S::ONE]);
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p(0.0, 0.0), p(0.0, 2.0), p(2.0, 0.0), p(2.0, 2.0)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        let face_id = model.insert_face(Face {
            surface,
            outer: BoundaryType::Vertex(crate::VertexId(0)),
            holes: Vec::new(),
            shell: ShellId(999),
        });

        let corners = [q0, q1, q2, q3];
        let verts: Vec<_> = corners
            .iter()
            .map(|&(u, v)| {
                model.insert_vertex(Vertex {
                    point: Vector3::from_array([
                        S::from_f64(u * 2.0),
                        S::from_f64(v * 2.0),
                        S::ZERO,
                    ]),
                })
            })
            .collect();
        let edges: Vec<_> = (0..4)
            .map(|i| {
                let (u0, v0) = corners[i];
                let (u1, v1) = corners[(i + 1) % 4];
                model.insert_edge(crate::Edge {
                    curve: geop_core_geometry::nurb_curve::NurbCurve3D::try_new(
                        1,
                        vec![
                            Vector4::from_array([
                                S::from_f64(u0 * 2.0),
                                S::from_f64(v0 * 2.0),
                                S::ZERO,
                                S::ONE,
                            ]),
                            Vector4::from_array([
                                S::from_f64(u1 * 2.0),
                                S::from_f64(v1 * 2.0),
                                S::ZERO,
                                S::ONE,
                            ]),
                        ],
                        vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
                    )
                    .unwrap(),
                    start_vertex: verts[i],
                    end_vertex: verts[(i + 1) % 4],
                })
            })
            .collect();
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
        let coedges: Vec<_> = (0..4)
            .map(|i| {
                model.insert_coedge(Coedge {
                    geometry: CoedgeGeometry::Edge(edges[i]),
                    sense: Sense::Forward,
                    pcurve: pc(corners[i], corners[(i + 1) % 4]),
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

    fn check_coincident_planes_with_disjoint_trims_pass<S: Scalar>() {
        let mut model = Model::<S>::new();
        // Two quadrants of the very same underlying plane, diagonally
        // opposite. Their surfaces coincide exactly — every alternating
        // projection converges somewhere on the shared plane — but their
        // *trims* never overlap, so the two faces occupy no common point and
        // there is nothing to report. This is the ordinary state of any two
        // coplanar faces of one solid, e.g. a figure-8's cap either side of
        // its neck; treating it as a violation reported hundreds of false
        // errors on exactly those models.
        quarter_face(&mut model, (0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5));
        quarter_face(&mut model, (0.5, 0.5), (1.0, 0.5), (1.0, 1.0), (0.5, 1.0));

        assert!(run(&model).is_empty(), "{:?}", run(&model));
    }
    fn check_coincident_planes_with_identical_trims_fail<S: Scalar>() {
        let mut model = Model::<S>::new();
        // The same quadrant twice: two faces sharing no edge or vertex yet
        // occupying exactly the same place. That is the violation — their
        // intersection is a whole region, and nothing in the topology
        // records it.
        quarter_face(&mut model, (0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5));
        quarter_face(&mut model, (0.0, 0.0), (0.5, 0.0), (0.5, 0.5), (0.0, 0.5));

        assert!(!run(&model).is_empty());
    }
    #[test]
    fn coincident_planes_with_identical_trims_fail() {
        for_all_scalars!(check_coincident_planes_with_identical_trims_fail);
    }

    #[test]
    fn coincident_planes_with_disjoint_trims_pass() {
        for_all_scalars!(check_coincident_planes_with_disjoint_trims_pass);
    }
}
