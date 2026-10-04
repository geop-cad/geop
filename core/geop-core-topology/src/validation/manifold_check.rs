use super::{ValidationParameters, validate};
use geop_core_math::{geop_error::GeopError, scalars::Scalar, vector::Vector3};

use crate::{
    Model, Sense, VertexId,
    contains::{
        rng::Rng,
        shell::{PointClassification, cast_ray, ray_length_for, shell_vertices_and_edges},
    },
};

/// Every edge of a 2-manifold shell must be shared by exactly two coedges
/// (one per adjoining face), one traversing it `Forward` and the other
/// `Reversed` — a free (1-coedge) or non-manifold (3+ coedge) edge breaks
/// the "shoot a ray and count crossings" containment strategy this whole
/// `contains` module relies on, and two coedges of the *same* sense would
/// mean both adjoining faces treat the edge as running the same direction —
/// a torn (rather than shared) seam, not a genuinely closed manifold edge.
///
/// The one exception is a sheet's border: a sheet bounds nothing, so an
/// edge only one of its faces uses is simply where it ends.
fn check_every_edge_has_two_coedges<S: Scalar>(errors: &mut Vec<GeopError>, model: &Model<S>) {
    for &edge_id in model.edges.keys() {
        let coedge_ids = model.coedges_of_edge(edge_id);
        let n = coedge_ids.len();
        let on_sheet = |coedge: &crate::CoedgeId| {
            model
                .body_of_face(model.coedges[coedge].face)
                .is_ok_and(|body| matches!(body, crate::Body::Sheet(_)))
        };
        if n == 1 && on_sheet(&coedge_ids[0]) {
            continue;
        }
        if n != 2 {
            errors.push(GeopError::new(format!(
                "edge {} has {} coedge(s) (expected exactly 2 for a manifold shell)",
                edge_id.0, n
            )));
            continue;
        }
        let senses: Vec<Sense> = coedge_ids
            .iter()
            .map(|id| model.coedges[id].sense)
            .collect();
        if senses[0] == senses[1] {
            errors.push(GeopError::new(format!(
                "edge {}'s two coedges ({}, {}) both have sense {:?} (expected one Forward and one Reversed)",
                edge_id.0, coedge_ids[0], coedge_ids[1], senses[0]
            )));
        }
    }
}

/// The axis-aligned bounding box of `vertex_ids`' positions, as plain
/// `f64`s (only used to pick random sample points, so exactness doesn't
/// matter here).
fn vertex_bounds<S: Scalar>(model: &Model<S>, vertex_ids: &[VertexId]) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for &vertex_id in vertex_ids {
        let p = model.vertices[&vertex_id].point;
        for axis in 0..3 {
            let x = p[axis].to_f64();
            lo[axis] = lo[axis].min(x);
            hi[axis] = hi[axis].max(x);
        }
    }
    (lo, hi)
}

/// For every shell, samples `params.manifold_ray_sample_count` random points
/// (drawn from a box around the shell, so both interior and exterior points
/// come up) and, for each, casts `params.manifold_ray_sample_count` random
/// ray directions (see `contains::shell::cast_ray`, the same per-direction
/// logic `shell_contains` itself retries on a degenerate hit) — a degenerate
/// direction is simply skipped (it gives no classification to compare).
///
/// A point near the shell's boundary can occasionally get one ray-cast
/// classification that disagrees with the rest, purely from numerical
/// fragility of the intersection search that close in — not a real topology
/// defect. So a single dissenting vote is tolerated; only genuine majority
/// disagreement (at least 2 votes on each side) is reported.
fn check_ray_direction_consistency<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    // Only a solid's shells enclose anything to be inside of.
    for (&shell_id, _) in model.shells.iter().filter(|(_, s)| s.solid.is_some()) {
        let (vertex_ids, edge_ids) = shell_vertices_and_edges(model, shell_id);
        if vertex_ids.is_empty() {
            continue;
        }
        let (lo, hi) = vertex_bounds(model, &vertex_ids);
        // Expand the box by half its size each way, so roughly-exterior
        // points come up too, not just interior ones.
        let margin: Vec<f64> = (0..3)
            .map(|axis| (hi[axis] - lo[axis]).max(1.0) * 0.5)
            .collect();

        let mut rng = Rng::new(params.manifold_seed ^ shell_id.0);

        for point_sample in 0..params.manifold_ray_sample_count {
            let point = Vector3::from_array([
                S::from_f64(rng.next_range(lo[0] - margin[0], hi[0] + margin[0])),
                S::from_f64(rng.next_range(lo[1] - margin[1], hi[1] + margin[1])),
                S::from_f64(rng.next_range(lo[2] - margin[2], hi[2] + margin[2])),
            ]);

            let Ok(ray_length) = ray_length_for(model, &vertex_ids, &point) else {
                continue;
            };

            let mut classifications = Vec::new();
            for dir_sample in 0..params.manifold_ray_sample_count {
                let direction = rng.next_direction3::<S>();
                let seed = params.manifold_seed
                    ^ shell_id.0
                    ^ (point_sample as u64).wrapping_mul(0x9E3779B97F4A7C15)
                    ^ (dir_sample as u64).wrapping_mul(0x2545_F491_4F6C_DD1D);
                match cast_ray(
                    model,
                    shell_id,
                    point,
                    direction,
                    &vertex_ids,
                    &edge_ids,
                    ray_length,
                    params.max_nodes,
                    params.min_subdivision_size,
                    seed,
                ) {
                    Ok(Ok(classification)) => classifications.push(classification),
                    // Degenerate direction (grazed a vertex/edge, or an
                    // ambiguous trim hit) — no claim to compare, skip.
                    Ok(Err(_)) => {}
                    Err(e) => errors.push(e.with_context(format!(
                        "shell {}, point {:?}, direction sample {}",
                        shell_id.0,
                        (point[0].to_f64(), point[1].to_f64(), point[2].to_f64()),
                        dir_sample
                    ))),
                }
            }

            if classifications.len() >= 2 {
                let inside_count = classifications
                    .iter()
                    .filter(|&&c| c == PointClassification::Inside)
                    .count();
                let outside_count = classifications.len() - inside_count;
                let minority = inside_count.min(outside_count);
                // A single dissenting vote is tolerated as numerical noise
                // near the boundary; only a genuine split (2+ votes on both
                // sides) is reported.
                if minority >= 2 {
                    errors.push(GeopError::new(format!(
                        "shell {}, point {:?}: ray directions disagree on inside/outside ({:?})",
                        shell_id.0,
                        (point[0].to_f64(), point[1].to_f64(), point[2].to_f64()),
                        classifications
                    )));
                }
            }
        }
    }
}

/// [`validate`] plus two extra whole-model sanity checks that go beyond
/// per-entity consistency: every edge is shared by exactly two coedges (a
/// precondition the whole `contains` module's ray-parity strategy relies
/// on), and — the check that actually exercises that strategy — every
/// shell classifies a batch of random points identically no matter which
/// (non-degenerate) random ray direction was used to test them.
pub fn validate_manifold<S: Scalar>(
    params: &ValidationParameters<S>,
    model: &Model<S>,
) -> Result<(), Vec<GeopError>> {
    let mut errors = match validate(params, model) {
        Ok(()) => Vec::new(),
        Err(e) => e,
    };

    check_every_edge_has_two_coedges(&mut errors, model);
    check_ray_direction_consistency(params, &mut errors, model);

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

#[cfg(test)]
mod tests {
    use super::validate_manifold;
    use crate::test_fixtures::test_cube_solid;
    use crate::{CoedgeGeometry, Model, validation::ValidationParameters};
    use geop_core_math::{for_all_scalars, scalars::Scalar};

    fn check_welded_cube_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        validate_manifold(&ValidationParameters::default(), &model).unwrap();
    }
    #[test]
    fn welded_cube_passes() {
        for_all_scalars!(check_welded_cube_passes);
    }

    fn check_missing_edge_coedge_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        // Detach one coedge from its edge onto a bogus new one, leaving the
        // original edge with only 1 coedge — non-manifold.
        let coedge_id = *model.coedges.keys().next().unwrap();
        let orphan_edge = {
            let template = model.edges[&model.coedges[&coedge_id].edge().unwrap()].clone();
            model.insert_edge(template)
        };
        model.coedges.get_mut(&coedge_id).unwrap().geometry = CoedgeGeometry::Edge(orphan_edge);

        let errors = validate_manifold(&ValidationParameters::default(), &model).unwrap_err();
        assert!(
            errors
                .iter()
                .any(|e| format!("{e:?}").contains("expected exactly 2"))
        );
    }
    #[test]
    fn missing_edge_coedge_fails() {
        for_all_scalars!(check_missing_edge_coedge_fails);
    }
}
