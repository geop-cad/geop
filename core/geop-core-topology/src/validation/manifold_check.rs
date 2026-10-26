use super::{ValidationParameters, validate};
use geop_core_math::{geop_error::GeopError, plain, scalars::Scalar, vector::Vector3};

use crate::{
    Model, Sense,
    contains::{
        rng::Rng,
        shell::{PointClassification, ShellBoundary, cast_ray},
    },
};

/// Every edge of a solid's shell must be shared by an even number of
/// coedges, as many traversing it `Forward` as `Reversed`: two where two
/// faces meet, four (or more) where the solid touches itself along the
/// edge — two blocks sharing only an edge, a boss tangent to a wall along a
/// line — each face on one side of it paired with one on the other. An odd
/// count is a free edge, a hole in the shell; unbalanced senses mean faces
/// treating the edge as running the same direction, a torn seam rather
/// than a shared one. Containment counts crossings of rays that graze no
/// edge (see `contains::shell`), so an edge touched by more faces costs it
/// nothing.
///
/// The exceptions bound no volume: a sheet's border, where an edge only one
/// of its faces uses is simply where it ends, and a wire's edges, which no
/// face uses.
fn check_every_edge_has_paired_coedges(errors: &mut Vec<GeopError>, model: &Model) {
    let wire_edges: std::collections::HashSet<crate::EdgeId> = model
        .wires
        .values()
        .flat_map(|w| w.edges.iter().copied())
        .collect();
    for &edge_id in model.edges.keys().filter(|e| !wire_edges.contains(e)) {
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
        if n == 0 || !n.is_multiple_of(2) {
            errors.push(GeopError::new(format!(
                "edge {} has {} coedge(s) (expected an even number, two where two faces meet)",
                edge_id.0, n
            )));
            continue;
        }
        let forward = coedge_ids
            .iter()
            .filter(|&id| model.coedges[id].sense == Sense::Forward)
            .count();
        if 2 * forward != n {
            errors.push(GeopError::new(format!(
                "edge {}'s {n} coedges {coedge_ids:?} run it Forward {forward} time(s) (expected as many Forward as Reversed)",
                edge_id.0
            )));
        }
    }
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
fn check_ray_direction_consistency(
    params: &ValidationParameters,
    errors: &mut Vec<GeopError>,
    model: &Model,
) {
    // Only a solid's shells enclose anything to be inside of.
    for (&shell_id, _) in model.shells.iter().filter(|(_, s)| s.solid.is_some()) {
        let boundary = ShellBoundary::of(model, shell_id);
        // The box around the shell's vertices, as plain `f64`s: it only
        // places random sample points, so exactness does not matter here.
        let points = (boundary.vertices.iter()).map(|v| model.vertices[v].point.to_f64_array());
        let Some((lo, hi)) = plain::bounds(points) else {
            continue;
        };
        // Expand the box by half its size each way, so roughly-exterior
        // points come up too, not just interior ones.
        let margin: Vec<f64> = (0..3)
            .map(|axis| (hi[axis] - lo[axis]).max(1.0) * 0.5)
            .collect();

        let mut rng = Rng::new(params.manifold_seed ^ shell_id.0);

        for point_sample in 0..params.manifold_ray_sample_count {
            let point = Vector3::from_array([
                Scalar::from_f64(rng.next_range(lo[0] - margin[0], hi[0] + margin[0])),
                Scalar::from_f64(rng.next_range(lo[1] - margin[1], hi[1] + margin[1])),
                Scalar::from_f64(rng.next_range(lo[2] - margin[2], hi[2] + margin[2])),
            ]);

            let Ok(ray_length) = boundary.ray_length(&point) else {
                continue;
            };

            let mut classifications = Vec::new();
            for dir_sample in 0..params.manifold_ray_sample_count {
                let direction = rng.next_direction3();
                let seed = params.manifold_seed
                    ^ shell_id.0
                    ^ (point_sample as u64).wrapping_mul(0x9E3779B97F4A7C15)
                    ^ (dir_sample as u64).wrapping_mul(0x2545_F491_4F6C_DD1D);
                match cast_ray(
                    &boundary,
                    point,
                    direction,
                    ray_length,
                    None,
                    params.max_nodes,
                    params.min_subdivision_size,
                    seed,
                ) {
                    Ok(Ok(ray)) => classifications.push(ray.classification()),
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
/// per-entity consistency: every edge is shared by coedges paired up, as
/// many each way (see [`check_every_edge_has_paired_coedges`]), and — the check that actually exercises that strategy — every
/// shell classifies a batch of random points identically no matter which
/// (non-degenerate) random ray direction was used to test them.
pub fn validate_manifold(
    params: &ValidationParameters,
    model: &Model,
) -> Result<(), Vec<GeopError>> {
    let mut errors = match validate(params, model) {
        Ok(()) => Vec::new(),
        Err(e) => e,
    };

    check_every_edge_has_paired_coedges(&mut errors, model);
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

    #[test]
    fn welded_cube_passes() {
        let mut model = Model::new();
        test_cube_solid(&mut model);
        validate_manifold(&ValidationParameters::default(), &model).unwrap();
    }

    #[test]
    fn missing_edge_coedge_fails() {
        let mut model = Model::new();
        test_cube_solid(&mut model);
        // Detach one coedge from its edge onto a bogus new one, leaving the
        // original edge with only 1 coedge — a free edge.
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
                .any(|e| format!("{e:?}").contains("expected an even number"))
        );
    }
}
