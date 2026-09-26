use crate::{Model, contains::face::face_interior_point, validation::ValidationParameters};
use geop_core_math::{geop_error::GeopError, scalars::Scalar};

/// Fixed seed for the ray casting behind [`face_interior_point`]. It retries
/// until it finds a ray grazing nothing, so the answer is seed-independent
/// and a constant keeps validation reproducible run to run.
const SEED: u64 = 0x1A7E_1D0F_0000_0001;

/// Checks that every face has a point strictly inside its trimmed region.
///
/// A face that encloses no area is *structurally* perfect — its loop closes,
/// its pcurves are continuous, every coedge points where it should — so
/// every other check in `validate_fast` accepts it. But it is not a face:
/// there is nowhere on it to evaluate a normal, classify against another
/// solid, or place anything at all. The failure surfaces much later and far
/// from its cause, as a boolean unable to classify a face it was handed.
///
/// This is deliberately in `validate_fast` rather than the full `validate`,
/// even though it casts rays: a degenerate face is a *construction* defect,
/// and the operations that create them (a face split along an edge that
/// already coincides with a boundary) are exactly what the fast sweep runs
/// after. Catching it here means the sweep that produces one also reports it.
pub fn check_faces_have_interior<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    for &face_id in model.faces.keys() {
        if let Err(e) = face_interior_point(
            model,
            face_id,
            params.max_nodes,
            params.min_subdivision_size,
            SEED,
        ) {
            errors.push(e.with_context(format!(
                "face {face_id} has no interior point, so it bounds no material and nothing can be classified against it"
            )));
        }
    }
}
