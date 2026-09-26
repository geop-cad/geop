use geop_core_math::{geop_error::GeopError, scalars::Scalar};

use crate::{Model, validation::ValidationParameters};

/// Checks that every coedge's own pcurve, evaluated at its end, lands
/// exactly where `coedge.next`'s pcurve starts — i.e. that a face's
/// boundary loop is genuinely closed in the face's own `(u, v)` parameter
/// space, not just in 3-D.
///
/// [`check_curves_and_surfaces_vertices`](super::check_curves_and_surfaces_vertices)
/// already checks that every individual coedge's pcurve lands on its own
/// edge's real 3-D endpoints — but two *consecutive* coedges can each pass
/// that independently while still leaving a gap between them in `(u, v)`
/// space: whenever a row of a surface collapses to a single vertex (e.g. a
/// pole), the whole iso-line at that row maps to the same 3-D point
/// regardless of `u`, so a coedge ending at `(1, 0)` and the next one
/// starting at `(0, 0)` can both be individually valid (both map to that
/// same pole) while the loop, walked purely in parameter space, jumps
/// straight from one `u` to the other without ever tracing the row between
/// them. That's invisible to any single-coedge check, but fatal to
/// `contains::face::face_contains`'s ray-casting, which relies on the
/// coedges' pcurves alone tracing a fully closed 2-D polygon — a ray that
/// happens to cross exactly through such a gap passes through uncounted,
/// corrupting the inside/outside parity for query points near it.
pub fn check_pcurve_loop_continuity<S: Scalar>(
    _params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    for (&coedge_id, coedge) in &model.coedges {
        let Some(next) = model.coedges.get(&coedge.next) else {
            continue;
        };
        let (Ok(end), Ok(next_start)) = (
            coedge.pcurve.evaluate(coedge.pcurve.domain().1),
            next.pcurve.evaluate(next.pcurve.domain().0),
        ) else {
            continue;
        };
        if !end.could_be_equal(&next_start) {
            errors.push(GeopError::new(format!(
                "coedge {}'s pcurve ends at {:?}, but its next coedge {}'s pcurve starts at {:?} \
                 — the boundary loop isn't closed in (u, v) parameter space",
                coedge_id.0, end, coedge.next.0, next_start
            )));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::check_pcurve_loop_continuity;
    use crate::{
        Model,
        test_fixtures::{line2, test_cube_solid},
        validation::ValidationParameters,
    };
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector2};

    fn run<S: Scalar>(model: &Model<S>) -> Vec<geop_core_math::geop_error::GeopError> {
        let mut errors = Vec::new();
        check_pcurve_loop_continuity(&ValidationParameters::default(), &mut errors, model);
        errors
    }

    fn check_valid_cube_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        assert!(run(&model).is_empty());
    }
    #[test]
    fn valid_cube_passes() {
        for_all_scalars!(check_valid_cube_passes);
    }

    fn check_broken_pcurve_gap_is_caught<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let coedge_id = *model.coedges.keys().next().unwrap();
        model.coedges.get_mut(&coedge_id).unwrap().pcurve = line2(
            Vector2::from_array([S::from_f64(0.1), S::from_f64(0.1)]),
            Vector2::from_array([S::from_f64(0.9), S::from_f64(0.9)]),
        );
        assert!(!run(&model).is_empty());
    }
    #[test]
    fn broken_pcurve_gap_is_caught() {
        for_all_scalars!(check_broken_pcurve_gap_is_caught);
    }
}
