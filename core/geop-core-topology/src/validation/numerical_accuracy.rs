use crate::{Model, validation::ValidationParameters};
use geop_core_math::{geop_error::GeopError, scalars::Scalar};

/// The widest a single coordinate of any stored entity may be.
///
/// Not a comparison tolerance — nothing is ever tested for equality against
/// it. It is a bound on how much uncertainty an entity is allowed to *carry*,
/// and every search and comparison downstream assumes something like it: a
/// containment search resolves to `min_subdivision_size` (also `1e-4`), so a
/// vertex already wider than that cannot be located on anything, and a
/// control point that wide makes a curve whose convex hull no longer pins the
/// curve down. Past this point the arithmetic stops meaning what the
/// algorithms assume it means.
const MAX_WIDTH: f64 = 1e-4;

/// Checks that no stored entity carries more than [`MAX_WIDTH`] of numerical
/// uncertainty in any coordinate: every vertex position, every edge curve
/// control point, every face surface control point.
///
/// Runs before every other check, because it explains them. A too-wide entity
/// does not fail in place — it fails somewhere downstream, as a containment
/// search that finds nothing, a pcurve that will not match its edge, or an
/// intersection that is missed entirely, and the report names that distant
/// symptom rather than the cause.
///
/// A failure here almost always means a **missing refinement**: some operation
/// returned a subdivision search's raw enclosure (as wide as its tolerance)
/// and stored it, where it should have polished it with Newton first — see
/// `NurbCurve::refine_parameter_at_point` and
/// `intersection::curve_surface::refine_crossing`. Widening is monotone
/// through arithmetic, so the first entity to exceed the bound is close to
/// wherever that refinement was skipped.
///
/// The same bound doubles as a *minimum* on how long an edge may be. An edge
/// shorter than the accuracy its own endpoints carry is not a feature of the
/// model, it is noise: nothing can be located along it, the searches cannot
/// tell its two ends apart, and splicing anything into a face across it
/// produces a region of no area. Such an edge always means an operation split
/// something it should have recognised as already coincident.
///
/// The placeholder surface a face carries before it is given real geometry
/// (`NurbSurface::everything`, every coordinate `ENTIRE`) is deliberately not
/// exempted: a finished model must not contain one, and reporting it here as
/// an unbounded width is exactly right.
pub fn check_numerical_accuracy<S: Scalar>(
    _params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    let limit = S::from_f64(MAX_WIDTH);
    // `width` is sharp by construction, and so is `limit`, so this comparison
    // is always decidable — which is the whole reason the bound is expressed
    // as a width rather than as a comparison between two uncertain values.
    let too_wide = |x: S| x.width().definitely_greater(limit);

    for (&vertex_id, vertex) in &model.vertices {
        for c in 0..3 {
            if too_wide(vertex.point[c]) {
                errors.push(GeopError::new(format!(
                    "vertex {vertex_id}'s coordinate {c} is {:?}, {:?} wide — wider than the {MAX_WIDTH:e} every search downstream assumes; something that produced it skipped a refinement",
                    vertex.point[c],
                    vertex.point[c].width()
                )));
            }
        }
    }

    for (&edge_id, edge) in &model.edges {
        for (i, p) in edge.curve.control_points.iter().enumerate() {
            for c in 0..4 {
                if too_wide(p[c]) {
                    errors.push(GeopError::new(format!(
                        "edge {edge_id}'s curve control point {i}, component {c} is {:?}, {:?} wide — wider than the {MAX_WIDTH:e} every search downstream assumes; something that produced it skipped a refinement",
                        p[c],
                        p[c].width()
                    )));
                }
            }
        }

        let (t0, t1) = edge.curve.domain();
        let (Ok(start), Ok(end)) = (edge.curve.evaluate(t0), edge.curve.evaluate(t1)) else {
            continue;
        };
        // Chord, not arc length: a curve whose two ends are closer together
        // than this is degenerate however it wanders in between, and the
        // chord is exactly what a subdivision search's convergence test sees.
        let chord = end.sub(&start).norm();
        if limit.definitely_greater(chord) {
            errors.push(GeopError::new(format!(
                "edge {edge_id} runs from {start:?} to {end:?}, a chord of {chord:?} — shorter than the {MAX_WIDTH:e} the searches can resolve, so it is noise rather than geometry; whatever produced it split something it should have recognised as already coincident"
            )));
        }
    }

    for (&face_id, face) in &model.faces {
        for (i, p) in face.surface.control_points.iter().enumerate() {
            for c in 0..4 {
                if too_wide(p[c]) {
                    errors.push(GeopError::new(format!(
                        "face {face_id}'s surface control point {i}, component {c} is {:?}, {:?} wide — wider than the {MAX_WIDTH:e} every search downstream assumes; something that produced it skipped a refinement",
                        p[c],
                        p[c].width()
                    )));
                }
            }
        }
    }
}
