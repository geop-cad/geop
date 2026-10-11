//! Newton refinement of a root a subdivision search has isolated — shared
//! by `geop_core_geometry`'s `NurbCurve::refine_parameter_at_point` and the
//! intersection searches' `refine_crossing`s.
//!
//! Subdivision and Newton divide the work. Subdivision is the global
//! method: it reliably finds and separates every solution, and recognizes
//! coincidence even for a partial overlap, but converges only one bit per
//! split — squeezing a parameter down to machine accuracy would take ~50
//! levels against the ~7 that isolate it. Newton cannot find anything, but
//! polishes an isolated solution quadratically. So the searches subdivide
//! to isolate, and a caller about to *use* a parameter — as a split point,
//! where `NurbCurve::split` cannot absorb a `min_subdivision_size`-wide one
//! (Boehm insertion amplifies its width without bound) — refines it here.

use crate::scalars::Scalar;

/// Newton iterations. Quadratic convergence makes a handful plenty; this
/// cannot change which solutions are found, only how tightly an
/// already-isolated one is pinned down.
const ITERATIONS: usize = 12;

/// Refine `start` — a box a subdivision search proved holds a root — by
/// Newton, within `domain`. `step` takes an iterate to the next one,
/// unsharpened, or `None` where Newton cannot go on (a singular Jacobian at
/// a tangential crossing or a pole, an evaluation failing).
///
/// Every iterate but the last is sharpened, which is legitimate: it is only
/// a seed for the next step. The final step is left unsharpened, so the
/// returned widths honestly state how well the root is determined (see
/// "Sharpen only where the value is a free choice" in `AGENTS.md`). Each
/// iterate is clamped into `domain` — sharpened first, so the clamp's
/// comparisons are exact — rather than given up on: a foot point can step
/// outside the domain on the way in.
///
/// The result is intersected with `start`: both enclose the same root, so
/// their intersection does too, and is tighter than either. Infallible by
/// construction: if Newton stops, or ends disjoint from `start` (it has
/// wandered out of the box the search proved the root lies in), `start` is
/// returned unchanged. Refinement can only tighten, never fail.
///
/// Plain, unverified Newton: it does not itself certify existence or
/// uniqueness, so callers check that the refined box can still hold what
/// they are looking for.
pub fn refine<S: Scalar, const N: usize>(
    start: [S; N],
    domain: [(S, S); N],
    step: impl Fn([S; N]) -> Option<[S; N]>,
) -> [S; N] {
    let mut x = start.map(|s| s.sharpen());
    for iteration in 0..ITERATIONS {
        let Some(next) = step(x) else {
            return start;
        };
        let next = if iteration + 1 == ITERATIONS {
            next
        } else {
            next.map(|s| s.sharpen())
        };
        x = std::array::from_fn(|i| next[i].clamped(domain[i].0, domain[i].1));
    }
    if (0..N).any(|i| !x[i].could_be_equal(start[i])) {
        return start;
    }
    std::array::from_fn(|i| start[i].intersect(x[i]))
}
