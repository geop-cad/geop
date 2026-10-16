//! Newton refinement of a root a subdivision search has isolated — shared
//! by `geop_core_geometry`'s `NurbCurve::refine_parameter_at_point` and the
//! intersection searches' `refine_crossing`s — and Newton's method on one
//! unknown from a seed ([`newton_1d`]), for a point that is a free choice
//! up to where a function vanishes.
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

use std::fmt::Debug;

use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use super::iterate::{End, Iterate, Iterated, Repeat, Step, iterate};

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
/// A seed repeating one before it ends the iteration early with the same
/// result ([`super::iterate::Repeat::Cycle`]).
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
    let clamp = |x: [S; N]| std::array::from_fn(|i| x[i].clamped(domain[i].0, domain[i].1));
    let sharp = start.map(|s| s.sharpen());
    let Ok(Iterated {
        answer: x,
        end: End::Exhausted,
        ..
    }) = iterate::<_, ()>(
        sharp,
        sharp,
        ITERATIONS,
        Repeat::Cycle,
        |next| (clamp(next.map(|s| s.sharpen())), clamp(*next)),
        |x| Ok(step(*x).map_or(Step::Stuck, Step::Next)),
    )
    else {
        return start;
    };
    if (0..N).any(|i| !x[i].could_be_equal(start[i])) {
        return start;
    }
    std::array::from_fn(|i| start[i].intersect(x[i]))
}

/// A number [`newton_1d`] runs in: plain `f64`, or a scalar.
pub trait Newton1d: Iterate + Copy + Debug {
    /// Whether `self`, a function's value, could be zero.
    fn could_be_zero(self) -> bool;
    /// The Newton step `value / slope` and its size, or why there is none.
    fn step(value: Self, slope: Self) -> GeopResult<(Self, f64)>;
    /// `self - step`, as the seed of the next step: clamped into
    /// `(lo, hi)`, and sharp — it is a free choice.
    fn settle(self, step: Self, lo: Self, hi: Self) -> Self;
}

impl Newton1d for f64 {
    fn could_be_zero(self) -> bool {
        self == 0.0
    }
    fn step(value: Self, slope: Self) -> GeopResult<(Self, f64)> {
        if slope == 0.0 {
            return Err(GeopError::new(format!(
                "Newton's step: the slope is zero where the value is {value:?}"
            )));
        }
        let step = value / slope;
        Ok((step, step.abs()))
    }
    fn settle(self, step: Self, lo: Self, hi: Self) -> Self {
        (self - step).clamp(lo, hi)
    }
}

/// Sharpened *before* clamping: `clamped`'s comparisons are exact on a
/// sharp value, so the seed stays in the domain.
impl<S: Scalar> Newton1d for S {
    fn could_be_zero(self) -> bool {
        self.could_be_equal(S::ZERO)
    }
    fn step(value: Self, slope: Self) -> GeopResult<(Self, f64)> {
        let step = value.div(slope)?;
        Ok((step, step.abs().upper().to_f64()))
    }
    fn settle(self, step: Self, lo: Self, hi: Self) -> Self {
        self.sub(step).sharpen().clamped(lo, hi)
    }
}

/// When [`newton_1d`] has arrived.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Arrive {
    /// Where the value could be zero.
    AtZero,
    /// Where a step is no shorter than the one before: as close as the
    /// arithmetic gets.
    StepsStopShrinking,
    /// After every step: the seed the last one left.
    Exhausted,
}

/// Newton's method on one unknown from `seed`, at most `steps` steps,
/// within `(lo, hi)`. `at(x)` gives the function's value and slope at `x`.
/// Every iterate is a sharp seed clamped into `(lo, hi)`: what is returned
/// is a point as good as any other where the function vanishes, not an
/// enclosure of the root. `seed` itself is asked as it is.
///
/// The last seed, and whether it arrived as `arrive` says (always, for
/// [`Arrive::Exhausted`]). A slope with no step — zero, or one that could
/// be — is an error.
pub fn newton_1d<T: Newton1d, E: From<GeopError>>(
    seed: T,
    (lo, hi): (T, T),
    steps: usize,
    arrive: Arrive,
    mut at: impl FnMut(T) -> Result<(T, T), E>,
) -> Result<(T, bool), E> {
    let mut last = f64::INFINITY;
    // Whether a step is done depends on the one before only when it stops
    // shrinking; otherwise a repeated seed shows where the steps would end.
    let repeat = match arrive {
        Arrive::StepsStopShrinking => Repeat::Never,
        Arrive::AtZero | Arrive::Exhausted => Repeat::Cycle,
    };
    let iterated = iterate(
        seed,
        seed,
        steps,
        repeat,
        |next| (*next, *next),
        |&x| -> Result<Step<T>, E> {
            let (value, slope) = at(x)?;
            if arrive == Arrive::AtZero && value.could_be_zero() {
                return Ok(Step::Done);
            }
            let (step, size) = T::step(value, slope)?;
            if arrive == Arrive::StepsStopShrinking {
                // A NaN step is no shorter either.
                if size.partial_cmp(&last) != Some(std::cmp::Ordering::Less) {
                    return Ok(Step::Done);
                }
                last = size;
            }
            Ok(Step::Next(x.settle(step, lo, hi)))
        },
    )?;
    let arrived = iterated.end == End::Done || arrive == Arrive::Exhausted;
    Ok((iterated.seed, arrived))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each stopping rule ends where it says: at a value that could be
    /// zero, where the steps stop shrinking, or after every step.
    #[test]
    fn newton_1d_arrives_as_asked() {
        let square = |x: f64| Ok::<_, GeopError>((x * x - 2.0, 2.0 * x));
        let root = |arrive| newton_1d(1.0, (0.0, 2.0), 50, arrive, square).unwrap();
        let (shrunk, arrived) = root(Arrive::StepsStopShrinking);
        assert!(arrived && (shrunk * shrunk - 2.0).abs() <= 4.0 * f64::EPSILON);
        assert_eq!(root(Arrive::Exhausted), (shrunk, true));
        // The value at a float is hardly ever exactly zero.
        let at_zero = newton_1d(3.0, (0.0, 4.0), 50, Arrive::AtZero, |x: f64| {
            Ok::<_, GeopError>((x - 2.0, 1.0))
        });
        assert_eq!(at_zero.unwrap(), (2.0, true));
    }
}
