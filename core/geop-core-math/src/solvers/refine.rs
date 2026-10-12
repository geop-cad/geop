//! Adaptive refinement: something built at a resolution — samples along a
//! curve, cells across a face, stations along a blend — built again finer
//! until what it measures of itself is good enough.
//!
//! The resolution is a free choice; whether the result at one is good
//! enough is the caller's measurement, made in `step`. The solver promises
//! that every step is taken from the resolution the one before asked for,
//! at most `most` of them, and says which way it ended: with the result a
//! step accepted ([`Refined::Done`]), or out of steps with the resolution
//! the last one asked for, never accepted ([`Refined::Exhausted`]). A caller
//! that has to stop at a coarse result says so by building it from the
//! exhausted resolution; one that must not, errs.

use crate::solvers::repeat::{Repeated, Step, repeat};

/// What one step made of its resolution.
pub enum Refine<X, T> {
    /// Good enough: the result.
    Done(T),
    /// Not yet: build again at this resolution.
    Finer(X),
}

/// How a refinement ended.
#[derive(Debug, PartialEq, Eq)]
pub enum Refined<X, T> {
    /// A step accepted this.
    Done(T),
    /// Every step was taken; the last asked for this resolution.
    Exhausted(X),
}

impl<X> Refined<X, X> {
    /// What it ended with, accepted or not.
    pub fn either(self) -> X {
        match self {
            Refined::Done(x) | Refined::Exhausted(x) => x,
        }
    }
}

/// Takes `step` from `start`, then from each resolution it asks for, at
/// most `most` times, until one is [`Refine::Done`]. `step` may keep state
/// of its own (a measure of the step before, to tell whether refining
/// still helps), since every step is taken once, in order.
pub fn refine<X, T, E>(
    start: X,
    most: usize,
    mut step: impl FnMut(X) -> Result<Refine<X, T>, E>,
) -> Result<Refined<X, T>, E> {
    // The resolution the next step is taken from: there between steps.
    let mut x = Some(start);
    let run = repeat(most, || {
        let at = x.take().expect("each step leaves the next resolution");
        Ok(match step(at)? {
            Refine::Done(done) => Step::Done(done),
            Refine::Finer(finer) => {
                x = Some(finer);
                Step::Again
            }
        })
    })?;
    Ok(match run {
        Repeated::Done { value, .. } => Refined::Done(value),
        Repeated::Exhausted { .. } => {
            Refined::Exhausted(x.expect("a step not done leaves the next resolution"))
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Midpoint sums of `x²` over `[0, 1]`, the intervals doubled until
    /// two successive sums agree to `1e-6`: the sum it accepts is that
    /// close to a third.
    #[test]
    fn doubling_until_successive_sums_agree() {
        let sum = |n: usize| {
            (0..n)
                .map(|i| ((i as f64 + 0.5) / n as f64).powi(2) / n as f64)
                .sum::<f64>()
        };
        let out = refine::<_, _, ()>((1usize, f64::INFINITY), 40, |(n, before)| {
            let s = sum(n);
            Ok(if (s - before).abs() < 1e-6 {
                Refine::Done(s)
            } else {
                Refine::Finer((2 * n, s))
            })
        })
        .unwrap();
        let Refined::Done(s) = out else {
            panic!("{out:?}")
        };
        assert!((s - 1.0 / 3.0).abs() < 1e-6);
    }

    /// A refinement that is never good enough reports that it ran out,
    /// with the resolution the last step asked for — not a result.
    #[test]
    fn running_out_is_reported() {
        let out = refine::<usize, (), ()>(1, 5, |n| Ok(Refine::Finer(2 * n))).unwrap();
        assert_eq!(out, Refined::Exhausted(32));
    }
}
