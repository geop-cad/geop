//! Fixed-point iteration from sharp seeds: the loop every Newton-type
//! method of the kernel runs — foot points, a ball rolled between faces,
//! a vertex moved onto offset surfaces, a traced curve's corrector.
//!
//! Each step takes a seed to the next iterate. The iterate is kept twice
//! (see "Sharpen only where the value is a free choice" in `AGENTS.md`):
//! as the *seed* of the next step, sharpened — any point of it serves
//! equally well, and left wide each step would compound the width of the
//! last — and as the *answer*, with the width the step honestly gave it.
//! Which of the two a caller returns says whether its value is a free
//! choice (a construction aid, a seed for something else) or an answer
//! that later interval tests compare against.
//!
//! A deterministic iteration whose sharp seed repeats one before it is
//! done: from there on the seeds repeat with that period, so what the
//! remaining steps would end at is known ([`Repeat`]). Not a convergence
//! tolerance — the seed is bit for bit one seen before.

use crate::{scalars::Scalar, vector::Vector};

/// A value an iteration can tell apart from another, bit for bit: what
/// [`Repeat`] compares seeds with.
pub trait Iterate: Clone {
    /// Whether `self` and `other` are the same value — the same enclosure,
    /// not merely overlapping ones.
    fn same(&self, other: &Self) -> bool;
}

impl<S: Scalar> Iterate for S {
    fn same(&self, other: &Self) -> bool {
        self.is_subset_of(*other) && other.is_subset_of(*self)
    }
}

impl Iterate for f64 {
    fn same(&self, other: &Self) -> bool {
        self == other
    }
}

impl<T: Iterate, const N: usize> Iterate for [T; N] {
    fn same(&self, other: &Self) -> bool {
        self.iter().zip(other).all(|(a, b)| a.same(b))
    }
}

impl<A: Iterate, B: Iterate> Iterate for (A, B) {
    fn same(&self, other: &Self) -> bool {
        self.0.same(&other.0) && self.1.same(&other.1)
    }
}

impl<S: Scalar, const N: usize> Iterate for Vector<S, N> {
    fn same(&self, other: &Self) -> bool {
        (0..N).all(|i| self[i].same(&other[i]))
    }
}

/// When an iteration stops because a seed repeats.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Repeat {
    /// Never: every step is taken.
    Never,
    /// When a step returns the seed it started from. Safe for a step that
    /// keeps state of its own beside the seed (a foot point's parameters
    /// carried along as the next projection's start), as long as that
    /// state is a function of the seed: the step would only repeat itself.
    FixedPoint,
    /// When a seed repeats any one before it: the seeds are periodic from
    /// there, and the last step's answer is the one taken at the same place
    /// in the period. Only for a step that is a function of its seed alone.
    Cycle,
}

/// What one step found at its seed.
pub enum Step<X> {
    /// The next iterate, as the step computed it: the caller's `settle`
    /// makes a seed and an answer of it.
    Next(X),
    /// The seed meets what the iteration was asked for: stop here.
    Done,
    /// No step can be taken from the seed — a singular Jacobian, a
    /// residual with no direction to go.
    Stuck,
}

/// How an iteration ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum End {
    /// A step said [`Step::Done`].
    Done,
    /// A step said [`Step::Stuck`].
    Stuck,
    /// Every step was taken, or a [`Repeat`] showed where they would end.
    Exhausted,
}

/// Where an iteration ended: its last seed — sharp, the start of a step
/// not taken — and the answer the last step taken gave, or the initial
/// answer if none was.
#[derive(Clone, Debug)]
pub struct Iterated<X> {
    pub seed: X,
    pub answer: X,
    pub end: End,
}

/// Iterates `step` from `seed` at most `steps` times. Each [`Step::Next`]
/// is made into the next seed and the answer by `settle` — typically the
/// iterate sharpened and clamped into a domain, and the iterate clamped
/// (sharpen *before* clamping: `clamped`'s comparisons are exact on a sharp
/// value, so the seed stays in the domain). `answer` is what is returned
/// as the answer if no step is taken. The iteration stops early, with the
/// result it would have reached, where `repeat` says.
pub fn iterate<X: Iterate, E>(
    seed: X,
    answer: X,
    steps: usize,
    repeat: Repeat,
    settle: impl Fn(&X) -> (X, X),
    mut step: impl FnMut(&X) -> Result<Step<X>, E>,
) -> Result<Iterated<X>, E> {
    let (mut seed, mut answer) = (seed, answer);
    // Every step's seed and the answer it gave, in order.
    let mut taken: Vec<(X, X)> = Vec::new();
    for _ in 0..steps {
        let next = match step(&seed)? {
            Step::Next(next) => next,
            Step::Done => {
                return Ok(Iterated {
                    seed,
                    answer,
                    end: End::Done,
                });
            }
            Step::Stuck => {
                return Ok(Iterated {
                    seed,
                    answer,
                    end: End::Stuck,
                });
            }
        };
        let (next_seed, next_answer) = settle(&next);
        taken.push((seed, next_answer.clone()));
        let repeated = match repeat {
            Repeat::Never => None,
            Repeat::FixedPoint => {
                let k = taken.len() - 1;
                taken[k].0.same(&next_seed).then_some(k)
            }
            Repeat::Cycle => taken.iter().position(|(s, _)| s.same(&next_seed)),
        };
        if let Some(k) = repeated {
            let period = taken.len() - k;
            if period == 1 {
                // A fixed point: the seed just made is the one returned.
                return Ok(Iterated {
                    seed: next_seed,
                    answer: next_answer,
                    end: End::Exhausted,
                });
            }
            let last = k + (steps - 1 - k) % period;
            let after = k + (steps - k) % period;
            return Ok(Iterated {
                seed: taken[after].0.clone(),
                answer: taken[last].1.clone(),
                end: End::Exhausted,
            });
        }
        seed = next_seed;
        answer = next_answer;
    }
    Ok(Iterated {
        seed,
        answer,
        end: End::Exhausted,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cycle stops the iteration early with what running every step
    /// would have ended at.
    #[test]
    fn a_cycle_ends_where_every_step_would() {
        // Seeds cycle 0 -> 1 -> 2 -> 1 -> 2 ...; the answer is the seed
        // plus a tenth.
        let next = |x: &f64| Step::Next(if *x == 2.0 { 1.0 } else { x + 1.0 });
        let settle = |x: &f64| (*x, x + 0.1);
        for steps in 0..9 {
            let run = |repeat| {
                iterate::<f64, ()>(0.0, -1.0, steps, repeat, settle, |x| Ok(next(x))).unwrap()
            };
            let (never, cycle) = (run(Repeat::Never), run(Repeat::Cycle));
            assert_eq!((never.seed, never.answer), (cycle.seed, cycle.answer));
        }
    }

    /// A fixed point stops the iteration at the step that found it.
    #[test]
    fn a_fixed_point_stops_at_once() {
        let mut calls = 0;
        let out = iterate::<f64, ()>(
            3.0,
            3.0,
            50,
            Repeat::FixedPoint,
            |x| (*x, *x),
            |x| {
                calls += 1;
                Ok(Step::Next((x / 2.0).max(1.0)))
            },
        )
        .unwrap();
        assert_eq!(out.seed, 1.0);
        assert_eq!(calls, 3);
    }
}
