//! Probing a sequence of trial values until one decides: a step halved
//! until a point stepped off a curve lands where it should, a reach
//! doubled until it brackets a crossing, a point marched on until it
//! passes one, a ray cast in one direction after another until one is
//! clear of every vertex.
//!
//! What is probed is a free choice — how far to step, how far to reach —
//! and what the probe finds there is the caller's to judge. The solver
//! promises only the sequence: `start`, then each value `next` makes of
//! the one before, at most `most` of them, each asked once and in order.
//! It answers `None` when none decided, never the last value tried.

use crate::{geop_error::GeopError, scalars::Scalar};

/// What a probe found at one value of the sequence.
pub enum Probe<T> {
    /// It decides: the search ends with this.
    Found(T),
    /// It does not: go on to the next value.
    Next,
    /// It does not, and no later value will: the search ends with nothing.
    Stop,
}

/// Asks `at` for `start` and each value `next` makes of the one before, at
/// most `most` of them, until one is [`Probe::Found`] or [`Probe::Stop`].
/// `next` is asked only for a value that will be probed. `at` may keep
/// state of its own (what the probes so far said), since every value is
/// asked once, in order.
pub fn probe<X, T, E>(
    start: X,
    most: usize,
    mut next: impl FnMut(&X) -> Result<X, E>,
    mut at: impl FnMut(&X) -> Result<Probe<T>, E>,
) -> Result<Option<T>, E> {
    let mut x = start;
    for k in 0..most {
        match at(&x)? {
            Probe::Found(found) => return Ok(Some(found)),
            Probe::Stop => return Ok(None),
            Probe::Next => {}
        }
        if k + 1 < most {
            x = next(&x)?;
        }
    }
    Ok(None)
}

/// [`probe`] with a step halved each time: `start`, `start / 2`, … — at
/// most `most` of them.
pub fn halve<T, E: From<GeopError>>(
    start: Scalar,
    most: usize,
    at: impl FnMut(&Scalar) -> Result<Probe<T>, E>,
) -> Result<Option<T>, E> {
    probe(start, most, |s| Ok(s.div(Scalar::TWO)?), at)
}

/// [`probe`] over attempts `0, 1, 2, …` — at most `most` of them — for a
/// sequence `at` makes itself (the `k`-th of a set of directions, the next
/// draw of a seeded generator).
pub fn attempts<T, E>(
    most: usize,
    at: impl FnMut(&usize) -> Result<Probe<T>, E>,
) -> Result<Option<T>, E> {
    probe(0, most, |k| Ok(k + 1), at)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Halving from 1 finds the first step below a third — and asks every
    /// step before it, in order, and none after.
    #[test]
    fn halving_finds_the_first_step_that_decides() {
        let mut asked = Vec::new();
        let found = halve::<_, GeopError>(Scalar::ONE, 10, |s| {
            asked.push(*s);
            Ok(if s.to_f64() < 1.0 / 3.0 {
                Probe::Found(*s)
            } else {
                Probe::Next
            })
        })
        .unwrap();
        // Halved in interval arithmetic, each step encloses the exact one.
        let exact = [1.0, 0.5, 0.25];
        assert_eq!(asked.len(), exact.len());
        for (a, e) in asked.iter().zip(exact) {
            assert!(a.could_be_equal(Scalar::from_f64(e)), "{a:?}");
        }
        assert!(found.is_some_and(|f| f.could_be_equal(Scalar::from_f64(0.25))));
    }

    /// A sequence that never decides reports nothing — not its last value —
    /// and makes no value past the last it may ask.
    #[test]
    fn a_probe_that_never_decides_finds_nothing() {
        let mut made = 0;
        let found = probe::<f64, f64, ()>(
            1.0,
            4,
            |x| {
                made += 1;
                Ok(x * 2.0)
            },
            |_| Ok(Probe::Next),
        )
        .unwrap();
        assert_eq!(found, None);
        assert_eq!(made, 3);
    }

    /// `Stop` ends the search at once, with nothing.
    #[test]
    fn stop_ends_the_search() {
        let mut asked = 0;
        let found = probe::<f64, f64, ()>(
            1.0,
            100,
            |x| Ok(x + 1.0),
            |&x| {
                asked += 1;
                Ok(if x >= 3.0 { Probe::Stop } else { Probe::Next })
            },
        )
        .unwrap();
        assert_eq!((found, asked), (None, 3));
    }

    /// Attempts are counted from zero, and none past `most`.
    #[test]
    fn attempts_count_from_zero() {
        let first_odd = attempts::<usize, ()>(5, |&k| {
            Ok(if k % 2 == 1 {
                Probe::Found(k)
            } else {
                Probe::Next
            })
        });
        assert_eq!(first_odd, Ok(Some(1)));
        let mut asked = Vec::new();
        let none = attempts::<(), ()>(3, |&k| {
            asked.push(k);
            Ok(Probe::Next)
        });
        assert_eq!((none, asked), (Ok(None), vec![0, 1, 2]));
    }
}
