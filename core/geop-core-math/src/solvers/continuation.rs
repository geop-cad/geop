//! Continuation: a problem solved for values far from where its solution
//! is now, by driving the values there in stages and solving at each one
//! from the solution of the stage before — a linkage turned half a turn
//! does not jump to another of its configurations on the way.
//!
//! Where each stage lies is a free choice: any values between serve, and
//! more stages only make each solve's start nearer its answer. So the
//! stages are equal fractions of the way, as few as keep every value's move
//! per stage within `max_step` — how hard the solve tries, never what its
//! answer means. The last stage, at the values themselves, is the caller's
//! to solve, often with more asked of it than of the stages on the way.

/// How many equal stages drive every value from `from` to `to` (`ranges`)
/// with none moving more than `max_step` in one: at least one per value
/// that moves, none for none.
pub fn stages(ranges: &[(f64, f64)], max_step: f64) -> usize {
    (ranges.iter())
        .map(|(from, to)| ((to - from).abs() / max_step).ceil() as usize)
        .max()
        .unwrap_or(0)
}

/// Calls `stage` with the values at every stage on the way from `from` to
/// `to` (`ranges`, see [`stages`]), in order, the end itself not among
/// them; an error from one ends the continuation with it. Each value at the
/// `k`-th of `n` stages is `from + (to - from) * k / n`.
pub fn continuation<E>(
    ranges: &[(f64, f64)],
    max_step: f64,
    mut stage: impl FnMut(&[f64]) -> Result<(), E>,
) -> Result<(), E> {
    let n = stages(ranges, max_step);
    for k in 1..n {
        let at: Vec<f64> = (ranges.iter())
            .map(|(from, to)| from + (to - from) * k as f64 / n as f64)
            .collect();
        stage(&at)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A half turn in steps of at most 30 takes six stages: five on the
    /// way, each the same fraction of every value's move, and the end left
    /// to the caller.
    #[test]
    fn stages_are_equal_and_short_enough() {
        let ranges = [(0.0, 180.0), (10.0, 40.0), (5.0, 5.0)];
        assert_eq!(stages(&ranges, 30.0), 6);
        let mut seen = Vec::new();
        continuation::<()>(&ranges, 30.0, |at| {
            seen.push(at.to_vec());
            Ok(())
        })
        .unwrap();
        assert_eq!(seen.len(), 5);
        for (k, at) in seen.iter().enumerate() {
            let f = (k + 1) as f64 / 6.0;
            assert!((at[0] - 180.0 * f).abs() < 1e-12);
            assert!((at[1] - (10.0 + 30.0 * f)).abs() < 1e-12);
            assert_eq!(at[2], 5.0);
        }
    }

    /// Nothing to drive, or a move within one step: no stage on the way.
    #[test]
    fn a_short_move_has_no_stage_on_the_way() {
        for ranges in [vec![], vec![(0.0, 20.0)]] {
            let mut called = false;
            continuation::<()>(&ranges, 30.0, |_| {
                called = true;
                Ok(())
            })
            .unwrap();
            assert!(!called);
        }
    }

    /// An error at one stage ends the continuation there.
    #[test]
    fn an_error_ends_the_continuation() {
        let mut calls = 0;
        let out = continuation(&[(0.0, 100.0)], 10.0, |_| {
            calls += 1;
            if calls == 3 { Err("lost") } else { Ok(()) }
        });
        assert_eq!((out, calls), (Err("lost"), 3));
    }
}
