//! Bisection of a bracket on which side of a crossing its middle lies —
//! in plain numbers, where where-exactly is a free choice found as far as
//! `f64` goes, or in sharp scalars, where the side is told by an interval
//! test that may not decide.

use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

/// A value a bracket can be bisected in.
pub trait Bisect: Copy {
    /// The middle of `lo` and `hi`: where the next side is asked.
    fn middle(lo: Self, hi: Self) -> GeopResult<Self>;
    /// Whether `mid` lies strictly between `lo` and `hi` — once it does not,
    /// the bracket cannot shrink any further.
    fn strictly_between(lo: Self, mid: Self, hi: Self) -> bool;
}

impl Bisect for f64 {
    fn middle(lo: Self, hi: Self) -> GeopResult<Self> {
        Ok(0.5 * (lo + hi))
    }
    fn strictly_between(lo: Self, mid: Self, hi: Self) -> bool {
        mid > lo.min(hi) && mid < lo.max(hi)
    }
}

/// Sharp: which point is asked is the search's own free choice, and a
/// sharp middle keeps the bracket from widening as it shrinks.
impl<S: Scalar> Bisect for S {
    fn middle(lo: Self, hi: Self) -> GeopResult<Self> {
        Ok(lo.add(hi).div(S::TWO)?.sharpen())
    }
    /// For `lo` below `hi`.
    fn strictly_between(lo: Self, mid: Self, hi: Self) -> bool {
        lo.definitely_less(mid) && mid.definitely_less(hi)
    }
}

/// Where a bisection ended: its bracket, and the middle whose side could
/// not be told, if that is what stopped it.
#[derive(Clone, Copy, Debug)]
pub struct Bracket<T> {
    pub lo: T,
    pub hi: T,
    pub undecided: Option<T>,
}

/// Bisects `(lo, hi)` at most `steps` times. `side(mid)` says whether the
/// middle lies on `lo`'s side (`Some(true)`, it becomes `lo`), on `hi`'s
/// (`Some(false)`), or cannot be told (`None`, which stops the bisection).
/// With `until_collapsed`, it also stops once the middle no longer lies
/// strictly inside the bracket. `side` may keep state of its own — a seed
/// carried from one middle to the next — since every step asks it once.
pub fn bisect<T: Bisect, E: From<GeopError>>(
    lo: T,
    hi: T,
    steps: usize,
    until_collapsed: bool,
    mut side: impl FnMut(T) -> Result<Option<bool>, E>,
) -> Result<Bracket<T>, E> {
    let (mut lo, mut hi) = (lo, hi);
    for _ in 0..steps {
        let mid = T::middle(lo, hi)?;
        if until_collapsed && !T::strictly_between(lo, mid, hi) {
            break;
        }
        match side(mid)? {
            Some(true) => lo = mid,
            Some(false) => hi = mid,
            None => {
                return Ok(Bracket {
                    lo,
                    hi,
                    undecided: Some(mid),
                });
            }
        }
    }
    Ok(Bracket {
        lo,
        hi,
        undecided: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bisects_to_the_last_bit() {
        let b = bisect::<f64, GeopError>(1.0, 2.0, 200, true, |x| Ok(Some(x * x < 2.0))).unwrap();
        assert_eq!(b.hi, f64::from_bits(b.lo.to_bits() + 1));
        assert!(b.lo * b.lo < 2.0 && b.hi * b.hi >= 2.0);
    }
}
