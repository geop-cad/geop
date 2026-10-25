//! Bisection of a bracket on which side of a crossing its middle lies —
//! in plain numbers, where where-exactly is a free choice found as far as
//! `f64` goes, or in sharp scalars, where the side is told by an interval
//! test that may not decide — and the scan of samples that finds the
//! brackets: every pair of neighbouring samples on different sides.

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
impl Bisect for Scalar {
    fn middle(lo: Self, hi: Self) -> GeopResult<Self> {
        Ok(lo.add(hi).div(Scalar::TWO)?.sharpen())
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

/// Neighbouring samples on different sides: see [`sign_changes`].
pub struct SignChanges<I: Iterator, F> {
    samples: I,
    side: F,
    /// The last sample whose side was told, and its side.
    last: Option<(I::Item, bool)>,
}

/// The pairs of neighbouring samples of `samples` that `side` puts on
/// different sides, in order, each with the side of its first: brackets
/// of a crossing. `side` says which side a sample lies on (`< cut` against
/// `<= 0`, a sign of a number or of an interval: the caller's choice), or
/// `None` where it cannot tell; such a sample is passed over, its
/// neighbours paired across it. Lazy: a scan for the first crossing asks
/// no sample past it. [`SignChanges::bisected`] bisects each bracket.
pub fn sign_changes<I: IntoIterator, F>(samples: I, side: F) -> SignChanges<I::IntoIter, F> {
    SignChanges {
        samples: samples.into_iter(),
        side,
        last: None,
    }
}

impl<T: Copy, E, I: Iterator<Item = T>, F: FnMut(T) -> Result<Option<bool>, E>> Iterator
    for SignChanges<I, F>
{
    type Item = Result<(T, T, bool), E>;

    fn next(&mut self) -> Option<Self::Item> {
        for t in self.samples.by_ref() {
            let s = match (self.side)(t) {
                Ok(Some(s)) => s,
                Ok(None) => continue,
                Err(e) => return Some(Err(e)),
            };
            if let Some((a, side_a)) = self.last.replace((t, s))
                && side_a != s
            {
                return Some(Ok((a, t, side_a)));
            }
        }
        None
    }
}

impl<I: Iterator, F> SignChanges<I, F> {
    /// Each bracket bisected at most `steps` times on the same `side`
    /// (see [`bisect`] for `until_collapsed`).
    pub fn bisected(self, steps: usize, until_collapsed: bool) -> Crossings<I, F> {
        Crossings {
            scan: self,
            steps,
            until_collapsed,
        }
    }
}

/// The brackets of a [`SignChanges`] scan, bisected: see
/// [`SignChanges::bisected`].
pub struct Crossings<I: Iterator, F> {
    scan: SignChanges<I, F>,
    steps: usize,
    until_collapsed: bool,
}

impl<T: Bisect, E: From<GeopError>, I: Iterator<Item = T>, F: FnMut(T) -> Result<Option<bool>, E>>
    Iterator for Crossings<I, F>
{
    type Item = Result<Bracket<T>, E>;

    fn next(&mut self) -> Option<Self::Item> {
        let (a, b, side_a) = match self.scan.next()? {
            Ok(bracket) => bracket,
            Err(e) => return Some(Err(e)),
        };
        let side = &mut self.scan.side;
        Some(bisect(a, b, self.steps, self.until_collapsed, |mid| {
            Ok(side(mid)?.map(|s| s == side_a))
        }))
    }
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

    /// A middle whose side cannot be told stops the bisection and is
    /// reported, the bracket left as it was told.
    #[test]
    fn an_undecided_middle_is_reported() {
        let b =
            bisect::<f64, GeopError>(0.0, 1.0, 50, false, |mid| Ok((mid < 0.6).then_some(true)))
                .unwrap();
        assert_eq!((b.lo, b.hi, b.undecided), (0.5, 1.0, Some(0.75)));
    }

    /// Every sign change between samples is bracketed and bisected, an
    /// undecided sample passed over.
    #[test]
    fn scans_and_bisects_every_sign_change() {
        let f = |x: f64| (x - 0.3) * (x - 2.7);
        let side = |x: f64| Ok::<_, GeopError>((x != 1.5).then_some(f(x) > 0.0));
        let roots: Vec<Bracket<f64>> = sign_changes((0..=6).map(|k| 0.5 * k as f64), side)
            .bisected(200, true)
            .collect::<Result<_, _>>()
            .unwrap();
        assert_eq!(roots.len(), 2);
        for (b, below) in roots.iter().zip([0.0, 2.5]) {
            assert!(b.lo > below && b.hi < below + 0.5);
            assert_eq!(b.hi, f64::from_bits(b.lo.to_bits() + 1));
            assert!(f(b.lo) * f(b.hi) <= 0.0);
        }
    }
}
