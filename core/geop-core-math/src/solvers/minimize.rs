//! Minimizing a merit over a point the caller is free to choose: the
//! nearest point of a curve or a surface, the parameter where a curve comes
//! nearest a point. The point found is a choice, not an enclosure of a
//! true minimum — what it is chosen for is measured at it afterwards — so
//! the merits are plain numbers.

use crate::{
    geop_error::GeopError,
    scalars::Scalar,
    solvers::probe::{Probe, halve},
};

/// Damped descent from `start`, at most `steps` steps. `direction(x)`
/// gives the step from `x` as the point it leads to at a `scale` of the
/// full step (`None` where there is none to take); a step is taken at the
/// first of `1, 1/2, 1/4, …` (`halvings` of them) at which it definitely
/// lowers `merit`, and the descent stops where none does. What is
/// returned is never worse than `start`.
pub fn descend<S: Scalar, X: Clone, E: From<GeopError>, D: Fn(S) -> X>(
    start: X,
    steps: usize,
    halvings: usize,
    merit: impl Fn(&X) -> Result<f64, E>,
    mut direction: impl FnMut(&X) -> Result<Option<D>, E>,
) -> Result<X, E> {
    let mut x = start;
    let mut current = merit(&x)?;
    for _ in 0..steps {
        let Some(to) = direction(&x)? else {
            break;
        };
        let lower = halve::<S, _, E>(S::ONE, halvings, |&scale| {
            let next = to(scale);
            let m = merit(&next)?;
            Ok(if m < current {
                Probe::Found((next, m))
            } else {
                Probe::Next
            })
        })?;
        let Some((next, m)) = lower else {
            break;
        };
        (x, current) = (next, m);
    }
    Ok(x)
}

/// Alternating projections towards the nearest pair of points of two sets:
/// from `start`, `b` moved onto the point of its set `onto_b` gives nearest
/// `a`, then `a` onto the point of its own set nearest that, as long as
/// the pair's `merit` (a distance) definitely drops, at most `steps` times.
/// Each move keeps the pair on the two sets, so what is returned is a pair
/// of their points, never worse by `merit` than `start` — a local least,
/// found from where `start` is, not a proof of the global one.
pub fn alternate<A, B, E>(
    start: (A, B),
    steps: usize,
    merit: impl Fn(&A, &B) -> Result<f64, E>,
    mut onto_b: impl FnMut(&A) -> Result<B, E>,
    mut onto_a: impl FnMut(&B) -> Result<A, E>,
) -> Result<(A, B), E> {
    let (mut a, mut b) = start;
    let mut current = merit(&a, &b)?;
    for _ in 0..steps {
        let nb = onto_b(&a)?;
        let na = onto_a(&nb)?;
        let m = merit(&na, &nb)?;
        if m >= current {
            break;
        }
        (current, a, b) = (m, na, nb);
    }
    Ok((a, b))
}

/// A golden-section search for the least of a unimodal merit on `[a, b]`,
/// in plain numbers. Its bracket and the two points inside it carry over
/// from one call of [`GoldenSection::narrow`] to the next, so the search
/// can go on with a finer merit for its last steps.
#[derive(Clone, Copy, Debug)]
pub struct GoldenSection {
    pub a: f64,
    pub b: f64,
    x: f64,
    y: f64,
}

impl GoldenSection {
    /// The ratio the bracket shrinks by each step, `(√5 - 1) / 2`.
    fn ratio() -> f64 {
        (5f64.sqrt() - 1.0) / 2.0
    }

    pub fn new(a: f64, b: f64) -> Self {
        let ratio = Self::ratio();
        GoldenSection {
            a,
            b,
            x: b - ratio * (b - a),
            y: a + ratio * (b - a),
        }
    }

    /// `steps` steps on `merit`, which is asked at both points inside the
    /// bracket first: nothing, for no steps.
    pub fn narrow<E>(
        &mut self,
        steps: usize,
        mut merit: impl FnMut(f64) -> Result<f64, E>,
    ) -> Result<(), E> {
        if steps == 0 {
            return Ok(());
        }
        let ratio = Self::ratio();
        let GoldenSection {
            mut a,
            mut b,
            mut x,
            mut y,
        } = *self;
        let (mut fx, mut fy) = (merit(x)?, merit(y)?);
        for _ in 0..steps {
            if fx <= fy {
                b = y;
                (y, fy) = (x, fx);
                x = b - ratio * (b - a);
                fx = merit(x)?;
            } else {
                a = x;
                (x, fx) = (y, fy);
                y = a + ratio * (b - a);
                fy = merit(y)?;
            }
        }
        *self = GoldenSection { a, b, x, y };
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn golden_section_finds_a_parabola_s_least() {
        let mut g = GoldenSection::new(-1.0, 3.0);
        g.narrow::<()>(60, |t| Ok((t - 0.7) * (t - 0.7))).unwrap();
        assert!((g.a - 0.7).abs() < 1e-7 && (g.b - 0.7).abs() < 1e-7);
    }

    /// Between the unit circle and the line `y = 2`, alternation moves from
    /// a far pair to the nearest one, `(0, 1)` and `(0, 2)`.
    #[test]
    fn alternation_finds_the_nearest_pair() {
        let on_line = |p: &[f64; 2]| -> Result<[f64; 2], ()> { Ok([p[0], 2.0]) };
        let on_circle = |p: &[f64; 2]| -> Result<[f64; 2], ()> {
            let r = p[0].hypot(p[1]);
            Ok([p[0] / r, p[1] / r])
        };
        let gap = |a: &[f64; 2], b: &[f64; 2]| Ok((a[0] - b[0]).hypot(a[1] - b[1]));
        let (a, b) = alternate(([0.6, 0.8], [5.0, 1.0]), 200, gap, on_line, on_circle).unwrap();
        assert!((gap(&a, &b).unwrap() - 1.0).abs() < 1e-9, "{a:?} {b:?}");
        assert!(a[0].abs() < 1e-6 && b[0].abs() < 1e-6);
    }

    /// A start no move improves on is returned as it is.
    #[test]
    fn alternation_never_makes_a_pair_worse() {
        let stay = |p: &f64| -> Result<f64, ()> { Ok(*p + 1.0) };
        let gap = |a: &f64, b: &f64| Ok((a - b).abs());
        assert_eq!(
            alternate((0.0, 0.0), 10, gap, stay, stay).unwrap(),
            (0.0, 0.0)
        );
    }
}
