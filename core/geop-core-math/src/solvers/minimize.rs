//! Minimizing a merit over a point the caller is free to choose: the
//! nearest point of a curve or a surface. The point found is a choice, not
//! an enclosure of a true minimum — what it is chosen for is measured at it
//! afterwards — so the merit is a plain number.

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
