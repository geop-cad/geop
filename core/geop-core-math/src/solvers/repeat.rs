//! Repeating a step until it says it is done: the driver of every loop that
//! runs until a condition holds — a worklist rewritten until nothing is
//! left to rewrite (the next split, merge or imprint found and made, until
//! none is found), a march stepped on until it arrives or leaves, passes
//! over a list until one changes nothing.
//!
//! What a step does, and when the work is done, are the caller's. The
//! solver promises the bound: at most `most` steps, each taken once and in
//! order, and an explicit [`Repeated::Exhausted`] when every one of them
//! was taken and none said it was done. Running out is never read as done:
//! a caller that can do nothing else with an unfinished run says so with
//! [`Repeated::done`] or [`Repeated::done_or`].
//!
//! [`repeat`] is the driver; [`rewrite`] is its worklist form (find the next
//! rewrite, make it, search again), and [`resolve`] its form for passes
//! over a list, which needs no budget: it ends by its own count.

use std::{convert::Infallible, fmt::Display};

use crate::geop_error::{GeopError, GeopResult};

/// What one step said.
pub enum Step<T> {
    /// Not done: take another step.
    Again,
    /// Done, with this.
    Done(T),
}

/// How a run of steps ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[must_use]
pub enum Repeated<T> {
    /// A step said it was done, with `value`, after `steps` steps — that
    /// one among them.
    Done { value: T, steps: usize },
    /// All `steps` steps were taken, and none said it was done: the work
    /// is unfinished.
    Exhausted { steps: usize },
}

impl<T> Repeated<T> {
    /// What the run was done with, or `exhausted(steps)` for a run that
    /// was not.
    pub fn done_or<E>(self, exhausted: impl FnOnce(usize) -> E) -> Result<T, E> {
        match self {
            Repeated::Done { value, .. } => Ok(value),
            Repeated::Exhausted { steps } => Err(exhausted(steps)),
        }
    }

    /// What the run was done with, or an error saying that `what` did not
    /// finish within the steps it was given.
    pub fn done(self, what: impl Display) -> GeopResult<T> {
        self.done_or(|steps| GeopError::new(format!("{what} did not finish within {steps} steps")))
    }
}

/// Takes `step` until it says [`Step::Done`], at most `most` times.
/// `step` keeps the state of the work itself, since every step is taken
/// once, in order.
pub fn repeat<T, E>(
    most: usize,
    mut step: impl FnMut() -> Result<Step<T>, E>,
) -> Result<Repeated<T>, E> {
    for steps in 1..=most {
        if let Step::Done(value) = step()? {
            return Ok(Repeated::Done { value, steps });
        }
    }
    Ok(Repeated::Exhausted { steps: most })
}

/// Rewrites `state` until nothing is left to rewrite: `find` looks for the
/// next rewrite in it, `apply` makes the one found, and the search starts
/// over from the state that leaves — a rewrite changes what there is to
/// find. At most `most` searches, the last of which, in a run that is done,
/// finds nothing; done with how many rewrites were made.
///
/// `find` gets the state to read and `apply` to change, so a search can
/// hold on to borrows of the state that a rewrite could not coexist with.
pub fn rewrite<St: ?Sized, X, E>(
    state: &mut St,
    most: usize,
    mut find: impl FnMut(&St) -> Result<Option<X>, E>,
    mut apply: impl FnMut(&mut St, X) -> Result<(), E>,
) -> Result<Repeated<usize>, E> {
    let mut made = 0;
    repeat(most, || match find(state)? {
        None => Ok(Step::Done(made)),
        Some(found) => {
            apply(state, found)?;
            made += 1;
            Ok(Step::Again)
        }
    })
}

/// Resolves `items` in passes, each trying every item still unresolved, in
/// order, until all are or a pass resolves none: what is left then never
/// will be, and comes back, in order, with what its last try said. A try
/// may depend on what the tries before it resolved, in this pass or an
/// earlier one — that is what a later pass is for. Every pass but the last
/// resolves one item at least, so it ends after as many passes as there
/// are items, and one more.
pub fn resolve<I, F>(items: Vec<I>, mut try_one: impl FnMut(&I) -> Result<(), F>) -> Vec<(I, F)> {
    let mut left = items;
    let passes = left.len() + 1;
    let run = repeat::<_, Infallible>(passes, || {
        let count = left.len();
        let mut failed = Vec::new();
        for item in std::mem::take(&mut left) {
            if let Err(why) = try_one(&item) {
                failed.push((item, why));
            }
        }
        if failed.len() == count {
            return Ok(Step::Done(failed));
        }
        left = failed.into_iter().map(|(item, _)| item).collect();
        Ok(Step::Again)
    });
    match run {
        Ok(Repeated::Done { value, .. }) => value,
        // A pass that resolves nothing ends the run, and there are no more
        // passes that resolve something than there are items.
        Ok(Repeated::Exhausted { .. }) => {
            unreachable!("resolve: every pass but the last resolves an item")
        }
        Err(never) => match never {},
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A step that says it is done ends the run there, with its value and
    /// how many steps it took.
    #[test]
    fn a_run_ends_at_the_step_that_is_done() {
        let mut n = 0;
        let run = repeat::<_, ()>(10, || {
            n += 1;
            Ok(if n == 4 {
                Step::Done(n * 10)
            } else {
                Step::Again
            })
        });
        assert_eq!(
            run,
            Ok(Repeated::Done {
                value: 40,
                steps: 4
            })
        );
    }

    /// A run that is never done reports that it ran out — after taking
    /// exactly `most` steps — and `done_or` turns that into the error.
    #[test]
    fn running_out_is_reported() {
        let mut taken = 0;
        let run = repeat::<(), ()>(5, || {
            taken += 1;
            Ok(Step::Again)
        })
        .unwrap();
        assert_eq!((run, taken), (Repeated::Exhausted { steps: 5 }, 5));
        assert_eq!(run.done_or(|steps| steps), Err(5));
        let message = format!("{:?}", run.done("the search").unwrap_err());
        assert!(
            message.contains("the search did not finish within 5 steps"),
            "{message}"
        );
    }

    /// An error from a step ends the run with it.
    #[test]
    fn an_error_ends_the_run() {
        let mut taken = 0;
        let run = repeat::<(), &str>(5, || {
            taken += 1;
            if taken == 2 {
                Err("no")
            } else {
                Ok(Step::Again)
            }
        });
        assert_eq!((run, taken), (Err("no"), 2));
    }

    /// Merging equal neighbours until none are left: each search starts
    /// over from the state the last merge left, and the run is done once a
    /// search finds nothing.
    #[test]
    fn a_rewrite_runs_until_nothing_is_found() {
        let mut v = vec![1, 1, 2, 2, 2, 3];
        let run = rewrite::<_, _, ()>(
            &mut v,
            10,
            |v| Ok(v.windows(2).position(|w| w[0] == w[1])),
            |v, i| {
                v.remove(i);
                Ok(())
            },
        );
        assert_eq!(run, Ok(Repeated::Done { value: 3, steps: 4 }));
        assert_eq!(v, [1, 2, 3]);
    }

    /// A rewrite that always finds something more runs out, rather than
    /// being taken for done.
    #[test]
    fn a_rewrite_that_never_settles_runs_out() {
        let mut v = vec![0];
        let run = rewrite::<_, _, ()>(
            &mut v,
            3,
            |_| Ok(Some(1)),
            |v, x| {
                v.push(x);
                Ok(())
            },
        );
        assert_eq!(run, Ok(Repeated::Exhausted { steps: 3 }));
        assert_eq!(v.len(), 4);
    }

    /// Definitions resolved in passes: one that depends on a later one is
    /// resolved in a later pass, and ones that depend on each other, or on
    /// nothing defined, are left with what their last try said.
    #[test]
    fn resolving_in_passes_leaves_what_never_resolves() {
        let defs = [
            ("a", "b"),
            ("b", ""),
            ("c", "a"),
            ("x", "y"),
            ("y", "x"),
            ("z", "q"),
        ];
        let mut known: Vec<&str> = Vec::new();
        let mut tries = 0;
        let left = resolve(defs.to_vec(), |&(name, needs)| {
            tries += 1;
            if needs.is_empty() || known.contains(&needs) {
                known.push(name);
                Ok(())
            } else {
                Err(format!("{name} needs {needs}"))
            }
        });
        assert_eq!(known, ["b", "a", "c"]);
        let left: Vec<&str> = left.iter().map(|((n, _), _)| *n).collect();
        assert_eq!(left, ["x", "y", "z"]);
        // Pass one tries all six and resolves b; pass two a, then c, which
        // needs a; pass three resolves nothing.
        assert_eq!(tries, 6 + 5 + 3);
    }
}
