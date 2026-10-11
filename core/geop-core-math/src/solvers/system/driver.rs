//! The driver every solve of a [`System`] shares: one group solved
//! exactly, and among where its constraints hold preferring what a caller
//! pulls towards and, after that, where things were — a sketch's numbers
//! and a program's state alike.
//!
//! A solve is one minimization by constrained Levenberg–Marquardt
//! ([`least_squares::minimize`]): the constraints held exactly, the pulls
//! and the damping in the sum (see [`DAMPING`]). Where the constraints
//! cannot all hold — or the minimization could not tell how to meet them —
//! a second one takes them into the sum too, as near as they come, least
//! squares, still damped so that what they leave free stays put. Each
//! minimization is reported as a [`Phase`]. Each block moves by steps along
//! it ([`Variable::retract`]).

use crate::{
    geop_error::GeopResult,
    scalars::{Ring, Scalar},
    solvers::least_squares::{self, Evaluation, Rows, Space, Stop},
};

use super::{Bound, Equations, ScalarOf, Solving, Term, Variable};

/// A residual holds once it is within this fraction of the system's size:
/// one part in `RELATIVE_TOLERANCE`.
pub const RELATIVE_TOLERANCE: i64 = 1_000_000_000;

/// How near zero a residual of a system of size `scale` must be to hold:
/// one part in [`RELATIVE_TOLERANCE`] of it.
pub fn tolerance<S: Scalar>(scale: S) -> GeopResult<S> {
    scale.div(S::from_i64(RELATIVE_TOLERANCE))
}

/// How strongly every value a solve may move and nothing pulls is held
/// where it was, as a ratio to a pull, which weighs one: the damping that
/// makes a solve prefer, among the many configurations the constraints
/// allow, the one nearest where things were. A value the constraints leave
/// free stays put; one they need moves as far as they need, and no further.
///
/// Ranked ten thousand to one below a pull: far enough apart that the
/// damping yields to a pull (to a hundred-millionth, as they enter
/// squared), and no further — the minimizer resolves the weaker only as
/// well as the stronger lets it, and weights ten orders of magnitude apart
/// in the sum leave the weaker unresolved.
pub const DAMPING: (i64, i64) = (1, 10_000);

/// The most steps one minimization takes: how hard a solve tries, never
/// what its answer means.
const MAX_ITERATIONS: usize = 200;

/// What a solve may do with a value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mobility {
    /// Nothing: it stays exactly as it is.
    Fixed,
    /// Move it as far as the constraints need, and no further: it is held
    /// where it was, by [`DAMPING`].
    Held,
    /// Move it as the constraints have it, and hold it nowhere: it follows
    /// the others — a joint's coordinate, measured from its parts. Held as
    /// well, it would pull against them.
    Follows,
}

impl Mobility {
    /// Whether a solve may move it.
    pub fn free(self) -> bool {
        self != Mobility::Fixed
    }
}

/// One minimization of a solve: why it stopped, after how many steps, and
/// the largest residual it left — an upper bound.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Phase<S: Scalar> {
    pub stop: Stop,
    pub iterations: usize,
    /// Steps proposed and turned down on the way (see
    /// [`least_squares::Outcome::turned_down`]).
    pub turned_down: usize,
    #[serde(with = "crate::scalars::as_f64")]
    pub max_residual: S,
}

/// The largest size of `residuals`: an upper bound.
pub(super) fn largest<S: Scalar>(residuals: impl IntoIterator<Item = S>) -> S {
    residuals.into_iter().fold(S::ZERO, |largest, r| {
        let size = r.abs().upper();
        if size.could_be_greater(largest) {
            size
        } else {
            largest
        }
    })
}

/// Where a solve of a group ended, and the phases that put it there.
pub(super) type Phased<E> = (Vec<<E as Equations>::Value>, Vec<Phase<ScalarOf<E>>>);

/// Where one minimization ended, how, and whether the constraints hold
/// there.
type Minimized<E> = (Vec<<E as Equations>::Value>, Phase<ScalarOf<E>>, bool);

/// The values of a group of blocks, as the points a minimization moves:
/// each stepped along its own steps ([`Variable::retract`]).
pub(super) struct Values<V>(pub(super) std::marker::PhantomData<V>);

impl<V: Variable> Space<V::Scalar> for Values<V> {
    type Point = Vec<V>;

    fn dimension(&self, at: &Vec<V>) -> usize {
        at.iter().map(Variable::dimension).sum()
    }

    fn step(&self, at: &Vec<V>, step: &[V::Scalar]) -> Option<Vec<V>> {
        let mut rest = step;
        let mut next = Vec::with_capacity(at.len());
        for value in at {
            let (own, after) = rest.split_at_checked(value.dimension())?;
            next.push(value.retract(own).ok()?);
            rest = after;
        }
        Some(next)
    }

    fn same(&self, a: &Vec<V>, b: &Vec<V>) -> bool {
        a.iter().zip(b).all(|(a, b)| a.same(b))
    }
}

impl<E: Equations> Solving<'_, '_, E> {
    /// The group solved from `start` with the limits `active` held at
    /// their bounds and the constraints `left_out` left out (see
    /// [`Asked::solve`]).
    pub(super) fn solve(
        &self,
        start: Vec<E::Value>,
        active: &[usize],
        left_out: &[usize],
    ) -> GeopResult<Phased<E>> {
        let hard: Vec<usize> = (self.group.constraints.iter())
            .filter(|c| !left_out.contains(c))
            .filter(|&&c| self.system.equations.bound(c) == Bound::Zero || active.contains(&c))
            .copied()
            .collect();
        Asked {
            solving: self,
            hard,
        }
        .solve(start)
    }
}

/// A group as one solve has it: the constraints `hard` to hold.
struct Asked<'a, 's, 'e, E: Equations> {
    solving: &'a Solving<'s, 'e, E>,
    hard: Vec<usize>,
}

/// The residuals of a group at a point, each a length, and their Jacobians
/// by steps from it: the constraints' — zero where they hold — the pulls'
/// — each the distance from where a value is pulled to — and those of every
/// value [`Mobility::Held`] and not pulled, from where it was.
struct Terms<S: Scalar> {
    constraints: Rows<S>,
    pulls: Rows<S>,
    stays: Rows<S>,
}

impl<E: Equations> Asked<'_, '_, '_, E> {
    /// The residuals at `values` — the pulls only if `pulling` (else
    /// nothing is pulled, and every value held stays), every value held
    /// staying where it is at `anchor`. `None` where one is undecidable,
    /// which makes `values` infeasible.
    fn terms(
        &self,
        values: &[E::Value],
        anchor: &[E::Value],
        pulling: bool,
    ) -> Option<Terms<ScalarOf<E>>> {
        let solving = self.solving;
        let identity = |_: usize| None;
        let mut which: Vec<Term> = self.hard.iter().map(|&c| Term::Constraint(c)).collect();
        if pulling {
            which.extend(
                solving
                    .group
                    .preferences
                    .iter()
                    .map(|&p| Term::Preference(p)),
            );
        }
        let made = solving.made(values, &which);
        let mut constraints = Rows::default();
        let mut pulls = Rows::default();
        let mut pulled = Vec::new();
        for (term, made) in which.iter().zip(made) {
            let made = made?;
            match term {
                Term::Constraint(_) => constraints.append(solving.rows(&made, &identity)),
                Term::Preference(_) => {
                    pulled.extend(made.jacobian.iter().map(|(block, _)| *block));
                    pulls.append(solving.rows(&made, &identity));
                }
            }
        }
        let mut stays = Rows::default();
        for (k, ((&block, value), at)) in (solving.group.blocks.iter())
            .zip(values)
            .zip(anchor)
            .enumerate()
        {
            if pulled.contains(&block) || solving.system.mobility[block] != Mobility::Held {
                continue;
            }
            let stay = value.stay(at, solving.system.scale).ok()?;
            let column = solving.columns[k];
            for (value, own) in stay.values.into_iter().zip(stay.jacobian) {
                let mut row = vec![ScalarOf::<E>::ZERO; solving.n];
                for (slot, v) in row.iter_mut().skip(column).zip(own) {
                    *slot = v;
                }
                stays.push(value, row);
            }
        }
        Some(Terms {
            constraints,
            pulls,
            stays,
        })
    }

    /// The largest residual of the constraints at `values` — an upper
    /// bound, infinite where one is undecidable (see [`largest`]) — and
    /// whether they all hold there.
    fn check(&self, values: &[E::Value]) -> GeopResult<(ScalarOf<E>, bool)> {
        let tolerance = tolerance(self.solving.system.scale)?;
        Ok(self.solving.check(values, &self.hard, tolerance))
    }

    /// Where the group has to be, from `start`, for its constraints to
    /// hold, as near where its pulls want it and, among those, as near
    /// `start` as the constraints allow — or, where they cannot all hold,
    /// where they come closest (see the module docs). The point is given
    /// whether or not they hold, with the phases that put it there.
    fn solve(&self, start: Vec<E::Value>) -> GeopResult<Phased<E>> {
        let (solved, phase, holds) = self.minimize(start, true)?;
        let mut phases = vec![phase];
        if holds {
            return Ok((solved, phases));
        }
        let (solved, phase, _) = self.minimize(solved, false)?;
        phases.push(phase);
        Ok((solved, phases))
    }

    /// One minimization from `from`: the constraints held exactly and the
    /// group pulled if `hard`, else the constraints in the sum; every value
    /// held where it is at `from`. Where it ended, how, and whether the
    /// constraints hold there.
    fn minimize(&self, from: Vec<E::Value>, hard: bool) -> GeopResult<Minimized<E>> {
        let space = &self.solving.space;
        if space.dimension(&from) == 0 {
            let (max_residual, holds) = self.check(&from)?;
            let phase = Phase {
                stop: Stop::NoChange { rejected: None },
                iterations: 0,
                turned_down: 0,
                max_residual,
            };
            return Ok((from, phase, holds));
        }
        let damping = ScalarOf::<E>::from_ratio(DAMPING.0, DAMPING.1)?;
        let anchor = from.clone();
        let evaluate = |at: &Vec<E::Value>| {
            let Terms {
                constraints,
                pulls,
                stays,
            } = self.terms(at, &anchor, hard)?;
            let mut evaluation = Evaluation::default();
            if hard {
                evaluation.constraints = constraints;
            } else {
                evaluation.sum = constraints;
            }
            let sum = &mut evaluation.sum;
            sum.append(pulls);
            for (value, row) in stays.values.into_iter().zip(stays.jacobian) {
                sum.push(
                    value.mul(damping),
                    row.into_iter().map(|d| d.mul(damping)).collect(),
                );
            }
            // An unbounded residual or slope is no number either.
            let finite = |rows: &Rows<ScalarOf<E>>| {
                (rows.values.iter())
                    .chain(rows.jacobian.iter().flatten())
                    .all(|v| v.is_finite())
            };
            (finite(&evaluation.constraints) && finite(&evaluation.sum)).then_some(evaluation)
        };
        let outcome = least_squares::minimize(
            space,
            evaluate,
            from,
            least_squares::Options {
                max_iterations: MAX_ITERATIONS,
                max_step: self.solving.system.scale,
            },
        )?;
        let (max_residual, holds) = self.check(&outcome.x)?;
        let phase = Phase {
            stop: outcome.stop,
            iterations: outcome.iterations,
            turned_down: outcome.turned_down,
            max_residual,
        };
        Ok((outcome.x, phase, holds))
    }
}
