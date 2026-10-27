//! Constraint systems: values a solve moves, in blocks ([`Variable`]s — a
//! number, a pose), and terms on them ([`Equations`]) — constraints that
//! hold where their residuals vanish (or, for a limit, stay at most zero),
//! and preferences a solve pulls towards among where the constraints hold.
//! Written once, for every system of the kernel: a sketch's numbers
//! (`geop_core_sketch`) and a program's state, where its parts are
//! (`geop_core_program::solve`), are each only an [`Equations`].
//!
//! On a [`System`] — the equations, how each block may move, the system's
//! size — the algorithms are generic:
//!
//! - **Groups.** Blocks no term ties together are solved, enclosed and
//!   measured apart ([`System::structure`]): a plate with hundreds of
//!   screws is hundreds of systems of one screw. The answer is the same,
//!   since what a solve minimizes is a sum over the groups; a group whose
//!   constraints hold and that nothing pulls is not solved at all. Within a
//!   group, a block whose constraints only hang it off others — a screw on
//!   a bracket, the bracket on a link — is a pendant: solved and enclosed
//!   after them, alone ([`structure`]), so a chain of links carrying
//!   hundreds of parts is a small system and hundreds of tiny ones, not one
//!   dense system of every part. The groups of a layer are worked on in
//!   lockstep ([`lockstep`]): one evaluation answers every group waiting,
//!   so the evaluations a solve takes do not grow with the number of
//!   groups.
//! - **Solve** ([`System::solve`]) by the driver every solve shares
//!   ([`driver`]): the constraints held exactly, the
//!   preferences and staying put ([`Variable::stay`]) only choosing among
//!   where they hold. Each block moves by steps along it
//!   ([`Variable::retract`]). A limit ([`Bound::AtMost`]) is kept by an
//!   active set: solved without the limits first, every one the solve ends
//!   beyond is held at its bound, and the group is solved again from where
//!   it started.
//! - **What holds** ([`System::held`]), to [`RELATIVE_TOLERANCE`]
//!   of the size.
//! - **Enclosure** ([`System::enclose`]): a rigorous box around a true
//!   solution near where a solve ended — Krawczyk's test over the
//!   independent rows, chosen over a box — or why there is none to be had:
//!   a singular configuration, or constraints that conflict, by name.
//! - **Freedom** ([`System::freedom`]): how many independent ways each
//!   block can still move, decided over the box the enclosure proved. A
//!   limit leaves the motion away from it free.
//! - **Conflicts** ([`System::conflicting`]): a smallest set of constraints
//!   that cannot all hold together.
//!
//! A block is enclosed in its own step coordinates around where it is: a
//! value `x = retract(x̃, δ)` for a box of steps `δ`. The Jacobian of a
//! term by `δ` is its Jacobian by the steps at `x` — what [`Equations`]
//! gives — times the differential of the retraction
//! ([`Variable::differential`]): the identity for a number, the right
//! Jacobian of the exponential map for a pose.

use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::least_squares::Rows,
};

mod driver;
mod enclose;
mod lockstep;
mod structure;

pub use driver::{DAMPING, Mobility, Phase, RELATIVE_TOLERANCE, tolerance};
use driver::{Values, largest};

pub use enclose::{EncloseError, Enclosure};
use lockstep::{Batch, Made};
pub use structure::Structure;

/// A value a solve moves: a block of variables, stepped along its own
/// tangent space.
pub trait Variable: Clone + std::fmt::Debug {
    /// How many numbers a step of it takes. None, for a value no solve
    /// moves.
    fn dimension(&self) -> usize;

    /// Where `step` — [`Variable::dimension`] numbers — takes it: the value
    /// a solver tries next, a free choice, so sharp.
    fn retract(&self, step: &[Scalar]) -> GeopResult<Self>;

    /// Every value a step in the box `step` takes it to: an enclosure,
    /// nothing sharpened — of the value it stands for, which a sharp value
    /// may only approximate (a rotation's unit quaternion).
    fn retract_over(&self, step: &[Scalar]) -> GeopResult<Self>;

    /// The differential of the retraction over the box `step`: the matrix
    /// `D`, one row per number of a step, such that for every `δ` in the box
    /// a step `dδ` further, `retract(δ + dδ)`, is to first order the step
    /// `D dδ` from `retract(δ)` — an enclosure for every `δ` in the box.
    /// `None` for the identity: a step that adds to the value.
    fn differential(&self, step: &[Scalar]) -> GeopResult<Option<Vec<Vec<Scalar>>>>;

    /// What keeps it near `at` where nothing else decides where it goes:
    /// residuals that vanish there, each a length — a turn measured where
    /// it moves a point `scale` from its axis — and their derivatives by
    /// its step.
    fn stay(&self, at: &Self, scale: Scalar) -> GeopResult<Rows>;

    /// Whether it is no different from `other`: a step that changed nothing.
    fn same(&self, other: &Self) -> bool;

    /// Where it goes carried along by `leader` moving from `from` to `to`,
    /// as if fastened to it — a seed for a solve of what hangs off the
    /// leader (see [`Structure`]), a free choice, so sharp. `None` for a
    /// value nothing carries: a number.
    fn carried(&self, from: &Self, to: &Self) -> Option<Self> {
        let _ = (from, to);
        None
    }
}

/// A number steps by what is added to it.
impl Variable for Scalar {
    fn dimension(&self) -> usize {
        1
    }

    fn retract(&self, step: &[Scalar]) -> GeopResult<Scalar> {
        Ok(self.add(*one(step)?).sharpen())
    }

    fn retract_over(&self, step: &[Scalar]) -> GeopResult<Scalar> {
        let d = *one(step)?;
        Ok(match d.is_zero() && d.is_sharp() {
            true => *self,
            false => self.add(d),
        })
    }

    fn differential(&self, step: &[Scalar]) -> GeopResult<Option<Vec<Vec<Scalar>>>> {
        one(step)?;
        Ok(None)
    }

    fn stay(&self, at: &Scalar, _: Scalar) -> GeopResult<Rows> {
        let mut rows = Rows::default();
        rows.push(self.sub(*at), vec![Scalar::ONE]);
        Ok(rows)
    }

    fn same(&self, other: &Scalar) -> bool {
        self.could_be_equal(*other)
    }
}

/// The one number of a step of a number.
fn one(step: &[Scalar]) -> GeopResult<&Scalar> {
    match step {
        [d] => Ok(d),
        _ => Err(GeopError::new(format!(
            "a step of a number is one number, not {}",
            step.len()
        ))),
    }
}

/// What a constraint asks of its residuals.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Bound {
    /// That they vanish: it holds where they are zero.
    #[default]
    Zero,
    /// That none is above zero: a limit, which holds anywhere short of it.
    AtMost,
}

impl Bound {
    /// Whether the residual `r` holds — within `tolerance` of zero, or,
    /// for a limit, not definitely above `tolerance`.
    pub fn holds(self, r: Scalar, tolerance: Scalar) -> bool {
        r.is_finite()
            && match self {
                Bound::Zero => !r.abs().definitely_greater(tolerance),
                Bound::AtMost => !r.definitely_greater(tolerance),
            }
    }
}

/// A term of a system: a constraint or a preference, by index.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum Term {
    Constraint(usize),
    Preference(usize),
}

/// A term's residuals where the blocks are, each a length, and their
/// derivatives: per block it reads, by index, one row per residual — its
/// derivative by that block's step from where it is.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Linearized {
    pub values: Vec<Scalar>,
    pub jacobian: Vec<(usize, Vec<Vec<Scalar>>)>,
}

/// The terms of a system: what an adapter — a sketch, a program — says of
/// its constraints and preferences.
pub trait Equations {
    type Value: Variable;

    /// How many constraints there are, and how many preferences.
    fn count(&self) -> (usize, usize);

    /// What the constraint `constraint` asks of its residuals.
    fn bound(&self, constraint: usize) -> Bound;

    /// The terms `which`, with every block at `values`: each its residuals
    /// and their derivatives, or `None` where it cannot be made there (a
    /// residual undecidable — a direction of zero length — or the term not
    /// made at all).
    fn evaluate(&self, values: &[Self::Value], which: &[Term]) -> Vec<Option<Linearized>>;
}

/// Blocks a solve works on together, and the terms on them (see
/// [`Structure`]): a core no term ties to another, or a pendant hanging
/// off blocks of earlier layers.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Group {
    /// Its blocks, in order: what its variables are, one after another.
    pub blocks: Vec<usize>,
    pub constraints: Vec<usize>,
    pub preferences: Vec<usize>,
    /// The blocks outside it its terms read: what it hangs off. None for a
    /// core.
    pub on: Vec<usize>,
    /// Its layer: none for a core, one past every group it hangs off for a
    /// pendant.
    pub layer: usize,
    /// The component it is part of, by index (see
    /// [`Structure::components`]).
    pub component: usize,
    /// For a pendant, how many ways its terms leave it to move, what it
    /// hangs off held.
    pub own: usize,
}

/// Which constraints hold, and the largest residual: an upper bound,
/// infinite where one cannot be made.
#[derive(Clone, Debug, PartialEq)]
pub struct Held {
    /// Every constraint holds, to [`RELATIVE_TOLERANCE`] of
    /// the size.
    pub converged: bool,
    pub max_residual: Scalar,
    /// The constraints that do not hold, by index.
    pub failed: Vec<usize>,
}

/// Where a solve put the blocks, and how it went.
#[derive(Debug)]
pub struct Solution<V: Variable> {
    pub values: Vec<V>,
    /// The groups moved: those a preference pulled, whose constraints did
    /// not hold, or that hang off a group that moved.
    pub solved: Vec<Group>,
    /// The limits the solve held at their bounds, by index.
    pub at_limit: Vec<usize>,
    /// The most steps any group took, and per phase the worst of the
    /// groups: why it stopped, after how many steps, the largest residual
    /// left.
    pub iterations: usize,
    pub phases: Vec<Phase>,
    /// How many times the solve made the terms — every group's at once:
    /// the work it took, its enclosure's apart.
    pub evaluations: usize,
    /// An enclosure of a true solution near where the solved groups ended
    /// (see [`System::enclose`]) — of those only: the others were not
    /// moved, and enclosing them is the caller's to ask
    /// ([`System::enclose_rest`]).
    pub enclosure: Enclosure<V>,
}

/// How free the blocks are (see [`System::freedom`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Freedom {
    /// Per block, how many independent ways it can still move, the others
    /// moving with it as the constraints need.
    pub blocks: Vec<usize>,
    /// How many independent ways they can move together.
    pub total: usize,
}

/// A system: its equations, how each block may move — every one a solve
/// may move, [`Mobility::Held`] or [`Mobility::Follows`] — and its size.
pub struct System<'e, E: Equations> {
    pub equations: &'e E,
    pub mobility: Vec<Mobility>,
    /// What a constraint holds to [`RELATIVE_TOLERANCE`] of,
    /// how far one step may go, and what a turn is measured against when a
    /// value stays where it was.
    pub scale: Scalar,
}

/// Every term of a system with `count` constraints and preferences.
fn every_term((constraints, preferences): (usize, usize)) -> Vec<Term> {
    (0..constraints)
        .map(Term::Constraint)
        .chain((0..preferences).map(Term::Preference))
        .collect()
}

impl<E: Equations> System<'_, E> {
    /// Which constraints hold at `at`.
    pub fn held(&self, at: &[E::Value]) -> GeopResult<Held> {
        let (count, _) = self.equations.count();
        let constraints: Vec<Term> = (0..count).map(Term::Constraint).collect();
        let tolerance = tolerance(self.scale)?;
        let mut failed = Vec::new();
        let mut values = Vec::new();
        for (c, made) in self.equations.evaluate(at, &constraints).iter().enumerate() {
            let bound = self.equations.bound(c);
            let holds = match made {
                Some(made) => made.values.iter().fold(true, |holds, &r| {
                    values.push(r);
                    holds && bound.holds(r, tolerance)
                }),
                None => {
                    values.push(Scalar::INFINITY);
                    false
                }
            };
            if !holds {
                failed.push(c);
            }
        }
        Ok(Held {
            converged: failed.is_empty(),
            max_residual: largest(values),
            failed,
        })
    }

    /// Where the blocks have to be, from `start`, for every constraint to
    /// hold, as near where the preferences pull them and, among those, as
    /// near `start` as the constraints allow — and an enclosure of a true
    /// solution near there.
    ///
    /// Layer by layer (see [`Structure`]): the cores first, then each
    /// pendant, alone, from where what it hangs off carried it
    /// ([`Variable::carried`]) — solved only where its constraints do not
    /// hold there, or where its own freedom has to stay as near where it
    /// was as they let it. A component a pendant of which cannot follow
    /// after all is solved again as one.
    ///
    /// The blocks are moved even where the constraints cannot all hold, to
    /// where they come closest; [`System::held`] says which do not.
    pub fn solve(&self, start: &[E::Value]) -> GeopResult<Solution<E::Value>> {
        let tolerance = tolerance(self.scale)?;
        let structure = self.structure(start);
        let mut values = start.to_vec();
        let mut solution = Solution {
            values: Vec::new(),
            solved: Vec::new(),
            at_limit: Vec::new(),
            iterations: 0,
            phases: Vec::new(),
            evaluations: 0,
            enclosure: Enclosure::empty(self.mobility.len()),
        };
        let mut failed: Vec<usize> = Vec::new();
        for layer in 0..structure.layers() {
            let groups: Vec<&Group> = (structure.layer(layer).into_iter())
                .map(|g| &structure.groups[g])
                .collect();
            let base = values.clone();
            let (ended, evaluations) = self.solve_groups(start, &base, &groups, tolerance)?;
            solution.evaluations += evaluations;
            for (group, ended) in groups.into_iter().zip(ended) {
                let Some(ended) = ended else {
                    continue;
                };
                if !ended.holds && group.layer > 0 && !failed.contains(&group.component) {
                    failed.push(group.component);
                }
                solution.record(&mut values, group.clone(), ended);
            }
        }
        if !failed.is_empty() {
            // Solved again as one, from the start.
            solution.solved.retain(|g| !failed.contains(&g.component));
            let whole: Vec<&Group> = (failed.iter()).map(|&c| &structure.components[c]).collect();
            for group in &whole {
                for &b in &group.blocks {
                    values[b] = start[b].clone();
                }
            }
            let base = values.clone();
            let (ended, evaluations) = self.solve_groups(start, &base, &whole, tolerance)?;
            solution.evaluations += evaluations;
            for (group, ended) in whole.into_iter().zip(ended) {
                if let Some(ended) = ended {
                    solution.record(&mut values, group.clone(), ended);
                }
            }
        }
        solution.at_limit.sort_unstable();
        solution.at_limit.dedup();
        solution.enclosure = self.enclose_groups(&values, &solution.solved)?;
        solution.values = values;
        Ok(solution)
    }

    /// The groups `groups` — of one layer — solved in lockstep, every
    /// block outside them where `base` has it, from where the solve
    /// started at `start`: per group where it ended, if it moved; and how
    /// many evaluations that took.
    fn solve_groups(
        &self,
        start: &[E::Value],
        base: &[E::Value],
        groups: &[&Group],
        tolerance: Scalar,
    ) -> GeopResult<(Vec<Option<Ended<E::Value>>>, usize)> {
        let batch = Batch::new(self, base);
        let ended = batch.run(groups.iter().map(|&group| {
            let batch = &batch;
            async move {
                let solving = Solving::new(self, base, group, batch);
                let (first, carried) = solving.carried(start);
                let holds = solving.check(&first, &group.constraints, tolerance).await.1;
                // What its own freedom does, carried, is decided by where
                // it was held.
                let own = carried
                    && group.own > 0
                    && (group.blocks.iter()).any(|&b| self.mobility[b] == Mobility::Held);
                if group.preferences.is_empty() && holds && !own {
                    return Ok(carried.then_some(Ended {
                        values: first,
                        phases: Vec::new(),
                        active: Vec::new(),
                        holds,
                    }));
                }
                let mut active = Vec::new();
                loop {
                    let (solved, phases) = solving.solve(first.clone(), &active, &[]).await?;
                    let beyond = solving.beyond(&solved, &active, tolerance).await;
                    if beyond.is_empty() {
                        let holds = solving
                            .check(&solved, &group.constraints, tolerance)
                            .await
                            .1;
                        return Ok(Some(Ended {
                            values: solved,
                            phases,
                            active,
                            holds,
                        }));
                    }
                    active.extend(beyond);
                }
            }
        }))?;
        Ok((ended, batch.evaluations()))
    }

    /// An enclosure of a true solution near `at` — a solution to the
    /// solver's tolerance — group by group (see [`Enclosure`]).
    pub fn enclose(&self, at: &[E::Value]) -> GeopResult<Enclosure<E::Value>> {
        self.enclose_groups(at, &self.structure(at).groups)
    }

    /// `enclosure` — of the blocks at `at`, as [`Solution::enclosure`] is —
    /// with every group it does not cover enclosed too: what a solve did
    /// not move, where the caller needs every group (see [`System::enclose`]).
    pub fn enclose_rest(
        &self,
        at: &[E::Value],
        enclosure: Enclosure<E::Value>,
    ) -> GeopResult<Enclosure<E::Value>> {
        let covered: Vec<bool> = (0..self.mobility.len())
            .map(|b| {
                enclosure.boxes[b].is_some()
                    || (enclosure.failures.iter()).any(|(blocks, _)| blocks.contains(&b))
            })
            .collect();
        let rest: Vec<Group> = (self.structure(at).groups.into_iter())
            .filter(|group| !group.blocks.iter().all(|&b| covered[b]))
            .collect();
        self.enclose_into(at, &rest, enclosure)
    }

    /// How free the blocks are where they are at `at`: the directions in
    /// which every constraint that must vanish stays put to first order —
    /// the null space of their Jacobian — and of those, how many move each
    /// block. Decided over the box an enclosure of the solution near `at`
    /// proves, where one is: a redundant constraint, which looks
    /// independent at a point a hair off the solution, does not take a
    /// freedom away. Where none is proven, at `at`.
    pub fn freedom(&self, at: &[E::Value]) -> GeopResult<Freedom> {
        let enclosure = self.enclose(at)?;
        Ok(Freedom {
            blocks: enclosure.freedom,
            total: enclosure.total,
        })
    }

    /// A smallest set of the constraints that cannot all hold together —
    /// none where they all hold after a solve from `start`, by index.
    ///
    /// Group by group: one whose constraints do not hold after a solve is
    /// enclosed where it ended, and the conflict it proves names the
    /// constraints that cannot hold together with those they depend on (see
    /// [`EncloseError::Conflict`]). Where nothing is proven — a singular
    /// configuration — each constraint of the group is left out in turn,
    /// and kept out where the rest still cannot all hold: as many solves as
    /// there are constraints, so for a dialog to show, never for every drag.
    pub fn conflicting(&self, start: &[E::Value]) -> GeopResult<Vec<usize>> {
        let tolerance = tolerance(self.scale)?;
        let groups = self.structure(start).groups;
        let batch = Batch::new(self, start);
        let found = batch.run(groups.iter().map(|group| {
            let batch = &batch;
            async move { self.conflicting_in(start, group, batch, tolerance).await }
        }))?;
        let mut conflicting: Vec<usize> = found.into_iter().flatten().collect();
        conflicting.sort_unstable();
        conflicting.dedup();
        Ok(conflicting)
    }

    /// [`System::conflicting`] of the group `group`, from `start`.
    async fn conflicting_in(
        &self,
        start: &[E::Value],
        group: &Group,
        batch: &Batch<'_, '_, E>,
        tolerance: Scalar,
    ) -> GeopResult<Vec<usize>> {
        let solving = Solving::new(self, start, group, batch);
        let first = solving.start();
        if (solving.check(&first, &group.constraints, tolerance).await).1 {
            return Ok(Vec::new());
        }
        let holds = async |left_out: &[usize]| -> GeopResult<(Vec<E::Value>, bool)> {
            let (solved, _) = solving.solve(first.clone(), &[], left_out).await?;
            let kept: Vec<usize> = (group.constraints.iter())
                .filter(|c| !left_out.contains(c))
                .copied()
                .collect();
            let holds = solving.check(&solved, &kept, tolerance).await.1;
            Ok((solved, holds))
        };
        let (solved, all) = holds(&[]).await?;
        if all {
            return Ok(Vec::new());
        }
        let mut at = start.to_vec();
        for (&block, value) in group.blocks.iter().zip(&solved) {
            at[block] = value.clone();
        }
        let proven = self.enclose_group(&at, group, batch).await?;
        if let Err(EncloseError::Conflict {
            conflicting, with, ..
        }) = proven.boxes
        {
            return Ok(conflicting
                .into_iter()
                .map(|(c, _)| c)
                .chain(with)
                .collect());
        }
        let mut left_out = Vec::new();
        for &c in &group.constraints {
            left_out.push(c);
            if holds(&left_out).await?.1 {
                left_out.pop();
            }
        }
        Ok((group.constraints.iter())
            .filter(|c| !left_out.contains(c))
            .copied()
            .collect())
    }
}

/// Where a group's solve ended: its values, the phases that put them
/// there, the limits held at their bounds, and whether its constraints
/// hold there.
struct Ended<V> {
    values: Vec<V>,
    phases: Vec<Phase>,
    active: Vec<usize>,
    holds: bool,
}

impl<V: Variable> Solution<V> {
    /// `group` moved as `ended` says: its values written into `values`.
    fn record(&mut self, values: &mut [V], group: Group, ended: Ended<V>) {
        self.iterations = (self.iterations).max(ended.phases.iter().map(|p| p.iterations).sum());
        worst(&mut self.phases, ended.phases);
        for (&block, value) in group.blocks.iter().zip(ended.values) {
            values[block] = value;
        }
        self.at_limit.extend(ended.active);
        self.solved.push(group);
    }
}

/// `phases` with each of `group`'s phases merged into the one of its
/// index: the most steps, and the largest residual.
fn worst(phases: &mut Vec<Phase>, group: Vec<Phase>) {
    for (k, phase) in group.into_iter().enumerate() {
        match phases.get_mut(k) {
            None => phases.push(phase),
            Some(worst) => {
                if phase.iterations > worst.iterations {
                    worst.stop = phase.stop;
                    worst.iterations = phase.iterations;
                }
                if phase.max_residual.could_be_greater(worst.max_residual) {
                    worst.max_residual = phase.max_residual;
                }
            }
        }
    }
}

/// One group of a system being worked on: the blocks outside it as they
/// stay, and where its variables are.
struct Solving<'s, 'e, E: Equations> {
    system: &'s System<'e, E>,
    base: &'s [E::Value],
    group: &'s Group,
    /// Where its terms are asked, with every other group's.
    batch: &'s Batch<'s, 'e, E>,
    /// Per block of the group, where its variables start.
    columns: Vec<usize>,
    /// How many variables the group has.
    n: usize,
    space: Values<E::Value>,
}

impl<'s, 'e, E: Equations> Solving<'s, 'e, E> {
    fn new(
        system: &'s System<'e, E>,
        base: &'s [E::Value],
        group: &'s Group,
        batch: &'s Batch<'s, 'e, E>,
    ) -> Self {
        let mut columns = Vec::with_capacity(group.blocks.len());
        let mut n = 0;
        for &block in &group.blocks {
            columns.push(n);
            n += base[block].dimension();
        }
        Self {
            system,
            base,
            group,
            batch,
            columns,
            n,
            space: Values(std::marker::PhantomData),
        }
    }

    /// The group's values where it starts.
    fn start(&self) -> Vec<E::Value> {
        (self.group.blocks.iter())
            .map(|&b| self.base[b].clone())
            .collect()
    }

    /// Where a solve of the group starts: for a pendant, carried along by
    /// the first block it hangs off that moved since `start` and can carry
    /// it — and whether one did; otherwise where it is.
    fn carried(&self, start: &[E::Value]) -> (Vec<E::Value>, bool) {
        let mut carried = false;
        let values = (self.group.blocks.iter())
            .map(|&b| {
                let by = (self.group.on.iter()).find_map(|&s| {
                    (!self.base[s].same(&start[s]))
                        .then(|| self.base[b].carried(&start[s], &self.base[s]))
                        .flatten()
                });
                carried |= by.is_some();
                by.unwrap_or_else(|| self.base[b].clone())
            })
            .collect();
        (values, carried)
    }

    /// The terms `which` with the group at `values` (see
    /// [`Equations::evaluate`]), asked with every other group's.
    async fn made(&self, values: &[E::Value], which: &[Term]) -> Made {
        self.batch.ask(&self.group.blocks, values, which).await
    }

    /// Where the variables of `block` are among the group's, if it is one
    /// of its blocks.
    fn column(&self, block: usize) -> Option<usize> {
        let at = self.group.blocks.binary_search(&block).ok()?;
        self.columns.get(at).copied()
    }

    /// The residuals of `term` as rows over the group's variables — each
    /// block's derivatives where its variables are, each composed with
    /// `differential` (per block of the group, `None` for the identity);
    /// those of blocks outside the group left out.
    fn rows<'d>(
        &self,
        term: &Linearized,
        differential: &dyn Fn(usize) -> Option<&'d Vec<Vec<Scalar>>>,
    ) -> Rows {
        let mut rows = Rows::default();
        for (r, &value) in term.values.iter().enumerate() {
            let mut row = vec![Scalar::ZERO; self.n];
            let mut set = vec![false; self.n];
            for (block, derivatives) in &term.jacobian {
                let (Some(column), Some(local)) = (self.column(*block), derivatives.get(r)) else {
                    continue;
                };
                let own = match differential(*block) {
                    None => local.clone(),
                    Some(d) => compose(local, d),
                };
                for (k, v) in own.into_iter().enumerate() {
                    let at = column + k;
                    if at >= self.n {
                        break;
                    }
                    // A block read twice adds its derivatives; read once,
                    // they are its own, not a zero added to.
                    row[at] = match set[at] {
                        true => row[at].add(v),
                        false => v,
                    };
                    set[at] = true;
                }
            }
            rows.push(value, row);
        }
        rows
    }

    /// The constraints `which` with the group at `values`; `None` if one
    /// cannot be made there.
    async fn constraints_at(
        &self,
        values: &[E::Value],
        which: &[usize],
    ) -> Option<Vec<Linearized>> {
        let terms: Vec<Term> = which.iter().map(|&c| Term::Constraint(c)).collect();
        self.made(values, &terms).await.into_iter().collect()
    }

    /// The largest residual of the constraints `which` at `values`, and
    /// whether they all hold there.
    async fn check(
        &self,
        values: &[E::Value],
        which: &[usize],
        tolerance: Scalar,
    ) -> (Scalar, bool) {
        let Some(made) = self.constraints_at(values, which).await else {
            return (Scalar::INFINITY, false);
        };
        let holds = (which.iter().zip(&made)).all(|(&c, made)| {
            let bound = self.system.equations.bound(c);
            made.values.iter().all(|&r| bound.holds(r, tolerance))
        });
        (
            largest(made.iter().flat_map(|m| m.values.iter().copied())),
            holds,
        )
    }

    /// The limits of the group not held at their bounds that `values` is
    /// beyond.
    async fn beyond(&self, values: &[E::Value], active: &[usize], tolerance: Scalar) -> Vec<usize> {
        let limits: Vec<usize> = (self.group.constraints.iter())
            .filter(|c| !active.contains(c))
            .filter(|&&c| self.system.equations.bound(c) == Bound::AtMost)
            .copied()
            .collect();
        let Some(made) = self.constraints_at(values, &limits).await else {
            return Vec::new();
        };
        (limits.iter().zip(&made))
            .filter(|(_, made)| !(made.values.iter()).all(|&r| Bound::AtMost.holds(r, tolerance)))
            .map(|(&c, _)| c)
            .collect()
    }
}

/// The row `local` times the matrix `d`: a derivative by a block's steps
/// at a point, as one by the steps from where the retraction started.
fn compose(local: &[Scalar], d: &[Vec<Scalar>]) -> Vec<Scalar> {
    let columns = d.first().map_or(0, Vec::len);
    (0..columns)
        .map(|j| {
            let mut sum: Option<Scalar> = None;
            for (l, row) in local.iter().zip(d) {
                let Some(&e) = row.get(j) else {
                    continue;
                };
                let term = l.mul(e);
                sum = Some(match sum {
                    None => term,
                    Some(s) => s.add(term),
                });
            }
            sum.unwrap_or(Scalar::ZERO)
        })
        .collect()
}

#[cfg(test)]
mod tests;
