//! Solving the state a program's [`Constraint`]s are on: the program's side
//! of the constraint systems of [`geop_core_math::solvers::system`], which
//! solve, check, enclose and measure them — the same algorithms a sketch's
//! numbers are solved by.
//!
//! The values of the state a solve may move are the caller's to say
//! ([`Problem::free`]), each a block of the system, stepped along its own
//! steps ([`geop_core_math::solvers::system::Variable::retract`]: a pose by
//! the exponential map of a twist). The constraints are those the program
//! defines, run again at every state tried: what did not read the state
//! that changed is reused, not built again. What a solve
//! should prefer among the states the constraints allow is the caller's
//! too: [`Preference`]s pull — a drag — and every value nothing pulls stays
//! as near where it was as the constraints let it.
//!
//! What steps let a solve move is said in the program itself: a step
//! declares a value of the state ([`crate::unknown::Unknown`]), how a solve
//! may move it, and where one starts from — so a solve of any program is set
//! up from the part alone ([`crate::unknown::Unknowns`]).
//!
//! Every solve comes with an enclosure of a true solution near where it
//! ended ([`Solved::enclosure`]), each value in its own step coordinates
//! around the state found — or with why none is proven: a configuration
//! singular there, or constraints that conflict, by name.

use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
};

use geop_core_math::{
    geop_error::GeopResult,
    solvers::system::{self, Bound, Equations, Linearized, Phase, System, Term},
};

pub use geop_core_math::solvers::system::Mobility;

use geop_core_program::{
    part::{Part, Reader},
    program::Program,
    program_runner::ProgramRunner,
    values::{Design, Settings, StateValue, StateValues},
};

use crate::constraint::{Constraint, Constraints};

/// A program to run again at every state a solve tries, with its inputs.
pub struct Run<'r> {
    pub runner: &'r ProgramRunner,
    pub part: &'r mut Part,
    pub program: &'r Program,
    pub inputs: &'r Settings,
}

thread_local! {
    static RUNS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// How many times this thread has run a program for a question about its
/// constraints — a solve, an enclosure, how free its values are — ever.
/// Each run costs what the program's steps do, so a test can bound the
/// work a question takes by the runs it makes, rather than time it.
pub fn runs() -> usize {
    RUNS.get()
}

impl Run<'_> {
    /// The part the program builds at `state`.
    fn at(&mut self, state: &StateValues) -> &Part {
        RUNS.set(RUNS.get() + 1);
        self.runner.run(self.part, self.program, self.inputs, state);
        self.part
    }
}

/// What a solve should prefer among the states where the constraints
/// hold: residuals it wants small — not zero at any price — where the part
/// is as `reader` reads it, each a length, and their derivatives by the
/// state. A drag, pulling a point of a placed part towards the pointer.
pub trait Preference {
    fn evaluate(&self, reader: &Reader<'_>) -> GeopResult<Constraint>;
}

/// What a [`solve`] may move, and towards what.
pub struct Problem<'p> {
    /// The values the solve may move, by name, and how; it leaves every
    /// other value — and every one [`Mobility::Fixed`] — as it is.
    pub free: BTreeMap<String, Mobility>,
    pub preferences: Vec<&'p dyn Preference>,
    /// The part's size: what a constraint holds to a part in
    /// [`geop_core_math::solvers::system::RELATIVE_TOLERANCE`] of, how
    /// far one step may go, and what a turn is measured against when a
    /// value stays where it was.
    pub scale: Design,
}

/// How a solve went.
#[derive(Clone, Debug, PartialEq)]
pub struct Report<T: geop_core_math::scalars::Scalar> {
    /// Every constraint holds.
    pub converged: bool,
    /// The constraints that do not, by name.
    pub failed: Vec<String>,
    /// The limits the solve stopped at, by name.
    pub at_limit: Vec<String>,
    /// The values it moved, by name.
    pub moved: Vec<String>,
    /// The most steps any group took, and per phase the worst of the
    /// groups: why it stopped, after how many steps, the largest residual
    /// left.
    pub iterations: usize,
    pub phases: Vec<Phase<T>>,
    /// How many times the constraints were made: by the solve, and to
    /// enclose its solution — the work, which a test can bound.
    pub evaluations: [usize; 2],
}

/// An enclosure of a true solution of the constraints: every value it
/// could have, by name — or, for the values whose groups none is proven
/// for, why not.
#[derive(Clone, Debug, PartialEq)]
pub struct Enclosure {
    /// Per value enclosed, the box its true value is in.
    pub state: StateValues,
    /// Per group of values not proven, its values and why: the constraints
    /// that conflict there, by name, or what is singular.
    pub unproven: Vec<(Vec<String>, String)>,
}

/// Where a solve put the state, how it went, and the enclosure of a true
/// solution near there of every group it solved.
pub struct Solved {
    pub state: StateValues,
    pub report: Report<Design>,
    pub enclosure: Enclosure,
}

/// How free the values of the state are, where they are now (see
/// [`freedom`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Freedom {
    /// Per value a solve may move, how many independent ways it can still
    /// move, the others moving with it as the constraints need.
    pub states: BTreeMap<String, usize>,
    /// How many independent ways they can move together.
    pub total: usize,
}

/// A part's constraints and preferences, on the values a problem frees: the
/// [`Equations`] of its system. Each value a solve may move is a block, in
/// order of their names; each constraint is one by its name, in order.
struct Solving<'r, 'p> {
    run: RefCell<Run<'r>>,
    /// The whole state, as the program was last run with: the blocks'
    /// values written into it.
    working: RefCell<StateValues>,
    /// Per block, the value it is.
    blocks: Vec<String>,
    index: BTreeMap<String, usize>,
    /// Per constraint, its name and what it asks.
    constraints: Vec<(String, Bound)>,
    preferences: &'p [&'p dyn Preference],
}

impl<'r, 'p> Solving<'r, 'p> {
    /// The system of the program `run` runs, its state at `state`, the
    /// values `problem` frees as its blocks: and their values where `state`
    /// has them.
    fn new(
        mut run: Run<'r>,
        state: &StateValues,
        problem: &'p Problem<'p>,
    ) -> GeopResult<(Self, Vec<StateValue>)> {
        let constraints = (Constraints::of(&run.at(state).current())?.iter())
            .map(|(name, constraint)| (name.to_string(), constraint.bound))
            .collect();
        let blocks: Vec<String> = (problem.free.iter())
            .filter(|(_, mobility)| mobility.free())
            .map(|(name, _)| name.clone())
            .collect();
        let values = (blocks.iter())
            .map(|name| state.get(name).cloned().unwrap_or_default())
            .collect();
        let index = (blocks.iter().enumerate())
            .map(|(k, name)| (name.clone(), k))
            .collect();
        let solving = Solving {
            run: RefCell::new(run),
            working: RefCell::new(state.clone()),
            blocks,
            index,
            constraints,
            preferences: &problem.preferences,
        };
        Ok((solving, values))
    }

    /// The system: how each block may move, and its size.
    fn system<'s>(&'s self, problem: &Problem) -> System<'s, Self> {
        System {
            equations: self,
            mobility: (self.blocks.iter())
                .map(|name| problem.free.get(name).copied().unwrap_or(Mobility::Fixed))
                .collect(),
            scale: problem.scale,
        }
    }

    /// The names of the constraints `which`.
    fn names(&self, which: &[usize]) -> Vec<String> {
        (which.iter())
            .filter_map(|&c| self.constraints.get(c).map(|(name, _)| name.clone()))
            .collect()
    }

    /// The part, left as the program builds it at `state`.
    fn finish(self, state: &StateValues) {
        self.run.into_inner().at(state);
    }

    /// `constraint` as a term of the system: its derivatives by the
    /// blocks it reads.
    fn linearized(&self, constraint: &Constraint) -> Linearized<Design> {
        Linearized {
            values: constraint.residuals.clone(),
            jacobian: (constraint.jacobian.iter())
                .filter_map(|(state, rows)| Some((*self.index.get(state)?, rows.clone())))
                .collect(),
        }
    }
}

impl Equations for Solving<'_, '_> {
    type Value = StateValue;

    fn count(&self) -> (usize, usize) {
        (self.constraints.len(), self.preferences.len())
    }

    fn bound(&self, constraint: usize) -> Bound {
        self.constraints
            .get(constraint)
            .map_or(Bound::Zero, |c| c.1)
    }

    fn evaluate(&self, values: &[StateValue], which: &[Term]) -> Vec<Option<Linearized<Design>>> {
        {
            let mut working = self.working.borrow_mut();
            for (name, value) in self.blocks.iter().zip(values) {
                if working.get(name) != Some(value) {
                    working.insert(name.clone(), value.clone());
                }
            }
        }
        let mut run = self.run.borrow_mut();
        let reader = run.at(&self.working.borrow()).current();
        let made = Constraints::of(&reader);
        (which.iter())
            .map(|term| match term {
                Term::Constraint(c) => {
                    let (name, _) = self.constraints.get(*c)?;
                    Some(self.linearized(made.as_ref().ok()?.get(name)?))
                }
                Term::Preference(p) => {
                    let preference = self.preferences.get(*p)?.evaluate(&reader).ok()?;
                    Some(self.linearized(&preference))
                }
            })
            .collect()
    }
}

/// The enclosure `enclosure` of `program`'s system, by name.
fn named(program: &Solving, enclosure: system::Enclosure<StateValue>) -> Enclosure {
    let state = (program.blocks.iter().zip(enclosure.boxes))
        .filter_map(|(name, boxed)| Some((name.clone(), boxed?)))
        .collect();
    let unproven = (enclosure.failures.into_iter())
        .map(|(blocks, error)| {
            let values = (blocks.iter())
                .filter_map(|&b| program.blocks.get(b).cloned())
                .collect();
            let name =
                |c: usize| (program.constraints.get(c)).map_or(format!("#{c}"), |c| c.0.clone());
            (values, error.named(name).root_message().to_string())
        })
        .collect();
    Enclosure { state, unproven }
}

/// Where the values `problem` frees have to be — from where `state` has
/// them — for every constraint the program `run` runs defines to hold, as
/// near where they are as the constraints allow, pulled as its preferences
/// ask (see [`System::solve`]), and an enclosure of a true solution near
/// there. The program is run at each state tried, and the part is left as
/// it builds at the one found.
///
/// The state is moved even where the constraints cannot all hold, to where
/// they come closest; the report says which do not.
pub fn solve(run: Run<'_>, state: &StateValues, problem: &Problem) -> GeopResult<Solved> {
    let (program, start) = Solving::new(run, state, problem)?;
    let system = program.system(problem);
    let solution = system.solve(&start)?;
    let held = system.held(&solution.values)?;
    let mut solved = state.clone();
    let mut moved = Vec::new();
    for ((name, value), was) in program.blocks.iter().zip(&solution.values).zip(&start) {
        if value != was {
            moved.push(name.clone());
            solved.insert(name.clone(), value.clone());
        }
    }
    let report = Report {
        converged: held.converged,
        failed: program.names(&held.failed),
        at_limit: program.names(&solution.at_limit),
        moved,
        iterations: solution.iterations,
        phases: solution.phases,
        evaluations: [solution.evaluations, solution.enclosure.evaluations],
    };
    let enclosure = named(&program, solution.enclosure);
    program.finish(&solved);
    Ok(Solved {
        state: solved,
        report,
        enclosure,
    })
}

/// An enclosure of a true solution of the constraints of the program `run`
/// runs near where
/// `state` has the values `problem` frees — a solution to the solver's
/// tolerance (see [`System::enclose`]).
pub fn enclose(run: Run<'_>, state: &StateValues, problem: &Problem) -> GeopResult<Enclosure> {
    let (program, start) = Solving::new(run, state, problem)?;
    let enclosure = program.system(problem).enclose(&start)?;
    let enclosure = named(&program, enclosure);
    program.finish(state);
    Ok(enclosure)
}

/// How free the values `problem` frees are where `state` has them: the
/// directions in which every constraint that must vanish stays put to
/// first order, and of those, how many move each value — decided over the
/// box an enclosure of the solution proves (see
/// [`System::freedom`]). A limit leaves free the motion away from it.
pub fn freedom(run: Run<'_>, state: &StateValues, problem: &Problem) -> GeopResult<Freedom> {
    let (program, start) = Solving::new(run, state, problem)?;
    let freedom = program.system(problem).freedom(&start)?;
    let states = (program.blocks.iter().cloned())
        .zip(freedom.blocks)
        .collect();
    program.finish(state);
    Ok(Freedom {
        states,
        total: freedom.total,
    })
}

/// A smallest set of the constraints of the program `run` runs that cannot
/// all hold together —
/// none, where they all hold after a solve from `state` (see
/// [`System::conflicting`]), by name.
pub fn conflicting(
    run: Run<'_>,
    state: &StateValues,
    problem: &Problem,
) -> GeopResult<Vec<String>> {
    let (program, start) = Solving::new(run, state, problem)?;
    let conflicting = program.system(problem).conflicting(&start)?;
    let names: BTreeSet<String> = program.names(&conflicting).into_iter().collect();
    program.finish(state);
    Ok(names.into_iter().collect())
}
