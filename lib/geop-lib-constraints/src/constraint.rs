//! What steps require of the state: [`Constraint`]s, defined by name, and
//! all of a part's together ([`Constraints`]).
//!
//! A step defines its constraints with [`define_constraints`] in a target
//! of its own, made by its make and made again only when what that
//! read changes. A solve gathers them all once ([`Constraints::of`]), and
//! at every state it tries makes only those it asks again, each by its own
//! step's generator, of the part as built and the state tried
//! ([`Remade`]): no program is run for it, and no constraint is
//! copied but those asked. No step builds on the constraints of the steps
//! before it, so a constraint that moves makes no other step's again —
//! which a target revised by every step would, at a cost growing with the
//! square of the steps.

use geop_core_math::scalars::Scalar;
use std::{any::Any, cell::RefCell, collections::BTreeMap, rc::Rc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    solvers::system::Variable,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    part::{Make, StepContext},
    placed::key_behind,
    reader::{Reader, Reads},
    target::{Target, TargetReference},
};

use crate::Generated;

pub use geop_core_math::solvers::system::Bound;

/// What an operation requires of the state: residuals that are zero — or,
/// for a limit, at most zero — where it holds, each a length, and how they
/// change as the values of the state they depend on move.
#[derive(Debug, Clone, PartialEq)]
pub struct Constraint {
    pub residuals: Vec<Scalar>,
    /// For each state the residuals depend on, one row per residual: its
    /// derivative by that state's step (see
    /// [`geop_core_math::solvers::system::Variable::retract`]), as many
    /// numbers as the state's dimension.
    pub jacobian: BTreeMap<String, Vec<Vec<Scalar>>>,
    pub bound: Bound,
}

impl Constraint {
    /// Checks that the jacobian has a row per residual, of as many numbers
    /// as a step of the state takes, for states the constraint read —
    /// `dimension` saying how many, `None` for one it did not read: a
    /// derivative by a state it never looked at cannot be right.
    pub(crate) fn check(
        &self,
        name: &str,
        dimension: impl Fn(&str) -> Option<usize>,
    ) -> GeopResult<()> {
        for (state, rows) in &self.jacobian {
            let Some(columns) = dimension(state) else {
                return Err(GeopError::new(format!(
                    "constraint `{name}` gives a derivative by state `{state}`, which it does not read"
                )));
            };
            if rows.len() != self.residuals.len() {
                return Err(GeopError::new(format!(
                    "constraint `{name}` has {} residuals but {} rows of derivatives by state `{state}`",
                    self.residuals.len(),
                    rows.len()
                )));
            }
            if let Some(row) = rows.iter().find(|row| row.len() != columns) {
                return Err(GeopError::new(format!(
                    "constraint `{name}` gives a derivative of {} numbers by state `{state}`, whose step takes {columns}",
                    row.len()
                )));
            }
        }
        Ok(())
    }

    /// Whether it holds: every residual within `tolerance` of zero — or,
    /// for a limit, not definitely above `tolerance`.
    pub fn holds(&self, tolerance: Scalar) -> bool {
        (self.residuals.iter()).all(|&r| self.bound.holds(r, tolerance))
    }
}

/// What the name of a step's constraints target starts with.
const CONSTRAINTS: &str = "constraints";

/// Every constraint the steps so far defined, by name, each with the step
/// that defined it: gathered from their targets ([`Constraints::of`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Constraints(BTreeMap<String, Defined>);

/// A constraint, the step that defined it, and where.
#[derive(Debug, Clone, PartialEq)]
struct Defined {
    step: String,
    constraint: Constraint,
    at: DefinedAt,
}

/// Where a constraint is defined: behind the instances `path`, outermost
/// first, by the step `step` of the part placed there, as `name` — what
/// makes it alone again (see [`Remade`]).
#[derive(Debug, Clone, PartialEq)]
pub struct DefinedAt {
    path: Vec<String>,
    step: String,
    name: String,
}

impl DefinedAt {
    /// Where its step's constraints are made, `reader` reading the part
    /// they are of: inside the instances of its path.
    fn inside<'a>(&self, reader: &Reader<'a>) -> GeopResult<Reader<'a>> {
        let mut inside = reader.clone();
        for instance in &self.path {
            inside = inside.placed_part(instance)?;
        }
        Ok(inside)
    }

    /// Its step's constraints made again by its generator, `after` reading
    /// the part it is of after the step, and what that read.
    fn make(&self, after: &Reader) -> Made {
        let own = TargetReference::<StepConstraints>::new(format!("{CONSTRAINTS}:{}", self.step));
        let own = match after.retrieve_element(&ElementReference::whole(own)) {
            Ok(own) => own,
            Err(e) => return (Reads::default(), Err(e)),
        };
        MADE_AGAIN.set(MADE_AGAIN.get() + 1);
        let (made, reads) = after.recorded(|reader| (own.again.make)(reader, &*own.again.defining));
        (reads, made)
    }
}

thread_local! {
    static MADE_AGAIN: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// How many times this thread has made a step's constraints again at a
/// state a solve tried ([`Remade::constraints`]), ever: what a solve's work
/// grows with, now that it runs no program for a state it tries, so a test
/// can bound it by a count rather than time it.
pub fn made_again() -> usize {
    MADE_AGAIN.get()
}

/// The constraints a solve made again at the states it tried
/// ([`Remade::constraints`]), by step, each with the state it read: a step
/// is made again only where a state it read changed. A solve asks a
/// pendant's constraints where it was carried, and again where its own
/// solve starts, from there; and where it ends, as its last step left it.
#[derive(Default)]
pub struct Remade(RefCell<BTreeMap<(Vec<String>, String), Made>>);

/// A step's constraints as made again, and what making them read.
type Made = (Reads, GeopResult<StepConstraints>);

impl Remade {
    /// The constraints `which` of the part `reader` reads, as
    /// [`Constraints::of`] names their states — behind their paths — each
    /// made again by its step's generator of what `reader` reads: at the
    /// state it tries ([`Reader::trying`]), the constraints there, without
    /// running the program again. What a solve reads at every state it
    /// tries. A step several of whose constraints are asked is made once,
    /// and one whose state read is as it was when last made, not at all.
    ///
    /// The make reads only the state and targets that read none — it
    /// refuses any other read ([`define_constraints`]) — so what it makes
    /// is what a run of the program at that state would, and only the
    /// state it read can change what it makes of the same part.
    ///
    /// Counted ([`made_again`]).
    pub fn constraints(
        &self,
        reader: &Reader,
        which: &[&DefinedAt],
    ) -> Vec<GeopResult<Constraint>> {
        let mut remade = self.0.borrow_mut();
        (which.iter())
            .map(|at| {
                let after = (at.inside(reader)).map(|inside| inside.after_step(&at.step))?;
                let key = (at.path.clone(), at.step.clone());
                let kept =
                    (remade.get(&key)).is_some_and(|(reads, _)| after.reads_same_state(reads));
                if !kept {
                    remade.insert(key.clone(), at.make(&after));
                }
                let (_, step) = remade.get(&key).ok_or_else(|| GeopError::new("made"))?;
                let step = (step.as_ref()).map_err(|e| GeopError::new(e.to_string()))?;
                let constraint = step.constraints.get(&at.name).ok_or_else(|| {
                    GeopError::new(format!(
                        "step `{}` defines no constraint `{}`",
                        at.step, at.name
                    ))
                })?;
                let mut constraint = constraint.clone();
                for instance in at.path.iter().rev() {
                    constraint.jacobian = (constraint.jacobian.into_iter())
                        .map(|(state, rows)| (key_behind(instance, &state), rows))
                        .collect();
                }
                Ok(constraint)
            })
            .collect()
    }
}

impl Constraints {
    /// The constraints of the part `reader` reads: none where no step
    /// defined any. A step whose constraints failed fails them all: a
    /// solve must not go on without a constraint it could not make. A name
    /// two steps defined is an error naming both.
    ///
    /// Those of every instance placed in it come up behind the instance's
    /// name: a constraint `c` of the instance `bolt` is `/bolt:c`, on the
    /// state `/bolt:s` where the instance's is on `s` (see
    /// [`geop_core_program::instance`]). A residual is the same wherever
    /// the instance is: it is a length within it.
    pub fn of(reader: &Reader) -> GeopResult<Self> {
        let mut all = Self::own(reader)?;
        for name in reader.instances() {
            let inside = (reader
                .placed_part(name)
                .and_then(|inside| Self::of(&inside)))
            .map_err(|e| e.with_context(format!("the constraints of the placed part `{name}`")))?;
            for (constraint, defined) in inside.0 {
                let jacobian = (defined.constraint.jacobian.into_iter())
                    .map(|(state, rows)| (key_behind(name, &state), rows))
                    .collect();
                let mut at = defined.at;
                at.path.insert(0, name.to_string());
                let defined = Defined {
                    step: key_behind(name, &defined.step),
                    constraint: Constraint {
                        jacobian,
                        ..defined.constraint
                    },
                    at,
                };
                all.0.insert(key_behind(name, &constraint), defined);
            }
        }
        Ok(all)
    }

    /// The constraints the part's own steps defined.
    fn own(reader: &Reader) -> GeopResult<Self> {
        if let Some(failed) = reader.failed_targets_of::<StepConstraints>().pop() {
            return Err(failed);
        }
        let mut all = Self::default();
        for own in reader.targets_of::<StepConstraints>() {
            let own = reader.retrieve_element(&ElementReference::whole(own))?;
            for (name, constraint) in &own.constraints {
                if let Some(earlier) = all.0.get(name) {
                    return Err(GeopError::new(format!(
                        "constraint `{name}` is defined twice: by step `{}` and by step `{}`",
                        earlier.step, own.step
                    )));
                }
                let defined = Defined {
                    step: own.step.clone(),
                    constraint: constraint.clone(),
                    at: DefinedAt {
                        path: Vec::new(),
                        step: own.step.clone(),
                        name: name.clone(),
                    },
                };
                all.0.insert(name.clone(), defined);
            }
        }
        Ok(all)
    }

    /// The constraint `name`, if a step defined it.
    pub fn get(&self, name: &str) -> Option<&Constraint> {
        self.0.get(name).map(|defined| &defined.constraint)
    }

    /// Every constraint, in order of their names.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Constraint)> {
        (self.0.iter()).map(|(name, defined)| (name.as_str(), &defined.constraint))
    }

    /// Where each is defined, in order of their names.
    pub fn defining(&self) -> impl Iterator<Item = (&str, &DefinedAt)> {
        (self.0.iter()).map(|(name, defined)| (name.as_str(), &defined.at))
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// The constraints one step defined, by name: its target — and how its
/// step makes them, to make them again at another state
/// ([`Remade`]).
#[derive(Clone, PartialEq)]
struct StepConstraints {
    step: String,
    constraints: BTreeMap<String, Constraint>,
    again: Again,
}

/// How a step makes its constraints: its arguments, a [`Defining`] of the
/// step's own arguments' type, and [`make`] of that type.
#[derive(Clone)]
struct Again {
    defining: Rc<dyn Any>,
    make: fn(&Reader, &dyn Any) -> GeopResult<StepConstraints>,
    equal: fn(&dyn Any, &dyn Any) -> bool,
}

/// Made from equal arguments by the same generator.
impl PartialEq for Again {
    fn eq(&self, other: &Self) -> bool {
        std::ptr::fn_addr_eq(self.equal, other.equal)
            && (self.equal)(&*self.defining, &*other.defining)
    }
}

/// [`make`] of the arguments `defining`, a [`Defining<Args>`].
fn make_any<Args: PartialEq + Clone + std::fmt::Debug + 'static>(
    reader: &Reader,
    defining: &dyn Any,
) -> GeopResult<StepConstraints> {
    let defining = (defining.downcast_ref::<Defining<Args>>())
        .ok_or_else(|| GeopError::new("a step's constraints made of another's arguments"))?;
    make(reader, defining)
}

/// Whether `a` and `b`, each a [`Defining<Args>`], are equal.
fn equal_any<Args: PartialEq + 'static>(a: &dyn Any, b: &dyn Any) -> bool {
    (a.downcast_ref::<Defining<Args>>())
        .is_some_and(|a| b.downcast_ref::<Defining<Args>>() == Some(a))
}

/// Each step's own.
impl Target for StepConstraints {}

/// One step's constraints, read whole.
impl Kind for StepConstraints {
    const NAME: &'static str = "step constraints";
}

impl HasElements<StepConstraints> for StepConstraints {
    type Element<'a> = &'a StepConstraints;

    fn element(&self, name: &str) -> GeopResult<&Self> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!(
                "the constraints of step `{}` have no element `{name}`: they are read whole",
                self.step
            ))),
        }
    }
}

/// What makes a step's constraints: its constraints, by name.
pub type ConstraintGenerator<Args> = Make<Args, BTreeMap<String, Constraint>>;

/// Defines the constraints `generator` makes of `args` in the step being
/// run, one of the part's [`Constraints`]. Each is checked: its
/// derivatives are by states its generator read, a row per residual, as
/// many numbers as a step of the state takes. A plain function, like a
/// make ([`Make`]).
///
/// A constraint whose residual reads a target that itself read the state —
/// geometry the state drives, a sketch following a placed part — is
/// refused, naming the constraint, the target and the state: a solve
/// differentiates a constraint by the states it reads, and could not see
/// how such a target moves with them.
///
/// A step that fails here leaves a failed target, which fails
/// [`Constraints::of`]: a solve must not go on without a constraint it
/// could not make.
pub fn define_constraints<Args: PartialEq + Clone + std::fmt::Debug + 'static>(
    ctx: &mut StepContext,
    args: &Args,
    generator: ConstraintGenerator<Args>,
) -> GeopResult<()> {
    let step = ctx.step();
    let own = TargetReference::<StepConstraints>::new(format!("{CONSTRAINTS}:{step}"));
    let generated = Generated {
        args: args.clone(),
        generator,
    };
    ctx.produce(&own, &(step.to_string(), generated), make)
}

/// The arguments of a step's own constraints: the step, and its generator
/// with the step's arguments.
type Defining<Args> = (String, Generated<Args, BTreeMap<String, Constraint>>);

/// The constraints of the step `step` its generator makes, checked.
fn make<Args: PartialEq + Clone + std::fmt::Debug + 'static>(
    reader: &Reader,
    defining: &Defining<Args>,
) -> GeopResult<StepConstraints> {
    let (step, generated) = defining;
    let constraints = (generated.generator)(reader, &generated.args)?;
    if let Some((target, state)) = reader.targets_reading_state().into_iter().next() {
        let names: Vec<&str> = constraints.keys().map(String::as_str).collect();
        return Err(GeopError::new(format!(
            "constraint `{}` reads target `{target}`, which itself reads the state `{state}`: a \
             solve could not see how it moves with the state — a constraint may read the state \
             only itself",
            names.join("`, `")
        )));
    }
    for (name, constraint) in &constraints {
        constraint.check(name, |state| {
            (reader.has_read_state(state)).then(|| reader.retrieve_state(state).dimension())
        })?;
    }
    Ok(StepConstraints {
        step: step.clone(),
        constraints,
        again: Again {
            defining: Rc::new(defining.clone()),
            make: make_any::<Args>,
            equal: equal_any::<Args>,
        },
    })
}
