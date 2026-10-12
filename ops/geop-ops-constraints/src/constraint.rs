//! What steps require of the state: [`Constraint`]s, defined by name, and
//! the part's one target holding them all ([`Constraints`]).
//!
//! A step defines its constraints with [`define_constraints`] in two
//! targets: its own constraints, made by its generator and rebuilt only
//! when what that read changes, and its revision of [`Constraints`], which
//! adds them to those of the steps before. A solve runs the program at every
//! state it tries; the split keeps a step's generator from running again
//! only because a constraint of an earlier step moved.

use std::{borrow::Cow, collections::BTreeMap};

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    part::{Generator, Input, Part, Reader},
    target::TargetReference,
};

use crate::{Generated, Solvable};

pub use geop_core_math::solvers::system::Bound;

/// What an operation requires of the state: residuals that are zero — or,
/// for a limit, at most zero — where it holds, each a length, and how they
/// change as the values of the state they depend on move.
#[derive(Debug, Clone, PartialEq)]
pub struct Constraint<S: Solvable> {
    pub residuals: Vec<S::Scalar>,
    /// For each state the residuals depend on, one row per residual: its
    /// derivative by that state's step (see
    /// [`geop_core_math::solvers::system::Variable::retract`]), as many
    /// numbers as the state's dimension.
    pub jacobian: BTreeMap<String, Vec<Vec<S::Scalar>>>,
    pub bound: Bound,
}

impl<S: Solvable> Constraint<S> {
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
    pub fn holds(&self, tolerance: S::Scalar) -> bool {
        (self.residuals.iter()).all(|&r| self.bound.holds(r, tolerance))
    }
}

/// The name of the part's [`Constraints`] target.
const CONSTRAINTS: &str = "constraints";

/// Every constraint the steps so far defined, by name, each with the step
/// that defined it. A part has one, the target [`constraints`], read whole:
/// it is a kind of element of its own (see [`HasElements`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Constraints<S: Solvable>(BTreeMap<String, Defined<S>>);

/// A constraint, and the step that defined it.
#[derive(Debug, Clone, PartialEq)]
struct Defined<S: Solvable> {
    step: String,
    constraint: Constraint<S>,
}

impl<S: Solvable> Constraints<S> {
    /// The constraints of the part `reader` reads: none where no step
    /// defined any, an error where one failed — every step after it fails
    /// too, since each revises what the one before made.
    pub fn of<'a, I: Input>(reader: &Reader<'a, I, S>) -> GeopResult<Cow<'a, Self>> {
        let reference = constraints::<S>();
        if !reader.defines(&reference) {
            return Ok(Cow::Owned(Self::default()));
        }
        (reader.retrieve_element(&ElementReference::whole(reference))).map(Cow::Borrowed)
    }

    /// The constraint `name`, if a step defined it.
    pub fn get(&self, name: &str) -> Option<&Constraint<S>> {
        self.0.get(name).map(|defined| &defined.constraint)
    }

    /// Every constraint, in order of their names.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Constraint<S>)> {
        (self.0.iter()).map(|(name, defined)| (name.as_str(), &defined.constraint))
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// The part's constraints, read whole.
impl<S: Solvable> Kind for Constraints<S> {
    const NAME: &'static str = CONSTRAINTS;
}

impl<S: Solvable> HasElements<Constraints<S>> for Constraints<S> {
    type Element<'a> = &'a Constraints<S>;
    const SINGLETON: Option<&'static str> = Some(CONSTRAINTS);

    fn element(&self, name: &str) -> GeopResult<&Self> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!(
                "the constraints have no element `{name}`: they are read whole"
            ))),
        }
    }
}

/// The target holding the part's constraints.
pub fn constraints<S: Solvable>() -> TargetReference<Constraints<S>> {
    TargetReference::new(CONSTRAINTS)
}

/// The constraints one step defined, by name: the target its revision of
/// [`Constraints`] adds to those before.
#[derive(Clone, PartialEq)]
struct StepConstraints<S: Solvable> {
    step: String,
    constraints: BTreeMap<String, Constraint<S>>,
}

/// One step's constraints, read whole.
impl<S: Solvable> Kind for StepConstraints<S> {
    const NAME: &'static str = "step constraints";
}

impl<S: Solvable> HasElements<StepConstraints<S>> for StepConstraints<S> {
    type Element<'a> = &'a StepConstraints<S>;

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

/// What a step's generator of constraints makes: its constraints, by name.
pub type ConstraintGenerator<I, S, Args> = Generator<I, S, Args, BTreeMap<String, Constraint<S>>>;

/// Defines the constraints `generator` makes of `args` in the step `step`,
/// and adds them to the part's [`Constraints`]. Each is checked: its
/// derivatives are by states its generator read, a row per residual, as
/// many numbers as a step of the state takes. A name an earlier step
/// defined is an error naming both steps. A plain function, like
/// [`Part::define_target`]'s.
///
/// A step that fails here still revises [`Constraints`], into a failure:
/// a solve must not go on without a constraint it could not make.
pub fn define_constraints<I: Input, S: Solvable, Args: PartialEq + Clone + 'static>(
    part: &mut Part<I, S>,
    step: &str,
    args: &Args,
    generator: ConstraintGenerator<I, S, Args>,
) -> GeopResult<()> {
    let own = TargetReference::<StepConstraints<S>>::new(format!("{CONSTRAINTS}:{step}"));
    let generated = Generated {
        args: args.clone(),
        generator,
    };
    let defined = part.define_target(&own, &(step.to_string(), generated), make);
    let added = part.revise_target(&constraints(), &own, add);
    defined.and(added)
}

/// The arguments of a step's own constraints: the step, and its generator
/// with the step's arguments.
type Defining<I, S, Args> = (
    String,
    Generated<I, S, Args, BTreeMap<String, Constraint<S>>>,
);

/// The constraints of the step `step` its generator makes, checked.
fn make<I: Input, S: Solvable, Args>(
    reader: &Reader<I, S>,
    (step, generated): &Defining<I, S, Args>,
) -> GeopResult<StepConstraints<S>> {
    let constraints = (generated.generator)(reader, &generated.args)?;
    for (name, constraint) in &constraints {
        constraint.check(name, |state| {
            (reader.has_read_state(state)).then(|| reader.retrieve_state(state).dimension())
        })?;
    }
    Ok(StepConstraints {
        step: step.clone(),
        constraints,
    })
}

/// `all` with the constraints of one step, `own`, added.
fn add<I: Input, S: Solvable>(
    reader: &Reader<I, S>,
    own: &TargetReference<StepConstraints<S>>,
) -> GeopResult<Constraints<S>> {
    let mut all: Constraints<S> = reader.previous()?;
    let own = reader.retrieve_element(&ElementReference::whole(own.clone()))?;
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
        };
        all.0.insert(name.clone(), defined);
    }
    Ok(all)
}
