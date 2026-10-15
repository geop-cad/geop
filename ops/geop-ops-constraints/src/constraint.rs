//! What steps require of the state: [`Constraint`]s, defined by name, and
//! all of a part's together ([`Constraints`]).
//!
//! A step defines its constraints with [`define_constraints`] in a target
//! of its own, made by its make and made again only when what that
//! read changes. A solve runs the program at every state it tries, and
//! gathers them all once per run ([`Constraints::of`]): no step builds on
//! the constraints of the steps before it, so a constraint that moves makes
//! no other step's again — which a target revised by every step would, at a
//! cost growing with the square of the steps.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    solvers::system::Variable,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    part::{Make, StepContext},
    placed::instance_path,
    reader::Reader,
    target::{Target, TargetReference},
    values::Design,
};

use crate::Generated;

pub use geop_core_math::solvers::system::Bound;

/// What an operation requires of the state: residuals that are zero — or,
/// for a limit, at most zero — where it holds, each a length, and how they
/// change as the values of the state they depend on move.
#[derive(Debug, Clone, PartialEq)]
pub struct Constraint {
    pub residuals: Vec<Design>,
    /// For each state the residuals depend on, one row per residual: its
    /// derivative by that state's step (see
    /// [`geop_core_math::solvers::system::Variable::retract`]), as many
    /// numbers as the state's dimension.
    pub jacobian: BTreeMap<String, Vec<Vec<Design>>>,
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
    pub fn holds(&self, tolerance: Design) -> bool {
        (self.residuals.iter()).all(|&r| self.bound.holds(r, tolerance))
    }
}

/// What the name of a step's constraints target starts with.
const CONSTRAINTS: &str = "constraints";

/// Every constraint the steps so far defined, by name, each with the step
/// that defined it: gathered from their targets ([`Constraints::of`]).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Constraints(BTreeMap<String, Defined>);

/// A constraint, and the step that defined it.
#[derive(Debug, Clone, PartialEq)]
struct Defined {
    step: String,
    constraint: Constraint,
}

impl Constraints {
    /// The constraints of the part `reader` reads: none where no step
    /// defined any. A step whose constraints failed fails them all: a
    /// solve must not go on without a constraint it could not make. A name
    /// two steps defined is an error naming both.
    ///
    /// Those of every instance placed in it come up behind the instance's
    /// name: a constraint `c` of the instance `bolt` is `bolt/c`, on the
    /// state `bolt/s` where the instance's is on `s` (see
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
                    .map(|(state, rows)| (instance_path(name, &state), rows))
                    .collect();
                let defined = Defined {
                    step: instance_path(name, &defined.step),
                    constraint: Constraint {
                        jacobian,
                        ..defined.constraint
                    },
                };
                all.0.insert(instance_path(name, &constraint), defined);
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

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// The constraints one step defined, by name: its target.
#[derive(Clone, PartialEq)]
struct StepConstraints {
    step: String,
    constraints: BTreeMap<String, Constraint>,
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
pub fn define_constraints<Args: PartialEq + Clone + 'static>(
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
fn make<Args>(reader: &Reader, (step, generated): &Defining<Args>) -> GeopResult<StepConstraints> {
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
    })
}
