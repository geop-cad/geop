//! Which values of the state a solve may move, as the steps declare them:
//! [`Unknown`]s, each step's in a target of its own, and all of a part's
//! together ([`Unknowns`]), so that a solve of any program is set up from
//! the part alone.
//!
//! Each step's own, not one target every step adds to: a joint's
//! coordinate is seeded where its parts are, so a step's unknowns are made
//! again whenever its parts move, and a target revised by every step would
//! make every later step's again with it — copying all the unknowns before
//! it, at a cost growing with the square of the steps. A drag of a robot
//! arm of a thousand parts spent a third of its time there.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    solvers::system::Mobility,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    part::{Make, StepContext},
    placed::key_behind,
    reader::Reader,
    target::{Target, TargetReference},
    values::StateValue,
};

use crate::Generated;

/// A value of the state as a step declares it for solving: the state it is,
/// and how a solve may move it — or that it holds it fixed. A solve starts
/// it from `seed` where the state has none of it, and — given a constraint
/// `unless` — where that constraint does not hold either: a joint's
/// coordinate, measured from where its parts are unless the joint holds.
#[derive(Clone, Debug, PartialEq)]
pub struct Unknown {
    pub state: String,
    pub mobility: Mobility,
    pub seed: Option<StateValue>,
    pub unless: Option<String>,
}

/// What the name of a step's unknowns target starts with.
const UNKNOWNS: &str = "unknowns";

/// Every unknown the steps so far declared, by the state it is: gathered
/// from their targets ([`Unknowns::of`]).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Unknowns(BTreeMap<String, Unknown>);

impl Unknowns {
    /// The unknowns of the part `reader` reads: none where no step declared
    /// any, an error where a declaration failed.
    ///
    /// Those of every instance placed in it come up behind the instance's
    /// name, as its constraints do (see [`crate::Constraints::of`]), each
    /// starting where the instance's part has it, if it has it at all. A
    /// state the part holds fixed stays fixed.
    pub fn of(reader: &Reader) -> GeopResult<Self> {
        if let Some(failed) = reader.failed_targets_of::<StepUnknowns>().pop() {
            return Err(failed);
        }
        let mut all = Self::default();
        for own in reader.targets_of::<StepUnknowns>() {
            let own = reader.retrieve_element(&ElementReference::whole(own))?;
            for declared in &own.0 {
                all.declare(declared.clone());
            }
        }
        for name in reader.instances() {
            let inside = reader.placed_part(name)?;
            let declared = Self::of(&inside)
                .map_err(|e| e.with_context(format!("the unknowns of the placed part `{name}`")))?;
            for unknown in declared.0.into_values() {
                let seed = inside.given_state(&unknown.state).or(unknown.seed);
                all.declare(Unknown {
                    state: key_behind(name, &unknown.state),
                    seed,
                    unless: (unknown.unless).map(|unless| key_behind(name, &unless)),
                    ..unknown
                });
            }
        }
        Ok(all)
    }

    /// The unknown the state `state` is, if a step declared it.
    pub fn get(&self, state: &str) -> Option<&Unknown> {
        self.0.get(state)
    }

    /// Every unknown, in order of their states' names.
    pub fn iter(&self) -> impl Iterator<Item = &Unknown> {
        self.0.values()
    }

    /// Adds `declared`. Of several declarations of one state, one that
    /// holds it fixed wins, and otherwise the first made.
    fn declare(&mut self, declared: Unknown) {
        match self.0.get_mut(&declared.state) {
            None => {
                self.0.insert(declared.state.clone(), declared);
            }
            Some(kept) if declared.mobility == Mobility::Fixed => kept.mobility = Mobility::Fixed,
            Some(_) => {}
        }
    }
}

/// The unknowns one step declared, in the order it made them: its target.
#[derive(Clone, Debug, PartialEq)]
struct StepUnknowns(Vec<Unknown>);

/// Each step's own.
impl Target for StepUnknowns {}

/// One step's unknowns, read whole.
impl Kind for StepUnknowns {
    const NAME: &'static str = "step unknowns";
}

impl HasElements<StepUnknowns> for StepUnknowns {
    type Element<'a> = &'a StepUnknowns;

    fn element(&self, name: &str) -> GeopResult<&Self> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!(
                "a step's unknowns have no element `{name}`: they are read whole"
            ))),
        }
    }
}

/// Declares the unknowns `generator` makes of `args` in the step being
/// run, of the part's [`Unknowns`]: of several declarations of one state,
/// one that holds it fixed wins, and otherwise the first made. A plain
/// function, like a make ([`Make`]).
pub fn declare_unknowns<Args: PartialEq + Clone + std::fmt::Debug + 'static>(
    ctx: &mut StepContext,
    args: &Args,
    generator: Make<Args, Vec<Unknown>>,
) -> GeopResult<()> {
    let own = TargetReference::<StepUnknowns>::new(format!("{UNKNOWNS}:{}", ctx.step()));
    let generated = Generated {
        args: args.clone(),
        generator,
    };
    ctx.produce(&own, &generated, declare)
}

/// The unknowns `generated` makes.
fn declare<Args>(
    reader: &Reader,
    generated: &Generated<Args, Vec<Unknown>>,
) -> GeopResult<StepUnknowns> {
    Ok(StepUnknowns((generated.generator)(
        reader,
        &generated.args,
    )?))
}
