//! Which values of the state a solve may move, as the steps declare them:
//! [`Unknown`]s, and the part's one target holding them all ([`Unknowns`]),
//! so that a solve of any program is set up from the part alone.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    solvers::system::Mobility,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    instance::instance_path,
    part::{Generator, Input, Part, Reader, State},
    target::TargetReference,
};

use crate::Generated;

/// A value of the state as a step declares it for solving: the state it is,
/// and how a solve may move it — or that it holds it fixed. A solve starts
/// it from `seed` where the state has none of it, and — given a constraint
/// `unless` — where that constraint does not hold either: a joint's
/// coordinate, measured from where its parts are unless the joint holds.
#[derive(Clone, Debug, PartialEq)]
pub struct Unknown<S: State> {
    pub state: String,
    pub mobility: Mobility,
    pub seed: Option<S>,
    pub unless: Option<String>,
}

/// The name of the part's [`Unknowns`] target.
const UNKNOWNS: &str = "unknowns";

/// Every unknown the steps so far declared, by the state it is. A part has
/// one, the target [`unknowns`], read whole: it is a kind of element of its
/// own (see [`HasElements`]).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Unknowns<S: State>(BTreeMap<String, Unknown<S>>);

impl<S: State> Unknowns<S> {
    /// The unknowns of the part `reader` reads: none where no step declared
    /// any, an error where a declaration failed.
    ///
    /// Those of every instance placed in it come up behind the instance's
    /// name, as its constraints do (see [`crate::Constraints::of`]), each
    /// starting where the instance's part has it, if it has it at all. A
    /// state the part holds fixed stays fixed.
    pub fn of<I: Input>(reader: &Reader<I, S>) -> GeopResult<Self> {
        let reference = unknowns::<S>();
        let mut all = match reader.defines(&reference) {
            true => reader
                .retrieve_element(&ElementReference::whole(reference))?
                .clone(),
            false => Self::default(),
        };
        for name in reader.subparts() {
            let inside = reader.inside(name)?;
            let declared = Self::of(&inside)
                .map_err(|e| e.with_context(format!("the unknowns of the placed part `{name}`")))?;
            for unknown in declared.0.into_values() {
                let seed = inside.given_state(&unknown.state).or(unknown.seed);
                all.declare(Unknown {
                    state: instance_path(name, &unknown.state),
                    seed,
                    unless: (unknown.unless).map(|unless| instance_path(name, &unless)),
                    ..unknown
                });
            }
        }
        Ok(all)
    }

    /// The unknown the state `state` is, if a step declared it.
    pub fn get(&self, state: &str) -> Option<&Unknown<S>> {
        self.0.get(state)
    }

    /// Every unknown, in order of their states' names.
    pub fn iter(&self) -> impl Iterator<Item = &Unknown<S>> {
        self.0.values()
    }

    /// Adds `declared`. Of several declarations of one state, one that
    /// holds it fixed wins, and otherwise the first made.
    fn declare(&mut self, declared: Unknown<S>) {
        match self.0.get_mut(&declared.state) {
            None => {
                self.0.insert(declared.state.clone(), declared);
            }
            Some(kept) if declared.mobility == Mobility::Fixed => kept.mobility = Mobility::Fixed,
            Some(_) => {}
        }
    }
}

/// The part's unknowns, read whole.
impl<S: State> Kind for Unknowns<S> {
    const NAME: &'static str = UNKNOWNS;
}

impl<S: State> HasElements<Unknowns<S>> for Unknowns<S> {
    type Element<'a> = &'a Unknowns<S>;
    const SINGLETON: Option<&'static str> = Some(UNKNOWNS);

    fn element(&self, name: &str) -> GeopResult<&Self> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!(
                "the unknowns have no element `{name}`: they are read whole"
            ))),
        }
    }
}

/// The target holding the part's unknowns.
pub fn unknowns<S: State>() -> TargetReference<Unknowns<S>> {
    TargetReference::new(UNKNOWNS)
}

/// Declares the unknowns `generator` makes of `args`, adding them to the
/// part's [`Unknowns`]: of several declarations of one state, one that
/// holds it fixed wins, and otherwise the first made. A plain function, like [`Part::define_target`]'s.
pub fn declare_unknowns<I: Input, S: State, Args: PartialEq + Clone + 'static>(
    part: &mut Part<I, S>,
    args: &Args,
    generator: Generator<I, S, Args, Vec<Unknown<S>>>,
) -> GeopResult<()> {
    let generated = Generated {
        args: args.clone(),
        generator,
    };
    part.revise_target(&unknowns(), &generated, declare)
}

/// `all` with the unknowns `generated` makes added.
fn declare<I: Input, S: State, Args>(
    reader: &Reader<I, S>,
    generated: &Generated<I, S, Args, Vec<Unknown<S>>>,
) -> GeopResult<Unknowns<S>> {
    let mut all: Unknowns<S> = reader.previous()?;
    for declared in (generated.generator)(reader, &generated.args)? {
        all.declare(declared);
    }
    Ok(all)
}
