//! Where the parts of instances are placed: what the step that loads a
//! part as an instance (see [`geop_core_program::instance`]) says of it,
//! as a target of its own ([`define_instance`]). The program only runs the
//! instance's part; where it goes is a value of the state, or a fixed
//! motion after another instance, composed outermost first
//! ([`placement`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    instance::INSTANCE_SEPARATOR,
    target::TargetReference,
};

use crate::{BuildPart, Design, PartReader, Settings, StateValue};

/// Where an instance is in the part it is in.
#[derive(Clone, Debug, PartialEq)]
pub enum Placement {
    /// Where the pose state `state` of the part puts it: moved by whatever
    /// solves the state.
    State(String),
    /// `motion` after where the instance `of` of the same part is: a copy,
    /// which goes with what it copies. Only where it is: what is in it is
    /// run for itself, with its own state.
    After { of: String, motion: Pose<Design> },
}

/// An instance placed, as its target holds it: where it is, and the file
/// and the inputs its part was loaded from — what a copy of it loads
/// again, and what a list of the parts tells them apart by. What is in it
/// is its part, which the part keeps (see
/// [`geop_core_program::program_runner::ProgramRunner::load_subpart`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Instance {
    pub placement: Placement,
    /// The file, as the program placing it names it.
    pub file: String,
    pub inputs: Settings,
}

/// Read whole.
impl Kind for Instance {
    const NAME: &'static str = "instance";
}

impl HasElements<Instance> for Instance {
    type Element<'a> = &'a Instance;

    fn element(&self, name: &str) -> GeopResult<&Instance> {
        whole(self, name)
    }
}

/// What the target of an instance is named, before the instance's name.
const PREFIX: &str = "instance:";

/// The target of the instance `name`.
fn instance_target(name: &str) -> TargetReference<Instance> {
    TargetReference::new(format!("{PREFIX}{name}"))
}

/// Defines the instance `name` as `instance`. Its part is loaded on its
/// own.
pub fn define_instance(part: &mut BuildPart, name: &str, instance: &Instance) -> GeopResult<()> {
    part.define_target(&instance_target(name), instance, |_, instance| Ok(instance.clone()))
}

/// The instance `name` placed in the part `reader` reads, the read
/// recorded.
pub fn instance<'a>(reader: &PartReader<'a>, name: &str) -> GeopResult<&'a Instance> {
    (reader.retrieve_element(&ElementReference::whole(instance_target(name))))
        .map_err(|e| e.with_context(format!("the placed part `{name}`")))
}

/// Every instance placed in the part `reader` reads — not in its
/// instances' parts — by name, in the order the steps loaded their parts
/// ([`geop_core_program::part::Reader::subparts`]), each read recorded. An
/// instance whose part is loaded must be placed: it is an error if it is
/// not.
pub fn instances<'a>(reader: &PartReader<'a>) -> GeopResult<Vec<(&'a str, &'a Instance)>> {
    (reader.subparts().into_iter())
        .map(|name| Ok((name, instance(reader, name)?)))
        .collect()
}

/// One factor of where an instance is, as [`placement`] gives it,
/// outermost first: the instance is where they all are, composed in order.
#[derive(Clone, Debug, PartialEq)]
pub enum Factor {
    /// The value of the state `state` of the part read — named as the part
    /// read names it, `asm/b.pose` — which a solve moves, and by which a
    /// constraint is differentiated.
    State { state: String, value: StateValue },
    /// A motion nothing in the part read moves: a copy's after what it
    /// copies.
    Fixed(Pose<Design>),
}

/// How deep a chain of copies may go before it is taken for a cycle.
const COPIES: usize = 1 << 16;

/// Where the instance at `path` — `bolt`, or `asm/bolt` — is in the part
/// `reader` reads, as factors composed outermost first: the values of the
/// state that place it and the parts it is in, read and recorded. A state
/// of an instance's own — what places a part in it — is named as the part
/// names it, behind the instance's path.
pub fn placement(reader: &PartReader, path: &str) -> GeopResult<Vec<Factor>> {
    let ctx = |e: GeopError| e.with_context(format!("where the placed part {path:?} is"));
    let mut factors = Vec::new();
    let mut level = reader.clone();
    let mut prefix = String::new();
    for name in path.split(INSTANCE_SEPARATOR) {
        factors.append(&mut placed(&level, name, &prefix, 0).map_err(ctx)?);
        prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
        level = level.inside(name).map_err(ctx)?;
    }
    Ok(factors)
}

/// The factors placing the instance `name` of the part `level` reads — the
/// part at `prefix` in the part asked — in that part: its state's value,
/// named behind `prefix`, or a copy's motion after where what it copies
/// is.
fn placed(level: &PartReader, name: &str, prefix: &str, depth: usize) -> GeopResult<Vec<Factor>> {
    if depth > COPIES {
        return Err(GeopError::new(format!(
            "the copies of `{name}` form a cycle"
        )));
    }
    Ok(match &instance(level, name)?.placement {
        Placement::State(state) => vec![Factor::State {
            value: level.retrieve_state(state),
            state: format!("{prefix}{state}"),
        }],
        Placement::After { of, motion } => {
            let mut factors = vec![Factor::Fixed(*motion)];
            factors.append(&mut placed(level, of, prefix, depth + 1)?);
            factors
        }
    })
}
