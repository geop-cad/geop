//! Where placed parts are: what the step that places a part (see
//! [`geop_core_program::placed`]) says of it, as a target of its own
//! ([`define_instance`]). The program only builds the placed part; where
//! it goes is a value of the state, or a fixed motion after another
//! instance, composed outermost first ([`placement`]), and where that puts
//! it ([`compose`], [`located`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::Scalar,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::StepContext,
    placed::{INSTANCE_SEPARATOR, Located, instance_path},
    reader::Reader,
    target::{Target, TargetReference},
    values::{Design, Settings, StateValue, pose_in},
};

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
/// and the inputs its part was placed from — what a copy of it places
/// again, and what a list of the parts tells them apart by. What is in it
/// is its placed part (see [`geop_core_program::placed::PlacedPart`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Instance {
    pub placement: Placement,
    /// The file, as the program placing it names it.
    pub file: String,
    pub inputs: Settings,
}

/// Defined once, by the step placing it.
impl Target for Instance {}

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

/// Defines the instance `name` as `instance`. Its part is placed on its
/// own ([`geop_core_program::placed::place`]).
pub fn define_instance(ctx: &mut StepContext, name: &str, instance: &Instance) -> GeopResult<()> {
    ctx.produce(&instance_target(name), instance, |_, instance| {
        Ok(instance.clone())
    })
}

/// The instance `name` placed in the part `reader` reads, the read
/// recorded.
pub fn instance<'a>(reader: &Reader<'a>, name: &str) -> GeopResult<&'a Instance> {
    (reader.retrieve_element(&ElementReference::whole(instance_target(name))))
        .map_err(|e| e.with_context(format!("the placed part `{name}`")))
}

/// Every instance placed in the part `reader` reads — not in its
/// instances' parts — by name, in the order the steps placed their parts
/// ([`geop_core_program::reader::Reader::instances`]), each read recorded.
/// An instance whose part is placed must be defined: it is an error if it
/// is not.
pub fn instances<'a>(reader: &Reader<'a>) -> GeopResult<Vec<(&'a str, &'a Instance)>> {
    (reader.instances().into_iter())
        .map(|name| Ok((name, instance(reader, name)?)))
        .collect()
}

/// One factor of where an instance is, as [`placement`] gives it,
/// outermost first: the instance is where they all are, composed in order.
#[derive(Clone, Debug, PartialEq)]
pub enum Factor {
    /// The value of the state `state` of the part read — named as the part
    /// read names it, `/asm:b.pose` — which a solve moves, and by which a
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
/// names it, behind the instance's path, and read so ([`Reader::given_state`]):
/// the part's value where it gives one.
pub fn placement(reader: &Reader, path: &str) -> GeopResult<Vec<Factor>> {
    let ctx = |e: GeopError| e.with_context(format!("where the placed part {path:?} is"));
    let mut factors = Vec::new();
    let mut level = reader.clone();
    let mut outer = String::new();
    for name in path.split(INSTANCE_SEPARATOR) {
        factors.append(&mut placed(reader, &level, name, &outer, 0).map_err(ctx)?);
        outer = instance_path(&outer, name);
        level = level.placed_part(name).map_err(ctx)?;
    }
    Ok(factors)
}

/// The factors placing the instance `name` of the part `level` reads — the
/// part at `path` in the part `outer` reads — in that part: its state's
/// value, named behind `path` and read so in `outer`, or a copy's motion
/// after where what it copies is.
fn placed(
    outer: &Reader,
    level: &Reader,
    name: &str,
    path: &str,
    depth: usize,
) -> GeopResult<Vec<Factor>> {
    if depth > COPIES {
        return Err(GeopError::new(format!(
            "the copies of `{name}` form a cycle"
        )));
    }
    Ok(match &instance(level, name)?.placement {
        Placement::State(state) => {
            let state = Located::in_instance(path, state.as_str()).key();
            vec![Factor::State {
                value: outer.retrieve_state(&state),
                state,
            }]
        }
        Placement::After { of, motion } => {
            let mut factors = vec![Factor::Fixed(*motion)];
            factors.append(&mut placed(outer, level, of, path, depth + 1)?);
            factors
        }
    })
}

/// How the pose of each state a placement reads enters it, computed in
/// `T`: as it is, or moved by a step to differentiate by it. Given the
/// state's name and the pose it holds.
pub type PoseOf<'f, T> = dyn FnMut(&str, Pose<Design>) -> Pose<T> + 'f;

/// The pose an instance is at whose placement is `factors` (see
/// [`placement`]), in `T`: theirs,
/// composed outermost first, each state's entering as `pose_of` has it. A
/// value that holds no pose — a state before anything solved for it —
/// is no motion.
pub fn compose<T: Scalar>(factors: &[Factor], pose_of: &mut PoseOf<'_, T>) -> Pose<T> {
    let mut poses = factors.iter().map(|factor| match factor {
        Factor::State { state, value } => {
            pose_of(state, pose_in(value).unwrap_or_else(Pose::identity))
        }
        Factor::Fixed(motion) => motion.cast(),
    });
    // The first as it is: composing it after no motion would round it.
    let first = poses.next().unwrap_or_else(Pose::identity);
    poses.fold(first, |world, pose| world.compose(&pose))
}

/// Where the instance at `path` is in the part `reader` reads: none — no
/// motion — for the part itself, at the empty path.
pub fn located(reader: &Reader, path: &str) -> GeopResult<Pose<Design>> {
    if path.is_empty() {
        return Ok(Pose::identity());
    }
    Ok(compose(&placement(reader, path)?, &mut |_, pose| pose))
}
