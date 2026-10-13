//! [`StateValue`]: what a part's state holds — where its placed parts are,
//! how its joints are turned — the values a solve moves (see
//! `geop-ops-constraints`), kept with the program between runs.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Scalar, as_f64},
    solvers::{least_squares::Rows, system::Variable},
};
use geop_core_program::instance::Factor;
use serde::{Deserialize, Serialize};

use crate::{Design, PartReader};

/// A value of a part's state: a number or a pose. Serialized as itself —
/// `2.5`, `{"position": [...], "rotation": [...]}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum StateValue {
    Number(#[serde(with = "as_f64")] Design),
    Pose(Pose<Design>),
}

/// The name of the state that is where the part placed as `instance` is:
/// `bolt.pose`.
pub fn pose_state(instance: &str) -> String {
    format!("{instance}.pose")
}

/// The pose a value holds, if it holds one: none for a number — what a
/// pose state reads as before anything solved for it.
pub fn pose_in(value: &StateValue) -> Option<Pose<Design>> {
    match value {
        StateValue::Pose(pose) => Some(*pose),
        StateValue::Number(_) => None,
    }
}

/// How the pose of each state a placement reads enters it, computed in
/// `T`: as it is, or moved by a step to differentiate by it. Given the
/// state's name and the pose it holds.
pub type PoseOf<'f, T> = dyn FnMut(&str, Pose<Design>) -> Pose<T> + 'f;

/// The pose an instance is at whose placement is `factors` (see
/// [`geop_core_program::part::Reader::placement`]), in `T`: theirs,
/// composed outermost first, each state's entering as `pose_of` has it. A
/// value that holds no pose — a state before anything solved for it —
/// is no motion.
pub fn compose<T: Scalar>(factors: &[Factor<StateValue>], pose_of: &mut PoseOf<'_, T>) -> Pose<T> {
    let mut poses = factors.iter().map(|factor| match factor {
        Factor::State { state, value } => {
            pose_of(state, pose_in(value).unwrap_or_else(Pose::identity))
        }
        Factor::Fixed(value) => pose_in(value).unwrap_or_else(Pose::identity).cast(),
    });
    // The first as it is: composing it after no motion would round it.
    let first = poses.next().unwrap_or_else(Pose::identity);
    poses.fold(first, |world, pose| world.compose(&pose))
}

/// Where the instance at `path` is in the part `reader` reads: none — no
/// motion — for the part itself, at the empty path.
pub fn located(reader: &PartReader, path: &str) -> GeopResult<Pose<Design>> {
    if path.is_empty() {
        return Ok(Pose::identity());
    }
    Ok(compose(&reader.placement(path)?, &mut |_, pose| pose))
}

/// A value the run gives none of reads as the number zero.
impl Default for StateValue {
    fn default() -> Self {
        StateValue::Number(Design::ZERO)
    }
}

/// A number steps by what is added to it; a pose by a twist `(ω, v)` in its
/// own frame — turning about its own origin, and moving along its own axes —
/// by the exponential map: `pose ∘ exp(ω, v)` (see [`Pose`]'s steps).
impl Variable for StateValue {
    type Scalar = Design;

    fn dimension(&self) -> usize {
        match self {
            StateValue::Number(v) => v.dimension(),
            StateValue::Pose(pose) => pose.dimension(),
        }
    }

    fn retract(&self, step: &[Design]) -> GeopResult<Self> {
        Ok(match self {
            StateValue::Number(v) => StateValue::Number(v.retract(step)?),
            StateValue::Pose(pose) => StateValue::Pose(pose.retract(step)?),
        })
    }

    fn retract_over(&self, step: &[Design]) -> GeopResult<Self> {
        Ok(match self {
            StateValue::Number(v) => StateValue::Number(v.retract_over(step)?),
            StateValue::Pose(pose) => StateValue::Pose(pose.retract_over(step)?),
        })
    }

    fn differential(&self, step: &[Design]) -> GeopResult<Option<Vec<Vec<Design>>>> {
        match self {
            StateValue::Number(v) => v.differential(step),
            StateValue::Pose(pose) => pose.differential(step),
        }
    }

    /// A number its difference from `at`; a pose its position's, and its
    /// rotation's — a turn as the distance it moves a point `scale` from
    /// its axis (see [`Pose`]'s).
    fn stay(&self, at: &Self, scale: Design) -> GeopResult<Rows<Design>> {
        match (self, at) {
            (StateValue::Number(v), StateValue::Number(at)) => v.stay(at, scale),
            (StateValue::Pose(pose), StateValue::Pose(at)) => pose.stay(at, scale),
            _ => Err(GeopError::new(format!(
                "{self:?} cannot stay at {at:?}: they are values of different kinds"
            ))),
        }
    }

    /// The same enclosure: what tells a step that changed nothing.
    fn same(&self, other: &Self) -> bool {
        self == other
    }
}
