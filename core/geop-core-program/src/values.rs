//! The values a program is run with from outside: what a program placing
//! it sets its parameters to ([`Setting`]), and what its state holds
//! ([`StateValue`]): where its placed parts are and how its joints are
//! turned — what a solve moves, kept with the program between runs. Both
//! are saved in a program file, so they are the file format's, not an
//! operation's.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Scalar, as_f64},
    solvers::{least_squares::Rows, system::Variable},
};
use serde::{Deserialize, Serialize};

/// What a parameter is set to where a program places the part: a number,
/// or the name of a table's row, or a colour. Saved as itself — `2.5`,
/// `"M5"`, `"#ff8800"`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Setting {
    Number(f64),
    Text(String),
}

/// Parameter settings, by name: the inputs of a run.
pub type Settings = BTreeMap<String, Setting>;

/// A value of a part's state: a number or a pose. Serialized as itself —
/// `2.5`, `{"position": [...], "rotation": [...]}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum StateValue {
    Number(#[serde(with = "as_f64")] Scalar),
    Pose(Pose),
}

/// A part's state, by name.
pub type StateValues = BTreeMap<String, StateValue>;

/// A value the run gives none of reads as the number zero.
impl Default for StateValue {
    fn default() -> Self {
        StateValue::Number(Scalar::ZERO)
    }
}

/// The name of the state that is where the part placed as `instance` is:
/// `bolt.pose`.
pub fn pose_state(instance: &str) -> String {
    format!("{instance}.pose")
}

/// The pose a value holds, if it holds one: none for a number — what a
/// pose state reads as before anything solved for it.
pub fn pose_in(value: &StateValue) -> Option<Pose> {
    match value {
        StateValue::Pose(pose) => Some(*pose),
        StateValue::Number(_) => None,
    }
}

/// A number steps by what is added to it; a pose by a twist `(ω, v)` in its
/// own frame — turning about its own origin, and moving along its own axes —
/// by the exponential map: `pose ∘ exp(ω, v)` (see [`Pose`]'s steps).
impl Variable for StateValue {
    fn dimension(&self) -> usize {
        match self {
            StateValue::Number(v) => v.dimension(),
            StateValue::Pose(pose) => pose.dimension(),
        }
    }

    fn retract(&self, step: &[Scalar]) -> GeopResult<Self> {
        Ok(match self {
            StateValue::Number(v) => StateValue::Number(v.retract(step)?),
            StateValue::Pose(pose) => StateValue::Pose(pose.retract(step)?),
        })
    }

    fn retract_over(&self, step: &[Scalar]) -> GeopResult<Self> {
        Ok(match self {
            StateValue::Number(v) => StateValue::Number(v.retract_over(step)?),
            StateValue::Pose(pose) => StateValue::Pose(pose.retract_over(step)?),
        })
    }

    fn differential(&self, step: &[Scalar]) -> GeopResult<Option<Vec<Vec<Scalar>>>> {
        match self {
            StateValue::Number(v) => v.differential(step),
            StateValue::Pose(pose) => pose.differential(step),
        }
    }

    /// A number its difference from `at`; a pose its position's, and its
    /// rotation's — a turn as the distance it moves a point `scale` from
    /// its axis (see [`Pose`]'s).
    fn stay(&self, at: &Self, scale: Scalar) -> GeopResult<Rows> {
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

    /// A pose by a pose, as poses are (see [`Pose`]'s); a number by
    /// nothing.
    fn carried(&self, from: &Self, to: &Self) -> Option<Self> {
        match (self, from, to) {
            (StateValue::Pose(pose), StateValue::Pose(from), StateValue::Pose(to)) => {
                pose.carried(from, to).map(StateValue::Pose)
            }
            _ => None,
        }
    }
}
