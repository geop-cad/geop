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

/// The scalar a program holds its design data in.
pub type Design = geop_core_math::scalars::ScalInF64;

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
    Number(#[serde(with = "as_f64")] Design),
    Pose(Pose<Design>),
}

/// A part's state, by name.
pub type StateValues = BTreeMap<String, StateValue>;

/// A value the run gives none of reads as the number zero.
impl Default for StateValue {
    fn default() -> Self {
        StateValue::Number(Design::ZERO)
    }
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
