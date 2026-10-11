//! [`ParamValue`]: the values a program is built with — given to it, not
//! built by it — that its steps read: its parameters, as its
//! [`crate::parameters::Parameters`] resolve them, and its state, where its
//! placed parts are.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Scalar, as_f64},
    solvers::{least_squares::Rows, system::Variable},
};
use serde::{Deserialize, Serialize};

use crate::Design;

/// A parameter's value: a number, a pose, or text — a table's row, a
/// colour. Serialized as itself — `2.5`, `{"position": [...], "rotation":
/// [...]}`, `"M5"`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ParamValue {
    Number(#[serde(with = "as_f64")] Design),
    Pose(Pose<Design>),
    Text(String),
}

/// The name of the parameter that is where the part placed as `instance`
/// is: `bolt.pose`.
pub fn pose_parameter(instance: &str) -> String {
    format!("{instance}.pose")
}

/// A program's state is made of parameter values: where its placed parts
/// are, how its joints are turned. A value the run gives none of reads as
/// the number zero.
impl Default for ParamValue {
    fn default() -> Self {
        ParamValue::Number(Design::ZERO)
    }
}

/// A number steps by what is added to it; a pose by a twist `(ω, v)` in its
/// own frame — turning about its own origin, and moving along its own axes —
/// by the exponential map: `pose ∘ exp(ω, v)` (see [`Pose`]'s steps); text
/// does not move.
impl Variable for ParamValue {
    type Scalar = Design;

    fn dimension(&self) -> usize {
        match self {
            ParamValue::Number(v) => v.dimension(),
            ParamValue::Pose(pose) => pose.dimension(),
            ParamValue::Text(_) => 0,
        }
    }

    fn retract(&self, step: &[Design]) -> GeopResult<Self> {
        Ok(match self {
            ParamValue::Number(v) => ParamValue::Number(v.retract(step)?),
            ParamValue::Pose(pose) => ParamValue::Pose(pose.retract(step)?),
            ParamValue::Text(_) => self.unmoved(step)?,
        })
    }

    fn retract_over(&self, step: &[Design]) -> GeopResult<Self> {
        Ok(match self {
            ParamValue::Number(v) => ParamValue::Number(v.retract_over(step)?),
            ParamValue::Pose(pose) => ParamValue::Pose(pose.retract_over(step)?),
            ParamValue::Text(_) => self.unmoved(step)?,
        })
    }

    fn differential(&self, step: &[Design]) -> GeopResult<Option<Vec<Vec<Design>>>> {
        match self {
            ParamValue::Number(v) => v.differential(step),
            ParamValue::Pose(pose) => pose.differential(step),
            ParamValue::Text(_) => self.unmoved(step).map(|_| None),
        }
    }

    /// A number its difference from `at`; a pose its position's, and its
    /// rotation's — a turn as the distance it moves a point `scale` from
    /// its axis (see [`Pose`]'s).
    fn stay(&self, at: &Self, scale: Design) -> GeopResult<Rows<Design>> {
        match (self, at) {
            (ParamValue::Number(v), ParamValue::Number(at)) => v.stay(at, scale),
            (ParamValue::Pose(pose), ParamValue::Pose(at)) => pose.stay(at, scale),
            (ParamValue::Text(_), ParamValue::Text(_)) => Ok(Rows::default()),
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

impl geop_core_program::part::State for ParamValue {}

impl ParamValue {
    /// Text, which a step of no numbers leaves as it is.
    fn unmoved(&self, step: &[Design]) -> GeopResult<Self> {
        match step {
            [] => Ok(self.clone()),
            _ => Err(GeopError::new(format!(
                "a step of {self:?} takes no numbers, not {}",
                step.len()
            ))),
        }
    }
}
