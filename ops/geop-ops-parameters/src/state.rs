//! [`StateValue`]: what a part's state holds — where its placed parts are,
//! how its joints are turned — the values a solve moves (see
//! `geop-ops-constraints`), kept with the program between runs.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Scalar, as_f64},
    solvers::{least_squares::Rows, system::Variable},
};
use serde::{Deserialize, Serialize};

use crate::Design;

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
