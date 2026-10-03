//! A [`Part`]'s state: the values a program is built with — given to
//! it, not built by it — that its steps read, each declaring what it reads.
//!
//! A program's state is its parameters: where its placed parts are, later
//! dimensions too. It is kept apart from its steps — its *structure* — so
//! that every step sees one value of each, however late
//! in the program what fixes it is: the mates of a later step that move a
//! part placed earlier move it for every step (see
//! [`crate::program::solve`]).

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::as_f64,
};
use serde::{Deserialize, Serialize};

use super::Part;
use crate::Design;

/// A parameter's value: a number, or a pose. Serialized as itself — `2.5`,
/// `{"position": [...], "rotation": [...]}`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum ParamValue {
    Number(#[serde(with = "as_f64")] Design),
    Pose(Pose<Design>),
}

/// Parameter values, by name.
pub type State = BTreeMap<String, ParamValue>;

/// The name of the parameter that is where the part placed as `instance`
/// is: `bolt.pose`.
pub fn pose_parameter(instance: &str) -> String {
    format!("{instance}.pose")
}

impl<S: geop_core_math::scalars::Scalar> Part<S> {
    /// The part, to be built with the parameter values `inputs`.
    pub fn with_state(mut self, inputs: State) -> Self {
        self.inputs = inputs;
        self
    }

    /// The value of the pose parameter `name` — `default` if the part is
    /// built without one — declared one of the part's parameters. Fails for
    /// a value that is no pose, and for a parameter declared twice.
    pub fn pose_parameter(
        &mut self,
        name: &str,
        default: Pose<Design>,
    ) -> GeopResult<Pose<Design>> {
        let value = match self.inputs.get(name) {
            None => default,
            Some(ParamValue::Pose(pose)) => *pose,
            Some(ParamValue::Number(n)) => {
                return Err(GeopError::new(format!(
                    "the parameter {name:?} is a pose, but the program gives it the number {n:?}"
                )));
            }
        };
        if self
            .declared
            .insert(name.to_string(), ParamValue::Pose(value))
            .is_some()
        {
            return Err(GeopError::new(format!(
                "the parameter {name:?} is declared twice"
            )));
        }
        Ok(value)
    }

    /// Every parameter the part's steps declared, with the value it was
    /// built with.
    pub fn state(&self) -> &State {
        &self.declared
    }

    /// The values the part is built with, declared or not.
    pub fn inputs(&self) -> &State {
        &self.inputs
    }
}
