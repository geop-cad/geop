//! A [`Part`]'s state: the values a program is built with — given to
//! it, not built by it — that its steps read, each declaring what it reads.
//!
//! A program's state is its parameters: where its placed parts are, and
//! the values its [`crate::parameters::Parameters`] resolve to. It is kept apart from its steps — its *structure* — so
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

/// Parameter values, by name.
pub type State = BTreeMap<String, ParamValue>;

/// The name of the parameter that is where the part placed as `instance`
/// is: `bolt.pose`.
pub fn pose_parameter(instance: &str) -> String {
    format!("{instance}.pose")
}

impl<S: geop_core_math::scalars::Scalar> Part<S> {
    /// The part, to be built with the parameter values `state`.
    pub fn with_state(mut self, state: State) -> Self {
        self.set_state(state);
        self
    }

    /// Sets the values the part is built with.
    pub(crate) fn set_state(&mut self, state: State) {
        self.store.set_state(state);
    }

    /// The value of the program input `name`, if there is one.
    pub fn input(&self, name: &str) -> Option<&ParamValue> {
        self.store.input(name)
    }

    /// The value of the pose parameter `name` — `default` if the part is
    /// built without one — declared one of the part's parameters. Fails for
    /// a value that is no pose, and for a parameter declared twice.
    pub fn pose_parameter(
        &mut self,
        name: &str,
        default: Pose<Design>,
    ) -> GeopResult<Pose<Design>> {
        let value = match self.store.input(name) {
            None => default,
            Some(ParamValue::Pose(pose)) => *pose,
            Some(other) => {
                return Err(GeopError::new(format!(
                    "the parameter {name:?} is a pose, but the program gives it {other:?}"
                )));
            }
        };
        if self
            .store
            .declared_mut()
            .insert(name.to_string(), ParamValue::Pose(value))
            .is_some()
        {
            return Err(GeopError::new(format!(
                "the parameter {name:?} is declared twice"
            )));
        }
        Ok(value)
    }

    /// The value of the formula `expression` — a plain number, or one
    /// reading the part's parameters by name (see
    /// [`crate::parameters::evaluate`]) — every parameter it reads declared
    /// read, so that a change to one rebuilds what read it.
    pub fn evaluate(&mut self, expression: &str) -> GeopResult<f64> {
        let mut read = Vec::new();
        let value = crate::parameters::evaluate(expression, |name| {
            let v = match self.store.input(name) {
                Some(ParamValue::Number(v)) => geop_core_math::scalars::Scalar::to_f64(*v),
                _ => return None,
            };
            read.push(name.to_string());
            Some(v)
        });
        for name in read {
            let value = self.store.input(&name).cloned();
            if let Some(value) = value {
                self.store.declared_mut().entry(name).or_insert(value);
            }
        }
        value
    }

    /// The parameters the part is defined with: what a program placing it
    /// can give other values.
    pub fn parameters(&self) -> &crate::parameters::Parameters {
        self.store.parameters()
    }

    /// The part, defined with `parameters`.
    pub fn with_parameters(mut self, parameters: crate::parameters::Parameters) -> Self {
        self.set_parameters(parameters);
        self
    }

    /// Defines the part with `parameters`.
    pub(crate) fn set_parameters(&mut self, parameters: crate::parameters::Parameters) {
        self.store.set_parameters(parameters);
    }

    /// The part's colour, `#rrggbb`, if it is given one.
    pub fn color(&self) -> Option<&str> {
        match self.store.input(crate::parameters::COLOR) {
            Some(ParamValue::Text(c)) => Some(c),
            _ => None,
        }
    }

    /// What the part is, in words, if it is given (see
    /// [`crate::parameters::Parameters::title`]).
    pub fn title(&self) -> Option<&str> {
        self.store.parameters().title.as_deref()
    }

    /// What the part, as built, is ordered as, if it is given: `ISO 4762
    /// M4x12` (see [`crate::parameters::Parameters::designate`]).
    pub fn designation(&self) -> Option<String> {
        self.store.parameters().designate(self.store.state())
    }

    /// What the part is made of, if it is given (see
    /// [`crate::parameters::Material`]).
    pub fn material(&self) -> Option<&crate::parameters::Material> {
        self.store.parameters().material.as_ref()
    }

    /// Every parameter the part's steps declared, with the value it was
    /// built with.
    pub fn declared(&self) -> &State {
        self.store.declared()
    }

    /// The values the part is built with, declared or not.
    pub fn state(&self) -> &State {
        self.store.state()
    }
}
