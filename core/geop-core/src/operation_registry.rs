use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

use crate::{
    operation::{ErasedOperation, Operation},
    program::{Program, ProgramStep},
};

/// A step as a program is saved.
#[derive(Serialize, Deserialize)]
struct SavedStep {
    id: String,
    operation: String,
    args: serde_json::Value,
}

/// The operations a program may use, by name.
pub struct OperationRegistry<I, S>(BTreeMap<&'static str, Box<dyn ErasedOperation<I, S>>>);

impl<I, S> Default for OperationRegistry<I, S> {
    fn default() -> Self {
        Self(BTreeMap::new())
    }
}

impl<I, S> OperationRegistry<I, S> {
    /// Adds `operation`. Fails if an operation of its name is already there: a
    /// program could not tell the two apart.
    pub fn with<O: Operation<Input = I, State = S>>(mut self, operation: O) -> GeopResult<Self> {
        if self.0.contains_key(O::NAME) {
            return Err(GeopError::new(format!(
                "operation `{}` is registered twice",
                O::NAME
            )));
        }
        self.0.insert(O::NAME, Box::new(operation));
        Ok(self)
    }

    pub fn get(&self, name: &str) -> GeopResult<&dyn ErasedOperation<I, S>> {
        self.0
            .get(name)
            .map(Box::as_ref)
            .ok_or_else(|| GeopError::new(format!("unknown operation `{name}`")))
    }

    pub fn save(&self, program: &Program) -> GeopResult<serde_json::Value> {
        let steps = program
            .steps
            .iter()
            .map(|step| {
                Ok(SavedStep {
                    id: step.id.clone(),
                    operation: step.operation.to_string(),
                    args: self.get(step.operation)?.args_to_json(&*step.args)?,
                })
            })
            .collect::<GeopResult<Vec<_>>>()?;
        serde_json::to_value(steps).map_err(|e| GeopError::new(format!("saving program: {e}")))
    }

    pub fn load(&self, json: serde_json::Value) -> GeopResult<Program> {
        let steps: Vec<SavedStep> = serde_json::from_value(json)
            .map_err(|e| GeopError::new(format!("loading program: {e}")))?;
        let steps = steps
            .into_iter()
            .map(|step| {
                let context = |e: GeopError| e.with_context(format!("loading step `{}`", step.id));
                let operation = self.get(&step.operation).map_err(context)?;
                Ok(ProgramStep {
                    args: operation.args_from_json(step.args).map_err(context)?,
                    id: step.id,
                    operation: operation.name(),
                })
            })
            .collect::<GeopResult<_>>()?;
        Ok(Program { steps })
    }
}
