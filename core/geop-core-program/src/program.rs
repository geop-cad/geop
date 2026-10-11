use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

use crate::operation::{Operation, saved};

/// One step of a program: which operation, with which arguments, as saved.
/// The operation reads its arguments from their saved form each time it
/// runs or is edited, so a program is plain data: it is copied for undo,
/// compared for change, and saved as it is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProgramStep {
    pub id: String,
    pub operation: String,
    pub args: serde_json::Value,
}

impl ProgramStep {
    pub fn new<O: Operation>(id: impl Into<String>, args: &O::Args) -> GeopResult<Self> {
        let id = id.into();
        let args = saved::<O>(args).map_err(|e| e.with_context(format!("step `{id}`")))?;
        Ok(Self {
            id,
            operation: O::INFO.kind.to_string(),
            args,
        })
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Program {
    pub steps: Vec<ProgramStep>,
}

impl Program {
    pub fn new(steps: Vec<ProgramStep>) -> Self {
        Self { steps }
    }

    /// The step `id`.
    pub fn step(&self, id: &str) -> GeopResult<&ProgramStep> {
        (self.steps.iter().find(|s| s.id == id)).ok_or_else(|| no_step(id))
    }

    pub fn step_mut(&mut self, id: &str) -> GeopResult<&mut ProgramStep> {
        (self.steps.iter_mut().find(|s| s.id == id)).ok_or_else(|| no_step(id))
    }
}

fn no_step(id: &str) -> GeopError {
    GeopError::new(format!("the program has no step `{id}`"))
}
