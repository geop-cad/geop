use crate::{
    operation::ErasedOperation,
    part::{Part, Pose},
};
use std::collections::BTreeMap;

pub struct ProgramStep {
    pub id: String,
    pub operation_name: String,
    pub args: Box<dyn std::any::Any>,
}

pub struct Program {
    pub steps: Vec<ProgramStep>,
}

impl Program {
    pub fn new(steps: Vec<ProgramStep>) -> Self {
        Self { steps }
    }
}

pub struct ProgramRunner {
    pub operation_registry: BTreeMap<String, Box<dyn ErasedOperation>>,
    pub part: Part,
}

pub struct RunOutput {
    /// The steps that failed, in program order, with why. A failed step does
    /// not stop the run: later steps that do not depend on it still build.
    pub step_errors: Vec<(String, Box<dyn std::error::Error>)>,
    pub residuals: BTreeMap<String, Vec<f64>>, // Error in any constraints TODO: Make this Scalar trait.
    pub jacobian_state: BTreeMap<String, Vec<Vec<[f64; 6]>>>, // How each value in state affects the residuals // TODO: Make this Scalar trait.
}

impl ProgramRunner {
    pub fn new(operation_registry: BTreeMap<String, Box<dyn ErasedOperation>>) -> Self {
        Self {
            operation_registry,
            part: Part::default(),
        }
    }

    /// Runs `program` on the given `inputs` and `state`, which are complete:
    /// what a previous run gave and this one does not is removed.
    pub fn run(
        &mut self,
        program: &Program,
        inputs: &BTreeMap<String, f64>,
        state: &BTreeMap<String, Pose>,
    ) -> RunOutput {
        self.part.begin_run(inputs, state);
        let mut step_errors = Vec::new();
        for step in &program.steps {
            if let Err(e) = self.run_step(step) {
                step_errors.push((step.id.clone(), e));
            }
        }
        self.part.end_run();

        RunOutput {
            step_errors,
            residuals: self.part.get_residuals().clone(),
            jacobian_state: self.part.get_jacobian_state().clone(),
        }
    }

    fn run_step(&mut self, step: &ProgramStep) -> Result<(), Box<dyn std::error::Error>> {
        self.part.begin_step(&step.id);
        let operation = self
            .operation_registry
            .get(&step.operation_name)
            .ok_or_else(|| format!("unknown operation `{}`", step.operation_name))?;
        operation.run(&step.id, &mut self.part, &*step.args)
    }
}
