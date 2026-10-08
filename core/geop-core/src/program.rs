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
    pub program: Program,
    pub part: Part,
}

pub struct RunOutput {
    pub residuals: BTreeMap<String, Vec<f64>>, // Error in any constraints TODO: Make this Scalar trait.
    pub jacobian_state: BTreeMap<String, Vec<Vec<[f64; 6]>>>, // How each value in state affects the residuals // TODO: Make this Scalar trait.
}

impl ProgramRunner {
    pub fn new(operation_registry: BTreeMap<String, Box<dyn ErasedOperation>>) -> Self {
        Self {
            operation_registry,
            program: Program { steps: Vec::new() },
            part: Part::new(),
        }
    }

    pub fn run(
        &mut self,
        program: &Program,
        inputs: &BTreeMap<String, f64>,
        state: &BTreeMap<String, Box<Pose>>,
    ) -> Result<RunOutput, Box<dyn std::error::Error>> {
        self.part.clear_changed_dependencies();

        for (id, value) in inputs {
            self.part.update_input(id, *value);
        }

        for (id, value) in state {
            self.part.update_state(id, (**value).clone());
        }

        for step in &program.steps {
            // Implementation for processing each program step goes here
            self.operation_registry
                .get(&step.operation_name)
                .unwrap()
                .run(&step.id, &mut self.part, &*step.args)?;
        }

        Ok(RunOutput {
            residuals: self.part.get_residuals().clone(),
            jacobian_state: self.part.get_jacobian_state().clone(),
        })
    }
}
