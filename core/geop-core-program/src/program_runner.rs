use std::collections::{BTreeMap, HashSet};

use geop_core_math::geop_error::GeopError;

use crate::{
    operation_registry::OperationRegistry,
    part::{Input, Part, State},
    program::Program,
};

pub struct ProgramRunner<I, S: State> {
    pub registry: OperationRegistry<I, S>,
    pub part: Part<I, S>,
}

/// How a run went.
pub struct RunOutput {
    /// The steps that failed, in program order, with why. A failed step does
    /// not stop the run: later steps that do not depend on it still build.
    pub step_errors: Vec<(String, GeopError)>,
}

impl<I: Input, S: State> ProgramRunner<I, S> {
    pub fn new(registry: OperationRegistry<I, S>) -> Self {
        Self {
            registry,
            part: Part::default(),
        }
    }

    /// Runs `program` on the given `inputs` and `state`, which are complete:
    /// what a previous run gave and this one does not is removed.
    pub fn run(
        &mut self,
        program: &Program,
        inputs: &BTreeMap<String, I>,
        state: &BTreeMap<String, S>,
    ) -> RunOutput {
        self.run_until(program, program.steps.len(), inputs, state)
    }

    /// Runs the first `end` steps of `program`, as the editor does up to its
    /// marker. What the later steps made is kept, to be reused when they
    /// run again.
    pub fn run_until(
        &mut self,
        program: &Program,
        end: usize,
        inputs: &BTreeMap<String, I>,
        state: &BTreeMap<String, S>,
    ) -> RunOutput {
        let (run, later) = program.steps.split_at(end.min(program.steps.len()));
        self.part.begin_run(inputs, state);
        let mut step_errors = Vec::new();
        let mut ids = HashSet::new();
        for step in run {
            self.part.begin_step();
            let ran = match ids.insert(step.id.as_str()) {
                true => (self.registry.get(&step.operation))
                    .and_then(|operation| operation.run(&step.id, &mut self.part, &step.args)),
                // A step's id names what it builds: two steps of one id
                // could not be told apart.
                false => Err(GeopError::new(format!(
                    "step `{}` appears twice in the program",
                    step.id
                ))),
            };
            if let Err(e) = ran {
                step_errors.push((step.id.clone(), e));
            }
        }
        self.part.end_run(later.len());

        RunOutput { step_errors }
    }
}
