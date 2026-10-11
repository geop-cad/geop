use std::{any::Any, collections::BTreeMap};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    constraint::Constraint,
    operation::{Edited, EditingPresentation, ErasedOperation, UserEvent},
    operation_registry::OperationRegistry,
    part::{Input, Part, State},
    program::{Program, ProgramStep},
};

pub struct ProgramRunner<I, S> {
    pub registry: OperationRegistry<I, S>,
    pub part: Part<I, S>,
}

pub struct RunOutput<S: State> {
    /// The steps that failed, in program order, with why. A failed step does
    /// not stop the run: later steps that do not depend on it still build.
    pub step_errors: Vec<(String, GeopError)>,
    /// The constraints the steps put on the state, by name, for a solver to
    /// drive to zero.
    pub constraints: BTreeMap<String, Constraint<S>>,
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
    ) -> RunOutput<S> {
        self.part.begin_run(inputs, state);
        let mut step_errors = Vec::new();
        for step in &program.steps {
            let ran = self.part.begin_step(&step.id).and_then(|()| {
                let operation = self.registry.get(step.operation)?;
                operation.run(&step.id, &mut self.part, &*step.args)
            });
            if let Err(e) = ran {
                step_errors.push((step.id.clone(), e));
            }
        }
        self.part.end_run();

        RunOutput {
            step_errors,
            constraints: (self.part.constraints().into_iter())
                .map(|(name, constraint)| (name.to_string(), constraint.clone()))
                .collect(),
        }
    }

    /// The step `step` of `program` and its operation. Steps are found by
    /// id, and so is the view an editor reads them against
    /// ([`Part::view_before`]): a program edited since the last run still
    /// edits each step against its own view, or fails naming it.
    fn step<'a>(
        &self,
        program: &'a Program,
        step: &str,
    ) -> GeopResult<(&'a ProgramStep, &dyn ErasedOperation<I, S>)> {
        let program_step = (program.steps.iter().find(|s| s.id == step))
            .ok_or_else(|| GeopError::new(format!("the program has no step `{step}`")))?;
        Ok((program_step, self.registry.get(program_step.operation)?))
    }

    /// The editing state the step `step` starts with.
    pub fn initial_state(&self, program: &Program, step: &str) -> GeopResult<Box<dyn Any>> {
        Ok(self.step(program, step)?.1.initial_state())
    }

    /// What the editor shows while the step `step` is edited.
    pub fn presentation(
        &self,
        program: &Program,
        step: &str,
        state: &dyn Any,
    ) -> GeopResult<EditingPresentation> {
        let (program_step, operation) = self.step(program, step)?;
        operation.presentation(&self.part.view_before(step)?, &*program_step.args, state)
    }

    /// The arguments and editing state of the step `step` after `event`.
    pub fn process_user_event(
        &self,
        program: &Program,
        step: &str,
        state: &dyn Any,
        event: UserEvent,
    ) -> GeopResult<Edited> {
        let (program_step, operation) = self.step(program, step)?;
        let view = self.part.view_before(step)?;
        operation.process_user_event(&view, &*program_step.args, state, event)
    }
}
