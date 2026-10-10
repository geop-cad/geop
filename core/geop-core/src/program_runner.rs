use std::{any::Any, collections::BTreeMap};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    constraint::Constraint,
    operation::{Edited, EditingPresentation, ErasedOperation, UserEvent},
    operation_registry::OperationRegistry,
    part::{Input, Part, State, View},
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

    /// The part as it was before the step at `step_index` of the last run;
    /// `program.steps.len()` for the finished part.
    pub fn view(&self, step_index: usize) -> View<'_, I, S> {
        self.part.view(step_index)
    }

    fn step<'a>(
        &self,
        program: &'a Program,
        step_index: usize,
    ) -> GeopResult<(&'a ProgramStep, &dyn ErasedOperation<I, S>)> {
        let step = program
            .steps
            .get(step_index)
            .ok_or_else(|| GeopError::new(format!("the program has no step {step_index}")))?;
        Ok((step, self.registry.get(step.operation)?))
    }

    /// The editing state a step starts with.
    pub fn initial_state(&self, program: &Program, step_index: usize) -> GeopResult<Box<dyn Any>> {
        Ok(self.step(program, step_index)?.1.initial_state())
    }

    /// What the editor shows while the step at `step_index` is edited.
    /// `program` is the program last run.
    pub fn presentation(
        &self,
        program: &Program,
        step_index: usize,
        state: &dyn Any,
    ) -> GeopResult<EditingPresentation> {
        let (step, operation) = self.step(program, step_index)?;
        operation.presentation(&self.view(step_index), &*step.args, state)
    }

    /// The arguments and editing state of the step at `step_index` after
    /// `event`. `program` is the program last run.
    pub fn process_user_event(
        &self,
        program: &Program,
        step_index: usize,
        state: &dyn Any,
        event: UserEvent,
    ) -> GeopResult<Edited> {
        let (step, operation) = self.step(program, step_index)?;
        operation.process_user_event(&self.view(step_index), &*step.args, state, event)
    }
}
