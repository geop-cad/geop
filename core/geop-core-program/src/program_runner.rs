use std::{any::Any, collections::BTreeMap};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    editing::{Event, Form},
    operation::{Context, Edited, ErasedOperation},
    operation_registry::OperationRegistry,
    part::{Input, Part, State},
    program::{Program, ProgramStep},
    step_editor::Scene,
};

pub struct ProgramRunner<I, S: State> {
    pub registry: OperationRegistry<I, S>,
    pub part: Part<I, S>,
}

/// A step being edited: its arguments as saved, its operation, and what
/// it is edited against.
struct Editing<'a, I, S: State> {
    args: &'a serde_json::Value,
    operation: &'a dyn ErasedOperation<I, S>,
    context: Context<'a, I, S>,
}

/// How a run went. The constraints the steps put on the state, for a
/// solver to drive to zero, are the part's ([`Part::constraints`]).
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
        for step in run {
            let ran = self.part.begin_step(&step.id).and_then(|()| {
                let operation = self.registry.get(&step.operation)?;
                operation.run(&step.id, &mut self.part, &step.args)
            });
            if let Err(e) = ran {
                step_errors.push((step.id.clone(), e));
            }
        }
        self.part
            .end_run(later.iter().map(|step| step.id.as_str()).collect());

        RunOutput { step_errors }
    }

    /// The step `step` of `program` and its operation, and what it is
    /// edited against. Steps are found by id, and so is the view an editor
    /// reads them against ([`Part::view_before`]): a program edited since
    /// the last run still edits each step against its own view, or fails
    /// naming it.
    fn editing<'a>(&'a self, program: &'a Program, step: &'a str) -> GeopResult<Editing<'a, I, S>> {
        let program_step = program.step(step)?;
        let context = Context {
            step,
            before: self.part.view_before(step)?,
            built: self.part.view_after(step),
            scene: None,
        };
        Ok(Editing {
            args: &program_step.args,
            operation: self.registry.get(&program_step.operation)?,
            context,
        })
    }

    /// A new step of the operation `kind`, `id`, to insert before the step
    /// `before` of the last run, or after the last step it ran.
    pub fn new_step(&self, kind: &str, id: &str, before: Option<&str>) -> GeopResult<ProgramStep> {
        let operation = self.registry.get(kind)?;
        let before = match before {
            Some(step) => self.part.view_before(step)?,
            None => self.part.current(),
        };
        let context = Context {
            step: id,
            before,
            built: None,
            scene: None,
        };
        Ok(ProgramStep {
            id: id.to_string(),
            operation: operation.info().kind.to_string(),
            args: operation.new_args(&context)?,
        })
    }

    /// The editing state the step `step` starts with.
    pub fn initial_editing(&self, program: &Program, step: &str) -> GeopResult<Box<dyn Any>> {
        Ok(self.editing(program, step)?.operation.initial_editing())
    }

    /// What the editor shows while the step `step` is edited.
    pub fn presentation(
        &self,
        program: &Program,
        step: &str,
        editing: &dyn Any,
    ) -> GeopResult<Form<S::Scalar>> {
        let step = self.editing(program, step)?;
        (step.operation).presentation(&step.context, step.args, editing)
    }

    /// What a list of steps shows of the step `step`: its form, as a step
    /// just opened shows it, against the part before it — and never what
    /// it built, so that a list of every step asks nothing of what each
    /// built.
    pub fn listed(&self, program: &Program, step: &str) -> GeopResult<Form<S::Scalar>> {
        let mut step = self.editing(program, step)?;
        step.context.built = None;
        let editing = step.operation.initial_editing();
        (step.operation).presentation(&step.context, step.args, &*editing)
    }

    /// The arguments and editing state of the step `step` after `event`,
    /// with what the pointer is over found in `scene`, if one is given.
    pub fn process_user_event(
        &self,
        program: &Program,
        step: &str,
        editing: &dyn Any,
        event: Event<S::Scalar>,
        scene: Option<&dyn Scene<S::Scalar>>,
    ) -> GeopResult<Edited> {
        let mut step = self.editing(program, step)?;
        step.context.scene = scene;
        (step.operation).process_user_event(&step.context, step.args, editing, event)
    }
}
