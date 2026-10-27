//! [`Program`]: what a program is, and what a program file holds — its
//! steps, each an operation and its saved arguments, and its state: where
//! its placed parts are and how its joints are turned, what solving it
//! changes. Serialized as `{"steps": [...], "state": {...}}`.
//!
//! A step's id names everything the step builds, so ids are unique in a
//! program and spelled in an alphabet a name can be built from
//! ([`validate_operation_id`]).

use std::collections::HashSet;

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

use crate::{
    operation::{Operation, StepArgs, saved},
    operation_registry::OperationRegistry,
    part::Part,
    runner::Runner,
    values::StateValues,
};

/// One step of a program: which operation, with which arguments, as saved,
/// and whether it runs. The operation reads its arguments from their saved
/// form each time it runs or is edited, so a program is plain data: it is
/// copied for undo, compared for change, and saved as it is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ProgramStep {
    pub id: String,
    pub operation: String,
    pub args: serde_json::Value,
    /// Whether the step runs, as a formula of the program's parameters
    /// ([`crate::numeric::evaluate`]): it runs where the formula is not 0
    /// — always, for none. A step that does not run is suppressed: it
    /// produces nothing, and the steps after it see the part as the steps
    /// before it left it (see [`crate::runner::Runner::run`]). `"0"`
    /// suppresses it outright; a column of the table a program's
    /// configurations are looked up in, `configuration.fillet`, suppresses
    /// it in the configurations whose row has 0 there.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub active: Option<String>,
}

impl ProgramStep {
    /// The step `id` of the operation `args` are for.
    pub fn of<A: StepArgs>(id: impl Into<String>, args: &A) -> GeopResult<Self> {
        let id = id.into();
        let args = serde_json::to_value(args).map_err(|e| {
            GeopError::new(format!("arguments of operation `{}`: {e}", A::KIND))
                .with_context(format!("step `{id}`"))
        })?;
        Ok(Self {
            id,
            operation: A::KIND.to_string(),
            args,
            active: None,
        })
    }

    pub fn new<O: Operation>(id: impl Into<String>, args: &O::Args) -> GeopResult<Self> {
        let id = id.into();
        let args = saved::<O>(args).map_err(|e| e.with_context(format!("step `{id}`")))?;
        Ok(Self {
            id,
            operation: O::KIND.to_string(),
            args,
            active: None,
        })
    }
}

/// A program: its steps, and the state it was saved with — what it is run
/// with: what a solve moves (see [`Program::run`]).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct Program {
    #[serde(default)]
    pub steps: Vec<ProgramStep>,
    #[serde(default, skip_serializing_if = "StateValues::is_empty")]
    pub state: StateValues,
}

/// Checks that `id` can be an operation id: non-empty, and only ASCII
/// letters, digits, `_`, `-` and `.`. Nothing that could be mistaken for the
/// `(`, `)` and `,` that structure a name, so the names built from it stay
/// unambiguous.
pub fn validate_operation_id(id: &str) -> GeopResult<()> {
    if !id.is_empty()
        && id
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.'))
    {
        Ok(())
    } else {
        Err(GeopError::new(format!(
            "{id:?} is not a valid operation id: use only ASCII letters, digits, '_', '-' and '.'"
        )))
    }
}

impl Program {
    /// A program of no steps and no state.
    pub fn new() -> Self {
        Self::default()
    }

    /// The program of `steps`, with no state.
    pub fn from_steps(steps: Vec<ProgramStep>) -> Self {
        Self {
            steps,
            state: StateValues::new(),
        }
    }

    /// The program the file `text` holds, its ids checked (see
    /// [`Program::validate_ids`]). What its steps are is checked against
    /// the operations they are run in ([`Program::validate`]). An empty
    /// file, as one is when just made, is the empty program.
    pub fn from_json(text: &str) -> GeopResult<Self> {
        if text.trim().is_empty() {
            return Ok(Self::new());
        }
        let program: Self = serde_json::from_str(text)
            .map_err(|e| GeopError::new(format!("reading a program: {e}")))?;
        program.validate_ids()?;
        Ok(program)
    }

    pub fn to_json(&self) -> GeopResult<String> {
        serde_json::to_string_pretty(self)
            .map_err(|e| GeopError::new(format!("writing a program: {e}")))
    }

    /// The step `id`.
    pub fn step(&self, id: &str) -> GeopResult<&ProgramStep> {
        (self.steps.iter().find(|s| s.id == id)).ok_or_else(|| no_step(id))
    }

    pub fn step_mut(&mut self, id: &str) -> GeopResult<&mut ProgramStep> {
        (self.steps.iter_mut().find(|s| s.id == id)).ok_or_else(|| no_step(id))
    }

    /// The position of the step `id`.
    pub fn index_of(&self, id: &str) -> GeopResult<usize> {
        (self.steps.iter().position(|s| s.id == id)).ok_or_else(|| no_step(id))
    }

    /// Appends the step `id`, `args` for the operation they are for: for a
    /// program written in Rust, whose arguments are plain data.
    #[allow(
        clippy::expect_used,
        reason = "arguments that do not serialize are a bug of the program written"
    )]
    pub fn push<A: StepArgs>(&mut self, id: impl Into<String>, args: A) {
        let step = ProgramStep::of(id, &args).expect("a step's arguments are plain data");
        self.steps.push(step);
    }

    /// An id no step has yet, for a new step of an operation labelled
    /// `label`: the label, lowercased, and the lowest number that makes it
    /// unique — `sketch1`, `extrude2`.
    pub fn fresh_id(&self, label: &str) -> String {
        let base = label.to_lowercase().replace(' ', "_");
        (1..)
            .map(|n| format!("{base}{n}"))
            .find(|id| self.steps.iter().all(|s| &s.id != id))
            .unwrap_or(base)
    }

    /// Runs it all on `part`, which keeps what earlier runs built, by
    /// `runner` (see [`Runner::run`]). Fails naming the first step that
    /// does.
    pub fn run(&self, runner: &Runner, part: &mut Part) -> GeopResult<()> {
        let output = runner.run(part, self, None);
        match output.step_errors.into_iter().next() {
            Some((step, error)) => Err(error.with_context(format!("step {step:?}"))),
            None => Ok(()),
        }
    }

    /// The step `id` as a program of its own, as a copy of it is kept: the
    /// step, and the state stored under its id — where a placed part is.
    pub fn excerpt(&self, id: &str) -> GeopResult<Self> {
        let step = self.step(id)?.clone();
        let prefix = format!("{id}.");
        let state = (self.state.iter())
            .filter(|(name, _)| name.starts_with(&prefix))
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();
        Ok(Self {
            steps: vec![step],
            state,
        })
    }

    /// Inserts the steps of `pasted` at `index`, in order, with the state
    /// stored under their ids: each under its own id where no step has it
    /// yet, else under a fresh one — `label` naming a step's operation. The
    /// ids they got, in order. `pasted` is checked against `registry` first
    /// (see [`Program::validate`]).
    ///
    /// A step names what it builds on by the ids of the steps that built
    /// it, so a step pasted under a fresh id is not renamed in the steps
    /// pasted with it: they still build on the steps they were copied
    /// with.
    pub fn paste(
        &mut self,
        pasted: Self,
        index: usize,
        registry: &OperationRegistry,
        label: impl Fn(&ProgramStep) -> String,
    ) -> GeopResult<Vec<String>> {
        pasted
            .validate(registry)
            .map_err(|e| e.with_context("pasting a program"))?;
        if index > self.steps.len() {
            return Err(GeopError::new(format!(
                "cannot paste at {index}: the program has {} steps",
                self.steps.len()
            )));
        }
        let mut ids = Vec::with_capacity(pasted.steps.len());
        for (k, step) in pasted.steps.into_iter().enumerate() {
            let id = match self.steps.iter().any(|s| s.id == step.id) {
                true => self.fresh_id(&label(&step)),
                false => step.id.clone(),
            };
            let prefix = format!("{}.", step.id);
            for (name, value) in &pasted.state {
                if let Some(rest) = name.strip_prefix(&prefix) {
                    self.state.insert(format!("{id}.{rest}"), value.clone());
                }
            }
            self.steps.insert(
                index + k,
                ProgramStep {
                    id: id.clone(),
                    ..step
                },
            );
            ids.push(id);
        }
        Ok(ids)
    }

    /// Checks that every step id is a valid operation id
    /// ([`validate_operation_id`]) and unique: every name a step creates is
    /// built from its id.
    pub fn validate_ids(&self) -> GeopResult<()> {
        let mut ids = HashSet::new();
        for step in &self.steps {
            validate_operation_id(&step.id)?;
            if !ids.insert(step.id.as_str()) {
                return Err(GeopError::new(format!(
                    "the program has more than one step with id {:?}",
                    step.id
                )));
            }
        }
        Ok(())
    }

    /// Checks the program against the operations of `registry`, which it is
    /// run in: its ids (see [`Program::validate_ids`]), and that every step
    /// is of an operation there is, with arguments that operation reads.
    /// An error names the step.
    pub fn validate(&self, registry: &OperationRegistry) -> GeopResult<()> {
        self.validate_ids()?;
        for step in &self.steps {
            (registry.get(&step.operation))
                .and_then(|operation| operation.check_args(&step.args))
                .map_err(|e| e.with_context(format!("step `{}`", step.id)))?;
        }
        Ok(())
    }
}

fn no_step(id: &str) -> GeopError {
    GeopError::new(format!("the program has no step `{id}`"))
}
