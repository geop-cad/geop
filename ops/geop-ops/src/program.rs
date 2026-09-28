//! [`Program`]: an ordered list of operations that builds a [`Part`]; the
//! edits it can undergo ([`ProgramEdit`]); and [`ProgramRunner`], which
//! builds it incrementally.
//!
//! All three are generic over the set of operations the program can use
//! (see [`Operations`]): which operations those are is for an application
//! to decide.
//!
//! Editing lives here, not in any editor, so that every editor — the
//! browser UI, a future desktop one, a script — changes programs the same
//! way and is only a more convenient way of writing them.

use std::collections::HashSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use serde::{Deserialize, Serialize};

use crate::{
    Part,
    operation::{Dialog, Handle, Operations},
    validate_operation_id,
};

/// One step of a [`Program`]: an operation with its arguments, and the id
/// everything it creates is named after. Serializes as
/// `{"id": "box", "operation": "extrude", "args": {...}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Step<O> {
    pub id: String,
    #[serde(flatten)]
    pub operation: O,
}

/// A recipe for building a [`Part`]: an ordered list of steps, each referring
/// to what earlier ones built only by name. Those names come from step ids
/// and sketch element ids, never from the internal ids a build happens to
/// assign (see `geop_ops`), so a program means the same thing every
/// time it is run — including after a round trip through JSON.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Program<O> {
    pub steps: Vec<Step<O>>,
}

impl<O> Default for Program<O> {
    fn default() -> Self {
        Self { steps: Vec::new() }
    }
}

/// A change to a [`Program`]. Every edit of a program — whoever makes it —
/// is one of these, applied by [`Program::update`].
///
/// Steps are addressed by id, not position, so an edit means the same thing
/// however the steps around it have moved. Serializes as, e.g.,
/// `{"edit": "update", "id": "box", "operation": "extrude", "args": {...}}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "edit", rename_all = "snake_case")]
pub enum ProgramEdit<O> {
    /// Insert `operation` as a new step at position `index` (the end, if it
    /// is the number of steps), with the id `id` — or, if that is `None`, a
    /// fresh one derived from the operation (see [`Program::fresh_id`]).
    Insert {
        index: usize,
        #[serde(default)]
        id: Option<String>,
        #[serde(flatten)]
        operation: O,
    },
    /// Give step `id` a new operation or new arguments, in place.
    Update {
        id: String,
        #[serde(flatten)]
        operation: O,
    },
    /// Remove step `id`. Steps that referred to what it built fail from then
    /// on, until they are edited — the program is left as the user made it.
    Remove { id: String },
    /// Move step `id` to position `index` among the remaining steps.
    Move { id: String, index: usize },
    /// Replace the whole program, e.g. with one loaded from a file.
    Replace { program: Program<O> },
}

impl<O: Operations> ProgramEdit<O> {
    /// What the edit does, in a few words — without the arguments, which
    /// can be a whole sketch.
    pub fn summary(&self) -> String {
        match self {
            ProgramEdit::Insert {
                index, operation, ..
            } => format!("insert a {} step at {index}", operation.kind()),
            ProgramEdit::Update { id, operation } => {
                format!("update step {id:?} to a {} step", operation.kind())
            }
            ProgramEdit::Remove { id } => format!("remove step {id:?}"),
            ProgramEdit::Move { id, index } => format!("move step {id:?} to {index}"),
            ProgramEdit::Replace { program } => {
                format!(
                    "replace the program by one of {} steps",
                    program.steps.len()
                )
            }
        }
    }
}

impl<O: Operations> Program<O> {
    pub fn new() -> Self {
        Self::default()
    }

    /// Appends the step `id`: `operation` with its arguments.
    pub fn push(&mut self, id: impl Into<String>, operation: impl Into<O>) {
        self.steps.push(Step {
            id: id.into(),
            operation: operation.into(),
        });
    }

    /// The position of step `id`.
    pub fn index_of(&self, id: &str) -> GeopResult<usize> {
        self.steps
            .iter()
            .position(|s| s.id == id)
            .ok_or_else(|| GeopError::new(format!("program has no step {id:?}")))
    }

    /// An id no step has yet, for a new step running `operation`: its
    /// label, lowercased, and the lowest number that makes it unique —
    /// `sketch1`, `extrude2`.
    pub fn fresh_id(&self, operation: &O) -> String {
        let base = operation.label().to_lowercase().replace(' ', "_");
        (1..)
            .map(|n| format!("{base}{n}"))
            .find(|id| self.steps.iter().all(|s| &s.id != id))
            .expect("some number is free")
    }

    /// Checks that every step id is a valid operation id and unique: every
    /// name a step creates is built from its id.
    pub fn validate(&self) -> GeopResult<()> {
        let mut ids = HashSet::new();
        for step in &self.steps {
            validate_operation_id(&step.id)?;
            if !ids.insert(step.id.as_str()) {
                return Err(GeopError::new(format!(
                    "program has more than one step with id {:?}",
                    step.id
                )));
            }
        }
        Ok(())
    }

    /// Applies `edit`, returning the id of the step it inserted, changed or
    /// moved (`None` for a removal or a replacement). An edit that would
    /// leave the program invalid — an unknown step, a position past the end,
    /// a duplicate or malformed id — is rejected and changes nothing.
    ///
    /// This only changes the recipe; whether the steps still build is for
    /// running it to say (see [`ProgramRunner`]).
    pub fn update(&mut self, edit: ProgramEdit<O>) -> GeopResult<Option<String>> {
        let summary = edit.summary();
        let ctx = with_context!("Program::update({summary})");
        let mut next = self.clone();
        let changed = match edit {
            ProgramEdit::Insert {
                index,
                id,
                operation,
            } => {
                if index > next.steps.len() {
                    return Err(GeopError::new(format!(
                        "cannot insert at {index}: the program has {} steps",
                        next.steps.len()
                    )))
                    .with_context(ctx);
                }
                let id = id.unwrap_or_else(|| next.fresh_id(&operation));
                next.steps.insert(
                    index,
                    Step {
                        id: id.clone(),
                        operation,
                    },
                );
                Some(id)
            }
            ProgramEdit::Update { id, operation } => {
                let index = next.index_of(&id).with_context(ctx)?;
                next.steps[index].operation = operation;
                Some(id)
            }
            ProgramEdit::Remove { id } => {
                let index = next.index_of(&id).with_context(ctx)?;
                next.steps.remove(index);
                None
            }
            ProgramEdit::Move { id, index } => {
                let from = next.index_of(&id).with_context(ctx)?;
                let step = next.steps.remove(from);
                if index > next.steps.len() {
                    return Err(GeopError::new(format!(
                        "cannot move to {index}: the program has {} other steps",
                        next.steps.len()
                    )))
                    .with_context(ctx);
                }
                next.steps.insert(index, step);
                Some(id)
            }
            ProgramEdit::Replace { program } => {
                next = program;
                None
            }
        };
        next.validate().with_context(ctx)?;
        *self = next;
        Ok(changed)
    }

    /// Runs every step in order, starting from `part` (typically
    /// [`Part::new`]), and returns the part the whole program builds — or
    /// the first error any step raises, at which point the steps after it
    /// never run.
    ///
    /// After each step, every entity of the part must have a name — an
    /// operation that leaves one unnamed has broken the one guarantee a
    /// program relies on.
    pub fn apply<S: Scalar>(&self, part: Part<S>) -> GeopResult<Part<S>> {
        self.validate()?;
        let mut part = part;
        for (index, step) in self.steps.iter().enumerate() {
            part = run_step(part, index, step)?;
        }
        Ok(part)
    }

    /// The program as pretty-printed JSON: one step per object, every sketch
    /// entity keyed by its id, so edits show up as small line diffs.
    pub fn to_json(&self) -> GeopResult<String> {
        serde_json::to_string_pretty(self)
            .map_err(|e| GeopError::new(format!("serializing program: {e}")))
    }

    pub fn from_json(json: &str) -> GeopResult<Self> {
        let program: Self = serde_json::from_str(json)
            .map_err(|e| GeopError::new(format!("reading program: {e}")))?;
        program.validate()?;
        Ok(program)
    }
}

/// Step `index` of a program applied to `part`, with every name checked.
fn run_step<S: Scalar, O: Operations>(
    part: Part<S>,
    index: usize,
    step: &Step<O>,
) -> GeopResult<Part<S>> {
    let ctx = with_context!("program step {index} ({:?})", step.id);
    let part = step.operation.apply(part, &step.id).with_context(ctx)?;
    part.check_names().with_context(ctx)?;
    Ok(part)
}

/// A handle (see [`Handle`]) of the step `step`.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepHandle {
    pub step: String,
    #[serde(flatten)]
    pub handle: Handle,
}

/// The dialog (see [`Dialog`]) of the step `step`.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepDialog {
    pub step: String,
    #[serde(flatten)]
    pub dialog: Dialog,
}

/// How one step of a run went.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepResult {
    pub id: String,
    /// Why the step failed; `None` if it succeeded.
    pub error: Option<String>,
}

/// Builds a program the way an editor needs it built: incrementally, and
/// only as far as asked.
///
/// It keeps the part after every step it has run. Running again after an
/// edit reuses the part after the longest unchanged prefix of steps, so
/// changing the last step replays one step, not the whole history. And a
/// run can stop early — while a step in the middle is being edited, only
/// the steps up to it need to run, however long the rest of the program is.
/// Parts past the stop are kept, not discarded, so moving the stop back
/// again costs nothing.
///
/// A run stops at the first step that fails: the steps after it would only
/// fail too, for want of what it should have built.
pub struct ProgramRunner<S: Scalar, O> {
    /// The steps the cache was built from.
    steps: Vec<Step<O>>,
    /// `parts[i]`: the part after `steps[..i]`. A failed step leaves the
    /// part as it was, so this stays one longer than `steps`.
    parts: Vec<Part<S>>,
    results: Vec<StepResult>,
    /// How many steps the last run covers.
    ran: usize,
}

impl<S: Scalar, O: Operations> ProgramRunner<S, O> {
    pub fn new() -> Self {
        Self {
            steps: Vec::new(),
            parts: vec![Part::new()],
            results: Vec::new(),
            ran: 0,
        }
    }

    /// Runs the first `stop` steps of `program` — all of them if `None` —
    /// reusing whatever the previous runs built that still applies. See
    /// [`ProgramRunner::part`] and [`ProgramRunner::results`] for the
    /// outcome.
    pub fn run(&mut self, program: &Program<O>, stop: Option<usize>) {
        let common = self
            .steps
            .iter()
            .zip(&program.steps)
            .take_while(|(a, b)| a == b)
            .count();
        self.steps.truncate(common);
        self.parts.truncate(common + 1);
        self.results.truncate(common);

        let target = stop.unwrap_or(program.steps.len()).min(program.steps.len());
        let failed = |results: &[StepResult]| results.iter().any(|r| r.error.is_some());
        while self.steps.len() < target && !failed(&self.results) {
            let index = self.steps.len();
            let step = &program.steps[index];
            let before = self.parts.last().expect("parts is never empty");
            let (part, error) = match run_step(before.clone(), index, step) {
                Ok(part) => (part, None),
                Err(e) => (before.clone(), Some(e.to_string())),
            };
            self.steps.push(step.clone());
            self.parts.push(part);
            self.results.push(StepResult {
                id: step.id.clone(),
                error,
            });
        }
        // Up to the stop, or up to and including the first failure.
        let first_failure = self.results.iter().position(|r| r.error.is_some());
        self.ran = match first_failure {
            Some(f) if f < target => f + 1,
            _ => target.min(self.steps.len()),
        };
    }

    /// The part the last run built.
    pub fn part(&self) -> &Part<S> {
        &self.parts[self.ran]
    }

    /// One result per step the last run covered.
    pub fn results(&self) -> &[StepResult] {
        &self.results[..self.ran]
    }

    /// Every handle of every step the last run built — all of them, of
    /// every group: which to offer is an editor's choice. Each is placed
    /// with the part as its step saw it.
    pub fn handles(&self) -> GeopResult<Vec<StepHandle>> {
        let mut handles = Vec::new();
        for (i, (step, result)) in self.steps.iter().zip(self.results()).enumerate() {
            if result.error.is_some() {
                continue;
            }
            let ctx = with_context!("handles of step {i} ({:?})", step.id);
            for handle in step.operation.handles(&self.parts[i]).with_context(ctx)? {
                handles.push(StepHandle {
                    step: step.id.clone(),
                    handle,
                });
            }
        }
        Ok(handles)
    }

    /// The dialog of every step the last run covered, the one that failed
    /// included: a dialog is what helps fix a step's arguments. Each is
    /// made with the part as its step saw it.
    pub fn dialogs(&self) -> Vec<StepDialog> {
        self.steps
            .iter()
            .zip(self.results())
            .enumerate()
            .map(|(i, (step, _))| StepDialog {
                step: step.id.clone(),
                dialog: step.operation.dialog(&self.parts[i]),
            })
            .collect()
    }
}

impl<S: Scalar, O: Operations> Default for ProgramRunner<S, O> {
    fn default() -> Self {
        Self::new()
    }
}
