//! [`Program`]: an ordered list of operations that builds a [`Part`], and
//! [`ProgramRunner`], which builds it incrementally.
//!
//! Both are generic over the set of operations the program can use (see
//! [`Operations`]): which operations those are is for an application to
//! decide.

use std::collections::HashSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use serde::{Deserialize, Serialize};

use crate::{Part, operation::Operations, validate_operation_id};

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

    /// The part the first `n` steps of the last run built — or, where the
    /// run stopped before, the part it built.
    pub fn part_at(&self, n: usize) -> &Part<S> {
        &self.parts[n.min(self.ran)]
    }

    /// One result per step the last run covered.
    pub fn results(&self) -> &[StepResult] {
        &self.results[..self.ran]
    }
}

impl<S: Scalar, O: Operations> Default for ProgramRunner<S, O> {
    fn default() -> Self {
        Self::new()
    }
}
