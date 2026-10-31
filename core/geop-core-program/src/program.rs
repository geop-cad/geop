//! [`Program`]: what a program is, and what a program file holds — its
//! steps, each an operation and its saved arguments, and its state: where
//! its placed parts are and how its joints are turned, what solving it
//! changes. Serialized as `{"steps": [...], "state": {...}}`.
//!
//! A step's id names everything the step builds, so ids are unique in a
//! program and spelled in an alphabet a name can be built from
//! ([`validate_operation_id`]).

use std::collections::{HashMap, HashSet};

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

use crate::{
    operation::{Operation, StepArgs, saved},
    operation_registry::OperationRegistry,
    part::Part,
    placed::{INSTANCE_SEPARATOR, Located, TARGET_SEPARATOR},
    runner::Runner,
    values::StateValues,
};

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
        })
    }

    pub fn new<O: Operation>(id: impl Into<String>, args: &O::Args) -> GeopResult<Self> {
        let id = id.into();
        let args = saved::<O>(args).map_err(|e| e.with_context(format!("step `{id}`")))?;
        Ok(Self {
            id,
            operation: O::KIND.to_string(),
            args,
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
    /// step, and the state that is its, every key naming it (`names_step`):
    /// where a placed part is, how its joints are turned.
    pub fn excerpt(&self, id: &str) -> GeopResult<Self> {
        let step = self.step(id)?.clone();
        let state = (self.state.iter())
            .filter(|(name, _)| names_step(name, id))
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();
        Ok(Self {
            steps: vec![step],
            state,
        })
    }

    /// Every text in its steps' arguments that `rewrite` rewrites,
    /// rewritten: the files they name, as the steps move to another file.
    pub fn rewrite_texts(&mut self, rewrite: impl Fn(&str) -> Option<String>) {
        fn walk(value: &mut serde_json::Value, rewrite: &dyn Fn(&str) -> Option<String>) {
            match value {
                serde_json::Value::String(text) => {
                    if let Some(rewritten) = rewrite(text) {
                        *text = rewritten;
                    }
                }
                serde_json::Value::Array(values) => {
                    values.iter_mut().for_each(|v| walk(v, rewrite));
                }
                serde_json::Value::Object(fields) => {
                    fields.values_mut().for_each(|v| walk(v, rewrite));
                }
                _ => {}
            }
        }
        for step in &mut self.steps {
            walk(&mut step.args, &rewrite);
        }
    }

    /// Inserts the steps of `pasted` at `index`, in order, with the state
    /// that is theirs (see [`Program::excerpt`]): each under its own id
    /// where no step has it yet, else under a fresh one — `label` naming a
    /// step's operation. The ids they got, in order. `pasted` is checked
    /// against `registry` first (see [`Program::validate`]).
    ///
    /// A step pasted under a fresh id is the step the steps pasted with it,
    /// itself too, name by that id: every name of theirs that names a step
    /// pasted, in their arguments and their state, is renamed with it
    /// (`rename_steps`). A sketch and its extrusion pasted twice are two
    /// extrusions, each of its own sketch; a part placed and mated, pasted,
    /// is a second part, mated as itself.
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
        let own: Vec<String> = pasted.steps.iter().map(|s| s.id.clone()).collect();
        let mut renamed = HashMap::new();
        let mut ids = Vec::with_capacity(pasted.steps.len());
        for (k, step) in pasted.steps.into_iter().enumerate() {
            let id = match self.steps.iter().any(|s| s.id == step.id) {
                true => self.fresh_id(&label(&step)),
                false => step.id.clone(),
            };
            if id != step.id {
                renamed.insert(step.id.clone(), id.clone());
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
        for step in &mut self.steps[index..index + ids.len()] {
            rename_in_args(&mut step.args, &renamed);
        }
        for (name, value) in pasted.state {
            if own.iter().any(|id| names_step(&name, id)) {
                self.state.insert(rename_steps(&name, &renamed, true), value);
            }
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

/// What separates the names a name is built of: `extrude(box,end)`,
/// `sketch:base` — the steps a name names are among the pieces between.
const NAME_SEPARATORS: [char; 4] = ['(', ')', ',', TARGET_SEPARATOR];

/// `name`, a name a part spells, with every step `renamed` maps renamed:
/// a target's or an element's name, located (`/hand:model`) or of the
/// part itself (`/:sketch:base`, `extrude(box,end)`) — or, a `key`, a key
/// of its state (`hand.pose`, `add_part(hand,m1).turn`). Behind an
/// instance's path only that instance is a step of the part: the rest of
/// the path, and the name, are the part placed's.
fn rename_steps(name: &str, renamed: &HashMap<String, String>, key: bool) -> String {
    map_steps(name, key, &mut |id| renamed.get(id).cloned())
}

/// Whether the key of the state `key` (see [`rename_steps`]) names the
/// step `id`: is of it.
fn names_step(key: &str, id: &str) -> bool {
    let mut named = false;
    map_steps(key, true, &mut |step| {
        named |= step == id;
        None
    });
    named
}

/// `name` (see [`rename_steps`]), each step it names as `map` maps it,
/// where it maps it.
fn map_steps(name: &str, key: bool, map: &mut dyn FnMut(&str) -> Option<String>) -> String {
    if name.starts_with(INSTANCE_SEPARATOR) {
        let Ok(located) = Located::parse(name) else {
            return name.to_string();
        };
        let path = located.path();
        if path.is_empty() {
            return Located::new(map_steps(located.name(), key, map)).to_string();
        }
        let (instance, rest) = path.split_once(INSTANCE_SEPARATOR).unwrap_or((path, ""));
        let Some(to) = map(instance) else {
            return name.to_string();
        };
        let path = match rest.is_empty() {
            true => to,
            false => format!("{to}{INSTANCE_SEPARATOR}{rest}"),
        };
        return Located::in_instance(path, located.name()).to_string();
    }
    let mut spelled = String::with_capacity(name.len());
    let mut rest = name;
    let mut first = true;
    loop {
        let end = rest.find(NAME_SEPARATORS).unwrap_or(rest.len());
        let (piece, after) = rest.split_at(end);
        // A key of the state begins with what it is of, then `.` and what
        // of it: `hand.pose`.
        let mapped = match map(piece) {
            Some(to) => Some(to),
            None if key && first => (piece.match_indices('.'))
                .find_map(|(at, _)| map(&piece[..at]).map(|to| format!("{to}{}", &piece[at..]))),
            None => None,
        };
        spelled.push_str(mapped.as_deref().unwrap_or(piece));
        let Some(separator) = after.chars().next() else {
            return spelled;
        };
        spelled.push(separator);
        rest = &after[separator.len_utf8()..];
        first = false;
    }
}

/// `args`, a step's arguments, every step `renamed` names in them renamed
/// ([`rename_steps`]), but in the name of an element behind an instance's
/// path, which is the part placed's: `{"target": "/hand:model", "name":
/// "extrude(box,end)"}` names the hand's step `box`, not the part's.
fn rename_in_args(args: &mut serde_json::Value, renamed: &HashMap<String, String>) {
    if renamed.is_empty() {
        return;
    }
    match args {
        serde_json::Value::String(text) => *text = rename_steps(text, renamed, false),
        serde_json::Value::Array(values) => {
            values.iter_mut().for_each(|v| rename_in_args(v, renamed));
        }
        serde_json::Value::Object(fields) => {
            let placed = (fields.get("target").and_then(|t| t.as_str()))
                .and_then(|t| Located::parse(t).ok())
                .is_some_and(|t| !t.path().is_empty());
            for (key, value) in fields.iter_mut() {
                if !(placed && key == "name") {
                    rename_in_args(value, renamed);
                }
            }
        }
        _ => {}
    }
}

fn no_step(id: &str) -> GeopError {
    GeopError::new(format!("the program has no step `{id}`"))
}
