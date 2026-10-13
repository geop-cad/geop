//! [`ProgramFile`]: a program as a `.geop` file holds it — its parameters,
//! its steps and its state.

use std::collections::{BTreeMap, HashSet};

use geop_core_editor::numeric_input::{formulas_in, names_in, parameter_of, rename_parameter_in};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    operation::StepArgs,
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
};
use geop_ops_model::validate_operation_id;
use serde::{Deserialize, Serialize};

use crate::{
    Design,
    about::About,
    parameters::{ParameterKind, Parameters, Settings, validate_name},
    state::StateValue,
};

/// The runner programs are built with: numbers are their inputs (see
/// [`Parameters::resolve`]), and their state is of [`StateValue`]s.
pub type Runner = ProgramRunner<Design, StateValue>;

/// A part's state, by name.
pub type StateValues = BTreeMap<String, StateValue>;

/// A program as a file holds it: the parameters its design is given by (see
/// [`crate::parameters`]), its steps, and its state — where its placed parts
/// are and how its joints are turned, what solving it changes. Serializes
/// as `{"parameters": {...}, "steps": [...], "state": {...}}`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ProgramFile {
    #[serde(default, skip_serializing_if = "Parameters::is_empty")]
    pub parameters: Parameters,
    #[serde(flatten)]
    pub program: Program,
    #[serde(default, skip_serializing_if = "StateValues::is_empty")]
    pub state: StateValues,
}

impl ProgramFile {
    /// A program of no steps.
    pub fn new() -> Self {
        Self::default()
    }

    /// The program the file `text` holds, checked (see
    /// [`ProgramFile::validate`]).
    pub fn from_json(text: &str) -> GeopResult<Self> {
        let file: Self = serde_json::from_str(text)
            .map_err(|e| GeopError::new(format!("reading a program: {e}")))?;
        file.validate()?;
        Ok(file)
    }

    pub fn to_json(&self) -> GeopResult<String> {
        serde_json::to_string_pretty(self)
            .map_err(|e| GeopError::new(format!("writing a program: {e}")))
    }

    /// The numbers a run of it is given: its parameters' values, with
    /// `overrides` — what a program placing it gives them — taking the
    /// place of their own (see [`Parameters::resolve`]). A parameter that
    /// does not resolve is left out, so what reads it fails, saying so.
    pub fn inputs(&self, overrides: &Settings) -> BTreeMap<String, Design> {
        self.parameters.resolve(overrides).numbers
    }

    /// What a run of it, in the file `file`, says of its part: its
    /// parameters, and what they are set to with `overrides`. The given
    /// step defines it (see [`crate::about::define_about`]).
    pub fn about(&self, file: &str, overrides: &Settings) -> About {
        About {
            file: file.to_string(),
            parameters: self.parameters.clone(),
            chosen: self.parameters.resolve(overrides).chosen,
        }
    }

    /// Appends the step `id`, `args` for the operation they are for: for a
    /// program written in Rust, whose arguments are plain data.
    #[allow(
        clippy::expect_used,
        reason = "arguments that do not serialize are a bug of the program written"
    )]
    pub fn push<A: StepArgs>(&mut self, id: impl Into<String>, args: A) {
        let step = ProgramStep::of(id, &args).expect("a step's arguments are plain data");
        self.program.steps.push(step);
    }

    /// Runs it with its parameters `overrides` instead of its own on
    /// `runner`, which keeps what earlier runs built and gives what else a
    /// run is given ([`Runner::given`]). Fails naming the first step that
    /// does.
    pub fn run(&self, runner: &mut Runner, overrides: &Settings) -> GeopResult<()> {
        let output = runner.run(&self.program, &self.inputs(overrides), &self.state);
        match output.step_errors.into_iter().next() {
            Some((step, error)) => Err(error.with_context(format!("step {step:?}"))),
            None => Ok(()),
        }
    }

    /// The position of the step `id`.
    pub fn index_of(&self, id: &str) -> GeopResult<usize> {
        (self.program.steps.iter().position(|s| s.id == id))
            .ok_or_else(|| GeopError::new(format!("the program has no step {id:?}")))
    }

    /// An id no step has yet, for a new step of an operation labelled
    /// `label`: the label, lowercased, and the lowest number that makes it
    /// unique — `sketch1`, `extrude2`.
    pub fn fresh_id(&self, label: &str) -> String {
        let base = label.to_lowercase().replace(' ', "_");
        (1..)
            .map(|n| format!("{base}{n}"))
            .find(|id| self.program.steps.iter().all(|s| &s.id != id))
            .unwrap_or(base)
    }

    /// What reads each parameter, by its name: the ids of the steps, and
    /// the names of the other parameters, whose formulas name it — what
    /// fails if it goes. A name no parameter has is listed too: what reads
    /// it fails already. A step's formulas are found in its saved arguments
    /// ([`formulas_in`]).
    pub fn parameter_uses(&self) -> BTreeMap<String, Vec<String>> {
        let mut uses: BTreeMap<String, Vec<String>> = BTreeMap::new();
        let mut note = |reader: &str, formula: &str| {
            for name in names_in(formula) {
                let readers = uses.entry(parameter_of(&name).to_string()).or_default();
                if !readers.iter().any(|r| r == reader) {
                    readers.push(reader.to_string());
                }
            }
        };
        for p in &self.parameters.values {
            if let ParameterKind::Number { expression, .. } = &p.kind {
                note(&p.name, expression);
            }
        }
        for step in &self.program.steps {
            for formula in formulas_in(&step.args) {
                note(&step.id, formula);
            }
        }
        uses
    }

    /// The parameter `from` named `to`, and every formula reading it —
    /// the other parameters', every step's — reading `to`. Fails for a
    /// parameter there is none of, and for a name that is taken or no name.
    pub fn rename_parameter(&mut self, from: &str, to: &str) -> GeopResult<()> {
        if self.parameters.get(from).is_none() {
            return Err(GeopError::new(format!("there is no parameter {from:?}")));
        }
        if from == to {
            return Ok(());
        }
        validate_name(to)?;
        if self.parameters.get(to).is_some() {
            return Err(GeopError::new(format!(
                "there is a parameter {to:?} already"
            )));
        }
        for step in &mut self.program.steps {
            rename_parameter_in(&mut step.args, from, to);
        }
        self.parameters.rename(from, to);
        Ok(())
    }

    /// The step `id` as a program of its own, as a copy of it is kept: the
    /// step, and the state stored under its id — where a placed part is.
    /// Not the parameters: pasted into another program, the step reads
    /// those of that program.
    pub fn excerpt(&self, id: &str) -> GeopResult<Self> {
        let step = self.program.steps[self.index_of(id)?].clone();
        let prefix = format!("{id}.");
        let state = (self.state.iter())
            .filter(|(name, _)| name.starts_with(&prefix))
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();
        Ok(Self {
            parameters: Parameters::default(),
            program: Program::new(vec![step]),
            state,
        })
    }

    /// Inserts the steps of `pasted` at `index`, in order, with the state
    /// stored under their ids: each under its own id where no step has it
    /// yet, else under a fresh one — `label` naming a step's operation. The
    /// ids they got, in order.
    ///
    /// A step names what it builds on by the ids of the steps that built
    /// it, so a step pasted under a fresh id is not renamed in the steps
    /// pasted with it: they still build on the steps they were copied
    /// with.
    pub fn paste(
        &mut self,
        pasted: Self,
        index: usize,
        label: impl Fn(&ProgramStep) -> String,
    ) -> GeopResult<Vec<String>> {
        pasted.validate()?;
        if index > self.program.steps.len() {
            return Err(GeopError::new(format!(
                "cannot paste at {index}: the program has {} steps",
                self.program.steps.len()
            )));
        }
        let mut ids = Vec::with_capacity(pasted.program.steps.len());
        for (k, step) in pasted.program.steps.into_iter().enumerate() {
            let id = match self.program.steps.iter().any(|s| s.id == step.id) {
                true => self.fresh_id(&label(&step)),
                false => step.id.clone(),
            };
            let prefix = format!("{}.", step.id);
            for (name, value) in &pasted.state {
                if let Some(rest) = name.strip_prefix(&prefix) {
                    self.state.insert(format!("{id}.{rest}"), value.clone());
                }
            }
            self.program.steps.insert(
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

    /// Checks its parameters, and that every step id is a valid operation
    /// id and unique: every name a step creates is built from its id.
    pub fn validate(&self) -> GeopResult<()> {
        self.parameters.validate()?;
        let mut ids = HashSet::new();
        for step in &self.program.steps {
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
}
