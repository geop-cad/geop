//! [`ProgramFile`]: a program as a `.geop` file holds it — its parameters,
//! its steps and its state — and [`Workspace`], the [`crate::placed::Library`]
//! of a set of such files.

mod library;

pub use library::{Files, FilesMut, Registry, Workspace, is_program, is_shared, relative, resolve};

use std::{
    collections::{BTreeMap, HashSet},
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
};
use serde::{Deserialize, Serialize};

use crate::{
    ParamValue,
    operation::Input,
    parameters::{
        ParameterKind, Parameters, Values, names_in, parameter_of, rename_in, validate_name,
    },
    placed::{ABOUT, About, LIBRARY, Library},
    validate_operation_id,
};

/// Arguments that say which operation they are for: what a program written
/// in Rust — an example, a standard part, a test — pushes as a step
/// ([`ProgramFile::push`]). [`part_operation!`](crate::part_operation)
/// implements it for every operation's arguments.
pub trait StepArgs: Serialize {
    /// The operation's kind, as a program stores it.
    const KIND: &'static str;
}

/// The runner programs are built with.
pub type Runner = ProgramRunner<Input, ParamValue>;

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
    #[serde(default, skip_serializing_if = "Values::is_empty")]
    pub state: Values,
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

    /// The values of its parameters, with `overrides` — what a program
    /// placing it gives them — taking the place of their own (see
    /// [`Parameters::resolve`]). A parameter that does not resolve is left
    /// out, so what reads it fails, saying so.
    pub fn values(&self, overrides: &Values) -> Values {
        self.parameters.resolve(overrides).values
    }

    /// What a run of it is given: the values of its parameters, with
    /// `overrides` taking the place of their own (see [`Self::values`]),
    /// the library it places parts from — `library` — and what it says of
    /// its part ([`About`]): the library's file, and its parameters.
    pub fn inputs(&self, overrides: &Values, library: Rc<dyn Library>) -> BTreeMap<String, Input> {
        let about = About {
            file: library.file(),
            parameters: self.parameters.clone(),
        };
        (self.values(overrides).into_iter())
            .map(|(name, value)| (name, Input::Value(value)))
            .chain([
                (LIBRARY.to_string(), Input::Library(library)),
                (ABOUT.to_string(), Input::About(Rc::new(about))),
            ])
            .collect()
    }

    /// Appends the step `id`, `args` for the operation they are for.
    pub fn push<A: StepArgs>(&mut self, id: impl Into<String>, args: A) {
        self.program.steps.push(ProgramStep {
            id: id.into(),
            operation: A::KIND.to_string(),
            args: serde_json::to_value(args).expect("a program's arguments are plain data"),
        });
    }

    /// Runs it with its parameters `overrides` instead of its own and
    /// `library` on `runner`, which keeps what earlier runs built. Fails
    /// naming the first step that does.
    pub fn run(
        &self,
        runner: &mut Runner,
        overrides: &Values,
        library: &Rc<dyn Library>,
    ) -> GeopResult<()> {
        let inputs = self.inputs(overrides, library.clone());
        let output = runner.run(&self.program, &inputs, &self.state);
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
    /// it fails already. A step's formulas are its operation's to say
    /// (see [`geop_core_program::operation::Operation::formulas`]).
    pub fn parameter_uses(&self, registry: &Registry) -> BTreeMap<String, Vec<String>> {
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
            if let Ok(operation) = registry.get(&step.operation) {
                let _ = operation.edit_formulas(&step.args, &mut |formula| note(&step.id, formula));
            }
        }
        uses
    }

    /// The parameter `from` named `to`, and every formula reading it —
    /// the other parameters', every step's — reading `to`. Fails for a
    /// parameter there is none of, and for a name that is taken or no name.
    pub fn rename_parameter(
        &mut self,
        registry: &Registry,
        from: &str,
        to: &str,
    ) -> GeopResult<()> {
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
        let mut steps = self.program.steps.clone();
        for step in &mut steps {
            let operation = registry.get(&step.operation)?;
            step.args = operation.edit_formulas(&step.args, &mut |formula| {
                *formula = rename_in(formula, from, to);
            })?;
        }
        self.program.steps = steps;
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
