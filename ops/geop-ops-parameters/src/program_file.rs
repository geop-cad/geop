//! [`ProgramFile`]: a program as a `.geop` file holds it — its steps, its
//! parameters among them, and its state.

use std::collections::{BTreeMap, HashSet};

use geop_core_editor::numeric_input::{
    ParameterValue, formulas_in, names_in, parameter, parameter_of, rename_parameter_in,
};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::ElementReference,
    operation::StepArgs,
    operation_registry::OperationRegistry,
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
};
use geop_ops_model::validate_operation_id;
use serde::{Deserialize, Serialize};

use crate::{
    BuildPart,
    parameters::{ParameterKind, Parameters, Setting, Settings, validate_name},
    state::StateValue,
    steps::{DefineParameters, DefineParametersArgs, PARAMETERS, PROPERTIES, PartPropertiesArgs},
};

/// The runner programs are built with: their inputs are what their
/// parameters are set to ([`Setting`]s), and their state is of
/// [`StateValue`]s.
pub type Runner = ProgramRunner<Setting, StateValue>;

/// A part's state, by name.
pub type StateValues = BTreeMap<String, StateValue>;

/// The parameters and properties of `program`, as its steps define them
/// (see [`crate::steps`]): none, where it has no such step.
pub fn parameters_of(program: &Program) -> Parameters {
    let args = |kind: &str| program.steps.iter().find(|s| s.operation == kind);
    let values = (args(PARAMETERS))
        .and_then(|step| serde_json::from_value::<DefineParametersArgs>(step.args.clone()).ok())
        .unwrap_or_default()
        .values;
    let properties = (args(PROPERTIES))
        .and_then(|step| serde_json::from_value::<PartPropertiesArgs>(step.args.clone()).ok())
        .unwrap_or_default();
    Parameters {
        color: properties.color,
        material: properties.material,
        title: properties.title,
        designation: properties.designation,
        values,
    }
}

/// `program` with its parameters and properties set to `parameters`: in
/// the steps that define them, put first where it has none yet — and none,
/// where there is nothing to define. The steps check them as they run.
pub fn set_parameters(program: &mut Program, parameters: Parameters) {
    let properties = PartPropertiesArgs {
        title: parameters.title,
        designation: parameters.designation,
        color: parameters.color,
        material: parameters.material,
    };
    let values = DefineParametersArgs {
        values: parameters.values,
    };
    let empty = properties == PartPropertiesArgs::default();
    set_step(program, PROPERTIES, (!empty).then_some(&properties));
    set_step(program, PARAMETERS, (!values.values.is_empty()).then_some(&values));
}

/// The step of `program` of the operation `kind` holding `args`: put first
/// where there is none, under the kind's name; taken out, for none.
#[allow(
    clippy::expect_used,
    reason = "the arguments of a parameters or properties step are plain data"
)]
fn set_step<A: Serialize>(program: &mut Program, kind: &str, args: Option<&A>) {
    let steps = &mut program.steps;
    let at = steps.iter().position(|s| s.operation == kind);
    let Some(args) = args else {
        if let Some(at) = at {
            steps.remove(at);
        }
        return;
    };
    let args = serde_json::to_value(args).expect("a step's arguments are plain data");
    match at {
        Some(at) => steps[at].args = args,
        None => {
            let taken = |id: &str| steps.iter().any(|s| s.id == id);
            let id = (std::iter::once(kind.to_string()))
                .chain((1..).map(|n| format!("{kind}{n}")))
                .find(|id| !taken(id))
                .unwrap_or_else(|| kind.to_string());
            let step = ProgramStep {
                id,
                operation: kind.to_string(),
                args,
            };
            steps.insert(0, step);
        }
    }
}

/// A program as a file holds it: its steps — the parameters its design is
/// given by and its properties among them (see [`crate::steps`]) — and its
/// state: where its placed parts are and how its joints are turned, what
/// solving it changes. Serializes as `{"steps": [...], "state": {...}}`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ProgramFile {
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

    /// Its parameters and properties, as its steps define them (see
    /// [`parameters_of`]).
    pub fn parameters(&self) -> Parameters {
        parameters_of(&self.program)
    }


    /// What its parameters come to as it defines them, by the names a
    /// formula reads them by: a number by its name, a table's columns as
    /// `table.column` — its step run on its own (see
    /// [`crate::steps::DefineParameters`]). Fails naming one that does not
    /// come to anything.
    pub fn parameter_numbers(&self) -> GeopResult<BTreeMap<String, f64>> {
        let steps = (self.program.steps.iter())
            .filter(|step| step.operation == PARAMETERS)
            .cloned()
            .collect();
        let runner = Runner::new(OperationRegistry::default().with(DefineParameters)?);
        let mut part = BuildPart::default();
        let program = Program::new(steps);
        let output = runner.run(&mut part, &program, &Settings::new(), &StateValues::new());
        if let Some((step, e)) = output.step_errors.into_iter().next() {
            return Err(e.with_context(format!("step {step:?}")));
        }
        let reader = part.current();
        let mut numbers = BTreeMap::new();
        for p in &self.parameters().values {
            let reference = ElementReference::whole(parameter(&p.name));
            let value = (reader.retrieve_element(&reference))
                .map_err(|e| e.with_context(format!("the parameter {:?}", p.name)))?;
            match value {
                ParameterValue::Number(n) => {
                    numbers.insert(p.name.clone(), *n);
                }
                ParameterValue::Row { columns, .. } => {
                    for (column, value) in columns {
                        numbers.insert(format!("{}.{column}", p.name), *value);
                    }
                }
            }
        }
        Ok(numbers)
    }

    /// Its parameters and properties set to `parameters` (see
    /// [`set_parameters`]).
    pub fn set_parameters(&mut self, parameters: Parameters) {
        set_parameters(&mut self.program, parameters);
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

    /// Runs it on `part`, which keeps what earlier runs built, by `runner`:
    /// its parameters set to `inputs` where they give them, and the values
    /// `state` gives of its state instead of its own. Fails naming the
    /// first step that does.
    pub fn run(
        &self,
        runner: &Runner,
        part: &mut BuildPart,
        inputs: &Settings,
        state: &StateValues,
    ) -> GeopResult<()> {
        let mut given = self.state.clone();
        given.extend(state.iter().map(|(name, value)| (name.clone(), value.clone())));
        let output = runner.run(part, &self.program, inputs, &given);
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
        for p in &self.parameters().values {
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
        let mut parameters = self.parameters();
        if parameters.get(from).is_none() {
            return Err(GeopError::new(format!("there is no parameter {from:?}")));
        }
        if from == to {
            return Ok(());
        }
        validate_name(to)?;
        if parameters.get(to).is_some() {
            return Err(GeopError::new(format!(
                "there is a parameter {to:?} already"
            )));
        }
        for step in &mut self.program.steps {
            rename_parameter_in(&mut step.args, from, to);
        }
        parameters.rename(from, to);
        self.set_parameters(parameters);
        Ok(())
    }

    /// The step `id` as a program of its own, as a copy of it is kept: the
    /// step, and the state stored under its id — where a placed part is.
    pub fn excerpt(&self, id: &str) -> GeopResult<Self> {
        let step = self.program.steps[self.index_of(id)?].clone();
        let prefix = format!("{id}.");
        let state = (self.state.iter())
            .filter(|(name, _)| name.starts_with(&prefix))
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();
        Ok(Self {
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

    /// Checks that every step id is a valid operation id and unique: every
    /// name a step creates is built from its id.
    pub fn validate(&self) -> GeopResult<()> {
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
