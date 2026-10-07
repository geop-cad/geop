//! [`Program`]: an ordered list of operations that builds a [`Part`], and
//! [`ProgramRunner`], which builds it incrementally — both with a
//! [`Library`], where a program finds the parts it places (see
//! [`library`]).
//!
//! Both are generic over the set of operations the program can use (see
//! [`Operations`]): which operations those are is for an application to
//! decide.

pub mod library;

pub use library::{Cache, Files, FilesMut, Library, MemoryCache, NoFiles, Workspace, is_program};

use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
    sync::Arc,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};
use serde::{Deserialize, Serialize};

use crate::{
    Instance, Part,
    operation::Operations,
    parameters::{Parameters, names_in, parameter_of, rename_in, validate_name},
    part::{Access, Log, State},
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
///
/// Its steps are its structure; its `parameters` the values its design is
/// given by (see [`crate::parameters`]); its `state` — the values its
/// steps read by name: where its placed parts are, and parameters a
/// program placing it gives other values (see [`crate::part::State`]) —
/// what solving it changes, as a sketch's points are what solving a sketch
/// changes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Program<O> {
    #[serde(default, skip_serializing_if = "Parameters::is_empty")]
    pub parameters: Parameters,
    pub steps: Vec<Step<O>>,
    #[serde(default, skip_serializing_if = "State::is_empty")]
    pub state: State,
}

impl<O> Default for Program<O> {
    fn default() -> Self {
        Self {
            parameters: Parameters::default(),
            steps: Vec::new(),
            state: State::new(),
        }
    }
}

impl<O> Program<O> {
    /// The values the program is built with: its state, and its parameters
    /// resolved with the state's overrides of them (see
    /// [`Parameters::resolve`]). A parameter that does not resolve is left
    /// out, so what reads it fails, saying so.
    pub fn inputs(&self) -> State {
        let mut inputs = self.state.clone();
        inputs.extend(self.parameters.resolve(&self.state).values);
        inputs
    }

    /// The part a build starts from: empty, with the program's parameters
    /// and the values it is built with.
    fn start<S: Scalar>(&self) -> Part<S> {
        Part::new()
            .with_state(self.inputs())
            .with_parameters(self.parameters.clone())
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

    /// What reads each parameter, by its name: the ids of the steps, and
    /// the names of the other parameters, whose formulas name it — what
    /// fails if it goes. A name no parameter has is listed too: what reads
    /// it fails already.
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
            if let crate::parameters::ParameterKind::Number { expression, .. } = &p.kind {
                note(&p.name, expression);
            }
        }
        for step in &self.steps {
            let mut operation = step.operation.clone();
            for formula in operation.formulas() {
                note(&step.id, formula);
            }
        }
        uses
    }

    /// The parameter `from` named `to`, and every formula reading it —
    /// the other parameters', every step's (see
    /// [`crate::operation::Operation::formulas`]) — reading `to`, as does
    /// the program's state. Fails for a parameter there is none of, and
    /// for a name that is taken or no name.
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
        self.parameters.rename(from, to);
        for step in &mut self.steps {
            for formula in step.operation.formulas() {
                *formula = rename_in(formula, from, to);
            }
        }
        if let Some(value) = self.state.remove(from) {
            self.state.insert(to.to_string(), value);
        }
        Ok(())
    }

    /// Checks that every step id is a valid operation id and unique: every
    /// name a step creates is built from its id.
    pub fn validate(&self) -> GeopResult<()> {
        self.parameters.validate()?;
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

    /// The step `id` as a program of its own, as a copy of it is kept: the
    /// step, and the state stored under its id — where a placed part is.
    /// Not the parameters: pasted into another program, the step reads
    /// those of that program.
    pub fn excerpt(&self, id: &str) -> GeopResult<Self>
    where
        O: Clone,
    {
        let step = self.steps[self.index_of(id)?].clone();
        let prefix = format!("{id}.");
        let state = self
            .state
            .iter()
            .filter(|(name, _)| name.starts_with(&prefix))
            .map(|(name, value)| (name.clone(), value.clone()))
            .collect();
        Ok(Self {
            parameters: Parameters::default(),
            steps: vec![step],
            state,
        })
    }

    /// Inserts the steps of `pasted` at `index`, in order, with the state
    /// stored under their ids: each under its own id where no step has it
    /// yet, else under a fresh one. The ids they got, in order.
    ///
    /// A step names what it builds on by the ids of the steps that built
    /// it, so a step pasted under a fresh id is not renamed in the steps
    /// pasted with it: they still build on the steps they were copied
    /// with.
    pub fn paste(&mut self, pasted: Self, index: usize) -> GeopResult<Vec<String>> {
        pasted.validate()?;
        if index > self.steps.len() {
            return Err(GeopError::new(format!(
                "cannot paste at {index}: the program has {} steps",
                self.steps.len()
            )));
        }
        let mut ids = Vec::with_capacity(pasted.steps.len());
        for (k, step) in pasted.steps.into_iter().enumerate() {
            let id = if self.steps.iter().any(|s| s.id == step.id) {
                self.fresh_id(&step.operation)
            } else {
                step.id.clone()
            };
            let prefix = format!("{}.", step.id);
            for (name, value) in &pasted.state {
                if let Some(rest) = name.strip_prefix(&prefix) {
                    self.state.insert(format!("{id}.{rest}"), value.clone());
                }
            }
            self.steps.insert(
                index + k,
                Step {
                    id: id.clone(),
                    operation: step.operation,
                },
            );
            ids.push(id);
        }
        Ok(ids)
    }

    /// Runs every step in order, starting from an empty part, and returns
    /// the part the whole program builds — or the first error any step
    /// raises, at which point the steps after it never run. A program
    /// always starts from nothing: its names are only guaranteed to be
    /// unique, and to mean the same thing on every run, when every entity
    /// was created by one of its own steps.
    ///
    /// After each step, every entity of the part must have a name — an
    /// operation that leaves one unnamed has broken the one guarantee a
    /// program relies on.
    ///
    /// The parts it places are found in `library`.
    pub fn build<S: Scalar>(&self, library: &dyn Library<S>) -> GeopResult<Part<S>> {
        self.validate()?;
        let mut part = self.start();
        for (index, step) in self.steps.iter().enumerate() {
            part = run_step(part, index, step, library, None)?.0;
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

/// Step `index` of a program applied to `part`, with every name checked —
/// and, if `log` is given, what the step read and wrote of `part` (see
/// [`crate::part::Cell`]): `None` if that is not known, because the step
/// returned a part that is no copy of the one it was given.
fn run_step<S: Scalar, O: Operations>(
    mut part: Part<S>,
    index: usize,
    step: &Step<O>,
    library: &dyn Library<S>,
    log: Option<&Arc<Log>>,
) -> GeopResult<(Part<S>, Option<Access>)> {
    let ctx = with_context!("program step {index} ({:?})", step.id);
    if let Some(log) = log {
        part.record(log);
    }
    let mut part = step
        .operation
        .apply(part, &step.id, library)
        .with_context(ctx)?;
    let access = log.and_then(|log| part.finish_recording(log));
    part.check_names().with_context(ctx)?;
    part.renew_revision();
    Ok((part, access))
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
/// It keeps, for every step it has run, the part before and after it, and
/// what the step read and wrote of the part (see [`crate::part::Cell`]).
/// Running again after an edit takes a step as it was built if it is the
/// same step and each cell of the part it read is as it was then: so
/// changing the last step runs one step, and changing a step in the middle
/// runs the steps after it that read what it wrote — not those that did
/// not, like the placing of an unrelated part. And a run can stop early —
/// while a step in the middle is being edited, only the steps up to it need
/// to run, however long the rest of the program is. What was built past the
/// stop is kept, so moving the stop back again costs nothing.
///
/// A run stops at the first step that fails: the steps after it would only
/// fail too, for want of what it should have built.
///
/// What a step builds can depend on more than the part: on the files its
/// library reads, which it notes per step: when some change,
/// [`ProgramRunner::forget`] runs again the steps that read one.
pub struct ProgramRunner<S: Scalar, O> {
    /// The steps the last run covers.
    steps: Vec<Step<O>>,
    /// `parts[i]`: the part after `steps[..i]`. A failed step leaves the
    /// part as it was, so this stays one longer than `steps`.
    parts: Vec<Arc<Part<S>>>,
    results: Vec<StepResult>,
    /// `reads[i]`: the files `steps[i]` built on — those of every part it
    /// placed (see [`Instance::files`]).
    reads: Vec<BTreeSet<String>>,
    /// What every step that was built, and did not fail, did, by its id.
    records: HashMap<String, Record<S, O>>,
    /// How many steps the last run covers.
    ran: usize,
    /// How many steps the last run built, rather than took from earlier
    /// runs.
    built_anew: usize,
    /// How many steps every run so far built, together.
    steps_built: usize,
}

/// What a step did when it was built (see [`ProgramRunner`]).
struct Record<S: Scalar, O> {
    step: Step<O>,
    before: Arc<Part<S>>,
    after: Arc<Part<S>>,
    access: Access,
    files: BTreeSet<String>,
}

/// A library that notes the files of every part it gives out: what a step
/// built with it read.
struct Recording<'l, S: Scalar> {
    library: &'l dyn Library<S>,
    read: RefCell<BTreeSet<String>>,
}

impl<S: Scalar> Library<S> for Recording<'_, S> {
    fn instance(&self, file: &str, overrides: &State) -> GeopResult<Instance<S>> {
        let instance = self.library.instance(file, overrides)?;
        self.read
            .borrow_mut()
            .extend(instance.files.iter().cloned());
        Ok(instance)
    }

    fn files(&self) -> Vec<String> {
        self.library.files()
    }

    fn read(&self, file: &str) -> GeopResult<(String, String)> {
        let (path, text) = self.library.read(file)?;
        self.read.borrow_mut().insert(path.clone());
        Ok((path, text))
    }

    fn cache(&self) -> Option<&dyn library::Cache> {
        self.library.cache()
    }
}

impl<S: Scalar, O: Operations> ProgramRunner<S, O> {
    pub fn new() -> Self {
        Self {
            steps: Vec::new(),
            parts: vec![Arc::new(Part::new())],
            results: Vec::new(),
            reads: Vec::new(),
            records: HashMap::new(),
            ran: 0,
            built_anew: 0,
            steps_built: 0,
        }
    }

    /// Forgets every part built: it builds another program now.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Forgets what was built from the first step that read one of the
    /// files `changed` — as the library names them — or failed, which it
    /// may have for want of one: those steps may build something else now.
    /// The steps before it read none, and are kept.
    pub fn forget(&mut self, changed: &BTreeSet<String>) {
        self.records.retain(|_, r| r.files.is_disjoint(changed));
        let first = self
            .reads
            .iter()
            .zip(&self.results)
            .position(|(read, result)| result.error.is_some() || !read.is_disjoint(changed));
        if let Some(first) = first {
            self.steps.truncate(first);
            self.parts.truncate(first + 1);
            self.results.truncate(first);
            self.reads.truncate(first);
            self.ran = self.ran.min(first);
        }
    }

    /// The part the whole of `program` builds, and every file it read, if
    /// the last run built all of it, every step without an error: what a
    /// library may keep as that program's file built (see
    /// [`Workspace::keep`](library::Workspace::keep)).
    pub fn built_whole(&self, program: &Program<O>) -> Option<(&Part<S>, BTreeSet<String>)> {
        let whole = self.ran == program.steps.len()
            && self.steps == program.steps
            && self.results.iter().all(|r| r.error.is_none());
        whole.then(|| (self.part(), self.reads.iter().flatten().cloned().collect()))
    }

    /// Runs the first `stop` steps of `program` — all of them if `None` —
    /// with `library`, taking from what earlier runs built whatever still
    /// applies. See [`ProgramRunner::part`] and [`ProgramRunner::results`]
    /// for the outcome.
    pub fn run(&mut self, program: &Program<O>, stop: Option<usize>, library: &dyn Library<S>) {
        let ids: HashSet<&str> = program.steps.iter().map(|s| s.id.as_str()).collect();
        self.records.retain(|id, _| ids.contains(id.as_str()));

        // The empty part the program starts from, with the values it is
        // built with: each input that differs from what it was gets a new
        // version, so what read it is built again.
        let mut start = (*self.parts[0]).clone();
        start.set_state(program.inputs());
        start.set_parameters(program.parameters.clone());

        let target = stop.unwrap_or(program.steps.len()).min(program.steps.len());
        let mut parts = vec![Arc::new(start)];
        let mut steps = Vec::new();
        let mut results = Vec::new();
        let mut reads = Vec::new();
        self.built_anew = 0;
        for index in 0..target {
            let step = &program.steps[index];
            let before = parts.last().expect("parts is never empty").clone();
            let taken = self
                .records
                .get(&step.id)
                .filter(|r| r.step == *step && before.would_write_the_same(&r.before, &r.access))
                .map(|r| {
                    (
                        before.replayed(&r.before, &r.after, &r.access),
                        r.files.clone(),
                    )
                });
            let (after, error, files) = match taken {
                Some((after, files)) => (after, None, files),
                None => {
                    let recording = Recording {
                        library,
                        read: RefCell::new(BTreeSet::new()),
                    };
                    let log = Log::new();
                    let built = run_step((*before).clone(), index, step, &recording, Some(&log));
                    let files = recording.read.into_inner();
                    self.built_anew += 1;
                    self.steps_built += 1;
                    match built {
                        Ok((after, access)) => {
                            let after = Arc::new(after);
                            match access {
                                Some(access) => {
                                    self.records.insert(
                                        step.id.clone(),
                                        Record {
                                            step: step.clone(),
                                            before: before.clone(),
                                            after: after.clone(),
                                            access,
                                            files: files.clone(),
                                        },
                                    );
                                }
                                None => {
                                    self.records.remove(&step.id);
                                }
                            }
                            (after, None, files)
                        }
                        Err(e) => {
                            self.records.remove(&step.id);
                            (before.clone(), Some(e.to_string()), files)
                        }
                    }
                }
            };
            let failed = error.is_some();
            steps.push(step.clone());
            parts.push(after);
            reads.push(files);
            results.push(StepResult {
                id: step.id.clone(),
                error,
            });
            if failed {
                break;
            }
        }
        self.ran = steps.len();
        self.steps = steps;
        self.parts = parts;
        self.results = results;
        self.reads = reads;
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

    /// The part step `index` built in the last run — `None` if the run did
    /// not reach it, or it failed.
    pub fn built(&self, index: usize) -> Option<&Part<S>> {
        let ran = self.results().get(index)?;
        ran.error.is_none().then(|| &*self.parts[index + 1])
    }

    /// One result per step the last run covered.
    pub fn results(&self) -> &[StepResult] {
        &self.results[..self.ran]
    }

    /// How many steps the last run built — the rest of those it covered
    /// it took as earlier runs built them: what an edit, or a change of a
    /// parameter, cost.
    pub fn built_anew(&self) -> usize {
        self.built_anew
    }

    /// How many steps every run so far built, together: what a sequence
    /// of edits cost, however many runs each made.
    pub fn steps_built(&self) -> usize {
        self.steps_built
    }
}

impl<S: Scalar, O: Operations> Default for ProgramRunner<S, O> {
    fn default() -> Self {
        Self::new()
    }
}
