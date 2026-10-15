//! [`Runner`]: runs a program on a part ([`Runner::run`]) — every step's
//! operation, in order, each declaring what it produces — and builds the
//! parts it places, sharing equal builds.

use std::{
    cell::{Cell, RefCell},
    collections::HashSet,
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    operation_registry::OperationRegistry,
    part::{Part, StepContext},
    placed::{BuildCache, BuildKey},
    program::{Program, ProgramStep},
    values::{Settings, StateValues},
    workspace::{Location, Workspace},
};

/// Runs programs on parts, with the operations of `registry`, reading
/// what they place and import from `workspace` as the program in the file
/// of `location` names it: `location.within` is the stack of files being
/// built, which a file must not place again.
pub struct Runner {
    pub registry: Rc<OperationRegistry>,
    pub workspace: Option<Rc<dyn Workspace>>,
    pub location: Location,
    /// What it shares with the runners of the parts it places.
    shared: Rc<Shared>,
}

/// What a runner shares with the runners of the parts it places.
#[derive(Default)]
struct Shared {
    cache: RefCell<BuildCache>,
    counts: Cell<Counts>,
}

/// How much work a runner did: what a test of what runs counts, never a
/// time.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counts {
    /// The makes that ran (see [`StepContext::produce`]).
    pub makes: u64,
    /// The makes of placed parts among them.
    pub placed: u64,
    /// The parts placed that were built, not taken from the build cache.
    pub builds: u64,
}

/// How a run went.
pub struct RunOutput {
    /// The steps that failed, in program order, with why. A failed step does
    /// not stop the run: later steps that do not depend on it still build.
    pub step_errors: Vec<(String, GeopError)>,
}

impl Runner {
    /// A runner of the operations of `registry`, of programs that read and
    /// place nothing.
    pub fn new(registry: OperationRegistry) -> Self {
        Self::of(Rc::new(registry), None, Location::default())
    }

    /// A runner of the operations of `registry`, of the program in the file
    /// of `location` of `workspace`, with a build cache of its own.
    pub fn of(
        registry: Rc<OperationRegistry>,
        workspace: Option<Rc<dyn Workspace>>,
        location: Location,
    ) -> Self {
        Self {
            registry,
            workspace,
            location,
            shared: Rc::default(),
        }
    }

    /// Runs the steps of `program` before the step `upto` — all of them, if
    /// `None` — on `part`, with the program's state and no inputs. The
    /// nodes of the steps after `upto` are kept for when they run again;
    /// those of steps no longer in the program are dropped.
    pub fn run(&self, part: &mut Part, program: &Program, upto: Option<&str>) -> RunOutput {
        self.run_with(part, program, &Settings::new(), upto)
    }

    /// [`Runner::run`], its parameters set to `inputs`.
    pub fn run_with(
        &self,
        part: &mut Part,
        program: &Program,
        inputs: &Settings,
        upto: Option<&str>,
    ) -> RunOutput {
        self.build(part, &program.steps, inputs, &program.state, upto)
    }

    /// Runs `steps` before `upto` on `part`, with `inputs` and `state`, and
    /// sweeps the build cache after a run of a part not itself placed.
    pub(crate) fn build(
        &self,
        part: &mut Part,
        steps: &[ProgramStep],
        inputs: &Settings,
        state: &StateValues,
        upto: Option<&str>,
    ) -> RunOutput {
        let order = steps.iter().map(|step| step.id.clone()).collect();
        part.begin(order, inputs, state);
        let end = (upto.and_then(|upto| steps.iter().position(|step| step.id == upto)))
            .unwrap_or(steps.len());
        let mut step_errors = Vec::new();
        let mut ids = HashSet::new();
        for (at, step) in steps.iter().enumerate().take(end) {
            part.begin_step(at);
            let ran = match ids.insert(step.id.as_str()) {
                true => (self.registry.get(&step.operation)).and_then(|operation| {
                    operation.run(&mut StepContext::new(self, part, &step.id, at), &step.args)
                }),
                // A step's id names what it produces: two steps of one id
                // could not be told apart.
                false => Err(GeopError::new(format!(
                    "step `{}` appears twice in the program",
                    step.id
                ))),
            };
            part.end_step(at);
            if let Err(e) = ran {
                step_errors.push((step.id.clone(), e));
            }
        }
        let failures = (step_errors.iter())
            .map(|(step, e)| (step.clone(), e.to_string()))
            .collect();
        part.end(failures);
        if self.location.within.is_empty() {
            self.shared.cache.borrow_mut().sweep();
        }
        RunOutput { step_errors }
    }

    /// Runs one step by hand, as the step `step` after those `part` ran,
    /// `body` declaring what it produces: what a test of one operation
    /// does. Fails as `body` does.
    pub fn run_step(
        &self,
        part: &mut Part,
        step: &str,
        body: impl FnOnce(&mut StepContext) -> GeopResult<()>,
    ) -> GeopResult<()> {
        let mut order = part.order().to_vec();
        if !order.iter().any(|id| id == step) {
            order.push(step.to_string());
        }
        let at = order.iter().position(|id| id == step).unwrap_or_default();
        let (inputs, state) = (part.inputs.clone(), part.state.clone());
        part.begin(order, &inputs, &state);
        part.begin_step(at);
        let ran = body(&mut StepContext::new(self, part, step, at));
        part.end_step(at);
        let failures = (ran.as_ref().err())
            .map(|e| (step.to_string(), e.to_string()))
            .into_iter()
            .collect();
        part.end(failures);
        ran
    }

    /// How much work it and the runners of the parts it placed did.
    pub fn counts(&self) -> Counts {
        self.shared.counts.get()
    }

    /// How many placed parts its build cache holds.
    pub fn cached_builds(&self) -> usize {
        self.shared.cache.borrow().len()
    }

    pub(crate) fn count_make(&self) {
        self.count(|counts| counts.makes += 1);
    }

    fn count(&self, add: impl FnOnce(&mut Counts)) {
        let mut counts = self.shared.counts.get();
        add(&mut counts);
        self.shared.counts.set(counts);
    }

    /// The runner of the part placed from the file of `location`.
    pub(crate) fn nested(&self, location: Location) -> Runner {
        Runner {
            registry: self.registry.clone(),
            workspace: self.workspace.clone(),
            location,
            shared: self.shared.clone(),
        }
    }

    /// The part built from `key`, from the build cache.
    pub(crate) fn cached(&self, key: &BuildKey) -> Option<Rc<Part>> {
        self.count(|counts| counts.placed += 1);
        (self.shared.cache.borrow()).get(key, |path| self.revision(path))
    }

    /// Keeps `part`, built from `key`, in the build cache.
    pub(crate) fn cache(&self, key: BuildKey, part: Rc<Part>) {
        self.count(|counts| counts.builds += 1);
        self.shared.cache.borrow_mut().insert(key, part);
    }

    /// The revision of the file at `path` of its workspace: 0 without one.
    pub fn revision(&self, path: &str) -> u64 {
        (self.workspace.as_ref()).map_or(0, |workspace| workspace.revision(path))
    }

    /// The workspace it reads from: an error, for a runner of programs that
    /// read and place nothing.
    fn workspace(&self) -> GeopResult<&dyn Workspace> {
        (self.workspace.as_deref()).ok_or_else(|| {
            GeopError::new("this program is run without a workspace: there are no files to read")
        })
    }

    /// The files the program it runs could read, as it names them: every
    /// file but its own. None without a workspace.
    pub fn files(&self) -> Vec<String> {
        let Ok(workspace) = self.workspace() else {
            return Vec::new();
        };
        let own = &self.location.file;
        (workspace.list().into_iter())
            .filter(|path| path != own)
            .map(|path| workspace.relative(own, &path))
            .collect()
    }

    /// The text of the file the program it runs names `name`, and its path.
    /// A make reads it through its reader, which records the read
    /// ([`crate::reader::Reader::read_file`]).
    pub fn read(&self, name: &str) -> GeopResult<(String, Rc<str>)> {
        let workspace = self.workspace()?;
        let path = workspace.resolve(&self.location.file, name);
        let text =
            (workspace.read(&path)).map_err(|e| e.with_context(format!("reading {path}")))?;
        Ok((path, text))
    }

    /// The program in the file the program it runs names `name`, and where
    /// it would be placed from here. Refuses a file placing the program it
    /// runs, however deep: the files would place themselves.
    pub fn program(&self, name: &str) -> GeopResult<(Rc<Program>, Location)> {
        let workspace = self.workspace()?;
        let path = workspace.resolve(&self.location.file, name);
        let Location { file, within } = &self.location;
        let within: Vec<String> = [within.clone(), vec![file.clone()]].concat();
        if within.contains(&path) {
            let cycle = within.iter().skip_while(|f| **f != path);
            let chain: Vec<&str> = cycle.map(String::as_str).chain([path.as_str()]).collect();
            return Err(GeopError::new(format!(
                "{file} places {path:?}, which places it back: {} — files must not place \
                 themselves, directly or through others",
                chain.join(" -> "),
            )));
        }
        let loaded =
            (workspace.program(&path)).map_err(|e| e.with_context(format!("placing {path}")))?;
        Ok((loaded, Location { file: path, within }))
    }
}
