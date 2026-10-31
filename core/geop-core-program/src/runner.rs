//! [`Runner`]: runs a program on a part ([`Runner::run`]) — every step's
//! operation, in order, each declaring what it produces — and builds the
//! parts it places, sharing equal builds. What a run made of the targets
//! kept between processes it notes in its workspace's cache, to be written
//! when the cache is next flushed (see [`crate::kept`]).

use std::{
    cell::{Cell, RefCell},
    collections::{HashMap, HashSet},
    rc::Rc,
};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    kept::{Digest, Made, digest_text},
    operation_registry::OperationRegistry,
    part::{Part, StepContext},
    placed::{BuildCache, BuildKey},
    program::{Program, ProgramStep},
    reader::Reader,
    target::Read,
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
    /// The digest of each file's text, by its path, and the revision it is
    /// of: what a node that read the file was made from, in any process.
    files: RefCell<HashMap<String, (u64, Option<Digest>)>>,
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
    /// The nodes taken from what an earlier process kept, rather than made
    /// (see [`crate::kept`]).
    pub restored: u64,
    /// The makes of targets kept between processes: what reopening a
    /// program that did not change does none of.
    pub kept_makes: u64,
}

/// `names`, each in backticks, joined: `` `a`, `b` ``.
pub(crate) fn quoted(names: &[&str]) -> String {
    let quoted: Vec<String> = names.iter().map(|name| format!("`{name}`")).collect();
    quoted.join(", ")
}

thread_local! {
    static STEPS_RUN: Cell<usize> = const { Cell::new(0) };
}

/// How many steps this thread has run, of every program and every part
/// placed — each walked whether or not it made anything again: what a run
/// costs at least, so a test can bound the runs an edit takes by a count.
pub fn steps_run() -> usize {
    STEPS_RUN.get()
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
        self.build(part, &program.steps, inputs, &program.state, upto, None)
    }

    /// Runs `steps` before `upto` on `part`, with `inputs` and `state` —
    /// placed in the part `outside` reads, if it is placed — and sweeps the
    /// build cache after a run of a part not itself placed.
    pub(crate) fn build(
        &self,
        part: &mut Part,
        steps: &[ProgramStep],
        inputs: &Settings,
        state: &StateValues,
        upto: Option<&str>,
        outside: Option<Rc<Reader>>,
    ) -> RunOutput {
        let end = (upto.and_then(|upto| steps.iter().position(|step| step.id == upto)))
            .unwrap_or(steps.len());
        // Nothing changed since it last ran: every step would reuse what
        // it made, and walking them all to find that is a run's cost — an
        // editor runs its program several times an event of a drag. Not for
        // a part that read the part placing it (`..`), which may have
        // changed since.
        if part.ran_as(steps, end, inputs, state)
            && part.outside_read.is_empty()
            && (part.files_read.iter()).all(|(path, read)| self.revision(path) == *read)
        {
            return RunOutput {
                step_errors: Vec::new(),
            };
        }
        let order = steps.iter().map(|step| step.id.clone()).collect();
        part.begin(order, inputs, state);
        let mut step_errors = Vec::new();
        let mut ids = HashSet::new();
        for (at, step) in steps.iter().enumerate().take(end) {
            STEPS_RUN.set(STEPS_RUN.get() + 1);
            part.begin_step(at);
            let ran = match ids.insert(step.id.as_str()) {
                true => self.run_step_of(part, step, at, outside.clone()),
                // A step's id names what it produces: two steps of one id
                // could not be told apart.
                false => Err(GeopError::new(format!(
                    "step `{}` appears twice in the program",
                    step.id
                ))),
            };
            part.end_step(at);
            if let Err(e) = ran {
                // What fails for what a step suppressed did not make says
                // which steps those are.
                let suppressed = part.suppressed_before(at);
                let e = match suppressed.is_empty() {
                    true => e,
                    false => {
                        e.with_context(format!("built with {} suppressed", quoted(&suppressed)))
                    }
                };
                step_errors.push((step.id.clone(), e));
            }
        }
        let failures = (step_errors.iter())
            .map(|(step, e)| (step.clone(), e.to_string()))
            .collect();
        part.end(failures);
        part.ran_through(steps, end);
        if self.location.within.is_empty() {
            self.shared.cache.borrow_mut().sweep();
            self.keep(part);
        }
        RunOutput { step_errors }
    }

    /// Runs `step`, at position `at`, on `part` — unless it is suppressed
    /// (see [`crate::program::ProgramStep::active`]): then it produces
    /// nothing, and the steps after it see the part as the steps before it
    /// left it.
    fn run_step_of(
        &self,
        part: &mut Part,
        step: &ProgramStep,
        at: usize,
        outside: Option<Rc<Reader>>,
    ) -> GeopResult<()> {
        let mut ctx = StepContext::new(self, part, &step.id, at, outside);
        if !ctx.active(step.active.as_deref())? {
            part.suppress(&step.id);
            return Ok(());
        }
        let operation = self.registry.get(&step.operation)?;
        operation.run(&mut ctx, &step.args)
    }

    /// Notes what `part`, built from the program it runs, and the parts
    /// placed in it, however deep, hold of the targets kept between
    /// processes in the cache of its workspace, each by the file of the
    /// program that built it: what was made since the cache was written is
    /// written when it is next flushed ([`crate::cache::Cache::flush`]).
    fn keep(&self, part: &Part) {
        let Some(cache) = self
            .workspace
            .as_deref()
            .and_then(|workspace| workspace.cache())
        else {
            return;
        };
        let mut seen = HashSet::new();
        let mut parts = vec![(self.location.file.clone(), part)];
        while let Some((path, part)) = parts.pop() {
            for (_, _, placed) in part.placed() {
                if seen.insert(Rc::as_ptr(&placed.part)) {
                    parts.push((placed.location.file.clone(), &placed.part));
                }
            }
            cache.builds(&path, |builds| {
                builds.order(part.order());
                for (name, node) in part.kept_nodes() {
                    let (Some((derivation, digest)), Some(write), Ok(value)) =
                        (&node.kept, node.write, &node.value)
                    else {
                        continue;
                    };
                    if node.noted.replace(true) || builds.has(&node.step, name, derivation, *digest)
                    {
                        continue;
                    }
                    let value = value.clone();
                    builds.add(Made {
                        step: node.step.clone(),
                        target: name.to_string(),
                        kind: node.kind_name,
                        derivation: derivation.clone(),
                        digest: *digest,
                        write: Box::new(move |pool| write(&*value, pool)),
                    });
                }
            });
        }
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
        let ran = body(&mut StepContext::new(self, part, step, at, None));
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

    /// Counts a make, of a target `kept` between processes or not.
    pub(crate) fn count_make(&self, kept: bool) {
        self.count(|counts| {
            counts.makes += 1;
            counts.kept_makes += u64::from(kept);
        });
    }

    pub(crate) fn count_restored(&self) {
        self.count(|counts| counts.restored += 1);
    }

    fn count(&self, add: impl FnOnce(&mut Counts)) {
        let mut counts = self.shared.counts.get();
        add(&mut counts);
        self.shared.counts.set(counts);
    }

    /// The runner of the part placed from the file of `location`: what an
    /// editor edits a step of it with.
    pub fn nested(&self, location: Location) -> Runner {
        Runner {
            registry: self.registry.clone(),
            workspace: self.workspace.clone(),
            location,
            shared: self.shared.clone(),
        }
    }

    /// The part built from `key`, from the build cache: one whose reads of
    /// the part it is placed in `holds` says hold.
    pub(crate) fn cached(&self, key: &BuildKey, holds: impl Fn(&Read) -> bool) -> Option<Rc<Part>> {
        self.count(|counts| counts.placed += 1);
        (self.shared.cache.borrow()).get(key, |path| self.revision(path), holds)
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

    /// The digest of the text of the file at `path` of its workspace, as it
    /// is now: what a node that read it was made from, in any process. None
    /// without a workspace, or for a file it does not have.
    pub(crate) fn file_digest(&self, path: &str) -> Option<Digest> {
        let workspace = self.workspace.as_deref()?;
        let revision = workspace.revision(path);
        if let Some((read, digest)) = self.shared.files.borrow().get(path)
            && *read == revision
        {
            return *digest;
        }
        let digest = workspace.digest(path).map(Digest);
        (self.shared.files.borrow_mut()).insert(path.to_string(), (revision, digest));
        digest
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

    /// The text of the file the program it runs names `name`, and its path:
    /// a file of bytes read as UTF-8, what is not replaced (see
    /// [`crate::workspace::text`]). A make reads it through its reader,
    /// which records the read ([`crate::reader::Reader::read_file`]).
    pub fn read(&self, name: &str) -> GeopResult<(String, Rc<str>)> {
        let (path, content) = self.content(name)?;
        let text = match crate::workspace::text(&content) {
            std::borrow::Cow::Borrowed(_) => content,
            std::borrow::Cow::Owned(text) => text.into(),
        };
        Ok((path, text))
    }

    /// The bytes of the file the program it runs names `name`, and its
    /// path (see [`crate::workspace::bytes`]): what a reader of a binary
    /// format reads ([`crate::reader::Reader::read_bytes`]).
    pub fn read_bytes(&self, name: &str) -> GeopResult<(String, Rc<[u8]>)> {
        let (path, content) = self.content(name)?;
        let bytes = crate::workspace::bytes(&content)
            .map_err(|e| e.with_context(format!("reading the bytes of {path}")))?;
        Ok((path, bytes.into()))
    }

    /// The content of the file the program it runs names `name`, as its
    /// workspace holds it, and its path.
    fn content(&self, name: &str) -> GeopResult<(String, Rc<str>)> {
        let workspace = self.workspace()?;
        let path = workspace.resolve(&self.location.file, name);
        let content =
            (workspace.read(&path)).map_err(|e| e.with_context(format!("reading {path}")))?;
        // What it was read as is what a node reading it was made from: its
        // digest is taken now, not by reading it again.
        let digest = Some(Digest(digest_text(&content)));
        let revision = workspace.revision(&path);
        (self.shared.files.borrow_mut()).insert(path.clone(), (revision, digest));
        Ok((path, content))
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
