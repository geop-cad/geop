use std::{collections::HashSet, rc::Rc};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::{
    operation_registry::OperationRegistry,
    part::Part,
    program::Program,
    target::Revision,
    values::{Settings, StateValues},
    workspace::{Location, Workspace},
};

/// Runs programs on parts, with the operations of `registry`, reading
/// what they place and import from `workspace` as the program in the file
/// of `location` names it. An operation is given the runner its step is
/// run by, to read with and to run programs of its own: the parts of
/// instances ([`ProgramRunner::load_subpart`]).
pub struct ProgramRunner {
    pub registry: Rc<OperationRegistry>,
    pub workspace: Option<Rc<dyn Workspace>>,
    pub location: Location,
}

/// What a run shares with the last, which ran its program to its end and
/// built every step.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Same {
    /// It runs the very same program, as the program of the same file,
    /// and no file changed since.
    pub(crate) program: bool,
}

/// How a run went.
pub struct RunOutput {
    /// The steps that failed, in program order, with why. A failed step does
    /// not stop the run: later steps that do not depend on it still build.
    pub step_errors: Vec<(String, GeopError)>,
}

impl ProgramRunner {
    /// A runner of the operations of `registry`, of programs that read and
    /// place nothing.
    pub fn new(registry: OperationRegistry) -> Self {
        Self {
            registry: Rc::new(registry),
            workspace: None,
            location: Location::default(),
        }
    }

    /// Runs `program` on `part` with the given `inputs` and `state`, which
    /// are complete: what a previous run gave and this one does not is
    /// removed.
    pub fn run(
        &self,
        part: &mut Part,
        program: &Program,
        inputs: &Settings,
        state: &StateValues,
    ) -> RunOutput {
        self.run_until(part, program, program.steps.len(), inputs, state)
    }

    /// Runs the first `end` steps of `program` on `part`, as the editor
    /// does up to its marker. What the later steps made is kept, to be
    /// reused when they run again.
    pub fn run_until(
        &self,
        part: &mut Part,
        program: &Program,
        end: usize,
        inputs: &Settings,
        state: &StateValues,
    ) -> RunOutput {
        let revision = part.next_run();
        self.run_as(part, revision, program, end, inputs, state, Same::default())
    }

    /// How many times a file of its workspace changed: none, without one.
    pub(crate) fn revision(&self) -> u64 {
        self.workspace
            .as_ref()
            .map_or(0, |workspace| workspace.revision())
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

    /// Loads the part of the instance `name` into `part`, in the step being
    /// run: runs the program in the file the program it runs names `file`
    /// (see [`ProgramRunner::program`]), in this runner's operations, on the
    /// part it built last for the instance — with `inputs`, and with the
    /// state `part` gives under the instance's name (`b.pose` for the state
    /// `name/b.pose`), the file's own where `part` gives none — and keeps
    /// it, to run again in the next run that loads it: what reads what
    /// changed is built again, the rest is reused (see
    /// [`crate::instance`]). Read it with [`crate::part::Reader::inside`].
    /// Fails if a step loaded an instance of that name before in this run,
    /// or if a step of the part fails — after loading it all the same, as
    /// far as it builds.
    pub fn load_subpart(
        &self,
        part: &mut Part,
        name: &str,
        file: &str,
        inputs: &Settings,
    ) -> GeopResult<()> {
        let ctx = |e: GeopError| e.with_context(format!("loading the part of `{name}`"));
        let (program, location) = self.program(file).map_err(ctx)?;
        let runner = ProgramRunner {
            registry: self.registry.clone(),
            workspace: self.workspace.clone(),
            location,
        };
        part.load_subpart(&runner, name, &program, inputs)
    }

    /// Runs the first `end` steps of `program` on `part` as the run
    /// `revision` (see [`ProgramRunner::run_until`]). The part of an
    /// instance is run as the run of the part it is in, so that when each
    /// version was made means the same in both. Given nothing otherwise than
    /// the last run, which ran it all and built every step, a run carries
    /// over what that built, without running the steps again ([`Same`]).
    #[allow(
        clippy::too_many_arguments,
        reason = "a run's own arguments, and whether it may carry the last over"
    )]
    pub(crate) fn run_as(
        &self,
        part: &mut Part,
        revision: Revision,
        program: &Program,
        end: usize,
        inputs: &Settings,
        state: &StateValues,
        same: Same,
    ) -> RunOutput {
        let (run, later) = program.steps.split_at(end.min(program.steps.len()));
        let last = part.last_built();
        let given_otherwise = part.begin_run_as(revision, inputs, state);
        // Every version reads only what the program, its inputs, its state
        // and the files it reads give, which the steps read again: all as
        // they were, each would be reused — the files too, since a step
        // passes what it read to what it defines.
        if same.program && !given_otherwise {
            part.carry(last);
            return RunOutput {
                step_errors: Vec::new(),
            };
        }
        let mut step_errors = Vec::new();
        let mut ids = HashSet::new();
        for step in run {
            part.begin_step();
            let ran = match ids.insert(step.id.as_str()) {
                true => (self.registry.get(&step.operation))
                    .and_then(|operation| operation.run(&step.id, self, part, &step.args)),
                // A step's id names what it builds: two steps of one id
                // could not be told apart.
                false => Err(GeopError::new(format!(
                    "step `{}` appears twice in the program",
                    step.id
                ))),
            };
            if let Err(e) = ran {
                step_errors.push((step.id.clone(), e));
            }
        }
        part.end_run(later.len());

        RunOutput { step_errors }
    }
}
