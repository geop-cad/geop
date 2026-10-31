//! [`Editor`]: the whole of editing a program, as one state machine an
//! editor drives with [`Command`]s and draws from the [`Update`]s it
//! answers.
//!
//! It holds the program, its undo and redo, how far it runs, and the step
//! being edited, and it builds whatever is shown — so an editor keeps no
//! state of its own beyond the camera, renders what it is sent, and sends
//! what the user did.
//!
//! What it offers is a [`Registry`]'s: the operations a step can be, how a
//! part is drawn and listed, the exporters and inspectors, the panels, the
//! tools and what settles the program's state after an edit. None of it is
//! named here: an editor of a registry of three operations, with no panel
//! and no settler, edits programs of those three.
//!
//! The program is the one of a file, among others it may place parts from
//! (see [`geop_core_program::workspace`]): the editor is told the file's path
//! with the program ([`Command::Load`]), and is sent the other files'
//! programs whenever they change ([`Command::Files`]) — by the front end,
//! which owns the files, be they a VS Code workspace's or the browser's.
//! Only the file it edits is its own: switching to another, it keeps the
//! program it leaves as that file's. What each step shows and does is the
//! operation's; see [`crate::step_editor::StepEditor`].

mod build;
mod context;
mod kept;
mod protocol;
#[cfg(test)]
mod tests;

pub use context::{Changed, EditorContext, Rerun};
pub use protocol::{
    Command, File, PlacedSteps, ProgramState, SceneState, StepInfo, StepState, Update,
};

use geop_core_program::{
    cache::Cache,
    element::PickedElement,
    operation_registry::OperationRegistry,
    part::Part,
    placed::{INSTANCE_SEPARATOR, Located, OUTSIDE, PlacedPart},
    program::{Program, ProgramStep},
    reader::{Reader, Seen},
    runner::Runner,
    values::StateValues,
    workspace::{FileStoreMut, Files, Location, Workspace},
};
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet, HashMap},
    rc::Rc,
};

use crate::{
    editable::Against,
    editing::{Extent, Presentation, StepEditEvent},
    output::{Content, Export, Inspection, Output},
    registry::{Drawings, Registry},
    step_editor::{Editing, StepEditor},
    tools::Tool,
    view::{EMPTY_EXTENT, PartView, ViewInstance, Visible, drawn},
};
use build::Build;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{CoordinateSystem, Pose},
    scalars::Scalar,
    vector::Vector3,
};
use kept::Kept;

/// How many files' builds an editor keeps (see `Editor::builds`): each
/// holds every step's part, so a few large files are as many as fit.
const KEPT_BUILDS: usize = 6;

/// A part drawn, by what it was made of (see [`Reader::seen`]).
type Drawing = (Seen, Rc<PartView>);

/// How many parts drawn an editor keeps whole (see `Editor::views`): the
/// part before the step edited, the part with it, and those of the last
/// steps opened.
const KEPT_VIEWS: usize = 4;

/// The files an editor reads the programs it places parts from, and
/// writes the files it is sent to: whatever its front end keeps, with
/// whatever library its maker put beside them.
type EditorFiles = Files<Box<dyn FileStoreMut>>;

/// A step being edited, as one edit left it: its arguments, and the
/// program's state.
type Edited = (serde_json::Value, StateValues);

/// The step being edited.
struct Open {
    /// Where it is, or goes.
    index: usize,
    /// Its id — for a new one, the id it gets once committed.
    id: String,
    /// Whether it is a new step, inserted at `index`, rather than the one
    /// already there.
    new: bool,
    /// The program with the step in it, as its edits leave it: what the
    /// step editor writes the step's arguments to — and its state, as the
    /// step's edits and the solves after them leave it.
    program: Program,
    editor: StepEditor,
    /// What picks test against, as drawn: the part before it, or what it
    /// builds (see [`crate::editable::OperationInfo::picks_built`]).
    view: Rc<PartView>,
    /// The step as it was before each edit made to it since it was opened,
    /// with the program's state: what undo goes back to while it is edited.
    undo: Vec<Edited>,
    redo: Vec<Edited>,
    /// The step as it was when the drag going on started: a drag is one
    /// edit, however many events it takes.
    dragging: Option<Edited>,
    /// For a step of a part placed in the program, where it is placed:
    /// `program` is then that part's, and the program run is the file's.
    within: Option<Within>,
}

/// A step of a part placed in the program, edited where it is placed: the
/// program run is the file's, the placed part's program — the step's edits
/// in it — put in the workspace in place of its file, so the part placed
/// is built anew and nothing else is. Its edits are made in the placed
/// part's own coordinates: what the pointer does is moved there, and what
/// the step shows moved out to where the part is placed.
struct Within {
    /// The instance it is placed as.
    instance: String,
    /// Where its program is: its file, and the files placing it.
    location: Location,
    /// The file's text when the step was opened: what cancelling puts back.
    text: String,
    /// Its program's runner.
    runner: Runner,
    /// The part placed, as the last run built it.
    part: Rc<Part>,
    /// Where it is placed in the part drawn.
    pose: Pose,
}

impl Open {
    /// The step, as its edits leave it.
    fn step(&self) -> GeopResult<&ProgramStep> {
        self.program.step(&self.id)
    }

    /// The step, and the program's state, as they are now: what an undo
    /// comes back to.
    fn edited(&self) -> GeopResult<Edited> {
        Ok((self.step()?.args.clone(), self.program.state.clone()))
    }
}

/// A step of a placed part to edit where it is placed: one of its steps, by
/// id, or a new one of the operation `kind`, after them.
#[derive(Debug)]
enum PlacedStep {
    Existing(String),
    New(String),
}

/// The tool in hand, and the key it was taken in hand by.
struct InHand {
    key: String,
    tool: Box<dyn Tool>,
}

/// Which part is drawn, and without what — or, for a step edited on a
/// sheet of its own, no part, the viewer framing the sheet: when it is the
/// same as last time, the scene is not sent again. The part is told by
/// what it is made of (see [`Reader::seen`]), not by when it was last
/// built: the part before a sketch is the same however the sketch changes,
/// and a whole assembly is not sent again for every line drawn on it.
#[derive(Clone, PartialEq)]
struct Shown {
    seen: Seen,
    steps: usize,
    hidden: Vec<String>,
    /// The colours placed parts are drawn in, by name (see
    /// [`Editor::colors`]).
    colors: BTreeMap<String, String>,
    /// The sheet's centre and size.
    sheet: Option<([f64; 3], f64)>,
}

/// A placed part as the viewer was sent it: the key of the component it is
/// drawn from, where it is, and the colour it is drawn in.
#[derive(PartialEq)]
struct Sent {
    component: String,
    frame: [f64; 12],
    color: Option<String>,
}

/// Editing a program: see the module docs.
pub struct Editor {
    /// What it offers.
    registry: Rc<Registry>,
    /// Its operations, as every runner it makes runs them.
    operations: Rc<OperationRegistry>,
    file: Program,
    /// The file the program is in, once told: until then, the files it
    /// places are named relative to the workspace's top folder.
    path: Option<String>,
    /// The other program files, which it places parts from.
    workspace: Rc<EditorFiles>,
    undo: Vec<Program>,
    redo: Vec<Program>,
    marker: Option<usize>,
    open: Option<Open>,
    preview: bool,
    build: Build,
    /// The builds of the files edited before, by path, the latest last:
    /// opened again, a file takes up its steps as they were built, rather
    /// than building them all again. At most [`KEPT_BUILDS`].
    builds: Vec<(String, Build)>,
    /// Counts runs (see [`EditorContext::runs`]).
    run: u64,
    /// What each step that ran builds on (see [`crate::editing::Dialog::picked`]),
    /// with the step it was read from: what a step picked is in its
    /// arguments, so it is read anew only when the step changed — not when
    /// a drag only moved the program's state.
    references: Vec<(ProgramStep, Vec<PickedElement>)>,
    shown: Option<Shown>,
    /// The files the last command added, for the update to say.
    added: Option<Vec<File>>,
    /// The other files the last command changed, for the update to carry.
    written: Vec<File>,
    /// The file the last command wrote, for the update to carry.
    exported: Option<Export>,
    /// The placed parts the viewer was sent, by name, as sent — `None`
    /// when the viewer may have none (see [`SceneState`]).
    placed: Option<HashMap<String, Sent>>,
    /// What the viewer keeps of the scenes it was sent: the components and
    /// the meshes of faces, by key.
    kept: Kept,
    /// The tool in hand, if one is.
    tool: Option<InHand>,
    /// The inspector the last command asked, and what it told it: to answer
    /// once it has run.
    asked: Option<(String, serde_json::Value, Vec<PickedElement>)>,
    /// The part as last drawn: what a tool picks from.
    drawn: Option<Rc<PartView>>,
    /// The last parts drawn, by what they were made of (see
    /// [`Reader::seen`]): a part shown again is the view it was — opening
    /// a step and cancelling it shows the same part. At most
    /// [`KEPT_VIEWS`], the last drawn last.
    views: RefCell<Vec<Drawing>>,
    /// What each drawer drew of the parts drawn, by what it read: a part
    /// drawn with other sketches than before — the part before a sketch
    /// opened — draws its sketches again, not its faces.
    drawings: Drawings,
    /// The steps of the parts placed, as listed, by file and build (see
    /// [`Editor::placed_steps`]).
    listed: RefCell<HashMap<(String, *const Part), ListedBuild>>,
}

/// A placed part's steps as listed, and the build and program they list:
/// listed again only for another build or another program.
struct ListedBuild {
    part: Rc<Part>,
    program: Rc<Program>,
    steps: Vec<StepInfo>,
}

impl Editor {
    /// An editor of what `registry` offers, editing a program of no steps,
    /// the files it places parts from read from `files`, what they are made
    /// into kept in memory.
    pub fn new(registry: Registry, files: impl FileStoreMut + 'static) -> Self {
        Self::with_cache(registry, files, Cache::new())
    }

    /// [`Editor::new`], what its files are made into kept in `cache`: a
    /// host process's, kept on disk too.
    pub fn with_cache(
        registry: Registry,
        files: impl FileStoreMut + 'static,
        cache: Cache,
    ) -> Self {
        let operations = Rc::new(registry.operations());
        let files: Box<dyn FileStoreMut> = Box::new(files);
        Self {
            workspace: Files::with_cache(files, operations.clone(), cache),
            build: Build::new(&operations),
            registry: Rc::new(registry),
            operations,
            file: Program::default(),
            path: None,
            undo: Vec::new(),
            redo: Vec::new(),
            marker: None,
            open: None,
            preview: true,
            builds: Vec::new(),
            run: 0,
            references: Vec::new(),
            shown: None,
            added: None,
            written: Vec::new(),
            exported: None,
            placed: None,
            kept: Kept::default(),
            tool: None,
            asked: None,
            drawn: None,
            views: RefCell::default(),
            drawings: Drawings::default(),
            listed: RefCell::default(),
        }
    }

    pub fn program(&self) -> &Program {
        &self.file
    }

    /// Forgets what the viewer was sent: the next update sends all of the
    /// scene, as to a viewer that has nothing — one starting afresh, or one
    /// that never saw the updates someone else asked for (see
    /// [`crate::agent::Session::new`]).
    pub fn forget_viewer(&mut self) {
        self.shown = None;
        self.placed = None;
    }

    /// What it offers.
    pub fn registry(&self) -> &Registry {
        &self.registry
    }

    /// The part the program builds, as far as it runs now.
    pub fn part(&self) -> Reader<'_> {
        self.build.part.current()
    }

    /// The step being edited, with its arguments as they now are.
    pub fn editing(&self) -> Option<&ProgramStep> {
        self.open.as_ref().and_then(|open| open.step().ok())
    }

    /// The step editor of the step being edited: its operation's editing
    /// state among what it keeps.
    pub fn step_editor(&self) -> Option<&StepEditor> {
        self.open.as_ref().map(|open| &open.editor)
    }

    /// The tool in hand, if it is a `T`.
    pub fn tool<T: Tool>(&self) -> Option<&T> {
        let tool: &dyn std::any::Any = self.tool.as_ref()?.tool.as_ref();
        tool.downcast_ref()
    }

    /// [`Editor::handle`] on the wire: `command` is a [`Command`] as JSON,
    /// and the answer an [`Update`] as JSON. Shared by every front end that
    /// is not Rust (the browser's wasm module, the VS Code host process),
    /// so they all speak the same protocol. `Err` is a message that is not
    /// itself an update: the command could not be read, or the update not
    /// written.
    pub fn handle_json(&mut self, command: &str) -> Result<String, String> {
        let command: Command =
            serde_json::from_str(command).map_err(|e| format!("reading command: {e}"))?;
        serde_json::to_string(&self.handle(command)).map_err(|e| format!("writing update: {e}"))
    }

    /// Tells the runner the workspace the program reads from, and the file
    /// it is in there.
    fn give(&mut self) {
        let path = self.path.clone().unwrap_or_default();
        self.build.runner.workspace = Some(self.workspace.clone());
        self.build.runner.location = Location {
            file: geop_core_program::workspace::resolve("", &path),
            within: Vec::new(),
        };
    }

    /// Runs the first `end` of the steps run now — all of them, if `None` —
    /// with their state (see [`Self::current`]), keeping how far it ran and
    /// why steps failed.
    fn run_to(&mut self, end: Option<usize>) {
        self.run_staged(end, self.preview);
    }

    /// [`Self::run_to`], a step of a placed part edited put in its file
    /// first — as edited, if `edited`, else as it was — and the part placed
    /// taken as built. As edited, the file ends at the step, as a step of
    /// the program's own is edited: the steps after it are built once the
    /// edit is done (`whole`).
    fn run_staged(&mut self, end: Option<usize>, edited: bool) {
        self.run_placed(end, edited, false);
    }

    /// [`Self::run_staged`], a placed part's file — as edited — whole, if
    /// `whole`: what a commit builds.
    fn run_placed(&mut self, end: Option<usize>, edited: bool, whole: bool) {
        self.give();
        if let Some(open) = &self.open
            && let Some(within) = &open.within
        {
            let mut program = open.program.clone();
            if !whole {
                program.steps.truncate(open.index + 1);
            }
            let text = match edited {
                true => program.to_json().ok(),
                false => None,
            };
            let text = text.unwrap_or_else(|| within.text.clone());
            self.workspace.write(&within.location.file, Some(text));
        }
        let program = match &self.open {
            Some(open) if open.within.is_none() => &open.program,
            _ => &self.file,
        };
        self.build.run(program, end);
        if let Some(open) = &mut self.open
            && let Some(within) = &mut open.within
            && let Some((_, _, placed)) = (self.build.part.placed().into_iter())
                .find(|(instance, _, _)| *instance == within.instance)
        {
            within.part = placed.part.clone();
        }
    }

    /// The steps run now, and the state they run with: the step being
    /// edited's, or the program's — for a step of a placed part, the
    /// program's.
    fn current(&self) -> &Program {
        match &self.open {
            Some(open) if open.within.is_none() => &open.program,
            _ => &self.file,
        }
    }

    /// The runner and the part the step `open` is edited with: the
    /// program's, or, for a step of a placed part, that part's, placed in
    /// the program's part where it is (see [`Against`]).
    fn edited_with<'s>(&'s self, open: &'s Open) -> (&'s Runner, Against<'s>) {
        match &open.within {
            Some(within) => (&within.runner, self.placed_against(within)),
            None => (&self.build.runner, Against::from(&self.build.part)),
        }
    }

    /// The part placed as `within` describes, placed in the program's part
    /// where it is: what a step of it reads there in context.
    fn placed_against<'s>(&'s self, within: &'s Within) -> Against<'s> {
        Against {
            part: &within.part,
            outside: self.build.part.placing(&within.instance),
        }
    }

    /// Why the step `open` does not build, if it does not.
    fn step_error(&self, open: &Open) -> Option<String> {
        match &open.within {
            Some(within) => (within.part.failures().iter())
                .find(|(step, _)| *step == open.id)
                .map(|(_, e)| e.clone()),
            None => self.build.step_error(open.index, &open.id),
        }
    }

    /// Applies `command`, and says what to show now.
    pub fn handle(&mut self, command: Command) -> Update {
        // What undo goes back to — not taken for what is sent as often as
        // the pointer moves, and cannot change the program; a key can, by
        // committing the step. Another file's program is not something to
        // undo back to.
        let keeps_program = match &command {
            Command::Event { event } => !matches!(event, StepEditEvent::Key { .. }),
            Command::Preview { .. } => true,
            _ => false,
        };
        let before = (!keeps_program).then(|| (self.file.clone(), self.path.clone()));
        let result = self.apply(command);
        if matches!(result, Ok(Changed::Program | Changed::Display))
            && let Some((before, path)) = before
            && path == self.path
            && self.file != before
        {
            self.undo.push(before);
            self.redo.clear();
        }
        if !matches!(result, Ok(Changed::Nothing | Changed::Display)) {
            // Settling runs the whole program, so what is shown runs after.
            self.settle();
            self.rerun();
        }
        if matches!(result, Ok(Changed::Program | Changed::Step)) {
            self.after_edit();
        }
        let step = self.step_state();
        let steps = self.shown_steps(step.as_ref());
        let mut hidden = self.hidden(step.as_ref().map(|s| &s.presentation.pickable[..]), steps);
        // What the step being edited asks to hide of what it builds.
        if let Some(step) = &step {
            hidden.extend(step.presentation.hidden.iter().cloned());
        }
        // Sorted: what a pick is made in looks names up in it (see
        // [`Visible`]).
        hidden.sort();
        hidden.dedup();
        let shown = Shown {
            seen: self.shown_part(steps).seen(),
            steps,
            hidden,
            colors: self.file.display.colors.clone(),
            sheet: (step.as_ref())
                .and_then(|s| s.presentation.sheet)
                .map(|e| (e.center.to_array().map(|c| c.to_f64()), e.size.to_f64())),
        };
        let scene = (self.shown.as_ref() != Some(&shown)).then(|| self.scene(shown));
        let mut error = result.as_ref().err().map(|e| e.to_string());
        let inspection = match self.answer(step.is_none()) {
            Ok(inspection) => inspection,
            Err(e) => {
                error = Some(e.to_string());
                None
            }
        };
        let program = match matches!(
            result,
            Ok(Changed::Program | Changed::Run | Changed::Display)
        ) {
            true => match self.program_state() {
                Ok(program) => Some(program),
                Err(e) => {
                    error = Some(e.to_string());
                    None
                }
            },
            false => None,
        };
        // What an edit built is kept once it is done — not while a step
        // being edited previews it, each event anew.
        if self.open.is_none()
            && let Some(cache) = self.workspace.cache()
        {
            cache.flush();
        }
        Update {
            error,
            program,
            scene,
            tool: self.tool_presentation(step.is_none()),
            step,
            files: self.added.take(),
            written: (!self.written.is_empty()).then(|| std::mem::take(&mut self.written)),
            inspection,
            export: self.exported.take(),
        }
    }

    /// What to draw, `shown`: the part, and of the parts placed in it what
    /// the viewer was not sent yet.
    fn scene(&mut self, shown: Shown) -> SceneState {
        self.shown = Some(shown.clone());
        let part = match shown.sheet {
            Some((center, size)) => Rc::new(PartView::blank(Extent {
                center: Vector3::from_array(center.map(Scalar::from_f64)),
                size: Scalar::from_f64(size),
            })),
            None => self.view_of(shown.steps),
        };
        let before = self.placed.take();
        let all = before.is_none();
        let before = before.unwrap_or_default();
        if all {
            self.kept.clear();
        }
        let sent = self.kept.send(&part);
        let mut placed = HashMap::new();
        let mut instances = Vec::new();
        for instance in &part.instances {
            let f = &instance.frame;
            let frame = [f.origin(), f.u(), f.v(), f.w()].map(|p| p.to_array().map(|c| c.to_f64()));
            let mut numbers = [0.0; 12];
            for (number, x) in numbers.iter_mut().zip(frame.iter().flatten()) {
                *number = *x;
            }
            let color = shown.colors.get(&instance.name).cloned();
            let drawn = Sent {
                component: instance.component.clone(),
                frame: numbers,
                color: color.clone(),
            };
            if before.get(&instance.name) != Some(&drawn) {
                instances.push(ViewInstance {
                    color,
                    ..instance.clone()
                });
            }
            placed.insert(instance.name.clone(), drawn);
        }
        let mut removed: Vec<String> = (before.keys())
            .filter(|name| !placed.contains_key(*name))
            .cloned()
            .collect();
        removed.sort();
        self.placed = Some(placed);
        self.drawn = Some(part.clone());
        let structure = (self.registry)
            .structure(&self.shown_part(shown.steps), &shown.hidden)
            .unwrap_or_default();
        SceneState {
            part,
            structure,
            hidden: shown.hidden,
            instances,
            all,
            removed,
            components: sent.components,
            forgotten: sent.forgotten,
            meshes: sent.meshes,
            forgotten_meshes: sent.forgotten_meshes,
        }
    }

    /// `ask` answered of the outputs offered and what was built, as an
    /// exporter or inspector is given it, dated `date`.
    fn ask<T>(
        &mut self,
        date: &str,
        ask: impl FnOnce(&crate::output::Outputs, &mut Output) -> GeopResult<T>,
    ) -> GeopResult<T> {
        let editing = (self.open.as_ref())
            .filter(|open| open.within.is_none())
            .map(|open| open.id.clone());
        let file = match &self.open {
            Some(open) if open.within.is_none() => &open.program,
            _ => &self.file,
        };
        let mut output = Output {
            runner: &self.build.runner,
            part: &mut self.build.part,
            file,
            editing: editing.as_deref(),
            date,
        };
        ask(self.registry.outputs(), &mut output)
    }

    /// The answer to the inspector the last command asked, if it asked one
    /// — else, `idle` (no step edited), the tool in hand's (see
    /// [`Tool::inspection`]).
    fn answer(&mut self, idle: bool) -> GeopResult<Option<Inspection>> {
        if let Some((kind, options, picked)) = self.asked.take() {
            return self.ask("", |outputs, output| {
                let inspector = outputs.get_inspector(&kind)?;
                inspector.inspect(output, &picked, &options).map(Some)
            });
        }
        let Some(mut in_hand) = self.tool.take().filter(|_| idle) else {
            return Ok(None);
        };
        let inspection = in_hand.tool.inspection(&mut EditorContext::new(self));
        self.tool = Some(in_hand);
        inspection
    }

    /// What the tool in hand shows, if one is and no step is edited.
    fn tool_presentation(&self, idle: bool) -> Option<Presentation> {
        let in_hand = self.tool.as_ref().filter(|_| idle)?;
        Some(in_hand.tool.presentation())
    }

    /// Opens the step `id` of `steps`, at `index` — `new`, inserted there,
    /// or one already there — to edit.
    fn open(&mut self, program: Program, index: usize, id: String, new: bool) -> GeopResult<()> {
        // It runs once before it is edited: an editor edits a step against
        // the part before it, as the last run left it — and with what it
        // builds, if it is previewed.
        self.give();
        self.build
            .run(&program, Some(index + usize::from(self.preview)));
        let Build { runner, part, .. } = &self.build;
        let editor = StepEditor::new(self.registry.editors(), runner, part, &program, &id, new)?;
        let view = self.draw(&self.build.view_at(&program, index), &self.build.runner)?;
        self.open = Some(Open {
            index,
            id,
            new,
            program,
            editor,
            view,
            undo: Vec::new(),
            redo: Vec::new(),
            dragging: None,
            within: None,
        });
        Ok(())
    }

    /// Opens the step `id` of the part placed as `instance` in the program,
    /// to edit where it is placed (see [`Within`]).
    /// `step` — one of its steps, by id, or a new one of an operation,
    /// after them.
    fn open_placed(&mut self, instance: String, step: PlacedStep) -> GeopResult<()> {
        let said = format!("open_placed(instance={instance}, {step:?})");
        let ctx = |e: GeopError| e.with_context(said.clone());
        self.run_to(self.marker);
        let (part, location) = (self.build.part.placed().into_iter())
            .find(|(placed, _, _)| *placed == instance)
            .map(|(_, _, placed)| (placed.part.clone(), placed.location.clone()))
            .ok_or_else(|| GeopError::new(format!("no part is placed as `{instance}`")))
            .map_err(ctx)?;
        let mut program = (*self.workspace.program(&location.file).map_err(ctx)?).clone();
        let text = self.workspace.files().read(&location.file).map_err(ctx)?;
        let current = self.build.part.current();
        let pose = (self.draw(&current, &self.build.runner)?.instances.iter())
            .find(|placed| placed.name == instance)
            .map(|placed| placed.pose)
            .ok_or_else(|| GeopError::new(format!("the part placed as `{instance}` is not drawn")))
            .map_err(ctx)?;
        let runner = self.build.runner.nested(location.clone());
        let against = Against {
            part: &part,
            outside: self.build.part.placing(&instance),
        };
        let (id, new) = match step {
            PlacedStep::Existing(id) => (id, false),
            PlacedStep::New(kind) => {
                let id = program.fresh_id(self.label_of(&kind)?);
                let editors = self.registry.editors();
                let step = editors.new_step(&runner, against.clone(), &program, &kind, &id, None);
                program.steps.push(step.map_err(ctx)?);
                (id, true)
            }
        };
        let index = program.index_of(&id).map_err(ctx)?;
        let editor = StepEditor::new(
            self.registry.editors(),
            &runner,
            against.clone(),
            &program,
            &id,
            new,
        )
        .map_err(ctx)?;
        let view = self.draw(&against.before(&program, &id)?, &runner)?;
        let view = self.beside(&instance, &pose, view);
        self.open = Some(Open {
            index,
            id,
            new,
            program,
            editor,
            view,
            undo: Vec::new(),
            redo: Vec::new(),
            dragging: None,
            within: Some(Within {
                instance,
                location,
                text,
                runner,
                part,
                pose,
            }),
        });
        Ok(())
    }

    /// Refuses while a step is edited.
    fn idle(&self) -> GeopResult<()> {
        match self.open {
            Some(_) => Err(GeopError::new("finish editing the step first")),
            None => Ok(()),
        }
    }

    /// Applies `command`, saying what it changed.
    fn apply(&mut self, command: Command) -> GeopResult<Changed> {
        Ok(match command {
            Command::Show => {
                // All of it: the viewer may have started afresh.
                self.forget_viewer();
                Changed::Run
            }
            Command::New { kind } => {
                self.idle()?;
                self.tool = None;
                let index = self.marker.unwrap_or(self.file.steps.len());
                self.run_to(Some(index));
                let label = self.label_of(&kind)?;
                let id = self.file.fresh_id(label);
                // The part as just run: the part before where the step goes.
                let Build { runner, part, .. } = &self.build;
                let step = (self.registry.editors())
                    .new_step(runner, part, &self.file, &kind, &id, None)?;
                let mut program = self.file.clone();
                program.steps.insert(index, step);
                self.open(program, index, id, true)?;
                Changed::Run
            }
            Command::Open { id, placed: None } => {
                self.idle()?;
                self.tool = None;
                let index = self.file.index_of(&id)?;
                self.open(self.file.clone(), index, id, false)?;
                Changed::Run
            }
            Command::Open {
                id,
                placed: Some(instance),
            } => {
                self.idle()?;
                self.tool = None;
                self.open_placed(instance, PlacedStep::Existing(id))?;
                Changed::Run
            }
            Command::NewIn { kind, placed } => {
                self.idle()?;
                self.tool = None;
                self.open_placed(placed, PlacedStep::New(kind))?;
                Changed::Run
            }
            Command::Event { event } => {
                // With nothing in progress in the step, Enter commits it, and
                // Escape — before it was edited, so nothing is lost — cancels
                // it: what every front end's keys do.
                if let Some(open) = &self.open
                    && let StepEditEvent::Key { key } = &event
                    && matches!(key.as_str(), "Enter" | "Escape")
                {
                    let (runner, part) = self.edited_with(open);
                    let editors = self.registry.editors();
                    let form = (open.editor).form(editors, runner, part, &open.program)?;
                    if open.editor.at_rest(&form) {
                        match key.as_str() {
                            "Enter" => return self.apply(Command::Commit),
                            _ if open.undo.is_empty() => return self.apply(Command::Cancel),
                            _ => {}
                        }
                    }
                }
                let Some(open) = &mut self.open else {
                    let Some(mut in_hand) = self.tool.take() else {
                        return Ok(Changed::Nothing);
                    };
                    let changed = in_hand.tool.handle(&mut EditorContext::new(self), &event);
                    self.tool = Some(in_hand);
                    return changed;
                };
                let before = open.edited()?;
                // A step of a placed part is edited in its own coordinates.
                let (event, runner, part) = match &open.within {
                    Some(within) => (
                        event.moved(&within.pose.inverse().motion())?,
                        &within.runner,
                        Against {
                            part: &within.part,
                            outside: self.build.part.placing(&within.instance),
                        },
                    ),
                    None => (event, &self.build.runner, Against::from(&self.build.part)),
                };
                // Picked in what the viewer was last told to draw.
                let scene = Visible {
                    view: &open.view,
                    hidden: (self.shown.as_ref()).map_or(&[], |shown| &shown.hidden),
                };
                let editing = Editing {
                    editors: self.registry.editors(),
                    runner,
                    part,
                    program: &mut open.program,
                    scene: &scene,
                };
                open.editor.handle(editing, &event)?;
                let after = open.edited()?;
                // What undo goes back to: the step before this edit — before
                // the whole drag, for one.
                let start = open.dragging.take().unwrap_or_else(|| before.clone());
                if matches!(event, StepEditEvent::Drag { done: false, .. }) {
                    open.dragging = Some(start);
                } else if start != after {
                    open.undo.push(start);
                    open.redo.clear();
                }
                // A drag changes neither, but what it pulls moves the parts —
                // of the program's own steps: a placed part's pulls nothing.
                // The pointer moving over the view pulls nothing new: what
                // the step pulled was solved for at the edit that pulled it,
                // and a hover is not solved for again (nor its form built
                // a second time).
                let moved_only = matches!(
                    event,
                    StepEditEvent::Hover { .. } | StepEditEvent::Leave | StepEditEvent::View { .. }
                );
                let pulls = open.within.is_none()
                    && !moved_only
                    && !(open
                        .editor
                        .form(
                            self.registry.editors(),
                            &self.build.runner,
                            &self.build.part,
                            &open.program,
                        )?
                        .pulls
                        .is_empty());
                match before == after && !pulls {
                    true => Changed::Nothing,
                    false => Changed::Step,
                }
            }
            Command::Commit => {
                let Some(open) = &self.open else {
                    return Err(GeopError::new("no step is being edited"));
                };
                let dialog = self.presentation(open)?.dialog;
                let missing = dialog.missing();
                if !missing.is_empty() {
                    return Err(GeopError::new(format!(
                        "pick the {} first",
                        missing.join(" and ")
                    )));
                }
                // Not previewed, it was not built: it is now, once.
                let index = open.index;
                let placed = open.within.is_some();
                if !self.preview || placed {
                    match placed {
                        true => self.run_placed(self.marker, true, true),
                        false => self.run_to(Some(index + 1)),
                    }
                }
                let Some(open) = &self.open else {
                    return Err(GeopError::new("no step is being edited"));
                };
                if let Some(error) = self.step_error(open) {
                    return Err(GeopError::new(format!("the step does not build: {error}")));
                }
                let committed = open.program.clone();
                committed.validate(&self.build.runner.registry)?;
                // A step of a placed part is committed to that part's file,
                // which the front end is given to keep.
                if let Some(within) = &open.within {
                    let path = within.location.file.clone();
                    self.written.push(File {
                        path,
                        program: committed,
                    });
                    self.open = None;
                    return Ok(Changed::Run);
                }
                // How the program is shown is the file's: it may have
                // changed while the step was edited.
                self.file = Program {
                    display: std::mem::take(&mut self.file.display),
                    ..committed
                };
                if open.new
                    && let Some(marker) = &mut self.marker
                {
                    *marker += 1;
                }
                self.open = None;
                Changed::Program
            }
            Command::Cancel => {
                // A placed part's file goes back to what it was.
                if let Some(within) = self.open.take().and_then(|open| open.within) {
                    (self.workspace).write(&within.location.file, Some(within.text));
                }
                Changed::Run
            }
            // Off, the step edited is no longer built; on, it is built again.
            Command::Preview { preview } => {
                self.preview = preview;
                Changed::Run
            }
            Command::Panel { key, payload } => {
                let registry = self.registry.clone();
                let panel = registry.get_panel(&key)?;
                panel.command(&mut EditorContext::new(self), payload)?
            }
            Command::Remove { id } => {
                self.idle()?;
                let index = self.file.index_of(&id)?;
                self.file.steps.remove(index);
                if let Some(marker) = &mut self.marker
                    && index < *marker
                {
                    *marker -= 1;
                }
                Changed::Program
            }
            Command::Move { id, index } => {
                self.idle()?;
                let from = self.file.index_of(&id)?;
                let steps = &mut self.file.steps;
                if index >= steps.len() {
                    return Err(GeopError::new(format!(
                        "cannot move step {id:?} to {index}: the program has {} steps",
                        steps.len()
                    )));
                }
                let step = steps.remove(from);
                steps.insert(index, step);
                Changed::Program
            }
            Command::Copy { id } => {
                self.exported = Some(Export {
                    name: format!("{id}.geop"),
                    media_type: "application/json".into(),
                    content: Content::Text(self.file.excerpt(&id)?.to_json()?),
                });
                Changed::Nothing
            }
            Command::Paste { text } => {
                self.idle()?;
                let pasted = Program::from_json(&text).map_err(|e| {
                    e.with_context("pasting: what is pasted is not steps copied from geop")
                })?;
                let index = self.marker.unwrap_or(self.file.steps.len());
                let editors = self.registry.editors();
                let label = |step: &ProgramStep| match editors.info(&step.operation) {
                    Some(listed) => listed.info.label.to_string(),
                    None => step.operation.clone(),
                };
                let ids = self
                    .file
                    .paste(pasted, index, &self.build.runner.registry, label)?;
                if let Some(marker) = &mut self.marker {
                    *marker += ids.len();
                }
                Changed::Program
            }
            Command::Seek { marker } => {
                self.idle()?;
                self.marker = marker.filter(|&m| m < self.file.steps.len());
                Changed::Run
            }
            Command::Load { program, path } => {
                self.idle()?;
                program.validate(&self.build.runner.registry)?;
                if let Some(path) = path
                    && Some(&path) != self.path.as_ref()
                {
                    // Another file: the one left is as it was edited, which
                    // the new one may place; its history is not the new
                    // one's, and what the new one places resolves from
                    // elsewhere.
                    let build = match self.builds.iter().position(|(kept, _)| *kept == path) {
                        Some(k) => self.builds.remove(k).1,
                        None => Build::new(&self.operations),
                    };
                    let left_build = std::mem::replace(&mut self.build, build);
                    if let Some(left) = self.path.replace(path) {
                        self.workspace.write(&left, Some(self.file.to_json()?));
                        self.builds.push((left, left_build));
                        if self.builds.len() > KEPT_BUILDS {
                            self.builds.remove(0);
                        }
                    }
                    self.undo.clear();
                    self.redo.clear();
                }
                self.file = program;
                self.marker = None;
                Changed::Program
            }
            Command::Files { files } => {
                // Only what was built from a file that changed is built
                // again; a file saved as it was changes nothing.
                let mut changed = false;
                for (path, text) in files {
                    changed |= self.workspace.write(&path, text);
                }
                if !changed {
                    return Ok(Changed::Nothing);
                }
                // The library's files changed: every step that reads them
                // reads them again, and takes what it kept where nothing
                // changed.
                // What the step being edited is built on may have changed.
                if let Some(index) = self.open.as_ref().map(|open| open.index) {
                    let placed = (self.open.as_ref()).is_some_and(|open| open.within.is_some());
                    self.run_to(if placed { self.marker } else { Some(index) });
                    if let Some(open) = &self.open {
                        let view = self.edit_view(open)?;
                        if let Some(open) = &mut self.open {
                            open.view = view;
                        }
                    }
                }
                Changed::Run
            }
            Command::LoadExample { name } => {
                self.idle()?;
                self.file = self.registry.example(&name)?;
                self.marker = None;
                Changed::Program
            }
            Command::LoadWorkspaceExample { name, folder } => {
                self.idle()?;
                let files: Vec<File> = (self.registry.workspace_example(&name)?.into_iter())
                    .map(|(path, program)| File {
                        path: match &folder {
                            Some(folder) => format!("{folder}/{path}"),
                            None => path.to_string(),
                        },
                        program,
                    })
                    .collect();
                for file in &files {
                    self.workspace
                        .write(&file.path, Some(file.program.to_json()?));
                }
                let first = files
                    .first()
                    .ok_or_else(|| GeopError::new(format!("the example {name:?} has no files")))?;
                self.file = first.program.clone();
                self.path = Some(first.path.clone());
                self.marker = None;
                self.undo.clear();
                self.redo.clear();
                self.build = Build::new(&self.operations);
                self.added = Some(files);
                Changed::Program
            }
            Command::Visibility { name, visible } => {
                self.file.display.visibility.insert(name, visible);
                Changed::Display
            }
            Command::Appearance { name, color } => {
                match color {
                    Some(color) => {
                        let hex = color.strip_prefix('#').unwrap_or_default();
                        if hex.len() != 6 || !hex.chars().all(|c| c.is_ascii_hexdigit()) {
                            return Err(GeopError::new(format!(
                                "{color:?} is no colour: a colour is `#rrggbb`"
                            )));
                        }
                        self.file.display.colors.insert(name, color)
                    }
                    None => self.file.display.colors.remove(&name),
                };
                Changed::Display
            }
            Command::Isolate { names } => {
                self.file.display.isolated = names.into_iter().collect();
                Changed::Display
            }
            #[expect(
                clippy::panic,
                reason = "the command is a panic on purpose, to check front ends recover"
            )]
            Command::Crash => panic!("the kernel was asked to crash"),
            Command::Export {
                exporter,
                format,
                options,
                date,
            } => {
                let export = self.ask(&date, |outputs, output| {
                    let exporter = outputs.get_exporter(&exporter)?;
                    exporter.export(output, format.as_deref(), &options)
                })?;
                self.exported = Some(export);
                Changed::Nothing
            }
            Command::Tool { key } => {
                self.idle()?;
                self.tool = match key {
                    Some(key) => Some(InHand {
                        tool: self.registry.tool_in_hand(&key)?,
                        key,
                    }),
                    None => None,
                };
                Changed::Run
            }
            Command::Inspect {
                inspector,
                options,
                picked,
            } => {
                self.asked = Some((inspector, options, picked));
                Changed::Nothing
            }
            Command::Undo | Command::Redo if self.open.is_some() => {
                // While a step is edited, its own edits.
                let Some(open) = self.open.as_mut() else {
                    return Ok(Changed::Nothing);
                };
                let now = open.edited()?;
                let (from, to) = match command {
                    Command::Undo => (&mut open.undo, &mut open.redo),
                    _ => (&mut open.redo, &mut open.undo),
                };
                let Some((args, state)) = from.pop() else {
                    return Ok(Changed::Nothing);
                };
                to.push(now);
                open.program.step_mut(&open.id)?.args = args;
                open.program.state = state;
                open.editor
                    .restart(self.registry.editors(), &open.program)?;
                Changed::Step
            }
            Command::Undo | Command::Redo => {
                let (from, to) = match command {
                    Command::Undo => (&mut self.undo, &mut self.redo),
                    _ => (&mut self.redo, &mut self.undo),
                };
                let Some(program) = from.pop() else {
                    return Ok(Changed::Nothing);
                };
                to.push(std::mem::replace(&mut self.file, program));
                self.marker = None;
                // Undoing is not itself an edit to undo.
                Changed::Run
            }
        })
    }

    /// Settles the program's state by the registry's settler (see
    /// [`crate::registry::Settle`]), and keeps where it put it: the
    /// program's state, which every step then sees. With no settler, the
    /// state is kept as it is given.
    ///
    /// While a step that pulls nothing ([`crate::editing::Pull`]) is edited,
    /// only the program up to it runs: the steps after it are built again
    /// once the edit is done, not on every move of the pointer — editing a
    /// sketch early in a long program would otherwise rebuild everything
    /// after it each time. What the step edited pulls, it pulls first;
    /// otherwise the settler moves the step's own state first (see
    /// [`crate::registry::Settle::settle`]). Once the whole program has
    /// run, state it no longer solves for goes.
    ///
    /// It may leave the runner having run the whole program, not what is
    /// shown: [`Self::rerun`] runs that.
    fn settle(&mut self) {
        let registry = self.registry.clone();
        let Some(settler) = registry.get_settler() else {
            return;
        };
        // Up to the step edited first: what it pulls is known from the part
        // it is applied to.
        self.run_to(match &self.open {
            Some(open) if open.within.is_none() => Some(self.runs_to(open)),
            _ => None,
        });
        // A step not previewed is not built, and pulls nothing along; nor
        // does a step of a placed part, whose state is its own.
        let pulls = match &self.open {
            Some(open) if self.preview && open.within.is_none() => (open.editor.form(
                self.registry.editors(),
                &self.build.runner,
                &self.build.part,
                &open.program,
            ))
            .map(|form| form.pulls)
            .unwrap_or_default(),
            _ => Vec::new(),
        };
        // The whole program, when the state it solves for may move: with no
        // step edited, or while the step edited pulls, which carries what
        // is after it along. Otherwise — a sketch, a feature — the steps
        // after it are built again once the edit is done.
        let whole = self.open.is_none() || !pulls.is_empty();
        if whole && self.open.is_some() {
            self.run_to(None);
        }
        let program = self.current().clone();
        // Which state the program solves for is only known once all of it
        // has run: until then, none is dropped.
        let complete =
            whole && self.build.ran == program.steps.len() && self.build.errors.is_empty();
        let own = (self.open.as_ref()).map(|open| open.id.clone());
        let mut rerun = Rerun {
            runner: &self.build.runner,
            part: &mut self.build.part,
            program: &program,
        };
        let moved = (settler.settle(&mut rerun, &pulls, own.as_deref())).unwrap_or_default();
        let mut next = program.state.clone();
        next.extend(moved);
        if complete && let Ok(solvable) = settler.solvable(&self.part()) {
            next.retain(|name, _| solvable.contains(name));
        }
        if next == program.state {
            return;
        }
        match &mut self.open {
            Some(open) if open.within.is_none() => open.program.state = next,
            _ => self.file.state = next,
        }
    }

    /// Tells every panel an edit was made (see
    /// [`crate::registry::Panel::after_edit`]): once an edit, never for
    /// every move of the pointer.
    fn after_edit(&mut self) {
        let registry = self.registry.clone();
        for panel in registry.panels() {
            panel.after_edit(&mut EditorContext::new(self));
        }
    }

    /// How far the program runs while `open` is edited: through it, if it
    /// is previewed — otherwise it is not built at all, however long it
    /// would take, until it is committed or previewed again.
    fn runs_to(&self, open: &Open) -> usize {
        open.index + usize::from(self.preview)
    }

    /// Runs what is shown: the program up to and including the step being
    /// edited, if it is previewed, or as far as it runs.
    fn rerun(&mut self) {
        self.run_to(match &self.open {
            Some(open) if open.within.is_none() => Some(self.runs_to(open)),
            _ => self.marker,
        });
        self.run += 1;
        let steps = self.current().clone();
        let before = std::mem::take(&mut self.references);
        self.references = (steps.steps.iter().take(self.build.ran).enumerate())
            .map(|(i, step)| {
                if let Some((_, picked)) = before.get(i).filter(|(read, _)| read == step) {
                    return (step.clone(), picked.clone());
                }
                // What a step picks is its arguments': not what it built,
                // which is why it is kept for as long as the step is the
                // same, and read from the form a list shows.
                let Build { runner, part, .. } = &self.build;
                let picked = (self
                    .registry
                    .editors()
                    .listed(runner, part, &steps, &step.id))
                .map(|form| form.dialog.picked().cloned().collect())
                .unwrap_or_default();
                (step.clone(), picked)
            })
            .collect();
        if let Some(open) = &self.open
            && self.picks_built(open)
            && let Ok(view) = self.edit_view(open)
            && let Some(open) = &mut self.open
        {
            open.view = view;
        }
    }

    /// What the step being edited shows.
    fn presentation(&self, open: &Open) -> GeopResult<Presentation> {
        let (runner, part) = self.edited_with(open);
        let editors = self.registry.editors();
        let scene = Visible {
            view: &open.view,
            hidden: (self.shown.as_ref()).map_or(&[], |shown| &shown.hidden),
        };
        let presentation =
            (open.editor).presentation(editors, runner, part, &open.program, &scene)?;
        match &open.within {
            Some(within) => presentation.placed(&within.instance, &within.pose.motion()),
            None => Ok(presentation),
        }
    }

    /// What the operation `kind` is called.
    fn label_of(&self, kind: &str) -> GeopResult<&'static str> {
        (self.registry.editors().info(kind))
            .map(|listed| listed.info.label)
            .ok_or_else(|| GeopError::new(format!("unknown operation `{kind}`")))
    }

    fn step_state(&self) -> Option<StepState> {
        let open = self.open.as_ref()?;
        let listed = self.registry.editors().info(&open.step().ok()?.operation)?;
        let info = listed.info;
        let presentation = self.presentation(open).ok()?;
        let missing: Vec<String> = (presentation.dialog.missing().into_iter())
            .map(str::to_string)
            .collect();
        Some(StepState {
            kind: listed.kind,
            label: info.label,
            doc: info.doc,
            id: (!open.new).then(|| open.id.clone()),
            // A step not previewed is not built, so whether it builds is not
            // known until it is committed.
            error: (missing.is_empty() && self.preview)
                .then(|| self.step_error(open))
                .flatten(),
            missing,
            presentation,
            preview: self.preview,
            can_undo: !open.undo.is_empty(),
            can_redo: !open.redo.is_empty(),
        })
    }

    /// How many steps the drawn part is built by: with the step being
    /// edited, if it is previewed, builds, and no plane is worked in —
    /// then what is drawn is drawn a second time, in the plane.
    fn shown_steps(&self, step: Option<&StepState>) -> usize {
        match (&self.open, step) {
            (Some(open), Some(step)) if open.within.is_none() => {
                let preview = self.preview
                    && step.missing.is_empty()
                    && step.error.is_none()
                    && step.presentation.focus.is_none();
                open.index + usize::from(preview)
            }
            _ => self.build.ran,
        }
    }

    /// The part `steps` steps build, as it is shown to be.
    fn shown_part(&self, steps: usize) -> Reader<'_> {
        self.build.view_at(self.current(), steps)
    }

    /// Whether the step `open` picks from what it builds (see
    /// [`crate::editable::OperationInfo::picks_built`]), now that it does.
    fn picks_built(&self, open: &Open) -> bool {
        let picks_built = (open.step().ok())
            .and_then(|step| self.registry.editors().info(&step.operation))
            .is_some_and(|listed| listed.info.picks_built);
        picks_built && self.step_error(open).is_none()
    }

    /// The part the step `open` picks from: the part before it — or, for
    /// one that picks from what it builds, that, once it builds.
    fn picks_in<'s>(&'s self, open: &'s Open) -> Reader<'s> {
        let built = self.picks_built(open);
        match &open.within {
            Some(within) if built => self.placed_against(within).after(&open.id),
            Some(within) => {
                let against = self.placed_against(within);
                (against.before(&open.program, &open.id)).unwrap_or_else(|_| against.current())
            }
            None => (self.build).view_at(&open.program, open.index + usize::from(built)),
        }
    }

    /// What the step `open` picks in, as drawn (see [`Self::picks_in`]):
    /// for a step of a placed part, with the parts placed beside it (see
    /// [`Self::beside`]).
    fn edit_view(&self, open: &Open) -> GeopResult<Rc<PartView>> {
        let view = self.draw(&self.picks_in(open), self.edited_with(open).0)?;
        Ok(match &open.within {
            Some(within) => self.beside(&within.instance, &within.pose, view),
            None => view,
        })
    }

    /// `view`, of the part placed as `instance` at `pose`, with the parts
    /// placed beside it before it — each named as that part reads it in
    /// context, `../bracket`, and where it is seen from there — so that a
    /// step of it picks what it is built against (see
    /// [`geop_core_program::placed::OUTSIDE`]). Those placed after it it
    /// may not read: they are only drawn.
    fn beside(&self, instance: &str, pose: &Pose, view: Rc<PartView>) -> Rc<PartView> {
        let placed = self.build.part.placed();
        let Some(at) = placed.iter().position(|(name, _, _)| *name == instance) else {
            return view;
        };
        let before: Vec<&str> = (placed.iter().take(at)).map(|(name, _, _)| *name).collect();
        let Ok(drawn) = self.draw(&self.build.part.current(), &self.build.runner) else {
            return view;
        };
        let back = pose.inverse();
        let origin = CoordinateSystem::world_at(Vector3::zero());
        let mut with = (*view).clone();
        for other in &drawn.instances {
            let outermost = other
                .name
                .split(INSTANCE_SEPARATOR)
                .next()
                .unwrap_or_default();
            if !before.contains(&outermost) {
                continue;
            }
            let pose = back.compose(&other.pose);
            let Ok(frame) = pose.motion().apply_frame(&origin) else {
                continue;
            };
            with.instances.push(ViewInstance {
                name: format!("{OUTSIDE}{INSTANCE_SEPARATOR}{}", other.name),
                frame,
                pose,
                state: None,
                fixed: true,
                ..other.clone()
            });
        }
        Rc::new(with)
    }

    /// What the shown steps built on — sketches, and datums as a whole —
    /// and what the drawers draw only to be picked from (the datums of the
    /// parts placed, see [`crate::registry::Drawer::hidden`]), but none
    /// that a field picking now, from what is drawn of the kinds
    /// `picking`, could pick something of.
    fn hidden(&self, picking: Option<&[String]>, steps: usize) -> Vec<String> {
        let picking = picking.unwrap_or_default();
        let picks = |kind: &str| picking.iter().any(|p| p == kind);
        let view = self.open.as_ref().map(|open| &open.view);
        // Of the part itself, or of a part placed in it, behind its path.
        let in_view = |key: &str, has: &dyn Fn(&PartView, &str) -> bool| {
            let Some(view) = view else {
                return false;
            };
            let Ok(located) = Located::of_key(key) else {
                return false;
            };
            let part = match located.path() {
                "" => Some(&**view),
                path => (view.instances.iter())
                    .find(|i| i.name == path)
                    .map(|i| &*i.view),
            };
            part.is_some_and(|part| has(part, located.name()))
        };
        // A frame is put on a datum too.
        let picks_datums = picks(drawn::DATUM) || picks(drawn::FRAME);
        let picks_sketches =
            picks(drawn::SKETCH) || picks(drawn::SKETCH_POINT) || picks(drawn::SKETCH_CURVE);
        let pickable = |name: &str| {
            let sketch = picks_sketches
                && in_view(name, &|v, name| v.sketches.iter().any(|s| s.name == name));
            let datum =
                picks_datums && in_view(name, &|v, name| v.datums.iter().any(|d| d.name == name));
            sketch || datum
        };
        // A sketch built on is hidden by its name, as is a datum built on as
        // a whole.
        let built_on = |picked: &PickedElement| {
            let own = Some(&picked.target).filter(|t| t.path().is_empty());
            if let Some(sketch) = own.and_then(|t| t.name().strip_prefix("sketch:")) {
                return Some(sketch.to_string());
            }
            (picked.kind == drawn::DATUM && !picked.name.contains(':')).then(|| picked.name.clone())
        };
        let mut hidden: Vec<String> = (self.references.iter())
            .flat_map(|(_, picked)| picked)
            .filter_map(built_on)
            .filter(|name| !pickable(name))
            .collect();
        hidden.sort();
        hidden.dedup();
        // What is drawn only to be picked from — the datums of the parts
        // placed — is shown only while a field waits to pick it: a mate's,
        // say, not the whole time a part is placed.
        hidden.extend(
            (self.registry.hidden(&self.shown_part(steps)).into_iter())
                .filter(|name| !pickable(name)),
        );
        // The placed parts not isolated, of those drawn, when some are: a
        // part is shown with the parts placed in it and those it is in.
        let display = &self.file.display;
        if !display.isolated.is_empty() {
            let related = |name: &str, other: &str| {
                let inside = |a: &str, b: &str| {
                    (a.strip_prefix(b)).is_some_and(|rest| rest.starts_with(INSTANCE_SEPARATOR))
                };
                name == other || inside(name, other) || inside(other, name)
            };
            for instance in &self.view_of(steps).instances {
                let name = &instance.name;
                if !display.isolated.iter().any(|i| related(name, i)) && !hidden.contains(name) {
                    hidden.push(name.clone());
                }
            }
        }
        // Then what the user chose — but what a click could pick now is
        // shown, to be picked.
        for (name, &visible) in &display.visibility {
            if visible || pickable(name) {
                hidden.retain(|h| h != name);
            } else if !hidden.contains(name) {
                hidden.push(name.clone());
            }
        }
        hidden
    }

    /// The part `steps` steps build, as drawn.
    fn view_of(&self, steps: usize) -> Rc<PartView> {
        (self.draw(&self.shown_part(steps), &self.build.runner)).unwrap_or_else(|_| {
            Rc::new(PartView::blank(Extent {
                center: Vector3::zero(),
                size: Scalar::from_f64(EMPTY_EXTENT),
            }))
        })
    }

    /// The part `reader` reads, as the registry draws it: as it was drawn
    /// last, if it is made of the same (see [`Self::views`]), else of what
    /// each drawer drew of what reads the same (see [`Self::drawings`]) —
    /// in this process, or, kept, in an earlier one, of the program
    /// `runner` runs, whose part it is.
    fn draw(&self, reader: &Reader, runner: &Runner) -> GeopResult<Rc<PartView>> {
        let seen = reader.seen();
        let mut views = self.views.borrow_mut();
        let view = match views.iter().position(|(drawn, _)| *drawn == seen) {
            Some(k) => views.remove(k).1,
            None => Rc::new(self.registry.draw(reader, &self.drawings, Some(runner))?),
        };
        views.push((seen, view.clone()));
        if views.len() > KEPT_VIEWS {
            views.remove(0);
        }
        Ok(view)
    }

    /// The steps of `program`, as `runner` last ran it on `part`: those
    /// before `runs_to` ran, failing as `error` says, the one `editing`
    /// edited — and, `placing`, each with the steps of the parts it placed,
    /// to edit where they are placed.
    #[allow(
        clippy::too_many_arguments,
        reason = "one listing, of the program or of a placed part"
    )]
    fn listed_steps(
        &self,
        runner: &Runner,
        part: &Part,
        program: &Program,
        runs_to: usize,
        editing: Option<&str>,
        error: impl Fn(&str) -> Option<String>,
        placing: bool,
    ) -> Vec<StepInfo> {
        let editors = self.registry.editors();
        // The parts placed, by the step that placed them.
        let mut placed: HashMap<&str, Vec<(&str, &PlacedPart)>> = HashMap::new();
        if placing {
            for (instance, by, part) in part.placed() {
                placed.entry(by).or_default().push((instance, part));
            }
        }
        (program.steps.iter().enumerate())
            .map(|(i, step)| {
                let listed = editors.info(&step.operation);
                // A summary is of the step's arguments, not of what it
                // built (see [`crate::editable::Editors::listed`]).
                let summary = (editors.listed(runner, part, program, &step.id))
                    .map(|form| form.dialog.summary())
                    .unwrap_or_default();
                let placed = (placed.get(step.id.as_str()).into_iter().flatten())
                    .filter_map(|(instance, placed)| self.placed_steps(instance, placed))
                    .collect();
                StepInfo {
                    id: step.id.clone(),
                    kind: listed.as_ref().map_or("", |l| l.kind),
                    label: listed.as_ref().map_or("", |l| l.info.label),
                    summary,
                    error: (i < runs_to).then(|| error(&step.id)).flatten(),
                    runs: i < runs_to,
                    suppressed: part.suppressed().contains(&step.id),
                    editing: editing == Some(step.id.as_str()),
                    placed,
                }
            })
            .collect()
    }

    /// The steps of the part placed as `instance`, built as `placed`: none
    /// if its program cannot be read.
    ///
    /// Listed once for every build and program — every screw of a size —
    /// and kept: a drag moves placed parts and builds none of them again,
    /// and a form of a placed standard part's sketch solves the sketch.
    fn placed_steps(&self, instance: &str, placed: &PlacedPart) -> Option<PlacedSteps> {
        let file = placed.location.file.clone();
        let open = (self.open.as_ref())
            .filter(|open| (open.within.as_ref()).is_some_and(|w| w.instance == instance));
        // The one edited is listed as edited, all of it: its file holds it
        // only as far as the step edited, while it is.
        let program = match open {
            Some(open) => Rc::new(open.program.clone()),
            None => self.workspace.program(&file).ok()?,
        };
        let key = (file.clone(), Rc::as_ptr(&placed.part));
        if open.is_none()
            && let Some(listed) = self.listed.borrow().get(&key)
            && Rc::ptr_eq(&listed.part, &placed.part)
            && (Rc::ptr_eq(&listed.program, &program) || listed.program == program)
        {
            return Some(PlacedSteps {
                instance: instance.to_string(),
                file,
                steps: listed.steps.clone(),
            });
        }
        let editing = open.map(|open| open.id.as_str());
        let failures = placed.part.failures();
        let error = |id: &str| {
            (failures.iter())
                .find(|(step, _)| step == id)
                .map(|(_, e)| e.clone())
        };
        let runner = self.build.runner.nested(placed.location.clone());
        let steps = self.listed_steps(
            &runner,
            &placed.part,
            &program,
            program.steps.len(),
            editing,
            error,
            false,
        );
        if open.is_none() {
            let listed = ListedBuild {
                part: placed.part.clone(),
                program,
                steps: steps.clone(),
            };
            self.listed.borrow_mut().insert(key, listed);
        }
        Some(PlacedSteps {
            instance: instance.to_string(),
            file,
            steps,
        })
    }

    fn program_state(&self) -> GeopResult<ProgramState> {
        let own = self.open.as_ref().filter(|open| open.within.is_none());
        let editing = own.filter(|open| !open.new).map(|open| open.id.as_str());
        let runs_to = match own {
            Some(open) => open.index,
            None => self.marker.unwrap_or(self.file.steps.len()),
        };
        let editors = self.registry.editors();
        let errors = |id: &str| self.build.errors.get(id).cloned();
        let (runner, part) = (&self.build.runner, &self.build.part);
        let steps = self.listed_steps(runner, part, &self.file, runs_to, editing, errors, true);
        // What no part placed now is built as is listed no more.
        let builds: BTreeSet<*const Part> = (part.placed().into_iter())
            .map(|(_, _, placed)| Rc::as_ptr(&placed.part))
            .collect();
        (self.listed.borrow_mut()).retain(|(_, build), _| builds.contains(build));
        let mut operations = editors.infos();
        operations.sort_by_key(|listed| (listed.info.group, listed.info.tier));
        let part = self.part();
        let mut panels = BTreeMap::new();
        let mut details = BTreeMap::new();
        for panel in self.registry.panels() {
            let (state, said) = panel.state(&self.file, &part)?;
            panels.insert(panel.key().to_string(), state);
            details.extend(said);
        }
        Ok(ProgramState {
            program: self.file.clone(),
            path: self.path.clone(),
            steps,
            marker: self.marker.unwrap_or(self.file.steps.len()),
            can_undo: match &self.open {
                Some(open) => !open.undo.is_empty(),
                None => !self.undo.is_empty(),
            },
            can_redo: match &self.open {
                Some(open) => !open.redo.is_empty(),
                None => !self.redo.is_empty(),
            },
            tool: self.tool.as_ref().map(|in_hand| in_hand.key.clone()),
            tools: self.registry.tools().into_iter().cloned().collect(),
            operations,
            exporters: self.registry.outputs().exporters(),
            inspectors: self.registry.outputs().inspectors(),
            examples: self.registry.example_names().collect(),
            workspace_examples: self.registry.workspace_example_names().collect(),
            panels,
            details,
        })
    }
}
