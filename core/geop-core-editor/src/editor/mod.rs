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
mod protocol;
#[cfg(test)]
mod tests;

pub use context::{Changed, EditorContext, Rerun};
pub use protocol::{Command, File, ProgramState, SceneState, StepInfo, StepState, Update};

use geop_core_program::{
    element::PickedElement,
    operation_registry::OperationRegistry,
    program::{Program, ProgramStep},
    reader::Reader,
    values::StateValues,
    workspace::{FileStoreMut, Files, Location},
};
use std::{
    collections::{BTreeMap, HashMap, HashSet},
    rc::Rc,
};

use crate::{
    editing::{Extent, Presentation, StepEditEvent},
    output::{Content, Export, Inspection, Output},
    registry::Registry,
    step_editor::{Editing, StepEditor},
    tools::Tool,
    view::{EMPTY_EXTENT, PartView, drawn},
};
use build::Build;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};

/// How many files' builds an editor keeps (see `Editor::builds`): each
/// holds every step's part, so a few large files are as many as fit.
const KEPT_BUILDS: usize = 6;

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
    view: PartView,
    /// The step as it was before each edit made to it since it was opened,
    /// with the program's state: what undo goes back to while it is edited.
    undo: Vec<Edited>,
    redo: Vec<Edited>,
    /// The step as it was when the drag going on started: a drag is one
    /// edit, however many events it takes.
    dragging: Option<Edited>,
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

/// The tool in hand, and the key it was taken in hand by.
struct InHand {
    key: String,
    tool: Box<dyn Tool>,
}

/// Which part is drawn, and without what — or, for a step edited on a
/// sheet of its own, no part, the viewer framing the sheet: when it is the
/// same as last time, the scene is not sent again.
#[derive(Clone, PartialEq)]
struct Shown {
    run: u64,
    steps: usize,
    hidden: Vec<String>,
    /// The sheet's centre and size.
    sheet: Option<([f64; 3], f64)>,
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
    /// Counts runs, so a scene is resent only after one.
    run: u64,
    /// What each step that ran builds on (see [`crate::editing::Dialog::picked`]),
    /// with the step it was read from: what a step picked is in its
    /// arguments, so it is read anew only when the step changed — not when
    /// a drag only moved the program's state.
    references: Vec<(ProgramStep, Vec<PickedElement>)>,
    shown: Option<Shown>,
    /// The files the last command added, for the update to say.
    added: Option<Vec<File>>,
    /// The file the last command wrote, for the update to carry.
    exported: Option<Export>,
    /// The placed parts the viewer was sent, by name: the key of the
    /// component each is drawn from, and where it is, as sent — `None`
    /// when the viewer may have none (see [`SceneState`]).
    placed: Option<HashMap<String, (String, [f64; 12])>>,
    /// The tool in hand, if one is.
    tool: Option<InHand>,
    /// The inspector the last command asked, and what it told it: to answer
    /// once it has run.
    asked: Option<(String, serde_json::Value)>,
    /// The part as last drawn: what a tool picks from.
    drawn: Option<PartView>,
    /// What the user chose to show or hide, by name, over what the editor
    /// would by itself (see [`Editor::hidden`]).
    visibility: BTreeMap<String, bool>,
}

impl Editor {
    /// An editor of what `registry` offers, editing a program of no steps,
    /// the files it places parts from read from `files`.
    pub fn new(registry: Registry, files: impl FileStoreMut + 'static) -> Self {
        let operations = Rc::new(registry.operations());
        let files: Box<dyn FileStoreMut> = Box::new(files);
        Self {
            workspace: Files::new(files, operations.clone()),
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
            exported: None,
            placed: None,
            tool: None,
            asked: None,
            drawn: None,
            visibility: BTreeMap::new(),
        }
    }

    pub fn program(&self) -> &Program {
        &self.file
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
        self.give();
        let program = match &self.open {
            Some(open) => &open.program,
            None => &self.file,
        };
        self.build.run(program, end);
    }

    /// The steps run now, and the state they run with: the step being
    /// edited's, or the program's.
    fn current(&self) -> &Program {
        match &self.open {
            Some(open) => &open.program,
            None => &self.file,
        }
    }

    /// Applies `command`, and says what to show now.
    pub fn handle(&mut self, command: Command) -> Update {
        // What undo goes back to — not taken for what is sent as often as
        // the pointer moves, and cannot change the program. Another file's
        // program is not something to undo back to.
        let before = (!matches!(command, Command::Event { .. } | Command::Preview { .. }))
            .then(|| (self.file.clone(), self.path.clone()));
        let result = self.apply(command);
        if matches!(result, Ok(Changed::Program))
            && let Some((before, path)) = before
            && path == self.path
            && self.file != before
        {
            self.undo.push(before);
            self.redo.clear();
        }
        if !matches!(result, Ok(Changed::Nothing)) {
            // Settling runs the whole program, so what is shown runs after.
            self.settle();
            self.rerun();
        }
        if matches!(result, Ok(Changed::Program | Changed::Step)) {
            self.after_edit();
        }
        let step = self.step_state();
        let steps = self.shown_steps(step.as_ref());
        let hidden = self.hidden(step.as_ref().map(|s| &s.presentation.pickable[..]), steps);
        let shown = Shown {
            run: self.run,
            steps,
            hidden,
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
        let program = match matches!(result, Ok(Changed::Program | Changed::Run)) {
            true => match self.program_state() {
                Ok(program) => Some(program),
                Err(e) => {
                    error = Some(e.to_string());
                    None
                }
            },
            false => None,
        };
        Update {
            error,
            program,
            scene,
            tool: self.tool_presentation(step.is_none()),
            step,
            files: self.added.take(),
            inspection,
            export: self.exported.take(),
        }
    }

    /// What to draw, `shown`: the part, and of the parts placed in it what
    /// the viewer was not sent yet.
    fn scene(&mut self, shown: Shown) -> SceneState {
        self.shown = Some(shown.clone());
        let part = match shown.sheet {
            Some((center, size)) => PartView::blank(Extent {
                center: Vector3::from_array(center.map(Scalar::from_f64)),
                size: Scalar::from_f64(size),
            }),
            None => self.view_of(shown.steps),
        };
        let before = self.placed.take();
        let all = before.is_none();
        let before = before.unwrap_or_default();
        let mut placed = HashMap::new();
        let mut instances = Vec::new();
        let mut components = BTreeMap::new();
        let used: HashSet<&String> = before.values().map(|(key, _)| key).collect();
        for instance in &part.instances {
            let f = &instance.frame;
            let frame = [f.origin(), f.u(), f.v(), f.w()].map(|p| p.to_array().map(|c| c.to_f64()));
            let mut numbers = [0.0; 12];
            for (number, x) in numbers.iter_mut().zip(frame.iter().flatten()) {
                *number = *x;
            }
            let drawn = (instance.component.clone(), numbers);
            if before.get(&instance.name) != Some(&drawn) {
                instances.push(instance.clone());
            }
            if !used.contains(&instance.component) && !components.contains_key(&instance.component)
            {
                components.insert(instance.component.clone(), (*instance.view).clone());
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
            components,
        }
    }

    /// `ask` answered of the outputs offered and what was built, as an
    /// exporter or inspector is given it, dated `date`.
    fn ask<T>(
        &mut self,
        date: &str,
        ask: impl FnOnce(&crate::output::Outputs, &mut Output) -> GeopResult<T>,
    ) -> GeopResult<T> {
        let editing = self.open.as_ref().map(|open| open.id.clone());
        let file = match &self.open {
            Some(open) => &open.program,
            None => &self.file,
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
        if let Some((kind, options)) = self.asked.take() {
            return self.ask("", |outputs, output| {
                let inspector = outputs.get_inspector(&kind)?;
                inspector.inspect(output, &[], &options).map(Some)
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
        // the part before it, as the last run left it.
        self.give();
        self.build.run(&program, Some(index + 1));
        let Build { runner, part, .. } = &self.build;
        let editor = StepEditor::new(self.registry.editors(), runner, part, &program, &id, new)?;
        let view = self.registry.draw(&self.build.view_at(&program, index))?;
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
                self.shown = None;
                self.placed = None;
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
            Command::Open { id } => {
                self.idle()?;
                self.tool = None;
                let index = self.file.index_of(&id)?;
                self.open(self.file.clone(), index, id, false)?;
                Changed::Run
            }
            Command::Event { event } => {
                let Some(open) = &mut self.open else {
                    let Some(mut in_hand) = self.tool.take() else {
                        return Ok(Changed::Nothing);
                    };
                    let changed = in_hand.tool.handle(&mut EditorContext::new(self), &event);
                    self.tool = Some(in_hand);
                    return changed;
                };
                let before = open.edited()?;
                let editing = Editing {
                    editors: self.registry.editors(),
                    runner: &self.build.runner,
                    part: &self.build.part,
                    program: &mut open.program,
                    scene: &open.view,
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
                // A drag changes neither, but what it pulls moves the parts.
                let pulls = !(open
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
                if let Some(error) = self.build.step_error(open.index, &open.id) {
                    return Err(GeopError::new(format!("the step does not build: {error}")));
                }
                let committed = open.program.clone();
                committed.validate(&self.build.runner.registry)?;
                self.file = committed;
                if open.new
                    && let Some(marker) = &mut self.marker
                {
                    *marker += 1;
                }
                self.open = None;
                Changed::Program
            }
            Command::Cancel => {
                self.open = None;
                Changed::Run
            }
            Command::Preview { preview } => {
                self.preview = preview;
                Changed::Nothing
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
                    self.visibility.clear();
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
                    self.run_to(Some(index));
                    let registry = self.registry.clone();
                    if let Some(open) = &mut self.open {
                        open.view = registry.draw(&self.build.view_at(&open.program, index))?;
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
                self.visibility.insert(name, visible);
                Changed::Nothing
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
            Command::Inspect { inspector, options } => {
                self.asked = Some((inspector, options));
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
        self.run_to(self.open.as_ref().map(|open| open.index + 1));
        let pulls = match &self.open {
            Some(open) => (open.editor.form(
                self.registry.editors(),
                &self.build.runner,
                &self.build.part,
                &open.program,
            ))
            .map(|form| form.pulls)
            .unwrap_or_default(),
            None => Vec::new(),
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
            Some(open) => open.program.state = next,
            None => self.file.state = next,
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

    /// Runs what is shown: the program up to and including the step being
    /// edited, or as far as it runs.
    fn rerun(&mut self) {
        self.run_to(match &self.open {
            Some(open) => Some(open.index + 1),
            None => self.marker,
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
            && let Ok(view) = self.registry.draw(&self.picks_in(open))
            && let Some(open) = &mut self.open
        {
            open.view = view;
        }
    }

    /// What the step being edited shows.
    fn presentation(&self, open: &Open) -> GeopResult<Presentation> {
        let Build { runner, part, .. } = &self.build;
        let editors = self.registry.editors();
        (open.editor).presentation(editors, runner, part, &open.program, &open.view)
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
            error: missing
                .is_empty()
                .then(|| self.build.step_error(open.index, &open.id))
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
            (Some(open), Some(step)) => {
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
        picks_built && self.build.step_error(open.index, &open.id).is_none()
    }

    /// The part the step `open` picks from: the part before it — or, for
    /// one that picks from what it builds, that, once it builds.
    fn picks_in(&self, open: &Open) -> Reader<'_> {
        let index = open.index + usize::from(self.picks_built(open));
        self.build.view_at(&open.program, index)
    }

    /// What the shown steps built on — sketches, and datums as a whole —
    /// but none that a field picking now, from what is drawn of the kinds
    /// `picking`, could pick something of.
    fn hidden(&self, picking: Option<&[String]>, steps: usize) -> Vec<String> {
        let picking = picking.unwrap_or_default();
        let picks = |kind: &str| picking.iter().any(|p| p == kind);
        let view = self.open.as_ref().map(|open| &open.view);
        let pickable = |name: &str| {
            let sketch =
                (picks(drawn::SKETCH) || picks(drawn::SKETCH_POINT) || picks(drawn::SKETCH_CURVE))
                    && view.is_some_and(|v| v.sketches.iter().any(|s| s.name == name));
            let datum = picks(drawn::DATUM)
                && view.is_some_and(|v| v.datums.iter().any(|d| d.name == name));
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
        // placed — is shown only while a step picks from what it builds:
        // a part being placed, for its mates to pick.
        let placing = (self.open.as_ref()).is_some_and(|open| self.picks_built(open));
        if !placing {
            hidden.extend(self.registry.hidden(&self.shown_part(steps)));
        }
        // Then what the user chose — but what a click could pick now is
        // shown, to be picked.
        for (name, &visible) in &self.visibility {
            if visible || pickable(name) {
                hidden.retain(|h| h != name);
            } else if !hidden.contains(name) {
                hidden.push(name.clone());
            }
        }
        hidden
    }

    /// The part `steps` steps build, as drawn.
    fn view_of(&self, steps: usize) -> PartView {
        (self.registry.draw(&self.shown_part(steps))).unwrap_or_else(|_| {
            PartView::blank(Extent {
                center: Vector3::zero(),
                size: Scalar::from_f64(EMPTY_EXTENT),
            })
        })
    }

    fn program_state(&self) -> GeopResult<ProgramState> {
        let editing = (self.open.as_ref())
            .filter(|o| !o.new)
            .map(|o| o.id.as_str());
        let runs_to = match &self.open {
            Some(open) => open.index,
            None => self.marker.unwrap_or(self.file.steps.len()),
        };
        let editors = self.registry.editors();
        let steps = (self.file.steps.iter().enumerate())
            .map(|(i, step)| {
                let listed = editors.info(&step.operation);
                // A summary is of the step's arguments, not of what it
                // built (see [`crate::editable::Editors::listed`]).
                let summary =
                    (editors.listed(&self.build.runner, &self.build.part, &self.file, &step.id))
                        .map(|form| form.dialog.summary())
                        .unwrap_or_default();
                StepInfo {
                    id: step.id.clone(),
                    kind: listed.as_ref().map_or("", |l| l.kind),
                    label: listed.as_ref().map_or("", |l| l.info.label),
                    summary,
                    error: (i < runs_to)
                        .then(|| self.build.errors.get(&step.id).cloned())
                        .flatten(),
                    runs: i < runs_to,
                    editing: editing == Some(step.id.as_str()),
                }
            })
            .collect();
        let mut operations = editors.infos();
        operations.sort_by_key(|listed| (listed.info.group, listed.info.tier));
        let part = self.part();
        let panels = (self.registry.panels())
            .map(|panel| Ok((panel.key().to_string(), panel.state(&self.file, &part)?)))
            .collect::<GeopResult<_>>()?;
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
        })
    }
}
