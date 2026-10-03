//! [`Editor`]: the whole of editing a program, as one state machine an
//! editor drives with [`Command`]s and draws from the [`Update`]s it
//! answers.
//!
//! It holds the program, its undo and redo, how far it runs, and the step
//! being edited, and it builds whatever is shown — so an editor keeps no
//! state of its own beyond the camera, renders what it is sent, and sends
//! what the user did.
//!
//! The program is the one of a file, among others it may place parts from
//! (see [`geop_ops::program::library`]): the editor is told the file's path
//! with the program ([`Command::Load`]), and is sent the other files'
//! programs whenever they change ([`Command::Files`]) — by the front end,
//! which owns the files, be they a VS Code workspace's or the browser's.
//! Only the file it edits is its own: switching to another, it keeps the
//! program it leaves as that file's. What each step shows and does is the operation's; see
//! [`geop_ops::ui::StepEditor`].

use std::collections::BTreeMap;

use geop_core_math::vector::Vector3;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_ops::{
    Context, EntityRef, Library, OperationInfo, Operations, Part, Step, StepResult,
    assembly::Drag,
    operation::Role,
    parameters::{Parameters, Resolved},
    part::{ParamValue, State},
    ui::{Dialog, PartView, Presentation, Shape, StepEditEvent, StepEditor, Style, Visual},
};
use serde::{Deserialize, Serialize};

use crate::{PartOperation, Program, ProgramRunner, Workspace, examples};

/// Something the user did.
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case", bound = "S: Scalar")]
pub enum Command<S: Scalar> {
    /// Nothing: only what to show, all of it — how an editor starts.
    Show,
    /// Start a new step of the operation `kind`, where the program runs to.
    New {
        kind: String,
    },
    /// Start editing the step `id`.
    Open {
        id: String,
    },
    /// Something the user did to the step being edited.
    Event {
        event: StepEditEvent<S>,
    },
    /// Put the step being edited into the program — refused, unless it
    /// builds.
    Commit,
    /// Stop editing the step, leaving the program as it was.
    Cancel,
    /// Whether the step being edited is shown as it builds, or the part
    /// before it.
    Preview {
        preview: bool,
    },
    Remove {
        id: String,
    },
    /// Move the step `id` to `index` among the other steps.
    Move {
        id: String,
        index: usize,
    },
    /// Run only the first `marker` steps — all of them, if `None` — and put
    /// new steps there.
    Seek {
        marker: Option<usize>,
    },
    /// Replace the program, e.g. with one read from a file — `path`, if
    /// given, which the files it places are named relative to.
    Load {
        program: Program,
        #[serde(default)]
        path: Option<String>,
    },
    /// The other program files are now these, by path: each set to its
    /// program's text, or — `None` — gone. What the program places from
    /// them is built anew.
    Files {
        files: BTreeMap<String, Option<String>>,
    },
    LoadExample {
        name: String,
    },
    /// Add the files of the example of several files `name` — put in
    /// `folder`, if given — replacing any of the same path, and edit the
    /// first of them. The update says what they are ([`Update::files`]):
    /// the front end keeps them.
    LoadWorkspaceExample {
        name: String,
        #[serde(default)]
        folder: Option<String>,
    },
    Undo,
    Redo,
    /// The program's parameters are now these (see
    /// [`geop_ops::parameters`]): what every step reading one is built
    /// with again. Allowed while a step is edited, which then sees them.
    Parameters {
        parameters: Parameters,
    },
    /// Take the drag tool in hand, or put it down: with no step being
    /// edited, dragging any placed part moves it as far as the program's
    /// mates let it.
    DragTool {
        on: bool,
    },
}

/// A step of the program, as a list of steps shows it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StepInfo {
    pub id: String,
    pub kind: &'static str,
    pub label: &'static str,
    /// Its fields in one line: `sketch=outline, distance=1.00`.
    pub summary: String,
    /// Why it failed, if it ran and did.
    pub error: Option<String>,
    /// Whether it runs: it is before where the program runs to.
    pub runs: bool,
    /// Whether it is the step being edited.
    pub editing: bool,
}

/// The program, as shown.
#[derive(Clone, Debug, Serialize)]
pub struct ProgramState {
    /// The program itself, to save.
    pub program: Program,
    /// The file it is in, once the editor has been told.
    pub path: Option<String>,
    pub steps: Vec<StepInfo>,
    /// How many steps run: new steps go there.
    pub marker: usize,
    pub can_undo: bool,
    pub can_redo: bool,
    /// Whether the drag tool is in hand.
    pub drag_tool: bool,
    /// What the program's parameters resolve to, and why those that do
    /// not resolve fail.
    pub parameters: Resolved,
    /// Every operation a step can be.
    pub operations: Vec<OperationInfo>,
    /// The names of the example programs [`Command::LoadExample`] loads.
    pub examples: Vec<&'static str>,
    /// The names of the examples of several files
    /// [`Command::LoadWorkspaceExample`] loads.
    pub workspace_examples: Vec<&'static str>,
}

/// What is drawn: a part, and which of its sketches and datums not to.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct SceneState<S: Scalar> {
    pub part: PartView<S>,
    /// The sketches and datums the steps shown have used: what was made
    /// from them shows them now. None of a kind while it is being picked.
    pub hidden: Vec<String>,
    /// The views of the components the part's instances are drawn from,
    /// by key (see [`geop_ops::Component::key`]) — those not sent before:
    /// a viewer keeps them, so moving a placed part sends where it is, not
    /// what it looks like.
    pub components: BTreeMap<String, PartView<S>>,
}

/// The step being edited, as shown.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct StepState<S: Scalar> {
    pub kind: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    /// Its id — none yet, for a new step.
    pub id: Option<String>,
    pub presentation: Presentation<S>,
    /// What it still needs picked, by its fields' labels: until it has it,
    /// it is not built, and there is no error to show.
    pub missing: Vec<String>,
    /// Why it does not build, once it has what it needs; only a step that
    /// builds can be committed.
    pub error: Option<String>,
    pub preview: bool,
}

/// A program file, as a front end keeps it.
#[derive(Clone, Debug, Serialize)]
pub struct File {
    pub path: String,
    pub program: Program,
}

/// What an editor shows after a command. What did not change since the
/// last update is left out.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Update<S: Scalar> {
    /// Why the command was refused, if it was: then it changed nothing.
    pub error: Option<String>,
    pub program: Option<ProgramState>,
    pub scene: Option<SceneState<S>>,
    /// The step being edited; `None` if there is none.
    pub step: Option<StepState<S>>,
    /// The program files the command added, the one now edited first.
    pub files: Option<Vec<File>>,
    /// What the drag tool shows, while it is in hand and no step is
    /// edited: the part it would drag lit, and whether a press grabs it.
    pub tool: Option<Presentation<S>>,
}

/// The drag tool, in hand: the placed part the pointer is over, and the one
/// being dragged.
#[derive(Default)]
struct DragTool<S: Scalar> {
    hover: Option<String>,
    grab: Option<Grabbed<S>>,
}

/// A placed part being dragged by the drag tool.
struct Grabbed<S: Scalar> {
    /// Its name, as drawn.
    name: String,
    /// The parameter its pose is.
    parameter: String,
    /// The point grabbed, in its own frame: where the pointer was, a free
    /// choice, so sharp.
    local: Vector3<S>,
    /// The plane it is dragged in: through the point grabbed, facing the
    /// eye — `(point, normal)`.
    plane: (Vector3<S>, Vector3<S>),
    /// The program when it was grabbed: what undoing the drag goes back to.
    before: Program,
}

/// The step being edited.
struct Open<S: Scalar> {
    /// Where it is, or goes.
    index: usize,
    /// Its id — for a new one, the id it gets once committed.
    id: String,
    /// Whether it is a new step, inserted at `index`, rather than the one
    /// already there.
    new: bool,
    editor: StepEditor<PartOperation, S>,
    /// What picks test against, as drawn: the part before it, or what it
    /// builds (see [`geop_ops::Operation::PICKS_BUILT`]).
    view: PartView<S>,
}

/// Which part is drawn, and without what: when it is the same as last
/// time, the scene is not sent again.
#[derive(Clone, PartialEq)]
struct Shown {
    run: u64,
    steps: usize,
    hidden: Vec<String>,
}

/// Editing a program: see the module docs.
pub struct Editor<S: Scalar> {
    program: Program,
    /// The file the program is in, once told: until then, the files it
    /// places are named relative to the workspace's top folder.
    path: Option<String>,
    /// The other program files, which it places parts from.
    workspace: Workspace<S>,
    undo: Vec<Program>,
    redo: Vec<Program>,
    marker: Option<usize>,
    open: Option<Open<S>>,
    preview: bool,
    runner: ProgramRunner<S>,
    /// Counts runs, so a scene is resent only after one.
    run: u64,
    /// What each step that ran builds on (see [`geop_ops::ui::Dialog::picked`]),
    /// with the step it was read from: what a step picked is in its
    /// arguments, so it is read anew only when the step changed — not when
    /// a drag only moved the program's state.
    references: Vec<(Step<PartOperation>, Vec<EntityRef>)>,
    shown: Option<Shown>,
    /// The names of the example programs, and of the examples made of
    /// several files — named once: building them is no work for every
    /// update.
    examples: Vec<&'static str>,
    workspace_examples: Vec<&'static str>,
    /// How the solve of the drag tool's last drag went.
    dragged: Option<geop_ops::assembly::MateReport>,
    /// The files the last command added, for the update to say.
    added: Option<Vec<File>>,
    /// The keys of the components whose views were sent: the viewer has
    /// them.
    sent: std::collections::HashSet<String>,
    /// The drag tool, while in hand.
    drag_tool: Option<DragTool<S>>,
    /// The part as last drawn: what the drag tool picks from.
    drawn: Option<PartView<S>>,
}

impl<S: Scalar> Default for Editor<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Scalar> Editor<S> {
    pub fn new() -> Self {
        Self {
            program: Program::new(),
            path: None,
            workspace: Workspace::new(BTreeMap::new()),
            undo: Vec::new(),
            redo: Vec::new(),
            marker: None,
            open: None,
            preview: true,
            runner: ProgramRunner::new(),
            run: 0,
            references: Vec::new(),
            shown: None,
            examples: examples::all().into_iter().map(|(name, _)| name).collect(),
            workspace_examples: examples::workspaces()
                .into_iter()
                .map(|(name, _)| name)
                .collect(),
            dragged: None,
            added: None,
            sent: std::collections::HashSet::new(),
            drag_tool: None,
            drawn: None,
        }
    }

    pub fn program(&self) -> &Program {
        &self.program
    }

    /// The step being edited, with its arguments as they now are.
    pub fn editing(&self) -> Option<&PartOperation> {
        self.open.as_ref().map(|open| open.editor.step())
    }

    /// [`Editor::handle`] on the wire: `command` is a [`Command`] as JSON,
    /// and the answer an [`Update`] as JSON. Shared by every front end that
    /// is not Rust (the browser's wasm module, the VS Code host process),
    /// so they all speak the same protocol. `Err` is a message that is not
    /// itself an update: the command could not be read, or the update not
    /// written.
    pub fn handle_json(&mut self, command: &str) -> Result<String, String> {
        let command: Command<S> =
            serde_json::from_str(command).map_err(|e| format!("reading command: {e}"))?;
        serde_json::to_string(&self.handle(command)).map_err(|e| format!("writing update: {e}"))
    }

    /// How the solve of the drag tool's last drag went: whether the mates
    /// hold, and how hard it was to get there.
    pub fn dragged(&self) -> Option<&geop_ops::assembly::MateReport> {
        self.dragged.as_ref()
    }

    /// Applies `command`, and says what to show now.
    pub fn handle(&mut self, command: Command<S>) -> Update<S> {
        // What undo goes back to — not taken for what is sent as often as
        // the pointer moves, and cannot change the program. Another file's
        // program is not something to undo back to.
        let before = (!matches!(command, Command::Event { .. } | Command::Preview { .. }))
            .then(|| (self.program.clone(), self.path.clone()));
        let result = self.apply(command);
        if matches!(result, Ok(Changed::Program))
            && let Some((before, path)) = before
            && path == self.path
            && self.program != before
        {
            self.undo.push(before);
            self.redo.clear();
        }
        if !matches!(result, Ok(Changed::Nothing)) {
            self.rerun();
            if self.settle() {
                self.rerun();
            }
        }
        let step = self.step_state();
        let steps = self.shown_steps(step.as_ref());
        let hidden = self.hidden(step.as_ref().map(|s| &s.presentation.pickable[..]), steps);
        let shown = Shown {
            run: self.run,
            steps,
            hidden,
        };
        let scene = (self.shown.as_ref() != Some(&shown)).then(|| {
            self.shown = Some(shown.clone());
            let part = self.view_of(shown.steps);
            let mut components = BTreeMap::new();
            for instance in &part.instances {
                if self.sent.insert(instance.component.clone())
                    && let Ok(view) = instance.component().view()
                {
                    components.insert(instance.component.clone(), view.clone());
                }
            }
            self.drawn = Some(part.clone());
            SceneState {
                part,
                hidden: shown.hidden,
                components,
            }
        });
        Update {
            error: result.as_ref().err().map(|e| e.to_string()),
            program: matches!(result, Ok(Changed::Program | Changed::Run))
                .then(|| self.program_state()),
            scene,
            tool: self.tool_presentation(step.is_none()),
            step,
            files: self.added.take(),
        }
    }

    /// What the drag tool shows, if it is in hand and no step is edited.
    fn tool_presentation(&self, idle: bool) -> Option<Presentation<S>> {
        let tool = self.drag_tool.as_ref().filter(|_| idle)?;
        let lit = tool.grab.as_ref().map(|g| &g.name).or(tool.hover.as_ref());
        Some(Presentation {
            dialog: Dialog::new(),
            visuals: lit
                .map(|name| {
                    let shape = Shape::Instance { name: name.clone() };
                    Visual::new("drag", shape, Style::Hover)
                })
                .into_iter()
                .collect(),
            highlights: Vec::new(),
            pickable: Vec::new(),
            focus: None,
            grab: lit.is_some(),
            prompt: None,
        })
    }

    /// An event for the drag tool: a hover finds the placed part a press
    /// would grab, a drag pulls the point grabbed towards the pointer — in
    /// the plane through it facing the eye — and solves the program's mates
    /// with that pull, every part that is not fixed free to give way. A
    /// drag, once released, is one edit to undo.
    fn drag(&mut self, mut tool: DragTool<S>, event: &StepEditEvent<S>) -> Changed {
        let part_at = |editor: &Self, pointer| {
            let view = editor.drawn.as_ref()?;
            let (instance, t) = view.part_to_drag(pointer)?;
            Some((
                instance.name.clone(),
                instance.parameter()?.to_string(),
                *instance.pose(),
                t,
            ))
        };
        let changed = match event {
            StepEditEvent::Hover { pointer, .. } => {
                tool.hover = part_at(self, pointer).map(|(name, ..)| name);
                Changed::Nothing
            }
            StepEditEvent::Leave => {
                tool.hover = None;
                Changed::Nothing
            }
            StepEditEvent::Drag { from, to, done, .. } => {
                if tool.grab.is_none()
                    && let Some((name, parameter, pose, t)) = part_at(self, from)
                {
                    let at = from.ray.at(t);
                    tool.grab = Some(Grabbed {
                        name,
                        parameter,
                        local: pose.inverse().apply(&at).sharpen(),
                        plane: (at, *from.ray.dir()),
                        before: self.program.clone(),
                    });
                }
                let Some(grab) = &tool.grab else {
                    self.drag_tool = Some(tool);
                    return Changed::Nothing;
                };
                let (point, normal) = &grab.plane;
                if let Some((_, target)) = to.ray.intersect_plane(point, normal) {
                    let library = library(&self.workspace, self.path.as_deref());
                    self.runner.run(&self.program, None, &library);
                    let drag = Drag {
                        parameter: grab.parameter.clone(),
                        local: grab.local,
                        target: target.sharpen(),
                    };
                    if let Ok((moved, report)) = self.runner.part().solve_mates(None, &[drag]) {
                        self.program.state.extend(moved);
                        self.dragged = Some(report);
                    }
                }
                if *done
                    && let Some(grab) = tool.grab.take()
                    && grab.before != self.program
                {
                    self.undo.push(grab.before);
                    self.redo.clear();
                }
                Changed::Run
            }
            _ => Changed::Nothing,
        };
        self.drag_tool = Some(tool);
        changed
    }

    /// Applies `command`, saying what it changed.
    fn apply(&mut self, command: Command<S>) -> GeopResult<Changed> {
        use geop_core_math::geop_error::GeopError;
        let idle = |editor: &Self| {
            if editor.open.is_some() {
                Err(GeopError::new("finish editing the step first"))
            } else {
                Ok(())
            }
        };
        Ok(match command {
            Command::Show => {
                // All of it: the viewer may have started afresh.
                self.shown = None;
                self.sent.clear();
                Changed::Run
            }
            Command::New { kind } => {
                idle(self)?;
                self.drag_tool = None;
                let index = self.marker.unwrap_or(self.program.steps.len());
                let library = library(&self.workspace, self.path.as_deref());
                self.runner.run(&self.program, Some(index), &library);
                let before = self.runner.part_at(index);
                let step = PartOperation::new_step(&kind, before)?;
                let id = self.program.fresh_id(&step);
                let context = Context::new(before, &id, &library).state(&self.program.state);
                let editor = StepEditor::new(step, context, true);
                self.open = Some(Open {
                    index,
                    view: PartView::of(before)?,
                    id,
                    new: true,
                    editor,
                });
                Changed::Run
            }
            Command::Open { id } => {
                idle(self)?;
                self.drag_tool = None;
                let index = self.program.index_of(&id)?;
                let library = library(&self.workspace, self.path.as_deref());
                self.runner.run(&self.program, Some(index), &library);
                let before = self.runner.part_at(index);
                let step = self.program.steps[index].operation.clone();
                let context = Context::new(before, &id, &library).state(&self.program.state);
                let editor = StepEditor::new(step, context, false);
                self.open = Some(Open {
                    index,
                    view: PartView::of(before)?,
                    id,
                    new: false,
                    editor,
                });
                Changed::Run
            }
            Command::Event { event } => {
                let Some(open) = &mut self.open else {
                    return Ok(match self.drag_tool.take() {
                        Some(tool) => self.drag(tool, &event),
                        None => Changed::Nothing,
                    });
                };
                let library = library(&self.workspace, self.path.as_deref());
                let context = Context::new(self.runner.part_at(open.index), &open.id, &library)
                    .built(self.runner.built(open.index));
                let before = (open.editor.step().clone(), open.editor.state().clone());
                open.editor.handle(context, &open.view, &event);
                let after = (open.editor.step(), open.editor.state());
                // A drag changes neither, but what it pulls moves the parts.
                let dragging = !open.editor.drags(context).is_empty();
                if (&before.0, &before.1) == after && !dragging {
                    Changed::Nothing
                } else {
                    Changed::Step
                }
            }
            Command::Commit => {
                let Some(open) = &self.open else {
                    return Err(GeopError::new("no step is being edited"));
                };
                let dialog = self.presentation(open).dialog;
                let missing = dialog.missing();
                if !missing.is_empty() {
                    return Err(GeopError::new(format!(
                        "pick the {} first",
                        missing.join(" and ")
                    )));
                }
                if let Some(error) = self.step_error(open) {
                    return Err(GeopError::new(format!("the step does not build: {error}")));
                }
                self.program = self.with_open(open);
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
            Command::Parameters { parameters } => {
                parameters.validate()?;
                self.program.parameters = parameters;
                Changed::Program
            }
            Command::Remove { id } => {
                idle(self)?;
                let index = self.program.index_of(&id)?;
                self.program.steps.remove(index);
                if let Some(marker) = &mut self.marker
                    && index < *marker
                {
                    *marker -= 1;
                }
                Changed::Program
            }
            Command::Move { id, index } => {
                idle(self)?;
                let from = self.program.index_of(&id)?;
                if index >= self.program.steps.len() {
                    return Err(GeopError::new(format!(
                        "cannot move step {id:?} to {index}: the program has {} steps",
                        self.program.steps.len()
                    )));
                }
                let step = self.program.steps.remove(from);
                self.program.steps.insert(index, step);
                Changed::Program
            }
            Command::Seek { marker } => {
                idle(self)?;
                self.marker = marker.filter(|&m| m < self.program.steps.len());
                Changed::Run
            }
            Command::Load { program, path } => {
                idle(self)?;
                program.validate()?;
                if let Some(path) = path
                    && Some(&path) != self.path.as_ref()
                {
                    // Another file: the one left is as it was edited, which
                    // the new one may place; its history is not the new
                    // one's, and what the new one places resolves from
                    // elsewhere.
                    if let Some(left) = self.path.replace(path) {
                        let text = self.program.to_json()?;
                        self.workspace.files_mut().insert(left, text);
                    }
                    self.undo.clear();
                    self.redo.clear();
                    self.runner.reset();
                }
                self.program = program;
                self.marker = None;
                Changed::Program
            }
            Command::Files { files } => {
                let known = self.workspace.files_mut();
                for (path, text) in files {
                    match text {
                        Some(text) => known.insert(path, text),
                        None => known.remove(&path),
                    };
                }
                self.runner.reset();
                // What the step being edited is built on may have changed.
                if let Some(open) = &self.open {
                    let library = library(&self.workspace, self.path.as_deref());
                    self.runner.run(&self.program, Some(open.index), &library);
                    let view = PartView::of(self.runner.part_at(open.index))?;
                    if let Some(open) = &mut self.open {
                        open.view = view;
                    }
                }
                Changed::Run
            }
            Command::LoadExample { name } => {
                idle(self)?;
                let (_, program) = examples::all()
                    .into_iter()
                    .find(|(n, _)| *n == name)
                    .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))?;
                self.program = program;
                self.marker = None;
                Changed::Program
            }
            Command::LoadWorkspaceExample { name, folder } => {
                idle(self)?;
                let (_, files) = examples::workspaces()
                    .into_iter()
                    .find(|(n, _)| *n == name)
                    .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))?;
                let files: Vec<File> = files
                    .into_iter()
                    .map(|(path, program)| File {
                        path: match &folder {
                            Some(folder) => format!("{folder}/{path}"),
                            None => path.to_string(),
                        },
                        program,
                    })
                    .collect();
                let texts = files
                    .iter()
                    .map(|f| Ok((f.path.clone(), f.program.to_json()?)))
                    .collect::<GeopResult<Vec<_>>>()?;
                self.workspace.files_mut().extend(texts);
                let first = files.first().expect("an example has files");
                self.program = first.program.clone();
                self.path = Some(first.path.clone());
                self.marker = None;
                self.undo.clear();
                self.redo.clear();
                self.runner.reset();
                self.added = Some(files);
                Changed::Program
            }
            Command::DragTool { on } => {
                idle(self)?;
                self.drag_tool = on.then(DragTool::default);
                Changed::Run
            }
            Command::Undo | Command::Redo => {
                idle(self)?;
                let (from, to) = match command {
                    Command::Undo => (&mut self.undo, &mut self.redo),
                    _ => (&mut self.redo, &mut self.undo),
                };
                let Some(program) = from.pop() else {
                    return Ok(Changed::Nothing);
                };
                to.push(std::mem::replace(&mut self.program, program));
                self.marker = None;
                // Undoing is not itself an edit to undo.
                Changed::Run
            }
        })
    }

    /// The program with the step being edited in it, and the state as
    /// its edits leave them.
    fn with_open(&self, open: &Open<S>) -> Program {
        let mut program = self.program.clone();
        program.state = open.editor.state().clone();
        let step = Step {
            id: open.id.clone(),
            operation: open.editor.step().clone(),
        };
        if open.new {
            program.steps.insert(open.index, step);
        } else {
            program.steps[open.index] = step;
        }
        program
    }

    /// Solves the program's mates — the whole program's, the steps after the
    /// one being edited included — from where its state puts its parts,
    /// and keeps where they put them: the program's state, which every step
    /// then sees (see [`geop_ops::part::State`]).
    ///
    /// A drag the step being edited asks for pulls first, every part that is
    /// not fixed free to give way — a linkage follows the link dragged.
    /// Otherwise the parts move only if a mate does not hold: the step's own
    /// part first, the others held — mating a new part moves it, not the
    /// part it is mated to — and every part that is not fixed only where
    /// that is not enough. State no step declares any more go, and
    /// those declared but not given are kept at what the steps took for
    /// them. Says whether the state changed.
    fn settle(&mut self) -> bool {
        let library = library(&self.workspace, self.path.as_deref());
        let program = match &self.open {
            Some(open) => self.with_open(open),
            None => self.program.clone(),
        };
        self.runner.run(&program, None, &library);
        let complete = self.runner.results().len() == program.steps.len()
            && self.runner.results().iter().all(|r| r.error.is_none());
        let part = self.runner.part();
        let (drags, own) = match &self.open {
            Some(open) => {
                let context = Context::new(self.runner.part_at(open.index), &open.id, &library)
                    .built(self.runner.built(open.index));
                let declared = self.runner.part_at(open.index).state();
                let own: Vec<String> = self
                    .runner
                    .part_at(open.index + 1)
                    .state()
                    .keys()
                    .filter(|name| !declared.contains_key(*name))
                    .cloned()
                    .collect();
                (open.editor.drags(context), own)
            }
            None => (Vec::new(), Vec::new()),
        };
        let holds = || part.check_mates().is_ok_and(|report| report.converged);
        let solved = if !drags.is_empty() {
            part.solve_mates(None, &drags).ok().map(|(moved, _)| moved)
        } else if holds() {
            None
        } else {
            let own_first = own
                .first()
                .and_then(|name| part.solve_mates(Some(name), &[]).ok())
                .filter(|(_, report)| report.converged);
            own_first
                .or_else(|| part.solve_mates(None, &[]).ok())
                .map(|(moved, _)| moved)
        };
        let mut state = program.state.clone();
        state.extend(solved.into_iter().flatten());
        if complete {
            // Every pose a step declares, and no other: the file says where
            // every placed part is. The numbers its steps read are its
            // parameters', defined apart from its state.
            let declared: State = part
                .state()
                .iter()
                .filter(|(_, value)| matches!(value, ParamValue::Pose(_)))
                .map(|(name, value)| (name.clone(), value.clone()))
                .collect();
            state.retain(|name, _| declared.contains_key(name));
            for (name, value) in declared {
                state.entry(name).or_insert(value);
            }
        }
        if state == program.state {
            return false;
        }
        match &mut self.open {
            Some(open) => open.editor.set_state(state),
            None => self.program.state = state,
        }
        true
    }

    /// Runs what is shown: the program up to and including the step being
    /// edited, or as far as it runs.
    fn rerun(&mut self) {
        let library = library(&self.workspace, self.path.as_deref());
        match &self.open {
            Some(open) => {
                let program = self.with_open(open);
                self.runner.run(&program, Some(open.index + 1), &library);
            }
            None => self.runner.run(&self.program, self.marker, &library),
        }
        self.run += 1;
        let ran = self.runner.results().len();
        let steps = match &self.open {
            Some(open) => self.with_open(open).steps,
            None => self.program.steps.clone(),
        };
        let before = std::mem::take(&mut self.references);
        self.references = steps[..ran]
            .iter()
            .enumerate()
            .map(|(i, step)| {
                if let Some((_, picked)) = before.get(i).filter(|(read, _)| read == step) {
                    return (step.clone(), picked.clone());
                }
                let session = step.operation.new_session();
                let context = Context::new(self.runner.part_at(i), &step.id, &library)
                    .built(self.runner.built(i));
                let form = step.operation.form(context, &*session, &[]);
                (step.clone(), form.dialog.picked().cloned().collect())
            })
            .collect();
        if let Some(open) = &self.open
            && open.editor.step().picks_built()
        {
            let view = PartView::of(self.picks_in(open)).ok();
            if let (Some(open), Some(view)) = (&mut self.open, view) {
                open.view = view;
            }
        }
    }

    /// The part the step being edited picks from: the part before it — or,
    /// for one that picks from what it builds, that, once it builds.
    fn picks_in(&self, open: &Open<S>) -> &Part<S> {
        let builds = open.editor.step().picks_built() && self.step_error(open).is_none();
        self.runner.part_at(open.index + usize::from(builds))
    }

    /// What the step being edited shows.
    fn presentation(&self, open: &Open<S>) -> Presentation<S> {
        let library = library(&self.workspace, self.path.as_deref());
        let context = Context::new(self.runner.part_at(open.index), &open.id, &library)
            .built(self.runner.built(open.index));
        open.editor
            .presentation(context, self.picks_in(open), &open.view)
    }

    /// Why the step being edited does not build, if it does not.
    fn step_error(&self, open: &Open<S>) -> Option<String> {
        match self.runner.results().get(open.index) {
            Some(StepResult { error, .. }) => error.clone(),
            None => Some(
                self.runner
                    .results()
                    .iter()
                    .find_map(|r| r.error.as_ref())
                    .map_or("it did not run".into(), |e| {
                        format!("an earlier step failed: {e}")
                    }),
            ),
        }
    }

    fn step_state(&self) -> Option<StepState<S>> {
        let open = self.open.as_ref()?;
        let step = open.editor.step();
        let info = PartOperation::infos()
            .into_iter()
            .find(|i| i.kind == step.kind())?;
        let presentation = self.presentation(open);
        let missing: Vec<String> = presentation
            .dialog
            .missing()
            .into_iter()
            .map(str::to_string)
            .collect();
        Some(StepState {
            kind: info.kind,
            label: info.label,
            doc: info.doc,
            id: (!open.new).then(|| open.id.clone()),
            error: missing.is_empty().then(|| self.step_error(open)).flatten(),
            missing,
            presentation,
            preview: self.preview,
        })
    }

    /// How many steps the drawn part is built by: with the step being
    /// edited, if it is previewed, builds, and no plane is worked in —
    /// then what is drawn is drawn a second time, in the plane.
    fn shown_steps(&self, step: Option<&StepState<S>>) -> usize {
        match (&self.open, step) {
            (Some(open), Some(step)) => {
                let preview = self.preview
                    && step.missing.is_empty()
                    && step.error.is_none()
                    && step.presentation.focus.is_none();
                open.index + usize::from(preview)
            }
            _ => self.runner.results().len(),
        }
    }

    /// What the shown steps built on — sketches, and datums as a whole —
    /// but none that something of could fill a role `picking` looks for.
    fn hidden(&self, picking: Option<&[Role]>, steps: usize) -> Vec<String> {
        let picking = picking.unwrap_or_default();
        let pickable = |r: &EntityRef| {
            self.open
                .as_ref()
                .is_some_and(|open| open.view.can_fill(r, picking))
        };
        let mut hidden: Vec<String> = self
            .references
            .iter()
            .flat_map(|(_, picked)| picked)
            .filter(|r| !pickable(r))
            .filter_map(|r| match r {
                EntityRef::Sketch { name }
                | EntityRef::Datum {
                    name,
                    component: None,
                } => Some(name.clone()),
                _ => None,
            })
            .collect();
        // The datums of the parts placed are theirs, not the part's: shown
        // only while a part is being placed, for its mates to pick.
        let placing = self
            .open
            .as_ref()
            .is_some_and(|open| matches!(open.editor.step(), PartOperation::AddPart(_)));
        if !placing {
            fn placed<S: Scalar>(part: &Part<S>, prefix: &str, out: &mut Vec<String>) {
                for (id, instance) in part.instances() {
                    let name = format!(
                        "{prefix}{}{}",
                        part.name_of(id).unwrap_or_default(),
                        geop_ops::operation::INSTANCE_SEPARATOR
                    );
                    let inner = instance.part();
                    out.extend(
                        inner
                            .datums()
                            .filter_map(|(datum, _)| inner.name_of(datum))
                            .map(|datum| format!("{name}{datum}")),
                    );
                    placed(inner, &name, out);
                }
            }
            placed(self.runner.part_at(steps), "", &mut hidden);
        }
        hidden
    }

    /// The part `steps` steps build, as drawn.
    fn view_of(&self, steps: usize) -> PartView<S> {
        let part = self.runner.part_at(steps);
        PartView::of(part)
            .unwrap_or_else(|_| PartView::of(&Part::new()).expect("an empty part can be drawn"))
    }

    fn program_state(&self) -> ProgramState {
        let ran = self.runner.results();
        let library = library(&self.workspace, self.path.as_deref());
        let editing = self.open.as_ref().filter(|o| !o.new).map(|o| o.id.as_str());
        let runs_to = match &self.open {
            Some(open) => open.index,
            None => self.marker.unwrap_or(self.program.steps.len()),
        };
        let steps = self
            .program
            .steps
            .iter()
            .enumerate()
            .map(|(i, step)| {
                let session = step.operation.new_session();
                let context = Context::new(self.runner.part_at(i), &step.id, &library)
                    .built(self.runner.built(i));
                let form = step.operation.form(context, &*session, &[]);
                StepInfo {
                    id: step.id.clone(),
                    kind: step.operation.kind(),
                    label: step.operation.label(),
                    summary: form.dialog.summary(),
                    error: ran
                        .get(i)
                        .filter(|_| i < runs_to)
                        .and_then(|r| r.error.clone()),
                    runs: i < runs_to,
                    editing: editing == Some(step.id.as_str()),
                }
            })
            .collect();
        ProgramState {
            program: self.program.clone(),
            path: self.path.clone(),
            steps,
            marker: self.marker.unwrap_or(self.program.steps.len()),
            can_undo: !self.undo.is_empty(),
            can_redo: !self.redo.is_empty(),
            drag_tool: self.drag_tool.is_some(),
            parameters: self.program.parameters.resolve(&self.program.state),
            operations: PartOperation::infos(),
            examples: self.examples.clone(),
            workspace_examples: self.workspace_examples.clone(),
        }
    }
}

/// The library the program in the file `path` is built with — with no file
/// yet, one in the workspace's top folder.
fn library<'w, S: Scalar>(workspace: &'w Workspace<S>, path: Option<&str>) -> impl Library<S> + 'w {
    workspace.scope(path.unwrap_or_default())
}

/// What a command changed.
enum Changed {
    Nothing,
    /// The arguments of the step being edited.
    Step,
    /// What runs: which steps, or up to where.
    Run,
    /// The program.
    Program,
}
