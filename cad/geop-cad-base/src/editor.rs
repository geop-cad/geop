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

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use geop_core_math::vector::Vector3;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_ops::{
    Context, Design, EntityRef, Library, OperationInfo, Operations, Part, Step, StepResult,
    assembly::{Drag, JointInfo, MateFreedom},
    operation::Role,
    parameters::{Parameters, Resolved},
    part::{ParamValue, State},
    program::library::resolve,
    ui::{
        Dialog, PartView, Presentation, Shape, StepEditEvent, StepEditor, Style, ViewInstance,
        Visual,
    },
};
use geop_ops_rasterize::stl;
use serde::{Deserialize, Serialize};

use crate::{
    PartOperation, Program, ProgramRunner, Workspace, examples,
    inspect::{self, Inspection, MeasureTool, Query},
    stdlib::WithStandardParts,
};

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
    /// Go back one edit — while a step is edited, one edit of the step —
    /// or forward again.
    Undo,
    Redo,
    /// The program's parameters are now these (see
    /// [`geop_ops::parameters`]): what every step reading one is built
    /// with again. Allowed while a step is edited, which then sees them.
    Parameters {
        parameters: Parameters,
    },
    /// Name the parameter `from` `to`, and every formula reading it read
    /// `to` (see [`geop_ops::Program::rename_parameter`]): the other
    /// parameters', the steps'. Refused while a step is edited.
    RenameParameter {
        from: String,
        to: String,
    },
    /// Show the datum, sketch, solid or placed part `name` of the part
    /// drawn, or hide it — whatever the editor would do by itself — for as
    /// long as the same file is edited.
    Visibility {
        name: String,
        visible: bool,
    },
    /// Take the drag tool in hand, or put it down: with no step being
    /// edited, dragging any placed part moves it as far as the program's
    /// mates let it.
    DragTool {
        on: bool,
    },
    /// Set the joint coordinate `parameter` — an angle in degrees, or a
    /// distance — to `value`: the parts move to it, everything else that
    /// is free giving way (see [`ProgramState::joints`]).
    Joint {
        parameter: String,
        value: f64,
    },
    /// Take the measure tool in hand, or put it down: with no step being
    /// edited, a click picks a vertex, an edge, a face or a datum to
    /// measure — up to two (see [`MeasureTool::handle`]) — and every update
    /// says what they measure ([`Update::inspection`]). Measuring is no
    /// edit: it changes nothing, and adds no step.
    MeasureTool {
        on: bool,
    },
    /// Ask a question of the part as drawn — its mass properties, which of
    /// its solids interfere — answered once, in [`Update::inspection`].
    /// Changes nothing.
    Inspect {
        query: Query,
    },
    /// Write a drawing of the part (see [`geop_ops_drawing`]) as an SVG or
    /// DXF file, dated `date`: the drawing step `id` — if not given, the
    /// one being edited, else the program's last; the default drawing of
    /// the whole part if it has none. The file comes back as
    /// [`Update::export`].
    ExportDrawing {
        #[serde(default)]
        id: Option<String>,
        format: geop_ops_drawing::Format,
        #[serde(default)]
        date: String,
    },
    /// Write the assembly — as far as the program runs, where its parts
    /// are — as a URDF robot (see [`geop_ops_urdf`]): a ZIP archive of
    /// `robot.urdf` and its meshes, named after the program's file, as
    /// [`Update::export`].
    ExportUrdf,
    /// Write the part shown — up to where the program runs, the parts it
    /// places included — as a STEP file: the update's [`Update::export`].
    ExportStep,
    /// Write every solid of the part shown — up to where the program runs,
    /// the parts it places included — as a binary STL mesh, as finely as
    /// the editor draws it: the update's [`Update::export`].
    ExportStl,
    /// Write the bill of materials of the part shown (see
    /// [`geop_ops_bom`]), laid out as `structure` says, as a CSV file named
    /// after the program's file: the update's [`Update::export`].
    ExportBom {
        #[serde(default)]
        structure: geop_ops_bom::Structure,
    },
    /// Write the flat pattern of a sheet-metal body of the part shown — the
    /// body `solid`, else the newest — as a DXF file for laser cutting (see
    /// [`geop_ops_sheetmetal::flat_pattern_dxf`]): its outline and holes on
    /// the `CUT` layer, its bend lines on the `BEND` layer. Named after the
    /// program's file, it is the update's [`Update::export`].
    ExportFlatPattern {
        #[serde(default)]
        solid: Option<String>,
    },
    /// Panic, on purpose: how the front ends' recovery from a kernel that
    /// crashed is checked (`web/e2e/`). A panic anywhere else is a bug;
    /// this one stands in for it.
    Crash,
}

/// How finely the curved faces of an exported mesh — an STL file, a robot's
/// links — are meshed: as finely as the editor draws them.
const MESH_QUALITY: usize = 24;

/// A file the editor wrote for the front end to save.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Export {
    /// What to call it: after the program file, or `drawing`.
    pub name: String,
    #[serde(flatten)]
    pub content: Content,
}

impl Export {
    /// Its text, if it is a text file.
    pub fn text(&self) -> Option<&str> {
        match &self.content {
            Content::Text(text) => Some(text),
            Content::Bytes(_) => None,
        }
    }
}

/// What an exported file holds: text — sent as `text` — or bytes, sent as
/// `bytes`, in base64.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Content {
    Text(String),
    #[serde(serialize_with = "base64")]
    Bytes(Vec<u8>),
}

/// `bytes` as base64 text (RFC 4648, padded).
fn base64<Ser: serde::Serializer>(bytes: &[u8], serializer: Ser) -> Result<Ser::Ok, Ser::Error> {
    const ALPHABET: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut text = String::with_capacity(bytes.len().div_ceil(3) * 4);
    for chunk in bytes.chunks(3) {
        let b = [0, 1, 2].map(|k| chunk.get(k).copied().unwrap_or(0) as u32);
        let n = (b[0] << 16) | (b[1] << 8) | b[2];
        for k in 0..4 {
            text.push(if k <= chunk.len() {
                ALPHABET[(n >> (18 - 6 * k) & 63) as usize] as char
            } else {
                '='
            });
        }
    }
    serializer.serialize_str(&text)
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
    /// Whether the measure tool is in hand.
    pub measure_tool: bool,
    /// The materials a part's can be picked from (see
    /// [`geop_ops::parameters::Material`]).
    pub materials: Vec<geop_ops::parameters::Material>,
    /// What the program's parameters resolve to, and why those that do
    /// not resolve fail.
    pub parameters: Resolved,
    /// What reads each parameter, by name: the steps and the other
    /// parameters whose formulas do, which fail if it goes (see
    /// [`geop_ops::Program::parameter_uses`]).
    pub parameter_uses: BTreeMap<String, Vec<String>>,
    /// Every operation a step can be, as the toolbar offers them: group by
    /// group, in [`geop_ops::OperationGroup`]'s order, and within a group
    /// by [`geop_ops::OperationTier`], the most used first, then in
    /// [`PartOperation`]'s order.
    pub operations: Vec<OperationInfo>,
    /// The names of the example programs [`Command::LoadExample`] loads.
    pub examples: Vec<&'static str>,
    /// The names of the examples of several files
    /// [`Command::LoadWorkspaceExample`] loads.
    pub workspace_examples: Vec<&'static str>,
    /// The joints of the part shown, with where their coordinates are:
    /// what [`Command::Joint`] sets.
    pub joints: Vec<JointInfo>,
    /// How free each of its placed parts is, and which of its mates
    /// conflict — if it has placed parts.
    pub freedom: Option<MateFreedom>,
}

/// What is drawn: a part, and which of its sketches and datums not to.
///
/// The parts placed in it — however many, however deep — are sent as what
/// changed since the last scene: a viewer keeps them by name, and the
/// views of their components by key, each sent once. So moving one placed
/// part sends where that one is, and placing a screw a hundred times sends
/// what a screw looks like once.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct SceneState<S: Scalar> {
    /// The part, without the parts placed in it: those are `instances`.
    pub part: PartView<S>,
    /// The sketches and datums the steps shown have used: what was made
    /// from them shows them now. None of a kind while it is being picked.
    pub hidden: Vec<String>,
    /// The parts placed in the part drawn, however deep, that are new or
    /// moved or drawn from another component since the last scene — every
    /// one, if `all`.
    pub instances: Vec<ViewInstance<S>>,
    /// Whether `instances` are all there are: a viewer forgets any others.
    pub all: bool,
    /// The names of the placed parts drawn last time and gone now.
    pub removed: Vec<String>,
    /// The views of the components the placed parts are drawn from, by key
    /// (see [`geop_ops::Component::key`]), that the last scene's did not
    /// use. A viewer keeps those the placed parts drawn use, and forgets
    /// the others: a component no placed part uses any more is sent again
    /// once one does.
    pub components: BTreeMap<String, PartView<S>>,
    /// What the part has beyond its faces, to list and show or hide.
    pub structure: Vec<StructureItem>,
}

/// What kind of thing a [`StructureItem`] is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum StructureKind {
    Datum,
    Sketch,
    Solid,
    /// A face standing on its own, of no solid.
    Face,
    Part,
    Mate,
}

/// A datum, sketch, solid, face standing on its own, placed part or mate of
/// the part drawn, by name:
/// whether it is shown, if it is something drawn at all — what
/// [`Command::Visibility`] switches.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StructureItem {
    pub kind: StructureKind,
    pub name: String,
    pub visible: Option<bool>,
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
    /// Whether one of its edits can be undone, or redone.
    pub can_undo: bool,
    pub can_redo: bool,
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
    /// The file [`Command::ExportDrawing`], [`Command::ExportUrdf`],
    /// [`Command::ExportStep`], [`Command::ExportStl`] or
    /// [`Command::ExportBom`] wrote.
    pub export: Option<Export>,
    /// What the drag tool or the measure tool shows, while it is in hand
    /// and no step is edited: the part it would drag lit, and whether a
    /// press grabs it; what is picked to measure, and the least distance.
    pub tool: Option<Presentation<S>>,
    /// What the measure tool's picks measure, while it is in hand — or the
    /// answer to [`Command::Inspect`].
    pub inspection: Option<Inspection<S>>,
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
    /// The step as it was before each edit made to it since it was opened,
    /// with the program's state: what undo goes back to while it is edited.
    undo: Vec<Edited>,
    redo: Vec<Edited>,
    /// The step as it was when the drag going on started: a drag is one
    /// edit, however many events it takes.
    dragging: Option<Edited>,
}

/// A step being edited, as one edit left it, with the program's state.
type Edited = (PartOperation, geop_ops::part::State);

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
    pub(crate) runner: ProgramRunner<S>,
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
    /// The file the last command wrote, for the update to carry.
    exported: Option<Export>,
    /// The placed parts the viewer was sent, by name: the key of the
    /// component each is drawn from, and where it is, as sent — `None`
    /// when the viewer may have none (see [`SceneState`]).
    placed: Option<HashMap<String, (String, [f64; 12])>>,
    /// The drag tool, while in hand.
    drag_tool: Option<DragTool<S>>,
    /// The measure tool, while in hand.
    measure_tool: Option<MeasureTool<S>>,
    /// The question the last command asked, to answer once it has run.
    query: Option<Query>,
    /// The part as last drawn: what the drag tool picks from.
    drawn: Option<PartView<S>>,
    /// What the user chose to show or hide, by name, over what the editor
    /// would by itself (see [`Editor::hidden`]).
    visibility: BTreeMap<String, bool>,
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
            workspace: Workspace::new(WithStandardParts(BTreeMap::new())),
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
            exported: None,
            placed: None,
            drag_tool: None,
            measure_tool: None,
            query: None,
            drawn: None,
            visibility: BTreeMap::new(),
        }
    }

    pub fn program(&self) -> &Program {
        &self.program
    }

    /// The part the program builds, as far as it runs now.
    pub fn part(&self) -> &Part<S> {
        self.runner.part()
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
            // Settling runs the whole program, so what is shown runs after.
            self.settle();
            self.rerun();
            self.give_parts_list();
        }
        let step = self.step_state();
        let steps = self.shown_steps(step.as_ref());
        let hidden = self.hidden(step.as_ref().map(|s| &s.presentation.pickable[..]), steps);
        let shown = Shown {
            run: self.run,
            steps,
            hidden,
            sheet: step
                .as_ref()
                .and_then(|s| s.presentation.sheet)
                .map(|e| (e.center.to_array().map(|c| c.to_f64()), e.size.to_f64())),
        };
        let scene = (self.shown.as_ref() != Some(&shown)).then(|| {
            self.shown = Some(shown.clone());
            let part = match shown.sheet {
                Some((center, size)) => PartView::blank(geop_ops::ui::Extent {
                    center: Vector3::from_array(center.map(S::from_f64)),
                    size: S::from_f64(size),
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
                let frame =
                    [f.origin(), f.u(), f.v(), f.w()].map(|p| p.to_array().map(|c| c.to_f64()));
                let drawn = (
                    instance.component.clone(),
                    frame.concat().try_into().expect("12 numbers"),
                );
                if before.get(&instance.name) != Some(&drawn) {
                    instances.push(instance.clone());
                }
                if !used.contains(&instance.component)
                    && !components.contains_key(&instance.component)
                    && let Ok(view) = instance.component().view()
                {
                    components.insert(instance.component.clone(), view.clone());
                }
                placed.insert(instance.name.clone(), drawn);
            }
            let mut removed: Vec<String> = before
                .keys()
                .filter(|name| !placed.contains_key(*name))
                .cloned()
                .collect();
            removed.sort();
            self.placed = Some(placed);
            self.drawn = Some(part.clone());
            SceneState {
                part,
                structure: self.structure(shown.steps, &shown.hidden),
                hidden: shown.hidden,
                instances,
                all,
                removed,
                components,
            }
        });
        let mut error = result.as_ref().err().map(|e| e.to_string());
        let shown_part = self.runner.part_at(steps);
        let inspection = match self.query.take() {
            Some(query) => inspect::answer(query, shown_part, &self.file())
                .map_err(|e| error = Some(e.to_string()))
                .ok(),
            None => self
                .measure_tool
                .as_mut()
                .filter(|_| step.is_none())
                .map(|tool| {
                    if !matches!(result, Ok(Changed::Nothing)) {
                        tool.remeasure(shown_part);
                    }
                    Inspection::Measure(tool.measurement().clone())
                }),
        };
        Update {
            error,
            program: matches!(result, Ok(Changed::Program | Changed::Run))
                .then(|| self.program_state()),
            scene,
            tool: self.tool_presentation(step.is_none()),
            step,
            files: self.added.take(),
            inspection,
            export: self.exported.take(),
        }
    }

    /// What the drag tool or the measure tool shows, if one is in hand and
    /// no step is edited.
    fn tool_presentation(&self, idle: bool) -> Option<Presentation<S>> {
        if let Some(measure) = self.measure_tool.as_ref().filter(|_| idle) {
            return Some(measure.presentation());
        }
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
            sheet: None,
            grab: lit.is_some(),
            prompt: None,
            gizmo: None,
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
                    if let Ok((moved, report)) = self.runner.part().solve_mates(None, &[], &[drag])
                    {
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
                self.placed = None;
                Changed::Run
            }
            Command::New { kind } => {
                idle(self)?;
                self.drag_tool = None;
                self.measure_tool = None;
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
                    undo: Vec::new(),
                    redo: Vec::new(),
                    dragging: None,
                });
                Changed::Run
            }
            Command::Open { id } => {
                idle(self)?;
                self.drag_tool = None;
                self.measure_tool = None;
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
                    undo: Vec::new(),
                    redo: Vec::new(),
                    dragging: None,
                });
                Changed::Run
            }
            Command::Event { event } => {
                let Some(open) = &mut self.open else {
                    if let Some(tool) = self.drag_tool.take() {
                        return Ok(self.drag(tool, &event));
                    }
                    if let (Some(tool), Some(view)) = (&mut self.measure_tool, &self.drawn) {
                        tool.handle(view, self.runner.part(), &event);
                    }
                    return Ok(Changed::Nothing);
                };
                let library = library(&self.workspace, self.path.as_deref());
                let context = Context::new(self.runner.part_at(open.index), &open.id, &library)
                    .built(self.runner.built(open.index));
                let before = (open.editor.step().clone(), open.editor.state().clone());
                open.editor.handle(context, &open.view, &event);
                let after = (open.editor.step(), open.editor.state());
                // What undo goes back to: the step before this edit — before
                // the whole drag, for one.
                let start = open.dragging.take().unwrap_or_else(|| before.clone());
                if matches!(event, StepEditEvent::Drag { done: false, .. }) {
                    open.dragging = Some(start);
                } else if (&start.0, &start.1) != after {
                    open.undo.push(start);
                    open.redo.clear();
                }
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
            Command::Joint { parameter, value } => {
                idle(self)?;
                let library = library(&self.workspace, self.path.as_deref());
                let mut state = self.program.state.clone();
                state.insert(
                    parameter.clone(),
                    ParamValue::Number(Design::from_f64(value)),
                );
                let program = Program {
                    state,
                    ..self.program.clone()
                };
                self.runner.run(&program, None, &library);
                let part = self.runner.part();
                if !part.solved_parameters().contains(&parameter) {
                    return Err(GeopError::new(format!(
                        "{parameter:?} is no joint coordinate of this program"
                    )));
                }
                let (moved, _) = part.solve_joints(std::slice::from_ref(&parameter))?;
                self.program.state = program.state;
                self.program.state.extend(moved);
                Changed::Program
            }
            Command::Parameters { parameters } => {
                parameters.validate()?;
                self.program.parameters = parameters;
                Changed::Program
            }
            Command::RenameParameter { from, to } => {
                idle(self)?;
                self.program.rename_parameter(&from, &to)?;
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
                        self.workspace.write(&left, Some(text));
                    }
                    self.undo.clear();
                    self.redo.clear();
                    self.visibility.clear();
                    self.runner.reset();
                }
                self.program = program;
                self.marker = None;
                Changed::Program
            }
            Command::Files { files } => {
                // Only what was built from a file that changed is built
                // again; a file saved as it was changes nothing.
                let changed: BTreeSet<String> = files
                    .into_iter()
                    .filter_map(|(path, text)| {
                        self.workspace
                            .write(&path, text)
                            .then(|| resolve("", &path))
                    })
                    .collect();
                if changed.is_empty() {
                    return Ok(Changed::Nothing);
                }
                self.runner.forget(&changed);
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
                self.program = program();
                self.marker = None;
                Changed::Program
            }
            Command::LoadWorkspaceExample { name, folder } => {
                idle(self)?;
                let (_, files) = examples::workspaces()
                    .into_iter()
                    .find(|(n, _)| *n == name)
                    .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))?;
                let files: Vec<File> = files()
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
                for (path, text) in texts {
                    self.workspace.write(&path, Some(text));
                }
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
            Command::Visibility { name, visible } => {
                self.visibility.insert(name, visible);
                Changed::Nothing
            }
            Command::ExportStep => {
                let stem = self.file_stem().unwrap_or_else(|| "part".to_string());
                let text = geop_ops_step::write_step(self.runner.part(), &stem)?;
                self.exported = Some(Export {
                    name: format!("{stem}.step"),
                    content: Content::Text(text),
                });
                Changed::Nothing
            }
            Command::Crash => panic!("the kernel was asked to crash"),
            Command::ExportFlatPattern { solid } => {
                let (_, dxf) =
                    geop_ops_sheetmetal::flat_pattern_dxf(self.runner.part(), solid.as_deref())?;
                let stem = self.file_stem().unwrap_or_else(|| "part".to_string());
                self.exported = Some(Export {
                    name: format!("{stem}_flat.dxf"),
                    content: Content::Text(dxf),
                });
                Changed::Nothing
            }
            Command::ExportStl => {
                let stem = self.file_stem().unwrap_or_else(|| "part".to_string());
                let triangles: Vec<_> = self
                    .runner
                    .part()
                    .solid_meshes(MESH_QUALITY)?
                    .iter()
                    .flat_map(|(_, triangles)| triangles.iter().map(stl::outward))
                    .collect();
                let mut bytes = Vec::new();
                stl::write_stl(&triangles, &stem, stl::StlFormat::Binary, &mut bytes)
                    .map_err(|e| GeopError::new(format!("writing {stem}.stl: {e}")))?;
                self.exported = Some(Export {
                    name: format!("{stem}.stl"),
                    content: Content::Bytes(bytes),
                });
                Changed::Nothing
            }
            Command::ExportBom { structure } => {
                let bom = inspect::bill_of_materials(self.runner.part(), &self.file(), structure)?;
                let stem = self.file_stem().unwrap_or_else(|| "part".to_string());
                self.exported = Some(Export {
                    name: format!("{stem}_bom.csv"),
                    content: Content::Text(bom.to_csv()),
                });
                Changed::Nothing
            }
            Command::DragTool { on } => {
                idle(self)?;
                self.drag_tool = on.then(DragTool::default);
                if on {
                    self.measure_tool = None;
                }
                Changed::Run
            }
            Command::MeasureTool { on } => {
                idle(self)?;
                self.measure_tool = on.then(|| MeasureTool::new(self.runner.part()));
                if on {
                    self.drag_tool = None;
                }
                Changed::Run
            }
            Command::Inspect { query } => {
                self.query = Some(query);
                Changed::Nothing
            }
            Command::ExportDrawing { id, format, date } => {
                let (index, mut args) = self.drawing(id.as_deref())?;
                let stem = self.file_stem();
                if args.name.is_empty() {
                    args.name = stem.clone().unwrap_or_default();
                }
                let library = library(&self.workspace, self.path.as_deref());
                self.runner.run(&self.program, Some(index), &library);
                let part = self.runner.part_at(index);
                let parts = inspect::parts_list(part, &self.file(), &args)?;
                // The drawing being edited is written from the views it
                // shows: they are projected already.
                let shown = self
                    .open
                    .as_ref()
                    .filter(|open| open.index == index)
                    .and_then(|open| {
                        open.editor
                            .session()
                            .downcast_ref::<geop_ops_drawing::operation::DrawingSession>()
                    });
                let sheet = match shown {
                    Some(session) => session.compose(part, &args, &date, &parts)?,
                    None => geop_ops_drawing::compose(part, &args, &date, &parts)?,
                };
                let text = format.write(&sheet);
                let base = stem.unwrap_or_else(|| "drawing".to_string());
                self.exported = Some(Export {
                    name: format!("{base}.{}", format.extension()),
                    content: Content::Text(text),
                });
                // Running only up to the drawing moved the runner: run again
                // as far as the program is shown.
                Changed::Run
            }
            Command::ExportUrdf => {
                idle(self)?;
                let index = self.marker.unwrap_or(self.program.steps.len());
                let library = library(&self.workspace, self.path.as_deref());
                self.runner.run(&self.program, Some(index), &library);
                let name = self.file_stem().unwrap_or_else(|| "robot".to_string());
                let robot = geop_ops_urdf::export(self.runner.part_at(index), &name, MESH_QUALITY)?;
                self.exported = Some(Export {
                    name: format!("{name}.zip"),
                    content: Content::Bytes(robot.zip()?),
                });
                Changed::Nothing
            }
            Command::Undo | Command::Redo if self.open.is_some() => {
                // While a step is edited, its own edits.
                let open = self.open.as_mut().expect("checked");
                let (from, to) = match command {
                    Command::Undo => (&mut open.undo, &mut open.redo),
                    _ => (&mut open.redo, &mut open.undo),
                };
                let Some((step, state)) = from.pop() else {
                    return Ok(Changed::Nothing);
                };
                to.push((open.editor.step().clone(), open.editor.state().clone()));
                open.editor.restore(step, state);
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

    /// Solves the program's mates from where its state puts its parts, and
    /// keeps where they put them: the program's state, which every step
    /// then sees (see [`geop_ops::part::State`]). While a step that moves
    /// no placed part is edited, only the program up to it runs: the steps
    /// after it are built again once the edit is done, not on every move of
    /// the pointer — editing a sketch early in a long program would
    /// otherwise rebuild everything after it each time.
    ///
    /// A drag the step being edited asks for pulls first, every part that is
    /// not fixed free to give way — a linkage follows the link dragged.
    /// Otherwise the parts move only if a mate does not hold: the step's own
    /// part first, the others held — mating a new part moves it, not the
    /// part it is mated to — and every part that is not fixed only where
    /// that is not enough. State no step declares any more go, and
    /// those declared but not given are kept at what the steps took for
    /// them.
    ///
    /// It may leave the runner having run the whole program, not what is
    /// shown: [`Self::rerun`] runs that.
    fn settle(&mut self) {
        let library = library(&self.workspace, self.path.as_deref());
        let program = match &self.open {
            Some(open) => self.with_open(open),
            None => self.program.clone(),
        };
        // Up to the step edited first: what it drags or holds is known
        // from the part it is applied to.
        self.runner.run(
            &program,
            self.open.as_ref().map(|open| open.index + 1),
            &library,
        );
        let (drags, holds, own) = match &self.open {
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
                (open.editor.drags(context), open.editor.holds(context), own)
            }
            None => (Vec::new(), Vec::new(), Vec::new()),
        };
        // The whole program, when the parts it places may move: with no
        // step edited, or while the step edited drags placed parts or holds
        // joints, which carry the parts placed after it along. Otherwise —
        // a sketch, a feature — the steps after it are built again once the
        // edit is done.
        let whole = self.open.is_none() || !drags.is_empty() || !holds.is_empty();
        if whole && self.open.is_some() {
            self.runner.run(&program, None, &library);
        }
        // Which state the program declares is only known once all of it
        // has run: until then, none is dropped.
        let complete = whole
            && self.runner.results().len() == program.steps.len()
            && self.runner.results().iter().all(|r| r.error.is_none());
        let part = self.runner.part();
        let hold = || {
            part.check_mates(|_| true)
                .is_ok_and(|report| report.converged)
        };
        let solved = if !drags.is_empty() {
            part.solve_mates(None, &holds, &drags)
                .ok()
                .map(|(moved, _)| moved)
        } else if hold() {
            None
        } else if !holds.is_empty() {
            part.solve_joints(&holds).ok().map(|(moved, _)| moved)
        } else {
            let own_first = own
                .first()
                .and_then(|name| part.solve_mates(Some(name), &holds, &[]).ok())
                .filter(|(_, report)| report.converged);
            own_first
                .or_else(|| part.solve_mates(None, &holds, &[]).ok())
                .map(|(moved, _)| moved)
        };
        let mut state = program.state.clone();
        state.extend(solved.into_iter().flatten());
        if complete {
            // Every pose a step declares, and every joint's coordinates, and
            // no other: the file says where every placed part is. The
            // numbers its steps read are its parameters', defined apart
            // from its state.
            let declared: State = part
                .state()
                .iter()
                .filter(|(_, value)| matches!(value, ParamValue::Pose(_)))
                .map(|(name, value)| (name.clone(), value.clone()))
                .collect();
            let solved = part.solved_parameters();
            state.retain(|name, _| declared.contains_key(name) || solved.contains(name));
            for (name, value) in declared {
                state.entry(name).or_insert(value);
            }
        }
        if state == program.state {
            return;
        }
        match &mut self.open {
            Some(open) => open.editor.set_state(state),
            None => self.program.state = state,
        }
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
                // What a step picks is its arguments': not what it built,
                // which is why it is kept for as long as the step is the
                // same. Without it, a form does none of the work it would
                // to show what was built — a placed part's mates checked
                // over the whole assembly, for every step.
                let session = step.operation.new_session();
                let context = Context::new(self.runner.part_at(i), &step.id, &library);
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

    /// Gives a drawing being edited the bill of materials of the part it
    /// draws, which it cannot list itself (see
    /// [`geop_ops_drawing::operation::DrawingSession::show`]) — once per
    /// part built, and whenever it starts or stops asking for one.
    fn give_parts_list(&mut self) {
        let file = self.file();
        let Some(open) = &mut self.open else {
            return;
        };
        let PartOperation::Drawing(args) = open.editor.step() else {
            return;
        };
        let args = args.clone();
        let before = self.runner.part_at(open.index);
        let Some(session) = open
            .editor
            .session_mut()
            .downcast_mut::<geop_ops_drawing::operation::DrawingSession>()
        else {
            return;
        };
        if session.wants_parts(before.revision(), args.bom) {
            let parts = inspect::parts_list(before, &file, &args);
            session.show(before.revision(), args.bom, parts);
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
            can_undo: !open.undo.is_empty(),
            can_redo: !open.redo.is_empty(),
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
                | EntityRef::Sketch3d { name }
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
        // Then what the user chose — but what a click could pick now is
        // shown, to be picked.
        for (name, &visible) in &self.visibility {
            let wanted = [
                EntityRef::datum(name.clone()),
                EntityRef::Sketch { name: name.clone() },
                EntityRef::Sketch3d { name: name.clone() },
                EntityRef::Solid { name: name.clone() },
                EntityRef::Face { name: name.clone() },
            ];
            if visible || wanted.iter().any(pickable) {
                hidden.retain(|h| h != name);
            } else if !hidden.contains(name) {
                hidden.push(name.clone());
            }
        }
        hidden
    }

    /// What the part `steps` steps build has beyond the faces of its solids:
    /// its datums, sketches, solids, faces standing on their own, the parts
    /// placed in it and their mates — each shown or not as `hidden` says,
    /// but a mate, which is never drawn.
    fn structure(&self, steps: usize, hidden: &[String]) -> Vec<StructureItem> {
        let part = self.runner.part_at(steps);
        let item = |kind, name: &str| StructureItem {
            kind,
            name: name.to_string(),
            visible: Some(!hidden.iter().any(|h| h == name)),
        };
        let named = |id: geop_ops::RefId| part.name_of(id).unwrap_or_default();
        let mut items: Vec<StructureItem> = part
            .datums()
            .map(|(id, _)| item(StructureKind::Datum, named(id.into())))
            .collect();
        items.extend(
            part.sketch_names()
                .iter()
                .chain(&part.sketch3d_names())
                .map(|n| item(StructureKind::Sketch, n)),
        );
        items.extend(
            part.solid_names()
                .iter()
                .map(|n| item(StructureKind::Solid, n)),
        );
        items.extend(
            part.sheet_face_names()
                .iter()
                .map(|n| item(StructureKind::Face, n)),
        );
        items.extend(
            part.instances()
                .map(|(id, _)| item(StructureKind::Part, named(id.into()))),
        );
        items.extend(part.mates().map(|(name, _)| StructureItem {
            kind: StructureKind::Mate,
            name: name.to_string(),
            visible: None,
        }));
        items
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
                // A summary is of the step's arguments and the program's
                // state, not of what it built: every step's form is asked
                // for on every change, and one shown with what it built
                // checks a placed part's mates over the whole assembly.
                let session = step.operation.new_session();
                let context = Context::new(self.runner.part_at(i), &step.id, &library)
                    .state(&self.program.state);
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
        let mut operations = PartOperation::infos();
        operations.sort_by_key(|info| (info.group, info.tier));
        ProgramState {
            program: self.program.clone(),
            path: self.path.clone(),
            steps,
            marker: self.marker.unwrap_or(self.program.steps.len()),
            can_undo: match &self.open {
                Some(open) => !open.undo.is_empty(),
                None => !self.undo.is_empty(),
            },
            can_redo: match &self.open {
                Some(open) => !open.redo.is_empty(),
                None => !self.redo.is_empty(),
            },
            drag_tool: self.drag_tool.is_some(),
            measure_tool: self.measure_tool.is_some(),
            materials: geop_ops::parameters::MATERIALS
                .iter()
                .map(|&(name, density)| geop_ops::parameters::Material {
                    name: name.into(),
                    density,
                })
                .collect(),
            parameters: self.program.parameters.resolve(&self.program.state),
            parameter_uses: self.program.parameter_uses(),
            operations,
            examples: self.examples.clone(),
            workspace_examples: self.workspace_examples.clone(),
            joints: self.runner.part().joints().unwrap_or_default(),
            freedom: (self.runner.part().instances().next().is_some())
                .then(|| self.runner.part().mate_freedom().ok())
                .flatten(),
        }
    }
}

/// The library the program in the file `path` is built with — with no file
/// yet, one in the workspace's top folder.
fn library<'w, S: Scalar>(workspace: &'w Workspace<S>, path: Option<&str>) -> impl Library<S> + 'w {
    workspace.scope(path.unwrap_or_default())
}

impl<S: Scalar> Editor<S> {
    /// The name of the program's file, without folders or `.geop` — what
    /// an exported file is named after — once the editor has been told it.
    /// The program's file, as the workspace names it — `part.geop` until
    /// the editor is told.
    fn file(&self) -> String {
        self.path.clone().unwrap_or_else(|| "part.geop".to_string())
    }

    fn file_stem(&self) -> Option<String> {
        self.path
            .as_deref()
            .and_then(|p| p.rsplit('/').next())
            .map(|f| f.trim_end_matches(".geop").to_string())
            .filter(|s| !s.is_empty())
    }

    /// The drawing to export, and how many steps run before it: the step
    /// `id`, else the drawing being edited, else the program's last; the
    /// default drawing of the part as far as it runs if there is none.
    fn drawing(&self, id: Option<&str>) -> GeopResult<(usize, geop_ops_drawing::DrawingArgs)> {
        use geop_core_math::geop_error::GeopError;
        if let Some(open) = &self.open
            && id.is_none_or(|id| id == open.id)
            && let PartOperation::Drawing(args) = open.editor.step()
        {
            return Ok((open.index, args.clone()));
        }
        if let Some(id) = id {
            let index = self.program.index_of(id)?;
            return match &self.program.steps[index].operation {
                PartOperation::Drawing(args) => Ok((index, args.clone())),
                _ => Err(GeopError::new(format!("the step {id:?} is no drawing"))),
            };
        }
        let runs = self.marker.unwrap_or(self.program.steps.len());
        let last = self.program.steps[..runs]
            .iter()
            .enumerate()
            .rev()
            .find_map(|(i, step)| match &step.operation {
                PartOperation::Drawing(args) => Some((i, args.clone())),
                _ => None,
            });
        Ok(last.unwrap_or((runs, geop_ops_drawing::DrawingArgs::default())))
    }
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
