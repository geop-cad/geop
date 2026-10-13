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
//! (see [`crate::workspace`]): the editor is told the file's path
//! with the program ([`Command::Load`]), and is sent the other files'
//! programs whenever they change ([`Command::Files`]) — by the front end,
//! which owns the files, be they a VS Code workspace's or the browser's.
//! Only the file it edits is its own: switching to another, it keeps the
//! program it leaves as that file's. What each step shows and does is the operation's; see
//! [`crate::step_editor::StepEditor`].

use std::{
    collections::{BTreeMap, BTreeSet, HashMap, HashSet},
    rc::Rc,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_editor::{
    editable::Listed,
    editing::{
        Dialog, Extent, Presentation, Pull, Shape, StepEditEvent, Style, Visual,
    },
};
use geop_core_program::{
    element::PickedElement,
    program::{Program as Steps, ProgramStep},
};
use geop_ops_assembly::{
    Library, instance_path, placed_parts,
    solving::{self, Freedom, Report, Rerun, SettingsState},
};
use geop_ops_model::Design;
use geop_ops_output::{Content, Export, ExporterInfo, Inspection, InspectorInfo, Output, Outputs};
use geop_ops_parameters::{Material, Parameters, PartReader, Settings, StateValues};
use geop_ops_view::{EMPTY_EXTENT, PartView, ViewInstance, drawn};
use serde::{Deserialize, Serialize};

use crate::{
    Program, Workspace, examples,
    inspect::PickTool,
    operations::{Editors, Runner, editors, outputs, workspace},
    part::{self, StructureItem, structure},
    step_editor::{Editing, StepEditor},
};

/// How many files' builds an editor keeps (see `Editor::builds`): each
/// holds every step's part, so a few large files are as many as fit.
const KEPT_BUILDS: usize = 6;
/// Something the user did.
#[derive(Clone, Debug, Deserialize)]
#[serde(tag = "command", rename_all = "snake_case")]
pub enum Command {
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
        event: StepEditEvent<Design>,
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
    /// The step `id` as text to paste ([`Program::excerpt`]), in the
    /// update's export.
    Copy {
        id: String,
    },
    /// Insert the steps of the program `text` — what [`Command::Copy`]
    /// gave, in this program or another — where new steps go
    /// ([`Program::paste`]).
    Paste {
        text: String,
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
    /// [`geop_ops_parameters`]): what every step reading one is built
    /// with again. Allowed while a step is edited, which then sees them.
    Parameters {
        parameters: Parameters,
    },
    /// Name the parameter `from` `to`, and every formula reading it read
    /// `to` (see [`geop_ops_parameters::ProgramFile::rename_parameter`]): the other
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
    /// Set the value `parameter` a step offers a user to set — a joint's
    /// coordinate: an angle in degrees, or a distance — to `value`: the
    /// parts move to it, everything else that is free giving way (see
    /// [`ProgramState::joints`]).
    Joint {
        parameter: String,
        value: f64,
    },
    /// Take the tool of the inspector `inspector` — one that picks (see
    /// [`InspectorInfo::picks`]) — in hand, or, `None`, put it down: with no
    /// step being edited, a click picks what the inspector takes, and every
    /// update carries its answer about the picks ([`Update::inspection`]).
    /// Inspecting is no edit: it changes nothing, and adds no step.
    PickTool {
        inspector: Option<String>,
    },
    /// Ask the inspector `inspector` about the part as drawn, told
    /// `options` (its own, see [`geop_ops_output::Inspector::Options`]):
    /// answered once, in [`Update::inspection`]. Changes nothing.
    Inspect {
        inspector: String,
        #[serde(default)]
        options: serde_json::Value,
    },
    /// Write what the program builds — as far as it runs, the parts it
    /// places included — with the exporter `exporter`, in `format` (its
    /// first, if not given), told `options` (its own, see
    /// [`geop_ops_output::Exporter::Options`]) and dated `date`
    /// (`YYYY-MM-DD`): the update's [`Update::export`]. Changes nothing.
    Export {
        exporter: String,
        #[serde(default)]
        format: Option<String>,
        #[serde(default)]
        options: serde_json::Value,
        #[serde(default)]
        date: String,
    },
    /// Panic, on purpose: how the front ends' recovery from a kernel that
    /// crashed is checked (`web/e2e/`). A panic anywhere else is a bug;
    /// this one stands in for it.
    Crash,
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
    /// The inspector whose pick tool is in hand, if one is.
    pub pick_tool: Option<&'static str>,
    /// The materials a part's can be picked from (see
    /// [`geop_ops_parameters::Material`]).
    pub materials: Vec<Material>,
    /// What the program's parameters resolve to, and why those that do
    /// not resolve fail.
    pub parameters: ParametersState,
    /// What reads each parameter, by name: the steps and the other
    /// parameters whose formulas do, which fail if it goes (see
    /// [`geop_ops_parameters::ProgramFile::parameter_uses`]).
    pub parameter_uses: BTreeMap<String, Vec<String>>,
    /// Every operation a step can be, as the toolbar offers them: group by
    /// group, in [`geop_core_program::operation::OperationGroup`]'s order, and
    /// within a group by [`geop_core_program::operation::OperationTier`], the most
    /// used first, then in the order [`crate::operations::infos`] lists
    /// them.
    pub operations: Vec<Listed>,
    /// What can be written of the part ([`Command::Export`]).
    pub exporters: Vec<&'static ExporterInfo>,
    /// What can be asked about it ([`Command::Inspect`]), or picked in it
    /// to ask about ([`Command::PickTool`]).
    pub inspectors: Vec<&'static InspectorInfo>,
    /// The names of the example programs [`Command::LoadExample`] loads.
    pub examples: Vec<&'static str>,
    /// The names of the examples of several files
    /// [`Command::LoadWorkspaceExample`] loads.
    pub workspace_examples: Vec<&'static str>,
    /// The values of the part shown a user sets, grouped as its steps
    /// declare them — a joint's coordinates — with where they are: what
    /// [`Command::Joint`] sets (see [`solving::settings`]).
    pub joints: Vec<SettingsState>,
    /// How free each of its placed parts is — if it has placed parts — as
    /// the last edit left them (see [`solving::freedom`]).
    pub freedom: Option<Freedom>,
    /// A smallest set of its constraints that cannot all hold together, by
    /// name: none where they all hold.
    pub conflicting: Vec<String>,
}

/// What is drawn: a part, and which of its sketches and datums not to.
///
/// The parts placed in it — however many, however deep — are sent as what
/// changed since the last scene: a viewer keeps them by name, and the
/// views of their components by key, each sent once. So moving one placed
/// part sends where that one is, and placing a screw a hundred times sends
/// what a screw looks like once.
#[derive(Clone, Debug, Serialize)]
pub struct SceneState {
    /// The part, without the parts placed in it: those are `instances`.
    pub part: PartView,
    /// The sketches and datums the steps shown have used: what was made
    /// from them shows them now. None of a kind while it is being picked.
    pub hidden: Vec<String>,
    /// The parts placed in the part drawn, however deep, that are new or
    /// moved or drawn from another component since the last scene — every
    /// one, if `all`.
    pub instances: Vec<ViewInstance>,
    /// Whether `instances` are all there are: a viewer forgets any others.
    pub all: bool,
    /// The names of the placed parts drawn last time and gone now.
    pub removed: Vec<String>,
    /// The views of the components the placed parts are drawn from, by key
    /// (see [`geop_ops_assembly::Component::key`]), that the last scene's did not
    /// use. A viewer keeps those the placed parts drawn use, and forgets
    /// the others: a component no placed part uses any more is sent again
    /// once one does.
    pub components: BTreeMap<String, PartView>,
    /// What the part has beyond its faces, to list and show or hide (see
    /// [`crate::part::structure`]).
    pub structure: Vec<StructureItem>,
}

/// The step being edited, as shown.
#[derive(Clone, Debug, Serialize)]
pub struct StepState {
    pub kind: &'static str,
    pub label: &'static str,
    pub doc: &'static str,
    /// Its id — none yet, for a new step.
    pub id: Option<String>,
    pub presentation: Presentation<Design>,
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
pub struct Update {
    /// Why the command was refused, if it was: then it changed nothing.
    pub error: Option<String>,
    pub program: Option<ProgramState>,
    pub scene: Option<SceneState>,
    /// The step being edited; `None` if there is none.
    pub step: Option<StepState>,
    /// The program files the command added, the one now edited first.
    pub files: Option<Vec<File>>,
    /// The file [`Command::Export`] wrote, or [`Command::Copy`] copied.
    pub export: Option<Export>,
    /// What the drag tool or the pick tool shows, while it is in hand and
    /// no step is edited: the part it would drag lit, and whether a press
    /// grabs it; what is picked, and what the answer about it draws.
    pub tool: Option<Presentation<Design>>,
    /// The answer of the pick tool's inspector about its picks, while it is
    /// in hand — or the answer to [`Command::Inspect`].
    pub inspection: Option<Inspection>,
}

/// The drag tool, in hand: the placed part the pointer is over, and the one
/// being dragged.
#[derive(Default)]
struct DragTool {
    hover: Option<String>,
    grab: Option<Grabbed>,
}

/// A placed part being dragged by the drag tool.
struct Grabbed {
    /// Its name, as drawn.
    name: String,
    /// The state its pose is.
    parameter: String,
    /// The point grabbed, in its own frame: where the pointer was, a free
    /// choice, so sharp.
    local: Vector3<Design>,
    /// The plane it is dragged in: through the point grabbed, facing the
    /// eye — `(point, normal)`.
    plane: (Vector3<Design>, Vector3<Design>),
    /// The program when it was grabbed: what undoing the drag goes back to.
    before: Program,
}

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
    /// The program's steps with the step in them, as its edits leave it:
    /// what the step editor writes the step's arguments to.
    steps: Steps,
    /// The program's state, as the step's edits and the solves after them
    /// leave it.
    state: StateValues,
    editor: StepEditor<Design>,
    /// What picks test against, as drawn: the part before it, or what it
    /// builds (see [`OperationInfo::picks_built`]).
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
    fn step(&self) -> &ProgramStep {
        &self.steps.steps[self.index]
    }

    /// The step, and the program's state, as they are now: what an undo
    /// comes back to.
    fn edited(&self) -> Edited {
        (self.step().args.clone(), self.state.clone())
    }
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
    file: Program,
    /// The file the program is in, once told: until then, the files it
    /// places are named relative to the workspace's top folder.
    path: Option<String>,
    /// The other program files, which it places parts from.
    workspace: Rc<Workspace>,
    /// The library the program is built with: the workspace from its file.
    /// Kept, so that a run reads the same library as the last one — a new
    /// one builds every placed part's step again — until a file changes.
    library: Rc<dyn Library>,
    undo: Vec<Program>,
    redo: Vec<Program>,
    marker: Option<usize>,
    open: Option<Open>,
    preview: bool,
    pub(crate) build: Build,
    /// The builds of the files edited before, by path, the latest last:
    /// opened again, a file takes up its steps as they were built, rather
    /// than building them all again. At most [`KEPT_BUILDS`].
    builds: Vec<(String, Build)>,
    /// Counts runs, so a scene is resent only after one.
    run: u64,
    /// What each step that ran builds on (see [`Dialog::picked`]), with the
    /// step it was read from: what a step picked is in its arguments, so it
    /// is read anew only when the step changed — not when a drag only moved
    /// the program's state.
    references: Vec<(ProgramStep, Vec<PickedElement>)>,
    shown: Option<Shown>,
    /// The names of the example programs, and of the examples made of
    /// several files — named once: building them is no work for every
    /// update.
    examples: Vec<&'static str>,
    workspace_examples: Vec<&'static str>,
    /// How the solve of the drag tool's last drag went.
    dragged: Option<Report>,
    /// A smallest set of the constraints that cannot all hold together, as
    /// the last settle found them (see [`solving::conflicting`]).
    conflicting: Vec<String>,
    /// How free the placed parts are, as the last edit left them (see
    /// [`solving::freedom`]): asked over the whole part, so once an edit,
    /// never for every move of the pointer.
    freedom: Option<Freedom>,
    /// The files the last command added, for the update to say.
    added: Option<Vec<File>>,
    /// The file the last command wrote, for the update to carry.
    exported: Option<Export>,
    /// The placed parts the viewer was sent, by name: the key of the
    /// component each is drawn from, and where it is, as sent — `None`
    /// when the viewer may have none (see [`SceneState`]).
    placed: Option<HashMap<String, (String, [f64; 12])>>,
    /// The drag tool, while in hand.
    drag_tool: Option<DragTool>,
    /// The pick tool, while in hand.
    pick_tool: Option<PickTool>,
    /// The inspector the last command asked, and what it told it: to answer
    /// once it has run.
    asked: Option<(String, serde_json::Value)>,
    /// The exporters and inspectors offered.
    pub(crate) outputs: Outputs,
    /// The operations offered, and how each is edited.
    pub(crate) editors: Editors,
    /// The part as last drawn: what the drag tool picks from.
    drawn: Option<PartView>,
    /// What the user chose to show or hide, by name, over what the editor
    /// would by itself (see [`Editor::hidden`]).
    visibility: BTreeMap<String, bool>,
}

impl Default for Editor {
    fn default() -> Self {
        Self::new()
    }
}

impl Editor {
    pub fn new() -> Self {
        let workspace = workspace(BTreeMap::new());
        Self {
            file: Program::default(),
            path: None,
            library: workspace.scope(""),
            workspace,
            undo: Vec::new(),
            redo: Vec::new(),
            marker: None,
            open: None,
            preview: true,
            build: Build::new(),
            builds: Vec::new(),
            run: 0,
            references: Vec::new(),
            shown: None,
            examples: examples::all().into_iter().map(|(name, _)| name).collect(),
            workspace_examples: (examples::workspaces().into_iter())
                .map(|(name, _)| name)
                .collect(),
            dragged: None,
            conflicting: Vec::new(),
            freedom: None,
            added: None,
            exported: None,
            placed: None,
            drag_tool: None,
            pick_tool: None,
            asked: None,
            outputs: outputs()
                .expect("the editor's exporters and inspectors each have a kind of their own"),
            editors: editors().expect("the editor's operations each have a kind of their own"),
            drawn: None,
            visibility: BTreeMap::new(),
        }
    }

    pub fn program(&self) -> &Program {
        &self.file
    }

    /// The part the program builds, as far as it runs now.
    pub fn part(&self) -> PartReader<'_> {
        self.build.runner.part.current()
    }

    /// The step being edited, with its arguments as they now are.
    pub fn editing(&self) -> Option<&ProgramStep> {
        self.open.as_ref().map(Open::step)
    }

    /// The step editor of the step being edited: its operation's editing
    /// state among what it keeps.
    pub fn step_editor(&self) -> Option<&StepEditor<Design>> {
        self.open.as_ref().map(|open| &open.editor)
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

    /// How the solve of the drag tool's last drag went: whether the
    /// constraints hold, and how hard it was to get there.
    pub fn dragged(&self) -> Option<&Report> {
        self.dragged.as_ref()
    }

    /// The numbers the program's parameters come to, as a run is given
    /// them.
    fn inputs(&self) -> BTreeMap<String, Design> {
        self.file.inputs(&Settings::new())
    }

    /// Gives the runner what a run of the program is given besides its
    /// numbers: what it says of its part, and the library it places parts
    /// from (see [`part::given`]).
    fn give(&mut self) {
        let path = self.path.clone().unwrap_or_default();
        let about = self.file.about(&path, &Settings::new());
        self.build.runner.given = Some(part::given(about, self.library.clone()));
    }

    /// Runs the first `end` of the steps run now — all of them, if `None` —
    /// with their state (see [`Self::current`]), keeping how far it ran and
    /// why steps failed.
    fn run_to(&mut self, end: Option<usize>) {
        let (steps, state) = self.current();
        let inputs = self.inputs();
        self.give();
        self.build.run(&steps, end, &inputs, &state);
    }

    /// The steps run now, and the state they run with: the step being
    /// edited's, or the program's.
    fn current(&self) -> (Steps, StateValues) {
        match &self.open {
            Some(open) => (open.steps.clone(), open.state.clone()),
            None => (self.file.program.clone(), self.file.state.clone()),
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
            self.measure_freedom();
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
        if !matches!(result, Ok(Changed::Nothing))
            && let Some(tool) = &mut self.pick_tool
        {
            tool.answer = None;
        }
        let inspection = match self.answer(step.is_none()) {
            Ok(inspection) => inspection,
            Err(e) => {
                error = Some(e.to_string());
                None
            }
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

    /// What to draw, `shown`: the part, and of the parts placed in it what
    /// the viewer was not sent yet.
    fn scene(&mut self, shown: Shown) -> SceneState {
        self.shown = Some(shown.clone());
        let part = match shown.sheet {
            Some((center, size)) => PartView::blank(Extent {
                center: Vector3::from_array(center.map(Design::from_f64)),
                size: Design::from_f64(size),
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
            for (k, x) in frame.iter().flatten().enumerate() {
                numbers[k] = *x;
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
        SceneState {
            part,
            structure: structure(&self.shown_part(shown.steps), &shown.hidden).unwrap_or_default(),
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
        ask: impl FnOnce(&Outputs, &mut Output) -> GeopResult<T>,
    ) -> GeopResult<T> {
        let (steps, state) = self.current();
        let editing = self.open.as_ref().map(|open| open.id.clone());
        let file = Program {
            parameters: self.file.parameters.clone(),
            program: steps,
            state,
        };
        let mut output = Output {
            runner: &mut self.build.runner,
            file: &file,
            editing: editing.as_deref(),
            date,
        };
        ask(&self.outputs, &mut output)
    }

    /// The answer to the inspector the last command asked, if it asked one
    /// — else, `idle` (no step edited), the pick tool's inspector's about
    /// its picks, asked again only if they or the part changed since.
    fn answer(&mut self, idle: bool) -> GeopResult<Option<Inspection>> {
        if let Some((kind, options)) = self.asked.take() {
            return self.ask("", |outputs, output| {
                let inspector = outputs.get_inspector(&kind)?;
                inspector.inspect(output, &[], &options).map(Some)
            });
        }
        let Some(tool) = self.pick_tool.as_ref().filter(|_| idle) else {
            return Ok(None);
        };
        if tool.answer.is_none() {
            let (kind, picked) = (tool.kind(), tool.picked().to_vec());
            let null = serde_json::Value::Null;
            let answer = self.ask("", |outputs, output| {
                outputs.get_inspector(kind)?.inspect(output, &picked, &null)
            })?;
            if let Some(tool) = &mut self.pick_tool {
                tool.answer = Some(answer);
            }
        }
        Ok(self.pick_tool.as_ref().and_then(|tool| tool.answer.clone()))
    }

    /// What the drag tool or the pick tool shows, if one is in hand and
    /// no step is edited.
    fn tool_presentation(&self, idle: bool) -> Option<Presentation<Design>> {
        if let Some(tool) = self.pick_tool.as_ref().filter(|_| idle) {
            return Some(tool.presentation());
        }
        let tool = self.drag_tool.as_ref().filter(|_| idle)?;
        let lit = (tool.grab.as_ref().map(|g| &g.name)).or(tool.hover.as_ref());
        Some(Presentation {
            dialog: Dialog::new(),
            visuals: (lit.map(|name| {
                let shape = Shape::Instance { name: name.clone() };
                Visual::new("drag", shape, Style::Hover)
            }))
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
    fn drag(&mut self, mut tool: DragTool, event: &StepEditEvent<Design>) -> Changed {
        let part_at = |editor: &Self, pointer| {
            let view = editor.drawn.as_ref()?;
            let (instance, t) = view.part_to_drag(pointer)?;
            let parameter = instance.state.clone()?;
            Some((instance.name.clone(), parameter, instance.pose, t))
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
                        before: self.file.clone(),
                    });
                }
                let Some(grab) = &tool.grab else {
                    self.drag_tool = Some(tool);
                    return Changed::Nothing;
                };
                let (point, normal) = &grab.plane;
                if let Some((_, target)) = to.ray.intersect_plane(point, normal) {
                    self.run_to(None);
                    let pull = Pull::Point {
                        parameter: grab.parameter.clone(),
                        local: grab.local,
                        target: target.sharpen(),
                    };
                    let (steps, state) = self.current();
                    let inputs = self.inputs();
                    let mut rerun = Rerun {
                        runner: &mut self.build.runner,
                        program: &steps,
                        inputs: &inputs,
                    };
                    if let Ok((moved, report)) = solving::solve(&mut rerun, &state, &[pull], None)
                    {
                        self.file.state.extend(moved);
                        self.dragged = Some(report);
                    }
                }
                if *done
                    && let Some(grab) = tool.grab.take()
                    && grab.before != self.file
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

    /// Opens the step `id` of `steps`, at `index` — `new`, inserted there,
    /// or one already there — to edit.
    fn open(&mut self, steps: Steps, index: usize, id: String, new: bool) -> GeopResult<()> {
        let state = self.file.state.clone();
        // It runs once before it is edited: an editor edits a step against
        // the part before it, as the last run left it.
        let inputs = self.inputs();
        self.give();
        self.build.run(&steps, Some(index + 1), &inputs, &state);
        let editor = StepEditor::new(&self.editors, &self.build.runner, &steps, &id, new)?;
        let view = part::draw(&self.build.view_at(&steps, index))?;
        self.open = Some(Open {
            index,
            id,
            new,
            steps,
            state,
            editor,
            view,
            undo: Vec::new(),
            redo: Vec::new(),
            dragging: None,
        });
        Ok(())
    }

    /// Applies `command`, saying what it changed.
    fn apply(&mut self, command: Command) -> GeopResult<Changed> {
        let idle = |editor: &Self| match editor.open {
            Some(_) => Err(GeopError::new("finish editing the step first")),
            None => Ok(()),
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
                self.pick_tool = None;
                let index = self.marker.unwrap_or(self.file.program.steps.len());
                self.run_to(Some(index));
                let label = self.label_of(&kind)?;
                let id = self.file.fresh_id(label);
                // The part as just run: the part before where the step goes.
                let program = self.file.program.clone();
                let step = (self.editors).new_step(&self.build.runner, &program, &kind, &id, None)?;
                let mut steps = self.file.program.clone();
                steps.steps.insert(index, step);
                self.open(steps, index, id, true)?;
                Changed::Run
            }
            Command::Open { id } => {
                idle(self)?;
                self.drag_tool = None;
                self.pick_tool = None;
                let index = self.file.index_of(&id)?;
                self.open(self.file.program.clone(), index, id, false)?;
                Changed::Run
            }
            Command::Event { event } => {
                let Some(open) = &mut self.open else {
                    if let Some(tool) = self.drag_tool.take() {
                        return Ok(self.drag(tool, &event));
                    }
                    if let (Some(tool), Some(view)) = (&mut self.pick_tool, &self.drawn) {
                        tool.handle(view, &event);
                    }
                    return Ok(Changed::Nothing);
                };
                let before = open.edited();
                let editing = Editing {
                    editors: &self.editors,
                    runner: &self.build.runner,
                    program: &mut open.steps,
                    scene: &open.view,
                };
                open.editor.handle(editing, &event)?;
                let after = open.edited();
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
                    .form(&self.editors, &self.build.runner, &open.steps)?
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
                if let Some(error) = self.build.step_error(open) {
                    return Err(GeopError::new(format!("the step does not build: {error}")));
                }
                self.file.program = open.steps.clone();
                self.file.state = open.state.clone();
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
                self.run_to(None);
                let settable = (solving::settings(&self.part()).into_iter())
                    .any(|s| s.values.iter().any(|v| v.state == parameter));
                if !settable {
                    return Err(GeopError::new(format!(
                        "{parameter:?} is no value of this program a user sets"
                    )));
                }
                let set = BTreeMap::from([(parameter, Design::from_f64(value))]);
                let (steps, state) = self.current();
                let inputs = self.inputs();
                let mut rerun = Rerun {
                    runner: &mut self.build.runner,
                    program: &steps,
                    inputs: &inputs,
                };
                let (moved, _) = solving::set(&mut rerun, &state, &set)?;
                self.file.state.extend(moved);
                Changed::Program
            }
            Command::Parameters { parameters } => {
                parameters.validate()?;
                self.file.parameters = parameters;
                Changed::Program
            }
            Command::RenameParameter { from, to } => {
                idle(self)?;
                self.file.rename_parameter(&from, &to)?;
                Changed::Program
            }
            Command::Remove { id } => {
                idle(self)?;
                let index = self.file.index_of(&id)?;
                self.file.program.steps.remove(index);
                if let Some(marker) = &mut self.marker
                    && index < *marker
                {
                    *marker -= 1;
                }
                Changed::Program
            }
            Command::Move { id, index } => {
                idle(self)?;
                let from = self.file.index_of(&id)?;
                let steps = &mut self.file.program.steps;
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
                idle(self)?;
                let pasted = Program::from_json(&text).map_err(|e| {
                    e.with_context("pasting: what is pasted is not steps copied from geop")
                })?;
                let index = self.marker.unwrap_or(self.file.program.steps.len());
                let infos = self.editors.infos();
                let label = |step: &ProgramStep| match infos.iter().find(|l| l.kind == step.operation) {
                    Some(listed) => listed.info.label.to_string(),
                    None => step.operation.clone(),
                };
                let ids = self.file.paste(pasted, index, label)?;
                if let Some(marker) = &mut self.marker {
                    *marker += ids.len();
                }
                Changed::Program
            }
            Command::Seek { marker } => {
                idle(self)?;
                self.marker = marker.filter(|&m| m < self.file.program.steps.len());
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
                    let build = match self.builds.iter().position(|(kept, _)| *kept == path) {
                        Some(k) => self.builds.remove(k).1,
                        None => Build::new(),
                    };
                    let left_build = std::mem::replace(&mut self.build, build);
                    if let Some(left) = self.path.replace(path) {
                        self.workspace.write(&left, Some(self.file.to_json()?));
                        self.builds.push((left, left_build));
                        if self.builds.len() > KEPT_BUILDS {
                            self.builds.remove(0);
                        }
                    }
                    self.library = self
                        .workspace
                        .scope(self.path.as_deref().unwrap_or_default());
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
                // A new library: every step that places a part asks it
                // again, and takes what it kept where nothing changed.
                self.library = self
                    .workspace
                    .scope(self.path.as_deref().unwrap_or_default());
                // What the step being edited is built on may have changed.
                if let Some(index) = self.open.as_ref().map(|open| open.index) {
                    self.run_to(Some(index));
                    if let Some(open) = &mut self.open {
                        open.view = part::draw(&self.build.view_at(&open.steps, index))?;
                    }
                }
                Changed::Run
            }
            Command::LoadExample { name } => {
                idle(self)?;
                let (_, program) = (examples::all().into_iter())
                    .find(|(n, _)| *n == name)
                    .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))?;
                self.file = program();
                self.marker = None;
                Changed::Program
            }
            Command::LoadWorkspaceExample { name, folder } => {
                idle(self)?;
                let (_, files) = (examples::workspaces().into_iter())
                    .find(|(n, _)| *n == name)
                    .ok_or_else(|| GeopError::new(format!("there is no example {name:?}")))?;
                let files: Vec<File> = (files().into_iter())
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
                self.library = self.workspace.scope(&first.path);
                self.marker = None;
                self.undo.clear();
                self.redo.clear();
                self.build = Build::new();
                self.added = Some(files);
                Changed::Program
            }
            Command::Visibility { name, visible } => {
                self.visibility.insert(name, visible);
                Changed::Nothing
            }
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
            Command::DragTool { on } => {
                idle(self)?;
                self.drag_tool = on.then(DragTool::default);
                if on {
                    self.pick_tool = None;
                }
                Changed::Run
            }
            Command::PickTool { inspector } => {
                idle(self)?;
                self.pick_tool = match inspector {
                    Some(kind) => {
                        let info = self.outputs.get_inspector(&kind)?.info();
                        self.drag_tool = None;
                        Some(PickTool::new(info)?)
                    }
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
                let now = open.edited();
                let (from, to) = match command {
                    Command::Undo => (&mut open.undo, &mut open.redo),
                    _ => (&mut open.redo, &mut open.undo),
                };
                let Some((args, state)) = from.pop() else {
                    return Ok(Changed::Nothing);
                };
                to.push(now);
                open.steps.steps[open.index].args = args;
                open.state = state;
                open.editor.restart(&self.editors, &open.steps)?;
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

    /// Solves the program's mates from where its state puts its parts, and
    /// keeps where they put them: the program's state, which every step
    /// then sees. While a step that pulls no placed part is edited, only
    /// the program up to it runs: the steps after it are built again once
    /// the edit is done, not on every move of the pointer — editing a
    /// sketch early in a long program would otherwise rebuild everything
    /// after it each time.
    ///
    /// What the step being edited pulls ([`Pull`]) pulls first, every part
    /// that is not fixed free to give way — a linkage follows the link
    /// dragged. Otherwise the parts move only if a mate does not hold: the
    /// step's own part first, the others held — mating a new part moves
    /// it, not the part it is mated to — and every part that is not fixed
    /// only where that is not enough. State nothing solves for any more
    /// goes.
    ///
    /// It may leave the runner having run the whole program, not what is
    /// shown: [`Self::rerun`] runs that.
    fn settle(&mut self) {
        let (steps, state) = self.current();
        // Up to the step edited first: what it pulls is known from the part
        // it is applied to.
        self.run_to(self.open.as_ref().map(|open| open.index + 1));
        let pulls = match &self.open {
            Some(open) => (open.editor.form(&self.editors, &self.build.runner, &open.steps))
                .map(|form| form.pulls)
                .unwrap_or_default(),
            None => Vec::new(),
        };
        // The whole program, when the parts it places may move: with no
        // step edited, or while the step edited pulls them, which carries
        // the parts placed after it along. Otherwise — a sketch, a feature
        // — the steps after it are built again once the edit is done.
        let whole = self.open.is_none() || !pulls.is_empty();
        if whole && self.open.is_some() {
            self.run_to(None);
        }
        // Which state the program solves for is only known once all of it
        // has run: until then, none is dropped.
        let complete = whole && self.build.ran == steps.steps.len() && self.build.errors.is_empty();
        let holds = solving::check(&self.part()).is_ok_and(|report| report.converged);
        let own = (self.open.as_ref()).map(|open| open.id.clone());
        let inputs = self.inputs();
        let mut rerun = Rerun {
            runner: &mut self.build.runner,
            program: &steps,
            inputs: &inputs,
        };
        let solved = if !pulls.is_empty() {
            solving::solve(&mut rerun, &state, &pulls, None).ok()
        } else if holds {
            None
        } else {
            let own_first = (own.as_deref())
                .and_then(|own| solving::solve(&mut rerun, &state, &[], Some(own)).ok())
                .filter(|(_, report)| report.converged);
            own_first.or_else(|| solving::solve(&mut rerun, &state, &[], None).ok())
        };
        // Which constraints conflict is asked only where they do not all
        // hold: a solve, and an enclosure of each group that fails.
        self.conflicting = match &solved {
            Some((_, report)) if !report.converged => {
                solving::conflicting(&mut rerun, &state).unwrap_or_default()
            }
            _ => Vec::new(),
        };
        let mut next = state.clone();
        next.extend(solved.into_iter().flat_map(|(moved, _)| moved));
        if complete && let Ok(solvable) = solving::solved_states(&self.part()) {
            next.retain(|name, _| solvable.contains(name));
        }
        if next == state {
            return;
        }
        match &mut self.open {
            Some(open) => open.state = next,
            None => self.file.state = next,
        }
    }

    /// How free the placed parts of what is shown are, kept for the program
    /// state to show (see [`solving::freedom`]): asked once an edit, never
    /// for every move of the pointer.
    fn measure_freedom(&mut self) {
        let (steps, state) = self.current();
        let placed = placed_parts(&self.part()).is_ok_and(|placed| !placed.is_empty());
        let inputs = self.inputs();
        let mut rerun = Rerun {
            runner: &mut self.build.runner,
            program: &steps,
            inputs: &inputs,
        };
        self.freedom = placed
            .then(|| solving::freedom(&mut rerun, &state, &BTreeSet::new()).ok())
            .flatten();
    }

    /// Runs what is shown: the program up to and including the step being
    /// edited, or as far as it runs.
    fn rerun(&mut self) {
        self.run_to(match &self.open {
            Some(open) => Some(open.index + 1),
            None => self.marker,
        });
        self.run += 1;
        let (steps, _) = self.current();
        let before = std::mem::take(&mut self.references);
        self.references = (steps.steps[..self.build.ran].iter().enumerate())
            .map(|(i, step)| {
                if let Some((_, picked)) = before.get(i).filter(|(read, _)| read == step) {
                    return (step.clone(), picked.clone());
                }
                // What a step picks is its arguments': not what it built,
                // which is why it is kept for as long as the step is the
                // same, and read from the form a list shows.
                let picked = (self.editors.listed(&self.build.runner, &steps, &step.id))
                    .map(|form| form.dialog.picked().cloned().collect())
                    .unwrap_or_default();
                (step.clone(), picked)
            })
            .collect();
        if let Some(open) = &self.open
            && self.build.picks_built(&self.editors, open)
            && let Ok(view) = part::draw(&self.build.picks_in(&self.editors, open))
            && let Some(open) = &mut self.open
        {
            open.view = view;
        }
    }

    /// What the step being edited shows.
    fn presentation(&self, open: &Open) -> GeopResult<Presentation<Design>> {
        (open.editor).presentation(&self.editors, &self.build.runner, &open.steps, &open.view)
    }

    /// What the operation `kind` is called.
    fn label_of(&self, kind: &str) -> GeopResult<&'static str> {
        (self.editors.infos().into_iter())
            .find(|listed| listed.kind == kind)
            .map(|listed| listed.info.label)
            .ok_or_else(|| GeopError::new(format!("unknown operation `{kind}`")))
    }

    fn step_state(&self) -> Option<StepState> {
        let open = self.open.as_ref()?;
        let listed = (self.editors.infos().into_iter())
            .find(|listed| listed.kind == open.step().operation)?;
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
                .then(|| self.build.step_error(open))
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
    fn shown_part(&self, steps: usize) -> PartReader<'_> {
        let (program, _) = self.current();
        self.build.view_at(&program, steps)
    }

    /// What the shown steps built on — sketches, and datums as a whole —
    /// but none that a field picking now, from what is drawn of the kinds
    /// `picking`, could pick something of.
    fn hidden(&self, picking: Option<&[String]>, steps: usize) -> Vec<String> {
        let picking = picking.unwrap_or_default();
        let picks = |kind: &str| picking.iter().any(|p| p == kind);
        let view = self.open.as_ref().map(|open| &open.view);
        let pickable = |name: &str| {
            let sketch = (picks(drawn::SKETCH) || picks(drawn::SKETCH_POINT) || picks(drawn::SKETCH_CURVE))
                && view.is_some_and(|v| v.sketches.iter().any(|s| s.name == name));
            let datum = picks(drawn::DATUM)
                && view.is_some_and(|v| v.datums.iter().any(|d| d.name == name));
            sketch || datum
        };
        // A sketch built on is hidden by its name, as is a datum built on as
        // a whole.
        let built_on = |picked: &PickedElement| {
            if let Some(sketch) = picked.target.strip_prefix("sketch:") {
                return Some(sketch.to_string());
            }
            (picked.kind == drawn::DATUM && !picked.name.contains(':'))
                .then(|| picked.name.clone())
        };
        let mut hidden: Vec<String> = (self.references.iter())
            .flat_map(|(_, picked)| picked)
            .filter_map(built_on)
            .filter(|name| !pickable(name))
            .collect();
        hidden.sort();
        hidden.dedup();
        // The datums of the parts placed are theirs, not the part's: shown
        // only while a part is being placed, for its mates to pick.
        let placing = (self.open.as_ref()).is_some_and(|open| self.build.picks_built(&self.editors, open));
        if !placing && let Ok(placed) = placed_parts(&self.shown_part(steps)) {
            for part in placed {
                if let Ok(drawn) = part.located.instance.component.drawn() {
                    hidden
                        .extend((drawn.datums.iter()).map(|d| instance_path(&part.name, &d.name)));
                }
            }
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
        part::draw(&self.shown_part(steps)).unwrap_or_else(|_| {
            PartView::blank(Extent {
                center: Vector3::zero(),
                size: Design::from_f64(EMPTY_EXTENT),
            })
        })
    }

    fn program_state(&self) -> ProgramState {
        let editing = (self.open.as_ref())
            .filter(|o| !o.new)
            .map(|o| o.id.as_str());
        let runs_to = match &self.open {
            Some(open) => open.index,
            None => self.marker.unwrap_or(self.file.program.steps.len()),
        };
        let infos = self.editors.infos();
        let steps = (self.file.program.steps.iter().enumerate())
            .map(|(i, step)| {
                let listed = infos.iter().find(|l| l.kind == step.operation);
                // A summary is of the step's arguments, not of what it
                // built (see [`Editors::listed`]).
                let summary = (self.editors.listed(&self.build.runner, &self.file.program, &step.id))
                    .map(|form| form.dialog.summary())
                    .unwrap_or_default();
                StepInfo {
                    id: step.id.clone(),
                    kind: listed.map_or("", |l| l.kind),
                    label: listed.map_or("", |l| l.info.label),
                    summary,
                    error: (i < runs_to)
                        .then(|| self.build.errors.get(&step.id).cloned())
                        .flatten(),
                    runs: i < runs_to,
                    editing: editing == Some(step.id.as_str()),
                }
            })
            .collect();
        let mut operations = crate::operations::infos();
        operations.sort_by_key(|listed| (listed.info.group, listed.info.tier));
        let part = self.part();
        let placed = placed_parts(&part).is_ok_and(|placed| !placed.is_empty());
        ProgramState {
            program: self.file.clone(),
            path: self.path.clone(),
            steps,
            marker: self.marker.unwrap_or(self.file.program.steps.len()),
            can_undo: match &self.open {
                Some(open) => !open.undo.is_empty(),
                None => !self.undo.is_empty(),
            },
            can_redo: match &self.open {
                Some(open) => !open.redo.is_empty(),
                None => !self.redo.is_empty(),
            },
            drag_tool: self.drag_tool.is_some(),
            pick_tool: self.pick_tool.as_ref().map(PickTool::kind),
            materials: Material::listed().collect(),
            parameters: ParametersState::of(&self.file.parameters),
            parameter_uses: self.file.parameter_uses(),
            operations,
            exporters: self.outputs.exporters(),
            inspectors: self.outputs.inspectors(),
            examples: self.examples.clone(),
            workspace_examples: self.workspace_examples.clone(),
            joints: solving::settings(&part),
            freedom: self.freedom.clone().filter(|_| placed),
            conflicting: self.conflicting.clone(),
        }
    }
}

/// What the editor built: the runner, holding every step's part, and how
/// its last run went.
pub(crate) struct Build {
    pub(crate) runner: Runner,
    /// How many steps the last run ran.
    ran: usize,
    /// Why each step of the last run that failed did, by its id.
    errors: BTreeMap<String, String>,
}

impl Build {
    fn new() -> Self {
        Self {
            runner: crate::operations::runner(),
            ran: 0,
            errors: BTreeMap::new(),
        }
    }

    /// Runs the first `end` of `steps` — all of them, if `None` — with
    /// `inputs` and the state `state`, keeping how far it ran and why steps
    /// failed.
    fn run(
        &mut self,
        steps: &Steps,
        end: Option<usize>,
        inputs: &BTreeMap<String, Design>,
        state: &StateValues,
    ) {
        let end = end.unwrap_or(steps.steps.len()).min(steps.steps.len());
        let output = self.runner.run_until(steps, end, inputs, state);
        self.ran = end;
        self.errors = (output.step_errors.into_iter())
            .map(|(step, error)| (step, error.to_string()))
            .collect();
    }

    /// The part before the step at `index` of `steps`, as the last run left
    /// it — or, at or past how far it ran, the part it built.
    fn view_at(&self, steps: &Steps, index: usize) -> PartReader<'_> {
        let part = &self.runner.part;
        match index < self.ran.min(steps.steps.len()) {
            true => part.reader(Runner::position(index)),
            false => part.current(),
        }
    }

    /// Why the step `open` does not build, if it does not.
    fn step_error(&self, open: &Open) -> Option<String> {
        if let Some(error) = self.errors.get(&open.id) {
            return Some(error.clone());
        }
        if open.index < self.ran {
            return None;
        }
        Some(match self.errors.values().next() {
            Some(error) => format!("an earlier step failed: {error}"),
            None => "it did not run".into(),
        })
    }

    /// Whether the step `open` picks from what it builds (see
    /// [`OperationInfo::picks_built`]), now that it does.
    fn picks_built(&self, editors: &Editors, open: &Open) -> bool {
        let picks_built = (editors.infos().into_iter())
            .any(|listed| listed.kind == open.step().operation && listed.info.picks_built);
        picks_built && self.step_error(open).is_none()
    }

    /// The part the step `open` picks from: the part before it — or, for
    /// one that picks from what it builds, that, once it builds.
    fn picks_in(&self, editors: &Editors, open: &Open) -> PartReader<'_> {
        let index = open.index + usize::from(self.picks_built(editors, open));
        self.view_at(&open.steps, index)
    }
}

/// A program's parameters as an editor shows them: what each comes to —
/// a number, a table's row, the colour — and why those that do not fail.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ParametersState {
    pub values: Settings,
    pub errors: BTreeMap<String, String>,
}

impl ParametersState {
    fn of(parameters: &Parameters) -> Self {
        let resolved = parameters.resolve(&Settings::new());
        let mut values = resolved.chosen;
        for (name, number) in resolved.numbers {
            values
                .entry(name)
                .or_insert(geop_ops_parameters::Setting::Number(number.to_f64()));
        }
        Self {
            values,
            errors: resolved.errors,
        }
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
