//! What an [`super::Editor`] is told ([`Command`]) and what it answers
//! ([`Update`]): the protocol every front end speaks, as JSON.

use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::program::Program;
use serde::{Deserialize, Serialize};

use crate::{
    editable::Listed,
    editing::{Presentation, StepEditEvent},
    output::{Export, ExporterInfo, Inspection, InspectorInfo},
    registry::{Panel, StructureItem},
    tools::ToolInfo,
    view::{PartView, ViewInstance},
};

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
    /// Something the user did to the step being edited — or, with none,
    /// with the tool in hand.
    Event {
        event: StepEditEvent,
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
    /// Load the example program `name` (see
    /// [`crate::registry::ExampleSet`]).
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
    /// Something the user did in the panel `key` (see
    /// [`crate::registry::Panel`]), which the panel reads `payload` as.
    Panel {
        key: String,
        payload: serde_json::Value,
    },
    /// Show the datum, sketch, solid or placed part `name` of the part
    /// drawn, or hide it — whatever the editor would do by itself — for as
    /// long as the same file is edited.
    Visibility {
        name: String,
        visible: bool,
    },
    /// Take the tool `key` in hand (see [`crate::tools::Tool`]), or —
    /// `None` — put the one in hand down. With no step being edited, the
    /// events of the viewport are the tool's. `pick:<inspector>` is the
    /// pick tool of an inspector that picks (see
    /// [`crate::tools::pick::PickTool`]): every update carries its answer
    /// about the picks ([`Update::inspection`]).
    Tool {
        key: Option<String>,
    },
    /// Ask the inspector `inspector` about the part as drawn, told
    /// `options` (its own, see [`crate::output::Inspector::Options`]):
    /// answered once, in [`Update::inspection`]. Changes nothing.
    Inspect {
        inspector: String,
        #[serde(default)]
        options: serde_json::Value,
    },
    /// Write what the program builds — as far as it runs, the parts it
    /// places included — with the exporter `exporter`, in `format` (its
    /// first, if not given), told `options` (its own, see
    /// [`crate::output::Exporter::Options`]) and dated `date`
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

impl Command {
    /// The command sending the panel `P` its command `command`: what a
    /// front end in Rust sends.
    pub fn panel<P: Panel>(command: &P::Command) -> GeopResult<Self>
    where
        P::Command: Serialize,
    {
        let payload = serde_json::to_value(command)
            .map_err(|e| GeopError::new(format!("a command of the panel `{}`: {e}", P::KEY)))?;
        Ok(Self::Panel {
            key: P::KEY.to_string(),
            payload,
        })
    }
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
    /// The key of the tool in hand, if one is ([`Command::Tool`]).
    pub tool: Option<String>,
    /// The tools to take in hand, offered beside the inspectors — none of
    /// them is an operation; the pick tools are the inspectors'.
    pub tools: Vec<ToolInfo>,
    /// Every operation a step can be, as the toolbar offers them: group by
    /// group, in [`crate::editable::OperationGroup`]'s order, and within a
    /// group by [`crate::editable::OperationTier`], the most used first,
    /// then in the order they were registered.
    pub operations: Vec<Listed>,
    /// What can be written of the part ([`Command::Export`]).
    pub exporters: Vec<&'static ExporterInfo>,
    /// What can be asked about it ([`Command::Inspect`]), or picked in it
    /// to ask about (the tool `pick:<inspector>`).
    pub inspectors: Vec<&'static InspectorInfo>,
    /// The names of the example programs [`Command::LoadExample`] loads.
    pub examples: Vec<&'static str>,
    /// The names of the examples of several files
    /// [`Command::LoadWorkspaceExample`] loads.
    pub workspace_examples: Vec<&'static str>,
    /// What each panel shows of the program, by its key (see
    /// [`crate::registry::Panel`]).
    pub panels: BTreeMap<String, serde_json::Value>,
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
    /// (see [`ViewInstance::component`]), that the last scene's did not
    /// use. A viewer keeps those the placed parts drawn use, and forgets
    /// the others: a component no placed part uses any more is sent again
    /// once one does.
    pub components: BTreeMap<String, PartView>,
    /// What the part has beyond its faces, to list and show or hide (see
    /// [`crate::registry::Drawer::structure`]).
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
    pub presentation: Presentation,
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
    /// What the tool in hand shows, while no step is edited: the drag
    /// tool's part it would drag lit, and whether a press grabs it; the
    /// pick tool's picks, and what the answer about them draws.
    pub tool: Option<Presentation>,
    /// The answer of the tool in hand about the part — the pick tool's
    /// inspector's about its picks — or the answer to [`Command::Inspect`].
    pub inspection: Option<Inspection>,
}
