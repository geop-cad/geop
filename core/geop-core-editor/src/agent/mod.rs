//! An [`Editor`] driven by an agent's tools: the one tool layer of every
//! front end an agent talks to — the MCP server (`geop mcp`) and the web
//! app's hosted assistant alike.
//!
//! Each tool ([`tools`]) is a transaction on one editor through its
//! protocol ([`Command`] in, [`Update`] out): a part is started or loaded,
//! steps are added, edited and removed as their saved JSON (what a `.geop`
//! file holds), the program is run, its elements listed by the stable names
//! a step refers to them by, inspected, exported and saved. None of them
//! names an operation: what there is to build, and how, is the registry's,
//! and the agent asks for it (`list_operations`).
//!
//! The tools that read or write files by path ([`AgentTool::files`]) need
//! the [`Files`] of the front end they run in; one with none — the browser,
//! whose downloads are the user's to start — does not offer them.
//!
//! A [`Session`] keeps what a front end keeps of the editor's answers — the
//! last scene, the program as last shown — whoever sent the command, so
//! that a front end driving the editor itself (the web app's own clicks)
//! and an agent's tools can share it.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    element::PickedElement,
    placed::Located,
    program::{Program, ProgramStep},
    values::{StateValue, StateValues},
};
use serde::Serialize;
use serde_json::{Value, json};

mod resources;

pub use resources::{AgentPrompt, AgentResource, Contents, prompts, resource_templates, resources};

use crate::{
    editing::{Pointer, Presentation, StepEditEvent, Value as DialogValue},
    editor::{Command, Editor, ProgramState, SceneState, StepState, Update},
    view::{
        PartView,
        lit::Lighting,
        picture::{Camera, picture},
    },
};

/// A tool an agent is offered: its name, what it does and the JSON schema
/// of its input.
#[derive(Clone, Debug, PartialEq)]
pub struct AgentTool {
    pub name: &'static str,
    pub description: &'static str,
    pub input_schema: Value,
    /// Whether it reads or writes the front end's files by path (see
    /// [`Files`]): offered only where there are such files — not in the
    /// browser, where a file is saved only when the user says so.
    pub files: bool,
}

impl AgentTool {
    /// As MCP's `tools/list` describes it.
    pub fn mcp(&self) -> Value {
        json!({ "name": self.name, "description": self.description, "inputSchema": self.input_schema })
    }

    /// As the Anthropic Messages API takes it (`tools`).
    pub fn anthropic(&self) -> Value {
        json!({ "name": self.name, "description": self.description, "input_schema": self.input_schema })
    }
}

/// Every tool, in the order they are offered.
pub fn tools() -> Vec<AgentTool> {
    let tool = |name, description, properties: Value, required: &[&str], files| AgentTool {
        name,
        description,
        input_schema: json!({
            "type": "object",
            "properties": properties,
            "required": required,
            "additionalProperties": false,
        }),
        files,
    };
    let string = |description: &str| json!({ "type": "string", "description": description });
    let object = |description: &str| json!({ "type": "object", "description": description });
    vec![
        tool(
            "new_part",
            "Start a new, empty part. Every part has the coordinate-system datum `origin`, whose planes a sketch is drawn on, as `add_sketch`'s `plane`: {\"datum\": \"origin:xy\"}, {\"datum\": \"origin:yz\"} or {\"datum\": \"origin:zx\"}. A point (a, b) of a sketch lies in the world at (a, b, 0) on `origin:xy` (normal +z), (0, a, b) on `origin:yz` (normal +x), and (a, 0, -b) on `origin:zx` (normal +y): there, the sketch's y runs along -z. A sketch's normal is the way an extent goes (see `add_step`). y is up in the world.",
            json!({}),
            &[],
            false,
        ),
        tool(
            "load_program",
            "Replace the part with a program: the `.geop` file at `path` (the other `.geop` files next to it are read too, for the parts it places), or the program `text` itself.",
            json!({ "path": string("A .geop file on disk."), "text": string("A program as JSON, as a .geop file holds it.") }),
            &[],
            true,
        ),
        tool(
            "list_operations",
            "Every operation a step can be: its kind (what `add_step` takes), label, group and what it does. Given a `kind`, that operation alone, with the arguments a new step starts with (`default_args`, the JSON `add_step` takes) and the form the editor shows for it (`form`: each field's key, control, label, value and, for a reference, the kinds of elements it accepts).",
            json!({ "kind": string("The operation to describe in full, e.g. `extrude`.") }),
            &[],
            false,
        ),
        tool(
            "add_step",
            "Add a step of the operation `kind` with `args` (its saved JSON, see `list_operations` with `kind`) at the end of the program, or before the step `before`. A step whose arguments do not read is refused and not added; one that reads but does not build is added, and its error says why. Answers with every step's status: an error names the step and what went wrong, and `moved` lists the parts the solver moved from where they were loaded or saved, the furthest first: mates that hold only because a part jumped are no error, so read it. An extent runs from the sketch's plane: `side1` along the normal (against it if `reversed` or negative) and `side2` the other way, a negative `side2` being that far along the normal, so that the solid starts away from the plane. Its `start` cap is at the `side2` end (the plane, with no `side2`) and its `end` cap at the `side1` end, whichever way that is in the world.",
            json!({ "kind": string("The operation, e.g. `add_sketch`, `extrude`, `fillet`."), "args": object("The step's arguments as saved in a .geop file."), "id": string("The step's id (letters, digits, `_`, `-`, `.`); chosen from the operation's label if left out. Later steps refer to what this one builds by it, e.g. `extrude(<id>)`."), "before": string("The step to add it before; the end of the program if left out.") }),
            &["kind", "args"],
            false,
        ),
        tool(
            "edit_step",
            "Change the step `id`: the keys of `args` replace those of its arguments (the others stay), so one dimension can be changed alone. The program is rebuilt from that step on.",
            json!({ "id": string("The step to change."), "args": object("The arguments to set.") }),
            &["id", "args"],
            false,
        ),
        tool(
            "delete_step",
            "Remove the step `id`. Steps that referred to what it built then fail, and say so.",
            json!({ "id": string("The step to remove.") }),
            &["id"],
            false,
        ),
        tool(
            "run",
            "Run the whole program and report every step: its summary, whether it ran, and its error with the kernel's context if it failed; and the solids the part has.",
            json!({}),
            &[],
            false,
        ),
        tool(
            "list_elements",
            "The names of the part's elements as they are now, for typed references: solids, faces (with their solid), edges (with their faces), vertices, sketches, datums and what else the part lists (threads, placed parts). A face is referred to by its name, as a sketch's plane {\"face\": ...}; an edge by its name, as in a fillet's `edges`. In a swept feature's names, `start` and `end` say which cap: `extrude(box,end)` is the end cap, `extrude(box,sketch,c4,end)` the edge of the side face swept by the curve c4 on that cap, and `extrude(box,sketch,p1,end)` the vertex over the point p1. Given `placed` (a placed part's path, `hip` or `hip/servo`), the elements of that part instead, in its own frame, with where it is placed (`frame`), its size (`extent`), its datums (the frames to mount by) and the `target` to refer to them by.",
            json!({ "placed": string("The placed part to list, e.g. `hip/servo`.") }),
            &[],
            false,
        ),
        tool(
            "inspect",
            "Ask about the part: `mass_properties` (volume, area and centroid of every solid; its mass and inertia if its part has a material, null if not), `interference` (which solids overlap), or `measure` with `elements` (a point, a length, an area, a radius; the distance and angle between two). On an assembly the first two can take minutes, and calls wait their turn behind them: narrow them with `options` {\"parts\": [\"left_leg\", \"hip/servo\"]}, the solids of those placed parts and what is placed in them (`interference`: each against everything, or, with \"only_among\": true, only against each other), and give `limit_seconds`: past it, the answer is what was found so far, with `stopped` saying how far it got. A cancelled call stops likewise, at the next pair or solid.",
            json!({ "inspector": { "type": "string", "enum": ["mass_properties", "interference", "measure"] }, "elements": { "type": "array", "description": "For `measure`: one or two elements, each {\"kind\": \"vertex\"|\"edge\"|\"face\"|\"datum\", \"name\": ...}.", "items": { "type": "object", "properties": { "kind": string("vertex, edge, face or datum"), "name": string("Its name, from list_elements") }, "required": ["kind", "name"] } }, "options": object("The inspector's own options: `parts`, `only_among`."), "limit_seconds": { "type": "number", "description": "How long it may take, where the front end keeps time (`geop mcp`; not in the browser)." } }),
            &["inspector"],
            false,
        ),
        tool(
            "export",
            "Write what the program builds to `path`, in the format its extension names unless `format` says otherwise: stl, step, svg or dxf (drawings and flat patterns), csv (bill of materials), zip (URDF).",
            json!({ "path": string("Where to write the file."), "format": string("The format, if not the path's extension."), "options": object("The exporter's own options, e.g. {\"quality\": 8} for STL."), "limit_seconds": { "type": "number", "description": "How long an STL of placed parts may take: past it, the export fails and nothing is written." } }),
            &["path"],
            true,
        ),
        tool(
            "save",
            "Write the program to `path` as a .geop file, and make that the file it is edited in.",
            json!({ "path": string("Where to write the program.") }),
            &["path"],
            true,
        ),
        tool(
            "look",
            "A picture of the part as the editor draws it — the placed parts, what is lit, and what the step open draws over it — as a PNG, seen orthographically from the view `view` (`iso`, `front` (from +z), `back`, `right` (from +x), `left`, `top` (from +y), `bottom`) or along `look`, with `up` up; with neither, as last seen, or, while a step works in a plane (a sketch's), facing it. y is up in the world. Framed to all of it. Its camera is what `step_event`'s pixels are in: answers with it, and where the world's origin is in it.",
            json!({ "view": { "type": "string", "enum": VIEWS.map(|(name, ..)| name) }, "look": { "type": "array", "items": { "type": "number" }, "description": "The direction to look along, [x, y, z]." }, "up": { "type": "array", "items": { "type": "number" }, "description": "Which way is up in the picture, [x, y, z]; y, if not given." }, "width": { "type": "integer", "description": "In pixels; 800 if not given." }, "height": { "type": "integer", "description": "In pixels; 600 if not given." } }),
            &[],
            false,
        ),
        tool(
            "undo",
            "Go back one edit of the program — while a step is open, one edit of that step — as the editor's Ctrl+Z does.",
            json!({}),
            &[],
            false,
        ),
        tool(
            "redo",
            "Make the edit `undo` went back from again.",
            json!({}),
            &[],
            false,
        ),
        tool(
            "open_step",
            "Open a step to edit it the way a user does in the editor — the step `id`, or a new step of the operation `kind` where new steps go — and keep it open across calls, for `step_event`, until `commit_step` or `cancel_step`. Given `placed` (a placed part's path, `neck` or `neck/head`), it is that placed part's step, or a new step of it, edited where it is placed. Answers with the step as the editor shows it: its form (each field's key, label and value), what it still needs picked, what a click picks now, a value asked for in place, and its error. While a step is open, the tools that change the program are refused.",
            json!({ "id": string("The step to open."), "kind": string("The operation of a new step, e.g. `add_sketch`."), "placed": string("The placed part whose step it is.") }),
            &[],
            false,
        ),
        tool(
            "step_event",
            "Do something to the open step, as a user does in the editor — or, with no step open, with the tool in hand (with none taken, the drag tool: dragging a placed part moves it as far as its mates let it). `event.type` is one of: `dialog` (set the form's field `key` to `value`), `click` (at `pointer`; `double`, `shift`), `hover` (move the pointer to `pointer` with no button held), `drag` (press at `from`, move to `to` in `steps` moves, release), `select_box` (a box from the pixel `from` to the pixel `to`: left to right selects what lies wholly in it, right to left what it touches; `add` keeps the selection), `key` (a key as a browser names it: `l` for the sketch's line tool, `Escape`, `Enter`, `Delete`) or `leave` (the pointer leaves the view). A pointer is `{\"pixel\": [x, y]}` in the last picture `look` took, or `{\"at\": [x, y, z]}`, a point of the world seen along that picture's view (while a step works in a plane, a sketch's, the view faces it head on), with `cycle` to mean the n-th of what lies under it, front to back. Answers with the step or the tool as it then is.",
            json!({ "event": { "type": "object", "description": "The event: {\"type\": \"dialog\", \"key\": ..., \"value\": ...}, {\"type\": \"click\", \"pointer\": {...}}, {\"type\": \"drag\", \"from\": {...}, \"to\": {...}}, {\"type\": \"key\", \"key\": \"Escape\"}, ...", "properties": { "type": { "type": "string", "enum": ["dialog", "click", "hover", "drag", "select_box", "key", "leave"] } }, "required": ["type"] } }),
            &["event"],
            false,
        ),
        tool(
            "commit_step",
            "Put the open step into the program, as the dialog's OK does: refused, and the step kept open, unless it builds.",
            json!({}),
            &[],
            false,
        ),
        tool(
            "cancel_step",
            "Close the open step and leave the program as it was.",
            json!({}),
            &[],
            false,
        ),
        tool(
            "take_tool",
            "Take the tool `key` in hand, as the editor's toolbar does, or put the one in hand down (no `key`). With no step open, `step_event` is the tool's: `pick:<inspector>` (e.g. `pick:measure`) picks elements to ask that inspector about, its answer coming with every event. `run` lists the tools there are.",
            json!({ "key": string("The tool, e.g. `pick:measure`; none puts the tool in hand down.") }),
            &[],
            false,
        ),
        tool(
            "panel",
            "What the panel `key` shows of the program — an assembly's joints and how free each placed part is, its configurations — or, given `payload`, something done in it first, as the panel reads it: `{\"command\": ..., ...}`. A payload the panel does not read is refused with what it expects. `run` lists the panels there are.",
            json!({ "key": string("The panel."), "payload": { "description": "What was done in it, as the panel reads it." } }),
            &["key"],
            false,
        ),
    ]
}

/// The views `look` names, as the editor's view cube does: where the eye
/// is from what it looks at, and which way is up. Isometric is the
/// editor's first view.
const VIEWS: [(&str, [f64; 3], [f64; 3]); 7] = [
    ("iso", [66.0, 44.0, 88.0], [0.0, 1.0, 0.0]),
    ("front", [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
    ("back", [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
    ("right", [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    ("left", [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    ("top", [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]),
    ("bottom", [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
];

/// How far a part has to have moved, in length or radians, for the status
/// to say so: well above what a solve leaves of a part that stayed.
const MOVED: f64 = 1e-6;

/// The size of a picture, unless `look` is told otherwise.
const PICTURE: (u32, u32) = (800, 600);

/// The tools that change the program, refused while a step is open: what
/// they would do is not the open step's, and would leave it behind.
const CHANGE_THE_PROGRAM: [&str; 6] = [
    "new_part",
    "load_program",
    "add_step",
    "edit_step",
    "delete_step",
    "open_step",
];

/// The files of the front end a session runs in, which the tools that
/// take a path read and write ([`AgentTool::files`]).
pub trait Files {
    /// The text of the file `path`.
    fn read(&mut self, path: &str) -> GeopResult<String>;
    /// The program files in the folder of `path` — `path` among them — by
    /// path, with their text: the files the program at `path` places parts
    /// from.
    fn programs_beside(&mut self, path: &str) -> BTreeMap<String, Option<String>>;
    /// Writes `bytes` to the file `path`.
    fn write(&mut self, path: &str, bytes: &[u8]) -> GeopResult<()>;
    /// Today, `YYYY-MM-DD`: what an export is dated.
    fn today(&self) -> String;
}

/// What a tool answered: its result as text — pretty JSON, or why it
/// failed — and every update the editor sent while it ran, in order, for a
/// front end that shows the editor to show.
#[derive(Clone, Debug, Serialize)]
pub struct ToolAnswer {
    pub text: String,
    pub is_error: bool,
    pub updates: Vec<Update>,
    /// The pictures it answered with too (`look`).
    pub images: Vec<Image>,
}

/// A picture a tool answered with.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Image {
    pub media_type: &'static str,
    /// Its bytes, as base64.
    #[serde(serialize_with = "as_base64")]
    pub data: Vec<u8>,
}

impl Image {
    /// Its bytes as base64, as JSON carries them.
    pub fn base64(&self) -> String {
        use base64::Engine;
        base64::engine::general_purpose::STANDARD.encode(&self.data)
    }
}

fn as_base64<S: serde::Serializer>(data: &[u8], serializer: S) -> Result<S::Ok, S::Error> {
    use base64::Engine;
    serializer.serialize_str(&base64::engine::general_purpose::STANDARD.encode(data))
}

/// One editor, driven by tools and by a front end alike, and what a front
/// end keeps of its answers.
pub struct Session {
    editor: Editor,
    /// The part as last drawn: the names `list_elements` answers with.
    scene: Option<SceneState>,
    /// The program as last shown: its steps' status, the operations.
    shown: Option<ProgramState>,
    /// The step being edited, as last shown.
    step: Option<StepState>,
    /// What the tool in hand shows, while no step is edited.
    tool: Option<Presentation>,
    /// What is lit, as last sent.
    lit: Option<Lighting>,
    /// The program as last known to be whole, to start a fresh editor from
    /// after a panic ([`Self::restart`]).
    program: Program,
    /// The file it is edited in, once told.
    path: Option<String>,
    /// The updates of the tool running, in order.
    sent: Vec<Update>,
    /// The pictures of the tool running.
    images: Vec<Image>,
    /// What the last picture saw, which pixels are in: none until `look`,
    /// or since the step open changed.
    camera: Option<Camera>,
    /// Whether the step open has been told which way is up in `camera`.
    told: bool,
    /// The program's state as it was loaded or saved, which the status
    /// says what moved from.
    saved: StateValues,
}

impl Session {
    /// A session of `editor`, shown whole.
    pub fn new(editor: Editor) -> Self {
        let mut session = Self {
            editor,
            scene: None,
            shown: None,
            step: None,
            tool: None,
            lit: None,
            program: Program::new(),
            path: None,
            sent: Vec::new(),
            images: Vec::new(),
            camera: None,
            told: false,
            saved: StateValues::new(),
        };
        session.handle(Command::Show);
        // That scene was the session's to know, not the viewer's to have:
        // the front end's first command gets all of it.
        session.editor.forget_viewer();
        session
    }

    /// The editor.
    pub fn editor(&self) -> &Editor {
        &self.editor
    }

    /// Applies `command`, as a front end sends it, keeping what the session
    /// keeps of the answer.
    pub fn handle(&mut self, command: Command) -> Update {
        let path = match &command {
            Command::Load {
                path: Some(path), ..
            } => Some(path.clone()),
            _ => None,
        };
        let update = self.editor.handle(command);
        if let Some(scene) = &update.scene {
            // What a scene does not send again is as the last sent it.
            let mut scene = scene.clone();
            if let Some(last) = self.scene.take() {
                scene.hidden = scene.hidden.or(last.hidden);
                scene.structure = scene.structure.or(last.structure);
                // The placed parts not sent again are as last sent, in
                // the colours they were given then.
                if !scene.all {
                    let sent: Vec<String> =
                        scene.instances.iter().map(|i| i.name.clone()).collect();
                    let kept = (last.instances.into_iter())
                        .filter(|i| !sent.contains(&i.name) && !scene.removed.contains(&i.name));
                    scene.instances.extend(kept);
                }
            }
            self.scene = Some(scene);
        }
        if let Some(program) = &update.program {
            self.shown = Some(program.clone());
        }
        // A step opened or closed, or one starting to work in a plane, is
        // seen afresh: facing that plane.
        let seen = |step: &Option<StepState>| {
            (step.as_ref()).map(|s| (s.kind, s.id.clone(), s.presentation.focus.is_some()))
        };
        if seen(&self.step) != seen(&update.step) {
            self.camera = None;
            self.told = false;
        }
        self.step = update.step.clone();
        self.tool = update.tool.clone();
        if let Some(lit) = &update.lit {
            self.lit = Some(lit.clone());
        }
        if update.error.is_none() {
            self.program = self.editor.program().clone();
            if path.is_some() {
                self.path = path;
            }
        }
        update
    }

    /// [`Self::handle`] for a command and an update as JSON.
    pub fn handle_json(&mut self, command: &str) -> Result<String, String> {
        let command: Command =
            serde_json::from_str(command).map_err(|e| format!("reading command: {e}"))?;
        serde_json::to_string(&self.handle(command)).map_err(|e| format!("writing update: {e}"))
    }

    /// Replaces an editor left in an unknown state — a kernel that panicked
    /// — with `editor`, on the program as it was last whole.
    pub fn restart(&mut self, editor: Editor) {
        self.editor = editor;
        self.scene = None;
        self.shown = None;
        self.lit = None;
        let program = self.program.clone();
        let path = self.path.clone();
        self.handle(Command::Load { program, path });
    }

    /// Calls the tool `name` with `input`, in a front end with `files`, if
    /// it has any: its result as pretty JSON, or why it failed.
    pub fn call(&mut self, name: &str, input: &Value, files: Option<&mut dyn Files>) -> ToolAnswer {
        self.sent.clear();
        self.images.clear();
        let (text, is_error) = match self.tool(name, input, files) {
            Ok(result) => (
                serde_json::to_string_pretty(&result).unwrap_or_else(|_| result.to_string()),
                false,
            ),
            Err(e) => (format!("{name} failed: {e}"), true),
        };
        ToolAnswer {
            text,
            is_error,
            updates: std::mem::take(&mut self.sent),
            images: std::mem::take(&mut self.images),
        }
    }

    /// Sends `command` for a tool: the update, or its error as an `Err`.
    fn send(&mut self, command: Command) -> GeopResult<Update> {
        let update = self.handle(command);
        self.sent.push(update.clone());
        match update.error {
            Some(error) => Err(GeopError::new(error)),
            None => Ok(update),
        }
    }

    /// The tool `name` applied to `args`: what it answers.
    fn tool(
        &mut self,
        name: &str,
        args: &Value,
        files: Option<&mut dyn Files>,
    ) -> GeopResult<Value> {
        let text = |key: &str| -> GeopResult<String> {
            args.get(key)
                .and_then(Value::as_str)
                .map(str::to_string)
                .ok_or_else(|| GeopError::new(format!("`{key}` (a string) is required")))
        };
        let offered = tools().into_iter().find(|tool| tool.name == name);
        let files = match (offered, files) {
            (None, _) => return Err(GeopError::new(format!("there is no tool `{name}`"))),
            (Some(tool), None) if tool.files => {
                return Err(GeopError::new(format!(
                    "`{name}` is not offered here: it reads or writes files by path, and there are none"
                )));
            }
            (_, files) => files,
        };
        if CHANGE_THE_PROGRAM.contains(&name)
            && let Some(step) = &self.step
        {
            return Err(GeopError::new(format!(
                "the step {} ({}) is open: commit_step or cancel_step first",
                step.id.as_deref().unwrap_or("being added"),
                step.kind
            )));
        }
        match (name, files) {
            ("new_part", _) => {
                self.saved = StateValues::new();
                self.load(Program::new())?;
                self.status()
            }
            ("load_program", Some(files)) => {
                if let Some(path) = args.get("path").and_then(Value::as_str) {
                    self.load_file(path, files)
                } else {
                    let program = Program::from_json(&text("text")?)?;
                    self.saved = program.state.clone();
                    self.load(program)?;
                    self.status()
                }
            }
            ("list_operations", _) => {
                self.list_operations(args.get("kind").and_then(Value::as_str))
            }
            ("add_step", _) => {
                let kind = text("kind")?;
                let step_args = (args.get("args").cloned())
                    .ok_or_else(|| GeopError::new("`args` (the step's arguments) is required"))?;
                let mut program = self.editor.program().clone();
                let id = match args.get("id").and_then(Value::as_str) {
                    Some(id) => id.to_string(),
                    None => program.fresh_id(&kind),
                };
                let at = match args.get("before").and_then(Value::as_str) {
                    Some(before) => program.index_of(before)?,
                    None => program.steps.len(),
                };
                let step = ProgramStep {
                    id: id.clone(),
                    operation: kind,
                    args: step_args,
                    active: None,
                };
                program.steps.insert(at, step);
                self.load(program)
                    .map_err(|e| e.with_context(format!("add_step(id={id})")))?;
                self.status()
            }
            ("edit_step", _) => {
                let id = text("id")?;
                let set = (args.get("args").cloned())
                    .ok_or_else(|| GeopError::new("`args` (the arguments to set) is required"))?;
                let mut program = self.editor.program().clone();
                let step = program.step_mut(&id)?;
                match (&mut step.args, set) {
                    (Value::Object(args), Value::Object(set)) => args.extend(set),
                    (args, set) => *args = set,
                }
                self.load(program)
                    .map_err(|e| e.with_context(format!("edit_step(id={id})")))?;
                self.status()
            }
            ("delete_step", _) => {
                let id = text("id")?;
                self.send(Command::Remove { id })?;
                self.status()
            }
            ("run", _) => {
                self.send(Command::Seek { marker: None })?;
                self.status()
            }
            ("list_elements", _) => match args.get("placed").and_then(Value::as_str) {
                Some(placed) => self.list_placed_elements(placed),
                None => self.list_elements(),
            },
            ("inspect", _) => {
                let inspector = text("inspector")?;
                let elements = args.get("elements").cloned().unwrap_or(json!([]));
                let options = args.get("options").cloned().unwrap_or(Value::Null);
                self.inspect(&inspector, &elements, options)
            }
            ("export", Some(files)) => {
                let path = text("path")?;
                let format = match args.get("format").and_then(Value::as_str) {
                    Some(format) => format.to_string(),
                    None => (std::path::Path::new(&path).extension())
                        .and_then(|e| e.to_str())
                        .map(str::to_lowercase)
                        .ok_or_else(|| GeopError::new(format!("{path} names no format")))?,
                };
                let options = args.get("options").cloned().unwrap_or(json!({}));
                self.export(&path, &format, options, files)
            }
            ("save", Some(files)) => {
                let path = text("path")?;
                let program = self.editor.program().clone();
                let json = program.to_json()?;
                files.write(&path, json.as_bytes())?;
                self.saved = program.state.clone();
                if self.path.as_deref() != Some(path.as_str()) {
                    self.send(Command::Load {
                        program,
                        path: Some(path.clone()),
                    })?;
                }
                Ok(json!({ "saved": path, "bytes": json.len() }))
            }
            ("look", _) => {
                let (png, camera) = self.picture(Some(args))?;
                self.images.push(Image {
                    media_type: "image/png",
                    data: png,
                });
                Ok(camera)
            }
            ("undo", _) => {
                self.send(Command::Undo)?;
                self.editing()
            }
            ("redo", _) => {
                self.send(Command::Redo)?;
                self.editing()
            }
            ("open_step", _) => {
                let placed = args
                    .get("placed")
                    .and_then(Value::as_str)
                    .map(str::to_string);
                let command = match (args.get("id").and_then(Value::as_str), placed) {
                    (Some(id), placed) => Command::Open {
                        id: id.to_string(),
                        placed,
                    },
                    (None, None) => Command::New {
                        kind: text("kind")?,
                    },
                    (None, Some(placed)) => Command::NewIn {
                        kind: text("kind")?,
                        placed,
                    },
                };
                self.send(command)?;
                self.editing()
            }
            ("step_event", _) => {
                let event =
                    (args.get("event")).ok_or_else(|| GeopError::new("`event` is required"))?;
                for event in self.events(event)? {
                    self.send(Command::Event { event })?;
                }
                self.editing()
            }
            ("commit_step", _) => {
                self.open()?;
                self.send(Command::Commit)?;
                self.editing()
            }
            ("cancel_step", _) => {
                self.open()?;
                self.send(Command::Cancel)?;
                self.editing()
            }
            ("take_tool", _) => {
                let key = args.get("key").and_then(Value::as_str).map(str::to_string);
                self.send(Command::Tool { key })?;
                self.editing()
            }
            ("panel", _) => {
                let key = text("key")?;
                if let Some(payload) = args.get("payload") {
                    self.send(Command::Panel {
                        key: key.clone(),
                        payload: payload.clone(),
                    })?;
                }
                let panels = &self.shown()?.panels;
                let shown = panels.get(&key).ok_or_else(|| {
                    let keys: Vec<&str> = panels.keys().map(String::as_str).collect();
                    GeopError::new(format!(
                        "there is no panel `{key}`; there are {}",
                        keys.join(", ")
                    ))
                })?;
                Ok(json!({ "panel": key, "shows": shown, "status": self.status()? }))
            }
            _ => Err(GeopError::new(format!("there is no tool `{name}`"))),
        }
    }

    /// The presentation shown: the open step's, or the tool's in hand.
    fn presentation(&self) -> Option<&Presentation> {
        (self.step.as_ref().map(|s| &s.presentation)).or(self.tool.as_ref())
    }

    /// The camera the next picture sees by — `view` (the tool `look`'s
    /// input), if given — kept for the pixels of the events after it.
    fn camera(&mut self, view: Option<&Value>) -> GeopResult<Camera> {
        let vector = |key: &str| -> GeopResult<Option<Vector3>> {
            let Some(value) = view.and_then(|v| v.get(key)) else {
                return Ok(None);
            };
            let read: Option<Vec<f64>> =
                (value.as_array()).map(|a| a.iter().filter_map(Value::as_f64).collect());
            match read.as_deref() {
                Some(&[x, y, z]) => Ok(Some(Vector3::from_array([x, y, z].map(Scalar::from_f64)))),
                _ => Err(GeopError::new(format!("`{key}` is [x, y, z]: {value}"))),
            }
        };
        let size = |key: &str, default: u32| -> GeopResult<u32> {
            match view.and_then(|v| v.get(key)) {
                None => Ok(default),
                Some(value) => (value.as_u64())
                    .filter(|&n| (16..=4096).contains(&n))
                    .map(|n| n as u32)
                    .ok_or_else(|| {
                        GeopError::new(format!("`{key}` is 16 to 4096 pixels: {value}"))
                    }),
            }
        };
        let named = view.and_then(|v| v.get("view")).map(|name| {
            (VIEWS.iter())
                .find(|(n, ..)| Some(*n) == name.as_str())
                .ok_or_else(|| GeopError::new(format!("there is no view {name}")))
        });
        let (look, up) = match (named.transpose()?, vector("look")?) {
            (Some(_), Some(_)) => return Err(GeopError::new("`view` or `look`, not both")),
            (Some((_, eye, up)), None) => (
                Vector3::from_array(eye.map(|x| Scalar::from_f64(-x))),
                Vector3::from_array(up.map(Scalar::from_f64)),
            ),
            (None, Some(look)) => (
                look,
                Vector3::from_array([0.0, 1.0, 0.0].map(Scalar::from_f64)),
            ),
            (None, None) => {
                let resized =
                    view.is_some_and(|v| v.get("width").is_some() || v.get("height").is_some());
                if let Some(camera) = self.camera.clone().filter(|_| !resized) {
                    return Ok(camera);
                }
                match self.presentation().and_then(|p| p.focus.as_ref()) {
                    // Head on, from the side its normal points to: as the
                    // editor turns the view to a plane worked in.
                    Some(focus) => (focus.w().neg(), *focus.v()),
                    None => {
                        let (_, eye, up) = VIEWS[0];
                        (
                            Vector3::from_array(eye.map(|x| Scalar::from_f64(-x))),
                            Vector3::from_array(up.map(Scalar::from_f64)),
                        )
                    }
                }
            }
        };
        let up = vector("up")?.unwrap_or(up);
        let (width, height) = (size("width", PICTURE.0)?, size("height", PICTURE.1)?);
        let camera = Camera::fitting(self.scene()?, look, up, width, height)?;
        // A step drawn on a sheet of its own is framed to it.
        let camera = match self.presentation().and_then(|p| p.sheet) {
            Some(sheet) => Camera {
                center: sheet.center,
                span: sheet.size.to_f64(),
                ..camera
            },
            None => camera,
        };
        if self.camera.as_ref() != Some(&camera) {
            self.told = false;
        }
        self.camera = Some(camera.clone());
        Ok(camera)
    }

    /// The picture `look` takes, told `view` (its input): a PNG, and the
    /// camera it was seen by.
    fn picture(&mut self, view: Option<&Value>) -> GeopResult<(Vec<u8>, Value)> {
        let camera = self.camera(view)?;
        self.tell_up()?;
        let scene = self.scene()?;
        let visuals = self.presentation().map(|p| &p.visuals[..]).unwrap_or(&[]);
        let png = picture(scene, self.lit.as_ref(), visuals, &camera)?;
        let vector = |v: &Vector3| v.to_array().map(|x| x.to_f64());
        let origin = camera.pixel(&Vector3::from_array([0.0; 3].map(Scalar::from_f64)));
        Ok((
            png,
            json!({
                "camera": {
                    "look": vector(&camera.look),
                    "up": vector(&camera.up),
                    "center": vector(&camera.center),
                    "span": camera.span,
                    "width": camera.width,
                    "height": camera.height,
                },
                "origin_at_pixel": origin,
                "pixels": "x from the left, y from the top; the picture shows `span` of the world from top to bottom",
            }),
        ))
    }

    /// Tells the step open which way is up in the camera, as the editor's
    /// viewer does when its view comes to rest: a sketch's horizontal is
    /// the picture's.
    fn tell_up(&mut self) -> GeopResult<()> {
        if self.told || self.step.is_none() {
            return Ok(());
        }
        let Some(camera) = &self.camera else {
            return Ok(());
        };
        let up = camera.up;
        self.send(Command::Event {
            event: StepEditEvent::View { up },
        })?;
        self.told = true;
        Ok(())
    }

    /// The pointer an agent describes (see the tool `step_event`), in
    /// `camera`.
    fn pointer(camera: &Camera, spec: &Value) -> GeopResult<Pointer> {
        let numbers = |key: &str| -> Option<Vec<f64>> {
            let value = spec.get(key)?.as_array()?;
            value.iter().map(Value::as_f64).collect()
        };
        let pointer = match (numbers("pixel").as_deref(), numbers("at").as_deref()) {
            (Some(&[x, y]), None) => camera.pointer(x, y)?,
            (None, Some(&[x, y, z])) => {
                camera.through(Vector3::from_array([x, y, z].map(Scalar::from_f64)))?
            }
            _ => {
                return Err(GeopError::new(format!(
                    "a pointer is {{\"pixel\": [x, y]}} or {{\"at\": [x, y, z]}}: {spec}"
                )));
            }
        };
        let cycle = spec.get("cycle").and_then(Value::as_u64).unwrap_or(0);
        Ok(pointer.cycled(cycle as usize))
    }

    /// Where `spec` (a pointer) is in `camera`'s picture.
    fn pixel_of(camera: &Camera, spec: &Value) -> GeopResult<[f64; 2]> {
        let pointer = Self::pointer(camera, spec)?;
        Ok(camera.pixel(pointer.ray.origin()))
    }

    /// The editor's events of a pointer's event an agent describes: the
    /// pointer moved there first, as a user's is, so that what it is over
    /// is lit and grabbed as the editor would light and grab it.
    fn pointer_events(&mut self, kind: &str, event: &Value) -> GeopResult<Vec<StepEditEvent>> {
        let camera = self.camera(None)?;
        self.tell_up()?;
        let field = |key: &str| {
            (event.get(key))
                .ok_or_else(|| GeopError::new(format!("a `{kind}` event needs `{key}`")))
        };
        let shift = event.get("shift").and_then(Value::as_bool).unwrap_or(false);
        let hover = |pointer: Pointer| StepEditEvent::Hover { pointer, shift };
        Ok(match kind {
            "hover" => vec![hover(Self::pointer(&camera, field("pointer")?)?)],
            "click" => {
                let pointer = Self::pointer(&camera, field("pointer")?)?;
                let double = event
                    .get("double")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                vec![
                    hover(pointer),
                    StepEditEvent::Click {
                        pointer,
                        double: false,
                        shift,
                    },
                ]
                .into_iter()
                .chain(double.then_some(StepEditEvent::Click {
                    pointer,
                    double: true,
                    shift,
                }))
                .collect()
            }
            "drag" => {
                let (from, to) = (field("from")?, field("to")?);
                let steps = event
                    .get("steps")
                    .and_then(Value::as_u64)
                    .unwrap_or(8)
                    .clamp(1, 100);
                let start = Self::pointer(&camera, from)?;
                let (a, b) = (Self::pixel_of(&camera, from)?, Self::pixel_of(&camera, to)?);
                let mut events = vec![hover(start)];
                for i in 1..=steps {
                    let done = i == steps;
                    let to = match done {
                        true => Self::pointer(&camera, to)?,
                        false => {
                            let t = i as f64 / steps as f64;
                            camera.pointer(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)?
                        }
                    };
                    events.push(StepEditEvent::Drag {
                        from: start,
                        to,
                        done,
                        shift,
                    });
                }
                events
            }
            "select_box" => {
                let corner = |key: &str| -> GeopResult<[f64; 2]> {
                    let value = field(key)?;
                    match (value.as_array())
                        .map(|a| a.iter().filter_map(Value::as_f64).collect::<Vec<_>>())
                        .as_deref()
                    {
                        Some(&[x, y]) => Ok([x, y]),
                        _ => Err(GeopError::new(format!(
                            "a select_box's `{key}` is a pixel [x, y]: {value}"
                        ))),
                    }
                };
                let ([x0, y0], [x1, y1]) = (corner("from")?, corner("to")?);
                let corners =
                    [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(|[x, y]| camera.pointer(x, y));
                let [a, b, c, d] = corners;
                vec![StepEditEvent::SelectBox {
                    corners: Box::new([a?, b?, c?, d?]),
                    crossing: x1 < x0,
                    add: event.get("add").and_then(Value::as_bool).unwrap_or(false),
                }]
            }
            _ => {
                return Err(GeopError::new(format!(
                    "there is no event `{kind}`: there are dialog, click, hover, drag, select_box, key and leave"
                )));
            }
        })
    }

    /// Refuses with no step open.
    fn open(&self) -> GeopResult<&StepState> {
        (self.step.as_ref()).ok_or_else(|| GeopError::new("no step is open: open_step first"))
    }

    /// The step open, as the editor shows it — or, with none, the tool in
    /// hand — and every step's status.
    fn editing(&self) -> GeopResult<Value> {
        let status = self.status()?;
        let shown = |presentation: &Presentation| -> GeopResult<Value> {
            let mut shown = serde_json::to_value(presentation)
                .map_err(|e| GeopError::new(format!("showing the step: {e}")))?;
            // What it draws is for a picture to show (`look`), and where
            // its dialog goes, for a screen.
            if let Value::Object(shown) = &mut shown {
                shown.remove("visuals");
                shown.remove("layout");
            }
            Ok(shown)
        };
        let Some(step) = &self.step else {
            let program = self.shown()?;
            return Ok(json!({
                "open": null,
                "tool": program.tool,
                "tool_shows": self.tool.as_ref().map(shown).transpose()?,
                "can_undo": program.can_undo,
                "can_redo": program.can_redo,
                "status": status,
            }));
        };
        Ok(json!({
            "open": {
                "id": step.id,
                "kind": step.kind,
                "label": step.label,
                "doc": step.doc,
                "missing": step.missing,
                "error": step.error,
                "can_undo": step.can_undo,
                "can_redo": step.can_redo,
                "shows": shown(&step.presentation)?,
            },
            "status": status,
        }))
    }

    /// The editor's events for the event an agent describes (see the tool
    /// `step_event`).
    fn events(&mut self, event: &Value) -> GeopResult<Vec<StepEditEvent>> {
        let kind = (event.get("type").and_then(Value::as_str))
            .ok_or_else(|| GeopError::new("an event needs `type`"))?;
        let field = |key: &str| {
            (event.get(key))
                .ok_or_else(|| GeopError::new(format!("a `{kind}` event needs `{key}`")))
        };
        Ok(match kind {
            "dialog" => {
                let key = (field("key")?.as_str())
                    .ok_or_else(|| GeopError::new("a dialog event's `key` is a string"))?;
                // A plain number or yes/no is what it looks like; anything
                // else is said as the editor's protocol says it.
                let value = match field("value")? {
                    Value::Number(n) => (n.as_f64().map(DialogValue::Number))
                        .ok_or_else(|| GeopError::new(format!("{n} is no number")))?,
                    Value::Bool(b) => DialogValue::Bool(*b),
                    value => serde_json::from_value(value.clone()).map_err(|e| {
                        GeopError::new(format!(
                            "a dialog value is a number, true or false, or {{\"type\": \"text\"|\"choice\"|\"press\"|..., \"value\": ...}}: {e}"
                        ))
                    })?,
                };
                vec![StepEditEvent::Dialog {
                    key: key.to_string(),
                    value,
                }]
            }
            "key" => {
                let key = (field("key")?.as_str())
                    .ok_or_else(|| GeopError::new("a key event's `key` is a string"))?;
                vec![StepEditEvent::Key {
                    key: key.to_string(),
                }]
            }
            "leave" => vec![StepEditEvent::Leave],
            _ => self.pointer_events(kind, event)?,
        })
    }

    /// Replaces the program with `program`, in the file it is edited in.
    fn load(&mut self, program: Program) -> GeopResult<Update> {
        self.send(Command::Load {
            program,
            path: None,
        })
    }

    /// Loads the program file `path`, with the other program files next to
    /// it, which the parts it places are read from.
    fn load_file(&mut self, path: &str, files: &mut dyn Files) -> GeopResult<Value> {
        let program = Program::from_json(&files.read(path)?)?;
        self.saved = program.state.clone();
        let beside = files.programs_beside(path);
        if !beside.is_empty() {
            self.send(Command::Files { files: beside })?;
        }
        self.send(Command::Load {
            program,
            path: Some(path.to_string()),
        })?;
        self.status()
    }

    /// The program as last shown.
    fn shown(&self) -> GeopResult<&ProgramState> {
        (self.shown.as_ref()).ok_or_else(|| GeopError::new("the editor showed no program"))
    }

    /// Every step, as the program's list shows it, and the part's solids.
    fn status(&self) -> GeopResult<Value> {
        let program = self.shown()?;
        let steps: Vec<Value> = (program.steps.iter())
            .map(|step| {
                json!({
                    "id": step.id,
                    "kind": step.kind,
                    "summary": step.summary,
                    "runs": step.runs,
                    "error": step.error,
                })
            })
            .collect();
        let failed = program.steps.iter().filter(|s| s.error.is_some()).count();
        let solids = self.scene.as_ref().map(|s| s.part.solids.clone());
        let tools: Vec<&str> = program.tools.iter().map(|t| t.key).collect();
        let panels: Vec<&str> = program.panels.keys().map(String::as_str).collect();
        Ok(json!({
            "steps": steps,
            "failed": failed,
            "solids": solids,
            "path": program.path,
            "open_step": self.step.as_ref().map(|s| json!({ "id": s.id, "kind": s.kind })),
            "moved": self.moved(),
            "tool": program.tool,
            "tools": tools,
            "panels": panels,
            "can_undo": program.can_undo,
            "can_redo": program.can_redo,
        }))
    }

    /// The poses in the program's state that are not where they were
    /// loaded or saved, the furthest first: how far each went and turned.
    /// Lengths are read to [`MOVED`], which is only what is worth saying:
    /// it decides nothing else.
    fn moved(&self) -> Vec<Value> {
        let mut moved: Vec<(f64, f64, &str)> = (self.editor.program().state.iter())
            .filter_map(|(name, now)| {
                let (StateValue::Pose(now), Some(StateValue::Pose(saved))) =
                    (now, self.saved.get(name))
                else {
                    return None;
                };
                let relative = saved.inverse().compose(now);
                let by = relative.position().to_array().map(|x| x.to_f64());
                let by = by.iter().map(|x| x * x).sum::<f64>().sqrt();
                let turn = relative.rotation();
                let across = [turn.x, turn.y, turn.z].map(|x| x.to_f64());
                let across = across.iter().map(|x| x * x).sum::<f64>().sqrt();
                let turned = 2.0 * across.atan2(turn.w.to_f64().abs());
                (by > MOVED || turned > MOVED).then_some((by, turned.to_degrees(), name.as_str()))
            })
            .collect();
        moved.sort_by(|a, b| b.0.total_cmp(&a.0).then(b.1.total_cmp(&a.1)));
        (moved.into_iter())
            .map(|(by, turned, name)| json!({ "state": name, "moved": by, "turned_degrees": turned }))
            .collect()
    }

    /// Every operation, by kind, label, group and what it does; or the one
    /// of `kind` alone, with the arguments a new step of it starts with and
    /// the form it is edited through.
    fn list_operations(&mut self, kind: Option<&str>) -> GeopResult<Value> {
        let operations = self.shown()?.operations.clone();
        let Some(kind) = kind else {
            let operations: Vec<Value> = (operations.iter())
                .map(|listed| {
                    json!({
                        "kind": listed.kind.to_string(),
                        "label": listed.info.label,
                        "group": listed.info.group,
                        "doc": listed.info.doc,
                        "experimental": listed.info.experimental,
                    })
                })
                .collect();
            return Ok(json!({ "operations": operations }));
        };
        let listed = (operations.iter())
            .find(|listed| listed.kind == kind)
            .ok_or_else(|| {
                let kinds: Vec<String> = (operations.iter())
                    .map(|listed| listed.kind.to_string())
                    .collect();
                GeopError::new(format!(
                    "there is no operation `{kind}`; there are {}",
                    kinds.join(", ")
                ))
            })?;
        let opened = self
            .send(Command::New {
                kind: kind.to_string(),
            })
            .map_err(|e| e.with_context(format!("opening a new `{kind}`")))?;
        let step = (opened.step)
            .ok_or_else(|| GeopError::new(format!("a new `{kind}` opened no step")))?;
        let default_args = self.editor.editing().map(|s| s.args.clone());
        // Cancelling what was just opened cannot be refused.
        let _ = self.send(Command::Cancel);
        let form = serde_json::to_value(&step.presentation.dialog)
            .map_err(|e| GeopError::new(format!("the form of `{kind}`: {e}")))?;
        Ok(json!({
            "kind": kind,
            "label": listed.info.label,
            "group": listed.info.group,
            "tier": listed.info.tier,
            "doc": if step.doc.is_empty() { listed.info.doc } else { step.doc },
            "experimental": listed.info.experimental,
            "default_args": default_args,
            "missing": step.missing,
            "form": form,
        }))
    }

    /// The scene as last drawn.
    fn scene(&self) -> GeopResult<&SceneState> {
        self.scene
            .as_ref()
            .ok_or_else(|| GeopError::new("nothing has been drawn yet: start with new_part"))
    }

    fn list_elements(&self) -> GeopResult<Value> {
        let scene = self.scene()?;
        let mut listed = Self::elements_of(&scene.part);
        let structure: Vec<Value> = (scene.structure.iter().flatten())
            .map(|item| json!({ "kind": item.kind, "name": item.name }))
            .collect();
        listed.insert("structure".into(), json!(structure));
        Ok(Value::Object(listed))
    }

    /// The elements of the part placed at `placed`, in its own frame, and
    /// how to refer to them from the part it is placed in.
    fn list_placed_elements(&self, placed: &str) -> GeopResult<Value> {
        let part = &self.scene()?.part;
        let instance = (part.instances.iter())
            .find(|instance| instance.name == placed)
            .ok_or_else(|| {
                let names: Vec<&str> = part.instances.iter().map(|i| i.name.as_str()).collect();
                GeopError::new(format!(
                    "no part is placed as `{placed}`; there are {}",
                    names.join(", ")
                ))
            })?;
        let view = &instance.view;
        let vector = |v: &Vector3| v.to_array().map(|x| x.to_f64());
        let mut listed = Self::elements_of(view);
        listed.insert("placed".into(), json!(placed));
        let target = Located::in_instance(placed, &view.model).key();
        listed.insert("target".into(), json!(target));
        listed.insert(
            "frame".into(),
            json!({
                "origin": vector(instance.frame.origin()),
                "x": vector(instance.frame.u()),
                "y": vector(instance.frame.v()),
                "z": vector(instance.frame.w()),
            }),
        );
        listed.insert(
            "extent".into(),
            json!({
                "center": vector(&view.extent.center),
                "size": view.extent.size.to_f64(),
            }),
        );
        Ok(Value::Object(listed))
    }

    /// The elements of `part`, by name.
    fn elements_of(part: &PartView) -> serde_json::Map<String, Value> {
        let faces: Vec<Value> = (part.faces.iter())
            .map(|f| json!({ "name": f.name, "solid": f.solid, "feature": f.feature }))
            .collect();
        let edges: Vec<Value> = (part.edges.iter())
            .map(|e| json!({ "name": e.name, "solid": e.solid, "faces": e.faces }))
            .collect();
        let vertices: Vec<Value> = (part.vertices.iter())
            .map(|v| {
                let at = v.at.to_array().map(|c| c.to_f64());
                json!({ "name": v.name, "solid": v.solid, "at": at })
            })
            .collect();
        let sketches: Vec<&str> = part.sketches.iter().map(|s| s.name.as_str()).collect();
        let datums: Vec<Value> = (part.datums.iter())
            .map(|d| json!({ "name": d.name, "kind": d.kind }))
            .collect();
        // Every part placed, not only those the last scene sent anew.
        let placed: Vec<&str> = part.instances.iter().map(|i| i.name.as_str()).collect();
        [
            ("solids", json!(part.solids)),
            ("faces", json!(faces)),
            ("edges", json!(edges)),
            ("vertices", json!(vertices)),
            ("sketches", json!(sketches)),
            ("datums", json!(datums)),
            ("placed_parts", json!(placed)),
        ]
        .into_iter()
        .map(|(key, value)| (key.to_string(), value))
        .collect()
    }

    /// Asks `inspector` about the part — of `elements`, for one that picks.
    fn inspect(&mut self, inspector: &str, elements: &Value, options: Value) -> GeopResult<Value> {
        let elements = (elements.as_array())
            .ok_or_else(|| GeopError::new("`elements` (a list of {kind, name}) is required"))?;
        let picked: Vec<PickedElement> = (elements.iter())
            .map(|e| {
                let field = |key: &str| {
                    e.get(key)
                        .and_then(Value::as_str)
                        .map(str::to_string)
                        .ok_or_else(|| GeopError::new(format!("an element needs `{key}`: {e}")))
                };
                Ok(PickedElement {
                    target: Located::new(self.scene()?.part.model.clone()),
                    kind: field("kind")?,
                    name: field("name")?,
                })
            })
            .collect::<GeopResult<_>>()?;
        let update = self.send(Command::Inspect {
            inspector: inspector.to_string(),
            options,
            picked,
        })?;
        let inspection = (update.inspection)
            .ok_or_else(|| GeopError::new(format!("`{inspector}` answered nothing")))?;
        Ok(inspection.report)
    }

    /// Writes the part to `path` with the exporter of `format`.
    fn export(
        &mut self,
        path: &str,
        format: &str,
        options: Value,
        files: &mut dyn Files,
    ) -> GeopResult<Value> {
        let outputs = self.editor.registry().outputs();
        let exporter = outputs.exporter_for(format)?.info().kind.to_string();
        let update = self.send(Command::Export {
            exporter: exporter.clone(),
            format: Some(format.to_string()),
            options,
            date: files.today(),
        })?;
        let export = (update.export)
            .ok_or_else(|| GeopError::new(format!("the exporter `{exporter}` wrote nothing")))?;
        files.write(path, export.bytes())?;
        Ok(json!({
            "written": path,
            "exporter": exporter,
            "format": format,
            "bytes": export.bytes().len(),
            "media_type": export.media_type,
        }))
    }
}
