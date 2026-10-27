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

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::PickedElement,
    placed::Located,
    program::{Program, ProgramStep},
};
use serde::Serialize;
use serde_json::{Value, json};

use crate::editor::{Command, Editor, ProgramState, SceneState, Update};

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
            "Start a new, empty part. Every part has the coordinate-system datum `origin`, whose planes a sketch is drawn on, as `add_sketch`'s `plane`: {\"datum\": \"origin:xy\"}, {\"datum\": \"origin:yz\"} or {\"datum\": \"origin:zx\"}.",
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
            "Add a step of the operation `kind` with `args` (its saved JSON, see `list_operations` with `kind`) at the end of the program, or before the step `before`. A step whose arguments do not read is refused and not added; one that reads but does not build is added, and its error says why. Answers with every step's status: an error names the step and what went wrong.",
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
            "The names of the part's elements as they are now, for typed references: solids, faces (with their solid), edges (with their faces), vertices, sketches, datums and what else the part lists (threads, placed parts). A face is referred to by its name, as a sketch's plane {\"face\": ...}; an edge by its name, as in a fillet's `edges`.",
            json!({}),
            &[],
            false,
        ),
        tool(
            "inspect",
            "Ask about the part: `mass_properties` (volume, area and centroid of every solid; its mass and inertia if its part has a material, null if not), `interference` (which solids overlap), or `measure` with `elements` (a point, a length, an area, a radius; the distance and angle between two).",
            json!({ "inspector": { "type": "string", "enum": ["mass_properties", "interference", "measure"] }, "elements": { "type": "array", "description": "For `measure`: one or two elements, each {\"kind\": \"vertex\"|\"edge\"|\"face\"|\"datum\", \"name\": ...}.", "items": { "type": "object", "properties": { "kind": string("vertex, edge, face or datum"), "name": string("Its name, from list_elements") }, "required": ["kind", "name"] } } }),
            &["inspector"],
            false,
        ),
        tool(
            "export",
            "Write what the program builds to `path`, in the format its extension names unless `format` says otherwise: stl, step, svg or dxf (drawings and flat patterns), csv (bill of materials), zip (URDF).",
            json!({ "path": string("Where to write the file."), "format": string("The format, if not the path's extension."), "options": object("The exporter's own options, e.g. {\"quality\": 8} for STL.") }),
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
    ]
}

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
}

/// One editor, driven by tools and by a front end alike, and what a front
/// end keeps of its answers.
pub struct Session {
    editor: Editor,
    /// The part as last drawn: the names `list_elements` answers with.
    scene: Option<SceneState>,
    /// The program as last shown: its steps' status, the operations.
    shown: Option<ProgramState>,
    /// The program as last known to be whole, to start a fresh editor from
    /// after a panic ([`Self::restart`]).
    program: Program,
    /// The file it is edited in, once told.
    path: Option<String>,
    /// The updates of the tool running, in order.
    sent: Vec<Update>,
}

impl Session {
    /// A session of `editor`, shown whole.
    pub fn new(editor: Editor) -> Self {
        let mut session = Self {
            editor,
            scene: None,
            shown: None,
            program: Program::new(),
            path: None,
            sent: Vec::new(),
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
            self.scene = Some(scene.clone());
        }
        if let Some(program) = &update.program {
            self.shown = Some(program.clone());
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
        let program = self.program.clone();
        let path = self.path.clone();
        self.handle(Command::Load { program, path });
    }

    /// Calls the tool `name` with `input`, in a front end with `files`, if
    /// it has any: its result as pretty JSON, or why it failed.
    pub fn call(&mut self, name: &str, input: &Value, files: Option<&mut dyn Files>) -> ToolAnswer {
        self.sent.clear();
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
        match (name, files) {
            ("new_part", _) => {
                self.load(Program::new())?;
                self.status()
            }
            ("load_program", Some(files)) => {
                if let Some(path) = args.get("path").and_then(Value::as_str) {
                    self.load_file(path, files)
                } else {
                    self.load(Program::from_json(&text("text")?)?)?;
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
            ("list_elements", _) => self.list_elements(),
            ("inspect", _) => {
                let inspector = text("inspector")?;
                let elements = args.get("elements").cloned().unwrap_or(json!([]));
                self.inspect(&inspector, &elements)
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
                if self.path.as_deref() != Some(path.as_str()) {
                    self.send(Command::Load {
                        program,
                        path: Some(path.clone()),
                    })?;
                }
                Ok(json!({ "saved": path, "bytes": json.len() }))
            }
            _ => Err(GeopError::new(format!("there is no tool `{name}`"))),
        }
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
        Ok(json!({
            "steps": steps,
            "failed": failed,
            "solids": solids,
            "path": program.path,
        }))
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
        let part = &scene.part;
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
        let structure: Vec<Value> = (scene.structure.iter())
            .map(|item| json!({ "kind": item.kind, "name": item.name }))
            .collect();
        // Every part placed, not only those the last scene sent anew.
        let placed: Vec<&str> = part.instances.iter().map(|i| i.name.as_str()).collect();
        Ok(json!({
            "solids": part.solids,
            "faces": faces,
            "edges": edges,
            "vertices": vertices,
            "sketches": sketches,
            "datums": datums,
            "structure": structure,
            "placed_parts": placed,
        }))
    }

    /// Asks `inspector` about the part — of `elements`, for one that picks.
    fn inspect(&mut self, inspector: &str, elements: &Value) -> GeopResult<Value> {
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
            options: Value::Null,
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
