//! `geop mcp`: the editor as a Model Context Protocol server.
//!
//! An agent (Claude Code, `claude mcp add geop -- geop mcp`) speaks JSON-RPC
//! 2.0 to it over stdin and stdout, one message per line, as MCP's stdio
//! transport does. The server offers *tools*, each a transaction on one
//! [`Editor`] — the same editor the web app runs — through its protocol
//! ([`Command`] in, [`Update`] out): a part is started or loaded, steps are
//! added, edited and removed as their saved JSON (what a `.geop` file
//! holds), the program is run, its elements listed by the stable names a
//! step refers to them by, inspected, exported and saved.
//!
//! The handful of methods MCP needs are written out here rather than
//! taken from a crate: `initialize`, `ping`, `tools/list` and `tools/call`,
//! and notifications, which are not answered. See `docs/mcp.md`.

use std::{
    collections::BTreeMap,
    io::{BufRead, Write},
    path::{Path, PathBuf},
};

use geop_cad_base::{Command, Editor, Update, editor, outputs};
use geop_core_editor::editor::SceneState;
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{element::PickedElement, placed::Located, program::Program};
use serde_json::{Value, json};

/// The MCP revision answered to `initialize`.
const PROTOCOL_VERSION: &str = "2025-06-18";

/// What the server tells a client about itself at `initialize`.
const INSTRUCTIONS: &str = "geop is a parametric CAD kernel. A part is a program of steps, each an \
operation with arguments; steps refer to what earlier steps built by stable names such as \
`extrude(box)` (the solid the step `box` extruded) or `extrude(box,end)` (its end cap). Start \
with `new_part` or `load_program`, see what operations there are and how their arguments look \
with `list_operations`, add steps with `add_step`, check `run` for errors, find names for \
references with `list_elements`, and finish with `export` or `save`.";

/// One editor, driven by tools, and what a front end would keep of its
/// answers: the last scene (the part's elements by name) and the file.
pub struct McpServer {
    editor: Editor,
    /// The part as last drawn: the names `list_elements` answers with.
    scene: Option<SceneState>,
    /// The program as last known to be whole, to start a fresh editor from
    /// after a panic.
    program: Program,
    path: Option<String>,
}

impl Default for McpServer {
    fn default() -> Self {
        Self::new()
    }
}

/// The JSON-RPC error `code` with `message`.
fn rpc_error(id: Value, code: i64, message: impl Into<String>) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": message.into() } })
}

/// The tools offered, as `tools/list` describes them: name, what it does,
/// and the JSON schema of its arguments.
fn tools() -> Value {
    let tool = |name: &str, description: &str, properties: Value, required: &[&str]| {
        json!({
            "name": name,
            "description": description,
            "inputSchema": {
                "type": "object",
                "properties": properties,
                "required": required,
                "additionalProperties": false,
            },
        })
    };
    let string = |description: &str| json!({ "type": "string", "description": description });
    let object = |description: &str| json!({ "type": "object", "description": description });
    json!({ "tools": [
        tool("new_part", "Start a new, empty part. Every part has the coordinate-system datum `origin`, whose planes a sketch is drawn on: {\"type\": \"Datum\", \"name\": \"origin\", \"component\": {\"plane\": \"z\"}}.",
            json!({}), &[]),
        tool("load_program", "Replace the part with a program: the `.geop` file at `path` (the other `.geop` files next to it are read too, for the parts it places), or the program `text` itself.",
            json!({ "path": string("A .geop file on disk."), "text": string("A program as JSON, as a .geop file holds it.") }), &[]),
        tool("list_operations", "Every operation a step can be: its kind (what `add_step` takes), label, group, what it does, the arguments a new step starts with (`default_args`, the JSON `add_step` takes) and the form the editor shows for it (`form`: each field's key, control, label, value and, for a reference, the kinds of elements it accepts).",
            json!({}), &[]),
        tool("add_step", "Append a step of the operation `kind` with `args` (its saved JSON, see `list_operations`), after the steps that run now. Answers with every step's status: an error names the step and what went wrong.",
            json!({ "kind": string("The operation, e.g. `add_sketch`, `extrude`, `fillet`."), "args": object("The step's arguments as saved in a .geop file."), "id": string("The step's id (letters, digits, `_`, `-`, `.`); chosen from the operation's label if left out. Later steps refer to what this one builds by it, e.g. `extrude(<id>)`.") }),
            &["kind", "args"]),
        tool("edit_step", "Change the step `id`: the keys of `args` replace those of its arguments (the others stay), so one dimension can be changed alone. The program is rebuilt from that step on.",
            json!({ "id": string("The step to change."), "args": object("The arguments to set.") }), &["id", "args"]),
        tool("delete_step", "Remove the step `id`. Steps that referred to what it built then fail, and say so.",
            json!({ "id": string("The step to remove.") }), &["id"]),
        tool("run", "Run the whole program and report every step: its summary, whether it ran, and its error with the kernel's context if it failed; and the solids the part has.",
            json!({}), &[]),
        tool("list_elements", "The names of the part's elements as they are now, for typed references: solids, faces (with their solid), edges (with their faces), vertices, sketches, datums and what else the part lists (threads, placed parts). A face is referred to as {\"type\": \"Face\", \"name\": ...}, an edge as {\"type\": \"Edge\", \"name\": ...}.",
            json!({}), &[]),
        tool("inspect", "Ask about the part: `mass_properties` (volume, area, mass, centre of mass, inertia of every solid), `interference` (which solids overlap), or `measure` with `elements` (a point, a length, an area, a radius; the distance and angle between two).",
            json!({ "inspector": { "type": "string", "enum": ["mass_properties", "interference", "measure"] }, "elements": { "type": "array", "description": "For `measure`: one or two elements, each {\"kind\": \"vertex\"|\"edge\"|\"face\"|\"datum\", \"name\": ...}.", "items": { "type": "object", "properties": { "kind": string("vertex, edge, face or datum"), "name": string("Its name, from list_elements") }, "required": ["kind", "name"] } } }),
            &["inspector"]),
        tool("export", "Write what the program builds to `path`, in the format its extension names unless `format` says otherwise: stl, step, svg or dxf (drawings and flat patterns), csv (bill of materials), zip (URDF).",
            json!({ "path": string("Where to write the file."), "format": string("The format, if not the path's extension."), "options": object("The exporter's own options, e.g. {\"quality\": 8} for STL.") }), &["path"]),
        tool("save", "Write the program to `path` as a .geop file, and make that the file it is edited in.",
            json!({ "path": string("Where to write the program.") }), &["path"]),
    ] })
}

impl McpServer {
    pub fn new() -> Self {
        let mut server = Self {
            editor: editor(),
            scene: None,
            program: Program::new(),
            path: None,
        };
        server.send(Command::Show);
        server
    }

    /// Sends `command` to the editor, keeping what a front end keeps of the
    /// answer: the scene, and the program as it is now.
    fn send(&mut self, command: Command) -> Update {
        let update = self.editor.handle(command);
        if let Some(scene) = &update.scene {
            self.scene = Some(scene.clone());
        }
        if update.error.is_none() {
            self.program = self.editor.program().clone();
        }
        update
    }

    /// Sends `command`; the update, or its error as an `Err`.
    fn send_ok(&mut self, command: Command) -> GeopResult<Update> {
        let update = self.send(command);
        match update.error {
            Some(error) => Err(GeopError::new(error)),
            None => Ok(update),
        }
    }

    /// Answers one JSON-RPC message, or nothing for a notification.
    pub fn handle(&mut self, line: &str) -> Option<Value> {
        let message: Value = match serde_json::from_str(line) {
            Ok(message) => message,
            Err(e) => return Some(rpc_error(Value::Null, -32700, format!("parse error: {e}"))),
        };
        let id = message.get("id").cloned().unwrap_or(Value::Null);
        let method = message.get("method").and_then(Value::as_str).unwrap_or("");
        let params = message.get("params").cloned().unwrap_or(Value::Null);
        if method.starts_with("notifications/") || (id.is_null() && !method.is_empty()) {
            return None;
        }
        let result = match method {
            "initialize" => Ok(json!({
                "protocolVersion": params.get("protocolVersion").cloned().unwrap_or_else(|| json!(PROTOCOL_VERSION)),
                "capabilities": { "tools": {} },
                "serverInfo": { "name": "geop", "version": env!("CARGO_PKG_VERSION") },
                "instructions": INSTRUCTIONS,
            })),
            "ping" => Ok(json!({})),
            "tools/list" => Ok(tools()),
            "tools/call" => {
                let name = params.get("name").and_then(Value::as_str).unwrap_or("");
                let arguments = params.get("arguments").cloned().unwrap_or(json!({}));
                Ok(self.call(name, arguments))
            }
            _ => Err(rpc_error(
                id.clone(),
                -32601,
                format!("method not found: {method}"),
            )),
        };
        Some(match result {
            Ok(result) => json!({ "jsonrpc": "2.0", "id": id, "result": result }),
            Err(error) => error,
        })
    }

    /// Calls the tool `name` with `arguments`, as `tools/call` answers: its
    /// result as text, or why it failed as an error result. A kernel that
    /// panics leaves the editor in an unknown state: a fresh one takes
    /// over, with the program as it was before the call.
    fn call(&mut self, name: &str, arguments: Value) -> Value {
        let answer =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.tool(name, &arguments)));
        let (text, is_error) = match answer {
            Ok(Ok(result)) => (pretty(&result), false),
            Ok(Err(e)) => (format!("{name} failed: {e}"), true),
            Err(panic) => {
                let what = panic
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_else(|| "unknown panic".to_string());
                self.editor = editor();
                let program = self.program.clone();
                let path = self.path.clone();
                self.send(Command::Load { program, path });
                (
                    format!(
                        "{name} crashed the kernel ({what}); the editor was restarted with the \
                         program as it was before the call"
                    ),
                    true,
                )
            }
        };
        json!({ "content": [{ "type": "text", "text": text }], "isError": is_error })
    }

    /// The tool `name` applied to `args`: what it answers.
    fn tool(&mut self, name: &str, args: &Value) -> GeopResult<Value> {
        let text = |key: &str| -> GeopResult<String> {
            args.get(key)
                .and_then(Value::as_str)
                .map(str::to_string)
                .ok_or_else(|| GeopError::new(format!("`{key}` (a string) is required")))
        };
        match name {
            "new_part" => {
                self.send_ok(Command::Load {
                    program: Program::new(),
                    path: None,
                })?;
                self.status()
            }
            "load_program" => {
                if let Some(path) = args.get("path").and_then(Value::as_str) {
                    self.load_file(path)
                } else {
                    let program = Program::from_json(&text("text")?)?;
                    self.send_ok(Command::Load {
                        program,
                        path: None,
                    })?;
                    self.status()
                }
            }
            "list_operations" => self.list_operations(),
            "add_step" => {
                let kind = text("kind")?;
                let step_args = (args.get("args").cloned())
                    .ok_or_else(|| GeopError::new("`args` (the step's arguments) is required"))?;
                let mut program = self.editor.program().clone();
                let id = match args.get("id").and_then(Value::as_str) {
                    Some(id) => id.to_string(),
                    None => program.fresh_id(&kind),
                };
                program.steps.push(geop_core_program::program::ProgramStep {
                    id: id.clone(),
                    operation: kind,
                    args: step_args,
                });
                self.send_ok(Command::Load {
                    program,
                    path: None,
                })
                .map_err(|e| e.with_context(format!("add_step(id={id})")))?;
                self.status()
            }
            "edit_step" => {
                let id = text("id")?;
                let set = (args.get("args").cloned())
                    .ok_or_else(|| GeopError::new("`args` (the arguments to set) is required"))?;
                let mut program = self.editor.program().clone();
                let step = program.step_mut(&id)?;
                match (&mut step.args, set) {
                    (Value::Object(args), Value::Object(set)) => args.extend(set),
                    (args, set) => *args = set,
                }
                self.send_ok(Command::Load {
                    program,
                    path: None,
                })
                .map_err(|e| e.with_context(format!("edit_step(id={id})")))?;
                self.status()
            }
            "delete_step" => {
                let id = text("id")?;
                self.send_ok(Command::Remove { id })?;
                self.status()
            }
            "run" => {
                self.send_ok(Command::Seek { marker: None })?;
                self.status()
            }
            "list_elements" => self.list_elements(),
            "inspect" => {
                let inspector = text("inspector")?;
                let elements = args.get("elements").cloned().unwrap_or(json!([]));
                self.inspect(&inspector, &elements)
            }
            "export" => {
                let path = PathBuf::from(text("path")?);
                let format = match args.get("format").and_then(Value::as_str) {
                    Some(format) => format.to_string(),
                    None => (path.extension())
                        .and_then(|e| e.to_str())
                        .map(str::to_lowercase)
                        .ok_or_else(|| {
                            GeopError::new(format!("{} names no format", path.display()))
                        })?,
                };
                let options = args.get("options").cloned().unwrap_or(json!({}));
                self.export(&path, &format, options)
            }
            "save" => {
                let path = text("path")?;
                let program = self.editor.program().clone();
                let json = program.to_json()?;
                std::fs::write(&path, &json)
                    .map_err(|e| GeopError::new(format!("writing {path}: {e}")))?;
                if self.path.as_deref() != Some(path.as_str()) {
                    self.send_ok(Command::Load {
                        program,
                        path: Some(path.clone()),
                    })?;
                    self.path = Some(path.clone());
                }
                Ok(json!({ "saved": path, "bytes": json.len() }))
            }
            _ => Err(GeopError::new(format!("there is no tool `{name}`"))),
        }
    }

    /// Loads the program file `path`, with the other program files next to
    /// it, which the parts it places are read from.
    fn load_file(&mut self, path: &str) -> GeopResult<Value> {
        let read = |path: &Path| {
            std::fs::read_to_string(path)
                .map_err(|e| GeopError::new(format!("reading {}: {e}", path.display())))
        };
        let program = Program::from_json(&read(Path::new(path))?)?;
        let dir = Path::new(path)
            .parent()
            .filter(|d| !d.as_os_str().is_empty());
        let mut files = BTreeMap::new();
        if let Some(dir) = dir
            && let Ok(entries) = std::fs::read_dir(dir)
        {
            for entry in entries.flatten() {
                let file = entry.path();
                if file.extension().is_some_and(|e| e == "geop")
                    && let Ok(text) = read(&file)
                {
                    files.insert(file.to_string_lossy().into_owned(), Some(text));
                }
            }
        }
        if !files.is_empty() {
            self.send_ok(Command::Files { files })?;
        }
        self.send_ok(Command::Load {
            program,
            path: Some(path.to_string()),
        })?;
        self.path = Some(path.to_string());
        self.status()
    }

    /// Every step, as the program's list shows it, and the part's solids.
    fn status(&mut self) -> GeopResult<Value> {
        let update = self.send(Command::Show);
        let program = (update.program.as_ref())
            .ok_or_else(|| GeopError::new("the editor showed no program"))?;
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

    /// Every operation, with the arguments a new step of it starts with and
    /// the form it is edited through.
    fn list_operations(&mut self) -> GeopResult<Value> {
        let shown = self.send_ok(Command::Show)?;
        let program = shown
            .program
            .ok_or_else(|| GeopError::new("the editor showed no program"))?;
        let mut operations = Vec::new();
        for listed in &program.operations {
            let kind = listed.kind.to_string();
            let opened = self.send(Command::New { kind: kind.clone() });
            let (default_args, form, missing, doc) = match (&opened.error, &opened.step) {
                (None, Some(step)) => {
                    let args = self.editor.editing().map(|s| s.args.clone());
                    let form = serde_json::to_value(&step.presentation.dialog)
                        .map_err(|e| GeopError::new(format!("the form of `{kind}`: {e}")))?;
                    (args, form, step.missing.clone(), step.doc)
                }
                _ => (None, Value::Null, Vec::new(), ""),
            };
            if opened.error.is_none() {
                self.send(Command::Cancel);
            }
            operations.push(json!({
                "kind": kind,
                "label": listed.info.label,
                "group": listed.info.group,
                "tier": listed.info.tier,
                "doc": if doc.is_empty() { listed.info.doc } else { doc },
                "experimental": listed.info.experimental,
                "default_args": default_args,
                "missing": missing,
                "form": form,
            }));
        }
        Ok(json!({ "operations": operations }))
    }

    /// The scene as last drawn.
    fn scene(&self) -> GeopResult<&SceneState> {
        self.scene
            .as_ref()
            .ok_or_else(|| GeopError::new("nothing has been drawn yet: start with new_part"))
    }

    fn list_elements(&mut self) -> GeopResult<Value> {
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
        Ok(json!({
            "solids": part.solids,
            "faces": faces,
            "edges": edges,
            "vertices": vertices,
            "sketches": sketches,
            "datums": datums,
            "structure": structure,
            "placed_parts": scene.instances.iter().map(|i| i.name.clone()).collect::<Vec<_>>(),
        }))
    }

    /// Asks `inspector` about the part — `measure`, about `elements`.
    fn inspect(&mut self, inspector: &str, elements: &Value) -> GeopResult<Value> {
        if inspector == "measure" {
            let model = Located::new(self.scene()?.part.model.clone());
            let picked: Vec<PickedElement> = (elements.as_array())
                .ok_or_else(|| GeopError::new("`elements` (a list of {kind, name}) is required"))?
                .iter()
                .map(|e| {
                    let field = |key: &str| {
                        e.get(key)
                            .and_then(Value::as_str)
                            .map(str::to_string)
                            .ok_or_else(|| GeopError::new(format!("an element needs `{key}`: {e}")))
                    };
                    Ok(PickedElement {
                        target: model.clone(),
                        kind: field("kind")?,
                        name: field("name")?,
                    })
                })
                .collect::<GeopResult<_>>()?;
            let measurement = geop_ops_inspect::measure(&self.editor.part(), &picked);
            return serde_json::to_value(&measurement)
                .map_err(|e| GeopError::new(format!("writing the measurement: {e}")));
        }
        let update = self.send_ok(Command::Inspect {
            inspector: inspector.to_string(),
            options: Value::Null,
        })?;
        let inspection = (update.inspection)
            .ok_or_else(|| GeopError::new(format!("`{inspector}` answered nothing")))?;
        Ok(inspection.report)
    }

    /// Writes the part to `path` with the exporter of `format`.
    fn export(&mut self, path: &Path, format: &str, options: Value) -> GeopResult<Value> {
        let exporter = outputs()?.exporter_for(format)?.info().kind.to_string();
        let update = self.send_ok(Command::Export {
            exporter: exporter.clone(),
            format: Some(format.to_string()),
            options,
            date: super::today(),
        })?;
        let export = (update.export)
            .ok_or_else(|| GeopError::new(format!("the exporter `{exporter}` wrote nothing")))?;
        std::fs::write(path, export.bytes())
            .map_err(|e| GeopError::new(format!("writing {}: {e}", path.display())))?;
        Ok(json!({
            "written": path.display().to_string(),
            "exporter": exporter,
            "format": format,
            "bytes": export.bytes().len(),
            "media_type": export.media_type,
        }))
    }
}

/// `value` as text: pretty JSON.
fn pretty(value: &Value) -> String {
    serde_json::to_string_pretty(value).unwrap_or_else(|_| value.to_string())
}

/// Runs the server until stdin closes: one JSON-RPC message per line in,
/// one answer per line out, nothing for a notification.
pub fn serve() -> GeopResult<()> {
    let io_err = |e: std::io::Error| GeopError::new(format!("serving MCP: {e}"));
    // The default hook prints every panic to stderr, which MCP clients log:
    // what crashed is answered in the tool's result instead.
    std::panic::set_hook(Box::new(|_| {}));
    let mut server = McpServer::new();
    let stdin = std::io::stdin().lock();
    let mut stdout = std::io::stdout().lock();
    for line in stdin.lines() {
        let line = line.map_err(io_err)?;
        if line.trim().is_empty() {
            continue;
        }
        if let Some(answer) = server.handle(&line) {
            writeln!(stdout, "{answer}")
                .and_then(|()| stdout.flush())
                .map_err(io_err)?;
        }
    }
    Ok(())
}
