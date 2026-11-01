//! `geop mcp`: the editor as a Model Context Protocol server.
//!
//! An agent (Claude Code, `claude mcp add geop -- geop mcp`) speaks JSON-RPC
//! 2.0 to it over stdin and stdout, one message per line, as MCP's stdio
//! transport does. The server offers the editor's agent tools
//! ([`geop_core_editor::agent`], which the web app's assistant is offered
//! too), every one of them: here, those that take a path read and write
//! files on disk.
//!
//! The handful of methods MCP needs are written out here rather than
//! taken from a crate: `initialize`, `ping`, `tools/list` and `tools/call`,
//! `resources/list`, `resources/templates/list` and `resources/read`,
//! `prompts/list` and `prompts/get`, and notifications, which are not
//! answered. See `docs/mcp.md`.

use std::{
    collections::BTreeMap,
    io::{BufRead, Write},
    path::Path,
};

use geop_cad_base::editor;
use geop_core_editor::agent::{
    AgentResource, Contents, Files, Session, prompts, resource_templates, resources, tools,
};
use geop_core_math::geop_error::{GeopError, GeopResult};
use serde_json::{Value, json};

/// The MCP revision answered to `initialize`.
const PROTOCOL_VERSION: &str = "2025-06-18";

/// What the server tells a client about itself at `initialize`.
const INSTRUCTIONS: &str = "geop is a parametric CAD kernel. A part is a program of steps, each an \
operation with arguments; steps refer to what earlier steps built by stable names such as \
`extrude(box)` (the solid the step `box` extruded) or `extrude(box,end)` (its end cap). Start \
with `new_part` or `load_program`, see what operations there are with `list_operations` and how \
one's arguments look with `list_operations` given its `kind`, add steps with `add_step`, check \
`run` for errors, find names for references with `list_elements`, `look` at the part to see it, \
and finish with `export` or `save`. A sketch is drawn on a plane such as \
`{\"datum\": \"origin:xy\"}` (or `origin:yz`, `origin:zx`). A step names what another step \
built by its target, `/:<target>:<step id>`: the sketch the step `base` drew is \
`{\"target\": \"/:sketch:base\"}`, and a reference left blank in `default_args` (`/:sketch:`) \
has to be filled in. A step that names a target no step before it produces says which targets \
there are. What is easier done than written — drawing a sketch, dragging a placed part — is done \
as a user does it: `open_step`, then `step_event` (clicks at pixels of the last `look`, or at \
points of the world, keys), then `commit_step`; `undo` takes an edit back. The resources \
`geop://examples/{name}` are whole example programs.";

/// The JSON-RPC error `code` with `message`.
fn rpc_error(id: Value, code: i64, message: impl Into<String>) -> Value {
    json!({ "jsonrpc": "2.0", "id": id, "error": { "code": code, "message": message.into() } })
}

/// The files on disk, as the tools that take a path read and write them.
struct Disk;

impl Files for Disk {
    fn read(&mut self, path: &str) -> GeopResult<String> {
        std::fs::read_to_string(path).map_err(|e| GeopError::new(format!("reading {path}: {e}")))
    }

    fn programs_beside(&mut self, path: &str) -> BTreeMap<String, Option<String>> {
        let mut files = BTreeMap::new();
        let dir = Path::new(path)
            .parent()
            .filter(|d| !d.as_os_str().is_empty());
        if let Some(dir) = dir
            && let Ok(entries) = std::fs::read_dir(dir)
        {
            for entry in entries.flatten() {
                let file = entry.path();
                if file.extension().is_some_and(|e| e == "geop")
                    && let Ok(text) = std::fs::read_to_string(&file)
                {
                    files.insert(file.to_string_lossy().into_owned(), Some(text));
                }
            }
        }
        files
    }

    fn write(&mut self, path: &str, bytes: &[u8]) -> GeopResult<()> {
        std::fs::write(path, bytes).map_err(|e| GeopError::new(format!("writing {path}: {e}")))
    }

    fn today(&self) -> String {
        super::today()
    }
}

/// One editor, driven by tools.
pub struct McpServer {
    session: Session,
}

impl Default for McpServer {
    fn default() -> Self {
        Self::new()
    }
}

impl McpServer {
    pub fn new() -> Self {
        Self {
            session: Session::new(editor()),
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
                "capabilities": { "tools": {}, "resources": {}, "prompts": {} },
                "serverInfo": { "name": "geop", "version": env!("CARGO_PKG_VERSION") },
                "instructions": INSTRUCTIONS,
            })),
            "ping" => Ok(json!({})),
            "tools/list" => {
                Ok(json!({ "tools": tools().iter().map(|t| t.mcp()).collect::<Vec<_>>() }))
            }
            "tools/call" => {
                let name = params.get("name").and_then(Value::as_str).unwrap_or("");
                let arguments = params.get("arguments").cloned().unwrap_or(json!({}));
                Ok(self.call(name, &arguments))
            }
            "resources/list" => Ok(
                json!({ "resources": resources().iter().map(|r| described(r, "uri")).collect::<Vec<_>>() }),
            ),
            "resources/templates/list" => Ok(json!({
                "resourceTemplates": resource_templates().iter().map(|r| described(r, "uriTemplate")).collect::<Vec<_>>(),
            })),
            "resources/read" => {
                let uri = params
                    .get("uri")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_string();
                match self.guarded(&uri, |session| session.read_resource(&uri)) {
                    Ok((Contents::Text(text), media_type)) => Ok(json!({
                        "contents": [{ "uri": uri, "mimeType": media_type, "text": text }],
                    })),
                    Ok((Contents::Image(image), media_type)) => Ok(json!({
                        "contents": [{ "uri": uri, "mimeType": media_type, "blob": image.base64() }],
                    })),
                    Err(message) => Err(rpc_error(id.clone(), -32002, message)),
                }
            }
            "prompts/list" => Ok(json!({
                "prompts": prompts().iter().map(|p| json!({
                    "name": p.name,
                    "description": p.description,
                    "arguments": p.arguments.iter().map(|(name, description, required)| json!({
                        "name": name, "description": description, "required": required,
                    })).collect::<Vec<_>>(),
                })).collect::<Vec<_>>(),
            })),
            "prompts/get" => {
                let name = params
                    .get("name")
                    .and_then(Value::as_str)
                    .unwrap_or("")
                    .to_string();
                let arguments = params.get("arguments").cloned().unwrap_or(json!({}));
                match self.guarded(&name, |session| session.prompt(&name, &arguments)) {
                    Ok((description, text)) => Ok(json!({
                        "description": description,
                        "messages": [{ "role": "user", "content": { "type": "text", "text": text } }],
                    })),
                    Err(message) => Err(rpc_error(id.clone(), -32602, message)),
                }
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
    fn call(&mut self, name: &str, arguments: &Value) -> Value {
        let answer = self.guarded(name, |session| {
            Ok(session.call(name, arguments, Some(&mut Disk)))
        });
        let (text, is_error, images) = match answer {
            Ok(answer) => (answer.text, answer.is_error, answer.images),
            Err(crashed) => (crashed, true, Vec::new()),
        };
        let mut content = vec![json!({ "type": "text", "text": text })];
        content.extend(images.iter().map(|image| {
            json!({ "type": "image", "data": image.base64(), "mimeType": image.media_type })
        }));
        json!({ "content": content, "isError": is_error })
    }

    /// `f` on the session, or why it failed. A kernel that panics leaves
    /// the editor in an unknown state: a fresh one takes over, with the
    /// program as it was before, and the panic is told as what `what` did.
    fn guarded<T>(
        &mut self,
        what: &str,
        f: impl FnOnce(&mut Session) -> GeopResult<T>,
    ) -> Result<T, String> {
        let answer =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| f(&mut self.session)));
        match answer {
            Ok(answer) => answer.map_err(|e| e.to_string()),
            Err(panic) => {
                let panic = panic
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_else(|| "unknown panic".to_string());
                self.session.restart(editor());
                Err(format!(
                    "{what} crashed the kernel ({panic}); the editor was restarted with the \
                     program as it was before"
                ))
            }
        }
    }
}

/// A resource as `resources/list` describes it, its URI under `uri`.
fn described(resource: &AgentResource, uri: &str) -> Value {
    json!({
        uri: resource.uri,
        "name": resource.name,
        "description": resource.description,
        "mimeType": resource.media_type,
    })
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
