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
//! and notifications, which are not answered. See `docs/mcp.md`.

use std::{
    collections::BTreeMap,
    io::{BufRead, Write},
    path::Path,
};

use geop_cad_base::editor;
use geop_core_editor::agent::{Files, Session, tools};
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
`run` for errors, find names for references with `list_elements`, and finish with `export` or \
`save`. A sketch is drawn on a plane such as `{\"datum\": \"origin:xy\"}` (or `origin:yz`, \
`origin:zx`). A step names what another step built by its target, `/:<target>:<step id>`: the \
sketch the step `base` drew is `{\"target\": \"/:sketch:base\"}`, and a reference left blank in \
`default_args` (`/:sketch:`) has to be filled in. A step that names a target no step before it \
produces says which targets there are.";

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
                "capabilities": { "tools": {} },
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
        let answer = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.session.call(name, arguments, Some(&mut Disk))
        }));
        let (text, is_error) = match answer {
            Ok(answer) => (answer.text, answer.is_error),
            Err(panic) => {
                let what = panic
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_else(|| "unknown panic".to_string());
                self.session.restart(editor());
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
