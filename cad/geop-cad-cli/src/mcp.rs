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
//!
//! The kernel asks one thing at a time. Calls wait their turn on a queue,
//! and a thread reads the input meanwhile, so that `ping` is answered and a
//! cancellation heard while a long call runs: it tells the call to stop at
//! its next turn (see [`geop_core_program::watch`]), or a call waiting not
//! to start. A long call's answer says how long it took and waited.

use std::{
    collections::{BTreeMap, HashSet},
    io::{BufRead, Write},
    path::Path,
    sync::{Arc, Mutex, mpsc},
    time::{Duration, Instant},
};

use geop_cad_base::editor;
use geop_core_editor::agent::{
    AgentResource, Contents, Files, Session, prompts, resource_templates, resources, tools,
};
use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::watch::{Progress, watching};
use serde_json::{Value, json};

/// The MCP revision answered to `initialize`.
const PROTOCOL_VERSION: &str = "2025-06-18";

/// How long a call has to have taken, waiting included, for its answer to
/// say so.
const WORTH_SAYING: Duration = Duration::from_secs(1);

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
    /// What is cancelled, and where to say how far a call is, as `serve`
    /// has them.
    heard: Option<(Cancelled, Out)>,
    /// When the message being answered came: how long it waited.
    came: Instant,
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
            heard: None,
            came: Instant::now(),
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
                let token = params.pointer("/_meta/progressToken").cloned();
                Ok(self.call(name, &arguments, &id, token))
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
    fn call(&mut self, name: &str, arguments: &Value, id: &Value, token: Option<Value>) -> Value {
        let started = Instant::now();
        let waited = started.duration_since(self.came);
        let limit = (arguments.get("limit_seconds").and_then(Value::as_f64))
            .map(|seconds| {
                Duration::try_from_secs_f64(seconds)
                    .map_err(|e| format!("`limit_seconds` is {seconds}: {e}"))
            })
            .transpose();
        let answer = limit.and_then(|limit| {
            let watcher = self.watcher(id, token, limit.map(|limit| started + limit));
            self.guarded(name, |session| {
                Ok(watching(watcher, || {
                    session.call(name, arguments, Some(&mut Disk))
                }))
            })
        });
        let (text, is_error, images) = match answer {
            Ok(answer) => (answer.text, answer.is_error, answer.images),
            Err(crashed) => (crashed, true, Vec::new()),
        };
        let mut content = vec![json!({ "type": "text", "text": text })];
        content.extend(images.iter().map(|image| {
            json!({ "type": "image", "data": image.base64(), "mimeType": image.media_type })
        }));
        let took = started.elapsed();
        if took + waited >= WORTH_SAYING {
            let said = format!(
                "Took {:.1} s, after waiting {:.1} s behind other calls: the kernel asks one thing at a time.",
                took.as_secs_f64(),
                waited.as_secs_f64()
            );
            content.push(json!({ "type": "text", "text": said }));
        }
        json!({ "content": content, "isError": is_error })
    }

    /// What watches the call `id`: it tells the client how far it is, if
    /// the client gave a `token` to say it by, no more than once a second,
    /// and stops it once it is cancelled, or at `deadline`.
    fn watcher(
        &self,
        id: &Value,
        token: Option<Value>,
        deadline: Option<Instant>,
    ) -> impl FnMut(&Progress) -> bool + 'static {
        let heard = self.heard.clone();
        let id = id.to_string();
        let (mut last, mut told) = (None::<Instant>, 0_u64);
        move |progress: &Progress| {
            let now = Instant::now();
            if let (Some(token), Some((_, out))) = (&token, &heard)
                && last.is_none_or(|last| now.duration_since(last) >= Duration::from_secs(1))
            {
                last = Some(now);
                told += 1;
                out.send(&json!({
                    "jsonrpc": "2.0",
                    "method": "notifications/progress",
                    "params": {
                        "progressToken": token,
                        "progress": told,
                        "message": format!("{}: {} of {}", progress.what, progress.done, progress.total),
                    },
                }));
            }
            let cancelled = heard.as_ref().is_some_and(|(cancelled, _)| {
                (cancelled.lock().unwrap_or_else(|e| e.into_inner())).contains(&id)
            });
            !cancelled && deadline.is_none_or(|deadline| now < deadline)
        }
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

/// The ids of the requests a client cancelled.
type Cancelled = Arc<Mutex<HashSet<String>>>;

/// Where answers and notifications go: one message per line, whichever
/// thread sends it.
#[derive(Clone)]
struct Out(Arc<Mutex<std::io::Stdout>>);

impl Out {
    fn send(&self, message: &Value) {
        let mut stdout = self.0.lock().unwrap_or_else(|e| e.into_inner());
        // Nobody left to tell, if the client has gone: stdin closes next.
        let _ = writeln!(stdout, "{message}").and_then(|()| stdout.flush());
    }
}

/// Reads the input until it closes: `ping` and cancellations at once, the
/// rest onto the queue, each with when it came.
fn read_input(queue: mpsc::Sender<(String, Instant)>, cancelled: Cancelled, out: Out) {
    for line in std::io::stdin().lock().lines() {
        let Ok(line) = line else { return };
        if line.trim().is_empty() {
            continue;
        }
        let message: Value = serde_json::from_str(&line).unwrap_or(Value::Null);
        match message.get("method").and_then(Value::as_str) {
            Some("notifications/cancelled") => {
                let id = &message["params"]["requestId"];
                (cancelled.lock().unwrap_or_else(|e| e.into_inner())).insert(id.to_string());
            }
            Some("ping") if message.get("id").is_some() => out.send(&json!({
                "jsonrpc": "2.0", "id": message["id"], "result": {},
            })),
            _ => {
                if queue.send((line, Instant::now())).is_err() {
                    return;
                }
            }
        }
    }
}

/// Runs the server until stdin closes: one JSON-RPC message per line in,
/// one answer per line out, nothing for a notification.
pub fn serve() -> GeopResult<()> {
    // The default hook prints every panic to stderr, which MCP clients log:
    // what crashed is answered in the tool's result instead.
    std::panic::set_hook(Box::new(|_| {}));
    let out = Out(Arc::new(Mutex::new(std::io::stdout())));
    let cancelled = Cancelled::default();
    let (queue, queued) = mpsc::channel();
    let reader = (cancelled.clone(), out.clone());
    std::thread::spawn(move || read_input(queue, reader.0, reader.1));
    let mut server = McpServer::new();
    server.heard = Some((cancelled.clone(), out.clone()));
    for (line, came) in queued {
        let id = (serde_json::from_str::<Value>(&line).ok())
            .and_then(|message| message.get("id").map(Value::to_string));
        let is_cancelled = || {
            id.as_ref().is_some_and(|id| {
                (cancelled.lock().unwrap_or_else(|e| e.into_inner())).contains(id)
            })
        };
        // Cancelled, it is answered with nothing, as MCP asks: waiting, it
        // does not start, and running, what it had found goes nowhere.
        if is_cancelled() {
            continue;
        }
        server.came = came;
        if let Some(answer) = server.handle(&line)
            && !is_cancelled()
        {
            out.send(&answer);
        }
    }
    Ok(())
}
