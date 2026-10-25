//! `geop open [DIR]`: the web editor in a browser, its kernel this process
//! and its workspace the folder `DIR`.
//!
//! The page is the web app as `vite build --mode open` builds it
//! (`web/src/backend.local.ts`), embedded in the executable with the
//! feature `frontend`. It talks to this process over HTTP on 127.0.0.1:
//!
//! - `GET /ws?token=…`: a WebSocket to the editor's [`Session`]. Each text
//!   message is one command of the JSON protocol `geop serve` speaks on its
//!   stdin, and is answered, in order, by one message: the update.
//! - `GET /api/files`: the folder's program and data files, by their paths
//!   relative to it, with the one to edit first, if one was named.
//! - `PUT /api/file?path=…` writes a file, `DELETE` removes one.
//! - Anything else is a file of the page itself.
//!
//! What can reach 127.0.0.1 is more than the user: every page the browser
//! has open can send requests to it, and a page of a name that resolves to
//! 127.0.0.1 (DNS rebinding) can even read the answers. So every request
//! must name this server as its `Host`, a request from a page must come
//! from this server's own origin, and the WebSocket and the file API take
//! a token that is made afresh each run and given only in the URL printed
//! (see [`Access`]). The file API reads and writes only files under `DIR`,
//! of the kinds the explorer shows, never through a link out of it (see
//! [`Folder`]).
//!
//! The editor is one, for every WebSocket: a thread owns it and answers
//! the commands the connections send it ([`SessionHandle`]). That is the
//! session another front end would attach to — an MCP client driving what
//! the page shows (`geop mcp --attach`, not yet written) would be another
//! sender of commands to it.

use crate::session::Session;
use geop_core_math::geop_error::{GeopError, GeopResult};
use std::{
    collections::BTreeMap,
    io::Read,
    path::{Path, PathBuf},
    sync::mpsc,
};

#[derive(clap::Args)]
pub struct OpenArgs {
    /// The folder to edit the files of — or a program in it, to edit
    /// first.
    #[arg(default_value = ".")]
    pub path: PathBuf,
    /// The port to serve on, on 127.0.0.1; a free one, if not given.
    #[arg(long)]
    pub port: Option<u16>,
    /// Print the editor's URL only, rather than opening it in the default
    /// browser too.
    #[arg(long)]
    pub no_browser: bool,
}

/// The most files, and bytes in all, of a folder the editor is sent: a
/// folder much larger is no workspace, but a home directory opened by
/// mistake.
const MOST_FILES: usize = 10_000;
const MOST_BYTES: u64 = 512 << 20;

/// The page, as `vite build --mode open` writes it (`web/dist-open`).
#[cfg(feature = "frontend")]
#[derive(rust_embed::RustEmbed)]
#[folder = "../../web/dist-open"]
struct Page;

/// The file of the page at `path`, if the page was built in.
fn page_file(path: &str) -> Option<std::borrow::Cow<'static, [u8]>> {
    #[cfg(feature = "frontend")]
    return Page::get(path).map(|file| file.data);
    #[cfg(not(feature = "frontend"))]
    {
        let _ = path;
        None
    }
}

/// What the page's file `path` is, for its `Content-Type`.
pub fn media_type(path: &str) -> &'static str {
    let extension = path.rsplit_once('.').map(|(_, e)| e.to_ascii_lowercase());
    match extension.as_deref() {
        Some("html") => "text/html; charset=utf-8",
        Some("js" | "mjs") => "text/javascript; charset=utf-8",
        Some("css") => "text/css; charset=utf-8",
        Some("json" | "map") => "application/json",
        Some("svg") => "image/svg+xml",
        Some("png") => "image/png",
        Some("jpg" | "jpeg") => "image/jpeg",
        Some("gif") => "image/gif",
        Some("webp") => "image/webp",
        Some("ico") => "image/x-icon",
        Some("woff") => "font/woff",
        Some("woff2") => "font/woff2",
        Some("wasm") => "application/wasm",
        Some("txt") => "text/plain; charset=utf-8",
        _ => "application/octet-stream",
    }
}

// ── the folder ─────────────────────────────────────────────────────────

/// The folder edited: the files under it of the extensions the explorer
/// shows — programs, and the data files operations read (see
/// `OperationInfo::reads`).
pub struct Folder {
    /// Canonical: what every file's own canonical path must start with.
    root: PathBuf,
    /// In lower case, `geop` among them.
    extensions: Vec<String>,
}

impl Folder {
    pub fn new(root: &Path, extensions: Vec<String>) -> GeopResult<Self> {
        let root = std::fs::canonicalize(root)
            .map_err(|e| GeopError::new(format!("opening {}: {e}", root.display())))?;
        if !root.is_dir() {
            return Err(GeopError::new(format!("{} is no folder", root.display())));
        }
        Ok(Self { root, extensions })
    }

    /// Whether `path` — relative, `/`-separated — can name a file of the
    /// folder: no empty, `.` or `..` segment, nothing a platform reads as
    /// a root, a drive or a stream (`/`, `\`, `:`), and an extension shown.
    pub fn names_file(&self, path: &str) -> bool {
        let segments_ok = (path.split('/'))
            .all(|s| !s.is_empty() && s != "." && s != ".." && !s.contains(['\\', ':', '\0']));
        let lower = path.to_ascii_lowercase();
        let extension_ok = (self.extensions.iter()).any(|e| lower.ends_with(&format!(".{e}")));
        segments_ok && extension_ok
    }

    /// The file `path` names under the folder — checked, before it is read
    /// or written, to stay in it: where it is already, its own canonical
    /// path; where it is not yet, that of the nearest folder above it
    /// that is. A link out of the folder is refused like `..`.
    pub fn file(&self, path: &str) -> GeopResult<PathBuf> {
        if !self.names_file(path) {
            return Err(GeopError::new(format!(
                "{path:?} names no file of the workspace"
            )));
        }
        let file = path.split('/').fold(self.root.clone(), |p, s| p.join(s));
        let existing = (file.ancestors())
            .find(|p| p.symlink_metadata().is_ok())
            .ok_or_else(|| GeopError::new(format!("{path:?}: nothing of it exists")))?;
        let canonical = std::fs::canonicalize(existing)
            .map_err(|e| GeopError::new(format!("{path:?}: {e}")))?;
        if !canonical.starts_with(&self.root) {
            return Err(GeopError::new(format!(
                "{path:?} lies outside the workspace"
            )));
        }
        Ok(file)
    }

    /// Every file of the folder, by path, its text read as for a compile
    /// (what is not UTF-8 replaced). Hidden folders, `node_modules` and
    /// `target` are left out, and so is a link out of the folder.
    pub fn files(&self) -> GeopResult<BTreeMap<String, String>> {
        let mut files = BTreeMap::new();
        let mut bytes = 0;
        let mut folders = vec![(self.root.clone(), String::new())];
        while let Some((dir, prefix)) = folders.pop() {
            let entries = std::fs::read_dir(&dir)
                .map_err(|e| GeopError::new(format!("listing {}: {e}", dir.display())))?;
            for entry in entries.flatten() {
                let Some(name) = entry.file_name().to_str().map(String::from) else {
                    continue;
                };
                let path = format!("{prefix}{name}");
                let Ok(kind) = entry.file_type() else {
                    continue;
                };
                if kind.is_dir() {
                    if !name.starts_with('.') && name != "node_modules" && name != "target" {
                        folders.push((entry.path(), format!("{path}/")));
                    }
                    continue;
                }
                // A link to a file is followed only within the folder; to
                // a folder, never (it could lead round in a circle).
                let Ok(file) = self.file(&path) else { continue };
                if !file.is_file() {
                    continue;
                }
                let text = std::fs::read(&file)
                    .map_err(|e| GeopError::new(format!("reading {}: {e}", file.display())))?;
                bytes += text.len() as u64;
                files.insert(path, String::from_utf8_lossy(&text).into_owned());
                if files.len() > MOST_FILES || bytes > MOST_BYTES {
                    return Err(GeopError::new(format!(
                        "{} holds more than {MOST_FILES} files or {} MB of programs and data: open a folder of one project",
                        self.root.display(),
                        MOST_BYTES >> 20
                    )));
                }
            }
        }
        Ok(files)
    }

    /// Write `text` to the file `path`, whole, then put it in place: a
    /// reader meanwhile sees the old file or the new, never half of one.
    pub fn write(&self, path: &str, text: &[u8]) -> GeopResult<()> {
        let file = self.file(path)?;
        let io = |e: std::io::Error| GeopError::new(format!("writing {}: {e}", file.display()));
        let dir = file.parent().expect("a file of the folder is in it");
        std::fs::create_dir_all(dir).map_err(io)?;
        let name = file
            .file_name()
            .expect("a file has a name")
            .to_string_lossy();
        let partial = dir.join(format!(".{name}.{}.partial", std::process::id()));
        let written =
            std::fs::write(&partial, text).and_then(|()| std::fs::rename(&partial, &file));
        if written.is_err() {
            let _ = std::fs::remove_file(&partial);
        }
        written.map_err(io)
    }

    /// Remove the file `path`; one already gone is no error.
    pub fn remove(&self, path: &str) -> GeopResult<()> {
        let file = self.file(path)?;
        match std::fs::remove_file(&file) {
            Err(e) if e.kind() != std::io::ErrorKind::NotFound => {
                Err(GeopError::new(format!("removing {}: {e}", file.display())))
            }
            _ => Ok(()),
        }
    }
}

// ── who may ask ────────────────────────────────────────────────────────

/// Who this server answers: requests naming it as their host, from its own
/// pages, and — for the editor and the files — knowing the token.
pub struct Access {
    pub port: u16,
    pub token: String,
}

/// How a request is let in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Route {
    /// A file of the page: no secret in it, so no token.
    Page,
    /// The file API: the token in an `Authorization: Bearer` header.
    Api,
    /// The WebSocket: the token in the query (a browser cannot give a
    /// WebSocket headers), and an `Origin`, which a browser always sends.
    Socket,
}

/// What a request says of where it comes from.
#[derive(Default)]
pub struct Claims<'a> {
    pub host: Option<&'a str>,
    pub origin: Option<&'a str>,
    /// The token it gives, from the header or the query its route takes.
    pub token: Option<&'a str>,
}

impl Access {
    /// A token no other page can guess: 256 random bits.
    pub fn new(port: u16) -> GeopResult<Self> {
        let mut bytes = [0u8; 32];
        getrandom::fill(&mut bytes).map_err(|e| GeopError::new(format!("making a token: {e}")))?;
        let token = bytes.iter().map(|b| format!("{b:02x}")).collect();
        Ok(Self { port, token })
    }

    /// Where the editor is, the token in the fragment: never sent to the
    /// server with the page's own files, nor to another site as a referrer.
    pub fn url(&self) -> String {
        format!("http://127.0.0.1:{}/#token={}", self.port, self.token)
    }

    /// The names a request may give this server by: `127.0.0.1` and
    /// `localhost`, at its port. Any other is a page of another name that
    /// resolved here.
    fn is_this(&self, host: &str) -> bool {
        let port = self.port.to_string();
        host.rsplit_once(':')
            .is_some_and(|(name, p)| p == port && (name == "127.0.0.1" || name == "localhost"))
    }

    /// Why `claims` may not take `route`, if they may not.
    pub fn refusal(&self, route: Route, claims: &Claims) -> Option<&'static str> {
        if !claims.host.is_some_and(|h| self.is_this(h)) {
            return Some("this server answers to 127.0.0.1 and localhost only");
        }
        let origin_ok =
            |origin: &str| (origin.strip_prefix("http://")).is_some_and(|host| self.is_this(host));
        match (route, claims.origin) {
            (Route::Socket, None) => return Some("a WebSocket must say its origin"),
            (_, Some(origin)) if !origin_ok(origin) => {
                return Some("requests from other sites are refused");
            }
            _ => {}
        }
        let token_ok = claims
            .token
            .is_some_and(|t| same(t.as_bytes(), self.token.as_bytes()));
        (route != Route::Page && !token_ok).then_some("the token is missing or wrong")
    }
}

/// Whether `a` and `b` are equal, in a time that does not tell how much of
/// them is.
fn same(a: &[u8], b: &[u8]) -> bool {
    a.len() == b.len() && a.iter().zip(b).fold(0, |d, (x, y)| d | (x ^ y)) == 0
}

/// The value of `key` in the query `query` (`a=1&b=2`), percent-decoded.
pub fn query_value(query: &str, key: &str) -> Option<String> {
    query.split('&').find_map(|pair| {
        let (k, v) = pair.split_once('=').unwrap_or((pair, ""));
        (k == key).then(|| percent_decoded(v)).flatten()
    })
}

/// `text` with every `%XX` and `+` decoded, if it is UTF-8 then.
pub fn percent_decoded(text: &str) -> Option<String> {
    let mut bytes = Vec::with_capacity(text.len());
    let mut rest = text.bytes();
    while let Some(b) = rest.next() {
        match b {
            b'%' => {
                let hex = [rest.next()?, rest.next()?];
                bytes.push(u8::from_str_radix(std::str::from_utf8(&hex).ok()?, 16).ok()?);
            }
            b'+' => bytes.push(b' '),
            b => bytes.push(b),
        }
    }
    String::from_utf8(bytes).ok()
}

// ── the editor ─────────────────────────────────────────────────────────

/// A command to the session, and where its answer goes.
type Call = (String, mpsc::Sender<String>);

/// The editor's session, answering the commands sent to it in the order
/// they come, from whichever sender. Cloned for every connection.
#[derive(Clone)]
pub struct SessionHandle(mpsc::Sender<Call>);

impl SessionHandle {
    /// A thread owning a session that keeps what files are made into under
    /// `root`. It ends once every handle is dropped.
    pub fn start(root: PathBuf) -> Self {
        let (sender, calls) = mpsc::channel::<Call>();
        std::thread::spawn(move || {
            let mut session = Session::new(Some(root));
            for (command, answer) in calls {
                let answered = session.answer(&command);
                // A kernel that panicked is replaced; the front end gives
                // the new one the program again (`backend.local.ts`).
                if answered.crashed {
                    session.restart();
                }
                let _ = answer.send(answered.reply);
            }
        });
        Self(sender)
    }

    /// The session's answer to `command`.
    pub fn call(&self, command: String) -> String {
        let (sender, answer) = mpsc::channel();
        let gone = || serde_json::json!({ "fatal": "the editor stopped" }).to_string();
        if self.0.send((command, sender)).is_err() {
            return gone();
        }
        answer.recv().unwrap_or_else(|_| gone())
    }
}

// ── serving ────────────────────────────────────────────────────────────

/// What every request is answered with.
struct Server {
    access: Access,
    folder: Folder,
    /// The program to edit first, if one was named.
    active: Option<String>,
    session: SessionHandle,
}

type Response = tiny_http::Response<std::io::Cursor<Vec<u8>>>;

fn header(name: &str, value: &str) -> tiny_http::Header {
    tiny_http::Header::from_bytes(name.as_bytes(), value.as_bytes())
        .expect("a header of ASCII text")
}

fn answer(status: u16, media_type: &str, body: Vec<u8>) -> Response {
    tiny_http::Response::from_data(body)
        .with_status_code(status)
        .with_header(header("Content-Type", media_type))
        .with_header(header("X-Content-Type-Options", "nosniff"))
        .with_header(header("Referrer-Policy", "no-referrer"))
        .with_header(header("X-Frame-Options", "DENY"))
        .with_header(header("Content-Security-Policy", "frame-ancestors 'none'"))
        .with_header(header("Cache-Control", "no-cache"))
}

fn text(status: u16, message: &str) -> Response {
    answer(
        status,
        "text/plain; charset=utf-8",
        message.as_bytes().to_vec(),
    )
}

fn json(value: serde_json::Value) -> Response {
    answer(200, "application/json", value.to_string().into_bytes())
}

impl Server {
    fn handle(&self, mut request: tiny_http::Request) {
        let get = |name: &'static str| {
            (request.headers().iter())
                .find(|h| h.field.equiv(name))
                .map(|h| h.value.as_str().to_string())
        };
        let (host, origin) = (get("Host"), get("Origin"));
        let authorization = get("Authorization");
        let upgrade = get("Upgrade");
        let key = get("Sec-WebSocket-Key");
        let url = request.url().to_string();
        let (path, query) = url.split_once('?').unwrap_or((&url, ""));
        let route = match path {
            "/ws" => Route::Socket,
            p if p.starts_with("/api/") => Route::Api,
            _ => Route::Page,
        };
        let query_token = query_value(query, "token");
        let token = match route {
            Route::Socket => query_token.as_deref(),
            _ => authorization
                .as_deref()
                .and_then(|a| a.strip_prefix("Bearer ")),
        };
        let claims = Claims {
            host: host.as_deref(),
            origin: origin.as_deref(),
            token,
        };
        if let Some(why) = self.access.refusal(route, &claims) {
            let _ = request.respond(text(403, why));
            return;
        }
        use tiny_http::Method::*;
        let response = match (route, request.method(), path) {
            (Route::Socket, Get, _) => {
                let (Some("websocket"), Some(key)) = (upgrade.as_deref(), key) else {
                    let _ = request.respond(text(400, "a WebSocket upgrade is expected"));
                    return;
                };
                let accept = tungstenite::handshake::derive_accept_key(key.as_bytes());
                let switching = tiny_http::Response::empty(101)
                    .with_header(header("Upgrade", "websocket"))
                    .with_header(header("Connection", "Upgrade"))
                    .with_header(header("Sec-WebSocket-Accept", &accept));
                let stream = request.upgrade("websocket", switching);
                socket(stream, &self.session);
                return;
            }
            (Route::Api, Get, "/api/files") => match self.folder.files() {
                Ok(files) => json(serde_json::json!({
                    "root": self.folder.root.display().to_string(),
                    "files": files,
                    "active": self.active,
                })),
                Err(e) => text(500, e.root_message()),
            },
            (Route::Api, method @ (Put | Delete), "/api/file") => {
                let Some(file) = query_value(query, "path") else {
                    let _ = request.respond(text(400, "which file? `?path=` is missing"));
                    return;
                };
                let done = if *method == Put {
                    let mut body = Vec::new();
                    let read = (request.as_reader().take(MOST_BYTES + 1))
                        .read_to_end(&mut body)
                        .map_err(|e| GeopError::new(format!("reading {file}: {e}")));
                    read.and_then(|_| self.folder.write(&file, &body))
                } else {
                    self.folder.remove(&file)
                };
                match done {
                    Ok(()) => text(200, "done"),
                    Err(e) => text(400, e.root_message()),
                }
            }
            (Route::Api, _, _) => text(404, "no such call"),
            (Route::Page, Get | Head, _) => {
                let file = path.trim_start_matches('/');
                let file = if file.is_empty() { "index.html" } else { file };
                match page_file(file) {
                    Some(data) => answer(200, media_type(file), data.into_owned()),
                    None => text(404, "not found"),
                }
            }
            _ => text(405, "not allowed"),
        };
        let _ = request.respond(response);
    }
}

/// Answer the commands of one WebSocket until it closes.
fn socket(stream: Box<dyn tiny_http::ReadWrite + Send>, session: &SessionHandle) {
    use tungstenite::{Message, protocol::Role, protocol::WebSocketConfig};
    // A command sends the folder's files, a STEP file among them perhaps:
    // as large as the folder may be.
    let size = MOST_BYTES as usize + (1 << 20);
    let config = WebSocketConfig::default()
        .max_message_size(Some(size))
        .max_frame_size(Some(size));
    let mut socket = tungstenite::WebSocket::from_raw_socket(stream, Role::Server, Some(config));
    loop {
        match socket.read() {
            Ok(Message::Text(command)) => {
                let reply = session.call(command.to_string());
                if socket.send(Message::text(reply)).is_err() {
                    return;
                }
            }
            Ok(Message::Close(_)) | Err(_) => return,
            Ok(_) => {}
        }
    }
}

/// Open the default browser at `url`.
fn open_browser(url: &str) -> std::io::Result<()> {
    let mut command = if cfg!(target_os = "macos") {
        std::process::Command::new("open")
    } else if cfg!(windows) {
        let mut start = std::process::Command::new("cmd");
        start.args(["/C", "start", ""]);
        start
    } else {
        std::process::Command::new("xdg-open")
    };
    command
        .arg(url)
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::null())
        .spawn()
        .map(drop)
}

/// The folder `path` names, and the program in it to edit first if it
/// names one.
fn workspace_of(path: &Path) -> GeopResult<(PathBuf, Option<String>)> {
    if path.is_dir() {
        return Ok((path.to_path_buf(), None));
    }
    let is_program = path.extension().is_some_and(|e| e == "geop");
    let name = path.file_name().and_then(|n| n.to_str());
    match (is_program && path.is_file(), name) {
        (true, Some(name)) => {
            let dir = path.parent().filter(|p| !p.as_os_str().is_empty());
            Ok((
                dir.unwrap_or(Path::new(".")).to_path_buf(),
                Some(name.to_string()),
            ))
        }
        _ => Err(GeopError::new(format!(
            "{} is neither a folder nor a .geop program",
            path.display()
        ))),
    }
}

pub fn open(args: &OpenArgs) -> GeopResult<()> {
    if page_file("index.html").is_none() {
        return Err(GeopError::new(
            "this geop was built without the editor's page. Build it with the page in: \
             `npm --prefix web run build:open`, then \
             `cargo build --release -p geop-cad-cli --features frontend`",
        ));
    }
    let (dir, active) = workspace_of(&args.path)?;
    let mut extensions: Vec<String> = vec!["geop".into()];
    for listed in geop_cad_base::registry()?.infos() {
        extensions.extend(listed.info.reads.iter().map(|e| e.to_ascii_lowercase()));
    }
    extensions.sort();
    extensions.dedup();
    let folder = Folder::new(&dir, extensions)?;

    let http = tiny_http::Server::http(("127.0.0.1", args.port.unwrap_or(0)))
        .map_err(|e| GeopError::new(format!("serving on 127.0.0.1: {e}")))?;
    let port = (http.server_addr().to_ip())
        .map(|a| a.port())
        .ok_or_else(|| GeopError::new("serving on no port"))?;
    let server = std::sync::Arc::new(Server {
        access: Access::new(port)?,
        session: SessionHandle::start(folder.root.clone()),
        folder,
        active,
    });
    let url = server.access.url();
    eprintln!("geop is editing {}", server.folder.root.display());
    eprintln!("open the editor at {url}");
    eprintln!("(Ctrl+C stops it)");
    if !args.no_browser && open_browser(&url).is_err() {
        eprintln!("could not open a browser: open the URL above yourself");
    }
    for request in http.incoming_requests() {
        let server = server.clone();
        std::thread::spawn(move || server.handle(request));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("geop-open-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn folder(dir: &Path) -> Folder {
        Folder::new(dir, vec!["geop".into(), "step".into(), "stp".into()]).unwrap()
    }

    #[test]
    fn a_path_names_a_file_of_the_folder_only_relatively_and_by_a_shown_extension() {
        let dir = scratch("names");
        let f = folder(&dir);
        for ok in ["part.geop", "parts/bolt.geop", "a b/c.STEP", "x.stp"] {
            assert!(f.names_file(ok), "{ok}");
        }
        for bad in [
            "",
            "/etc/passwd.geop",
            "../up.geop",
            "a/../../up.geop",
            "./part.geop",
            "a//b.geop",
            "a\\..\\b.geop",
            "C:/x.geop",
            "c:x.geop",
            "part.geop:stream",
            "notes.txt",
            "part.geop/",
            "part.geopc",
        ] {
            assert!(!f.names_file(bad), "{bad}");
            assert!(f.file(bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn files_are_read_and_written_in_the_folder() {
        let dir = scratch("rw");
        let f = folder(&dir);
        f.write("parts/bolt.geop", b"{\"steps\": []}\n").unwrap();
        f.write("part.geop", b"{}").unwrap();
        std::fs::write(dir.join("notes.txt"), "not shown").unwrap();
        std::fs::create_dir_all(dir.join(".git")).unwrap();
        std::fs::write(dir.join(".git/hidden.geop"), "{}").unwrap();
        let files = f.files().unwrap();
        assert_eq!(
            files.keys().collect::<Vec<_>>(),
            ["part.geop", "parts/bolt.geop"]
        );
        assert_eq!(files["parts/bolt.geop"], "{\"steps\": []}\n");
        f.remove("part.geop").unwrap();
        f.remove("part.geop").unwrap();
        assert!(!dir.join("part.geop").exists());
        // No partial file is left behind.
        let left: Vec<_> = std::fs::read_dir(dir.join("parts")).unwrap().collect();
        assert_eq!(left.len(), 1);
    }

    #[cfg(unix)]
    #[test]
    fn a_link_out_of_the_folder_is_refused() {
        let dir = scratch("links");
        let outside = scratch("links-outside");
        std::fs::write(outside.join("secret.geop"), "secret").unwrap();
        let root = dir.join("root");
        std::fs::create_dir_all(&root).unwrap();
        std::fs::write(root.join("inside.geop"), "inside").unwrap();
        std::os::unix::fs::symlink(&outside, root.join("out")).unwrap();
        std::os::unix::fs::symlink(outside.join("secret.geop"), root.join("secret.geop")).unwrap();
        std::os::unix::fs::symlink(root.join("inside.geop"), root.join("alias.geop")).unwrap();
        let f = folder(&root);
        assert!(f.file("out/secret.geop").is_err());
        assert!(f.file("out/new/new.geop").is_err());
        assert!(f.file("secret.geop").is_err());
        assert!(f.write("out/written.geop", b"x").is_err());
        assert!(!outside.join("written.geop").exists());
        assert!(f.write("secret.geop", b"x").is_err());
        assert_eq!(
            std::fs::read_to_string(outside.join("secret.geop")).unwrap(),
            "secret"
        );
        assert!(f.remove("secret.geop").is_err());
        assert!(outside.join("secret.geop").exists());
        // Listed: what is in the folder, a link within it included.
        let files = f.files().unwrap();
        assert_eq!(
            files.keys().collect::<Vec<_>>(),
            ["alias.geop", "inside.geop"]
        );
    }

    fn access() -> Access {
        Access {
            port: 4321,
            token: "t0k3n".into(),
        }
    }

    #[test]
    fn a_request_must_name_this_server_as_its_host() {
        let a = access();
        let page = |host| {
            a.refusal(
                Route::Page,
                &Claims {
                    host,
                    ..Default::default()
                },
            )
        };
        assert_eq!(page(Some("127.0.0.1:4321")), None);
        assert_eq!(page(Some("localhost:4321")), None);
        assert!(page(Some("127.0.0.1:4322")).is_some());
        assert!(page(Some("evil.example:4321")).is_some());
        assert!(page(Some("127.0.0.1.evil.example:4321")).is_some());
        assert!(page(Some("127.0.0.1")).is_some());
        assert!(page(None).is_some());
    }

    #[test]
    fn the_api_and_the_socket_take_the_token_and_this_origin_only() {
        let a = access();
        let host = Some("127.0.0.1:4321");
        let ask = |route, origin, token| {
            a.refusal(
                route,
                &Claims {
                    host,
                    origin,
                    token,
                },
            )
        };
        let here = Some("http://127.0.0.1:4321");
        assert_eq!(ask(Route::Api, None, Some("t0k3n")), None);
        assert_eq!(ask(Route::Api, here, Some("t0k3n")), None);
        assert_eq!(ask(Route::Socket, here, Some("t0k3n")), None);
        assert_eq!(
            ask(Route::Socket, Some("http://localhost:4321"), Some("t0k3n")),
            None
        );
        assert!(ask(Route::Api, None, None).is_some());
        assert!(ask(Route::Api, None, Some("t0k3")).is_some());
        assert!(ask(Route::Api, None, Some("t0k3n!")).is_some());
        assert!(ask(Route::Socket, None, Some("t0k3n")).is_some());
        assert!(ask(Route::Socket, here, None).is_some());
        assert!(ask(Route::Socket, Some("https://evil.example"), Some("t0k3n")).is_some());
        assert!(ask(Route::Api, Some("http://evil.example:4321"), Some("t0k3n")).is_some());
        assert!(ask(Route::Page, Some("null"), None).is_some());
        assert_eq!(ask(Route::Page, here, None), None);
    }

    #[test]
    fn every_token_is_new_and_in_the_fragment() {
        let (a, b) = (Access::new(80).unwrap(), Access::new(80).unwrap());
        assert_ne!(a.token, b.token);
        assert_eq!(a.token.len(), 64);
        assert_eq!(a.url(), format!("http://127.0.0.1:80/#token={}", a.token));
    }

    #[test]
    fn queries_are_percent_decoded() {
        assert_eq!(
            query_value("token=abc&path=parts%2Fa%20b.geop", "path").as_deref(),
            Some("parts/a b.geop")
        );
        assert_eq!(
            query_value("path=a+b.geop", "path").as_deref(),
            Some("a b.geop")
        );
        assert_eq!(query_value("token=abc", "path"), None);
        assert_eq!(query_value("path=%2", "path"), None);
        assert_eq!(query_value("path=%ff", "path"), None);
        assert_eq!(percent_decoded("%2e%2E%2fx").as_deref(), Some("../x"));
    }

    #[test]
    fn a_program_named_is_opened_in_its_folder() {
        let dir = scratch("workspace-of");
        std::fs::write(dir.join("a.geop"), "{}").unwrap();
        assert_eq!(workspace_of(&dir).unwrap(), (dir.clone(), None));
        assert_eq!(
            workspace_of(&dir.join("a.geop")).unwrap(),
            (dir.clone(), Some("a.geop".into()))
        );
        assert!(workspace_of(&dir.join("missing.geop")).is_err());
        assert!(workspace_of(&dir.join("missing")).is_err());
    }

    #[test]
    fn the_page_is_served_as_what_it_is() {
        assert_eq!(media_type("index.html"), "text/html; charset=utf-8");
        assert_eq!(
            media_type("assets/index-abc.js"),
            "text/javascript; charset=utf-8"
        );
        assert_eq!(media_type("favicon.SVG"), "image/svg+xml");
        assert_eq!(media_type("LICENSE"), "application/octet-stream");
    }

    /// One editor answers every connection, in order, and a command it
    /// cannot read is answered too.
    #[test]
    fn the_session_answers_each_command() {
        let dir = scratch("session");
        let session = SessionHandle::start(dir);
        let other = session.clone();
        let shown: serde_json::Value =
            serde_json::from_str(&other.call(r#"{"command": "show"}"#.into())).unwrap();
        assert!(shown["error"].is_null(), "{shown}");
        let fatal: serde_json::Value = serde_json::from_str(&session.call("nope".into())).unwrap();
        assert!(fatal["fatal"].is_string(), "{fatal}");
    }
}
