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
//! - `POST /api/raise` brings the window showing the page to the front:
//!   only in `geop app`, where it is how a second `geop app` of the same
//!   folder hands over to the first (see `app.rs`).
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
use geop_cad_base::disk::{Folder, MOST_BYTES, extensions, workspace_of};
use geop_core_math::geop_error::{GeopError, GeopResult};
use std::{
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

// ── who may ask ────────────────────────────────────────────────────────

/// Who this server answers: requests naming it as their host, from its own
/// pages, and — for the editor and the files — knowing the token.
#[derive(Clone)]
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

/// What a `POST /api/raise` does: bring the window showing the page to
/// the front (`geop app`, asked by another `geop app` of the same folder).
pub type Raise = Box<dyn Fn() + Send + Sync>;

/// What every request is answered with.
struct Server {
    access: Access,
    folder: Folder,
    /// The program to edit first, if one was named.
    active: Option<String>,
    session: SessionHandle,
    raise: Option<Raise>,
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
                    "root": self.folder.root().display().to_string(),
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
                    read.and_then(|_| self.folder.write(&file, &String::from_utf8_lossy(&body)))
                } else {
                    self.folder.remove(&file)
                };
                match done {
                    Ok(()) => text(200, "done"),
                    Err(e) => text(400, e.root_message()),
                }
            }
            (Route::Api, Post, "/api/raise") if self.raise.is_some() => {
                (self.raise.as_ref().expect("checked"))();
                text(200, "raised")
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
pub fn open_browser(url: &str) -> std::io::Result<()> {
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

/// Why this `geop` cannot serve the editor's page, if it cannot: it was
/// built without it.
pub fn page_missing() -> Option<GeopError> {
    page_file("index.html").is_none().then(|| {
        GeopError::new(
            "this geop was built without the editor's page. Build it with the page in: \
             `npm --prefix web run build:open`, then \
             `cargo build --release -p geop-cad-cli --features frontend`",
        )
    })
}

/// The folder `path` names — or the one a program `path` is in — with
/// the program to edit first, if one was named.
pub fn folder_of(path: &Path) -> GeopResult<(Folder, Option<String>)> {
    let (dir, active) = workspace_of(path)?;
    let folder = Folder::new(&dir, extensions(&geop_cad_base::registry()?))?;
    Ok((folder, active))
}

/// The editor of a folder, served on 127.0.0.1: bound, and answering
/// once [`Serving::run`] is.
pub struct Serving {
    http: tiny_http::Server,
    server: std::sync::Arc<Server>,
}

impl Serving {
    /// Serve `folder` on `port`, a free one if `None`, `active` the
    /// program to edit first; `raise` answers `POST /api/raise`, which
    /// is no call without it.
    pub fn start(
        folder: Folder,
        active: Option<String>,
        port: Option<u16>,
        raise: Option<Raise>,
    ) -> GeopResult<Self> {
        let http = tiny_http::Server::http(("127.0.0.1", port.unwrap_or(0)))
            .map_err(|e| GeopError::new(format!("serving on 127.0.0.1: {e}")))?;
        let port = (http.server_addr().to_ip())
            .map(|a| a.port())
            .ok_or_else(|| GeopError::new("serving on no port"))?;
        let server = std::sync::Arc::new(Server {
            access: Access::new(port)?,
            session: SessionHandle::start(folder.root().to_path_buf()),
            folder,
            active,
            raise,
        });
        Ok(Self { http, server })
    }

    /// Who may ask: the port, and the token the page is given.
    pub fn access(&self) -> &Access {
        &self.server.access
    }

    /// The folder edited, as its canonical path.
    pub fn root(&self) -> &Path {
        self.server.folder.root()
    }

    /// Answer every request, each on a thread of its own, for as long as
    /// the process runs.
    pub fn run(self) {
        for request in self.http.incoming_requests() {
            let server = self.server.clone();
            std::thread::spawn(move || server.handle(request));
        }
    }
}

pub fn open(args: &OpenArgs) -> GeopResult<()> {
    if let Some(missing) = page_missing() {
        return Err(missing);
    }
    let (folder, active) = folder_of(&args.path)?;
    let serving = Serving::start(folder, active, args.port, None)?;
    let url = serving.access().url();
    eprintln!("geop is editing {}", serving.root().display());
    eprintln!("open the editor at {url}");
    eprintln!("(Ctrl+C stops it)");
    if !args.no_browser && open_browser(&url).is_err() {
        eprintln!("could not open a browser: open the URL above yourself");
    }
    serving.run();
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
