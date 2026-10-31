//! `geop app [DIR | program.geop]`: the web editor in a window of its own,
//! editing the files of the folder `DIR` on the kernel of this process.
//!
//! It is `geop open` (see [`crate::open`]) without the browser: the same
//! page, served by the same server on 127.0.0.1, shown in the system's
//! webview (WebKitGTK, WKWebView, WebView2, through `wry` and `tao`) with
//! no browser chrome around it. Everything the editor does is the page's,
//! so it behaves as the web app does; what the window adds is only what a
//! browser tab cannot do:
//!
//! - its title is the page's, which names the program edited and the
//!   folder (`backend.local.ts`);
//! - a folder to edit is asked for in the system's dialog, when none is
//!   given and from the page's File menu ("Open folder…", which starts
//!   another `geop app`); a file chosen "from disk" is chosen in the
//!   system's dialog, which the webview shows for the page's file inputs;
//! - a file the page downloads (Save, an export) is saved where the
//!   system's save dialog says;
//! - a link out of the page opens in the default browser;
//! - its size is kept from one run to the next ([`WindowState`]);
//! - one app edits a folder: a second `geop app` of the same folder
//!   brings the first to the front and ends ([`Claim`]).
//!
//! What runs without a window — the server, and how one app finds the one
//! already editing its folder — is here, built in every `geop`; the window
//! needs the feature `app`.

// Built without the window, what it would run is only the tests'.
#![cfg_attr(not(feature = "app"), allow(dead_code, unused_imports))]

use crate::open::{Access, folder_of, page_missing};
use geop_core_math::geop_error::{GeopError, GeopResult};
use sha2::Digest;
use std::{
    io::{Read, Write},
    net::{SocketAddr, TcpStream},
    path::{Path, PathBuf},
    time::Duration,
};

#[derive(clap::Args)]
pub struct AppArgs {
    /// The folder to edit the files of — or a program in it, to edit
    /// first. Asked for in a dialog, if not given.
    pub path: Option<PathBuf>,
}

// ── one app per folder ─────────────────────────────────────────────────

/// Where the app editing `root` says how to reach it: a file per folder,
/// named by a digest of its canonical path, in the user's runtime folder.
fn claim_file(root: &Path) -> PathBuf {
    let digest = sha2::Sha256::digest(root.as_os_str().as_encoded_bytes());
    let name: String = digest[..8].iter().map(|b| format!("{b:02x}")).collect();
    let dir = std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .filter(|d| d.is_dir())
        .unwrap_or_else(std::env::temp_dir);
    dir.join(format!("geop-app-{name}.json"))
}

/// This app's claim on its folder: the port and token another `geop app`
/// of the folder raises it with (`POST /api/raise`). Readable by the user
/// alone, since the token opens the folder's files; given up when the app
/// ends ([`Claim::release`]).
pub struct Claim {
    file: PathBuf,
    text: String,
}

impl Claim {
    /// Claim `root` for the app `access` reaches.
    pub fn new(root: &Path, access: &Access) -> GeopResult<Self> {
        let file = claim_file(root);
        let text = serde_json::json!({ "port": access.port, "token": access.token }).to_string();
        let failed = |e: std::io::Error| {
            GeopError::new(format!(
                "claiming {}: writing {}: {e}",
                root.display(),
                file.display()
            ))
        };
        let _ = std::fs::remove_file(&file);
        let mut options = std::fs::OpenOptions::new();
        options.write(true).create_new(true);
        #[cfg(unix)]
        std::os::unix::fs::OpenOptionsExt::mode(&mut options, 0o600);
        options
            .open(&file)
            .and_then(|mut f| f.write_all(text.as_bytes()))
            .map_err(failed)?;
        Ok(Self { file, text })
    }

    /// Give the folder up, unless another app has claimed it since.
    pub fn release(self) {
        if std::fs::read_to_string(&self.file).is_ok_and(|t| t == self.text) {
            let _ = std::fs::remove_file(&self.file);
        }
    }
}

/// Bring the app editing `root` to the front, if one is: whether one was.
/// A claim whose app is gone (it crashed, or was killed) is no app.
pub fn raise_running(root: &Path) -> bool {
    let Ok(text) = std::fs::read_to_string(claim_file(root)) else {
        return false;
    };
    let Ok(claim) = serde_json::from_str::<serde_json::Value>(&text) else {
        return false;
    };
    let (Some(port), Some(token)) = (claim["port"].as_u64(), claim["token"].as_str()) else {
        return false;
    };
    let Ok(port) = u16::try_from(port) else {
        return false;
    };
    // By hand, not by an HTTP client that might send it through a proxy.
    let raised = || -> std::io::Result<bool> {
        let address = SocketAddr::from(([127, 0, 0, 1], port));
        let mut stream = TcpStream::connect_timeout(&address, Duration::from_secs(2))?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        write!(
            stream,
            "POST /api/raise HTTP/1.1\r\nHost: 127.0.0.1:{port}\r\n\
             Authorization: Bearer {token}\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
        )?;
        let mut status = [0u8; 12];
        stream.read_exact(&mut status)?;
        Ok(status.starts_with(b"HTTP/1.1 200") || status.starts_with(b"HTTP/1.0 200"))
    };
    raised().unwrap_or(false)
}

// ── what is kept between runs ──────────────────────────────────────────

/// The folder geop keeps its settings in: the platform's for each user.
fn config_dir() -> Option<PathBuf> {
    let var = |name| std::env::var_os(name).map(PathBuf::from);
    let base = if cfg!(windows) {
        var("APPDATA")
    } else if cfg!(target_os = "macos") {
        var("HOME").map(|h| h.join("Library/Application Support"))
    } else {
        var("XDG_CONFIG_HOME").or_else(|| var("HOME").map(|h| h.join(".config")))
    };
    base.map(|b| b.join("geop"))
}

/// The window's size as the last app left it, in logical pixels.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindowState {
    pub width: f64,
    pub height: f64,
    pub maximized: bool,
}

impl Default for WindowState {
    fn default() -> Self {
        Self {
            width: 1280.0,
            height: 800.0,
            maximized: false,
        }
    }
}

impl WindowState {
    /// The smallest window the editor's panels fit in.
    const SMALLEST: f64 = 480.0;

    fn file() -> Option<PathBuf> {
        config_dir().map(|d| d.join("app-window.json"))
    }

    /// The state `text` holds, if it holds one of a usable size.
    fn read(text: &str) -> Option<Self> {
        let value: serde_json::Value = serde_json::from_str(text).ok()?;
        let state = Self {
            width: value["width"].as_f64()?,
            height: value["height"].as_f64()?,
            maximized: value["maximized"].as_bool().unwrap_or(false),
        };
        let usable = |x: f64| x.is_finite() && (Self::SMALLEST..=100_000.0).contains(&x);
        (usable(state.width) && usable(state.height)).then_some(state)
    }

    fn text(&self) -> String {
        serde_json::json!({
            "width": self.width,
            "height": self.height,
            "maximized": self.maximized,
        })
        .to_string()
    }

    /// The state kept, or the default.
    pub fn load() -> Self {
        (Self::file().and_then(|f| std::fs::read_to_string(f).ok()))
            .and_then(|t| Self::read(&t))
            .unwrap_or_default()
    }

    /// Keep this state for the next run; a failure only loses it.
    pub fn store(&self) {
        if let Some(file) = Self::file() {
            let _ = file.parent().map(std::fs::create_dir_all);
            let _ = std::fs::write(file, self.text());
        }
    }
}

// ── the app ────────────────────────────────────────────────────────────

/// The page's own address: what it may navigate to in the window. Any
/// other is a link out of it, for the browser.
fn is_page(url: &str, access: &Access) -> bool {
    let origin = |host: &str| format!("http://{host}:{}/", access.port);
    url.starts_with(&origin("127.0.0.1"))
        || url.starts_with(&origin("localhost"))
        || ["blob:", "data:", "about:"]
            .iter()
            .any(|s| url.starts_with(s))
}

/// Whether `url` is one for the browser: a page on the web or a mail.
fn is_for_browser(url: &str) -> bool {
    ["https://", "http://", "mailto:"]
        .iter()
        .any(|s| url.starts_with(s))
}

#[cfg(not(feature = "app"))]
pub fn run(_: &AppArgs) -> GeopResult<()> {
    Err(GeopError::new(
        "this geop was built without its app. Build it with the app in: \
         `npm --prefix web run build:open`, then \
         `cargo build --release -p geop-cad-cli --features frontend,app` \
         (on Linux, with WebKitGTK: `sudo apt install libwebkit2gtk-4.1-dev`). \
         Or edit the folder in your browser: `geop open`",
    ))
}

#[cfg(feature = "app")]
pub fn run(args: &AppArgs) -> GeopResult<()> {
    if let Some(missing) = page_missing() {
        return Err(missing);
    }
    let path = match &args.path {
        Some(path) => path.clone(),
        None => match window::pick_folder(&std::env::current_dir().unwrap_or_default()) {
            Some(folder) => folder,
            None => return Ok(()),
        },
    };
    let (folder, active) = folder_of(&path)?;
    if raise_running(folder.root()) {
        eprintln!(
            "geop is editing {} already: brought its window to the front",
            folder.root().display()
        );
        return Ok(());
    }
    window::run(folder, active)
}

/// The window, in the system's webview.
#[cfg(feature = "app")]
mod window {
    use super::{Claim, WindowState, is_for_browser, is_page};
    use crate::open::{Serving, open_browser};
    use geop_cad_base::disk::Folder;
    use geop_core_math::geop_error::{GeopError, GeopResult};
    use std::{
        path::{Path, PathBuf},
        rc::Rc,
        sync::Mutex,
    };
    use tao::{
        dpi::LogicalSize,
        event::{Event, WindowEvent},
        event_loop::{ControlFlow, EventLoopBuilder},
        window::{Icon, Window, WindowBuilder},
    };
    use wry::{NewWindowResponse, WebViewBuilder};

    /// What reaches the window from elsewhere.
    #[derive(Debug)]
    enum AppEvent {
        /// Another `geop app` of this folder was started.
        Raise,
        /// The page's "Open folder…".
        OpenFolder,
    }

    /// What the page is told it is in: the File menu's "Open folder…"
    /// asks the window (`backend.local.ts`). And what goes wrong in the
    /// page, which a window shows no console of, is said in the terminal
    /// the app was started from, as `geop open`'s is in the browser's.
    const IN_APP: &str = r#"
window.geopApp = { openFolder: () => window.ipc.postMessage("open-folder") };
{
  const say = (text) => window.ipc.postMessage("error:" + text);
  const original = console.error.bind(console);
  console.error = (...args) => { original(...args); say(args.map(String).join(" ")); };
  window.addEventListener("error", (e) => say(`${e.message} (${e.filename}:${e.lineno})`));
  window.addEventListener("unhandledrejection", (e) => say(`unhandled: ${e.reason?.stack ?? e.reason}`));}
"#;

    /// A folder, asked for in the system's dialog, starting in `dir`.
    pub fn pick_folder(dir: &Path) -> Option<PathBuf> {
        rfd::FileDialog::new()
            .set_title("Open a folder in geop")
            .set_directory(dir)
            .pick_folder()
    }

    /// The window's icon: the logo the page shows, drawn at 256 pixels.
    fn icon() -> Option<Icon> {
        const LOGO: &str = include_str!("../../../web/src/assets/logo.svg");
        const SIZE: u32 = 256;
        let tree = resvg::usvg::Tree::from_str(LOGO, &Default::default()).ok()?;
        let mut pixmap = resvg::tiny_skia::Pixmap::new(SIZE, SIZE)?;
        let scale = SIZE as f32 / tree.size().width().max(tree.size().height());
        let transform = resvg::tiny_skia::Transform::from_scale(scale, scale);
        resvg::render(&tree, transform, &mut pixmap.as_mut());
        let rgba = (pixmap.pixels().iter())
            .flat_map(|p| {
                let c = p.demultiply();
                [c.red(), c.green(), c.blue(), c.alpha()]
            })
            .collect();
        Icon::from_rgba(rgba, SIZE, SIZE).ok()
    }

    /// Where the page's download `suggested` goes: asked in the system's
    /// save dialog, starting in the folder edited.
    fn save_as(window: &Window, root: &Path, suggested: &Path) -> Option<PathBuf> {
        let mut dialog = rfd::FileDialog::new()
            .set_title("Save")
            .set_directory(root)
            .set_parent(window);
        if let Some(name) = suggested.file_name().and_then(|n| n.to_str()) {
            dialog = dialog.set_file_name(name);
        }
        dialog.save_file()
    }

    /// Start another `geop app` on `folder`: it raises this one if
    /// `folder` is this one's.
    fn start_app(folder: &Path) -> GeopResult<()> {
        let exe = std::env::current_exe()
            .map_err(|e| GeopError::new(format!("finding geop itself: {e}")))?;
        std::process::Command::new(exe)
            .arg("app")
            .arg(folder)
            .stdin(std::process::Stdio::null())
            .spawn()
            .map(drop)
            .map_err(|e| GeopError::new(format!("starting geop app {}: {e}", folder.display())))
    }

    pub fn run(folder: Folder, active: Option<String>) -> GeopResult<()> {
        let event_loop = EventLoopBuilder::<AppEvent>::with_user_event().build();
        let proxy = Mutex::new(event_loop.create_proxy());
        let raise = Box::new(move || {
            let _ = proxy.lock().map(|p| p.send_event(AppEvent::Raise));
        });
        let serving = Serving::start(folder, active, None, Some(raise))?;
        let access = serving.access().clone();
        let root = serving.root().to_path_buf();
        let mut claim = Some(Claim::new(&root, &access)?);
        std::thread::spawn(move || serving.run());

        let state = WindowState::load();
        let window = WindowBuilder::new()
            .with_title("geop")
            .with_inner_size(LogicalSize::new(state.width, state.height))
            .with_maximized(state.maximized)
            .with_window_icon(icon())
            .build(&event_loop)
            .map_err(|e| GeopError::new(format!("opening a window: {e}")))?;
        let window = Rc::new(window);

        let builder = WebViewBuilder::new()
            .with_url(access.url())
            .with_initialization_script(IN_APP)
            .with_clipboard(true)
            .with_devtools(cfg!(debug_assertions))
            .with_document_title_changed_handler({
                let window = window.clone();
                move |title| window.set_title(&title)
            })
            .with_navigation_handler(move |url| {
                if is_page(&url, &access) {
                    return true;
                }
                if is_for_browser(&url) {
                    let _ = open_browser(&url);
                }
                false
            })
            .with_new_window_req_handler(|url, _| {
                if is_for_browser(&url) {
                    let _ = open_browser(&url);
                }
                NewWindowResponse::Deny
            })
            .with_download_started_handler({
                let (window, root) = (window.clone(), root.clone());
                move |_url, path| match save_as(&window, &root, path) {
                    Some(chosen) => {
                        *path = chosen;
                        true
                    }
                    None => false,
                }
            })
            .with_ipc_handler({
                let proxy = event_loop.create_proxy();
                move |request| {
                    let message = request.body();
                    if message == "open-folder" {
                        let _ = proxy.send_event(AppEvent::OpenFolder);
                    } else if let Some(error) = message.strip_prefix("error:") {
                        eprintln!("the editor's page: {error}");
                    }
                }
            });
        #[cfg(not(any(target_os = "windows", target_os = "macos")))]
        let webview = {
            use tao::platform::unix::WindowExtUnix;
            use wry::WebViewBuilderExtUnix;
            let vbox = (window.default_vbox())
                .ok_or_else(|| GeopError::new("opening a window: it has no GTK box"))?;
            builder.build_gtk(vbox)
        };
        #[cfg(any(target_os = "windows", target_os = "macos"))]
        let webview = builder.build(&*window);
        let webview = webview
            .map_err(|e| GeopError::new(format!("showing the editor in the window: {e}")))?;
        eprintln!("geop is editing {}", root.display());

        event_loop.run(move |event, _, control_flow| {
            *control_flow = ControlFlow::Wait;
            // Kept alive as long as the window is.
            let _ = &webview;
            match event {
                Event::UserEvent(AppEvent::Raise) => {
                    window.set_minimized(false);
                    window.set_visible(true);
                    window.set_focus();
                }
                Event::UserEvent(AppEvent::OpenFolder) => {
                    if let Some(folder) = pick_folder(&root)
                        && let Err(e) = start_app(&folder)
                    {
                        eprintln!("{}", e.root_message());
                    }
                }
                Event::WindowEvent {
                    event: WindowEvent::CloseRequested,
                    ..
                } => {
                    let size = window.inner_size().to_logical::<f64>(window.scale_factor());
                    let maximized = window.is_maximized();
                    // A maximized window keeps the size it is restored to.
                    let kept = WindowState::load();
                    let (width, height) = if maximized {
                        (kept.width, kept.height)
                    } else {
                        (size.width, size.height)
                    };
                    WindowState {
                        width,
                        height,
                        maximized,
                    }
                    .store();
                    *control_flow = ControlFlow::Exit;
                }
                Event::LoopDestroyed => {
                    if let Some(claim) = claim.take() {
                        claim.release();
                    }
                }
                _ => {}
            }
        })
    }
}

/// Serve `folder` as `geop app` does, without the window: what the tests
/// drive, as the window's page would.
#[cfg(test)]
fn serve_headless(
    folder: geop_cad_base::disk::Folder,
    raise: crate::open::Raise,
) -> GeopResult<(Access, Claim)> {
    let serving = crate::open::Serving::start(folder, None, None, Some(raise))?;
    let access = serving.access().clone();
    let claim = Claim::new(serving.root(), &access)?;
    std::thread::spawn(move || serving.run());
    Ok((access, claim))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };

    fn scratch(test: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("geop-app-{test}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// The server of the app answers the editor's protocol over its
    /// WebSocket and lists the folder's files, as `geop open`'s does; a
    /// second app of the folder raises it, and none raises it once it
    /// has given the folder up.
    #[test]
    fn the_app_serves_the_editor_and_is_raised_by_a_second_app() {
        let dir = scratch("serve");
        std::fs::write(dir.join("part.geop"), "{\"steps\": []}\n").unwrap();
        let (folder, _) = folder_of(&dir).unwrap();
        let root = folder.root().to_path_buf();
        let raised = Arc::new(AtomicUsize::new(0));
        let counter = raised.clone();
        let raise = Box::new(move || {
            counter.fetch_add(1, Ordering::SeqCst);
        });
        let (access, claim) = serve_headless(folder, raise).unwrap();
        let here = format!("127.0.0.1:{}", access.port);

        // The editor, as the page reaches it.
        use tungstenite::client::IntoClientRequest;
        let mut request = format!("ws://{here}/ws?token={}", access.token)
            .into_client_request()
            .unwrap();
        let origin = format!("http://{here}").parse().unwrap();
        request.headers_mut().insert("Origin", origin);
        let (mut socket, _) = tungstenite::connect(request).unwrap();
        socket
            .send(tungstenite::Message::text(r#"{"command": "show"}"#))
            .unwrap();
        let update = socket.read().unwrap().into_text().unwrap();
        let update: serde_json::Value = serde_json::from_str(&update).unwrap();
        assert!(update["error"].is_null(), "{update}");

        // The files, as the page lists them.
        let mut stream = TcpStream::connect(&here).unwrap();
        write!(
            stream,
            "GET /api/files HTTP/1.1\r\nHost: {here}\r\nAuthorization: Bearer {}\r\n\
             Connection: close\r\n\r\n",
            access.token
        )
        .unwrap();
        let mut answer = String::new();
        stream.read_to_string(&mut answer).unwrap();
        assert!(answer.starts_with("HTTP/1.1 200"), "{answer}");
        assert!(answer.contains("part.geop"), "{answer}");

        assert!(raise_running(&root));
        assert_eq!(raised.load(Ordering::SeqCst), 1);
        claim.release();
        assert!(!raise_running(&root));
        assert_eq!(raised.load(Ordering::SeqCst), 1);
    }

    /// A claim left by an app that is gone raises nothing: the next app
    /// takes the folder.
    #[test]
    fn a_claim_of_an_app_that_is_gone_raises_nothing() {
        let dir = scratch("gone");
        let root = std::fs::canonicalize(&dir).unwrap();
        // A port nothing listens on: one just let go of.
        let port = std::net::TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let claim = Claim::new(&root, &Access::new(port).unwrap()).unwrap();
        assert!(!raise_running(&root));
        claim.release();
        assert!(!claim_file(&root).exists());
    }

    #[test]
    fn each_folder_is_claimed_apart() {
        assert_ne!(claim_file(Path::new("/a")), claim_file(Path::new("/b")));
        assert_eq!(claim_file(Path::new("/a")), claim_file(Path::new("/a")));
    }

    #[test]
    fn the_window_size_kept_is_read_back_and_a_useless_one_is_not() {
        let state = WindowState {
            width: 1440.0,
            height: 900.5,
            maximized: true,
        };
        assert_eq!(WindowState::read(&state.text()), Some(state));
        assert_eq!(WindowState::read(r#"{"width": 10, "height": 900}"#), None);
        assert_eq!(WindowState::read(r#"{"width": 1e9, "height": 900}"#), None);
        assert_eq!(WindowState::read("not json"), None);
    }

    #[test]
    fn links_out_of_the_page_go_to_the_browser() {
        let access = Access {
            port: 4321,
            token: "t".into(),
        };
        assert!(is_page("http://127.0.0.1:4321/#token=t", &access));
        assert!(is_page("http://localhost:4321/assets/a.js", &access));
        assert!(is_page("blob:http://127.0.0.1:4321/x", &access));
        assert!(!is_page("http://127.0.0.1:4322/", &access));
        assert!(!is_page("https://discord.gg/btRDgrcwJ5", &access));
        assert!(is_for_browser("https://github.com/geop-cad/geop"));
        assert!(!is_for_browser("file:///etc/passwd"));
        assert!(!is_for_browser("javascript:alert(1)"));
    }
}
