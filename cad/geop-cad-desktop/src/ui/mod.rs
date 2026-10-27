//! The window: egui panels around a wgpu viewport, drawing what the
//! [`Controller`] holds and turning every click, key and drag into a call
//! of it. Laid out as the web app is — the operations' ribbon on top, the
//! program's steps and the files on the left, the viewport in the middle,
//! the step's dialog (or, with none open, the part's structure and the
//! inspectors) on the right, a status bar under it all — with a command
//! palette (Ctrl+K) over everything, listing every operation and action.

pub mod actions;
mod dialog;
mod explorer;
pub mod icons;
mod palette;
mod panels;
mod ribbon;
mod sidebar;
mod theme;
mod timeline;
mod viewport;

use std::{
    path::PathBuf,
    sync::{Arc, mpsc},
};

use eframe::egui::{self, Event, RichText};

use crate::{
    camera::Projection,
    controller::{Chosen, Controller, Effect, Section, SectionAxis, export_bytes},
    draw::Statics,
};
use actions::Action;

pub use theme::Theme;

/// What the window keeps beside the controller: how it is laid out, and
/// what is under way in it.
pub struct GeopApp {
    pub app: Controller,
    pub theme: Theme,
    /// Whether the files are shown, left of the steps.
    pub files_shown: bool,
    pub palette: palette::Palette,
    pub shortcuts_open: bool,
    view: viewport::ViewState,
    statics: Arc<Statics>,
    chooser: Option<explorer::Chooser>,
    explorer: explorer::ExplorerState,
    /// Files chosen from disk for a dialog's file field, read off the
    /// window's thread.
    chosen: (mpsc::Sender<Chosen>, mpsc::Receiver<Chosen>),
    /// A folder chosen to open.
    folders: (mpsc::Sender<PathBuf>, mpsc::Receiver<PathBuf>),
    /// Files chosen from disk to add to the workspace, read.
    added: (
        mpsc::Sender<Vec<(String, String)>>,
        mpsc::Receiver<Vec<(String, String)>>,
    ),
    /// What was said of a file saved, or why it was not.
    notes: (mpsc::Sender<String>, mpsc::Receiver<String>),
    pub note: Option<(String, f64)>,
    /// The window's title, as last set.
    title: String,
    /// The theme egui was last set to.
    applied: Option<Theme>,
    wake: Arc<dyn Fn() + Send + Sync>,
}

impl GeopApp {
    pub fn new(app: Controller, wake: Arc<dyn Fn() + Send + Sync>) -> Self {
        Self {
            app,
            theme: Theme::Dark,
            files_shown: true,
            palette: palette::Palette::default(),
            shortcuts_open: false,
            view: viewport::ViewState::default(),
            statics: Arc::new(Statics::default()),
            chooser: None,
            explorer: explorer::ExplorerState::default(),
            chosen: mpsc::channel(),
            folders: mpsc::channel(),
            added: mpsc::channel(),
            notes: mpsc::channel(),
            note: None,
            title: String::new(),
            applied: None,
            wake,
        }
    }

    /// Do `action`.
    pub fn act(&mut self, ctx: &egui::Context, action: Action) {
        if !action.enabled(&self.app) {
            return;
        }
        let app = &mut self.app;
        match action {
            Action::NewStep(kind) => app.new_step(&kind),
            Action::Commit => app.commit(),
            Action::Cancel => app.cancel(),
            Action::Undo => app.undo(),
            Action::Redo => app.redo(),
            Action::CopyStep => {
                if let Some(id) = app.step.as_ref().and_then(|s| s.id.clone()) {
                    app.copy_step(&id);
                }
            }
            Action::Paste => ctx.send_viewport_cmd(egui::ViewportCommand::RequestPaste),
            Action::Fit => app.fit(None),
            Action::ResetView => app.reset_view(),
            Action::ToggleProjection => {
                app.camera.projection = match app.camera.projection {
                    Projection::Perspective => Projection::Orthographic,
                    Projection::Orthographic => Projection::Perspective,
                }
            }
            Action::ToggleTheme => self.theme = self.theme.other(),
            Action::ToggleFiles => self.files_shown = !self.files_shown,
            Action::ToggleReduced => app.reduced = !app.reduced,
            Action::TogglePreview => {
                let preview = app.step.as_ref().is_some_and(|s| s.preview);
                app.command(geop_core_editor::editor::Command::Preview { preview: !preview });
            }
            Action::NewPart => app.new_part(),
            Action::OpenFolder => self.pick_folder(),
            Action::Reread => app.reread(),
            Action::OpenFile(path) => app.open_file(&path),
            Action::OpenStep(id) => app.open_step(&id, None),
            Action::LoadExample(name) => app.load_example(&name),
            Action::LoadWorkspaceExample(name) => app.load_workspace_example(&name),
            Action::Export(exporter) => {
                app.export(&exporter, None, serde_json::Value::Null, today())
            }
            Action::Inspect(inspector) => app.inspect(&inspector, serde_json::Value::Null),
            Action::Tool(key) => app.command(geop_core_editor::editor::Command::Tool { key }),
            Action::ToggleSection => {
                app.section = match app.section {
                    Some(_) => None,
                    None => Some(Section {
                        axis: SectionAxis::Z,
                        offset: 0.0,
                        flip: false,
                        plane: None,
                    }),
                }
            }
            Action::Palette => self.palette.toggle(),
            Action::Shortcuts => self.shortcuts_open = !self.shortcuts_open,
        }
    }

    /// The hotkeys, and what the keyboard sends the step being edited.
    fn keys(&mut self, ctx: &egui::Context) {
        let typing = ctx.egui_wants_keyboard_input();
        let mut done = Vec::new();
        ctx.input_mut(|input| {
            for action in Action::with_shortcuts() {
                // Typing, a field's own keys are its: undo, a word deleted.
                let while_typing = matches!(action, Action::Palette | Action::Commit);
                if let Some(shortcut) = action.shortcut()
                    && (!typing || while_typing)
                    && input.consume_shortcut(&shortcut)
                {
                    done.push(action);
                }
            }
            if !typing && input.consume_key(egui::Modifiers::COMMAND, egui::Key::Y) {
                done.push(Action::Redo);
            }
        });
        let events = ctx.input(|i| i.events.clone());
        for action in done {
            self.act(ctx, action);
        }
        if typing || self.palette.open || self.chooser.is_some() {
            return;
        }
        for event in events {
            match event {
                Event::Copy => self.act(ctx, Action::CopyStep),
                Event::Paste(text) if !text.is_empty() => self.app.paste(&text),
                Event::Key {
                    key,
                    pressed: true,
                    modifiers,
                    ..
                } if !modifiers.command && !modifiers.alt => {
                    if let Some(name) = browser_key(key) {
                        self.app.key(&name);
                    }
                }
                _ => {}
            }
        }
    }

    /// What the controller asks of the window: files to save, text to copy.
    fn effects(&mut self, ctx: &egui::Context) {
        for effect in self.app.effects() {
            match effect {
                Effect::Copy(text) => {
                    ctx.copy_text(text);
                    self.note(
                        "Step copied — paste it with Ctrl+V, here or in another file",
                        ctx,
                    );
                }
                Effect::Save(export) => {
                    let notes = self.notes.0.clone();
                    let wake = self.wake.clone();
                    let start = self.app.workspace.as_ref().map(|w| w.root().to_path_buf());
                    dialog_thread(move || {
                        let mut dialog = rfd::FileDialog::new().set_file_name(&export.name);
                        if let Some(start) = start {
                            dialog = dialog.set_directory(start);
                        }
                        if let Some(path) = dialog.save_file() {
                            let said = match std::fs::write(&path, export_bytes(&export)) {
                                Ok(()) => format!("Saved {}", path.display()),
                                Err(e) => format!("Could not save {}: {e}", path.display()),
                            };
                            let _ = notes.send(said);
                            wake();
                        }
                    });
                }
            }
        }
        while let Ok(chosen) = self.chosen.1.try_recv() {
            self.app.choose_file(chosen);
        }
        while let Ok(folder) = self.folders.1.try_recv() {
            match Controller::open_folder(&folder, None, self.wake.clone()) {
                Ok(app) => {
                    let reduced = self.app.reduced;
                    self.app = app;
                    self.app.reduced = reduced;
                    self.statics = Arc::new(Statics::default());
                }
                Err(e) => self.app.error = Some(e.root_message().to_string()),
            }
        }
        while let Ok(files) = self.added.1.try_recv() {
            self.app.add_files(files);
        }
        while let Ok(note) = self.notes.1.try_recv() {
            self.note(&note, ctx);
        }
    }

    /// Where the view is in the window, once drawn.
    pub fn view_rect(&self) -> Option<egui::Rect> {
        self.view.rect
    }

    fn note(&mut self, text: &str, ctx: &egui::Context) {
        self.note = Some((text.to_string(), ctx.input(|i| i.time) + 4.0));
    }

    /// Choose files from disk to add to the workspace: programs, and the
    /// data files operations read.
    pub fn pick_files(&mut self) {
        let mut extensions = vec!["geop".to_string(), "json".to_string()];
        for o in self.app.program.iter().flat_map(|p| &p.operations) {
            extensions.extend(o.info.reads.iter().map(|e| e.to_string()));
        }
        let sender = self.added.0.clone();
        let wake = self.wake.clone();
        dialog_thread(move || {
            let Some(paths) = rfd::FileDialog::new()
                .add_filter("programs and data", &extensions)
                .pick_files()
            else {
                return;
            };
            let read = (paths.iter())
                .filter_map(|p| {
                    let name = p.file_name()?.to_string_lossy().into_owned();
                    Some((name, Controller::content_of(std::fs::read(p).ok()?)))
                })
                .collect();
            let _ = sender.send(read);
            wake();
        });
    }

    fn pick_folder(&mut self) {
        let sender = self.folders.0.clone();
        let wake = self.wake.clone();
        dialog_thread(move || {
            if let Some(folder) = rfd::FileDialog::new().pick_folder() {
                let _ = sender.send(folder);
                wake();
            }
        });
    }

    /// Choose a file from disk for the dialog's file field `field`, of the
    /// extensions `extensions`: read off the window's thread, then put
    /// beside the program.
    fn upload(&mut self, field: String, extensions: Vec<String>) {
        let sender = self.chosen.0.clone();
        let wake = self.wake.clone();
        dialog_thread(move || {
            let extensions: Vec<&str> = extensions.iter().map(String::as_str).collect();
            let Some(path) = rfd::FileDialog::new()
                .add_filter("files", &extensions)
                .pick_file()
            else {
                return;
            };
            let Ok(bytes) = std::fs::read(&path) else {
                return;
            };
            let name = path
                .file_name()
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_default();
            let _ = sender.send(Chosen {
                field,
                name,
                content: Controller::content_of(bytes),
            });
            wake();
        });
    }

    fn status(&mut self, ui: &mut egui::Ui) {
        let app = &self.app;
        ui.horizontal(|ui| {
            if app.busy() || !app.ready() {
                ui.add(egui::Spinner::new().size(12.0));
                ui.label(
                    RichText::new(if app.ready() {
                        "Building…"
                    } else {
                        "Loading geop…"
                    })
                    .weak(),
                );
            } else {
                if let Some(dot) = icons::image("dot", 12.0, self.theme.ok()) {
                    ui.add(dot);
                }
                ui.label(RichText::new("Ready").weak());
            }
            ui.separator();
            if let Some(active) = app.active() {
                ui.label(RichText::new(active).monospace()).on_hover_text(
                    app.workspace
                        .as_ref()
                        .map(|w| w.root().display().to_string())
                        .unwrap_or_default(),
                );
                ui.separator();
            }
            let steps = app.program.as_ref().map_or(0, |p| p.steps.len());
            ui.label(RichText::new(format!("{steps} steps")).weak());
            ui.label(RichText::new(format!("{} triangles", app.scene.triangles())).weak());
            if let Some((note, _)) = &self.note {
                ui.separator();
                ui.label(RichText::new(note).color(self.theme.accent()));
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    RichText::new("Ctrl+K: commands · F1: shortcuts")
                        .weak()
                        .small(),
                );
            });
        });
    }
}

/// A file dialog, on a thread of its own where the platform allows, so the
/// window goes on drawing: macOS wants its dialogs on the main thread.
fn dialog_thread(dialog: impl FnOnce() + Send + 'static) {
    if cfg!(target_os = "macos") {
        dialog();
    } else {
        std::thread::spawn(dialog);
    }
}

/// Today, `YYYY-MM-DD`, in UTC: what an export is dated.
pub fn today() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs()) as i64;
    let days = secs.div_euclid(86_400);
    // Civil from days (Howard Hinnant).
    let z = days + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02}")
}

/// A key as the browser names it, which the editor takes: letters
/// lowercased, digits, and the keys a step reads by name.
pub fn browser_key(key: egui::Key) -> Option<String> {
    use egui::Key::*;
    let named = match key {
        Escape => "Escape",
        Enter => "Enter",
        Delete => "Delete",
        Backspace => "Backspace",
        Tab => "Tab",
        Space => " ",
        ArrowUp => "ArrowUp",
        ArrowDown => "ArrowDown",
        ArrowLeft => "ArrowLeft",
        ArrowRight => "ArrowRight",
        Home => "Home",
        End => "End",
        PageUp => "PageUp",
        PageDown => "PageDown",
        F1 => return None,
        _ => {
            let name = key.name();
            let mut chars = name.chars();
            return match (chars.next(), chars.next()) {
                (Some(c), None) if c.is_ascii_alphanumeric() => {
                    Some(c.to_ascii_lowercase().to_string())
                }
                _ => None,
            };
        }
    };
    Some(named.to_string())
}

impl eframe::App for GeopApp {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.app.poll();
        self.effects(ctx);
        if self
            .note
            .as_ref()
            .is_some_and(|(_, until)| ctx.input(|i| i.time) > *until)
        {
            self.note = None;
        }
        self.keys(ctx);
        if self.applied != Some(self.theme) {
            self.theme.apply(ctx);
            self.applied = Some(self.theme);
        }
        let title = match self.app.active() {
            Some(active) => format!("{active} — geop"),
            None => "geop".to_string(),
        };
        if self.title != title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.title = title;
        }
        if self.app.busy() || !self.app.ready() {
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        egui::Panel::top("ribbon")
            .frame(self.theme.bar_frame())
            .show(ui, |ui| ribbon::ribbon(self, ui));
        egui::Panel::bottom("status")
            .frame(self.theme.bar_frame())
            .show(ui, |ui| self.status(ui));
        if self.files_shown && self.app.workspace.is_some() {
            egui::Panel::left("files")
                .resizable(true)
                .default_size(190.0)
                .size_range(140.0..=420.0)
                .frame(self.theme.side_frame())
                .show(ui, |ui| explorer::explorer(self, ui));
        }
        egui::Panel::left("program")
            .resizable(true)
            .default_size(230.0)
            .size_range(170.0..=480.0)
            .frame(self.theme.side_frame())
            .show(ui, |ui| timeline::program(self, ui));
        let sidebar_dialog = (self.app.step.as_ref())
            .is_some_and(|s| s.presentation.layout == geop_core_editor::editing::Layout::Sidebar);
        egui::Panel::right("detail")
            .resizable(true)
            .default_size(if sidebar_dialog { 320.0 } else { 280.0 })
            .size_range(220.0..=620.0)
            .frame(self.theme.side_frame())
            .show(ui, |ui| {
                if sidebar_dialog {
                    dialog::sidebar(self, ui);
                } else {
                    sidebar::part(self, ui);
                }
            });
        if sidebar_dialog || self.app.step.is_some() {
            self.app.lights.clear();
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::new())
            .show(ui, |ui| viewport::viewport(self, ui));
        if !sidebar_dialog && self.app.step.is_some() {
            dialog::popup(self, &ctx);
        }
        if let Some(mut chooser) = self.chooser.take() {
            if explorer::chooser(self, &ctx, &mut chooser) {
                self.chooser = Some(chooser);
            }
        }
        palette::show(self, &ctx);
        if self.shortcuts_open {
            palette::shortcuts(self, &ctx);
        }
    }
}

/// Run the app on the folder (or program) `path`.
pub fn run(path: PathBuf) -> eframe::Result {
    let (dir, active) = geop_cad_base::disk::workspace_of(&path)
        .unwrap_or_else(|_| (std::env::current_dir().unwrap_or_default(), None));
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("geop")
            .with_inner_size([1400.0, 900.0])
            .with_min_inner_size([800.0, 500.0]),
        renderer: eframe::Renderer::Wgpu,
        ..Default::default()
    };
    eframe::run_native(
        "geop",
        options,
        Box::new(move |cc| Ok(Box::new(GeopApp::create(cc, &dir, active)?))),
    )
}

impl GeopApp {
    /// The window's app, made once its context and renderer are: editing
    /// the folder `dir`, the program `active` first.
    pub fn create(
        cc: &eframe::CreationContext,
        dir: &std::path::Path,
        active: Option<String>,
    ) -> Result<Self, String> {
        egui_extras::install_image_loaders(&cc.egui_ctx);
        if let Some(state) = &cc.wgpu_render_state {
            crate::render::install(state);
        }
        let ctx = cc.egui_ctx.clone();
        let wake: Arc<dyn Fn() + Send + Sync> = Arc::new(move || ctx.request_repaint());
        let app = Controller::open_folder(dir, active, wake.clone())
            .map_err(|e| e.root_message().to_string())?;
        Ok(GeopApp::new(app, wake))
    }
}

#[cfg(test)]
mod tests;
