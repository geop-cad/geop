//! What the desktop app is, without its window: the editor it talks to
//! (on the kernel's thread, see [`crate::kernel`]), what the editor last
//! showed — the program, the scene, the step being edited, the tool in
//! hand — the camera, and the files of the workspace. Every button, key
//! and pointer event of the window is a call here that sends the editor a
//! [`Command`], as the web app's `App.tsx` does; nothing here knows an
//! operation. The window ([`crate::ui`]) draws what this holds.
//!
//! The window never waits for the kernel: a command is sent and its
//! answer taken in on a later frame ([`Controller::poll`]). The pointer's
//! hovers and drags are sent one at a time, the latest one waiting while
//! one is answered, so a slow kernel falls behind by one answer, never by a
//! growing queue.

use std::{
    collections::BTreeMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use geop_core_editor::{
    editing::{Button, Pointer, Presentation, StepEditEvent, Value},
    editor::{Command, ProgramState, StepState},
    output::{Content, Export, Inspection},
};
use geop_core_program::{element::PickedElement, program::Program};

use crate::{
    camera::{Camera, DEFAULT_POSE, Drawn, Pose, head_on, wants_framing},
    kernel::{Answer, Kernel},
    scene::{Frame, Kept},
    workspace::{
        Workspace, folder_of, free_folder, free_path, is_program, parse_program, program_text,
    },
};

/// What a command was sent for: what its answer is taken as.
#[derive(Clone, Debug, PartialEq)]
enum Sent {
    /// Any command, by its kind.
    Command(&'static str),
    /// A hover, leave or drag of the pointer: one is answered at a time.
    Pointer,
    /// A question to an inspector: its answer is kept by its kind.
    Inspect,
    /// What is copied of a step, for the clipboard.
    Copy,
    /// What an exporter wrote, to save.
    Export,
}

/// What the window does for the controller: what it cannot do itself.
#[derive(Clone, Debug, PartialEq)]
pub enum Effect {
    /// Save a file an exporter wrote, asking where.
    Save(Export),
    /// Put text on the clipboard.
    Copy(String),
}

/// A section view: a viewing aid that cuts the view along a plane, never
/// the model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Section {
    pub axis: SectionAxis,
    /// Along the plane's normal, from the part's centre — or from the
    /// picked plane's origin.
    pub offset: f64,
    /// Which side is cut away.
    pub flip: bool,
    /// The plane picked, for [`SectionAxis::Picked`].
    pub plane: Option<Frame>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SectionAxis {
    X,
    Y,
    Z,
    Picked,
}

impl Section {
    /// Where it cuts, around the part's centre `center`: a point of the
    /// plane, and the normal pointing to what is cut away.
    pub fn plane(&self, center: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
        let (from, normal) = match self.axis {
            SectionAxis::X => (center, [1.0, 0.0, 0.0]),
            SectionAxis::Y => (center, [0.0, 1.0, 0.0]),
            SectionAxis::Z => (center, [0.0, 0.0, 1.0]),
            SectionAxis::Picked => {
                let plane = self.plane?;
                (plane.origin, plane.w)
            }
        };
        let origin = std::array::from_fn(|k| from[k] + normal[k] * self.offset);
        let normal = if self.flip {
            normal.map(|x| -x)
        } else {
            normal
        };
        Some((origin, normal))
    }
}

/// A file a field of the dialog chose from disk, read: to put beside the
/// program, and set the field to.
#[derive(Clone, Debug)]
pub struct Chosen {
    pub field: String,
    pub name: String,
    pub content: String,
}

/// The app without its window (see the module docs).
pub struct Controller {
    kernel: Kernel,
    sent: BTreeMap<u64, Sent>,
    /// The pointer event answered now, and the latest one waiting.
    pointer_sent: Option<u64>,
    pointer_waiting: Option<StepEditEvent>,
    /// How many hovers and drags were sent: what coalescing saves.
    pub pointer_events_sent: usize,
    /// Whether the kernel is editing one of the files yet: until then what
    /// it shows is its own empty program, never written over a file.
    editing_file: bool,

    pub program: Option<ProgramState>,
    pub scene: Kept,
    pub step: Option<StepState>,
    /// What the tool in hand shows, while no step is edited.
    pub tool: Option<Presentation>,
    /// Why the last command was refused, if it was.
    pub error: Option<String>,
    /// The pick tool's answer about its picks.
    pub picked: Option<Inspection>,
    /// The last answers of the inspectors asked of the whole part, by
    /// kind, until the part changes.
    pub answers: BTreeMap<String, Inspection>,
    /// What the panels and lists light, beyond what the editor does.
    pub lights: Vec<PickedElement>,
    pub section: Option<Section>,
    /// What panicked in the kernel, until the user has read it: the
    /// kernel was started afresh, with the program as it was.
    pub crashed: Option<String>,
    pub camera: Camera,
    pub workspace: Option<Workspace>,
    /// What the experimental operations are left out of: on by default.
    pub reduced: bool,

    /// Where the camera was before it turned to face the plane worked in.
    before_plane: Option<Pose>,
    /// The plane worked in, as last shown.
    plane: Option<Frame>,
    /// The drawing last shown, and whether the one shown now wants framing.
    drawn: Option<Drawn>,
    frame_wanted: bool,
    effects: Vec<Effect>,
}

/// The pointer events of which only the latest matters.
fn coalesced(event: &StepEditEvent) -> bool {
    matches!(
        event,
        StepEditEvent::Hover { .. }
            | StepEditEvent::Leave
            | StepEditEvent::Drag { done: false, .. }
    )
}

impl Controller {
    /// An app on a kernel that keeps what files are made into under
    /// `cache_root`, waking the window through `wake` whenever the kernel
    /// answers.
    pub fn new(cache_root: Option<PathBuf>, wake: Arc<dyn Fn() + Send + Sync>) -> Self {
        let mut this = Self {
            kernel: Kernel::start(cache_root, wake),
            sent: BTreeMap::new(),
            pointer_sent: None,
            pointer_waiting: None,
            pointer_events_sent: 0,
            editing_file: false,
            program: None,
            scene: Kept::default(),
            step: None,
            tool: None,
            error: None,
            picked: None,
            answers: BTreeMap::new(),
            lights: Vec::new(),
            section: None,
            crashed: None,
            camera: Camera::new(),
            workspace: None,
            reduced: true,
            before_plane: None,
            plane: None,
            drawn: None,
            frame_wanted: false,
            effects: Vec::new(),
        };
        this.send(Command::Show, Sent::Command("show"));
        this
    }

    /// An app editing the files of the folder `dir`, the program `active`
    /// first (see `geop_cad_base::disk::workspace_of`).
    pub fn open_folder(
        dir: &Path,
        active: Option<String>,
        wake: Arc<dyn Fn() + Send + Sync>,
    ) -> geop_core_math::geop_error::GeopResult<Self> {
        let registry = geop_cad_base::registry()?;
        let workspace = Workspace::open(dir, geop_cad_base::disk::extensions(&registry), active)?;
        let mut this = Self::new(Some(workspace.root().to_path_buf()), wake);
        let files = (workspace.files.iter())
            .map(|(path, content)| (path.clone(), Some(content.clone())))
            .collect();
        let active = workspace.active.clone();
        this.workspace = Some(workspace);
        this.command(Command::Files { files });
        this.editing_file = true;
        this.open_file(&active);
        Ok(this)
    }

    // ── talking to the kernel ──────────────────────────────────────────

    fn send(&mut self, command: Command, sent: Sent) -> u64 {
        let id = self.kernel.send(command);
        self.sent.insert(id, sent);
        id
    }

    /// Send `command`, whose answer is shown when it comes.
    pub fn command(&mut self, command: Command) {
        let kind = kind(&command);
        self.send(command, Sent::Command(kind));
    }

    /// Whether commands are waiting for their answers: the app is busy.
    pub fn busy(&self) -> bool {
        self.sent.values().any(|s| *s != Sent::Pointer)
    }

    /// Whether the editor answered its first command.
    pub fn ready(&self) -> bool {
        self.program.is_some()
    }

    /// Take in every answer that is ready, showing each.
    pub fn poll(&mut self) {
        while let Some(answer) = self.kernel.try_answer() {
            self.take(answer);
        }
    }

    /// Wait until every command sent is answered, showing each answer:
    /// for tests, which drive the app as the window does.
    pub fn settle(&mut self) {
        while !self.sent.is_empty() {
            match self.kernel.answer() {
                Some(answer) => self.take(answer),
                None => return,
            }
        }
    }

    fn take(&mut self, answer: Answer) {
        let sent = self.sent.remove(&answer.id).unwrap_or(Sent::Command("?"));
        if let Some(panic) = &answer.crashed {
            self.restarted(panic);
            return;
        }
        self.show(answer, &sent);
        if sent == Sent::Pointer {
            self.pointer_sent = None;
            if let Some(next) = self.pointer_waiting.take() {
                self.pointer_now(next);
            }
        }
    }

    /// The kernel panicked and was started afresh: it is given the files
    /// and the program edited again, as they were before the command.
    fn restarted(&mut self, panic: &str) {
        self.sent.clear();
        self.pointer_sent = None;
        self.pointer_waiting = None;
        self.scene = Kept::default();
        self.error = Some(format!(
            "the kernel crashed ({panic}). This is a bug in geop: please report it. \
             It was restarted with your program as it was before."
        ));
        self.crashed = self.error.take();
        match &self.workspace {
            Some(workspace) => {
                let files = (workspace.files.iter())
                    .map(|(path, content)| (path.clone(), Some(content.clone())))
                    .collect();
                let path = workspace.active.clone();
                let program = parse_program(&workspace.files[&path]).unwrap_or_default();
                self.command(Command::Files { files });
                self.command(Command::Load {
                    program,
                    path: Some(path),
                });
            }
            None => {
                let program = self
                    .program
                    .as_ref()
                    .map(|p| p.program.clone())
                    .unwrap_or_default();
                self.command(Command::Load {
                    program,
                    path: None,
                });
            }
        }
    }

    /// Show `answer`, to the command sent as `sent`.
    fn show(&mut self, answer: Answer, sent: &Sent) {
        if let Some(program) = answer.program {
            self.program = Some(program);
        }
        let mut failed = None;
        if let Some(workspace) = &mut self.workspace {
            for file in answer.written.iter().chain(&answer.files).flatten() {
                if let Err(e) = workspace.write(&file.path, &program_text(&file.program)) {
                    failed = Some(e.root_message().to_string());
                }
            }
            // The program edited is the file the editor says it is in.
            if let (Some(shown), true) = (&self.program, self.editing_file)
                && let Some(path) = &shown.path
            {
                let text = program_text(&shown.program);
                if let Err(e) = workspace.write(path, &text) {
                    failed = Some(e.root_message().to_string());
                }
                workspace.active = path.clone();
            }
        }
        if let Some(scene) = answer.scene {
            self.scene.take(scene);
            self.answers.clear();
            let drawn = Drawn {
                size: self.scene.part.size,
                solids: self.scene.solids(),
            };
            if let Some(before) = self.drawn
                && wants_framing(before, drawn, &self.camera.pose)
            {
                self.frame_wanted = true;
            }
            self.drawn = Some(drawn);
        }
        self.step = answer.step;
        self.tool = answer.tool;
        match sent {
            Sent::Inspect => {
                if let Some(inspection) = answer.inspection {
                    self.answers.insert(inspection.kind.to_string(), inspection);
                }
            }
            _ => self.picked = answer.inspection,
        }
        match (sent, answer.export) {
            (Sent::Copy, Some(export)) => {
                if let Some(text) = export.text() {
                    self.effects.push(Effect::Copy(text.to_string()));
                }
            }
            (Sent::Export, Some(export)) => self.effects.push(Effect::Save(export)),
            _ => {}
        }
        // A hover's answer says nothing new of what was refused before.
        if *sent != Sent::Pointer || answer.error.is_some() {
            self.error = answer.error.or(failed);
        }
        self.follow_plane();
        if std::mem::take(&mut self.frame_wanted) {
            self.fit(None);
        }
    }

    /// Working in a plane, the camera turns to face it, and comes back to
    /// where it was once done — framing the drawing from there if it wants
    /// framing now.
    fn follow_plane(&mut self) {
        let presentation = self.presentation();
        let plane = presentation.and_then(|p| p.focus.as_ref()).map(Frame::of);
        let sheet = presentation.is_some_and(|p| p.sheet.is_some());
        if plane == self.plane {
            return;
        }
        self.plane = plane;
        let frame = std::mem::take(&mut self.frame_wanted);
        match plane {
            Some(plane) => {
                let before = *self.before_plane.get_or_insert(self.camera.pose);
                let _ = before;
                let head = head_on(&plane, &self.camera.pose);
                if frame || sheet {
                    self.fit(Some(head));
                } else {
                    self.camera.glide_to(head);
                }
            }
            None => match self.before_plane.take() {
                Some(before) if frame => self.fit(Some(before)),
                Some(before) => self.camera.glide_to(before),
                None if frame => self.fit(None),
                None => {}
            },
        }
    }

    /// What is done for the controller that only the window can do.
    pub fn effects(&mut self) -> Vec<Effect> {
        std::mem::take(&mut self.effects)
    }

    // ── what is shown ──────────────────────────────────────────────────

    /// What the step edited shows — or the tool in hand.
    pub fn presentation(&self) -> Option<&Presentation> {
        self.step
            .as_ref()
            .map(|s| &s.presentation)
            .or(self.tool.as_ref())
    }

    /// The plane worked in.
    pub fn plane(&self) -> Option<Frame> {
        self.plane
    }

    /// Whether a press where the pointer last hovered starts a drag.
    pub fn grab(&self) -> bool {
        self.presentation().is_some_and(|p| p.grab)
    }

    /// Everything lit: what the editor lights, and what the panels do.
    pub fn highlights(&self) -> Vec<PickedElement> {
        let mut lit: Vec<PickedElement> = self
            .presentation()
            .map(|p| p.highlights.clone())
            .unwrap_or_default();
        lit.extend(self.lights.iter().cloned());
        lit
    }

    pub fn can_undo(&self) -> bool {
        match &self.step {
            Some(step) => step.can_undo,
            None => self.program.as_ref().is_some_and(|p| p.can_undo),
        }
    }

    pub fn can_redo(&self) -> bool {
        match &self.step {
            Some(step) => step.can_redo,
            None => self.program.as_ref().is_some_and(|p| p.can_redo),
        }
    }

    /// Whether the open step can be put into the program.
    pub fn can_commit(&self) -> bool {
        self.step
            .as_ref()
            .is_some_and(|s| s.error.is_none() && s.missing.is_empty())
    }

    // ── the camera ─────────────────────────────────────────────────────

    /// Frame the drawing, looking from `from`'s direction, else from the
    /// camera's.
    pub fn fit(&mut self, from: Option<Pose>) {
        let part = &self.scene.part;
        self.camera.fit(part.center, part.size, from);
    }

    pub fn reset_view(&mut self) {
        self.fit(Some(DEFAULT_POSE));
    }

    /// Which way is up on screen, for a step to name what it shows as
    /// seen: sent when the view settles, and when the pointer comes in.
    pub fn view_settled(&mut self) {
        let up = self.camera.up();
        let v = |p: [f64; 3]| {
            geop_core_math::vector::Vector3::from_array(
                p.map(geop_core_math::scalars::Scalar::from_f64),
            )
        };
        self.event(StepEditEvent::View { up: v(up) });
    }

    // ── what the user does ─────────────────────────────────────────────

    /// Something the user did to the step being edited, or with the tool
    /// in hand.
    pub fn event(&mut self, event: StepEditEvent) {
        if coalesced(&event) {
            if self.pointer_sent.is_some() {
                self.pointer_waiting = Some(event);
            } else {
                self.pointer_now(event);
            }
            return;
        }
        // A click or the end of a drag says where the pointer is now: a
        // hover or drag waiting says nothing more.
        if event.pointer().is_some() {
            self.pointer_waiting = None;
        }
        self.command(Command::Event { event });
    }

    fn pointer_now(&mut self, event: StepEditEvent) {
        self.pointer_events_sent += 1;
        let id = self.send(Command::Event { event }, Sent::Pointer);
        self.pointer_sent = Some(id);
    }

    pub fn hover(&mut self, pointer: Pointer, shift: bool) {
        self.event(StepEditEvent::Hover { pointer, shift });
    }

    pub fn click(&mut self, pointer: Pointer, button: Button, double: bool, shift: bool) {
        self.event(StepEditEvent::Click {
            pointer,
            button,
            double,
            shift,
        });
    }

    pub fn drag(&mut self, from: Pointer, to: Pointer, done: bool, shift: bool) {
        self.event(StepEditEvent::Drag {
            from,
            to,
            done,
            shift,
        });
    }

    pub fn dialog(&mut self, key: &str, value: Value) {
        self.event(StepEditEvent::Dialog {
            key: key.to_string(),
            value,
        });
    }

    /// A key, as the browser names it: to the step being edited, if one
    /// is; with none, `f` frames the drawing and Home resets the view.
    pub fn key(&mut self, key: &str) {
        if self.step.is_some() {
            self.event(StepEditEvent::Key {
                key: key.to_string(),
            });
        } else if key.eq_ignore_ascii_case("f") {
            self.fit(None);
        } else if key == "Home" {
            self.reset_view();
        } else if self.tool.is_some() {
            self.event(StepEditEvent::Key {
                key: key.to_string(),
            });
        }
    }

    /// Start a step of the operation `kind` — or, if one is open and new
    /// already, give it up.
    pub fn new_step(&mut self, kind: &str) {
        let open_new = self
            .step
            .as_ref()
            .is_some_and(|s| s.kind == kind && s.id.is_none());
        if open_new {
            self.command(Command::Cancel);
        } else {
            self.command(Command::New {
                kind: kind.to_string(),
            });
        }
    }

    pub fn open_step(&mut self, id: &str, placed: Option<String>) {
        self.command(Command::Open {
            id: id.to_string(),
            placed,
        });
    }

    pub fn commit(&mut self) {
        self.command(Command::Commit);
    }

    pub fn cancel(&mut self) {
        self.command(Command::Cancel);
    }

    pub fn undo(&mut self) {
        self.command(Command::Undo);
    }

    pub fn redo(&mut self) {
        self.command(Command::Redo);
    }

    /// Copy the step `id`, as text a paste inserts: the clipboard is
    /// given it ([`Effect::Copy`]).
    pub fn copy_step(&mut self, id: &str) {
        self.send(Command::Copy { id: id.to_string() }, Sent::Copy);
    }

    pub fn paste(&mut self, text: &str) {
        self.command(Command::Paste {
            text: text.to_string(),
        });
    }

    /// Run only the steps before the slot `slot` — all of them at the end.
    pub fn seek(&mut self, slot: usize) {
        let steps = self.program.as_ref().map_or(0, |p| p.steps.len());
        self.command(Command::Seek {
            marker: (slot < steps).then_some(slot),
        });
    }

    pub fn inspect(&mut self, inspector: &str, options: serde_json::Value) {
        self.send(
            Command::Inspect {
                inspector: inspector.to_string(),
                options,
                picked: Vec::new(),
            },
            Sent::Inspect,
        );
    }

    /// Write the part with the exporter `exporter`, in `format` — its
    /// first if none — told `options`, dated `date`: the window is asked
    /// to save it ([`Effect::Save`]).
    pub fn export(
        &mut self,
        exporter: &str,
        format: Option<String>,
        options: serde_json::Value,
        date: String,
    ) {
        self.send(
            Command::Export {
                exporter: exporter.to_string(),
                format,
                options,
                date,
            },
            Sent::Export,
        );
    }

    // ── the files ──────────────────────────────────────────────────────

    /// Edit the file `path`. A data file is no program: it is not opened.
    pub fn open_file(&mut self, path: &str) {
        if !is_program(path) {
            let lower = path.to_ascii_lowercase();
            let reader = (self.program.iter().flat_map(|p| &p.operations)).find(|o| {
                o.info
                    .reads
                    .iter()
                    .any(|e| lower.ends_with(&format!(".{e}")))
            });
            self.error = Some(match reader {
                Some(o) => format!(
                    "{path} is no program: read it into one with \"{}\"",
                    o.info.label
                ),
                None => format!("{path} is no program"),
            });
            return;
        }
        let Some(text) = self
            .workspace
            .as_ref()
            .and_then(|w| w.files.get(path))
            .cloned()
        else {
            self.error = Some(format!("there is no file {path}"));
            return;
        };
        match parse_program(&text) {
            Ok(program) => {
                self.command(Command::Load {
                    program,
                    path: Some(path.to_string()),
                });
                self.frame_wanted = true;
            }
            Err(e) => self.error = Some(format!("{path}: {}", e.root_message())),
        }
    }

    /// Tell the editor the files `files` changed — `None` for one gone —
    /// keeping them on disk.
    pub fn change_files(&mut self, files: BTreeMap<String, Option<String>>) {
        if let Some(workspace) = &mut self.workspace {
            for (path, content) in &files {
                let done = match content {
                    Some(content) => workspace.write(path, content),
                    None => workspace.remove(path),
                };
                if let Err(e) = done {
                    self.error = Some(e.root_message().to_string());
                }
            }
        }
        self.command(Command::Files { files });
    }

    /// Make the program file `path`, holding `program`, and edit it.
    pub fn create_file(&mut self, path: &str, program: &Program) {
        let text = program_text(program);
        if let Some(workspace) = &mut self.workspace
            && let Err(e) = workspace.write(path, &text)
        {
            self.error = Some(e.root_message().to_string());
            return;
        }
        // Opened first, so the editor builds the new file, not the one left.
        self.open_file(path);
        self.command(Command::Files {
            files: BTreeMap::from([(path.to_string(), Some(text))]),
        });
    }

    /// A new, empty part, in a file of its own.
    pub fn new_part(&mut self) {
        let path = free_path(&self.files(), "part.geop");
        self.create_file(&path, &Program::default());
    }

    pub fn rename_file(&mut self, from: &str, to: &str) {
        let Some(text) = self.files().get(from).cloned() else {
            return;
        };
        let active = self.workspace.as_ref().is_some_and(|w| w.active == from);
        if let Some(workspace) = &mut self.workspace
            && let Err(e) = workspace.write(to, &text)
        {
            self.error = Some(e.root_message().to_string());
            return;
        }
        if active {
            self.open_file(to);
        }
        self.change_files(BTreeMap::from([
            (from.to_string(), None),
            (to.to_string(), Some(text)),
        ]));
    }

    /// Delete the files `paths`, opening another if the one edited goes.
    pub fn delete_files(&mut self, paths: &[String]) {
        let files = self.files();
        if let Some(active) = self.workspace.as_ref().map(|w| w.active.clone())
            && paths.contains(&active)
        {
            let other = files
                .keys()
                .find(|p| is_program(p) && !paths.contains(p))
                .cloned();
            match other {
                Some(other) => self.open_file(&other),
                None => {
                    let path = free_path(&BTreeMap::new(), "part.geop");
                    self.create_file(&path, &Program::default());
                }
            }
        }
        self.change_files(paths.iter().map(|p| (p.clone(), None)).collect());
    }

    /// The files of the workspace, by path.
    pub fn files(&self) -> BTreeMap<String, String> {
        self.workspace
            .as_ref()
            .map(|w| w.files.clone())
            .unwrap_or_default()
    }

    /// The file edited.
    pub fn active(&self) -> Option<&str> {
        self.workspace.as_ref().map(|w| w.active.as_str())
    }

    /// Read the folder again, telling the editor what changed outside.
    pub fn reread(&mut self) {
        let Some(workspace) = &mut self.workspace else {
            return;
        };
        match workspace.reread() {
            Ok(changed) if !changed.is_empty() => {
                let active = workspace.active.clone();
                let reload = changed.get(&active).cloned().flatten();
                self.command(Command::Files { files: changed });
                if let Some(text) = reload
                    && let Ok(program) = parse_program(&text)
                    && self.program.as_ref().is_none_or(|p| p.program != program)
                {
                    self.command(Command::Load {
                        program,
                        path: Some(active),
                    });
                }
            }
            Ok(_) => {}
            Err(e) => self.error = Some(e.root_message().to_string()),
        }
    }

    /// Load the example program `name`, in a file of its own.
    pub fn load_example(&mut self, name: &str) {
        if self.workspace.is_some() {
            let path = free_path(&self.files(), &format!("{name}.geop"));
            self.create_file(&path, &Program::default());
        }
        self.command(Command::LoadExample {
            name: name.to_string(),
        });
        self.frame_wanted = true;
    }

    /// Add the files of the example `name`, in a folder of their own, and
    /// edit the first.
    pub fn load_workspace_example(&mut self, name: &str) {
        let folder = free_folder(&self.files(), &format!("examples/{name}"));
        self.command(Command::LoadWorkspaceExample {
            name: name.to_string(),
            folder: Some(folder),
        });
        self.frame_wanted = true;
    }

    /// Put `chosen`, a file chosen from disk for a file field, beside the
    /// program edited — under another name if one of its name there holds
    /// something else — and set the field to its path relative to the
    /// program.
    pub fn choose_file(&mut self, chosen: Chosen) {
        let files = self.files();
        let folder = self.active().map(folder_of).unwrap_or("").to_string();
        let wanted = format!("{folder}{}", chosen.name);
        let path = match files.get(&wanted) {
            Some(held) if *held != chosen.content => free_path(&files, &wanted),
            _ => wanted,
        };
        self.change_files(BTreeMap::from([(path.clone(), Some(chosen.content))]));
        self.dialog(
            &chosen.field,
            Value::Choice(path[folder.len()..].to_string()),
        );
    }

    /// Bytes chosen from disk, as a workspace holds a file.
    pub fn content_of(bytes: Vec<u8>) -> String {
        geop_core_program::workspace::content(bytes)
    }
}

/// The kind of a command, as the protocol names it.
fn kind(command: &Command) -> &'static str {
    match command {
        Command::Show => "show",
        Command::New { .. } => "new",
        Command::Open { .. } => "open",
        Command::Event { .. } => "event",
        Command::Commit => "commit",
        Command::Cancel => "cancel",
        Command::Preview { .. } => "preview",
        Command::Remove { .. } => "remove",
        Command::Move { .. } => "move",
        Command::Copy { .. } => "copy",
        Command::Paste { .. } => "paste",
        Command::Seek { .. } => "seek",
        Command::Load { .. } => "load",
        Command::Files { .. } => "files",
        Command::LoadExample { .. } => "load_example",
        Command::LoadWorkspaceExample { .. } => "load_workspace_example",
        Command::Undo => "undo",
        Command::Redo => "redo",
        Command::Panel { .. } => "panel",
        Command::Visibility { .. } => "visibility",
        Command::Tool { .. } => "tool",
        Command::Inspect { .. } => "inspect",
        Command::Export { .. } => "export",
        Command::Crash => "crash",
    }
}

/// The bytes of an exported file.
pub fn export_bytes(export: &Export) -> Vec<u8> {
    match &export.content {
        Content::Text(text) => text.as_bytes().to_vec(),
        Content::Bytes(bytes) => bytes.clone(),
    }
}
