//! The kernel, off the window's thread: one thread owns the
//! [`geop_core_editor::editor::Editor`] and answers the [`Command`]s sent
//! to it in order, each with an [`Answer`] — the editor's
//! [`geop_core_editor::editor::Update`], its scene made plain data on the
//! way ([`SceneData`]), so that the window never waits for the kernel and
//! never converts a mesh.
//!
//! A panic in the kernel is caught: the answer says what panicked, and the
//! editor is started afresh, for the front end to give the program again —
//! as `geop serve` does (`cad/geop-cad-cli/src/session.rs`).

use std::{
    cell::RefCell,
    path::PathBuf,
    sync::{Arc, Once, mpsc},
};

use geop_cad_base::{disk::DiskCache, editor_with};
use geop_core_editor::{
    editing::Presentation,
    editor::{Command, File, ProgramState, StepState, Update},
    output::{Export, Inspection},
};

use crate::scene::SceneData;

thread_local! {
    /// What last panicked on this thread, and where.
    static PANICKED: RefCell<String> = const { RefCell::new(String::new()) };
}

/// Note every panic's message in [`PANICKED`], besides printing it.
fn note_panics() {
    static HOOK: Once = Once::new();
    HOOK.call_once(|| {
        let default_hook = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            PANICKED.with(|p| *p.borrow_mut() = info.to_string());
            default_hook(info);
        }));
    });
}

/// The editor's answer to the command sent as `id`.
#[derive(Debug)]
pub struct Answer {
    pub id: u64,
    pub error: Option<String>,
    pub program: Option<ProgramState>,
    pub scene: Option<SceneData>,
    pub step: Option<StepState>,
    pub files: Option<Vec<File>>,
    pub written: Option<Vec<File>>,
    pub export: Option<Export>,
    pub tool: Option<Presentation>,
    pub inspection: Option<Inspection>,
    /// What panicked, if the kernel did: it was started afresh, and knows
    /// no program and no files.
    pub crashed: Option<String>,
}

impl Answer {
    fn of(id: u64, update: Update) -> Self {
        Self {
            id,
            error: update.error,
            program: update.program,
            scene: update.scene.as_ref().map(SceneData::of),
            step: update.step,
            files: update.files,
            written: update.written,
            export: update.export,
            tool: update.tool,
            inspection: update.inspection,
            crashed: None,
        }
    }

    fn crashed(id: u64, message: String) -> Self {
        Self {
            id,
            error: None,
            program: None,
            scene: None,
            step: None,
            files: None,
            written: None,
            export: None,
            tool: None,
            inspection: None,
            crashed: Some(message),
        }
    }
}

/// The kernel's thread, and the way to it and back.
pub struct Kernel {
    jobs: mpsc::Sender<(u64, Command)>,
    answers: mpsc::Receiver<Answer>,
    next: u64,
}

impl Kernel {
    /// A kernel on a thread of its own, keeping what files are made into
    /// beside them under `cache_root` — or in memory only — and calling
    /// `wake` whenever an answer is ready.
    pub fn start(cache_root: Option<PathBuf>, wake: Arc<dyn Fn() + Send + Sync>) -> Self {
        note_panics();
        let (jobs, inbox) = mpsc::channel::<(u64, Command)>();
        let (outbox, answers) = mpsc::channel();
        let fresh =
            move || editor_with(DiskCache::cache(cache_root.clone().and_then(DiskCache::of)));
        std::thread::Builder::new()
            .name("geop kernel".into())
            .spawn(move || {
                let mut editor = fresh();
                for (id, command) in inbox {
                    let answered = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        Answer::of(id, editor.handle(command))
                    }));
                    let answer = answered.unwrap_or_else(|_| {
                        editor = fresh();
                        Answer::crashed(id, PANICKED.with(|p| p.borrow().clone()))
                    });
                    if outbox.send(answer).is_err() {
                        return;
                    }
                    wake();
                }
            })
            .expect("a thread for the kernel");
        Self {
            jobs,
            answers,
            next: 0,
        }
    }

    /// Send `command`: its id, which its answer has.
    pub fn send(&mut self, command: Command) -> u64 {
        self.next += 1;
        // A kernel whose thread is gone answers nothing more: the window
        // goes on showing what it last had.
        let _ = self.jobs.send((self.next, command));
        self.next
    }

    /// An answer that is ready, if one is.
    pub fn try_answer(&self) -> Option<Answer> {
        self.answers.try_recv().ok()
    }

    /// The next answer, waiting for it: for tests, and for leaving.
    pub fn answer(&self) -> Option<Answer> {
        self.answers.recv().ok()
    }
}
