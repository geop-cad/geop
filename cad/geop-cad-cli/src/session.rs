//! One editor (see `geop_core_editor::editor`) answering commands of the
//! JSON protocol, one line each: what `geop serve` runs over stdin/stdout
//! and `geop open` over a WebSocket.
//!
//! Every command is answered by exactly one line: the update, or
//! `{"fatal": "..."}` if the command could not be read at all, or
//! `{"crashed": "<what panicked, where>"}` if the kernel panicked running
//! it. A panic leaves the editor in an unknown state: the session is then
//! started afresh, and the front end gives it the program it holds again
//! (`web/src/backend.ts`, `vscode-extension/src/server.ts`).

use crate::DiskCache;
use geop_cad_base::{Editor, editor_with};
use std::{cell::RefCell, path::PathBuf, sync::Once};

thread_local! {
    /// What last panicked on this thread and where, as the default hook
    /// prints it: the panic hook runs on the thread that panics, so each
    /// session reads its own.
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

/// What a session answered to one command.
pub struct Answer {
    /// The line to send back.
    pub reply: String,
    /// Whether the kernel panicked: the editor answering it is gone.
    pub crashed: bool,
}

/// An editor and where what it makes of files is kept.
pub struct Session {
    editor: Editor,
    cache_root: Option<PathBuf>,
}

impl Session {
    /// A fresh editor, keeping what it makes of the files under
    /// `cache_root` beside each (see [`DiskCache`]), or in memory only.
    pub fn new(cache_root: Option<PathBuf>) -> Self {
        note_panics();
        let editor = editor_with(DiskCache::cache(cache_root.clone().and_then(DiskCache::of)));
        Self { editor, cache_root }
    }

    /// The answer to the command `line`.
    pub fn answer(&mut self, line: &str) -> Answer {
        let answer = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            self.editor.handle_json(line)
        }));
        match answer {
            Ok(Ok(update)) => Answer {
                reply: update,
                crashed: false,
            },
            Ok(Err(message)) => Answer {
                reply: serde_json::json!({ "fatal": message }).to_string(),
                crashed: false,
            },
            Err(_) => {
                let message = PANICKED.with(|p| p.borrow().clone());
                Answer {
                    reply: serde_json::json!({ "crashed": message }).to_string(),
                    crashed: true,
                }
            }
        }
    }

    /// Start afresh, as [`Session::new`] did: after a crash.
    pub fn restart(&mut self) {
        *self = Self::new(self.cache_root.take());
    }
}
