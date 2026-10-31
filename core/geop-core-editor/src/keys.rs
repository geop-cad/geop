//! The keys of what every front end offers besides the operations — the
//! command palette, the key list, committing and cancelling a step,
//! undoing, framing the view — named once, here, so that the web app and
//! VS Code press the same keys for the same things (see `docs/ux.md`). An operation's key is its own
//! ([`crate::editable::OperationInfo::shortcut`]).
//!
//! A key is written as the front ends read it: modifiers before the key,
//! joined by `+` — `Mod` is Ctrl, or ⌘ on a Mac — `Mod+K`, `Shift+F`, `?`.
//! Plain keys belong to the step being edited (its tools), so the view's
//! plain keys (`F`, `Home`, `O`) work only while no step is; the key
//! list's (`?`) and those with `Mod` always. Escape is the step's while
//! one is edited: it ends what is in progress in it first, and with
//! nothing in progress cancels it (see [`crate::editor::Editor::handle`]).

use serde::Serialize;

/// Something every front end offers, and its key.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct AppKey {
    /// What it does, as the front ends name it: `commit`, `fit`.
    pub action: &'static str,
    /// Its key (see the module docs).
    pub key: &'static str,
    /// What it does, said as a menu says it.
    pub label: &'static str,
}

/// The app's keys (see the module docs).
pub const APP_KEYS: &[AppKey] = &[
    AppKey {
        action: "palette",
        key: "Mod+K",
        label: "Command palette",
    },
    AppKey {
        action: "keys",
        key: "?",
        label: "Keyboard shortcuts",
    },
    AppKey {
        action: "commit",
        key: "Mod+Enter",
        label: "OK: put the step into the program",
    },
    AppKey {
        action: "escape",
        key: "Escape",
        label: "Stop what is in progress, else cancel the step",
    },
    AppKey {
        action: "cancel",
        key: "Mod+Backspace",
        label: "Cancel the step, whatever is in progress",
    },
    AppKey {
        action: "undo",
        key: "Mod+Z",
        label: "Undo",
    },
    AppKey {
        action: "redo",
        key: "Mod+Shift+Z",
        label: "Redo",
    },
    AppKey {
        action: "fit",
        key: "F",
        label: "Fit the part in the view",
    },
    AppKey {
        action: "home",
        key: "Home",
        label: "Reset the view",
    },
    AppKey {
        action: "projection",
        key: "O",
        label: "Switch perspective / orthographic",
    },
];

/// The key of `action`, if it has one.
pub fn key_of(action: &str) -> Option<&'static str> {
    (APP_KEYS.iter())
        .find(|k| k.action == action)
        .map(|k| k.key)
}

/// Whether `key` is a plain key of the app's: one no operation may take.
pub fn is_app_key(key: &str) -> bool {
    APP_KEYS.iter().any(|k| k.key == key)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_key_is_of_one_action() {
        for (i, a) in APP_KEYS.iter().enumerate() {
            for b in &APP_KEYS[i + 1..] {
                assert_ne!(a.key, b.key, "{} and {}", a.action, b.action);
                assert_ne!(a.action, b.action);
            }
        }
        assert_eq!(key_of("commit"), Some("Mod+Enter"));
        assert!(is_app_key("F") && !is_app_key("E"));
    }
}
