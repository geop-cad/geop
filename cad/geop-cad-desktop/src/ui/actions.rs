//! Everything the app does at the user's word, by one name: what a menu,
//! a button, a hotkey and the command palette do alike ([`Action`]), the
//! hotkey of each ([`Action::shortcut`]), and the list the palette offers
//! ([`palette`]) — operations from the kernel's listing, examples, files,
//! tools and inspectors among them.

use eframe::egui::{Key, KeyboardShortcut, Modifiers};

use crate::controller::Controller;

/// What the user can ask for.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    NewStep(String),
    Commit,
    Cancel,
    Undo,
    Redo,
    CopyStep,
    Paste,
    Fit,
    ResetView,
    ToggleProjection,
    ToggleTheme,
    ToggleFiles,
    ToggleReduced,
    TogglePreview,
    NewPart,
    OpenFolder,
    Reread,
    OpenFile(String),
    LoadExample(String),
    LoadWorkspaceExample(String),
    Export(String),
    Inspect(String),
    Tool(Option<String>),
    ToggleSection,
    Palette,
    Shortcuts,
}

const fn key(modifiers: Modifiers, key: Key) -> Option<KeyboardShortcut> {
    Some(KeyboardShortcut::new(modifiers, key))
}

const COMMAND: Modifiers = Modifiers::COMMAND;
const COMMAND_SHIFT: Modifiers = Modifiers::COMMAND.plus(Modifiers::SHIFT);

impl Action {
    /// Its hotkey, if it has one. Plain keys belong to the step being
    /// edited — a sketch's tools — so every hotkey of the app's own holds
    /// the command key, but those of the view with no step open (F, Home),
    /// which the web app has too.
    pub fn shortcut(&self) -> Option<KeyboardShortcut> {
        match self {
            Action::Palette => key(COMMAND, Key::K),
            Action::Undo => key(COMMAND, Key::Z),
            Action::Redo => key(COMMAND_SHIFT, Key::Z),
            Action::Commit => key(COMMAND, Key::Enter),
            Action::Cancel => key(COMMAND, Key::Backspace),
            Action::CopyStep => key(COMMAND, Key::C),
            Action::Paste => key(COMMAND, Key::V),
            Action::NewPart => key(COMMAND, Key::N),
            Action::OpenFolder => key(COMMAND, Key::O),
            Action::Fit => key(Modifiers::NONE, Key::F),
            Action::ResetView => key(Modifiers::NONE, Key::Home),
            Action::ToggleProjection => key(COMMAND_SHIFT, Key::P),
            Action::ToggleTheme => key(COMMAND_SHIFT, Key::L),
            Action::ToggleFiles => key(COMMAND, Key::B),
            Action::TogglePreview => key(COMMAND_SHIFT, Key::V),
            Action::ToggleSection => key(COMMAND_SHIFT, Key::X),
            Action::Reread => key(COMMAND_SHIFT, Key::R),
            Action::Shortcuts => key(Modifiers::NONE, Key::F1),
            _ => None,
        }
    }

    /// The actions with hotkeys of their own, to check each frame.
    pub fn with_shortcuts() -> Vec<Action> {
        vec![
            Action::Palette,
            Action::Redo,
            Action::Undo,
            Action::Commit,
            Action::Cancel,
            Action::NewPart,
            Action::OpenFolder,
            Action::ToggleProjection,
            Action::ToggleTheme,
            Action::ToggleFiles,
            Action::TogglePreview,
            Action::ToggleSection,
            Action::Reread,
            Action::Shortcuts,
        ]
    }

    /// Whether it can be done now.
    pub fn enabled(&self, app: &Controller) -> bool {
        let idle = app.step.is_none();
        match self {
            Action::NewStep(kind) => {
                app.ready() && app.step.as_ref().is_none_or(|s| s.kind == kind)
            }
            Action::Commit => app.can_commit(),
            Action::Cancel | Action::TogglePreview => app.step.is_some(),
            Action::Undo => app.can_undo(),
            Action::Redo => app.can_redo(),
            Action::CopyStep => app.step.as_ref().is_some_and(|s| s.id.is_some()),
            Action::Inspect(_) | Action::Tool(_) | Action::Export(_) => app.ready() && idle,
            Action::OpenFile(_) | Action::LoadExample(_) | Action::LoadWorkspaceExample(_) => {
                app.ready() && idle
            }
            Action::NewPart => app.ready() && idle,
            _ => true,
        }
    }
}

/// An entry of the command palette: what it says, what it does.
#[derive(Clone, Debug)]
pub struct Entry {
    pub label: String,
    /// What kind of thing it is: `Operation`, `Example`, `File`, `View`.
    pub group: &'static str,
    pub detail: String,
    pub icon: Option<String>,
    pub action: Action,
}

/// What the palette offers now.
pub fn palette(app: &Controller) -> Vec<Entry> {
    let mut entries = Vec::new();
    let entry = |label: &str, group, detail: &str, icon: Option<&str>, action| Entry {
        label: label.to_string(),
        group,
        detail: detail.to_string(),
        icon: icon.map(str::to_string),
        action,
    };
    if let Some(program) = &app.program {
        for listed in &program.operations {
            if app.reduced && listed.info.experimental {
                continue;
            }
            entries.push(entry(
                listed.info.label,
                "Operation",
                listed.info.doc,
                listed.info.icon,
                Action::NewStep(listed.kind.to_string()),
            ));
        }
        for tool in &program.tools {
            entries.push(entry(
                tool.label,
                "Tool",
                tool.doc,
                tool.icon,
                Action::Tool(Some(tool.key.to_string())),
            ));
        }
        for inspector in &program.inspectors {
            entries.push(entry(
                inspector.label,
                "Inspect",
                inspector.doc,
                inspector.icon,
                match inspector.picks {
                    Some(_) => Action::Tool(Some(format!("pick:{}", inspector.kind))),
                    None => Action::Inspect(inspector.kind.to_string()),
                },
            ));
        }
        for exporter in &program.exporters {
            entries.push(entry(
                &format!("Export {}", exporter.label),
                "Export",
                exporter.doc,
                Some("save"),
                Action::Export(exporter.kind.to_string()),
            ));
        }
        for name in &program.examples {
            entries.push(entry(
                name,
                "Example",
                "Open the example in a file of its own",
                Some("example"),
                Action::LoadExample(name.to_string()),
            ));
        }
        for name in &program.workspace_examples {
            entries.push(entry(
                name,
                "Example",
                "Add the example's files in a folder of their own",
                Some("example"),
                Action::LoadWorkspaceExample(name.to_string()),
            ));
        }
    }
    for path in app.files().keys() {
        entries.push(entry(
            path,
            "File",
            "Open the file",
            Some("file"),
            Action::OpenFile(path.clone()),
        ));
    }
    let actions = [
        ("OK — put the step into the program", Action::Commit),
        ("Cancel the step", Action::Cancel),
        ("Undo", Action::Undo),
        ("Redo", Action::Redo),
        ("Copy the step", Action::CopyStep),
        ("Paste steps", Action::Paste),
        ("Fit the part in the view", Action::Fit),
        ("Reset the view", Action::ResetView),
        (
            "Switch perspective / orthographic",
            Action::ToggleProjection,
        ),
        ("Switch dark / light theme", Action::ToggleTheme),
        ("Show / hide the files", Action::ToggleFiles),
        ("Show / hide experimental operations", Action::ToggleReduced),
        ("Preview the step as it builds", Action::TogglePreview),
        ("Section view", Action::ToggleSection),
        ("New part", Action::NewPart),
        ("Open a folder…", Action::OpenFolder),
        ("Read the folder again", Action::Reread),
        ("Put the tool down", Action::Tool(None)),
        ("Keyboard shortcuts", Action::Shortcuts),
    ];
    for (label, action) in actions {
        entries.push(entry(label, "App", "", None, action));
    }
    entries
}

/// How well `query` matches `text`: its letters in order, earlier and
/// together better; `None` for no match.
pub fn score(query: &str, text: &str) -> Option<i64> {
    let query: Vec<char> = query
        .to_lowercase()
        .chars()
        .filter(|c| !c.is_whitespace())
        .collect();
    if query.is_empty() {
        return Some(0);
    }
    let text: Vec<char> = text.to_lowercase().chars().collect();
    let mut score = 0i64;
    let mut at = 0usize;
    let mut last: Option<usize> = None;
    for q in query {
        let found = (at..text.len()).find(|&i| text[i] == q)?;
        score -= found as i64;
        if last.is_some_and(|l| l + 1 == found) {
            score += 10;
        }
        if found == 0 || !text[found - 1].is_alphanumeric() {
            score += 8;
        }
        last = Some(found);
        at = found + 1;
    }
    Some(score)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn letters_in_order_match_words_starts_best() {
        assert!(score("ext", "Extrude").is_some());
        assert!(score("dx", "Extrude").is_none());
        assert!(score("fil", "Fillet").unwrap() > score("fil", "Profile line").unwrap());
        assert_eq!(score("", "anything"), Some(0));
    }

    #[test]
    fn every_hotkey_is_of_one_action() {
        let all = Action::with_shortcuts();
        for (i, a) in all.iter().enumerate() {
            for b in &all[i + 1..] {
                assert_ne!(a.shortcut(), b.shortcut(), "{a:?} and {b:?}");
            }
        }
    }
}
