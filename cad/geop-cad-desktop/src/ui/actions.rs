//! Everything the app does at the user's word, by one name: what a menu,
//! a button, a hotkey and the command palette do alike ([`Action`]), the
//! hotkey of each ([`Action::shortcut`]), and the list the palette offers
//! ([`palette`]) — operations from the kernel's listing, examples, files,
//! tools and inspectors among them.

use eframe::egui::{InputState, Key, KeyboardShortcut, Modifiers};
use geop_core_editor::keys::key_of;

use crate::controller::Controller;

/// What the user can ask for.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    /// A new step of the operation `kind`, started by the key the kernel
    /// gives it (`OperationInfo::shortcut`), if it gives one.
    NewStep(String, Option<&'static str>),
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
    /// Edit the step of this id.
    OpenStep(String),
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
    /// Its hotkey, if it has one: of what every front end offers, the one
    /// the kernel names (`geop_core_editor::keys`), so the web app presses
    /// the same; of the desktop app's own, one holding the command key.
    /// Plain keys belong to the step being edited — a sketch's tools — so
    /// the view's (F, Home, O) work only with no step open.
    pub fn shortcut(&self) -> Option<KeyboardShortcut> {
        let app = |action| key_of(action).and_then(chord);
        match self {
            Action::NewStep(_, shortcut) => shortcut.and_then(chord),
            Action::Palette => app("palette"),
            Action::Shortcuts => app("keys"),
            Action::Undo => app("undo"),
            Action::Redo => app("redo"),
            Action::Commit => app("commit"),
            Action::Cancel => app("cancel"),
            Action::Fit => app("fit"),
            Action::ResetView => app("home"),
            Action::ToggleProjection => app("projection"),
            Action::CopyStep => key(COMMAND, Key::C),
            Action::Paste => key(COMMAND, Key::V),
            Action::NewPart => key(COMMAND, Key::N),
            Action::OpenFolder => key(COMMAND, Key::O),
            Action::ToggleTheme => key(COMMAND_SHIFT, Key::L),
            Action::ToggleFiles => key(COMMAND, Key::B),
            Action::TogglePreview => key(COMMAND_SHIFT, Key::V),
            Action::ToggleSection => key(COMMAND_SHIFT, Key::X),
            Action::Reread => key(COMMAND_SHIFT, Key::R),
            _ => None,
        }
    }

    /// Whether its key works while a step is edited: one holding the
    /// command key, or the key list's.
    pub fn while_editing(&self) -> bool {
        matches!(self, Action::Shortcuts) || self.shortcut().is_some_and(|s| s.modifiers.command)
    }

    /// The actions with hotkeys of their own, to check each frame.
    pub fn with_shortcuts() -> Vec<Action> {
        vec![
            Action::Palette,
            Action::Shortcuts,
            Action::Redo,
            Action::Undo,
            Action::Commit,
            Action::Cancel,
            Action::Fit,
            Action::ResetView,
            Action::ToggleProjection,
            Action::NewPart,
            Action::OpenFolder,
            Action::ToggleTheme,
            Action::ToggleFiles,
            Action::TogglePreview,
            Action::ToggleSection,
            Action::Reread,
        ]
    }

    /// Whether it can be done now.
    pub fn enabled(&self, app: &Controller) -> bool {
        let idle = app.step.is_none();
        match self {
            Action::NewStep(kind, _) => {
                app.ready() && app.step.as_ref().is_none_or(|s| s.kind == kind)
            }
            Action::Commit => app.can_commit(),
            Action::Cancel | Action::TogglePreview => app.step.is_some(),
            Action::Undo => app.can_undo(),
            Action::Redo => app.can_redo(),
            Action::CopyStep => app.step.as_ref().is_some_and(|s| s.id.is_some()),
            Action::Inspect(_) | Action::Tool(_) | Action::Export(_) => app.ready() && idle,
            Action::OpenFile(_)
            | Action::OpenStep(_)
            | Action::LoadExample(_)
            | Action::LoadWorkspaceExample(_) => app.ready() && idle,
            Action::NewPart => app.ready() && idle,
            _ => true,
        }
    }
}

/// A key the kernel names — `E`, `Shift+F`, `Mod+Enter`, `?` (see
/// `geop_core_editor::keys`) — as egui presses it.
pub fn chord(text: &str) -> Option<KeyboardShortcut> {
    let mut parts: Vec<&str> = text.split('+').collect();
    // `+` itself is the key of `Shift++`.
    let key = match parts.pop()? {
        "" => "+",
        key => key,
    };
    let mut modifiers = Modifiers::NONE;
    for part in parts {
        modifiers = modifiers
            | match part {
                "Mod" => Modifiers::COMMAND,
                "Shift" => Modifiers::SHIFT,
                "Alt" => Modifiers::ALT,
                _ => return None,
            };
    }
    Some(KeyboardShortcut::new(modifiers, Key::from_name(key)?))
}

/// Whether `shortcut` is pressed now, taking the press if it is. A letter
/// or a named key is pressed with Shift only if its shortcut holds Shift
/// — Shift+B is not B — while a symbol (`?`) is the symbol, whatever Shift
/// it takes to type it: as the web app matches keys.
pub fn take(input: &mut InputState, shortcut: &KeyboardShortcut) -> bool {
    let symbol = shortcut.logical_key.symbol_or_name().chars().count() == 1
        && !shortcut
            .logical_key
            .symbol_or_name()
            .chars()
            .all(char::is_alphanumeric);
    if !symbol && input.modifiers.shift != shortcut.modifiers.shift {
        return false;
    }
    input.consume_shortcut(shortcut)
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
                Action::NewStep(listed.kind.to_string(), listed.info.shortcut),
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
        for step in &program.steps {
            entries.push(entry(
                &step.id,
                "Step",
                &format!("{}: {}", step.label, step.summary),
                None,
                Action::OpenStep(step.id.clone()),
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
    // The app's own, grouped as the web app groups them, named as the
    // kernel names those every front end offers.
    let label = |action: &str, otherwise: &'static str| {
        (geop_core_editor::keys::APP_KEYS.iter())
            .find(|k| k.action == action)
            .map_or(otherwise, |k| k.label)
    };
    let actions = [
        (label("commit", ""), "Edit", Action::Commit),
        (label("cancel", ""), "Edit", Action::Cancel),
        (label("undo", ""), "Edit", Action::Undo),
        (label("redo", ""), "Edit", Action::Redo),
        ("Copy the step", "Edit", Action::CopyStep),
        ("Paste steps", "Edit", Action::Paste),
        (
            "Preview the step as it builds",
            "Edit",
            Action::TogglePreview,
        ),
        (label("fit", ""), "View", Action::Fit),
        (label("home", ""), "View", Action::ResetView),
        (label("projection", ""), "View", Action::ToggleProjection),
        ("Switch dark / light theme", "View", Action::ToggleTheme),
        ("Show / hide the files", "View", Action::ToggleFiles),
        ("Section view", "View", Action::ToggleSection),
        (
            "Show / hide experimental operations",
            "View",
            Action::ToggleReduced,
        ),
        ("New part", "File", Action::NewPart),
        ("Open a folder…", "File", Action::OpenFolder),
        ("Read the folder again", "File", Action::Reread),
        ("Put the tool down", "Tool", Action::Tool(None)),
        (label("palette", ""), "Help", Action::Palette),
        (label("keys", ""), "Help", Action::Shortcuts),
    ];
    for (label, group, action) in actions {
        entries.push(entry(label, group, "", None, action));
    }
    entries
}

/// How well `query` matches `text`, for the palette to rank by: `None`
/// if its letters do not all appear in `text` in order. Higher is better:
/// the text starting with the query, then a word starting with it, then the
/// letters closest together, earliest — as the web app ranks
/// (`web/src/shortcuts.ts`), so both find the same first.
pub fn score(query: &str, text: &str) -> Option<i64> {
    let q = query.trim().to_lowercase();
    let t = text.to_lowercase();
    let len = t.chars().count() as i64;
    if q.is_empty() {
        return Some(0);
    }
    if t == q {
        return Some(2000);
    }
    if t.starts_with(&q) {
        return Some(1000 - len);
    }
    let words = t.split(|c: char| c.is_whitespace() || ":_-/·".contains(c));
    if let Some(word) = words
        .filter(|w| !w.is_empty())
        .position(|w| w.starts_with(&q))
    {
        return Some(800 - word as i64 * 10 - len);
    }
    let chars: Vec<char> = t.chars().collect();
    if let Some(at) = t.find(&q) {
        let at = t[..at].chars().count() as i64;
        return Some(600 - at - len);
    }
    // The letters in order, as close together as they come.
    let mut from = 0usize;
    let mut spread = 0i64;
    let mut first: Option<usize> = None;
    for c in q.chars().filter(|c| *c != ' ') {
        let i = (from..chars.len()).find(|&i| chars.get(i) == Some(&c))?;
        match first {
            None => first = Some(i),
            Some(_) => spread += (i - from) as i64,
        }
        from = i + 1;
    }
    Some(300 - spread * 5 - first.unwrap_or(0) as i64 - len)
}

/// Of `entries`, those matching `query`, best first — those that cannot be
/// done now last — and, with no query, as given; a group's name finds all
/// of it, after what the query names itself. As the web app searches.
pub fn search(entries: Vec<Entry>, query: &str, enabled: impl Fn(&Action) -> bool) -> Vec<Entry> {
    let q = query.trim().to_lowercase();
    let mut scored: Vec<(bool, i64, usize, Entry)> = (entries.into_iter().enumerate())
        .filter_map(|(i, entry)| {
            let group = !q.is_empty() && entry.group.to_lowercase().starts_with(&q);
            let s = score(query, &entry.label).or(group.then_some(-1000))?;
            Some((!enabled(&entry.action), s, i, entry))
        })
        .collect();
    scored.sort_by(|a, b| a.0.cmp(&b.0).then(b.1.cmp(&a.1)).then(a.2.cmp(&b.2)));
    scored.into_iter().map(|(_, _, _, entry)| entry).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn letters_in_order_match_words_starts_best() {
        // As `web/test/shortcuts.test.ts` has them.
        assert!(score("ext", "Extrude").is_some());
        assert!(score("dx", "Extrude").is_none());
        assert!(score("fil", "Fillet").unwrap() > score("fil", "Profile line").unwrap());
        assert_eq!(score("", "anything"), Some(0));
        assert!(score("revol", "Revolve").unwrap() > score("revol", "Revolve cut").unwrap());
        assert!(score("cut", "Revolve cut").unwrap() > score("cut", "Shortcuts").unwrap());
        assert!(score("xtr", "Extrude").unwrap() > score("etd", "Extrude").unwrap());
    }

    #[test]
    fn search_ranks_what_can_be_done_first_and_finds_groups() {
        let entry = |label: &str, group, action| Entry {
            label: label.into(),
            group,
            detail: String::new(),
            icon: None,
            action,
        };
        let entries = vec![
            entry("Undo", "Edit", Action::Undo),
            entry(
                "Revolve",
                "Operation",
                Action::NewStep("revolve".into(), None),
            ),
            entry("Fit the part in the view", "View", Action::Fit),
        ];
        let can = |a: &Action| *a != Action::Undo;
        let found = search(entries.clone(), "re", can);
        assert_eq!(found[0].label, "Revolve");
        let found = search(entries.clone(), "view", can);
        assert_eq!(found.len(), 1, "a group finds its own");
        let all = search(entries, "", can);
        assert_eq!(
            all.last().map(|e| e.label.as_str()),
            Some("Undo"),
            "what cannot be done is last"
        );
    }

    #[test]
    fn the_kernels_keys_are_pressed_as_named() {
        assert_eq!(
            chord("Shift+F"),
            Some(KeyboardShortcut::new(Modifiers::SHIFT, Key::F))
        );
        assert_eq!(
            chord("E"),
            Some(KeyboardShortcut::new(Modifiers::NONE, Key::E))
        );
        assert_eq!(chord("Mod+Shift+Z"), key(COMMAND_SHIFT, Key::Z));
        assert_eq!(chord("?"), key(Modifiers::NONE, Key::Questionmark));
        // Every key the kernel names for the app is one egui presses.
        for app in geop_core_editor::keys::APP_KEYS {
            assert!(chord(app.key).is_some(), "{}", app.key);
        }
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
