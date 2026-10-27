//! Files as a tree of folders: the workspace's, to open, make, rename and
//! delete ([`explorer`]); and a dialog's file field's, to choose one from
//! ([`chooser`]) — the one chosen revealed and selected, those the field
//! does not take greyed, found by typing, moved through by the arrow keys,
//! chosen by a double click or Enter, and — where the field takes files
//! from elsewhere — one chosen from disk, put beside the program.

use std::collections::BTreeSet;

use eframe::egui::{self, RichText, Ui};
use geop_core_editor::editing::{FileEntry, Value};

use super::{GeopApp, icons};
use crate::workspace::{folder_of, is_program};

/// A folder or a file of a tree. A folder's key is the start of its files'
/// paths: `parts/`, `std:`.
#[derive(Clone, Debug, PartialEq)]
pub enum Node {
    Folder {
        name: String,
        key: String,
        children: Vec<Node>,
    },
    File {
        name: String,
        path: String,
        accepted: bool,
    },
}

impl Node {
    pub fn key(&self) -> &str {
        match self {
            Node::Folder { key, .. } => key,
            Node::File { path, .. } => path,
        }
    }
}

/// The scheme a path of a shared library starts with, `std:`, if any.
fn scheme_of(path: &str) -> Option<&str> {
    let i = path.find(':')?;
    (i >= 2 && path[..i].chars().all(|c| c.is_ascii_alphabetic())).then(|| &path[..=i])
}

/// The folders a path is in, outermost first, by key.
pub fn folders_of(path: &str) -> Vec<String> {
    let scheme = scheme_of(path);
    let rest = scheme.map_or(path, |s| &path[s.len()..]);
    let mut key = scheme.unwrap_or("").to_string();
    let mut folders: Vec<String> = scheme.map(|s| vec![s.to_string()]).unwrap_or_default();
    let segments: Vec<&str> = rest.split('/').collect();
    for segment in &segments[..segments.len().saturating_sub(1)] {
        key.push_str(segment);
        key.push('/');
        folders.push(key.clone());
    }
    folders
}

/// `files` as a tree: folders first, then files, each sorted by name —
/// `..` before every other folder, a shared library's root after all.
pub fn tree(files: &[(String, bool)]) -> Vec<Node> {
    fn insert(
        level: &mut Vec<Node>,
        folders: &[String],
        name: String,
        path: String,
        accepted: bool,
    ) {
        match folders.split_first() {
            None => level.push(Node::File {
                name,
                path,
                accepted,
            }),
            Some((key, rest)) => {
                let at = level
                    .iter()
                    .position(|n| matches!(n, Node::Folder { key: k, .. } if k == key));
                let at = at.unwrap_or_else(|| {
                    let folder_name = if scheme_of(key) == Some(key.as_str()) {
                        key.clone()
                    } else {
                        key.trim_end_matches('/')
                            .rsplit('/')
                            .next()
                            .unwrap_or(key)
                            .to_string()
                    };
                    level.push(Node::Folder {
                        name: folder_name,
                        key: key.clone(),
                        children: Vec::new(),
                    });
                    level.len() - 1
                });
                if let Node::Folder { children, .. } = &mut level[at] {
                    insert(children, rest, name, path, accepted);
                }
            }
        }
    }
    fn rank(n: &Node) -> u8 {
        match n {
            Node::File { .. } => 2,
            Node::Folder { key, .. } if scheme_of(key) == Some(key.as_str()) => 3,
            Node::Folder { name, .. } if name == ".." => 0,
            Node::Folder { .. } => 1,
        }
    }
    fn sort(nodes: &mut [Node]) {
        nodes.sort_by(|a, b| {
            let name = |n: &Node| match n {
                Node::Folder { name, .. } | Node::File { name, .. } => name.to_lowercase(),
            };
            rank(a).cmp(&rank(b)).then(name(a).cmp(&name(b)))
        });
        for n in nodes {
            if let Node::Folder { children, .. } = n {
                sort(children);
            }
        }
    }
    let mut root = Vec::new();
    for (path, accepted) in files {
        let scheme = scheme_of(path);
        let rest = scheme.map_or(path.as_str(), |s| &path[s.len()..]);
        let name = rest.rsplit('/').next().unwrap_or(rest).to_string();
        insert(&mut root, &folders_of(path), name, path.clone(), *accepted);
    }
    sort(&mut root);
    root
}

/// Whether every word of `query` is in `path`, ignoring case.
fn matches(path: &str, query: &str) -> bool {
    let lower = path.to_lowercase();
    query
        .to_lowercase()
        .split_whitespace()
        .all(|w| lower.contains(w))
}

/// A row of a tree as shown.
#[derive(Clone, Debug)]
pub struct Row<'a> {
    pub node: &'a Node,
    pub depth: usize,
    pub open: bool,
}

/// The rows shown: the files `query` matches — and, `accepted_only`, those
/// a field takes — in the folders open: those `expanded`, and every folder
/// holding a match while searching. A folder with nothing to show is not.
pub fn rows<'a>(
    tree: &'a [Node],
    expanded: &BTreeSet<String>,
    query: &str,
    accepted_only: bool,
) -> Vec<Row<'a>> {
    let searching = !query.trim().is_empty();
    fn shows(node: &Node, query: &str, searching: bool, accepted_only: bool) -> bool {
        match node {
            Node::File { path, accepted, .. } => {
                (!accepted_only || *accepted) && (!searching || matches(path, query.trim()))
            }
            Node::Folder { children, .. } => children
                .iter()
                .any(|c| shows(c, query, searching, accepted_only)),
        }
    }
    let mut out = Vec::new();
    fn walk<'a>(
        nodes: &'a [Node],
        depth: usize,
        out: &mut Vec<Row<'a>>,
        f: &dyn Fn(&Node) -> bool,
        open: &dyn Fn(&Node) -> bool,
    ) {
        for node in nodes {
            if !f(node) {
                continue;
            }
            let is_open = open(node);
            out.push(Row {
                node,
                depth,
                open: is_open,
            });
            if let (Node::Folder { children, .. }, true) = (node, is_open) {
                walk(children, depth + 1, out, f, open);
            }
        }
    }
    let f = |n: &Node| shows(n, query, searching, accepted_only);
    let open =
        |n: &Node| matches!(n, Node::Folder { key, .. } if searching || expanded.contains(key));
    walk(tree, 0, &mut out, &f, &open);
    out
}

// ── the workspace's files ───────────────────────────────────────────────

#[derive(Default)]
pub struct ExplorerState {
    expanded: BTreeSet<String>,
    /// A file being renamed, and what it is being called.
    renaming: Option<(String, String)>,
    /// A new file being named.
    creating: Option<String>,
    /// Files asked to be deleted, waiting for a yes.
    deleting: Option<Vec<String>>,
    seen_active: Option<String>,
}

pub fn explorer(window: &mut GeopApp, ui: &mut Ui) {
    let theme = window.theme;
    let enabled = window.app.step.is_none() && window.app.ready();
    let files: Vec<(String, bool)> = window
        .app
        .files()
        .keys()
        .map(|p| (p.clone(), true))
        .collect();
    let active = window.app.active().map(str::to_string);
    let mut upload = false;
    let state = &mut window.explorer;
    if active != state.seen_active {
        if let Some(active) = &active {
            state.expanded.extend(folders_of(active));
        }
        state.seen_active = active.clone();
    }
    ui.horizontal(|ui| {
        ui.label(RichText::new("Files").strong());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let new = icons::image("new", 15.0, ui.visuals().text_color()).map(egui::Button::image);
            if let Some(b) = new
                && ui
                    .add_enabled(enabled, b.frame(false))
                    .on_hover_text("New program file")
                    .clicked()
            {
                state.creating = Some("part.geop".into());
            }
            let add =
                icons::image("upload", 15.0, ui.visuals().text_color()).map(egui::Button::image);
            if let Some(b) = add
                && ui
                    .add_enabled(enabled, b.frame(false))
                    .on_hover_text("Add files from disk: programs, and the data they read")
                    .clicked()
            {
                upload = true;
            }
            let open =
                icons::image("open", 15.0, ui.visuals().text_color()).map(egui::Button::image);
            if let Some(b) = open
                && ui
                    .add(b.frame(false))
                    .on_hover_text("Open another folder (Ctrl+O)")
                    .clicked()
            {
                window.palette.pending = Some(super::actions::Action::OpenFolder);
            }
        });
    });
    if let Some(root) = window
        .app
        .workspace
        .as_ref()
        .map(|w| w.root().display().to_string())
    {
        ui.add(egui::Label::new(RichText::new(root).small().color(theme.weak())).truncate());
    }
    ui.separator();
    let state = &mut window.explorer;
    let mut open = None;
    let mut create = None;
    let mut rename = None;
    let mut delete = None;
    if let Some(name) = &mut state.creating {
        ui.horizontal(|ui| {
            let r = ui.add(
                egui::TextEdit::singleline(name)
                    .desired_width(ui.available_width() - 30.0)
                    .hint_text("name.geop"),
            );
            r.request_focus();
            if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                let mut path = name.trim().to_string();
                if !path.ends_with(".geop") {
                    path.push_str(".geop");
                }
                create = Some(path);
            }
            if ui.input(|i| i.key_pressed(egui::Key::Escape))
                || icons::button(ui, "close", "Give up").clicked()
            {
                create = Some(String::new());
            }
        });
    }
    let tree = tree(&files);
    let shown = rows(&tree, &state.expanded, "", false);
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for row in shown {
                let key = row.node.key().to_string();
                ui.horizontal(|ui| {
                    ui.add_space(row.depth as f32 * 12.0);
                    match row.node {
                        Node::Folder { name, .. } => {
                            let arrow = if row.open {
                                "chevron-down"
                            } else {
                                "chevron-right"
                            };
                            if let Some(image) = icons::image(arrow, 13.0, theme.weak()) {
                                ui.add(image);
                            }
                            if ui.selectable_label(false, name.as_str()).clicked() {
                                if !state.expanded.remove(&key) {
                                    state.expanded.insert(key.clone());
                                }
                            }
                        }
                        Node::File { name, path, .. } => {
                            if let Some((from, to)) = &mut state.renaming
                                && from == path
                            {
                                let r = ui.add(
                                    egui::TextEdit::singleline(to)
                                        .desired_width(ui.available_width()),
                                );
                                r.request_focus();
                                if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                                    rename = Some((from.clone(), to.trim().to_string()));
                                } else if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                                    rename = Some((from.clone(), from.clone()));
                                }
                                return;
                            }
                            let current = active.as_deref() == Some(path.as_str());
                            let icon = if is_program(path) { "file" } else { "example" };
                            if let Some(image) = icons::image(
                                icon,
                                13.0,
                                if current {
                                    theme.accent()
                                } else {
                                    theme.weak()
                                },
                            ) {
                                ui.add(image);
                            }
                            let mut text = RichText::new(name);
                            if current {
                                text = text.strong().color(theme.accent());
                            } else if !is_program(path) {
                                text = text.color(theme.weak());
                            }
                            let r = ui.add_enabled(
                                enabled,
                                egui::Button::selectable(current, text).frame_when_inactive(false),
                            );
                            let r = r.on_hover_text(if is_program(path) {
                                format!("{path} — open it")
                            } else {
                                format!("{path} — data a program reads")
                            });
                            if r.clicked() {
                                open = Some(path.clone());
                            }
                            r.context_menu(|ui| {
                                if ui.button("Rename…").clicked() {
                                    state.renaming = Some((path.clone(), path.clone()));
                                    ui.close();
                                }
                                if ui.button("Delete…").clicked() {
                                    delete = Some(vec![path.clone()]);
                                    ui.close();
                                }
                            });
                        }
                    }
                });
            }
        });
    if let Some(paths) = delete {
        state.deleting = Some(paths);
    }
    let mut confirmed = None;
    if let Some(paths) = &state.deleting {
        let modal = egui::Modal::new(egui::Id::new("geop delete")).show(ui.ctx(), |ui| {
            ui.set_max_width(360.0);
            ui.heading("Delete?");
            ui.label(format!(
                "Delete {}? This cannot be undone.",
                paths.join(", ")
            ));
            ui.horizontal(|ui| {
                if ui
                    .button(RichText::new("Delete").color(theme.error()))
                    .clicked()
                {
                    confirmed = Some(true);
                }
                if ui.button("Keep").clicked() {
                    confirmed = Some(false);
                }
            });
        });
        if modal.should_close() {
            confirmed.get_or_insert(false);
        }
    }
    let state = &mut window.explorer;
    match confirmed {
        Some(true) => {
            if let Some(paths) = state.deleting.take() {
                window.app.delete_files(&paths);
            }
        }
        Some(false) => state.deleting = None,
        None => {}
    }
    let state = &mut window.explorer;
    if let Some(path) = create {
        state.creating = None;
        if !path.is_empty() {
            if window
                .app
                .workspace
                .as_ref()
                .is_some_and(|w| w.names_file(&path))
            {
                let path = crate::workspace::free_path(&window.app.files(), &path);
                window.app.create_file(&path, &Default::default());
            } else {
                window.app.error = Some(format!("{path:?} can name no program file of the folder"));
            }
        }
    }
    let state = &mut window.explorer;
    if let Some((from, to)) = rename {
        state.renaming = None;
        if from != to && !to.is_empty() {
            window.app.rename_file(&from, &to);
        }
    }
    if let Some(path) = open {
        window.app.open_file(&path);
    }
    if upload {
        window.pick_files();
    }
}

// ── a file field's chooser ──────────────────────────────────────────────

/// A file field's chooser, while it is open.
pub struct Chooser {
    field: String,
    label: String,
    value: String,
    files: Vec<FileEntry>,
    upload: Vec<String>,
    query: String,
    accepted_only: bool,
    expanded: BTreeSet<String>,
    selected: Option<String>,
    focus: bool,
}

impl Chooser {
    pub fn new(
        field: &str,
        label: &str,
        value: &str,
        files: Vec<FileEntry>,
        upload: Vec<String>,
    ) -> Self {
        Self {
            field: field.to_string(),
            label: label.to_string(),
            value: value.to_string(),
            expanded: folders_of(value).into_iter().collect(),
            selected: (!value.is_empty()).then(|| value.to_string()),
            files,
            upload,
            query: String::new(),
            accepted_only: false,
            focus: true,
        }
    }
}

/// Show the chooser: whether it stays open.
pub fn chooser(window: &mut GeopApp, ctx: &egui::Context, c: &mut Chooser) -> bool {
    let theme = window.theme;
    let entries: Vec<(String, bool)> = c
        .files
        .iter()
        .map(|f| (f.path.clone(), f.accepted))
        .collect();
    let tree = tree(&entries);
    let mut keep = true;
    let mut chosen = None;
    let mut upload = false;
    let modal = egui::Modal::new(egui::Id::new("geop file chooser")).show(ctx, |ui| {
        ui.set_width(460.0);
        ui.horizontal(|ui| {
            ui.heading(format!("Choose {}", c.label));
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if icons::button(ui, "close", "Close (Escape)").clicked() {
                    keep = false;
                }
            });
        });
        let search = ui.add(
            egui::TextEdit::singleline(&mut c.query)
                .hint_text("Search files…")
                .desired_width(f32::INFINITY),
        );
        if c.focus {
            search.request_focus();
            c.focus = false;
        }
        if search.changed() {
            let shown = rows(&tree, &c.expanded, &c.query, c.accepted_only);
            c.selected = shown.iter().find_map(|r| match r.node {
                Node::File {
                    path,
                    accepted: true,
                    ..
                } => Some(path.clone()),
                _ => None,
            });
        }
        let shown = rows(&tree, &c.expanded, &c.query, c.accepted_only);
        let index = shown
            .iter()
            .position(|r| Some(r.node.key()) == c.selected.as_deref());
        // The keys move through the rows, open and close folders, choose.
        let searching = !c.query.trim().is_empty();
        ui.input(|i| {
            let mv = |to: isize| {
                shown
                    .get(to.clamp(0, shown.len() as isize - 1) as usize)
                    .map(|r| r.node.key().to_string())
            };
            if i.key_pressed(egui::Key::ArrowDown) {
                c.selected = mv(index.map_or(0, |x| x as isize + 1)).or(c.selected.take());
            }
            if i.key_pressed(egui::Key::ArrowUp) {
                c.selected = mv(index.map_or(shown.len() as isize - 1, |x| x as isize - 1))
                    .or(c.selected.take());
            }
            if let Some(row) = index.map(|x| &shown[x]) {
                if i.key_pressed(egui::Key::ArrowRight)
                    && matches!(row.node, Node::Folder { .. })
                    && !row.open
                    && !searching
                {
                    c.expanded.insert(row.node.key().to_string());
                }
                if i.key_pressed(egui::Key::ArrowLeft) {
                    if matches!(row.node, Node::Folder { .. }) && row.open && !searching {
                        c.expanded.remove(row.node.key());
                    } else if let Some(parent) =
                        folders_of(row.node.key().trim_end_matches('/')).pop()
                    {
                        c.selected = Some(parent);
                    }
                }
                if i.key_pressed(egui::Key::Enter) {
                    match row.node {
                        Node::Folder { key, .. } if !searching => {
                            if !c.expanded.remove(key) {
                                c.expanded.insert(key.clone());
                            }
                        }
                        Node::File {
                            path,
                            accepted: true,
                            ..
                        } => chosen = Some(path.clone()),
                        _ => {}
                    }
                }
            }
            if i.key_pressed(egui::Key::Escape) {
                keep = false;
            }
        });
        egui::ScrollArea::vertical()
            .max_height(320.0)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for row in &shown {
                    let key = row.node.key().to_string();
                    let selected = c.selected.as_deref() == Some(key.as_str());
                    ui.horizontal(|ui| {
                        ui.add_space(row.depth as f32 * 14.0);
                        match row.node {
                            Node::Folder { name, .. } => {
                                let arrow = if row.open {
                                    "chevron-down"
                                } else {
                                    "chevron-right"
                                };
                                if let Some(image) = icons::image(arrow, 13.0, theme.weak()) {
                                    ui.add(image);
                                }
                                let r = ui.selectable_label(selected, name.as_str());
                                if r.clicked() {
                                    c.selected = Some(key.clone());
                                    if !searching && !c.expanded.remove(&key) {
                                        c.expanded.insert(key.clone());
                                    }
                                }
                            }
                            Node::File {
                                name,
                                path,
                                accepted,
                            } => {
                                let mut text = RichText::new(name);
                                if !accepted {
                                    text = text.color(theme.weak()).italics();
                                }
                                if *path == c.value {
                                    text = text.strong();
                                }
                                let r = ui.add(
                                    egui::Button::selectable(selected, text)
                                        .frame_when_inactive(false),
                                );
                                let r = if *accepted {
                                    r.on_hover_text(path)
                                } else {
                                    r.on_hover_text(format!(
                                        "{path} — this field takes no such file"
                                    ))
                                };
                                if r.clicked() {
                                    c.selected = Some(key.clone());
                                }
                                if r.double_clicked() && *accepted {
                                    chosen = Some(path.clone());
                                }
                                if selected {
                                    r.scroll_to_me(None);
                                }
                            }
                        }
                    });
                }
                if shown.is_empty() {
                    ui.label(RichText::new("No file matches.").color(theme.weak()));
                }
            });
        ui.separator();
        ui.horizontal(|ui| {
            let accepted = c.files.iter().filter(|f| f.accepted).count();
            ui.checkbox(
                &mut c.accepted_only,
                format!("Only what it takes ({accepted})"),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                let current = index.map(|x| &shown[x]).and_then(|r| match r.node {
                    Node::File {
                        path,
                        accepted: true,
                        ..
                    } => Some(path.clone()),
                    _ => None,
                });
                if ui
                    .add_enabled(current.is_some(), egui::Button::new("Choose"))
                    .clicked()
                {
                    chosen = current;
                }
                if !c.upload.is_empty() {
                    let label = format!("From disk… (.{})", c.upload.join(", ."));
                    if ui
                        .button(label)
                        .on_hover_text("Choose a file from disk: it is put beside the program")
                        .clicked()
                    {
                        upload = true;
                    }
                }
            });
        });
    });
    if modal.should_close() {
        keep = false;
    }
    if let Some(path) = chosen {
        window.app.dialog(&c.field, Value::Choice(path));
        keep = false;
    }
    if upload {
        window.upload(c.field.clone(), c.upload.clone());
        keep = false;
    }
    let _ = folder_of;
    keep
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn files_are_a_tree_folders_first_the_library_last() {
        let files: Vec<(String, bool)> = [
            "b.geop",
            "parts/a.geop",
            "../up.geop",
            "std:nut.geop",
            "a.step",
        ]
        .iter()
        .map(|p| (p.to_string(), p.ends_with(".geop")))
        .collect();
        let tree = tree(&files);
        let keys: Vec<&str> = tree.iter().map(Node::key).collect();
        assert_eq!(keys, ["../", "parts/", "a.step", "b.geop", "std:"]);
        let all = rows(&tree, &BTreeSet::new(), "nut", false);
        assert_eq!(
            all.iter().map(|r| r.node.key()).collect::<Vec<_>>(),
            ["std:", "std:nut.geop"]
        );
        let taken = rows(&tree, &BTreeSet::from(["parts/".to_string()]), "", true);
        assert!(taken.iter().all(|r| r.node.key() != "a.step"));
        assert!(taken.iter().any(|r| r.node.key() == "parts/a.geop"));
    }
}
