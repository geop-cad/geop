//! The command palette (Ctrl+K): every operation, example, file, tool,
//! inspector and action of the app, found by typing a few of its letters,
//! moved through by the arrow keys and done with Enter — and the list of
//! the keyboard's shortcuts (F1).

use eframe::egui::{self, RichText};

use super::{
    GeopApp,
    actions::{Action, palette, score},
    icons,
};

#[derive(Default)]
pub struct Palette {
    pub open: bool,
    query: String,
    selected: usize,
    focus: bool,
    /// An action asked for from somewhere that cannot do it itself.
    pub pending: Option<Action>,
}

impl Palette {
    pub fn toggle(&mut self) {
        self.open = !self.open;
        self.query.clear();
        self.selected = 0;
        self.focus = true;
    }
}

pub fn show(window: &mut GeopApp, ctx: &egui::Context) {
    if let Some(action) = window.palette.pending.take() {
        window.act(ctx, action);
    }
    if !window.palette.open {
        return;
    }
    let theme = window.theme;
    let mut entries: Vec<(i64, usize, super::actions::Entry)> = palette(&window.app)
        .into_iter()
        .enumerate()
        .filter_map(|(i, e)| {
            let text = format!("{} {}", e.label, e.group);
            score(&window.palette.query, &text).map(|s| (s, i, e))
        })
        .collect();
    entries.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    entries.truncate(60);
    let p = &mut window.palette;
    p.selected = p.selected.min(entries.len().saturating_sub(1));
    let mut run = None;
    let mut close = false;
    egui::Area::new(egui::Id::new("geop palette"))
        .anchor(egui::Align2::CENTER_TOP, egui::vec2(0.0, 70.0))
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            theme.card().show(ui, |ui| {
                ui.set_width(520.0);
                let r = ui.add(
                    egui::TextEdit::singleline(&mut p.query)
                        .hint_text("Type a command, an operation, an example, a file…")
                        .desired_width(f32::INFINITY)
                        .font(egui::TextStyle::Heading),
                );
                if p.focus {
                    r.request_focus();
                    p.focus = false;
                }
                if r.changed() {
                    p.selected = 0;
                }
                ui.input(|i| {
                    if i.key_pressed(egui::Key::ArrowDown) {
                        p.selected = (p.selected + 1).min(entries.len().saturating_sub(1));
                    }
                    if i.key_pressed(egui::Key::ArrowUp) {
                        p.selected = p.selected.saturating_sub(1);
                    }
                    if i.key_pressed(egui::Key::Enter) {
                        run = entries.get(p.selected).map(|e| e.2.action.clone());
                    }
                    if i.key_pressed(egui::Key::Escape) {
                        close = true;
                    }
                });
                ui.separator();
                egui::ScrollArea::vertical()
                    .max_height(380.0)
                    .show(ui, |ui| {
                        for (k, (_, _, entry)) in entries.iter().enumerate() {
                            let enabled = entry.action.enabled(&window.app);
                            let selected = k == p.selected;
                            let response = ui
                                .horizontal(|ui| {
                                    let tint = if enabled {
                                        ui.visuals().text_color()
                                    } else {
                                        theme.weak()
                                    };
                                    match entry
                                        .icon
                                        .as_deref()
                                        .and_then(|i| icons::image(i, 16.0, tint))
                                    {
                                        Some(image) => {
                                            ui.add(image);
                                        }
                                        None => {
                                            ui.add_space(20.0);
                                        }
                                    }
                                    let mut label = RichText::new(&entry.label);
                                    if !enabled {
                                        label = label.color(theme.weak());
                                    }
                                    let r = ui.add(
                                        egui::Button::selectable(selected, label)
                                            .frame_when_inactive(false),
                                    );
                                    ui.with_layout(
                                        egui::Layout::right_to_left(egui::Align::Center),
                                        |ui| {
                                            if let Some(s) = entry.action.shortcut() {
                                                ui.label(
                                                    RichText::new(ctx.format_shortcut(&s))
                                                        .monospace()
                                                        .small()
                                                        .color(theme.weak()),
                                                );
                                            }
                                            ui.label(
                                                RichText::new(entry.group)
                                                    .small()
                                                    .color(theme.weak()),
                                            );
                                        },
                                    );
                                    r
                                })
                                .inner;
                            let response = if entry.detail.is_empty() {
                                response
                            } else {
                                response.on_hover_text(&entry.detail)
                            };
                            if selected {
                                response.scroll_to_me(None);
                            }
                            if response.clicked() {
                                run = Some(entry.action.clone());
                            }
                        }
                        if entries.is_empty() {
                            ui.label(RichText::new("Nothing matches.").color(theme.weak()));
                        }
                    });
            });
        });
    if ctx.input(|i| i.pointer.any_pressed())
        && let Some(at) = ctx.input(|i| i.pointer.interact_pos())
        && ctx
            .layer_id_at(at)
            .is_none_or(|l| l.id != egui::Id::new("geop palette"))
    {
        close = true;
    }
    if let Some(action) = run {
        window.palette.open = false;
        window.act(ctx, action);
    } else if close {
        window.palette.open = false;
    }
}

/// The keyboard's shortcuts, and what the pointer does in the view.
pub fn shortcuts(window: &mut GeopApp, ctx: &egui::Context) {
    let theme = window.theme;
    let mut open = true;
    egui::Window::new("Keyboard and pointer")
        .open(&mut open)
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
        .frame(theme.card())
        .show(ctx, |ui| {
            egui::Grid::new("shortcuts")
                .num_columns(2)
                .spacing([16.0, 6.0])
                .show(ui, |ui| {
                    let mut row = |keys: String, what: &str| {
                        ui.label(RichText::new(keys).monospace().color(theme.accent()));
                        ui.label(what);
                        ui.end_row();
                    };
                    let named = [
                        (
                            Action::Palette,
                            "Command palette: every operation and action",
                        ),
                        (Action::Undo, "Undo — a step's own edits while one is open"),
                        (Action::Redo, "Redo (Ctrl+Y too)"),
                        (Action::Commit, "OK: put the step into the program"),
                        (Action::Cancel, "Cancel the step"),
                        (
                            Action::CopyStep,
                            "Copy the step edited, to paste here or in another file",
                        ),
                        (Action::Paste, "Paste steps where new steps go"),
                        (Action::NewPart, "New part"),
                        (Action::OpenFolder, "Open a folder"),
                        (Action::Fit, "Fit the part in the view (no step open)"),
                        (Action::ResetView, "Reset the view (no step open)"),
                        (Action::ToggleProjection, "Perspective / orthographic"),
                        (Action::ToggleSection, "Section view"),
                        (Action::TogglePreview, "Preview the step as it builds"),
                        (Action::ToggleFiles, "Show / hide the files"),
                        (Action::ToggleTheme, "Dark / light theme"),
                        (Action::Reread, "Read the folder again"),
                        (Action::Shortcuts, "This list"),
                    ];
                    for (action, what) in named {
                        if let Some(s) = action.shortcut() {
                            row(ctx.format_shortcut(&s), what);
                        }
                    }
                    row(
                        "Esc".into(),
                        "End what is in progress; with nothing, put the tool down",
                    );
                    row(
                        "Enter".into(),
                        "Finish what is in progress: a chain drawn, a constraint",
                    );
                    row("Delete".into(), "Delete the selection");
                    row(
                        "letters".into(),
                        "A step's tools, as its dialog's buttons say",
                    );
                    row(
                        "left drag".into(),
                        "Turn the view — pan, working in a plane",
                    );
                    row("right / middle drag".into(), "Pan");
                    row("Shift+drag".into(), "Pan; Shift held turns snapping off");
                    row("wheel".into(), "Zoom to the pointer");
                    row(
                        "double click".into(),
                        "As a step reads it: select a chain, edit a dimension",
                    );
                });
        });
    if !open {
        window.shortcuts_open = false;
    }
}
