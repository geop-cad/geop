//! The dialog of the step being edited, drawn from the kernel's
//! [`Control`]s alone — so every operation gets its dialog without the app
//! knowing it — and the controls every step has: preview, OK, Cancel. In
//! the layout the step asks for: beside the viewport, or as a wide popup
//! over it for a form of tables; the fields are the same in both.
//!
//! What is typed into a field is applied with Enter, and put back when the
//! field is left without it — as the web app's fields do.

use eframe::egui::{self, Color32, RichText, Ui};
use geop_core_editor::{
    editing::{
        Action as DialogAction, Control, Number, Reference, Table, TableEdit, Tone, Unit, Value,
    },
    editor::StepState,
};
use geop_core_program::element::PickedElement;

use super::{GeopApp, explorer::Chooser, icons, theme::Theme};

/// What the dialog asks for, done once it is drawn.
enum Act {
    Dialog(String, Value),
    Download(String, String),
    Choose(Chooser),
    Preview(bool),
    Commit,
    Cancel,
}

/// How an element held is shown: its name — a whole sketch's, its
/// sketch's — behind its instance's path.
pub fn element_label(e: &PickedElement) -> String {
    let name = if e.name.is_empty() {
        e.target.name()
    } else {
        &e.name
    };
    match e.target.path() {
        "" => name.to_string(),
        path => format!("/{path}:{name}"),
    }
}

/// The dialog beside the viewport.
pub fn sidebar(window: &mut GeopApp, ui: &mut Ui) {
    let acts = {
        let window = &*window;
        let Some(step) = &window.app.step else {
            return;
        };
        head(window, ui, step, false);
        ui.separator();
        let mut acts = egui::ScrollArea::vertical()
            .auto_shrink([false, true])
            .max_height(ui.available_height() - 96.0)
            .show(ui, |ui| fields(window, ui, step))
            .inner;
        status(window, ui, step);
        footer(window, ui, step, &mut acts);
        acts
    };
    apply(window, acts);
}

/// The dialog as a wide popup over the middle of the viewport.
pub fn popup(window: &mut GeopApp, ctx: &egui::Context) {
    let mut acts = Vec::new();
    {
        let window = &*window;
        let Some(step) = &window.app.step else {
            return;
        };
        egui::Window::new(RichText::new(step.label).strong())
            .id(egui::Id::new("geop step popup"))
            .collapsible(false)
            .resizable(true)
            .default_width(760.0)
            .anchor(egui::Align2::CENTER_CENTER, egui::Vec2::ZERO)
            .frame(window.theme.card())
            .show(ctx, |ui| {
                head(window, ui, step, true);
                ui.separator();
                acts = egui::ScrollArea::vertical()
                    .max_height(ctx.content_rect().height() * 0.65)
                    .show(ui, |ui| fields(window, ui, step))
                    .inner;
                status(window, ui, step);
                footer(window, ui, step, &mut acts);
            });
    }
    apply(window, acts);
}

fn apply(window: &mut GeopApp, acts: Vec<Act>) {
    for act in acts {
        match act {
            Act::Dialog(key, value) => window.app.dialog(&key, value),
            Act::Download(exporter, format) => window.app.export(
                &exporter,
                Some(format),
                serde_json::Value::Null,
                super::today(),
            ),
            Act::Choose(chooser) => window.chooser = Some(chooser),
            Act::Preview(preview) => window
                .app
                .command(geop_core_editor::editor::Command::Preview { preview }),
            Act::Commit => window.app.commit(),
            Act::Cancel => window.app.cancel(),
        }
    }
}

fn head(window: &GeopApp, ui: &mut Ui, step: &StepState, popup: bool) {
    ui.horizontal(|ui| {
        if let Some(icon) = (window.app.program.as_ref())
            .and_then(|p| p.operations.iter().find(|o| o.kind == step.kind))
            .and_then(|o| o.info.icon)
        {
            icons::show(ui, icon, 18.0);
        }
        if !popup {
            ui.label(RichText::new(step.label).heading().strong())
                .on_hover_text(step.doc);
        }
        if let Some(id) = &step.id {
            ui.label(RichText::new(format!("· {id}")).color(window.theme.weak()));
        }
    });
    if popup {
        ui.label(RichText::new(step.doc).small().color(window.theme.weak()));
    }
}

fn status(window: &GeopApp, ui: &mut Ui, step: &StepState) {
    let theme = window.theme;
    if let Some(error) = &window.app.error {
        ui.label(RichText::new(error).color(theme.error()));
    }
    if !step.missing.is_empty() {
        ui.label(
            RichText::new(format!(
                "Pick the {} in the viewport to continue.",
                step.missing.join(" and ")
            ))
            .color(theme.accent()),
        );
    } else if let Some(error) = &step.error {
        ui.label(RichText::new(error).color(theme.error()));
    }
}

fn footer(window: &GeopApp, ui: &mut Ui, step: &StepState, acts: &mut Vec<Act>) {
    ui.add_space(4.0);
    let blocked = step.error.is_some() || !step.missing.is_empty();
    let format = |action: super::actions::Action| {
        action
            .shortcut()
            .map(|s| ui.ctx().format_shortcut(&s))
            .unwrap_or_default()
    };
    let (ok_key, cancel_key) = (
        format(super::actions::Action::Commit),
        format(super::actions::Action::Cancel),
    );
    ui.horizontal(|ui| {
        let mut preview = step.preview;
        if ui
            .checkbox(&mut preview, "Preview")
            .on_hover_text("Show the step as it builds, or the part before it")
            .changed()
        {
            acts.push(Act::Preview(preview));
        }
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let ok = egui::Button::new(RichText::new("  OK  ").strong().color(Color32::WHITE))
                .fill(window.theme.accent());
            let tip = if step.missing.is_empty() {
                format!("Put the step into the program ({ok_key})")
            } else {
                format!("Pick the {} first", step.missing.join(" and "))
            };
            if ui
                .add_enabled(!blocked, ok)
                .on_hover_text(tip.clone())
                .on_disabled_hover_text(tip)
                .clicked()
            {
                acts.push(Act::Commit);
            }
            if ui
                .button("Cancel")
                .on_hover_text(format!("Leave the program as it was ({cancel_key})"))
                .clicked()
            {
                acts.push(Act::Cancel);
            }
        });
    });
}

/// Every field of the step's dialog.
fn fields(window: &GeopApp, ui: &mut Ui, step: &StepState) -> Vec<Act> {
    let mut acts = Vec::new();
    // Nothing of the dialog widens the panel it is in: it wraps.
    ui.set_max_width(ui.available_width());
    let theme = window.theme;
    let busy = window.app.busy();
    for field in &step.presentation.dialog.0 {
        let key = field.key.as_str();
        ui.push_id(key, |ui| {
            control(ui, theme, busy, key, &field.control, &mut acts);
            // Why the step does not build with this field's value.
            if let Some(failure) = &field.failure {
                ui.label(RichText::new(failure).small().color(theme.error()));
            }
        });
        ui.add_space(2.0);
    }
    acts
}

/// A field's label beside its control, the one grid every labelled field
/// is laid out in.
fn labelled<R>(ui: &mut Ui, label: &str, unit: &str, add: impl FnOnce(&mut Ui) -> R) -> R {
    ui.horizontal(|ui| {
        let width = (ui.available_width() * 0.4).clamp(70.0, 140.0);
        ui.allocate_ui_with_layout(
            egui::vec2(width, ui.spacing().interact_size.y),
            egui::Layout::left_to_right(egui::Align::Center),
            |ui| {
                ui.set_min_width(width);
                ui.add(egui::Label::new(label).truncate())
                    .on_hover_text(label);
                if !unit.is_empty() {
                    ui.label(RichText::new(unit).small().weak());
                }
            },
        );
        add(ui)
    })
    .inner
}

fn unit(unit: Unit) -> &'static str {
    match unit {
        Unit::Length => "mm",
        Unit::Angle => "°",
        Unit::Fraction | Unit::Count => "",
    }
}

/// Text typed in place: `value` until typed into; Enter gives what is
/// typed, leaving the field puts `value` back. `None` until given.
pub fn typed(
    ui: &mut Ui,
    id: egui::Id,
    value: &str,
    edit: impl FnOnce(&mut String) -> egui::TextEdit,
) -> (Option<String>, egui::Response) {
    let mut held: (String, String) = ui.data_mut(|d| d.get_temp(id)).unwrap_or_default();
    if held.0 != value {
        held = (value.to_string(), value.to_string());
    }
    let response = ui.add(edit(&mut held.1).id(id.with("edit")));
    let mut given = None;
    if response.lost_focus() {
        if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
            given = Some(held.1.clone());
        } else {
            held.1 = held.0.clone();
        }
    }
    if response.has_focus() && ui.input(|i| i.key_pressed(egui::Key::Escape)) {
        held.1 = held.0.clone();
        response.surrender_focus();
    }
    ui.data_mut(|d| d.insert_temp(id, held));
    (given, response)
}

fn shown_number(value: f64) -> String {
    let rounded = (value * 1e6).round() / 1e6;
    if rounded == 0.0 {
        "0".into()
    } else {
        format!("{rounded}")
    }
}

/// A number typed in place, with buttons a step up and down.
fn number_input(ui: &mut Ui, id: egui::Id, value: f64, step: f64, acts: &mut Vec<Act>, key: &str) {
    ui.spacing_mut().item_spacing.x = 2.0;
    let width = (ui.available_width() - 44.0).max(50.0);
    let (given, response) = typed(ui, id, &shown_number(value), |text| {
        egui::TextEdit::singleline(text)
            .desired_width(width)
            .horizontal_align(egui::Align::RIGHT)
    });
    response.on_hover_text("Enter applies · the arrows step it");
    if let Some(text) = given
        && let Ok(v) = text.trim().parse::<f64>()
    {
        acts.push(Act::Dialog(key.to_string(), Value::Number(v)));
    }
    if icons::button(ui, "minus", &format!("− {}", shown_number(step))).clicked() {
        acts.push(Act::Dialog(key.to_string(), Value::Number(value - step)));
    }
    if icons::button(ui, "plus", &format!("+ {}", shown_number(step))).clicked() {
        acts.push(Act::Dialog(key.to_string(), Value::Number(value + step)));
    }
}

fn control(
    ui: &mut Ui,
    theme: Theme,
    busy: bool,
    key: &str,
    control: &Control,
    acts: &mut Vec<Act>,
) {
    let send = |acts: &mut Vec<Act>, value: Value| acts.push(Act::Dialog(key.to_string(), value));
    let id = ui.id().with(key);
    match control {
        Control::Heading { text } => {
            ui.add_space(4.0);
            ui.label(RichText::new(text).strong().size(13.5));
        }
        Control::Text { text, tone } => {
            let mut text = RichText::new(text);
            if let Some(color) = theme.tone(*tone) {
                text = text.color(color);
            }
            ui.add(egui::Label::new(text).wrap());
        }
        Control::Input {
            label,
            value,
            placeholder,
        } => labelled(ui, label, "", |ui| {
            let width = ui.available_width();
            let (given, _) = typed(ui, id, value, |text| {
                let edit = egui::TextEdit::singleline(text).desired_width(width);
                match placeholder {
                    Some(p) => edit.hint_text(p.as_str()),
                    None => edit,
                }
            });
            if let Some(text) = given {
                send(acts, Value::Text(text));
            }
        }),
        Control::Reference(reference) => self::reference(ui, theme, key, reference, acts),
        Control::Actions { actions } => self::actions(ui, key, actions, acts),
        Control::Download { exporter, formats } => {
            ui.horizontal_wrapped(|ui| {
                for format in formats {
                    let b = match icons::image("save", 14.0, ui.visuals().text_color()) {
                        Some(image) => egui::Button::image_and_text(
                            image,
                            format!("Download {}", format.label),
                        ),
                        None => egui::Button::new(format!("Download {}", format.label)),
                    };
                    if ui
                        .add(b)
                        .on_hover_text(format!("Save as {}", format.label))
                        .clicked()
                    {
                        acts.push(Act::Download(exporter.clone(), format.value.clone()));
                    }
                }
            });
        }
        Control::Checkbox { label, value } => labelled(ui, label, "", |ui| {
            let mut checked = *value;
            if ui.checkbox(&mut checked, "").changed() {
                send(acts, Value::Bool(checked));
            }
        }),
        Control::Number(number) => self::number(ui, theme, id, key, number, acts),
        Control::Select {
            label,
            value,
            options,
            searchable,
        } => labelled(ui, label, "", |ui| {
            let current = options
                .iter()
                .find(|o| o.value == *value)
                .map_or(value.as_str(), |o| o.label.as_str());
            let width = ui.available_width();
            egui::ComboBox::from_id_salt(id)
                .selected_text(if current.is_empty() { "—" } else { current })
                .width(width)
                .height(320.0)
                .show_ui(ui, |ui| {
                    let mut query = String::new();
                    if *searchable {
                        query = ui
                            .data_mut(|d| d.get_temp::<String>(id.with("q")))
                            .unwrap_or_default();
                        let r = ui.add(egui::TextEdit::singleline(&mut query).hint_text("Search…"));
                        if !r.has_focus() && query.is_empty() {
                            r.request_focus();
                        }
                        ui.data_mut(|d| d.insert_temp(id.with("q"), query.clone()));
                    }
                    let q = query.to_lowercase();
                    let mut first = None;
                    for option in options
                        .iter()
                        .filter(|o| q.is_empty() || o.label.to_lowercase().contains(&q))
                        .take(200)
                    {
                        first.get_or_insert(option.value.clone());
                        let text = if option.label.is_empty() {
                            "—"
                        } else {
                            option.label.as_str()
                        };
                        if ui.selectable_label(option.value == *value, text).clicked() {
                            send(acts, Value::Choice(option.value.clone()));
                        }
                    }
                    if *searchable
                        && ui.input(|i| i.key_pressed(egui::Key::Enter))
                        && let Some(first) = first
                    {
                        send(acts, Value::Choice(first));
                        ui.close();
                    }
                });
        }),
        Control::File {
            label,
            value,
            files,
            upload,
        } => labelled(ui, label, "", |ui| {
            let text = if value.is_empty() {
                "choose a file…".to_string()
            } else {
                value.clone()
            };
            let width = ui.available_width();
            let b = egui::Button::new(RichText::new(format!("{text}  …")).monospace())
                .min_size(egui::vec2(width, 0.0));
            if ui
                .add(b)
                .on_hover_text(if value.is_empty() {
                    "Browse the files".into()
                } else {
                    format!("{value} — browse for another")
                })
                .clicked()
            {
                acts.push(Act::Choose(Chooser::new(
                    key,
                    label,
                    value,
                    files.clone(),
                    upload.clone(),
                )));
            }
        }),
        Control::Color { label, value } => labelled(ui, label, "", |ui| {
            let parse = |v: &str| -> Option<[u8; 3]> {
                let hex = v.strip_prefix('#').filter(|h| h.len() == 6)?;
                let n = u32::from_str_radix(hex, 16).ok()?;
                Some([(n >> 16) as u8, (n >> 8) as u8, n as u8])
            };
            // Shown at once as it is dragged; given on one at a time.
            let mut rgb: [u8; 3] = ui
                .data_mut(|d| d.get_temp(id))
                .filter(|_| busy)
                .or_else(|| parse(value))
                .unwrap_or([128; 3]);
            let changed = ui.color_edit_button_srgb(&mut rgb).changed();
            ui.label(RichText::new(value).monospace().weak());
            ui.data_mut(|d| d.insert_temp(id, rgb));
            let hex = format!("#{:02x}{:02x}{:02x}", rgb[0], rgb[1], rgb[2]);
            let waiting = ui
                .data_mut(|d| d.get_temp::<bool>(id.with("waiting")))
                .unwrap_or(false);
            if (changed || waiting) && hex != *value {
                if busy {
                    ui.data_mut(|d| d.insert_temp(id.with("waiting"), true));
                } else {
                    ui.data_mut(|d| d.insert_temp(id.with("waiting"), false));
                    send(acts, Value::Text(hex));
                }
            }
        }),
        Control::List { items, empty } => {
            if items.is_empty() {
                ui.label(RichText::new(empty).color(theme.weak()));
            }
            for item in items {
                ui.push_id(&item.key, |ui| {
                    let frame = egui::Frame::new()
                        .inner_margin(egui::Margin::symmetric(4, 2))
                        .corner_radius(4)
                        .fill(if item.selected {
                            theme.accent().gamma_multiply(0.25)
                        } else {
                            Color32::TRANSPARENT
                        });
                    let r = frame.show(ui, |ui| {
                        ui.horizontal(|ui| {
                            let mut label = RichText::new(&item.label);
                            if item.tone == Tone::Error {
                                label = label.color(theme.error());
                            }
                            let clicked = ui
                                .add(
                                    egui::Label::new(label)
                                        .truncate()
                                        .sense(egui::Sense::click()),
                                )
                                .on_hover_text(&item.label)
                                .clicked();
                            if clicked {
                                acts.push(Act::Dialog(item.key.clone(), Value::Press));
                            }
                            if let Some(detail) = &item.detail {
                                ui.add(
                                    egui::Label::new(
                                        RichText::new(detail).small().color(theme.weak()),
                                    )
                                    .truncate(),
                                )
                                .on_hover_text(detail);
                            }
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    if item.removable
                                        && icons::button(ui, "close", "Remove").clicked()
                                    {
                                        acts.push(Act::Dialog(item.key.clone(), Value::Remove));
                                    }
                                    let id = ui.id().with("value");
                                    if let Some(text) = &item.text {
                                        let (given, _) = typed(ui, id, text, |t| {
                                            egui::TextEdit::singleline(t)
                                                .desired_width(90.0)
                                                .font(egui::TextStyle::Monospace)
                                        });
                                        if let Some(text) = given {
                                            acts.push(Act::Dialog(
                                                item.key.clone(),
                                                Value::Text(text),
                                            ));
                                        }
                                    } else if let Some(value) = item.value {
                                        let (given, _) = typed(ui, id, &shown_number(value), |t| {
                                            egui::TextEdit::singleline(t).desired_width(70.0)
                                        });
                                        if let Some(v) =
                                            given.and_then(|t| t.trim().parse::<f64>().ok())
                                        {
                                            acts.push(Act::Dialog(
                                                item.key.clone(),
                                                Value::Number(v),
                                            ));
                                        }
                                    }
                                },
                            );
                        });
                    });
                    if r.response.interact(egui::Sense::click()).clicked() {
                        acts.push(Act::Dialog(item.key.clone(), Value::Press));
                    }
                });
            }
        }
        Control::Table(table) => self::table(ui, theme, id, table, &mut |edit| {
            acts.push(Act::Dialog(key.to_string(), Value::Table(edit)))
        }),
    }
}

fn number(
    ui: &mut Ui,
    theme: Theme,
    id: egui::Id,
    key: &str,
    number: &Number,
    acts: &mut Vec<Act>,
) {
    match &number.text {
        Some(text) => {
            let formula = !text.trim().is_empty() && text.trim().parse::<f64>().is_err();
            labelled(ui, &number.label, unit(number.unit), |ui| {
                ui.label(RichText::new("ƒ").italics().color(theme.accent()))
                    .on_hover_text("A number, or a formula of the parameters: width / 2, screw.d · Enter applies");
                let width = ui.available_width();
                let (given, _) = typed(ui, id, text, |t| {
                    egui::TextEdit::singleline(t)
                        .desired_width(width)
                        .font(egui::TextStyle::Monospace)
                });
                if let Some(text) = given.filter(|t| !t.trim().is_empty()) {
                    acts.push(Act::Dialog(key.to_string(), Value::Text(text)));
                }
            });
            if formula && number.error.is_none() {
                ui.label(
                    RichText::new(format!("= {}", shown_number(number.value)))
                        .small()
                        .color(theme.weak()),
                );
            }
            if let Some(error) = &number.error {
                ui.label(RichText::new(error).small().color(theme.error()));
            }
        }
        None => labelled(ui, &number.label, unit(number.unit), |ui| {
            number_input(ui, id, number.value, number.step, acts, key);
        }),
    }
}

fn reference(ui: &mut Ui, theme: Theme, key: &str, r: &Reference, acts: &mut Vec<Act>) {
    let send = |acts: &mut Vec<Act>, value: Value| acts.push(Act::Dialog(key.to_string(), value));
    let text = match (r.value.as_slice(), r.multiple, r.armed) {
        ([], _, true) => "click in the viewport…".to_string(),
        ([], _, false) => "pick…".to_string(),
        (held, true, true) => format!("{} picked — click to add or take out…", held.len()),
        (held, true, false) => format!("{} picked — pick more…", held.len()),
        ([first, ..], false, _) => element_label(&first.element),
    };
    labelled(ui, &r.label, "", |ui| {
        let width = ui.available_width() - if r.value.is_empty() { 0.0 } else { 26.0 };
        let tint = if r.armed {
            theme.accent()
        } else {
            ui.visuals().text_color()
        };
        let mut button = match icons::image("pick", 14.0, tint) {
            Some(image) => egui::Button::image_and_text(image, RichText::new(text).color(tint)),
            None => egui::Button::new(text),
        };
        button = button
            .min_size(egui::vec2(width.max(40.0), 0.0))
            .selected(r.armed);
        if !r.value.is_empty() && !r.armed {
            button = button.stroke(egui::Stroke::new(1.0, theme.accent().gamma_multiply(0.6)));
        }
        let tip = if r.multiple {
            "Pick in the viewport — again to take out"
        } else {
            "Pick in the viewport"
        };
        if ui.add(button).on_hover_text(tip).clicked() {
            send(acts, Value::Press);
        }
        if !r.value.is_empty()
            && icons::button(
                ui,
                "close",
                if r.multiple {
                    "Take everything out"
                } else {
                    "Take it out"
                },
            )
            .clicked()
        {
            send(acts, Value::Clear);
        }
    });
    let failed = r.value.iter().find(|p| p.tone == Tone::Error);
    if let (Some(failed), false) = (failed, r.multiple)
        && let Some(detail) = &failed.detail
    {
        ui.label(RichText::new(detail).small().color(theme.error()));
    }
    if r.multiple && !r.value.is_empty() {
        egui::Frame::new()
            .inner_margin(egui::Margin {
                left: 12,
                ..Default::default()
            })
            .show(ui, |ui| {
                for (i, picked) in r.value.iter().enumerate() {
                    ui.horizontal(|ui| {
                        let label = element_label(&picked.element);
                        let mut text = RichText::new(&label).small();
                        if picked.tone == Tone::Error {
                            text = text.color(theme.error());
                        }
                        ui.add(egui::Label::new(text).truncate())
                            .on_hover_text(&label);
                        if let Some(detail) = &picked.detail {
                            ui.label(RichText::new(detail).small().color(theme.weak()));
                        }
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if icons::button(ui, "close", "Take out").clicked() {
                                send(acts, Value::RemoveAt(i));
                            }
                        });
                    });
                }
            });
    }
}

/// Actions grouped as their groups say, in the order the groups first come.
fn groups(actions: &[DialogAction]) -> Vec<(Option<&str>, Vec<&DialogAction>)> {
    let mut out: Vec<(Option<&str>, Vec<&DialogAction>)> = Vec::new();
    for action in actions {
        match out.last_mut() {
            Some((group, list)) if *group == action.group.as_deref() => list.push(action),
            _ => out.push((action.group.as_deref(), vec![action])),
        }
    }
    out
}

fn actions(ui: &mut Ui, key: &str, actions: &[DialogAction], acts: &mut Vec<Act>) {
    let press = |ui: &mut Ui, a: &DialogAction, acts: &mut Vec<Act>, icon_only: bool| {
        let tint = if a.active {
            ui.visuals().selection.stroke.color
        } else {
            ui.visuals().text_color()
        };
        let button = match (a.icon, icon_only) {
            (Some(icon), true) => match icons::image(icon, 18.0, tint) {
                Some(image) => egui::Button::image(image).min_size(egui::vec2(28.0, 26.0)),
                None => egui::Button::new(&a.label),
            },
            _ => egui::Button::new(&a.label),
        };
        let tip = a.title.clone().unwrap_or_else(|| a.label.clone());
        let r = ui
            .add_enabled(a.enabled, button.selected(a.active))
            .on_hover_text(tip.clone())
            .on_disabled_hover_text(tip);
        if r.clicked() {
            acts.push(Act::Dialog(key.to_string(), Value::Choice(a.value.clone())));
        }
    };
    if actions.iter().all(|a| a.icon.is_some()) {
        // A group under its name, its buttons wrapped to the dialog's width.
        for (group, list) in groups(actions) {
            if let Some(group) = group {
                ui.label(RichText::new(group).small().weak());
            }
            ui.horizontal_wrapped(|ui| {
                ui.spacing_mut().item_spacing = egui::vec2(2.0, 2.0);
                for a in list {
                    press(ui, a, acts, true);
                }
            });
            ui.add_space(3.0);
        }
    } else if actions.iter().all(|a| a.group.is_none()) {
        ui.horizontal_wrapped(|ui| {
            for a in actions {
                press(ui, a, acts, false);
            }
        });
    } else {
        for (group, list) in groups(actions) {
            if let Some(group) = group {
                ui.label(RichText::new(group).small().weak());
            }
            ui.horizontal_wrapped(|ui| {
                for a in list {
                    press(ui, a, acts, false);
                }
            });
        }
    }
}

/// A block of text pasted, as rows of cells: columns split at tabs, rows
/// at line ends, as a spreadsheet copies them; `None` for a single cell,
/// which is typed as any text is (as `web/src/tableClipboard.ts`).
pub fn parse_block(text: &str) -> Option<Vec<Vec<String>>> {
    let text = text.replace("\r\n", "\n").replace('\r', "\n");
    let text = text.strip_suffix('\n').unwrap_or(&text);
    let cells: Vec<Vec<String>> = text
        .split('\n')
        .map(|l| l.split('\t').map(str::to_string).collect())
        .collect();
    (cells.len() > 1 || cells[0].len() > 1).then_some(cells)
}

/// A table edited in place: a header per column, an input per cell, the
/// row chosen marked, rows added, moved and taken out by their buttons, a
/// block pasted from a spreadsheet sent as one paste.
fn table(ui: &mut Ui, theme: Theme, id: egui::Id, table: &Table, send: &mut dyn FnMut(TableEdit)) {
    let rows = table.rows.len();
    if rows == 0 {
        ui.label(RichText::new(&table.empty).color(theme.weak()));
    }
    let actions = table.rows_editable || table.rows.iter().any(|r| r.removable);
    egui::ScrollArea::horizontal()
        .id_salt(id.with("scroll"))
        .show(ui, |ui| {
            egui::Grid::new(id.with("grid"))
                .striped(true)
                .spacing([4.0, 3.0])
                .show(ui, |ui| {
                    if table.selectable {
                        ui.label("✓");
                    }
                    for (k, column) in table.columns.iter().enumerate() {
                        let angle = matches!(
                            column.kind,
                            geop_core_editor::editing::ColumnKind::Number { unit: Unit::Angle }
                        );
                        let label = format!("{}{}", column.label, if angle { " (°)" } else { "" });
                        if column.editable {
                            ui.horizontal(|ui| {
                                let (given, _) =
                                    typed(ui, id.with(("head", k)), &column.label, |t| {
                                        egui::TextEdit::singleline(t).desired_width(80.0)
                                    });
                                if let Some(text) = given {
                                    send(TableEdit::Column { column: k, text });
                                }
                                if icons::button(
                                    ui,
                                    "close",
                                    &format!("Remove the column {}", column.label),
                                )
                                .clicked()
                                {
                                    send(TableEdit::RemoveColumn { column: k });
                                }
                            });
                        } else {
                            ui.label(RichText::new(label).strong());
                        }
                    }
                    if table.columns_editable
                        && ui
                            .small_button("+ column")
                            .on_hover_text("Add a column")
                            .clicked()
                    {
                        send(TableEdit::AddColumn);
                    }
                    if actions {
                        ui.label("");
                    }
                    ui.end_row();
                    for (row, r) in table.rows.iter().enumerate() {
                        if table.selectable
                            && ui
                                .radio(r.selected, "")
                                .on_hover_text("Choose this row")
                                .clicked()
                        {
                            send(TableEdit::SelectRow { row });
                        }
                        for (column, cell) in r.cells.iter().enumerate() {
                            let editable =
                                table.columns.get(column).is_some_and(|c| c.editable) || true;
                            ui.vertical(|ui| {
                                let cell_id = id.with(("cell", row, column));
                                if editable {
                                    let pasted = ui.input(|i| {
                                        i.events.iter().find_map(|e| match e {
                                            egui::Event::Paste(text) => Some(text.clone()),
                                            _ => None,
                                        })
                                    });
                                    let focused = ui.memory(|m| m.has_focus(cell_id.with("edit")));
                                    if focused
                                        && let Some(cells) = pasted.as_deref().and_then(parse_block)
                                    {
                                        ui.memory_mut(|m| m.surrender_focus(cell_id.with("edit")));
                                        send(TableEdit::Paste { row, column, cells });
                                    }
                                    let (given, response) = typed(ui, cell_id, &cell.text, |t| {
                                        let mut edit =
                                            egui::TextEdit::singleline(t).desired_width(90.0);
                                        if cell.tone == Tone::Error {
                                            edit = edit.text_color(theme.error());
                                        }
                                        edit
                                    });
                                    if let Some(detail) = &cell.detail {
                                        response.on_hover_text(detail);
                                    }
                                    if let Some(text) = given {
                                        send(TableEdit::Cell { row, column, text });
                                        if row + 1 == rows && table.rows_editable {
                                            // Enter in the last row adds one below.
                                        }
                                    }
                                }
                                if let Some(detail) = &cell.detail {
                                    let mut text = RichText::new(detail).small();
                                    if let Some(color) = theme.tone(cell.tone) {
                                        text = text.color(color);
                                    }
                                    ui.label(text);
                                }
                            });
                        }
                        if table.columns_editable {
                            ui.label("");
                        }
                        if actions {
                            ui.horizontal(|ui| {
                                ui.spacing_mut().item_spacing.x = 1.0;
                                if table.rows_editable {
                                    if icons::button_enabled(ui, row > 0, "chevron-up", "Move up")
                                        .clicked()
                                    {
                                        send(TableEdit::MoveRow { row, to: row - 1 });
                                    }
                                    if icons::button_enabled(
                                        ui,
                                        row + 1 < rows,
                                        "chevron-down",
                                        "Move down",
                                    )
                                    .clicked()
                                    {
                                        send(TableEdit::MoveRow { row, to: row + 1 });
                                    }
                                }
                                if r.removable
                                    && icons::button(ui, "close", "Remove the row").clicked()
                                {
                                    send(TableEdit::RemoveRow { row });
                                }
                            });
                        }
                        ui.end_row();
                    }
                });
        });
    if table.rows_editable && ui.button("+ row").on_hover_text("Add a row").clicked() {
        send(TableEdit::AddRow);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_block_is_split_into_rows_of_cells() {
        assert_eq!(
            parse_block("1\t2\n3\t4\n"),
            Some(vec![
                vec!["1".into(), "2".into()],
                vec!["3".into(), "4".into()]
            ])
        );
        assert_eq!(parse_block("one"), None);
        assert_eq!(
            parse_block("a\r\nb"),
            Some(vec![vec!["a".into()], vec!["b".into()]])
        );
    }
}
