//! Beside the viewport while no step is edited: the part's structure —
//! its solids, sketches, datums, placed parts and what its steps list,
//! grouped as the kernel labels them, each shown or hidden by its eye and
//! lighting what it refers to while pointed at — and the inspectors: the
//! tools to take in hand, the questions to ask, a section view, and every
//! answer as the kernel lays it out.

use eframe::egui::{self, RichText, Ui};
use geop_core_editor::{
    editor::Command,
    output::{Block, Does, Inspection, Row},
};
use geop_core_program::element::PickedElement;

use super::{GeopApp, icons, theme::Theme};
use crate::controller::{Section, SectionAxis};

pub fn part(window: &mut GeopApp, ui: &mut Ui) {
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            let mut lights: Option<Vec<PickedElement>> = None;
            egui::CollapsingHeader::new(RichText::new("Part").strong())
                .default_open(true)
                .show(ui, |ui| structure(window, ui, &mut lights));
            ui.add_space(6.0);
            egui::CollapsingHeader::new(RichText::new("Inspect").strong())
                .default_open(true)
                .show(ui, |ui| inspect(window, ui, &mut lights));
            // Lit while pointed at, and no longer once not.
            let lights = lights.unwrap_or_default();
            if window.app.lights != lights {
                window.app.lights = lights;
            }
        });
}

fn structure(window: &mut GeopApp, ui: &mut Ui, lights: &mut Option<Vec<PickedElement>>) {
    let theme = window.theme;
    let items = &window.app.scene.structure;
    let no_details = Default::default();
    let details = (window.app.program.as_ref()).map_or(&no_details, |p| &p.details);
    if items.is_empty() {
        ui.label(RichText::new("Nothing yet.").color(theme.weak()));
        return;
    }
    let mut groups: Vec<(String, Vec<&geop_core_editor::registry::StructureItem>)> = Vec::new();
    for item in items {
        match groups.iter_mut().find(|(k, _)| *k == item.kind_label) {
            Some((_, list)) => list.push(item),
            None => groups.push((item.kind_label.clone(), vec![item])),
        }
    }
    let enabled = window.app.ready();
    let mut visibility = None;
    for (title, list) in groups {
        egui::CollapsingHeader::new(format!("{title}  {}", list.len()))
            .id_salt(&title)
            .default_open(true)
            .show(ui, |ui| {
                for item in list {
                    let row = ui.horizontal(|ui| {
                        let mut text = RichText::new(item.label.as_deref().unwrap_or(&item.name));
                        if item.visible == Some(false) {
                            text = text.color(theme.weak());
                        }
                        ui.add(egui::Label::new(text).truncate())
                            .on_hover_text(&item.name);
                        if let Some(detail) = details.get(&item.name) {
                            ui.label(RichText::new(detail).small().color(theme.weak()));
                        }
                        if let Some(visible) = item.visible {
                            ui.with_layout(
                                egui::Layout::right_to_left(egui::Align::Center),
                                |ui| {
                                    let icon = if visible { "eye" } else { "eye-shut" };
                                    let tint = if visible {
                                        ui.visuals().text_color()
                                    } else {
                                        theme.weak()
                                    };
                                    if let Some(image) = icons::image(icon, 15.0, tint) {
                                        let tip = format!(
                                            "{} {}",
                                            if visible { "Hide" } else { "Show" },
                                            item.name
                                        );
                                        if ui
                                            .add_enabled(
                                                enabled,
                                                egui::Button::image(image).frame(false),
                                            )
                                            .on_hover_text(tip)
                                            .clicked()
                                        {
                                            visibility = Some((item.name.clone(), !visible));
                                        }
                                    }
                                },
                            );
                        }
                    });
                    if row.response.contains_pointer() && !item.elements.is_empty() {
                        *lights = Some(item.elements.clone());
                    }
                }
            });
    }
    if let Some((name, visible)) = visibility {
        window.app.command(Command::Visibility { name, visible });
    }
}

fn inspect(window: &mut GeopApp, ui: &mut Ui, lights: &mut Option<Vec<PickedElement>>) {
    let theme = window.theme;
    let app = &window.app;
    let Some(program) = &app.program else { return };
    let enabled = app.ready() && app.step.is_none();
    let in_hand = program.tool.clone();
    let busy = app.busy();
    let mut command = None;
    let mut ask = None;
    let mut section = app.section;
    ui.horizontal_wrapped(|ui| {
        let button = |ui: &mut Ui,
                      icon: Option<&str>,
                      label: &str,
                      doc: &str,
                      active: bool,
                      enabled: bool| {
            let tint = if active {
                theme.accent()
            } else {
                ui.visuals().text_color()
            };
            let b = match icon.and_then(|i| icons::image(i, 16.0, tint)) {
                Some(image) => egui::Button::image_and_text(image, label),
                None => egui::Button::new(label),
            };
            ui.add_enabled(enabled, b.selected(active))
                .on_hover_text(doc)
                .clicked()
        };
        for tool in &program.tools {
            let active = in_hand.as_deref() == Some(tool.key);
            if button(ui, tool.icon, tool.label, tool.doc, active, enabled) {
                command = Some(Command::Tool {
                    key: (!active).then(|| tool.key.to_string()),
                });
            }
        }
        for inspector in &program.inspectors {
            let picks = inspector.picks.is_some();
            let key = format!("pick:{}", inspector.kind);
            let active = picks && in_hand.as_deref() == Some(key.as_str());
            if button(
                ui,
                inspector.icon,
                inspector.label,
                inspector.doc,
                active,
                enabled && (picks || !busy),
            ) {
                if picks {
                    command = Some(Command::Tool {
                        key: (!active).then_some(key),
                    });
                } else {
                    ask = Some((inspector.kind.to_string(), serde_json::Value::Null));
                }
            }
        }
        if button(
            ui,
            Some("section"),
            "Section",
            "Section view: cut the view along a plane",
            section.is_some(),
            true,
        ) {
            section = match section {
                Some(_) => None,
                None => Some(Section {
                    axis: SectionAxis::Z,
                    offset: 0.0,
                    flip: false,
                    plane: None,
                }),
            };
        }
    });
    let picked_plane = (app.picked.as_ref())
        .and_then(|p| p.shown.plane.as_ref())
        .map(crate::scene::Frame::of);
    let mut exports = None;
    if let Some(picked) = &app.picked {
        result(
            ui,
            theme,
            picked,
            enabled,
            busy,
            &mut ask,
            &mut exports,
            lights,
        );
    }
    if let Some(s) = &mut section {
        egui::Frame::group(ui.style()).show(ui, |ui| {
            ui.horizontal(|ui| {
                for (axis, label) in [
                    (SectionAxis::X, "X"),
                    (SectionAxis::Y, "Y"),
                    (SectionAxis::Z, "Z"),
                    (SectionAxis::Picked, "Picked plane"),
                ] {
                    let allowed = axis != SectionAxis::Picked || picked_plane.is_some();
                    let tip = if axis == SectionAxis::Picked {
                        "The plane of the face or datum plane picked to inspect".to_string()
                    } else {
                        format!("Across {label}")
                    };
                    if ui
                        .add_enabled(allowed, egui::Button::new(label).selected(s.axis == axis))
                        .on_hover_text(tip)
                        .clicked()
                    {
                        s.axis = axis;
                        s.offset = 0.0;
                        s.plane = if axis == SectionAxis::Picked {
                            picked_plane
                        } else {
                            None
                        };
                    }
                }
                if ui
                    .button("Flip")
                    .on_hover_text("Cut away the other side")
                    .clicked()
                {
                    s.flip = !s.flip;
                }
            });
            let size = window.app.scene.part.size.max(1e-6);
            ui.add(
                egui::Slider::new(&mut s.offset, -size / 2.0..=size / 2.0)
                    .suffix(" mm")
                    .text("offset"),
            );
        });
    }
    for inspection in app.answers.values() {
        result(
            ui,
            theme,
            inspection,
            enabled,
            busy,
            &mut ask,
            &mut exports,
            lights,
        );
    }
    let app = &mut window.app;
    app.section = section;
    if let Some(command) = command {
        app.command(command);
    }
    if let Some((kind, options)) = ask {
        app.inspect(&kind, options);
    }
    if let Some((exporter, options)) = exports {
        app.export(&exporter, None, options, super::today());
    }
}

/// An inspector's answer: as it is shown, or — showing nothing — its
/// report as it comes.
#[allow(clippy::too_many_arguments)]
fn result(
    ui: &mut Ui,
    theme: Theme,
    inspection: &Inspection,
    enabled: bool,
    busy: bool,
    ask: &mut Option<(String, serde_json::Value)>,
    exports: &mut Option<(String, serde_json::Value)>,
    lights: &mut Option<Vec<PickedElement>>,
) {
    egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.set_min_width(ui.available_width());
        if inspection.shown.blocks.is_empty() {
            let text = serde_json::to_string_pretty(&inspection.report).unwrap_or_default();
            ui.add(egui::Label::new(RichText::new(text).monospace().small()).wrap());
            return;
        }
        for (i, block) in inspection.shown.blocks.iter().enumerate() {
            ui.push_id(i, |ui| {
                self::block(
                    ui,
                    theme,
                    inspection.kind,
                    block,
                    enabled,
                    busy,
                    ask,
                    exports,
                    lights,
                )
            });
        }
    });
}

fn row_text(row: &Row) -> String {
    row.label
        .iter()
        .chain(&row.cells)
        .cloned()
        .collect::<Vec<_>>()
        .join(" ")
}

#[allow(clippy::too_many_arguments)]
fn block(
    ui: &mut Ui,
    theme: Theme,
    kind: &str,
    block: &Block,
    enabled: bool,
    busy: bool,
    ask: &mut Option<(String, serde_json::Value)>,
    exports: &mut Option<(String, serde_json::Value)>,
    lights: &mut Option<Vec<PickedElement>>,
) {
    let mut point = |response: egui::Response, row: &Row| {
        let response = match &row.detail {
            Some(detail) => response.on_hover_text(detail),
            None => response,
        };
        if response.contains_pointer() && !row.light.is_empty() {
            *lights = Some(row.light.clone());
        }
    };
    match block {
        Block::Text { text, tone } => {
            let color = theme.tone(*tone).unwrap_or(theme.weak());
            ui.add(egui::Label::new(RichText::new(text).color(color)).wrap());
        }
        Block::List { items } => {
            for item in items {
                let mut text = RichText::new(row_text(item));
                if let Some(color) = theme.tone(item.tone) {
                    text = text.color(color);
                }
                let r = ui.add(egui::Label::new(text).wrap().sense(egui::Sense::hover()));
                point(r, item);
            }
        }
        Block::Table { head, rows, foot } => {
            egui::Grid::new(ui.id().with("table"))
                .striped(true)
                .spacing([10.0, 3.0])
                .show(ui, |ui| {
                    if !head.is_empty() {
                        for cell in head {
                            ui.label(RichText::new(cell).strong().small());
                        }
                        ui.end_row();
                    }
                    for (k, row) in rows.iter().chain(foot).enumerate() {
                        let strong = k >= rows.len();
                        let mut cells: Vec<egui::Response> = Vec::new();
                        if let Some(label) = &row.label {
                            cells.push(ui.label(RichText::new(label).strong()));
                        }
                        for cell in &row.cells {
                            let mut text = RichText::new(cell);
                            if strong {
                                text = text.strong();
                            }
                            if let Some(color) = theme.tone(row.tone) {
                                text = text.color(color);
                            }
                            cells.push(ui.label(text));
                        }
                        if let Some(first) = cells.into_iter().reduce(|a, b| a.union(b)) {
                            point(first, row);
                        }
                        ui.end_row();
                    }
                });
        }
        Block::Group {
            title,
            detail,
            open,
            light,
            blocks,
        } => {
            let header = egui::CollapsingHeader::new(match detail {
                Some(d) => format!("{title}  {d}"),
                None => title.clone(),
            })
            .default_open(*open)
            .show(ui, |ui| {
                for (i, b) in blocks.iter().enumerate() {
                    ui.push_id(i, |ui| {
                        self::block(ui, theme, kind, b, enabled, busy, ask, exports, lights)
                    });
                }
            });
            if header.header_response.contains_pointer() && !light.is_empty() {
                *lights = Some(light.clone());
            }
        }
        Block::Actions { actions } => {
            ui.horizontal_wrapped(|ui| {
                for action in actions {
                    let allowed = enabled && !(matches!(action.does, Does::Inspect { .. }) && busy);
                    if ui
                        .add_enabled(
                            allowed,
                            egui::Button::new(&action.label).selected(action.chosen),
                        )
                        .on_hover_text(&action.doc)
                        .clicked()
                    {
                        match &action.does {
                            Does::Inspect { options } => {
                                *ask = Some((kind.to_string(), options.clone()))
                            }
                            Does::Export { exporter, options } => {
                                *exports = Some((exporter.to_string(), options.clone()))
                            }
                        }
                    }
                }
            });
        }
    }
}
