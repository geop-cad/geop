//! The program: its panels (the joints), then its steps one under another
//! with the seeker — the line after which nothing runs. Click a step to
//! edit it, drag it to move it, drag the seeker to go back in time; copy
//! or delete a step by its buttons; unfold a step placing parts to edit
//! their steps where they are placed. A drag only drops on release; until
//! then the slot it would drop into is drawn.

use eframe::egui::{self, Color32, RichText, Sense, Stroke, Ui};

use super::{GeopApp, icons, panels};
use crate::workspace::program_name;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Dragging {
    Step(usize),
    Seeker,
}

/// What is being dragged, kept in egui's memory between frames.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Drag {
    what: Dragging,
    slot: usize,
}

pub fn program(window: &mut GeopApp, ui: &mut Ui) {
    panels::program_panels(window, ui);
    let theme = window.theme;
    let app = &window.app;
    let Some(program) = &app.program else {
        ui.add(egui::Spinner::new());
        return;
    };
    ui.label(RichText::new("Program").strong());
    ui.add_space(2.0);
    let enabled = app.ready() && app.step.is_none();
    let steps = &program.steps;
    let seeker = program.marker;
    let icon_of = |kind: &str| {
        (program.operations.iter())
            .find(|o| o.kind == kind)
            .and_then(|o| o.info.icon)
    };
    let drag_id = ui.id().with("timeline drag");
    let mut drag: Option<Drag> = ui.data_mut(|d| d.get_temp(drag_id));
    let unfold_id = ui.id().with("unfolded");
    let mut unfolded: std::collections::BTreeSet<String> =
        ui.data_mut(|d| d.get_temp(unfold_id)).unwrap_or_default();
    let mut boxes: Vec<egui::Rect> = Vec::new();
    let mut open = None;
    let mut open_placed = None;
    let mut remove = None;
    let mut copy = None;
    let mut released: Option<Drag> = None;
    let pointer = ui.input(|i| i.pointer.interact_pos());
    let seeker_slot = match drag {
        Some(Drag {
            what: Dragging::Seeker,
            slot,
        }) => slot,
        _ => seeker,
    };
    let drop_slot = match drag {
        Some(Drag {
            what: Dragging::Step(_),
            slot,
        }) => Some(slot),
        _ => None,
    };
    let seeker_row = |ui: &mut Ui, drag: &mut Option<Drag>, released: &mut Option<Drag>| {
        let (rect, response) =
            ui.allocate_exact_size(egui::vec2(ui.available_width(), 10.0), Sense::drag());
        let color = if seeker_slot < steps.len() {
            Color32::from_rgb(0xff, 0xa0, 0x40)
        } else {
            theme.accent()
        };
        let y = rect.center().y;
        ui.painter().line_segment(
            [
                egui::pos2(rect.left() + 8.0, y),
                egui::pos2(rect.right(), y),
            ],
            Stroke::new(2.0, color),
        );
        ui.painter()
            .circle_filled(egui::pos2(rect.left() + 5.0, y), 4.0, color);
        let response = response.on_hover_text(
            "Drag to go back in time: only the steps above run, and new steps go here.",
        );
        if response.hovered() || response.dragged() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::ResizeVertical);
        }
        if enabled && response.drag_started() {
            *drag = Some(Drag {
                what: Dragging::Seeker,
                slot: seeker,
            });
        }
        if response.drag_stopped() {
            *released = drag.take();
        }
    };
    egui::ScrollArea::vertical()
        .auto_shrink([false, false])
        .show(ui, |ui| {
            for (i, step) in steps.iter().enumerate() {
                if seeker_slot == i {
                    seeker_row(ui, &mut drag, &mut released);
                }
                if drop_slot == Some(i) {
                    drop_line(ui, theme.accent());
                }
                let dim = !step.editing && !step.runs;
                let dragged = matches!(drag, Some(Drag { what: Dragging::Step(k), .. }) if k == i);
                let name = match step.placed.as_slice() {
                    [one] => format!("{}: {}", program_name(&one.file), step.id),
                    _ => step.id.clone(),
                };
                let fill = if step.editing {
                    theme.accent().gamma_multiply(0.28)
                } else if dragged {
                    ui.visuals().widgets.active.weak_bg_fill
                } else {
                    Color32::TRANSPARENT
                };
                // The row's own press first, so the buttons drawn over it win.
                let (rect, _) =
                    ui.allocate_exact_size(egui::vec2(ui.available_width(), 24.0), Sense::hover());
                boxes.push(rect);
                let response =
                    ui.interact(rect, ui.id().with(("step", i)), Sense::click_and_drag());
                if fill != Color32::TRANSPARENT || response.hovered() {
                    let fill = if fill == Color32::TRANSPARENT {
                        ui.visuals().widgets.hovered.weak_bg_fill
                    } else {
                        fill
                    };
                    ui.painter().rect_filled(rect, 4, fill);
                }
                let mut row_ui = ui.new_child(
                    egui::UiBuilder::new()
                        .max_rect(rect.shrink2(egui::vec2(4.0, 2.0)))
                        .layout(egui::Layout::left_to_right(egui::Align::Center)),
                );
                {
                    let ui = &mut row_ui;
                    let mut tint = ui.visuals().text_color();
                    if step.error.is_some() {
                        tint = theme.error();
                    } else if dim {
                        tint = theme.weak().gamma_multiply(0.7);
                    }
                    match icon_of(step.kind).and_then(|i| icons::image(i, 16.0, tint)) {
                        Some(image) => {
                            ui.add(image);
                        }
                        None => {
                            ui.add_space(16.0);
                        }
                    }
                    if !step.placed.is_empty() {
                        let listed = unfolded.contains(&step.id)
                            || step
                                .placed
                                .iter()
                                .any(|p| p.steps.iter().any(|s| s.editing));
                        let (icon, tip) = if listed {
                            ("chevron-down", "Hide the placed part's steps")
                        } else {
                            (
                                "chevron-right",
                                "Show the placed part's steps, to edit them here",
                            )
                        };
                        if icons::button(ui, icon, tip).clicked() && !unfolded.remove(&step.id) {
                            unfolded.insert(step.id.clone());
                        }
                    }
                    let mut text = RichText::new(&name).color(tint);
                    if step.editing {
                        text = text.strong();
                    }
                    ui.add(egui::Label::new(text).truncate().selectable(false));
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.spacing_mut().item_spacing.x = 0.0;
                        let small = |ui: &mut Ui, icon: &str, tip: &str| {
                            let image = icons::image(icon, 13.0, theme.weak());
                            image.is_some_and(|image| {
                                ui.add_enabled(enabled, egui::Button::image(image).frame(false))
                                    .on_hover_text(tip)
                                    .clicked()
                            })
                        };
                        if small(ui, "trash", "Delete this step") {
                            remove = Some(step.id.clone());
                        }
                        if small(
                            ui,
                            "copy",
                            "Copy this step — paste it with Ctrl+V, here or in another file",
                        ) {
                            copy = Some(step.id.clone());
                        }
                    });
                }
                let title = format!("{}: {}", step.label, step.summary);
                let response = match &step.error {
                    Some(e) => response.on_hover_text(format!("{title}\n\n{e}")),
                    None => response.on_hover_text(title),
                };
                if enabled {
                    if response.clicked() {
                        open = Some(step.id.clone());
                    }
                    if response.drag_started() {
                        drag = Some(Drag {
                            what: Dragging::Step(i),
                            slot: i,
                        });
                    }
                    if response.drag_stopped() {
                        released = drag.take();
                    }
                }
                let listed = unfolded.contains(&step.id)
                    || step
                        .placed
                        .iter()
                        .any(|p| p.steps.iter().any(|s| s.editing));
                if listed {
                    for placed in &step.placed {
                        for inner in &placed.steps {
                            ui.horizontal(|ui| {
                                ui.add_space(22.0);
                                let tint = if inner.error.is_some() {
                                    theme.error()
                                } else {
                                    ui.visuals().text_color()
                                };
                                if let Some(image) =
                                    icon_of(inner.kind).and_then(|i| icons::image(i, 14.0, tint))
                                {
                                    ui.add(image);
                                }
                                let mut text = RichText::new(&inner.id).color(tint).small();
                                if inner.editing {
                                    text = text.strong().color(theme.accent());
                                }
                                let title = format!(
                                    "{}: {}: {}",
                                    program_name(&placed.file),
                                    inner.label,
                                    inner.summary
                                );
                                let r = ui.add_enabled(
                                    enabled,
                                    egui::Button::selectable(inner.editing, text)
                                        .frame_when_inactive(false),
                                );
                                if r.on_hover_text(title).clicked() {
                                    open_placed = Some((placed.instance.clone(), inner.id.clone()));
                                }
                            });
                        }
                    }
                }
            }
            if drop_slot == Some(steps.len()) {
                drop_line(ui, theme.accent());
            }
            if seeker_slot == steps.len() {
                seeker_row(ui, &mut drag, &mut released);
            }
            if steps.is_empty() {
                ui.label(
                    RichText::new("No steps yet: pick an operation above, or Ctrl+K.")
                        .color(theme.weak()),
                );
            }
        });
    // Where a drag would drop: after every box whose middle is above it.
    if let (Some(d), Some(p)) = (&mut drag, pointer) {
        d.slot = boxes.iter().filter(|r| p.y > r.center().y).count();
        ui.ctx().request_repaint();
    }
    ui.data_mut(|d| {
        match drag {
            Some(drag) => {
                d.insert_temp(drag_id, drag);
            }
            None => d.remove::<Drag>(drag_id),
        }
        d.insert_temp(unfold_id, unfolded);
    });
    let mut moved = None;
    let mut seek = None;
    if let Some(Drag { what, .. }) = released
        && let Some(p) = pointer
    {
        let slot = boxes.iter().filter(|r| p.y > r.center().y).count();
        match what {
            Dragging::Seeker if slot != seeker => seek = Some(slot),
            Dragging::Step(i) => {
                // Among the other steps, the ones before the slot keep their place.
                let to = if slot > i { slot - 1 } else { slot };
                if to != i {
                    moved = Some(geop_core_editor::editor::Command::Move {
                        id: steps[i].id.clone(),
                        index: to,
                    });
                }
            }
            _ => {}
        }
    }
    let app = &mut window.app;
    if let Some(slot) = seek {
        app.seek(slot);
    }
    if let Some(command) = moved {
        app.command(command);
    }
    if let Some(id) = open {
        app.open_step(&id, None);
    }
    if let Some((instance, id)) = open_placed {
        app.open_step(&id, Some(instance));
    }
    if let Some(id) = remove {
        app.command(geop_core_editor::editor::Command::Remove { id });
    }
    if let Some(id) = copy {
        app.copy_step(&id);
    }
}

fn drop_line(ui: &mut Ui, color: Color32) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 4.0), Sense::hover());
    ui.painter().line_segment(
        [rect.left_center(), rect.right_center()],
        Stroke::new(2.0, color),
    );
}
