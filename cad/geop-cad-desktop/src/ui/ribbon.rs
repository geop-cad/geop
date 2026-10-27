//! The ribbon: the app's menu, undo and redo, then every operation the
//! kernel lists, group by group in its order — the most used big, those
//! used often small, the rest in the group's menu — and on the right the
//! command palette, the theme and the shortcuts.

use eframe::egui::{self, RichText, Ui};
use geop_core_editor::editable::{Listed, OperationTier};

use super::{GeopApp, actions::Action, icons};

/// The name a group is shown by, as the kernel names it.
fn group_name(listed: &Listed) -> String {
    serde_json::to_value(listed.info.group)
        .ok()
        .and_then(|v| v.as_str().map(str::to_string))
        .unwrap_or_default()
}

/// A button of the app's own, its hotkey in its tooltip.
fn app_button(
    ui: &mut Ui,
    ctx: &egui::Context,
    icon: &str,
    tip: &str,
    action: &Action,
    enabled: bool,
) -> bool {
    let tint = if enabled {
        ui.visuals().text_color()
    } else {
        ui.visuals().weak_text_color()
    };
    let Some(image) = icons::image(icon, 18.0, tint) else {
        return false;
    };
    let tip = match action.shortcut() {
        Some(s) => format!("{tip} ({})", ctx.format_shortcut(&s)),
        None => tip.to_string(),
    };
    ui.add_enabled(
        enabled,
        egui::Button::image(image)
            .frame(false)
            .min_size(egui::vec2(28.0, 28.0)),
    )
    .on_hover_text(tip.clone())
    .on_disabled_hover_text(tip)
    .clicked()
}

pub fn ribbon(window: &mut GeopApp, ui: &mut Ui) {
    let ctx = ui.ctx().clone();
    // A label never wraps: what does not fit scrolls.
    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
    let mut acted: Option<Action> = None;
    let app = &window.app;
    ui.horizontal(|ui| {
        ui.label(
            RichText::new("geop")
                .strong()
                .size(16.0)
                .color(window.theme.accent()),
        );
        ui.add_space(4.0);
        ui.menu_button("File", |ui| {
            let mut item = |ui: &mut Ui, label: &str, action: Action| {
                let mut b = egui::Button::new(label);
                if let Some(s) = action.shortcut() {
                    b = b.shortcut_text(ctx.format_shortcut(&s));
                }
                if ui.add_enabled(action.enabled(app), b).clicked() {
                    acted = Some(action);
                    ui.close();
                }
            };
            item(ui, "New part", Action::NewPart);
            item(ui, "Open folder…", Action::OpenFolder);
            item(ui, "Read the folder again", Action::Reread);
            ui.separator();
            if let Some(program) = &app.program {
                ui.menu_button("Examples", |ui| {
                    egui::ScrollArea::vertical()
                        .max_height(420.0)
                        .show(ui, |ui| {
                            for name in &program.examples {
                                if ui.button(*name).clicked() {
                                    acted = Some(Action::LoadExample(name.to_string()));
                                    ui.close();
                                }
                            }
                        });
                });
                if !program.workspace_examples.is_empty() {
                    ui.menu_button("Examples of several files", |ui| {
                        for name in &program.workspace_examples {
                            if ui.button(*name).clicked() {
                                acted = Some(Action::LoadWorkspaceExample(name.to_string()));
                                ui.close();
                            }
                        }
                    });
                }
                ui.menu_button("Export", |ui| {
                    for exporter in &program.exporters {
                        let b = egui::Button::new(format!(
                            "{} ({})",
                            exporter.label,
                            exporter.formats.join(", ")
                        ));
                        if ui
                            .add_enabled(app.step.is_none(), b)
                            .on_hover_text(exporter.doc)
                            .clicked()
                        {
                            acted = Some(Action::Export(exporter.kind.to_string()));
                            ui.close();
                        }
                    }
                });
            }
            ui.separator();
            let mut reduced = !app.reduced;
            if ui
                .checkbox(&mut reduced, "Experimental operations")
                .on_hover_text(
                    "Offer the operations that are not yet robust enough to offer by default",
                )
                .changed()
            {
                acted = Some(Action::ToggleReduced);
            }
        });
        ui.separator();
        if app_button(ui, &ctx, "undo", "Undo", &Action::Undo, app.can_undo()) {
            acted = Some(Action::Undo);
        }
        if app_button(ui, &ctx, "redo", "Redo", &Action::Redo, app.can_redo()) {
            acted = Some(Action::Redo);
        }
        ui.separator();
        let right = 120.0;
        let width = (ui.available_width() - right).max(100.0);
        ui.allocate_ui_with_layout(
            egui::vec2(width, 78.0),
            egui::Layout::left_to_right(egui::Align::Center),
            |ui| {
                egui::ScrollArea::horizontal()
                    .id_salt("ribbon scroll")
                    .show(ui, |ui| {
                        ui.horizontal(|ui| operations(window, ui, &ctx, &mut acted));
                    });
            },
        );
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let theme_icon = if window.theme.dark() { "sun" } else { "moon" };
            if app_button(ui, &ctx, "command", "Commands", &Action::Palette, true) {
                acted = Some(Action::Palette);
            }
            if app_button(
                ui,
                &ctx,
                theme_icon,
                "Switch the theme",
                &Action::ToggleTheme,
                true,
            ) {
                acted = Some(Action::ToggleTheme);
            }
            if ui
                .button("?")
                .on_hover_text("Keyboard shortcuts (F1)")
                .clicked()
            {
                acted = Some(Action::Shortcuts);
            }
        });
    });
    if let Some(action) = acted {
        window.act(&ctx, action);
    }
}

/// The operations, group by group.
fn operations(window: &GeopApp, ui: &mut Ui, ctx: &egui::Context, acted: &mut Option<Action>) {
    let app = &window.app;
    let Some(program) = &app.program else {
        ui.add(egui::Spinner::new());
        return;
    };
    let listed: Vec<&Listed> = (program.operations.iter())
        .filter(|l| !app.reduced || !l.info.experimental)
        .collect();
    let mut groups: Vec<(String, Vec<&Listed>)> = Vec::new();
    for l in listed {
        let name = group_name(l);
        match groups.last_mut() {
            Some((g, list)) if *g == name => list.push(l),
            _ => groups.push((name, vec![l])),
        }
    }
    let theme = window.theme;
    for (name, tools) in groups {
        ui.vertical(|ui| {
            ui.horizontal(|ui| {
                ui.spacing_mut().item_spacing.x = 2.0;
                let big: Vec<&&Listed> = tools
                    .iter()
                    .filter(|t| t.info.tier == OperationTier::Big)
                    .collect();
                let small: Vec<&&Listed> = tools
                    .iter()
                    .filter(|t| t.info.tier == OperationTier::Small)
                    .collect();
                for tool in big {
                    operation(app, ui, tool, true, theme, acted);
                }
                if !small.is_empty() {
                    for column in small.chunks(2) {
                        ui.vertical(|ui| {
                            ui.spacing_mut().item_spacing.y = 1.0;
                            for tool in column {
                                operation(app, ui, tool, false, theme, acted);
                            }
                        });
                    }
                }
            });
            let open_new = |l: &Listed| {
                app.step
                    .as_ref()
                    .is_some_and(|s| s.kind == l.kind && s.id.is_none())
            };
            let any_active = tools
                .iter()
                .any(|l| open_new(l) && l.info.tier == OperationTier::Menu);
            let mut caption = RichText::new(name.clone()).small().color(theme.weak());
            if any_active {
                caption = caption.color(theme.accent());
            }
            ui.menu_button(caption, |ui| {
                let mut before = None;
                for l in &tools {
                    if before.is_some_and(|t| t != OperationTier::Menu)
                        && l.info.tier == OperationTier::Menu
                    {
                        ui.separator();
                    }
                    before = Some(l.info.tier);
                    let tint = ui.visuals().text_color();
                    let mut b = match l.info.icon.and_then(|i| icons::image(i, 16.0, tint)) {
                        Some(image) => egui::Button::image_and_text(image, l.info.label),
                        None => egui::Button::new(l.info.label),
                    };
                    b = b.selected(open_new(l));
                    let action = Action::NewStep(l.kind.to_string(), l.info.shortcut);
                    let enabled = action.enabled(app);
                    if let Some(s) = action.shortcut() {
                        b = b.shortcut_text(ctx.format_shortcut(&s));
                    }
                    if ui
                        .add_enabled(enabled, b)
                        .on_hover_text(l.info.doc)
                        .clicked()
                    {
                        *acted = Some(action);
                        ui.close();
                    }
                }
            });
            let _ = ctx;
        });
        ui.separator();
    }
}

/// An operation as a button: big — its icon over its label — or small.
fn operation(
    app: &crate::controller::Controller,
    ui: &mut Ui,
    l: &Listed,
    big: bool,
    theme: super::Theme,
    acted: &mut Option<Action>,
) {
    let action = Action::NewStep(l.kind.to_string(), l.info.shortcut);
    let enabled = action.enabled(app);
    let active = app
        .step
        .as_ref()
        .is_some_and(|s| s.kind == l.kind && s.id.is_none());
    let tint = if active {
        theme.accent()
    } else if enabled {
        ui.visuals().text_color()
    } else {
        ui.visuals().weak_text_color()
    };
    let size = if big { 22.0 } else { 14.0 };
    let image = l.info.icon.and_then(|i| icons::image(i, size, tint));
    let response = if big {
        let (rect, response) = ui.allocate_exact_size(egui::vec2(56.0, 52.0), egui::Sense::click());
        response.widget_info(|| {
            egui::WidgetInfo::labeled(egui::WidgetType::Button, enabled, l.info.label)
        });
        let visuals = ui.style().interact_selectable(&response, active);
        if response.hovered() || active {
            ui.painter().rect_filled(rect, 6, visuals.weak_bg_fill);
        }
        if let Some(image) = image {
            image.paint_at(
                ui,
                egui::Rect::from_center_size(
                    rect.center_top() + egui::vec2(0.0, 17.0),
                    egui::Vec2::splat(size),
                ),
            );
        }
        let galley = ui.painter().layout(
            l.info.label.to_string(),
            egui::FontId::proportional(11.0),
            tint,
            rect.width() - 2.0,
        );
        let at = egui::pos2(rect.center().x - galley.size().x / 2.0, rect.top() + 32.0);
        ui.painter().galley(at, galley, tint);
        response
    } else {
        let b = match image {
            Some(image) => {
                egui::Button::image_and_text(image, RichText::new(l.info.label).small().color(tint))
            }
            None => egui::Button::new(RichText::new(l.info.label).small()),
        };
        ui.add(b.frame(false).selected(active))
    };
    let tip = match action.shortcut() {
        Some(s) => format!(
            "{} ({})\n\n{}",
            l.info.label,
            ui.ctx().format_shortcut(&s),
            l.info.doc
        ),
        None => format!("{}\n\n{}", l.info.label, l.info.doc),
    };
    let response = if enabled {
        response.on_hover_text(tip)
    } else {
        response.on_hover_text(format!("{tip}\n\nFinish the step edited first"))
    };
    if enabled && response.clicked() {
        *acted = Some(action);
    }
}
