//! The 3-D view: what the pointer does in it — sent to the editor as rays,
//! or moving the camera — and what is drawn there: the part on the GPU
//! (see [`crate::render`]), the labels, the prompt and the gizmo's readout
//! over it, and the view's own buttons.
//!
//! A press is a click if it does not move more than [`CLICK_PX`]; one that
//! moves turns the camera — or, where the last hover offered a grab, is a
//! drag of what is under it, sent to the editor. Two clicks within
//! [`DOUBLE`] seconds make a double click. As the web app's viewer does.

use std::sync::Arc;

use eframe::egui::{
    self, Align2, Color32, CornerRadius, Event, Pos2, Rect, RichText, Sense, Stroke, Vec2,
};
use eframe::egui_wgpu;
use geop_core_editor::editing::{Button, Pointer, Style};

use super::{GeopApp, actions::Action, icons};
use crate::{
    camera::{Drag, Projection},
    draw::{self, CAP, FlatVertex, Shown},
    math::{add, dot, scale, sub},
    render::{Frame, Uniforms, ViewCallback},
};

/// How far a press may move and still be a click, in points.
const CLICK_PX: f32 = 4.0;
/// How soon a second click makes a double click, in seconds.
const DOUBLE: f64 = 0.35;

struct Press {
    button: egui::PointerButton,
    /// Where it went down, and where the pointer last was, in the view.
    at: Pos2,
    last: Pos2,
    /// Whether the editor takes it, as a drag of what is under it.
    grabbed: bool,
    from: Option<Pointer>,
    moved: bool,
}

/// What the view keeps between frames.
#[derive(Default)]
pub struct ViewState {
    press: Option<Press>,
    last_click: Option<(Pos2, f64)>,
    /// Where the pointer was last sent as hovering, in the view.
    hovered: Option<Pos2>,
    shift: bool,
    inside: bool,
    /// The prompt typed into, by its key, and what is typed.
    prompt: Option<(String, String)>,
    last_time: Option<f64>,
    /// Where the view was last drawn, in the window.
    pub rect: Option<Rect>,
}

pub fn viewport(window: &mut GeopApp, ui: &mut egui::Ui) {
    let rect = ui.available_rect_before_wrap();
    let response = ui.allocate_rect(rect, Sense::click_and_drag());
    let ctx = ui.ctx().clone();
    let layer = ui.layer_id();
    window.view.rect = Some(rect);
    let app = &mut window.app;
    app.camera.size = [rect.width() as f64, rect.height() as f64];
    let local = |p: Pos2| p - rect.min.to_vec2();
    let pointer_at =
        |app: &crate::controller::Controller, p: Pos2| app.camera.pointer(p.x as f64, p.y as f64);

    // The camera moves on, gliding or dying away, frame by frame.
    let now = ctx.input(|i| i.time);
    let dt = window
        .view
        .last_time
        .map_or(1.0 / 60.0, |t| (now - t).clamp(0.0, 0.1));
    window.view.last_time = Some(now);
    let moving = app.camera.advance(dt);
    if app.camera.settled(window.view.press.is_some()) {
        app.view_settled();
    }

    // What the pointer did.
    let (events, modifiers, hover_pos) =
        ctx.input(|i| (i.events.clone(), i.modifiers, i.pointer.hover_pos()));
    let over_view = |p: Pos2| rect.contains(p) && ctx.layer_id_at(p) == Some(layer);
    let inside = hover_pos.is_some_and(over_view);
    let view = &mut window.view;
    let shift_changed = modifiers.shift != view.shift;
    view.shift = modifiers.shift;
    if inside && !view.inside {
        // A step opened since the view last settled has not been told it.
        app.view_settled();
    }
    for event in &events {
        match event {
            Event::PointerButton {
                pos,
                button,
                pressed: true,
                ..
            } if over_view(*pos) && view.press.is_none() && !app.camera.gliding() => {
                app.camera.hold();
                let grabbed = *button == egui::PointerButton::Primary && app.grab();
                let at = local(*pos);
                view.press = Some(Press {
                    button: *button,
                    at,
                    last: at,
                    grabbed,
                    from: pointer_at(app, at),
                    moved: false,
                });
            }
            Event::PointerMoved(pos) => {
                let p = local(*pos);
                if let Some(press) = &mut view.press {
                    if (p - press.at).length() > CLICK_PX {
                        press.moved = true;
                    }
                    if !press.moved {
                        continue;
                    }
                    if press.grabbed {
                        if let (Some(from), Some(to)) = (press.from, pointer_at(app, p)) {
                            app.drag(from, to, false, modifiers.shift);
                        }
                    } else {
                        let d = p - press.last;
                        let in_plane = app.plane().is_some();
                        match press.button {
                            egui::PointerButton::Primary if !in_plane && !modifiers.shift => {
                                app.camera.tumble(d.x as f64, d.y as f64)
                            }
                            _ => app.camera.pan(d.x as f64, d.y as f64),
                        }
                    }
                    press.last = p;
                }
            }
            Event::PointerButton {
                pos,
                button,
                pressed: false,
                ..
            } => {
                let Some(press) = view.press.take_if(|p| p.button == *button) else {
                    continue;
                };
                let p = local(*pos);
                if press.grabbed && press.moved {
                    if let (Some(from), Some(to)) = (press.from, pointer_at(app, p)) {
                        app.drag(from, to, true, modifiers.shift);
                    }
                    view.hovered = None;
                    continue;
                }
                if press.moved {
                    let in_plane = app.plane().is_some();
                    app.camera.release(match press.button {
                        egui::PointerButton::Primary if !in_plane && !modifiers.shift => {
                            Drag::Tumble
                        }
                        _ => Drag::Pan,
                    });
                    continue;
                }
                let button = match press.button {
                    egui::PointerButton::Primary => Button::Primary,
                    egui::PointerButton::Secondary => Button::Secondary,
                    _ => continue,
                };
                let double = (view.last_click)
                    .is_some_and(|(at, time)| now - time < DOUBLE && (p - at).length() <= CLICK_PX);
                view.last_click = if double { None } else { Some((p, now)) };
                if let Some(pointer) = pointer_at(app, p) {
                    app.click(pointer, button, double, modifiers.shift);
                }
            }
            Event::MouseWheel { delta, unit, .. } if inside => {
                let steps = match unit {
                    egui::MouseWheelUnit::Point => delta.y / 60.0,
                    egui::MouseWheelUnit::Line => delta.y,
                    egui::MouseWheelUnit::Page => delta.y * 4.0,
                };
                if let Some(p) = hover_pos {
                    let p = local(p);
                    app.camera.zoom(steps as f64, p.x as f64, p.y as f64);
                }
            }
            Event::Zoom(factor) if inside => {
                if let Some(p) = hover_pos {
                    let p = local(p);
                    app.camera.zoom(
                        (*factor as f64).ln() / 0.85f64.ln().abs(),
                        p.x as f64,
                        p.y as f64,
                    );
                }
            }
            _ => {}
        }
    }
    // Hovering: the latest position, when it moved or Shift changed.
    if view.press.is_none() {
        match hover_pos.filter(|p| over_view(*p)).map(local) {
            Some(p) if view.hovered != Some(p) || shift_changed => {
                view.hovered = Some(p);
                if let Some(pointer) = pointer_at(app, p) {
                    app.hover(pointer, modifiers.shift);
                }
            }
            None if view.inside => {
                view.hovered = None;
                app.event(geop_core_editor::editing::StepEditEvent::Leave);
            }
            _ => {}
        }
    }
    view.inside = inside;
    let cursor = match &view.press {
        Some(p) if p.grabbed => Some(egui::CursorIcon::Grabbing),
        None if inside && app.grab() => Some(egui::CursorIcon::Grab),
        Some(p) if p.moved => Some(egui::CursorIcon::AllScroll),
        _ => None,
    };
    if let Some(cursor) = cursor {
        ctx.set_cursor_icon(cursor);
    }
    let _ = response;

    // What is drawn.
    let highlights = app.highlights();
    let section = app.section.and_then(|s| s.plane(app.scene.part.center));
    let shown = Shown {
        kept: &app.scene,
        presentation: app.presentation(),
        highlights: &highlights,
        plane: app.plane(),
        section,
        dark: window.theme.dark(),
    };
    let drawing = draw::drawing(&shown, &app.camera, &window.statics);
    window.statics = drawing.statics.clone();
    let ppp = ctx.pixels_per_point();
    let size = [
        (rect.width() * ppp).round().max(1.0) as u32,
        (rect.height() * ppp).round().max(1.0) as u32,
    ];
    let camera = &app.camera;
    let f = |v: [f64; 3], w: f64| [v[0] as f32, v[1] as f32, v[2] as f32, w as f32];
    let ortho = if camera.projection == Projection::Orthographic {
        1.0
    } else {
        0.0
    };
    let (cut, cut_on, cap) = match &drawing.cut {
        Some(cut) => {
            let n = cut.normal;
            let center = app.scene.part.center;
            let on_plane = sub(center, scale(n, dot(sub(center, cut.origin), n)));
            let (a, b) = {
                let other = if n[0].abs() < 0.9 {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 1.0, 0.0]
                };
                let a = crate::math::normalized(crate::math::cross(n, other));
                (a, crate::math::cross(n, a))
            };
            let h = cut.size / 2.0;
            let corner = |x: f64, y: f64| add(on_plane, add(scale(a, x * h), scale(b, y * h)));
            let color = CAP.f32();
            let q = [
                corner(-1.0, -1.0),
                corner(1.0, -1.0),
                corner(1.0, 1.0),
                corner(-1.0, 1.0),
            ];
            let cap: Vec<FlatVertex> = [q[0], q[1], q[2], q[0], q[2], q[3]]
                .into_iter()
                .map(|p| FlatVertex {
                    position: p.map(|x| x as f32),
                    color,
                })
                .collect();
            (f(n, dot(n, cut.origin)), [0.0, 0.0, 0.0, 1.0], cap)
        }
        None => ([0.0; 4], [0.0; 4], Vec::new()),
    };
    let uniforms = Uniforms {
        view_proj: crate::math::to_f32(&camera.view_projection(app.scene.part.size)),
        eye: f(camera.pose.position, ortho),
        back: f(camera.pose.back(), 0.0),
        viewport: [size[0] as f32, size[1] as f32, ppp, 0.0],
        cut,
        cut_on,
    };
    let labels = drawing.labels.clone();
    let readout = drawing.readout.clone();
    let frame = Frame {
        size,
        uniforms,
        background: window.theme.backdrop(),
        drawing: Arc::new(drawing),
        cap,
    };
    ui.painter().add(egui_wgpu::Callback::new_paint_callback(
        rect,
        ViewCallback(Arc::new(frame)),
    ));

    // Over the view: labels, the gizmo's readout, the prompt.
    let painter = ui.painter_at(rect);
    let screen = |p: [f64; 3]| {
        camera
            .project(p)
            .map(|[x, y]| rect.min + Vec2::new(x as f32, y as f32))
    };
    for label in &labels {
        let Some(at) = screen(label.at) else { continue };
        let (fill, stroke) = match label.style {
            Style::Selected => (
                Color32::from_rgb(0x6a, 0x5a, 0x20),
                Color32::from_rgb(0xff, 0xd8, 0x4a),
            ),
            Style::Hover => (
                window.theme.label_fill(),
                Color32::from_rgb(0xff, 0xe0, 0xa0),
            ),
            Style::Failed => (
                Color32::from_rgb(0x5a, 0x20, 0x20),
                Color32::from_rgb(0xff, 0x5d, 0x5d),
            ),
            Style::Snap => (
                window.theme.label_fill(),
                Color32::from_rgb(0x58, 0xe0, 0x7a),
            ),
            Style::Paper => (Color32::TRANSPARENT, Color32::TRANSPARENT),
            _ => (window.theme.label_fill(), window.theme.line()),
        };
        let size = if label.style == Style::Paper {
            9.5
        } else {
            11.0
        };
        let galley = painter.layout_no_wrap(
            label.text.clone(),
            egui::FontId::proportional(size),
            window.theme.text(),
        );
        let r = Rect::from_center_size(at, galley.size() + Vec2::new(8.0, 2.0));
        painter.rect(
            r,
            CornerRadius::same(3),
            fill,
            Stroke::new(1.0, stroke),
            egui::StrokeKind::Inside,
        );
        painter.galley(r.min + Vec2::new(4.0, 1.0), galley, window.theme.text());
    }
    if let Some((at, text)) = readout
        && let Some(at) = screen(at)
    {
        let at = at + Vec2::new(0.0, 3.0 * crate::camera::REACH_PX as f32 + 12.0);
        let galley = painter.layout_no_wrap(
            text,
            egui::FontId::proportional(12.0),
            Color32::from_rgb(0x17, 0x19, 0x1c),
        );
        let r = Rect::from_center_size(at, galley.size() + Vec2::new(12.0, 4.0));
        painter.rect_filled(
            r,
            CornerRadius::same(3),
            Color32::from_rgb(0xff, 0xd8, 0x4a),
        );
        painter.galley(r.min + Vec2::new(6.0, 2.0), galley, Color32::BLACK);
    }
    prompt(window, &ctx, rect);
    overlay(window, &ctx, rect);
    if moving || window.view.press.is_some() || window.app.camera.gliding() {
        ctx.request_repaint();
    }
}

/// A value asked for in place: typed where its point is, Enter giving it
/// and Escape giving up.
fn prompt(window: &mut GeopApp, ctx: &egui::Context, rect: Rect) {
    let app = &mut window.app;
    let Some(prompt) = app.presentation().and_then(|p| p.prompt.clone()) else {
        window.view.prompt = None;
        return;
    };
    let at = crate::scene::v3(&prompt.at);
    let Some([x, y]) = app.camera.project(at) else {
        return;
    };
    let id = format!("{}:{}", prompt.key, prompt.label);
    let fresh = window.view.prompt.as_ref().is_none_or(|(k, _)| *k != id);
    if fresh {
        window.view.prompt = Some((id.clone(), prompt.value.clone()));
    }
    let Some((_, text)) = &mut window.view.prompt else {
        return;
    };
    let mut given = None;
    let mut gave_up = false;
    egui::Area::new(egui::Id::new("geop prompt"))
        .fixed_pos(rect.min + Vec2::new(x as f32, y as f32))
        .pivot(Align2::CENTER_CENTER)
        .order(egui::Order::Foreground)
        .show(ctx, |ui| {
            window.theme.card().inner_margin(4).show(ui, |ui| {
                let edit = egui::TextEdit::singleline(text)
                    .desired_width(110.0)
                    .font(egui::TextStyle::Monospace);
                let response = ui.add(edit).on_hover_text(format!(
                    "{} — a number, or a formula of the parameters · Enter applies, Esc cancels",
                    prompt.label
                ));
                if fresh {
                    response.request_focus();
                }
                // A line of text gives its focus up on Enter and on Escape.
                if response.lost_focus() || response.has_focus() {
                    if ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                        given = Some(text.clone());
                    }
                    if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                        gave_up = true;
                    }
                }
            });
        });
    if let Some(text) = given {
        app.dialog(&prompt.key, geop_core_editor::editing::Value::Text(text));
    }
    if gave_up {
        app.key("Escape");
    }
}

/// The view's own buttons, what the step works in, why something was
/// refused, and — with nothing yet — the examples to start from.
fn overlay(window: &mut GeopApp, ctx: &egui::Context, rect: Rect) {
    let theme = window.theme;
    let mut acted = None;
    egui::Area::new(egui::Id::new("geop view buttons"))
        .fixed_pos(rect.right_top() + Vec2::new(-10.0, 10.0))
        .pivot(Align2::RIGHT_TOP)
        .order(egui::Order::Middle)
        .show(ctx, |ui| {
            theme.card().inner_margin(4).show(ui, |ui| {
                ui.horizontal(|ui| {
                    let mut button = |ui: &mut egui::Ui, icon: &str, tip: &str, action: Action| {
                        let shortcut = action
                            .shortcut()
                            .map(|s| ctx.format_shortcut(&s))
                            .unwrap_or_default();
                        let image = icons::image(icon, 16.0, ui.visuals().text_color());
                        let b = match image {
                            Some(image) => egui::Button::image(image),
                            None => egui::Button::new(icon),
                        };
                        let tip = if shortcut.is_empty() {
                            tip.to_string()
                        } else {
                            format!("{tip} ({shortcut})")
                        };
                        if ui.add(b.frame(false)).on_hover_text(tip).clicked() {
                            acted = Some(action);
                        }
                    };
                    button(ui, "fit", "Frame the whole part in the view", Action::Fit);
                    button(
                        ui,
                        "home",
                        "Back to the standard view of the whole part",
                        Action::ResetView,
                    );
                    let (icon, tip) = match window.app.camera.projection {
                        Projection::Perspective => {
                            ("perspective", "Perspective — switch to orthographic")
                        }
                        Projection::Orthographic => {
                            ("orthographic", "Orthographic — switch to perspective")
                        }
                    };
                    button(ui, icon, tip, Action::ToggleProjection);
                });
            });
        });
    let app = &window.app;
    let badge = app
        .step
        .as_ref()
        .filter(|_| app.plane().is_some())
        .map(|s| format!("{}: working in its plane", s.label));
    let crashed = app.crashed.clone();
    let mut dismissed = false;
    let refused = app.error.clone().filter(|_| app.step.is_none());
    let welcome = app.ready()
        && !app.busy()
        && app.step.is_none()
        && app.program.as_ref().is_some_and(|p| p.steps.is_empty());
    egui::Area::new(egui::Id::new("geop view notes"))
        .fixed_pos(rect.left_top() + Vec2::new(10.0, 10.0))
        .order(egui::Order::Middle)
        .show(ctx, |ui| {
            ui.set_max_width((rect.width() - 160.0).max(200.0));
            if let Some(badge) = badge {
                theme.card().inner_margin(6).show(ui, |ui| {
                    ui.label(RichText::new(badge).color(theme.accent()));
                });
            }
            if let Some(crashed) = crashed {
                theme.card().show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.label(RichText::new(crashed).color(theme.error()));
                        if ui.small_button("Dismiss").clicked() {
                            dismissed = true;
                        }
                    });
                });
            }
            if let Some(refused) = refused {
                theme.card().show(ui, |ui| {
                    ui.label(RichText::new(refused).color(theme.error()));
                });
            }
        });
    if dismissed {
        window.app.crashed = None;
    }
    if welcome {
        welcome_card(window, ctx, rect, &mut acted);
    }
    if let Some(action) = acted {
        window.act(ctx, action);
    }
}

/// What to start from, while the program has no steps: the examples.
fn welcome_card(window: &GeopApp, ctx: &egui::Context, rect: Rect, acted: &mut Option<Action>) {
    let Some(program) = &window.app.program else {
        return;
    };
    let theme = window.theme;
    egui::Area::new(egui::Id::new("geop welcome"))
        .fixed_pos(rect.center())
        .pivot(Align2::CENTER_CENTER)
        .order(egui::Order::Middle)
        .show(ctx, |ui| {
            theme.card().inner_margin(16).show(ui, |ui| {
                ui.set_max_width(460.0);
                ui.heading("Start a part");
                ui.label(
                    RichText::new(
                        "Pick an operation in the ribbon — a sketch first — or open an example. \
                     Ctrl+K lists everything the app does.",
                    )
                    .color(theme.weak()),
                );
                ui.add_space(6.0);
                egui::ScrollArea::vertical()
                    .max_height(260.0)
                    .show(ui, |ui| {
                        ui.horizontal_wrapped(|ui| {
                            for name in &program.examples {
                                if ui
                                    .button(*name)
                                    .on_hover_text("Open the example in a file of its own")
                                    .clicked()
                                {
                                    *acted = Some(Action::LoadExample(name.to_string()));
                                }
                            }
                            for name in &program.workspace_examples {
                                if ui
                                    .button(format!("{name} …"))
                                    .on_hover_text(
                                        "Add the example's files in a folder of their own",
                                    )
                                    .clicked()
                                {
                                    *acted = Some(Action::LoadWorkspaceExample(name.to_string()));
                                }
                            }
                        });
                    });
            });
        });
}
