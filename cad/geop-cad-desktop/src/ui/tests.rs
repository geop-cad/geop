//! The window driven as a user drives it, headless (egui_kittest): its
//! buttons found by what they say and clicked, keys pressed — and the view
//! drawn by wgpu into an image, where a GPU (or a software one) is there.
//! `GEOP_DESKTOP_SHOTS=<folder>` keeps the images drawn.

use egui_kittest::{Harness, kittest::Queryable};

use super::{GeopApp, actions::Action};
use crate::tests::scratch;

/// The app on a fresh folder, in a window of 1280 × 800, its kernel ready.
fn window(name: &str, wgpu: bool) -> Harness<'static, GeopApp> {
    let dir = scratch(name);
    let mut builder = Harness::builder().with_size(egui::vec2(1280.0, 800.0));
    if wgpu {
        builder = builder.wgpu();
    }
    let mut harness = builder.build_eframe(move |cc| GeopApp::create(cc, &dir, None).unwrap());
    idle(&mut harness);
    harness
}

use eframe::egui;

/// Frames drawn until the kernel has answered everything and the camera
/// has come to rest.
fn idle(harness: &mut Harness<'_, GeopApp>) {
    for _ in 0..2000 {
        harness.step();
        let app = &harness.state().app;
        if app.ready() && !app.busy() && !app.camera.gliding() {
            harness.run_steps(2);
            return;
        }
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
    panic!("the app never came to rest");
}

#[test]
fn a_sketch_is_started_from_the_ribbon_and_cancelled() {
    let mut harness = window("ui-sketch", false);
    // The operation, before its group's menu of the same name.
    harness.get_all_by_label("Sketch").next().unwrap().click();
    idle(&mut harness);
    let step = harness
        .state()
        .app
        .step
        .clone()
        .expect("a sketch is edited");
    assert_eq!(step.kind, "add_sketch");
    // The dialog beside the view, as the kernel lays it out.
    harness.get_by_label("Cancel").click();
    idle(&mut harness);
    assert!(harness.state().app.step.is_none());
}

#[test]
fn the_palette_finds_an_operation_by_a_few_letters() {
    let mut harness = window("ui-palette", false);
    harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::K);
    harness.run_steps(2);
    assert!(harness.state().palette.open);
    harness
        .get_by_role(egui::accesskit::Role::TextInput)
        .type_text("sketch");
    harness.run_steps(2);
    harness.key_press(egui::Key::Enter);
    idle(&mut harness);
    assert!(!harness.state().palette.open);
    assert_eq!(
        harness.state().app.step.as_ref().map(|s| s.kind),
        Some("add_sketch")
    );
    // Escape goes to the step, never closes it.
    harness.key_press(egui::Key::Escape);
    idle(&mut harness);
    assert!(harness.state().app.step.is_some());
    harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::Backspace);
    idle(&mut harness);
    assert!(harness.state().app.step.is_none());
}

#[test]
fn undo_and_redo_are_buttons_and_hotkeys() {
    let mut harness = window("ui-undo", false);
    let name = harness.state().app.program.as_ref().unwrap().examples[0].to_string();
    harness
        .state_mut()
        .act(&egui::Context::default(), Action::LoadExample(name));
    idle(&mut harness);
    let steps = |h: &Harness<'_, GeopApp>| h.state().app.program.as_ref().unwrap().steps.len();
    let before = steps(&harness);
    assert!(before > 0);
    harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::Z);
    idle(&mut harness);
    assert!(steps(&harness) < before);
    harness.key_press_modifiers(
        egui::Modifiers::COMMAND | egui::Modifiers::SHIFT,
        egui::Key::Z,
    );
    idle(&mut harness);
    assert_eq!(steps(&harness), before);
}

/// The view draws the part: an example's solid covers the middle of the
/// view, where the backdrop showed before.
#[test]
fn the_view_draws_an_example() {
    let mut harness = window("ui-render", true);
    let shot = |harness: &mut Harness<'_, GeopApp>, name: &str| {
        let image = harness.render().expect("rendered");
        if let Ok(dir) = std::env::var("GEOP_DESKTOP_SHOTS") {
            let _ = image.save(std::path::Path::new(&dir).join(format!("{name}.png")));
        }
        image
    };
    let empty = shot(&mut harness, "empty");
    let name = harness.state().app.program.as_ref().unwrap().examples[0].to_string();
    harness
        .state_mut()
        .act(&egui::Context::default(), Action::LoadExample(name.clone()));
    idle(&mut harness);
    let drawn = shot(&mut harness, &name);
    // The middle of the viewport, between the panels.
    let (x, y) = (empty.width() / 2 + 40, empty.height() / 2);
    let differs = (0..40)
        .flat_map(|dx| (0..40).map(move |dy| (x - 20 + dx, y - 20 + dy)))
        .filter(|&(x, y)| empty.get_pixel(x, y) != drawn.get_pixel(x, y))
        .count();
    assert!(
        differs > 200,
        "the part is drawn where the view is: {differs} pixels changed"
    );
    // A sketch of it opened: the view turns to face its plane, and its
    // dialog shows beside the view.
    let first = harness.state().app.program.as_ref().unwrap().steps[0]
        .id
        .clone();
    harness.state_mut().app.open_step(&first, None);
    idle(&mut harness);
    assert!(harness.state().app.plane().is_some());
    shot(&mut harness, "sketch");
}

/// A primary click in the view where `p` is seen: the pointer moved
/// there, pressed and let go, as a mouse does.
fn click_in_view(harness: &mut Harness<'_, GeopApp>, p: [f64; 3]) {
    let rect = harness.state().view_rect().expect("the view is drawn");
    let [x, y] = harness.state().app.camera.project(p).expect("seen");
    let pos = rect.min + egui::vec2(x as f32, y as f32);
    harness.event(egui::Event::PointerMoved(pos));
    idle(harness);
    for pressed in [true, false] {
        harness.event(egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        });
        harness.step();
    }
    idle(harness);
    assert_eq!(harness.state().app.error, None);
}

/// A sketch drawn in the window as a user draws it: the plane picked by a
/// click where it is seen, the line tool taken by its key, two clicks,
/// Escape, and OK by its hotkey.
#[test]
fn a_line_is_sketched_with_the_mouse_and_keys() {
    let mut harness = window("ui-line", false);
    harness.get_all_by_label("Sketch").next().unwrap().click();
    idle(&mut harness);
    let reach = harness.state().app.camera.reach_at([0.0; 3]);
    click_in_view(&mut harness, [3.5 * reach, 3.5 * reach, 0.0]);
    assert!(
        harness.state().app.plane().is_some(),
        "the xy plane is picked"
    );
    harness.key_press(egui::Key::L);
    idle(&mut harness);
    click_in_view(&mut harness, [10.0, 10.0, 0.0]);
    click_in_view(&mut harness, [30.0, 15.0, 0.0]);
    harness.key_press(egui::Key::Escape);
    harness.key_press(egui::Key::Escape);
    idle(&mut harness);
    harness.key_press_modifiers(egui::Modifiers::COMMAND, egui::Key::Enter);
    idle(&mut harness);
    let app = &harness.state().app;
    assert!(app.step.is_none(), "{:?}", app.error);
    assert_eq!(app.program.as_ref().unwrap().steps.len(), 1);
}
