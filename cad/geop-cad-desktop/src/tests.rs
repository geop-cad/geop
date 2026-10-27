//! The app driven as its window drives it, with no window: a folder of
//! files opened, examples loaded, a sketch drawn by pointer events made of
//! pixels through the camera, committed, and the file on disk checked.

use std::{path::PathBuf, sync::Arc};

use geop_core_editor::editing::Button;

use crate::{
    camera::Projection,
    controller::{Controller, Effect},
    workspace::parse_program,
};

/// A fresh folder for the test `name`.
pub fn scratch(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("geop-desktop-{name}-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

pub fn opened(name: &str) -> (Controller, PathBuf) {
    let dir = scratch(name);
    let mut app = Controller::open_folder(&dir, None, Arc::new(|| {})).unwrap();
    app.settle();
    app.camera.size = [1000.0, 800.0];
    (app, dir)
}

/// Let the camera come to rest, as frames passing do.
fn glide(app: &mut Controller) {
    for _ in 0..200 {
        if !app.camera.advance(1.0 / 60.0) {
            break;
        }
    }
    if app.camera.settled(false) {
        app.view_settled();
    }
    app.settle();
}

/// A primary click at the pixel where `p` is seen.
fn click_at(app: &mut Controller, p: [f64; 3]) {
    let [x, y] = app.camera.project(p).expect("in front of the camera");
    let pointer = app.camera.pointer(x, y).unwrap();
    app.hover(pointer, false);
    app.click(pointer, Button::Primary, false, false);
    app.settle();
    assert_eq!(app.error, None);
}

fn on_disk(dir: &std::path::Path, path: &str) -> serde_json::Value {
    let text = std::fs::read_to_string(dir.join(path)).unwrap();
    serde_json::to_value(parse_program(&text).unwrap()).unwrap()
}

#[test]
fn an_empty_folder_gets_a_part_to_edit() {
    let (app, dir) = opened("empty");
    assert!(app.ready());
    assert_eq!(app.active(), Some("part.geop"));
    assert!(dir.join("part.geop").exists());
    let program = app.program.as_ref().unwrap();
    assert!(!program.operations.is_empty());
    assert_eq!(program.path.as_deref(), Some("part.geop"));
}

/// A sketch drawn as the window draws it: the xy plane picked where it is
/// seen, the view turning to face it, a line drawn by two clicks, and the
/// step put into the program — and into its file.
#[test]
fn a_line_is_sketched_by_clicks_in_the_view_and_saved() {
    let (mut app, dir) = opened("sketch");
    app.camera.projection = Projection::Perspective;
    app.new_step("add_sketch");
    app.settle();
    assert_eq!(app.step.as_ref().map(|s| s.kind), Some("add_sketch"));
    // The xy plane of the origin, picked where its square is drawn.
    let reach = app.camera.reach_at([0.0; 3]);
    click_at(&mut app, [3.5 * reach, 3.5 * reach, 0.0]);
    let plane = app.plane().expect("the sketch works in the plane picked");
    assert!((plane.w[2].abs() - 1.0).abs() < 1e-9, "{plane:?}");
    glide(&mut app);
    assert!(app.camera.pose.back()[2] > 0.999, "{:?}", app.camera.pose);
    app.key("l");
    click_at(&mut app, [10.0, 10.0, 0.0]);
    click_at(&mut app, [30.0, 20.0, 0.0]);
    app.key("Escape");
    app.key("Escape");
    app.settle();
    assert!(
        app.can_commit(),
        "{:?}",
        app.step.as_ref().map(|s| &s.error)
    );
    app.commit();
    app.settle();
    assert!(app.step.is_none(), "{:?}", app.error);
    let steps = &app.program.as_ref().unwrap().steps;
    assert_eq!(steps.len(), 1);
    // Back where it was, the plane put away.
    glide(&mut app);
    assert!(app.plane().is_none());
    let file = on_disk(&dir, "part.geop");
    let curves = &file["steps"][0]["args"]["sketch"]["curves"];
    let drawn = (curves.as_object().unwrap().values())
        .filter(|c| !c["fixed"].as_bool().unwrap_or(false))
        .count();
    assert_eq!(drawn, 1, "{curves}");
}

#[test]
fn an_example_gets_a_file_of_its_own_and_is_framed() {
    let (mut app, dir) = opened("example");
    let name = app.program.as_ref().unwrap().examples[0];
    app.load_example(name);
    app.settle();
    assert_eq!(app.error, None);
    let path = format!("{name}.geop");
    assert_eq!(app.active(), Some(path.as_str()));
    let file = on_disk(&dir, &path);
    assert!(!file["steps"].as_array().unwrap().is_empty());
    assert!(app.camera.gliding(), "framed");
    assert!(app.scene.triangles() > 0);
    // Undone, the file follows.
    app.undo();
    app.settle();
    let steps = on_disk(&dir, &path)["steps"].as_array().unwrap().len();
    assert!(steps < file["steps"].as_array().unwrap().len());
}

#[test]
fn a_copied_step_goes_to_the_clipboard_and_pastes_after() {
    let (mut app, _dir) = opened("copy");
    let name = app.program.as_ref().unwrap().examples[0];
    app.load_example(name);
    app.settle();
    let first = app.program.as_ref().unwrap().steps[0].id.clone();
    let count = app.program.as_ref().unwrap().steps.len();
    app.copy_step(&first);
    app.settle();
    let effects = app.effects();
    let [Effect::Copy(text)] = effects.as_slice() else {
        panic!("a copy: {effects:?}");
    };
    app.paste(text);
    app.settle();
    assert_eq!(app.error, None);
    assert_eq!(app.program.as_ref().unwrap().steps.len(), count + 1);
}

/// The kernel panics on purpose; it is started afresh with the files and
/// the program edited, and says so.
#[test]
fn a_crashed_kernel_is_given_the_program_again() {
    let (mut app, _dir) = opened("crash");
    let name = app.program.as_ref().unwrap().examples[0];
    app.load_example(name);
    app.settle();
    let steps = app.program.as_ref().unwrap().steps.len();
    app.command(geop_core_editor::editor::Command::Crash);
    app.settle();
    assert!(app.crashed.is_some());
    assert_eq!(app.program.as_ref().unwrap().steps.len(), steps);
    assert!(app.scene.triangles() > 0);
}

/// Hovers wait while one is answered, the latest one sent next; a click
/// is never dropped.
#[test]
fn hovers_are_sent_one_at_a_time_the_latest_winning() {
    let (mut app, _dir) = opened("hover");
    let pointer = app.camera.pointer(500.0, 400.0).unwrap();
    for _ in 0..50 {
        app.hover(pointer, false);
    }
    app.click(pointer, Button::Primary, false, false);
    app.settle();
    assert_eq!(app.error, None);
    assert_eq!(
        app.pointer_events_sent, 1,
        "the first hover: the click says where the rest went"
    );
}
