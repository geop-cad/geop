//! The direct edits set up as the front end sets them up: faces picked by
//! clicks, the shift dragged or typed into the dialog.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{Button, StepEditEvent, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{Stepping, dialog, pointer},
};
use geop_core_program::testing::args;
use geop_lib_fixtures::bodies::block;
use geop_ops_model::element;

/// What these tests edit: a given box, and this crate's edits.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_lib_fixtures::register)
        .and_then(crate::register)
        .expect("the operations each have a name of their own")
}

fn click(at: [f64; 3], towards: [f64; 3]) -> Command {
    Command::Event {
        event: StepEditEvent::Click {
            pointer: pointer(at, towards),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    }
}

/// An editor on the 2 x 2 x 1 box `block(body)`.
fn on_a_box() -> Editor {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = editor.handle(Command::Load {
        program: block(),
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor
}

/// A new move picks the box's top by a click from above, is shifted up by
/// typing, and builds the taller box.
#[test]
fn move_face_picks_a_face_and_shifts_it() {
    let mut editor = on_a_box();
    editor.handle(Command::New {
        kind: "move_face".into(),
    });
    let update = editor.handle(click([1.0, 1.0, 10.0], [0.0, 0.0, -1.0]));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("z", Value::Number(0.5)));
    let update = editor.handle(Command::Commit);
    let program = update.program.expect("the program changed");
    assert!(
        program.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        program.steps
    );
    let args: crate::MoveFaceArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.faces, [element("block(body,end)")], "{args:?}");
    assert_eq!(args.translation, [0.0, 0.0, 0.5]);
    let scene = editor
        .handle(Command::Show)
        .scene
        .expect("the scene is sent");
    assert!(
        scene
            .part
            .solids
            .iter()
            .any(|s| s.starts_with("move_face(")),
        "{:?}",
        scene.part.solids
    );
}

/// A new offset picks a side by a click and pulls it out by the distance
/// typed in.
#[test]
fn offset_face_picks_a_face_and_pulls_it() {
    let mut editor = on_a_box();
    editor.handle(Command::New {
        kind: "offset_face".into(),
    });
    let update = editor.handle(click([10.0, 1.0, 0.5], [-1.0, 0.0, 0.0]));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("distance", Value::Number(0.25)));
    let update = editor.handle(Command::Commit);
    let program = update.program.expect("the program changed");
    assert!(
        program.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        program.steps
    );
    let args: crate::OffsetFaceArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.faces.len(), 1, "{args:?}");
    assert_eq!(args.distance, 0.25);
}

/// A new delete picks the box's top by a click, is told to leave it open,
/// and leaves a sheet: no solid in the scene.
#[test]
fn delete_face_picks_a_face_and_leaves_it_open() {
    let mut editor = on_a_box();
    editor.handle(Command::New {
        kind: "delete_face".into(),
    });
    let update = editor.handle(click([1.0, 1.0, 10.0], [0.0, 0.0, -1.0]));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("heal", Value::Choice("open".into())));
    let update = editor.handle(Command::Commit);
    let program = update.program.expect("the program changed");
    assert!(
        program.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        program.steps
    );
    let args: crate::DeleteFaceArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.faces, [element("block(body,end)")], "{args:?}");
    assert_eq!(args.heal, crate::HealGap::Open);
    let scene = editor
        .handle(Command::Show)
        .scene
        .expect("the scene is sent");
    assert!(scene.part.solids.is_empty(), "{:?}", scene.part.solids);
}

/// A new merge takes the newest solid as it opens, and builds.
#[test]
fn merge_faces_takes_the_newest_solid() {
    let mut editor = on_a_box();
    editor.handle(Command::New {
        kind: "merge_faces".into(),
    });
    let update = editor.handle(Command::Commit);
    let program = update.program.expect("the program changed");
    assert!(
        program.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        program.steps
    );
    let args: crate::MergeFacesArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.bodies.len(), 1, "{args:?}");
    assert_eq!(args.bodies[0].name(), "block(body)");
}

/// A new projection is told to project along the face's normal: the step
/// holds it.
#[test]
fn project_curve_projects_along_the_face_normal() {
    let mut step = Stepping::new(on_a_box(), "project_curve");
    let args: crate::ProjectCurveArgs = step.args();
    assert_eq!(args.along, crate::Along::SketchNormal);
    step.dialog("along", Value::Choice("face_normal".into()));
    let args: crate::ProjectCurveArgs = step.args();
    assert_eq!(args.along, crate::Along::FaceNormal);
}
