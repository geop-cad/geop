//! A shell set up as the front end sets it up: its solid and the faces to
//! open clicked on the drilled box.

use geop_core_editor::{
    editing::{Button, StepEditEvent, Value},
    editor::Command,
    registry::Registry,
    testing::{dialog, editor_with, from_above},
};
use geop_core_program::testing::args;
use geop_lib_fixtures::drilled_box;

/// What these tests edit: the drilled box's given solids, and shells.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_lib_fixtures::register)
        .and_then(crate::register)
        .expect("the operations each have a name of their own")
}

/// `value` as it is saved: what arguments holding typed references are
/// compared by.
fn saved(value: impl serde::Serialize) -> serde_json::Value {
    serde_json::to_value(value).unwrap()
}

/// A new shell picks its solid, then the faces to open — faces of that
/// solid, clicked on it.
#[test]
fn new_shell_picks_its_faces_on_the_solid() {
    let mut editor = editor_with(registry(), drilled_box());
    editor.handle(Command::New {
        kind: "shell".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // From above, onto the box's top beside the hole.
    let above = |x: f64, y: f64| from_above(x, y);
    let update = editor.handle(click(above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("faces", Value::Press));
    let update = editor.handle(click(above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the shell is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: crate::ShellArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(saved(&args.faces), saved(["block(box,end)"]), "{args:?}");
}

/// Faces of another thickness are picked as the open ones are, and their
/// thickness is asked only once there are some: the box's top, clicked,
/// gets walls 0.3 thick.
#[test]
fn faces_of_another_thickness_are_picked_on_the_solid() {
    let mut editor = editor_with(registry(), drilled_box());
    editor.handle(Command::New {
        kind: "shell".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let update = editor.handle(click(from_above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the shell is edited");
    assert!(
        !format!("{:?}", step.presentation).contains("other_thickness"),
        "{:?}",
        step.presentation
    );
    editor.handle(dialog("other_faces", Value::Press));
    let update = editor.handle(click(from_above(0.3, 0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(dialog("other_thickness", Value::Number(0.3)));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: crate::ShellArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        saved(&args.other_faces),
        saved(["block(box,end)"]),
        "{args:?}"
    );
    assert_eq!(saved(&args.other_thickness), saved(0.3), "{args:?}");
}
