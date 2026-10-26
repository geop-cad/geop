//! A hole made as the front end makes one: its face and centre clicked on
//! a plate, its kind, size and end chosen in the dialog.

use geop_core_editor::{
    editing::{Control, Value},
    editor::Command,
    registry::Registry,
    testing::{click_at, dialog, editor_with, handled, pointer},
};
use geop_core_program::testing::args;
use geop_ops_model::element;

/// What these tests edit: given plates, the sketches of the holes' centres,
/// and holes.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_lib_fixtures::register)
        .and_then(geop_ops_sketch::register)
        .and_then(crate::register)
        .expect("the operations each have a name of their own")
}

/// A new hole, the way the front end makes one: the plate's top clicked
/// as the face, the sketch's point clicked as the centre, a counterbored
/// M5 through all chosen in the dialog — which says what that comes to —
/// committed, and the plate drilled; made tapped, its thread drawn.
#[test]
fn new_hole_from_the_dialog() {
    let mut editor = editor_with(registry(), crate::testing::plate(&[[20.0, 20.0]]));
    editor.handle(Command::New {
        kind: "hole".into(),
    });
    let above = |x: f64, y: f64| pointer([x, y, 50.0], [0.0, 0.0, -1.0]);
    handled(&mut editor, click_at(above(40.0, 30.0)));
    editor.handle(dialog("points", Value::Press));
    handled(&mut editor, click_at(above(20.0, 20.0)));
    editor.handle(dialog("kind", Value::Choice("counterbore".into())));
    editor.handle(dialog("size", Value::Choice("M5".into())));
    let update = editor.handle(dialog("end", Value::Choice("through_all".into())));
    let step = update.step.expect("the hole is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let shown = &step.presentation.dialog;
    match shown.get("dimensions") {
        Some(Control::Text { text, .. }) => {
            assert_eq!(text, "Ø5.5, counterbore Ø10 × 5 deep")
        }
        other => panic!("{other:?}"),
    }
    assert!(shown.get("depth").is_none(), "through all has no depth");
    // The hole and its counterbore, drawn on the face.
    let circles = step
        .presentation
        .visuals
        .iter()
        .filter(|v| v.key.starts_with("hole0"))
        .count();
    assert_eq!(circles, 2);
    let update = handled(&mut editor, Command::Commit);
    let args = args::<crate::HoleArgs>(editor.program().steps.last().unwrap());
    assert_eq!(args.face, Some(element(crate::testing::PLATE_TOP)));
    assert_eq!(
        args.points,
        [crate::HoleCentre::SketchPoint(
            geop_ops_model::sketch::sketch_element("centres", "p0")
        )]
    );
    assert_eq!(args.kind, crate::HoleKind::Counterbore);
    assert_eq!(args.end, geop_ops_model::Extent::ThroughAll);
    let scene = update.scene.expect("the scene changed");
    assert_eq!(scene.part.solids, ["hole(hole1)"]);

    // Tapped instead: the scene draws its cosmetic thread.
    editor.handle(Command::Open {
        id: "hole1".into(),
        placed: None,
    });
    editor.handle(dialog("kind", Value::Choice("tapped".into())));
    handled(&mut editor, Command::Commit);
    let scene = editor
        .handle(Command::Show)
        .scene
        .expect("the scene changed");
    assert_eq!(scene.part.annotations.len(), 1);
    assert_eq!(scene.part.annotations[0].label, "M5x0.8");
}

/// A pipe-thread hole made in the dialog: NPT picked as the standard makes
/// the hole tapped, the dialog says it is reamed to the taper, and the
/// scene draws its tapered cosmetic thread.
#[test]
fn npt_hole_from_the_dialog() {
    let mut editor = editor_with(registry(), crate::testing::plate(&[[20.0, 20.0]]));
    editor.handle(Command::New {
        kind: "hole".into(),
    });
    let above = |x: f64, y: f64| pointer([x, y, 50.0], [0.0, 0.0, -1.0]);
    handled(&mut editor, click_at(above(40.0, 30.0)));
    editor.handle(dialog("points", Value::Press));
    handled(&mut editor, click_at(above(20.0, 20.0)));
    editor.handle(dialog("standard", Value::Choice("npt".into())));
    let update = editor.handle(dialog("size", Value::Choice("1/8".into())));
    let step = update.step.expect("the hole is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let shown = &step.presentation.dialog;
    match shown.get("kind") {
        Some(Control::Select { value, .. }) => assert_eq!(value, "tapped"),
        other => panic!("{other:?}"),
    }
    match shown.get("dimensions") {
        Some(Control::Text { text, .. }) => {
            assert_eq!(text, "reamed 1:16 from Ø8.737, thread 1/8-27 NPT")
        }
        other => panic!("{other:?}"),
    }
    handled(&mut editor, Command::Commit);
    let args = args::<crate::HoleArgs>(editor.program().steps.last().unwrap());
    assert_eq!(args.kind, crate::HoleKind::Tapped);
    assert_eq!(args.standard, crate::Standard::Npt { size: "1/8".into() });
    let scene = editor
        .handle(Command::Show)
        .scene
        .expect("the scene changed");
    assert_eq!(scene.part.solids, ["hole(hole1)"]);
    assert_eq!(scene.part.annotations.len(), 1);
    assert_eq!(scene.part.annotations[0].label, "1/8-27 NPT");
}
