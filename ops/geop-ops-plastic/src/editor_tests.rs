//! Drafts and ribs set up as the front end sets them up, on the shelled
//! enclosure: faces and planes picked by clicks, directions chosen in the
//! dialog.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{Button, StepEditEvent, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{dialog, pointer},
};
use geop_core_program::testing::args;
use geop_ops_model::element;

/// What these tests edit: the enclosure's sketches, extrusions, shell and
/// reference plane, and the plastic part's features.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_reference_geometry::register)
        .and_then(geop_ops_extrude_revolve::register)
        .and_then(geop_ops_shell::register)
        .and_then(crate::register)
        .and_then(geop_ops_parameters::register)
        .expect("the operations each have a name of their own")
}

/// A new draft picks the faces to tilt by clicking them, then the neutral
/// plane: the enclosure's +x wall from the side, its bottom from below.
#[test]
fn new_draft_picks_its_faces_and_neutral_plane() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = editor.handle(Command::Load {
        program: crate::testing::enclosure(),
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "draft".into(),
    });
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let update = editor.handle(click(pointer([10.0, 1.0, 0.5], [-1.0, 0.0, 0.0])));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("neutral", Value::Press));
    let update = editor.handle(click(pointer([1.0, 1.0, -10.0], [0.0, 0.0, 1.0])));
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(dialog("angle", Value::Number(5.0)));
    let step = editor
        .handle(Command::Commit)
        .program
        .expect("the program changed");
    assert!(step.steps.iter().all(|s| s.error.is_none()));
    let args: crate::DraftArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.faces,
        [element(crate::testing::wall("c5"))],
        "{args:?}"
    );
    assert_eq!(
        args.neutral,
        Some(geop_ops_sketch::SketchPlane::Face(element(
            "extrude(box,start)"
        )))
    );
    assert_eq!(args.angle, 5.0);
    let scene = editor
        .handle(Command::Seek { marker: None })
        .scene
        .expect("the scene is sent");
    assert!(
        scene.part.solids.iter().any(|s| s.starts_with("draft(")),
        "{:?}",
        scene.part.solids
    );
}

/// A new rib starts from the newest sketch and solid; set to grow down,
/// parallel to its sketch, it builds.
#[test]
fn new_rib_grows_from_the_newest_sketch() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = editor.handle(Command::Load {
        program: crate::testing::enclosure_with_rib_sketch(),
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New { kind: "rib".into() });
    let step = update.step.expect("a step is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    editor.handle(dialog("direction", Value::Choice("parallel".into())));
    let update = editor.handle(dialog("flipped", Value::Bool(true)));
    let step = update.step.expect("the rib is edited");
    assert_eq!(step.error, None);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: crate::RibArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.sketch, geop_ops_sketch::whole_sketch("rib_sketch"));
    assert_eq!(args.solid.name(), "shell(s)");
    assert!(args.flipped);
    let scene = update.scene.expect("the scene is sent anew");
    assert!(
        scene.part.solids.iter().any(|s| s.starts_with("rib(")),
        "{:?}",
        scene.part.solids
    );
}
