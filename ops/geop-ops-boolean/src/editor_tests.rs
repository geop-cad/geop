//! Imprint and section set up as the front end sets them up: the body,
//! the tool and the face picked by clicks.

use geop_core_editor::{
    editing::Value,
    editor::{Command, Editor},
    registry::Registry,
    testing::{click_at, dialog, editor_with, from_above, handled, pointer},
};
use geop_core_program::{program::Program, testing::args};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::element;

use crate::{ImprintArgs, SectionArgs, SectionWith};

/// What these tests edit: given blocks, and this crate's operations.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_lib_fixtures::register)
        .and_then(crate::register)
        .expect("the operations each have a name of their own")
}

/// An editor on two unit blocks, `block(a)` and `block(b)`, the second
/// moved by half along each axis.
fn on_two_blocks() -> Editor {
    let mut program = Program::new();
    program.push("a", PrimitiveArgs::block([0.0; 3], [1.0; 3]));
    program.push("b", PrimitiveArgs::block([0.5; 3], [1.5; 3]));
    editor_with(registry(), program)
}

/// Commits the step open in `editor`, which has to build without an error.
fn committed(editor: &mut Editor) {
    let update = handled(editor, Command::Commit);
    let program = update.program.expect("the program changed");
    assert!(
        program.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        program.steps
    );
}

/// A new imprint takes the two blocks, the newer on the older; a click
/// from above on the older's top, beside the newer, narrows it to that
/// face. Both blocks stay.
#[test]
fn imprint_picks_the_face_to_divide() {
    let mut editor = on_two_blocks();
    handled(
        &mut editor,
        Command::New {
            kind: "imprint".into(),
        },
    );
    handled(&mut editor, dialog("face", Value::Press));
    handled(&mut editor, click_at(from_above(0.25, 0.25)));
    committed(&mut editor);
    let args: ImprintArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.body, element("block(a)"));
    assert_eq!(args.tool, element("block(b)"));
    let face = args.face.expect("a face picked");
    assert!(face.name().starts_with("block(a,"), "{face:?}");
    let scene = (editor.handle(Command::Show).scene).expect("the scene is sent");
    assert_eq!(scene.part.solids.len(), 2, "{:?}", scene.part.solids);
}

/// A new section cuts the newer block; a click on the older block's side
/// cuts it with that block instead of the default plane.
#[test]
fn section_picks_the_body_to_cut_with() {
    let mut editor = on_two_blocks();
    handled(
        &mut editor,
        Command::New {
            kind: "section".into(),
        },
    );
    handled(&mut editor, dialog("with", Value::Press));
    handled(
        &mut editor,
        click_at(pointer([-10.0, 0.25, 0.25], [1.0, 0.0, 0.0])),
    );
    committed(&mut editor);
    let args: SectionArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.body, element("block(b)"));
    assert_eq!(args.with, SectionWith::Body(element("block(a)")));
}
