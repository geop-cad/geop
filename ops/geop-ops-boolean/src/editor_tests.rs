//! Imprint, section and split face set up as the front end sets them up:
//! the body, the tool, the face and the sketch picked by clicks.

use geop_core_editor::{
    editing::Value,
    editor::{Command, Editor},
    registry::Registry,
    testing::{click_at, dialog, editor_with, from_above, handled, pointer},
};
use geop_core_math::{primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{program::Program, testing::args};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::element;
use geop_ops_model::sketch::{
    Sketch,
    figures::{n, on_origin},
    whole_sketch,
};
use geop_ops_sketch::AddSketchArgs;

use crate::{ImprintArgs, SectionArgs, SectionWith, SplitFaceArgs};

/// What these tests edit: given blocks, and this crate's operations.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_lib_fixtures::register)
        .and_then(geop_ops_sketch::register)
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
    // The face is what it still waits for: a click takes it.
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

/// An editor on a block `[-1, 1]^2 x [1, 2]` over the plane `z = 0`, and the
/// two sketches on that plane: a circle `disc`, then a line `across`.
fn on_a_block_and_sketches() -> Editor {
    let mut program = Program::new();
    program.push(
        "block",
        PrimitiveArgs::block([-1.0, -1.0, 1.0], [1.0, 1.0, 2.0]),
    );
    let mut disc = Sketch::new();
    let centre = disc.add_point(n(0.25), n(0.25));
    disc.add_circle(centre, n(0.4));
    program.push("disc", AddSketchArgs::on(on_origin(FrameAxis::Z), disc));
    let mut across = Sketch::new();
    let (a, b) = (
        across.add_point(n(-0.5), n(-2.0)),
        across.add_point(n(-0.5), n(2.0)),
    );
    across.add_line(a, b);
    program.push("across", AddSketchArgs::on(on_origin(FrameAxis::Z), across));
    editor_with(registry(), program)
}

/// A new split takes the newest sketch; a click from above on the block's
/// top takes the face it still waits for. The line is then an edge of the
/// view, between the piece and what kept the name, and the block is whole.
#[test]
fn split_face_picks_the_face_and_takes_the_newest_sketch() {
    let mut editor = on_a_block_and_sketches();
    handled(
        &mut editor,
        Command::New {
            kind: "split_face".into(),
        },
    );
    handled(&mut editor, click_at(from_above(0.25, 0.25)));
    committed(&mut editor);
    let args: SplitFaceArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(args.sketch, whole_sketch("across"));
    let top = args.face.expect("a face picked").name().to_string();
    assert!(top.starts_with("block(block,"), "{top}");

    let scene = (editor.handle(Command::Show).scene).expect("the scene is sent");
    assert_eq!(scene.part.solids.len(), 1, "{:?}", scene.part.solids);
    let step = &editor.program().steps.last().unwrap().id;
    let edge = (scene.part.edges.iter())
        .find(|edge| edge.name.starts_with(&format!("split_face({step},across,")))
        .expect("the line is an edge of the view");
    let piece = format!("split_face({step},{top},across,c2)");
    let mut faces = edge.faces.clone();
    faces.sort();
    let mut expected = vec![piece.clone(), top.clone()];
    expected.sort();
    assert_eq!(faces, expected);
    assert!(edge.polyline.len() >= 2);
    // It runs along the line, over the block's top.
    for p in &edge.polyline {
        assert!(p[0].could_be_equal(Scalar::from_f64(-0.5)), "{p:?}");
        assert!(p[2].could_be_equal(Scalar::from_f64(2.0)), "{p:?}");
    }
    assert!((scene.part.faces.iter()).any(|face| face.name == piece));
}

/// The older sketch is picked by a click on it, from below, where it lies
/// clear of the block: a disc is cut out of the top, and the piece can be
/// picked by a click on it.
#[test]
fn split_face_picks_another_sketch_and_the_piece_is_picked() {
    let mut editor = on_a_block_and_sketches();
    handled(
        &mut editor,
        Command::New {
            kind: "split_face".into(),
        },
    );
    handled(&mut editor, click_at(from_above(0.25, 0.25)));
    handled(&mut editor, dialog("sketch", Value::Press));
    handled(
        &mut editor,
        click_at(pointer([0.25, 0.25, -10.0], [0.0, 0.0, 1.0])),
    );
    committed(&mut editor);
    let picked: SplitFaceArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(picked.sketch, whole_sketch("disc"));

    // The disc is a face: a new split takes it by a click inside it.
    handled(
        &mut editor,
        Command::New {
            kind: "split_face".into(),
        },
    );
    handled(&mut editor, click_at(from_above(0.25, 0.25)));
    let again: SplitFaceArgs = args(editor.editing().expect("a step is edited"));
    let again = again.face.expect("a face picked");
    assert!(again.name().starts_with("split_face("), "{again:?}");
}

/// A refusal reaches the user on the step, naming the sketch and the face,
/// and beside its field.
#[test]
fn split_face_says_why_it_refuses() {
    let mut program = Program::new();
    program.push(
        "block",
        PrimitiveArgs::block([-1.0, -1.0, 1.0], [1.0, 1.0, 2.0]),
    );
    let mut beside = Sketch::new();
    let (a, b) = (
        beside.add_point(n(5.0), n(-2.0)),
        beside.add_point(n(5.0), n(2.0)),
    );
    beside.add_line(a, b);
    program.push("beside", AddSketchArgs::on(on_origin(FrameAxis::Z), beside));
    let mut editor = editor_with(registry(), program);
    handled(
        &mut editor,
        Command::New {
            kind: "split_face".into(),
        },
    );
    let update = editor.handle(click_at(from_above(0.25, 0.25)));
    let step = update.step.expect("the step is still edited");
    let error = step.error.expect("the step says why");
    assert!(error.contains("nothing crosses"), "{error}");
    assert!(error.contains("beside"), "{error}");
}
