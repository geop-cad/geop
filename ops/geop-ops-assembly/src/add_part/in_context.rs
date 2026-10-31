//! Top-down design: a part placed in an assembly built against a part
//! placed beside it, in context (`/../bracket:…`). A datum of the mount
//! lies on the bracket's top, wherever the bracket is and however thick it
//! is made; the mount is built again when the bracket changes; a mount
//! placed before the bracket is refused by name; and editing the mount's
//! step where it is placed picks the bracket's face.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::{
    editing::Value,
    editor::{Command, Editor},
    testing::{click_at, dialog, from_above, handled},
};
use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_program::{
    program::Program,
    testing::{args as step_args, assert_incremental, assert_reopens, assert_round_trips},
    values::{StateValue, StateValues, pose_state},
    workspace::{Files, Workspace},
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_lib_stdlib::WithStandardParts;
use geop_ops_model::{
    datums::{ORIGIN, component_name, datum, read_datums},
    element,
    selection::Selected,
};
use geop_ops_reference_geometry::{AddDatumArgs, Construction};

use crate::{
    AddPartArgs,
    examples::pose,
    program_tests::{build_file, editor, registry},
};

/// A bracket `thickness` thick on the origin: a block of 10 × 10.
fn bracket(thickness: f64) -> Program {
    let mut program = Program::new();
    program.push(
        "bracket",
        PrimitiveArgs::block([0.0; 3], [10.0, 10.0, thickness]),
    );
    program
}

/// A mount whose datum `top` lies on what `selected` names: the face the
/// bracket placed beside it has on top, read in context — or, before it is
/// put there, its own origin's `yz` plane.
fn mount(selected: Selected) -> Program {
    let mut program = Program::new();
    program.push(
        "top",
        AddDatumArgs {
            selection: vec![selected],
            construction: Construction::Offset {
                distance: 0.0.into(),
            },
        },
    );
    program
}

/// The bracket's top, read from a part placed beside it.
fn brackets_top() -> Selected {
    Selected::Face(element("block(bracket,end)")).behind("../bracket")
}

/// The mount's own `yz` plane.
fn own_plane() -> Selected {
    let name = component_name(ORIGIN, DatumComponent::Plane(FrameAxis::X));
    Selected::Datum(datum(&name))
}

/// An assembly of the bracket and the mount — in that order, or the mount
/// first — both fixed, the mount 5 above the origin.
fn assembly(mount_first: bool) -> Program {
    let place = |file: &str| {
        AddPartArgs {
            file: file.into(),
            ..AddPartArgs::default()
        }
        .fixed()
    };
    let mut program = Program::new();
    let parts = [("bracket", "bracket.geop"), ("mount", "mount.geop")];
    let order: Vec<_> = match mount_first {
        true => parts.iter().rev().collect(),
        false => parts.iter().collect(),
    };
    for (id, file) in order {
        program.push(*id, place(file));
    }
    program.state = StateValues::from([
        (
            pose_state("bracket"),
            StateValue::Pose(pose([0.0; 3], [0.0; 3])),
        ),
        (
            pose_state("mount"),
            StateValue::Pose(pose([0.0, 0.0, 5.0], [0.0; 3])),
        ),
    ]);
    program
}

/// The files of the assembly, a bracket `thickness` thick and the mount.
fn files(thickness: f64, mount_first: bool) -> BTreeMap<String, String> {
    BTreeMap::from([
        ("asm.geop".into(), assembly(mount_first).to_json().unwrap()),
        ("bracket.geop".into(), bracket(thickness).to_json().unwrap()),
        (
            "mount.geop".into(),
            mount(brackets_top()).to_json().unwrap(),
        ),
    ])
}

/// The workspace of `files`, beside the standard parts.
fn workspace(files: BTreeMap<String, String>) -> Rc<dyn Workspace> {
    Files::new(WithStandardParts(files), Rc::new(registry().operations()))
}

/// The height of the mount's datum `top`, in the mount's own frame, as
/// the assembly of a bracket `thickness` thick builds it.
fn height_of_top(thickness: f64) -> f64 {
    let files = files(thickness, false);
    let built = build_file(&assembly(false), "asm.geop", workspace(files)).unwrap();
    let mount = built.part.current().placed_part("mount").unwrap();
    let top = read_datums(&mount).unwrap().get("top").unwrap();
    top.frame.origin()[2].to_f64()
}

/// The mount's datum lies on the bracket's top as the assembly has it,
/// seen from the mount 5 above: 4 below it for a bracket 1 thick, 3 below
/// for one 2 thick.
#[test]
fn a_part_is_built_against_the_part_placed_beside_it() {
    assert!((height_of_top(1.0) + 4.0).abs() < 1e-9);
    assert!((height_of_top(2.0) + 3.0).abs() < 1e-9);
}

/// A mount placed before the bracket it reads is refused, naming both:
/// a part reads only the parts placed before it.
#[test]
fn a_part_placed_before_what_it_reads_is_refused_by_name() {
    let built = build_file(&assembly(true), "asm.geop", workspace(files(1.0, true)));
    let error = built.err().expect("refused").to_string();
    assert!(
        error.contains("`mount` reads `bracket` placed beside it in context"),
        "{error}"
    );
}

/// The assembly is saved and read back as it was; the bracket made
/// thicker builds as from scratch; and reopened, it makes nothing it kept.
#[test]
fn an_assembly_built_in_context_round_trips_reopens_and_builds_as_from_scratch() {
    let operations = Rc::new(registry().operations());
    let program = assembly(false);
    assert_round_trips(
        operations.clone(),
        Some((workspace(files(1.0, false)), "asm.geop")),
        &program,
    );
    for thickness in [1.0, 2.0] {
        assert_incremental(
            operations.clone(),
            Some((workspace(files(thickness, false)), "asm.geop")),
            &[program.clone(), program.clone()],
        );
        let files = files(thickness, false);
        let [first, again] = assert_reopens(
            operations.clone(),
            |cache| Files::with_cache(WithStandardParts(files.clone()), operations.clone(), cache),
            "asm.geop",
        );
        assert!(first.kept_makes > 0);
        assert_eq!(again.kept_makes, 0);
    }
}

/// An editor of the assembly, its mount's datum still on its own plane.
fn assembly_editor() -> Editor {
    let mut editor = editor();
    let mut files = files(1.0, false);
    files.insert("mount.geop".into(), mount(own_plane()).to_json().unwrap());
    let files = files.into_iter().map(|(p, t)| (p, Some(t))).collect();
    handled(&mut editor, Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program: assembly(false),
            path: Some("asm.geop".into()),
        },
    );
    editor
}

/// Editing the mount's datum where the mount is placed, a click on the
/// bracket's top picks it in context; committed, the mount's file holds
/// it, and the datum lies on the bracket. The bracket made thicker, the
/// datum follows.
#[test]
fn a_step_edited_where_its_part_is_placed_picks_the_part_beside_it() {
    let mut editor = assembly_editor();
    handled(
        &mut editor,
        Command::Open {
            id: "top".into(),
            placed: Some("mount".into()),
        },
    );
    handled(&mut editor, dialog("selection", Value::Clear));
    handled(&mut editor, dialog("selection", Value::Press));
    let update = handled(&mut editor, click_at(from_above(5.0, 5.0)));
    let step = update.step.expect("the datum is edited");
    assert!(
        step.presentation
            .highlights
            .iter()
            .any(|picked| picked.target.to_string() == "/bracket:model"),
        "the bracket's top is lit where it is drawn: {:?} {:?} {:?}",
        step.presentation.highlights,
        step.presentation.dialog.get("selection"),
        step.presentation.pickable,
    );
    assert_eq!(step.error, None);
    let update = handled(&mut editor, Command::Commit);
    let written = update.written.expect("the mount's file is written");
    assert_eq!(written[0].path, "mount.geop");
    let datum: AddDatumArgs = step_args(&written[0].program.steps[0]);
    assert_eq!(datum.selection, [brackets_top()]);

    let height = |editor: &Editor| {
        let mount = editor.part().placed_part("mount").unwrap();
        let top = read_datums(&mount).unwrap().get("top").unwrap();
        top.frame.origin()[2].to_f64()
    };
    assert!((height(&editor) + 4.0).abs() < 1e-9, "{}", height(&editor));
    let mount = written[0].program.to_json().unwrap();
    let files = BTreeMap::from([
        ("mount.geop".to_string(), Some(mount)),
        (
            "bracket.geop".to_string(),
            Some(bracket(2.0).to_json().unwrap()),
        ),
    ]);
    handled(&mut editor, Command::Files { files });
    assert!((height(&editor) + 3.0).abs() < 1e-9, "{}", height(&editor));
}

/// A new step of the mount, started where the mount is placed, picks the
/// bracket's top at once — what it is built on is what it needs first —
/// and is written after the mount's steps, to the mount's file.
#[test]
fn a_new_step_of_a_placed_part_is_built_against_the_part_beside_it() {
    let mut editor = assembly_editor();
    handled(
        &mut editor,
        Command::NewIn {
            kind: "add_datum".into(),
            placed: "mount".into(),
        },
    );
    handled(&mut editor, click_at(from_above(5.0, 5.0)));
    handled(
        &mut editor,
        dialog("construction", Value::Choice("offset".into())),
    );
    let update = handled(&mut editor, Command::Commit);
    let written = update.written.expect("the mount's file is written");
    assert_eq!(written[0].path, "mount.geop");
    let steps = &written[0].program.steps;
    assert_eq!(steps.len(), 2, "{steps:?}");
    let datum: AddDatumArgs = step_args(&steps[1]);
    assert_eq!(datum.selection, [brackets_top()]);
    assert!(
        editor.program().step(&steps[1].id).is_err(),
        "not the assembly's"
    );
}
