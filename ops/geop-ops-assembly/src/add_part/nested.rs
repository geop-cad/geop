//! Editing a part placed in a part placed in a part, where it is placed:
//! a duck places a neck, the neck a collar and then a head, the head an
//! eye and then a mouth. The mouth's steps are listed under the head's,
//! under the neck's; a step of the mouth started where it is placed reads
//! the eye beside it (`../eye`) and the collar beside the head
//! (`../../collar`) in context; committed, it is written to the mouth's
//! file, and undone, the file is as it was.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::Value,
    editor::{Command, Editor, PlacedSteps, StepInfo},
    selectable::Selectable,
    testing::{dialog, handled},
};
use geop_core_program::{
    program::Program,
    testing::args as step_args,
    values::{StateValue, StateValues, pose_state},
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::{datums::read_datums, element, placed::locate, selection::Selected};
use geop_ops_reference_geometry::AddDatumArgs;

use crate::{AddPartArgs, examples::pose, program_tests::editor};

/// A part of the block `id` from the origin to `top`, placing the parts
/// `placed` — `(id, file, where)` — each fixed where its pose puts it.
fn part(id: &str, top: [f64; 3], placed: &[(&str, &str, [f64; 3])]) -> Program {
    let mut program = Program::new();
    program.push(id, PrimitiveArgs::block([0.0; 3], top));
    let mut state = StateValues::new();
    for (id, file, at) in placed {
        let args = AddPartArgs {
            file: file.to_string(),
            ..AddPartArgs::default()
        };
        program.push(*id, args.fixed());
        state.insert(pose_state(id), StateValue::Pose(pose(*at, [0.0; 3])));
    }
    program.state = state;
    program
}

/// Where the head is in the neck, and the mouth in the head.
const HEAD: [f64; 3] = [0.0, 0.0, 10.0];
const MOUTH: [f64; 3] = [5.0, 0.0, 1.0];

/// The duck's files: the eye is 3 tall, the collar 4.
fn files() -> BTreeMap<String, String> {
    let neck = part(
        "neck",
        [1.0, 1.0, 10.0],
        &[
            ("collar", "collar.geop", [0.0; 3]),
            ("head", "head.geop", HEAD),
        ],
    );
    let head = part(
        "head",
        [2.0, 2.0, 2.0],
        &[
            ("eye", "eye.geop", [0.0; 3]),
            ("mouth", "mouth.geop", MOUTH),
        ],
    );
    [
        (
            "duck.geop",
            part("body", [4.0, 4.0, 1.0], &[("neck", "neck.geop", [0.0; 3])]),
        ),
        ("neck.geop", neck),
        ("collar.geop", part("collar", [1.0, 1.0, 4.0], &[])),
        ("head.geop", head),
        ("eye.geop", part("eye", [1.0, 1.0, 3.0], &[])),
        ("mouth.geop", part("mouth", [1.0, 1.0, 1.0], &[])),
    ]
    .into_iter()
    .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
    .collect()
}

/// An editor of the duck, every file sent.
fn duck_editor() -> (Editor, Vec<StepInfo>) {
    let mut editor = editor();
    let mut files = files();
    let duck = Program::from_json(&files.remove("duck.geop").unwrap()).unwrap();
    let files = files.into_iter().map(|(p, t)| (p, Some(t))).collect();
    handled(&mut editor, Command::Files { files });
    let update = handled(
        &mut editor,
        Command::Load {
            program: duck,
            path: Some("duck.geop".into()),
        },
    );
    // Every part where its file places it: the mouth 11 up in the neck.
    let mouth = locate(&editor.part(), "neck/head/mouth").unwrap().world;
    let at = mouth.position().to_array().map(|c| c.to_f64());
    assert_eq!(at, [5.0, 0.0, 11.0], "{:?}", editor.program().state);
    (editor, update.program.unwrap().steps)
}

/// The part placed at `path` of what the step `id` of `steps` placed.
fn placed_at<'s>(steps: &'s [StepInfo], id: &str, path: &str) -> &'s PlacedSteps {
    let step = steps
        .iter()
        .find(|s| s.id == id)
        .expect("the step is listed");
    let listed: Vec<&str> = step.placed.iter().map(|p| p.instance.as_str()).collect();
    (step.placed.iter())
        .find(|p| p.instance == path)
        .unwrap_or_else(|| panic!("{path} is not listed under {id}: {listed:?}"))
}

/// The parts placed are listed in the structure however deep, by their
/// paths; the steps of the mouth are listed under the head's step placing it,
/// under the neck's placing the head, under the duck's placing the neck:
/// each part by its path from the duck, from its own file.
#[test]
fn the_parts_placed_in_placed_parts_are_listed_however_deep() {
    let (mut editor, steps) = duck_editor();
    // The structure lists them as a tree: each before those placed in it.
    let scene = handled(&mut editor, Command::Show).scene.unwrap();
    let parts: Vec<String> = (scene.structure.unwrap().into_iter())
        .filter(|item| item.kind == "part")
        .map(|item| item.name)
        .collect();
    assert_eq!(
        parts,
        [
            "neck",
            "neck/collar",
            "neck/head",
            "neck/head/eye",
            "neck/head/mouth"
        ]
    );
    let neck = placed_at(&steps, "neck", "neck");
    assert_eq!(neck.file, "neck.geop");
    let head = placed_at(&neck.steps, "head", "neck/head");
    assert_eq!(head.file, "head.geop");
    let mouth = placed_at(&head.steps, "mouth", "neck/head/mouth");
    assert_eq!(mouth.file, "mouth.geop");
    let ids: Vec<&str> = mouth.steps.iter().map(|s| s.id.as_str()).collect();
    assert_eq!(ids, ["mouth"]);
    assert!(mouth.steps[0].placed.is_empty());
}

/// The mouth with a datum `top` on what `selected` names, read in context.
fn mouth_on(selected: Selected) -> Program {
    let mut mouth = part("mouth", [1.0, 1.0, 1.0], &[]);
    mouth.push(
        "top",
        AddDatumArgs {
            selection: vec![selected],
            construction: geop_ops_reference_geometry::Construction::Offset {
                distance: 0.0.into(),
            },
        },
    );
    mouth
}

/// Built from its files, the mouth reads in context through each level: a
/// datum on the eye's top beside it lies 2 above it, one on the collar's
/// top beside the head 7 below it.
#[test]
fn a_part_three_levels_down_reads_what_is_beside_it_at_each_level() {
    for (selected, want) in [
        (
            Selected::Face(element("block(eye,end)")).behind("../eye"),
            2.0,
        ),
        (
            Selected::Face(element("block(collar,end)")).behind("../../collar"),
            -7.0,
        ),
    ] {
        let mut files = files();
        files.insert("mouth.geop".into(), mouth_on(selected).to_json().unwrap());
        let duck = Program::from_json(&files["duck.geop"]).unwrap();
        let workspace = crate::program_tests::workspace(files);
        let built = crate::program_tests::build_file(&duck, "duck.geop", workspace).unwrap();
        let mouth = built.part.current().placed_part("neck/head/mouth").unwrap();
        let top = read_datums(&mouth).unwrap().get("top").unwrap();
        let z = top.frame.origin()[2].to_f64();
        assert!((z - want).abs() < 1e-9, "{z}, not {want}");
    }
}

/// The height of the mouth's datum `id`, in the mouth's own frame.
fn height(editor: &Editor, id: &str) -> Option<f64> {
    let mouth = editor.part().placed_part("neck/head/mouth").unwrap();
    let datums = read_datums(&mouth).unwrap();
    Some(datums.get(id).ok()?.frame.origin()[2].to_f64())
}

/// A new datum of the mouth started where the mouth is placed, three
/// levels down, on what `selected` names: committed, it is written to the
/// mouth's file — what the duck's steps say is the same — and the step's
/// id.
fn new_datum(editor: &mut Editor, selected: Selected) -> String {
    handled(
        editor,
        Command::NewIn {
            kind: "add_datum".into(),
            placed: "neck/head/mouth".into(),
        },
    );
    handled(
        editor,
        dialog("selection", Value::Elements(vec![selected.picked()])),
    );
    handled(
        editor,
        dialog("construction", Value::Choice("offset".into())),
    );
    let before = editor.program().clone();
    let update = handled(editor, Command::Commit);
    let written = update.written.expect("the mouth's file is written");
    assert_eq!(written.len(), 1);
    assert_eq!(written[0].path, "mouth.geop");
    assert_eq!(
        *editor.program(),
        before,
        "the duck's own file is as it was"
    );
    let steps = &written[0].program.steps;
    let datum: AddDatumArgs = step_args(steps.last().unwrap());
    assert_eq!(datum.selection, [selected]);
    steps.last().unwrap().id.clone()
}

/// A datum of the mouth offset (by the dialog's 1) from the eye's top,
/// beside it in the head, lies 3 above the mouth (the eye 3 tall, the
/// mouth 1 up in the head); one from the collar's top, beside the head in
/// the neck, 6 below it (the collar 4 tall, the mouth 11 up in the neck).
/// Each is written to the mouth's file; undone, the mouth's file is as it
/// was, and the datum is gone. A step of the mouth opened where it is
/// placed is the mouth's, listed as edited three levels down.
#[test]
fn a_part_placed_three_levels_down_is_edited_where_it_is_placed() {
    let (mut editor, _) = duck_editor();
    let eye = Selected::Face(element("block(eye,end)")).behind("../eye");
    let on_eye = new_datum(&mut editor, eye);
    let got = height(&editor, &on_eye);
    assert!(got.is_some_and(|z| (z - 3.0).abs() < 1e-9), "{got:?}");

    let collar = Selected::Face(element("block(collar,end)")).behind("../../collar");
    let on_collar = new_datum(&mut editor, collar);
    let got = height(&editor, &on_collar);
    assert!(got.is_some_and(|z| (z + 6.0).abs() < 1e-9), "{got:?}");

    let update = handled(&mut editor, Command::Undo);
    let written = update.written.expect("undo writes the mouth's file back");
    assert_eq!(written[0].path, "mouth.geop");
    assert_eq!(
        written[0].program.steps.len(),
        2,
        "the datum on the eye stays"
    );
    assert_eq!(
        height(&editor, &on_collar),
        None,
        "the datum on the collar is gone"
    );
    assert!(height(&editor, &on_eye).is_some());

    let update = handled(
        &mut editor,
        Command::Open {
            id: on_eye.clone(),
            placed: Some("neck/head/mouth".into()),
        },
    );
    let step = update.step.expect("the mouth's step is edited");
    assert_eq!(step.id.as_deref(), Some(on_eye.as_str()));
    assert_eq!(step.error, None);
    let program = update.program.expect("the program is listed");
    let neck = placed_at(&program.steps, "neck", "neck");
    let head = placed_at(&neck.steps, "head", "neck/head");
    let mouth = placed_at(&head.steps, "mouth", "neck/head/mouth");
    let editing: Vec<&str> = (mouth.steps.iter())
        .filter(|s| s.editing)
        .map(|s| s.id.as_str())
        .collect();
    assert_eq!(editing, [on_eye.as_str()]);
    handled(&mut editor, Command::Cancel);
}
