//! Parts derived from another, and mirrored components: a part made of
//! another file's bodies, mirrored into the other hand of it, follows that
//! file; an assembly places a part mirrored, listed apart from the part as
//! it is; and both are built incrementally, saved, and reopened as from
//! scratch.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::{
    editing::{Control, StepEditEvent, Value},
    editor::Command,
    testing::{dialog, handled},
};
use geop_core_math::primitives::FrameAxis;
use geop_core_program::{
    program::Program,
    testing::{args as step_args, assert_incremental, assert_reopens, assert_round_trips},
    workspace::{Files, Workspace},
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_lib_stdlib::WithStandardParts;
use geop_ops_model::{
    NamedModel,
    parameters::{Material, PartPropertiesArgs},
    placed::bom::{LineKind, Structure, bom},
    read_model,
};

use crate::{
    AddPartArgs, DerivedPartArgs, MirrorPartArgs,
    program_tests::{build_file, editor, registry},
};

/// A steel bracket `length` long along `x`, 2 wide and 1 thick, from the
/// origin: not symmetric in its `yz` plane.
fn bracket(length: f64) -> Program {
    let mut program = Program::new();
    program.push(
        "properties",
        PartPropertiesArgs {
            material: Material::named("Steel"),
            ..PartPropertiesArgs::default()
        },
    );
    program.push(
        "bracket",
        PrimitiveArgs::block([0.0; 3], [length, 2.0, 1.0]),
    );
    program
}

/// The left-hand bracket: the bracket's bodies, mirrored in its `yz`
/// plane.
fn left_hand() -> Program {
    let mut program = Program::new();
    program.push(
        "d",
        DerivedPartArgs {
            file: "bracket.geop".into(),
            ..DerivedPartArgs::default()
        },
    );
    program.push(
        "m",
        MirrorPartArgs {
            normal: FrameAxis::X,
        },
    );
    program
}

/// An assembly of the bracket and — `mirrored` — its other hand, both
/// fixed where they are.
fn assembly(mirrored: bool) -> Program {
    let mut program = Program::new();
    for (id, mirror) in [("right", None), ("left", mirrored.then_some(FrameAxis::X))] {
        let placed = AddPartArgs {
            file: "bracket.geop".into(),
            mirror,
            ..AddPartArgs::default()
        };
        program.push(id, placed.fixed());
    }
    program
}

/// The files of the bracket `length` long, its left hand, and the
/// assembly of both.
fn files(length: f64, mirrored: bool) -> BTreeMap<String, String> {
    BTreeMap::from([
        ("bracket.geop".into(), bracket(length).to_json().unwrap()),
        ("left.geop".into(), left_hand().to_json().unwrap()),
        ("asm.geop".into(), assembly(mirrored).to_json().unwrap()),
    ])
}

/// The workspace of `files`, beside the standard parts.
fn workspace(files: BTreeMap<String, String>) -> Rc<dyn Workspace> {
    Files::new(WithStandardParts(files), Rc::new(registry().operations()))
}

/// How far `model` reaches along `x`: its least and greatest vertex.
fn reach_along_x(model: &NamedModel) -> [f64; 2] {
    let xs = (model.topology().vertices.values()).map(|v| v.point[0].to_f64());
    let (lo, hi) = xs.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), x| {
        (lo.min(x), hi.max(x))
    });
    [lo, hi]
}

/// The model of the left hand, of a bracket `length` long.
fn left_model(length: f64) -> NamedModel {
    let built = build_file(&left_hand(), "left.geop", workspace(files(length, false))).unwrap();
    read_model(&built.part.current()).unwrap().into_owned()
}

/// The left hand is the bracket mirrored: its solid, renamed as derived,
/// reaches from `-length` to 0 along `x`, and follows the bracket made
/// longer.
#[test]
fn a_derived_part_mirrored_is_the_other_hand_and_follows_its_file() {
    let left = left_model(4.0);
    assert!(left.id_of("derived(d,block(bracket))").is_some());
    let [lo, hi] = reach_along_x(&left);
    assert!((lo + 4.0).abs() < 1e-9 && hi.abs() < 1e-9, "{lo} {hi}");
    let [lo, _] = reach_along_x(&left_model(6.0));
    assert!((lo + 6.0).abs() < 1e-9, "{lo}");
}

/// A mirrored component is placed as the part mirrored — its own part's
/// solid on the other side of its `yz` plane — and listed apart from the
/// part as it is, the same mass each.
#[test]
fn a_mirrored_component_is_the_other_hand_and_listed_apart() {
    let built = build_file(&assembly(true), "asm.geop", workspace(files(4.0, true))).unwrap();
    let left = built.part.current().placed_part("left").unwrap();
    let [lo, hi] = reach_along_x(&read_model(&left).unwrap());
    assert!((lo + 4.0).abs() < 1e-9 && hi.abs() < 1e-9, "{lo} {hi}");
    let listed = bom(&built.part.current(), "asm.geop", Structure::Flat).unwrap();
    let lines: Vec<(String, u64, f64)> = (listed.lines.iter())
        .map(|line| match &line.kind {
            LineKind::Part {
                parameters,
                unit_mass,
                ..
            } => (
                parameters.clone(),
                line.quantity,
                unit_mass.as_ref().map_or(0.0, |m| m.value),
            ),
            LineKind::Wire { .. } => panic!("no wires"),
        })
        .collect();
    assert_eq!(lines.len(), 2, "{lines:?}");
    assert_eq!((lines[0].0.as_str(), lines[0].1), ("", 1));
    assert_eq!((lines[1].0.as_str(), lines[1].1), ("mirrored=yz", 1));
    assert!((lines[0].2 - lines[1].2).abs() < 1e-12, "{lines:?}");

    let unmirrored = build_file(&assembly(false), "asm.geop", workspace(files(4.0, false)));
    let listed = bom(
        &unmirrored.unwrap().part.current(),
        "asm.geop",
        Structure::Flat,
    );
    assert_eq!(
        listed.unwrap().lines.len(),
        1,
        "the same part twice is one line"
    );
}

/// The left hand and the assembly with a mirrored component are saved and
/// read back as they were, build as from scratch however they are edited,
/// and reopened make nothing they kept.
#[test]
fn derived_and_mirrored_parts_round_trip_reopen_and_build_as_from_scratch() {
    let operations = Rc::new(registry().operations());
    let files = files(4.0, true);
    for (file, program) in [("left.geop", left_hand()), ("asm.geop", assembly(true))] {
        assert_round_trips(
            operations.clone(),
            Some((workspace(files.clone()), file)),
            &program,
        );
        let [first, again] = assert_reopens(
            operations.clone(),
            |cache| Files::with_cache(WithStandardParts(files.clone()), operations.clone(), cache),
            file,
        );
        assert!(first.kept_makes > 0, "{file}");
        assert_eq!(again.kept_makes, 0, "{file}");
    }
    assert_incremental(
        operations,
        Some((workspace(files), "asm.geop")),
        &[assembly(true), assembly(false), assembly(true)],
    );
}

/// Placing a part offers to mirror it, in a plane of its own origin; the
/// one chosen is what the step places.
#[test]
fn a_part_is_mirrored_where_it_is_placed() {
    let mut editor = editor();
    let files = (files(4.0, false).into_iter())
        .map(|(p, t)| (p, Some(t)))
        .collect();
    handled(&mut editor, Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program: assembly(false),
            path: Some("asm.geop".into()),
        },
    );
    let update = handled(
        &mut editor,
        Command::Open {
            id: "left".into(),
            placed: None,
        },
    );
    let presentation = update.step.unwrap().presentation;
    let Some(Control::Select { options, value, .. }) = presentation.dialog.get("mirror") else {
        panic!("a choice of mirrors: {:?}", presentation.dialog)
    };
    assert_eq!(value, "");
    let planes: Vec<&str> = options.iter().map(|o| o.value.as_str()).collect();
    assert_eq!(planes, ["", "yz", "zx", "xy"]);
    handled(&mut editor, dialog("mirror", Value::Choice("yz".into())));
    handled(&mut editor, Command::Commit);
    let args: AddPartArgs = step_args(editor.program().step("left").unwrap());
    assert_eq!(args.mirror, Some(FrameAxis::X));
    let left = editor.part().placed_part("left").unwrap();
    let [lo, _] = reach_along_x(&read_model(&left).unwrap());
    assert!((lo + 4.0).abs() < 1e-9, "{lo}");
}

/// A derived part's file is chosen among the workspace's programs; the
/// part then holds that file's bodies.
#[test]
fn a_part_is_derived_from_a_file_chosen() {
    let mut editor = editor();
    let files = (files(4.0, false).into_iter())
        .map(|(p, t)| (p, Some(t)))
        .collect();
    handled(&mut editor, Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program: Program::new(),
            path: Some("new.geop".into()),
        },
    );
    handled(
        &mut editor,
        Command::New {
            kind: "derived_part".into(),
        },
    );
    handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Dialog {
                key: "file".into(),
                value: Value::Choice("bracket.geop".into()),
            },
        },
    );
    handled(&mut editor, Command::Commit);
    let model = read_model(&editor.part()).unwrap();
    let id = editor.program().steps[0].id.clone();
    assert!(
        model
            .id_of(&format!("derived({id},block(bracket))"))
            .is_some()
    );
}
