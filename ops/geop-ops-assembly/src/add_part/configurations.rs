//! A part of configurations placed in an assembly: each instance built in
//! the configuration its step chooses, listed by it in the bill of
//! materials — and the assembly built incrementally, saved, and reopened
//! from what an earlier process kept as it would be built from scratch.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::{
    editing::{Control, StepEditEvent, Value},
    editor::Command,
    testing::handled,
};
use geop_core_program::{
    program::Program,
    testing::{args as step_args, assert_incremental, assert_reopens, assert_round_trips},
    values::{Setting, Settings},
    workspace::{Files, Workspace},
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_lib_stdlib::WithStandardParts;
use geop_ops_model::{
    parameters::{CONFIGURATION, LookupTableArgs, Material, PartPropertiesArgs, Row},
    placed::bom::{LineKind, Structure, bom},
};

use crate::{
    AddPartArgs,
    program_tests::{build_file, editor, registry},
};

/// A steel bracket of two configurations: `full`, a plate with a boss on
/// it, and `plain`, the plate alone — the boss's step suppressed there.
fn bracket() -> Program {
    let mut program = Program::new();
    program.push(
        "configurations",
        LookupTableArgs {
            input: CONFIGURATION.into(),
            columns: vec!["boss".into()],
            rows: vec![
                Row {
                    name: "full".into(),
                    values: vec![1.0],
                },
                Row {
                    name: "plain".into(),
                    values: vec![0.0],
                },
            ],
            selected: "full".into(),
        },
    );
    program.push(
        "properties",
        PartPropertiesArgs {
            material: Material::named("Steel"),
            ..PartPropertiesArgs::default()
        },
    );
    program.push("plate", PrimitiveArgs::block([0.0; 3], [10.0, 10.0, 1.0]));
    program.push(
        "boss",
        PrimitiveArgs::block([4.0, 4.0, 1.0], [6.0, 6.0, 3.0]).join("block(plate)"),
    );
    program.steps[3].active = Some("configuration.boss".into());
    program
}

/// An assembly of the bracket placed twice, as `a` in `first` and as `b`
/// in `second` — a configuration by name, or the bracket's own.
fn assembly(first: Option<&str>, second: Option<&str>) -> Program {
    let configured = |row: Option<&str>| -> Settings {
        row.map(|row| (CONFIGURATION.to_string(), Setting::Text(row.into())))
            .into_iter()
            .collect()
    };
    let mut program = Program::new();
    for (id, row) in [("a", first), ("b", second)] {
        let placed = AddPartArgs {
            file: "bracket.geop".into(),
            parameters: configured(row),
            ..AddPartArgs::default()
        };
        program.push(id, placed.fixed());
    }
    program
}

/// The files of the assembly `assembly.geop` and the bracket.
fn files(assembly: &Program) -> BTreeMap<String, String> {
    BTreeMap::from([
        ("assembly.geop".into(), assembly.to_json().unwrap()),
        ("bracket.geop".into(), bracket().to_json().unwrap()),
    ])
}

/// The workspace of `files`, beside the standard parts.
fn workspace(files: BTreeMap<String, String>) -> Rc<dyn Workspace> {
    Files::new(WithStandardParts(files), Rc::new(registry().operations()))
}

/// The bill of materials of `assembly`: each line's parameters, quantity
/// and mass of one, in kg.
fn bill(assembly: &Program) -> Vec<(String, u64, f64)> {
    let built = build_file(assembly, "assembly.geop", workspace(files(assembly))).unwrap();
    let bom = bom(&built.part.current(), "assembly.geop", Structure::Flat).unwrap();
    (bom.lines.iter())
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
        .collect()
}

/// Each instance is built in the configuration its step chooses, or the
/// bracket's own: two configurations are two lines of the bill of
/// materials, by name, the plain one the lighter; one configuration twice
/// is one line of two.
#[test]
fn each_instance_is_built_in_the_configuration_it_is_placed_in() {
    let lines = bill(&assembly(None, Some("plain")));
    assert_eq!(lines.len(), 2, "{lines:?}");
    let (full, plain) = (&lines[0], &lines[1]);
    assert_eq!((full.0.as_str(), full.1), ("configuration=full", 1));
    assert_eq!((plain.0.as_str(), plain.1), ("configuration=plain", 1));
    // Steel, 7850 kg/m³: 100 mm³ and the boss's 8 mm³ more.
    assert!((plain.2 - 100.0 * 7850.0e-9).abs() < 1e-9, "{plain:?}");
    assert!((full.2 - 108.0 * 7850.0e-9).abs() < 1e-9, "{full:?}");
    let lines = bill(&assembly(Some("plain"), Some("plain")));
    assert_eq!(lines.len(), 1);
    assert_eq!(
        (lines[0].0.as_str(), lines[0].1),
        ("configuration=plain", 2)
    );
}

/// Placing the bracket offers its configurations to choose from, as a
/// table's rows; the one chosen is what the step places, and the bill
/// lists.
#[test]
fn a_configuration_is_chosen_where_the_part_is_placed() {
    let mut editor = editor();
    let assembly = assembly(None, None);
    let files = files(&assembly)
        .into_iter()
        .map(|(p, t)| (p, Some(t)))
        .collect();
    handled(&mut editor, Command::Files { files });
    handled(
        &mut editor,
        Command::Load {
            program: assembly,
            path: Some("assembly.geop".into()),
        },
    );
    let update = handled(
        &mut editor,
        Command::Open {
            id: "b".into(),
            placed: None,
        },
    );
    let dialog = update.step.unwrap().presentation.dialog;
    let Some(Control::Select { options, value, .. }) =
        dialog.get(&format!("parameter:{CONFIGURATION}"))
    else {
        panic!("the configuration from its table: {dialog:?}")
    };
    assert_eq!(value, "full");
    let rows: Vec<&str> = options.iter().map(|o| o.value.as_str()).collect();
    assert_eq!(rows, ["full", "plain"]);
    handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Dialog {
                key: format!("parameter:{CONFIGURATION}"),
                value: Value::Choice("plain".into()),
            },
        },
    );
    handled(&mut editor, Command::Commit);
    let args: AddPartArgs = step_args(editor.program().step("b").unwrap());
    assert_eq!(
        args.parameters[CONFIGURATION],
        Setting::Text("plain".into())
    );
    assert_eq!(bill(editor.program()).len(), 2);
}

/// The assembly of configurations is saved and read back as it was;
/// switching an instance's configuration builds as from scratch; and
/// reopened, it makes nothing it kept, each configuration of the bracket
/// read back as it was made.
#[test]
fn an_assembly_of_configurations_round_trips_reopens_and_builds_as_from_scratch() {
    let operations = Rc::new(registry().operations());
    let program = assembly(None, Some("plain"));
    let files = files(&program);
    let workspace = workspace(files.clone());
    assert_round_trips(
        operations.clone(),
        Some((workspace.clone(), "assembly.geop")),
        &program,
    );
    let programs = [
        program.clone(),
        assembly(Some("plain"), Some("plain")),
        assembly(Some("plain"), Some("full")),
        program.clone(),
    ];
    assert_incremental(
        operations.clone(),
        Some((workspace, "assembly.geop")),
        &programs,
    );
    let operations_kept = operations.clone();
    let [first, again] = assert_reopens(
        operations_kept,
        |cache| Files::with_cache(WithStandardParts(files.clone()), operations.clone(), cache),
        "assembly.geop",
    );
    assert!(first.kept_makes > 0);
    assert_eq!(again.kept_makes, 0);
}
