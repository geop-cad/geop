//! Bills of generated assemblies: repeated parts grouped, sub-assemblies
//! flattened or indented, masses multiplied, sheet thickness and wires —
//! and of the example assemblies, whose standard parts are designated by
//! their norms and weighed of their own material; asked for and exported
//! through the editor, as the inspect panel and the File menu do.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::{
    editor::{Command, Editor},
    registry::Registry,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::Pose,
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_program::{
    part::Part,
    program::Program,
    runner::Runner,
    values::{Setting, Settings, StateValue, pose_state},
    workspace::{self, Files, Location},
};
use geop_ops_parameters::{
    Material, Parameter, ParameterKind, Parameters, parameters::Row, set_parameters,
};

use geop_lib_stdlib::WithStandardParts;
use geop_lib_sweep::shapes::block;
use geop_ops_assembly::{
    AddPartArgs,
    examples::{metric_plate, pose, workspaces},
    placed::placed_parts,
    testing::{Shelf, given_model, place, runner},
};
use geop_ops_model::{CutWire, NamedModel, SolidNotes, read_model};

use crate::{Bom, LineKind, Structure, bom};

/// Steel: 7850 kg/m³, so a cube of side 1 mm weighs 7.85e-6 kg.
const CUBE: f64 = 7.85e-6;

fn steel() -> Parameters {
    Parameters {
        material: Some(Material {
            name: "Steel".into(),
            density: 7850.0,
        }),
        ..Parameters::default()
    }
}

/// The program of a part: its parameters and properties, a step giving its
/// model and the notes on its solids, and one placing each of `placed` —
/// its name, its file and the settings it is built with — where its pose
/// state, the identity, puts it.
fn program(
    parameters: Parameters,
    model: NamedModel<S>,
    notes: Option<SolidNotes>,
    placed: &[(&str, &str, Settings)],
) -> Program {
    let mut file = Program::new();
    set_parameters(&mut file, parameters);
    file.steps.push(given_model("model", model, notes));
    for (name, placed, settings) in placed {
        file.steps.push(place(name, placed, settings));
        (file.state).insert(pose_state(name), StateValue::Pose(Pose::identity()));
    }
    file
}

/// The part `root`, the program of the file `file` on `shelf`, builds.
fn build(shelf: Shelf, file: &str, root: Program) -> Part {
    let runner = runner(shelf.with(file, root.clone()), file);
    let mut part = Part::default();
    (root.run(&runner, &mut part)).unwrap();
    part
}

/// The settings that build a screw of the size `size`.
fn size(size: &str) -> Settings {
    Settings::from([("size".into(), Setting::Text(size.into()))])
}

/// A "standard" screw, a steel cube of side 1, of the sizes M4 and M5:
/// called a test screw, and ordered as `TEST M4`.
fn screw() -> Program {
    let mut parameters = steel();
    parameters.title = Some("Test screw".into());
    parameters.designation = Some("TEST".into());
    parameters.values.push(Parameter {
        name: "size".into(),
        kind: ParameterKind::Table {
            columns: vec!["d".into()],
            rows: vec![
                Row {
                    name: "M4".into(),
                    values: vec![4.0],
                },
                Row {
                    name: "M5".into(),
                    values: vec![5.0],
                },
            ],
            selected: "M4".into(),
        },
    });
    let mut model = NamedModel::new();
    block(&mut model, "body", [0.0; 3], [1.0; 3]);
    program(parameters, model, None, &[])
}

/// A bracket of side 2, with two M4 screws in it.
fn bracket() -> Program {
    let mut model = NamedModel::new();
    block(&mut model, "body", [0.0; 3], [2.0; 3]);
    let m4 = size("M4");
    let placed = [
        ("s1", "std:screw.geop", m4.clone()),
        ("s2", "std:screw.geop", m4),
    ];
    program(steel(), model, None, &placed)
}

/// Two brackets, three more M4 screws — one of them placed with its size
/// given apart — and one M5.
fn assembly() -> Part {
    let (m4, screw_file) = (size("M4"), "std:screw.geop");
    let placed = [
        ("b1", "bracket.geop", Settings::new()),
        ("b2", "bracket.geop", Settings::new()),
        ("s1", screw_file, m4.clone()),
        ("s2", screw_file, m4.clone()),
        ("s3", screw_file, size("M4")),
        ("s4", screw_file, size("M5")),
    ];
    let top = program(Parameters::default(), NamedModel::new(), None, &placed);
    let shelf = (Shelf::default().with(screw_file, screw())).with("bracket.geop", bracket());
    build(shelf, "top.geop", top)
}

/// The unit and total mass of a part line.
#[track_caller]
fn masses(bom: &Bom, item: &str) -> (f64, f64) {
    let line = bom.line(item).unwrap();
    let LineKind::Part {
        unit_mass,
        total_mass,
        ..
    } = &line.kind
    else {
        panic!("{line:?} is no part");
    };
    (unit_mass.unwrap().value, total_mass.unwrap().value)
}

#[track_caller]
fn assert_close(got: f64, want: f64) {
    assert!(
        (got - want).abs() <= 1e-9 * want.abs(),
        "{got} is not {want}"
    );
}

/// Flat: every M4 screw — in the brackets and beside them, of whichever
/// build — on one line, seven of them; the M5 on its own; the bracket once
/// for its own body, weighed without its screws.
#[test]
fn a_flat_bill_groups_repeated_parts() {
    let bom = bom(&assembly().current(), "top.geop", Structure::Flat).unwrap();
    let rows: Vec<(&str, u64, Option<&str>, usize)> = bom
        .lines
        .iter()
        .map(|l| {
            (
                l.item.as_str(),
                l.quantity,
                l.designation.as_deref(),
                l.level,
            )
        })
        .collect();
    assert_eq!(
        rows,
        [
            ("1", 2, None, 0),
            ("2", 7, Some("TEST M4"), 0),
            ("3", 1, Some("TEST M5"), 0),
        ],
        "{bom:#?}"
    );
    assert_eq!(bom.line("1").unwrap().name, "bracket");
    assert_eq!(bom.line("2").unwrap().name, "Test screw");
    // Where each is placed, from the assembly: what a drawing balloons.
    assert_eq!(bom.line("1").unwrap().placements, ["b1", "b2"]);
    assert_eq!(
        bom.line("2").unwrap().placements,
        ["b1/s1", "b1/s2", "b2/s1", "b2/s2", "s1", "s2", "s3"]
    );
    assert_eq!(bom.line("3").unwrap().placements, ["s4"]);
    let LineKind::Part {
        parameters,
        material,
        assumed,
        ..
    } = &bom.line("2").unwrap().kind
    else {
        panic!()
    };
    assert_eq!(parameters, "size=M4");
    assert_eq!((material.as_str(), *assumed), ("Steel", false));

    let (unit, total) = masses(&bom, "1");
    assert_close(unit, 8.0 * CUBE);
    assert_close(total, 16.0 * CUBE);
    let (unit, total) = masses(&bom, "2");
    assert_close(unit, CUBE);
    assert_close(total, 7.0 * CUBE);
    assert_close(bom.total_mass.unwrap().value, (16.0 + 8.0) * CUBE);
}

/// Indented: the brackets with their two screws each under them, the
/// screws placed beside them on lines of their own; a bracket weighs its
/// body and its screws.
#[test]
fn an_indented_bill_follows_the_sub_assemblies() {
    let bom = bom(&assembly().current(), "top.geop", Structure::Indented).unwrap();
    let rows: Vec<(&str, usize, u64, &str)> = bom
        .lines
        .iter()
        .map(|l| (l.item.as_str(), l.level, l.quantity, l.name.as_str()))
        .collect();
    assert_eq!(
        rows,
        [
            ("1", 1, 2, "bracket"),
            ("1.1", 2, 2, "Test screw"),
            ("2", 1, 3, "Test screw"),
            ("3", 1, 1, "Test screw"),
        ],
        "{bom:#?}"
    );
    let (unit, total) = masses(&bom, "1");
    assert_close(unit, 10.0 * CUBE);
    assert_close(total, 20.0 * CUBE);
    assert_close(bom.total_mass.unwrap().value, (20.0 + 4.0) * CUBE);
}

/// A part that places nothing is its own bill: one line, named after its
/// file, of no given material — weighed as water, and said so — with the
/// thickness recorded on its sheet-metal body.
#[test]
fn a_plain_part_is_one_line_with_its_sheet_thickness() {
    let mut model = NamedModel::new();
    block(&mut model, "sheet", [0.0; 3], [1.0; 3]);
    let mut noted = SolidNotes::default();
    noted.note_mut("cube(sheet)").thickness = Some(1.5);
    let cover = program(Parameters::default(), model, Some(noted), &[]);
    let file = "parts/cover.geop";
    let part = build(Shelf::default(), file, cover);
    for structure in [Structure::Flat, Structure::Indented] {
        let bom = bom(&part.current(), file, structure).unwrap();
        assert_eq!(bom.lines.len(), 1, "{bom:#?}");
        let line = &bom.lines[0];
        assert_eq!((line.item.as_str(), line.name.as_str()), ("1", "cover"));
        let LineKind::Part {
            thickness, assumed, ..
        } = &line.kind
        else {
            panic!()
        };
        assert_eq!(thickness, &[1.5]);
        assert!(assumed);
        assert_close(masses(&bom, "1").0, 1e-6);
    }
}

/// A harness routed in a sub-assembly: each wire a line, as many as the
/// sub-assembly is placed — its bundle neither a body nor weighed — and
/// the CSV with a row each and the total.
#[test]
fn wires_are_lines_of_their_own() {
    let mut model = NamedModel::new();
    block(&mut model, "cable", [0.0; 3], [1.0; 3]);
    let wire = |name: &str, gauge| CutWire {
        name: name.into(),
        colour: "#ff0000".into(),
        diameter: 1.2,
        gauge,
        cut_length: S::from_f64(250.0),
    };
    let mut noted = SolidNotes::default();
    let bundle = noted.note_mut("cube(cable)");
    bundle.drawn_only = true;
    bundle.wires = vec![wire("power", Some(22)), wire("signal, shielded", None)];
    let harness = program(Parameters::default(), model, Some(noted), &[]);
    let placed = [
        ("h1", "harness.geop", Settings::new()),
        ("h2", "harness.geop", Settings::new()),
    ];
    let top = program(Parameters::default(), NamedModel::new(), None, &placed);
    let top = build(
        Shelf::default().with("harness.geop", harness),
        "top.geop",
        top,
    );

    let flat = bom(&top.current(), "top.geop", Structure::Flat).unwrap();
    let rows: Vec<(&str, u64, Option<&str>)> = flat
        .lines
        .iter()
        .map(|l| (l.name.as_str(), l.quantity, l.designation.as_deref()))
        .collect();
    // The harness part has no bodies but its bundle, and places nothing:
    // it is still listed, weighing nothing.
    assert_eq!(
        rows,
        [
            ("harness", 2, None),
            ("power", 2, Some("AWG 22")),
            ("signal, shielded", 2, Some("Ø1.2 mm")),
        ],
        "{flat:#?}"
    );
    assert_close(masses(&flat, "1").1 + 1.0, 1.0);
    let LineKind::Wire { total_length, .. } = &flat.line("2").unwrap().kind else {
        panic!()
    };
    assert_close(total_length.value, 500.0);

    let csv = flat.to_csv();
    let lines: Vec<&str> = csv.lines().collect();
    assert_eq!(lines.len(), 1 + 3 + 1, "{csv}");
    assert!(lines[0].starts_with("Item,Level,Quantity,Name,Designation"));
    assert!(
        lines[3].contains("\"signal, shielded\"") && lines[3].ends_with(",250.0,500.0"),
        "{csv}"
    );
    assert!(lines[4].contains("Total"), "{csv}");

    let indented = bom(&top.current(), "top.geop", Structure::Indented).unwrap();
    let items: Vec<(&str, usize)> = indented
        .lines
        .iter()
        .map(|l| (l.item.as_str(), l.level))
        .collect();
    assert_eq!(items, [("1", 1), ("1.1", 2), ("1.2", 2)], "{indented:#?}");
    assert_eq!(indented.line("1.1").unwrap().quantity, 1);
}

// The example assemblies, built beside the standard parts.

/// The operations the example assemblies are written in — sketched,
/// extruded, placed, the standard parts' patterns and parameters — each
/// crate's `register`, as the editor's registry has them, and the bill of
/// materials.
fn registry() -> Registry {
    let registry = (|| -> GeopResult<Registry> {
        let r = Registry::default();
        let r = geop_ops_model::register(r)?;
        let r = geop_ops_sketch::register(r)?;
        let r = geop_ops_datums::register(r)?;
        let r = geop_ops_reference_geometry::register(r)?;
        let r = geop_ops_extrude_revolve::register(r)?;
        let r = geop_ops_pattern::register(r)?;
        let r = geop_ops_assembly::register(r)?;
        let r = crate::register(r)?;
        geop_ops_parameters::register(r)
    })();
    registry.expect("the operations each have a name of their own")
}

/// `program`, saved as `path` among `files` beside the standard parts,
/// built.
fn build_files(files: BTreeMap<String, String>, path: &str, program: &Program) -> Part {
    let operations = registry().operations();
    let mut runner = Runner::new(operations.clone());
    runner.workspace = Some(Files::new(WithStandardParts(files), Rc::new(operations)));
    runner.location = Location {
        file: workspace::resolve("", path),
        within: Vec::new(),
    };
    let mut part = Part::default();
    program.run(&runner, &mut part).unwrap();
    part
}

/// The bill of materials of `part`, the program of the file `file`, laid
/// out as `structure` says.
fn bill_of_materials(part: &Part, file: &str, structure: Structure) -> Bom {
    bom(&part.current(), file, structure).unwrap()
}

/// `(item, level, quantity, name, designation)` of every line.
fn rows(bom: &Bom) -> Vec<(&str, usize, u64, &str, Option<&str>)> {
    bom.lines
        .iter()
        .map(|l| {
            (
                l.item.as_str(),
                l.level,
                l.quantity,
                l.name.as_str(),
                l.designation.as_deref(),
            )
        })
        .collect()
}

/// The material, and the unit and total mass (kg), of a part line.
#[track_caller]
fn weighed(bom: &Bom, item: &str) -> (String, bool, f64, f64) {
    let line = bom.line(item).unwrap();
    let LineKind::Part {
        material,
        assumed,
        unit_mass,
        total_mass,
        error,
        ..
    } = &line.kind
    else {
        panic!("{line:?} is no part");
    };
    assert!(error.is_none(), "{line:?}");
    (
        material.clone(),
        *assumed,
        unit_mass.unwrap().value,
        total_mass.unwrap().value,
    )
}

/// The bolted plate: one plate — of no given material, weighed as water —
/// one ISO 4762 M4x12 screw and one ISO 4032 M4 nut, both steel, weighing
/// what steel of their volume does.
#[test]
fn the_bolted_plate_lists_its_screw_and_nut() {
    let (_, files) = workspaces()
        .into_iter()
        .find(|(name, _)| *name == "bolted_plate")
        .unwrap();
    let files = files();
    let program = files[0].1.clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    let part = build_files(files, "bolted_plate.geop", &program);
    let bom = bill_of_materials(&part, "bolted_plate.geop", Structure::Flat);
    assert_eq!(
        rows(&bom),
        [
            ("1", 0, 1, "plate", None),
            (
                "2",
                0,
                1,
                "ISO 4762 socket head cap screw",
                Some("ISO 4762 M4x12")
            ),
            ("3", 0, 1, "ISO 4032 hex nut", Some("ISO 4032 M4")),
        ],
        "{bom:#?}"
    );
    // The plate: 40 x 40 x 5 with a hole 4.5 across, of water.
    let (_, assumed, plate, _) = weighed(&bom, "1");
    assert!(assumed);
    let volume = 40.0 * 40.0 * 5.0 - std::f64::consts::PI * 2.25 * 2.25 * 5.0;
    assert!((plate - volume * 1e-6).abs() < 1e-6 * plate, "{plate}");
    // An M4x12 cap screw of steel weighs a couple of grams, its nut one.
    let (material, assumed, screw, _) = weighed(&bom, "2");
    assert_eq!((material.as_str(), assumed), ("Steel", false));
    assert!((1e-3..3e-3).contains(&screw), "{screw} kg");
    let (_, _, nut, _) = weighed(&bom, "3");
    assert!((0.5e-3..1.5e-3).contains(&nut), "{nut} kg");
    let total = bom.total_mass.unwrap().value;
    assert!((total - (plate + screw + nut)).abs() < 1e-9, "{total}");
}

/// Places `file`, sized `size`, as `id`, at `x` — free: it has no mates.
fn placed(program: &mut Program, id: &str, file: &str, size: Option<&str>, x: f64) {
    let parameters = size
        .map(|row| Settings::from([("size".to_string(), Setting::Text(row.into()))]))
        .unwrap_or_default();
    program.push(
        id,
        AddPartArgs {
            file: file.into(),
            parameters,
            ..Default::default()
        },
    );
    program.state.insert(
        pose_state(id),
        StateValue::Pose(pose([x, 0.0, 0.0], [0.0; 3])),
    );
}

const SCREW: &str = "std:iso4762_socket_head_cap_screw.geop";
const NUT: &str = "std:iso4032_hex_nut.geop";

/// A robot of three modules — each a plate with four M3x10 screws and four
/// M3 nuts — two more M3x10 screws, and one M3x12: the screws of either
/// size grouped across the modules in the flat bill, and under each
/// module in the indented one.
#[test]
fn repeated_standard_parts_are_grouped() {
    let mut module = Program::new();
    placed(&mut module, "plate", "plate.geop", None, 0.0);
    for i in 0..4 {
        placed(
            &mut module,
            &format!("screw{i}"),
            SCREW,
            Some("M3x10"),
            5.0 * i as f64,
        );
        placed(
            &mut module,
            &format!("nut{i}"),
            NUT,
            Some("M3"),
            5.0 * i as f64,
        );
    }
    let mut robot = Program::new();
    for i in 0..3 {
        placed(
            &mut robot,
            &format!("module{i}"),
            "module.geop",
            None,
            50.0 * i as f64,
        );
    }
    placed(&mut robot, "extra0", SCREW, Some("M3x10"), -10.0);
    placed(&mut robot, "extra1", SCREW, Some("M3x10"), -20.0);
    placed(&mut robot, "long", SCREW, Some("M3x12"), -30.0);
    let files = BTreeMap::from([
        ("module.geop".to_string(), module.to_json().unwrap()),
        ("plate.geop".to_string(), metric_plate().to_json().unwrap()),
    ]);
    let part = build_files(files, "robot.geop", &robot);

    let flat = bill_of_materials(&part, "robot.geop", Structure::Flat);
    let screw = "ISO 4762 socket head cap screw";
    let nut = "ISO 4032 hex nut";
    assert_eq!(
        rows(&flat),
        [
            ("1", 0, 3, "plate", None),
            ("2", 0, 14, screw, Some("ISO 4762 M3x10")),
            ("3", 0, 12, nut, Some("ISO 4032 M3")),
            ("4", 0, 1, screw, Some("ISO 4762 M3x12")),
        ],
        "{flat:#?}"
    );
    let (_, _, unit, total) = weighed(&flat, "2");
    assert!(
        (total - 14.0 * unit).abs() < 1e-12,
        "{unit} x 14 is not {total}"
    );

    let indented = bill_of_materials(&part, "robot.geop", Structure::Indented);
    assert_eq!(
        rows(&indented),
        [
            ("1", 1, 3, "module", None),
            ("1.1", 2, 1, "plate", None),
            ("1.2", 2, 4, screw, Some("ISO 4762 M3x10")),
            ("1.3", 2, 4, nut, Some("ISO 4032 M3")),
            ("2", 1, 2, screw, Some("ISO 4762 M3x10")),
            ("3", 1, 1, screw, Some("ISO 4762 M3x12")),
        ],
        "{indented:#?}"
    );
    // A module weighs its plate, screws and nuts; the whole weighs the
    // same either way it is listed.
    let (_, _, module_mass, _) = weighed(&indented, "1");
    let parts: f64 = ["1.1", "1.2", "1.3"]
        .iter()
        .map(|item| weighed(&indented, item).3)
        .sum();
    assert!(
        (module_mass - parts).abs() < 1e-12,
        "{module_mass} vs {parts}"
    );
    let (flat_total, indented_total) = (
        flat.total_mass.unwrap().value,
        indented.total_mass.unwrap().value,
    );
    assert!((flat_total - indented_total).abs() < 1e-12);
}

/// Weighing the bolted plate's plate, screw and nut takes a bounded number
/// of surface points — counted, not timed (see AGENTS.md). Their faces are
/// planes and cylinders, trimmed by curves the booleans fitted through 48
/// points each: the inner integral runs along a direction in which every
/// one of these surfaces is a polynomial, and is exact in three points.
/// Along the circle direction it took fifteen and more, and the three
/// bodies 745 000 points.
#[test]
fn the_bolted_plate_is_weighed_in_bounded_work() {
    let (_, files) = workspaces()
        .into_iter()
        .find(|(name, _)| *name == "bolted_plate")
        .unwrap();
    let files = files();
    let program = files[0].1.clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    let part = build_files(files, "bolted_plate.geop", &program);
    let reader = part.current();
    let placed = placed_parts(&reader).unwrap();
    let mut parts = vec![reader.clone()];
    parts.extend(placed.iter().map(|p| p.located.part.clone()));
    let mut evaluations = Vec::new();
    for p in parts {
        // An assembly that only places parts has an empty model of its own.
        let p = read_model(&p).unwrap();
        for name in p.solid_names() {
            let mass = p
                .topology()
                .mass_properties(p.solid_id(&name).unwrap())
                .unwrap();
            assert!(mass.converged, "{name}: {mass:?}");
            evaluations.push((name, mass.evaluations));
        }
    }
    let total: usize = evaluations.iter().map(|(_, n)| n).sum();
    assert!(total < 170_000, "{evaluations:?}");
}

/// The bill of materials, asked for the way the inspect panel asks — the
/// command and the update as JSON: the bolted plate's three parts, the
/// screw and nut designated by their norms. Of the part shown: seeking
/// back before the nut drops it. Exported as CSV, indented, named after
/// the file.
#[test]
fn the_bill_of_materials_is_asked_for_and_exported() {
    let mut editor = Editor::new(registry(), WithStandardParts(BTreeMap::new()));
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "bolted_plate".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let ask = r#"{"command": "inspect", "inspector": "bom", "options": {"structure": "flat"}}"#;
    let json = editor.handle_json(ask).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["inspection"]["kind"], "bom");
    let bom = &update["inspection"]["report"];
    assert_eq!(bom["structure"], "flat");
    let designations: Vec<&str> = (bom["lines"].as_array().unwrap().iter())
        .map(|l| l["designation"].as_str().unwrap_or("-"))
        .collect();
    assert_eq!(designations, ["-", "ISO 4762 M4x12", "ISO 4032 M4"]);
    let screw = &bom["lines"][1];
    assert_eq!(
        (&screw["kind"], &screw["quantity"]),
        (&"part".into(), &1.into())
    );
    assert_eq!(screw["material"], "Steel");
    assert!(screw["unit_mass"]["value"].as_f64().unwrap() > 0.0);
    assert!(bom["total_mass"]["value"].as_f64().unwrap() > 0.0);

    // Up to the screw: no nut yet.
    editor.handle(Command::Seek { marker: Some(2) });
    let json = editor.handle_json(ask).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    let lines = update["inspection"]["report"]["lines"].as_array().unwrap();
    assert_eq!(lines.len(), 2);
    editor.handle(Command::Seek { marker: None });

    let json = editor
        .handle_json(
            r#"{"command": "export", "exporter": "bom", "options": {"structure": "indented"}}"#,
        )
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["export"]["name"], "bolted_plate_bom.csv");
    let csv = update["export"]["text"].as_str().unwrap();
    let rows: Vec<&str> = csv.lines().collect();
    assert_eq!(rows.len(), 1 + 3 + 1, "{csv}");
    assert!(
        rows[2].starts_with("2,1,1,ISO 4762 socket head cap screw,ISO 4762 M4x12,"),
        "{csv}"
    );
    assert!(rows[4].contains("Total"), "{csv}");
}
