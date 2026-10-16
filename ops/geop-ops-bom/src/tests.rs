//! Bills of generated assemblies: repeated parts grouped, sub-assemblies
//! flattened or indented, masses multiplied, sheet thickness and wires.

use geop_core_math::{
    primitives::Pose,
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_program::{
    part::Part,
    program::Program,
    values::{Setting, Settings, StateValue, pose_state},
};
use geop_ops_parameters::{
    Material, Parameter, ParameterKind, Parameters, parameters::Row, set_parameters,
};

use geop_lib_sweep::shapes::block;
use geop_ops_assembly::testing::{Shelf, given_model, place, runner};
use geop_ops_model::{CutWire, NamedModel, SolidNotes};

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
