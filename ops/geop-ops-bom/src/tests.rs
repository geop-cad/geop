//! Bills of generated assemblies: repeated parts grouped, sub-assemblies
//! flattened or indented, masses multiplied, sheet thickness and wires.

use std::{collections::BTreeSet, sync::Arc};

use geop_core_math::{
    primitives::Pose,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    Component, Instance, Part,
    parameters::{Material, Parameter, ParameterKind, Parameters, Row},
    part::{Cable, CutWire, ParamValue, State},
};
use geop_ops_extrude_revolve::shapes::cube::cube_solid;
use geop_ops_sheetmetal::FlatPatternData;

use crate::{Bom, LineKind, Standard, Structure, bom};

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

/// A cube of side `side` named `name` in `part`, its corner at the origin.
fn cube(part: &mut Part<S>, name: &str, side: f64) {
    let v = |x: f64| Vector3::from_array([x; 3].map(S::from_f64));
    cube_solid(part, name, v(0.0), v(side)).unwrap();
}

/// A "standard" screw, a steel cube of side 1, built at the size `size`.
fn screw(size: &str) -> Arc<Component<S>> {
    let mut parameters = steel();
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
    let inputs = parameters
        .resolve(&State::from([(
            "size".into(),
            ParamValue::Text(size.into()),
        )]))
        .values;
    let mut part = Part::new().with_parameters(parameters).with_state(inputs);
    cube(&mut part, "body", 1.0);
    component("std:screw.geop", part)
}

fn component(file: &str, part: Part<S>) -> Arc<Component<S>> {
    Arc::new(Component::new(file.into(), part, BTreeSet::new()))
}

/// Places `component` in `part` under `name`, somewhere: where it is is no
/// part of what it is.
fn place(part: &mut Part<S>, component: &Arc<Component<S>>, name: &str, x: f64) {
    let pose = Pose::identity().with_position(Vector3::from_array([x, 0.0, 0.0].map(S::from_f64)));
    let instance = Instance {
        component: component.clone(),
        pose,
        parameter: None,
        fixed: false,
        flexible: false,
    };
    part.add_instance(instance, name).unwrap();
}

/// A bracket of side 2, with two M4 screws in it.
fn bracket() -> Arc<Component<S>> {
    let mut part = Part::new().with_parameters(steel());
    cube(&mut part, "body", 2.0);
    let m4 = screw("M4");
    place(&mut part, &m4, "s1", 0.0);
    place(&mut part, &m4, "s2", 1.0);
    component("bracket.geop", part)
}

/// Two brackets, three more M4 screws — one of them a separate build of
/// the same file and size — and one M5.
fn assembly() -> Part<S> {
    let mut part = Part::new();
    let bracket = bracket();
    place(&mut part, &bracket, "b1", 0.0);
    place(&mut part, &bracket, "b2", 10.0);
    let m4 = screw("M4");
    place(&mut part, &m4, "s1", 20.0);
    place(&mut part, &m4, "s2", 21.0);
    place(&mut part, &screw("M4"), "s3", 22.0);
    place(&mut part, &screw("M5"), "s4", 23.0);
    part
}

fn standard(file: &str, values: &State) -> Option<Standard> {
    let Some(ParamValue::Text(size)) = values.get("size") else {
        return None;
    };
    (file == "std:screw.geop").then(|| Standard {
        title: "Test screw".into(),
        designation: format!("TEST {size}"),
    })
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
    let bom = bom(&assembly(), "top.geop", Structure::Flat, &standard).unwrap();
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
    let bom = bom(&assembly(), "top.geop", Structure::Indented, &standard).unwrap();
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
    let mut part = Part::<S>::new();
    cube(&mut part, "sheet", 1.0);
    part.set_body_data(
        "cube(sheet)",
        FlatPatternData {
            thickness: 1.5,
            bends: Vec::new(),
        },
    )
    .unwrap();
    for structure in [Structure::Flat, Structure::Indented] {
        let bom = bom(&part, "parts/cover.geop", structure, &standard).unwrap();
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
    let mut harness = Part::<S>::new();
    cube(&mut harness, "cable", 1.0);
    let wire = |name: &str, gauge| CutWire {
        name: name.into(),
        colour: "#ff0000".into(),
        diameter: 1.2,
        gauge,
        cut_length: S::from_f64(250.0),
    };
    harness
        .add_cable(
            "cube(cable)",
            Cable {
                length: S::from_f64(230.0),
                diameter: 3.0,
                min_bend_radius: 6.0,
                tightest_bend: None,
                wires: vec![wire("power", Some(22)), wire("signal, shielded", None)],
            },
        )
        .unwrap();
    let harness = component("harness.geop", harness);
    let mut top = Part::<S>::new();
    place(&mut top, &harness, "h1", 0.0);
    place(&mut top, &harness, "h2", 5.0);

    let flat = bom(&top, "top.geop", Structure::Flat, &standard).unwrap();
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

    let indented = bom(&top, "top.geop", Structure::Indented, &standard).unwrap();
    let items: Vec<(&str, usize)> = indented
        .lines
        .iter()
        .map(|l| (l.item.as_str(), l.level))
        .collect();
    assert_eq!(items, [("1", 1), ("1.1", 2), ("1.2", 2)], "{indented:#?}");
    assert_eq!(indented.line("1.1").unwrap().quantity, 1);
}
