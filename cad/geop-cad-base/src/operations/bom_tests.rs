//! Bills of materials of whole programs: standard parts designated by
//! their norms and weighed of their own material, repeated parts grouped
//! across sub-assemblies, flat and indented.

use geop_core_program::{
    part::Part,
    program::Program,
    values::{Setting, Settings, StateValue, pose_state},
};
use std::collections::BTreeMap;

use geop_ops_assembly::{AddPartArgs, placed::placed_parts};
use geop_ops_bom::{Bom, LineKind, Structure, bom};
use geop_ops_model::read_model;

use super::workspace;
use crate::examples::{self, pose};

/// `program`, saved as `path` among `files`, built.
fn build(files: BTreeMap<String, String>, path: &str, program: &Program) -> Part {
    super::build(program, path, workspace(files)).unwrap().part
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
    let (_, files) = examples::workspaces()
        .into_iter()
        .find(|(name, _)| *name == "bolted_plate")
        .unwrap();
    let files = files();
    let program = files[0].1.clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    let part = build(files, "bolted_plate.geop", &program);
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
        (
            "plate.geop".to_string(),
            examples::metric_plate().to_json().unwrap(),
        ),
    ]);
    let part = build(files, "robot.geop", &robot);

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
    let (_, files) = examples::workspaces()
        .into_iter()
        .find(|(name, _)| *name == "bolted_plate")
        .unwrap();
    let files = files();
    let program = files[0].1.clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    let part = build(files, "bolted_plate.geop", &program);
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
