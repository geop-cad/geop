//! Bills of materials of whole programs: standard parts designated by
//! their norms and weighed of their own material, repeated parts grouped
//! across sub-assemblies, flat and indented.

use std::collections::BTreeMap;

use geop_core_math::scalars::ScalInF64 as S;
use geop_ops::{
    Part,
    part::{ParamValue, State, pose_parameter},
};
use geop_ops_assembly::AddPartArgs;
use geop_ops_bom::{Bom, LineKind, Structure};

use crate::examples::{self, pose};
use crate::inspect::bill_of_materials;
use crate::{Program, Workspace, stdlib::WithStandardParts};

/// `program`, saved as `path` among `files`, built.
fn build(files: BTreeMap<String, String>, path: &str, program: &Program) -> Part<S> {
    let workspace = Workspace::<S>::new(WithStandardParts(files));
    program.build(&workspace.scope(path)).unwrap()
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
    let program = files[0].1.clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    let part = build(files, "bolted_plate.geop", &program);
    let bom = bill_of_materials(&part, "bolted_plate.geop", Structure::Flat).unwrap();
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
        .map(|row| State::from([("size".to_string(), ParamValue::Text(row.into()))]))
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
        pose_parameter(id),
        ParamValue::Pose(pose([x, 0.0, 0.0], [0.0; 3])),
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

    let flat = bill_of_materials(&part, "robot.geop", Structure::Flat).unwrap();
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

    let indented = bill_of_materials(&part, "robot.geop", Structure::Indented).unwrap();
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
