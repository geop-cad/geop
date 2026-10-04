//! The standard parts: every family builds one valid solid of the size
//! its table says, with the datums it is mated by.

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::validation::{ValidationParameters, validate_fast};
use geop_ops::{
    NoFiles, Part,
    parameters::{ParameterKind, number},
    part::ParamValue,
};

use super::{StandardPart, parts, steps::SIZE};
use crate::{Program, operations::regression_tests::check_valid};

/// How far out from the `z` axis, and from where to where along it, the
/// part of `file` reaches, by the norm: `value` reads a column of the size
/// built, or a parameter.
fn expected(file: &str, value: impl Fn(&str) -> f64) -> [f64; 3] {
    let corner = |s: f64| s / 3f64.sqrt();
    match file {
        "std:iso4762_socket_head_cap_screw.geop" => [value("dk") / 2.0, -value("l"), value("k")],
        "std:iso7380_button_head_screw.geop" => {
            // The dome's top is cut away by the socket, whose rim is
            // highest at the middle of the flats facing `±x` — where the
            // dome's quarter faces meet, so a vertex is there.
            let (dk, k) = (value("dk"), value("k"));
            let radius = (dk * dk / 4.0 + k * k) / (2.0 * k);
            let top = k - radius + (radius.powi(2) - (value("s") / 2.0).powi(2)).sqrt();
            [dk / 2.0, -value("l"), top]
        }
        "std:iso10642_countersunk_screw.geop" => [value("dk") / 2.0, -value("l"), 0.0],
        "std:iso4017_hex_head_screw.geop" => [corner(value("s")), -value("l"), value("k")],
        "std:iso4032_hex_nut.geop" => [corner(value("s")), 0.0, value("m")],
        "std:iso10511_nylon_insert_nut.geop" => [corner(value("s")), 0.0, value("h")],
        "std:iso7089_washer.geop" | "std:iso7090_chamfered_washer.geop" => {
            [value("d2") / 2.0, 0.0, value("h")]
        }
        "std:iso8734_dowel_pin.geop" => [value("d") / 2.0, 0.0, value("l")],
        "std:hex_standoff.geop" => [corner(value("s")), 0.0, value("l")],
        "std:ball_bearing.geop" => [value("D") / 2.0, 0.0, value("B")],
        "std:tslot_2020.geop" => [200f64.sqrt(), 0.0, value("length")],
        "std:tslot_2040.geop" => [500f64.sqrt(), 0.0, value("length")],
        "std:nema17_stepper.geop" => [21.15f64.hypot(17.15), -value("L"), 24.0],
        "std:spur_gear.geop" => [
            value("m") * (value("teeth.z") / 2.0 + 1.0),
            0.0,
            value("width"),
        ],
        "std:gt2_pulley.geop" => [
            value("df") / 2.0,
            0.0,
            value("lh") + 2.0 * value("t") + value("w"),
        ],
        "std:shaft_collar.geop" => [value("D") / 2.0, 0.0, value("b")],
        "std:flange_coupling.geop" => [value("F") / 2.0, 0.0, value("L")],
        "std:gear_rack.geop" => [(value("b") / 2.0).hypot(value("h")), 0.0, value("length")],
        other => panic!("no dimensions are known for {other}"),
    }
}

/// The part `program` — of the family `part`, at some size — builds: one
/// solid, valid as far as a fast check sees and fully if `fully`, with its
/// datums and threaded faces named and reaching where its table says — or
/// what is wrong with it.
fn built(part: &StandardPart, program: &Program, fully: bool) -> Result<Part<S>, String> {
    let built = program
        .build::<S>(&NoFiles)
        .map_err(|e| format!("does not build: {e}"))?;
    if fully {
        check_valid(&built).map_err(|e| format!("is not valid: {e}"))?;
    } else if let Err(errors) = validate_fast(&ValidationParameters::default(), built.topology()) {
        let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
        return Err(format!("is not valid: {}", messages.join("\n")));
    }
    let solids = built.solid_names();
    if solids.len() != 1 {
        return Err(format!("is not one solid: {solids:?}"));
    }
    for datum in ["axis", part.base] {
        built
            .datum_id(datum)
            .map_err(|_| format!("has no datum {datum}"))?;
    }
    for face in &part.threaded {
        built
            .face_id(face)
            .map_err(|_| format!("has no threaded face {face}"))?;
    }
    let inputs = program.inputs();
    let want = expected(part.file, |name| {
        number(&inputs, &format!("{SIZE}.{name}"))
            .or_else(|| number(&inputs, name))
            .unwrap_or_else(|| panic!("{} has no value {name}", part.file))
    });
    let points: Vec<[f64; 3]> = built
        .topology()
        .vertices
        .values()
        .map(|v| [0, 1, 2].map(|k| v.point[k].to_f64()))
        .collect();
    let radius = points.iter().map(|p| p[0].hypot(p[1])).fold(0.0, f64::max);
    let low = points.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
    let high = points
        .iter()
        .map(|p| p[2])
        .fold(f64::NEG_INFINITY, f64::max);
    let got = [radius, low, high];
    if (0..3).any(|k| (got[k] - want[k]).abs() > 1e-6) {
        return Err(format!(
            "reaches [radius, lowest, highest] = {got:?}, not {want:?}"
        ));
    }
    Ok(built)
}

/// Checks the family placed from `file` builds, at the size its file has.
fn assert_family_builds(file: &str) {
    let part = super::part(file).unwrap();
    if let Err(e) = built(part, &part.program, false) {
        panic!("{file} {e}");
    }
}

macro_rules! families_build {
    ($($(#[$attr:meta])* $test:ident: $file:literal,)*) => {
        $(
            #[test]
            $(#[$attr])*
            fn $test() {
                assert_family_builds($file);
            }
        )*

        /// Every family has its test above.
        #[test]
        fn every_family_is_tested() {
            let tested = [$($file),*];
            for part in parts().unwrap() {
                assert!(tested.contains(&part.file), "{} is not tested", part.file);
            }
        }
    };
}

families_build! {
    socket_head_cap_screws_build: "std:iso4762_socket_head_cap_screw.geop",
    button_head_screws_build: "std:iso7380_button_head_screw.geop",
    countersunk_screws_build: "std:iso10642_countersunk_screw.geop",
    hex_head_screws_build: "std:iso4017_hex_head_screw.geop",
    hex_nuts_build: "std:iso4032_hex_nut.geop",
    nylon_insert_nuts_build: "std:iso10511_nylon_insert_nut.geop",
    washers_build: "std:iso7089_washer.geop",
    chamfered_washers_build: "std:iso7090_chamfered_washer.geop",
    dowel_pins_build: "std:iso8734_dowel_pin.geop",
    hex_standoffs_build: "std:hex_standoff.geop",
    ball_bearings_build: "std:ball_bearing.geop",
    tslot_2020_builds: "std:tslot_2020.geop",
    tslot_2040_builds: "std:tslot_2040.geop",
    nema17_steppers_build: "std:nema17_stepper.geop",
    spur_gears_build: "std:spur_gear.geop",
    gt2_pulleys_build: "std:gt2_pulley.geop",
    shaft_collars_build: "std:shaft_collar.geop",
    flange_couplings_build: "std:flange_coupling.geop",
    gear_racks_build: "std:gear_rack.geop",
}

/// The programs of `part` at every size it offers: a row of its table, or
/// a few lengths of an extrusion.
fn every_size(part: &StandardPart) -> Vec<(String, Program)> {
    let with = |name: &str, value: ParamValue| {
        let mut program = part.program.clone();
        program.state.insert(name.into(), value);
        program
    };
    match part.program.parameters.get(SIZE).map(|p| &p.kind) {
        Some(ParameterKind::Table { rows, .. }) => rows
            .iter()
            .map(|r| (r.name.clone(), with(SIZE, ParamValue::Text(r.name.clone()))))
            .collect(),
        _ => [20.0, 333.3, 1000.0]
            .into_iter()
            .map(|l| {
                let length = ParamValue::Number(geop_ops::Design::from_f64(l));
                (format!("length {l}"), with("length", length))
            })
            .collect(),
    }
}

/// Every size of every family builds one solid reaching where its table
/// says — fully validated at the shortest and longest length of each
/// size, by a fast check at the lengths between. Prints how long each
/// family took to build, per size, with `--nocapture`.
#[test]
#[ignore = "slow: every size of every standard part — run with `cargo test -- --ignored`"]
fn every_size_of_every_family_builds_to_its_table() {
    let mut failures = Vec::new();
    for part in parts().unwrap() {
        let sizes = every_size(part);
        let size_of = |name: &str| name.split('x').next().unwrap_or(name).to_string();
        let start = std::time::Instant::now();
        let mut slowest = (String::new(), 0.0);
        for (i, (name, program)) in sizes.iter().enumerate() {
            let first = i == 0 || size_of(&sizes[i - 1].0) != size_of(name);
            let last = i + 1 == sizes.len() || size_of(&sizes[i + 1].0) != size_of(name);
            let one = std::time::Instant::now();
            if let Err(e) = built(part, program, first || last) {
                failures.push(format!("{} {name} {e}", part.file));
            }
            let took = one.elapsed().as_secs_f64();
            if took > slowest.1 {
                slowest = (name.clone(), took);
            }
        }
        let total = start.elapsed().as_secs_f64();
        println!(
            "{:<45} {:>3} sizes {total:>8.2} s, {:>6.2} s each, slowest {} {:.2} s",
            part.file,
            sizes.len(),
            total / sizes.len() as f64,
            slowest.0,
            slowest.1
        );
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}

/// The `bolted_plate` example holds together: the screw down the hole, the
/// nut under the plate, upside down — its base on the plate's bottom.
#[test]
fn the_bolted_plate_holds_together() {
    use std::collections::BTreeMap;

    use geop_core_math::vector::Vector3;
    use geop_ops::Design;

    use super::WithStandardParts;
    use crate::{Workspace, examples};

    let files = BTreeMap::from([(
        "plate.geop".to_string(),
        examples::metric_plate().to_json().unwrap(),
    )]);
    let workspace = Workspace::<S>::new(WithStandardParts(files));
    let part = examples::bolted_plate()
        .build(&workspace.scope("bolted_plate.geop"))
        .unwrap();
    assert!(part.check_mates(|_| true).unwrap().converged);
    let v = |p: [f64; 3]| Vector3::from_array(p.map(Design::from_f64));
    for (instance, local, want) in [
        ("screw", [0.0, 0.0, -12.0], [20.0, 20.0, -7.0]),
        ("nut", [0.0, 0.0, 3.2], [20.0, 20.0, -3.2]),
    ] {
        let id = part.instance_id(instance).unwrap();
        let got = part.instance(id).unwrap().pose.apply(&v(local));
        let got = [0, 1, 2].map(|k| got[k].to_f64());
        assert!(
            (0..3).all(|k| (got[k] - want[k]).abs() < 1e-6),
            "{instance} {local:?} is at {got:?}, not {want:?}"
        );
    }
}

/// One gap of a 20-tooth gear of module 1 cut from its blank, turned
/// `angle` degrees round: each is one boolean of building the gear.
fn one_gap_cut(angle: f64) -> Result<(), String> {
    use crate::operations::PartOperation;
    use geop_ops_pattern::Spacing;

    let mut program = super::part("std:spur_gear.geop").unwrap().program.clone();
    let k = program.index_of("teeth").unwrap();
    let PartOperation::CircularPattern(args) = &mut program.steps[k].operation else {
        panic!("the teeth are a circular pattern");
    };
    args.count = 2.0.into();
    args.angle = Spacing::step(angle);
    let built = program
        .build::<S>(&NoFiles)
        .map_err(|e| format!("does not build: {e}"))?;
    check_valid(&built).map_err(|e| format!("is not valid: {e}"))
}

#[test]
fn a_gear_gap_turned_234_degrees_is_cut() {
    one_gap_cut(234.0).unwrap();
}
