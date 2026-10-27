//! The standard parts: every family builds one valid solid of the size
//! its table says, with the datums it is mated by. An integration test, as
//! the parts are placed in assemblies and `geop-ops-assembly` depends on
//! this crate: a unit test would hold a second copy of it.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::registry::Registry;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{
    part::Part,
    program::Program,
    runner::Runner,
    testing::{edit_args, step},
    values::{Setting, Settings, StateValue, StateValues, pose_state},
    workspace::{self, Files, Location},
};
use geop_core_topology::validation::{ValidationParameters, validate_fast};
use geop_ops_model::datums::read_datums;
use geop_ops_model::parameters::{
    ParameterKind, parameter_numbers, parameters_of, read_properties, set_parameters,
    steps::{LOOKUP, PARAMETERS, PROPERTIES},
};
use geop_ops_model::spacing::Spacing;
use geop_ops_model::{
    NamedModel, RefId, read_model,
    testing::{check_valid, model_of},
};
use geop_ops_pattern::CircularPatternArgs;

use geop_lib_stdlib::{SIZE, StandardPart, TEETH, WithStandardParts, parts};

/// The operations the standard parts are written in, and those placing
/// them: each crate's `register`, as the editor's registry has them.
fn registry() -> Registry {
    let registry = (|| -> GeopResult<Registry> {
        let r = Registry::default();
        let r = geop_ops_model::register(r)?;
        let r = geop_ops_sketch::register(r)?;
        let r = geop_ops_reference_geometry::register(r)?;
        let r = geop_ops_extrude_revolve::register(r)?;
        let r = geop_ops_pattern::register(r)?;
        let r = geop_ops_assembly::register(r)?;
        Ok(r)
    })();
    registry.expect("the operations each have a name of their own")
}

/// The files `files`, beside the standard parts, whose programs are
/// written in the operations of [`registry`].
fn workspace(
    files: BTreeMap<String, String>,
) -> Rc<Files<WithStandardParts<BTreeMap<String, String>>>> {
    Files::new(WithStandardParts(files), Rc::new(registry().operations()))
}

/// A part a program built, and the runner that built it, to run it again.
struct BuiltPart {
    runner: Runner,
    part: Part,
}

/// Builds `program`, the file `file` of `workspace`, with its parameters
/// as it defines them.
fn build(
    program: &Program,
    file: &str,
    workspace: Rc<dyn workspace::Workspace>,
) -> GeopResult<BuiltPart> {
    let mut runner = Runner::new(registry().operations());
    runner.workspace = Some(workspace);
    runner.location = Location {
        file: workspace::resolve("", file),
        within: Vec::new(),
    };
    let mut part = Part::default();
    program.run(&runner, &mut part)?;
    Ok(BuiltPart { runner, part })
}

/// Building a program on its own, in an empty [`workspace`].
trait Built {
    /// The part the program builds.
    fn built(&self) -> GeopResult<Part>;

    /// The model the program builds, or why a step fails.
    fn build_model(&self) -> GeopResult<NamedModel> {
        Ok(model_of(&self.built()?))
    }
}

impl Built for Program {
    fn built(&self) -> GeopResult<Part> {
        Ok(build(self, "", workspace(BTreeMap::new()))?.part)
    }
}

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
        "std:gt2_pulley_16t.geop" | "std:gt2_pulley_20t.geop" | "std:gt2_pulley_36t.geop" => {
            let teeth: f64 = file[15..17].parse().unwrap();
            let outside = (2.0 * teeth / std::f64::consts::PI - 0.508) / 2.0;
            [outside + 2.0, 0.0, value("lh") + 9.0]
        }
        "std:shaft_collar.geop" => [value("D") / 2.0, 0.0, value("b")],
        "std:flange_coupling.geop" => [value("F") / 2.0, 0.0, value("L")],
        "std:gear_rack.geop" => [
            (value("b") / 2.0).hypot(value("h")),
            0.0,
            value("teeth") * std::f64::consts::PI * value("m"),
        ],
        "std:linear_rail.geop" => [(value("W") / 2.0).hypot(value("H")), 0.0, value("length")],
        "std:linear_carriage.geop" => [
            (value("W") / 2.0).hypot(value("H")),
            -value("L") / 2.0,
            value("L") / 2.0,
        ],
        "std:servo_sg90.geop" => [(5.4f64 + 16.1).hypot(6.1), -15.9, 6.8 + 4.0 + 3.2],
        "std:servo_mg996r.geop" => [(10.2f64 + 27.0).hypot(9.85), -27.0, 10.0 + 5.0 + 4.0],
        other => panic!("no dimensions are known for {other}"),
    }
}

/// The part `program` — of the family `part`, at some size — builds: one
/// solid, valid as far as a fast check sees and fully if `fully`, with its
/// datums and threaded faces named and reaching where its table says — or
/// what is wrong with it.
fn built(family: &StandardPart, program: &Program, fully: bool) -> Result<NamedModel, String> {
    let part = program
        .built()
        .map_err(|e| format!("does not build: {e}"))?;
    let view = part.current();
    let built = read_model(&view)
        .map_err(|e| format!("does not build: {e}"))?
        .into_owned();
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
    let datums = read_datums(&view).map_err(|e| format!("does not build: {e}"))?;
    for name in ["axis", family.base] {
        datums
            .get(name)
            .map_err(|_| format!("has no datum {name}"))?;
    }
    for face in &family.threaded {
        built.face_id(face).map_err(|_| {
            // What the step that cut it did name, to compare.
            let step = face.split(',').next().unwrap_or(face);
            let step = step.split('(').nth(1).unwrap_or(step);
            let mut named: Vec<&str> = built
                .names()
                .iter()
                .filter(|(id, name)| matches!(id, RefId::Face(_)) && name.contains(step))
                .map(|(_, name)| name)
                .collect();
            named.sort();
            format!("has no threaded face {face}: its step named the faces {named:?}")
        })?;
    }
    let numbers = parameter_numbers(program).map_err(|e| e.to_string())?;
    let want = expected(family.file, |name| {
        (numbers.get(&format!("{SIZE}.{name}")))
            .or_else(|| numbers.get(name))
            .copied()
            .unwrap_or_else(|| panic!("{} has no value {name}", family.file))
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
    let part = geop_lib_stdlib::part(file).unwrap();
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
    #[ignore = "slow: twenty booleans, see `a_gear_gap_turned_234_degrees_is_cut_valid` — run with `cargo test -- --ignored`"]
    spur_gears_build: "std:spur_gear.geop",
    gt2_16t_pulleys_build: "std:gt2_pulley_16t.geop",
    #[ignore = "slow: like the 16-tooth one, with more grooves — run with `cargo test -- --ignored`"]
    gt2_20t_pulleys_build: "std:gt2_pulley_20t.geop",
    #[ignore = "slow: like the 16-tooth one, with more grooves — run with `cargo test -- --ignored`"]
    gt2_36t_pulleys_build: "std:gt2_pulley_36t.geop",
    shaft_collars_build: "std:shaft_collar.geop",
    flange_couplings_build: "std:flange_coupling.geop",
    gear_racks_build: "std:gear_rack.geop",
    linear_rails_build: "std:linear_rail.geop",
    linear_carriages_build: "std:linear_carriage.geop",
    sg90_servos_build: "std:servo_sg90.geop",
    mg996r_servos_build: "std:servo_mg996r.geop",
}

/// `program` with its parameter `name` set to `value`: the row of a table
/// picked by its name, or a number's formula.
fn sized(program: &Program, name: &str, value: &str) -> Program {
    let mut program = program.clone();
    let mut parameters = parameters_of(&program);
    let parameter = (parameters.values.iter_mut())
        .find(|p| p.name == name)
        .unwrap_or_else(|| panic!("there is no parameter {name}"));
    match &mut parameter.kind {
        ParameterKind::Table { selected, .. } => *selected = value.into(),
        ParameterKind::Number { expression, .. } => *expression = value.into(),
    }
    set_parameters(&mut program, parameters);
    program
}

/// The programs of `part` at every size it offers: a row of its table, or
/// a few lengths of an extrusion.
fn every_size(part: &StandardPart) -> Vec<(String, Program)> {
    // A gear's module and tooth count are tables of their own: every
    // module with 20 teeth, and module 1 with each of a range of counts.
    let parameters = parameters_of(&part.program);
    let gear = matches!(
        parameters.get(TEETH).map(|p| &p.kind),
        Some(ParameterKind::Table { .. })
    );
    match parameters.get(SIZE).map(|p| &p.kind) {
        Some(ParameterKind::Table { rows, .. }) if gear => {
            let geared = |module: &str, teeth: usize| {
                let z = format!("z{teeth}");
                let program = sized(&sized(&part.program, SIZE, module), TEETH, &z);
                (format!("{module} {z}"), program)
            };
            rows.iter()
                .map(|r| geared(&r.name, 20))
                .chain(GEAR_TEETH.into_iter().map(|z| geared("m1", z)))
                .collect()
        }
        Some(ParameterKind::Table { rows, .. }) => rows
            .iter()
            .map(|r| (r.name.clone(), sized(&part.program, SIZE, &r.name)))
            .collect(),
        _ => [20.0, 333.3, 1000.0]
            .into_iter()
            .map(|l| {
                (
                    format!("length {l}"),
                    sized(&part.program, "length", &l.to_string()),
                )
            })
            .collect(),
    }
}

/// The tooth counts a gear of module 1 is built with by
/// [`every_size_of_every_family_builds_to_its_table`].
const GEAR_TEETH: [usize; 8] = [12, 17, 25, 33, 42, 60, 80, 120];

/// The sizes of the family `part` builds to its table — every one but
/// those of a gear, of which a range (see [`every_size`]) — fully validated
/// at the first and last length of each size, by a fast check at the
/// lengths between: what is wrong, one line per size. Prints how long the
/// family took to build, per size.
fn build_every_size(part: &StandardPart) -> Vec<String> {
    let mut failures = Vec::new();
    let sizes = every_size(part);
    let size_of = |name: &str| name.split('x').next().unwrap_or(name).to_string();
    let start = std::time::Instant::now();
    let mut slowest = (String::new(), 0.0);
    for (i, (name, program)) in sizes.iter().enumerate() {
        let first = i == 0 || size_of(&sizes[i - 1].0) != size_of(name);
        let last = i + 1 == sizes.len() || size_of(&sizes[i + 1].0) != size_of(name);
        let one = std::time::Instant::now();
        // A panic is one size's failure too, by its name, not the sweep's.
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            built(part, program, first || last)
        }));
        match result {
            Ok(Ok(_)) => {}
            Ok(Err(e)) => failures.push(format!("{} {name} {e}", part.file)),
            Err(panic) => {
                let message = panic
                    .downcast_ref::<String>()
                    .cloned()
                    .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                    .unwrap_or_default();
                failures.push(format!("{} {name} panics: {message}", part.file));
            }
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
    failures
}

/// Every size of every family builds one solid reaching where its table
/// says (see [`build_every_size`]). Prints how long each family took to
/// build, per size, with `--nocapture`.
#[test]
#[ignore = "slow: every size of every standard part — run with `cargo test -- --ignored`"]
fn every_size_of_every_family_builds_to_its_table() {
    let failures: Vec<String> = parts().unwrap().flat_map(build_every_size).collect();
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}

/// The spur gears of [`every_size_of_every_family_builds_to_its_table`]
/// alone: every module, and a range of tooth counts.
#[test]
#[ignore = "slow: sixteen gears, up to 120 teeth, half an hour — run with `cargo test -- --ignored`"]
fn every_spur_gear_builds_to_its_table() {
    let failures = build_every_size(geop_lib_stdlib::part("std:spur_gear.geop").unwrap());
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}

/// The fewest teeth a gear has build into a valid gear, twelve booleans.
/// The default suite checks the family by two gaps cut valid instead (see
/// `a_gear_gap_turned_234_degrees_is_cut_valid`).
#[test]
#[ignore = "slow: twelve booleans — run with `cargo test -- --ignored`"]
fn a_12_tooth_spur_gear_builds() {
    let part = geop_lib_stdlib::part("std:spur_gear.geop").unwrap();
    let program = sized(&part.program, TEETH, "z12");
    if let Err(e) = built(part, &program, false) {
        panic!("a 12-tooth spur gear {e}");
    }
}

/// The drive and motion parts are ordered as a bill of materials lists
/// them — every parameter a size is chosen by — and are made of what they
/// are made of; a servo weighs what it does.
#[test]
fn drive_and_motion_parts_are_designated_and_made_of_something() {
    let runner = Runner::new(registry().operations());
    let text = |pairs: &[(&str, &str)]| -> Settings {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), Setting::Text(v.to_string())))
            .collect()
    };
    let number = Setting::Number;
    let mut gear = text(&[("size", "m1.5"), ("teeth", "z32")]);
    gear.insert("width".into(), number(12.0));
    gear.insert("bore".into(), number(8.0));
    let mut rail = text(&[("size", "MGN12")]);
    rail.insert("length".into(), number(250.0));
    let mut rack = text(&[("size", "m1")]);
    rack.insert("teeth".into(), number(25.0));
    for (file, values, want, material) in [
        (
            "std:spur_gear.geop",
            gear,
            "Spur gear m1.5 z32 width 12 bore 8",
            "Steel",
        ),
        ("std:gear_rack.geop", rack, "Gear rack m1 teeth 25", "Steel"),
        (
            "std:gt2_pulley_20t.geop",
            text(&[("size", "20T-8")]),
            "GT2 pulley 20T-8",
            "Aluminium 6061",
        ),
        (
            "std:shaft_collar.geop",
            text(&[("size", "8")]),
            "DIN 705 A 8",
            "Steel",
        ),
        (
            "std:flange_coupling.geop",
            text(&[("size", "5x8")]),
            "Flange coupling 5x8",
            "Aluminium 6061",
        ),
        (
            "std:linear_rail.geop",
            rail,
            "Linear rail MGN12 length 250",
            "Steel",
        ),
        (
            "std:linear_carriage.geop",
            text(&[("size", "MGN12H")]),
            "Linear carriage MGN12H",
            "Steel",
        ),
        (
            "std:servo_sg90.geop",
            Settings::new(),
            "Servo SG90",
            "Plastic",
        ),
        (
            "std:servo_mg996r.geop",
            Settings::new(),
            "Servo MG996R",
            "Plastic",
        ),
    ] {
        // Its properties and parameters run on their own, with `values`:
        // what it is designated as. Unset parameters are the family's own.
        let mut program = geop_lib_stdlib::part(file).unwrap().program.clone();
        (program.steps).retain(|s| [PROPERTIES, PARAMETERS, LOOKUP].contains(&&*s.operation));
        let mut part = Part::default();
        let output = runner.run_with(&mut part, &program, &values, None);
        assert!(
            output.step_errors.is_empty(),
            "{file}: {:?}",
            output.step_errors
        );
        let properties = read_properties(&part.current()).unwrap().into_owned();
        assert_eq!(properties.designation.as_deref(), Some(want), "{file}");
        let made_of = parameters_of(&program).material;
        assert!(
            made_of
                .as_ref()
                .is_some_and(|m| m.name.starts_with(material)),
            "{file}: {made_of:?}"
        );
    }
    // The SG90's envelope, weighed as its material says: 9 g.
    let sg90 = geop_lib_stdlib::part("std:servo_sg90.geop").unwrap();
    let density = parameters_of(&sg90.program).material.unwrap().density;
    let volume = 22.8 * 12.2 * (15.9 + 6.8)
        + (32.2 - 22.8) * 12.2 * 2.5
        + std::f64::consts::PI / 4.0 * (11.4f64.powi(2) * 4.0 + 4.8f64.powi(2) * 3.2);
    assert!((density * volume * 1e-6 - 9.0).abs() < 1e-9);
}

/// One gap of a gear of module 1 and `teeth` cut from its blank, turned
/// `angle` degrees round: each is one boolean of building the gear.
fn gaps_cut(teeth: &str, count: usize, angle: f64) -> Result<(), String> {
    let gear = &geop_lib_stdlib::part("std:spur_gear.geop").unwrap().program;
    let mut program = sized(gear, TEETH, teeth);
    edit_args(
        step(&mut program, "teeth"),
        |args: &mut CircularPatternArgs| {
            args.count = (count as f64).into();
            args.angle = Spacing::step(angle);
        },
    );
    let built = program
        .build_model()
        .map_err(|e| format!("does not build: {e}"))?;
    check_valid(&built).map_err(|e| format!("is not valid: {e}"))
}

/// The curves where a gap's flanks cross the gear's sides are traced, and
/// an involute's curvature changes tenfold along them: fitted as finely as
/// usual, they came out 7e-4 wide, and the gear was not valid.
#[test]
fn a_gear_gap_turned_234_degrees_is_cut_valid() {
    gaps_cut("z20", 2, 234.0).unwrap();
}

/// Every gap cut splits the rim where it crosses it, twice on each side:
/// the ninth gap of a 42-tooth gear once found a quarter of the rim, split
/// sixteen times, too wide to cut, when each cut carried the width of its
/// parameter into the pieces.
#[test]
#[ignore = "slow: twelve booleans — run with `cargo test -- --ignored`"]
fn twelve_gaps_of_a_42_tooth_gear_are_cut_valid() {
    gaps_cut("z42", 12, 360.0 / 42.0).unwrap();
}

/// The thirty gaps of a 120-tooth gear that cross one quarter of its rim,
/// each side's quarter split sixty times.
#[test]
#[ignore = "slow: thirty booleans — run with `cargo test -- --ignored`"]
fn a_quarter_of_a_120_tooth_gear_is_cut_valid() {
    gaps_cut("z120", 30, 3.0).unwrap();
}

/// A NEMA 17 stepper driving a gear pair: a 12-tooth pinion on its shaft,
/// meshing with a 15-tooth wheel beside it — module 1, so their centres
/// are `(12 + 15) / 2 = 13.5` apart, both 10 up from the motor's face. Each
/// turns on a revolute joint about an axis of the assembly's own, and a
/// gear coupling of ratio `15 / 12`, reversed, ties the two: the motor
/// turning the pinion turns the wheel four fifths as far the other way.
/// With an odd count, the wheel has a gap facing the pinion's tooth on
/// `+x` unturned, and the coupling keeps them in mesh.
fn gear_drive() -> Program {
    use geop_core_math::{
        primitives::{DatumComponent, FrameAxis, Pose},
        vector::Vector3,
    };
    use geop_ops_assembly::{AddPartArgs, CouplingKind, Joint, JointKind, Mate, Transmission};
    use geop_ops_model::datums::{ORIGIN, component_name, datum};
    use geop_ops_model::mate_entity::MateEntity;
    use geop_ops_model::selection::Selected;
    use geop_ops_reference_geometry::{AddDatumArgs, Construction};

    let n = Scalar::from_f64;
    let pose = |position: [f64; 3], degrees: [f64; 3]| {
        Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n)).unwrap()
    };

    let mut program = Program::new();
    program.push(
        "motor",
        AddPartArgs {
            file: "std:nema17_stepper.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    // The axes the gears turn about: the motor's, and one 13.5 along `x`,
    // through points 10 up.
    let z_axis = Selected::Datum(datum(component_name(
        ORIGIN,
        DatumComponent::Axis(FrameAxis::Z),
    )));
    for (id, x) in [("pinion", 0.0), ("wheel", 13.5)] {
        program.push(
            format!("{id}_centre"),
            AddDatumArgs {
                selection: vec![Selected::Datum(datum(ORIGIN))],
                construction: Construction::Point {
                    x: x.into(),
                    y: 0.0.into(),
                    z: 10.0.into(),
                },
            },
        );
        program.push(
            format!("{id}_shaft"),
            AddDatumArgs {
                selection: vec![
                    Selected::Datum(datum(format!("{id}_centre"))),
                    z_axis.clone(),
                ],
                construction: Construction::Parallel {},
            },
        );
    }
    let gear = |teeth: &str, bore: f64| {
        Settings::from([
            ("teeth".to_string(), Setting::Text(teeth.into())),
            ("bore".to_string(), Setting::Number(bore)),
        ])
    };
    let turning = |id: &str| {
        Mate::joint(
            JointKind::new(Joint::Revolute),
            vec![
                MateEntity::on(Selected::Datum(datum(format!("{id}_shaft")))),
                MateEntity::of(id, Selected::Datum(datum("axis"))),
            ],
        )
    };
    program.push(
        "pinion",
        AddPartArgs {
            file: "std:spur_gear.geop".into(),
            parameters: gear("z12", 5.0),
            mates: BTreeMap::from([("m1".into(), turning("pinion"))]),
        },
    );
    let coupling = Mate::coupling(
        CouplingKind {
            transmission: Transmission::Gear,
            value: n(15.0 / 12.0),
            reverse: true,
            offset: None,
        },
        vec!["add_part(pinion,m1)".into(), "add_part(wheel,m1)".into()],
    );
    program.push(
        "wheel",
        AddPartArgs {
            file: "std:spur_gear.geop".into(),
            parameters: gear("z15", 6.0),
            mates: BTreeMap::from([("m1".into(), turning("wheel")), ("m2".into(), coupling)]),
        },
    );
    program.state = StateValues::from([
        (
            pose_state("motor"),
            StateValue::Pose(pose([0.0; 3], [0.0; 3])),
        ),
        (
            pose_state("pinion"),
            StateValue::Pose(pose([0.0, 0.0, 10.0], [0.0; 3])),
        ),
        (
            pose_state("wheel"),
            StateValue::Pose(pose([13.5, 0.0, 10.0], [0.0; 3])),
        ),
    ]);
    program
}

/// The gear drive holds together where it is drawn, and turning the
/// pinion 90° turns the wheel 72° the other way.
#[test]
#[ignore = "slow: builds a 12- and a 15-tooth gear — run with `cargo test -- --ignored`"]
fn a_motor_drives_a_gear_pair_at_its_ratio() {
    use geop_core_editor::editor::Rerun;
    use geop_ops_assembly::solving;

    let program = gear_drive();
    let mut built = build(&program, "gear_drive.geop", workspace(BTreeMap::new())).unwrap();
    let driver = "add_part(pinion,m1).angle";
    let set = BTreeMap::from([(driver.to_string(), Scalar::from_f64(90.0))]);
    let mut rerun = Rerun {
        runner: &built.runner,
        part: &mut built.part,
        program: &program,
    };
    let (moved, report) = solving::set(&mut rerun, &program.state, &set).unwrap();
    assert!(report.converged, "{report:?}");
    let StateValue::Number(driven) = moved["add_part(wheel,m1).angle"] else {
        panic!("the wheel's joint has an angle");
    };
    assert!((driven.to_f64() + 72.0).abs() < 1e-9, "{driven:?}");
}
