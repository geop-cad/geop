//! STEP export of an assembly: the parts a program places, written as
//! products and occurrences, and read back where they are placed.

use geop_core_program::{
    program::Program,
    values::{Design, StateValue, pose_state},
};
use std::collections::BTreeMap;

use geop_core_math::{
    plain,
    primitives::{FrameAxis, Pose, Quaternion},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_topology::Model;
use geop_ops_assembly::AddPartArgs;
use geop_ops_extrude_revolve::{Extents, testing::model_of};
use geop_ops_model::Combine;
use geop_ops_step::{ImportStepArgs, write_part};

use crate::registry::testing::{disc, extrude_sketch, on_origin, rectangle};
use crate::registry::{build, workspace};

/// A step placing `file`, fixed where the program's state puts it.
fn placing(file: &str) -> AddPartArgs {
    AddPartArgs {
        file: file.into(),
        ..Default::default()
    }
    .fixed()
}

fn at(x: f64, y: f64, z: f64) -> Pose<Design> {
    let v = Vector3::from_array([x, y, z].map(S::from_f64));
    Pose::new(v, Quaternion::identity()).unwrap()
}

/// `program` placing each of `placed` as its name, where its pose puts it.
fn with_placed(mut program: Program, placed: &[(&str, &str, Pose<Design>)]) -> Program {
    for (name, file, pose) in placed {
        program.push(*name, placing(file));
        (program.state).insert(pose_state(name), StateValue::Pose(*pose));
    }
    program
}

/// An assembly — a base plate placing a peg twice, once turned to lie along
/// `x`, and a subassembly placing the same peg — is written as products:
/// one per distinct component, each placement an occurrence. Read back, it
/// is flattened into its solids where they are placed.
#[test]
fn an_assembly_is_written_as_products_and_occurrences() {
    // A peg of radius 0.5 standing 2 high on the origin.
    let mut peg = Program::new();
    let (up, new) = (Extents::blind(2.0), Combine::NewBody);
    extrude_sketch(
        &mut peg,
        "c",
        on_origin(FrameAxis::Z),
        disc([0.0, 0.0], 0.5),
        up,
        new,
    );
    let sub = with_placed(
        Program::new(),
        &[("p", "parts/peg.geop", at(0.0, 0.0, 1.0))],
    );

    let mut base = Program::new();
    let plate = rectangle(0.0, 0.0, 10.0, 10.0);
    let (down, new) = (Extents::blind(-1.0), Combine::NewBody);
    extrude_sketch(&mut base, "base", on_origin(FrameAxis::Z), plate, down, new);
    // About `y` by a quarter turn: the peg's `z` onto `x`.
    let half = std::f64::consts::FRAC_PI_4;
    let turn = Quaternion::new(
        S::from_f64(half.cos()),
        S::ZERO,
        S::from_f64(half.sin()),
        S::ZERO,
    );
    let turned = Pose::new(Vector3::from_array([9.0, 8.0, 0.5].map(S::from_f64)), turn).unwrap();
    let robot = with_placed(
        base,
        &[
            ("a", "parts/peg.geop", at(2.0, 2.0, 0.0)),
            ("b", "parts/peg.geop", turned),
            ("s", "sub.geop", at(8.0, 2.0, 0.0)),
        ],
    );

    let files = BTreeMap::from([
        ("parts/peg.geop".to_string(), peg.to_json().unwrap()),
        ("sub.geop".to_string(), sub.to_json().unwrap()),
    ]);
    let runner = build(&robot, "robot.geop", workspace(files)).unwrap();
    let text = write_part(&runner.part.current(), "robot").unwrap();
    // To check it in another CAD system: written where STEP_EXPORT_DIR says.
    if let Some(dir) = std::env::var_os("STEP_EXPORT_DIR") {
        std::fs::write(std::path::Path::new(&dir).join("robot.step"), &text).unwrap();
    }
    let count = |what: &str| text.matches(what).count();
    // TODO: the peg written once, however often it is placed, once the
    // parts of no state are shared between instances (see
    // `geop_core_program::part::Part`): 3 products and 2 solids.
    assert_eq!(
        count("=PRODUCT('"),
        5,
        "the assembly, the subassembly, and each of the three pegs"
    );
    assert_eq!(count("NEXT_ASSEMBLY_USAGE_OCCURRENCE"), 4);
    assert_eq!(count("ITEM_DEFINED_TRANSFORMATION"), 4);
    assert_eq!(count("MANIFOLD_SOLID_BREP"), 4, "the base and each peg");
    assert!(text.contains("PRODUCT('peg','peg'"));

    // Read back by a program importing it.
    let mut program = Program::new();
    let file = "robot.step".to_string();
    program.push("imp", ImportStepArgs { file: file.clone() });
    let files = BTreeMap::from([(file, text)]);
    let part = build(&program, "main.geop", workspace(files)).unwrap().part;
    let back = model_of(&part);
    let model: &Model<Design> = back.topology();
    assert_eq!(model.solids.len(), 4, "the base and three pegs");
    // The pegs reach z = 2 (a), 3 (in the subassembly) and x = 11 (b).
    let (lo, hi) = plain::bounds(model.vertices.values().map(|v| v.point.to_f64_array())).unwrap();
    let close = |a: [f64; 3], b: [f64; 3]| (0..3).all(|k| (a[k] - b[k]).abs() < 1e-6);
    assert!(close(lo, [0.0, 0.0, -1.0]), "{lo:?}");
    assert!(close(hi, [11.0, 10.0, 3.0]), "{hi:?}");
}
