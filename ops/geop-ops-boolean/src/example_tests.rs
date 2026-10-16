//! The examples of [`crate::examples`]: each builds a valid part, saves,
//! loads and builds again the very same part, and builds incrementally,
//! edit by edit, what it builds from scratch.

use std::rc::Rc;

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    operation_registry::OperationRegistry,
    testing::{assert_incremental, assert_round_trips, edit_sequence},
};
use geop_core_topology::contains::shell::PointClassification;
use geop_ops_extrude_revolve::testing::model_of;
use geop_ops_model::{
    PartDescription,
    testing::{check_valid, classify},
};

use crate::{Split, examples::split_plate};

/// The operations the examples are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?.with(Split)
}

#[test]
fn split_plate_round_trips() {
    let built = assert_round_trips(Rc::new(registry().unwrap()), None, &split_plate());
    let part = model_of(&built);
    if let Err(report) = check_valid(&part) {
        panic!("split_plate: {report}");
    }
    let description = PartDescription::of(&part).unwrap();
    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["split(halves,0)", "split(halves,1)"]
    );
    // The face it was cut with stays, standing on its own.
    assert_eq!(part.sheet_face_names(), ["extrude(cut,cut_sketch,c2)"]);
    // One half each side of the cut at x = 2, neither in the hole.
    let at = |p: [f64; 3]| {
        ["split(halves,0)", "split(halves,1)"]
            .map(|half| classify(&part, half, p) == PointClassification::Inside)
    };
    let (left, right) = (at([0.2, 0.2, 0.0]), at([2.5, 0.5, 0.1]));
    assert!(
        left[0] != left[1] && right == [left[1], left[0]],
        "{left:?} {right:?}"
    );
    assert_eq!(at([0.75, 0.5, 0.0]), [false, false]);
}

#[test]
fn split_plate_builds_incrementally() {
    let programs = edit_sequence(&split_plate());
    assert_incremental(Rc::new(registry().unwrap()), None, &programs);
}
