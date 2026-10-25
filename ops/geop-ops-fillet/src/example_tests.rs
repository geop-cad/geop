//! The examples of [`crate::examples`]: each builds a valid part, saves,
//! loads and builds again the very same part, and builds incrementally,
//! edit by edit, what it builds from scratch.

use std::rc::Rc;

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    operation_registry::OperationRegistry,
    testing::{assert_incremental, assert_round_trips, edit_sequence},
};
use geop_ops_model::testing::{check_valid, model_of};

use crate::{Fillet, examples::rounded_box};

/// The operations the examples are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?.with(Fillet)
}

/// Every edge rounded, and a ball's piece at each of the eight corners.
#[test]
fn rounded_box_round_trips() {
    let built = assert_round_trips(Rc::new(registry().unwrap()), None, &rounded_box());
    let part = model_of(&built);
    if let Err(report) = check_valid(&part) {
        panic!("rounded_box: {report}");
    }
    assert_eq!(part.solid_names(), ["fillet(round)"]);
    let corners = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .filter(|name| name.ends_with(",corner)"))
        .count();
    assert_eq!(corners, 8);
}

#[test]
fn rounded_box_builds_incrementally() {
    let programs = edit_sequence(&rounded_box());
    assert_incremental(Rc::new(registry().unwrap()), None, &programs);
}
