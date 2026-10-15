//! Routes in programs: laid between points of what the steps before
//! built, their cables recorded and their bundles noted as only drawn.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    values::Design,
};
use geop_ops_add_datum::Selected;
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{
    Extents,
    testing::{Built, extrude_sketch, on_origin, rectangle},
};
use geop_ops_model::{model, notes_of};

use crate::{Route, RouteArgs, Wire, WireSize, cable};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?.with(Route)
}

/// The vertex `name` of the part's model.
fn vertex(name: &str) -> Selected {
    Selected::Vertex(ElementReference::new(model(), name))
}

/// A route along the bottom edge of a block 4 long, from one corner to the
/// next: a straight cable 4 long, recorded under the bundle's name, and the
/// bundle noted as only drawn, its wire ordered in its place.
#[test]
fn a_route_between_corners_of_a_block() {
    let mut program = Program::new();
    extrude_sketch(
        &mut program,
        "block",
        on_origin(FrameAxis::Z),
        rectangle(0.0, 0.0, 4.0, 1.0),
        Extents::blind(1.0),
        Combine::NewBody,
    );
    program.push(
        "cable",
        RouteArgs {
            through: vec![
                vertex("extrude(block,block_sketch,p0,start)"),
                vertex("extrude(block,block_sketch,p1,start)"),
            ],
            wires: vec![Wire {
                size: WireSize::Diameter(0.1),
                ..Wire::new("signal")
            }],
            fill: 1.0,
            bend_factor: 5.0,
            service_loop: 0.5,
        },
    );
    let part = program.built(registry).unwrap();
    let part = part.current();
    let laid = cable(&part, "route(cable)").expect("the cable is recorded");
    assert!(
        laid.length.could_be_equal(Design::from_f64(4.0)),
        "{laid:?}"
    );
    assert!(laid.tightest_bend.is_none());
    let wire = &laid.wires[0];
    assert!(
        wire.cut_length.could_be_equal(Design::from_f64(5.0)),
        "{wire:?}"
    );
    let note = notes_of(&part)
        .and_then(|notes| notes.get("route(cable)"))
        .expect("the bundle is noted");
    assert!(note.drawn_only);
    assert_eq!(note.wires, laid.wires);
    let built = program.build_model(registry).unwrap();
    assert!(built.solid_id("route(cable)").is_ok());
}
