//! Routes in programs: laid between points of what the steps before
//! built, their cables recorded and their bundles noted as only drawn.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    values::Design,
};
use geop_ops_extrude_revolve::{Extents, testing::extrude_sketch};
use geop_ops_model::selection::Selected;
use geop_ops_model::sketch::{
    figures::{n, on_origin, rectangle_outline},
    sketch_element,
};
use geop_ops_model::sketch3d::Sketch3d;
use geop_ops_model::{Combine, model, notes_of, testing::Built};
use geop_ops_sketch::AddSketchArgs;
use geop_ops_sketch3d::{AddSketch3d, AddSketch3dArgs};

use crate::{Route, RouteArgs, Wire, WireSize, cable};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    (geop_ops_extrude_revolve::testing::registry()?)
        .with(AddSketch3d)?
        .with(Route)
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
        rectangle_outline(0.0, 0.0, 4.0, 1.0),
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

/// A route through the corners of a 3-D sketch — vertices like any other —
/// and a planar sketch's point: routed through them, at least as long as
/// the straight runs between them.
#[test]
fn a_route_runs_through_sketch_points() {
    let mut program = Program::new();
    let mut corners = Sketch3d::new();
    let at = |p: [f64; 3]| geop_core_math::vector::Vector3::from_array(p.map(n));
    let (a, b) = (
        corners.add_point(at([0.0, 0.0, 0.0])),
        corners.add_point(at([4.0, 0.0, 0.0])),
    );
    corners.add_line(a, b);
    program.push(
        "corners",
        AddSketch3dArgs {
            sketch: corners,
            references: Vec::new(),
        },
    );
    let mut floor = geop_ops_model::sketch::Sketch::new();
    let p = floor.add_point(n(4.0), n(4.0));
    program.push("floor", AddSketchArgs::on(on_origin(FrameAxis::Z), floor));
    program.push(
        "route",
        RouteArgs {
            through: vec![
                vertex("sketch3d(corners,p0)"),
                vertex("sketch3d(corners,p1)"),
                Selected::SketchPoint(sketch_element("floor", p)),
            ],
            wires: vec![Wire {
                size: WireSize::Diameter(0.1),
                ..Wire::new("signal")
            }],
            fill: 1.0,
            bend_factor: 5.0,
            service_loop: 0.0,
        },
    );
    let part = program.built(registry).unwrap();
    let reader = part.current();
    let cable = cable(&reader, "route(route)").unwrap();
    let length = cable.length.to_f64();
    // Two runs of 4 with a quarter-turn bend between them, rounded.
    assert!(length > 7.5 && length < 10.0, "{:?}", cable.length);
}
