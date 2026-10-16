//! SubD bodies in programs: built from their cage, and worked on further
//! by the other operations like any solid.

use crate::{Cage, Mirror, Subd, SubdArgs};
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_ops_boolean::{Boolean, BooleanArgs};
use geop_ops_extrude_revolve::{
    Extents,
    testing::{Built, extrusion, on_origin, outline, sketch},
};
use geop_ops_model::{BooleanOp, Combine, solid};

use geop_ops_model::testing::assert_valid;

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(Boolean)?
        .with(Subd)
}

/// A rounded blob — the limit of a box cage two units a side around the
/// origin — united with a block reaching into it from one corner: one
/// valid solid.
#[test]
fn subd_body_united_with_a_box() {
    let mut program = Program::new();
    program.push(
        "blob",
        SubdArgs {
            cage: Cage::cuboid([2.0, 2.0, 2.0]),
            mirror: Mirror::None,
        },
    );
    let corners = [[0.1, 0.15], [1.4, 0.15], [1.4, 1.35], [0.1, 1.35]];
    program.push(
        "block_sketch",
        sketch(on_origin(FrameAxis::Z), outline(&corners)),
    );
    program.push(
        "block",
        extrusion("block_sketch", Extents::blind(1.4), Combine::NewBody),
    );
    program.push(
        "join",
        BooleanArgs {
            a: solid("subd(blob)"),
            b: solid("extrude(block)"),
            op: BooleanOp::Union,
        },
    );
    let part = program
        .build_model(registry)
        .unwrap_or_else(|e| panic!("{e}"));
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 1, "{:?}", part.solid_names());
}

/// A mouse's sole is flat: the crease around its bottom keeps the
/// bottom face in the cage's bottom plane. Saved and loaded, the example
/// builds what it built.
#[test]
fn subd_mouse_round_trips() {
    use geop_core_math::scalars::Scalar;
    use geop_core_program::values::Design;
    use geop_ops_model::read_model;

    let registry = std::rc::Rc::new(registry().unwrap());
    let program = crate::examples::subd_mouse();
    let built = geop_core_program::testing::assert_round_trips(registry, None, &program);
    let reader = built.current();
    let part = read_model(&reader).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["subd(mouse)"]);
    let sole = part.face_id("subd(mouse,f8)").unwrap();
    let surface = &part.topology().get_face(sole).unwrap().surface;
    let p = surface
        .evaluate(Design::from_f64(0.7), Design::from_f64(1.2))
        .unwrap();
    assert!(p[2].could_be_equal(Design::from_f64(-0.4)), "{p:?}");
}
