//! The operations run as programs: splitting a solid with a face standing
//! on its own.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    values::Design,
};
use geop_ops_edit::{DeleteBody, DeleteBodyArgs, ExtractFace, ExtractFaceArgs};
use geop_ops_extrude_revolve::testing::{Built, extrusion, on_origin, polygon, sketch, two_ways};
use geop_ops_model::{Combine, NamedModel, model, solid, testing::assert_valid};
use geop_ops_sketch::Sketch;

use crate::{Split, SplitArgs};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(Split)?
        .with(DeleteBody)?
        .with(ExtractFace)
}

/// The element `name` of the part's model, as a `K`.
fn named<K>(name: impl Into<String>) -> ElementReference<NamedModel<Design>, K> {
    ElementReference::new(model(), name)
}

/// A block's side wall, extracted, cuts a smaller block standing through
/// it in two; deleting the first block and the extracted face leaves
/// only the halves.
#[test]
fn extract_a_wall_split_with_it_and_delete_the_rest() {
    let mut big = Sketch::new();
    let (_, sides) = polygon(&mut big, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut small = Sketch::new();
    polygon(
        &mut small,
        &[[0.5, 0.2], [1.5, 0.2], [1.5, 0.8], [0.5, 0.8]],
    );
    let wall = format!("extrude(a,big,{})", sides[1]);
    let extracted = format!("extract(x,{wall})");

    let on_xy = |drawn| sketch(on_origin(FrameAxis::Z), drawn);
    let mut program = Program::new();
    program.push("big", on_xy(big));
    program.push("a", extrusion("big", two_ways(1.0, 1.0), Combine::NewBody));
    program.push("small", on_xy(small));
    program.push(
        "b",
        extrusion("small", two_ways(0.5, 0.5), Combine::NewBody),
    );
    program.push("x", ExtractFaceArgs { face: named(&wall) });
    program.push(
        "halves",
        SplitArgs {
            solid: solid("extrude(b)"),
            face: named(&extracted),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.sheet_face_names(), [extracted.as_str()]);
    assert_eq!(
        part.solid_names(),
        ["extrude(a)", "split(halves,0)", "split(halves,1)"]
    );
    assert!(part.face_id(&wall).is_ok(), "the wall itself stays");

    program.push(
        "clean",
        DeleteBodyArgs {
            bodies: vec![named("extrude(a)"), named(extracted)],
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.solid_names(), ["split(halves,0)", "split(halves,1)"]);
}
