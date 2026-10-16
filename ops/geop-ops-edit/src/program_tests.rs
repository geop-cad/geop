//! The edit operations run as programs: projecting sketches onto faces.
//! Deleting bodies and extracting faces, and splitting with those, are run
//! by `geop-ops-boolean`'s tests.

use crate::{ProjectCurve, ProjectCurveArgs};
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    values::Design,
};
use geop_ops_extrude_revolve::{
    Extent, Extents, ExtrudeArgs,
    testing::{Built, extrusion, n, on_origin, sketch},
};
use geop_ops_model::{
    Combine, NamedModel, model,
    testing::{assert_valid, error},
};
use geop_ops_sketch::{AddSketchArgs, Sketch, whole_sketch};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?.with(ProjectCurve)
}

fn on_xy(drawn: Sketch) -> AddSketchArgs {
    sketch(on_origin(FrameAxis::Z), drawn)
}

/// The element `name` of the part's model, as a `K`.
fn named<K>(name: impl Into<String>) -> ElementReference<NamedModel<Design>, K> {
    ElementReference::new(model(), name)
}

/// An extrude of `sketch` as a new body, from `from` to `to` along its
/// plane's normal.
fn extrude(sketch: &str, from: f64, to: f64) -> ExtrudeArgs {
    let extent = Extents {
        side1: Extent::blind(to),
        symmetric: false,
        side2: (from != 0.0).then_some(Extent::blind(-from)),
        reversed: false,
    };
    extrusion(sketch, extent, Combine::NewBody)
}

/// A rectangle from `lo` to `hi`, its corners counter-clockwise: the ids
/// of its sides, the first along the bottom.
fn rectangle(s: &mut Sketch, lo: [f64; 2], hi: [f64; 2]) -> Vec<geop_core_sketch::CurveId> {
    let corners = [
        [lo[0], lo[1]],
        [hi[0], lo[1]],
        [hi[0], hi[1]],
        [lo[0], hi[1]],
    ];
    let p: Vec<_> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect()
}

/// A block of `[-1, 1]^2 x [0, 1]` with the sketch `mark` on the plane
/// `z = 0` projected onto its top.
fn project_onto_top(mark: Sketch) -> Program {
    let mut block = Sketch::new();
    rectangle(&mut block, [-1.0, -1.0], [1.0, 1.0]);
    let mut program = Program::new();
    program.push("block_sketch", on_xy(block));
    program.push("block", extrude("block_sketch", 0.0, 1.0));
    program.push("mark", on_xy(mark));
    program.push(
        "p",
        ProjectCurveArgs {
            sketch: whole_sketch("mark"),
            face: named("extrude(block,end)"),
        },
    );
    program
}

/// A circle projected up onto a block's top cuts a disc out of it: a face
/// of its own, inside a hole of the rest of the top.
#[test]
fn project_a_circle_onto_a_block_top() {
    let mut mark = Sketch::new();
    let c = mark.add_point(n(0.2), n(0.1));
    mark.add_circle(c, n(0.4));
    let part = project_onto_top(mark).build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.sheet_face_names().is_empty());
    assert_eq!(part.topology().faces.len(), 7);
    let top = part.face_id("extrude(block,end)").unwrap();
    assert_eq!(part.topology().get_face(top).unwrap().holes.len(), 1);
}

/// A line projected onto a block's top divides it in two, and leaves the
/// bottom — on the line's own plane — as it was.
#[test]
fn project_a_line_across_a_block_top() {
    let mut mark = Sketch::new();
    let a = mark.add_point(n(0.3), n(-2.0));
    let b = mark.add_point(n(0.3), n(2.0));
    mark.add_line(a, b);
    let part = project_onto_top(mark).build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 7);
    assert_eq!(
        part.topology()
            .get_face(part.face_id("extrude(block,start)").unwrap())
            .unwrap()
            .holes
            .len(),
        0
    );
}

/// A line ending on the top divides nothing, and is refused.
#[test]
fn project_a_line_ending_on_the_face_fails() {
    let mut mark = Sketch::new();
    let a = mark.add_point(n(0.3), n(-2.0));
    let b = mark.add_point(n(0.3), n(0.0));
    mark.add_line(a, b);
    let error = error(project_onto_top(mark).build_model(registry));
    assert!(error.root_message().contains("end inside"), "{error:?}");
}
