//! Programs from bug reports, rebuilt as the smallest programs found that
//! still fail.

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
};
use geop_core_topology::Body;
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_fillet::{Fillet, FilletArgs};
use geop_ops_model::{
    NamedModel, model, solid,
    testing::{Built, assert_valid, edges_with_middle_where, face_towards},
};

use crate::{Shell, ShellArgs};

/// The operations these programs are written in: given solids, rounded,
/// then shelled.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Fillet)?.with(Shell)
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The name of the planar face of `solid` facing `normal`.
fn face_name(part: &NamedModel, solid: &str, normal: [f64; 3]) -> String {
    let id = part.solid_id(solid).unwrap();
    let face = face_towards(part, id, normal);
    part.name_of(face).unwrap().to_string()
}

/// An enclosure (Fusion 360 "Electronics enclosure" tutorial: a box, its
/// vertical edges rounded, shelled open at the top): a 10 x 6 x 3 block
/// with one upright edge rounded by 1, then shelled 0.3 thick open at its
/// top. The shell refuses the edge where the blend meets the block's
/// bottom:
///
/// > the edge is neither straight nor circular: offsetting it is not
/// > supported — the inner copy of edge
/// > fillet(corner,block(box,p0),block(box,end),...)
///
/// That edge is where a cylinder (the blend, around a vertical axis) meets
/// a horizontal plane: a quarter circle. The fillet's boolean traced it as
/// a fitted curve instead of recognising the arc, so the shell cannot
/// offset it. Ruled out: the open face (opening a side instead of the top,
/// with a top edge rounded, fails on the same kind of edge), the number of
/// fillets (all four uprights fail alike), the size (the 100 x 60 x 30
/// enclosure with radius 5 and walls 2 fails alike).
///
/// Shelling a filleted body is the first thing every enclosure tutorial
/// does; today a fillet must come after the shell, and then it rounds only
/// the outside.
#[test]
#[ignore = "bug: shelling a block whose upright edge was filleted refuses the blend's run-out edge on the bottom as 'neither straight nor circular' though it is a quarter circle (the fillet's boolean traced it); shell before fillet works"]
fn shell_of_a_block_with_a_rounded_upright_edge() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [10.0, 6.0, 3.0]),
    );
    let before = program.build_model(registry).unwrap();
    let edge = edges_with_middle_where(&before, |m| {
        near(m[0], 0.0) && near(m[1], 0.0) && near(m[2], 1.5)
    });
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("corner", FilletArgs::constant(edge, 1.0));
    let rounded = program.build_model(registry).unwrap();
    assert_valid(&rounded);
    let top = face_name(&rounded, "fillet(corner)", [0.0, 0.0, 1.0]);
    program.push(
        "s",
        ShellArgs {
            solid: solid("fillet(corner)"),
            faces: vec![ElementReference::new(model(), top)],
            thickness: 0.3.into(),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The outside's faces, their inner copies less the top, and the rim.
    let id = part.solid_id("shell(s)").unwrap();
    let outside = rounded.solid_id("fillet(corner)").unwrap();
    let faces = |part: &NamedModel, id| part.topology().body_faces(Body::Solid(id)).unwrap().len();
    assert_eq!(faces(&part, id), 2 * faces(&rounded, outside) - 1 + 1);
}
