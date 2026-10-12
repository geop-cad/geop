//! SubD bodies in programs: built from their cage, and worked on further
//! by the other operations like any solid.

use geop_core_math::primitives::FrameAxis;
use geop_ops_booleans::{BooleanArgs, Combine, boolean::BooleanOp};
use geop_ops_extrude_revolve::Extents;
use geop_ops_subd::{Cage, Mirror, SubdArgs};

use crate::operations::testing::{Built, assert_valid, extrusion, origin_plane, outline, sketch};
use crate::{Program, Push};

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
        sketch(origin_plane(FrameAxis::Z), outline(&corners)),
    );
    program.push(
        "block",
        extrusion("block_sketch", Extents::blind(1.4), Combine::NewBody),
    );
    program.push(
        "join",
        BooleanArgs {
            a: "subd(blob)".into(),
            b: "extrude(block)".into(),
            op: BooleanOp::Union,
        },
    );
    let part = program.legacy_build_model().unwrap_or_else(|e| panic!("{e}"));
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 1, "{:?}", part.solid_names());
}
