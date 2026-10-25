//! Programs from bug reports, rebuilt with the coordinates they were
//! reported with: a "clean" equivalent may not carry the same case at all.
//! Each is the smallest program found that still fails.

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::{
    BooleanOp, solid,
    testing::{Built, assert_valid, assert_volume},
};

use crate::{Boolean, BooleanArgs};

/// The operations these programs are written in: given solids, combined.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Boolean)
}

/// A bracket modelled far from the origin (Fusion 360 "L-bracket"
/// tutorial, the part placed at `(10000, -5000, 2000)` as a part of a
/// larger assembly is): its base plate, 80 x 50 x 6, with a bolt's
/// clearance hole of radius 2.75 cut through it. The cut builds an invalid
/// solid: the three pieces of the hole's wall the boolean names
/// `combine(bore,...)` have
///
/// > face's normal points into solid rather than out of it: at its
/// > interior point (10018.06, -4978.06, 2000.5) the normal is
/// > (0.707, -0.707, -2e-9)
///
/// At the origin, at `(1000, 0, 0)` and at `(10000, 0, 0)` the same cut
/// builds valid; it needs the offset in all three coordinates. The
/// counterbored, countersunk and simple holes of the hole operation fail
/// alike there, so the plate's holes fail before any fillet is reached. The
/// normals reported are those of a cylinder around a vertical axis, so the
/// wall pieces are not reversed; what points "into the solid" is decided by
/// the interior point classification, which is suspect this far from the
/// origin (see AGENTS.md, "Evaluate relative to the patch, not the origin").
#[test]
#[ignore = "bug: a cylinder cut from a block at (10000, -5000, 2000) builds an invalid solid, three hole-wall pieces 'pointing into the solid'; at the origin, (1000, 0, 0) and (10000, 0, 0) it builds valid"]
fn cylinder_cut_from_a_block_far_from_the_origin() {
    let o = [10000.0, -5000.0, 2000.0];
    let mut program = Program::new();
    program.push(
        "base",
        PrimitiveArgs::block(o, [o[0] + 80.0, o[1] + 50.0, o[2] + 6.0]),
    );
    program.push(
        "bore",
        PrimitiveArgs::cylinder_z([o[0] + 20.0, o[1] + 20.0, o[2] - 1.0], 2.75, 8.0),
    );
    program.push(
        "drilled",
        BooleanArgs {
            a: solid("block(base)"),
            b: solid("cylinder(bore)"),
            op: BooleanOp::Difference,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let hole = std::f64::consts::PI * 2.75 * 2.75 * 6.0;
    assert_volume(&part, "boolean(drilled)", 80.0 * 50.0 * 6.0 - hole, 1.0);
}

/// A screw boss standing against the inner wall of an enclosure (Fusion
/// 360 "Electronics enclosure" tutorial: bosses are placed touching the
/// walls so they can be filleted into them): a cylinder of radius 3 whose
/// side is tangent to a block's side face, united with it. The union
/// builds a non-manifold solid:
///
/// > edge 70 has 4 coedge(s) (expected exactly 2 for a manifold shell)
/// > where EdgeId(70) = cylinder(boss,p3)
///
/// `cylinder(boss,p3)` is the cylinder's own seam between its quarters
/// `c2` and `c3`, at 270°, which is exactly the line along which the
/// block's face touches it. The same union with the tangent line away
/// from a seam (a prism whose side touches the cylinder at 45°) builds
/// valid, and so does the same cylinder cut from the block instead of
/// united. Inside a pocket (the boss against the pocket's wall) both seams
/// `p1` and `p3` touch and both come out with four coedges.
#[test]
#[ignore = "bug: uniting a cylinder with a block whose face is tangent to it along the cylinder's seam cylinder(boss,p3) leaves that seam with four coedges; tangent at 45° (no seam) and the cut build valid"]
fn union_of_a_cylinder_tangent_to_a_block_along_its_seam() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [20.0, 10.0, 10.0]),
    );
    program.push(
        "boss",
        PrimitiveArgs::cylinder_z([10.0, 13.0, 0.0], 3.0, 10.0),
    );
    program.push(
        "joined",
        BooleanArgs {
            a: solid("block(box)"),
            b: solid("cylinder(boss)"),
            op: BooleanOp::Union,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_volume(
        &part,
        "boolean(joined)",
        20.0 * 10.0 * 10.0 + std::f64::consts::PI * 9.0 * 10.0,
        1.0,
    );
}
