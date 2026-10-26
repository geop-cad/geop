//! Programs from bug reports, rebuilt with the coordinates they were
//! reported with: a "clean" equivalent may not carry the same case at all.
//! Each is the smallest program found that still fails.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::{
    BooleanOp, solid,
    testing::{Built, assert_valid, assert_volume, solid_volume},
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
/// built valid; it needed the offset in all three coordinates. The
/// counterbored, countersunk and simple holes of the hole operation failed
/// alike there, so the plate's holes failed before any fillet was reached.
///
/// The normals were right; the check was wrong, and not by numerics. It
/// probed a point a distance off the face on either side, starting from the
/// shell's "extent", which took the least coordinate over all three axes
/// (`-5000`, the `y`) and the greatest (`10080`, the `x`): 15080 for a plate
/// 80 long. Halved twelve times that stops at 7.4, and on the way the probe
/// at 29.4 from a wall 18 from the plate's side landed outside the plate
/// against the normal and, across the bore, in the plate along it: "solid
/// on the normal's side". The three pieces whose normals have a component
/// towards a side 18 away were the three that failed. Which side of a face
/// the solid is on is now decided by a ray leaving the face, with no probe
/// distance at all (`solid_contains_beside`).
///
/// The volume is measured twice: by the kernel's integration, which must
/// enclose the exact one wherever the plate is, and by the rasterized mesh
/// (`enclosed_volume`, whose tetrahedra are summed from a corner of the
/// mesh since this test: from the origin, the slivers between faces
/// weighed 5.5 at the first placement).
#[test]
fn cylinder_cut_from_a_block_far_from_the_origin() {
    for o in [
        [10000.0, -5000.0, 2000.0],
        [-10000.0, 5000.0, -2000.0],
        [1e6, 0.0, 0.0],
        [0.0, 1e6, 0.0],
        [0.0, 0.0, 1e6],
    ] {
        cylinder_cut_from_a_block_at(o);
    }
}

/// A plate 80 x 50 x 6 at `o`, with a bore of radius 2.75 cut through it
/// 20 from two of its sides: valid, and of the volume it should have.
fn cylinder_cut_from_a_block_at(o: [f64; 3]) {
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
    let expected = 80.0 * 50.0 * 6.0 - hole;
    let mass = part
        .topology()
        .mass_properties(part.solid_id("boolean(drilled)").unwrap())
        .unwrap();
    assert!(
        mass.volume.could_be_equal(Scalar::from_f64(expected)),
        "the plate at {o:?} encloses {:?}, not {expected}",
        mass.volume
    );
    let got = solid_volume(&part, "boolean(drilled)");
    assert!(
        (got - expected).abs() <= 1.0,
        "the plate at {o:?}, rasterized, encloses {got}, not {expected}"
    );
}

/// A screw boss standing against the inner wall of an enclosure (Fusion
/// 360 "Electronics enclosure" tutorial: bosses are placed touching the
/// walls so they can be filleted into them): a cylinder of radius 3 whose
/// side is tangent to a block's side face, united with it. The two touch
/// along the cylinder's seam `cylinder(boss,p3)` alone, so the union's
/// boundary touches itself there: an edge of four faces, which the boolean
/// built and full validation once refused ("edge 70 has 4 coedge(s)"). A
/// solid may touch itself along an edge, its faces paired around it, and
/// validation now says so too.
#[test]
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
