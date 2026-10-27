//! Programs from tutorials and bug reports, rebuilt as the smallest
//! programs found that still fail.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::{Primitive, PrimitiveArgs};
use geop_ops_extrude_revolve::{Extent, Extents, testing::extrusion};
use geop_ops_model::datums::origin_axis;
use geop_ops_model::sketch::figures::{disc, on_origin};
use geop_ops_model::spacing::Spacing;
use geop_ops_model::{
    Combine,
    testing::{Built, assert_valid},
};
use geop_ops_sketch::AddSketchArgs;

use crate::{CircularPattern, CircularPatternArgs, Seeds, TurnAxis, body, feature};

/// The operations these programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    (geop_ops_extrude_revolve::testing::registry()?.with(Primitive)?).with(CircularPattern)
}

/// A flange with a ring of holes (the Fusion 360 "flange" tutorial, with
/// more holes than fit apart): a disc of radius 40 and height 10, a hole
/// of radius 4 cut through it at radius 30, and that feature patterned 24
/// times round the `Z` axis. The holes' centres are 7.83 apart, so each
/// overlaps the next by a sliver 0.17 wide.
///
/// The pattern builds, but invalid: eight edges have control points wider
/// than the 1e-4 every search downstream assumes, e.g.
///
/// > edge EdgeId(1713)'s curve control point 4, component 1 is [22.94915,
/// > 22.94932], 0.00017 wide — something that produced it skipped a
/// > refinement
///
/// each an edge of `combine(p,4,hole,..)`, `combine(p,10,hole,..)`, ...,
/// where one copy's wall crosses the next's. With 6 holes (apart) or 30
/// (overlapping more) the pattern builds valid.
#[test]
#[ignore = "bug: a circular pattern of 24 holes that overlap their neighbours by a sliver builds an invalid body — edges where one copy's wall crosses the next's carry control points ~1.7e-4 wide; 6 or 30 holes build valid"]
fn ring_of_24_holes_each_just_overlapping_the_next() {
    let mut program = Program::new();
    program.push(
        "disc",
        PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 40.0, 10.0),
    );
    program.push(
        "h",
        AddSketchArgs::on(on_origin(FrameAxis::Z), disc([30.0, 0.0], 4.0)),
    );
    program.push(
        "hole",
        extrusion(
            "h",
            Extents {
                side1: Extent::ThroughAll,
                ..Extents::blind(10.0)
            },
            Combine::cut("cylinder(disc)"),
        ),
    );
    program.push(
        "p",
        CircularPatternArgs {
            seeds: Seeds::features(vec![feature("hole")]),
            axis: Some(TurnAxis::Datum(origin_axis(FrameAxis::Z))),
            reversed: false,
            count: 24.0.into(),
            angle: Spacing::extent(360.0),
        },
    );
    assert_valid(&program.build_model(registry).unwrap());
}

/// A block turned a quarter round its own upright edge and joined to
/// itself (a cross of four arms made by patterning one round its corner,
/// or here just two): the 20 x 20 x 10 block from the origin, patterned
/// twice round the `Z` axis 90 degrees apart, joined. The copy stands
/// beside the block, the two sharing their side on the plane `x = 0`
/// exactly. The join fails:
///
/// > loops_contain: could not find a ray direction clear of every vertex
/// > after many attempts
///
/// (between `face_a` and `face_b`, the two coincident sides). Four copies
/// round the full turn fail alike; the block mirrored in the plane
/// `x = 0` and joined, the same pair of coincident sides, builds; and so
/// does the same pattern of a block whose top edge is rounded in its
/// profile (extruded along `x`).
#[test]
#[ignore = "bug: a block patterned round its own upright edge 90 degrees apart and joined fails — \"loops_contain: could not find a ray direction clear of every vertex after many attempts\" — on the two coincident sides; mirrored into the same place, it joins"]
fn block_turned_a_quarter_round_its_edge_and_joined() {
    let mut program = Program::new();
    program.push(
        "arm",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [20.0, 20.0, 10.0]),
    );
    program.push(
        "p",
        CircularPatternArgs {
            seeds: Seeds::bodies(vec![body("block(arm)")], Combine::join("block(arm)")),
            axis: Some(TurnAxis::Datum(origin_axis(FrameAxis::Z))),
            reversed: false,
            count: 2.0.into(),
            angle: Spacing::step(90.0),
        },
    );
    assert_valid(&program.build_model(registry).unwrap());
}
