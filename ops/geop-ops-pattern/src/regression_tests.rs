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
///
/// Measured (not fixed): `EdgeId(1713)` is the vertical line where copy
/// 3's wall crosses copy 4's, traced from `VertexId(1657)`, the crossing
/// of their bottom arcs (`combine(p,4,hole,..c1#1,start..c1#2,start..,0,1)`,
/// `1e-11` wide). During that boolean another vertex, `VertexId(1664)`,
/// lies on both walls' surfaces to within its own width, `1.5e-4` by
/// `3.1e-4`, `6.7e-4` from 1657 on the bottom face; it is gone from the
/// finished model. The trace takes it for the end the curve runs to
/// (`candidate_within`'s half step) and marches `7e-5`, `1.1e-4`, then
/// strides of `0.3` to `3`: the cubic through points spaced that unevenly
/// (knots `0, 2.2e-5, 3.2e-5, 0.083, 0.25, 0.5, 1`) is widened to
/// `1.7e-4` in its middle. The pair's other line, traced in
/// even strides, is `7e-6` wide. To find next: which crossing made 1664
/// that wide (an edge x face crossing left unrefined?), and that a vertex
/// within its own width of a surface is proximity, not membership.
#[test]
#[ignore = "bug: a circular pattern of 24 holes that overlap their neighbours by a sliver builds an invalid body — the trace from the crossing of copies 3 and 4 half-steps towards a transient vertex 3e-4 wide, 6.7e-4 away, and the cubic through its uneven march is 1.7e-4 wide; 6 or 30 holes build valid"]
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
/// exactly. The join failed: "loops_contain: could not find a ray
/// direction clear of every vertex", tracing from the corner
/// `block(arm,p0,start)` between the block's side `block(arm,c3)` and the
/// copy's bottom. The corrector clamped the trial step onto the edge of the
/// bottom's domain, `u = 0`, while the bottom's side runs at `u = 1.1e-309`:
/// a cap's patch is the box of its profile rounded outwards, and an exact
/// zero rounded outwards was `MIN_POSITIVE`, `2.2e-308`, not the next
/// number. Every ray from the step met that side at a parameter it could
/// not tell from zero, since every value near zero carried `2.2e-308` of
/// rounding. Zero now rounds to the next number, as every other value does.
#[test]
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

/// A sphere of radius 1 drilled through, scaled up ten thousand times. The
/// curves the hole traced on the sphere are fitted to within `1.7e-6`, and
/// that is honest: scaled, they are within `0.017` of the curves of the
/// scaled solid, far past the `1e-4` an entity may carry, which is no
/// fraction of the part's size. The kernel's budget is absolute, so the
/// scale is refused, naming what it would make too wide and what to do —
/// as a block scaled so small its edges are shorter than the searches
/// resolve is. A sphere not drilled scales up valid: its enclosures are
/// sharp.
#[test]
fn a_drilled_sphere_scaled_ten_thousand_times() {
    use crate::{ScaleBody, ScaleBodyArgs};
    let registry = || registry()?.with(ScaleBody);
    let scaled = |program: &Program, solid: &str, factor: f64| {
        let mut program = program.clone();
        program.push(
            "scaled",
            ScaleBodyArgs {
                bodies: vec![body(solid)],
                about: [0.3, 0.2, 0.1],
                factors: [factor; 3],
            },
        );
        program.build_model(registry)
    };
    let mut ball = Program::new();
    ball.push("ball", PrimitiveArgs::sphere([0.0; 3], 1.0));
    assert_valid(&scaled(&ball, "sphere(ball)", 1e4).unwrap());
    let mut drilled = ball;
    drilled.push(
        "hole",
        PrimitiveArgs::cylinder_z([0.0, 0.0, -2.0], 0.3, 4.0).cut("sphere(ball)"),
    );
    let error = scaled(&drilled, "cylinder(hole)", 1e4).unwrap_err().to_string();
    assert!(error.contains("not as precise as the kernel needs"), "{error}");
    assert!(error.contains("wide"), "{error}");
    let mut block = Program::new();
    block.push("b", PrimitiveArgs::block([0.0; 3], [0.5; 3]));
    let error = scaled(&block, "block(b)", 1e-4).unwrap_err().to_string();
    assert!(error.contains("shorter than"), "{error}");
}
