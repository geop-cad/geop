//! 3-D sketches drawn the way the surfacing tutorials draw them, built and
//! checked as bodies.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_ops_model::sketch3d::Sketch3d;
use geop_ops_model::testing::{Built, assert_valid_sheets};

use crate::{AddSketch3d, AddSketch3dArgs};

fn registry() -> GeopResult<OperationRegistry> {
    OperationRegistry::default().with(AddSketch3d)
}

fn v(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

/// The network of curves a UV surface is spanned through, drawn as one 3-D
/// sketch, as the tutorials draw it: a spline along `x` and a spline along
/// `y`, each through its own points, crossing at `(1, 1, 0)` — a point
/// both pass through but neither shares with the other — and a third curve
/// starting where the first ends, at `(0, 1, 0)`, from a point of its own.
///
/// The wire built from it is invalid: `edge sketch3d(net,c3) and edge
/// sketch3d(net,c7) intersect at a point that is not an existing vertex`,
/// and, for the third curve, `vertex sketch3d(net,p0) and vertex
/// sketch3d(net,p8) could overlap — they should be the same vertex, or
/// genuinely disjoint`. Nothing of the sketch is wrong: a 3-D sketch's
/// curves may cross, and they cross exactly where both were drawn through.
/// The body built from it should share the vertices and split the curves
/// where they cross, or a wire's curves should be allowed to. The
/// `curved_panel` example avoids both by drawing the ribs and the spines in
/// two sketches, each curve through points of its own; a user drawing a
/// network in one sketch gets an invalid part without being told, and
/// `assert_valid` passes it, since it checks solids.
#[test]
#[ignore = "bug: a 3-D sketch of splines crossing at a drawn point, and a curve starting at another's end from a point of its own, builds an invalid wire: sketch3d(net,c3) and c7 cross at no vertex, p0 and p8 could overlap"]
fn curves_crossing_at_a_drawn_point_build_a_valid_wire() {
    let mut s = Sketch3d::new();
    let along_x: Vec<_> = [0.0, 1.0, 2.0]
        .iter()
        .map(|&x| s.add_point(v([x, 1.0, 0.0])))
        .collect();
    s.add_spline(along_x);
    let along_y: Vec<_> = [0.0, 1.0, 2.0]
        .iter()
        .map(|&y| s.add_point(v([1.0, y, 0.0])))
        .collect();
    s.add_spline(along_y);
    let a = s.add_point(v([0.0, 1.0, 0.0]));
    let b = s.add_point(v([0.0, 2.0, 0.0]));
    s.add_line(a, b);
    let mut program = Program::new();
    program.push(
        "net",
        AddSketch3dArgs {
            sketch: s,
            references: Vec::new(),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid_sheets(&part);
}
