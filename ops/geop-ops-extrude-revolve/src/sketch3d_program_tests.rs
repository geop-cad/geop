//! Sweeps along 3-D sketches, run as programs: the solids they build, and
//! what they refuse.

use crate::{
    Output, PathRef, SweepArgs,
    testing::{Built, disc, n, on_origin, sketch},
};
use geop_core_math::{
    geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar, vector::Vector3,
};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_core_sketch::space::End;
use geop_lib_sweep::Orientation;
use geop_ops_model::Combine;
use geop_ops_model::testing::assert_valid;
use geop_ops_sketch::{AddSketchArgs, whole_sketch};
use geop_ops_sketch3d::{AddSketch3d, AddSketch3dArgs, Constraint3d, Sketch3d, sketch3d_path};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    crate::testing::registry()?.with(AddSketch3d)
}

fn v(p: [f64; 3]) -> Vector3<geop_core_program::values::Design> {
    Vector3::from_array(p.map(n))
}

/// A circle of radius `r` around the origin, on the origin's `x` plane.
fn section(r: f64) -> AddSketchArgs {
    sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], r))
}

/// The spline through `points`, leaving the first along `x`.
fn spline(points: &[[f64; 3]]) -> Sketch3d {
    let mut s = Sketch3d::new();
    let p = points.iter().map(|&p| s.add_point(v(p))).collect();
    let curve = s.add_spline(p);
    s.constrain(Constraint3d::TangentTo {
        curve,
        end: End::Start,
        direction: v([1.0, 0.0, 0.0]),
    });
    s
}

/// The circle `section(r)` swept along the 3-D sketch `route`.
fn piped(r: f64, route: Sketch3d) -> Program {
    let mut program = Program::new();
    program.push("section", section(r));
    program.push(
        "route",
        AddSketch3dArgs {
            sketch: route,
            references: Vec::new(),
        },
    );
    program.push(
        "pipe",
        SweepArgs {
            profile: whole_sketch("section"),
            path: Some(PathRef::Sketch3d(sketch3d_path("route"))),
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// Along a spline through space: one wall per quarter of the circle, two
/// caps, the far one around the spline's end, square to it.
#[test]
fn sweep_along_a_3d_spline() {
    let end = [5.0, 0.0, 3.0];
    let program = piped(
        0.2,
        spline(&[[0.0, 0.0, 0.0], [2.0, 0.5, 0.5], [4.0, -0.5, 1.5], end]),
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 4 + 2);
    let model = part.topology();
    let cap = part.face_id("sweep(pipe,end)").unwrap();
    // The frames along a spline are a free choice in plain numbers (see
    // `path_sweep`), so the cap is where they put it, to their precision.
    for c in model.iterate_face_coedges(cap) {
        let p = model.coedge_start_vertex(c).unwrap().point;
        let r = (0..3)
            .map(|k| (p[k].to_f64() - end[k]).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!((r - 0.2).abs() < 1e-6, "{p:?} is {r} from the end");
    }
}

/// A 3-D sketch of two chains is no path: refused, saying so.
#[test]
fn a_path_of_two_chains_is_refused() {
    let mut route = Sketch3d::new();
    for x in [0.0, 3.0] {
        let a = route.add_point(v([x, 0.0, 0.0]));
        let b = route.add_point(v([x + 1.0, 0.0, 0.0]));
        route.add_line(a, b);
    }
    let error = piped(0.2, route)
        .build_model(registry)
        .expect_err("refused");
    assert!(error.root_message().contains("2 chains"), "{error:?}");
}

/// Sweeps along splines bending gently and sharply, in and out of a plane,
/// each piped with circles from thin to as thick as the tightest bend
/// allows: every one builds a valid solid. (Polylines with bends of
/// several radii are swept by the example `pipe`'s tests.)
#[test]
#[ignore = "slow: sweeps along paths of varied curvature — run with `cargo test -- --ignored`"]
fn sweeps_along_splines_of_varied_curvature() {
    let splines: [&[[f64; 3]]; 4] = [
        &[[0.0, 0.0, 0.0], [3.0, 0.2, 0.1], [6.0, 0.0, 0.3]],
        &[
            [0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [3.0, 3.0, 0.0],
            [2.0, 5.0, 0.0],
        ],
        &[
            [0.0, 0.0, 0.0],
            [2.0, 2.0, 1.0],
            [0.0, 4.0, 2.0],
            [-2.0, 2.0, 3.0],
            [0.0, 0.0, 4.0],
        ],
        &[
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 1.0],
            [1.5, 1.0, 1.5],
            [1.0, 2.0, 1.0],
            [2.0, 3.0, 0.0],
        ],
    ];
    for (i, points) in splines.iter().enumerate() {
        for r in [0.05, 0.15, 0.3] {
            let built = piped(r, spline(points)).build_model(registry);
            match built {
                Ok(part) => assert_valid(&part),
                Err(e) => panic!("spline {i}, radius {r}: {e:?}"),
            }
        }
    }
}
