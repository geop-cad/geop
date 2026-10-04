//! Sweeps along 3-D sketches, run as programs: the solids they build, and
//! what they refuse.

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_sketch::space::End;
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Orientation, SweepArgs};
use geop_ops_sketch::{AddSketchArgs, Sketch};
use geop_ops_sketch3d::{AddSketch3dArgs, Constraint3d, Sketch3d};

use crate::Program;
use crate::examples::n;

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
}

fn v(p: [f64; 3]) -> Vector3<geop_ops::Design> {
    Vector3::from_array(p.map(n))
}

/// A circle of radius `r` around the origin, on the origin's `x` plane.
fn section(r: f64) -> AddSketchArgs {
    let mut s = Sketch::new();
    let c = s.add_point(n(0.0), n(0.0));
    s.add_circle(c, n(r));
    AddSketchArgs {
        plane: Some(EntityRef::datum_component(
            ORIGIN,
            DatumComponent::Plane(FrameAxis::X),
        )),
        sketch: s,
        ..Default::default()
    }
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
            profile: "section".into(),
            path: "route".into(),
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
            face: false,
            combine: Combine::NewBody,
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
    let part = program.build::<S>(&NoFiles).unwrap();
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
        .build::<S>(&NoFiles)
        .err()
        .expect("refused");
    assert!(error.root_message().contains("2 chains"), "{error:?}");
}

/// A pipe bent tighter than it is thick is refused, naming the bend —
/// rather than built crossing itself.
#[test]
fn a_bend_tighter_than_the_pipe_is_refused() {
    let route = crate::examples::pipe_route(0.25);
    let error = piped(0.3, route)
        .build::<S>(&NoFiles)
        .err()
        .expect("refused");
    assert!(error.root_message().contains("bend"), "{error:?}");
}

/// Sweeps along paths of every kind of curvature: splines bending gently
/// and sharply, in and out of a plane, and polylines with bends of
/// several radii — each piped with circles from thin to as thick as the
/// tightest bend allows, and beyond. Every one builds a valid solid, or is
/// refused for a bend tighter than the pipe.
#[test]
#[ignore = "slow: sweeps along paths of varied curvature — run with `cargo test -- --ignored`"]
fn sweeps_along_paths_of_varied_curvature() {
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
            let built = piped(r, spline(points)).build::<S>(&NoFiles);
            match built {
                Ok(part) => assert_valid(&part),
                Err(e) => panic!("spline {i}, radius {r}: {e:?}"),
            }
        }
    }
    for bend in [0.4, 1.0, 1.5] {
        let route = crate::examples::pipe_route(bend);
        for r in [0.1, 0.3, 0.39, 0.5] {
            match piped(r, route.clone()).build::<S>(&NoFiles) {
                Ok(part) => assert_valid(&part),
                Err(e) if r >= bend && e.root_message().contains("bend") => {}
                Err(e) => panic!("bend {bend}, radius {r}: {e:?}"),
            }
        }
    }
}
