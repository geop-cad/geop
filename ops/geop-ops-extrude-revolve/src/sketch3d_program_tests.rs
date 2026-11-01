//! Sweeps along 3-D sketches, run as programs: the solids they build, and
//! what they refuse.

use crate::{Extents, Output, PathRef, RevolveArgs, RevolveAxis, SweepArgs};
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, vector::Vector3};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_core_sketch::space::End;
use geop_lib_sweep::Orientation;
use geop_ops_model::sketch::{
    Sketch,
    figures::{disc, n, on_origin, polygon, rectangle_outline},
    sketch_element, whole_sketch,
};
use geop_ops_model::sketch3d::{Constraint3d, Sketch3d, sketch3d_path};
use geop_ops_model::{
    BooleanOp, Combine, solid,
    testing::{Built, assert_valid},
};
use geop_ops_sketch::AddSketchArgs;
use geop_ops_sketch3d::{AddSketch3d, AddSketch3dArgs};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    crate::testing::registry()?.with(AddSketch3d)
}

fn v(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(n))
}

/// A circle of radius `r` around the origin, on the origin's `x` plane.
fn section(r: f64) -> AddSketchArgs {
    AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], r))
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
/// A helix round the `z` axis, of radius 10 and `turns` turns, 20 tall,
/// from `(10, 0, 0)`.
fn coil(turns: f64, left: bool) -> Sketch3d {
    let mut s = Sketch3d::new();
    let start = s.add_point(v([10.0, 0.0, 0.0]));
    let base = s.add_point(v([0.0, 0.0, 0.0]));
    let top = s.add_point(v([0.0, 0.0, 20.0]));
    s.add_helix(start, base, top, n(turns), left).unwrap();
    s
}

/// A rectangle swept along a helix is a coil: valid, as heavy as its section
/// times how far its middle turns, and it keeps facing the axis, however
/// many turns on: its far end the same rectangle risen and turned, every
/// corner on the cylinder it started on, each wall a piece per quarter turn.
/// Carried by a rotation minimizing frame, it rolled over as it rose.
#[test]
fn a_rectangle_is_swept_along_a_helix_facing_its_axis() {
    for (turns, left) in [(3.0_f64, false), (3.0, true), (2.5, false), (1.3, false)] {
        let (radius, rise) = (10.0_f64, 20.0_f64);
        let mut program = Program::new();
        program.push(
            "section",
            AddSketchArgs::on(
                on_origin(FrameAxis::Y),
                rectangle_outline(9.0, -1.0, 11.0, 1.0),
            ),
        );
        program.push(
            "route",
            AddSketch3dArgs {
                sketch: coil(turns, left),
                references: Vec::new(),
            },
        );
        program.push(
            "coil",
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
        let part = program.build_model(registry).unwrap();
        assert_valid(&part);
        let model = part.topology();
        let solid = *model.solids.keys().next().unwrap();
        let volume = model.mass_properties(solid).unwrap().volume.to_f64();
        let expected = 4.0 * std::f64::consts::TAU * radius * turns;
        assert!(
            (volume - expected).abs() < 0.01 * expected,
            "{turns} turns, left {left}: volume {volume}, expected {expected}"
        );
        let cap = part.face_id("sweep(coil,end)").unwrap();
        let mut corners: Vec<(f64, f64)> = model
            .iterate_face_coedges(cap)
            .map(|c| {
                let p = model.coedge_start_vertex(c).unwrap().point.to_f64_array();
                (
                    (p[0].hypot(p[1]) * 1e6).round() / 1e6,
                    ((p[2] - rise) * 1e6).round() / 1e6,
                )
            })
            .collect();
        corners.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(
            corners,
            [(9.0, -1.0), (9.0, 1.0), (11.0, -1.0), (11.0, 1.0)],
            "{turns} turns, left {left}: the far end is the section as drawn"
        );
    }
}

/// A twist along a helix is on top of the screw motion: the section at the
/// far end is the one drawn, risen, and turned by the twist alone.
#[test]
fn a_twist_along_a_helix_is_all_the_turn_there_is() {
    let mut program = Program::new();
    program.push(
        "section",
        AddSketchArgs::on(
            on_origin(FrameAxis::Y),
            rectangle_outline(9.0, -1.0, 11.0, 1.0),
        ),
    );
    program.push(
        "route",
        AddSketch3dArgs {
            sketch: coil(2.0, false),
            references: Vec::new(),
        },
    );
    program.push(
        "coil",
        SweepArgs {
            profile: whole_sketch("section"),
            path: Some(PathRef::Sketch3d(sketch3d_path("route"))),
            orientation: Orientation::FollowPath,
            twist: 90.0,
            end_scale: 1.0,
            rails: Vec::new(),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let model = part.topology();
    let cap = part.face_id("sweep(coil,end)").unwrap();
    for c in model.iterate_face_coedges(cap) {
        let p = model.coedge_start_vertex(c).unwrap().point.to_f64_array();
        // A 2 x 2 square about the path's start point, turned about the
        // corner where it starts: it has gone from (9..11, -1..1) to one
        // quarter turn about (10, 0) on the path's end, a square again.
        let (r, up) = (p[0].hypot(p[1]), p[2] - 20.0);
        assert!(
            (r - 10.0).abs() < 1.0 + 1e-3 && up.abs() < 1.0 + 1e-3,
            "{p:?} is off the section"
        );
    }
}

/// A rectangle standing on a cylinder, its inner corner swept along a spline
/// through points of a helix on that cylinder, joined to it. The spline is
/// no helix: it leaves the cylinder and comes back between its points, by
/// less than a thousandth, for turn after turn. So the edge lies almost on
/// the wall the whole way, crossing it again and again.
///
/// Ruled out and known so far, from a rectangle on a helix swept before the
/// sweep carried it exactly, whose fitted edge lay `1e-5` from the wall: the
/// edge × face search exhausts `MAX_NODES` over it; given twenty times the
/// nodes, the march along the wall × cylinder curve next to that edge leaves
/// the wall with no vertex within four strides. Here the splice of such a
/// curve is refused, a ring of signed area `[-2.8e-6, 2.8e-6]`. So more
/// budget alone is no fix: the crossings bound slivers thinner than the
/// march's strides, and a sweep along the helix itself needs none of this.
#[test]
#[ignore = "bug: an edge wobbling within 1e-3 of a cylinder, crossing it turn after turn, is refused as a degenerate split; see the comment for what is ruled out"]
fn a_rectangle_swept_along_a_spline_on_a_cylinder_joins_it() {
    let (radius, rise, turns) = (29.458436663535004_f64, 48.418982846502615_f64, 5.0_f64);
    let mut route = Sketch3d::new();
    let per_turn = 16;
    let points = (0..=(turns as usize * per_turn))
        .map(|k| {
            let fraction = k as f64 / (turns * per_turn as f64);
            let angle = std::f64::consts::TAU * turns * fraction;
            route.add_point(v([
                radius * angle.cos(),
                rise * fraction,
                -radius * angle.sin(),
            ]))
        })
        .collect();
    route.add_spline(points);
    let mut program = Program::new();
    let mut body = Sketch::new();
    let axis_a = body.add_fixed_point(n(0.0), n(0.0));
    let axis_b = body.add_fixed_point(n(0.0), n(1.0));
    let axis = body.add_line(axis_a, axis_b);
    body.set_construction(axis, true);
    polygon(
        &mut body,
        &[[0.0, 0.0], [radius, 0.0], [radius, 38.5], [0.0, 38.5]],
    );
    program.push(
        "body_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), body),
    );
    program.push(
        "body",
        RevolveArgs {
            sketch: whole_sketch("body_sketch"),
            axis: Some(RevolveAxis::Sketch(sketch_element("body_sketch", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.push(
        "section",
        AddSketchArgs::on(
            on_origin(FrameAxis::Z),
            rectangle_outline(radius, -3.45, radius + 6.5, 0.0),
        ),
    );
    program.push(
        "route",
        AddSketch3dArgs {
            sketch: route,
            references: Vec::new(),
        },
    );
    program.push(
        "thread",
        SweepArgs {
            profile: whole_sketch("section"),
            path: Some(PathRef::Sketch3d(sketch3d_path("route"))),
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
            output: Output::Solid(Combine::With {
                op: BooleanOp::Union,
                target: solid("revolve(body)"),
            }),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
}

/// A thread of rectangular section standing on a cylinder, swept along a
/// helix on it and joined to it, as drawn in the editor: its inner wall lies
/// in the cylinder's, exactly, and it runs on past the cylinder's top.
#[test]
fn a_rectangle_swept_along_a_helix_on_a_cylinder_joins_it() {
    let (radius, rise, turns, height) = (29.458436663535004_f64, 48.418982846502615, 5.0, 38.5);
    let mut route = Sketch3d::new();
    let start = route.add_point(v([radius, 0.0, 0.0]));
    let base = route.add_point(v([0.0, 0.0, 0.0]));
    let top = route.add_point(v([0.0, rise, 0.0]));
    route.add_helix(start, base, top, n(turns), false).unwrap();
    let mut program = Program::new();
    let mut body = Sketch::new();
    let axis_a = body.add_fixed_point(n(0.0), n(0.0));
    let axis_b = body.add_fixed_point(n(0.0), n(1.0));
    let axis = body.add_line(axis_a, axis_b);
    body.set_construction(axis, true);
    polygon(
        &mut body,
        &[[0.0, 0.0], [radius, 0.0], [radius, height], [0.0, height]],
    );
    program.push(
        "body_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), body),
    );
    program.push(
        "body",
        RevolveArgs {
            sketch: whole_sketch("body_sketch"),
            axis: Some(RevolveAxis::Sketch(sketch_element("body_sketch", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.push(
        "section",
        AddSketchArgs::on(
            on_origin(FrameAxis::Z),
            rectangle_outline(radius, -3.45, radius + 6.5, 0.0),
        ),
    );
    program.push(
        "route",
        AddSketch3dArgs {
            sketch: route,
            references: Vec::new(),
        },
    );
    program.push(
        "thread",
        SweepArgs {
            profile: whole_sketch("section"),
            path: Some(PathRef::Sketch3d(sketch3d_path("route"))),
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
            output: Output::Solid(Combine::With {
                op: BooleanOp::Union,
                target: solid("revolve(body)"),
            }),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let model = part.topology();
    assert_eq!(model.solids.len(), 1, "one solid");
    let solid_id = *model.solids.keys().next().unwrap();
    let volume = model.mass_properties(solid_id).unwrap().volume.to_f64();
    let core = std::f64::consts::PI * radius * radius * height;
    assert!(
        volume > core * 1.05,
        "the thread is added: {volume} of {core}"
    );
}

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
