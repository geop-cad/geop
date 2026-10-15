//! Patterns, mirrors and moves of bodies built by programs: holes drilled
//! by patterning a tool body, symmetric parts mirrored from a half, names
//! of the copies, and sweeps over counts and awkward angles.

use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
    values::Design,
};
use std::f64::consts::PI;

use crate::{
    BodyRef, CircularPattern, CircularPatternArgs, Direction, LinearPattern, LinearPatternArgs,
    Mirror, MirrorArgs, MoveBody, MoveBodyArgs, PatternDirection, Seeds, Spacing, TurnAxis, body,
};
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_ops_booleans::{Boolean, Combine, solid};
use geop_ops_extrude_revolve::{
    Extents,
    testing::{Built, both_ways, disc, extrude_sketch, n, on_origin, outline, sketch},
};
use geop_ops_model::{
    NamedModel, model,
    testing::{assert_valid, assert_volume},
};
use geop_ops_sketch::{Sketch, SketchPlane, sketch_element};

use geop_ops_datums::{origin_axis, origin_plane};
use geop_ops_sketch3d::AddSketch3d;

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    (geop_ops_extrude_revolve::testing::registry()?.with(AddSketch3d)?)
        .with(Boolean)?
        .with(LinearPattern)?
        .with(CircularPattern)?
        .with(Mirror)?
        .with(MoveBody)
}

/// A sketch on the Z plane, and `name` its extrude, as `extent` says, a
/// new body.
fn extruded(program: &mut Program, name: &str, sketch: Sketch, extent: Extents) {
    let plane = on_origin(FrameAxis::Z);
    extrude_sketch(program, name, plane, sketch, extent, Combine::NewBody);
}

fn bodies(name: &str) -> Vec<BodyRef> {
    vec![body(name)]
}

/// Along the origin's axis `axis`.
fn origin_direction(axis: FrameAxis) -> Option<PatternDirection> {
    Some(PatternDirection::Datum(origin_axis(axis)))
}

/// About the origin's axis `axis`.
fn origin_turn(axis: FrameAxis) -> Option<TurnAxis> {
    Some(TurnAxis::Datum(origin_axis(axis)))
}

/// In the origin's plane normal to `normal`.
fn origin_mirror(normal: FrameAxis) -> Option<SketchPlane> {
    Some(SketchPlane::Datum(origin_plane(normal)))
}

/// A plate 8 x 2 x 0.5, and a pin of radius 0.3 at (1, 1) through it — a
/// tool kept as a body of its own.
fn plate_and_pin() -> Program {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        outline(&[[0.0, 0.0], [8.0, 0.0], [8.0, 2.0], [0.0, 2.0]]),
        Extents::blind(0.5),
    );
    let through = both_ways(2.0);
    extruded(&mut program, "pin", disc([1.0, 1.0], 0.3), through);
    program
}

/// Builds `program`, with every step read back from its JSON first: what
/// a program file holds builds the same part.
fn build(program: &Program) -> NamedModel<Design> {
    let json = serde_json::to_string(program).unwrap();
    let read: Program = serde_json::from_str(&json).unwrap();
    assert_eq!(&read, program);
    read.build_model(registry).unwrap()
}

/// The pin patterned four times two apart and cut from the plate — itself
/// and every copy: a row of four holes through it.
#[test]
fn row_of_holes_cut_by_a_patterned_tool() {
    let mut program = plate_and_pin();
    program.push(
        "holes",
        LinearPatternArgs {
            seeds: Seeds::bodies(bodies("extrude(pin)"), Combine::cut("extrude(plate)")),
            first: Direction {
                along: origin_direction(FrameAxis::X),
                reversed: false,
                count: 4.0.into(),
                spacing: Spacing::step(2.0),
            },
            second: None,
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(holes)"]);
    let expected = 8.0 * 2.0 * 0.5 - 4.0 * PI * 0.3 * 0.3 * 0.5;
    assert_volume(&part, "linear_pattern(holes)", expected, 1e-3);
    // The wall of the third copy's hole is the pin's side, copied — cut
    // where the plate's faces cross it, and named by the cut after it.
    let wall = part.names().iter().any(|(id, name)| {
        matches!(id, geop_ops_model::RefId::Face(_))
            && name.starts_with("combine(holes,3,linear_pattern(holes,3,extrude(pin,pin_sketch,c1")
    });
    assert!(wall, "no face of the third hole");
}

/// The pin turned six times around a hole's axis through the middle of a
/// square plate: a bolt circle.
#[test]
fn bolt_circle_cut_by_a_patterned_tool() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "plate",
        outline(&[[-3.0, -3.0], [3.0, -3.0], [3.0, 3.0], [-3.0, 3.0]]),
        Extents::blind(0.5),
    );
    let through = both_ways(2.0);
    extruded(&mut program, "pin", disc([2.0, 0.0], 0.3), through);
    program.push(
        "bolts",
        CircularPatternArgs {
            seeds: Seeds::bodies(bodies("extrude(pin)"), Combine::cut("extrude(plate)")),
            axis: origin_turn(FrameAxis::Z),
            reversed: false,
            count: 6.0.into(),
            angle: Spacing::extent(360.0),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let expected = 36.0 * 0.5 - 6.0 * PI * 0.3 * 0.3 * 0.5;
    assert_volume(&part, "circular_pattern(bolts)", expected, 1e-3);
}

/// An L standing on the `yz` plane, mirrored in it and joined to itself: a
/// symmetric T, twice the L.
#[test]
fn half_mirrored_and_joined() {
    let mut program = Program::new();
    let l = [
        [0.0, 0.0],
        [2.0, 0.0],
        [2.0, 0.5],
        [0.5, 0.5],
        [0.5, 3.0],
        [0.0, 3.0],
    ];
    extruded(&mut program, "half", outline(&l), Extents::blind(1.0));
    program.push(
        "m",
        MirrorArgs {
            seeds: Seeds::bodies(bodies("extrude(half)"), Combine::join("extrude(half)")),
            plane: origin_mirror(FrameAxis::X),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["mirror(m)"]);
    assert_volume(&part, "mirror(m)", 2.0 * 2.25, 1e-6);
}

/// An L away from the plane, mirrored as a new body: the image's corners
/// are the L's mirrored — its foot pointing the other way — and it faces
/// out.
#[test]
fn asymmetric_body_mirrored() {
    let mut program = Program::new();
    let l = [
        [1.0, 0.0],
        [3.0, 0.0],
        [3.0, 0.5],
        [1.5, 0.5],
        [1.5, 3.0],
        [1.0, 3.0],
    ];
    extruded(&mut program, "l", outline(&l), Extents::blind(1.0));
    program.push(
        "m",
        MirrorArgs {
            seeds: Seeds::bodies(bodies("extrude(l)"), Combine::NewBody),
            plane: origin_mirror(FrameAxis::X),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let image = part.solid_id("mirror(m,extrude(l))").unwrap();
    let model = part.topology();
    let mut corners: Vec<[i64; 3]> = model
        .iter_body_vertices(image)
        .unwrap()
        .map(|v| {
            let p = model.get_vertex(v).unwrap().point;
            [0, 1, 2].map(|k| (p[k].to_f64() * 2.0).round() as i64)
        })
        .collect();
    corners.sort();
    let mut expected: Vec<[i64; 3]> = l
        .iter()
        .flat_map(|c| [0, 2].map(|z| [(-c[0] * 2.0) as i64, (c[1] * 2.0) as i64, z]))
        .collect();
    expected.sort();
    assert_eq!(corners, expected);
    assert_volume(&part, "mirror(m,extrude(l))", 2.25, 1e-6);
}

/// A pin moved into place and cut from the plate, rather than drawn
/// there: the plate with one hole at (5, 1).
#[test]
fn tool_moved_into_place_and_cut() {
    let mut program = plate_and_pin();
    program.push(
        "mv",
        MoveBodyArgs {
            bodies: bodies("extrude(pin)"),
            translation: [4.0, 0.0, 0.0],
            axis: None,
            angle: 0.0,
            copy: false,
            combine: Combine::cut("extrude(plate)"),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let expected = 8.0 - PI * 0.09 * 0.5;
    assert_volume(&part, "move(mv)", expected, 1e-3);
}

/// A circle of boxes, up to 24 of them, at awkward angles, each valid,
/// and joined where they overlap.
#[test]
#[ignore = "slow: circular patterns of up to 24 copies at awkward angles — run with `cargo test -- --ignored`"]
fn circular_patterns_sweep() {
    for (count, angle, joined) in [
        (24, Spacing::extent(360.0), false),
        (7, Spacing::step(37.3), false),
        (5, Spacing::extent(-251.7), false),
        (2, Spacing::step(13.0), true),
        (12, Spacing::extent(360.0), true),
    ] {
        let mut program = Program::new();
        extruded(
            &mut program,
            "b",
            outline(&[[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]),
            Extents::blind(1.0),
        );
        program.push(
            "r",
            CircularPatternArgs {
                seeds: Seeds::bodies(
                    bodies("extrude(b)"),
                    if joined {
                        Combine::join("extrude(b)")
                    } else {
                        Combine::NewBody
                    },
                ),
                axis: origin_turn(FrameAxis::Z),
                reversed: false,
                count: (count as f64).into(),
                angle: angle.clone(),
            },
        );
        let part = program
            .build_model(registry)
            .unwrap_or_else(|e| panic!("{count} at {angle:?}: {e:?}"));
        assert_valid(&part);
        let solids = part.solid_names();
        assert_eq!(solids.len(), if joined { 1 } else { count }, "{solids:?}");
        if !joined {
            for name in &solids {
                assert_volume(&part, name, 0.8, 1e-6);
            }
        }
    }
}

/// Three boxes, each turned 13° further about `z` than the last, drawn
/// where they are rather than patterned, and joined one after the other —
/// what a circular pattern joined to its seed does.
#[test]
fn three_turned_boxes_joined() {
    let mut program = Program::new();
    for (k, name) in ["a", "b", "c"].into_iter().enumerate() {
        let (s, c) = (13.0 * k as f64).to_radians().sin_cos();
        let corners: Vec<[f64; 2]> = [[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]
            .iter()
            .map(|p| [c * p[0] - s * p[1], s * p[0] + c * p[1]])
            .collect();
        extruded(&mut program, name, outline(&corners), Extents::blind(1.0));
    }
    program.push(
        "ab",
        geop_ops_booleans::BooleanArgs {
            a: solid("extrude(a)"),
            b: solid("extrude(b)"),
            op: geop_ops_model::BooleanOp::Union,
        },
    );
    program.push(
        "abc",
        geop_ops_booleans::BooleanArgs {
            a: solid("boolean(ab)"),
            b: solid("extrude(c)"),
            op: geop_ops_model::BooleanOp::Union,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
}

/// The same three boxes, the last a copy of the first turned 26°.
///
/// The second join, (a ∪ b) ∪ c, once failed imprinting c's bottom edge
/// onto b's bottom cap. b's inner top edge lies in c's top plane and ends
/// on c's inner side; the piercing search met c's cap only at that end
/// vertex, and its last box, 1.4e-4 short of the end and 1.4e-5 outside
/// the cap, was split at as a crossing — Newton cannot refine a curve
/// lying in the surface's tangent plane. That left vertices 5e-5 wide, and
/// a pcurve through one too wide for the imprint to tell its ring's area
/// from zero. Drawn boxes passed only because their caps' patches extend
/// past the edge: there it lay on the patch and was imprinted. Only a
/// transversal crossing is now a piercing.
#[test]
fn three_turned_boxes_one_a_turned_copy_joined() {
    let mut program = Program::new();
    for (k, name) in ["a", "b"].into_iter().enumerate() {
        let (s, c) = (13.0 * k as f64).to_radians().sin_cos();
        let corners: Vec<[f64; 2]> = [[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]
            .iter()
            .map(|p| [c * p[0] - s * p[1], s * p[0] + c * p[1]])
            .collect();
        extruded(&mut program, name, outline(&corners), Extents::blind(1.0));
    }
    program.push(
        "c",
        MoveBodyArgs {
            bodies: bodies("extrude(a)"),
            translation: [0.0; 3],
            axis: origin_turn(FrameAxis::Z),
            angle: 26.0,
            copy: true,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "ab",
        geop_ops_booleans::BooleanArgs {
            a: solid("extrude(a)"),
            b: solid("extrude(b)"),
            op: geop_ops_model::BooleanOp::Union,
        },
    );
    program.push(
        "abc",
        geop_ops_booleans::BooleanArgs {
            a: solid("boolean(ab)"),
            b: solid("move(c,extrude(a))"),
            op: geop_ops_model::BooleanOp::Union,
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let (width, edge) = widest_pcurve(&part);
    assert!(width < 1e-9, "{edge}: {width:e}");
}

/// The widest control point of any pcurve of `part`, and the edge it
/// belongs to.
fn widest_pcurve(part: &NamedModel<Design>) -> (f64, String) {
    let model = part.topology();
    let mut widest = (0.0, String::new());
    for coedge in model.coedges.values() {
        let width = coedge
            .pcurve
            .control_points
            .iter()
            .flat_map(|p| [p[0], p[1], p[2]])
            .map(|c| c.width().to_f64())
            .fold(0.0, f64::max);
        if width > widest.0 {
            let name = match coedge.geometry {
                geop_core_topology::CoedgeGeometry::Edge(e) => {
                    part.name_of(e).unwrap_or("?").to_string()
                }
                geop_core_topology::CoedgeGeometry::Vertex(v) => {
                    part.name_of(v).unwrap_or("?").to_string()
                }
            };
            widest = (width, name);
        }
    }
    widest
}

/// A box joined to a copy of itself turned 13° about `z`: the edges the
/// join splits keep pcurves as tight as the same join of two boxes drawn
/// where they are.
#[test]
fn turned_copy_joined_stays_tight() {
    let mut program = Program::new();
    extruded(
        &mut program,
        "b",
        outline(&[[2.0, -0.4], [3.0, -0.4], [3.0, 0.4], [2.0, 0.4]]),
        Extents::blind(1.0),
    );
    program.push(
        "r",
        CircularPatternArgs {
            seeds: Seeds::bodies(bodies("extrude(b)"), Combine::join("extrude(b)")),
            axis: origin_turn(FrameAxis::Z),
            reversed: false,
            count: 2.0.into(),
            angle: Spacing::step(13.0),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let (width, edge) = widest_pcurve(&part);
    assert!(width < 1e-9, "{edge}: {width:e}");
}

/// Rows and grids of up to 24 holes cut from a long plate by a patterned
/// pin, and copies of the plate turned at awkward angles about one of its
/// edges and shifted: every result valid, holding what it should.
#[test]
#[ignore = "slow: rows and grids of up to 24 holes, copies turned at awkward angles — run with `cargo test -- --ignored`"]
fn linear_patterns_and_moves_sweep() {
    for (along, across) in [(24, 1), (6, 4), (3, 3)] {
        let mut program = Program::new();
        extruded(
            &mut program,
            "plate",
            outline(&[[0.0, 0.0], [30.0, 0.0], [30.0, 6.0], [0.0, 6.0]]),
            Extents::blind(0.5),
        );
        let through = both_ways(2.0);
        extruded(&mut program, "pin", disc([1.0, 1.0], 0.3), through);
        program.push(
            "holes",
            LinearPatternArgs {
                seeds: Seeds::bodies(bodies("extrude(pin)"), Combine::cut("extrude(plate)")),
                first: Direction {
                    along: origin_direction(FrameAxis::X),
                    reversed: false,
                    count: (along as f64).into(),
                    spacing: Spacing::step(1.2),
                },
                second: (across > 1).then(|| Direction {
                    along: origin_direction(FrameAxis::Y),
                    reversed: false,
                    count: (across as f64).into(),
                    spacing: Spacing::extent(1.3 * (across - 1) as f64),
                }),
            },
        );
        let part = program
            .build_model(registry)
            .unwrap_or_else(|e| panic!("{along} x {across}: {e:?}"));
        assert_valid(&part);
        let holes = (along * across) as f64;
        let expected = 30.0 * 6.0 * 0.5 - holes * PI * 0.09 * 0.5;
        assert_volume(&part, "linear_pattern(holes)", expected, 1e-2);
    }
    for degrees in [1.0, 17.3, 89.99, 133.7, 271.1] {
        let mut program = plate_and_pin();
        program.push(
            "mv",
            MoveBodyArgs {
                bodies: bodies("extrude(plate)"),
                translation: [0.3, -1.7, 2.2],
                axis: Some(TurnAxis::Edge(ElementReference::new(
                    model(),
                    "extrude(plate,plate_sketch,p0)",
                ))),
                angle: degrees,
                copy: true,
                combine: Combine::NewBody,
            },
        );
        let part = program
            .build_model(registry)
            .unwrap_or_else(|e| panic!("{degrees}°: {e:?}"));
        assert_valid(&part);
        assert_volume(&part, "move(mv,extrude(plate))", 8.0, 1e-6);
    }
}

/// The pin patterned along a 3-D sketch's line — drawn beside the plate,
/// along `x` — and along a planar sketch's line across it: a grid of holes,
/// as along the origin's axes. The sketch's lines are picked as any
/// straight edge is.
#[test]
fn holes_patterned_along_sketch_lines() {
    let mut program = plate_and_pin();
    let mut route = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| geop_core_math::vector::Vector3::from_array(p.map(n));
    let (a, b) = (
        route.add_point(at([0.0, 3.0, 0.0])),
        route.add_point(at([1.0, 3.0, 0.0])),
    );
    route.add_line(a, b);
    program.push(
        "rail",
        geop_ops_sketch3d::AddSketch3dArgs {
            sketch: route,
            references: Vec::new(),
        },
    );
    let mut across = Sketch::new();
    let (a, b) = (
        across.add_point(n(-1.0), n(0.0)),
        across.add_point(n(-1.0), n(1.0)),
    );
    let line = across.add_line(a, b);
    program.push("across", sketch(on_origin(FrameAxis::Z), across));
    program.push(
        "holes",
        LinearPatternArgs {
            seeds: Seeds::bodies(bodies("extrude(pin)"), Combine::cut("extrude(plate)")),
            first: Direction {
                along: Some(PatternDirection::Edge(ElementReference::new(
                    model(),
                    "sketch3d(rail,c2)",
                ))),
                reversed: false,
                count: 4.0.into(),
                spacing: Spacing::step(2.0),
            },
            second: Some(Direction {
                along: Some(PatternDirection::Sketch(sketch_element("across", line))),
                reversed: true,
                count: 2.0.into(),
                spacing: Spacing::step(0.65),
            }),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let expected = 8.0 * 2.0 * 0.5 - 8.0 * PI * 0.3 * 0.3 * 0.5;
    assert_volume(&part, "linear_pattern(holes)", expected, 1e-3);
}
