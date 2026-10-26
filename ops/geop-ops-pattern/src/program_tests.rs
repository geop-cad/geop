//! Patterns, mirrors and moves of bodies built by programs: holes drilled
//! by patterning a tool body, symmetric parts mirrored from a half, names
//! of the copies, and sweeps over counts and awkward angles.

use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
};
use std::f64::consts::PI;

use crate::{
    BodyRef, CircularPattern, CircularPatternArgs, Direction, LinearPattern, LinearPatternArgs,
    Mirror, MirrorArgs, MoveBody, MoveBodyArgs, PatternDirection, Seeds, TurnAxis, body,
};
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_lib_fixtures::{Primitive, PrimitiveArgs};
use geop_ops_boolean::Boolean;
use geop_ops_model::sketch::{
    Sketch, SketchPlane,
    figures::{n, on_origin},
    sketch_element,
};
use geop_ops_model::spacing::Spacing;
use geop_ops_model::{
    Combine, NamedModel, model, solid,
    testing::{Built, assert_valid, assert_volume, edge_at},
};
use geop_ops_sketch::AddSketchArgs;

use geop_ops_model::datums::{origin_axis, origin_plane};
use geop_ops_reference_geometry::AddDatum;
use geop_ops_sketch3d::AddSketch3d;

/// The operations these tests' programs are written in: given solids, the
/// sketches a pattern runs along, and the sketches, extrusions, lofts and
/// datums of the examples.
fn registry() -> GeopResult<OperationRegistry> {
    (geop_ops_extrude_revolve::testing::registry()?.with(AddSketch3d)?)
        .with(Primitive)?
        .with(Boolean)?
        .with(LinearPattern)?
        .with(CircularPattern)?
        .with(Mirror)?
        .with(MoveBody)?
        .with(AddDatum)
}

/// The block `block(name)` over `[x0, x1] x [y0, y1]`, `height` up from
/// the Z plane, as the step `name`.
fn block(program: &mut Program, name: &str, [x0, y0]: [f64; 2], [x1, y1]: [f64; 2], height: f64) {
    program.push(name, PrimitiveArgs::block([x0, y0, 0.0], [x1, y1, height]));
}

/// The pin `cylinder(pin)`: a cylinder of radius 0.3 around `(x, y)`
/// through the Z plane, 2 either way — a tool kept as a body of its own.
fn pin(program: &mut Program, [x, y]: [f64; 2]) {
    program.push("pin", PrimitiveArgs::cylinder_z([x, y, -2.0], 0.3, 4.0));
}

/// The prism `prism(name)` over `corners`, 1 up from the Z plane, as the
/// step `name`.
fn prism(program: &mut Program, name: &str, corners: &[[f64; 2]]) {
    program.push(name, PrimitiveArgs::prism(corners, 0.0, 1.0));
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
    block(&mut program, "plate", [0.0, 0.0], [8.0, 2.0], 0.5);
    pin(&mut program, [1.0, 1.0]);
    program
}

/// Builds `program`, with every step read back from its JSON first: what
/// a program file holds builds the same part.
fn build(program: &Program) -> NamedModel {
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
            seeds: Seeds::bodies(bodies("cylinder(pin)"), Combine::cut("block(plate)")),
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
    // where the plate's faces cross it, its one piece left keeping the
    // copy's name.
    let wall = part.names().iter().any(|(id, name)| {
        matches!(id, geop_ops_model::RefId::Face(_))
            && name.starts_with("linear_pattern(holes,3,cylinder(pin,c")
    });
    assert!(wall, "no face of the third hole");
}

/// The pin turned six times around a hole's axis through the middle of a
/// square plate: a bolt circle.
#[test]
fn bolt_circle_cut_by_a_patterned_tool() {
    let mut program = Program::new();
    block(&mut program, "plate", [-3.0, -3.0], [3.0, 3.0], 0.5);
    pin(&mut program, [2.0, 0.0]);
    program.push(
        "bolts",
        CircularPatternArgs {
            seeds: Seeds::bodies(bodies("cylinder(pin)"), Combine::cut("block(plate)")),
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
    prism(&mut program, "half", &l);
    program.push(
        "m",
        MirrorArgs {
            seeds: Seeds::bodies(bodies("prism(half)"), Combine::join("prism(half)")),
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
    prism(&mut program, "l", &l);
    program.push(
        "m",
        MirrorArgs {
            seeds: Seeds::bodies(bodies("prism(l)"), Combine::NewBody),
            plane: origin_mirror(FrameAxis::X),
        },
    );
    let part = build(&program);
    assert_valid(&part);
    let image = part.solid_id("mirror(m,prism(l))").unwrap();
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
    assert_volume(&part, "mirror(m,prism(l))", 2.25, 1e-6);
}

/// A pin moved into place and cut from the plate, rather than drawn
/// there: the plate with one hole at (5, 1).
#[test]
fn tool_moved_into_place_and_cut() {
    let mut program = plate_and_pin();
    program.push(
        "mv",
        MoveBodyArgs {
            bodies: bodies("cylinder(pin)"),
            translation: [4.0, 0.0, 0.0],
            axis: None,
            angle: 0.0,
            copy: false,
            combine: Combine::cut("block(plate)"),
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
        block(&mut program, "b", [2.0, -0.4], [3.0, 0.4], 1.0);
        program.push(
            "r",
            CircularPatternArgs {
                seeds: Seeds::bodies(
                    bodies("block(b)"),
                    if joined {
                        Combine::join("block(b)")
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
        prism(&mut program, name, &corners);
    }
    program.push(
        "ab",
        geop_ops_boolean::BooleanArgs {
            a: solid("prism(a)"),
            b: solid("prism(b)"),
            op: geop_ops_model::BooleanOp::Union,
        },
    );
    program.push(
        "abc",
        geop_ops_boolean::BooleanArgs {
            a: solid("boolean(ab)"),
            b: solid("prism(c)"),
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
        prism(&mut program, name, &corners);
    }
    program.push(
        "c",
        MoveBodyArgs {
            bodies: bodies("prism(a)"),
            translation: [0.0; 3],
            axis: origin_turn(FrameAxis::Z),
            angle: 26.0,
            copy: true,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "ab",
        geop_ops_boolean::BooleanArgs {
            a: solid("prism(a)"),
            b: solid("prism(b)"),
            op: geop_ops_model::BooleanOp::Union,
        },
    );
    program.push(
        "abc",
        geop_ops_boolean::BooleanArgs {
            a: solid("boolean(ab)"),
            b: solid("move(c,prism(a))"),
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
fn widest_pcurve(part: &NamedModel) -> (f64, String) {
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
    block(&mut program, "b", [2.0, -0.4], [3.0, 0.4], 1.0);
    program.push(
        "r",
        CircularPatternArgs {
            seeds: Seeds::bodies(bodies("block(b)"), Combine::join("block(b)")),
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
        block(&mut program, "plate", [0.0, 0.0], [30.0, 6.0], 0.5);
        pin(&mut program, [1.0, 1.0]);
        program.push(
            "holes",
            LinearPatternArgs {
                seeds: Seeds::bodies(bodies("cylinder(pin)"), Combine::cut("block(plate)")),
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
    // The plate's upright edge at its corner on the origin.
    let corner = edge_at(
        &plate_and_pin().build_model(registry).unwrap(),
        [0.0, 0.0, 0.25],
    );
    for degrees in [1.0, 17.3, 89.99, 133.7, 271.1] {
        let mut program = plate_and_pin();
        program.push(
            "mv",
            MoveBodyArgs {
                bodies: bodies("block(plate)"),
                translation: [0.3, -1.7, 2.2],
                axis: Some(TurnAxis::Edge(ElementReference::new(model(), &corner))),
                angle: degrees,
                copy: true,
                combine: Combine::NewBody,
            },
        );
        let part = program
            .build_model(registry)
            .unwrap_or_else(|e| panic!("{degrees}°: {e:?}"));
        assert_valid(&part);
        assert_volume(&part, "move(mv,block(plate))", 8.0, 1e-6);
    }
}

/// The pin patterned along a 3-D sketch's line — drawn beside the plate,
/// along `x` — and along a planar sketch's line across it: a grid of holes,
/// as along the origin's axes. The sketch's lines are picked as any
/// straight edge is.
#[test]
fn holes_patterned_along_sketch_lines() {
    let mut program = plate_and_pin();
    let mut route = geop_ops_model::sketch3d::Sketch3d::new();
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
    program.push("across", AddSketchArgs::on(on_origin(FrameAxis::Z), across));
    program.push(
        "holes",
        LinearPatternArgs {
            seeds: Seeds::bodies(bodies("cylinder(pin)"), Combine::cut("block(plate)")),
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

/// The patterned plate example is one solid with four holes in a row, 2
/// apart — solid between them — and, saved and loaded, builds what it
/// built.
#[test]
fn patterned_plate_round_trips() {
    use geop_core_topology::contains::shell::PointClassification::{Inside, Outside};
    use geop_ops_model::testing::classify;

    let registry = std::rc::Rc::new(registry().unwrap());
    let program = crate::examples::patterned_plate();
    let built = geop_core_program::testing::assert_round_trips(registry, None, &program);
    let part = geop_ops_model::testing::model_of(&built);
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["linear_pattern(holes)"]);
    for (p, expected) in [
        ([5.0, 1.5, 0.25], Outside),
        ([5.0, 1.9, 0.25], Inside),
        ([7.0, 1.5, 0.25], Outside),
        ([6.0, 1.5, 0.25], Inside),
    ] {
        assert_eq!(
            classify(&part, "linear_pattern(holes)", p),
            expected,
            "{p:?}"
        );
    }
}

/// The example `patterned_plate`, edited as a user edits it, builds
/// incrementally what it builds from scratch.
#[test]
fn patterned_plate_builds_incrementally() {
    let registry = std::rc::Rc::new(registry().unwrap());
    let programs = geop_core_program::testing::edit_sequence(&crate::examples::patterned_plate());
    geop_core_program::testing::assert_incremental(registry, None, &programs);
}

/// The examples drawn in the editor build valid parts and round-trip, as
/// the ones written in Rust do: the turbine's blades joined to its hub in
/// one solid, the flange cut sixteen times.
#[test]
fn drawn_examples_round_trip() {
    use crate::examples::{cool_flange, turbine_blade};

    for (program, solids) in [
        (turbine_blade(), vec!["circular_pattern(circular_pattern1)"]),
        (cool_flange(), vec!["circular_pattern(circular_pattern1)"]),
    ] {
        let registry = std::rc::Rc::new(registry().unwrap());
        let built = geop_core_program::testing::assert_round_trips(registry, None, &program);
        let part = geop_ops_model::testing::model_of(&built);
        assert_valid(&part);
        assert_eq!(part.solid_names(), solids);
    }
}

/// The examples drawn in the editor, edited as a user edits them, build
/// incrementally what they build from scratch.
#[test]
#[ignore = "slow: 20 s, two patterns built a dozen times over — run with `cargo test -- --ignored`"]
fn drawn_examples_build_incrementally() {
    use crate::examples::{cool_flange, turbine_blade};

    for program in [turbine_blade(), cool_flange()] {
        let registry = std::rc::Rc::new(registry().unwrap());
        let programs = geop_core_program::testing::edit_sequence(&program);
        geop_core_program::testing::assert_incremental(registry, None, &programs);
    }
}
