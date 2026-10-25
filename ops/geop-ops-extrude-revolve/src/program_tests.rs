//! The operations run as programs: sketches extruded and revolved into
//! solids and faces — as far as a length, up to the next face of a solid
//! or through all of it — swept along paths and lofted through profiles,
//! combined with others, and what they refuse.

use crate::{
    Extent, Extents, ExtrudeArgs, LoftArgs, Output, PathRef, RevolveArgs, RevolveAxis, SweepArgs,
    examples::box_with_drill_hole,
    testing::{block_and_post, both_ways, extrude, extrusion, one_way, two_ways},
};
use geop_core_program::{
    element::ElementReference,
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    testing::{as_step, edit_args, step},
    values::Design,
};
use geop_lib_sweep::Orientation;
use geop_ops_model::sketch::{
    Constraint, PlacedSketch, Sketch, SketchPlane,
    figures::{disc, n, on_face, on_origin, outline, polygon},
    sketch as sketch_target, sketch_element, whole_sketch,
};
use geop_ops_model::{
    BooleanOp, Combine, Curve, NamedModel, read_model, solid,
    testing::{Built, assert_valid, runner_of},
};
use geop_ops_sketch::AddSketchArgs;
use std::f64::consts::PI;

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_sketch::PointId;
use geop_core_topology::validation::{ValidationParameters, validate};
use geop_ops_model::datums::{AnyDatum, Datums, ORIGIN, component_name, datums, origin_axis};
use geop_ops_model::selection::Selected;
use geop_ops_model::sketch3d::sketch3d_path;
use geop_ops_reference_geometry::{AddDatum, AddDatumArgs, Construction};
use geop_ops_sketch3d::AddSketch3d;

/// The sketch `name` as a path.
fn sketch_path(name: &str) -> PathRef {
    PathRef::Sketch(ElementReference::whole(sketch_target(name)))
}

/// The datum `name`, to sketch on.
fn on_datum(name: &str) -> SketchPlane {
    SketchPlane::Datum(ElementReference::new(datums(), name))
}

/// The plane of the origin normal to `normal`, to build a datum on.
fn origin_plane(normal: FrameAxis) -> Selected {
    let name = component_name(ORIGIN, DatumComponent::Plane(normal));
    Selected::Datum(ElementReference::<Datums, AnyDatum>::new(datums(), name))
}

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    crate::testing::registry()?
        .with(AddDatum)?
        .with(AddSketch3d)
}

/// The square of side `2 half` around the origin.
fn square(half: f64) -> Sketch {
    outline(&[[-half, -half], [half, -half], [half, half], [-half, half]])
}

/// The open chain of lines through `points`.
fn polyline(points: &[[f64; 2]]) -> Sketch {
    let mut s = Sketch::new();
    let p: Vec<PointId> = points
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for w in p.windows(2) {
        s.add_line(w[0], w[1]);
    }
    s
}

/// A path in the `xy` plane: along `x` from the origin to `(2, 0)`, a
/// quarter turn left to `(3, 1)`, and on along `y` to `(3, 3)`.
fn bend() -> Sketch {
    let mut s = Sketch::new();
    let p = [
        s.add_point(n(0.0), n(0.0)),
        s.add_point(n(2.0), n(0.0)),
        s.add_point(n(3.0), n(1.0)),
        s.add_point(n(3.0), n(3.0)),
    ];
    s.add_line(p[0], p[1]);
    s.add_arc(p[1], p[2], n(PI / 2.0));
    s.add_line(p[2], p[3]);
    s
}

fn swept(profile: &str, path: &str) -> SweepArgs {
    SweepArgs {
        profile: whole_sketch(profile),
        path: Some(sketch_path(path)),
        orientation: Orientation::FollowPath,
        twist: 0.0,
        end_scale: 1.0,
        rails: Vec::new(),
        output: Output::Solid(Combine::NewBody),
    }
}

fn lofted(profiles: &[&str]) -> LoftArgs {
    LoftArgs {
        profiles: profiles.iter().map(|p| whole_sketch(p)).collect(),
        matches: Vec::new(),
        guides: Vec::new(),
        output: Output::Solid(Combine::NewBody),
    }
}

/// The plane `distance` above the `xy` plane, as the datum `name`.
fn lifted(program: &mut Program, name: &str, distance: f64) {
    program.push(
        name,
        AddDatumArgs {
            selection: vec![origin_plane(FrameAxis::Z)],
            construction: Construction::Offset {
                distance: distance.into(),
            },
        },
    );
}

/// A square tube round a corner: mitred where the two lines of the path
/// meet, a solid of eight walls and two caps.
#[test]
fn sweep_a_square_round_a_corner() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), square(0.25)),
    );
    program.push(
        "path",
        AddSketchArgs::on(
            on_origin(FrameAxis::Z),
            polyline(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]),
        ),
    );
    program.push("tube", swept("profile", "path"));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert_eq!(part.topology().faces.len(), 4 * 2 + 2);
    assert!(part.face_id("sweep(tube,start)").is_ok());
    assert!(part.face_id("sweep(tube,end)").is_ok());
}

/// A round pipe through a bend: a line, an arc tangent to it, a line —
/// every wall exact, the end cap square to the last line.
#[test]
fn sweep_a_pipe_through_a_bend() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), bend()));
    program.push("pipe", swept("profile", "path"));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 3 * 4 + 2);
    let model = part.topology();
    let end = part.face_id("sweep(pipe,end)").unwrap();
    assert!(model.iterate_face_coedges(end).all(|c| {
        model.coedge_start_vertex(c).unwrap().point[1].could_be_equal(S::from_f64(3.0))
    }));
}

/// The profile drawn at the far end of the path: the path is run from
/// there, so the solid lies along it all the same.
#[test]
fn sweep_starts_at_the_end_of_the_path_the_profile_is_at() {
    let mut program = Program::new();
    // In the plane `y = 3`, at `x = 3`: where the bend ends.
    let mut far = Sketch::new();
    polygon(
        &mut far,
        &[[2.75, -0.25], [3.25, -0.25], [3.25, 0.25], [2.75, 0.25]],
    );
    program.push(
        "far_plane",
        AddDatumArgs {
            selection: vec![origin_plane(FrameAxis::Y)],
            construction: Construction::Offset {
                distance: 3.0.into(),
            },
        },
    );
    program.push("profile", AddSketchArgs::on(on_datum("far_plane"), far));
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), bend()));
    program.push("bar", swept("profile", "path"));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The start cap where the profile is drawn, the end cap at the origin.
    let model = part.topology();
    let end = part.face_id("sweep(bar,end)").unwrap();
    assert!(
        model
            .iterate_face_coedges(end)
            .all(|c| { model.coedge_start_vertex(c).unwrap().point[0].could_be_equal(S::ZERO) })
    );
}

/// A circle swept round a circle: a torus, closed on itself, no caps.
#[test]
fn sweep_a_circle_round_a_circle_is_a_torus() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::Y), disc([2.0, 0.0], 0.5)),
    );
    program.push(
        "path",
        AddSketchArgs::on(on_origin(FrameAxis::Z), disc([0.0, 0.0], 2.0)),
    );
    program.push("ring", swept("profile", "path"));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 16);
    assert!(part.face_id("sweep(ring,start)").is_err());
}

/// A pipe through a bend, joined to a block it starts inside of.
#[test]
fn sweep_joins_a_block() {
    let mut program = Program::new();
    let block = outline(&[[-1.0, -1.0], [0.5, -1.0], [0.5, 1.0], [-1.0, 1.0]]);
    program.push(
        "block_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), block),
    );
    program.push(
        "block",
        extrusion("block_sketch", both_ways(1.0), Combine::NewBody),
    );
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), bend()));
    program.push(
        "pipe",
        SweepArgs {
            output: Output::Solid(Combine::join("extrude(block)")),
            ..swept("profile", "path")
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.solid_id("sweep(pipe)").is_ok());
    // The pipe's far end stands clear of the block, as swept.
    assert!(part.face_id("sweep(pipe,end)").is_ok());
    assert!(part.face_id("sweep(pipe,start)").is_err());
}

/// An open profile swept into faces standing on their own.
#[test]
fn sweep_an_open_profile_as_faces() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(
            on_origin(FrameAxis::X),
            polyline(&[[-0.5, 0.0], [0.0, 0.5], [0.5, 0.0]]),
        ),
    );
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), bend()));
    program.push(
        "sheet",
        SweepArgs {
            output: Output::Faces { up_to: None },
            ..swept("profile", "path")
        },
    );
    let part = program.build_model(registry).unwrap();
    if let Err(e) = validate(&ValidationParameters::default(), part.topology()) {
        panic!("{e:?}");
    }
    assert_eq!(part.topology().solids.len(), 0);
    assert_eq!(part.topology().faces.len(), 2 * 3);
}

/// The profile and the path must be two sketches; a path turning a corner
/// into an arc cannot be mitred.
#[test]
fn sweep_refuses_what_it_cannot_sweep() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("same", swept("profile", "profile"));
    assert!(program.build_model(registry).is_err());

    let mut kinked = Sketch::new();
    let p = [
        kinked.add_point(n(0.0), n(0.0)),
        kinked.add_point(n(2.0), n(0.0)),
        kinked.add_point(n(1.0), n(1.0)),
    ];
    kinked.add_line(p[0], p[1]);
    kinked.add_arc(p[1], p[2], n(PI / 2.0));
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), kinked));
    program.push("pipe", swept("profile", "path"));
    let Err(error) = program.build_model(registry) else {
        panic!("built");
    };
    assert!(error.root_message().contains("corner"), "{error:?}");
}

/// A square below, a circle above: a solid of four ruled walls and two
/// caps.
#[test]
fn loft_a_square_to_a_circle() {
    let mut program = Program::new();
    program.push(
        "bottom",
        AddSketchArgs::on(on_origin(FrameAxis::Z), square(1.0)),
    );
    lifted(&mut program, "top_plane", 2.0);
    program.push(
        "top",
        AddSketchArgs::on(on_datum("top_plane"), disc([0.0, 0.0], 0.6)),
    );
    program.push("transition", lofted(&["bottom", "top"]));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert_eq!(part.topology().faces.len(), 4 + 2);
    assert!(part.face_id("loft(transition,start)").is_ok());
    assert!(part.face_id("loft(transition,end)").is_ok());
}

/// Through three profiles — a square, a circle, a smaller square — and
/// cut out of a block: a loft as a tool.
#[test]
fn loft_through_three_profiles_cuts_a_block() {
    let mut program = Program::new();
    let mut block = Sketch::new();
    polygon(
        &mut block,
        &[[-2.0, -2.0], [2.0, -2.0], [2.0, 2.0], [-2.0, 2.0]],
    );
    program.push(
        "block_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), block),
    );
    program.push(
        "block",
        extrusion("block_sketch", Extents::blind(1.5), Combine::NewBody),
    );
    lifted(&mut program, "low", -0.5);
    program.push("a", AddSketchArgs::on(on_datum("low"), square(1.0)));
    lifted(&mut program, "middle", 0.75);
    program.push(
        "b",
        AddSketchArgs::on(on_datum("middle"), disc([0.0, 0.0], 0.7)),
    );
    lifted(&mut program, "high", 2.0);
    program.push("c", AddSketchArgs::on(on_datum("high"), square(0.5)));
    program.push(
        "pocket",
        LoftArgs {
            output: Output::Solid(Combine::cut("extrude(block)")),
            ..lofted(&["a", "b", "c"])
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.solid_id("loft(pocket)").is_ok());
    // The block's top and bottom are pierced: each keeps a hole where the
    // loft passed through it.
    let model = part.topology();
    for cap in ["extrude(block,start)", "extrude(block,end)"] {
        let face = model.get_face(part.face_id(cap).unwrap()).unwrap();
        assert_eq!(face.holes.len(), 1, "{cap}");
    }
}

/// Two open curves loft into the ruled face between them.
#[test]
fn loft_between_two_curves_as_a_face() {
    let mut program = Program::new();
    program.push(
        "low",
        AddSketchArgs::on(
            on_origin(FrameAxis::Z),
            polyline(&[[-1.0, 0.0], [1.0, 0.0]]),
        ),
    );
    lifted(&mut program, "high_plane", 1.0);
    let mut arc = Sketch::new();
    let p = [
        arc.add_point(n(1.0), n(0.0)),
        arc.add_point(n(-1.0), n(0.0)),
    ];
    arc.add_arc(p[0], p[1], n(PI / 2.0));
    program.push("high", AddSketchArgs::on(on_datum("high_plane"), arc));
    program.push(
        "skin",
        LoftArgs {
            output: Output::Faces { up_to: None },
            ..lofted(&["low", "high"])
        },
    );
    let part = program.build_model(registry).unwrap();
    if let Err(e) = validate(&ValidationParameters::default(), part.topology()) {
        panic!("{e:?}");
    }
    // A quarter turn is one piece, as the line is: one ruled face.
    assert_eq!(part.topology().faces.len(), 1);
    assert_eq!(part.topology().solids.len(), 0);
}

/// A loft through one profile, or through a profile with a hole, is
/// refused.
#[test]
fn loft_refuses_what_it_cannot_loft() {
    let mut program = Program::new();
    program.push(
        "only",
        AddSketchArgs::on(on_origin(FrameAxis::Z), square(1.0)),
    );
    program.push("one", lofted(&["only"]));
    assert!(program.build_model(registry).is_err());

    let mut holed = square(1.0);
    let c = holed.add_point(n(0.0), n(0.0));
    holed.add_circle(c, n(0.3));
    let mut program = Program::new();
    program.push("holed", AddSketchArgs::on(on_origin(FrameAxis::Z), holed));
    // Refused for its first profile, wherever the second lies.
    program.push(
        "top",
        AddSketchArgs::on(on_origin(FrameAxis::X), square(0.5)),
    );
    program.push("loft", lofted(&["holed", "top"]));
    let Err(error) = program.build_model(registry) else {
        panic!("built");
    };
    assert!(error.root_message().contains("holes"), "{error:?}");
}

/// A block with a slab-shaped top and bottom at `z = ±0.5`, spanning
/// `[-1, 1.5] x [-3, 3]` — for the solids below to be cut from.
fn slab(program: &mut Program) {
    let block = outline(&[[-1.0, -3.0], [1.5, -3.0], [1.5, 3.0], [-1.0, 3.0]]);
    program.push(
        "slab_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), block),
    );
    program.push(
        "slab",
        extrusion("slab_sketch", both_ways(1.0), Combine::NewBody),
    );
}

/// A torus cut from a slab it crosses: the ring's part inside the slab
/// leaves a curved channel through it.
#[test]
fn sweep_a_ring_cuts_a_slab() {
    let mut program = Program::new();
    slab(&mut program);
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::Y), disc([2.0, 0.0], 0.3)),
    );
    program.push(
        "path",
        AddSketchArgs::on(on_origin(FrameAxis::Z), disc([0.0, 0.0], 2.0)),
    );
    program.push(
        "channel",
        SweepArgs {
            output: Output::Solid(Combine::cut("extrude(slab)")),
            ..swept("profile", "path")
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
}

/// A pipe along a spline, cut through a slab.
#[test]
fn sweep_along_a_spline_cuts_a_slab() {
    let mut program = Program::new();
    slab(&mut program);
    let mut wave = Sketch::new();
    let p = [
        wave.add_point(n(-2.0), n(-1.0)),
        wave.add_point(n(0.0), n(-1.0)),
        wave.add_point(n(0.5), n(1.0)),
        wave.add_point(n(2.5), n(1.0)),
    ];
    wave.add_spline(p.to_vec());
    // Square to the spline where it starts, along x.
    program.push(
        "start_plane",
        AddDatumArgs {
            selection: vec![origin_plane(FrameAxis::X)],
            construction: Construction::Offset {
                distance: (-2.0).into(),
            },
        },
    );
    program.push(
        "profile",
        AddSketchArgs::on(on_datum("start_plane"), disc([-1.0, 0.0], 0.2)),
    );
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), wave));
    program.push(
        "hole",
        SweepArgs {
            output: Output::Solid(Combine::cut("extrude(slab)")),
            ..swept("profile", "path")
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
}

/// Sweep and loft steps are saved and read back like any other.
#[test]
fn sweep_and_loft_steps_round_trip() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), bend()));
    program.push("pipe", swept("profile", "path"));
    program.push(
        "bottom",
        AddSketchArgs::on(on_origin(FrameAxis::Z), square(1.0)),
    );
    program.push(
        "top",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.6)),
    );
    program.push("transition", lofted(&["bottom", "top"]));
    let json = serde_json::to_string(&program).unwrap();
    assert!(json.contains("\"operation\":\"sweep\""), "{json}");
    assert!(json.contains("\"operation\":\"loft\""), "{json}");
    let back: Program = serde_json::from_str(&json).unwrap();
    assert_eq!(back, program);
}

/// Rebuilt from a part whose loft failed while enclosing its first sketch:
/// a D — two sides joined by a half circle tangent to both, closed by an
/// upright side — lofted into a triangle a unit above.
#[test]
fn loft_a_d_into_a_triangle() {
    let mut d = Sketch::new();
    let p = [
        d.add_point(n(-0.8559366928294662), n(0.6532261597610688)),
        d.add_point(n(0.6690494426064505), n(0.6532261597610689)),
        d.add_point(n(0.6690494426064503), n(-0.40501734890700397)),
        d.add_point(n(-0.8559366928294662), n(-0.405017348907004)),
    ];
    let top = d.add_line(p[0], p[1]);
    let end = d.add_arc(p[1], p[2], n(-PI));
    let bottom = d.add_line(p[2], p[3]);
    let side = d.add_line(p[3], p[0]);
    d.constrain(Constraint::Horizontal { line: top });
    d.constrain(Constraint::Tangent { a: top, b: end });
    d.constrain(Constraint::Horizontal { line: bottom });
    d.constrain(Constraint::Tangent { a: end, b: bottom });
    d.constrain(Constraint::Vertical { line: side });
    let mut triangle = Sketch::new();
    polygon(
        &mut triangle,
        &[
            [-0.4948716405501765, 0.879258752572198],
            [-0.4948716405501765, -0.6736145210517424],
            [0.7189527425188181, 0.1424512068640546],
        ],
    );
    let mut program = Program::new();
    program.push("d", AddSketchArgs::on(on_origin(FrameAxis::Z), d));
    lifted(&mut program, "above", 1.0);
    program.push("triangle", AddSketchArgs::on(on_datum("above"), triangle));
    program.push("loft", lofted(&["d", "triangle"]));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["loft(loft)"]);
}

/// A rectangle, a triangle a unit above it and a circle a unit above that,
/// lofted through in turn.
#[test]
fn loft_a_rectangle_through_a_triangle_into_a_circle() {
    let mut rectangle = Sketch::new();
    polygon(
        &mut rectangle,
        &[[-0.6, -0.4], [0.6, -0.4], [0.6, 0.4], [-0.6, 0.4]],
    );
    let mut triangle = Sketch::new();
    polygon(&mut triangle, &[[-0.5, -0.4], [0.5, -0.4], [0.0, 0.5]]);
    let mut program = Program::new();
    program.push(
        "rectangle",
        AddSketchArgs::on(on_origin(FrameAxis::Z), rectangle),
    );
    for (name, distance) in [("one", 1.0), ("two", 2.0)] {
        lifted(&mut program, name, distance);
    }
    let on = on_datum;
    program.push("triangle", AddSketchArgs::on(on("one"), triangle));
    program.push(
        "circle",
        AddSketchArgs::on(on("two"), disc([0.0, 0.0], 0.4)),
    );
    program.push("loft", lofted(&["rectangle", "triangle", "circle"]));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["loft(loft)"]);
}

/// A square's corner matched with a point a third of the way round a
/// circle above it: the loft runs from that corner to that point — the
/// circle is split there — and the loft is a valid solid.
#[test]
fn loft_with_matched_points() {
    let mut square = Sketch::new();
    let corners = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]];
    let p: Vec<PointId> = corners
        .iter()
        .map(|c| square.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..4 {
        square.add_line(p[i], p[(i + 1) % 4]);
    }
    let angle = 2.0 * PI / 3.0;
    let mut round = Sketch::new();
    let c = round.add_point(n(0.0), n(0.0));
    let rim = round.add_circle(c, n(0.5));
    let mark = round.add_point(n(0.5 * angle.cos()), n(0.5 * angle.sin()));
    round.constrain(Constraint::PointOnCurve {
        point: mark,
        curve: rim,
    });
    let mut program = Program::new();
    program.push("square", AddSketchArgs::on(on_origin(FrameAxis::Z), square));
    lifted(&mut program, "above", 1.0);
    program.push("round", AddSketchArgs::on(on_datum("above"), round));
    let mut args = lofted(&["square", "round"]);
    args.matches = vec![
        sketch_element("square", p[2]),
        sketch_element("round", mark),
    ];
    program.push("loft", args);
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let model = part.topology();
    let at = |q: [f64; 3]| {
        model
            .vertices
            .iter()
            .find(|(_, v)| (0..3).all(|k| v.point[k].could_be_equal(S::from_f64(q[k]))))
            .map(|(&id, _)| id)
    };
    let corner = at([0.5, 0.5, 0.0]).expect("the matched corner");
    let target = at([0.5 * angle.cos(), 0.5 * angle.sin(), 1.0]).expect("the matched point");
    assert!(
        model
            .edges
            .values()
            .any(|e| (e.start_vertex == corner && e.end_vertex == target)
                || (e.start_vertex == target && e.end_vertex == corner)),
        "no edge runs from the matched corner to the matched point"
    );
}

/// The D of [`loft_a_d_into_a_triangle`] with two pairs of points matched:
/// its upright side's top and bottom corners with those of the triangle's
/// upright side. The loft joins each matched pair by an edge.
#[test]
fn loft_a_d_into_a_triangle_matched_at_two_points() {
    let mut d = Sketch::new();
    let p = [
        d.add_point(n(-0.8559366928294662), n(0.6532261597610688)),
        d.add_point(n(0.6690494426064505), n(0.6532261597610689)),
        d.add_point(n(0.6690494426064503), n(-0.40501734890700397)),
        d.add_point(n(-0.8559366928294662), n(-0.405017348907004)),
    ];
    let top = d.add_line(p[0], p[1]);
    let end = d.add_arc(p[1], p[2], n(-PI));
    let bottom = d.add_line(p[2], p[3]);
    let side = d.add_line(p[3], p[0]);
    d.constrain(Constraint::Horizontal { line: top });
    d.constrain(Constraint::Tangent { a: top, b: end });
    d.constrain(Constraint::Horizontal { line: bottom });
    d.constrain(Constraint::Tangent { a: end, b: bottom });
    d.constrain(Constraint::Vertical { line: side });
    let corners = [
        [-0.4948716405501765, 0.879258752572198],
        [-0.4948716405501765, -0.6736145210517424],
        [0.7189527425188181, 0.1424512068640546],
    ];
    let mut triangle = Sketch::new();
    let q: Vec<PointId> = corners
        .iter()
        .map(|c| triangle.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..3 {
        triangle.add_line(q[i], q[(i + 1) % 3]);
    }
    let mut program = Program::new();
    program.push("d", AddSketchArgs::on(on_origin(FrameAxis::Z), d));
    lifted(&mut program, "above", 1.0);
    program.push("triangle", AddSketchArgs::on(on_datum("above"), triangle));
    let point = |sketch: &str, point: PointId| sketch_element(sketch, point);
    let mut args = lofted(&["d", "triangle"]);
    args.matches = vec![
        point("d", p[0]),
        point("triangle", q[0]),
        point("d", p[3]),
        point("triangle", q[1]),
    ];
    program.push("loft", args);
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let model = part.topology();
    let at = |x: f64, y: f64, z: f64| {
        model
            .vertices
            .iter()
            .find(|(_, v)| {
                [x, y, z]
                    .iter()
                    .enumerate()
                    .all(|(k, &c)| v.point[k].could_be_equal(S::from_f64(c)))
            })
            .map(|(&id, _)| id)
            .unwrap_or_else(|| panic!("no vertex at {x}, {y}, {z}"))
    };
    let joined = |a, b| {
        model.edges.values().any(|e| {
            (e.start_vertex == a && e.end_vertex == b) || (e.start_vertex == b && e.end_vertex == a)
        })
    };
    let d_top = at(-0.8559366928294662, 0.6532261597610688, 0.0);
    let d_bottom = at(-0.8559366928294662, -0.405017348907004, 0.0);
    let t_top = at(corners[0][0], corners[0][1], 1.0);
    let t_bottom = at(corners[1][0], corners[1][1], 1.0);
    assert!(joined(d_top, t_top), "the top corners are not joined");
    assert!(
        joined(d_bottom, t_bottom),
        "the bottom corners are not joined"
    );
}

// ── rails, twist, scale and guides ─────────────────────────────────────────

/// A circle swept along a line with one rail drifting out: a cone, its
/// far end the circle the rail's end lies on.
#[test]
fn sweep_along_a_rail_tapers() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.5)),
    );
    program.push(
        "path",
        AddSketchArgs::on(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.0], [2.0, 0.0]])),
    );
    program.push(
        "rail",
        AddSketchArgs::on(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.5], [2.0, 1.0]])),
    );
    program.push(
        "cone",
        SweepArgs {
            rails: vec![sketch_path("rail")],
            ..swept("profile", "path")
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let model = part.topology();
    let end = part.face_id("sweep(cone,end)").unwrap();
    for c in model.iterate_face_coedges(end) {
        let p = model.coedge_start_vertex(c).unwrap().point;
        let r = (p[1].to_f64().powi(2) + p[2].to_f64().powi(2)).sqrt();
        assert!((r - 1.0).abs() < 1e-9, "{p:?}");
    }
}

/// A square bar twisted a quarter turn and shrunk to half its size along
/// a bent path.
#[test]
fn sweep_a_twisted_tapering_bar() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), square(0.25)),
    );
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), bend()));
    program.push(
        "bar",
        SweepArgs {
            twist: 90.0,
            end_scale: 0.5,
            ..swept("profile", "path")
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 3 * 4 + 2);
}

/// A loft between two squares with a guide bowing out beside them, along
/// the middle of one side: the walls bulge with it, joined to a block.
#[test]
fn loft_along_a_guide() {
    let mut program = Program::new();
    program.push(
        "bottom",
        AddSketchArgs::on(on_origin(FrameAxis::Z), square(1.0)),
    );
    lifted(&mut program, "top_plane", 2.0);
    program.push("top", AddSketchArgs::on(on_datum("top_plane"), square(1.0)));
    // In the plane y = 0, drawn in (x, -z): from (1, 0, 0) out to x = 1.5
    // and back to (1, 0, 2).
    let mut bow = Sketch::new();
    let p = [
        bow.add_point(n(1.0), n(0.0)),
        bow.add_point(n(1.5), n(-1.0)),
        bow.add_point(n(1.0), n(-2.0)),
    ];
    bow.add_spline(p.to_vec());
    program.push("guide", AddSketchArgs::on(on_origin(FrameAxis::Y), bow));
    program.push(
        "bulge",
        LoftArgs {
            guides: vec![sketch_path("guide")],
            ..lofted(&["bottom", "top"])
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The edge along the guide bulges out to it.
    let edge = part.edge_id("loft(bulge,bottom,guide)").unwrap();
    let curve = &part.topology().edges[&edge].curve;
    let middle = curve.evaluate(S::from_f64(0.5)).unwrap();
    assert!(middle[0].to_f64() > 1.2, "{middle:?}");
}

/// Rails and guides are saved and read back with their steps.
#[test]
fn rails_and_guides_round_trip() {
    let mut program = Program::new();
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.5)),
    );
    program.push(
        "path",
        AddSketchArgs::on(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.0], [2.0, 0.0]])),
    );
    program.push(
        "rail",
        AddSketchArgs::on(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.5], [2.0, 1.0]])),
    );
    program.push(
        "cone",
        SweepArgs {
            rails: vec![sketch_path("rail")],
            orientation: Orientation::FixedNormal,
            ..swept("profile", "path")
        },
    );
    program.push(
        "bar",
        SweepArgs {
            twist: 45.0,
            end_scale: 2.0,
            ..swept("profile", "path")
        },
    );
    program.push(
        "skin",
        LoftArgs {
            guides: vec![sketch_path("rail")],
            ..lofted(&["profile", "path"])
        },
    );
    let json = serde_json::to_string(&program).unwrap();
    assert!(
        json.contains("\"rails\":[{\"sketch\":{\"target\":\"/:sketch:rail\"}}]"),
        "{json}"
    );
    assert!(json.contains("\"fixed_normal\""), "{json}");
    let back: Program = serde_json::from_str(&json).unwrap();
    assert_eq!(back, program);
}

/// The loft of [`loft_along_a_guide`], its guide a 3-D sketch's spline
/// drawn through the same points in space: the same bulge.
#[test]
fn loft_along_a_3d_sketch_guide() {
    let mut program = Program::new();
    program.push(
        "bottom",
        AddSketchArgs::on(on_origin(FrameAxis::Z), square(1.0)),
    );
    lifted(&mut program, "top_plane", 2.0);
    program.push("top", AddSketchArgs::on(on_datum("top_plane"), square(1.0)));
    let mut bow = geop_ops_model::sketch3d::Sketch3d::new();
    let p = [[1.0, 0.0, 0.0], [1.5, 0.0, 1.0], [1.0, 0.0, 2.0]]
        .map(|p| bow.add_point(geop_core_math::vector::Vector3::from_array(p.map(n))));
    bow.add_spline(p.to_vec());
    program.push(
        "guide",
        geop_ops_sketch3d::AddSketch3dArgs {
            sketch: bow,
            references: Vec::new(),
        },
    );
    program.push(
        "bulge",
        LoftArgs {
            guides: vec![PathRef::Sketch3d(sketch3d_path("guide"))],
            ..lofted(&["bottom", "top"])
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let edge = part.edge_id("loft(bulge,bottom,guide)").unwrap();
    let curve = &part.topology().edges[&edge].curve;
    let middle = curve.evaluate(S::from_f64(0.5)).unwrap();
    assert!(middle[0].to_f64() > 1.2, "{middle:?}");
}

/// A sketch's curves are named as one chain the same as each is named on
/// its own (`K,c3` for a curve and `K,p1` for a point), so a path or a rail
/// is spoken of in the names everything else uses.
#[test]
fn a_sketchs_chain_is_named_as_its_curves_are() {
    let mut program = Program::new();
    program.push("path", AddSketchArgs::on(on_origin(FrameAxis::Z), bend()));
    let part = program.built(registry).unwrap();
    let reader = part.current();
    let chain = sketch_path("path").resolve(&reader).unwrap();
    assert!(!chain.is_closed());
    let placed: &PlacedSketch<S> = reader.retrieve_element(&whole_sketch("path")).unwrap();
    let sketch = &placed.sketch;
    assert_eq!(chain.curves.len(), sketch.curves.len());
    for (k, id) in sketch.curves.keys().enumerate() {
        let alone = reader
            .retrieve_element(&sketch_element::<Curve>("path", id))
            .unwrap();
        assert_eq!(chain.curve_names[k], alone.curve_names[0]);
        let mut joints = [&chain.joint_names[k], &chain.joint_names[k + 1]];
        let mut ends = [&alone.joint_names[0], &alone.joint_names[1]];
        joints.sort();
        ends.sort();
        assert_eq!(joints, ends, "the joints of {}", alone.name);
    }
}

/// A sweep runs along an edge of a solid as along a sketch: here the edge a
/// block's front stands on, under a round profile at its start.
#[test]
fn sweep_along_an_edge() {
    let mut program = Program::new();
    let footprint = outline(&[[0.0, 0.0], [2.0, 0.0], [2.0, 1.0], [0.0, 1.0]]);
    program.push(
        "footprint",
        AddSketchArgs::on(on_origin(FrameAxis::Z), footprint),
    );
    program.push(
        "block",
        extrusion("footprint", Extents::blind(1.0), Combine::NewBody),
    );
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.1)),
    );
    let part = program.build_model(registry).unwrap();
    let model = part.topology();
    let along_x = model
        .edges
        .iter()
        .find(|(_, e)| {
            let (a, b) = (
                model.vertices[&e.start_vertex].point,
                model.vertices[&e.end_vertex].point,
            );
            let near = |p: [f64; 3], q: [f64; 3]| (0..3).all(|k| (p[k] - q[k]).abs() < 1e-9);
            let (a, b) = (
                [a[0].to_f64(), a[1].to_f64(), a[2].to_f64()],
                [b[0].to_f64(), b[1].to_f64(), b[2].to_f64()],
            );
            let (o, x) = ([0.0, 0.0, 0.0], [2.0, 0.0, 0.0]);
            (near(a, o) && near(b, x)) || (near(a, x) && near(b, o))
        })
        .map(|(id, _)| part.name_of(*id).unwrap().to_string())
        .expect("the block has an edge from the origin to (2, 0, 0)");
    program.push(
        "rod",
        SweepArgs {
            path: Some(PathRef::Edge(ElementReference::new(
                geop_ops_model::model(),
                along_x,
            ))),
            ..swept("profile", "unused")
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 2);
    assert!(part.face_id("sweep(rod,start)").is_ok());
    assert!(part.face_id("sweep(rod,end)").is_ok());
}

/// The block and the post's sketch of [`block_and_post`], and `steps`
/// after them, built.
fn on_block(z0: f64, z1: f64, steps: Vec<ProgramStep>) -> GeopResult<NamedModel<Design>> {
    block_and_post(z0, z1, steps).build_model(registry)
}

/// A symmetric extrude of a unit square on the Z plane by total thickness
/// 1 spans z in [-0.5, 0.5] — the sketch plane through the middle of the
/// solid, not at one face of it.
#[test]
fn symmetric_extrude_is_centered_on_the_sketch_plane() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push("square", AddSketchArgs::on(on_origin(FrameAxis::Z), s));
    program.push("slab", extrude("square", 1.0, true));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    for v in part.topology().vertices.values() {
        assert!(
            v.point[2].could_be_equal(S::from_f64(0.5))
                || v.point[2].could_be_equal(S::from_f64(-0.5)),
            "vertex at unexpected height: {:?}",
            v.point
        );
    }
}

/// A plate with a round hole and a slot-shaped outline (lines + half
/// circles), extruded upwards from the XY plane.
#[test]
fn extrude_slot_plate_with_hole() {
    let mut s = Sketch::new();
    let p = [
        s.add_point(n(0.0), n(0.0)),
        s.add_point(n(2.0), n(0.0)),
        s.add_point(n(2.0), n(1.0)),
        s.add_point(n(0.0), n(1.0)),
    ];
    s.add_line(p[0], p[1]);
    s.add_arc(p[1], p[2], n(PI));
    s.add_line(p[2], p[3]);
    s.add_arc(p[3], p[0], n(PI));
    let c = s.add_point(n(1.0), n(0.5));
    s.add_circle(c, n(0.25));
    let mut program = Program::new();
    program.push("slot", AddSketchArgs::on(on_origin(FrameAxis::Z), s));
    program.push("plate", extrude("slot", 0.5, false));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // 2 caps + 6 outer walls (the half circles are split in quarters) + 4
    // hole walls.
    assert_eq!(part.topology().faces.len(), 12);
}

/// A half disc whose diameter is the axis itself: revolving gives a sphere,
/// from an arc profile.
#[test]
fn revolve_half_disc_is_sphere() {
    let mut s = Sketch::new();
    let top = s.add_point(n(0.0), n(1.0));
    let bottom = s.add_point(n(0.0), n(-1.0));
    let axis = s.add_line(bottom, top);
    s.add_arc(top, bottom, n(PI));
    let mut program = Program::new();
    program.push("half_disc", AddSketchArgs::on(on_origin(FrameAxis::X), s));
    program.push(
        "ball",
        RevolveArgs {
            sketch: whole_sketch("half_disc"),
            axis: Some(RevolveAxis::Sketch(sketch_element("half_disc", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    // The half circle is split into two quarters.
    assert_eq!(part.topology().faces.len(), 8);
}

/// A profile straddling the axis is rejected rather than revolved into a
/// self-intersecting solid.
#[test]
fn revolve_across_axis_fails() {
    let mut s = Sketch::new();
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(2.0));
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    polygon(&mut s, &[[-0.5, 0.0], [1.0, 0.0], [1.0, 1.0], [-0.5, 1.0]]);
    let mut program = Program::new();
    program.push("profile", AddSketchArgs::on(on_origin(FrameAxis::Z), s));
    program.push(
        "bad",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(RevolveAxis::Sketch(sketch_element("profile", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    assert!(program.build_model(registry).is_err());
}

/// Revolves `s`, sketched on the origin's zx plane, around `axis`.
fn revolved_on_zx(s: Sketch, axis: RevolveAxis) -> GeopResult<NamedModel<Design>> {
    let mut program = Program::new();
    program.push("profile", AddSketchArgs::on(on_origin(FrameAxis::Y), s));
    program.push(
        "ring",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(axis),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.build_model(registry)
}

/// A square clear of an axis from outside the sketch — the origin's z axis,
/// which lies in the sketch's plane — revolves into a ring with a square
/// cross section.
#[test]
fn revolve_square_around_an_outside_axis_is_a_ring() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]);
    let z = RevolveAxis::Datum(origin_axis(FrameAxis::Z));
    let part = revolved_on_zx(s, z).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 16);
}

/// A circle beside a construction line of its own sketch, not touching it,
/// revolves into a torus.
#[test]
fn revolve_circle_clear_of_its_axis_is_a_torus() {
    let mut s = Sketch::new();
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(1.0));
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    let center = s.add_point(n(3.0), n(0.0));
    s.add_circle(center, n(1.0));
    let part = revolved_on_zx(s, RevolveAxis::Sketch(sketch_element("profile", axis))).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 16);
}

/// A profile touching an axis from outside the sketch is refused: nothing
/// says which of its edges lie on that axis.
#[test]
fn revolve_touching_an_outside_axis_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let z = RevolveAxis::Datum(origin_axis(FrameAxis::Z));
    let Err(error) = revolved_on_zx(s, z) else {
        panic!("revolved");
    };
    assert!(error.root_message().contains("touches"), "{error:?}");
}

/// An axis out of the sketch's plane is refused.
#[test]
fn revolve_around_an_axis_off_the_plane_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [2.0, 1.0], [2.0, 2.0], [1.0, 2.0]]);
    let y = RevolveAxis::Datum(origin_axis(FrameAxis::Y));
    let Err(error) = revolved_on_zx(s, y) else {
        panic!("revolved");
    };
    assert!(error.root_message().contains("plane"), "{error:?}");
}

/// Extrude refers to a sketch by name; naming a solid instead is an error.
#[test]
fn extrude_of_non_sketch_fails() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push("square", AddSketchArgs::on(on_origin(FrameAxis::Z), s));
    program.push("slab", extrude("square", 1.0, false));
    program.push("again", extrude("extrude(slab)", 1.0, false));
    assert!(program.build_model(registry).is_err());
}

/// The boss again, joined to the block by its own extrude: the result is
/// named after the extrude, and the block and the tool are gone.
#[test]
fn extrude_combines_with_its_target() {
    let mut block = Sketch::new();
    polygon(
        &mut block,
        &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
    );
    let mut boss = Sketch::new();
    let c = boss.add_point(n(0.5), n(0.5));
    boss.add_circle(c, n(0.3));
    let mut program = Program::new();
    program.push("base", AddSketchArgs::on(on_origin(FrameAxis::Z), block));
    program.push("block", extrude("base", 1.0, false));
    program.push(
        "boss_sketch",
        AddSketchArgs::on(on_face("extrude(block,end)"), boss),
    );
    program.push(
        "boss",
        ExtrudeArgs {
            output: Output::Solid(Combine::join("extrude(block)")),
            ..extrude("boss_sketch", 0.4, false)
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
    assert!(part.solid_id("extrude(boss)").is_ok());
    assert!(part.solid_id("extrude(block)").is_err());
    assert!(part.solid_id("extrude(boss,tool)").is_err());
    // The faces keep the names their extrudes gave them.
    assert!(part.face_id("extrude(boss,end)").is_ok());
    assert!(part.face_id("extrude(block,start)").is_ok());
}

/// A rectangle beside a construction axis, its inner edge constrained onto
/// the axis, revolved as far as `extent` says — or, as a `face`, its curves
/// swept into faces standing on their own.
fn revolved_rectangle(extent: Extents, face: bool) -> GeopResult<NamedModel<Design>> {
    let mut s = Sketch::new();
    let a0 = s.add_point(n(0.0), n(-1.0));
    let a1 = s.add_point(n(0.0), n(2.0));
    let axis = s.add_line(a0, a1);
    s.set_construction(axis, true);
    let (p, _) = polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    for point in [p[0], p[3]] {
        s.constrain(Constraint::PointOnCurve { point, curve: axis });
    }
    let mut program = Program::new();
    program.push("profile", AddSketchArgs::on(on_origin(FrameAxis::Y), s));
    program.push(
        "turn",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(RevolveAxis::Sketch(sketch_element("profile", axis))),
            extent,
            output: if face {
                Output::Faces { up_to: None }
            } else {
                Output::default()
            },
        },
    );
    program.build_model(registry)
}

/// Any angle up to a full turn, either way, symmetric or each side its
/// own: a partial turn is closed by two caps sharing the edge on the axis,
/// three sides off the axis sweeping a face per span of at most a quarter
/// turn; sides adding up to a full turn close on themselves.
#[test]
fn revolve_turns_any_angle() {
    for (extent, faces) in [
        // A full turn: a cylinder, three profile edges off the axis, a
        // quadrant face each per 90°.
        (Extents::blind(360.0), 12),
        (Extents::blind(90.0), 3 + 2),
        (Extents::blind(-270.0), 9 + 2),
        (Extents::blind(45.0), 3 + 2),
        (both_ways(180.0), 6 + 2),
        (two_ways(30.0, 100.0), 6 + 2),
        (two_ways(200.0, 160.0), 12),
    ] {
        let part = revolved_rectangle(extent.clone(), false).unwrap();
        assert_valid(&part);
        assert_eq!(part.topology().faces.len(), faces, "{extent:?}");
        assert_eq!(part.face_id("revolve(turn,start)").is_ok(), faces != 12);
    }
    assert!(revolved_rectangle(two_ways(200.0, 200.0), false).is_err());
}

/// A square with a square hole, clear of the axis: a full turn sweeps the
/// hole into a void inside the ring, a quarter turn a tunnel through it.
#[test]
fn revolve_with_a_hole() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[1.0, 1.0], [4.0, 1.0], [4.0, 4.0], [1.0, 4.0]]);
    polygon(&mut s, &[[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0]]);
    let z = RevolveAxis::Datum(origin_axis(FrameAxis::Z));
    let full = revolved_on_zx(s.clone(), z.clone()).unwrap();
    assert_valid(&full);
    let ring = full.solid_id("revolve(ring)").unwrap();
    assert_eq!(full.topology().get_solid(ring).unwrap().shells.len(), 2);

    let mut program = Program::new();
    program.push("profile", AddSketchArgs::on(on_origin(FrameAxis::Y), s));
    program.push(
        "ring",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(z),
            extent: Extents::blind(90.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let quarter = program.build_model(registry).unwrap();
    assert_valid(&quarter);
    assert_eq!(quarter.topology().faces.len(), 4 + 4 + 2);
}

/// One sketch is one area: a sketch of two separate ones is refused.
#[test]
fn a_sketch_of_two_areas_is_refused() {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    polygon(&mut s, &[[2.0, 0.0], [3.0, 0.0], [3.0, 1.0], [2.0, 1.0]]);
    let mut program = Program::new();
    program.push("squares", AddSketchArgs::on(on_origin(FrameAxis::Z), s));
    program.push("slabs", extrude("squares", 1.0, false));
    let Err(error) = program.build_model(registry) else {
        panic!("extruded");
    };
    assert!(error.root_message().contains("separate areas"), "{error:?}");
}

/// A unit square on the Z plane, and `step` built on it.
fn on_unit_square(step: ExtrudeArgs) -> GeopResult<NamedModel<Design>> {
    let mut s = Sketch::new();
    polygon(&mut s, &[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]);
    let mut program = Program::new();
    program.push("square", AddSketchArgs::on(on_origin(FrameAxis::Z), s));
    program.push("e", step);
    program.build_model(registry)
}

/// Each side its own length: from one below the plane to the other above.
#[test]
fn extrude_each_side_its_own_length() {
    let part = on_unit_square(ExtrudeArgs {
        extent: two_ways(1.0, 0.5),
        ..extrude("square", 1.0, false)
    })
    .unwrap();
    assert_valid(&part);
    let height = |name: &str| {
        let v = part.vertex_id(name).unwrap();
        part.topology().get_vertex(v).unwrap().point[2]
    };
    assert!(height("extrude(e,square,p0,start)").could_be_equal(S::from_f64(-0.5)));
    assert!(height("extrude(e,square,p0,end)").could_be_equal(S::from_f64(1.0)));
}

/// As a face, the area's outline sweeps a tube of faces standing on their
/// own — no caps, no solid.
#[test]
fn extrude_an_area_as_faces() {
    let part = on_unit_square(ExtrudeArgs {
        output: Output::Faces { up_to: None },
        ..extrude("square", 1.0, false)
    })
    .unwrap();
    let model = part.topology();
    assert!(model.solids.is_empty());
    assert_eq!(part.sheet_face_names().len(), 4);
    if let Err(e) = validate(&ValidationParameters::default(), model) {
        panic!("{e:?}");
    }
}

/// As a face, a sketch of a line enclosing nothing sweeps a single face —
/// which a solid can then be split with; a revolved arc sweeps a surface of
/// revolution.
#[test]
fn sweep_a_line_as_a_face() {
    let mut s = Sketch::new();
    let a = s.add_point(n(1.0), n(0.0));
    let b = s.add_point(n(2.0), n(1.0));
    let line = s.add_line(a, b);
    let mut program = Program::new();
    program.push("slant", AddSketchArgs::on(on_origin(FrameAxis::Y), s));
    program.push(
        "wall",
        ExtrudeArgs {
            output: Output::Faces { up_to: None },
            ..extrude("slant", 1.0, true)
        },
    );
    program.push(
        "cone",
        RevolveArgs {
            sketch: whole_sketch("slant"),
            axis: Some(RevolveAxis::Datum(origin_axis(FrameAxis::Z))),
            extent: Extents::blind(360.0),
            output: Output::Faces { up_to: None },
        },
    );
    let part = program.build_model(registry).unwrap();
    assert!(part.topology().solids.is_empty());
    assert!(part.face_id(&format!("extrude(wall,slant,{line})")).is_ok());
    assert_eq!(part.sheet_face_names().len(), 1 + 4);
    if let Err(e) = validate(&ValidationParameters::default(), part.topology()) {
        panic!("{e:?}");
    }
}

/// An extrude going up to the next face of a target: the post sketched
/// under the block grows up to the block's bottom and joins it there;
/// cut into the block, it cuts up to where it comes out of it.
#[test]
fn extrude_up_to_next() {
    let up_to_next = |combine: Combine| {
        as_step(
            "post",
            ExtrudeArgs {
                extent: one_way(Extent::UpToNext),
                output: Output::Solid(combine),
                ..extrude("post_sketch", 1.0, false)
            },
        )
    };
    let target = || "extrude(block)".to_string();

    let joined = on_block(2.0, 3.0, vec![up_to_next(Combine::join(target()))]).unwrap();
    assert_valid(&joined);
    assert_eq!(joined.solid_names(), ["extrude(post)"]);
    // The post's foot on the sketch, its head on the block's bottom: none of
    // the tool past that is left.
    assert_eq!(heights(&joined), [0, 2000, 3000]);
    // The block's six faces, the post's four walls and its foot — and the
    // block's top face in two: the tool, built long enough to reach past
    // the block, was imprinted on it where it came out again, and the
    // pieces it leaves are not merged back.
    assert_eq!(joined.topology().faces.len(), 6 + 4 + 1 + 1);

    let cut = on_block(0.0, 1.0, vec![up_to_next(Combine::cut(target()))]).unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [0, 1000]);
    assert_eq!(cut.topology().faces.len(), 6 + 4);

    // Nothing ahead to stop at.
    let Err(error) = on_block(-2.0, -1.0, vec![up_to_next(Combine::join(target()))]) else {
        panic!("extruded");
    };
    assert!(error.root_message().contains("nothing"), "{error:?}");

    // A new body stops at the next face of any solid, as a body of its
    // own, the block left as it was.
    let apart = on_block(2.0, 3.0, vec![up_to_next(Combine::NewBody)]).unwrap();
    assert_valid(&apart);
    assert_eq!(apart.solid_names(), ["extrude(block)", "extrude(post)"]);
    let block = apart.solid_id("extrude(block)").unwrap();
    assert_eq!(apart.topology().solid_faces(block).unwrap().len(), 6);
    let post = apart.solid_id("extrude(post)").unwrap();
    let mut post_heights: Vec<i64> = apart
        .topology()
        .iter_body_vertices(post)
        .unwrap()
        .map(|v| (apart.topology().vertices[&v].point[2].to_f64() * 1000.0).round() as i64)
        .collect();
    post_heights.sort();
    post_heights.dedup();
    assert_eq!(post_heights, [0, 2000]);
}

/// The post of [`on_block`] extruded up from the Z plane as far as `side1`
/// says, combined with the block as `combine` says — `reversed` turning it
/// the other way.
fn post_to_block(
    z0: f64,
    z1: f64,
    side1: Extent,
    reversed: bool,
    op: BooleanOp,
) -> GeopResult<NamedModel<Design>> {
    let post = ExtrudeArgs {
        extent: Extents {
            side1,
            symmetric: false,
            side2: None,
            reversed,
        },
        output: Output::Solid(Combine::With {
            op,
            target: solid("extrude(block)"),
        }),
        ..extrude("post_sketch", 1.0, false)
    };
    on_block(z0, z1, vec![as_step("post", post)])
}

/// The heights of `part`'s vertices, in thousandths.
fn heights(part: &NamedModel<Design>) -> Vec<i64> {
    let mut z: Vec<i64> = part
        .topology()
        .vertices
        .values()
        .map(|v| (v.point[2].to_f64() * 1000.0).round() as i64)
        .collect();
    z.sort();
    z.dedup();
    z
}

/// A cut or an intersection up to next from a sketch outside the target
/// goes up to the target, and from there as far as its next face: through
/// the block it meets.
#[test]
fn cut_and_intersect_up_to_next_from_outside() {
    let cut = post_to_block(2.0, 3.0, Extent::UpToNext, false, BooleanOp::Difference).unwrap();
    assert_valid(&cut);
    assert_eq!(cut.solid_names(), ["extrude(post)"]);
    assert_eq!(heights(&cut), [2000, 3000]);
    assert_eq!(cut.topology().faces.len(), 6 + 4);

    let kept = post_to_block(2.0, 3.0, Extent::UpToNext, false, BooleanOp::Intersection).unwrap();
    assert_valid(&kept);
    assert_eq!(heights(&kept), [2000, 3000]);
    assert_eq!(kept.topology().faces.len(), 6);
}

/// Through all goes through the whole target, whichever way it is combined.
#[test]
fn through_all() {
    let cut = post_to_block(2.0, 3.0, Extent::ThroughAll, false, BooleanOp::Difference).unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [2000, 3000]);
    let kept = post_to_block(2.0, 3.0, Extent::ThroughAll, false, BooleanOp::Intersection).unwrap();
    assert_valid(&kept);
    assert_eq!(kept.topology().faces.len(), 6);
    // Nothing of the block ahead the other way.
    assert!(post_to_block(2.0, 3.0, Extent::ThroughAll, true, BooleanOp::Difference).is_err());
}

/// Turned the other way, a cut from a sketch above the block goes down into
/// it; going away from it, it meets nothing, and says so.
#[test]
fn reversed_cut_up_to_next() {
    let cut = post_to_block(-1.0, 0.0, Extent::UpToNext, true, BooleanOp::Difference).unwrap();
    assert_valid(&cut);
    assert_eq!(heights(&cut), [-1000, 0]);
    let Err(error) = post_to_block(-1.0, 0.0, Extent::UpToNext, false, BooleanOp::Difference)
    else {
        panic!("cut");
    };
    assert!(error.root_message().contains("nothing"), "{error:?}");
}

/// A face going up to the next face of a solid stops where it meets it,
/// and leaves the solid as it is; through all, it goes past it.
#[test]
fn faces_up_to_next_and_through_all() {
    let face = |side1: Extent| {
        let post = ExtrudeArgs {
            extent: Extents {
                side1,
                symmetric: false,
                side2: None,
                reversed: false,
            },
            output: Output::Faces {
                up_to: Some(solid("extrude(block)")),
            },
            ..extrude("post_sketch", 1.0, false)
        };
        on_block(2.0, 3.0, vec![as_step("post", post)]).unwrap()
    };
    let up_to = face(Extent::UpToNext);
    let model = up_to.topology();
    if let Err(e) = validate(&ValidationParameters::default(), model) {
        panic!("{e:?}");
    }
    // The block untouched, the post's walls from the plane to its bottom.
    let block = up_to.solid_id("extrude(block)").unwrap();
    assert_eq!(model.solid_faces(block).unwrap().len(), 6);
    assert_eq!(up_to.sheet_face_names().len(), 4);
    assert_eq!(heights(&up_to), [0, 2000, 3000]);

    let through = face(Extent::ThroughAll);
    assert!(heights(&through).last().unwrap() > &3000);
}

/// A post joined up to the next face of a wedge hanging over half of it,
/// its edge pointing down: the half under the wedge meets it, the other
/// half never does, so nothing stops the post all round. It goes through
/// all — not flat up to the wedge's lowest edge, where its end would touch
/// the wedge along that existing edge alone.
#[test]
fn join_up_to_next_partly_under_a_wedge_goes_through_all() {
    let mut wedge = Sketch::new();
    // Sketch x along world x, sketch y along world -z: the edge at `z = 2`,
    // the top at `z = 4`.
    polygon(&mut wedge, &[[1.0, -2.0], [3.0, -4.0], [-1.0, -4.0]]);
    let y_plane = on_origin(FrameAxis::Y);
    let wedge_steps = vec![
        as_step("wedge_sketch", AddSketchArgs::on(y_plane, wedge)),
        // From `y = 0` to `y = 1`: over half the post, `y` in `[0.5, 1.5]`.
        as_step("wedge", extrude("wedge_sketch", 1.0, false)),
        as_step(
            "post",
            ExtrudeArgs {
                extent: one_way(Extent::UpToNext),
                output: Output::Solid(Combine::join("extrude(wedge)")),
                ..extrude("post_sketch", 1.0, false)
            },
        ),
    ];
    let mut program = Program::new();
    let mut post = Sketch::new();
    polygon(&mut post, &[[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]]);
    let z_plane = on_origin(FrameAxis::Z);
    program.push("post_sketch", AddSketchArgs::on(z_plane, post));
    program.steps.extend(wedge_steps);
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["extrude(post)"]);
    // Past the wedge's top.
    assert!(
        heights(&part).last().unwrap() > &4000,
        "{:?}",
        heights(&part)
    );
}

/// A step that fails is reported by id, and does not stop the run; the
/// part before it is what the steps before it built.
#[test]
fn runner_reports_the_failing_step() {
    let mut program = box_with_drill_hole();
    edit_args(step(&mut program, "hole"), |args: &mut ExtrudeArgs| {
        args.output = Output::Solid(Combine::cut("extrude(nothing)"))
    });
    let mut part = Part::default();
    let output = runner_of(registry().unwrap()).run(&mut part, &program, None);
    let errors: Vec<(String, String)> = (output.step_errors.into_iter())
        .map(|(step, error)| (step, error.to_string()))
        .collect();
    let [(id, error)] = errors.as_slice() else {
        panic!("one step fails: {errors:?}");
    };
    assert_eq!(id, "hole");
    assert!(error.contains("extrude(nothing)"), "{error}");
    // The model the steps before it built is there for it to build on.
    let before = part.before("hole");
    let built = read_model(&before).unwrap();
    assert!(built.solid_id("extrude(box)").is_ok());
}
