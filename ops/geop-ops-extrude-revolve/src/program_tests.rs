//! Sweeps along paths and lofts through profiles, run as programs: the
//! solids and faces they build, combined with others, and what they refuse.

use crate::{
    Extents, LoftArgs, Output, PathRef, SweepArgs,
    testing::{Built, both_ways, disc, extrusion, n, on_origin, outline, polygon, sketch},
};
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
};
use geop_lib_sweep::Orientation;
use std::f64::consts::PI;

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{DatumComponent, FrameAxis},
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_sketch::PointId;
use geop_core_topology::validation::{ValidationParameters, validate};
use geop_ops_add_datum::{AddDatum, AddDatumArgs, Construction, Selected};
use geop_ops_datums::{AnyDatum, Datums, ORIGIN, component_name, datums};
use geop_ops_model::Combine;
use geop_ops_model::{Curve, testing::assert_valid};
use geop_ops_sketch::{
    Constraint, PlacedSketch, Sketch, SketchPlane, sketch as sketch_target, sketch_element,
    whole_sketch,
};
use geop_ops_sketch3d::{AddSketch3d, sketch3d_path};

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
    program.push("profile", sketch(on_origin(FrameAxis::X), square(0.25)));
    program.push(
        "path",
        sketch(
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
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", sketch(on_origin(FrameAxis::Z), bend()));
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
    program.push("profile", sketch(on_datum("far_plane"), far));
    program.push("path", sketch(on_origin(FrameAxis::Z), bend()));
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
        sketch(on_origin(FrameAxis::Y), disc([2.0, 0.0], 0.5)),
    );
    program.push(
        "path",
        sketch(on_origin(FrameAxis::Z), disc([0.0, 0.0], 2.0)),
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
    program.push("block_sketch", sketch(on_origin(FrameAxis::Z), block));
    program.push(
        "block",
        extrusion("block_sketch", both_ways(1.0), Combine::NewBody),
    );
    program.push(
        "profile",
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", sketch(on_origin(FrameAxis::Z), bend()));
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
        sketch(
            on_origin(FrameAxis::X),
            polyline(&[[-0.5, 0.0], [0.0, 0.5], [0.5, 0.0]]),
        ),
    );
    program.push("path", sketch(on_origin(FrameAxis::Z), bend()));
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
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
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
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", sketch(on_origin(FrameAxis::Z), kinked));
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
    program.push("bottom", sketch(on_origin(FrameAxis::Z), square(1.0)));
    lifted(&mut program, "top_plane", 2.0);
    program.push("top", sketch(on_datum("top_plane"), disc([0.0, 0.0], 0.6)));
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
    program.push("block_sketch", sketch(on_origin(FrameAxis::Z), block));
    program.push(
        "block",
        extrusion("block_sketch", Extents::blind(1.5), Combine::NewBody),
    );
    lifted(&mut program, "low", -0.5);
    program.push("a", sketch(on_datum("low"), square(1.0)));
    lifted(&mut program, "middle", 0.75);
    program.push("b", sketch(on_datum("middle"), disc([0.0, 0.0], 0.7)));
    lifted(&mut program, "high", 2.0);
    program.push("c", sketch(on_datum("high"), square(0.5)));
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
        sketch(
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
    program.push("high", sketch(on_datum("high_plane"), arc));
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
    program.push("only", sketch(on_origin(FrameAxis::Z), square(1.0)));
    program.push("one", lofted(&["only"]));
    assert!(program.build_model(registry).is_err());

    let mut holed = square(1.0);
    let c = holed.add_point(n(0.0), n(0.0));
    holed.add_circle(c, n(0.3));
    let mut program = Program::new();
    program.push("holed", sketch(on_origin(FrameAxis::Z), holed));
    // Refused for its first profile, wherever the second lies.
    program.push("top", sketch(on_origin(FrameAxis::X), square(0.5)));
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
    program.push("slab_sketch", sketch(on_origin(FrameAxis::Z), block));
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
        sketch(on_origin(FrameAxis::Y), disc([2.0, 0.0], 0.3)),
    );
    program.push(
        "path",
        sketch(on_origin(FrameAxis::Z), disc([0.0, 0.0], 2.0)),
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
        sketch(on_datum("start_plane"), disc([-1.0, 0.0], 0.2)),
    );
    program.push("path", sketch(on_origin(FrameAxis::Z), wave));
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
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.3)),
    );
    program.push("path", sketch(on_origin(FrameAxis::Z), bend()));
    program.push("pipe", swept("profile", "path"));
    program.push("bottom", sketch(on_origin(FrameAxis::Z), square(1.0)));
    program.push(
        "top",
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.6)),
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
    program.push("d", sketch(on_origin(FrameAxis::Z), d));
    lifted(&mut program, "above", 1.0);
    program.push("triangle", sketch(on_datum("above"), triangle));
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
    program.push("rectangle", sketch(on_origin(FrameAxis::Z), rectangle));
    for (name, distance) in [("one", 1.0), ("two", 2.0)] {
        lifted(&mut program, name, distance);
    }
    let on = on_datum;
    program.push("triangle", sketch(on("one"), triangle));
    program.push("circle", sketch(on("two"), disc([0.0, 0.0], 0.4)));
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
    program.push("square", sketch(on_origin(FrameAxis::Z), square));
    lifted(&mut program, "above", 1.0);
    program.push("round", sketch(on_datum("above"), round));
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
    program.push("d", sketch(on_origin(FrameAxis::Z), d));
    lifted(&mut program, "above", 1.0);
    program.push("triangle", sketch(on_datum("above"), triangle));
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
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.5)),
    );
    program.push(
        "path",
        sketch(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.0], [2.0, 0.0]])),
    );
    program.push(
        "rail",
        sketch(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.5], [2.0, 1.0]])),
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
    program.push("profile", sketch(on_origin(FrameAxis::X), square(0.25)));
    program.push("path", sketch(on_origin(FrameAxis::Z), bend()));
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
    program.push("bottom", sketch(on_origin(FrameAxis::Z), square(1.0)));
    lifted(&mut program, "top_plane", 2.0);
    program.push("top", sketch(on_datum("top_plane"), square(1.0)));
    // In the plane y = 0, drawn in (x, -z): from (1, 0, 0) out to x = 1.5
    // and back to (1, 0, 2).
    let mut bow = Sketch::new();
    let p = [
        bow.add_point(n(1.0), n(0.0)),
        bow.add_point(n(1.5), n(-1.0)),
        bow.add_point(n(1.0), n(-2.0)),
    ];
    bow.add_spline(p.to_vec());
    program.push("guide", sketch(on_origin(FrameAxis::Y), bow));
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
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.5)),
    );
    program.push(
        "path",
        sketch(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.0], [2.0, 0.0]])),
    );
    program.push(
        "rail",
        sketch(on_origin(FrameAxis::Z), polyline(&[[0.0, 0.5], [2.0, 1.0]])),
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
        json.contains("\"rails\":[{\"sketch\":{\"target\":\"sketch:rail\"}}]"),
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
    program.push("bottom", sketch(on_origin(FrameAxis::Z), square(1.0)));
    lifted(&mut program, "top_plane", 2.0);
    program.push("top", sketch(on_datum("top_plane"), square(1.0)));
    let mut bow = geop_ops_sketch3d::Sketch3d::new();
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
    program.push("path", sketch(on_origin(FrameAxis::Z), bend()));
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
    program.push("footprint", sketch(on_origin(FrameAxis::Z), footprint));
    program.push(
        "block",
        extrusion("footprint", Extents::blind(1.0), Combine::NewBody),
    );
    program.push(
        "profile",
        sketch(on_origin(FrameAxis::X), disc([0.0, 0.0], 0.1)),
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
