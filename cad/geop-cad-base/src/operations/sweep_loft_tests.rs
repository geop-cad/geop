//! Sweeps along paths and lofts through profiles, run as programs: the
//! solids and faces they build, combined with others, and what they refuse.

use std::f64::consts::PI;

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_sketch::{Constraint, PointId};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, LoftArgs, Orientation, SweepArgs};
use geop_ops_sketch::{AddSketchArgs, Sketch};

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

/// The origin's plane square to `normal`: `X` draws in `(y, z)`, `Y` in
/// `(x, -z)`, `Z` in `(x, y)`.
fn base(normal: FrameAxis) -> EntityRef {
    EntityRef::datum_component(ORIGIN, DatumComponent::Plane(normal))
}

fn sketch(plane: EntityRef, sketch: Sketch) -> AddSketchArgs {
    AddSketchArgs {
        plane: Some(plane),
        sketch,
        ..Default::default()
    }
}

/// A closed polygon through `corners`.
fn polygon(s: &mut Sketch, corners: &[[f64; 2]]) {
    let p: Vec<PointId> = corners
        .iter()
        .map(|c| s.add_point(n(c[0]), n(c[1])))
        .collect();
    for i in 0..p.len() {
        s.add_line(p[i], p[(i + 1) % p.len()]);
    }
}

/// The square of side `2 half` around the origin.
fn square(half: f64) -> Sketch {
    let mut s = Sketch::new();
    polygon(
        &mut s,
        &[[-half, -half], [half, -half], [half, half], [-half, half]],
    );
    s
}

/// The circle of radius `r` around `(x, y)`.
fn circle(x: f64, y: f64, r: f64) -> Sketch {
    let mut s = Sketch::new();
    let c = s.add_point(n(x), n(y));
    s.add_circle(c, n(r));
    s
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
        profile: profile.into(),
        path: path.into(),
        orientation: Orientation::FollowPath,
        twist: 0.0,
        end_scale: 1.0,
        rails: Vec::new(),
        face: false,
        combine: Combine::NewBody,
    }
}

fn lofted(profiles: &[&str]) -> LoftArgs {
    LoftArgs {
        profiles: profiles.iter().map(|p| p.to_string()).collect(),
        matches: Vec::new(),
        guides: Vec::new(),
        face: false,
        combine: Combine::NewBody,
    }
}

/// The plane `distance` above the `xy` plane, as the datum `name`.
fn lifted(program: &mut Program, name: &str, distance: f64) {
    program.push(
        name,
        AddDatumArgs {
            selection: vec![base(FrameAxis::Z)],
            construction: Construction::Offset { distance },
        },
    );
}

/// A square tube round a corner: mitred where the two lines of the path
/// meet, a solid of eight walls and two caps.
#[test]
fn sweep_a_square_round_a_corner() {
    let mut program = Program::new();
    program.push("profile", sketch(base(FrameAxis::X), square(0.25)));
    program.push(
        "path",
        sketch(
            base(FrameAxis::Z),
            polyline(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]),
        ),
    );
    program.push("tube", swept("profile", "path"));
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("profile", sketch(base(FrameAxis::X), circle(0.0, 0.0, 0.3)));
    program.push("path", sketch(base(FrameAxis::Z), bend()));
    program.push("pipe", swept("profile", "path"));
    let part = program.build::<S>(&NoFiles).unwrap();
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
            selection: vec![base(FrameAxis::Y)],
            construction: Construction::Offset { distance: 3.0 },
        },
    );
    program.push("profile", sketch(EntityRef::datum("far_plane"), far));
    program.push("path", sketch(base(FrameAxis::Z), bend()));
    program.push("bar", swept("profile", "path"));
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("profile", sketch(base(FrameAxis::Y), circle(2.0, 0.0, 0.5)));
    program.push("path", sketch(base(FrameAxis::Z), circle(0.0, 0.0, 2.0)));
    program.push("ring", swept("profile", "path"));
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 16);
    assert!(part.face_id("sweep(ring,start)").is_err());
}

/// A pipe through a bend, joined to a block it starts inside of.
#[test]
fn sweep_joins_a_block() {
    let mut program = Program::new();
    let mut block = Sketch::new();
    polygon(
        &mut block,
        &[[-1.0, -1.0], [0.5, -1.0], [0.5, 1.0], [-1.0, 1.0]],
    );
    program.push("block_sketch", sketch(base(FrameAxis::Z), block));
    program.push(
        "block",
        ExtrudeArgs {
            sketch: "block_sketch".into(),
            extent: Extents {
                side1: Extent::Blind(1.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push("profile", sketch(base(FrameAxis::X), circle(0.0, 0.0, 0.3)));
    program.push("path", sketch(base(FrameAxis::Z), bend()));
    program.push(
        "pipe",
        SweepArgs {
            combine: Combine::Union {
                target: "extrude(block)".into(),
            },
            ..swept("profile", "path")
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
            base(FrameAxis::X),
            polyline(&[[-0.5, 0.0], [0.0, 0.5], [0.5, 0.0]]),
        ),
    );
    program.push("path", sketch(base(FrameAxis::Z), bend()));
    program.push(
        "sheet",
        SweepArgs {
            face: true,
            ..swept("profile", "path")
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("profile", sketch(base(FrameAxis::X), circle(0.0, 0.0, 0.3)));
    program.push("same", swept("profile", "profile"));
    assert!(program.build::<S>(&NoFiles).is_err());

    let mut kinked = Sketch::new();
    let p = [
        kinked.add_point(n(0.0), n(0.0)),
        kinked.add_point(n(2.0), n(0.0)),
        kinked.add_point(n(1.0), n(1.0)),
    ];
    kinked.add_line(p[0], p[1]);
    kinked.add_arc(p[1], p[2], n(PI / 2.0));
    let mut program = Program::new();
    program.push("profile", sketch(base(FrameAxis::X), circle(0.0, 0.0, 0.3)));
    program.push("path", sketch(base(FrameAxis::Z), kinked));
    program.push("pipe", swept("profile", "path"));
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("built");
    };
    assert!(error.root_message().contains("corner"), "{error:?}");
}

/// A square below, a circle above: a solid of four ruled walls and two
/// caps.
#[test]
fn loft_a_square_to_a_circle() {
    let mut program = Program::new();
    program.push("bottom", sketch(base(FrameAxis::Z), square(1.0)));
    lifted(&mut program, "top_plane", 2.0);
    program.push(
        "top",
        sketch(EntityRef::datum("top_plane"), circle(0.0, 0.0, 0.6)),
    );
    program.push("transition", lofted(&["bottom", "top"]));
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("block_sketch", sketch(base(FrameAxis::Z), block));
    program.push(
        "block",
        ExtrudeArgs {
            sketch: "block_sketch".into(),
            extent: Extents::blind(1.5),
            face: false,
            combine: Combine::NewBody,
        },
    );
    lifted(&mut program, "low", -0.5);
    program.push("a", sketch(EntityRef::datum("low"), square(1.0)));
    lifted(&mut program, "middle", 0.75);
    program.push(
        "b",
        sketch(EntityRef::datum("middle"), circle(0.0, 0.0, 0.7)),
    );
    lifted(&mut program, "high", 2.0);
    program.push("c", sketch(EntityRef::datum("high"), square(0.5)));
    program.push(
        "pocket",
        LoftArgs {
            combine: Combine::Difference {
                target: "extrude(block)".into(),
            },
            ..lofted(&["a", "b", "c"])
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
        sketch(base(FrameAxis::Z), polyline(&[[-1.0, 0.0], [1.0, 0.0]])),
    );
    lifted(&mut program, "high_plane", 1.0);
    let mut arc = Sketch::new();
    let p = [
        arc.add_point(n(1.0), n(0.0)),
        arc.add_point(n(-1.0), n(0.0)),
    ];
    arc.add_arc(p[0], p[1], n(PI / 2.0));
    program.push("high", sketch(EntityRef::datum("high_plane"), arc));
    program.push(
        "skin",
        LoftArgs {
            face: true,
            ..lofted(&["low", "high"])
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("only", sketch(base(FrameAxis::Z), square(1.0)));
    program.push("one", lofted(&["only"]));
    assert!(program.build::<S>(&NoFiles).is_err());

    let mut holed = square(1.0);
    let c = holed.add_point(n(0.0), n(0.0));
    holed.add_circle(c, n(0.3));
    let mut program = Program::new();
    program.push("holed", sketch(base(FrameAxis::Z), holed));
    lifted(&mut program, "top_plane", 1.0);
    program.push("top", sketch(EntityRef::datum("top_plane"), square(0.5)));
    program.push("loft", lofted(&["holed", "top"]));
    let Err(error) = program.build::<S>(&NoFiles) else {
        panic!("built");
    };
    assert!(error.root_message().contains("holes"), "{error:?}");
}

/// A block with a slab-shaped top and bottom at `z = ±0.5`, spanning
/// `[-1, 1.5] x [-3, 3]` — for the solids below to be cut from.
fn slab(program: &mut Program) {
    let mut block = Sketch::new();
    polygon(
        &mut block,
        &[[-1.0, -3.0], [1.5, -3.0], [1.5, 3.0], [-1.0, 3.0]],
    );
    program.push("slab_sketch", sketch(base(FrameAxis::Z), block));
    program.push(
        "slab",
        ExtrudeArgs {
            sketch: "slab_sketch".into(),
            extent: Extents {
                side1: Extent::Blind(1.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            face: false,
            combine: Combine::NewBody,
        },
    );
}

/// A torus cut from a slab it crosses: the ring's part inside the slab
/// leaves a curved channel through it.
#[test]
fn sweep_a_ring_cuts_a_slab() {
    let mut program = Program::new();
    slab(&mut program);
    program.push("profile", sketch(base(FrameAxis::Y), circle(2.0, 0.0, 0.3)));
    program.push("path", sketch(base(FrameAxis::Z), circle(0.0, 0.0, 2.0)));
    program.push(
        "channel",
        SweepArgs {
            combine: Combine::Difference {
                target: "extrude(slab)".into(),
            },
            ..swept("profile", "path")
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
            selection: vec![base(FrameAxis::X)],
            construction: Construction::Offset { distance: -2.0 },
        },
    );
    program.push(
        "profile",
        sketch(EntityRef::datum("start_plane"), circle(-1.0, 0.0, 0.2)),
    );
    program.push("path", sketch(base(FrameAxis::Z), wave));
    program.push(
        "hole",
        SweepArgs {
            combine: Combine::Difference {
                target: "extrude(slab)".into(),
            },
            ..swept("profile", "path")
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().solids.len(), 1);
}

/// Sweep and loft steps are saved and read back like any other.
#[test]
fn sweep_and_loft_steps_round_trip() {
    let mut program = Program::new();
    program.push("profile", sketch(base(FrameAxis::X), circle(0.0, 0.0, 0.3)));
    program.push("path", sketch(base(FrameAxis::Z), bend()));
    program.push("pipe", swept("profile", "path"));
    program.push("bottom", sketch(base(FrameAxis::Z), square(1.0)));
    lifted(&mut program, "top_plane", 2.0);
    program.push(
        "top",
        sketch(EntityRef::datum("top_plane"), circle(0.0, 0.0, 0.6)),
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
    program.push("d", sketch(base(FrameAxis::Z), d));
    lifted(&mut program, "above", 1.0);
    program.push(
        "triangle",
        sketch(
            EntityRef::Datum {
                name: "above".into(),
                component: None,
            },
            triangle,
        ),
    );
    program.push("loft", lofted(&["d", "triangle"]));
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("rectangle", sketch(base(FrameAxis::Z), rectangle));
    for (name, distance) in [("one", 1.0), ("two", 2.0)] {
        lifted(&mut program, name, distance);
    }
    let on = |name: &str| EntityRef::Datum {
        name: name.into(),
        component: None,
    };
    program.push("triangle", sketch(on("one"), triangle));
    program.push("circle", sketch(on("two"), circle(0.0, 0.0, 0.4)));
    program.push("loft", lofted(&["rectangle", "triangle", "circle"]));
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("square", sketch(base(FrameAxis::Z), square));
    lifted(&mut program, "above", 1.0);
    program.push(
        "round",
        sketch(
            EntityRef::Datum {
                name: "above".into(),
                component: None,
            },
            round,
        ),
    );
    let mut args = lofted(&["square", "round"]);
    args.matches = vec![
        EntityRef::SketchPoint {
            sketch: "square".into(),
            point: p[2],
        },
        EntityRef::SketchPoint {
            sketch: "round".into(),
            point: mark,
        },
    ];
    program.push("loft", args);
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("d", sketch(base(FrameAxis::Z), d));
    lifted(&mut program, "above", 1.0);
    program.push(
        "triangle",
        sketch(
            EntityRef::Datum {
                name: "above".into(),
                component: None,
            },
            triangle,
        ),
    );
    let point = |sketch: &str, point: PointId| EntityRef::SketchPoint {
        sketch: sketch.into(),
        point,
    };
    let mut args = lofted(&["d", "triangle"]);
    args.matches = vec![
        point("d", p[0]),
        point("triangle", q[0]),
        point("d", p[3]),
        point("triangle", q[1]),
    ];
    program.push("loft", args);
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("profile", sketch(base(FrameAxis::X), circle(0.0, 0.0, 0.5)));
    program.push(
        "path",
        sketch(base(FrameAxis::Z), polyline(&[[0.0, 0.0], [2.0, 0.0]])),
    );
    program.push(
        "rail",
        sketch(base(FrameAxis::Z), polyline(&[[0.0, 0.5], [2.0, 1.0]])),
    );
    program.push(
        "cone",
        SweepArgs {
            rails: vec!["rail".into()],
            ..swept("profile", "path")
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("profile", sketch(base(FrameAxis::X), square(0.25)));
    program.push("path", sketch(base(FrameAxis::Z), bend()));
    program.push(
        "bar",
        SweepArgs {
            twist: 90.0,
            end_scale: 0.5,
            ..swept("profile", "path")
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), 3 * 4 + 2);
}

/// A loft between two squares with a guide bowing out beside them, along
/// the middle of one side: the walls bulge with it, joined to a block.
#[test]
fn loft_along_a_guide() {
    let mut program = Program::new();
    program.push("bottom", sketch(base(FrameAxis::Z), square(1.0)));
    lifted(&mut program, "top_plane", 2.0);
    program.push("top", sketch(EntityRef::datum("top_plane"), square(1.0)));
    // In the plane y = 0, drawn in (x, -z): from (1, 0, 0) out to x = 1.5
    // and back to (1, 0, 2).
    let mut bow = Sketch::new();
    let p = [
        bow.add_point(n(1.0), n(0.0)),
        bow.add_point(n(1.5), n(-1.0)),
        bow.add_point(n(1.0), n(-2.0)),
    ];
    bow.add_spline(p.to_vec());
    program.push("guide", sketch(base(FrameAxis::Y), bow));
    program.push(
        "bulge",
        LoftArgs {
            guides: vec!["guide".into()],
            ..lofted(&["bottom", "top"])
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
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
    program.push("profile", sketch(base(FrameAxis::X), circle(0.0, 0.0, 0.5)));
    program.push(
        "path",
        sketch(base(FrameAxis::Z), polyline(&[[0.0, 0.0], [2.0, 0.0]])),
    );
    program.push(
        "rail",
        sketch(base(FrameAxis::Z), polyline(&[[0.0, 0.5], [2.0, 1.0]])),
    );
    program.push(
        "cone",
        SweepArgs {
            rails: vec!["rail".into()],
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
            guides: vec!["rail".into()],
            ..lofted(&["profile", "path"])
        },
    );
    let json = serde_json::to_string(&program).unwrap();
    assert!(json.contains("\"rails\":[\"rail\"]"), "{json}");
    assert!(json.contains("\"fixed_normal\""), "{json}");
    let back: Program = serde_json::from_str(&json).unwrap();
    assert_eq!(back, program);
}
