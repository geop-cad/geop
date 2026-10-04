//! Example programs, written in Rust against the operation types.
//!
//! Each refers to what earlier steps built only by name, and so reads as
//! the recipe it is: `extrude(box,end)` is the end cap of the step `box`,
//! whatever internal id it happens to get.

use std::collections::BTreeMap;

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Pose},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_sketch::{CurveId, PointId};
use geop_ops::{
    Design, EntityRef, ORIGIN,
    assembly::{JointKind, Kind, Mate, Motion, joint_parameter},
    parameters::{Material, Parameter, ParameterKind, Parameters, Row},
    part::{ParamValue, State, pose_parameter},
};
use geop_ops_assembly::AddPartArgs;
use geop_ops_booleans::{Combine, SplitArgs};
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{
    Extent, Extents, ExtrudeArgs, LoftArgs, Orientation, RevolveArgs, SweepArgs,
};
use geop_ops_hole::{HoleArgs, HoleKind, Standard, iso::Fit};
use geop_ops_pattern::{Direction, LinearPatternArgs, Spacing};
use geop_ops_sheetmetal::{
    BaseFlangeArgs, EdgeFlangeArgs, FlangePosition, LengthReference, SheetMetalRules,
};
use geop_ops_sketch::{
    AddSketchArgs, Constraint, Sketch,
    references::{Reference, Source},
};
use geop_ops_sketch3d::{AddSketch3dArgs, Constraint3d, Sketch3d};
use geop_ops_subd::{Cage, Mirror, SubdArgs};

use crate::Program;

/// `[x, y]`, as an example is written down.
type P2 = [f64; 2];

/// A number of an example, as design data.
pub(crate) fn n(x: f64) -> Design {
    Design::from_f64(x)
}

/// A pose of an example: at `position`, turned by `degrees` (see
/// [`Pose::from_euler`]).
pub(crate) fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<Design> {
    Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n))
        .expect("an example's rotation is one")
}

/// A closed polygon through `corners`, one line per side: its points and
/// lines.
fn polygon(sketch: &mut Sketch, corners: &[P2]) -> (Vec<PointId>, Vec<CurveId>) {
    let points: Vec<PointId> = corners
        .iter()
        .map(|c| sketch.add_point(n(c[0]), n(c[1])))
        .collect();
    let lines = (0..points.len())
        .map(|i| sketch.add_line(points[i], points[(i + 1) % points.len()]))
        .collect();
    (points, lines)
}

/// Solves `sketch`, which the examples all constrain fully.
pub(crate) fn solved(mut sketch: Sketch) -> Sketch {
    let report = sketch.solve().expect("example sketches are valid");
    assert!(
        report.converged,
        "example sketch does not solve: {report:?}"
    );
    sketch
}

/// A `width` x `depth` rectangle with its first corner at `origin`, drawn
/// roughly and fully constrained.
pub(crate) fn rectangle(sketch: &mut Sketch, origin: P2, width: f64, depth: f64) -> Vec<CurveId> {
    let [x, y] = origin;
    // Deliberately a little off: the constraints decide the shape.
    let (p, l) = polygon(
        sketch,
        &[
            [x + 0.05, y - 0.02],
            [x + width, y + 0.03],
            [x + width - 0.04, y + depth],
            [x, y + depth + 0.01],
        ],
    );
    sketch.constrain(Constraint::Fix {
        point: p[0],
        x: n(x),
        y: n(y),
    });
    sketch.constrain(Constraint::Horizontal { line: l[0] });
    sketch.constrain(Constraint::Horizontal { line: l[2] });
    sketch.constrain(Constraint::Vertical { line: l[1] });
    sketch.constrain(Constraint::Vertical { line: l[3] });
    sketch.constrain(Constraint::Length {
        curve: l[0],
        value: n(width),
    });
    sketch.constrain(Constraint::Length {
        curve: l[1],
        value: n(depth),
    });
    l
}

/// A circle of `radius` around `center`, fully constrained.
pub(crate) fn circle(sketch: &mut Sketch, center: P2, radius: f64) -> CurveId {
    let c = sketch.add_point(n(center[0]), n(center[1]));
    let circle = sketch.add_circle(c, n(radius * 1.1));
    sketch.constrain(Constraint::Fix {
        point: c,
        x: n(center[0]),
        y: n(center[1]),
    });
    sketch.constrain(Constraint::Radius {
        curve: circle,
        value: n(radius),
    });
    circle
}

/// A 2 x 2 x 1 box with a blind hole drilled into its top: a rectangle
/// sketched on the Z plane and extruded up (`box`), and a circle sketched on
/// the box's end cap `extrude(box,end)` and extruded back into it, cutting
/// it out of the box (`hole`) — the drilled box is `extrude(hole)`.
pub fn box_with_drill_hole() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 2.0, 2.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
    );

    let mut hole = Sketch::new();
    circle(&mut hole, [1.0, 1.0], 0.4);
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::Face {
                name: "extrude(box,end)".into(),
            }),
            sketch: solved(hole),
            ..Default::default()
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            extent: Extents::blind(-0.5),
            face: false,
            combine: Combine::Difference {
                target: "extrude(box)".into(),
            },
        },
    );
    program
}

/// A 40 x 40 x 10 mm bracket with a 6 mm hole through its middle: a
/// rectangle sketched on the Z plane and extruded up (`block`), and a
/// circle sketched on the block's end cap and extruded back through its
/// full thickness, cutting it out of the block (`hole`). Unlike
/// `box_with_drill_hole`, the cut ends exactly on the bottom face, so the
/// hole goes all the way through. The program the landing page's agent demo
/// transcript writes, step for step.
pub fn bracket() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 40.0, 40.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(10.0),
            face: false,
            combine: Combine::NewBody,
        },
    );

    let mut hole = Sketch::new();
    circle(&mut hole, [20.0, 20.0], 3.0);
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::Face {
                name: "extrude(block,end)".into(),
            }),
            sketch: solved(hole),
            ..Default::default()
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            extent: Extents::blind(-10.0),
            face: false,
            combine: Combine::Difference {
                target: "extrude(block)".into(),
            },
        },
    );
    program
}

/// A stepped shaft revolved around the world z-axis, cross-drilled through
/// its thinner end: a half section sketched on the X plane, closed by the
/// axis itself (`shaft`), and a circle on the Y plane extruded through both
/// sides of it and cut out of it (`bore`) — the drilled shaft is
/// `extrude(bore)`.
pub fn cross_drilled_shaft() -> Program {
    let mut program = Program::new();

    // Sketch x runs along world y, sketch y along world z.
    let mut section = Sketch::new();
    let (p, l) = polygon(
        &mut section,
        &[
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 1.0],
            [0.6, 1.0],
            [0.6, 3.0],
            [0.0, 3.0],
        ],
    );
    let axis = l[5];
    section.constrain(Constraint::Fix {
        point: p[0],
        x: n(0.0),
        y: n(0.0),
    });
    section.constrain(Constraint::Vertical { line: axis });
    for (line, horizontal) in [
        (l[0], true),
        (l[1], false),
        (l[2], true),
        (l[3], false),
        (l[4], true),
    ] {
        section.constrain(if horizontal {
            Constraint::Horizontal { line }
        } else {
            Constraint::Vertical { line }
        });
    }
    section.constrain(Constraint::Length {
        curve: l[0],
        value: n(1.0),
    });
    section.constrain(Constraint::Length {
        curve: l[1],
        value: n(1.0),
    });
    section.constrain(Constraint::Length {
        curve: l[3],
        value: n(2.0),
    });
    section.constrain(Constraint::Length {
        curve: l[4],
        value: n(0.6),
    });
    program.push(
        "section",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            sketch: solved(section),
            ..Default::default()
        },
    );
    program.push(
        "shaft",
        RevolveArgs {
            sketch: "section".into(),
            axis: Some(EntityRef::SketchCurve {
                sketch: "section".into(),
                curve: axis,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::NewBody,
        },
    );

    // Sketch x runs along world x, sketch y along world -z: a bore across
    // the thin end, at z = 2.2.
    let mut bore = Sketch::new();
    circle(&mut bore, [0.0, -2.2], 0.25);
    program.push(
        "bore_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )),
            sketch: solved(bore),
            ..Default::default()
        },
    );
    program.push(
        "bore",
        ExtrudeArgs {
            sketch: "bore_sketch".into(),
            extent: Extents {
                side1: Extent::Blind(3.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            face: false,
            combine: Combine::Difference {
                target: "revolve(shaft)".into(),
            },
        },
    );
    program
}

/// A plate with a round hole, cut in two: the plate (`plate`) extruded both
/// ways from its sketch, a line across it sketched on the plane through its
/// middle and extruded into a face standing on its own (`cut`), and the
/// plate split along that face (`halves`) into `split(halves,0)` and
/// `split(halves,1)`, one of them with the hole.
pub fn split_plate() -> Program {
    let mut program = Program::new();
    let mut plate = Sketch::new();
    rectangle(&mut plate, [0.0, 0.0], 3.0, 1.0);
    circle(&mut plate, [0.75, 0.5], 0.3);
    program.push(
        "plate_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(plate),
            ..Default::default()
        },
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: "plate_sketch".into(),
            extent: Extents {
                side1: Extent::Blind(0.5),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            face: false,
            combine: Combine::NewBody,
        },
    );

    // Sketch x runs along world x, sketch y along world -z: a line across
    // the plate's thickness at x = 2.
    let mut cut = Sketch::new();
    let (a, b) = (
        cut.add_point(n(2.0), n(-1.0)),
        cut.add_point(n(2.0), n(1.0)),
    );
    let line = cut.add_line(a, b);
    for (point, y) in [(a, -1.0), (b, 1.0)] {
        cut.constrain(Constraint::Fix {
            point,
            x: n(2.0),
            y: n(y),
        });
    }
    program.push(
        "cut_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )),
            sketch: solved(cut),
            ..Default::default()
        },
    );
    program.push(
        "cut",
        ExtrudeArgs {
            sketch: "cut_sketch".into(),
            extent: Extents {
                side1: Extent::Blind(3.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            face: true,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "halves",
        SplitArgs {
            solid: "extrude(plate)".into(),
            face: format!("extrude(cut,cut_sketch,{line})"),
        },
    );
    program
}

/// A box with a round boss standing on it, sketched on a reference plane:
/// the plane half a unit above the box's top (`lifted`, offset from
/// `extrude(box,end)`), a circle sketched on it, and extruded back down
/// through the gap and into the box, joined to it (`boss`) — the part is
/// `extrude(boss)`.
pub fn boss_on_reference_plane() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 2.0, 2.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "lifted",
        AddDatumArgs {
            selection: vec![EntityRef::Face {
                name: "extrude(box,end)".into(),
            }],
            construction: Construction::Offset { distance: 0.5 },
        },
    );
    let mut boss = Sketch::new();
    circle(&mut boss, [1.0, 1.0], 0.5);
    program.push(
        "boss_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum("lifted")),
            sketch: solved(boss),
            ..Default::default()
        },
    );
    program.push(
        "boss",
        ExtrudeArgs {
            sketch: "boss_sketch".into(),
            extent: Extents::blind(-0.75),
            face: false,
            combine: Combine::Union {
                target: "extrude(box)".into(),
            },
        },
    );
    program
}

/// A hand-drawn handle-shaped outline (an irregular blob, splined, with a
/// smaller splined region inside it — as a free-hand sketch in the web
/// editor draws, unconstrained) extruded, then cross-drilled with a
/// circular hole near one end.
///
/// Reproduces a real user bug report (2026-09-27) with the exact reported
/// coordinates, unconstrained, rather than a re-derived "clean" equivalent
/// that might not carry the same numerical case at all: the hole's boolean
/// difference failed with a degenerate-face error from
/// `face_interior_point`. (A second report, of the same outline with a
/// slightly different hole center, left a non-watertight hole instead; its
/// coordinates were not recorded.)
pub fn handle_with_hole() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    let outer_points = [
        (0.01007682018456503, 0.8939399106232252),
        (-0.9693901017551558, 0.8213868052943569),
        (-1.00566665441959, 0.3175457960661055),
        (-0.9452057333121998, -0.3555857922628385),
        (0.09472210973491128, -0.9481028191152622),
        (0.695300592734987, -0.3878316168534466),
        (1.384555093359235, -0.4523232660346628),
        (1.6233271012047248, -0.9224957048864859),
        (1.9214381194506664, 0.15401630544608172),
        (1.2092840203075834, 0.8578895429712221),
    ]
    .map(|(x, y)| outline.add_point(n(x), n(y)));
    let mut outer_loop = outer_points.to_vec();
    outer_loop.push(outer_points[0]);
    outline.add_spline(outer_loop);

    let inner_points = [
        (0.27854547786607586, 0.568500810276078),
        (-0.3241022513094386, 0.561767316095346),
        (-0.3645032163938306, 0.00962079327532167),
        (0.03950643445008967, -0.4246895813818926),
        (0.507484280010964, -0.11831559615858644),
        (0.8643594715897602, 0.12072344725739975),
        (1.1202322504575766, -0.1452495728815144),
        (1.2919363520662426, 0.18805838906471978),
        (1.0427970673791584, 0.470865144655464),
    ]
    .map(|(x, y)| outline.add_point(n(x), n(y)));
    let mut inner_loop = inner_points.to_vec();
    inner_loop.push(inner_points[0]);
    outline.add_spline(inner_loop);

    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "handle",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
    );

    let mut hole = Sketch::new();
    let center = hole.add_point(n(0.32989396295411244), n(-0.5632891678453777));
    hole.add_circle(center, n(0.31727744879927977));
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )),
            sketch: solved(hole),
            ..Default::default()
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            extent: Extents {
                side1: Extent::Blind(2.44),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            face: false,
            combine: Combine::Difference {
                target: "extrude(handle)".into(),
            },
        },
    );
    program
}

/// A rounded-top luggage tag: two vertical sides and a horizontal bottom,
/// closed by a semicircular top tangent to both sides, with a hang-hole
/// through the top (concentric with it) and a rectangular window in the
/// body, inset 0.15 from each of the two sides and the bottom.
pub fn luggage_tag() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    let p0 = outline.add_point(n(-0.7051397478635735), n(0.7809633733011653));
    let p1 = outline.add_point(n(-0.7051397478646341), n(-0.6729821470985564));
    let p4 = outline.add_point(n(0.695311597099524), n(-0.6729821470973941));
    let p7 = outline.add_point(n(0.695311597101934), n(0.7809633732995341));
    let left = outline.add_line(p0, p1);
    let bottom = outline.add_line(p1, p4);
    let right = outline.add_line(p4, p7);
    let top = outline.add_arc(p7, p0, n(std::f64::consts::PI));
    outline.constrain(Constraint::Vertical { line: left });
    outline.constrain(Constraint::Horizontal { line: bottom });
    outline.constrain(Constraint::Vertical { line: right });
    outline.constrain(Constraint::Tangent { a: top, b: right });
    outline.constrain(Constraint::Tangent { a: top, b: left });

    let hole_center = outline.add_point(n(-0.004914075380876311), n(0.7809633733016882));
    let hole = outline.add_circle(hole_center, n(0.30711303824557856));
    outline.constrain(Constraint::Concentric { a: hole, b: top });

    let w_tl = outline.add_point(n(-0.5551397478558091), n(0.08482972707824256));
    let w_tr = outline.add_point(n(0.5453115971039223), n(0.08482972707824256));
    let w_br = outline.add_point(n(0.5453115971014348), n(-0.5229821471055477));
    let w_bl = outline.add_point(n(-0.5551397478598076), n(-0.5229821471049918));
    let w_top = outline.add_line(w_tl, w_tr);
    let w_right = outline.add_line(w_tr, w_br);
    let w_bottom = outline.add_line(w_br, w_bl);
    let w_left = outline.add_line(w_bl, w_tl);
    outline.constrain(Constraint::Horizontal { line: w_top });
    outline.constrain(Constraint::Vertical { line: w_right });
    outline.constrain(Constraint::Horizontal { line: w_bottom });
    outline.constrain(Constraint::Vertical { line: w_left });
    outline.constrain(Constraint::PointLineDistance {
        point: w_br,
        line: right,
        value: n(0.15),
    });
    outline.constrain(Constraint::PointLineDistance {
        point: w_bl,
        line: left,
        value: n(0.15),
    });
    outline.constrain(Constraint::PointLineDistance {
        point: w_br,
        line: bottom,
        value: n(0.15),
    });

    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "tag",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents {
                side1: Extent::Blind(1.0 / 3.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

/// A box with a round hole through it, and a triangle sketched on its side
/// face, revolved around one of its own edges and joined to the box: a double
/// cone whose axis lies in the side face, so that both apexes sit on it and
/// half the cone stands out of the box.
///
/// Reproduces a real user bug report (2026-09-30) with the exact reported
/// coordinates and sketch ids: the `revolve1` union failed in
/// `remesh_edges_x_faces` with "the spokes ... are not coplanar: an apex has
/// no single normal" from `NurbSurface::pole_normal`: `trace_one_side` asked
/// for the cone's normal at its apex, to find which way a curve leaves it.
pub fn revolved_cone_on_box() -> Program {
    let mut program = Program::new();

    let mut outline = Sketch::new();
    let p0 = outline.add_point(n(-2.046526714311965), n(1.182776884816516));
    let p1 = outline.add_point(n(2.2456482324612383), n(-1.2774916901086213));
    let p2 = outline.add_point(n(2.2456482324612383), n(1.182776884816516));
    let p3 = outline.add_point(n(-2.046526714311965), n(-1.2774916901086213));
    let top = outline.add_line(p0, p2);
    outline.constrain(Constraint::Horizontal { line: top });
    let right = outline.add_line(p2, p1);
    outline.constrain(Constraint::Vertical { line: right });
    let bottom = outline.add_line(p1, p3);
    outline.constrain(Constraint::Horizontal { line: bottom });
    let left = outline.add_line(p3, p0);
    outline.constrain(Constraint::Vertical { line: left });
    let center = outline.add_point(n(-0.6692695471128323), n(0.04905776553659026));
    outline.add_circle(center, n(0.771466957205246));
    program.push(
        "sketch1",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: outline,
            ..Default::default()
        },
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: "sketch1".into(),
            extent: Extents::blind(1.8900000000000001),
            face: false,
            combine: Combine::NewBody,
        },
    );

    let mut triangle = Sketch::new();
    let q0 = triangle.add_point(n(-0.2654953575323612), n(1.4039697276195047));
    let q1 = triangle.add_point(n(-0.8683910652620968), n(0.8073854454124275));
    let axis = triangle.add_line(q0, q1);
    let q3 = triangle.add_point(n(1.9210929721682684), n(0.8073854454124275));
    let base = triangle.add_line(q1, q3);
    triangle.constrain(Constraint::Horizontal { line: base });
    triangle.add_line(q3, q0);
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: Some(EntityRef::Face {
                name: format!("extrude(extrude1,sketch1,{right})"),
            }),
            sketch: triangle,
            ..Default::default()
        },
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: "sketch2".into(),
            axis: Some(EntityRef::SketchCurve {
                sketch: "sketch2".into(),
                curve: axis,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::Union {
                target: "extrude(extrude1)".into(),
            },
        },
    );
    program
}

/// A pin: a circle of radius 0.4 around the origin, extruded 2 up — made to
/// fit the hole of [`box_with_drill_hole`].
pub fn pin() -> Program {
    let mut program = Program::new();
    let mut sketch = Sketch::new();
    circle(&mut sketch, [0.0, 0.0], 0.4);
    program.push(
        "pin_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(sketch),
            ..Default::default()
        },
    );
    program.push(
        "pin",
        ExtrudeArgs {
            sketch: "pin_sketch".into(),
            extent: Extents::blind(2.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

/// The assembly of `pin_in_plate`: the plate (`plate.geop`, the box of
/// [`box_with_drill_hole`]) placed fixed, and the pin (`pin.geop`, [`pin`])
/// mated into its hole — its side on the hole's axis, its bottom on the
/// hole's. Its state puts both where those mates hold them.
pub fn pin_in_plate_assembly() -> Program {
    let mut program = Program::new();
    program.push(
        "plate",
        AddPartArgs {
            file: "plate.geop".into(),
            fixed: true,
            flexible: false,
            mates: BTreeMap::new(),
            ..Default::default()
        },
    );
    let face = |name: &str| EntityRef::Face { name: name.into() };
    let mate = |kind, a: &str, b: &str| Mate::constraint(kind, vec![face(a), face(b)]);
    program.push(
        "pin",
        AddPartArgs {
            file: "pin.geop".into(),
            fixed: false,
            flexible: false,
            mates: BTreeMap::from([
                (
                    "m1".into(),
                    mate(
                        Kind::Concentric,
                        "pin/extrude(pin,pin_sketch,c1)",
                        "plate/extrude(hole,hole_sketch,c1)",
                    ),
                ),
                (
                    "m2".into(),
                    mate(
                        Kind::Coincident,
                        "pin/extrude(pin,start)",
                        "plate/extrude(hole,end)",
                    ),
                ),
            ]),
            ..Default::default()
        },
    );
    let pose = |position| ParamValue::Pose(pose(position, [0.0; 3]));
    program.state = State::from([
        (pose_parameter("plate"), pose([0.0; 3])),
        (pose_parameter("pin"), pose([1.0, 1.0, 0.5])),
    ]);
    program
}

/// A link's sketch: a 4 x 1 bar around the origin's `x` axis, from -0.5
/// to 3.5, with a hole of radius 0.3 at each end, 3 apart — and the holes.
/// A slotted bar `length` long between the centers of the holes at its
/// ends — the near one at the origin, the far one along `x` — and its
/// holes, near one first. Built alike for every length, so its entities are
/// named alike.
fn link_sketch(length: f64) -> (Sketch, [CurveId; 2]) {
    let mut sketch = Sketch::new();
    rectangle(&mut sketch, [-0.5, -0.5], length + 1.0, 1.0);
    let near = circle(&mut sketch, [0.0, 0.0], 0.3);
    let far = circle(&mut sketch, [length, 0.0], 0.3);
    (solved(sketch), [near, far])
}

/// A bar: [`link_sketch`] `length` long, extruded 0.2 up, to be linked by
/// its holes.
pub fn bar(length: f64) -> Program {
    let mut program = Program::new();
    program.push(
        "link_sketch",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: link_sketch(length).0,
            ..Default::default()
        },
    );
    program.push(
        "link",
        ExtrudeArgs {
            sketch: "link_sketch".into(),
            extent: Extents::blind(0.2),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

/// The link of the chain: a [`bar`] 3 long.
pub fn link() -> Program {
    bar(3.0)
}

/// The faces of the holes of a [`bar`] placed as `instance`, near one first.
pub fn link_holes(instance: &str) -> [EntityRef; 2] {
    link_sketch(3.0).1.map(|hole| EntityRef::Face {
        name: format!("{instance}/extrude(link,link_sketch,{hole})"),
    })
}

/// The [`bar`] placed as `above` lies on the one placed as `below`: its
/// bottom face on the other's top — so bars linked in a stack never cut
/// into each other.
fn on_top(above: &str, below: &str) -> Mate {
    Mate::constraint(
        Kind::Coincident,
        vec![
            EntityRef::Face {
                name: format!("{above}/extrude(link,start)"),
            },
            EntityRef::Face {
                name: format!("{below}/extrude(link,end)"),
            },
        ],
    )
}

/// The hole `hole` of the bar placed as `link` on the axis of the hole `pin`
/// of the one placed as `on`.
fn pinned(link: &str, hole: usize, on: &str, pin: usize) -> Mate {
    Mate::constraint(
        Kind::Concentric,
        vec![link_holes(on)[pin].clone(), link_holes(link)[hole].clone()],
    )
}

/// A part placed from `file`, held by `mates`.
fn placed(file: &str, fixed: bool, mates: Vec<Mate>) -> AddPartArgs {
    AddPartArgs {
        file: file.into(),
        fixed,
        flexible: false,
        mates: mates
            .into_iter()
            .enumerate()
            .map(|(i, mate)| (format!("m{}", i + 1), mate))
            .collect(),
        ..Default::default()
    }
}

/// The assembly of `chain`: three links (`link.geop`, [`link`]), the first
/// fixed, each next one's near hole on the axis of the one before's far
/// hole, lying on it — and nothing else: each turns about its pin.
pub fn chain_assembly() -> Program {
    let mut program = Program::new();
    program.push("link1", placed("link.geop", true, Vec::new()));
    for (before, link) in [("link1", "link2"), ("link2", "link3")] {
        program.push(
            link,
            placed(
                "link.geop",
                false,
                vec![pinned(link, 0, before, 1), on_top(link, before)],
            ),
        );
    }
    let pose = |x, z| ParamValue::Pose(pose([x, 0.0, z], [0.0; 3]));
    program.state = State::from([
        (pose_parameter("link1"), pose(0.0, 0.0)),
        (pose_parameter("link2"), pose(3.0, 0.2)),
        (pose_parameter("link3"), pose(6.0, 0.4)),
    ]);
    program
}

/// The circular edge of the hole `hole` (0 near, 1 far) of the [`link`]
/// placed as `instance`, on its top (`end`) or bottom (`start`) face.
pub fn link_rim(instance: &str, hole: usize, cap: &str) -> EntityRef {
    EntityRef::Edge {
        name: format!(
            "{instance}/extrude(link,link_sketch,{},{cap})",
            link_sketch(3.0).1[hole]
        ),
    }
}

/// The [`link`] placed as `link` turns on the one placed as `on`: a
/// revolute joint from the rim of `on`'s far hole on its top to the rim of
/// `link`'s near hole on its bottom — so it lies on it, pinned there —
/// turning from `min` to `max` degrees.
fn hinged(link: &str, on: &str, min: f64, max: f64) -> Mate {
    Mate::joint(
        JointKind::Revolute {
            min: Some(n(min)),
            max: Some(n(max)),
        },
        vec![link_rim(on, 1, "end"), link_rim(link, 0, "start")],
    )
}

/// The assembly of `arm`: three links (`link.geop`, [`link`]) — the first
/// fixed, each next one hinged on the far end of the one before by a
/// revolute joint (see [`hinged`]), the elbow turning at most 150° either
/// way, the wrist 120°. Its state sets the joints — 30° and -45° — and puts
/// the links where those joints have them.
pub fn arm_assembly() -> Program {
    let mut program = Program::new();
    program.push("upper", placed("link.geop", true, Vec::new()));
    program.push(
        "fore",
        placed(
            "link.geop",
            false,
            vec![hinged("fore", "upper", -150.0, 150.0)],
        ),
    );
    program.push(
        "hand",
        placed(
            "link.geop",
            false,
            vec![hinged("hand", "fore", -120.0, 120.0)],
        ),
    );
    let (elbow, wrist) = (30.0_f64, -45.0_f64);
    let turned = |angle: f64| {
        [
            3.0 * angle.to_radians().cos(),
            3.0 * angle.to_radians().sin(),
        ]
    };
    let fore = [3.0, 0.0];
    let [dx, dy] = turned(elbow);
    let hand = [fore[0] + dx, fore[1] + dy];
    let at = |[x, y]: [f64; 2], z, turn| ParamValue::Pose(pose([x, y, z], [0.0, 0.0, turn]));
    program.state = State::from([
        (pose_parameter("upper"), at([0.0, 0.0], 0.0, 0.0)),
        (pose_parameter("fore"), at(fore, 0.2, elbow)),
        (pose_parameter("hand"), at(hand, 0.4, elbow + wrist)),
        (
            joint_parameter("add_part(fore,m1)", Motion::Turn),
            ParamValue::Number(n(elbow)),
        ),
        (
            joint_parameter("add_part(hand,m1)", Motion::Turn),
            ParamValue::Number(n(wrist)),
        ),
    ]);
    program
}

/// The assembly of `four_bar`: a four-bar linkage of [`bar`]s — the ground
/// (4 long, fixed), the crank (1.5) and the rocker (3) each pinned to one end
/// of it and lying on it, and the coupler (4) pinned to their far ends,
/// lying on the crank. The crank is the shortest bar and
/// `1.5 + 4 ≤ 4 + 3`, so it turns all the way round while the rocker rocks.
pub fn four_bar_assembly() -> Program {
    let mut program = Program::new();
    program.push("ground", placed("ground.geop", true, Vec::new()));
    program.push(
        "crank",
        placed(
            "crank.geop",
            false,
            vec![pinned("crank", 0, "ground", 0), on_top("crank", "ground")],
        ),
    );
    program.push(
        "rocker",
        placed(
            "rocker.geop",
            false,
            vec![pinned("rocker", 0, "ground", 1), on_top("rocker", "ground")],
        ),
    );
    program.push(
        "coupler",
        placed(
            "coupler.geop",
            false,
            vec![
                pinned("coupler", 0, "crank", 1),
                pinned("coupler", 1, "rocker", 1),
                on_top("coupler", "crank"),
            ],
        ),
    );
    // The crank at 60°, and where that puts the others (worked out from the
    // bars' lengths; the mates hold them there exactly).
    let pose = |position, turn| ParamValue::Pose(pose(position, [0.0, 0.0, turn]));
    program.state = State::from([
        (pose_parameter("ground"), pose([0.0, 0.0, 0.0], 0.0)),
        (pose_parameter("crank"), pose([0.0, 0.0, 0.2], 60.0)),
        (
            pose_parameter("rocker"),
            pose([4.0, 0.0, 0.2], 82.690_722_887_668_12),
        ),
        (
            pose_parameter("coupler"),
            pose([0.75, 1.299_038_105_676_658, 0.4], 24.780_674_143_948_414),
        ),
    ]);
    program
}

/// A number parameter of an example: `expression`, offered on a slider
/// from `min` to `max`.
fn number_parameter(name: &str, expression: &str, min: f64, max: f64) -> Parameter {
    Parameter {
        name: name.into(),
        kind: ParameterKind::Number {
            expression: expression.into(),
            min: Some(min),
            max: Some(max),
        },
    }
}

/// A plate designed by its parameters: `width` and `depth` — half the
/// width, unless placed otherwise — and a blind hole in its middle sized for the
/// screw chosen from a table of sizes, and its colour. The hole's sketch,
/// on the plate's top, projects the top, and puts the hole in the middle
/// of it. The plate is meant to be placed with other values, and
/// everything follows.
pub fn parametric_plate() -> Program {
    let mut program = Program::new();
    program.parameters = Parameters {
        color: Some("#d0893e".into()),
        material: Some(Material {
            name: "Aluminium 6061".into(),
            density: 2700.0,
        }),
        values: vec![
            number_parameter("width", "4", 1.0, 10.0),
            number_parameter("depth", "width / 2", 0.5, 10.0),
            Parameter {
                name: "screw".into(),
                kind: ParameterKind::Table {
                    columns: vec!["clearance".into(), "head".into()],
                    rows: [
                        ("M3", 0.34, 0.6),
                        ("M4", 0.45, 0.8),
                        ("M5", 0.55, 1.0),
                        ("M6", 0.66, 1.2),
                    ]
                    .map(|(name, clearance, head)| Row {
                        name: name.into(),
                        values: vec![clearance, head],
                    })
                    .to_vec(),
                    selected: "M4".into(),
                },
            },
        ],
    };

    let mut outline = Sketch::new();
    let lines = rectangle(&mut outline, [0.0, 0.0], 4.0, 2.0);
    let length = |line: CurveId| {
        outline
            .constraints
            .iter()
            .find(|(_, c)| matches!(c, Constraint::Length { curve, .. } if *curve == line))
            .map(|(&id, _)| id)
            .expect("the rectangle's lengths")
    };
    let formulas = BTreeMap::from([
        (length(lines[0]), "width".to_string()),
        (length(lines[1]), "depth".to_string()),
    ]);
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            formulas,
            ..Default::default()
        },
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(0.5),
            face: false,
            combine: Combine::NewBody,
        },
    );

    // The hole's center is the middle of the top's diagonal, between two
    // of its corners as projected: wherever the parameters put them.
    let mut hole = Sketch::new();
    let top = EntityRef::Face {
        name: "extrude(plate,end)".into(),
    };
    let mut projection = Reference::new(Source::Projection {
        entity: top.clone(),
    });
    let built = program
        .build::<Design>(&geop_ops::NoFiles)
        .expect("the plate builds");
    let plane = top.resolve_plane(&built).expect("the top is planar");
    projection
        .update(&mut hole, &built, &plane)
        .expect("the top projects");
    let corner = |p: &str| projection.points[&format!("extrude(plate,outline,{p},end)")];
    let diagonal = hole.add_line(corner("p0"), corner("p2"));
    hole.set_construction(diagonal, true);
    let center = hole.add_point(n(2.0), n(1.0));
    let circle = hole.add_circle(center, n(0.2));
    hole.constrain(Constraint::Midpoint {
        point: center,
        curve: diagonal,
    });
    let diameter = hole.constrain(Constraint::Diameter {
        curve: circle,
        value: n(0.45),
    });
    program.push(
        "hole_sketch",
        AddSketchArgs {
            plane: Some(top),
            sketch: solved(hole),
            references: vec![projection],
            formulas: BTreeMap::from([(diameter, "screw.clearance".to_string())]),
            ..Default::default()
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            extent: Extents::blind(-0.3),
            face: false,
            combine: Combine::Difference {
                target: "extrude(plate)".into(),
            },
        },
    );
    program
}

/// Two of [`parametric_plate`], placed with other values: one as it is,
/// one 5 wide with an M6 hole, in blue.
pub fn plates_assembly() -> Program {
    let mut program = Program::new();
    program.push(
        "small",
        AddPartArgs {
            file: "plate.geop".into(),
            fixed: true,
            ..Default::default()
        },
    );
    program.push(
        "large",
        AddPartArgs {
            file: "plate.geop".into(),
            fixed: true,
            parameters: State::from([
                ("width".to_string(), ParamValue::Number(n(5.0))),
                ("screw".to_string(), ParamValue::Text("M6".into())),
                ("color".to_string(), ParamValue::Text("#3e7bd0".into())),
            ]),
            ..Default::default()
        },
    );
    program.state = State::from([
        (
            pose_parameter("small"),
            ParamValue::Pose(pose([0.0; 3], [0.0; 3])),
        ),
        (
            pose_parameter("large"),
            ParamValue::Pose(pose([0.0, 3.0, 0.0], [0.0; 3])),
        ),
    ]);
    program
}

/// A plate 40 x 40 x 5 with a 4.5 hole through it at `(20, 20)` — in
/// millimetres, as the standard parts are: clearance for an M4 screw.
pub fn metric_plate() -> Program {
    use crate::stdlib::{drawing::Drawing, steps::outline_plane};

    let none = Parameters::default();
    let corner = |x: &str, y: &str| [x.to_string(), y.to_string()];
    let mut outline = Drawing::new(&none).expect("no parameters to resolve");
    let mut hole = Drawing::new(&none).expect("no parameters to resolve");
    outline
        .polygon(&[
            corner("0", "0"),
            corner("40", "0"),
            corner("40", "40"),
            corner("0", "40"),
        ])
        .expect("the corners are numbers");
    let centre = hole.point("20", "20").expect("the centre is numbers");
    hole.circle(centre, "4.5")
        .expect("the diameter is a number");
    let mut program = Program::new();
    program.push("outline", outline.on(outline_plane()).expect("it solves"));
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(5.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push("hole_sketch", hole.on(outline_plane()).expect("it solves"));
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: "hole_sketch".into(),
            extent: Extents {
                side1: Extent::ThroughAll,
                ..Extents::blind(0.0)
            },
            face: false,
            combine: Combine::Difference {
                target: "extrude(plate)".into(),
            },
        },
    );
    program
}

/// The name of the face of [`metric_plate`]'s hole.
pub fn metric_plate_hole() -> String {
    let part = metric_plate()
        .build::<Design>(&geop_ops::NoFiles)
        .expect("the plate builds");
    geop_ops::PartDescription::of(&part)
        .expect("the plate describes itself")
        .faces
        .into_keys()
        .find(|name| name.starts_with("extrude(hole,hole_sketch,"))
        .expect("the hole has a face")
}

/// [`metric_plate`] (`plate.geop`) bolted together from standard parts: an
/// ISO 4762 M4 x 12 cap screw down its hole, and an ISO 4032 M4 nut under
/// it — each mated by its datums `axis` and `seat` or `base`.
pub fn bolted_plate() -> Program {
    let mut program = Program::new();
    program.push(
        "plate",
        AddPartArgs {
            file: "plate.geop".into(),
            fixed: true,
            ..Default::default()
        },
    );
    let mate = |kind, a: EntityRef, b: EntityRef| Mate::constraint(kind, vec![a, b]);
    let face = |name: String| EntityRef::Face { name };
    let size = |row: &str| State::from([("size".to_string(), ParamValue::Text(row.into()))]);
    program.push(
        "screw",
        AddPartArgs {
            file: "std:iso4762_socket_head_cap_screw.geop".into(),
            parameters: size("M4x12"),
            mates: BTreeMap::from([
                (
                    "m1".into(),
                    mate(
                        Kind::Concentric,
                        EntityRef::datum("screw/axis"),
                        face(format!("plate/{}", metric_plate_hole())),
                    ),
                ),
                (
                    "m2".into(),
                    mate(
                        Kind::Coincident,
                        EntityRef::datum("screw/seat"),
                        face("plate/extrude(plate,end)".into()),
                    ),
                ),
            ]),
            ..Default::default()
        },
    );
    program.push(
        "nut",
        AddPartArgs {
            file: "std:iso4032_hex_nut.geop".into(),
            parameters: size("M4"),
            mates: BTreeMap::from([
                (
                    "m1".into(),
                    mate(
                        Kind::Concentric,
                        EntityRef::datum("nut/axis"),
                        EntityRef::datum("screw/axis"),
                    ),
                ),
                (
                    "m2".into(),
                    mate(
                        Kind::Coincident,
                        EntityRef::datum("nut/base"),
                        face("plate/extrude(plate,start)".into()),
                    ),
                ),
            ]),
            ..Default::default()
        },
    );
    program.state = State::from([
        (
            pose_parameter("plate"),
            ParamValue::Pose(pose([0.0; 3], [0.0; 3])),
        ),
        (
            pose_parameter("screw"),
            ParamValue::Pose(pose([20.0, 20.0, 5.0], [0.0; 3])),
        ),
        (
            pose_parameter("nut"),
            ParamValue::Pose(pose([20.0, 20.0, 0.0], [180.0, 0.0, 0.0])),
        ),
    ]);
    program
}

/// The files of an example made of several: each file's path and program,
/// the one to open first first.
pub type ExampleFiles = Vec<(&'static str, Program)>;

/// An example by name, with what makes it: a program, or the files of one
/// made of several.
pub type Example<T> = (&'static str, fn() -> T);

/// Every example made of several files, by name, each with what makes its
/// files: listing them builds nothing (see [`all`]).
pub fn workspaces() -> Vec<Example<ExampleFiles>> {
    vec![
        ("pin_in_plate", || {
            vec![
                ("assembly.geop", pin_in_plate_assembly()),
                ("plate.geop", box_with_drill_hole()),
                ("pin.geop", pin()),
            ]
        }),
        ("chain", || {
            vec![("chain.geop", chain_assembly()), ("link.geop", link())]
        }),
        ("parametric_plates", || {
            vec![
                ("plates.geop", plates_assembly()),
                ("plate.geop", parametric_plate()),
            ]
        }),
        ("four_bar", || {
            vec![
                ("four_bar.geop", four_bar_assembly()),
                ("ground.geop", bar(4.0)),
                ("crank.geop", bar(1.5)),
                ("rocker.geop", bar(3.0)),
                ("coupler.geop", bar(4.0)),
            ]
        }),
        ("arm", || {
            vec![("arm.geop", arm_assembly()), ("link.geop", link())]
        }),
        ("bolted_plate", || {
            vec![
                ("bolted_plate.geop", bolted_plate()),
                ("plate.geop", metric_plate()),
            ]
        }),
    ]
}

/// The NACA 2412 section, chord 1 from its leading edge at the origin to
/// its trailing edge at `(1, 0)`: the upper side from the trailing edge to
/// the leading edge, then the lower side back — points of the four-digit
/// formula, cosine spaced, used as the control points of a spline each.
const NACA_2412_UPPER: [P2; 15] = [
    [1.0, 0.0],
    [0.9876, 0.0026],
    [0.9509, 0.0101],
    [0.8917, 0.0214],
    [0.8129, 0.0349],
    [0.7182, 0.049],
    [0.6123, 0.062],
    [0.5006, 0.0723],
    [0.3886, 0.0784],
    [0.2813, 0.0782],
    [0.1853, 0.071],
    [0.1056, 0.0576],
    [0.0464, 0.0399],
    [0.0107, 0.0201],
    [0.0, 0.0],
];
const NACA_2412_LOWER: [P2; 15] = [
    [0.0, 0.0],
    [0.0144, -0.0176],
    [0.0526, -0.0306],
    [0.1126, -0.0387],
    [0.1912, -0.0422],
    [0.2848, -0.0416],
    [0.3889, -0.0384],
    [0.4994, -0.0334],
    [0.6102, -0.027],
    [0.7157, -0.0202],
    [0.8106, -0.0138],
    [0.8901, -0.0082],
    [0.9501, -0.0038],
    [0.9873, -0.001],
    [1.0, 0.0],
];

/// The NACA 2412 section of chord `chord`, its leading edge at `(x, 0)`,
/// drawn in a plane of the origin's `y` axis — which draws in `(x, -z)`,
/// so the section is drawn upside down to stand the right way up.
fn naca_2412(chord: f64, x: f64) -> Sketch {
    let mut s = Sketch::new();
    let point = |s: &mut Sketch, p: P2| s.add_point(n(x + chord * p[0]), n(-chord * p[1]));
    let trailing = point(&mut s, NACA_2412_UPPER[0]);
    let leading = point(&mut s, NACA_2412_LOWER[0]);
    let side = |s: &mut Sketch, points: &[P2], first: PointId, last: PointId| {
        let mut control = vec![first];
        for &p in &points[1..points.len() - 1] {
            control.push(point(s, p));
        }
        control.push(last);
        s.add_spline(control);
    };
    side(&mut s, &NACA_2412_UPPER, trailing, leading);
    side(&mut s, &NACA_2412_LOWER, leading, trailing);
    s
}

/// A wing: the NACA 2412 section, chord 1 at the root, lofted into the
/// same section of chord 0.6 at the tip, 3 out along `y` and swept back
/// 0.3.
pub fn airfoil_wing() -> Program {
    let mut program = Program::new();
    program.push(
        "root",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )),
            sketch: naca_2412(1.0, 0.0),
            ..Default::default()
        },
    );
    program.push(
        "span",
        AddDatumArgs {
            selection: vec![EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )],
            construction: Construction::Offset { distance: 3.0 },
        },
    );
    program.push(
        "tip",
        AddSketchArgs {
            plane: Some(EntityRef::Datum {
                name: "span".into(),
                component: None,
            }),
            sketch: naca_2412(0.6, 0.3),
            ..Default::default()
        },
    );
    program.push(
        "wing",
        LoftArgs {
            profiles: vec!["root".into(), "tip".into()],
            matches: Vec::new(),
            guides: Vec::new(),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

/// A horn: a circle swept along a straight path, its size set by a rail —
/// a spline flaring out beside the path, which the circle's point on it
/// follows.
pub fn horn() -> Program {
    let base = |axis| {
        Some(EntityRef::datum_component(
            ORIGIN,
            DatumComponent::Plane(axis),
        ))
    };
    let mut program = Program::new();
    let mut mouth = Sketch::new();
    let c = mouth.add_point(n(0.0), n(0.0));
    mouth.add_circle(c, n(0.5));
    program.push(
        "mouth",
        AddSketchArgs {
            plane: base(FrameAxis::X),
            sketch: mouth,
            ..Default::default()
        },
    );
    let mut axis = Sketch::new();
    let (a, b) = (
        axis.add_point(n(0.0), n(0.0)),
        axis.add_point(n(4.0), n(0.0)),
    );
    axis.add_line(a, b);
    program.push(
        "axis",
        AddSketchArgs {
            plane: base(FrameAxis::Z),
            sketch: axis,
            ..Default::default()
        },
    );
    let mut flare = Sketch::new();
    let points =
        [[0.0, 0.5], [1.5, 0.4], [3.0, 0.9], [4.0, 1.6]].map(|[x, y]| flare.add_point(n(x), n(y)));
    flare.add_spline(points.to_vec());
    program.push(
        "flare",
        AddSketchArgs {
            plane: base(FrameAxis::Z),
            sketch: flare,
            ..Default::default()
        },
    );
    program.push(
        "horn",
        SweepArgs {
            profile: "mouth".into(),
            path: "axis".into(),
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: vec!["flare".into()],
            face: false,
            combine: Combine::NewBody,
        },
    );
    program
}

/// An 8 x 3 x 0.5 plate with a row of four holes: a pin sketched on the Z
/// plane and extruded through the plate as a body of its own (`pin`), then
/// patterned four times 2 apart along `x` and cut from the plate, itself
/// and every copy (`holes`) — the drilled plate is `linear_pattern(holes)`,
/// and the holes follow the pin when it changes.
pub fn patterned_plate() -> Program {
    let mut program = Program::new();
    let z_plane = || EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z));
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 8.0, 3.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(0.5),
            face: false,
            combine: Combine::NewBody,
        },
    );
    let mut pin = Sketch::new();
    circle(&mut pin, [1.0, 1.5], 0.3);
    program.push(
        "pin_sketch",
        AddSketchArgs {
            plane: Some(z_plane()),
            sketch: solved(pin),
            ..Default::default()
        },
    );
    let mut through = Extents::blind(2.0);
    through.symmetric = true;
    program.push(
        "pin",
        ExtrudeArgs {
            sketch: "pin_sketch".into(),
            extent: through,
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "holes",
        LinearPatternArgs {
            bodies: vec![EntityRef::Solid {
                name: "extrude(pin)".into(),
            }],
            first: Direction {
                along: Some(EntityRef::datum_component(
                    ORIGIN,
                    DatumComponent::Axis(FrameAxis::X),
                )),
                reversed: false,
                count: 4,
                spacing: Spacing::Step(2.0),
            },
            second: None,
            combine: Combine::Difference {
                target: "extrude(plate)".into(),
            },
        },
    );
    program
}

/// A 60 x 40 x 10 mm mounting plate: four counterbored M5 holes through
/// its corners, at the points of one sketch on its top (`bolts`), and a
/// tapped M6 hole 8 deep in its middle (`tapped`), which carries the
/// cosmetic thread M6x1 — the result is `hole(tapped)`.
pub fn hole_plate() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 60.0, 40.0);
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: "outline".into(),
            extent: Extents::blind(10.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    let points = |at: &[P2]| {
        let mut sketch = Sketch::new();
        let ids: Vec<PointId> = at
            .iter()
            .map(|p| sketch.add_point(n(p[0]), n(p[1])))
            .collect();
        for (id, p) in ids.into_iter().zip(at) {
            sketch.constrain(Constraint::Fix {
                point: id,
                x: n(p[0]),
                y: n(p[1]),
            });
        }
        solved(sketch)
    };
    let top = || EntityRef::Face {
        name: "extrude(plate,end)".into(),
    };
    program.push(
        "corners",
        AddSketchArgs {
            plane: Some(top()),
            sketch: points(&[[8.0, 8.0], [52.0, 8.0], [52.0, 32.0], [8.0, 32.0]]),
            ..Default::default()
        },
    );
    program.push(
        "bolts",
        HoleArgs {
            face: "extrude(plate,end)".into(),
            points: vec![EntityRef::Sketch {
                name: "corners".into(),
            }],
            kind: HoleKind::Counterbore,
            standard: Standard::Iso {
                size: "M5".into(),
                fit: Fit::Normal,
            },
            end: Extent::ThroughAll,
            drill_point: false,
            thread_length: None,
        },
    );
    program.push(
        "middle",
        AddSketchArgs {
            plane: Some(top()),
            sketch: points(&[[30.0, 20.0]]),
            ..Default::default()
        },
    );
    program.push(
        "tapped",
        HoleArgs {
            face: "extrude(plate,end)".into(),
            points: vec![EntityRef::Sketch {
                name: "middle".into(),
            }],
            kind: HoleKind::Tapped,
            standard: Standard::Iso {
                size: "M6".into(),
                fit: Fit::Normal,
            },
            end: Extent::Blind(8.0),
            drill_point: true,
            thread_length: None,
        },
    );
    program
}

/// The route of [`pipe`], its bends of radius `bend`: from the origin
/// along `x`, a bend turning to `y` at `(4, 0, 0)`, a bend turning up to
/// `z` at `(4, 4, 0)`, and up to `(4, 4, 4)` — every bend tangent to the
/// lines it joins.
pub fn pipe_route(bend: f64) -> Sketch3d {
    let mut s = Sketch3d::new();
    let (r, h) = (bend, std::f64::consts::FRAC_1_SQRT_2);
    let v = |x: f64, y: f64, z: f64| Vector3::from_array([x, y, z].map(n));
    let p = [
        s.add_point(v(0.0, 0.0, 0.0)),
        s.add_point(v(4.0 - r, 0.0, 0.0)),
        s.add_point(v(4.0 - r + r * h, r - r * h, 0.0)),
        s.add_point(v(4.0, r, 0.0)),
        s.add_point(v(4.0, 4.0 - r, 0.0)),
        s.add_point(v(4.0, 4.0 - r + r * h, r - r * h)),
        s.add_point(v(4.0, 4.0, r)),
        s.add_point(v(4.0, 4.0, 4.0)),
    ];
    let curves = [
        s.add_line(p[0], p[1]),
        s.add_arc(p[1], p[2], p[3]),
        s.add_line(p[3], p[4]),
        s.add_arc(p[4], p[5], p[6]),
        s.add_line(p[6], p[7]),
    ];
    for axis in geop_core_sketch::space::Coordinate::ALL {
        s.constrain(Constraint3d::Coordinate {
            point: p[0],
            axis,
            value: n(0.0),
        });
    }
    s.constrain(Constraint3d::ParallelTo {
        line: curves[0],
        direction: v(1.0, 0.0, 0.0),
    });
    s.constrain(Constraint3d::Length {
        line: curves[0],
        value: n(4.0 - r),
    });
    for w in curves.windows(2) {
        s.constrain(Constraint3d::Tangent { a: w[0], b: w[1] });
    }
    for arc in [curves[1], curves[3]] {
        s.constrain(Constraint3d::Radius { arc, value: n(r) });
    }
    let report = s.solve().expect("the route is valid");
    assert!(report.converged, "the route does not solve: {report:?}");
    s
}

/// A pipe bent through space — a cable conduit, say: a circle of radius
/// 0.3 on the `x` plane (`section`), swept along a 3-D sketch of lines and
/// bends (`route`, see [`pipe_route`]) into the solid `sweep(pipe)`.
pub fn pipe() -> Program {
    let mut program = Program::new();
    let mut section = Sketch::new();
    circle(&mut section, [0.0, 0.0], 0.3);
    program.push(
        "section",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            sketch: solved(section),
            ..Default::default()
        },
    );
    program.push(
        "route",
        AddSketch3dArgs {
            sketch: pipe_route(1.0),
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

/// A sheet-metal mounting bracket: a 2 x 1.2 plate 0.08 thick with two
/// holes (`plate`), a flange bent up along part of its front edge with
/// reliefs beside it (`front`), and one along all of its back edge
/// (`back`) — the bent body is `edge_flange(back)`.
pub fn sheet_metal_bracket() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    let lines = rectangle(&mut outline, [0.0, 0.0], 2.0, 1.2);
    circle(&mut outline, [0.5, 0.6], 0.15);
    circle(&mut outline, [1.5, 0.6], 0.15);
    program.push(
        "outline",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "plate",
        BaseFlangeArgs {
            sketch: "outline".into(),
            rules: SheetMetalRules {
                thickness: 0.08,
                bend_radius: 0.08,
                ..SheetMetalRules::default()
            },
            depth: 1.0,
            flip: false,
        },
    );
    let edge = |line: CurveId| format!("base_flange(plate,outline,{line},b)");
    let flange = |line: CurveId, length: f64, offset: f64| EdgeFlangeArgs {
        edge: edge(line),
        angle: 90.0,
        length,
        reference: LengthReference::OuterSharp,
        position: FlangePosition::MaterialInside,
        radius: None,
        offset_start: offset,
        offset_end: offset,
    };
    program.push("front", flange(lines[0], 0.6, 0.3));
    program.push("back", flange(lines[2], 0.4, 0.0));
    program
}

/// A computer mouse, shaped as a subdivision surface: half a 1.2 x 2 x 0.8
/// box cage, mirrored in `x = 0`, its back raised into a hump and the rim
/// of its bottom creased, so that it stands on a flat sole with a sharp
/// edge while everything above rounds off. The body is `subd(mouse)`.
pub fn subd_mouse() -> Program {
    let mut cage = Cage::cuboid([1.2, 2.0, 0.8]);
    cage.halve(0).expect("the box reaches across x = 0");
    let bottom = cage.face(8).expect("the box's bottom").vertices.clone();
    let on_plane = |v: u32| cage.vertex(v).is_ok_and(|x| x.at[0] == 0.0);
    let rim: Vec<[u32; 2]> = (0..bottom.len())
        .map(|k| [bottom[k], bottom[(k + 1) % bottom.len()]])
        .filter(|&[a, b]| !(on_plane(a) && on_plane(b)))
        .collect();
    cage.set_crease(&rim, true);
    let back = cage
        .vertices
        .iter()
        .filter(|v| v.at[1] > 0.0 && v.at[2] > 0.0)
        .map(|v| v.id)
        .collect();
    cage.transform(&back, Mirror::X, |p| [p[0], p[1], p[2] + 0.3]);
    let mut program = Program::new();
    program.push(
        "mouse",
        SubdArgs {
            cage,
            mirror: Mirror::X,
        },
    );
    program
}

/// Every example, by name, each with what makes its program: listing them
/// builds nothing — an editor lists them when it starts, and making one
/// may solve sketches, or build a part to name its faces.
pub fn all() -> Vec<Example<Program>> {
    vec![
        ("box_with_drill_hole", box_with_drill_hole),
        ("bracket", bracket),
        ("cross_drilled_shaft", cross_drilled_shaft),
        ("split_plate", split_plate),
        ("boss_on_reference_plane", boss_on_reference_plane),
        ("handle_with_hole", handle_with_hole),
        ("luggage_tag", luggage_tag),
        ("revolved_cone_on_box", revolved_cone_on_box),
        ("pin", pin),
        ("link", link),
        ("parametric_plate", parametric_plate),
        ("airfoil_wing", airfoil_wing),
        ("subd_mouse", subd_mouse),
        ("sheet_metal_bracket", sheet_metal_bracket),
        ("pipe", pipe),
        ("patterned_plate", patterned_plate),
        ("horn", horn),
        ("hole_plate", hole_plate),
    ]
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;

    use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
    use geop_core_topology::{
        contains::shell::{PointClassification, shell_contains},
        validation::{ValidationParameters, validate},
    };
    use geop_ops::{NoFiles, Part, PartDescription, RefId};

    use super::*;

    fn outputs_dir() -> std::path::PathBuf {
        let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../outputs/parts");
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    /// Every named entity's exact geometry, by name: vertex points, edge
    /// curves and face surfaces as their full `Debug` enclosures — equal only
    /// if the two parts agree to the last bit of every interval.
    fn geometry_by_name(part: &Part<S>) -> BTreeMap<String, String> {
        let model = part.topology();
        part.names()
            .iter()
            .filter_map(|(id, name)| {
                let geometry = match id {
                    RefId::Vertex(v) => format!("{:?}", model.get_vertex(v).ok()?.point),
                    RefId::Edge(e) => format!("{:?}", model.get_edge(e).ok()?.curve),
                    RefId::Face(f) => format!("{:?}", model.get_face(f).ok()?.surface),
                    RefId::Solid(_)
                    | RefId::Sketch(_)
                    | RefId::Sketch3d(_)
                    | RefId::Datum(_)
                    | RefId::Instance(_) => {
                        return None;
                    }
                };
                Some((name.to_string(), geometry))
            })
            .collect()
    }

    /// Builds `program`, writes it and a description of the part it builds
    /// to `outputs/parts/`, reads the program back from its JSON, and
    /// requires the read-back program to be the same program and to build
    /// the very same part: every name, every piece of topology between
    /// names, and every bit of geometry.
    fn build_and_round_trip(name: &str, program: &Program) -> Part<S> {
        let part = program.build::<S>(&NoFiles).unwrap();
        let validation = ValidationParameters::default();
        if let Err(errors) = validate(&validation, part.topology()) {
            panic!(
                "{name}: {} validation error(s): {}",
                errors.len(),
                errors[0]
            );
        }

        let json = program.to_json().unwrap();
        let dir = outputs_dir();
        std::fs::write(dir.join(format!("{name}.geop")), &json).unwrap();
        let description = PartDescription::of(&part).unwrap();
        std::fs::write(
            dir.join(format!("{name}.part.json")),
            serde_json::to_string_pretty(&description).unwrap(),
        )
        .unwrap();
        geop_ops_rasterize::rasterize(part.topology(), 16)
            .unwrap()
            .scene(|_| geop_ops_rasterize::debug::Color10::Blue)
            .save_to_file(dir.join(format!("{name}.html")).to_str().unwrap())
            .unwrap();

        let read_back = Program::from_json(&json).unwrap();
        assert_eq!(
            &read_back, program,
            "{name}: JSON round trip changed the program"
        );
        assert_eq!(
            read_back.to_json().unwrap(),
            json,
            "{name}: JSON is not stable"
        );

        let rebuilt = read_back.build::<S>(&NoFiles).unwrap();
        assert_eq!(
            PartDescription::of(&rebuilt).unwrap(),
            description,
            "{name}: the read-back program built a different part"
        );
        assert_eq!(
            geometry_by_name(&rebuilt),
            geometry_by_name(&part),
            "{name}: the read-back program built different geometry"
        );
        part
    }

    fn inside(part: &Part<S>, solid: &str, p: [f64; 3]) -> PointClassification {
        let model = part.topology();
        let solid = part.solid_id(solid).unwrap();
        let point = Vector3::from_array(p.map(S::from_f64));
        let mut result = PointClassification::Outside;
        for &shell in &model.get_solid(solid).unwrap().shells {
            match shell_contains(model, shell, point, 2000, S::from_f64(1e-6), 7).unwrap() {
                PointClassification::Outside => {}
                other => result = other,
            }
        }
        result
    }

    /// The pipe through space is one valid closed solid: four quarters of
    /// wall along each of its five curves and two caps — the far one square
    /// to its last line, at `z = 4` — hollow nowhere and solid all along.
    #[test]
    fn pipe_round_trips() {
        let part = build_and_round_trip("pipe", &pipe());
        let validation = ValidationParameters::default();
        if let Err(errors) =
            geop_core_topology::validation::validate_manifold(&validation, part.topology())
        {
            panic!("{errors:?}");
        }
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["sweep(pipe)"]
        );
        assert_eq!(description.faces.len(), 5 * 4 + 2);
        let model = part.topology();
        let end = part.face_id("sweep(pipe,end)").unwrap();
        assert!(model.iterate_face_coedges(end).all(|c| {
            model.coedge_start_vertex(c).unwrap().point[2].could_be_equal(S::from_f64(4.0))
        }));
        let half = std::f64::consts::FRAC_1_SQRT_2;
        for (p, expected) in [
            ([1.5, 0.0, 0.0], PointClassification::Inside),
            ([1.5, 0.4, 0.0], PointClassification::Outside),
            ([3.0 + half, 1.0 - half, 0.0], PointClassification::Inside),
            ([4.0, 2.0, 0.2], PointClassification::Inside),
            ([4.0, 3.0 + half, 1.0 - half], PointClassification::Inside),
            ([4.0, 4.0, 3.0], PointClassification::Inside),
            ([4.0, 4.0, 4.2], PointClassification::Outside),
        ] {
            assert_eq!(inside(&part, "sweep(pipe)", p), expected, "at {p:?}");
        }
    }

    #[test]
    fn box_with_drill_hole_round_trips() {
        let part = build_and_round_trip("box_with_drill_hole", &box_with_drill_hole());
        let description = PartDescription::of(&part).unwrap();

        // One solid, named after the step that cut the hole.
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(hole)"]
        );
        // The box's top keeps its name, and now has the hole in it.
        let top = &description.faces["extrude(box,end)"];
        assert_eq!(top.holes.len(), 1, "{top:?}");
        // The hole's bottom is the hole tool's end cap; its wall is the
        // four quarters swept by the circle.
        assert!(description.faces.contains_key("extrude(hole,end)"));
        let circle = box_with_drill_hole().steps[2].clone();
        let crate::PartOperation::AddSketch(args) = circle.operation else {
            unreachable!()
        };
        let circle_id = *args.sketch.curves.keys().next().unwrap();
        for piece in ["", "#1", "#2", "#3"] {
            let wall = format!("extrude(hole,hole_sketch,{circle_id}{piece})");
            assert!(description.faces.contains_key(&wall), "no face {wall}");
        }

        assert_eq!(
            inside(&part, "extrude(hole)", [0.3, 0.3, 0.5]),
            PointClassification::Inside
        );
        assert_eq!(
            inside(&part, "extrude(hole)", [1.0, 1.0, 0.8]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(hole)", [1.0, 1.0, 0.3]),
            PointClassification::Inside
        );
    }

    #[test]
    fn cross_drilled_shaft_round_trips() {
        let part = build_and_round_trip("cross_drilled_shaft", &cross_drilled_shaft());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(bore)"]
        );
        assert_eq!(
            inside(&part, "extrude(bore)", [0.0, 0.8, 0.5]),
            PointClassification::Inside
        );
        assert_eq!(
            inside(&part, "extrude(bore)", [0.0, 0.0, 2.2]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(bore)", [0.0, 0.0, 2.7]),
            PointClassification::Inside
        );
    }

    #[test]
    fn split_plate_round_trips() {
        let part = build_and_round_trip("split_plate", &split_plate());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["split(halves,0)", "split(halves,1)"]
        );
        // The face it was cut with stays, standing on its own.
        assert_eq!(part.sheet_face_names(), ["extrude(cut,cut_sketch,c2)"]);
        // One half each side of the cut at x = 2, neither in the hole.
        let at = |p: [f64; 3]| {
            ["split(halves,0)", "split(halves,1)"]
                .map(|half| inside(&part, half, p) == PointClassification::Inside)
        };
        let (left, right) = (at([0.2, 0.2, 0.0]), at([2.5, 0.5, 0.1]));
        assert!(
            left[0] != left[1] && right == [left[1], left[0]],
            "{left:?} {right:?}"
        );
        assert_eq!(at([0.75, 0.5, 0.0]), [false, false]);
    }

    #[test]
    fn boss_on_reference_plane_round_trips() {
        let part = build_and_round_trip("boss_on_reference_plane", &boss_on_reference_plane());
        let description = PartDescription::of(&part).unwrap();
        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(boss)"]
        );
        assert_eq!(description.datums, ["lifted", "origin"]);
        // The boss stands on the box: from the box's top up to the plane.
        assert_eq!(
            inside(&part, "extrude(boss)", [1.0, 1.0, 1.3]),
            PointClassification::Inside
        );
        assert_eq!(
            inside(&part, "extrude(boss)", [1.0, 1.0, 1.6]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(boss)", [0.2, 0.2, 1.3]),
            PointClassification::Outside
        );
        assert_eq!(
            inside(&part, "extrude(boss)", [0.2, 0.2, 0.5]),
            PointClassification::Inside
        );
    }

    /// The names of a program's entities don't depend on its numbers: a
    /// taller box with a wider hole has the very same names.
    /// The hole goes all the way through: both the top and the bottom face
    /// carry it as a hole, it has no bottom of its own, and its axis is
    /// outside the solid at every height.
    #[test]
    fn bracket_round_trips() {
        let part = build_and_round_trip("bracket", &bracket());
        let description = PartDescription::of(&part).unwrap();

        assert_eq!(
            description.solids.keys().collect::<Vec<_>>(),
            ["extrude(hole)"]
        );
        for cap in ["extrude(block,start)", "extrude(block,end)"] {
            let face = &description.faces[cap];
            assert_eq!(face.holes.len(), 1, "{cap}: {face:?}");
        }
        assert!(!description.faces.contains_key("extrude(hole,end)"));
        for z in [1.0, 5.0, 9.0] {
            assert_eq!(
                inside(&part, "extrude(hole)", [20.0, 20.0, z]),
                PointClassification::Outside,
                "z = {z}"
            );
        }
        assert_eq!(
            inside(&part, "extrude(hole)", [5.0, 5.0, 5.0]),
            PointClassification::Inside
        );
    }

    #[test]
    fn names_survive_a_change_of_dimensions() {
        let names = |program: &Program| {
            let part = program.build::<S>(&NoFiles).unwrap();
            let description = PartDescription::of(&part).unwrap();
            (
                description.faces.keys().cloned().collect::<Vec<_>>(),
                description.edges.keys().cloned().collect::<Vec<_>>(),
                description.vertices.keys().cloned().collect::<Vec<_>>(),
            )
        };
        let original = box_with_drill_hole();
        let mut edited = original.clone();
        for step in &mut edited.steps {
            match &mut step.operation {
                crate::PartOperation::Extrude(args) if step.id == "box" => {
                    args.extent = Extents::blind(1.5)
                }
                crate::PartOperation::AddSketch(args) if step.id == "hole_sketch" => {
                    for c in args.sketch.constraints.values_mut() {
                        if let Constraint::Radius { value, .. } = c {
                            *value = n(0.6);
                        }
                    }
                    args.sketch.solve().unwrap();
                }
                _ => {}
            }
        }
        assert_ne!(original, edited);
        assert_eq!(names(&edited), names(&original));
    }

    /// An unknown name is reported, not guessed at.
    #[test]
    fn referring_to_a_missing_entity_fails() {
        let mut program = box_with_drill_hole();
        let crate::PartOperation::AddSketch(args) = &mut program.steps[2].operation else {
            unreachable!()
        };
        args.plane = Some(EntityRef::Face {
            name: "extrude(box,side)".into(),
        });
        let Err(err) = program.build::<S>(&NoFiles) else {
            panic!("a sketch on a face that doesn't exist was placed somewhere");
        };
        assert!(format!("{err:?}").contains("extrude(box,side)"), "{err:?}");
    }

    /// Step ids have to be unique: every name a step creates is built from
    /// its id.
    #[test]
    fn duplicate_step_ids_are_rejected() {
        let mut program = box_with_drill_hole();
        program.steps[3].id = "box".into();
        assert!(program.build::<S>(&NoFiles).is_err());
    }

    /// Reproduces the real bug report described on `handle_with_hole`.
    ///
    /// Used to fail in the "hole" step's `Difference`, with
    /// `face_interior_point` finding no interior point on a side wall of the
    /// inner spline. The wall's loop carried a spur whose pcurve ran across
    /// the whole face: `fit_pcurve` seeded its first Newton projection from
    /// the patch's parametric middle, and on this strongly curved wall that
    /// converged to a foot point clamped against the far domain bound. The
    /// walk is now seeded where the curve actually starts on the surface.
    #[test]
    fn handle_with_hole_round_trips() {
        build_and_round_trip("handle_with_hole", &handle_with_hole());
    }

    #[test]
    fn luggage_tag_round_trips() {
        build_and_round_trip("luggage_tag", &luggage_tag());
    }

    /// Reproduces the real bug report described on `revolved_cone_on_box`.
    #[test]
    fn airfoil_wing_round_trips() {
        let part = build_and_round_trip("airfoil_wing", &airfoil_wing());
        assert_eq!(part.solid_names(), ["loft(wing)"]);
    }

    #[test]
    fn patterned_plate_round_trips() {
        let part = build_and_round_trip("patterned_plate", &patterned_plate());
        assert_eq!(part.solid_names(), ["linear_pattern(holes)"]);
        for (p, expected) in [
            ([5.0, 1.5, 0.25], PointClassification::Outside),
            ([5.0, 1.9, 0.25], PointClassification::Inside),
            ([7.0, 1.5, 0.25], PointClassification::Outside),
            ([6.0, 1.5, 0.25], PointClassification::Inside),
        ] {
            assert_eq!(inside(&part, "linear_pattern(holes)", p), expected, "{p:?}");
        }
    }

    #[test]
    fn revolved_cone_on_box_round_trips() {
        let part = build_and_round_trip("revolved_cone_on_box", &revolved_cone_on_box());
        // The side face is the plane x = 2.2456, with sketch x along world y
        // and sketch y along world z. The triangle's centroid, turned a
        // quarter around the axis, lies 0.654 off that plane on either side.
        for (p, expected) in [
            ([2.9, -0.198, 1.471], PointClassification::Inside),
            ([1.59, -0.198, 1.471], PointClassification::Inside),
            ([2.9, 1.0, 0.3], PointClassification::Outside),
            ([-0.669, 0.049, 1.0], PointClassification::Outside),
            ([1.5, -1.0, 1.0], PointClassification::Inside),
        ] {
            assert_eq!(inside(&part, "revolve(revolve1)", p), expected, "{p:?}");
        }
    }

    /// A mouse's sole is flat: the crease around its bottom keeps the
    /// bottom face in the cage's bottom plane.
    #[test]
    fn subd_mouse_round_trips() {
        let part = build_and_round_trip("subd_mouse", &subd_mouse());
        assert_eq!(part.solid_names(), ["subd(mouse)"]);
        let sole = part.face_id("subd(mouse,f8)").unwrap();
        let surface = &part.topology().get_face(sole).unwrap().surface;
        let p = surface
            .evaluate(S::from_f64(0.7), S::from_f64(1.2))
            .unwrap();
        assert!(p[2].could_be_equal(S::from_f64(-0.4)), "{p:?}");
    }
}
