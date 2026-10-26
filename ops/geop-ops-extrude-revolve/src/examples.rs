//! The examples of extruding, revolving and sweeping: programs written in
//! Rust against the operation types, which an editor offers to load
//! ([`set`]).
//!
//! Each refers to what earlier steps built only by name, and so reads as
//! the recipe it is: `extrude(box,end)` is the end cap of the step `box`,
//! whatever internal id it happens to get.

use std::collections::BTreeMap;

use geop_core_editor::registry::ExampleSet;
use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, vector::Vector3};
use geop_core_program::{
    element::ElementReference, numeric::NumericInput, operation_registry::OperationRegistry,
    part::Part, program::Program, runner::Runner,
};
use geop_core_sketch::CurveId;
use geop_lib_sweep::Orientation;
use geop_ops_model::datums::origin_plane;
use geop_ops_model::parameters::{
    DefineParameters, LookupTable, Material, Parameter, ParameterKind, Parameters, PartProperties,
    Row, set_parameters,
};
use geop_ops_model::sketch::{
    Constraint, Sketch, SketchPlane,
    figures::{circle, n, polygon, rectangle, solved},
    sketch, sketch_element, whole_sketch,
};
use geop_ops_model::sketch3d::{Constraint3d, Sketch3d, sketch3d_path};
use geop_ops_model::{Combine, element};
use geop_ops_sketch::{
    AddSketch, AddSketchArgs,
    references::{Projected, Reference, Source},
};
use geop_ops_sketch3d::AddSketch3dArgs;

use crate::{
    Extent, Extents, Extrude, ExtrudeArgs, Output, PathRef, RevolveArgs, RevolveAxis, SweepArgs,
};

/// The examples of this crate, as an editor offers them.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![
            ("box_with_drill_hole", box_with_drill_hole),
            ("bracket", bracket),
            ("cross_drilled_shaft", cross_drilled_shaft),
            ("handle_with_hole", handle_with_hole),
            ("luggage_tag", luggage_tag),
            ("revolved_cone_on_box", revolved_cone_on_box),
            ("parametric_plate", parametric_plate),
            ("pipe", pipe),
            ("horn", horn),
            ("motor_flange", motor_flange),
            ("edge_part", edge_part),
        ],
        workspaces: Vec::new(),
    }
}

/// The part `program` builds, written in the parameters, sketches and
/// extrusions: what an example reads to draw its next sketch from.
fn built(program: &Program) -> GeopResult<Part> {
    let operations = (OperationRegistry::default())
        .with(DefineParameters)?
        .with(LookupTable)?
        .with(PartProperties)?
        .with(AddSketch)?
        .with(Extrude)?;
    let mut part = Part::default();
    program.run(&Runner::new(operations), &mut part)?;
    Ok(part)
}

/// The flat face `name` of the part, as a plane to sketch on.
fn on_face(name: impl Into<String>) -> SketchPlane {
    SketchPlane::Face(element(name))
}

/// The plane of the origin normal to `normal`, as a plane to sketch on.
fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
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
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(1.0),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );

    let mut hole = Sketch::new();
    circle(&mut hole, [1.0, 1.0], 0.4);
    program.push(
        "hole_sketch",
        AddSketchArgs::on(on_face("extrude(box,end)"), solved(hole)),
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: Extents::blind(-0.5),
            output: Output::Solid(Combine::cut("extrude(box)")),
            draft: 0.0,
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
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(10.0),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );

    let mut hole = Sketch::new();
    circle(&mut hole, [20.0, 20.0], 3.0);
    program.push(
        "hole_sketch",
        AddSketchArgs::on(on_face("extrude(block,end)"), solved(hole)),
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: Extents::blind(-10.0),
            output: Output::Solid(Combine::cut("extrude(block)")),
            draft: 0.0,
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
        AddSketchArgs::on(on_origin(FrameAxis::X), solved(section)),
    );
    program.push(
        "shaft",
        RevolveArgs {
            sketch: whole_sketch("section"),
            axis: Some(RevolveAxis::Sketch(sketch_element("section", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );

    // Sketch x runs along world x, sketch y along world -z: a bore across
    // the thin end, at z = 2.2.
    let mut bore = Sketch::new();
    circle(&mut bore, [0.0, -2.2], 0.25);
    program.push(
        "bore_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Y), solved(bore)),
    );
    program.push(
        "bore",
        ExtrudeArgs {
            sketch: whole_sketch("bore_sketch"),
            extent: Extents {
                side1: Extent::blind(3.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            output: Output::Solid(Combine::cut("revolve(shaft)")),
            draft: 0.0,
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
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "handle",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(1.0),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );

    let mut hole = Sketch::new();
    let center = hole.add_point(n(0.32989396295411244), n(-0.5632891678453777));
    hole.add_circle(center, n(0.31727744879927977));
    program.push(
        "hole_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Y), solved(hole)),
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: Extents {
                side1: Extent::blind(2.44),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            output: Output::Solid(Combine::cut("extrude(handle)")),
            draft: 0.0,
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
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "tag",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents {
                side1: Extent::blind(1.0 / 3.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
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
        AddSketchArgs::on(on_origin(FrameAxis::Z), outline),
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: whole_sketch("sketch1"),
            extent: Extents::blind(1.8900000000000001),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
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
        AddSketchArgs::on(
            on_face(format!("extrude(extrude1,sketch1,{right})")),
            triangle,
        ),
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: whole_sketch("sketch2"),
            axis: Some(RevolveAxis::Sketch(sketch_element("sketch2", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::join("extrude(extrude1)")),
        },
    );
    program
}

/// A plate designed by its parameters: `width` and `depth` — half the
/// width, unless placed otherwise — extruded `thickness` thick, and a blind
/// hole in its middle sized for the screw chosen from a table of sizes,
/// and its colour. The hole's sketch,
/// on the plate's top, projects the top, and puts the hole in the middle
/// of it. The plate is meant to be placed with other values, and
/// everything follows.
pub fn parametric_plate() -> Program {
    let mut program = Program::new();
    set_parameters(
        &mut program,
        Parameters {
            title: None,
            designation: None,
            color: Some("#d0893e".into()),
            material: Material::named("Aluminium 6061"),
            values: vec![
                Parameter::number("width", "4", 1.0, 10.0),
                Parameter::number("depth", "width / 2", 0.5, 10.0),
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
                Parameter::number("thickness", "0.5", 0.1, 2.0),
            ],
        },
    );

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
        (length(lines[0]), NumericInput::from("width")),
        (length(lines[1]), NumericInput::from("depth")),
    ]);
    program.push(
        "outline",
        AddSketchArgs {
            formulas,
            ..AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline))
        },
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind("thickness"),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );

    // The hole's center is the middle of the top's diagonal, between two
    // of its corners as projected: wherever the parameters put them.
    let mut hole = Sketch::new();
    let top = "extrude(plate,end)";
    let mut projection = Reference::new(Source::Projection {
        entity: Projected::Face(element(top)),
    });
    let part = built(&program).expect("the plate builds");
    let reader = part.current();
    let plane = (on_face(top).resolve(&reader)).expect("the top is planar");
    projection
        .update(&mut hole, &reader, &plane)
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
            references: vec![projection],
            formulas: BTreeMap::from([(diameter, NumericInput::from("screw.clearance"))]),
            ..AddSketchArgs::on(on_face(top), solved(hole))
        },
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: Extents::blind(-0.3),
            output: Output::Solid(Combine::cut("extrude(plate)")),
            draft: 0.0,
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
        AddSketchArgs::on(on_origin(FrameAxis::X), solved(section)),
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

/// A horn: a circle swept along a straight path, its size set by a rail —
/// a spline flaring out beside the path, which the circle's point on it
/// follows.
pub fn horn() -> Program {
    let base = |axis| Some(on_origin(axis));
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
            profile: whole_sketch("mouth"),
            path: Some(PathRef::Sketch(ElementReference::whole(sketch("axis")))),
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: vec![PathRef::Sketch(ElementReference::whole(sketch("flare")))],
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// A motor flange 30 across and 2.5 thick, drawn the way the sketch tools
/// draw it: its bore an offset of its rim, 10 in, so the wall follows the
/// rim; and its bolt circle one hole patterned six times round its middle,
/// so the copies follow the first hole.
pub fn motor_flange() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    let middle = outline.add_point(n(0.0), n(0.0));
    outline.constrain(Constraint::Fix {
        point: middle,
        x: n(0.0),
        y: n(0.0),
    });
    let rim = outline.add_circle(middle, n(16.0));
    outline.constrain(Constraint::Radius {
        curve: rim,
        value: n(15.0),
    });
    outline
        .offset(&[rim], n(10.0), geop_core_sketch::offset::Corners::Round)
        .expect("a circle offsets inwards by less than its radius");
    let bolt = circle(&mut outline, [11.0, 0.0], 1.2);
    outline
        .pattern(
            &[bolt],
            &geop_core_sketch::copies::Step::Round {
                center: middle,
                angle: n(std::f64::consts::PI / 3.0),
            },
            6,
        )
        .expect("a hole off the middle turns round it");
    program.push(
        "outline",
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "flange",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(2.5),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );
    program
}

/// Three outlines sketched on the origin's three planes, each extruded and
/// joined to the one before, as drawn in the editor.
///
/// An example drawn in the editor and kept as the program it saved (in
/// `src/drawn/`): its sketches as they were drawn, which no code says
/// better. Its tests build it like every other example, so a change to the
/// format that breaks it is found there: save it again from the editor.
pub fn edge_part() -> Program {
    Program::from_json(include_str!("drawn/edge_part.geop")).expect("a drawn example is a program")
}
