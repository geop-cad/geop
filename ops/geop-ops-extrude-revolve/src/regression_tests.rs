//! Programs from bug reports, with the coordinates they were reported with:
//! a "clean" equivalent may not carry the same numerical case at all.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis, scalars::Scalar};
use geop_core_program::{operation_registry::OperationRegistry, program::Program, testing::args};
use geop_lib_fixtures::{GivenDatum, GivenDatumArgs};
use geop_ops_model::datums::datum;
use geop_ops_model::parameters::{ParameterKind, parameters_of, set_parameters};
use geop_ops_model::sketch::{
    Constraint, Sketch, SketchPlane,
    figures::{disc, n, on_face, on_origin, rectangle_outline},
    sketch_element, whole_sketch,
};
use geop_ops_model::{
    BooleanOp, Combine, solid,
    testing::{Built, assert_draws_its_trims, assert_valid},
};
use geop_ops_sketch::AddSketchArgs;

use crate::{
    Extent, Extents, LoftArgs, Output, RevolveArgs, RevolveAxis, examples, testing::extrusion,
};

/// The operations these programs are written in. The datums the reports
/// built from faces and edges are given at the frames those constructions
/// produced, so a change to the datum operation does not retest these. The
/// mug's cup is shelled by the shell operation, as reported.
fn registry() -> GeopResult<OperationRegistry> {
    crate::testing::registry()?
        .with(GivenDatum)?
        .with(geop_ops_shell::Shell)
}

/// Checks the part `program` builds is valid (see [`assert_valid`]) and is
/// drawn as it is (see [`assert_draws_its_trims`]).
fn assert_builds_valid(program: &Program) {
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_draws_its_trims(&part);
}

/// A 5.6 x 3.7 x 1 plate with a round hole through it, and a cylinder
/// extruded from a datum plane halfway between the plate's top and its back
/// side, joined to the plate. The cylinder reaches over the hole, so its end
/// cap is trimmed by the hole's wall.
///
/// Reported (2026-09-29) as a flat face drawn into the hole: the plate's
/// top, drawn over part of its own hole, not meeting the hole's wall. With
/// the cylinder crossing the hole's rim, the top's hole is no hole any
/// more: one outer boundary runs around the plate, the cylinder and the
/// hole alike. The rasterizer clipped that concave boundary to each grid
/// cell whole, and where it entered a cell twice, the two pieces came back
/// as one polygon bridged along the cell's edge — triangulated, across the
/// gap between them. It now splits a concave boundary into triangles before
/// clipping (see `geop_core_rasterize::grid`).
///
/// The model was not valid either: the join built two pcurves — on the
/// cylinder's side, along the short arcs where it crosses the plate's
/// bottom — that ran up to 5e-6 off their edges. Two causes, both in how an
/// interpolant was made to enclose the curve it was sampled from:
///
/// - Each arc was crossed in a single marching stride, and the tracer, which
///   halved every stride, fitted it through three points: a quadratic, off
///   the true plane x cylinder ellipse by ~4e-6. Every traced curve now gets
///   at least `MIN_TRACED_LEGS` legs, which leaves it a cubic within ~1e-8.
/// - That drift is enclosed as width by measuring it at true points between
///   the samples. In an end interval, where a clamped interpolant drifts
///   most, it was measured at quarters, and a peak between them escaped by
///   ~10%: the branch passed 1.36e-8 from its curve, widened by 1.25e-8, so
///   the pcurve projected from it could not hold the branch either. End
///   intervals are now measured at eighths (`true_point_fractions`).
///
/// The sketches are as drawn, unsolved, and built in the order that gives
/// their points and curves the ids they were reported with.
#[test]
fn cylinder_joined_over_a_hole() {
    let mut program = Program::new();
    let mut plate = Sketch::new();
    let (left, right) = (-2.67283351891566, 2.92361789846824);
    let (bottom, top) = (-1.9899258884866573, 1.7575634084240286);
    let corners = [
        plate.add_point(n(left), n(top)),
        plate.add_point(n(right), n(bottom)),
        plate.add_point(n(right), n(top)),
        plate.add_point(n(left), n(bottom)),
    ];
    for (i, [a, b]) in [[0, 2], [2, 1], [1, 3], [3, 0]].into_iter().enumerate() {
        let line = plate.add_line(corners[a], corners[b]);
        plate.constrain(if i % 2 == 0 {
            Constraint::Horizontal { line }
        } else {
            Constraint::Vertical { line }
        });
    }
    let center = plate.add_point(n(-0.15839013445426114), n(-0.010618161245098214));
    plate.add_circle(center, n(1.1037893580479698));
    program.push(
        "sketch1",
        AddSketchArgs {
            plane: Some(on_origin(FrameAxis::Z)),
            sketch: plate,
            ..Default::default()
        },
    );
    program.push(
        "extrude1",
        extrusion("sketch1", Extents::blind(1.0), Combine::NewBody),
    );
    // The midplane between the plate's top (`z = 1`) and its back side
    // (`y = top`): through the point halfway between the planes' projections
    // of the origin, projected onto it, as that construction puts it.
    program.push(
        "reference1",
        GivenDatumArgs::plane(
            [0.0, (3.0 * top - 1.0) / 4.0, (3.0 - top) / 4.0],
            [0.0, -1.0, 1.0],
        ),
    );
    let mut boss = Sketch::new();
    let center = boss.add_point(n(1.4470132029404787), n(0.7057548548938737));
    boss.add_circle(center, n(1.1539711985023275));
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: Some(SketchPlane::Datum(datum("reference1"))),
            sketch: boss,
            ..Default::default()
        },
    );
    program.push(
        "extrude2",
        extrusion(
            "sketch2",
            Extents::blind(1.0),
            Combine::join("extrude(extrude1)"),
        ),
    );
    assert_builds_valid(&program);
}

/// The drilled box's outline revolved a full turn around its own right
/// edge and joined to the box: a half-cylinder-like solid whose end disks
/// lie in the box's front and back faces, and whose sweep starts in the
/// box's bottom face — coplanar faces and coincident edges on every side.
/// The outline is the solved sketch, so its corners sit ~1e-11 off the
/// round values.
///
/// It panicked in `remesh_edges_x_edges`, about to insert a vertex 3e-12
/// from the box's corner. That crossing was real geometry, not an
/// under-resolved search: the solver meets `Horizontal` and `Vertical` only
/// to its tolerance, so the outline's bottom was ~5e-12 rad off
/// perpendicular to the axis, the box's front face tilted with it, and the
/// revolve swept a cone that flat instead of a disk. Both were built from
/// sharp `f64` positions that claimed to be exact. A sketch is now built
/// from an enclosure of its exact solution (`Sketch::enclose`, a Krawczyk
/// test), so the two faces carry the uncertainty the solve really left, and
/// the crossing could be the corner — which the corner then is.
#[test]
fn outline_revolved_around_its_edge_joined_to_its_box() {
    let mut program = examples::box_with_drill_hole();
    let outline: AddSketchArgs = args(&program.steps[0]);
    let sketch = &outline.sketch;
    let right = sketch
        .curves
        .iter()
        .find(|(_, c)| {
            c.points()
                .iter()
                .all(|p| (sketch.points[p].x.to_f64() - 2.0).abs() < 1e-9)
        })
        .map(|(&id, _)| id)
        .unwrap();
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: whole_sketch("outline"),
            axis: Some(RevolveAxis::Sketch(sketch_element("outline", right))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::join("extrude(hole)")),
        },
    );
    assert_builds_valid(&program);
}

/// A closed spline revolved a full turn around a vertical line beside it —
/// a lumpy torus — and the same sketch then extruded by 1 and joined to it:
/// the extruded spline cylinder cuts through the torus, its side meeting
/// the torus' surface along curves that run near its seam.
///
/// Reported (2026-10-01) as the join failing in `predictor_corrector_step`
/// with a singular Jacobian while tracing the extrusion's side across the
/// torus. Both surfaces contain the profile and the sketch's normal, so they
/// touch tangentially all along it, and the curve where the torus' outer
/// side swings round into the wall leaves the profile at its lowest point —
/// a branch point no vertex marked, so the trace had nowhere to end:
///
/// - Approaching it, the surfaces meet at an ever shallower angle. The
///   marching direction, a normalized cross product of nearly parallel
///   normals, came out wide by percents, and interval elimination on the
///   nearly singular `J^T J` widened the surfaces' own width until a pivot
///   held zero. Both the direction and the matrix only steer the corrector,
///   so both are now sharpened (see `predictor_corrector_step`).
/// - Past that, the march ran into the profile at no vertex, took a step
///   whose enclosure spanned the model, and ended at an unrelated vertex
///   0.45 away that the wide box made look near. Branch points like this
///   one — where the two surfaces' mean curvatures agree along an edge they
///   touch along — are now split into the edge before anything else
///   (`remesh_tangent_branches`).
///
/// The sketch is as drawn, unsolved, and built in the order that gives its
/// points and curves the ids they were reported with.
#[test]
fn spline_revolved_then_extruded_and_joined() {
    let mut program = Program::new();
    let mut sketch = Sketch::new();
    let outline: Vec<_> = [
        (-3.1375704298183145, 2.5293125606373428),
        (-5.344138256576594, 1.9223976345215419),
        (-5.219142443284211, 0.13987914384067612),
        (-2.525717167424303, -0.491323660236492),
        (-1.3584048099029444, 1.0913093781025136),
    ]
    .into_iter()
    .map(|(x, y)| sketch.add_point(n(x), n(y)))
    .collect();
    let mut control_points = outline.clone();
    control_points.push(outline[0]);
    sketch.add_spline(control_points);
    let top = sketch.add_point(n(0.4655769138883318), n(2.6284122396718033));
    let bottom = sketch.add_point(n(0.4655769138883318), n(-0.7084336105288418));
    let axis = sketch.add_line(top, bottom);
    sketch.constrain(Constraint::Vertical { line: axis });
    program.push(
        "sketch1",
        geop_ops_sketch::AddSketchArgs::on(on_origin(FrameAxis::Z), sketch),
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: whole_sketch("sketch1"),
            axis: Some(RevolveAxis::Sketch(sketch_element("sketch1", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.push(
        "extrude1",
        extrusion(
            "sketch1",
            Extents::blind(1.0),
            Combine::join("revolve(revolve1)"),
        ),
    );
    assert_builds_valid(&program);
}

/// A plate with a hole, and on a plane across it a stepped profile drawn
/// from the plate's top edge, revolved about that sketch's own `x` axis and
/// joined to the plate — as reported, coordinates and all. The revolve's
/// flat ring at `y = 6.59` is built of two halves whose seam lies in the
/// plate's bottom plane, exactly where the ring crosses the plate's bottom
/// face: the intersection curve is that seam.
#[test]
fn ring_revolved_onto_plate() {
    let mut program = Program::new();

    let mut plate = Sketch::new();
    let corners: Vec<_> = [
        (7.500000000000002, 7.499999999999998),
        (-7.500000000000002, 7.499999999999998),
        (-7.500000000000002, -7.499999999999998),
        (7.500000000000002, -7.5),
    ]
    .into_iter()
    .map(|(x, y)| plate.add_point(n(x), n(y)))
    .collect();
    for i in 0..4 {
        plate.add_line(corners[i], corners[(i + 1) % 4]);
    }
    let center = plate.add_point(n(0.0), n(0.0));
    plate.add_circle(center, n(1.4999999999999998));
    program.push("sketch1", AddSketchArgs::on(on_origin(FrameAxis::Z), plate));
    program.push(
        "extrude1",
        extrusion("sketch1", Extents::blind(1.0), Combine::NewBody),
    );

    let mut ring = Sketch::new();
    let profile: Vec<_> = [
        (7.500000000000001, 1.0000000000000004),
        (7.500000000000001, 7.680535586464354),
        (11.729022707616181, 7.680535586464354),
        (11.729022707616181, 14.13762302641227),
        (10.279264816059785, 14.137623026412271),
        (10.279264816059785, 8.858670838665377),
        (6.586912686002086, 8.858670838665377),
        (6.586912686002086, 1.0000000000000004),
    ]
    .into_iter()
    .map(|(x, y)| ring.add_point(n(x), n(y)))
    .collect();
    for i in 0..profile.len() {
        ring.add_line(profile[i], profile[(i + 1) % profile.len()]);
    }
    let origin = ring.add_point(n(0.0), n(0.0));
    let along = ring.add_point(n(1.0), n(0.0));
    let axis = ring.add_line(origin, along);
    ring.set_construction(axis, true);
    program.push("sketch2", AddSketchArgs::on(on_origin(FrameAxis::X), ring));
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: whole_sketch("sketch2"),
            axis: Some(RevolveAxis::Sketch(sketch_element("sketch2", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::join("extrude(extrude1)")),
        },
    );
    assert_builds_valid(&program);
}

/// The frame axes every sketch drawn in the editor starts with: fixed
/// construction lines from its origin along `x` and `y`. Returns the `x`
/// line.
fn frame_axes(sketch: &mut Sketch) -> geop_core_sketch::CurveId {
    let origin = sketch.add_fixed_point(n(0.0), n(0.0));
    let x = sketch.add_fixed_point(n(1.0), n(0.0));
    let y = sketch.add_fixed_point(n(0.0), n(1.0));
    let x_axis = sketch.add_line(origin, x);
    let y_axis = sketch.add_line(origin, y);
    for line in [x_axis, y_axis] {
        sketch.set_construction(line, true);
    }
    x_axis
}

/// The two brackets of `bracket_with_torus`: their outlines' lower-left
/// corners, which differ, and where the ring's sketch puts its line.
struct Bracket {
    corners: [(f64, f64); 4],
    line: (f64, f64, f64),
}

/// The first bracket: a torus around its line turns a half turn out of the
/// bracket, partly lands on its slanted arm, and partly passes on.
const PARTLY_STOPPED: Bracket = Bracket {
    corners: [
        (-1.643567, -2.654248),
        (-2.560606, 1.310979),
        (-3.335569, 1.033284),
        (-2.231247, -3.816692),
    ],
    line: (-1.050451, 1.550927, 0.42898),
};

/// The second: the arm is wide enough to stop the torus all round after a
/// half turn.
const FULLY_STOPPED: Bracket = Bracket {
    corners: [
        (-1.367377992154214, -2.470121649185356),
        (-2.2844172960685256, 1.4951046367539953),
        (-3.5606118527937647, 0.9514499643977747),
        (-2.456289874136388, -3.898525509120877),
    ],
    line: (-1.306181007097579, 1.7657405500456889, 0.6437934059762662),
};

/// A bracket-like outline extruded both ways from the X plane, a round hole
/// cut into it up to next from a plane through two of its edges, and a
/// circle drawn on its start face revolved into a torus and joined to it —
/// around the sketch's own `x` axis, which lies in that face, or around a
/// line of the sketch beside the circle. The torus around the `x` axis
/// meets the face exactly along two of its own meridians.
///
/// Around the `x` axis, remesh traced an intersection curve along a
/// meridian already lying in the face and failed to splice it ("degenerate
/// split"); around the line, the boolean worked, but drawing the result ran
/// out of memory (see `a_concave_hole_of_many_corners_stays_cheap` in
/// `geop-core-rasterize`).
fn bracket_with_torus(
    bracket: &Bracket,
    around_x_axis: bool,
    extent: Extents,
    combine: Combine,
) -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    frame_axes(&mut outline);
    let [c0, c1, c2, c3] = bracket.corners;
    let p = [
        (1.798277, -1.687521),
        (1.798277, -0.759183),
        (-0.465176, -0.759183),
        (-0.465176, 1.860703),
        (-1.351225, 1.860703),
        (-0.674863, -2.137606),
        c0,
        c1,
        c2,
        c3,
        (-1.131652, -3.164469),
    ]
    .map(|(x, y)| outline.add_point(n(x), n(y)));
    let lines: Vec<_> = (0..p.len())
        .map(|i| outline.add_line(p[i], p[(i + 1) % p.len()]))
        .collect();
    outline.constrain(Constraint::Vertical { line: lines[0] });
    outline.constrain(Constraint::Horizontal { line: lines[1] });
    outline.constrain(Constraint::Vertical { line: lines[2] });
    outline.constrain(Constraint::Horizontal { line: lines[3] });
    program.push(
        "sketch1",
        AddSketchArgs {
            plane: Some(on_origin(FrameAxis::X)),
            sketch: outline,
            ..Default::default()
        },
    );
    program.push(
        "extrude1",
        extrusion(
            "sketch1",
            Extents {
                side2: Some(crate::Extent::blind(1.0)),
                ..Extents::blind(1.0)
            },
            Combine::NewBody,
        ),
    );
    // The plane through the edges swept by the outline's second and fourth
    // corners, both along `x`: from the second's start, at `x = -1`, square
    // to `x` and to the line between the corners (which the sketch on the
    // X plane puts at `(y, z)`), as that construction puts it.
    let (y1, z1) = (1.798277, -0.759183);
    let (y3, z3) = (-0.465176, 1.860703);
    program.push(
        "reference1",
        GivenDatumArgs::plane([-1.0, y1, z1], [0.0, -(z3 - z1), y3 - y1]),
    );
    let mut hole = Sketch::new();
    frame_axes(&mut hole);
    let center = hole.add_point(n(1.03222), n(1.727961));
    hole.add_circle(center, n(0.3591930147933918));
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: Some(SketchPlane::Datum(datum("reference1"))),
            sketch: hole,
            ..Default::default()
        },
    );
    program.push(
        "extrude2",
        extrusion(
            "sketch2",
            Extents {
                side1: crate::Extent::UpToNext,
                ..Extents::blind(1.0)
            },
            Combine::cut("extrude(extrude1)"),
        ),
    );
    let mut ring = Sketch::new();
    let x_axis = frame_axes(&mut ring);
    let (x, y0, y1) = bracket.line;
    let (a, b) = (ring.add_point(n(x), n(y0)), ring.add_point(n(x), n(y1)));
    let line = ring.add_line(a, b);
    ring.constrain(Constraint::Vertical { line });
    let center = ring.add_point(n(-0.275127), n(1.238878));
    ring.add_circle(center, n(0.39413315729396897));
    program.push(
        "sketch3",
        AddSketchArgs {
            plane: Some(on_face("extrude(extrude1,start)")),
            sketch: ring,
            ..Default::default()
        },
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: whole_sketch("sketch3"),
            axis: Some(RevolveAxis::Sketch(sketch_element(
                "sketch3",
                if around_x_axis { x_axis } else { line },
            ))),
            extent,
            output: Output::Solid(combine),
        },
    );
    program
}

/// Joined to the bracket.
fn joined() -> Combine {
    Combine::join("extrude(extrude2)")
}

#[test]
fn torus_around_a_line_beside_it_joined_to_a_bracket() {
    assert_builds_valid(&bracket_with_torus(
        &PARTLY_STOPPED,
        false,
        Extents::blind(360.0),
        joined(),
    ));
}

#[test]
fn torus_around_an_axis_in_its_face_joined_to_a_bracket() {
    assert_builds_valid(&bracket_with_torus(
        &PARTLY_STOPPED,
        true,
        Extents::blind(360.0),
        joined(),
    ));
}

/// The torus turned up to the next face of the bracket, `reversed` or not,
/// around `around_x_axis` or its line.
fn torus_turned_up_to_next(
    bracket: &Bracket,
    around_x_axis: bool,
    reversed: bool,
    combine: Combine,
) -> Program {
    let extent = Extents {
        side1: crate::Extent::UpToNext,
        reversed,
        ..Extents::blind(1.0)
    };
    bracket_with_torus(bracket, around_x_axis, extent, combine)
}

/// The torus turned up to the next face of the bracket, either way, around
/// `around_x_axis` or its line, combined every way: a new body, joined,
/// cut, intersected. A new body the bracket does not stop all round has no
/// way to go on all the way round — it would overlap the bracket — and is
/// refused, saying nothing stops it.
fn torus_up_to_next(bracket: &Bracket, around_x_axis: bool, stopped_all_round: bool) {
    let target = || "extrude(extrude2)".to_string();
    let mut failures = Vec::new();
    for reversed in [false, true] {
        for combine in [
            Combine::NewBody,
            Combine::join(target()),
            Combine::cut(target()),
            Combine::With {
                op: BooleanOp::Intersection,
                target: solid(target()),
            },
        ] {
            let program =
                torus_turned_up_to_next(bracket, around_x_axis, reversed, combine.clone());
            let case = format!("around_x_axis={around_x_axis}, reversed={reversed}, {combine:?}");
            if combine == Combine::NewBody && !stopped_all_round {
                match program.build_model(registry) {
                    Err(e) if e.root_message().contains("nothing stops the profile") => {}
                    other => failures.push(format!("{case}: not refused: {:?}", other.err())),
                }
                continue;
            }
            if std::panic::catch_unwind(|| assert_builds_valid(&program)).is_err() {
                failures.push(case);
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}

/// Stopped all round after a half turn: the piece up to there, cut from
/// the bracket.
#[test]
fn torus_up_to_next_stopped_all_round() {
    let target = "extrude(extrude2)".to_string();
    assert_builds_valid(&torus_turned_up_to_next(
        &FULLY_STOPPED,
        false,
        false,
        Combine::cut(target),
    ));
}

/// [`torus_up_to_next_stopped_all_round`] either way, combined every way.
#[test]
#[ignore = "slow: eight booleans with a torus (10 s) — run with `cargo test -- --ignored`"]
fn torus_up_to_next_stopped_all_round_every_way() {
    torus_up_to_next(&FULLY_STOPPED, false, true);
}

/// Stopped by only part of the profile: as far as it first meets the
/// bracket — for a cut or an intersection coming from outside, on to where
/// it next does, or all the way round if part of it never meets it again.
///
/// The reversed cut around the `x` axis is the case that pins the second
/// half of that: it first meets the bracket flat against the start face, a
/// half turn round. Stopping flat at that first contact put the tool's end
/// face exactly through the corner where the drilled hole's rim crosses the
/// torus, and the boolean then had to cross an edge at its own vertex — a
/// degenerate configuration that came out too wide to validate. Going on
/// up to the next face from there (or all the way) never stops on a corner
/// that is already there.
#[test]
fn torus_up_to_next_partly_stopped() {
    let target = "extrude(extrude2)".to_string();
    assert_builds_valid(&torus_turned_up_to_next(
        &PARTLY_STOPPED,
        true,
        true,
        Combine::cut(target),
    ));
}

/// [`torus_up_to_next_partly_stopped`] around either axis, either way,
/// combined every way.
#[test]
#[ignore = "slow: sixteen booleans with a torus (30 s) — run with `cargo test -- --ignored`"]
fn torus_up_to_next_partly_stopped_every_way() {
    torus_up_to_next(&PARTLY_STOPPED, false, false);
    torus_up_to_next(&PARTLY_STOPPED, true, false);
}

/// The box of the user's conebox part, and a spindle revolved a full turn
/// about an axis in a plane through the box's top edge, turned 45 degrees
/// from the top (see [`conebox`]): the axis crosses that edge at its
/// middle. Reported (2026-10-06) as the join failing in `splice_edge_into_face`
/// ("cannot tell whether an edge leaving ... runs into the corner"): "New
/// part" built, "Join" did not.
fn conebox(combine: Combine) -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    frame_axes(&mut outline);
    let (x, y) = (20.416121033435203, 17.42175661519803);
    let corners = [
        outline.add_point(n(x), n(y)),
        outline.add_point(n(-x), n(y)),
        outline.add_point(n(-x), n(-y)),
        outline.add_point(n(x), n(-y)),
    ];
    let sides: Vec<_> = (0..4)
        .map(|i| outline.add_line(corners[i], corners[(i + 1) % 4]))
        .collect();
    for (i, &line) in sides.iter().enumerate() {
        outline.constrain(if i % 2 == 0 {
            Constraint::Horizontal { line }
        } else {
            Constraint::Vertical { line }
        });
    }
    let diagonal = outline.add_line(corners[0], corners[2]);
    outline.set_construction(diagonal, true);
    outline.constrain(Constraint::Midpoint {
        point: geop_core_sketch::PointId(0),
        curve: diagonal,
    });
    program.push(
        "sketch1",
        AddSketchArgs {
            plane: Some(on_origin(FrameAxis::Z)),
            sketch: outline,
            ..Default::default()
        },
    );
    program.push(
        "extrude1",
        extrusion(
            "sketch1",
            Extents {
                symmetric: true,
                ..Extents::blind(27.240000000000002)
            },
            Combine::NewBody,
        ),
    );
    // The plane through the box's top edge at `x` (the one its last side
    // swept, along `y`), turned 45 degrees from the top: from the edge's
    // start, with its normal between `-x` and `z`, as that construction
    // puts it.
    program.push(
        "reference1",
        GivenDatumArgs::plane([x, y, 27.240000000000002 / 2.0], [-1.0, 0.0, 1.0]),
    );
    let mut spindle = Sketch::new();
    frame_axes(&mut spindle);
    let apex = geop_core_sketch::PointId(0);
    let tip = spindle.add_point(n(-22.390558010610484), n(0.0));
    let axis = spindle.add_line(apex, tip);
    let belly = spindle.add_point(n(-12.732689325241626), n(-9.735061404254687));
    spindle.add_line(tip, belly);
    spindle.add_line(belly, apex);
    spindle.constrain(Constraint::Horizontal { line: axis });
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: Some(SketchPlane::Datum(datum("reference1"))),
            sketch: spindle,
            ..Default::default()
        },
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: whole_sketch("sketch2"),
            axis: Some(RevolveAxis::Sketch(sketch_element("sketch2", axis))),
            extent: Extents::blind(360.0),
            output: Output::Solid(combine),
        },
    );
    program
}

#[test]
fn spindle_revolved_about_an_axis_through_the_middle_of_a_box_edge_as_a_new_body() {
    assert_builds_valid(&conebox(Combine::NewBody));
}

#[test]
fn spindle_revolved_about_an_axis_through_the_middle_of_a_box_edge_and_joined() {
    assert_builds_valid(&conebox(Combine::join("extrude(extrude1)")));
}

/// The parametric plate made 6 wide with an M6 hole. Its cut once left the
/// result open around the hole's rim: the disc cut from the top was taken
/// for outside the hole, because the point classifying it, beside the rim,
/// was found inside the disc by a ray grazing the rim — cast from a box
/// around the point, it was a strip, and touching the rim counted as one
/// crossing. Reduced to a plate and a cylinder in the booleans' tests.
#[test]
fn wide_plate_with_an_m6_hole() {
    let mut plate = examples::parametric_plate();
    let mut parameters = parameters_of(&plate);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
        panic!("width is a number")
    };
    *expression = "6".into();
    let screw = parameters.values.iter_mut().find(|p| p.name == "screw");
    let Some(ParameterKind::Table { selected, .. }) = screw.map(|p| &mut p.kind) else {
        panic!("screw is a table")
    };
    *selected = "M6".into();
    set_parameters(&mut plate, parameters);
    assert_builds_valid(&plate);
}

/// Two "D" sections lofted — a computer mouse's body, as the surfacing
/// tutorials start it: on the X plane at `x = 0` a flat bottom line from
/// `(-1, 0)` to `(1, 0)` under two arcs meeting at the apex `(0, 0.9)`, and
/// at `x = 2` the same shape wider and taller, its bottom from `(-1.4, 0)`
/// to `(1.4, 0)` and its apex at `(0, 1.5)`. Both sketches have three
/// curves and three corners, the corners at the ends of the bottom line.
///
/// The second section's arcs each sweep 94°, more than a quarter turn, so
/// the kernel draws each in two pieces: five curves against the first's
/// three. The loft used to make the counts agree by halving the first
/// section's longest curves — its bottom line first — and then match the
/// pieces in order: the bottom's second half was lofted into the first
/// piece of an arc, and the face from it left `z = 0`, its far corner at
/// `(2, -1.03, 1.05)` on the dome: the bottom of the body, flat in every
/// section, twisted, with no error. Now the corners of the sections —
/// where the curves meet at an angle — are matched, and the bottom line
/// lofts into the bottom line.
///
/// Ruled out: with arcs under a quarter turn in both sections (an apex of
/// 0.9 at both) the bottom stays flat; it is the kernel's own splitting of
/// an arc at a quarter turn that makes the counts differ, which a user
/// cannot see in either sketch.
#[test]
fn loft_of_two_d_sections_keeps_the_bottom_flat() {
    /// A D on the X plane at `x`: the bottom from `(-w, 0)` to `(w, 0)`,
    /// two arcs through `(0, h)` over it.
    fn section(program: &mut Program, name: &str, x: f64, w: f64, h: f64) {
        program.push(
            format!("{name}_plane"),
            GivenDatumArgs::plane([x, 0.0, 0.0], [1.0, 0.0, 0.0]),
        );
        let mut s = Sketch::new();
        let a = s.add_point(n(-w), n(0.0));
        let b = s.add_point(n(w), n(0.0));
        let top = s.add_point(n(0.0), n(h));
        s.add_line(a, b);
        // The circle through the three points has radius (w² + h²) / 2h;
        // each arc's sweep is twice the half angle its chord subtends.
        let r = (w * w + h * h) / (2.0 * h);
        let sweep = 2.0 * ((w * w + h * h).sqrt() / (2.0 * r)).asin();
        s.add_arc(b, top, n(sweep));
        s.add_arc(top, a, n(sweep));
        program.push(
            name,
            AddSketchArgs::on(SketchPlane::Datum(datum(format!("{name}_plane"))), s),
        );
    }
    let mut program = Program::new();
    section(&mut program, "s0", 0.0, 1.0, 0.9);
    section(&mut program, "s1", 2.0, 1.4, 1.5);
    program.push(
        "body",
        LoftArgs {
            profiles: vec![whole_sketch("s0"), whole_sketch("s1")],
            matches: Vec::new(),
            guides: Vec::new(),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let model = part.topology();
    // Every face lofted from the bottom line `c3` (or a piece of it) lies in
    // z = 0 — the caps `loft(body,start)` and `loft(body,end)` aside.
    let mut off = Vec::new();
    for (&id, _) in &model.faces {
        let name = part.name_of(id).unwrap();
        if !name.contains(",c3") {
            continue;
        }
        for c in model.iterate_face_coedges(id) {
            let p = model.coedge_start_vertex(c).unwrap().point;
            if !p[2].could_be_equal(Scalar::ZERO) {
                off.push((name.to_string(), p.to_f64_array()));
            }
        }
    }
    assert!(
        off.is_empty(),
        "faces lofted from the bottom line leave z = 0: {off:?}"
    );
}

/// A 2 x 2 x 1 block with a round hole of radius 0.5 cut through it around
/// `(1, 1.5)`: the hole's wall touches the block's side `y = 2` along a
/// line. From the SolidWorks "zero-thickness geometry" lesson: every
/// tutorial of a boss or a cut tangent to a wall warns that the kernel must
/// refuse it, since the result's boundary touches itself along the line of
/// tangency and is no solid.
///
/// The cut builds without complaint, and `assert_valid` then fails with
/// "edge .. has 4 coedge(s) (expected exactly 2 for a manifold shell)" on
/// the edge where the hole's wall meets the side. Expected: a refusal
/// naming the tangent faces (the cylinder's wall and the block's side), or
/// a solid that is valid. Ruled out: the hole's ends (the tool stands out
/// of both caps by 1) and the sketch (the disc is the sketch's own circle).
#[test]
#[ignore = "bug: a cut tangent to a side face builds a non-manifold solid (an edge with 4 coedges where the hole's wall touches the side) instead of being refused by name"]
fn cut_tangent_to_a_side_face_is_not_refused() {
    let mut program = Program::new();
    program.push(
        "outline",
        AddSketchArgs::on(
            on_origin(FrameAxis::Z),
            rectangle_outline(0.0, 0.0, 2.0, 2.0),
        ),
    );
    program.push(
        "block",
        extrusion("outline", Extents::blind(1.0), Combine::NewBody),
    );
    program.push(
        "hole_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), disc([1.0, 1.5], 0.5)),
    );
    program.push(
        "hole",
        extrusion(
            "hole_sketch",
            Extents {
                side1: Extent::blind(2.0),
                symmetric: false,
                side2: Some(Extent::blind(1.0)),
                reversed: false,
            },
            Combine::cut("extrude(block)"),
        ),
    );
    match program.build_model(registry) {
        Ok(part) => assert_valid(&part),
        Err(e) => {
            let text = e.to_string();
            assert!(text.contains("tangent"), "{text}");
        }
    }
}

/// The user's mug: a cup of radius 40 and height 95 on the XY plane,
/// shelled 4 thick open at its top, and on the ZX plane an annulus beside
/// it — radii 16 and 9 around `(x, z) = (69.19, 52.36)` — turned around
/// the cup's seam (`extrude(body,body_sk,c1@0)`) and joined to the cup.
fn mug(extent: Extents) -> Program {
    use geop_core_sketch::{Curve, CurveId, CurveKind, Point, PointId};
    let mut program = Program::new();
    let mut body = Sketch::new();
    let center = body.add_fixed_point(n(0.0), n(0.0));
    body.add_circle(center, n(40.0));
    program.push("body_sk", AddSketchArgs::on(on_origin(FrameAxis::Z), body));
    program.push(
        "body",
        extrusion("body_sk", Extents::blind(95.0), Combine::NewBody),
    );
    program.push(
        "hollow",
        geop_ops_shell::ShellArgs {
            solid: solid("extrude(body)"),
            faces: vec![geop_ops_model::element("extrude(body,end)")],
            thickness: 4.0.into(),
        },
    );
    // As drawn in the editor, ids and all: the annulus, and the sketch's
    // own origin and axes.
    let mut handle = Sketch::new();
    let point = |x: f64, y: f64, fixed| Point {
        x: n(x),
        y: n(y),
        fixed,
    };
    for (id, p) in [
        (0, point(0.0, 0.0, true)),
        (1, point(69.18884740659647, -52.35578658107838, false)),
        (3, point(0.0, 0.0, true)),
        (4, point(1.0, 0.0, true)),
        (5, point(0.0, 1.0, true)),
    ] {
        handle.points.insert(PointId(id), p);
    }
    let curve = |kind, fixed| Curve {
        kind,
        construction: fixed,
        fixed,
    };
    let circle = |radius| CurveKind::Circle {
        center: PointId(1),
        radius: n(radius),
    };
    let line = |end| CurveKind::Line {
        start: PointId(3),
        end: PointId(end),
    };
    for (id, c) in [
        (1, curve(circle(16.0), false)),
        (2, curve(circle(9.0), false)),
        (6, curve(line(4), true)),
        (7, curve(line(5), true)),
    ] {
        handle.curves.insert(CurveId(id), c);
    }
    handle.next_id = 8;
    program.push(
        "handle_sk",
        AddSketchArgs::on(on_origin(FrameAxis::Y), handle),
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: whole_sketch("handle_sk"),
            axis: Some(RevolveAxis::Edge(geop_ops_model::element(
                "extrude(body,body_sk,c1@0)",
            ))),
            extent,
            output: Output::Solid(Combine::join("shell(hollow)")),
        },
    );
    program
}

/// [`mug`] turned up to next: the annulus, 13.19 to 45.19 from the seam,
/// sweeps round it from where it was drawn until it meets the cup's outer
/// wall, each ring of it at its own angle — the handle from the profile to
/// the wall, the cup's inside left as it was.
///
/// Reported (2026-10-25) as refused: "the target lies all around the
/// revolve axis". The cup only touches the axis, along its seam, and fills
/// exactly the half turn on the far side of it. `turn_past` judged that
/// from the convex hull of the cup's control points, which three things
/// made reach round the axis: the seam's own control point lies on the
/// axis and was taken for a hull the axis runs through; the floor inside
/// the cup, a disc of radius 36, lies on a plane patch of 240 square, whose
/// corners stand out on every side; and the corners of the outer cylinder's
/// control net lie on the plane touching it along the seam, the cup's
/// half turn rounded to 179.9999999999999 degrees. Now a face is hulled
/// over the box of its own outer loop (`face_hull`), and the hull's points
/// are measured as enclosed: a point that could lie on the axis says
/// nothing, and the gap is too small only where its two bounding points
/// definitely leave less than a half turn.
#[test]
fn mug_handle_turned_up_to_next_around_the_cup_seam() {
    let program = mug(crate::testing::one_way(Extent::UpToNext));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_draws_its_trims(&part);
    // The cup, π (40² 95 - 36² 91), and the handle: each point of the
    // annulus at `r` from the seam turned by `acos(-r / 80)`, where it
    // reaches the wall, `∫ r acos(-r / 80) dA` = 31877.
    let cup = std::f64::consts::PI * (40.0f64.powi(2) * 95.0 - 36.0f64.powi(2) * 91.0);
    let mug = geop_ops_model::testing::solid_volume(&part, "revolve(revolve1)");
    assert!(
        (mug - (cup + 31877.0)).abs() < 300.0,
        "the mug encloses {mug}, the cup {cup}"
    );
    // A ring of the annulus, 12.5 out from its centre and 41.69 from the
    // seam, turned 60 degrees one way: in the handle, on one side of the
    // cup only; turned 130, past the wall at 121.4: in the cup's hollow,
    // left empty.
    use geop_core_topology::contains::shell::PointClassification::{Inside, Outside};
    let ring = |degrees: f64, side: f64| {
        let (r, a) = (69.18884740659647 - 40.0 + 12.5, degrees.to_radians());
        let p = [40.0 + r * a.cos(), side * r * a.sin(), 52.35578658107838];
        geop_ops_model::testing::classify(&part, "revolve(revolve1)", p)
    };
    let sides = [ring(60.0, 1.0), ring(60.0, -1.0)];
    assert!(
        sides == [Inside, Outside] || sides == [Outside, Inside],
        "{sides:?}"
    );
    assert_eq!([ring(130.0, 1.0), ring(130.0, -1.0)], [Outside, Outside]);
}

/// [`mug`] turned up to next the other way round the seam: the mirror
/// image of [`mug_handle_turned_up_to_next_around_the_cup_seam`].
#[test]
fn mug_handle_turned_up_to_next_the_other_way_around_the_cup_seam() {
    let program = mug(Extents {
        reversed: true,
        ..crate::testing::one_way(Extent::UpToNext)
    });
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let cup = std::f64::consts::PI * (40.0f64.powi(2) * 95.0 - 36.0f64.powi(2) * 91.0);
    let mug = geop_ops_model::testing::solid_volume(&part, "revolve(revolve1)");
    assert!(
        (mug - (cup + 31877.0)).abs() < 300.0,
        "the mug encloses {mug}, the cup {cup}"
    );
}

/// [`mug`] turned up to next both ways: the whole handle, from the wall
/// round to the wall, twice the volume of one side.
///
/// Each side reached a turn past the cup on its own, 315 degrees, so the
/// second side's tool, turned 315 back, ended at 45 forward, inside the
/// handle the first side had just joined to the cup. Between 45 and the
/// wall the two tools are the same tori, and the second side's union failed
/// on them: an edge it traced (`combine(revolve1,side2,~56)`, its control
/// points 1.5e-2 wide, as a curve traced between two coincident surfaces
/// comes out) was imprinted into the second tool's outer torus over that
/// stretch (`revolve(revolve1,side2,handle_sk,c1#3,q3)`) along its edge
/// `c1@3,q3`: "degenerate split". The sides now share the turn (see
/// `Extents::plan`): each stops halfway through where they would overlap,
/// at 180, in the cup, where both have met the wall before.
#[test]
fn mug_handle_turned_up_to_next_both_ways_around_the_cup_seam() {
    let program = mug(Extents {
        side2: Some(Extent::UpToNext),
        ..crate::testing::one_way(Extent::UpToNext)
    });
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    assert_draws_its_trims(&part);
    let cup = std::f64::consts::PI * (40.0f64.powi(2) * 95.0 - 36.0f64.powi(2) * 91.0);
    let mug = geop_ops_model::testing::solid_volume(&part, "revolve(revolve1)");
    assert!(
        (mug - (cup + 2.0 * 31877.0)).abs() < 600.0,
        "the mug encloses {mug}, the cup {cup}"
    );
    // The ring of [`mug_handle_turned_up_to_next_around_the_cup_seam`],
    // either way: in the handle where it was drawn, at 60 degrees, and at
    // 115, outside the cup just before it meets the wall at 121.4; at 130,
    // past the wall, in the cup's hollow, left empty.
    use geop_core_topology::contains::shell::PointClassification::{Inside, Outside};
    let ring = |degrees: f64, side: f64| {
        let (r, a) = (69.18884740659647 - 40.0 + 12.5, degrees.to_radians());
        let p = [40.0 + r * a.cos(), side * r * a.sin(), 52.35578658107838];
        geop_ops_model::testing::classify(&part, "revolve(revolve1)", p)
    };
    assert_eq!(ring(0.0, 1.0), Inside);
    for degrees in [60.0, 115.0] {
        assert_eq!([ring(degrees, 1.0), ring(degrees, -1.0)], [Inside, Inside]);
    }
    assert_eq!([ring(130.0, 1.0), ring(130.0, -1.0)], [Outside, Outside]);
}
