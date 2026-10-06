//! Programs from bug reports, with the coordinates they were reported with:
//! a "clean" equivalent may not carry the same numerical case at all.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::{
    boundary::BoundaryType,
    loop_sampling::sample_loop_to_polygon,
    validation::{ValidationParameters, validate, validate_manifold},
};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, RevolveArgs};
use geop_ops_rasterize::face_triangles_uv;
use geop_ops_sketch::AddSketchArgs;
use geop_ops_sketch::{Constraint, Sketch};

use crate::Program;
use crate::examples::n;

/// Checks the part `program` builds is a valid model — with every validation error's root message in
/// the failure, one per line — and is drawn as it is (see
/// [`assert_draws_its_trims`]).
fn assert_builds_valid(program: &Program) {
    let part = program.build::<S>(&NoFiles).unwrap();
    if let Err(errors) = validate(&ValidationParameters::default(), part.topology()) {
        let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
        let all = messages.join("\n");
        panic!(
            "{} validation error(s):\n{all}\nwhere {}",
            messages.len(),
            names_mentioned(&part, &all).join(", ")
        );
    }
    assert_draws_its_trims(&part);
}

/// Whether `part` is a valid manifold model — if not, every validation
/// error's root message, one per line, and the names of the entities they
/// mention.
pub(crate) fn check_valid(part: &Part<S>) -> Result<(), String> {
    // `validate_manifold` runs `validate` first.
    let Err(errors) = validate_manifold(&ValidationParameters::default(), part.topology()) else {
        return Ok(());
    };
    let all = errors
        .iter()
        .map(|e| e.root_message())
        .collect::<Vec<_>>()
        .join("\n");
    Err(format!(
        "{all}\nwhere {}",
        names_mentioned(part, &all).join(", ")
    ))
}

/// The names of the vertices, edges and faces of `part` that `text` — a
/// validation report, which knows only ids — mentions, as `id = name`.
pub(crate) fn names_mentioned(part: &Part<S>, text: &str) -> Vec<String> {
    let model = part.topology();
    let mentioned = |forms: [String; 3]| forms.iter().any(|f| text.contains(f.as_str()));
    let mut names = Vec::new();
    for &id in model.vertices.keys() {
        if mentioned([
            format!("{id}"),
            format!("vertex {} ", id.0),
            format!("vertex {}'", id.0),
        ]) {
            names.push(format!("{id} = {}", part.name_of(id).unwrap_or("?")));
        }
    }
    for &id in model.edges.keys() {
        if mentioned([
            format!("{id}"),
            format!("edge {} ", id.0),
            format!("edge {}'", id.0),
        ]) {
            names.push(format!("{id} = {}", part.name_of(id).unwrap_or("?")));
        }
    }
    for &id in model.faces.keys() {
        if mentioned([
            format!("{id}"),
            format!("face {} ", id.0),
            format!("face {},", id.0),
        ]) {
            names.push(format!("{id} = {}", part.name_of(id).unwrap_or("?")));
        }
    }
    names.sort();
    names
}

/// Twice the signed area of a closed polygon.
fn area2(polygon: &[[f64; 2]]) -> f64 {
    (0..polygon.len())
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum()
}

/// Checks every face of `part` is drawn exactly where its trims say: the
/// triangles the rasterizer draws it with cover, in `(u, v)`, the area its
/// outer boundary encloses less its holes' — as the rasterizer samples
/// them. A face drawn over a hole it goes around, or leaving a gap, covers
/// more or less. Plain `f64`, like the rasterizer.
fn assert_draws_its_trims(part: &Part<S>) {
    const N: usize = 24;
    let model = part.topology();
    for (&id, face) in &model.faces {
        let enclosed: f64 = face
            .boundaries()
            .filter_map(|b| match b {
                BoundaryType::Loop(anchor) => Some(anchor),
                BoundaryType::Vertex(_) => None,
            })
            .map(|anchor| {
                let polygon: Vec<[f64; 2]> = sample_loop_to_polygon(model, anchor, N)
                    .unwrap()
                    .iter()
                    .map(|p| [p[0].to_f64(), p[1].to_f64()])
                    .collect();
                area2(&polygon) / 2.0
            })
            .sum::<f64>()
            .abs();
        let covered: f64 = face_triangles_uv(model, face, N)
            .unwrap()
            .iter()
            .map(|(a, b, c)| area2(&[a, b, c].map(|p| [p[0].to_f64(), p[1].to_f64()])).abs() / 2.0)
            .sum();
        assert!(
            (covered - enclosed).abs() <= 1e-9 * enclosed.max(1e-9),
            "face {:?} is drawn covering {covered} of its (u, v), where its trims enclose {enclosed}",
            part.name_of(id),
        );
    }
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
/// clipping (see `geop_ops_rasterize::grid`).
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
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: plate,
            ..Default::default()
        },
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: "sketch1".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "reference1",
        AddDatumArgs {
            selection: vec![
                EntityRef::Face {
                    name: "extrude(extrude1,end)".into(),
                },
                EntityRef::Face {
                    name: "extrude(extrude1,sketch1,c4)".into(),
                },
            ],
            construction: Construction::Midplane { other: false },
        },
    );
    let mut boss = Sketch::new();
    let center = boss.add_point(n(1.4470132029404787), n(0.7057548548938737));
    boss.add_circle(center, n(1.1539711985023275));
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: Some(EntityRef::datum("reference1")),
            sketch: boss,
            ..Default::default()
        },
    );
    program.push(
        "extrude2",
        ExtrudeArgs {
            sketch: "sketch2".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::Union {
                target: "extrude(extrude1)".into(),
            },
        },
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
    let mut program = crate::examples::box_with_drill_hole();
    let crate::PartOperation::AddSketch(outline) = &program.steps[0].operation else {
        panic!("the outline comes first");
    };
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
            sketch: "outline".into(),
            axis: Some(EntityRef::SketchCurve {
                sketch: "outline".into(),
                curve: right,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::Union {
                target: "extrude(hole)".into(),
            },
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
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch,
            ..Default::default()
        },
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: "sketch1".into(),
            axis: Some(EntityRef::SketchCurve {
                sketch: "sketch1".into(),
                curve: axis,
            }),
            extent: Extents::blind(360.0),
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: "sketch1".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::Union {
                target: "revolve(revolve1)".into(),
            },
        },
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
    program.push(
        "sketch1",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::Z),
            )),
            sketch: plate,
            ..Default::default()
        },
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: "sketch1".into(),
            extent: Extents::blind(1.0),
            face: false,
            combine: Combine::NewBody,
        },
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
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            sketch: ring,
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
/// `geop-ops-rasterize`).
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
            plane: Some(EntityRef::datum_component(
                ORIGIN,
                DatumComponent::Plane(FrameAxis::X),
            )),
            sketch: outline,
            ..Default::default()
        },
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: "sketch1".into(),
            extent: Extents {
                side2: Some(geop_ops_extrude_revolve::Extent::blind(1.0)),
                ..Extents::blind(1.0)
            },
            face: false,
            combine: Combine::NewBody,
        },
    );
    // The edges swept by the outline's second and fourth corners.
    let edge = |k: usize| EntityRef::Edge {
        name: format!("extrude(extrude1,sketch1,{})", p[k]),
    };
    program.push(
        "reference1",
        AddDatumArgs {
            selection: vec![edge(1), edge(3)],
            construction: Construction::TwoLines {},
        },
    );
    let mut hole = Sketch::new();
    frame_axes(&mut hole);
    let center = hole.add_point(n(1.03222), n(1.727961));
    hole.add_circle(center, n(0.3591930147933918));
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: Some(EntityRef::datum("reference1")),
            sketch: hole,
            ..Default::default()
        },
    );
    program.push(
        "extrude2",
        ExtrudeArgs {
            sketch: "sketch2".into(),
            extent: Extents {
                side1: geop_ops_extrude_revolve::Extent::UpToNext,
                ..Extents::blind(1.0)
            },
            face: false,
            combine: Combine::Difference {
                target: "extrude(extrude1)".into(),
            },
        },
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
            plane: Some(EntityRef::Face {
                name: "extrude(extrude1,start)".into(),
            }),
            sketch: ring,
            ..Default::default()
        },
    );
    program.push(
        "revolve1",
        RevolveArgs {
            sketch: "sketch3".into(),
            axis: Some(EntityRef::SketchCurve {
                sketch: "sketch3".into(),
                curve: if around_x_axis { x_axis } else { line },
            }),
            extent,
            face: false,
            combine,
        },
    );
    program
}

/// Joined to the bracket.
fn joined() -> Combine {
    Combine::Union {
        target: "extrude(extrude2)".into(),
    }
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
        side1: geop_ops_extrude_revolve::Extent::UpToNext,
        reversed,
        ..Extents::blind(1.0)
    };
    bracket_with_torus(bracket, around_x_axis, extent, combine)
}

/// The torus turned up to the next face of the bracket, either way, around
/// `around_x_axis` or its line, combined every way: a new body, joined,
/// cut, intersected.
fn torus_up_to_next(bracket: &Bracket, around_x_axis: bool) {
    let target = || "extrude(extrude2)".to_string();
    for reversed in [false, true] {
        for combine in [
            Combine::NewBody,
            Combine::Union { target: target() },
            Combine::Difference { target: target() },
            Combine::Intersection { target: target() },
        ] {
            let program =
                torus_turned_up_to_next(bracket, around_x_axis, reversed, combine.clone());
            let built = std::panic::catch_unwind(|| assert_builds_valid(&program));
            assert!(
                built.is_ok(),
                "around_x_axis={around_x_axis}, reversed={reversed}, {combine:?}"
            );
        }
    }
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
        Combine::Difference { target },
    ));
}

/// [`torus_up_to_next_stopped_all_round`] either way, combined every way.
#[test]
#[ignore = "slow: eight booleans with a torus (10 s) — run with `cargo test -- --ignored`"]
fn torus_up_to_next_stopped_all_round_every_way() {
    torus_up_to_next(&FULLY_STOPPED, false);
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
        Combine::Difference { target },
    ));
}

/// [`torus_up_to_next_partly_stopped`] around either axis, either way,
/// combined every way.
#[test]
#[ignore = "slow: sixteen booleans with a torus (30 s) — run with `cargo test -- --ignored`"]
fn torus_up_to_next_partly_stopped_every_way() {
    torus_up_to_next(&PARTLY_STOPPED, false);
    torus_up_to_next(&PARTLY_STOPPED, true);
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
            extent: Extents {
                symmetric: true,
                ..Extents::blind(27.240000000000002)
            },
            face: false,
            combine: Combine::NewBody,
        },
    );
    program.push(
        "reference1",
        AddDatumArgs {
            selection: vec![
                EntityRef::Edge {
                    name: format!("extrude(extrude1,sketch1,{},end)", sides[3]),
                },
                EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            ],
            construction: Construction::Angle { angle: 45.0.into() },
        },
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
            plane: Some(EntityRef::datum("reference1")),
            sketch: spindle,
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
            combine,
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
    assert_builds_valid(&conebox(Combine::Union {
        target: "extrude(extrude1)".into(),
    }));
}
