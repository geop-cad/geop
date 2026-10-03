//! Programs from bug reports, with the coordinates they were reported with:
//! a "clean" equivalent may not carry the same numerical case at all.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::{
    boundary::BoundaryType,
    loop_sampling::sample_loop_to_polygon,
    validation::{ValidationParameters, validate},
};
use geop_ops::{EntityRef, NoFiles, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::{ExtrudeArgs, RevolveArgs};
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
        panic!(
            "{} validation error(s):\n{}",
            messages.len(),
            messages.join("\n")
        );
    }
    assert_draws_its_trims(&part);
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
            distance: 1.0,
            symmetric: false,
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
            distance: 1.0,
            symmetric: false,
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
            combine: Combine::NewBody,
        },
    );
    program.push(
        "extrude1",
        ExtrudeArgs {
            sketch: "sketch1".into(),
            distance: 1.0,
            symmetric: false,
            combine: Combine::Union {
                target: "revolve(revolve1)".into(),
            },
        },
    );
    assert_builds_valid(&program);
}
