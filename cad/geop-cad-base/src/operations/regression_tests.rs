//! Programs from bug reports, with the coordinates they were reported with:
//! a "clean" equivalent may not carry the same numerical case at all.

use geop_core_math::primitives::{DatumComponent, FrameAxis};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_sketch::{Constraint, Sketch};
use geop_core_topology::{
    boundary::BoundaryType,
    loop_sampling::sample_loop_to_polygon,
    validation::{ValidationParameters, validate},
};
use geop_ops::{EntityRef, ORIGIN, Part};
use geop_ops_booleans::Combine;
use geop_ops_datums::{AddDatumArgs, Construction};
use geop_ops_extrude_revolve::ExtrudeArgs;
use geop_ops_rasterize::face_triangles_uv;
use geop_ops_sketch::AddSketchArgs;

use crate::Program;

/// Checks the part `program` builds is a valid model — with every validation error's root message in
/// the failure, one per line — and is drawn as it is (see
/// [`assert_draws_its_trims`]).
fn assert_builds_valid(program: &Program) {
    let part = program.build::<S>().unwrap();
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
        plate.add_point(left, top),
        plate.add_point(right, bottom),
        plate.add_point(right, top),
        plate.add_point(left, bottom),
    ];
    for (i, [a, b]) in [[0, 2], [2, 1], [1, 3], [3, 0]].into_iter().enumerate() {
        let line = plate.add_line(corners[a], corners[b]);
        plate.constrain(if i % 2 == 0 {
            Constraint::Horizontal { line }
        } else {
            Constraint::Vertical { line }
        });
    }
    let center = plate.add_point(-0.15839013445426114, -0.010618161245098214);
    plate.add_circle(center, 1.1037893580479698);
    program.push(
        "sketch1",
        AddSketchArgs {
            plane: EntityRef::datum_component(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
            sketch: plate,
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
    let center = boss.add_point(1.4470132029404787, 0.7057548548938737);
    boss.add_circle(center, 1.1539711985023275);
    program.push(
        "sketch2",
        AddSketchArgs {
            plane: EntityRef::datum("reference1"),
            sketch: boss,
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
