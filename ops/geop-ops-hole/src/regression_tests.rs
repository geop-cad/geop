//! Programs from bug reports, rebuilt with the coordinates they were
//! reported with: a "clean" equivalent may not carry the same case at all.

use geop_core_math::{primitives::FrameAxis, scalars::Scalar, vector::Vector3};
use geop_core_program::program::Program;
use geop_core_topology::Body;
use geop_lib_fixtures::GivenDatumArgs;
use geop_ops_extrude_revolve::{Extents, Output, RevolveArgs, RevolveAxis};
use geop_ops_model::datums::{datum, origin_axis};
use geop_ops_model::sketch::{
    Sketch,
    figures::{on_origin, polygon},
    sketch_element, whole_sketch,
};
use geop_ops_model::spacing::Spacing;
use geop_ops_model::{
    Combine, Extent, NamedModel,
    testing::{Built, assert_valid, assert_volume},
};
use geop_ops_pattern::{CircularPatternArgs, Seeds, TurnAxis, feature};
use geop_ops_sketch::AddSketchArgs;

use crate::{
    HoleArgs, HoleCentre, HoleKind, Standard,
    iso::Fit,
    program_tests::{named, registry},
};

/// The planar faces of `solid` facing `normal`, `height` along it, by name
/// in order.
fn plane_faces_at(part: &NamedModel, solid: &str, normal: [f64; 3], height: f64) -> Vec<String> {
    let id = part.solid_id(solid).unwrap();
    let model = part.topology();
    let n = Vector3::from_array(normal.map(Scalar::from_f64));
    let mut faces: Vec<String> = (model.body_faces(Body::Solid(id)).unwrap().into_iter())
        .filter(|&f| {
            let plane = model.get_face(f).unwrap().surface.as_plane().unwrap();
            plane.is_some_and(|p| {
                p.normal.could_be_equal(&n)
                    && p.point
                        .prod_dot(&n)
                        .could_be_equal(Scalar::from_f64(height))
            })
        })
        .map(|f| part.name_of(f).unwrap().to_string())
        .collect();
    faces.sort();
    faces
}

/// A hole of `kind` and `standard` at the datum point `point`, drilled into
/// `face` right through.
fn hole_at(face: &str, point: &str, kind: HoleKind, standard: Standard) -> HoleArgs {
    HoleArgs {
        face: Some(named(face)),
        points: vec![HoleCentre::Datum(datum(point))],
        kind,
        standard,
        end: Extent::ThroughAll,
        drill_point: false,
        thread_length: None,
    }
}

/// A pipe flange (Fusion 360 "Flange" tutorial: a half section revolved,
/// a bore through the hub, one bolt hole on the bolt circle, patterned six
/// times): a disc of radius 40, 6 thick, with a hub of radius 15 up to
/// `z = 20`, revolved from its half section around the section's side on
/// the axis; a bore of diameter 16 through the hub; an M6 clearance hole
/// at `(28 * 0.866, 28 * 0.5)` on the disc's top — 30° from the `x` axis,
/// as drawn, to a thousandth — patterned six times at 60° around `z`.
///
/// The pattern panics (not an error) in the booleans' remesh:
///
/// > remesh: about to insert a new vertex at (-2e-12, 28.0, 6.0), only
/// > 0.00071 away from existing vertex VertexId(475) at (-0.000356,
/// > 27.99938, 6.0) (threshold=0.001) — likely a spurious near-duplicate
/// > from an under-resolved intersect
///
/// at `remesh_edges_x_edges.rs:256`. The new vertex lies on the cap's
/// quarter seam `x = 0` at radius 28, where the third copy's centre lands
/// to within a thousandth; the existing one 0.0007 off is on the copy's
/// tool. Ruled out, each building valid: the same with the bolt exactly at
/// 30° (`cos`, `sin` of 30°) or at 45°; five copies instead of six; the
/// bolt without the bore; a plain disc instead of the flange, bored and
/// patterned alike; a hole centred on the seam itself, drilled directly.
/// What remains is a copy's centre a thousandth off a seam, met by the
/// sixth copy's boolean.
#[test]
#[ignore = "bug: six M6 holes patterned around a bored flange panic in remesh_edges_x_edges (near-duplicate vertex 7e-4 from a copy's centre on the cap's seam); five copies, an exact 30° seed, or no bore build valid"]
fn bolt_circle_of_a_bored_flange() {
    let mut program = Program::new();
    let mut section = Sketch::new();
    let (_, lines) = polygon(
        &mut section,
        &[
            [0.0, 0.0],
            [40.0, 0.0],
            [40.0, 6.0],
            [15.0, 6.0],
            [15.0, 20.0],
            [0.0, 20.0],
        ],
    );
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), section),
    );
    program.push(
        "flange",
        RevolveArgs {
            sketch: whole_sketch("profile"),
            axis: Some(RevolveAxis::Sketch(sketch_element("profile", lines[5]))),
            extent: Extents::blind(360.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let flange = program.build_model(registry).unwrap();
    let hub_top = plane_faces_at(&flange, "revolve(flange)", [0.0, 0.0, 1.0], 20.0);
    program.push("c", GivenDatumArgs::point([0.0, 0.0, 20.0]));
    program.push(
        "bore",
        hole_at(
            &hub_top[0],
            "c",
            HoleKind::Simple,
            Standard::Custom {
                diameter: 16.0,
                head_diameter: 0.0,
                head_depth: 0.0,
            },
        ),
    );
    let bored = program.build_model(registry).unwrap();
    let ring = plane_faces_at(&bored, "hole(bore)", [0.0, 0.0, 1.0], 6.0);
    program.push("b0", GivenDatumArgs::point([28.0 * 0.866, 28.0 * 0.5, 6.0]));
    program.push(
        "bolt",
        hole_at(
            &ring[0],
            "b0",
            HoleKind::Simple,
            Standard::Iso {
                size: "M6".into(),
                fit: Fit::Normal,
            },
        ),
    );
    program.push(
        "bolts",
        CircularPatternArgs {
            seeds: Seeds::features(vec![feature("bolt")]),
            axis: Some(TurnAxis::Datum(origin_axis(FrameAxis::Z))),
            reversed: false,
            count: 6.0.into(),
            angle: Spacing::step(60.0),
        },
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let pi = std::f64::consts::PI;
    let bolt = pi * 3.3 * 3.3 * 6.0;
    let flange = pi * (40.0 * 40.0 * 6.0 + 15.0 * 15.0 * 14.0 - 8.0 * 8.0 * 20.0);
    assert_volume(
        &part,
        "circular_pattern(bolts)",
        flange - 6.0 * bolt,
        flange * 0.005,
    );
}
