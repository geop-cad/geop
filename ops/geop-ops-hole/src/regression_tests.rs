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
/// The pattern panicked in the booleans' remesh: "about to insert a new
/// vertex at (0, 28, 6), only 0.00071 away from existing vertex
/// (-0.000356, 27.99938, 6)". The seed's angle is `atan(0.5 / 0.866)`,
/// 30.0007°, so the second copy's centre lands at 90.0007°, `3.56e-4`
/// beside the cap's quarter seam `x = 0` (`revolve(flange,profile,c8,a0)`).
/// The copy's own cap is flush with the ring, and its quarter seam
/// `hole(bolt,b0,top,a0)`, a radial line from the centre pole at 60°,
/// crosses the flange's seam at `(0, 28)`, `7.1e-4` along it: one genuine
/// transversal crossing of two straight edges, `7.1e-4` from the pole. The
/// remesh refused the new vertex by a distance threshold standing in for a
/// decision; the crossing is legitimate, and a `7e-4` long edge piece is
/// legitimate topology. The minimal geometry is in `geop-lib-booleans`
/// (`blind_hole_whose_cap_seam_crosses_the_caps_seam_beside_its_centre`).
#[test]
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
