//! Programs from tutorials and bug reports, rebuilt in Rust: parts written
//! as STEP and read back, as a user exports them to another system and
//! imports them again.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::{Primitive, PrimitiveArgs};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, Output};
use geop_ops_model::datums::{origin_axis, origin_plane};
use geop_ops_model::sketch::{
    Sketch, SketchPlane,
    figures::{on_origin, polygon},
    whole_sketch,
};
use geop_ops_model::spacing::Spacing;
use geop_ops_model::{
    Combine, NamedModel, Namer,
    testing::{Built, assert_valid},
};
use geop_ops_pattern::{
    Direction, LinearPattern, LinearPatternArgs, Mirror, MirrorArgs, PatternDirection, Seeds, body,
};
use geop_ops_sketch::AddSketchArgs;

use crate::{add_bodies, read_step, write_step};

/// The operations these programs are written in: given solids, sketches
/// and extrusions, patterned and mirrored.
fn registry() -> GeopResult<OperationRegistry> {
    geop_ops_extrude_revolve::testing::registry()?
        .with(Primitive)?
        .with(LinearPattern)?
        .with(Mirror)
}

/// `part` written as STEP and read back into a valid model with as many
/// solids, each of as many faces and the same volume.
fn round_trip(part: &NamedModel) {
    assert_valid(part);
    let text = write_step(part, "part").unwrap();
    let bodies = read_step(&text).unwrap_or_else(|e| panic!("reading failed: {e}"));
    let mut back = NamedModel::new();
    add_bodies(&mut back, &Namer::new("import", "i").unwrap(), &bodies)
        .unwrap_or_else(|e| panic!("adding failed: {e}"));
    assert_valid(&back);
    let summary = |m: &NamedModel| -> Vec<(usize, f64)> {
        let model = m.topology();
        let mut solids: Vec<(usize, f64)> = (m.solid_names().iter())
            .map(|name| {
                let solid = m.solid_id(name).unwrap();
                let faces = model.solid_faces(solid).unwrap().len();
                (faces, model.mass_properties(solid).unwrap().volume.to_f64())
            })
            .collect();
        solids.sort_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)));
        solids
    };
    let (before, after) = (summary(part), summary(&back));
    assert_eq!(before.len(), after.len(), "{before:?} vs {after:?}");
    for (a, b) in before.iter().zip(&after) {
        assert_eq!(a.0, b.0, "face counts {before:?} vs {after:?}");
        assert!(
            (a.1 - b.1).abs() < 1e-6 * a.1.abs().max(1.0),
            "volumes {before:?} vs {after:?}"
        );
    }
}

/// `part` with the topology it does not need taken out of each solid:
/// faces on one surface made one (see
/// [`NamedModel::merge_faces_on_one_surface`]).
fn merged(part: &NamedModel) -> NamedModel {
    let mut part = part.clone();
    for name in part.solid_names() {
        let solid = part.solid_id(&name).unwrap();
        part.merge_faces_on_one_surface(solid.into()).unwrap();
    }
    assert_valid(&part);
    part
}

/// A row of four unit blocks 0.6 apart, each overlapping the next, joined
/// into one bar by a linear pattern of a body (the SolidWorks "linear
/// pattern with merge" tutorial).
fn four_overlapping_blocks() -> NamedModel {
    let mut program = Program::new();
    program.push("a", PrimitiveArgs::block([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]));
    program.push(
        "row",
        LinearPatternArgs {
            seeds: Seeds::bodies(vec![body("block(a)")], Combine::join("block(a)")),
            first: Direction {
                along: Some(PatternDirection::Datum(origin_axis(FrameAxis::X))),
                reversed: false,
                count: 4.0.into(),
                spacing: Spacing::step(0.6),
            },
            second: None,
        },
    );
    program.build_model(registry).unwrap()
}

/// The bar of [`four_overlapping_blocks`], exported as STEP and imported
/// again.
///
/// The pattern builds a valid bar of volume 2.8. Read back from STEP, the
/// bar fails `assert_valid` with "curve_curve_crossings: exhausted
/// max_nodes=5000 with 2019 pairs pending; the result would be
/// incomplete", four times, naming no entity. The same four blocks joined
/// one by one with four join steps — the same solid, as far as volume and
/// face count tell — round-trip cleanly, as do patterns of two and three
/// copies; and so does the bar with its faces on one surface merged
/// ([`four_overlapping_blocks_merged_round_trip`]): the union leaves many
/// coplanar faces, and those are what the import cannot check.
#[test]
#[ignore = "bug: a linear pattern of 4 overlapping unit blocks joined round-trips through STEP into a body validation cannot check (curve_curve_crossings exhausted, no entity named); 3 copies, the 4 blocks joined step by step, and the bar with its coplanar faces merged round-trip cleanly"]
fn pattern_of_four_overlapping_blocks_joined_round_trips() {
    round_trip(&four_overlapping_blocks());
}

/// The bar, its faces on one surface merged first, round-trips.
#[test]
fn four_overlapping_blocks_merged_round_trip() {
    round_trip(&merged(&four_overlapping_blocks()));
}

/// An L bracket (the Onshape part tutorial): an L profile sketched on the
/// X plane and extruded 20, a slot cut into the upright's outer face, and
/// the half mirrored across that face and joined into the whole bracket.
fn mirrored_l_bracket() -> NamedModel {
    let mut program = Program::new();
    let mut profile = Sketch::new();
    polygon(
        &mut profile,
        &[
            [0.0, 0.0],
            [60.0, 0.0],
            [60.0, 8.0],
            [8.0, 8.0],
            [8.0, 50.0],
            [0.0, 50.0],
        ],
    );
    program.push(
        "profile",
        AddSketchArgs::on(on_origin(FrameAxis::X), profile),
    );
    program.push(
        "half",
        ExtrudeArgs {
            sketch: whole_sketch("profile"),
            extent: Extents::blind(20.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    // A slot 3 deep into the upright's outer face `y = 0`, 10 along `x`
    // and 20 up `z`.
    program.push(
        "slot",
        PrimitiveArgs::block([2.0, -3.0, 20.0], [12.0, 3.0, 40.0]).cut("extrude(half)"),
    );
    program.push(
        "whole",
        MirrorArgs {
            seeds: Seeds::bodies(vec![body("block(slot)")], Combine::join("block(slot)")),
            plane: Some(SketchPlane::Datum(origin_plane(FrameAxis::Y))),
        },
    );
    program.build_model(registry).unwrap()
}

/// The bracket of [`mirrored_l_bracket`], exported as STEP and imported
/// again.
///
/// The bracket builds valid. Read back from STEP, it fails `assert_valid`
/// with "curve_curve_crossings: exhausted max_nodes=5000 with 2029 pairs
/// pending; the result would be incomplete", naming no entity. Without the
/// slot the mirrored L round-trips cleanly; so does a plain block notched
/// the same way and mirrored across the notched face, and the bracket with
/// its faces on one surface merged ([`mirrored_l_bracket_merged_round_trips`]).
#[test]
#[ignore = "bug: a mirrored L bracket with a slot across the mirror face round-trips through STEP into a body validation cannot check (curve_curve_crossings exhausted, no entity named); without the slot, or with its coplanar faces merged, it round-trips cleanly"]
fn mirrored_l_bracket_with_a_slot_round_trips() {
    round_trip(&mirrored_l_bracket());
}

/// The bracket, its faces on one surface merged first, round-trips.
#[test]
fn mirrored_l_bracket_merged_round_trips() {
    round_trip(&merged(&mirrored_l_bracket()));
}
