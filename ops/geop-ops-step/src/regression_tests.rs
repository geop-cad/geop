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

/// A row of four unit blocks 0.6 apart, each overlapping the next, joined
/// into one bar by a linear pattern of a body (the SolidWorks "linear
/// pattern with merge" tutorial), exported as STEP and imported again.
///
/// The pattern builds a valid bar of volume 2.8. Read back from STEP, the
/// bar fails `assert_valid` with "curve_curve_crossings: exhausted
/// max_nodes=5000 with 2019 pairs pending; the result would be
/// incomplete", four times, naming no entity. The same four blocks joined
/// one by one with four join steps — the same solid, as far as volume and
/// face count tell — round-trip cleanly, as do patterns of two and three
/// copies.
#[test]
#[ignore = "bug: a linear pattern of 4 overlapping unit blocks joined round-trips through STEP into a body validation cannot check (curve_curve_crossings exhausted, no entity named); 3 copies, and the 4 blocks joined step by step, round-trip cleanly"]
fn pattern_of_four_overlapping_blocks_joined_round_trips() {
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
    let part = program.build_model(registry).unwrap();
    round_trip(&part);
}

/// An L bracket (the Onshape part tutorial): an L profile sketched on the
/// X plane and extruded 20, a slot cut into the upright's outer face, and
/// the half mirrored across that face and joined into the whole bracket —
/// exported as STEP and imported again.
///
/// The bracket builds valid. Read back from STEP, it fails `assert_valid`
/// with "curve_curve_crossings: exhausted max_nodes=5000 with 2029 pairs
/// pending; the result would be incomplete", naming no entity. Without the
/// slot the mirrored L round-trips cleanly; so does a plain block notched
/// the same way and mirrored across the notched face.
#[test]
#[ignore = "bug: a mirrored L bracket with a slot across the mirror face round-trips through STEP into a body validation cannot check (curve_curve_crossings exhausted, no entity named); without the slot it round-trips cleanly"]
fn mirrored_l_bracket_with_a_slot_round_trips() {
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
            draft: 0.0,
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
    let part = program.build_model(registry).unwrap();
    round_trip(&part);
}

/// A bar cut in two by a slab across its middle is one solid of two lumps,
/// and a block with a ball's room left inside it one lump with a void. Each
/// lump is written as a brep of its own, with the voids inside it, and
/// read back as a solid: the bar as two solids of six faces, together the
/// bar's volume, the block as one of two shells.
#[test]
fn lumps_and_voids_are_written_as_breps() {
    let mut program = Program::new();
    program.push(
        "bar",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [4.0, 1.0, 1.0]),
    );
    program.push(
        "slab",
        PrimitiveArgs::block([1.5, -1.0, -1.0], [2.5, 2.0, 2.0]).cut("block(bar)"),
    );
    program.push(
        "cube",
        PrimitiveArgs::block([10.0, 0.0, 0.0], [14.0, 4.0, 4.0]),
    );
    program.push(
        "ball",
        PrimitiveArgs::sphere([12.0, 2.0, 2.0], 1.0).cut("block(cube)"),
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let text = write_step(&part, "part").unwrap();
    let bodies = read_step(&text).unwrap_or_else(|e| panic!("reading failed: {e}"));
    let mut back = NamedModel::new();
    add_bodies(&mut back, &Namer::new("import", "i").unwrap(), &bodies)
        .unwrap_or_else(|e| panic!("adding failed: {e}"));
    assert_valid(&back);
    let model = back.topology();
    let mut solids: Vec<(usize, usize, f64)> = (back.solid_names().iter())
        .map(|name| {
            let solid = back.solid_id(name).unwrap();
            (
                model.get_solid(solid).unwrap().shells.len(),
                model.solid_faces(solid).unwrap().len(),
                model.mass_properties(solid).unwrap().volume.to_f64(),
            )
        })
        .collect();
    solids.sort_by(|a, b| a.0.cmp(&b.0).then(a.2.total_cmp(&b.2)));
    let ball = 4.0 / 3.0 * std::f64::consts::PI;
    assert_eq!(solids.len(), 3, "{solids:?}");
    let (bars, block) = solids.split_at(2);
    assert_eq!(
        (bars[0].0, bars[0].1, bars[1].0, bars[1].1),
        (1, 6, 1, 6),
        "{solids:?}"
    );
    assert!((bars[0].2 + bars[1].2 - 3.0).abs() < 1e-6, "{solids:?}");
    assert_eq!(block[0].0, 2, "{solids:?}");
    assert!((block[0].2 - (64.0 - ball)).abs() < 1e-4, "{solids:?}");
}
