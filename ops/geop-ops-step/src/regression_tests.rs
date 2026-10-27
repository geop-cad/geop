//! Programs from tutorials and bug reports, rebuilt in Rust: parts written
//! as STEP and read back, as a user exports them to another system and
//! imports them again.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::element::ElementReference;
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::{Primitive, PrimitiveArgs};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, Output};
use geop_ops_fillet::{Fillet, FilletArgs};
use geop_ops_model::datums::{origin_axis, origin_plane};
use geop_ops_model::sketch::{
    Sketch, SketchPlane,
    figures::{on_origin, polygon},
    whole_sketch,
};
use geop_ops_model::spacing::Spacing;
use geop_ops_model::{
    Combine, NamedModel, Namer, model,
    testing::{Built, assert_valid, edges_with_middle_where, face_towards},
};
use geop_ops_pattern::{
    Direction, LinearPattern, LinearPatternArgs, Mirror, MirrorArgs, PatternDirection, Seeds, body,
};
use geop_ops_shell::{Shell, ShellArgs};
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
        part.merge_faces_on_one_surface(solid.into(), |_, _| true)
            .unwrap();
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

/// The operations of [`registry`], and the fillet and shell.
fn finishing_registry() -> GeopResult<OperationRegistry> {
    registry()?.with(Fillet)?.with(Shell)
}

/// A 40 x 20 x 10 block with its upright edge at the origin rounded by 3
/// (the first fillet of every tutorial), exported as STEP and imported
/// again.
///
/// The rounded block builds valid. Read back, it fails `assert_valid`
/// twice, on the two planar sides the blend is tangent to:
///
/// > coedge 28's pcurve sample 0 (t=0.000) maps through its face's surface
/// > to a point that is not on edge 11's curve at all (coedge on face 27,
/// > (u, v) = (0, 0.15), point (0, 3, ..))
///
/// and the same at `(3, 0, ..)` on the side `y = 0`: the start of each
/// side's pcurve along the edge where the blend leaves it. The edges are
/// `import(.., e0)` and `import(.., e13)`, the blend's tangent lines, and
/// validation finds every sample of them off the side they lie in: the
/// line read back for the side `x = 0` runs at
/// `x = [-8.9e-16, -6.7e-80]`, definitely off the plane by a hair, and the
/// import did not widen it to reach the plane. A block with a rounded
/// rectangle extruded as its outline, the same solid with arcs instead of
/// a blend, round-trips cleanly.
#[test]
#[ignore = "bug: a block with one upright edge rounded round-trips through STEP into an invalid body: the planar sides' pcurves start at the blend's tangent vertices (0,3,z)/(3,0,z), not on the imported tangent edges' curves; a rounded rectangle extruded round-trips"]
fn block_with_one_rounded_edge_round_trips() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [40.0, 20.0, 10.0]),
    );
    let before = program.build_model(finishing_registry).unwrap();
    let edge = edges_with_middle_where(&before, |m| {
        m[0].abs() < 1e-9 && m[1].abs() < 1e-9 && (m[2] - 5.0).abs() < 1e-9
    });
    assert_eq!(edge.len(), 1, "{edge:?}");
    program.push("round", FilletArgs::constant(edge, 3.0));
    round_trip(&program.build_model(finishing_registry).unwrap());
}

/// A cup: a cylinder of radius 5 and height 10 shelled 1 thick, open at
/// its top (the Onshape "cup" tutorial), exported as STEP and imported
/// again.
///
/// The cup builds valid. Reading it back fails at once:
///
/// > #77: its knots decrease
/// > reading the edge #78 / reading the face #125 / reading the body
/// > (shell(sh))
///
/// `#77` is the curve of `shell(sh,extrude(..,end))`, a quarter of the
/// inner rim, a rational quadratic clamped at its start by three knots
/// near zero. They are written as `(2,1,3)` multiplicities of the knots
/// `(7.216449660063519E-16, 7.216449660062508E-16, 1.0)`: the first three
/// knots are three enclosures of one value, two of them equal and the
/// third not, so `knots_and_multiplicities` writes them as two distinct
/// knots whose midpoints decrease. The block shelled the same way
/// round-trips cleanly, and so does the cylinder itself
/// ([`crate::tests`]).
#[test]
#[ignore = "bug: a cylinder shelled open at the top is written to STEP with an edge curve whose knots decrease (#77), so reading it back fails; a shelled block and the cylinder itself round-trip"]
fn cup_round_trips() {
    let mut program = Program::new();
    program.push("rod", PrimitiveArgs::cylinder_z([0.0, 0.0, 0.0], 5.0, 10.0));
    let before = program.build_model(finishing_registry).unwrap();
    let solid = before.solid_names()[0].clone();
    let top = face_towards(&before, before.solid_id(&solid).unwrap(), [0.0, 0.0, 1.0]);
    program.push(
        "sh",
        ShellArgs {
            solid: geop_ops_model::solid(&solid),
            faces: vec![ElementReference::new(
                model(),
                before.name_of(top).unwrap().to_string(),
            )],
            thickness: 1.0.into(),
            other_faces: Vec::new(),
            other_thickness: 0.2.into(),
            outward: false,
        },
    );
    round_trip(&program.build_model(finishing_registry).unwrap());
}

/// A 40 x 20 x 10 block with a round pocket of radius 10 milled 5 deep
/// from its top, the cutter as wide as the block: the pocket's wall
/// touches the block's two long sides along a line each (a "cylinder
/// tangent to a wall"). It builds valid, and exported as STEP it cannot be
/// read back:
///
/// > the edge e6 (#69) it runs along twice is used by another face too
///
/// (`e5 (#63)` for the same pocket sketched and extruded.) The pocket's
/// wall meets each side in a line; as written, one face's loop runs along
/// that edge twice and another face uses it too, which the reader refuses,
/// though the kernel built and validated the part. A pocket of radius 5
/// in the block's corner, touching two sides, fails alike.
#[test]
#[ignore = "bug: a block with a round pocket tangent to its sides round-trips through STEP into \"the edge e6 (#69) it runs along twice is used by another face too\"; the kernel's own model is valid"]
fn round_pocket_tangent_to_the_sides_round_trips() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [40.0, 20.0, 10.0]),
    );
    program.push(
        "pocket",
        PrimitiveArgs::cylinder_z([20.0, 10.0, 5.0], 10.0, 10.0).cut("block(box)"),
    );
    round_trip(&program.build_model(registry).unwrap());
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
