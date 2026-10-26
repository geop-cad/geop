//! Every face of boxes, cylinders and spheres — whole, drilled, pocketed,
//! stepped, cut off, with a boss (see `geop_lib_fixtures::bodies`) —
//! pushed in and pulled out ([`OffsetFace`]), shifted ([`MoveFace`]) and
//! deleted, healed over and left open ([`DeleteFace`]).
//! An edit may refuse what it does not support, saying so; what it builds
//! has to be a valid solid.
//!
//! Each takes a fraction of a second: they all run with every `cargo test`.

use geop_core_program::element::ElementReference;
use geop_core_program::program::Program;
use geop_lib_fixtures::bodies::{
    block, block_corner_cut_off, block_drilled, block_pocketed, block_stepped, block_with_boss,
    cylinder, cylinder_drilled, cylinder_pocketed, cylinder_with_boss, cylinder_with_flat,
    hemisphere, one_solid, sphere_drilled,
};
use geop_ops_model::{
    element, model,
    testing::{Cases, outcome},
};

use crate::{
    DeleteFace, DeleteFaceArgs, HealGap, MergeFaces, MergeFacesArgs, MoveFace, MoveFaceArgs,
    OffsetFace, OffsetFaceArgs,
};

/// Whether an error's root message is an edit refusing what it does not
/// support, or a face moved so far that the solid would change its
/// topology, rather than failing at what it does.
fn is_refusal(message: &str) -> bool {
    [
        "not supported",
        "turns around",
        "would cross",
        "only planes and surfaces of revolution",
        "not determined",
        "no longer reaches",
        "the other way round",
        "passes through itself",
        "past its",
        "neither pair of its opposite sides meet",
        "part of no feature",
    ]
    .iter()
    .any(|refusal| message.contains(refusal))
}

/// Each face of the solid `program` builds offset by `distance` either way
/// and shifted by `distance` along each axis; panics listing every case
/// gone wrong.
fn stress(program: Program, distance: f64) {
    let (part, solid) = one_solid(&program);
    let mut faces: Vec<String> = (part.topology().faces.keys())
        .map(|&id| part.name_of(id).unwrap().to_string())
        .collect();
    faces.sort();
    let mut cases = Cases::default();
    let merge = MergeFacesArgs {
        bodies: vec![ElementReference::new(model(), &solid)],
    };
    cases.record(
        "faces merged".into(),
        outcome::<MergeFaces>(&part, &merge, is_refusal),
    );
    for face in faces {
        for d in [distance, -distance] {
            let args = OffsetFaceArgs {
                faces: vec![element(&face)],
                distance: d,
            };
            let case = format!("{face} offset by {d}");
            cases.record(case, outcome::<OffsetFace>(&part, &args, is_refusal));
        }
        for (heal, whole_feature) in [
            (HealGap::Heal, false),
            (HealGap::Open, false),
            (HealGap::Heal, true),
        ] {
            let args = DeleteFaceArgs {
                faces: vec![element(&face)],
                heal,
                whole_feature,
            };
            let case = format!("{face} deleted, {heal:?}, whole feature: {whole_feature}");
            cases.record(case, outcome::<DeleteFace>(&part, &args, is_refusal));
        }
        for k in 0..3 {
            let mut translation = [0.0; 3];
            translation[k] = distance;
            let args = MoveFaceArgs {
                faces: vec![element(&face)],
                translation,
            };
            let case = format!("{face} moved by {translation:?}");
            cases.record(case, outcome::<MoveFace>(&part, &args, is_refusal));
        }
    }
    cases.assert_none_wrong();
}

#[test]
fn stress_block() {
    stress(block(), 0.1);
}

#[test]
fn stress_block_stepped() {
    stress(block_stepped(), 0.1);
}

#[test]
fn stress_block_drilled() {
    stress(block_drilled(), 0.1);
}

#[test]
fn stress_block_corner_cut_off() {
    stress(block_corner_cut_off(), 0.1);
}

#[test]
fn stress_block_pocketed() {
    stress(block_pocketed(), 0.1);
}

#[test]
fn stress_block_with_boss() {
    stress(block_with_boss(), 0.1);
}

#[test]
fn stress_cylinder() {
    stress(cylinder(), 0.1);
}

#[test]
fn stress_cylinder_with_flat() {
    stress(cylinder_with_flat(), 0.1);
}

#[test]
fn stress_cylinder_drilled() {
    stress(cylinder_drilled(), 0.1);
}

#[test]
fn stress_cylinder_pocketed() {
    stress(cylinder_pocketed(), 0.1);
}

#[test]
fn stress_cylinder_with_boss() {
    stress(cylinder_with_boss(), 0.1);
}

#[test]
fn stress_hemisphere() {
    stress(hemisphere(), 0.1);
}

#[test]
fn stress_sphere_drilled() {
    stress(sphere_drilled(), 0.1);
}
