//! Faces of boxes, cylinders and drilled boxes put on new surfaces —
//! moved, offset — and the solids rebuilt around them: valid, of the
//! volume they should have, every name kept; and what cannot be rebuilt
//! refused by name.

use geop_core_math::{primitives::Motion, scalars::Scalar};
use geop_core_topology::{FaceId, SolidId};
use geop_lib_booleans::{RemeshParams, boolean};
use geop_lib_sweep::shapes::{block, cylinder, v3};
use geop_ops_model::{
    BooleanOp, NamedModel, Namer,
    testing::{assert_valid, face_towards, volume},
};

use super::{Smooth, replace_faces};

/// How closely [`volume`] measures a solid whose faces are flat.
const FLAT: f64 = 1e-5;

/// How closely it measures one with curved faces, drawn in triangles.
const CURVED: f64 = 2e-2;

fn namer() -> Namer {
    Namer::new("move", "m").unwrap()
}

/// The 2 x 2 x 1 box from the origin.
fn unit_box() -> (NamedModel, SolidId) {
    let mut part = NamedModel::new();
    let solid = block(&mut part, "b", [0., 0., 0.], [2., 2., 1.]);
    (part, solid)
}

/// The box with a blind hole of radius 0.4 from its top, 0.5 deep.
fn drilled_box() -> (NamedModel, SolidId) {
    let (mut part, body) = unit_box();
    let drill = cylinder(&mut part, "h", [1., 1., 0.5], 0.4, 0.5);
    let namer = Namer::new("drilled", "d").unwrap();
    let solid = boolean(
        &mut part,
        &namer,
        body,
        drill,
        BooleanOp::Difference,
        RemeshParams::default(),
    )
    .unwrap()
    .expect("a drilled box");
    assert_valid(&part);
    (part, solid)
}

/// The faces `faces` of `part` moved by `motion`.
fn moved(
    mut part: NamedModel,
    faces: &[FaceId],
    motion: &Motion,
) -> Result<(NamedModel, SolidId), String> {
    let solid = replace_faces(&mut part, &namer(), faces, Smooth::Follow, |_, s| {
        Ok(s.transform(motion))
    })
    .map_err(|e| format!("{e}"))?;
    Ok((part, solid))
}

fn up(by: f64) -> Motion {
    Motion::translation(v3(0., 0., by))
}

/// A box's top pulled up by a half: half as much again, every name kept.
#[test]
fn box_top_pulled_up() {
    let (part, solid) = unit_box();
    let top = face_towards(&part, solid, [0., 0., 1.]);
    let names: Vec<String> = part.names().iter().map(|(_, n)| n.to_string()).collect();
    let (part, solid) = moved(part, &[top], &up(0.5)).unwrap();
    assert_valid(&part);
    assert!((volume(&part, solid, 16) - 6.0).abs() < FLAT);
    for name in names.iter().filter(|n| !n.starts_with("cube(")) {
        assert!(part.id_of(name).is_some(), "{name} is gone");
    }
    assert_eq!(part.name_of(solid), Some("move(m)"));
}

/// A box's side pushed in and turned a little: a wedge of a box.
#[test]
fn box_side_pushed_in_and_turned() {
    let (part, solid) = unit_box();
    let side = face_towards(&part, solid, [1., 0., 0.]);
    let turn = geop_core_math::primitives::Pose::rotation_about(
        &v3(1.5, 0., 0.),
        &v3(0., 1., 0.),
        Scalar::from_f64(0.1),
    )
    .unwrap()
    .motion();
    let push = Motion::translation(v3(-0.5, 0., 0.));
    let (part, _) = moved(part, &[side], &push).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("move(m)").unwrap();
    assert!((volume(&part, solid, 16) - 3.0).abs() < FLAT);
    let side = face_towards(&part, solid, [1., 0., 0.]);
    let (part, _) = moved(part, &[side], &turn).unwrap();
    assert_valid(&part);
}

/// A box's top pushed down through its bottom: the sides would vanish,
/// and it is refused, naming the edge that turns around.
#[test]
fn box_top_pushed_through_is_refused() {
    let (part, solid) = unit_box();
    let top = face_towards(&part, solid, [0., 0., 1.]);
    let error = moved(part, &[top], &up(-1.5)).unwrap_err();
    assert!(error.contains("turns around"), "{error}");
}

/// The top of a box drilled from the top pulled up: the hole's wall grows
/// along its seams with it.
#[test]
fn drilled_box_top_pulled_up() {
    let (part, solid) = drilled_box();
    let before = volume(&part, solid, 64);
    let top = face_towards(&part, solid, [0., 0., 1.]);
    let (part, solid) = moved(part, &[top], &up(0.25)).unwrap();
    assert_valid(&part);
    let hole = std::f64::consts::PI * 0.4 * 0.4;
    let grown = 0.25 * (4.0 - hole);
    let after = volume(&part, solid, 64);
    assert!(
        (after - before - grown).abs() < CURVED,
        "{before} -> {after}"
    );
}

/// A cylinder's wall offset outward: a wider cylinder, its caps grown
/// with it.
#[test]
fn cylinder_wall_offset_outward() {
    let mut part = NamedModel::new();
    let solid = cylinder(&mut part, "c", [0., 0., 0.], 1.0, 1.0);
    let walls: Vec<FaceId> = part
        .topology()
        .solid_faces(solid)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            part.topology()
                .get_face(f)
                .unwrap()
                .surface
                .as_plane()
                .unwrap()
                .is_none()
        })
        .collect();
    let mut part = part;
    let solid = replace_faces(&mut part, &namer(), &walls, Smooth::Follow, |_, s| {
        Ok(s.offset(Scalar::from_f64(0.5))?
            .expect("a cylinder grown has an offset"))
    })
    .unwrap();
    assert_valid(&part);
    let wider = std::f64::consts::PI * 1.5 * 1.5;
    let after = volume(&part, solid, 64);
    assert!((after - wider).abs() < CURVED, "{after} vs {wider}");
}
