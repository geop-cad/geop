//! Knitting copies of a cube's faces back into a box, and sheets that do
//! not close.

use geop_core_math::scalars::Scalar;
use geop_core_topology::Body;
use geop_lib_sweep::shapes::block;
use geop_ops_model::NamedModel;
use geop_ops_model::testing::apply_to_model;

use super::*;
use crate::boundary::tests::refused;
use crate::test_refs::*;
use geop_ops_model::testing::assert_valid_sheets;

/// The six faces of a unit cube, each copied into a sheet of its own, the
/// cube itself deleted: what is left is six faces standing on their own,
/// the `k`-th face's sheet named `f{k}(...)`, its faces' names returned.
fn six_sheets() -> (NamedModel, Vec<String>) {
    let mut part = NamedModel::new();
    let cube = block(&mut part, "c", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let faces = part.topology().solid_faces(cube).unwrap();
    let mut names = Vec::new();
    for (k, &f) in faces.iter().enumerate() {
        let built = part
            .copy_faces(&[f], None, |name| format!("f{k}({name})"))
            .unwrap();
        names.push(part.name_of(built.faces[0]).unwrap().to_string());
    }
    part.assemble_solid(&[Body::Solid(cube)], &[], "").unwrap();
    // Some of them turned around: knitting turns them back.
    for name in &names[..2] {
        let face = part.face_id(name).unwrap();
        part.reverse_face(face).unwrap();
    }
    (part, names)
}

fn knit_faces(part: NamedModel, faces: &[String], solid: bool) -> GeopResult<NamedModel> {
    let args = KnitArgs {
        faces: faces.iter().map(named).collect(),
        solid,
        gap: 0.0,
    };
    apply_to_model::<Knit>(part, "k", &args)
}

/// Six faces standing on their own, meeting edge to edge: knit into a box.
#[test]
fn six_faces_knit_into_a_box() {
    let (part, faces) = six_sheets();
    let part = knit_faces(part, &faces, true).unwrap();
    assert_valid_sheets(&part);
    let solid = part.solid_id("knit(k)").unwrap();
    assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 6);
    assert_eq!(part.topology().edges.len(), 12);
    assert_eq!(part.topology().vertices.len(), 8);
    assert!(part.sheet_face_names().is_empty());
    // The faces keep their names.
    for name in &faces {
        assert!(part.face_id(name).is_ok(), "{name}");
    }
}

/// Five of the six: an open box, a sheet — or, asked for a solid, refused
/// naming the four open edges round the missing face.
#[test]
fn five_faces_knit_into_an_open_box() {
    let (part, faces) = six_sheets();
    let err = refused(knit_faces(part.clone(), &faces[..5], true));
    assert!(err.contains("do not close up"), "{err}");
    let opened = knit_faces(part, &faces[..5], false).unwrap();
    assert_valid_sheets(&opened);
    assert!(opened.solid_names().is_empty());
    assert_eq!(opened.sheet_face_names().len(), 6 - 1 + 1);
}

/// Two faces of the cube that do not meet do not hang together.
#[test]
fn faces_that_do_not_meet_are_refused() {
    let (part, faces) = six_sheets();
    // The cube's top and bottom.
    let top_and_bottom: Vec<String> = faces
        .iter()
        .filter(|name| {
            let face = part.face_id(name).unwrap();
            let surface = &part.topology().get_face(face).unwrap().surface;
            let normal = surface.as_plane().unwrap().unwrap().normal;
            !normal[2].could_be_equal(Scalar::ZERO)
        })
        .cloned()
        .collect();
    assert_eq!(top_and_bottom.len(), 2);
    let err = refused(knit_faces(part, &top_and_bottom, false));
    assert!(err.contains("do not hang together"), "{err}");
}

/// `faces` knit, what is within `gap` of each other as one.
fn knit_across(part: NamedModel, faces: &[String], gap: f64) -> GeopResult<NamedModel> {
    let args = KnitArgs {
        faces: faces.iter().map(named).collect(),
        solid: true,
        gap,
    };
    apply_to_model::<Knit>(part, "k", &args)
}

/// The face of `solid` towards `normal`, copied into a sheet named after
/// `prefix`: its name.
fn sheet_of(
    part: &mut NamedModel,
    solid: geop_core_topology::SolidId,
    normal: [f64; 3],
    prefix: &str,
) -> String {
    let face = geop_ops_model::testing::face_towards(part, solid, normal);
    let built = part
        .copy_faces(&[face], None, |name| format!("{prefix}({name})"))
        .unwrap();
    part.name_of(built.faces[0]).unwrap().to_string()
}

/// A cube's five sides as sheets and its top as two halves: each side
/// along the halves meets both, end to end with neither — T-junctions,
/// split where the halves meet, and knit into a box of seven faces.
#[test]
fn a_top_in_two_halves_knits_across_t_junctions() {
    let mut part = NamedModel::new();
    let cube = block(&mut part, "c", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let left = block(&mut part, "l", [0.0, 0.0, 0.0], [0.5, 1.0, 1.0]);
    let right = block(&mut part, "r", [0.5, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let mut faces = Vec::new();
    for (k, normal) in [
        [1., 0., 0.],
        [-1., 0., 0.],
        [0., 1., 0.],
        [0., -1., 0.],
        [0., 0., -1.],
    ]
    .into_iter()
    .enumerate()
    {
        faces.push(sheet_of(&mut part, cube, normal, &format!("s{k}")));
    }
    faces.push(sheet_of(&mut part, left, [0., 0., 1.], "tl"));
    faces.push(sheet_of(&mut part, right, [0., 0., 1.], "tr"));
    part.delete_bodies(&[Body::Solid(cube), Body::Solid(left), Body::Solid(right)])
        .unwrap();
    let part = knit_across(part, &faces, 0.0).unwrap();
    assert_valid_sheets(&part);
    let solid = part.solid_id("knit(k)").unwrap();
    assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 7);
    assert_eq!(part.topology().vertices.len(), 10);
}

/// The six faces of a cube, one shifted a hundredth of a thousandth off
/// the others: nothing meets it, unless the gap it is off by is allowed —
/// then its corners are moved to where the surfaces meet, and the box is
/// as sharp as the planes agree.
#[test]
fn a_face_off_by_a_gap_knits_across_it() {
    let (mut part, faces) = six_sheets();
    let face = part.face_id(&faces[2]).unwrap();
    let sheet = part.topology().get_face(face).unwrap().shell;
    let shift = geop_core_math::primitives::Motion::translation(geop_lib_sweep::shapes::v3(
        2e-5, 1e-5, 0.0,
    ));
    part.transform_body(Body::Sheet(sheet), &shift).unwrap();
    let err = refused(knit_across(part.clone(), &faces, 0.0));
    assert!(err.contains("do not close up"), "{err}");
    let part = knit_across(part, &faces, 1e-4).unwrap();
    assert_valid_sheets(&part);
    let solid = part.solid_id("knit(k)").unwrap();
    assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 6);
    for vertex in part.topology().vertices.values() {
        for k in 0..3 {
            assert!(
                vertex.point[k].width().to_f64() < 1e-9,
                "{:?}",
                vertex.point
            );
        }
    }
}
