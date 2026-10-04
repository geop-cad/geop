//! Knitting copies of a cube's faces back into a box, and sheets that do
//! not close.

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::Body;
use geop_ops::{NoFiles, Part};
use geop_ops_extrude_revolve::shapes::cube_solid;

use super::*;
use crate::boundary::tests::{assert_valid, refused, v3};

/// The six faces of a unit cube, each copied into a sheet of its own, the
/// cube itself deleted: what is left is six faces standing on their own,
/// the `k`-th face's sheet named `f{k}(...)`, its faces' names returned.
fn six_sheets() -> (Part<S>, Vec<String>) {
    let mut part = Part::<S>::new();
    let cube = cube_solid(&mut part, "c", v3(0., 0., 0.), v3(1., 1., 1.)).unwrap();
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

fn knit_faces(part: Part<S>, faces: &[String], solid: bool) -> GeopResult<Part<S>> {
    let args = KnitArgs {
        faces: faces.to_vec(),
        solid,
    };
    Knit.apply(part, "k", &args, &NoFiles)
}

/// Six faces standing on their own, meeting edge to edge: knit into a box.
#[test]
fn six_faces_knit_into_a_box() {
    let (part, faces) = six_sheets();
    let part = knit_faces(part, &faces, true).unwrap();
    assert_valid(&part);
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
    assert_valid(&opened);
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
            !normal[2].could_be_equal(S::ZERO)
        })
        .cloned()
        .collect();
    assert_eq!(top_and_bottom.len(), 2);
    let err = refused(knit_faces(part, &top_and_bottom, false));
    assert!(err.contains("do not hang together"), "{err}");
}
