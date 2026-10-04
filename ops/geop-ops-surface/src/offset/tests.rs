//! Offsetting the round sides of a cylinder and a cube's faces.

use geop_core_math::scalars::{Field, Ring, ScalInF64 as S, Scalar};
use geop_ops::{NoFiles, Part};
use geop_ops_extrude_revolve::shapes::{cube_solid, cylinder::revolved_cylinder};

use super::*;
use crate::boundary::tests::{assert_valid, refused, v3};

fn offset(part: Part<S>, faces: Vec<String>, distance: f64) -> GeopResult<Part<S>> {
    let args = OffsetSurfaceArgs { faces, distance };
    OffsetSurface.apply(part, "o", &args, &NoFiles)
}

/// The round sides of a cylinder of radius one, offset by 0.2: a sheet of
/// radius 1.2 beside the cylinder, which stays as it was.
#[test]
fn offset_a_cylinder_face() {
    let mut part = Part::<S>::new();
    let cylinder = revolved_cylinder(
        &mut part,
        "cyl",
        v3(0., 0., 0.),
        S::from_f64(1.0),
        S::from_f64(2.0),
    )
    .unwrap();
    let sides: Vec<String> = part
        .topology()
        .solid_faces(cylinder)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let surface = &part.topology().get_face(f).unwrap().surface;
            surface.as_plane().unwrap().is_none()
        })
        .map(|f| part.name_of(f).unwrap().to_string())
        .collect();
    let faces_before = part.topology().faces.len();
    let part = offset(part, sides.clone(), 0.2).unwrap();
    assert_valid(&part);
    assert_eq!(part.topology().faces.len(), faces_before + sides.len());
    for name in &sides {
        let copy = part.face_id(&format!("offset(o,{name})")).unwrap();
        let surface = &part.topology().get_face(copy).unwrap().surface;
        let (u0, u1) = surface.domain_u();
        let (v0, v1) = surface.domain_v();
        let p = surface
            .evaluate(
                u0.add(u1).div(S::TWO).unwrap(),
                v0.add(v1).div(S::TWO).unwrap(),
            )
            .unwrap();
        let r = p[0].mul(p[0]).add(p[1].mul(p[1])).sqrt().unwrap();
        assert!(r.could_be_equal(S::from_f64(1.2)), "{name}: {r:?}");
    }
}

/// Three faces of a cube meeting at a corner, offset inward by a tenth:
/// their copies meet where the offset planes do.
#[test]
fn offset_three_faces_of_a_cube_inward() {
    let mut part = Part::<S>::new();
    let cube = cube_solid(&mut part, "c", v3(0., 0., 0.), v3(1., 1., 1.)).unwrap();
    let faces: Vec<String> = part
        .topology()
        .solid_faces(cube)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            // The faces through the corner at the origin.
            let surface = &part.topology().get_face(f).unwrap().surface;
            let plane = surface.as_plane().unwrap().unwrap();
            plane
                .signed_distance(&v3(0., 0., 0.))
                .could_be_equal(S::ZERO)
        })
        .map(|f| part.name_of(f).unwrap().to_string())
        .collect();
    assert_eq!(faces.len(), 3);
    let part = offset(part, faces, -0.1).unwrap();
    assert_valid(&part);
    let corner = part
        .topology()
        .vertices
        .values()
        .any(|v| v.point.could_be_equal(&v3(0.1, 0.1, 0.1)));
    assert!(corner);
}

/// Offsetting by nothing is refused.
#[test]
fn zero_distance_is_refused() {
    let mut part = Part::<S>::new();
    let cube = cube_solid(&mut part, "c", v3(0., 0., 0.), v3(1., 1., 1.)).unwrap();
    let face = part.topology().solid_faces(cube).unwrap()[0];
    let face = part.name_of(face).unwrap().to_string();
    let err = refused(offset(part, vec![face], 0.0));
    assert!(err.contains("other than zero"), "{err}");
}
