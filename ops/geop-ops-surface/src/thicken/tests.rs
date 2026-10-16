//! Thickening a flat boundary surface on either side or both, and the
//! round sides of a cylinder into a tube.

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::Body;
use geop_lib_sweep::shapes::{block, revolved_cylinder};
use geop_ops_model::NamedModel;
use geop_ops_model::testing::apply_to_model;

use super::*;
use crate::boundary::tests::{edge_between, refused};
use crate::test_refs::*;
use crate::{BoundarySurface, BoundarySurfaceArgs};
use geop_lib_sweep::shapes::v3;
use geop_ops_model::testing::assert_valid_sheets;

/// A unit cube's top loop filled with a boundary surface, the cube then
/// deleted: a flat square sheet at `z = 1`, named `boundary(b)`, and
/// which way its normal points along `z`.
pub(crate) fn square_sheet() -> (NamedModel<S>, f64) {
    let mut part = NamedModel::<S>::new();
    let cube = block(&mut part, "c", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let corners = [[0., 0., 1.], [1., 0., 1.], [1., 1., 1.], [0., 1., 1.]];
    let edges = (0..4)
        .map(|k| edge_curve(edge_between(&part, corners[k], corners[(k + 1) % 4])))
        .collect();
    let args = BoundarySurfaceArgs {
        edges,
        tangent: Vec::new(),
    };
    let mut part = apply_to_model::<BoundarySurface>(part, "b", &args).unwrap();
    part.assemble_solid(&[Body::Solid(cube)], &[], "").unwrap();
    let face = part.face_id("boundary(b)").unwrap();
    let surface = &part.topology().get_face(face).unwrap().surface;
    let up = surface.as_plane().unwrap().unwrap().normal[2].to_f64();
    (part, up.signum())
}

fn thickened(part: NamedModel<S>, face: &str, side: ThickenSide) -> GeopResult<NamedModel<S>> {
    let args = ThickenArgs {
        face: named(face),
        thickness: 0.1,
        side,
    };
    apply_to_model::<Thicken>(part, "t", &args)
}

/// The heights of the vertices of `part`, sorted, each once.
fn heights(part: &NamedModel<S>) -> Vec<f64> {
    let mut z: Vec<f64> = part
        .topology()
        .vertices
        .values()
        .map(|v| (v.point[2].to_f64() * 1e9).round() / 1e9)
        .collect();
    z.sort_by(f64::total_cmp);
    z.dedup();
    z
}

/// A flat boundary surface thickened behind its normal, along it, and on
/// both sides: a slab a tenth thick where each says.
#[test]
fn thicken_a_boundary_surface() {
    for (side, offsets) in [
        (ThickenSide::Against, [-0.1, 0.0]),
        (ThickenSide::Along, [0.0, 0.1]),
        (ThickenSide::Both, [-0.05, 0.05]),
    ] {
        let (part, up) = square_sheet();
        let part = thickened(part, "boundary(b)", side).unwrap();
        assert_valid_sheets(&part);
        let solid = part.solid_id("thicken(t)").unwrap();
        assert_eq!(part.topology().solid_faces(solid).unwrap().len(), 6);
        assert!(part.sheet_face_names().is_empty(), "{side:?}");
        let mut want: Vec<f64> = offsets.iter().map(|d| 1.0 + d * up).collect();
        want.sort_by(f64::total_cmp);
        assert_eq!(heights(&part), want, "{side:?}");
    }
}

/// The round sides of a cylinder, copied into a sheet and thickened behind
/// their normals, which point out: a tube, its walls rings.
#[test]
fn thicken_a_cylinders_sides_into_a_tube() {
    let mut part = NamedModel::<S>::new();
    let cylinder = revolved_cylinder(
        &mut part,
        "cyl",
        v3(0., 0., 0.),
        S::from_f64(1.0),
        S::from_f64(2.0),
    )
    .unwrap();
    let sides: Vec<_> = part
        .topology()
        .solid_faces(cylinder)
        .unwrap()
        .into_iter()
        .filter(|&f| {
            let surface = &part.topology().get_face(f).unwrap().surface;
            surface.as_plane().unwrap().is_none()
        })
        .collect();
    let built = part
        .copy_faces(&sides, None, |name| format!("copy({name})"))
        .unwrap();
    part.assemble_solid(&[Body::Solid(cylinder)], &[], "")
        .unwrap();
    let face = part.name_of(built.faces[0]).unwrap().to_string();
    let part = thickened(part, &face, ThickenSide::Against).unwrap();
    assert_valid_sheets(&part);
    let radii: Vec<f64> = part
        .topology()
        .vertices
        .values()
        .map(|v| (v.point[0].to_f64().hypot(v.point[1].to_f64()) * 1e9).round() / 1e9)
        .collect();
    assert!(radii.iter().all(|&r| r == 1.0 || r == 0.9), "{radii:?}");
}

/// A solid's face is not a sheet to thicken.
#[test]
fn a_solids_face_is_refused() {
    let mut part = NamedModel::<S>::new();
    let cube = block(&mut part, "c", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
    let face = part.topology().solid_faces(cube).unwrap()[0];
    let face = part.name_of(face).unwrap().to_string();
    let err = refused(thickened(part, &face, ThickenSide::Against));
    assert!(err.contains("not one standing on its own"), "{err}");
}
