//! Shelling the basic solids: a box closed all round, opened at one face,
//! at two side by side and at two opposite ones; a prism; a revolved
//! cylinder, whose caps are four faces each.

use geop_core_math::{
    for_all_scalars,
    scalars::{ScalInF64, Scalar},
    vector::Vector3,
};
use geop_core_topology::{
    Body, SolidId,
    validation::{ValidationParameters, validate, validate_manifold},
};
use geop_ops::{Namer, Part};
use geop_ops_extrude_revolve::shapes::{
    cube_solid,
    cylinder::{extruded_cylinder, revolved_cylinder},
    sphere::sphere_solid,
};

use super::shell;

fn v<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

fn assert_valid<S: Scalar>(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate(&params, part.topology()) {
        panic!("{errors:#?}");
    }
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        panic!("{errors:#?}");
    }
}

/// The 2 x 2 x 1 box from the origin, `cube(b)`.
fn unit_box<S: Scalar>() -> (Part<S>, SolidId) {
    let mut part = Part::new();
    let solid = cube_solid(&mut part, "b", v(0., 0., 0.), v(2., 2., 1.)).unwrap();
    (part, solid)
}

/// `part` with `solid` shelled `thickness` thick, open at the faces named
/// `open`, as the operation `s`.
fn shelled<S: Scalar>(
    mut part: Part<S>,
    solid: SolidId,
    open: &[&str],
    thickness: f64,
) -> (Part<S>, SolidId) {
    let open: Vec<_> = open
        .iter()
        .map(|name| part.face_id(name).unwrap())
        .collect();
    let namer = Namer::new("shell", "s").unwrap();
    let solid = shell(&mut part, &namer, solid, &open, S::from_f64(thickness)).unwrap();
    (part, solid)
}

fn faces<S: Scalar>(part: &Part<S>, solid: SolidId) -> usize {
    part.topology()
        .body_faces(Body::Solid(solid))
        .unwrap()
        .len()
}

/// Every vertex of `part`, sorted, rounded to what tells them apart here.
fn corners<S: Scalar>(part: &Part<S>) -> Vec<[i64; 3]> {
    let mut out: Vec<[i64; 3]> = part
        .topology()
        .vertices
        .values()
        .map(|v| [0, 1, 2].map(|k| (v.point[k].to_f64() * 1000.0).round() as i64))
        .collect();
    out.sort();
    out
}

fn check_closed_box<S: Scalar>() {
    let (part, solid) = unit_box::<S>();
    let (part, solid) = shelled(part, solid, &[], 0.25);
    assert_valid(&part);
    let model = part.topology();
    assert_eq!(model.get_solid(solid).unwrap().shells.len(), 2);
    assert_eq!(faces(&part, solid), 12);
    assert_eq!(part.name_of(solid), Some("shell(s)"));
    let inner: Vec<[i64; 3]> = corners(&part)
        .into_iter()
        .filter(|p| p.iter().all(|&x| x != 0 && x != 2000 && x != 1000))
        .collect();
    let mut expected = Vec::new();
    for x in [250, 1750] {
        for y in [250, 1750] {
            for z in [250, 750] {
                expected.push([x, y, z]);
            }
        }
    }
    assert_eq!(inner, expected);
}
#[test]
fn a_box_closed_all_round_gets_a_void() {
    for_all_scalars!(check_closed_box);
}

fn check_open_top<S: Scalar>() {
    let (part, solid) = unit_box::<S>();
    let top = "cube(b,start)";
    assert!(part.face_id(top).is_ok());
    let (part, solid) = shelled(part, solid, &[top], 0.25);
    assert_valid(&part);
    assert_eq!(part.topology().get_solid(solid).unwrap().shells.len(), 1);
    // 5 outer faces, their 5 inner copies, the rim.
    assert_eq!(faces(&part, solid), 11);
    let rim = part.face_id("shell(s,cube(b,start))").unwrap();
    assert_eq!(part.topology().get_face(rim).unwrap().holes.len(), 1);
    // The inner floor is a quarter up.
    assert!(corners(&part).contains(&[250, 250, 250]));
    assert!(corners(&part).contains(&[250, 250, 1000]));
}
#[test]
fn a_box_opened_at_its_top() {
    for_all_scalars!(check_open_top);
}

fn check_open_two_sides<S: Scalar>() {
    let (part, solid) = unit_box::<S>();
    let names: Vec<String> = part
        .topology()
        .body_faces(Body::Solid(solid))
        .unwrap()
        .into_iter()
        .map(|f| part.name_of(f).unwrap().to_string())
        .collect();
    let side = names
        .iter()
        .find(|n| n.contains(",c"))
        .expect("a side face")
        .clone();
    let (part, solid) = shelled(part, solid, &["cube(b,start)", &side], 0.25);
    assert_valid(&part);
    // 4 outer faces and their inner copies, and two rims, each one face —
    // a U, not a ring.
    assert_eq!(faces(&part, solid), 10);
    for rim in ["shell(s,cube(b,start))", &format!("shell(s,{side})")] {
        let face = part.face_id(rim).unwrap();
        assert!(part.topology().get_face(face).unwrap().holes.is_empty());
    }
}
#[test]
fn a_box_opened_at_its_top_and_a_side() {
    for_all_scalars!(check_open_two_sides);
}

fn check_tube<S: Scalar>() {
    let (part, solid) = unit_box::<S>();
    let (part, solid) = shelled(part, solid, &["cube(b,start)", "cube(b,end)"], 0.25);
    assert_valid(&part);
    // 4 walls, inside and out, and a rim at either end.
    assert_eq!(faces(&part, solid), 10);
}
#[test]
fn a_box_opened_at_both_ends_is_a_tube() {
    for_all_scalars!(check_tube);
}

fn check_prism<S: Scalar>() {
    let mut part = Part::new();
    let solid = extruded_cylinder(
        &mut part,
        "p",
        v(0., 0., 0.),
        S::from_f64(1.),
        S::from_f64(1.5),
        6,
    )
    .unwrap();
    let (part, solid) = shelled(part, solid, &["cylinder(p,start)"], 0.1);
    assert_valid(&part);
    assert_eq!(faces(&part, solid), 7 + 7 + 1);
}
#[test]
fn a_hexagonal_prism_opened_at_its_bottom() {
    for_all_scalars!(check_prism);
}

fn check_revolved_cylinder<S: Scalar>() {
    let mut part = Part::new();
    let solid = revolved_cylinder(
        &mut part,
        "c",
        v(0., 0., 0.),
        S::from_f64(1.),
        S::from_f64(2.),
    )
    .unwrap();
    // The top cap, in quarters.
    let top: Vec<String> = part
        .topology()
        .body_faces(Body::Solid(solid))
        .unwrap()
        .into_iter()
        .map(|f| part.name_of(f).unwrap().to_string())
        .filter(|n| n.starts_with("cylinder(c,c0"))
        .collect();
    assert_eq!(top.len(), 4, "{top:?}");
    let top: Vec<&str> = top.iter().map(String::as_str).collect();
    let (part, solid) = shelled(part, solid, &top, 0.2);
    assert_valid(&part);
    // Side and bottom in quarters, inside and out, and a quarter of the rim
    // for each quarter of the top.
    assert_eq!(faces(&part, solid), 8 + 8 + 4);
}
#[test]
fn a_revolved_cylinder_opened_at_its_top() {
    for_all_scalars!(check_revolved_cylinder);
}

#[test]
fn a_closed_revolved_cylinder() {
    let mut part = Part::<ScalInF64>::new();
    let solid = revolved_cylinder(
        &mut part,
        "c",
        v(0., 0., 0.),
        ScalInF64::from_f64(1.),
        ScalInF64::from_f64(2.),
    )
    .unwrap();
    let (part, solid) = shelled(part, solid, &[], 0.2);
    assert_valid(&part);
    assert_eq!(faces(&part, solid), 24);
}

/// Walls thicker than half the box is high would turn its inside out.
#[test]
fn walls_too_thick_are_refused() {
    let (mut part, solid) = unit_box::<ScalInF64>();
    let namer = Namer::new("shell", "s").unwrap();
    let error = shell(&mut part, &namer, solid, &[], ScalInF64::from_f64(0.6)).unwrap_err();
    assert!(error.root_message().contains("too thick"), "{error:?}");
}

/// Only in `ScalInF64`: in `ScalInFPA64`'s 32 fractional bits, the normals
/// a curved face's vertices are moved along come out wide enough that a
/// moved vertex is wider than validation accepts.
#[test]
fn a_sphere_closed_all_round() {
    type S = ScalInF64;
    let mut part = Part::new();
    let solid = sphere_solid(&mut part, "s", v(1., 0., 0.), S::from_f64(2.)).unwrap();
    let (part, solid) = shelled(part, solid, &[], 0.5);
    assert_valid(&part);
    assert_eq!(part.topology().get_solid(solid).unwrap().shells.len(), 2);
    assert!(corners(&part).contains(&[1000, 0, 1500]));
    assert!(corners(&part).contains(&[-500, 0, 0]));
    // Moved along curved faces, the vertices are still as sharp as the
    // rounding of a few steps.
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

/// A sphere cannot be opened at some of its faces: where a face taken
/// away meets one kept, they are tangent, and the walls would have to end
/// on a smooth surface.
#[test]
fn a_sphere_does_not_open_along_a_smooth_seam() {
    let mut part = Part::<ScalInF64>::new();
    let solid = sphere_solid(&mut part, "s", v(0., 0., 0.), ScalInF64::from_f64(1.)).unwrap();
    let north: Vec<_> = (0..4)
        .map(|k| part.face_id(&format!("sphere(s,north,q{k})")).unwrap())
        .collect();
    let namer = Namer::new("shell", "s").unwrap();
    let error = shell(&mut part, &namer, solid, &north, ScalInF64::from_f64(0.1)).unwrap_err();
    assert!(error.root_message().contains("tangent"), "{error:?}");
}
