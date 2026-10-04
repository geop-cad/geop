//! The plastic features on a box and on a shelled box — an enclosure:
//! valid, and of the volume they should have.

use geop_core_geometry::shape::Plane;
use geop_core_math::{
    scalars::{ScalInF64, Scalar},
    vector::Vector3,
};
use geop_core_topology::{
    Body, FaceId, SolidId,
    validation::{ValidationParameters, validate, validate_manifold},
};
use geop_ops::{Namer, Part};
use geop_ops_extrude_revolve::shapes::cube_solid;
use geop_ops_rasterize::{rasterize, stl::stl_triangles};
use geop_ops_shell::shell::shell;

use crate::draft::draft;

type S = ScalInF64;

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate(&params, part.topology()) {
        panic!("{errors:#?}");
    }
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        panic!("{errors:#?}");
    }
}

/// How closely [`volume`] measures: its triangles' corners are `f32`, as
/// STL stores them. A test's measuring stick, not a kernel comparison.
const VOLUME_TOLERANCE: f64 = 1e-5;

/// The volume `solid` encloses, from its outward-wound triangles — exact
/// for faces that are flat, up to the `f32` corners.
fn volume(part: &Part<S>, solid: SolidId) -> f64 {
    let raster = rasterize(part.topology(), 16).unwrap();
    let faces = part.topology().body_faces(Body::Solid(solid)).unwrap();
    stl_triangles(&raster, &faces)
        .iter()
        .map(|t| {
            let [a, b, c] = t.corners.map(|p| p.map(f64::from));
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

/// The 2 x 2 x 1 box from the origin, `cube(b)`.
fn unit_box() -> (Part<S>, SolidId) {
    let mut part = Part::new();
    let solid = cube_solid(&mut part, "b", v(0., 0., 0.), v(2., 2., 1.)).unwrap();
    (part, solid)
}

/// The unit box shelled `thickness` thick, open at its top: an enclosure.
fn enclosure(thickness: f64) -> (Part<S>, SolidId) {
    let (mut part, solid) = unit_box();
    let top = face_towards(&part, solid, [0., 0., 1.]);
    let namer = Namer::new("shell", "s").unwrap();
    let solid = shell(&mut part, &namer, solid, &[top], S::from_f64(thickness)).unwrap();
    (part, solid)
}

/// The planar face of `solid` whose outward normal is `normal`, lying
/// furthest that way.
fn face_towards(part: &Part<S>, solid: SolidId, normal: [f64; 3]) -> FaceId {
    let n = v(normal[0], normal[1], normal[2]);
    let model = part.topology();
    model
        .body_faces(Body::Solid(solid))
        .unwrap()
        .into_iter()
        .filter_map(|f| {
            let plane = model.get_face(f).unwrap().surface.as_plane().unwrap()?;
            plane
                .normal
                .could_be_equal(&n)
                .then(|| (f, plane.point.prod_dot(&n).to_f64()))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(f, _)| f)
        .expect("a face that way")
}

/// The four outer walls of a box or an enclosure.
fn outer_walls(part: &Part<S>, solid: SolidId) -> Vec<FaceId> {
    [[1., 0., 0.], [-1., 0., 0.], [0., 1., 0.], [0., -1., 0.]]
        .iter()
        .map(|n| face_towards(part, solid, *n))
        .collect()
}

/// The ground plane, pulled up.
fn ground() -> Plane<S> {
    Plane::try_new(v(0., 0., 0.), v(0., 0., 1.)).unwrap()
}

fn drafted(mut part: Part<S>, faces: &[FaceId], degrees: f64) -> (Part<S>, SolidId) {
    let namer = Namer::new("draft", "d").unwrap();
    let solid = draft(
        &mut part,
        &namer,
        faces,
        &ground(),
        S::from_f64(degrees.to_radians()),
    )
    .unwrap();
    (part, solid)
}

/// The volume of the frustum a `side` x `side` square makes, `height`
/// high, its sides leaning in at `degrees`.
fn frustum(side: f64, height: f64, degrees: f64) -> f64 {
    let top = side - 2.0 * height * degrees.to_radians().tan();
    height * (side * side + top * top + side * top) / 3.0
}

/// One side of a box drafted 5 degrees about its bottom: its top edge moves
/// in by `tan 5`, taking a wedge of that much off, and every name stays.
#[test]
fn box_side_drafted() {
    let (part, solid) = unit_box();
    let before = volume(&part, solid);
    let side = face_towards(&part, solid, [1., 0., 0.]);
    let name = part.name_of(side).unwrap().to_string();
    let (part, solid) = drafted(part, &[side], 5.0);
    assert_valid(&part);
    let after = volume(&part, solid);
    let wedge = 0.5 * 1.0 * 5f64.to_radians().tan() * 2.0;
    assert!(
        (before - after - wedge).abs() < VOLUME_TOLERANCE,
        "{before} - {after} vs {wedge}"
    );
    assert!(part.face_id(&name).is_ok());
    assert_eq!(part.name_of(solid), Some("draft(d)"));
}

/// All four walls of an enclosure drafted outside: each wall's outside
/// leans in towards the top, the inside stays.
#[test]
fn enclosure_outside_drafted() {
    let (part, solid) = enclosure(0.2);
    let before = volume(&part, solid);
    let walls = outer_walls(&part, solid);
    let (part, solid) = drafted(part, &walls, 3.0);
    assert_valid(&part);
    let after = volume(&part, solid);
    let taken = 4.0 - frustum(2.0, 1.0, 3.0);
    assert!(
        (before - after - taken).abs() < VOLUME_TOLERANCE,
        "{before} - {after} vs {taken}"
    );
}
