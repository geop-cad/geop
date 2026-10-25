//! The plastic features on a box and on a shelled box — an enclosure:
//! valid, and of the volume they should have.

use geop_core_geometry::shape::Plane;
use geop_core_math::{primitives::CoordinateSystem, scalars::Scalar, vector::Vector2};
use geop_core_topology::{FaceId, SolidId};
use geop_lib_shell::shell;
use geop_lib_sweep::{
    polyline,
    shapes::{block, v3},
};
use geop_ops_model::Profile;
use geop_ops_model::{
    NamedModel, Namer,
    testing::{assert_valid, face_towards, volume},
};

use crate::{
    draft::draft,
    lip::{LipSize, groove, lip},
    rib::{Growth, rib},
};

/// How closely [`volume`] measures: its triangles' corners are `f32`, as
/// STL stores them — exact for faces that are flat, up to those corners.
const VOLUME_TOLERANCE: f64 = 1e-5;

/// The 2 x 2 x 1 box from the origin, `cube(b)`.
fn unit_box() -> (NamedModel, SolidId) {
    let mut part = NamedModel::new();
    let solid = block(&mut part, "b", [0., 0., 0.], [2., 2., 1.]);
    (part, solid)
}

/// The unit box shelled `thickness` thick, open at its top: an enclosure.
fn enclosure(thickness: f64) -> (NamedModel, SolidId) {
    let (mut part, solid) = unit_box();
    let top = face_towards(&part, solid, [0., 0., 1.]);
    let namer = Namer::new("shell", "s").unwrap();
    let solid = shell(
        &mut part,
        &namer,
        solid,
        &[top],
        Scalar::from_f64(thickness),
    )
    .unwrap();
    (part, solid)
}

/// The four outer walls of a box or an enclosure.
fn outer_walls(part: &NamedModel, solid: SolidId) -> Vec<FaceId> {
    [[1., 0., 0.], [-1., 0., 0.], [0., 1., 0.], [0., -1., 0.]]
        .iter()
        .map(|n| face_towards(part, solid, *n))
        .collect()
}

/// The ground plane, pulled up.
fn ground() -> Plane {
    Plane::try_new(v3(0., 0., 0.), v3(0., 0., 1.)).unwrap()
}

fn drafted(mut part: NamedModel, faces: &[FaceId], degrees: f64) -> (NamedModel, SolidId) {
    let namer = Namer::new("draft", "d").unwrap();
    let solid = draft(
        &mut part,
        &namer,
        faces,
        &ground(),
        Scalar::from_f64(degrees.to_radians()),
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
    let before = volume(&part, solid, 16);
    let side = face_towards(&part, solid, [1., 0., 0.]);
    let name = part.name_of(side).unwrap().to_string();
    let (part, solid) = drafted(part, &[side], 5.0);
    assert_valid(&part);
    let after = volume(&part, solid, 16);
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
    let before = volume(&part, solid, 16);
    let walls = outer_walls(&part, solid);
    let (part, solid) = drafted(part, &walls, 3.0);
    assert_valid(&part);
    let after = volume(&part, solid, 16);
    let taken = 4.0 - frustum(2.0, 1.0, 3.0);
    assert!(
        (before - after - taken).abs() < VOLUME_TOLERANCE,
        "{before} - {after} vs {taken}"
    );
}

/// The enclosure's rim, the face its open top leaves.
fn rim(part: &NamedModel, solid: SolidId) -> FaceId {
    face_towards(part, solid, [0., 0., 1.])
}

/// The area of the ring between the enclosure's 1.6 x 1.6 cavity and the
/// square `out` further out all round.
fn ring(out: f64) -> f64 {
    (1.6 + 2.0 * out).powi(2) - 1.6 * 1.6
}

/// A lip along the inside of the enclosure's rim, the default: the ring
/// `width` wide, `height` high, joined on.
#[test]
fn enclosure_lip() {
    let (mut part, solid) = enclosure(0.2);
    let before = volume(&part, solid, 16);
    let face = rim(&part, solid);
    let namer = Namer::new("lip", "l").unwrap();
    let size = LipSize {
        width: Scalar::from_f64(0.08),
        height: Scalar::from_f64(0.1),
    };
    lip(&mut part, &namer, "l", face, &[], size).unwrap();
    assert_valid(&part);
    let solid = part.solid_id("lip(l)").unwrap();
    let added = volume(&part, solid, 16) - before;
    let expected = ring(0.08) * 0.1;
    assert!(
        (added - expected).abs() < VOLUME_TOLERANCE,
        "{added} vs {expected}"
    );
}

/// The groove taking that lip, with 0.02 clearance: the ring 0.1 wide cut
/// 0.12 deep.
#[test]
fn enclosure_groove() {
    let (mut part, solid) = enclosure(0.2);
    let before = volume(&part, solid, 16);
    let face = rim(&part, solid);
    let namer = Namer::new("groove", "g").unwrap();
    let size = LipSize {
        width: Scalar::from_f64(0.08),
        height: Scalar::from_f64(0.1),
    };
    groove(
        &mut part,
        &namer,
        "g",
        face,
        &[],
        size,
        Scalar::from_f64(0.02),
    )
    .unwrap();
    assert_valid(&part);
    let solid = part.solid_id("groove(g)").unwrap();
    let taken = before - volume(&part, solid, 16);
    let expected = ring(0.1) * 0.12;
    assert!(
        (taken - expected).abs() < VOLUME_TOLERANCE,
        "{taken} vs {expected}"
    );
}

/// A rib across the enclosure's cavity from the line `line` in `plane`,
/// grown `growth` down to the floor, its ends run on into the walls — and
/// the volume it adds: the cavity's width 1.6, from the line at 0.7 down
/// to the floor at 0.2, 0.05 thick.
fn check_rib(plane: CoordinateSystem, line: [[f64; 2]; 2], growth: Growth) {
    let (mut part, solid) = enclosure(0.2);
    let before = volume(&part, solid, 16);
    let profile = Profile::open(polyline(&[q(line[0]), q(line[1])]).unwrap());
    let namer = Namer::new("rib", "r").unwrap();
    let half = Scalar::from_f64(0.025);
    rib(
        &mut part,
        &namer,
        "r",
        &plane,
        &profile,
        solid,
        (Scalar::from_f64(-0.025), half),
        growth,
    )
    .unwrap();
    assert_valid(&part);
    let solid = part.solid_id("rib(r)").unwrap();
    let added = volume(&part, solid, 16) - before;
    let expected = 1.6 * 0.5 * 0.05;
    assert!(
        (added - expected).abs() < VOLUME_TOLERANCE,
        "{added} vs {expected}"
    );
}

fn q(p: [f64; 2]) -> Vector2 {
    Vector2::from_array(p.map(Scalar::from_f64))
}

/// The plane through the enclosure's middle, `u` along `x` and `v` along
/// `z`, at `y = 1`.
fn middle_plane() -> CoordinateSystem {
    CoordinateSystem::try_new(
        v3(0., 1., 0.),
        v3(1., 0., 0.),
        v3(0., 0., 1.),
        v3(0., -1., 0.),
    )
    .unwrap()
}

/// Grown parallel to its sketch, down from the line to the floor.
#[test]
fn rib_parallel_to_its_sketch() {
    check_rib(
        middle_plane(),
        [[0.5, 0.7], [1.5, 0.7]],
        Growth::Parallel { flipped: true },
    );
}

/// Grown normal to its sketch, a plane across the cavity at 0.7, down to
/// the floor.
#[test]
fn rib_normal_to_its_sketch() {
    check_rib(
        CoordinateSystem::world_at(v3(0., 0., 0.7)),
        [[0.5, 1.0], [1.5, 1.0]],
        Growth::Normal { flipped: true },
    );
}

/// A rib grown up, out of the open top, meets nothing that stops it: it is
/// refused, saying so.
#[test]
fn rib_growing_out_of_the_top_is_refused() {
    let (mut part, solid) = enclosure(0.2);
    let plane = CoordinateSystem::world_at(v3(0., 0., 0.7));
    let profile = Profile::open(polyline(&[q([0.5, 1.0]), q([1.5, 1.0])]).unwrap());
    let namer = Namer::new("rib", "r").unwrap();
    let half = Scalar::from_f64(0.025);
    let Err(error) = rib(
        &mut part,
        &namer,
        "r",
        &plane,
        &profile,
        solid,
        (Scalar::from_f64(-0.025), half),
        Growth::Normal { flipped: false },
    ) else {
        panic!("a rib out of the top is refused");
    };
    let message = format!("{error}");
    assert!(
        message.contains("without meeting it all along"),
        "{message}"
    );
}
