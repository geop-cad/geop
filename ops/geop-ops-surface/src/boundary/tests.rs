//! Boundary surfaces between the edges of cubes and of sheets built for the
//! purpose.

use geop_core_geometry::nurb_curve::NurbCurve;
use geop_core_math::{
    scalars::{Ring, ScalInF64 as S, Scalar},
    vector::{Vector3, Vector4},
};
use geop_core_topology::{
    Body, Curve3,
    validation::{ValidationParameters, validate},
};
use geop_ops::{NoFiles, Part};
use geop_ops_extrude_revolve::shapes::cube_solid;

use super::*;

pub(crate) fn v3(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

pub(crate) fn assert_valid(part: &Part<S>) {
    if let Err(errors) = validate(&ValidationParameters::default(), part.topology()) {
        let messages: Vec<String> = errors.iter().map(|e| format!("{e}")).collect();
        panic!("{} errors:\n{}", messages.len(), messages.join("\n"));
    }
    part.check_names().unwrap();
}

/// The name of the edge of `part` from `a` to `b`, either way.
pub(crate) fn edge_between(part: &Part<S>, a: [f64; 3], b: [f64; 3]) -> String {
    let model = part.topology();
    let (a, b) = (v3(a[0], a[1], a[2]), v3(b[0], b[1], b[2]));
    let at = |id, p: &Vector3<S>| model.get_vertex(id).unwrap().point.could_be_equal(p);
    let found: Vec<_> = model
        .edges
        .iter()
        .filter(|(_, e)| {
            (at(e.start_vertex, &a) && at(e.end_vertex, &b))
                || (at(e.start_vertex, &b) && at(e.end_vertex, &a))
        })
        .map(|(&id, _)| part.name_of(id).unwrap().to_string())
        .collect();
    assert_eq!(found.len(), 1, "edges from {a:?} to {b:?}: {found:?}");
    found.into_iter().next().unwrap()
}

fn unit_cube() -> Part<S> {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "c", v3(0., 0., 0.), v3(1., 1., 1.)).unwrap();
    part
}

fn span(part: Part<S>, edges: &[String], tangent: &[String]) -> GeopResult<Part<S>> {
    let args = BoundarySurfaceArgs {
        edges: edges.to_vec(),
        tangent: tangent.to_vec(),
    };
    BoundarySurface.apply(part, "b", &args, &NoFiles)
}

/// Why `result` failed — it must have.
pub(crate) fn refused(result: GeopResult<Part<S>>) -> String {
    match result {
        Ok(_) => panic!("built, but should have been refused"),
        Err(e) => format!("{e}"),
    }
}

fn sheet_faces(part: &Part<S>) -> usize {
    part.sheet_face_names().len()
}

pub(crate) fn pt(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
    Vector4::from_array([x * w, y * w, z * w, w].map(S::from_f64))
}

/// A sheet of one face, the Coons patch of `sides`, its entities named
/// `name(...)`.
pub(crate) fn coons_sheet(part: &mut Part<S>, name: &str, sides: [Curve3<S>; 4]) {
    let corners: Vec<Vector3<S>> = sides
        .iter()
        .map(|c| c.evaluate(c.domain().0).unwrap())
        .collect();
    let surface = NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]]).unwrap();
    let spec = BodySpec {
        vertices: corners,
        edges: sides
            .iter()
            .enumerate()
            .map(|(k, c)| EdgeSpec {
                curve: c.clone(),
                start: k,
                end: (k + 1) % 4,
            })
            .collect(),
        faces: vec![patch_face(surface, [0, 1, 2, 3].map(|k| (k, Sense::Forward))).unwrap()],
        shells: vec![vec![0]],
        solid: false,
    };
    let names = BodyNames {
        vertices: (0..4).map(|k| format!("{name}(v{k})")).collect(),
        edges: (0..4).map(|k| format!("{name}(e{k})")).collect(),
        faces: vec![format!("{name}(f)")],
        solid: None,
    };
    part.build_body(spec, names).unwrap();
}

pub(crate) fn line(a: [f64; 3], b: [f64; 3]) -> Curve3<S> {
    line3(v3(a[0], a[1], a[2]), v3(b[0], b[1], b[2])).unwrap()
}

/// The quarter circle in the plane `x = x0` from `(x0, 0, 0)` round the
/// center `(x0, 0, 1)` to `(x0, 1, 1)`: it rises from the plane `z = 0`,
/// leaving it along `y`.
pub(crate) fn rising_arc(x0: f64) -> Curve3<S> {
    NurbCurve::try_new(
        2,
        vec![
            pt(x0, 0., 0., 1.),
            pt(x0, 1., 0., std::f64::consts::SQRT_2 / 2.0),
            pt(x0, 1., 1., 1.),
        ],
        [0., 0., 0., 1., 1., 1.].map(S::from_f64).to_vec(),
    )
    .unwrap()
}

/// A cubic from `(1, 1, 1)` to `(0, 1, 1)` bending up and down.
pub(crate) fn wavy_top() -> Curve3<S> {
    NurbCurve::try_new(
        3,
        vec![
            pt(0., 1., 1., 1.),
            pt(0.3, 1., 1.5, 1.),
            pt(0.6, 1., 0.6, 1.),
            pt(0.8, 1., 1.3, 1.),
            pt(1., 1., 1., 1.),
        ],
        [0., 0., 0., 0., 0.5, 1., 1., 1., 1.]
            .map(S::from_f64)
            .to_vec(),
    )
    .unwrap()
    .reverse()
}

/// Two opposite edges of a cube's top, ruled: a copy of the top.
#[test]
fn ruled_between_opposite_edges() {
    let part = unit_cube();
    let a = edge_between(&part, [0., 0., 1.], [1., 0., 1.]);
    // Picked running the other way: turned to line up.
    let b = edge_between(&part, [0., 1., 1.], [1., 1., 1.]);
    let part = span(part, &[a.clone(), b], &[]).unwrap();
    assert_valid(&part);
    assert_eq!(part.sheet_face_names(), ["boundary(b)"]);
    assert!(part.edge_id(&format!("boundary(b,{a})")).is_ok());
}

/// Two edges meeting at a corner would make a ruled face come to a point.
#[test]
fn ruled_between_edges_sharing_a_corner_is_refused() {
    let part = unit_cube();
    let a = edge_between(&part, [0., 0., 1.], [1., 0., 1.]);
    let b = edge_between(&part, [1., 0., 1.], [1., 1., 1.]);
    let err = refused(span(part, &[a, b], &[]));
    assert!(format!("{err}").contains("share an end"), "{err}");
}

/// The four edges of a cube's face, in any order: flat, so filled with
/// one flat face.
#[test]
fn flat_loop_of_four_is_one_flat_face() {
    let part = unit_cube();
    let edges = [
        edge_between(&part, [0., 0., 1.], [1., 0., 1.]),
        edge_between(&part, [1., 1., 1.], [0., 1., 1.]),
        edge_between(&part, [1., 0., 1.], [1., 1., 1.]),
        edge_between(&part, [0., 1., 1.], [0., 0., 1.]),
    ];
    let part = span(part, &edges, &[]).unwrap();
    assert_valid(&part);
    let face = part.face_id("boundary(b)").unwrap();
    let surface = &part.topology().get_face(face).unwrap().surface;
    assert!(surface.as_plane().unwrap().is_some());
}

/// Six edges of a cube that run round it, not in any one plane: a patch of
/// six quadrilaterals around a center.
#[test]
fn hexagon_round_a_cube_is_six_quadrilaterals() {
    let part = unit_cube();
    let corners = [
        [1., 0., 0.],
        [1., 1., 0.],
        [0., 1., 0.],
        [0., 1., 1.],
        [0., 0., 1.],
        [1., 0., 1.],
    ];
    let edges: Vec<String> = (0..6)
        .map(|k| edge_between(&part, corners[k], corners[(k + 1) % 6]))
        .collect();
    let part = span(part, &edges, &[]).unwrap();
    assert_valid(&part);
    assert_eq!(sheet_faces(&part), 6);
    let center = part.vertex_id("boundary(b,center)").unwrap();
    let at = part.topology().get_vertex(center).unwrap().point;
    // The center is a free choice, sharpened: the average of the edges'
    // middles, to within rounding.
    assert!(
        (0..3).all(|k| (at[k].to_f64() - 0.5).abs() < 1e-12),
        "{at:?}"
    );
}

/// The four edges of a twisted ruled sheet: their Coons patch runs exactly
/// along each of them.
#[test]
fn coons_patch_of_a_twisted_loop() {
    let mut part = Part::<S>::new();
    coons_sheet(
        &mut part,
        "twist",
        [
            line([0., 0., 0.], [1., 0., 0.]),
            line([1., 0., 0.], [1., 1., 0.5]),
            line([1., 1., 0.5], [0., 1., 1.]),
            line([0., 1., 1.], [0., 0., 0.]),
        ],
    );
    let edges: Vec<String> = (0..4).map(|k| format!("twist(e{k})")).collect();
    let part = span(part, &edges, &[]).unwrap();
    assert_valid(&part);
    let face = part.face_id("boundary(b)").unwrap();
    let face = part.topology().get_face(face).unwrap();
    assert!(face.surface.as_plane().unwrap().is_none());
    for k in 0..4 {
        let e = part.edge_id(&format!("boundary(b,twist(e{k}))")).unwrap();
        let curve = &part.topology().get_edge(e).unwrap().curve;
        for i in 0..=4 {
            let t = S::from_ratio(i, 4).unwrap();
            let p = curve.evaluate(t).unwrap();
            let (u, v) = match k {
                0 => (t, S::ZERO),
                1 => (S::ONE, t),
                2 => (S::ONE.sub(t), S::ONE),
                _ => (S::ZERO, S::ONE.sub(t)),
            };
            assert!(face.surface.evaluate(u, v).unwrap().could_be_equal(&p));
        }
    }
}

/// A cover rising from a flat flange: filled between the flange's edge,
/// two quarter circles leaving the flange along it and a wavy top, and
/// tangent to the flange along its edge — its normal there the flange's.
#[test]
fn cover_tangent_to_a_flange() {
    let mut part = Part::<S>::new();
    coons_sheet(
        &mut part,
        "flange",
        [
            line([0., 0., 0.], [0., -1., 0.]),
            line([0., -1., 0.], [1., -1., 0.]),
            line([1., -1., 0.], [1., 0., 0.]),
            line([1., 0., 0.], [0., 0., 0.]),
        ],
    );
    coons_sheet(
        &mut part,
        "guide",
        [
            line([0., 0., 0.], [1., 0., 0.]),
            rising_arc(1.),
            wavy_top(),
            rising_arc(0.).reverse(),
        ],
    );
    let lip = "flange(e3)".to_string();
    let edges = [
        lip.clone(),
        "guide(e1)".into(),
        "guide(e2)".into(),
        "guide(e3)".into(),
    ];
    let tangent = span(part.clone(), &edges, std::slice::from_ref(&lip)).unwrap();
    assert_valid(&tangent);
    let face = tangent.face_id("boundary(b)").unwrap();
    let face = tangent.topology().get_face(face).unwrap();
    let up = v3(0., 0., 1.);
    let along_lip = |surface: &NurbSurface3D<S>| {
        (0..=8).all(|i| {
            let u = S::from_ratio(i, 8).unwrap();
            let n = surface.normal(u, S::ZERO).unwrap();
            n.prod_cross(&up).norm_sq().could_be_equal(S::ZERO)
        })
    };
    assert!(along_lip(&face.surface));
    // Without tangency, the patch leaves the flange at an angle.
    let plain = span(part.clone(), &edges, &[]).unwrap();
    let face = plain.face_id("boundary(b)").unwrap();
    assert!(!along_lip(&plain.topology().get_face(face).unwrap().surface));
    // Tangent to a guide edge, whose face is curved: refused.
    let err = refused(span(part, &edges, &["guide(e2)".into()]));
    assert!(format!("{err}").contains("not flat"), "{err}");
}

/// Edges that do not close a loop are refused, naming where it breaks off.
#[test]
fn open_chain_is_refused() {
    let part = unit_cube();
    let edges = [
        edge_between(&part, [0., 0., 1.], [1., 0., 1.]),
        edge_between(&part, [1., 0., 1.], [1., 1., 1.]),
        edge_between(&part, [1., 1., 1.], [0., 1., 1.]),
    ];
    let err = refused(span(part, &edges, &[]));
    assert!(format!("{err}").contains("not a closed loop"), "{err}");
}

/// The circle round a cylinder's end: one flat disc.
#[test]
fn circle_is_filled_flat() {
    let mut part = Part::<S>::new();
    geop_ops_extrude_revolve::shapes::cylinder::revolved_cylinder(
        &mut part,
        "cyl",
        v3(0., 0., 0.),
        S::from_f64(1.0),
        S::from_f64(2.0),
    )
    .unwrap();
    let rims: Vec<String> = part
        .topology()
        .edges
        .iter()
        .filter(|(_, e)| e.curve.as_arc().unwrap().is_some())
        .filter(|(_, e)| {
            let p = part.topology().get_vertex(e.start_vertex).unwrap().point;
            p[2].could_be_equal(S::from_f64(2.0))
        })
        .map(|(&id, _)| part.name_of(id).unwrap().to_string())
        .collect();
    let part = span(part, &rims, &[]).unwrap();
    assert_valid(&part);
    let sheet = part.face_id("boundary(b)").unwrap();
    assert!(matches!(
        part.topology().body_of_face(sheet).unwrap(),
        Body::Sheet(_)
    ));
}
