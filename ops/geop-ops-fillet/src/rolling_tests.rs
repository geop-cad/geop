//! Rolling-ball fillets: free-form edges, and radii that change along an
//! edge.

use geop_core_math::{
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{Namer, Part};
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::shapes::{cylinder::revolved_cylinder, sphere::sphere_solid};

use crate::blend::{BlendShape, blend};
use crate::rolling::DEVIATION;

fn v<T: Scalar>(x: f64, y: f64, z: f64) -> Vector3<T> {
    Vector3::from_array([T::from_f64(x), T::from_f64(y), T::from_f64(z)])
}

/// Checks `part` is a valid manifold model.
fn assert_valid(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
}

/// The names of the edges of `part` whose middle `keep` accepts, as plain
/// numbers.
fn edges_where(part: &Part<S>, keep: impl Fn([f64; 3]) -> bool) -> Vec<String> {
    let model = part.topology();
    let mut names: Vec<String> = model
        .edges
        .iter()
        .filter(|(_, e)| {
            let (t0, t1) = e.curve.domain();
            let p = e.curve.evaluate(S::interpolate(t0, t1, S::from_f64(0.5))).unwrap();
            keep([p[0].to_f64(), p[1].to_f64(), p[2].to_f64()])
        })
        .map(|(&id, _)| part.name_of(id).unwrap().to_string())
        .collect();
    names.sort();
    names
}

/// A sphere of radius 1 around the origin with a cylinder of radius 0.4
/// standing on its axis, from `z = 0.5` to `1.5`, joined: they meet in a
/// circle at `z = sqrt(0.84)`, a concave edge between the cylinder and the
/// sphere. The part, and the names of the edges of that circle.
fn boss_on_sphere() -> (Part<S>, Vec<String>) {
    let mut part = Part::new();
    let ball = sphere_solid(&mut part, "s", v(0.0, 0.0, 0.0), S::ONE).unwrap();
    let boss = revolved_cylinder(&mut part, "c", v(0.0, 0.0, 0.5), S::from_f64(0.4), S::ONE)
        .unwrap();
    let namer = Namer::new("union", "u").unwrap();
    boolean(
        &mut part,
        &namer,
        ball,
        boss,
        BooleanOp::Union,
        RemeshParams::default(),
    )
    .unwrap()
    .unwrap();
    let z = 0.84f64.sqrt();
    let rim = edges_where(&part, |p| {
        ((p[0] * p[0] + p[1] * p[1]).sqrt() - 0.4).abs() < 1e-6 && (p[2] - z).abs() < 1e-6
    });
    assert!(!rim.is_empty(), "no edge where the cylinder meets the sphere");
    (part, rim)
}

/// The boss's rim filleted with radius 0.1: the ball, 0.1 from the
/// cylinder and outside the sphere, rolls round a circle of radius 0.5 at
/// `z = sqrt(1.1^2 - 0.5^2)`, so the blend is a piece of the torus around
/// it — checked at samples of every blend face, to within the deviation the
/// blend is held to.
#[test]
fn boss_on_sphere_fillet_is_a_torus() {
    let (mut part, rim) = boss_on_sphere();
    let namer = Namer::new("fillet", "F").unwrap();
    let r = 0.1;
    blend(&mut part, &namer, &rim[..1], &BlendShape::round(r)).unwrap();
    assert_valid(&part);
    let zc = (1.1f64 * 1.1 - 0.5 * 0.5).sqrt();
    let model = part.topology();
    let mut blend_faces = 0;
    for (&id, face) in &model.faces {
        let name = part.name_of(id).unwrap();
        if !name.starts_with(&format!("fillet(F,{},fillet", rim[0])) {
            continue;
        }
        blend_faces += 1;
        let ((u0, u1), (v0, v1)) = (face.surface.domain_u(), face.surface.domain_v());
        for i in 0..=8 {
            for j in 0..=8 {
                let u = S::interpolate(u0, u1, S::from_f64(i as f64 / 8.0));
                let w = S::interpolate(v0, v1, S::from_f64(j as f64 / 8.0));
                let p = face.surface.evaluate(u, w).unwrap();
                let (x, y, z) = (p[0].to_f64(), p[1].to_f64(), p[2].to_f64());
                let off = ((x * x + y * y).sqrt() - 0.5).hypot(z - zc) - r;
                assert!(
                    off.abs() <= 2.0 * DEVIATION * r,
                    "{name} at ({i}, {j}) is {off:e} off the torus"
                );
            }
        }
    }
    assert!(blend_faces > 0, "no blend face");
}
