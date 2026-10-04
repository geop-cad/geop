//! Rolling-ball fillets: free-form edges, and radii that change along an
//! edge.

use geop_core_geometry::nurb_surface::NurbSurface3D;
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
use geop_ops_extrude_revolve::shapes::{
    cube::cube_solid,
    cylinder::{Axis, revolved_cylinder, revolved_cylinder_along_axis},
    sphere::sphere_solid,
};

use crate::blend::{BlendShape, blend};
use crate::rolling::{DEVIATION, Radii};

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

/// Blends `edges` of `part` into `shape` as the step `F`, and checks the
/// result is valid — failing, with every entity the error mentions by name.
fn blended(part: &mut Part<S>, edges: &[String], shape: &BlendShape) {
    let namer = Namer::new("fillet", "F").unwrap();
    if let Err(e) = blend(part, &namer, edges, shape) {
        let text = format!("{e:?}");
        let model = part.topology();
        let mut names = Vec::new();
        for &id in model.vertices.keys() {
            if text.contains(&format!("{id:?}")) {
                names.push(format!("{id:?} = {:?}", part.name_of(id)));
            }
        }
        for &id in model.edges.keys() {
            if text.contains(&format!("{id:?}")) {
                names.push(format!("{id:?} = {:?}", part.name_of(id)));
            }
        }
        for &id in model.faces.keys() {
            if text.contains(&format!("{id:?}")) {
                names.push(format!("{id:?} = {:?}", part.name_of(id)));
            }
        }
        panic!("{text}\nwhere {names:#?}");
    }
    assert_valid(part);
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
            let p = e
                .curve
                .evaluate(S::interpolate(t0, t1, S::from_f64(0.5)))
                .unwrap();
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
    let boss =
        revolved_cylinder(&mut part, "c", v(0.0, 0.0, 0.5), S::from_f64(0.4), S::ONE).unwrap();
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
    assert!(
        !rim.is_empty(),
        "no edge where the cylinder meets the sphere"
    );
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
    let r = 0.1;
    blended(&mut part, &rim[..1], &BlendShape::round(r));
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

/// The faces of `part` that are pieces of the blend of edge `edge` of step
/// `F` (see [`blend`]).
fn blend_faces<'a>(part: &'a Part<S>, edge: &str) -> Vec<(&'a str, &'a NurbSurface3D<S>)> {
    let prefix = format!("fillet(F,{edge},fillet");
    part.topology()
        .faces
        .iter()
        .filter_map(|(&id, face)| {
            let name = part.name_of(id)?;
            name.starts_with(&prefix).then_some((name, &face.surface))
        })
        .collect()
}

/// Samples of `surface` over its domain, as plain numbers.
fn samples(surface: &NurbSurface3D<S>, n: usize) -> Vec<[f64; 3]> {
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    let mut out = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            let u = S::interpolate(u0, u1, S::from_f64(i as f64 / n as f64));
            let w = S::interpolate(v0, v1, S::from_f64(j as f64 / n as f64));
            let p = surface.evaluate(u, w).unwrap();
            out.push([p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]);
        }
    }
    out
}

/// A cylinder of radius 0.5 lying along `x` with a cylinder of radius 0.3
/// standing on it along `z`, from `z = 0` up, joined: they meet in a saddle
/// curve, no circle — a free-form concave edge. The part, and the names of
/// the edges of that curve.
fn tee() -> (Part<S>, Vec<String>) {
    let mut part = Part::new();
    let pipe = revolved_cylinder_along_axis(
        &mut part,
        "p",
        v(-1.0, 0.0, 0.0),
        S::from_f64(0.5),
        S::from_f64(2.0),
        Axis::X,
    )
    .unwrap();
    let branch =
        revolved_cylinder(&mut part, "b", v(0.0, 0.0, 0.0), S::from_f64(0.3), S::ONE).unwrap();
    let namer = Namer::new("union", "u").unwrap();
    boolean(
        &mut part,
        &namer,
        pipe,
        branch,
        BooleanOp::Union,
        RemeshParams::default(),
    )
    .unwrap()
    .unwrap();
    let saddle = edges_where(&part, |p| {
        ((p[0] * p[0] + p[1] * p[1]).sqrt() - 0.3).abs() < 1e-6
            && ((p[1] * p[1] + p[2] * p[2]).sqrt() - 0.5).abs() < 1e-6
    });
    assert!(!saddle.is_empty(), "no edge where the cylinders meet");
    (part, saddle)
}

/// The branch of the tee filleted with radius 0.1: the ball's center runs
/// 0.4 from the branch's axis and 0.6 from the pipe's — on the curve
/// `((0.4 cos t, 0.4 sin t, sqrt(0.36 - 0.16 sin^2 t))` — so every point of
/// the blend is 0.1 from that curve, checked at samples to within the
/// deviation the blend is held to.
#[test]
fn tee_fillet_rolls_around_the_saddle() {
    let (mut part, saddle) = tee();
    let r = 0.1;
    blended(&mut part, &saddle[..1], &BlendShape::round(r));
    let spine = |t: f64| {
        let (x, y) = (0.4 * t.cos(), 0.4 * t.sin());
        [x, y, (0.36 - y * y).sqrt()]
    };
    let dist = |p: [f64; 3], t: f64| {
        let c = spine(t);
        ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2)).sqrt()
    };
    let faces = blend_faces(&part, &saddle[0]);
    assert!(!faces.is_empty(), "no blend face");
    for (name, surface) in faces {
        for p in samples(surface, 8) {
            // The nearest point of the spine: sampled, then golden-section.
            let mut t = (0..720)
                .map(|i| i as f64 * std::f64::consts::TAU / 720.0)
                .min_by(|&a, &b| dist(p, a).total_cmp(&dist(p, b)))
                .unwrap();
            let (mut lo, mut hi) = (t - 0.01, t + 0.01);
            for _ in 0..100 {
                let (a, b) = (lo + 0.382 * (hi - lo), lo + 0.618 * (hi - lo));
                if dist(p, a) < dist(p, b) {
                    hi = b;
                } else {
                    lo = a;
                }
                t = (lo + hi) / 2.0;
            }
            let off = dist(p, t) - r;
            assert!(
                off.abs() <= 2.0 * DEVIATION * r,
                "{name} at {p:?} is {off:e} off the rolling ball's surface"
            );
        }
    }
}

/// The unit cube `[0, 1]^3`, named `cube(b)`.
fn unit_cube() -> Part<S> {
    let mut part = Part::new();
    cube_solid(&mut part, "b", v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)).unwrap();
    part
}

/// The cube's upright edge at the origin rounded from radius 0.1 where it
/// starts to 0.3 where it ends: the round meets the two sides 0.1 from the
/// edge at one end of it and 0.3 at the other, and the blend is a cone
/// between.
#[test]
fn variable_radius_on_a_straight_edge() {
    let mut part = unit_cube();
    let edge = "cube(b,p0)";
    let id = part.edge_id(edge).unwrap();
    let e = part.topology().get_edge(id).unwrap();
    let z = |v| part.topology().get_vertex(v).unwrap().point[2].to_f64();
    let (z_start, z_end) = (z(e.start_vertex), z(e.end_vertex));
    let shape = BlendShape::Fillet {
        radii: Radii {
            radius: 0.1,
            end_radius: Some(0.3),
            at_vertices: Vec::new(),
        },
    };
    blended(&mut part, &[edge.to_string()], &shape);
    for (r, z) in [(0.1, z_start), (0.3, z_end)] {
        for p in [[r, 0.0, z], [0.0, r, z]] {
            let p = v::<S>(p[0], p[1], p[2]);
            assert!(
                part.topology()
                    .vertices
                    .values()
                    .any(|vertex| vertex.point.could_be_equal(&p)),
                "no vertex at {p:?}"
            );
        }
    }
    // Every point of the blend is as far from the edge's diagonal plane's
    // ball centers as the radius there: `(x - r, y - r)` of length `r`.
    for (name, surface) in blend_faces(&part, edge) {
        for p in samples(surface, 6) {
            let f = (p[2] - z_start) / (z_end - z_start);
            let r = 0.1 + 0.2 * f;
            let off = ((p[0] - r).hypot(p[1] - r) - r).abs();
            assert!(off <= 1e-2 * r, "{name} at {p:?} is {off:e} off the round");
        }
    }
}

/// A cylinder of radius 0.5 and height 1 standing on the origin, with a
/// flat cut along it at `x = 0.3`: a D-shaped shaft.
fn d_shaft() -> Part<S> {
    let mut part = Part::new();
    let shaft =
        revolved_cylinder(&mut part, "c", v(0.0, 0.0, 0.0), S::from_f64(0.5), S::ONE).unwrap();
    let flat = cube_solid(&mut part, "f", v(0.3, -1.0, -1.0), v(1.0, 1.0, 2.0)).unwrap();
    let namer = Namer::new("flat", "f").unwrap();
    boolean(
        &mut part,
        &namer,
        shaft,
        flat,
        BooleanOp::Difference,
        RemeshParams::default(),
    )
    .unwrap()
    .unwrap();
    part
}

/// The edges at the D-shaft's two top corners rounded: the flat's top
/// edge, straight between planes, swept; the top's rim and the flat's
/// sides, each between a plane and the cylinder, rolled. All end at the
/// balls touching the top, the flat and the cylinder, whose pieces round
/// the corners.
#[test]
fn d_shaft_fillet_top_corners() {
    let mut part = d_shaft();
    let y = 0.16f64.sqrt();
    let model = part.topology();
    let corners: Vec<_> = model
        .vertices
        .iter()
        .filter(|(_, vertex)| {
            [y, -y]
                .iter()
                .any(|&y| vertex.point.could_be_equal(&v(0.3, y, 1.0)))
        })
        .map(|(&id, _)| id)
        .collect();
    assert_eq!(corners.len(), 2);
    let mut edges: Vec<String> = model
        .edges
        .iter()
        .filter(|(_, e)| corners.contains(&e.start_vertex) || corners.contains(&e.end_vertex))
        .map(|(&id, _)| part.name_of(id).unwrap().to_string())
        .collect();
    edges.sort();
    edges.dedup();
    let r = 0.1;
    blended(&mut part, &edges, &BlendShape::round(r));
    // `r` below the top, from the flat and inside the cylinder.
    let y = (0.16f64 - 0.04).sqrt();
    let centers = [y, -y].map(|y| v::<S>(0.3 - r, y, 1.0 - r));
    let balls: Vec<_> = part
        .topology()
        .faces
        .iter()
        .filter(|&(&id, _)| part.name_of(id).unwrap().ends_with(",corner)"))
        .collect();
    assert_eq!(balls.len(), 2);
    for (_, face) in balls {
        let ps = samples(&face.surface, 4);
        let near = |c: &Vector3<S>| ps[0][1] * c[1].to_f64() > 0.0;
        let center = centers.iter().find(|c| near(c)).unwrap();
        for p in ps {
            let off = v::<S>(p[0], p[1], p[2]).sub(center).norm().to_f64() - r;
            assert!(
                off.abs() <= 2.0 * DEVIATION * r,
                "{p:?} is {off:e} off the ball"
            );
        }
    }
}

/// The D-shaft's flat side at `y > 0` bevelled, a straight edge between the
/// flat and the cylinder: rolled, its sections chords `0.1` into each face
/// in the plane square to the edge — on the flat `0.1` along it, on the
/// cylinder `0.1` as the crow flies. The chamfer face is flat, and meets
/// the top and bottom where the chords' ends are.
#[test]
fn d_shaft_chamfer_flat_side() {
    let mut part = d_shaft();
    let y = 0.16f64.sqrt();
    let side = edges_where(&part, |p| {
        (p[0] - 0.3).abs() < 1e-6 && (p[1] - y).abs() < 1e-6
    });
    assert_eq!(side.len(), 1, "{side:?}");
    let d = 0.1;
    blended(&mut part, &side, &BlendShape::Chamfer { distances: [d, d] });
    let faces: Vec<&str> = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .filter(|n| n.ends_with(",chamfer)"))
        .collect();
    assert_eq!(faces.len(), 1, "{faces:?}");
    // Where the chord meets the flat and the cylinder, at the top.
    let on_flat = [0.3, y - d, 1.0];
    // On the cylinder, `d` from the edge: `x^2 + y^2 = 0.25`, `|(x - 0.3,
    // y' - y)| = d`.
    let angle = (y / 0.3f64).atan() + 2.0 * (d / 2.0 / 0.5f64).asin();
    let on_cylinder = [0.5 * angle.cos(), 0.5 * angle.sin(), 1.0];
    for p in [on_flat, on_cylinder] {
        let p = v::<S>(p[0], p[1], p[2]);
        let near = part
            .topology()
            .vertices
            .values()
            .map(|vertex| vertex.point.sub(&p).norm().to_f64())
            .fold(f64::INFINITY, f64::min);
        assert!(
            near <= 2.0 * DEVIATION * d,
            "no vertex at {p:?}: {near:e} away"
        );
    }
}

/// The tee's saddle bevelled: a closed free-form edge, concave, so the
/// chamfer fills in, its chords `0.1` into the branch and the pipe.
#[test]
fn tee_chamfer_around_the_saddle() {
    let (mut part, saddle) = tee();
    blended(
        &mut part,
        &saddle[..1],
        &BlendShape::Chamfer {
            distances: [0.1, 0.1],
        },
    );
    let chamfered = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .any(|n| n.starts_with(&format!("fillet(F,{},chamfer", saddle[0])));
    assert!(chamfered);
}
