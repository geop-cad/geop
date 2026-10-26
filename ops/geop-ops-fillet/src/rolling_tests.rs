//! Rolling-ball fillets: free-form edges, and radii that change along an
//! edge.

use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{scalars::Scalar, vector::Vector3};
use geop_lib_sweep::shapes::{
    Axis, block, cube_solid, cylinder, revolved_cylinder, revolved_cylinder_along_axis,
    sphere_solid, v3,
};
use geop_ops_model::BooleanOp;
use geop_ops_model::{
    NamedModel, Namer,
    testing::{assert_valid, edges_with_middle_where, has_vertex, vertex_at},
};

use crate::blend::{BlendShape, blend};
use crate::rolling::{DEVIATION, Radii};
use crate::tests::{combined, unit_cube};

/// Blends `edges` of `model` into `shape` as the step `F`, and checks the
/// result is valid — failing, with every entity the error mentions by name.
fn blended(model: &mut NamedModel, edges: &[String], shape: &BlendShape) {
    let namer = Namer::new("fillet", "F").unwrap();
    if let Err(e) = blend(model, &namer, edges, shape) {
        let text = format!("{e:?}");
        let topology = model.topology();
        let mut names = Vec::new();
        for &id in topology.vertices.keys() {
            if text.contains(&format!("{id:?}")) {
                names.push(format!("{id:?} = {:?}", model.name_of(id)));
            }
        }
        for &id in topology.edges.keys() {
            if text.contains(&format!("{id:?}")) {
                names.push(format!("{id:?} = {:?}", model.name_of(id)));
            }
        }
        for &id in topology.faces.keys() {
            if text.contains(&format!("{id:?}")) {
                names.push(format!("{id:?} = {:?}", model.name_of(id)));
            }
        }
        panic!("{text}\nwhere {names:#?}");
    }
    assert_valid(model);
}

/// A sphere of radius 1 around the origin with a cylinder of radius 0.4
/// standing on its axis, from `z = 0.5` to `1.5`, joined: they meet in a
/// circle at `z = sqrt(0.84)`, a concave edge between the cylinder and the
/// sphere. The part, and the names of the edges of that circle.
fn boss_on_sphere() -> (NamedModel, Vec<String>) {
    let mut model = NamedModel::new();
    let ball = sphere_solid(&mut model, "s", v3(0.0, 0.0, 0.0), Scalar::ONE).unwrap();
    let boss = cylinder(&mut model, "c", [0.0, 0.0, 0.5], 0.4, 1.0);
    combined(&mut model, ("union", "u"), ball, boss, BooleanOp::Union);
    let z = 0.84f64.sqrt();
    let rim = edges_with_middle_where(&model, |p| {
        ((p[0] * p[0] + p[1] * p[1]).sqrt() - 0.4).abs() < 1e-6 && (p[2] - z).abs() < 1e-6
    });
    assert!(
        !rim.is_empty(),
        "no edge where the cylinder meets the sphere"
    );
    (model, rim)
}

/// The boss's rim filleted with radius 0.1: the ball, 0.1 from the
/// cylinder and outside the sphere, rolls round a circle of radius 0.5 at
/// `z = sqrt(1.1^2 - 0.5^2)`, so the blend is a piece of the torus around
/// it — checked at samples of every blend face, to within the deviation the
/// blend is held to.
#[test]
#[ignore = "slow: 4 s — run with `cargo test -- --ignored`"]
fn boss_on_sphere_fillet_is_a_torus() {
    let (mut model, rim) = boss_on_sphere();
    let r = 0.1;
    blended(&mut model, &rim[..1], &BlendShape::round(r));
    let zc = (1.1f64 * 1.1 - 0.5 * 0.5).sqrt();
    let topology = model.topology();
    let mut blend_faces = 0;
    for (&id, face) in &topology.faces {
        let name = model.name_of(id).unwrap();
        if !name.starts_with(&format!("fillet(F,{},fillet", rim[0])) {
            continue;
        }
        blend_faces += 1;
        let ((u0, u1), (v0, v1)) = (face.surface.domain_u(), face.surface.domain_v());
        for i in 0..=8 {
            for j in 0..=8 {
                let u = Scalar::interpolate(u0, u1, Scalar::from_f64(i as f64 / 8.0));
                let w = Scalar::interpolate(v0, v1, Scalar::from_f64(j as f64 / 8.0));
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

/// The faces of `model` that are pieces of the blend of edge `edge` of step
/// `F` (see [`blend`]).
fn blend_faces<'a>(model: &'a NamedModel, edge: &str) -> Vec<(&'a str, &'a NurbSurface3D)> {
    let prefix = format!("fillet(F,{edge},fillet");
    model
        .topology()
        .faces
        .iter()
        .filter_map(|(&id, face)| {
            let name = model.name_of(id)?;
            name.starts_with(&prefix).then_some((name, &face.surface))
        })
        .collect()
}

/// Samples of `surface` over its domain, as plain numbers.
fn samples(surface: &NurbSurface3D, n: usize) -> Vec<[f64; 3]> {
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    let mut out = Vec::new();
    for i in 0..=n {
        for j in 0..=n {
            let u = Scalar::interpolate(u0, u1, Scalar::from_f64(i as f64 / n as f64));
            let w = Scalar::interpolate(v0, v1, Scalar::from_f64(j as f64 / n as f64));
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
fn tee() -> (NamedModel, Vec<String>) {
    let mut model = NamedModel::new();
    let pipe = revolved_cylinder_along_axis(
        &mut model,
        "p",
        v3(-1.0, 0.0, 0.0),
        Scalar::from_f64(0.5),
        Scalar::from_f64(2.0),
        Axis::X,
    )
    .unwrap();
    let branch = cylinder(&mut model, "b", [0.0, 0.0, 0.0], 0.3, 1.0);
    combined(&mut model, ("union", "u"), pipe, branch, BooleanOp::Union);
    let saddle = edges_with_middle_where(&model, |p| {
        ((p[0] * p[0] + p[1] * p[1]).sqrt() - 0.3).abs() < 1e-6
            && ((p[1] * p[1] + p[2] * p[2]).sqrt() - 0.5).abs() < 1e-6
    });
    assert!(!saddle.is_empty(), "no edge where the cylinders meet");
    (model, saddle)
}

/// The branch of the tee filleted with radius 0.1: the ball's center runs
/// 0.4 from the branch's axis and 0.6 from the pipe's — on the curve
/// `((0.4 cos t, 0.4 sin t, sqrt(0.36 - 0.16 sin^2 t))` — so every point of
/// the blend is 0.1 from that curve, checked at samples to within the
/// deviation the blend is held to.
#[test]
fn tee_fillet_rolls_around_the_saddle() {
    let (mut model, saddle) = tee();
    let r = 0.1;
    blended(&mut model, &saddle[..1], &BlendShape::round(r));
    let spine = |t: f64| {
        let (x, y) = (0.4 * t.cos(), 0.4 * t.sin());
        [x, y, (0.36 - y * y).sqrt()]
    };
    let dist = |p: [f64; 3], t: f64| {
        let c = spine(t);
        ((p[0] - c[0]).powi(2) + (p[1] - c[1]).powi(2) + (p[2] - c[2]).powi(2)).sqrt()
    };
    let faces = blend_faces(&model, &saddle[0]);
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

/// The cube's upright edge at the origin rounded from radius 0.1 where it
/// starts to 0.3 where it ends: the round meets the two sides 0.1 from the
/// edge at one end of it and 0.3 at the other, and the blend is a cone
/// between.
#[test]
fn variable_radius_on_a_straight_edge() {
    let mut model = unit_cube();
    let edge = "cube(b,p0)";
    let id = model.edge_id(edge).unwrap();
    let e = model.topology().get_edge(id).unwrap();
    let z = |v| model.topology().get_vertex(v).unwrap().point[2].to_f64();
    let (z_start, z_end) = (z(e.start_vertex), z(e.end_vertex));
    let shape = BlendShape::Fillet {
        radii: Radii {
            radius: 0.1,
            end_radius: Some(0.3),
            at_vertices: Vec::new(),
        },
    };
    blended(&mut model, &[edge.to_string()], &shape);
    for (r, z) in [(0.1, z_start), (0.3, z_end)] {
        for p in [[r, 0.0, z], [0.0, r, z]] {
            assert!(has_vertex(&model, p), "no vertex at {p:?}");
        }
    }
    // Every point of the blend is as far from the edge's diagonal plane's
    // ball centers as the radius there: `(x - r, y - r)` of length `r`.
    for (name, surface) in blend_faces(&model, edge) {
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
fn d_shaft() -> NamedModel {
    shaft_with_flat(0.5, 1.0, 0.3)
}

/// A cylinder of `radius` and `height` standing on the origin, with a flat
/// cut along it at `x = flat`.
fn shaft_with_flat(radius: f64, height: f64, flat: f64) -> NamedModel {
    let mut model = NamedModel::new();
    let (r, h) = (Scalar::from_f64(radius), Scalar::from_f64(height));
    let shaft = revolved_cylinder(&mut model, "c", v3(0.0, 0.0, 0.0), r, h).unwrap();
    let (min, max) = (
        v3(flat, -2.0 * radius, -1.0),
        v3(2.0 * radius, 2.0 * radius, 2.0),
    );
    let flat = cube_solid(&mut model, "f", min, max).unwrap();
    combined(
        &mut model,
        ("flat", "f"),
        shaft,
        flat,
        BooleanOp::Difference,
    );
    model
}

/// The edges at the D-shaft's two top corners rounded: the flat's top
/// edge, straight between planes, swept; the top's rim and the flat's
/// sides, each between a plane and the cylinder, rolled. All end at the
/// balls touching the top, the flat and the cylinder, whose pieces round
/// the corners.
#[test]
fn d_shaft_fillet_top_corners() {
    let mut model = d_shaft();
    let y = 0.16f64.sqrt();
    let topology = model.topology();
    let corners: Vec<_> = topology
        .vertices
        .iter()
        .filter(|(_, vertex)| {
            [y, -y]
                .iter()
                .any(|&y| vertex.point.could_be_equal(&v3(0.3, y, 1.0)))
        })
        .map(|(&id, _)| id)
        .collect();
    assert_eq!(corners.len(), 2);
    let mut edges: Vec<String> = topology
        .edges
        .iter()
        .filter(|(_, e)| corners.contains(&e.start_vertex) || corners.contains(&e.end_vertex))
        .map(|(&id, _)| model.name_of(id).unwrap().to_string())
        .collect();
    edges.sort();
    edges.dedup();
    let r = 0.1;
    blended(&mut model, &edges, &BlendShape::round(r));
    // `r` below the top, from the flat and inside the cylinder.
    let y = (0.16f64 - 0.04).sqrt();
    let centers = [y, -y].map(|y| v3(0.3 - r, y, 1.0 - r));
    let balls: Vec<_> = model
        .topology()
        .faces
        .iter()
        .filter(|&(&id, _)| model.name_of(id).unwrap().ends_with(",corner)"))
        .collect();
    assert_eq!(balls.len(), 2);
    for (_, face) in balls {
        let ps = samples(&face.surface, 4);
        let near = |c: &Vector3| ps[0][1] * c[1].to_f64() > 0.0;
        let center = centers.iter().find(|c| near(c)).unwrap();
        for p in ps {
            let off = v3(p[0], p[1], p[2]).sub(center).norm().to_f64() - r;
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
    let mut model = d_shaft();
    let y = 0.16f64.sqrt();
    let side = edges_with_middle_where(&model, |p| {
        (p[0] - 0.3).abs() < 1e-6 && (p[1] - y).abs() < 1e-6
    });
    assert_eq!(side.len(), 1, "{side:?}");
    let d = 0.1;
    blended(
        &mut model,
        &side,
        &BlendShape::Chamfer { distances: [d, d] },
    );
    let faces: Vec<&str> = model
        .topology()
        .faces
        .keys()
        .filter_map(|&f| model.name_of(f))
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
        let p = v3(p[0], p[1], p[2]);
        let near = model
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
    let (mut model, saddle) = tee();
    blended(
        &mut model,
        &saddle[..1],
        &BlendShape::Chamfer {
            distances: [0.1, 0.1],
        },
    );
    let chamfered = model
        .topology()
        .faces
        .keys()
        .filter_map(|&f| model.name_of(f))
        .any(|n| n.starts_with(&format!("fillet(F,{},chamfer", saddle[0])));
    assert!(chamfered);
}

/// A block of 2 x 2 x 1 with a D-shaped pocket 0.4 deep in its top: a
/// circle of radius 0.5 around `(1, 1)` cut off at `x = 1.3`.
fn d_pocket() -> NamedModel {
    let mut model = NamedModel::new();
    let stock = block(&mut model, "b", [0.0, 0.0, 0.0], [2.0, 2.0, 1.0]);
    let round = cylinder(&mut model, "c", [1.0, 1.0, 0.6], 0.5, 1.0);
    let off = block(&mut model, "o", [1.3, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let pocket = combined(&mut model, ("cut", "d"), round, off, BooleanOp::Difference);
    combined(
        &mut model,
        ("cut", "p"),
        stock,
        pocket,
        BooleanOp::Difference,
    );
    model
}

/// The D-shaped pocket's rim rounded: the arc rolled, the flat side swept,
/// meeting at two inward corners, each blend ending flush with the other's
/// wall. Each corner is cut by the two run on past it, intersected: what
/// both would cut there. (Mitring them in the plane halving the corner,
/// exact only where the two sections mirror each other in it, was refused
/// for a rolled blend.)
#[test]
fn d_pocket_rim_fillet() {
    let mut model = d_pocket();
    let rim = edges_with_middle_where(&model, |p| {
        (p[2] - 1.0).abs() < 1e-6 && (p[0] - 1.0).hypot(p[1] - 1.0) < 0.51
    });
    blended(&mut model, &rim, &BlendShape::round(0.1));
    // The rim's corner where the arc meets the flat at `(1.3, 1.4, 1)`:
    // cut round on both sides of it, just below the top.
    let at = |p: [f64; 3]| geop_ops_model::testing::classify(&model, "fillet(F)", p);
    use geop_core_topology::contains::shell::PointClassification as P;
    assert_eq!(at([1.31, 1.2, 0.995]), P::Outside);
    assert_eq!(at([1.31, 1.2, 0.8]), P::Inside);
    assert_eq!(at([0.49, 1.0, 0.995]), P::Outside);
}

/// A sphere of radius 1 around the origin, its half below `z = 0` cut off.
fn hemisphere() -> NamedModel {
    let mut model = NamedModel::new();
    let ball = sphere_solid(&mut model, "s", v3(0.0, 0.0, 0.0), Scalar::ONE).unwrap();
    let below = block(&mut model, "c", [-2.0, -2.0, -2.0], [2.0, 2.0, 0.0]);
    combined(
        &mut model,
        ("half", "h"),
        ball,
        below,
        BooleanOp::Difference,
    );
    model
}

/// The hemisphere's rim bevelled: a closed chain of quarter circles
/// between the flat and the sphere's quarters, the chamfer's point on the
/// sphere moving from quarter to quarter across their seams.
#[test]
fn hemisphere_rim_chamfer() {
    let mut model = hemisphere();
    let rim = edges_with_middle_where(&model, |p| p[2].abs() < 1e-6);
    assert!(!rim.is_empty());
    blended(
        &mut model,
        &rim[..1],
        &BlendShape::Chamfer {
            distances: [0.1, 0.1],
        },
    );
}

/// A cylinder of radius 1 and height 1.5 with a flat at `x = 0.6`, its
/// flat's side bevelled.
#[test]
fn wide_d_shaft_chamfer_flat_side() {
    let mut model = shaft_with_flat(1.0, 1.5, 0.6);
    let side = edges_with_middle_where(&model, |p| {
        (p[0] - 0.6).abs() < 1e-6 && (p[1] - 0.8).abs() < 1e-6
    });
    assert_eq!(side.len(), 1, "{side:?}");
    blended(
        &mut model,
        &side,
        &BlendShape::Chamfer {
            distances: [0.1, 0.1],
        },
    );
}

/// The three edges at the cube's corner `(0, 0, 1)` rounded with a radius
/// growing from 0.1 at their far ends to 0.2 at the corner, given a radius
/// of its own: rolled, each reaching 0.2 where it meets the corner's ball,
/// whose piece rounds the corner.
#[test]
fn cube_corner_rounded_with_a_varying_radius() {
    let mut model = unit_cube();
    let corner = vertex_at(&model, [0.0, 0.0, 1.0]);
    let edges: Vec<String> = ["cube(b,c0,start)", "cube(b,c3,start)", "cube(b,p0)"]
        .map(String::from)
        .to_vec();
    let shape = BlendShape::Fillet {
        radii: Radii {
            radius: 0.1,
            end_radius: None,
            at_vertices: vec![(corner, 0.2)],
        },
    };
    blended(&mut model, &edges, &shape);
    let balls: Vec<_> = model
        .topology()
        .faces
        .iter()
        .filter(|&(&id, _)| model.name_of(id).unwrap().ends_with(",corner)"))
        .collect();
    assert_eq!(balls.len(), 1);
    let center = v3(0.2, 0.2, 0.8);
    for p in samples(&balls[0].1.surface, 4) {
        let off = v3(p[0], p[1], p[2]).sub(&center).norm().to_f64() - 0.2;
        assert!(off.abs() <= 1e-9, "{p:?} is {off:e} off the ball");
    }
}

/// The same edges with a radius from 0.1 to 0.2 along each: they meet the
/// corner with different radii, as they run to or from it. There is no one
/// ball to round it with: the corner is bevelled, each fillet run out past
/// it and the boolean meeting them.
#[test]
fn cube_corner_with_mixed_radii_is_bevelled() {
    let model = unit_cube();
    let edges: Vec<String> = ["cube(b,c0,start)", "cube(b,c3,start)", "cube(b,p0)"]
        .map(String::from)
        .to_vec();
    let shape = BlendShape::Fillet {
        radii: Radii {
            radius: 0.1,
            end_radius: Some(0.2),
            at_vertices: Vec::new(),
        },
    };
    let mut model = model;
    blended(&mut model, &edges, &shape);
    assert!(
        !(model.topology().faces.keys()).any(|&f| model.name_of(f).unwrap().ends_with(",corner)"))
    );
    assert!(!has_vertex(&model, [0.0, 0.0, 1.0]));
}
