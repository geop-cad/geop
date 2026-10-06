//! Blending the edges of basic solids.

use geop_core_math::{
    scalars::{Ring, ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_topology::validation::{ValidationParameters, validate, validate_manifold};
use geop_ops::{Namer, Part};
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::shapes::{cube::cube_solid, cylinder::revolved_cylinder};

use crate::blend::{BlendShape, blend};

fn v<T: Scalar>(x: f64, y: f64, z: f64) -> Vector3<T> {
    Vector3::from_array([T::from_f64(x), T::from_f64(y), T::from_f64(z)])
}

/// The unit cube `[0, 1]^3`, named `cube(b)`.
fn unit_cube() -> Part<S> {
    let mut part = Part::new();
    cube_solid(&mut part, "b", v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)).unwrap();
    part
}

/// A cylinder of radius 0.5 and height 1 standing on the origin, named
/// `cylinder(c)`.
fn cylinder() -> Part<S> {
    let mut part = Part::new();
    revolved_cylinder(&mut part, "c", v(0.0, 0.0, 0.0), S::from_f64(0.5), S::ONE).unwrap();
    part
}

/// Blends `edges` of `part` into `shape` as the step `F`, and checks the
/// result is a valid solid named `fillet(F)`.
fn blended(mut part: Part<S>, edges: &[&str], shape: BlendShape) -> Part<S> {
    let edges: Vec<String> = edges.iter().map(|e| e.to_string()).collect();
    let namer = Namer::new("fillet", "F").unwrap();
    blend(&mut part, &namer, &edges, &shape).unwrap();
    let params = ValidationParameters::default();
    if let Err(e) = validate(&params, part.topology()) {
        panic!("{e:?}");
    }
    if let Err(e) = validate_manifold(&params, part.topology()) {
        panic!("{e:?}");
    }
    assert_eq!(part.solid_names(), vec!["fillet(F)".to_string()]);
    part
}

/// Checks `part` has a face named `name`, saying which it has if not.
fn assert_has_face(part: &Part<S>, name: &str) {
    let faces: Vec<&str> = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .collect();
    assert!(
        part.face_id(name).is_ok(),
        "no face {name:?}, only {faces:?}"
    );
}

/// Whether `part` has a vertex that could be at `p`.
fn has_vertex(part: &Part<S>, p: [f64; 3]) -> bool {
    let p = v::<S>(p[0], p[1], p[2]);
    part.topology()
        .vertices
        .values()
        .any(|vertex| vertex.point.could_be_equal(&p))
}

/// One vertical edge of the cube rounded: one round face more, touching
/// the two sides `0.2` from the edge, which is gone.
#[test]
fn cube_fillet_one_edge() {
    let part = blended(unit_cube(), &["cube(b,p0)"], BlendShape::round(0.2));
    assert_eq!(part.topology().faces.len(), 7);
    assert_has_face(&part, "fillet(F,cube(b,p0),fillet)");
    for p in [
        [0.2, 0.0, 0.0],
        [0.0, 0.2, 0.0],
        [0.2, 0.0, 1.0],
        [0.0, 0.2, 1.0],
    ] {
        assert!(has_vertex(&part, p), "no vertex at {p:?}");
    }
    assert!(!has_vertex(&part, [0.0, 0.0, 0.0]));
}

/// One edge of the cube's top, along `x` at `y = 0`, bevelled alike and
/// with two distances: the first into the face on the edge's left — the
/// top, for this edge — and the second into the side.
#[test]
fn cube_chamfer_one_edge() {
    for [top, side] in [[0.2, 0.2], [0.1, 0.3]] {
        let part = blended(
            unit_cube(),
            &["cube(b,c0,start)"],
            BlendShape::Chamfer {
                distances: [top, side],
            },
        );
        assert_eq!(part.topology().faces.len(), 7);
        assert_has_face(&part, "fillet(F,cube(b,c0,start),chamfer)");
        for x in [0.0, 1.0] {
            for p in [[x, top, 1.0], [x, 0.0, 1.0 - side]] {
                assert!(has_vertex(&part, p), "no vertex at {p:?}");
            }
        }
    }
}

/// Two parallel edges of the cube, rounded one after the other.
#[test]
fn cube_fillet_two_parallel_edges() {
    let part = blended(
        unit_cube(),
        &["cube(b,p0)", "cube(b,p2)"],
        BlendShape::round(0.3),
    );
    assert_eq!(part.topology().faces.len(), 8);
}

/// The cylinder's top rim rounded: picking one quarter arc blends the whole
/// circle, and picking two of them does too, once.
#[test]
fn cylinder_rim_fillet() {
    for edges in [
        vec!["cylinder(c,p1,q0)"],
        vec!["cylinder(c,p1,q0)", "cylinder(c,p1,q2)"],
    ] {
        let part = blended(cylinder(), &edges, BlendShape::round(0.1));
        assert_has_face(&part, "fillet(F,cylinder(c,p1,q0),fillet,q0)");
        let on_circle = |r: f64, z: f64| {
            part.topology().vertices.values().any(|vertex| {
                let p = vertex.point;
                p[2].could_be_equal(S::from_f64(z))
                    && p[0]
                        .mul(p[0])
                        .add(p[1].mul(p[1]))
                        .could_be_equal(S::from_f64(r * r))
            })
        };
        assert!(on_circle(0.4, 1.0) && on_circle(0.5, 0.9));
        assert!(!on_circle(0.5, 1.0));
    }
}

/// Both rims of the cylinder rounded, one after the other.
#[test]
fn cylinder_both_rims_fillet() {
    blended(
        cylinder(),
        &["cylinder(c,p1,q0)", "cylinder(c,p2,q0)"],
        BlendShape::round(0.2),
    );
}

/// The four edges around the cube's top rounded: each pair meets at a
/// corner, where the second blend crosses the first.
#[test]
fn cube_fillet_top_edges() {
    blended(
        unit_cube(),
        &[
            "cube(b,c0,start)",
            "cube(b,c1,start)",
            "cube(b,c2,start)",
            "cube(b,c3,start)",
        ],
        BlendShape::round(0.2),
    );
}

/// Three edges meeting at one corner of the cube bevelled.
#[test]
fn cube_chamfer_corner_edges() {
    blended(
        unit_cube(),
        &["cube(b,c0,start)", "cube(b,c3,start)", "cube(b,p0)"],
        BlendShape::Chamfer {
            distances: [0.2, 0.2],
        },
    );
}

/// The cylinder's bottom rim bevelled.
#[test]
fn cylinder_rim_chamfer() {
    blended(
        cylinder(),
        &["cylinder(c,p2,q1)"],
        BlendShape::Chamfer {
            distances: [0.1, 0.1],
        },
    );
}

/// Why blending `edges` of `part` into `shape` is refused, which it has to
/// be.
fn refusal(mut part: Part<S>, edges: &[&str], shape: BlendShape) -> String {
    let namer = Namer::new("fillet", "F").unwrap();
    let edges: Vec<String> = edges.iter().map(|e| e.to_string()).collect();
    match blend(&mut part, &namer, &edges, &shape) {
        Ok(()) => panic!("blending {edges:?} into {shape:?} is not refused"),
        Err(e) => format!("{e:?}"),
    }
}

/// What is refused: a radius that is not positive, no edges, an edge that
/// is not there, and blends too large for the faces they run into — past
/// their boundary, or ending exactly on the cube's opposite edge.
#[test]
fn refuses_what_it_cannot_blend() {
    let round = |radius| BlendShape::round(radius);
    let bevel = |d| BlendShape::Chamfer { distances: [d, d] };
    refusal(unit_cube(), &["cube(b,p0)"], round(0.0));
    refusal(unit_cube(), &[], round(0.1));
    refusal(unit_cube(), &["nothing"], round(0.1));
    let past = refusal(unit_cube(), &["cube(b,p0)"], round(1.5));
    assert!(past.contains("too large"), "{past}");
    let on_edge = refusal(unit_cube(), &["cube(b,p0)"], bevel(1.0));
    assert!(on_edge.contains("exactly on another edge"), "{on_edge}");
    let wide = refusal(cylinder(), &["cylinder(c,p1,q0)"], round(0.6));
    assert!(wide.contains("too large"), "{wide}");
}

/// The names of every edge of `part`, sorted.
fn all_edges(part: &Part<S>) -> Vec<String> {
    let mut edges: Vec<String> = part
        .topology()
        .edges
        .keys()
        .map(|&id| part.name_of(id).unwrap().to_string())
        .collect();
    edges.sort();
    edges
}

/// Checks the faces of `part` whose names end in `corner)` are pieces of
/// spheres of `radius`, one around each of `centers`, sampled over them.
fn assert_corner_balls(part: &Part<S>, radius: f64, centers: &[[f64; 3]]) {
    let mut found = vec![0; centers.len()];
    for (&id, face) in &part.topology().faces {
        let name = part.name_of(id).unwrap();
        if !name.ends_with(",corner)") {
            continue;
        }
        let surface = &face.surface;
        let at = |i: usize, j: usize| {
            let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
            let u = S::interpolate(u0, u1, S::from_f64(i as f64 / 4.0));
            let w = S::interpolate(v0, v1, S::from_f64(j as f64 / 4.0));
            surface.evaluate(u, w).unwrap()
        };
        let near = at(1, 1);
        let distance = |c: [f64; 3]| near.sub(&v(c[0], c[1], c[2])).norm().to_f64();
        let k = (0..centers.len())
            .min_by(|&a, &b| distance(centers[a]).total_cmp(&distance(centers[b])))
            .unwrap();
        found[k] += 1;
        let c = v::<S>(centers[k][0], centers[k][1], centers[k][2]);
        for i in 0..=4 {
            for j in 0..=4 {
                let r = at(i, j).sub(&c).norm();
                assert!(
                    r.could_be_equal(S::from_f64(radius)),
                    "{name} at ({i}, {j}) is {r:?} from {c:?}"
                );
            }
        }
    }
    assert!(
        found.iter().all(|&n| n == 1),
        "corner faces by center: {found:?}"
    );
}

/// The three edges at the cube's corner `(0, 0, 1)` rounded: their fillets
/// end where the ball of their radius touches all three faces, and its
/// piece between them rounds the corner.
#[test]
fn cube_fillet_one_corner() {
    let part = blended(
        unit_cube(),
        &["cube(b,c0,start)", "cube(b,c3,start)", "cube(b,p0)"],
        BlendShape::round(0.2),
    );
    assert_eq!(part.topology().faces.len(), 10);
    assert_corner_balls(&part, 0.2, &[[0.2, 0.2, 0.8]]);
    for p in [[0.0, 0.2, 0.8], [0.2, 0.0, 0.8], [0.2, 0.2, 1.0]] {
        assert!(has_vertex(&part, p), "no vertex at {p:?}");
    }
}

/// All twelve edges of the cube rounded: a ball's octant at every corner.
#[test]
fn cube_fillet_all_edges() {
    let part = unit_cube();
    let edges = all_edges(&part);
    let edges: Vec<&str> = edges.iter().map(String::as_str).collect();
    let r = 0.2;
    let part = blended(part, &edges, BlendShape::round(r));
    assert_eq!(part.topology().faces.len(), 26);
    let mut centers = Vec::new();
    for x in [r, 1.0 - r] {
        for y in [r, 1.0 - r] {
            for z in [r, 1.0 - r] {
                centers.push([x, y, z]);
            }
        }
    }
    assert_corner_balls(&part, r, &centers);
}

/// A block of 2 x 2 x 1 with a pocket of 1 x 1, 0.4 deep, in its top.
fn pocketed() -> Part<S> {
    let mut part = Part::new();
    let block = cube_solid(&mut part, "b", v(0.0, 0.0, 0.0), v(2.0, 2.0, 1.0)).unwrap();
    let pocket = cube_solid(&mut part, "p", v(0.5, 0.5, 0.6), v(1.5, 1.5, 1.5)).unwrap();
    let namer = Namer::new("pocket", "P").unwrap();
    boolean(
        &mut part,
        &namer,
        block,
        pocket,
        BooleanOp::Difference,
        RemeshParams::default(),
    )
    .unwrap()
    .unwrap();
    part
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

/// The pocket's floor and upright edges rounded: concave, so the fillets
/// fill material in, and so does the ball's piece in each of the floor's
/// corners.
#[test]
fn pocket_floor_fillet() {
    let part = pocketed();
    let inside = |p: [f64; 3]| (0.4..=1.6).contains(&p[0]) && (0.4..=1.6).contains(&p[1]);
    let edges = edges_where(&part, |p| inside(p) && p[2] < 0.99);
    assert_eq!(edges.len(), 8, "{edges:?}");
    let edges: Vec<&str> = edges.iter().map(String::as_str).collect();
    let r = 0.1;
    let part = blended(part, &edges, BlendShape::round(r));
    let mut centers = Vec::new();
    for x in [0.5 + r, 1.5 - r] {
        for y in [0.5 + r, 1.5 - r] {
            centers.push([x, y, 0.6 + r]);
        }
    }
    assert_corner_balls(&part, r, &centers);
}

/// Every edge of the pocketed block rounded is refused: at the rim's
/// corners the rim is cut away and the upright edge filled in.
#[test]
fn pocketed_block_fillet_every_edge_is_refused() {
    let part = pocketed();
    let edges = all_edges(&part);
    let edges: Vec<&str> = edges.iter().map(String::as_str).collect();
    let why = refusal(part, &edges, BlendShape::round(0.1));
    assert!(why.contains("bend the same way"), "{why}");
}

/// Every edge of the pocketed block but the pocket's upright ones rounded:
/// the block's corners by a ball's piece, the pocket's rim by mitred
/// fillets, its floor by fillets filling in up to the walls.
#[test]
#[ignore = "slow: twenty edges blended — run with `cargo test -- --ignored`"]
fn pocketed_block_fillet_all_but_upright_edges() {
    let part = pocketed();
    let upright = |p: [f64; 3]| {
        let on = |x: f64| (x - 0.5).abs() < 1e-9 || (x - 1.5).abs() < 1e-9;
        on(p[0]) && on(p[1])
    };
    let edges = edges_where(&part, |p| !upright(p));
    assert_eq!(edges.len(), all_edges(&part).len() - 4);
    let edges: Vec<&str> = edges.iter().map(String::as_str).collect();
    let r = 0.1;
    let built = blended(part, &edges, BlendShape::round(r));
    let mut centers = Vec::new();
    for x in [r, 2.0 - r] {
        for y in [r, 2.0 - r] {
            for z in [r, 1.0 - r] {
                centers.push([x, y, z]);
            }
        }
    }
    assert_corner_balls(&built, r, &centers);
}

/// A 4 x 4 x 1 base with a plate 1 thick and 2 high standing on it, flush
/// with its sides: the union leaves each side two faces in one plane.
fn plate_on_base() -> Part<S> {
    let mut part = Part::new();
    let base = cube_solid(&mut part, "b", v(0.0, 0.0, 0.0), v(4.0, 4.0, 1.0)).unwrap();
    let plate = cube_solid(&mut part, "p", v(0.0, 0.0, 1.0), v(1.0, 4.0, 3.0)).unwrap();
    let namer = Namer::new("join", "J").unwrap();
    boolean(
        &mut part,
        &namer,
        base,
        plate,
        BooleanOp::Union,
        RemeshParams::default(),
    )
    .unwrap()
    .unwrap();
    part
}

/// The edge where the plate meets the base's top, rounded: it ends at the
/// flush sides, at corners of four faces, two of them one plane (reported
/// 2026-10-06 on a plate whose side was flush with the part it stands on:
/// "the edge ends at vertex ..., where 2 more faces meet"). The two faces
/// are one wall, and the fillet runs flush up to it.
#[test]
fn fillet_up_to_a_wall_two_faces_make() {
    let part = plate_on_base();
    let edges = edges_where(&part, |p| {
        (p[0] - 1.0).abs() < 1e-9 && (p[1] - 2.0).abs() < 1e-9 && (p[2] - 1.0).abs() < 1e-9
    });
    assert_eq!(edges.len(), 1, "{edges:?}");
    let edges: Vec<&str> = edges.iter().map(String::as_str).collect();
    let part = blended(part, &edges, BlendShape::round(0.2));
    // Where the fillet meets the sides it ends flush: its corners are at
    // the sides' planes.
    for y in [0.0, 4.0] {
        assert!(
            has_vertex(&part, [1.2, y, 1.0]),
            "no vertex at x 1.2, y {y}"
        );
    }
}
