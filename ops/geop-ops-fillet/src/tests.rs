//! Blending the edges of basic solids.

use geop_core_math::scalars::{Ring, ScalInF64 as S, Scalar};
use geop_core_topology::SolidId;
use geop_ops_booleans::{
    boolean::{BooleanOp, boolean},
    remesh::remesh::RemeshParams,
};
use geop_ops_extrude_revolve::shapes::{self, block, block_part, v3};
use geop_ops_model::{
    NamedModel, Namer,
    testing::{assert_valid, edges_where, edges_with_middle_where, has_vertex},
};

use crate::blend::{BlendShape, blend};

/// The unit cube `[0, 1]^3`, named `cube(b)`.
pub(super) fn unit_cube() -> NamedModel<S> {
    block_part("b", [0.0, 0.0, 0.0], [1.0, 1.0, 1.0])
}

/// A cylinder of radius 0.5 and height 1 standing on the origin, named
/// `cylinder(c)`.
fn cylinder() -> NamedModel<S> {
    let mut model = NamedModel::new();
    shapes::cylinder(&mut model, "c", [0.0, 0.0, 0.0], 0.5, 1.0);
    model
}

/// `a` and `b` of `model` combined by `op`, as the step `step` of the
/// operation `operation`.
pub(super) fn combined(
    model: &mut NamedModel<S>,
    (operation, step): (&str, &str),
    a: SolidId,
    b: SolidId,
    op: BooleanOp,
) -> SolidId {
    let namer = Namer::new(operation, step).unwrap();
    boolean(model, &namer, a, b, op, RemeshParams::default())
        .unwrap()
        .unwrap()
}

/// Blends `edges` of `model` into `shape` as the step `F`, and checks the
/// result is a valid solid named `fillet(F)`.
fn blended(mut model: NamedModel<S>, edges: &[impl ToString], shape: BlendShape) -> NamedModel<S> {
    let edges: Vec<String> = edges.iter().map(|e| e.to_string()).collect();
    let namer = Namer::new("fillet", "F").unwrap();
    blend(&mut model, &namer, &edges, &shape).unwrap();
    assert_valid(&model);
    assert_eq!(model.solid_names(), vec!["fillet(F)".to_string()]);
    model
}

/// Checks `model` has a face named `name`, saying which it has if not.
fn assert_has_face(model: &NamedModel<S>, name: &str) {
    let faces: Vec<&str> = model
        .topology()
        .faces
        .keys()
        .filter_map(|&f| model.name_of(f))
        .collect();
    assert!(
        model.face_id(name).is_ok(),
        "no face {name:?}, only {faces:?}"
    );
}

/// One vertical edge of the cube rounded: one round face more, touching
/// the two sides `0.2` from the edge, which is gone.
#[test]
fn cube_fillet_one_edge() {
    let model = blended(unit_cube(), &["cube(b,p0)"], BlendShape::round(0.2));
    assert_eq!(model.topology().faces.len(), 7);
    assert_has_face(&model, "fillet(F,cube(b,p0),fillet)");
    for p in [
        [0.2, 0.0, 0.0],
        [0.0, 0.2, 0.0],
        [0.2, 0.0, 1.0],
        [0.0, 0.2, 1.0],
    ] {
        assert!(has_vertex(&model, p), "no vertex at {p:?}");
    }
    assert!(!has_vertex(&model, [0.0, 0.0, 0.0]));
}

/// One edge of the cube's top, along `x` at `y = 0`, bevelled alike and
/// with two distances: the first into the face on the edge's left — the
/// top, for this edge — and the second into the side.
#[test]
fn cube_chamfer_one_edge() {
    for [top, side] in [[0.2, 0.2], [0.1, 0.3]] {
        let model = blended(
            unit_cube(),
            &["cube(b,c0,start)"],
            BlendShape::Chamfer {
                distances: [top, side],
            },
        );
        assert_eq!(model.topology().faces.len(), 7);
        assert_has_face(&model, "fillet(F,cube(b,c0,start),chamfer)");
        for x in [0.0, 1.0] {
            for p in [[x, top, 1.0], [x, 0.0, 1.0 - side]] {
                assert!(has_vertex(&model, p), "no vertex at {p:?}");
            }
        }
    }
}

/// Two parallel edges of the cube, rounded one after the other.
#[test]
fn cube_fillet_two_parallel_edges() {
    let model = blended(
        unit_cube(),
        &["cube(b,p0)", "cube(b,p2)"],
        BlendShape::round(0.3),
    );
    assert_eq!(model.topology().faces.len(), 8);
}

/// The cylinder's top rim rounded: picking one quarter arc blends the whole
/// circle, and picking two of them does too, once.
#[test]
fn cylinder_rim_fillet() {
    for edges in [
        vec!["cylinder(c,p1,q0)"],
        vec!["cylinder(c,p1,q0)", "cylinder(c,p1,q2)"],
    ] {
        let model = blended(cylinder(), &edges, BlendShape::round(0.1));
        assert_has_face(&model, "fillet(F,cylinder(c,p1,q0),fillet,q0)");
        let on_circle = |r: f64, z: f64| {
            model.topology().vertices.values().any(|vertex| {
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

/// Why blending `edges` of `model` into `shape` is refused, which it has to
/// be.
pub(super) fn refusal(
    mut model: NamedModel<S>,
    edges: &[impl ToString],
    shape: BlendShape,
) -> String {
    let namer = Namer::new("fillet", "F").unwrap();
    let edges: Vec<String> = edges.iter().map(|e| e.to_string()).collect();
    match blend(&mut model, &namer, &edges, &shape) {
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
    refusal(unit_cube(), &[] as &[&str], round(0.1));
    refusal(unit_cube(), &["nothing"], round(0.1));
    let past = refusal(unit_cube(), &["cube(b,p0)"], round(1.5));
    assert!(past.contains("too large"), "{past}");
    let on_edge = refusal(unit_cube(), &["cube(b,p0)"], bevel(1.0));
    assert!(on_edge.contains("exactly on another edge"), "{on_edge}");
    let wide = refusal(cylinder(), &["cylinder(c,p1,q0)"], round(0.6));
    assert!(wide.contains("too large"), "{wide}");
}

/// Checks the faces of `model` whose names end in `corner)` are pieces of
/// spheres of `radius`, one around each of `centers`, sampled over them.
fn assert_corner_balls(model: &NamedModel<S>, radius: f64, centers: &[[f64; 3]]) {
    let mut found = vec![0; centers.len()];
    for (&id, face) in &model.topology().faces {
        let name = model.name_of(id).unwrap();
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
        let distance = |c: [f64; 3]| near.sub(&v3(c[0], c[1], c[2])).norm().to_f64();
        let k = (0..centers.len())
            .min_by(|&a, &b| distance(centers[a]).total_cmp(&distance(centers[b])))
            .unwrap();
        found[k] += 1;
        let c = v3::<S>(centers[k][0], centers[k][1], centers[k][2]);
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
    let model = blended(
        unit_cube(),
        &["cube(b,c0,start)", "cube(b,c3,start)", "cube(b,p0)"],
        BlendShape::round(0.2),
    );
    assert_eq!(model.topology().faces.len(), 10);
    assert_corner_balls(&model, 0.2, &[[0.2, 0.2, 0.8]]);
    for p in [[0.0, 0.2, 0.8], [0.2, 0.0, 0.8], [0.2, 0.2, 1.0]] {
        assert!(has_vertex(&model, p), "no vertex at {p:?}");
    }
}

/// All twelve edges of the cube rounded: a ball's octant at every corner.
#[test]
fn cube_fillet_all_edges() {
    let model = unit_cube();
    let edges = edges_where(&model, |_| true);
    let r = 0.2;
    let model = blended(model, &edges, BlendShape::round(r));
    assert_eq!(model.topology().faces.len(), 26);
    let mut centers = Vec::new();
    for x in [r, 1.0 - r] {
        for y in [r, 1.0 - r] {
            for z in [r, 1.0 - r] {
                centers.push([x, y, z]);
            }
        }
    }
    assert_corner_balls(&model, r, &centers);
}

/// A block of 2 x 2 x 1 with a pocket of 1 x 1, 0.4 deep, in its top.
fn pocketed() -> NamedModel<S> {
    let mut model = NamedModel::new();
    let stock = block(&mut model, "b", [0.0, 0.0, 0.0], [2.0, 2.0, 1.0]);
    let pocket = block(&mut model, "p", [0.5, 0.5, 0.6], [1.5, 1.5, 1.5]);
    combined(
        &mut model,
        ("pocket", "P"),
        stock,
        pocket,
        BooleanOp::Difference,
    );
    model
}

/// The pocket's floor and upright edges rounded: concave, so the fillets
/// fill material in, and so does the ball's piece in each of the floor's
/// corners.
#[test]
fn pocket_floor_fillet() {
    let model = pocketed();
    let inside = |p: [f64; 3]| (0.4..=1.6).contains(&p[0]) && (0.4..=1.6).contains(&p[1]);
    let edges = edges_with_middle_where(&model, |p| inside(p) && p[2] < 0.99);
    assert_eq!(edges.len(), 8, "{edges:?}");
    let r = 0.1;
    let model = blended(model, &edges, BlendShape::round(r));
    let mut centers = Vec::new();
    for x in [0.5 + r, 1.5 - r] {
        for y in [0.5 + r, 1.5 - r] {
            centers.push([x, y, 0.6 + r]);
        }
    }
    assert_corner_balls(&model, r, &centers);
}

/// Every edge of the pocketed block rounded is refused: at the rim's
/// corners the rim is cut away and the upright edge filled in.
#[test]
fn pocketed_block_fillet_every_edge_is_refused() {
    let model = pocketed();
    let edges = edges_where(&model, |_| true);
    let why = refusal(model, &edges, BlendShape::round(0.1));
    assert!(why.contains("bend the same way"), "{why}");
}

/// Every edge of the pocketed block but the pocket's upright ones rounded:
/// the block's corners by a ball's piece, the pocket's rim by mitred
/// fillets, its floor by fillets filling in up to the walls.
#[test]
#[ignore = "slow: twenty edges blended — run with `cargo test -- --ignored`"]
fn pocketed_block_fillet_all_but_upright_edges() {
    let model = pocketed();
    let upright = |p: [f64; 3]| {
        let on = |x: f64| (x - 0.5).abs() < 1e-9 || (x - 1.5).abs() < 1e-9;
        on(p[0]) && on(p[1])
    };
    let edges = edges_with_middle_where(&model, |p| !upright(p));
    assert_eq!(edges.len(), edges_where(&model, |_| true).len() - 4);
    let r = 0.1;
    let built = blended(model, &edges, BlendShape::round(r));
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
fn plate_on_base() -> NamedModel<S> {
    let mut model = NamedModel::new();
    let base = block(&mut model, "b", [0.0, 0.0, 0.0], [4.0, 4.0, 1.0]);
    let plate = block(&mut model, "p", [0.0, 0.0, 1.0], [1.0, 4.0, 3.0]);
    combined(&mut model, ("join", "J"), base, plate, BooleanOp::Union);
    model
}

/// The edge where the plate meets the base's top, rounded: it ends at the
/// flush sides, at corners of four faces, two of them one plane (reported
/// 2026-10-06 on a plate whose side was flush with the part it stands on:
/// "the edge ends at vertex ..., where 2 more faces meet"). The two faces
/// are one wall, and the fillet runs flush up to it.
#[test]
fn fillet_up_to_a_wall_two_faces_make() {
    let model = plate_on_base();
    let edges = edges_with_middle_where(&model, |p| {
        (p[0] - 1.0).abs() < 1e-9 && (p[1] - 2.0).abs() < 1e-9 && (p[2] - 1.0).abs() < 1e-9
    });
    assert_eq!(edges.len(), 1, "{edges:?}");
    let model = blended(model, &edges, BlendShape::round(0.2));
    // Where the fillet meets the sides it ends flush: its corners are at
    // the sides' planes.
    for y in [0.0, 4.0] {
        assert!(
            has_vertex(&model, [1.2, y, 1.0]),
            "no vertex at x 1.2, y {y}"
        );
    }
}

/// A bar of the press handle's size, radius 3 and 236 long with its end
/// 461 from the origin, its rim rounded. The rounded bar's cap is left as
/// four quarters of radius 2 meeting at the centre, and validating it asks
/// whether each quarter's radial edge crosses the neighbouring quarter's
/// surface, which it only touches at that surface's pole. Clipping closed in
/// on the pole by a constant factor without end (see `fat_line::restriction`
/// and the curve-surface test `blended_spoke_touching_cap_quadrant_at_its_pole`).
#[test]
fn far_bar_rim_fillet() {
    use geop_ops_extrude_revolve::shapes::cylinder::{Axis, revolved_cylinder_along_axis};
    for (axis, base) in [
        (Axis::X, v3(225.0, 0.0, 0.0)),
        (Axis::Z, v3(0.0, 0.0, 225.0)),
    ] {
        let mut model = NamedModel::new();
        revolved_cylinder_along_axis(
            &mut model,
            "c",
            base,
            S::from_f64(3.0),
            S::from_f64(236.0),
            axis,
        )
        .unwrap();
        blended(model, &["cylinder(c,p1,q0)"], BlendShape::round(1.0));
    }
}
