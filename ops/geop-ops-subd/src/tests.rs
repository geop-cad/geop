//! The limit surfaces of cages: closed ones are valid solids, open ones
//! sheets; where the cage is regular the surface is exactly the bicubic
//! B-spline Catmull–Clark converges to; creases stay sharp; and every
//! edit leaves a cage whose limit is valid.

use geop_core_math::{scalars::Scalar, vector::Vector3};
use geop_core_topology::{
    Body,
    validation::{ValidationParameters, validate},
};
use geop_ops_model::{NamedModel, testing::assert_valid};

use crate::{Cage, Mirror, SubdArgs, build_subd, cage::undirected};

/// The model one subd step `s` of `args` builds.
fn subd(args: &SubdArgs) -> geop_core_math::geop_error::GeopResult<NamedModel> {
    let mut model = NamedModel::new();
    build_subd(&mut model, "s", args)?;
    Ok(model)
}

/// The limit of `cage`, mirrored in `mirror`, built as step `s`.
fn built(cage: Cage, mirror: Mirror) -> NamedModel {
    subd(&SubdArgs { cage, mirror }).unwrap_or_else(|e| panic!("{e}"))
}

fn faces(part: &NamedModel) -> usize {
    part.topology().faces.len()
}

fn v(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

/// A box cage: one face per cage face, one edge per cage edge, one vertex
/// per cage vertex, closed into a valid solid.
#[test]
fn box_cage_is_a_valid_solid() {
    let part = built(Cage::cuboid([2.0, 2.0, 2.0]), Mirror::None);
    assert_valid(&part);
    let solid = part.solid_id("subd(s)").unwrap();
    assert_eq!(
        part.topology()
            .body_faces(Body::Solid(solid))
            .unwrap()
            .len(),
        6
    );
    assert_eq!(part.topology().edges.len(), 12);
    assert_eq!(part.topology().vertices.len(), 8);
    // Cage face 13 is the box's left side, cage vertex 6 its corner at
    // (1, 1, 1).
    part.face_id("subd(s,f13)").unwrap();
    part.vertex_id("subd(s,v6)").unwrap();
    part.edge_id("subd(s,e2-6)").unwrap();
}

/// A wavy 5 x 5 plane: around its middle face every vertex is regular, so
/// the limit there is the uniform bicubic B-spline of the cage — at the
/// face's centre, the tensor product of the cubic B-spline's weights at
/// one half, `(1, 23, 23, 1) / 48`, over the 4 x 4 cage vertices around
/// it; at a vertex, `(16 v + 4 Σ edge neighbours + Σ diagonal ones) / 36`.
#[test]
fn regular_patch_matches_the_limit() {
    let mut cage = Cage::plane(5.0, 5);
    for vertex in &mut cage.vertices {
        let [x, y, _] = vertex.at;
        vertex.at[2] = 0.3 * (1.3 * x).sin() * (0.7 * y + 0.2).cos() + 0.1 * x * y;
    }
    // Vertex (i, j) of the 6 x 6 grid is id `6 j + i`; the face at (i, j)
    // is id `36 + 5 j + i`.
    let at = |i: usize, j: usize| v(cage.vertices[6 * j + i].at);
    let part = built(cage.clone(), Mirror::None);

    let face = part.face_id("subd(s,f48)").unwrap();
    let surface = &part.topology().faces[&face].surface;
    let w = [1.0, 23.0, 23.0, 1.0].map(|x| x / 48.0);
    let mut expected = Vector3::zero();
    for a in 0..4 {
        for b in 0..4 {
            let p = at(1 + a, 1 + b);
            expected = expected.add(&p.prod_scalar(Scalar::from_f64(w[a] * w[b])));
        }
    }
    let centre = surface.evaluate(Scalar::ONE, Scalar::ONE).unwrap();
    assert!(
        centre.could_be_equal(&expected),
        "{centre:?} vs {expected:?}"
    );

    let vertex = part.vertex_id("subd(s,v14)").unwrap();
    let limit = part.topology().vertices[&vertex].point;
    let (i, j) = (2, 2);
    let mut sum = at(i, j).prod_scalar(Scalar::from_i64(16));
    for (di, dj) in [(1, 0), (0, 1), (-1, 0), (0, -1)] {
        let p = at((i as i64 + di) as usize, (j as i64 + dj) as usize);
        sum = sum.add(&p.prod_scalar(Scalar::from_i64(4)));
    }
    for (di, dj) in [(1, 1), (-1, 1), (-1, -1), (1, -1)] {
        sum = sum.add(&at((i as i64 + di) as usize, (j as i64 + dj) as usize));
    }
    let expected = sum.map(|c| c.div(Scalar::from_i64(36)).unwrap());
    assert!(limit.could_be_equal(&expected), "{limit:?} vs {expected:?}");
}

/// The largest angles, in degrees, between the normals of the two faces of
/// any edge of `part`: sampled along every edge, and at its ends only.
fn largest_kink(part: &NamedModel) -> (f64, f64) {
    let model = part.topology();
    let (mut largest, mut at_ends): (f64, f64) = (0.0, 0.0);
    for &edge in model.edges.keys() {
        let coedges: Vec<_> = model
            .coedges
            .values()
            .filter(|c| c.geometry == geop_core_topology::CoedgeGeometry::Edge(edge))
            .collect();
        for k in 0..=8 {
            let normals: Vec<Vector3> = coedges
                .iter()
                .map(|c| {
                    let (t0, t1) = c.pcurve.domain();
                    let t = t0.add(t1.sub(t0).mul(Scalar::from_f64(k as f64 / 8.0)));
                    let t = match c.sense {
                        geop_core_topology::Sense::Forward => t,
                        geop_core_topology::Sense::Reversed => t0.add(t1).sub(t),
                    };
                    let uv = c.pcurve.evaluate(t).unwrap();
                    model.faces[&c.face].surface.normal(uv[0], uv[1]).unwrap()
                })
                .collect();
            let [a, b] = normals[..] else { continue };
            let cos = a.prod_dot(&b).to_f64() / (a.norm().to_f64() * b.norm().to_f64());
            let angle = cos.clamp(-1.0, 1.0).acos().to_degrees();
            largest = largest.max(angle);
            if k == 0 || k == 8 {
                at_ends = at_ends.max(angle);
            }
        }
    }
    (largest, at_ends)
}

/// Across the edges out of a box cage's corners — three faces each, so
/// extraordinary — the patches do not meet tangentially, but the normal
/// turns by less than three degrees (measured: 2.6). At the vertices
/// themselves every patch has the limit tangent plane, so it does not turn
/// there at all, but for rounding.
#[test]
fn normals_turn_little_at_extraordinary_vertices() {
    let part = built(Cage::cuboid([2.0, 2.0, 2.0]), Mirror::None);
    let (kink, at_vertices) = largest_kink(&part);
    assert!(kink < 3.0, "the normal turns by {kink} degrees");
    // `acos` near one turns a rounding of 1e-16 in the cosine into 1e-6
    // degrees.
    assert!(
        at_vertices < 1e-4,
        "the normal turns by {at_vertices} degrees at a vertex"
    );
}

/// A box with every edge creased is the box itself; one with only its top
/// edges creased keeps those sharp — the top face and a side meet at an
/// angle along them — while the rest rounds off.
#[test]
fn creases_stay_sharp() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    let edges: Vec<[u32; 2]> = cage.edges().into_iter().collect();
    cage.set_sharpness(&edges, 1.0);
    let part = built(cage, Mirror::None);
    assert_valid(&part);
    let corner = part.vertex_id("subd(s,v6)").unwrap();
    assert!(
        part.topology().vertices[&corner]
            .point
            .could_be_equal(&v([1.0, 1.0, 1.0]))
    );
    let top = part.face_id("subd(s,f9)").unwrap();
    let surface = &part.topology().faces[&top].surface;
    let p = surface
        .evaluate(Scalar::from_f64(0.3), Scalar::from_f64(1.7))
        .unwrap();
    assert!(p[2].could_be_equal(Scalar::ONE), "{p:?}");

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.set_sharpness(&[[4, 5], [5, 6], [6, 7], [4, 7]], 1.0);
    let part = built(cage, Mirror::None);
    assert_valid(&part);
    // Along the crease between the top (f9, u along v4 -> v5) and the
    // front (f10, its last side running from v4 to v5's ... v5 -> v4).
    let normal = |face: &str, u: f64, w: f64| {
        let id = part.face_id(face).unwrap();
        let s = &part.topology().faces[&id].surface;
        s.normal(Scalar::from_f64(u), Scalar::from_f64(w)).unwrap()
    };
    let top = normal("subd(s,f9)", 1.0, 0.0);
    let front = normal("subd(s,f10)", 1.0, 2.0);
    let cos = top
        .prod_dot(&front)
        .div(top.norm().mul(front.norm()))
        .unwrap();
    assert!(cos.to_f64() < 0.5, "the crease is not sharp: cos = {cos:?}");
}

/// A box with its top extruded, a loop inserted around it, and its sides
/// pulled in: still one valid solid.
#[test]
fn extruded_face_cage_is_valid() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.extrude(&[9].into(), 0.8, Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 10);
    let part = built(cage.clone(), Mirror::None);
    assert_valid(&part);
    assert_eq!(faces(&part), 10);

    cage.insert_loop(0, 4, 0.5).unwrap();
    assert_eq!(cage.faces.len(), 14);
    let part = built(cage, Mirror::None);
    assert_valid(&part);
}

/// A plane is an open cage, whose limit is a sheet; a prism's ends are
/// triangles, each split into a face per corner around its centre.
#[test]
fn plane_and_prism_build() {
    let part = built(Cage::plane(2.0, 3), Mirror::None);
    assert_valid(&part);
    assert!(part.solid_names().is_empty());
    assert_eq!(faces(&part), 9);

    let part = built(Cage::cylinder(1.0, 1.0, 3), Mirror::None);
    assert_valid(&part);
    assert_eq!(faces(&part), 6 + 2 * 3);
    part.face_id("subd(s,f9,v1)").unwrap();
    part.vertex_id("subd(s,f9,centre)").unwrap();
    part.edge_id("subd(s,f9,e0-1)").unwrap();
    part.edge_id("subd(s,e0-1,v0)").unwrap();
    part.vertex_id("subd(s,e0-1,mid)").unwrap();
}

/// A cylinder's ends are octagons, and a sphere-like cage has eight
/// vertices of three faces: both build valid solids.
#[test]
fn round_primitives_build() {
    let part = built(Cage::cylinder(1.0, 2.0, 8), Mirror::None);
    assert_valid(&part);
    assert_eq!(faces(&part), 16 + 2 * 8);
    part.face_id("subd(s,f24,v3)").unwrap();

    let part = built(Cage::sphere(1.0), Mirror::None);
    assert_valid(&part);
    assert_eq!(faces(&part), 24);
}

/// Half a box, mirrored in `x = 0`: the halves join smoothly along the
/// plane into one solid, symmetric about it.
#[test]
fn mirrored_cage_is_one_symmetric_solid() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.halve(0).unwrap();
    assert_eq!(cage.faces.len(), 5);
    let part = built(cage.clone(), Mirror::X);
    assert_valid(&part);
    assert_eq!(faces(&part), 10);
    let left = part.vertex_id("subd(s,v6m)").unwrap();
    let right = part.vertex_id("subd(s,v6)").unwrap();
    let (l, r) = (
        part.topology().vertices[&left].point,
        part.topology().vertices[&right].point,
    );
    assert!(l[0].neg().could_be_equal(r[0]) && l[1].could_be_equal(r[1]));

    let whole = cage.unmirrored(Mirror::X).unwrap();
    assert_eq!(whole.faces.len(), 10);
    assert_valid(&built(whole, Mirror::None));
}

/// What no limit surface can be built from is refused, naming the
/// element: a face turned against its neighbours, an inside-out cage, a
/// vertex across the mirror plane.
#[test]
fn bad_cages_are_refused() {
    let refused = |cage: Cage, mirror: Mirror| {
        subd(&SubdArgs { cage, mirror })
            .expect_err("refused")
            .to_string()
    };
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.faces[1].vertices.reverse();
    let e = refused(cage, Mirror::None);
    assert!(e.contains("f9"), "{e}");

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    for f in &mut cage.faces {
        f.vertices.reverse();
    }
    let e = refused(cage, Mirror::None);
    assert!(e.contains("inside out"), "{e}");

    let e = refused(Cage::cuboid([2.0, 2.0, 2.0]), Mirror::X);
    assert!(e.contains("v0") && e.contains("far side"), "{e}");
}

/// The arguments round-trip through JSON unchanged.
#[test]
fn args_round_trip() {
    let mut cage = Cage::cuboid([2.0, 1.0, 3.0]);
    cage.set_sharpness(&[[1, 2]], 1.0);
    let args = SubdArgs {
        cage,
        mirror: Mirror::Z,
    };
    let json = serde_json::to_string(&args).unwrap();
    let back: SubdArgs = serde_json::from_str(&json).unwrap();
    assert_eq!(back, args);
}

/// The top and front of a box extruded together, the edge between them
/// creased: the region leaves that edge behind for a new one, and the
/// crease goes with it rather than naming an edge that is gone.
#[test]
fn extruding_across_a_crease_carries_it_along() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.set_sharpness(&[[4, 5]], 1.0);
    cage.extrude(&[9, 10].into(), 0.5, Mirror::None).unwrap();
    let edges = cage.edges();
    assert_eq!(cage.creases.len(), 1);
    assert!(edges.contains(&cage.creases[0].edge), "{:?}", cage.creases);
    assert_ne!(cage.creases[0].edge, [4, 5]);
    crate::cage::Mesh::new(&cage, Mirror::None)
        .unwrap()
        .topology()
        .unwrap();
}

/// A small deterministic random number generator, for the sweep.
struct Random(u64);

impl Random {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }

    fn between(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * (self.next() % 1_000_000) as f64 / 1_000_000.0
    }
}

/// One random edit of `cage`, as the editor makes them: a vertex nudged, a
/// face extruded, inset or shrunk about its centre, a loop cut, an edge
/// creased sharp or soft, bevelled, slid or dissolved. Says what it did.
fn random_edit(cage: &mut Cage, mirror: Mirror, random: &mut Random) -> String {
    let some_edge = |cage: &Cage, random: &mut Random| {
        let edges: Vec<[u32; 2]> = cage.edges().into_iter().collect();
        edges[random.below(edges.len())]
    };
    match random.below(9) {
        5 => {
            let f = cage.faces[random.below(cage.faces.len())].id;
            let t = random.between(0.1, 0.3);
            match cage.inset(&[f].into(), t, mirror) {
                Ok(()) => format!("inset f{f} by {t}"),
                Err(e) => format!("could not inset f{f}: {e}"),
            }
        }
        6 => {
            let [a, b] = some_edge(cage, random);
            let w = random.between(0.05, 0.2);
            match cage.bevel(&[[a, b]], w, mirror) {
                Ok(()) => format!("bevelled e{a}-{b} by {w}"),
                Err(e) => format!("could not bevel e{a}-{b}: {e}"),
            }
        }
        7 => {
            let [a, b] = some_edge(cage, random);
            let s = random.between(-0.4, 0.4);
            match cage.edge_slide(&[[a, b]], s, mirror) {
                Ok(()) => format!("slid e{a}-{b} by {s}"),
                Err(e) => format!("could not slide e{a}-{b}: {e}"),
            }
        }
        8 => {
            let [a, b] = some_edge(cage, random);
            match cage.dissolve_edges(&[[a, b]], mirror) {
                Ok(()) => format!("dissolved e{a}-{b}"),
                Err(e) => format!("could not dissolve e{a}-{b}: {e}"),
            }
        }
        0 => {
            let v = cage.vertices[random.below(cage.vertices.len())].id;
            let by = [0; 3].map(|_| random.between(-0.15, 0.15));
            cage.transform(&[v].into(), mirror, |p| [0, 1, 2].map(|c| p[c] + by[c]));
            format!("moved v{v} by {by:?}")
        }
        1 => {
            let f = cage.faces[random.below(cage.faces.len())].id;
            let d = random.between(0.2, 0.6);
            match cage.extrude(&[f].into(), d, mirror) {
                Ok(()) => format!("extruded f{f} by {d}"),
                Err(e) => format!("could not extrude f{f}: {e}"),
            }
        }
        2 => {
            let [a, b] = some_edge(cage, random);
            let t = random.between(0.2, 0.8);
            match cage.insert_loop(a, b, t) {
                Ok(()) => format!("cut a loop across e{a}-{b} at {t}"),
                Err(e) => format!("could not cut a loop across e{a}-{b}: {e}"),
            }
        }
        3 => {
            let [a, b] = some_edge(cage, random);
            let sharpness = match random.below(3) {
                0 => 0.0,
                1 => 1.0,
                _ => random.between(0.2, 0.8),
            };
            cage.set_sharpness(&[[a, b]], sharpness);
            format!("creased e{a}-{b}: {sharpness}")
        }
        _ => {
            let f = cage.faces[random.below(cage.faces.len())].id;
            let s = random.between(0.6, 0.9);
            let vertices = cage.vertices_of(&[crate::cage::face_key(f)]);
            let c = cage.centre(&vertices).unwrap();
            cage.transform(&vertices, mirror, |p| {
                [0, 1, 2].map(|k| c[k] + s * (p[k] - c[k]))
            });
            format!("scaled f{f} by {s}")
        }
    }
}

/// Random sequences of edits of a box cage, whole and mirrored, each built
/// and validated: every cage the edits leave builds a valid solid.
#[test]
#[ignore = "slow: builds and validates dozens of edited cages — run with `cargo test -- --ignored`"]
fn random_cage_edits_build_valid_solids() {
    for seed in 1..=24u64 {
        let mirror = if seed % 2 == 0 {
            Mirror::X
        } else {
            Mirror::None
        };
        let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
        if let Some(axis) = mirror.axis() {
            cage.halve(axis).unwrap();
        }
        let mut random = Random(0x9E37_79B9_7F4A_7C15 ^ seed);
        let mut done = Vec::new();
        for _ in 0..6 {
            done.push(random_edit(&mut cage, mirror, &mut random));
        }
        let part = subd(&SubdArgs {
            cage: cage.clone(),
            mirror,
        })
        .unwrap_or_else(|e| panic!("seed {seed}, after {done:#?}: {e}"));
        let params = ValidationParameters::default();
        if let Err(errors) = validate(&params, part.topology()) {
            let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
            panic!("seed {seed}, after {done:#?}:\n{}", messages.join("\n"));
        }
    }
}

/// The seeds of the sweep that once failed, step by step — even seeds on a
/// mirrored half box, odd ones on a whole one, as the sweep has them: the
/// sixth inset a face too small for the thickness, which is refused now;
/// the fourteenth cut a loop across an edge with no quad to cut, refused
/// too; the twentieth dissolved an edge whose end was left on the mirror
/// plane between one face and its image, which goes with it; the
/// twenty-fourth bevelled past a cap. Each cage along the way builds a
/// valid solid, or the edit refuses by name, and the cage is as it was.
#[test]
fn sweep_seeds_that_once_failed_stay_valid_step_by_step() {
    for seed in [6u64, 7, 14, 20, 24] {
        let mirror = if seed % 2 == 0 {
            Mirror::X
        } else {
            Mirror::None
        };
        let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
        if let Some(axis) = mirror.axis() {
            cage.halve(axis).unwrap();
        }
        let mut random = Random(0x9E37_79B9_7F4A_7C15 ^ seed);
        let mut done = Vec::new();
        for _ in 0..6 {
            done.push(random_edit(&mut cage, mirror, &mut random));
            let part = subd(&SubdArgs {
                cage: cage.clone(),
                mirror,
            })
            .unwrap_or_else(|e| panic!("seed {seed}, after {done:#?}: {e}\n{cage:#?}"));
            let params = ValidationParameters::default();
            if let Err(errors) = validate(&params, part.topology()) {
                let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
                panic!(
                    "seed {seed}, after {done:#?}:\n{}\n{cage:#?}",
                    messages.join("\n")
                );
            }
        }
    }
}

/// A box's top inset: the top shrinks, keeping its id, a ring of four
/// quads fills the gap, and every corner of the inset top is the
/// thickness in from both rim edges. Extruded then, it is a boss with a
/// flat ledge around it — still one valid solid.
#[test]
fn inset_face_is_a_ring_inside_the_rim() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.inset(&[9].into(), 0.25, Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 10);
    let top = cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = cage.vertex(v).unwrap().at;
        assert!(
            (at[0].abs() - 0.75).abs() < 1e-12
                && (at[1].abs() - 0.75).abs() < 1e-12
                && at[2] == 1.0,
            "{at:?}"
        );
    }
    assert_valid(&built(cage.clone(), Mirror::None));
    cage.extrude(&[9].into(), 0.5, Mirror::None).unwrap();
    assert_valid(&built(cage, Mirror::None));
}

/// Insetting half a box's top, mirrored: the corners on the mirror plane
/// stay on it, and no quad is made along the open edge there, so the
/// mirrored whole is one valid solid with a symmetric inset.
#[test]
fn inset_keeps_the_mirror_plane() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.halve(0).unwrap();
    cage.inset(&[9].into(), 0.25, Mirror::X).unwrap();
    let on_plane = (cage.face(9).unwrap().vertices.iter())
        .filter(|&&v| cage.vertex(v).unwrap().at[0] == 0.0)
        .count();
    assert_eq!(on_plane, 2);
    assert_valid(&built(cage, Mirror::X));
}

/// A loop cut a third of the way across a box's vertical ring: four new
/// vertices at `z = -1/3`, four quads split in two, and a crease on one of
/// the ring's edges carried by both halves.
#[test]
fn loop_cut_at_a_fraction() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.set_sharpness(&[[0, 4]], 1.0);
    cage.insert_loop(0, 4, 1.0 / 3.0).unwrap();
    assert_eq!(cage.faces.len(), 10);
    let new: Vec<[f64; 3]> = (cage.vertices.iter())
        .filter(|v| v.id >= 14)
        .map(|v| v.at)
        .collect();
    assert_eq!(new.len(), 4);
    for at in &new {
        assert!((at[2] + 1.0 / 3.0).abs() < 1e-12, "{at:?}");
    }
    assert_eq!(cage.creases.len(), 2);
    assert_valid(&built(cage, Mirror::None));
}

/// The ring across a box's vertical edge is the four vertical edges,
/// closed. The box's corners have three edges, so the loop through any of
/// its edges stops at both ends, as that edge alone; the loop through an
/// edge of a loop cut across the box runs round its four edges, and on a
/// cylinder with a loop cut round all eight.
#[test]
fn rings_and_loops_follow_the_quads() {
    let cage = Cage::cuboid([2.0, 2.0, 2.0]);
    let ring = cage.edge_ring(0, 4).unwrap();
    assert!(ring.closed);
    assert_eq!(ring.edges.len(), 4);
    assert_eq!(ring.quads.len(), 4);
    let mut edges: Vec<[u32; 2]> = ring.edges.iter().map(|&(a, b)| undirected(a, b)).collect();
    edges.sort();
    assert_eq!(edges, [[0, 4], [1, 5], [2, 6], [3, 7]]);
    assert_eq!(cage.edge_loop(0, 4).unwrap(), [[0, 4]]);
    assert_eq!(cage.edge_loop(4, 5).unwrap(), [[4, 5]]);
    let mut cage = cage;
    cage.insert_loop(0, 4, 0.5).unwrap();
    let cut = cage.edges().into_iter().find(|&[a, _]| a >= 14).unwrap();
    assert_eq!(cage.edge_loop(cut[0], cut[1]).unwrap().len(), 4);

    let mut cage = Cage::cylinder(1.0, 2.0, 8);
    cage.insert_loop(0, 8, 0.5).unwrap();
    let cut = cage.edges().into_iter().find(|&[a, _]| a >= 42).unwrap();
    let around = cage.edge_loop(cut[0], cut[1]).unwrap();
    assert_eq!(around.len(), 8, "{around:?}");
    assert_valid(&built(cage, Mirror::None));
}

/// A loop cut's vertices slid a quarter of the way along the box's
/// vertical edges: all four move alike, and slid back the other way by a
/// half they end on the other side of where they started. The cage stays
/// valid.
#[test]
fn edge_slide_moves_a_loop_along_its_cross_edges() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.insert_loop(0, 4, 0.5).unwrap();
    let loop_edges: Vec<[u32; 2]> = cage
        .edges()
        .into_iter()
        .filter(|&[a, b]| a >= 14 && b >= 14)
        .collect();
    assert_eq!(loop_edges.len(), 4);
    let heights = |cage: &Cage| -> Vec<f64> {
        (cage.vertices.iter())
            .filter(|v| v.id >= 14)
            .map(|v| v.at[2])
            .collect()
    };
    cage.edge_slide(&loop_edges, 0.25, Mirror::None).unwrap();
    let after = heights(&cage);
    assert_eq!(after.len(), 4);
    let z = after[0];
    assert!((z.abs() - 0.25).abs() < 1e-12, "{after:?}");
    assert!(after.iter().all(|&h| (h - z).abs() < 1e-12), "{after:?}");
    cage.edge_slide(&loop_edges, -0.5, Mirror::None).unwrap();
    let back = heights(&cage);
    assert!(back[0] * z < 0.0, "{z} -> {back:?}");
    assert_valid(&built(cage, Mirror::None));
}

/// Bevelling all twelve edges of a box: every face shrinks, a strip
/// replaces each edge and a triangle each corner — 6 + 12 + 8 faces, the
/// shrunk top's corners `width` in from both of its edges — and the cage is
/// a valid solid. Bevelling the top's four edges alone makes a chamfered
/// rim: four strips and no corner faces.
#[test]
fn bevel_replaces_edges_by_strips() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    let edges: Vec<[u32; 2]> = cage.edges().into_iter().collect();
    cage.bevel(&edges, 0.2, Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 6 + 12 + 8);
    assert_eq!(cage.vertices.len(), 24);
    let top = cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = cage.vertex(v).unwrap().at;
        assert!(
            (at[0].abs() - 0.8).abs() < 1e-12 && (at[1].abs() - 0.8).abs() < 1e-12 && at[2] == 1.0,
            "{at:?}"
        );
    }
    assert_valid(&built(cage, Mirror::None));

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.bevel(&[[4, 5], [5, 6], [6, 7], [4, 7]], 0.3, Mirror::None)
        .unwrap();
    assert_eq!(cage.faces.len(), 10);
    assert_valid(&built(cage, Mirror::None));
}

/// One edge of a box bevelled on its own ends on the third face at each
/// corner: the strip's end vertices lie `width` along the corner's two
/// other edges, which split there, the corner itself is left between the
/// third face and the strip's cap and goes with it, and the third face
/// becomes a pentagon through both. Seven faces, ten vertices, the top
/// still a quad. A width longer than an edge beside the corner, and a
/// boundary edge, are refused by name.
#[test]
fn bevel_of_one_edge_is_capped() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.bevel(&[[4, 5]], 0.3, Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 7);
    assert_eq!(cage.vertices.len(), 10);
    assert!(cage.vertex(4).is_err() && cage.vertex(5).is_err());
    assert_eq!(cage.face(9).unwrap().vertices.len(), 4);
    assert_eq!(cage.face(13).unwrap().vertices.len(), 5);
    let at: Vec<[f64; 3]> = (cage.face(9).unwrap().vertices.iter())
        .map(|&v| cage.vertex(v).unwrap().at)
        .collect();
    assert!(
        at.iter()
            .any(|p| (p[1] + 0.7).abs() < 1e-12 && p[0] == -1.0),
        "{at:?}"
    );
    assert_valid(&built(cage, Mirror::None));

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    let e = cage
        .bevel(&[[4, 5]], 2.5, Mirror::None)
        .expect_err("too wide");
    assert!(e.to_string().contains("too short"), "{e}");

    let mut plane = Cage::plane(2.0, 2);
    let e = plane.bevel(&[[0, 1]], 0.1, Mirror::None).expect_err("open");
    assert!(e.to_string().contains("e0-1"), "{e}");
}

/// A box's top edge collapsed by merging its two vertices: the top and the
/// front become triangles, the sides keep their four corners, and the
/// solid is valid. Merging two opposite corners of one face is refused by
/// name.
#[test]
fn merge_collapses_an_edge() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.merge_vertices(&[4, 5].into(), Mirror::None).unwrap();
    assert_eq!(cage.vertices.len(), 7);
    assert_eq!(cage.face(9).unwrap().vertices.len(), 3);
    assert_eq!(cage.vertex(4).unwrap().at, [0.0, -1.0, 1.0]);
    assert_valid(&built(cage, Mirror::None));

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    let e = cage
        .merge_vertices(&[4, 6].into(), Mirror::None)
        .expect_err("a pinched face");
    assert!(e.to_string().contains("f9"), "{e}");
}

/// Dissolving the edge between a box's top and front merges them into one
/// face — a quad, since the edge's ends are left between two faces and go
/// with it: a wedge. Dissolving a vertex of the loop cut on a box merges
/// the four faces at it into one, and the box's two corners along its edge,
/// left between two faces, go with it; dissolving a corner of the box
/// merges its three faces, and the three neighbours left along the merged
/// face's sides go too: a tetrahedron. Each builds a valid solid.
#[test]
fn dissolve_merges_the_faces_around() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.dissolve_edges(&[[4, 5]], Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 5);
    assert_eq!(cage.vertices.len(), 6);
    assert_eq!(cage.face(9).unwrap().vertices.len(), 4);
    assert_valid(&built(cage, Mirror::None));

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.insert_loop(0, 4, 0.5).unwrap();
    let middle = (cage.vertices.iter()).find(|v| v.id >= 14).unwrap().id;
    cage.dissolve_vertices(&[middle].into(), Mirror::None)
        .unwrap();
    assert_eq!(cage.faces.len(), 7);
    assert_eq!(cage.vertices.len(), 9);
    assert!(cage.vertex(middle).is_err());
    assert_valid(&built(cage, Mirror::None));

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.dissolve_vertices(&[6].into(), Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 4);
    assert_eq!(cage.vertices.len(), 4);
    assert_eq!(cage.face(9).unwrap().vertices.len(), 3);
    assert_valid(&built(cage, Mirror::None));
}

/// Two opposite faces of a box, inset and bridged, make a tunnel: both go,
/// four quads join their rims, and the solid is a valid ring.
#[test]
fn bridge_makes_a_tunnel() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.inset(&[9].into(), 0.4, Mirror::None).unwrap();
    cage.inset(&[8].into(), 0.4, Mirror::None).unwrap();
    cage.bridge(9, 8, Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 4 + 4 + 4 + 4);
    assert!(cage.face(9).is_err() && cage.face(8).is_err());
    let part = built(cage.clone(), Mirror::None);
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 1);
}

/// A box with its top deleted is open; filling the hole closes it again
/// with a face through the four rim vertices, facing out, and the solid
/// is valid. The seam of a mirrored half is no hole to fill.
#[test]
fn fill_closes_a_hole() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.delete_faces(&[9].into());
    assert!(cage.is_boundary(4, 5));
    assert_eq!(cage.boundary_run(4, 5).unwrap().len(), 4);
    let part = built(cage.clone(), Mirror::None);
    assert!(part.solid_names().is_empty());
    cage.fill_hole(4, 5, Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 6);
    let part = built(cage, Mirror::None);
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 1);

    let mut half = Cage::cuboid([2.0, 2.0, 2.0]);
    half.halve(0).unwrap();
    let seam = half
        .edges()
        .into_iter()
        .find(|&[a, b]| {
            half.vertex(a).unwrap().at[0] == 0.0 && half.vertex(b).unwrap().at[0] == 0.0
        })
        .unwrap();
    let e = half
        .fill_hole(seam[0], seam[1], Mirror::X)
        .expect_err("the seam");
    assert!(e.to_string().contains("mirror plane"), "{e}");
}

/// A soft crease pulls the surface towards the edge without a corner: the
/// top face of a box with its rim creased at a half lies between the smooth
/// box's and the sharp one's at the rim's middle, and the surface normal
/// still turns smoothly across the edge. Saved, the sharpness comes back.
#[test]
fn soft_creases_pull_without_a_corner() {
    let rim = [[4, 5], [5, 6], [6, 7], [4, 7]];
    let height = |sharpness: f64| {
        let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
        if sharpness > 0.0 {
            cage.set_sharpness(&rim, sharpness);
        }
        let part = built(cage, Mirror::None);
        assert_valid(&part);
        let top = part.face_id("subd(s,f9)").unwrap();
        let surface = &part.topology().faces[&top].surface;
        // The middle of the side from v4 to v5, on the rim.
        surface
            .evaluate(Scalar::ONE, Scalar::ZERO)
            .unwrap()
            .to_f64_array()
    };
    let (smooth, soft, sharp) = (height(0.0), height(0.5), height(1.0));
    assert!(
        smooth[2] < soft[2] && soft[2] < sharp[2],
        "{smooth:?} {soft:?} {sharp:?}"
    );
    assert!((sharp[2] - 1.0).abs() < 1e-9, "{sharp:?}");

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.set_sharpness(&rim, 0.5);
    let part = built(cage.clone(), Mirror::None);
    let (kink, _) = largest_kink(&part);
    assert!(kink < 15.0, "the normal turns by {kink} degrees");
    let json = serde_json::to_string(&cage).unwrap();
    let back: Cage = serde_json::from_str(&json).unwrap();
    assert_eq!(back.sharpness(4, 5), 0.5);
}
