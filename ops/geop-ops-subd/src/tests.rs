//! The limit surfaces of cages: closed ones are valid solids, open ones
//! sheets; where the cage is regular the surface is exactly the bicubic
//! B-spline Catmull–Clark converges to; creases stay sharp; and every
//! edit leaves a cage whose limit is valid.

use geop_core_math::{
    for_all_scalars,
    scalars::{Field, Ring, ScalInF64, Scalar},
    vector::Vector3,
};
use geop_core_topology::{
    Body,
    validation::{ValidationParameters, validate, validate_manifold},
};
use geop_ops::{NoFiles, Part, operation::Operation};

use crate::{Cage, Mirror, Subd, SubdArgs};

/// Valid, and a manifold (`validate_manifold` runs `validate` first: once
/// is enough), its names checked.
fn assert_valid<S: Scalar>(part: &Part<S>) {
    let params = ValidationParameters::default();
    if let Err(errors) = validate_manifold(&params, part.topology()) {
        let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
        panic!(
            "{} validation errors:\n{}",
            messages.len(),
            messages.join("\n")
        );
    }
    part.check_names().unwrap();
}

/// The limit of `cage`, mirrored in `mirror`, built as step `s`.
fn built<S: Scalar>(cage: Cage, mirror: Mirror) -> Part<S> {
    Subd.apply(Part::new(), "s", &SubdArgs { cage, mirror }, &NoFiles)
        .unwrap_or_else(|e| panic!("{e}"))
}

fn faces<S: Scalar>(part: &Part<S>) -> usize {
    part.topology().faces.len()
}

fn v<S: Scalar>(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

/// A box cage: one face per cage face, one edge per cage edge, one vertex
/// per cage vertex, closed into a valid solid.
fn check_box_cage_is_a_valid_solid<S: Scalar>() {
    let part = built::<S>(Cage::cuboid([2.0, 2.0, 2.0]), Mirror::None);
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
#[test]
fn box_cage_is_a_valid_solid() {
    for_all_scalars!(check_box_cage_is_a_valid_solid);
}

/// A wavy 5 x 5 plane: around its middle face every vertex is regular, so
/// the limit there is the uniform bicubic B-spline of the cage — at the
/// face's centre, the tensor product of the cubic B-spline's weights at
/// one half, `(1, 23, 23, 1) / 48`, over the 4 x 4 cage vertices around
/// it; at a vertex, `(16 v + 4 Σ edge neighbours + Σ diagonal ones) / 36`.
#[test]
fn regular_patch_matches_the_limit() {
    type S = ScalInF64;
    let mut cage = Cage::plane(5.0, 5);
    for vertex in &mut cage.vertices {
        let [x, y, _] = vertex.at;
        vertex.at[2] = 0.3 * (1.3 * x).sin() * (0.7 * y + 0.2).cos() + 0.1 * x * y;
    }
    // Vertex (i, j) of the 6 x 6 grid is id `6 j + i`; the face at (i, j)
    // is id `36 + 5 j + i`.
    let at = |i: usize, j: usize| v::<S>(cage.vertices[6 * j + i].at);
    let part = built::<S>(cage.clone(), Mirror::None);

    let face = part.face_id("subd(s,f48)").unwrap();
    let surface = &part.topology().faces[&face].surface;
    let w = [1.0, 23.0, 23.0, 1.0].map(|x| x / 48.0);
    let mut expected = Vector3::<S>::zero();
    for a in 0..4 {
        for b in 0..4 {
            let p = at(1 + a, 1 + b);
            expected = expected.add(&p.prod_scalar(S::from_f64(w[a] * w[b])));
        }
    }
    let centre = surface.evaluate(S::ONE, S::ONE).unwrap();
    assert!(
        centre.could_be_equal(&expected),
        "{centre:?} vs {expected:?}"
    );

    let vertex = part.vertex_id("subd(s,v14)").unwrap();
    let limit = part.topology().vertices[&vertex].point;
    let (i, j) = (2, 2);
    let mut sum = at(i, j).prod_scalar(S::from_i64(16));
    for (di, dj) in [(1, 0), (0, 1), (-1, 0), (0, -1)] {
        let p = at((i as i64 + di) as usize, (j as i64 + dj) as usize);
        sum = sum.add(&p.prod_scalar(S::from_i64(4)));
    }
    for (di, dj) in [(1, 1), (-1, 1), (-1, -1), (1, -1)] {
        sum = sum.add(&at((i as i64 + di) as usize, (j as i64 + dj) as usize));
    }
    let expected = sum.map(|c| c.div(S::from_i64(36)).unwrap());
    assert!(limit.could_be_equal(&expected), "{limit:?} vs {expected:?}");
}

/// The largest angles, in degrees, between the normals of the two faces of
/// any edge of `part`: sampled along every edge, and at its ends only.
fn largest_kink(part: &Part<ScalInF64>) -> (f64, f64) {
    type S = ScalInF64;
    let model = part.topology();
    let (mut largest, mut at_ends): (f64, f64) = (0.0, 0.0);
    for &edge in model.edges.keys() {
        let coedges: Vec<_> = model
            .coedges
            .values()
            .filter(|c| c.geometry == geop_core_topology::CoedgeGeometry::Edge(edge))
            .collect();
        for k in 0..=8 {
            let normals: Vec<Vector3<S>> = coedges
                .iter()
                .map(|c| {
                    let (t0, t1) = c.pcurve.domain();
                    let t = t0.add(t1.sub(t0).mul(S::from_f64(k as f64 / 8.0)));
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
    let part = built::<ScalInF64>(Cage::cuboid([2.0, 2.0, 2.0]), Mirror::None);
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
fn check_creases_stay_sharp<S: Scalar>() {
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    let edges: Vec<[u32; 2]> = cage.edges().into_iter().collect();
    cage.set_crease(&edges, true);
    let part = built::<S>(cage, Mirror::None);
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
        .evaluate(S::from_f64(0.3), S::from_f64(1.7))
        .unwrap();
    assert!(p[2].could_be_equal(S::ONE), "{p:?}");

    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.set_crease(&[[4, 5], [5, 6], [6, 7], [4, 7]], true);
    let part = built::<S>(cage, Mirror::None);
    assert_valid(&part);
    // Along the crease between the top (f9, u along v4 -> v5) and the
    // front (f10, its last side running from v4 to v5's ... v5 -> v4).
    let normal = |face: &str, u: f64, w: f64| {
        let id = part.face_id(face).unwrap();
        let s = &part.topology().faces[&id].surface;
        s.normal(S::from_f64(u), S::from_f64(w)).unwrap()
    };
    let top = normal("subd(s,f9)", 1.0, 0.0);
    let front = normal("subd(s,f10)", 1.0, 2.0);
    let cos = top
        .prod_dot(&front)
        .div(top.norm().mul(front.norm()))
        .unwrap();
    assert!(cos.to_f64() < 0.5, "the crease is not sharp: cos = {cos:?}");
}
#[test]
fn creases_stay_sharp() {
    for_all_scalars!(check_creases_stay_sharp);
}

/// A box with its top extruded, a loop inserted around it, and its sides
/// pulled in: still one valid solid.
#[test]
fn extruded_face_cage_is_valid() {
    type S = ScalInF64;
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.extrude(&[9].into(), 0.8, Mirror::None).unwrap();
    assert_eq!(cage.faces.len(), 10);
    let part = built::<S>(cage.clone(), Mirror::None);
    assert_valid(&part);
    assert_eq!(faces(&part), 10);

    cage.insert_loop(0, 4).unwrap();
    assert_eq!(cage.faces.len(), 14);
    let part = built::<S>(cage, Mirror::None);
    assert_valid(&part);
}

/// A plane is an open cage, whose limit is a sheet; a prism's ends are
/// triangles, each split into a face per corner around its centre.
#[test]
fn plane_and_prism_build() {
    type S = ScalInF64;
    let part = built::<S>(Cage::plane(2.0, 3), Mirror::None);
    assert_valid(&part);
    assert!(part.solid_names().is_empty());
    assert_eq!(faces(&part), 9);

    let part = built::<S>(Cage::cylinder(1.0, 1.0, 3), Mirror::None);
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
    type S = ScalInF64;
    let part = built::<S>(Cage::cylinder(1.0, 2.0, 8), Mirror::None);
    assert_valid(&part);
    assert_eq!(faces(&part), 16 + 2 * 8);
    part.face_id("subd(s,f24,v3)").unwrap();

    let part = built::<S>(Cage::sphere(1.0), Mirror::None);
    assert_valid(&part);
    assert_eq!(faces(&part), 24);
}

/// Half a box, mirrored in `x = 0`: the halves join smoothly along the
/// plane into one solid, symmetric about it.
#[test]
fn mirrored_cage_is_one_symmetric_solid() {
    type S = ScalInF64;
    let mut cage = Cage::cuboid([2.0, 2.0, 2.0]);
    cage.halve(0).unwrap();
    assert_eq!(cage.faces.len(), 5);
    let part = built::<S>(cage.clone(), Mirror::X);
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
    assert_valid(&built::<S>(whole, Mirror::None));
}

/// What no limit surface can be built from is refused, naming the
/// element: a face turned against its neighbours, an inside-out cage, a
/// vertex across the mirror plane.
#[test]
fn bad_cages_are_refused() {
    type S = ScalInF64;
    let refused = |cage: Cage, mirror: Mirror| {
        Subd.apply(Part::<S>::new(), "s", &SubdArgs { cage, mirror }, &NoFiles)
            .err()
            .expect("refused")
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
    cage.set_crease(&[[1, 2]], true);
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
    cage.set_crease(&[[4, 5]], true);
    cage.extrude(&[9, 10].into(), 0.5, Mirror::None).unwrap();
    let edges = cage.edges();
    assert_eq!(cage.creases.len(), 1);
    assert!(edges.contains(&cage.creases[0]), "{:?}", cage.creases);
    assert_ne!(cage.creases[0], [4, 5]);
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
/// face extruded or shrunk about its centre, a loop inserted, an edge
/// creased. Says what it did.
fn random_edit(cage: &mut Cage, mirror: Mirror, random: &mut Random) -> String {
    match random.below(5) {
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
            let edges: Vec<[u32; 2]> = cage.edges().into_iter().collect();
            let [a, b] = edges[random.below(edges.len())];
            match cage.insert_loop(a, b) {
                Ok(()) => format!("inserted a loop across e{a}-{b}"),
                Err(e) => format!("could not insert a loop across e{a}-{b}: {e}"),
            }
        }
        3 => {
            let edges: Vec<[u32; 2]> = cage.edges().into_iter().collect();
            let [a, b] = edges[random.below(edges.len())];
            let sharp = !cage.is_crease(a, b);
            cage.set_crease(&[[a, b]], sharp);
            format!("creased e{a}-{b}: {sharp}")
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
    type S = ScalInF64;
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
        let part = Subd
            .apply(
                Part::<S>::new(),
                "s",
                &SubdArgs {
                    cage: cage.clone(),
                    mirror,
                },
                &NoFiles,
            )
            .unwrap_or_else(|e| panic!("seed {seed}, after {done:#?}: {e}"));
        let params = ValidationParameters::default();
        if let Err(errors) = validate(&params, part.topology()) {
            let messages: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
            panic!("seed {seed}, after {done:#?}:\n{}", messages.join("\n"));
        }
    }
}
