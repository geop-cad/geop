//! One Catmull–Clark step, with sharp creases: what every face of the
//! cage becomes before its limit surface is built (see [`crate::limit`]).
//!
//! A face of `n` sides becomes `n` quads around its *face point*, each
//! running from one of its corners — that corner's *vertex point* — to the
//! *edge points* on the two sides there. So after one step every face is a
//! quad, whatever the cage was made of, and the limit surface is the same
//! as the cage's.
//!
//! The rules, with the mesh's boundary counted as creased:
//!
//! - face point: the average of the face's corners;
//! - edge point: on a creased edge its midpoint, else the average of its
//!   two ends and its two faces' face points;
//! - vertex point, by how many creased edges meet at the vertex: none or
//!   one (a *dart*) — `(ΣF + ΣE + n(n - 2) V) / n²` over its `n` faces' face
//!   points `F` and the far ends `E` of its edges; two — a *crease* —
//!   `(A + 6 V + B) / 8` with `A`, `B` the creased edges' far ends; three or
//!   more, or a boundary vertex of one face — a *corner* — stays where it
//!   is.
//!
//! A *soft* crease, of sharpness `s` below one (see [`crate::cage::Crease`]),
//! is semi-sharp: its edge point is `(1 - s)` of the smooth rule's and `s`
//! of the creased one's, and a vertex with soft creases is moved
//! `(1 - s̄)` of the way by the rule counting its sharp creases alone and `s̄`
//! by the rule counting the soft ones too, `s̄` the mean sharpness of its
//! creases. Its halves are smooth after that: the limit surface rounds
//! off tighter there, without a corner.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use crate::cage::{Mesh, Topology, next, prev, undirected};

/// How the surface behaves at a vertex.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VertexKind {
    /// Smooth all round: no creased edge, or one.
    Smooth,
    /// Along a crease: exactly two creased edges meet, to these vertices.
    Crease(usize, usize),
    /// A sharp corner: three creased edges or more, or a boundary vertex
    /// of a single face.
    Corner,
}

/// Whether the edge between `a` and `b` is sharp: creased, or on the
/// mesh's boundary.
pub fn is_sharp(sharp: &BTreeSet<[usize; 2]>, topology: &Topology, a: usize, b: usize) -> bool {
    sharp.contains(&undirected(a, b)) || topology.is_boundary(a, b)
}

/// What kind of vertex `v` is (see [`VertexKind`]).
pub fn vertex_kind(
    faces: &[Vec<usize>],
    sharp: &BTreeSet<[usize; 2]>,
    topology: &Topology,
    v: usize,
) -> VertexKind {
    let (fan, closed) = &topology.fans[v];
    let mut neighbours: Vec<usize> = fan.iter().map(|&c| next(faces, c)).collect();
    if !closed {
        neighbours.push(prev(faces, fan[fan.len() - 1]));
    }
    let sharp_ends: Vec<usize> = neighbours
        .into_iter()
        .filter(|&w| is_sharp(sharp, topology, v, w))
        .collect();
    match sharp_ends[..] {
        _ if !closed && fan.len() == 1 => VertexKind::Corner,
        [a, b] => VertexKind::Crease(a, b),
        [] | [_] => VertexKind::Smooth,
        _ => VertexKind::Corner,
    }
}

/// `sum / count`, one division per coordinate.
pub fn divide(sum: Vector3, count: i64) -> GeopResult<Vector3> {
    let d = Scalar::from_i64(count);
    Ok(Vector3::from_array([
        sum[0].div(d)?,
        sum[1].div(d)?,
        sum[2].div(d)?,
    ]))
}

/// `(1 - s) a + s b`: `a` pulled the fraction `s` of the way to `b`.
fn blend(a: Vector3, b: Vector3, s: f64) -> Vector3 {
    let s = Scalar::from_f64(s);
    a.add(&b.sub(&a).prod_scalar(s))
}

/// `Σ wᵢ pᵢ` for integer weights.
pub fn weighted(terms: &[(i64, Vector3)]) -> Vector3 {
    terms.iter().fold(Vector3::zero(), |sum, (w, p)| {
        if *w == 1 {
            sum.add(p)
        } else {
            sum.add(&p.prod_scalar(Scalar::from_i64(*w)))
        }
    })
}

/// The mesh after one step: quads only, with where each of its vertices
/// and faces came from.
#[derive(Clone, Debug)]
pub struct Subdivided {
    pub positions: Vec<Vector3>,
    /// Face `face_start[f] + k` is the quad at corner `k` of the cage's
    /// face `f`: its vertex point, the edge point after it, the face point,
    /// the edge point before it.
    pub faces: Vec<Vec<usize>>,
    pub face_start: Vec<usize>,
    /// The cage's vertex `v` is vertex `v` here too; its face `f`'s face
    /// point is `face_point[f]`, and its edge `(a, b)`'s edge point
    /// `edge_point[&[a, b]]`, the smaller index first.
    pub face_point: Vec<usize>,
    pub edge_point: BTreeMap<[usize; 2], usize>,
    /// The halves of every creased edge.
    pub sharp: BTreeSet<[usize; 2]>,
    pub topology: Topology,
}

/// `mesh`, of topology `topology`, after one Catmull–Clark step.
pub fn subdivide(mesh: &Mesh, topology: &Topology) -> GeopResult<Subdivided> {
    let p: Vec<Vector3> = mesh
        .positions
        .iter()
        .map(|a| Vector3::from_array(a.map(Scalar::from_f64)))
        .collect();
    let faces = &mesh.faces;
    let face_points = faces
        .iter()
        .map(|face| {
            divide(
                weighted(&face.iter().map(|&w| (1, p[w])).collect::<Vec<_>>()),
                face.len() as i64,
            )
        })
        .collect::<GeopResult<Vec<_>>>()?;
    // The vertex point of `v` with the edges `sharp` counted as creased.
    let vertex_point = |v: usize, sharp: &BTreeSet<[usize; 2]>| -> GeopResult<Vector3> {
        Ok(match vertex_kind(faces, sharp, topology, v) {
            VertexKind::Corner => p[v],
            VertexKind::Crease(a, b) => divide(weighted(&[(1, p[a]), (6, p[v]), (1, p[b])]), 8)?,
            VertexKind::Smooth => {
                let fan = &topology.fans[v].0;
                let n = fan.len() as i64;
                let mut terms = vec![(n * (n - 2), p[v])];
                for &c in fan {
                    terms.push((1, face_points[c.0]));
                    terms.push((1, p[next(faces, c)]));
                }
                divide(weighted(&terms), n * n)?
            }
        })
    };
    // Every crease, soft ones too, for the rule a soft vertex is pulled
    // towards.
    let all_creased: BTreeSet<[usize; 2]> =
        mesh.sharp.iter().chain(mesh.soft.keys()).copied().collect();
    let mut positions = Vec::with_capacity(p.len() + 2 * faces.len());
    for v in 0..p.len() {
        let point = vertex_point(v, &mesh.sharp)?;
        // The creases at `v`, by sharpness: the mean decides how far it is
        // pulled towards the rule counting them all.
        let creases: Vec<f64> = all_creased
            .iter()
            .filter(|&&[a, b]| a == v || b == v)
            .map(|e| mesh.soft.get(e).copied().unwrap_or(1.0))
            .collect();
        let soft = creases.iter().any(|&s| s < 1.0);
        let point = if soft {
            let mean = creases.iter().sum::<f64>() / creases.len() as f64;
            blend(point, vertex_point(v, &all_creased)?, mean)
        } else {
            point
        };
        positions.push(point);
    }
    let face_point: Vec<usize> = (0..faces.len()).map(|f| positions.len() + f).collect();
    positions.extend(face_points.iter().copied());
    let mut edge_point = BTreeMap::new();
    for ([a, b], adjacent) in topology.edges() {
        let middle = || divide(p[a].add(&p[b]), 2);
        let point = if is_sharp(&mesh.sharp, topology, a, b) {
            middle()?
        } else {
            let terms = [
                (1, p[a]),
                (1, p[b]),
                (1, face_points[adjacent[0]]),
                (1, face_points[adjacent[1]]),
            ];
            let smooth = divide(weighted(&terms), 4)?;
            match mesh.soft.get(&[a, b]) {
                Some(&s) => blend(smooth, middle()?, s),
                None => smooth,
            }
        };
        edge_point.insert([a, b], positions.len());
        positions.push(point);
    }
    let edge = |a: usize, b: usize| edge_point[&undirected(a, b)];
    let mut quads = Vec::new();
    let mut face_start = Vec::with_capacity(faces.len());
    for (f, face) in faces.iter().enumerate() {
        face_start.push(quads.len());
        let n = face.len();
        for k in 0..n {
            let (v, after, before) = (face[k], face[(k + 1) % n], face[(k + n - 1) % n]);
            quads.push(vec![v, edge(v, after), face_point[f], edge(before, v)]);
        }
    }
    let mut sharp = BTreeSet::new();
    for &[a, b] in &mesh.sharp {
        let m = edge(a, b);
        sharp.insert(undirected(a, m));
        sharp.insert(undirected(b, m));
    }
    let names: Vec<String> = (0..positions.len())
        .map(|i| format!("subdivided vertex {i}"))
        .collect();
    let face_names: Vec<String> = (0..quads.len())
        .map(|i| format!("subdivided face {i}"))
        .collect();
    let topology = Topology::new(positions.len(), &quads, &names, &face_names)?;
    Ok(Subdivided {
        positions,
        faces: quads,
        face_start,
        face_point,
        edge_point,
        sharp,
        topology,
    })
}
