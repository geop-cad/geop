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

use std::collections::{BTreeMap, BTreeSet};

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use crate::cage::{Mesh, Topology};

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

/// The faces around `v`'s fan in order: the corner after `v` in face `c`,
/// the one before it, and the one opposite (for a quad).
pub fn next(faces: &[Vec<usize>], (f, k): (usize, usize)) -> usize {
    faces[f][(k + 1) % faces[f].len()]
}

pub fn prev(faces: &[Vec<usize>], (f, k): (usize, usize)) -> usize {
    let n = faces[f].len();
    faces[f][(k + n - 1) % n]
}

/// Whether the edge between `a` and `b` is sharp: creased, or on the
/// mesh's boundary.
pub fn is_sharp(sharp: &BTreeSet<(usize, usize)>, topology: &Topology, a: usize, b: usize) -> bool {
    sharp.contains(&(a.min(b), a.max(b))) || topology.is_boundary(a, b)
}

/// What kind of vertex `v` is (see [`VertexKind`]).
pub fn vertex_kind(
    faces: &[Vec<usize>],
    sharp: &BTreeSet<(usize, usize)>,
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
pub fn divide<S: Scalar>(sum: Vector3<S>, count: i64) -> GeopResult<Vector3<S>> {
    let d = S::from_i64(count);
    Ok(Vector3::from_array([
        sum[0].div(d)?,
        sum[1].div(d)?,
        sum[2].div(d)?,
    ]))
}

/// `Σ wᵢ pᵢ` for integer weights.
pub fn weighted<S: Scalar>(terms: &[(i64, Vector3<S>)]) -> Vector3<S> {
    terms.iter().fold(Vector3::zero(), |sum, (w, p)| {
        if *w == 1 {
            sum.add(p)
        } else {
            sum.add(&p.prod_scalar(S::from_i64(*w)))
        }
    })
}

/// The mesh after one step: quads only, with where each of its vertices
/// and faces came from.
#[derive(Clone, Debug)]
pub struct Subdivided<S: Scalar> {
    pub positions: Vec<Vector3<S>>,
    /// Face `face_start[f] + k` is the quad at corner `k` of the cage's
    /// face `f`: its vertex point, the edge point after it, the face point,
    /// the edge point before it.
    pub faces: Vec<Vec<usize>>,
    pub face_start: Vec<usize>,
    /// The cage's vertex `v` is vertex `v` here too; its face `f`'s face
    /// point is `face_point[f]`, and its edge `(a, b)`'s edge point
    /// `edge_point[&(a, b)]`, the smaller index first.
    pub face_point: Vec<usize>,
    pub edge_point: BTreeMap<(usize, usize), usize>,
    /// The halves of every creased edge.
    pub sharp: BTreeSet<(usize, usize)>,
    pub topology: Topology,
}

/// `mesh`, of topology `topology`, after one Catmull–Clark step.
pub fn subdivide<S: Scalar>(mesh: &Mesh, topology: &Topology) -> GeopResult<Subdivided<S>> {
    let p: Vec<Vector3<S>> = mesh
        .positions
        .iter()
        .map(|a| Vector3::from_array(a.map(S::from_f64)))
        .collect();
    let faces = &mesh.faces;
    let face_points = faces
        .iter()
        .map(|face| divide(weighted(&face.iter().map(|&w| (1, p[w])).collect::<Vec<_>>()), face.len() as i64))
        .collect::<GeopResult<Vec<_>>>()?;
    let mut positions = Vec::with_capacity(p.len() + 2 * faces.len());
    for v in 0..p.len() {
        let point = match vertex_kind(faces, &mesh.sharp, topology, v) {
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
        };
        positions.push(point);
    }
    let face_point: Vec<usize> = (0..faces.len()).map(|f| positions.len() + f).collect();
    positions.extend(face_points.iter().copied());
    let mut edge_point = BTreeMap::new();
    for ((a, b), adjacent) in topology.edges() {
        let point = if is_sharp(&mesh.sharp, topology, a, b) {
            divide(p[a].add(&p[b]), 2)?
        } else {
            let terms = [
                (1, p[a]),
                (1, p[b]),
                (1, face_points[adjacent[0]]),
                (1, face_points[adjacent[1]]),
            ];
            divide(weighted(&terms), 4)?
        };
        edge_point.insert((a, b), positions.len());
        positions.push(point);
    }
    let edge = |a: usize, b: usize| edge_point[&(a.min(b), a.max(b))];
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
    for &(a, b) in &mesh.sharp {
        let m = edge(a, b);
        sharp.insert((a.min(m), a.max(m)));
        sharp.insert((b.min(m), b.max(m)));
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
