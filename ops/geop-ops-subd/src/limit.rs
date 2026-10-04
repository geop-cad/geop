//! The limit surface of a once-subdivided cage (see [`crate::subdivide`]),
//! as one bicubic Bézier patch per quad.
//!
//! Every quad's patch is put together from three kinds of control points,
//! each computed once and shared by every patch it belongs to:
//!
//! - a *corner* per vertex: its limit position;
//! - two *edge* points per edge, one near each end, which with the corners
//!   make up the cubic every patch along that edge has as its boundary —
//!   computed per edge, not per patch, so neighbouring patches meet along
//!   exactly the same curve, and the B-rep built from them is closed;
//! - an *interior* point per corner of every quad.
//!
//! This is Loop and Schaefer's approximation of Catmull–Clark surfaces by
//! bicubic patches: where all four corners of a quad and its neighbours are
//! regular — smooth, four faces each — it gives exactly the uniform bicubic
//! B-spline that is the limit surface there. Around an extraordinary vertex
//! it is an approximation: the patches meet along shared curves, with a
//! continuous normal at every vertex — every edge point next to a smooth
//! vertex lies in its limit tangent plane — but across the edges out of an
//! extraordinary vertex the normal turns by a few degrees.
//!
//! Creased and boundary edges are sharp: their curve is the cubic B-spline
//! of their vertices, which the patches on either side share, and the
//! interior points next to them are computed per *sector*, the faces around
//! a vertex between two sharp edges, as if that sector were mirrored into a
//! regular vertex's: weight `n` is the vertex's number of faces where it is
//! smooth, twice the sector's along a crease, four times at a corner.

use std::collections::HashMap;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use crate::subdivide::{
    Subdivided, VertexKind, divide, is_sharp, next, prev, vertex_kind, weighted,
};

/// A bicubic Bézier patch, `b[i][j]` with `i` along its first side.
pub type Patch<S> = [[Vector3<S>; 4]; 4];

/// The shared control points of every patch of a subdivided mesh.
#[derive(Clone, Debug)]
pub struct ControlNet<S: Scalar> {
    /// Per vertex: its limit position.
    pub corner: Vec<Vector3<S>>,
    /// Per directed edge `(v, w)`: the edge point near `v`.
    pub edge: HashMap<(usize, usize), Vector3<S>>,
    /// Per quad, per corner: its interior point.
    pub interior: Vec<[Vector3<S>; 4]>,
}

impl<S: Scalar> ControlNet<S> {
    /// The patch of quad `f`, its first side running from its corner
    /// `start` to the next one.
    pub fn patch(&self, mesh: &Subdivided<S>, f: usize, start: usize) -> Patch<S> {
        let k = |i: usize| (start + i) % 4;
        let q = |i: usize| mesh.faces[f][k(i)];
        let l = |i: usize| self.corner[q(i)];
        let e = |a: usize, b: usize| self.edge[&(q(a), q(b))];
        let n = |i: usize| self.interior[f][k(i)];
        [
            [l(0), e(0, 3), e(3, 0), l(3)],
            [e(0, 1), n(0), n(3), e(3, 2)],
            [e(1, 0), n(1), n(2), e(2, 3)],
            [l(1), e(1, 2), e(2, 1), l(2)],
        ]
    }
}

/// For every corner of `v`'s fan, how many faces the sector it is in has:
/// the run of faces around `v` between two sharp edges.
fn sector_sizes<S: Scalar>(mesh: &Subdivided<S>, v: usize) -> Vec<usize> {
    let (fan, closed) = &mesh.topology.fans[v];
    let m = fan.len();
    // Between fan[i] and fan[i + 1] lies the edge to fan[i]'s previous
    // corner.
    let cut = |i: usize| {
        let w = prev(&mesh.faces, fan[i]);
        is_sharp(&mesh.sharp, &mesh.topology, v, w)
    };
    let start = if *closed {
        // Begin just after a sharp edge; a smooth vertex has none, and is
        // one sector all round.
        match (0..m).find(|&i| cut(i)) {
            Some(i) => (i + 1) % m,
            None => return vec![m; m],
        }
    } else {
        0
    };
    let mut sizes = vec![0; m];
    let mut run: Vec<usize> = Vec::new();
    for step in 0..m {
        let i = (start + step) % m;
        run.push(i);
        if step + 1 == m || cut(i) {
            for &j in &run {
                sizes[j] = run.len();
            }
            run.clear();
        }
    }
    sizes
}

/// The normal of `v`'s limit tangent plane, for a smooth vertex of `n`
/// faces: the cross product of its two limit tangents, from the
/// eigenvector masks of Catmull–Clark subdivision. A free choice of how
/// long, so it is sharpened.
fn tangent_normal<S: Scalar>(mesh: &Subdivided<S>, v: usize) -> Vector3<S> {
    let fan = &mesh.topology.fans[v].0;
    let n = fan.len();
    let p = &mesh.positions;
    let angle = |i: usize| 2.0 * std::f64::consts::PI * i as f64 / n as f64;
    let c = (std::f64::consts::PI / n as f64).cos();
    let a = 1.0 + angle(1).cos() + c * (2.0 * (9.0 + angle(1).cos())).sqrt();
    let tangent = |trig: fn(f64) -> f64| {
        fan.iter()
            .enumerate()
            .fold(Vector3::zero(), |sum, (i, &corner)| {
                let e = p[next(&mesh.faces, corner)].sub(&p[v]);
                let d = p[mesh.faces[corner.0][(corner.1 + 2) % 4]].sub(&p[v]);
                let we = S::from_f64(a * trig(angle(i)));
                let wd = S::from_f64(trig(angle(i)) + trig(angle(i + 1)));
                sum.add(&e.prod_scalar(we)).add(&d.prod_scalar(wd))
            })
    };
    tangent(f64::cos).prod_cross(&tangent(f64::sin)).sharpen()
}

/// The control net of `mesh`'s limit surface (see the module docs).
pub fn control_net<S: Scalar>(mesh: &Subdivided<S>) -> GeopResult<ControlNet<S>> {
    let p = &mesh.positions;
    let faces = &mesh.faces;
    let topology = &mesh.topology;
    let kinds: Vec<VertexKind> = (0..p.len())
        .map(|v| vertex_kind(faces, &mesh.sharp, topology, v))
        .collect();

    let mut weight = vec![[0usize; 4]; faces.len()];
    for v in 0..p.len() {
        let fan = &topology.fans[v].0;
        let sizes = sector_sizes(mesh, v);
        for (i, &(f, k)) in fan.iter().enumerate() {
            weight[f][k] = match kinds[v] {
                VertexKind::Smooth => fan.len(),
                VertexKind::Crease(..) => 2 * sizes[i],
                VertexKind::Corner => 4 * sizes[i],
            };
        }
    }
    let mut interior = Vec::with_capacity(faces.len());
    for (f, face) in faces.iter().enumerate() {
        let mut points = [Vector3::zero(); 4];
        for k in 0..4 {
            let n = weight[f][k] as i64;
            let terms = [
                (n, p[face[k]]),
                (2, p[face[(k + 1) % 4]]),
                (2, p[face[(k + 3) % 4]]),
                (1, p[face[(k + 2) % 4]]),
            ];
            points[k] = divide(weighted(&terms), n + 5)?;
        }
        interior.push(points);
    }

    let mut corner = Vec::with_capacity(p.len());
    for v in 0..p.len() {
        corner.push(match kinds[v] {
            VertexKind::Corner => p[v],
            VertexKind::Crease(a, b) => divide(weighted(&[(1, p[a]), (4, p[v]), (1, p[b])]), 6)?,
            VertexKind::Smooth => {
                let fan = &topology.fans[v].0;
                let n = fan.len() as i64;
                let mut terms = vec![(n * n, p[v])];
                for &(f, k) in fan {
                    terms.push((4, p[next(faces, (f, k))]));
                    terms.push((1, p[faces[f][(k + 2) % 4]]));
                }
                divide(weighted(&terms), n * (n + 5))?
            }
        });
    }

    let mut normals: HashMap<usize, Vector3<S>> = HashMap::new();
    let mut edge = HashMap::new();
    for (&(v, w), &(f, k)) in &topology.directed {
        for (from, to) in [(v, w), (w, v)] {
            if edge.contains_key(&(from, to)) {
                continue;
            }
            let sharp = is_sharp(&mesh.sharp, topology, from, to);
            let point = if sharp && kinds[from] != VertexKind::Smooth {
                divide(weighted(&[(2, p[from]), (1, p[to])]), 3)?
            } else {
                // Not on the boundary, so two faces run along it: `f` from
                // `v` to `w`, and its neighbour back.
                let &(g, j) = &topology.directed[&(w, v)];
                let at = |face: usize, corner_of: usize, from: usize| {
                    if faces[face][corner_of] == from {
                        interior[face][corner_of]
                    } else {
                        interior[face][(corner_of + 1) % 4]
                    }
                };
                let sum = at(f, k, from).add(&at(g, j, from));
                let average = divide(sum, 2)?;
                let n = topology.fans[from].0.len();
                if kinds[from] == VertexKind::Smooth && n != 4 {
                    let normal = *normals
                        .entry(from)
                        .or_insert_with(|| tangent_normal(mesh, from));
                    let off = average.sub(&corner[from]);
                    let along = off.prod_dot(&normal).div(normal.norm_sq())?;
                    average.sub(&normal.prod_scalar(along))
                } else {
                    average
                }
            };
            edge.insert((from, to), point);
        }
    }
    Ok(ControlNet {
        corner,
        edge,
        interior,
    })
}
