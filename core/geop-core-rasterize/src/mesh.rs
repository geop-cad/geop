//! A face's mesh joined to its neighbours': every edge sampled once, and
//! every face meeting it along exactly those samples and each other's.
//!
//! Each face is triangulated on its own (see [`crate::grid`]), its trim
//! the `(u, v)` polygon of its edges' shared samples ([`Boundary`]). Where
//! a grid line crosses the trim, the clipping puts a point on it that only
//! that face has; where a clipped cell meets a whole one, a point on their
//! side that only the clipped one has. Both leave a vertex in the middle
//! of a neighbouring triangle's side — a T, which a mesh reader sees as a
//! crack. So the mesh is made conforming:
//!
//! 1. **Within a face** ([`Mesh::conform`]): a triangle side used by one
//!    triangle only, with a vertex of the face lying on it, is split there.
//! 2. **Across an edge** ([`zip`]): every point any face put on an edge,
//!    as a position along its polyline, is put on it in every face that
//!    edge bounds, the triangle along it split there.
//!
//! A point on an edge is placed in space on the edge's polyline, at its
//! position along it ([`placed`]) — not where the face's surface has its
//! `(u, v)` — so two faces' points there are one point, bit for bit: an STL
//! of a solid is a closed mesh. Rendering is what sampling is for (see
//! [`crate::grid`]): plain `f64`, a point the clipping put on a side of the
//! trim found on it within a hair of the face's size.

use std::collections::{BTreeMap, HashMap, HashSet};

use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::{CoedgeGeometry, CoedgeId, EdgeId, Face, Model, Sense, VertexId};

use crate::clip::Point;

/// Newton steps placing an edge's sample on a face's surface.
const PROJECTION_ITERATIONS: usize = 24;
/// A point lies on a side when it is this close to it, relative to the
/// face's `(u, v)` size: the rounding of the clipping that put it there.
const ON_SIDE: f64 = 1e-9;
/// Two positions along an edge's polyline this close, in samples, are one.
const SAME_POSITION: f64 = 1e-9;

/// Where a mesh point lies: inside its face, at a position along an edge's
/// polyline (`k + f`, between its samples `k` and `k + 1`), or at a vertex
/// of the model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum On {
    Inside,
    Edge(EdgeId, f64),
    Vertex(VertexId),
}

/// A trim loop of a face: `(u, v)` points, where each lies, and the edge
/// each side runs along — side `i` from point `i` to the next — none along
/// a pole.
pub(crate) struct Boundary {
    pub uv: Vec<Point>,
    pub on: Vec<On>,
    pub edge: Vec<Option<EdgeId>>,
}

/// The trim loop anchored at `first` of `face`, from the polylines `edges`
/// its edges were sampled into: each sample placed on the surface by
/// Newton from where the coedge's pcurve has it, as far along — a free
/// choice of seed. A coedge at a pole is its pcurve, `n` points of it.
pub(crate) fn boundary(
    model: &Model,
    face: &Face,
    first: CoedgeId,
    n: usize,
    edges: &BTreeMap<EdgeId, Vec<Vector3>>,
) -> GeopResult<Boundary> {
    let mut b = Boundary {
        uv: Vec::new(),
        on: Vec::new(),
        edge: Vec::new(),
    };
    let plain = |p: Vector2| [p[0].to_f64(), p[1].to_f64()];
    for c in model.iterate_loop_coedges(first) {
        let coedge = model.get_coedge(c)?;
        let (s0, s1) = coedge.pcurve.domain();
        let along = |j: usize, m: usize| -> GeopResult<Vector2> {
            let f = Scalar::from_ratio(j as i64, m as i64)?;
            coedge.pcurve.evaluate(s0.add(s1.sub(s0).mul(f)))
        };
        match coedge.geometry {
            CoedgeGeometry::Edge(e) => {
                let edge = model.get_edge(e)?;
                let points = &edges[&e];
                let m = points.len() - 1;
                let forward = coedge.sense == Sense::Forward;
                for j in 0..m {
                    let k = if forward { j } else { m - j };
                    let seed = along(j, m)?;
                    let at = match (j, k) {
                        // A vertex: where the pcurve ends.
                        (0, _) => seed,
                        _ => face
                            .surface
                            .project(points[k], seed[0], seed[1], PROJECTION_ITERATIONS)
                            .map_or(seed, |(u, v)| Vector2::from_array([u, v])),
                    };
                    b.uv.push(plain(at));
                    b.on.push(match k {
                        0 => On::Vertex(edge.start_vertex),
                        k if k == m => On::Vertex(edge.end_vertex),
                        k => On::Edge(e, k as f64),
                    });
                    b.edge.push(Some(e));
                }
            }
            CoedgeGeometry::Vertex(v) => {
                let m = n.max(2);
                for j in 0..m {
                    b.uv.push(plain(along(j, m)?));
                    b.on.push(On::Vertex(v));
                    b.edge.push(None);
                }
            }
        }
    }
    Ok(b)
}

/// The trim of a face, and how to tell where along it a point lies.
pub(crate) struct Trim<'a> {
    pub model: &'a Model,
    pub boundaries: &'a [Boundary],
    /// Per edge, the index of its polyline's last sample.
    pub last: &'a BTreeMap<EdgeId, usize>,
}

impl Trim<'_> {
    /// How close to a side a point has to be to lie on it.
    fn tolerance(&self) -> f64 {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for p in self.boundaries.iter().flat_map(|b| &b.uv) {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        let size = (hi[0] - lo[0]).hypot(hi[1] - lo[1]);
        ON_SIDE * if size.is_finite() { size } else { 1.0 }
    }

    /// The side of the trim `p` lies on, strictly between its ends: the
    /// loop, the side, and how far along it.
    fn side_of(&self, p: Point) -> Option<(usize, usize, f64)> {
        let tolerance = self.tolerance();
        for (l, b) in self.boundaries.iter().enumerate() {
            let n = b.uv.len();
            for i in 0..n {
                if let Some((t, d)) = along(p, b.uv[i], b.uv[(i + 1) % n])
                    && d <= tolerance
                {
                    return Some((l, i, t));
                }
            }
        }
        None
    }

    /// The positions along its edge of side `i` of loop `l`'s two ends.
    fn ends(&self, l: usize, i: usize) -> Option<(EdgeId, f64, f64)> {
        let b = &self.boundaries[l];
        let e = b.edge[i]?;
        let m = *self.last.get(&e)?;
        let (from, to) = (b.on[i], b.on[(i + 1) % b.on.len()]);
        let at = |on: On, other: On| -> Option<f64> {
            match on {
                On::Edge(f, x) if f == e => Some(x),
                On::Vertex(v) => {
                    let edge = self.model.get_edge(e).ok()?;
                    // At an edge that starts where it ends, the end next
                    // to the side's other end.
                    let near_start = match other {
                        On::Edge(_, x) => x < m as f64 / 2.0,
                        _ => edge.start_vertex == v,
                    };
                    match (edge.start_vertex == v, edge.end_vertex == v) {
                        (true, true) => Some(if near_start { 0.0 } else { m as f64 }),
                        (true, false) => Some(0.0),
                        (false, true) => Some(m as f64),
                        (false, false) => None,
                    }
                }
                _ => None,
            }
        };
        Some((e, at(from, to)?, at(to, from)?))
    }

    /// Where `p`, a point of the mesh, lies — on a corner of the trim, on
    /// a side of it at its position there, or inside — and the `(u, v)` it
    /// is kept at: a corner's own, where it is one.
    fn locate(&self, p: Point) -> (On, Place, Point) {
        // A point the rounding left a hair off a corner of the trim is that
        // corner: one point in space, not two a hair apart.
        let tolerance = self.tolerance();
        for (l, b) in self.boundaries.iter().enumerate() {
            for (i, q) in b.uv.iter().enumerate() {
                if distance2(p, *q) <= tolerance {
                    return (b.on[i], Place::Corner(l, i), *q);
                }
            }
        }
        let Some((l, i, t)) = self.side_of(p) else {
            return (On::Inside, Place::Inside(key(p)), p);
        };
        let b = &self.boundaries[l];
        let n = b.uv.len();
        let corner = |i: usize| (b.on[i], Place::Corner(l, i), b.uv[i]);
        let Some((e, x, y)) = self.ends(l, i) else {
            // Along a pole, every point is the pole.
            return corner(i);
        };
        // Counted in samples, the same for every face: a rounding off one
        // is the sample.
        let at = x + (y - x) * t;
        let near = at.round();
        let at = if (at - near).abs() <= SAME_POSITION {
            near
        } else {
            at
        };
        if at == x {
            corner(i)
        } else if at == y {
            corner((i + 1) % n)
        } else {
            (On::Edge(e, at), Place::Side(l, i, at.to_bits()), p)
        }
    }
}

/// Where a mesh point is, as a key: see [`Mesh::new`].
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
enum Place {
    Inside([u64; 2]),
    /// Corner `i` of trim loop `l`.
    Corner(usize, usize),
    /// On side `i` of trim loop `l`, at a position along its edge.
    Side(usize, usize, u64),
}

/// A face's triangles, by their corners' indices into its points.
pub(crate) struct Mesh {
    pub uv: Vec<Point>,
    pub on: Vec<On>,
    pub triangles: Vec<[usize; 3]>,
}

impl Mesh {
    /// The mesh of the `(u, v)` triangles `triangles` of a face trimmed by
    /// `trim`: every point once, each where on the trim it lies, made
    /// conforming within the face (see [`Mesh::conform`]).
    pub(crate) fn new(triangles: &[[Point; 3]], trim: &Trim) -> GeopResult<Self> {
        let mut mesh = Mesh {
            uv: Vec::new(),
            on: Vec::new(),
            triangles: Vec::new(),
        };
        let mut corners: HashMap<[u64; 2], (On, Place, Point)> = HashMap::new();
        for (l, b) in trim.boundaries.iter().enumerate() {
            for (i, (p, on)) in b.uv.iter().zip(&b.on).enumerate() {
                corners.insert(key(*p), (*on, Place::Corner(l, i), *p));
            }
        }
        // Points by where they are: inside, by their `(u, v)`; on the trim,
        // by where on it — two points the clipping put a rounding apart on
        // one side are one.
        let mut located: HashMap<[u64; 2], (On, Place, Point)> = HashMap::new();
        let mut index: HashMap<Place, usize> = HashMap::new();
        for t in triangles {
            let mut corner = [0; 3];
            for (k, &p) in t.iter().enumerate() {
                let (on, place, at) = *located.entry(key(p)).or_insert_with(|| {
                    corners
                        .get(&key(p))
                        .copied()
                        .unwrap_or_else(|| trim.locate(p))
                });
                corner[k] = *index.entry(place).or_insert_with(|| {
                    mesh.uv.push(at);
                    mesh.on.push(on);
                    mesh.uv.len() - 1
                });
            }
            if corner[0] != corner[1] && corner[1] != corner[2] && corner[2] != corner[0] {
                mesh.triangles.push(corner);
            }
        }
        mesh.merge_close(trim.tolerance());
        mesh.conform(trim.tolerance());
        Ok(mesh)
    }

    /// Points within `tolerance` of each other made one, unless both are
    /// on the trim apart from one edge: where two cells clipped one line
    /// each on its own, a rounding apart.
    fn merge_close(&mut self, tolerance: f64) {
        let mut by_u: Vec<usize> = (0..self.uv.len()).collect();
        by_u.sort_by(|&a, &b| self.uv[a][0].total_cmp(&self.uv[b][0]));
        let mut same: Vec<usize> = (0..self.uv.len()).collect();
        for (k, &a) in by_u.iter().enumerate() {
            if same[a] != a {
                continue;
            }
            for &b in &by_u[k + 1..] {
                if self.uv[b][0] - self.uv[a][0] > tolerance {
                    break;
                }
                let inside = |p: usize| self.on[p] == On::Inside;
                // Two points on one edge a rounding apart: two cells
                // clipped it along one line each on its own.
                let one_edge = matches!(
                    (self.on[a], self.on[b]),
                    (On::Edge(e, _), On::Edge(f, _)) if e == f
                );
                if same[b] == b
                    && (inside(a) || inside(b) || one_edge)
                    && distance2(self.uv[a], self.uv[b]) <= tolerance
                {
                    // The one on the trim stays where it is.
                    let (keep, gone) = if inside(b) { (a, b) } else { (b, a) };
                    same[gone] = keep;
                }
            }
        }
        let root = |mut p: usize| {
            while same[p] != p {
                p = same[p];
            }
            p
        };
        self.triangles = self
            .triangles
            .iter()
            .map(|t| t.map(root))
            .filter(|t| t[0] != t[1] && t[1] != t[2] && t[2] != t[0])
            .collect();
    }

    /// The sides of triangles no other triangle shares: `(a, b, triangle)`,
    /// running as the triangle runs them.
    fn open_sides(&self) -> Vec<(usize, usize, usize)> {
        let mut uses: HashMap<(usize, usize), usize> = HashMap::new();
        for t in &self.triangles {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *uses.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let mut open = Vec::new();
        for (i, t) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if uses[&(a.min(b), a.max(b))] == 1 {
                    open.push((a, b, i));
                }
            }
        }
        open
    }

    /// Splits triangle `i` along its side from `a` to `b` at `points`, in
    /// order from `a`: a fan from its third corner.
    fn split(&mut self, i: usize, a: usize, b: usize, points: &[usize]) {
        let t = self.triangles[i];
        let k = (0..3)
            .find(|&k| t[k] == a)
            .expect("a corner of the triangle");
        let c = t[(k + 2) % 3];
        let mut chain = vec![a];
        chain.extend_from_slice(points);
        chain.push(b);
        self.triangles[i] = [chain[0], chain[1], c];
        for w in chain[1..].windows(2) {
            self.triangles.push([w[0], w[1], c]);
        }
    }

    /// Splits every open side (see [`Mesh::open_sides`]) at the points of
    /// the mesh lying on it within `tolerance`, strictly between its ends:
    /// a T where a clipped cell meets a whole one, or where two cells
    /// clipped the trim each on its own.
    fn conform(&mut self, tolerance: f64) {
        while self.conform_once(tolerance) {}
    }

    /// One pass of [`Mesh::conform`], each triangle split once at most:
    /// whether any was.
    fn conform_once(&mut self, tolerance: f64) -> bool {
        let mut split = HashSet::new();
        let mut by_u: Vec<usize> = (0..self.uv.len()).collect();
        by_u.sort_by(|&a, &b| self.uv[a][0].total_cmp(&self.uv[b][0]));
        let us: Vec<f64> = by_u.iter().map(|&i| self.uv[i][0]).collect();
        for (a, b, i) in self.open_sides() {
            if split.contains(&i) {
                continue;
            }
            let (pa, pb) = (self.uv[a], self.uv[b]);
            let (lo, hi) = (pa[0].min(pb[0]) - tolerance, pa[0].max(pb[0]) + tolerance);
            let start = us.partition_point(|&u| u < lo);
            let end = us.partition_point(|&u| u <= hi);
            let mut on: Vec<(f64, usize)> = by_u[start..end]
                .iter()
                .filter(|&&p| p != a && p != b)
                .filter_map(|&p| {
                    let (t, d) = along(self.uv[p], pa, pb)?;
                    (d <= tolerance).then_some((t, p))
                })
                .collect();
            if on.is_empty() {
                continue;
            }
            on.sort_by(|x, y| x.0.total_cmp(&y.0));
            let points: Vec<usize> = on.into_iter().map(|(_, p)| p).collect();
            self.split(i, a, b, &points);
            split.insert(i);
        }
        !split.is_empty()
    }

    /// Splits each open side along the trim at every position on its edge
    /// in `positions` strictly between its ends: what other faces put
    /// there (see [`zip`]).
    fn zip(&mut self, trim: &Trim, positions: &BTreeMap<EdgeId, Vec<f64>>) {
        while self.zip_once(trim, positions) {}
    }

    /// One pass of [`Mesh::zip`], each triangle split once at most:
    /// whether any was.
    fn zip_once(&mut self, trim: &Trim, positions: &BTreeMap<EdgeId, Vec<f64>>) -> bool {
        let mut split = HashSet::new();
        for (a, b, i) in self.open_sides() {
            if split.contains(&i) {
                continue;
            }
            let (pa, pb) = (self.uv[a], self.uv[b]);
            let middle = [(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0];
            let Some((l, side, _)) = trim.side_of(middle) else {
                continue;
            };
            let Some((e, x, y)) = trim.ends(l, side) else {
                continue;
            };
            // Each end's position: its own on the edge, or the side's end
            // it is.
            let at = |p: usize| match self.on[p] {
                On::Edge(f, z) if f == e => Some(z),
                On::Vertex(_) => {
                    let s = &trim.boundaries[l].uv;
                    let to_start = distance2(self.uv[p], s[side]);
                    let to_end = distance2(self.uv[p], s[(side + 1) % s.len()]);
                    Some(if to_start <= to_end { x } else { y })
                }
                _ => None,
            };
            let (Some(xa), Some(xb)) = (at(a), at(b)) else {
                continue;
            };
            let Some(list) = positions.get(&e) else {
                continue;
            };
            let (lo, hi) = (xa.min(xb), xa.max(xb));
            let mut between: Vec<f64> =
                list.iter().copied().filter(|&z| lo < z && z < hi).collect();
            if between.is_empty() {
                continue;
            }
            if xa > xb {
                between.reverse();
            }
            let points: Vec<usize> = between
                .into_iter()
                .map(|z| {
                    let t = (z - xa) / (xb - xa);
                    self.uv
                        .push([pa[0] + (pb[0] - pa[0]) * t, pa[1] + (pb[1] - pa[1]) * t]);
                    self.on.push(On::Edge(e, z));
                    self.uv.len() - 1
                })
                .collect();
            self.split(i, a, b, &points);
            split.insert(i);
        }
        !split.is_empty()
    }
}

/// Puts every point any face put on an edge on it in every face the edge
/// bounds (see the module docs): each of `meshes`, trimmed by `trims`.
pub(crate) fn zip(meshes: &mut [Mesh], trims: &[Trim]) {
    let mut positions: BTreeMap<EdgeId, Vec<f64>> = BTreeMap::new();
    for mesh in meshes.iter() {
        for &on in &mesh.on {
            if let On::Edge(e, at) = on {
                positions.entry(e).or_default().push(at);
            }
        }
    }
    // Positions a rounding apart, put there by two faces each on its own,
    // are one: the first of them.
    for list in positions.values_mut() {
        list.sort_by(f64::total_cmp);
        list.dedup_by(|b, a| *b - *a <= SAME_POSITION);
    }
    for mesh in meshes.iter_mut() {
        for on in &mut mesh.on {
            if let On::Edge(e, at) = on {
                let list = &positions[e];
                let k = list.partition_point(|&x| x < *at - SAME_POSITION);
                if let Some(&x) = list.get(k)
                    && (x - *at).abs() <= SAME_POSITION
                {
                    *at = x;
                }
            }
        }
    }
    for (mesh, trim) in meshes.iter_mut().zip(trims) {
        mesh.zip(trim, &positions);
    }
}

/// How far along the segment from `a` to `b` the point `p` is, strictly
/// between its ends, and how far off it: none past either end.
fn along(p: Point, a: Point, b: Point) -> Option<(f64, f64)> {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
    if len2 == 0.0 {
        return None;
    }
    let t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / len2;
    if t <= 0.0 || t >= 1.0 {
        return None;
    }
    let q = [a[0] + ab[0] * t, a[1] + ab[1] * t];
    Some((t, distance2(p, q)))
}

fn distance2(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn key(p: Point) -> [u64; 2] {
    [p[0].to_bits(), p[1].to_bits()]
}

/// The point `on` stands for, if it is on an edge or a vertex: on the
/// edge's polyline `edges[e]` at its position, or the vertex's point.
pub(crate) fn placed(
    model: &Model,
    on: On,
    edges: &BTreeMap<EdgeId, Vec<Vector3>>,
) -> GeopResult<Option<Vector3>> {
    Ok(match on {
        On::Inside => None,
        On::Vertex(v) => Some(model.get_vertex(v)?.point),
        On::Edge(e, at) => {
            let points = &edges[&e];
            let k = (at.floor() as usize).min(points.len() - 2);
            let f = at - k as f64;
            Some(if f == 0.0 {
                points[k]
            } else {
                let (a, b) = (points[k].to_f64_array(), points[k + 1].to_f64_array());
                Vector3::from_array(std::array::from_fn(|i| {
                    Scalar::from_f64(a[i] + (b[i] - a[i]) * f)
                }))
            })
        }
    })
}
