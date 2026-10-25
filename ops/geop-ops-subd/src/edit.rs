//! What a cage starts as — a box, a cylinder, a sphere, a plane — and the
//! edits that shape it: moving, turning and scaling its elements,
//! extruding and insetting faces, bevelling edges, cutting and sliding
//! edge loops, merging and dissolving, bridging two faces, filling a hole,
//! creasing edges, deleting faces, and cutting it in half at a mirror
//! plane or making the mirrored half its own.
//!
//! An edit keeps the id of every element it does not take away, and gives
//! what it adds ids no element has had, so that names built from them stay
//! put (see [`crate::cage`]). Every edit is a function of the cage and its
//! arguments alone, its order fixed by the ids, so that the same edit of
//! the same cage makes the same cage.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain::{add, centroid, cross, dot, norm, normalize, scale, sub},
};

use crate::cage::{
    Cage, CageFace, CageVertex, Crease, Mesh, Mirror, edge_key, face_key, next, prev, undirected,
    vertex_key,
};

/// An element of a cage, as a selection names it (see
/// [`crate::cage::vertex_key`], [`crate::cage::edge_key`],
/// [`crate::cage::face_key`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Element {
    Vertex(u32),
    /// By its two vertices, the smaller id first.
    Edge(u32, u32),
    Face(u32),
}

impl Element {
    /// The element `key` names: `v3`, `e3-5`, `f7`.
    pub fn parse(key: &str) -> Option<Element> {
        let (kind, rest) = key.split_at_checked(1)?;
        match kind {
            "v" => rest.parse().ok().map(Element::Vertex),
            "f" => rest.parse().ok().map(Element::Face),
            "e" => {
                let (a, b) = rest.split_once('-')?;
                let (a, b): (u32, u32) = (a.parse().ok()?, b.parse().ok()?);
                Some(Element::Edge(a.min(b), a.max(b)))
            }
            _ => None,
        }
    }

    /// The key that names the element.
    pub fn key(self) -> String {
        match self {
            Element::Vertex(v) => vertex_key(v),
            Element::Edge(a, b) => edge_key(a, b),
            Element::Face(f) => face_key(f),
        }
    }
}

/// A cage built from `points` and faces through them by index, ids given
/// in that order: vertices first.
fn cage_of(points: Vec<[f64; 3]>, faces: Vec<Vec<usize>>) -> Cage {
    let n = points.len() as u32;
    Cage {
        vertices: points
            .into_iter()
            .enumerate()
            .map(|(i, at)| CageVertex { id: i as u32, at })
            .collect(),
        faces: faces
            .into_iter()
            .enumerate()
            .map(|(i, f)| CageFace {
                id: n + i as u32,
                vertices: f.into_iter().map(|v| v as u32).collect(),
            })
            .collect(),
        creases: Vec::new(),
        next_id: 0,
    }
    .numbered()
}

/// Edges each directed the way a face runs along it.
type DirectedEdges = BTreeSet<(u32, u32)>;

/// The vertices of `face` as a cycle starting at `at`, which it runs
/// through.
fn rotated(face: &[u32], at: u32) -> Vec<u32> {
    let k = face.iter().position(|&v| v == at).expect("on its face");
    face[k..].iter().chain(&face[..k]).copied().collect()
}

/// `cycle` with each run of equal neighbours — around the end too — kept
/// once.
fn deduplicated(cycle: &[u32]) -> Vec<u32> {
    let mut out: Vec<u32> = Vec::with_capacity(cycle.len());
    for &v in cycle {
        if out.last() != Some(&v) {
            out.push(v);
        }
    }
    while out.len() > 1 && out.first() == out.last() {
        out.pop();
    }
    out
}

/// The vertex `cycle` runs through twice, if any.
fn repeated(cycle: &[u32]) -> Option<u32> {
    let mut seen = BTreeSet::new();
    cycle.iter().copied().find(|&v| !seen.insert(v))
}

impl Cage {
    /// The same cage with `next_id` past every id in it.
    fn numbered(mut self) -> Self {
        let ids = self
            .vertices
            .iter()
            .map(|v| v.id)
            .chain(self.faces.iter().map(|f| f.id));
        self.next_id = ids.max().map_or(0, |m| m + 1);
        self
    }

    /// A box around the origin, `size` long along each axis: eight
    /// vertices, six quads.
    pub fn cuboid(size: [f64; 3]) -> Self {
        let [x, y, z] = size.map(|s| s / 2.0);
        let points = vec![
            [-x, -y, -z],
            [x, -y, -z],
            [x, y, -z],
            [-x, y, -z],
            [-x, -y, z],
            [x, -y, z],
            [x, y, z],
            [-x, y, z],
        ];
        let faces = vec![
            vec![0, 3, 2, 1],
            vec![4, 5, 6, 7],
            vec![0, 1, 5, 4],
            vec![1, 2, 6, 5],
            vec![2, 3, 7, 6],
            vec![3, 0, 4, 7],
        ];
        cage_of(points, faces)
    }

    /// A cylinder around the `z` axis, centred on the origin: `segments`
    /// quads around, two rows high, closed by an n-gon at each end.
    pub fn cylinder(radius: f64, height: f64, segments: usize) -> Self {
        let rows = 2;
        let mut points = Vec::new();
        for r in 0..=rows {
            let z = height * (r as f64 / rows as f64 - 0.5);
            for i in 0..segments {
                let a = 2.0 * std::f64::consts::PI * i as f64 / segments as f64;
                points.push([radius * a.cos(), radius * a.sin(), z]);
            }
        }
        let at = |r: usize, i: usize| r * segments + i % segments;
        let mut faces = vec![(0..segments).rev().map(|i| at(0, i)).collect::<Vec<_>>()];
        faces.push((0..segments).map(|i| at(rows, i)).collect());
        for r in 0..rows {
            for i in 0..segments {
                faces.push(vec![at(r, i), at(r, i + 1), at(r + 1, i + 1), at(r + 1, i)]);
            }
        }
        cage_of(points, faces)
    }

    /// A sphere-like cage around the origin: a cube of 2 x 2 quads a side,
    /// its vertices pushed out onto the sphere of `radius`.
    pub fn sphere(radius: f64) -> Self {
        let mut index: HashMap<[i32; 3], usize> = HashMap::new();
        let mut points = Vec::new();
        let mut faces = Vec::new();
        for axis in 0..3 {
            for sign in [-1, 1] {
                // `u x v` points out of the cube face.
                let (u, v) = ((axis + 1) % 3, (axis + 2) % 3);
                let (u, v) = if sign > 0 { (u, v) } else { (v, u) };
                let mut vertex = |a: i32, b: i32| {
                    let mut c = [0; 3];
                    c[axis] = sign;
                    c[u] = a;
                    c[v] = b;
                    *index.entry(c).or_insert_with(|| {
                        let p = c.map(f64::from);
                        let length = norm(p);
                        points.push(p.map(|x| radius * x / length));
                        points.len() - 1
                    })
                };
                for a in -1..1 {
                    for b in -1..1 {
                        faces.push(vec![
                            vertex(a, b),
                            vertex(a + 1, b),
                            vertex(a + 1, b + 1),
                            vertex(a, b + 1),
                        ]);
                    }
                }
            }
        }
        cage_of(points, faces)
    }

    /// A flat square in the `z = 0` plane around the origin, `size` wide,
    /// of `n` x `n` quads facing `+z`: an open cage, whose limit is a
    /// sheet.
    pub fn plane(size: f64, n: usize) -> Self {
        let mut points = Vec::new();
        for j in 0..=n {
            for i in 0..=n {
                let at = |k: usize| size * (k as f64 / n as f64 - 0.5);
                points.push([at(i), at(j), 0.0]);
            }
        }
        let at = |i: usize, j: usize| j * (n + 1) + i;
        let mut faces = Vec::new();
        for j in 0..n {
            for i in 0..n {
                faces.push(vec![at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j + 1)]);
            }
        }
        cage_of(points, faces)
    }

    /// Whether the cage has the element `element`.
    pub fn has(&self, element: Element) -> bool {
        match element {
            Element::Vertex(v) => self.vertices.iter().any(|x| x.id == v),
            Element::Face(f) => self.faces.iter().any(|x| x.id == f),
            Element::Edge(a, b) => self.edges().contains(&[a, b]),
        }
    }

    /// The vertices of the elements `keys` name, by id.
    pub fn vertices_of(&self, keys: &[String]) -> BTreeSet<u32> {
        let mut vertices = BTreeSet::new();
        for key in keys {
            match Element::parse(key) {
                Some(Element::Vertex(v)) => {
                    vertices.insert(v);
                }
                Some(Element::Edge(a, b)) => {
                    vertices.extend([a, b]);
                }
                Some(Element::Face(f)) => {
                    if let Ok(face) = self.face(f) {
                        vertices.extend(face.vertices.iter().copied());
                    }
                }
                None => {}
            }
        }
        vertices.retain(|&v| self.vertex(v).is_ok());
        vertices
    }

    /// The centre of the vertices `vertices`: their average.
    pub fn centre(&self, vertices: &BTreeSet<u32>) -> Option<[f64; 3]> {
        let ats = vertices
            .iter()
            .map(|&v| Some(self.vertex(v).ok()?.at))
            .collect::<Option<Vec<_>>>()?;
        centroid(ats)
    }

    /// Whether vertex `v` lies on the plane of `mirror`, where the mirror
    /// image joins the cage.
    fn on_plane(&self, v: u32, mirror: Mirror) -> GeopResult<bool> {
        let at = self.vertex(v)?.at;
        Ok(mirror.axis().is_some_and(|a| at[a] == 0.0))
    }

    /// The vertices `vertices` moved to where `f` takes them — those on the
    /// plane of `mirror` kept on it, where the mirror image joins them.
    pub fn transform(
        &mut self,
        vertices: &BTreeSet<u32>,
        mirror: Mirror,
        f: impl Fn([f64; 3]) -> [f64; 3],
    ) {
        for vertex in &mut self.vertices {
            if vertices.contains(&vertex.id) {
                let on_plane = mirror.axis().filter(|&a| vertex.at[a] == 0.0);
                vertex.at = f(vertex.at);
                if let Some(axis) = on_plane {
                    vertex.at[axis] = 0.0;
                }
            }
        }
    }

    /// Creases the edges `edges` at `sharpness` (see [`Crease`]): one is
    /// sharp, zero or less smooths them again, and more than one is sharp.
    pub fn set_sharpness(&mut self, edges: &[[u32; 2]], sharpness: f64) {
        for &[a, b] in edges {
            let edge = undirected(a, b);
            self.creases.retain(|c| c.edge != edge);
            if sharpness > 0.0 {
                self.creases.push(Crease {
                    edge,
                    sharpness: sharpness.min(1.0),
                });
            }
        }
        self.creases.sort_by_key(|c| c.edge);
    }

    /// Renames the vertices of every crease by `to`, dropping a crease
    /// whose two vertices became one.
    fn rename_creases(&mut self, to: impl Fn([u32; 2]) -> [u32; 2]) {
        let mut creases: Vec<Crease> = std::mem::take(&mut self.creases);
        for c in &mut creases {
            let [a, b] = to(c.edge);
            c.edge = undirected(a, b);
        }
        creases.retain(|c| c.edge[0] != c.edge[1]);
        creases.sort_by_key(|c| c.edge);
        creases.dedup_by_key(|c| c.edge);
        self.creases = creases;
    }

    /// Forgets vertices no face runs through any more, and creases of
    /// edges no face has any more.
    fn tidy(&mut self) {
        let used: BTreeSet<u32> = self
            .faces
            .iter()
            .flat_map(|f| f.vertices.iter().copied())
            .collect();
        self.vertices.retain(|v| used.contains(&v.id));
        let edges = self.edges();
        self.creases.retain(|c| edges.contains(&c.edge));
    }

    /// Deletes the faces `faces`, and every vertex and crease that leaves
    /// without a face.
    pub fn delete_faces(&mut self, faces: &BTreeSet<u32>) {
        self.faces.retain(|f| !faces.contains(&f.id));
        self.tidy();
    }

    /// The faces any of the edges `edges` is on, by id.
    pub fn faces_along(&self, edges: &[[u32; 2]]) -> BTreeSet<u32> {
        let directed = self.directed_edges();
        edges
            .iter()
            .flat_map(|&[a, b]| [(a, b), (b, a)])
            .filter_map(|e| directed.get(&e).map(|&(f, _)| self.faces[f].id))
            .collect()
    }

    /// The faces any of the vertices `vertices` is a corner of, by id.
    pub fn faces_around(&self, vertices: &BTreeSet<u32>) -> BTreeSet<u32> {
        (self.faces.iter())
            .filter(|f| f.vertices.iter().any(|v| vertices.contains(v)))
            .map(|f| f.id)
            .collect()
    }

    /// The orientation of the faces `faces`, for moving them along it: `z`
    /// along their normals together, `x` along the first one's first edge,
    /// both unit and square to each other. `None` for no faces, or faces
    /// whose normals cancel out.
    pub fn frame(&self, faces: &BTreeSet<u32>) -> Option<[[f64; 3]; 3]> {
        let mut normal = [0.0; 3];
        for &f in faces {
            let n = self.normal(self.face(f).ok()?).ok()?;
            (0..3).for_each(|i| normal[i] += n[i]);
        }
        let w = normalize(normal)?;
        let first = self.face(*faces.first()?).ok()?;
        let a = self.vertex(*first.vertices.first()?).ok()?.at;
        let b = self.vertex(*first.vertices.get(1)?).ok()?.at;
        let edge = sub(b, a);
        let u = normalize(sub(edge, scale(w, dot(edge, w))))?;
        Some([u, cross(w, u), w])
    }

    /// The outward normal of face `face`, by Newell's method: as long as
    /// its area.
    fn normal(&self, face: &CageFace) -> GeopResult<[f64; 3]> {
        let mut normal = [0.0; 3];
        let n = face.vertices.len();
        for k in 0..n {
            let a = self.vertex(face.vertices[k])?.at;
            let b = self.vertex(face.vertices[(k + 1) % n])?.at;
            let c = cross(a, b);
            for i in 0..3 {
                normal[i] += c[i] / 2.0;
            }
        }
        Ok(normal)
    }

    /// The unit direction from the edge `(a, b)` of face `face` into the
    /// face, square to the edge in the face's plane: to the left of the
    /// edge the way the face runs. `None` for a face of no area.
    fn inward(&self, face: &CageFace, a: u32, b: u32) -> GeopResult<Option<[f64; 3]>> {
        let n = self.normal(face)?;
        let d = sub(self.vertex(b)?.at, self.vertex(a)?.at);
        let runs_a_to_b = (face.vertices.iter())
            .position(|&v| v == a)
            .is_some_and(|k| face.vertices[(k + 1) % face.vertices.len()] == b);
        let d = if runs_a_to_b { d } else { scale(d, -1.0) };
        Ok(normalize(cross(n, d)))
    }

    /// The edges of the faces `faces` with a face of the region on one side
    /// only, directed the way the region's face runs; and every directed
    /// edge of the cage.
    fn rim(&self, faces: &BTreeSet<u32>) -> (Vec<(u32, u32)>, DirectedEdges) {
        let mut directed: BTreeSet<(u32, u32)> = BTreeSet::new();
        let mut everywhere: BTreeSet<(u32, u32)> = BTreeSet::new();
        for face in &self.faces {
            let n = face.vertices.len();
            for k in 0..n {
                let edge = (face.vertices[k], face.vertices[(k + 1) % n]);
                everywhere.insert(edge);
                if faces.contains(&face.id) {
                    directed.insert(edge);
                }
            }
        }
        let rim = directed
            .iter()
            .copied()
            .filter(|&(a, b)| !directed.contains(&(b, a)))
            .collect();
        (rim, everywhere)
    }

    /// Moves the region of faces `faces` by `offset` per vertex, keeping
    /// the faces' ids, and joins each edge on the region's rim to its moved
    /// copy by a new quad. A vertex inside the region just moves; one on
    /// its rim is left behind for the faces around, and a new one moves.
    /// With a `mirror`, what is on its plane stays on it, and an open edge
    /// along the plane gets no quad, which would lie in it. A crease whose
    /// edge only the region had moves with it; one on its rim stays. The
    /// name is for the error messages.
    fn offset_region(
        &mut self,
        name: &str,
        faces: &BTreeSet<u32>,
        mirror: Mirror,
        offset: impl Fn(&Cage, u32) -> GeopResult<[f64; 3]>,
    ) -> GeopResult<()> {
        if faces.is_empty() {
            return Err(GeopError::new(format!("{name}: no face selected")));
        }
        for &f in faces {
            self.face(f)?;
        }
        let (rim, everywhere) = self.rim(faces);
        let region: BTreeSet<u32> = (self.faces.iter())
            .filter(|f| faces.contains(&f.id))
            .flat_map(|f| f.vertices.iter().copied())
            .collect();
        let mut offsets: BTreeMap<u32, [f64; 3]> = BTreeMap::new();
        for &v in &region {
            let mut d = offset(self, v)?;
            if self.on_plane(v, mirror)? {
                d[mirror.axis().expect("on a plane")] = 0.0;
            }
            if !d.iter().all(|c| c.is_finite()) {
                return Err(GeopError::new(format!(
                    "{name}: vertex {} cannot be moved: {d:?}",
                    vertex_key(v)
                )));
            }
            offsets.insert(v, d);
        }
        let rim_vertices: BTreeSet<u32> = rim.iter().flat_map(|&(a, b)| [a, b]).collect();
        let mut moved: BTreeMap<u32, u32> = BTreeMap::new();
        for (&v, d) in &offsets {
            let to = add(self.vertex(v)?.at, *d);
            if rim_vertices.contains(&v) {
                let id = self.fresh_id();
                self.vertices.push(CageVertex { id, at: to });
                moved.insert(v, id);
            } else {
                self.vertex_mut(v)?.at = to;
            }
        }
        for face in &mut self.faces {
            if faces.contains(&face.id) {
                for v in &mut face.vertices {
                    if let Some(&m) = moved.get(v) {
                        *v = m;
                    }
                }
            }
        }
        for (a, b) in rim {
            let open = !everywhere.contains(&(b, a));
            if open && self.on_plane(a, mirror)? && self.on_plane(b, mirror)? {
                continue;
            }
            let id = self.fresh_id();
            self.faces.push(CageFace {
                id,
                vertices: vec![a, b, moved[&b], moved[&a]],
            });
        }
        let edges = self.edges();
        let to = |v: u32| moved.get(&v).copied().unwrap_or(v);
        self.rename_creases(|edge| {
            if edges.contains(&edge) {
                edge
            } else {
                [to(edge[0]), to(edge[1])]
            }
        });
        self.tidy();
        Ok(())
    }

    /// Extrudes the faces `faces` `distance` along their normals (see
    /// [`Cage::offset_region`]).
    pub fn extrude(
        &mut self,
        faces: &BTreeSet<u32>,
        distance: f64,
        mirror: Mirror,
    ) -> GeopResult<()> {
        let mut normal: BTreeMap<u32, [f64; 3]> = BTreeMap::new();
        for face in self.faces.iter().filter(|f| faces.contains(&f.id)) {
            let n = self.normal(face)?;
            for &v in &face.vertices {
                let sum = normal.entry(v).or_insert([0.0; 3]);
                *sum = add(*sum, n);
            }
        }
        self.offset_region("extrude", faces, mirror, |cage, v| {
            let mut n = normal[&v];
            if cage.on_plane(v, mirror)? {
                n[mirror.axis().expect("on a plane")] = 0.0;
            }
            normalize(n).map(|n| scale(n, distance)).ok_or_else(|| {
                GeopError::new(format!(
                    "extrude: the faces around vertex {} face no one way",
                    vertex_key(v)
                ))
            })
        })
    }

    /// Insets the faces `faces`: their rim moves `thickness` inwards, in
    /// the faces' own planes, and a new ring of quads fills the gap to where
    /// the rim was (see [`Cage::offset_region`]). A vertex inside the region
    /// stays.
    pub fn inset(
        &mut self,
        faces: &BTreeSet<u32>,
        thickness: f64,
        mirror: Mirror,
    ) -> GeopResult<()> {
        if faces.is_empty() {
            return Err(GeopError::new("inset: no face selected"));
        }
        let (rim, _) = self.rim(faces);
        let mut inward: BTreeMap<u32, Vec<[f64; 3]>> = BTreeMap::new();
        let directed = self.directed_edges();
        for &(a, b) in &rim {
            let (f, _) = directed[&(a, b)];
            let Some(d) = self.inward(&self.faces[f], a, b)? else {
                return Err(GeopError::new(format!(
                    "inset: face {} has no area",
                    face_key(self.faces[f].id)
                )));
            };
            inward.entry(a).or_default().push(d);
            inward.entry(b).or_default().push(d);
        }
        self.offset_region("inset", faces, mirror, |_, v| {
            let Some(ds) = inward.get(&v) else {
                return Ok([0.0; 3]);
            };
            match ds[..] {
                [d] => Ok(scale(d, thickness)),
                [d, e] => {
                    // The mitre: `thickness` from both rim edges.
                    let along = 1.0 + dot(d, e);
                    if along <= 0.0 {
                        return Err(GeopError::new(format!(
                            "inset: the rim turns back on itself at vertex {}",
                            vertex_key(v)
                        )));
                    }
                    Ok(scale(add(d, e), thickness / along))
                }
                _ => Err(GeopError::new(format!(
                    "inset: the rim passes vertex {} more than once",
                    vertex_key(v)
                ))),
            }
        })
    }

    /// Cuts a new edge loop across the edge between `a` and `b`: through
    /// every edge of its ring (see [`Cage::edge_ring`]) at the fraction `t`
    /// of the way along it from `a`'s side, splitting each quad of the
    /// ring in two. The halves of a creased edge keep its crease.
    pub fn insert_loop(&mut self, a: u32, b: u32, t: f64) -> GeopResult<()> {
        if !(0.0 < t && t < 1.0) {
            return Err(GeopError::new(format!(
                "loop cut: the cut must be between the edge's ends, not at {t} of the way"
            )));
        }
        let ring = self.edge_ring(a, b)?;
        let mut middle: HashMap<[u32; 2], u32> = HashMap::new();
        for &(x, y) in &ring.edges {
            let (p, q) = (self.vertex(x)?.at, self.vertex(y)?.at);
            let id = self.fresh_id();
            self.vertices.push(CageVertex {
                id,
                at: [0, 1, 2].map(|c| p[c] + t * (q[c] - p[c])),
            });
            middle.insert(undirected(x, y), id);
            let sharpness = self.sharpness(x, y);
            if sharpness > 0.0 {
                self.set_sharpness(&[[x, y]], 0.0);
                self.set_sharpness(&[[x, id], [id, y]], sharpness);
            }
        }
        for face in &mut self.faces {
            let n = face.vertices.len();
            let mut vertices = Vec::with_capacity(n + 2);
            for k in 0..n {
                let (x, y) = (face.vertices[k], face.vertices[(k + 1) % n]);
                vertices.push(x);
                if let Some(&m) = middle.get(&undirected(x, y)) {
                    vertices.push(m);
                }
            }
            face.vertices = vertices;
        }
        for f in ring.quads {
            let vertices = self.faces[f].vertices.clone();
            let cuts: Vec<usize> = (0..vertices.len())
                .filter(|&i| middle.values().any(|&m| m == vertices[i]))
                .collect();
            let [i, j] = cuts[..] else {
                return Err(GeopError::new(format!(
                    "loop cut: face {} is crossed by the loop {} times",
                    face_key(self.faces[f].id),
                    cuts.len()
                )));
            };
            let first: Vec<u32> = vertices[i..=j].to_vec();
            let second: Vec<u32> = vertices[j..]
                .iter()
                .chain(&vertices[..=i])
                .copied()
                .collect();
            self.faces[f].vertices = first;
            let id = self.fresh_id();
            self.faces.push(CageFace {
                id,
                vertices: second,
            });
        }
        Ok(())
    }

    /// Slides the vertices of the chain of edges `edges` along the edges
    /// leaving the chain, by the fraction `factor` of their length: towards
    /// the side the faces running along the chain, oriented from its first
    /// edge, lie on for a positive factor, the other side for a negative
    /// one. Each vertex needs one edge to slide along on that side, which
    /// the quads of the chain's faces give it: a vertex where the chain
    /// branches, or where its faces disagree, is refused by name.
    pub fn edge_slide(
        &mut self,
        edges: &[[u32; 2]],
        factor: f64,
        mirror: Mirror,
    ) -> GeopResult<()> {
        if edges.is_empty() {
            return Err(GeopError::new("edge slide: no edge selected"));
        }
        if !(-1.0 < factor && factor < 1.0) {
            return Err(GeopError::new(format!(
                "edge slide: the slide must stay short of the next loop, not {factor} of the way"
            )));
        }
        let selected: BTreeSet<[u32; 2]> = edges.iter().map(|&[a, b]| undirected(a, b)).collect();
        let directed = self.directed_edges();
        // The selected edges at each vertex.
        let mut at: BTreeMap<u32, Vec<[u32; 2]>> = BTreeMap::new();
        for &e in &selected {
            at.entry(e[0]).or_default().push(e);
            at.entry(e[1]).or_default().push(e);
        }
        // Each chain oriented from its lowest edge, as directed edges.
        let mut oriented: BTreeMap<[u32; 2], (u32, u32)> = BTreeMap::new();
        for &start in &selected {
            if oriented.contains_key(&start) {
                continue;
            }
            let mut pending = vec![(start[0], start[1])];
            while let Some((x, y)) = pending.pop() {
                if oriented.insert(undirected(x, y), (x, y)).is_some() {
                    continue;
                }
                for (from, to) in [(x, y), (y, x)] {
                    let others: Vec<[u32; 2]> = at[&to]
                        .iter()
                        .copied()
                        .filter(|e| *e != undirected(x, y))
                        .collect();
                    if let [e] = others[..]
                        && !oriented.contains_key(&e)
                    {
                        let z = if e[0] == to { e[1] } else { e[0] };
                        // Onwards from `to` if we came in through `from`.
                        pending.push(if from == x { (to, z) } else { (z, to) });
                    }
                }
            }
        }
        // Where each vertex slides to: the vertex across from it in the
        // face on the chosen side of each of its chain edges.
        let mut target: BTreeMap<u32, u32> = BTreeMap::new();
        for &(x, y) in oriented.values() {
            let side = if factor > 0.0 { (x, y) } else { (y, x) };
            let Some(&(f, k)) = directed.get(&side) else {
                return Err(GeopError::new(format!(
                    "edge slide: edge {} has no face on that side",
                    edge_key(x, y)
                )));
            };
            let face = &self.faces[f].vertices;
            let n = face.len();
            // The face runs `side.0 -> side.1`: across from `side.1` is the
            // vertex after it, across from `side.0` the one before.
            let pairs = [(side.1, face[(k + 2) % n]), (side.0, face[(k + n - 1) % n])];
            for (v, across) in pairs {
                if let Some(&other) = target.get(&v)
                    && other != across
                {
                    return Err(GeopError::new(format!(
                        "edge slide: the faces along the chain disagree at vertex {}: it would slide to {} or to {}",
                        vertex_key(v),
                        vertex_key(other),
                        vertex_key(across)
                    )));
                }
                target.insert(v, across);
            }
        }
        for (v, t) in target {
            let p = self.vertex(v)?.at;
            let q = self.vertex(t)?.at;
            let mut to = [0, 1, 2].map(|c| p[c] + factor.abs() * (q[c] - p[c]));
            if self.on_plane(v, mirror)? {
                to[mirror.axis().expect("on a plane")] = 0.0;
            }
            self.vertex_mut(v)?.at = to;
        }
        Ok(())
    }

    /// Bevels the edges `edges`: each becomes a strip of one quad, `width`
    /// wide, between the two faces along it, and each vertex where three or
    /// more bevelled edges meet a face of its own. A vertex is split into
    /// one vertex per *sector* — the faces around it between two bevelled
    /// edges — placed `width` from both edges in the sector's first and
    /// last face. A vertex with one bevelled edge stays, and the strip ends
    /// at it in a cap: a vertex `width` into each of the edge's two faces,
    /// and a triangle between them and the vertex. Edges on the boundary
    /// cannot be bevelled.
    pub fn bevel(&mut self, edges: &[[u32; 2]], width: f64, mirror: Mirror) -> GeopResult<()> {
        if edges.is_empty() {
            return Err(GeopError::new("bevel: no edge selected"));
        }
        let (mesh, topology) = self.indexed()?;
        let ids: Vec<u32> = self.vertices.iter().map(|v| v.id).collect();
        let id_of = |i: usize| ids[i];
        let index_of: HashMap<u32, usize> = (self.vertices.iter().enumerate())
            .map(|(i, v)| (v.id, i))
            .collect();
        let mut bevelled: BTreeSet<[usize; 2]> = BTreeSet::new();
        for &[a, b] in edges {
            let (Some(&i), Some(&j)) = (index_of.get(&a), index_of.get(&b)) else {
                return Err(GeopError::new(format!(
                    "bevel: the cage has no edge {}",
                    edge_key(a, b)
                )));
            };
            if !topology.directed.contains_key(&(i, j)) && !topology.directed.contains_key(&(j, i))
            {
                return Err(GeopError::new(format!(
                    "bevel: the cage has no edge {}",
                    edge_key(a, b)
                )));
            }
            if topology.is_boundary(i, j) {
                return Err(GeopError::new(format!(
                    "bevel: edge {} is open — on the boundary or along the mirror plane — and has no second face to bevel between",
                    edge_key(a, b)
                )));
            }
            bevelled.insert(undirected(i, j));
        }
        let faces = &mesh.faces;
        let face_ids: Vec<u32> = self.faces.iter().map(|f| f.id).collect();
        let normals: Vec<[f64; 3]> = (self.faces.iter())
            .map(|f| self.normal(f))
            .collect::<GeopResult<_>>()?;
        // The unit direction from the edge between corner `(f, k)` and `w`
        // into face `f`, square to the edge in its plane (see
        // [`Cage::inward`]).
        let into = |(f, k): (usize, usize), w: usize| -> GeopResult<[f64; 3]> {
            let face = &faces[f];
            let d = sub(mesh.positions[w], mesh.positions[face[k]]);
            let d = if face[(k + 1) % face.len()] == w {
                d
            } else {
                scale(d, -1.0)
            };
            normalize(cross(normals[f], d)).ok_or_else(|| {
                GeopError::new(format!("bevel: face {} has no area", face_key(face_ids[f])))
            })
        };
        // Per touched vertex, per corner `(f, k)` of it: the new vertex of
        // the sector that corner is in.
        let mut sector_of: HashMap<(usize, usize), u32> = HashMap::new();
        // Per touched vertex: its sector vertices in fan order, and whether
        // the fan closes.
        let mut sectors: BTreeMap<usize, (Vec<u32>, bool)> = BTreeMap::new();
        // Per corner `(f, k)` of a vertex with one bevelled edge: the cap
        // vertex put into that face beside it — after the corner where the
        // face runs along the edge, before it on the other side.
        let mut caps: HashMap<(usize, usize), u32> = HashMap::new();
        let mut cap_faces: Vec<[u32; 3]> = Vec::new();
        let touched: BTreeSet<usize> = bevelled.iter().flat_map(|&[a, b]| [a, b]).collect();
        for &v in &touched {
            let (fan, closed) = &topology.fans[v];
            let m = fan.len();
            // Between `fan[i]` and `fan[i + 1]` lies the edge to `fan[i]`'s
            // previous corner; an open fan is cut at both ends too.
            let cut = |i: usize| -> Option<usize> {
                let w = prev(faces, fan[i]);
                (bevelled.contains(&undirected(v, w)) || (!closed && i + 1 == m)).then_some(w)
            };
            let first_cut = (0..m)
                .find(|&i| cut(i).is_some())
                .expect("a bevelled edge at v");
            let start = if *closed { (first_cut + 1) % m } else { 0 };
            let mut runs: Vec<Vec<usize>> = Vec::new();
            let mut run = Vec::new();
            for step in 0..m {
                let i = (start + step) % m;
                run.push(i);
                if cut(i).is_some() || step + 1 == m {
                    runs.push(std::mem::take(&mut run));
                }
            }
            let p = |i: usize| mesh.positions[i];
            let placed = |at: [f64; 3]| -> [f64; 3] {
                let mut at = at;
                if let Some(axis) = mirror.axis()
                    && p(v)[axis] == 0.0
                {
                    at[axis] = 0.0;
                }
                at
            };
            if let [run] = &runs[..]
                && *closed
            {
                // One bevelled edge all round: the vertex stays, and the
                // strip is capped at it. The run starts with the face
                // running from `v` along the edge, and ends with the one
                // running back.
                let (first, last) = (fan[run[0]], fan[run[run.len() - 1]]);
                let w = next(faces, first);
                let mut cap = [id_of(v), 0, 0];
                for (corner, slot) in [(first, 2), (last, 1)] {
                    let at = placed(add(p(v), scale(into(corner, w)?, width)));
                    let id = self.fresh_id();
                    self.vertices.push(CageVertex { id, at });
                    caps.insert(corner, id);
                    cap[slot] = id;
                }
                cap_faces.push(cap);
                continue;
            }
            let mut ids = Vec::with_capacity(runs.len());
            for run in &runs {
                let first = fan[run[0]];
                let last = fan[run[run.len() - 1]];
                // The bevelled edge before the run — the one to the first
                // corner's next vertex — and after it.
                let before = next(faces, first);
                let after = prev(faces, last);
                let before_bevelled = bevelled.contains(&undirected(v, before));
                let after_bevelled = bevelled.contains(&undirected(v, after));
                let offset = match (before_bevelled, after_bevelled) {
                    (true, true) => {
                        let (d, e) = (into(first, before)?, into(last, after)?);
                        let along = 1.0 + dot(d, e);
                        if along <= 0.0 {
                            return Err(GeopError::new(format!(
                                "bevel: the bevelled edges fold back at vertex {}",
                                vertex_key(id_of(v))
                            )));
                        }
                        scale(add(d, e), width / along)
                    }
                    (true, false) => scale(into(first, before)?, width),
                    (false, true) => scale(into(last, after)?, width),
                    (false, false) => unreachable!("a sector is bounded by a bevelled edge"),
                };
                let at = placed(add(p(v), offset));
                let id = self.fresh_id();
                self.vertices.push(CageVertex { id, at });
                for &i in run {
                    sector_of.insert(fan[i], id);
                }
                ids.push(id);
            }
            sectors.insert(v, (ids, *closed));
        }
        // The faces shrink to their sector vertices, and take the cap
        // vertices in beside the vertices they cap: after the corner in the
        // face running along the bevelled edge, before it in the other.
        for (f, face) in self.faces.iter_mut().enumerate() {
            let mut vertices = Vec::with_capacity(face.vertices.len() + 2);
            for (k, &v) in face.vertices.iter().enumerate() {
                match (sector_of.get(&(f, k)), caps.get(&(f, k))) {
                    (Some(&id), _) => vertices.push(id),
                    (None, Some(&cap)) => {
                        let n = face.vertices.len();
                        let along = bevelled.contains(&undirected(
                            index_of[&v],
                            index_of[&face.vertices[(k + 1) % n]],
                        ));
                        if along {
                            vertices.extend([v, cap]);
                        } else {
                            vertices.extend([cap, v]);
                        }
                    }
                    (None, None) => vertices.push(v),
                }
            }
            face.vertices = vertices;
        }
        // A strip per bevelled edge, a face per vertex of three sectors or
        // more, and the caps.
        let new_corner = |f: usize, k: usize| -> u32 {
            (sector_of.get(&(f, k)).or_else(|| caps.get(&(f, k))))
                .copied()
                .unwrap_or_else(|| id_of(faces[f][k]))
        };
        let corner = |x: usize, y: usize| -> u32 {
            let &(f, k) = &topology.directed[&(x, y)];
            new_corner(f, k)
        };
        let corner_after = |x: usize, y: usize| -> u32 {
            // The corner of `y` in the face running `x -> y`.
            let &(f, k) = &topology.directed[&(x, y)];
            new_corner(f, (k + 1) % faces[f].len())
        };
        for &[a, b] in &bevelled {
            let strip = deduplicated(&[
                corner_after(a, b),
                corner(a, b),
                corner_after(b, a),
                corner(b, a),
            ]);
            if strip.len() >= 3 {
                let id = self.fresh_id();
                self.faces.push(CageFace {
                    id,
                    vertices: strip,
                });
            }
        }
        for (ids, closed) in sectors.values() {
            if *closed && ids.len() >= 3 {
                let id = self.fresh_id();
                self.faces.push(CageFace {
                    id,
                    vertices: ids.clone(),
                });
            }
        }
        for cap in cap_faces {
            let id = self.fresh_id();
            self.faces.push(CageFace {
                id,
                vertices: cap.to_vec(),
            });
        }
        // A crease of an edge not bevelled follows its corners; one of a
        // bevelled edge is gone with it.
        let creases = std::mem::take(&mut self.creases);
        for c in creases {
            let [a, b] = c.edge;
            let (i, j) = (index_of[&a], index_of[&b]);
            if bevelled.contains(&undirected(i, j)) {
                continue;
            }
            let &(f, k) = topology
                .directed
                .get(&(i, j))
                .or_else(|| topology.directed.get(&(j, i)))
                .expect("a crease is on an edge");
            let n = faces[f].len();
            let x = sector_of
                .get(&(f, k))
                .copied()
                .unwrap_or(id_of(faces[f][k]));
            let y = sector_of
                .get(&(f, (k + 1) % n))
                .copied()
                .unwrap_or(id_of(faces[f][(k + 1) % n]));
            self.set_sharpness(&[[x, y]], c.sharpness);
        }
        self.tidy();
        Ok(())
    }

    /// Merges the vertices `vertices` into one at their centre — on the
    /// mirror plane if any of them was — keeping the lowest id. A face left
    /// with fewer than three corners goes; one that would run through the
    /// merged vertex twice is refused by name.
    pub fn merge_vertices(&mut self, vertices: &BTreeSet<u32>, mirror: Mirror) -> GeopResult<()> {
        if vertices.len() < 2 {
            return Err(GeopError::new("merge: select at least two vertices"));
        }
        let kept = *vertices.first().expect("two at least");
        let mut centre = self.centre(vertices).ok_or_else(|| {
            GeopError::new(format!(
                "merge: the cage has no vertex {}",
                vertex_key(
                    *vertices
                        .iter()
                        .find(|&&v| self.vertex(v).is_err())
                        .expect("one is missing")
                )
            ))
        })?;
        if let Some(axis) = mirror.axis()
            && vertices
                .iter()
                .any(|&v| self.vertex(v).is_ok_and(|x| x.at[axis] == 0.0))
        {
            centre[axis] = 0.0;
        }
        self.vertex_mut(kept)?.at = centre;
        let to = |v: u32| if vertices.contains(&v) { kept } else { v };
        let mut faces = Vec::with_capacity(self.faces.len());
        for face in &self.faces {
            let cycle: Vec<u32> = face.vertices.iter().map(|&v| to(v)).collect();
            let cycle = deduplicated(&cycle);
            if cycle.len() < 3 {
                continue;
            }
            if let Some(v) = repeated(&cycle) {
                return Err(GeopError::new(format!(
                    "merge: face {} would run through vertex {} twice",
                    face_key(face.id),
                    vertex_key(v)
                )));
            }
            faces.push(CageFace {
                id: face.id,
                vertices: cycle,
            });
        }
        self.faces = faces;
        self.rename_creases(|[a, b]| [to(a), to(b)]);
        self.tidy();
        Ok(())
    }

    /// Takes the vertices among `candidates` that are left between two
    /// faces only, with two edges, out of those faces: no corner any more,
    /// such a vertex lies along an edge, where the limit surface has no
    /// use for it.
    fn drop_two_edged(&mut self, candidates: &[u32]) {
        let directed = self.directed_edges();
        let mut drop: BTreeSet<u32> = BTreeSet::new();
        for &v in candidates {
            let out: Vec<u32> = directed
                .keys()
                .filter(|&&(x, _)| x == v)
                .map(|&(_, y)| y)
                .collect();
            let inner = out.len() == 2 && out.iter().all(|&y| directed.contains_key(&(y, v)));
            if inner {
                drop.insert(v);
            }
        }
        if drop.is_empty() {
            return;
        }
        for face in &mut self.faces {
            face.vertices.retain(|v| !drop.contains(v));
        }
        self.tidy();
    }

    /// Dissolves the edges `edges`: the two faces along each become one,
    /// keeping the lower id, and an end left between those two faces alone
    /// goes with it. An edge with one face, or two faces that meet along
    /// more than that edge, is refused by name.
    pub fn dissolve_edges(&mut self, edges: &[[u32; 2]]) -> GeopResult<()> {
        if edges.is_empty() {
            return Err(GeopError::new("dissolve: no edge selected"));
        }
        let mut edges: Vec<[u32; 2]> = edges.iter().map(|&[a, b]| undirected(a, b)).collect();
        edges.sort();
        edges.dedup();
        for [a, b] in edges {
            let directed = self.directed_edges();
            let (Some(&(f, _)), Some(&(g, _))) = (directed.get(&(a, b)), directed.get(&(b, a)))
            else {
                return Err(GeopError::new(format!(
                    "dissolve: edge {} has one face only: delete the face instead",
                    edge_key(a, b)
                )));
            };
            let first = rotated(&self.faces[f].vertices, b);
            let second = rotated(&self.faces[g].vertices, a);
            let mut merged = first;
            merged.extend(&second[1..second.len() - 1]);
            if let Some(v) = repeated(&merged) {
                return Err(GeopError::new(format!(
                    "dissolve: faces {} and {} meet at vertex {} as well as along edge {}",
                    face_key(self.faces[f].id),
                    face_key(self.faces[g].id),
                    vertex_key(v),
                    edge_key(a, b)
                )));
            }
            let (keep, drop) = if self.faces[f].id < self.faces[g].id {
                (f, g)
            } else {
                (g, f)
            };
            self.faces[keep].vertices = merged;
            self.faces.remove(drop);
            self.drop_two_edged(&[a, b]);
        }
        self.tidy();
        Ok(())
    }

    /// Dissolves the vertices `vertices`: the faces around each become one
    /// face, keeping the lowest id, and the vertex goes — as does any
    /// neighbour of it left between two faces alone. Faces around it that
    /// meet elsewhere than at it are refused by name.
    pub fn dissolve_vertices(&mut self, vertices: &BTreeSet<u32>) -> GeopResult<()> {
        if vertices.is_empty() {
            return Err(GeopError::new("dissolve: no vertex selected"));
        }
        for &v in vertices {
            let (mesh, topology) = self.indexed()?;
            let Some(i) = self.vertices.iter().position(|x| x.id == v) else {
                // Dissolving a neighbour took it with it.
                continue;
            };
            let (fan, _) = &topology.fans[i];
            let mut merged: Vec<u32> = Vec::new();
            for &(f, k) in fan {
                let face = &mesh.faces[f];
                let n = face.len();
                // From the corner after `v` up to the one before it, that
                // one left to the next face of the fan.
                for j in 1..n - 1 {
                    merged.push(self.vertices[face[(k + j) % n]].id);
                }
            }
            // An open fan ends at the boundary: its last face's last
            // corner closes the run.
            if !topology.fans[i].1 {
                let (f, k) = fan[fan.len() - 1];
                let n = mesh.faces[f].len();
                merged.push(self.vertices[mesh.faces[f][(k + n - 1) % n]].id);
            }
            let merged = deduplicated(&merged);
            if merged.len() < 3 {
                return Err(GeopError::new(format!(
                    "dissolve: the faces around vertex {} leave no face",
                    vertex_key(v)
                )));
            }
            if let Some(w) = repeated(&merged) {
                return Err(GeopError::new(format!(
                    "dissolve: the faces around vertex {} meet at vertex {} too",
                    vertex_key(v),
                    vertex_key(w)
                )));
            }
            let around: BTreeSet<u32> = fan.iter().map(|&(f, _)| self.faces[f].id).collect();
            let keep = *around.first().expect("a vertex has a face");
            self.faces
                .retain(|face| face.id == keep || !around.contains(&face.id));
            self.face_mut(keep)?.vertices = merged.clone();
            self.drop_two_edged(&merged);
        }
        self.tidy();
        Ok(())
    }

    fn face_mut(&mut self, id: u32) -> GeopResult<&mut CageFace> {
        self.faces
            .iter_mut()
            .find(|f| f.id == id)
            .ok_or_else(|| GeopError::new(format!("the cage has no face {}", face_key(id))))
    }

    /// Bridges the faces `a` and `b`: both go, and a ring of quads joins
    /// their rims, each corner of `a` to the nearest corner of `b` going
    /// the other way round — a tunnel through the body, or a handle between
    /// two of its faces. The faces need the same number of corners.
    pub fn bridge(&mut self, a: u32, b: u32) -> GeopResult<()> {
        let first = self.face(a)?.vertices.clone();
        let second = self.face(b)?.vertices.clone();
        let n = first.len();
        if n != second.len() {
            return Err(GeopError::new(format!(
                "bridge: face {} has {n} corners and face {} has {}: they need the same number",
                face_key(a),
                face_key(b),
                second.len()
            )));
        }
        if first.iter().any(|v| second.contains(v)) {
            return Err(GeopError::new(format!(
                "bridge: faces {} and {} share a vertex",
                face_key(a),
                face_key(b)
            )));
        }
        // `b` seen from `a`'s side runs the other way round; of its
        // rotations, the one whose corners lie nearest `a`'s.
        let reversed: Vec<u32> = second.iter().rev().copied().collect();
        let at = |v: u32| self.vertex(v).map(|x| x.at);
        let mut best: Option<(f64, usize)> = None;
        for r in 0..n {
            let mut total = 0.0;
            for i in 0..n {
                total += norm(sub(at(first[i])?, at(reversed[(i + r) % n])?));
            }
            if best.is_none_or(|(d, _)| total < d) {
                best = Some((total, r));
            }
        }
        let (_, r) = best.expect("a face has corners");
        self.faces.retain(|f| f.id != a && f.id != b);
        for i in 0..n {
            let id = self.fresh_id();
            self.faces.push(CageFace {
                id,
                vertices: vec![
                    first[i],
                    first[(i + 1) % n],
                    reversed[(i + 1 + r) % n],
                    reversed[(i + r) % n],
                ],
            });
        }
        self.tidy();
        Ok(())
    }

    /// Fills the hole the open edge between `a` and `b` is on with one face
    /// through all the vertices around it (see [`Cage::boundary_run`]).
    /// With a `mirror`, a hole whose rim lies in the plane is the seam to
    /// the mirror image, and is refused.
    pub fn fill_hole(&mut self, a: u32, b: u32, mirror: Mirror) -> GeopResult<()> {
        let run = self.boundary_run(a, b)?;
        if mirror.axis().is_some()
            && run
                .iter()
                .all(|&v| self.on_plane(v, mirror).unwrap_or(false))
        {
            return Err(GeopError::new(format!(
                "fill: the hole through edge {} lies in the mirror plane, where the mirror image closes it",
                edge_key(a, b)
            )));
        }
        let id = self.fresh_id();
        self.faces.push(CageFace {
            id,
            vertices: run.into_iter().rev().collect(),
        });
        Ok(())
    }

    /// Cuts the cage at the plane where coordinate `axis` is zero and keeps
    /// the half where it is positive, as a mirrored cage holds it. Every
    /// edge reaching across the plane is split where it crosses, the new
    /// vertex exactly on the plane. A face crossing the plane more than
    /// twice is refused, by name.
    pub fn halve(&mut self, axis: usize) -> GeopResult<()> {
        let side = |at: [f64; 3]| at[axis];
        let mut crossing: HashMap<[u32; 2], u32> = HashMap::new();
        for [a, b] in self.edges() {
            let (p, q) = (self.vertex(a)?.at, self.vertex(b)?.at);
            if side(p) * side(q) < 0.0 {
                let t = side(p) / (side(p) - side(q));
                let mut at = [0, 1, 2].map(|c| p[c] + t * (q[c] - p[c]));
                at[axis] = 0.0;
                let id = self.fresh_id();
                self.vertices.push(CageVertex { id, at });
                crossing.insert([a, b], id);
                let sharpness = self.sharpness(a, b);
                if sharpness > 0.0 {
                    self.set_sharpness(&[[a, id], [id, b]], sharpness);
                }
            }
        }
        let position: HashMap<u32, f64> =
            self.vertices.iter().map(|v| (v.id, side(v.at))).collect();
        let mut kept = Vec::new();
        for face in &self.faces {
            let n = face.vertices.len();
            let mut ring = Vec::with_capacity(n + 2);
            for k in 0..n {
                let (x, y) = (face.vertices[k], face.vertices[(k + 1) % n]);
                ring.push(x);
                if let Some(&m) = crossing.get(&undirected(x, y)) {
                    ring.push(m);
                }
            }
            let at = |v: u32| position[&v];
            if ring.iter().all(|&v| at(v) <= 0.0) {
                continue;
            }
            if ring.iter().all(|&v| at(v) >= 0.0) {
                kept.push(CageFace {
                    id: face.id,
                    vertices: ring,
                });
                continue;
            }
            // Mixed: keep the one run of vertices on the positive side,
            // from one vertex on the plane to the next.
            let m = ring.len();
            let first = (0..m)
                .find(|&i| at(ring[i]) < 0.0 && at(ring[(i + 1) % m]) >= 0.0)
                .expect("mixed");
            // `ring[first]` is on the negative side: the run ends by it.
            let run: Vec<u32> = (1..=m)
                .map(|j| ring[(first + j) % m])
                .take_while(|&v| at(v) >= 0.0)
                .collect();
            let positive = ring.iter().filter(|&&v| at(v) > 0.0).count();
            if run.iter().filter(|&&v| at(v) > 0.0).count() != positive || run.len() < 3 {
                return Err(GeopError::new(format!(
                    "the mirror plane crosses face {} more than twice",
                    face_key(face.id)
                )));
            }
            kept.push(CageFace {
                id: face.id,
                vertices: run,
            });
        }
        self.faces = kept;
        self.tidy();
        Ok(())
    }

    /// The whole cage the mirrored half stands for, as a cage of its own:
    /// the half keeps its ids, the mirror image gets new ones.
    pub fn unmirrored(&self, mirror: Mirror) -> GeopResult<Cage> {
        if mirror == Mirror::None {
            return Ok(self.clone());
        }
        let mesh = Mesh::new(self, mirror)?;
        let mut cage = self.clone();
        let mut ids: Vec<u32> = Vec::with_capacity(mesh.positions.len());
        for (i, name) in mesh.vertex_names.iter().enumerate() {
            match name.strip_suffix('m') {
                Some(_) => {
                    let id = cage.fresh_id();
                    cage.vertices.push(CageVertex {
                        id,
                        at: mesh.positions[i],
                    });
                    ids.push(id);
                }
                None => ids.push(name[1..].parse().expect("a vertex key")),
            }
        }
        for (f, name) in mesh.face_names.iter().enumerate() {
            if name.ends_with('m') {
                let id = cage.fresh_id();
                cage.faces.push(CageFace {
                    id,
                    vertices: mesh.faces[f].iter().map(|&v| ids[v]).collect(),
                });
            }
        }
        for &[a, b] in &mesh.sharp {
            cage.set_sharpness(&[[ids[a], ids[b]]], 1.0);
        }
        for (&[a, b], &s) in &mesh.soft {
            cage.set_sharpness(&[[ids[a], ids[b]]], s);
        }
        Ok(cage)
    }
}
