//! What a cage starts as — a box, a cylinder, a sphere, a plane — and the
//! edits that shape it: moving, turning and scaling its elements,
//! extruding faces, inserting edge loops, creasing edges, deleting faces,
//! and cutting it in half at a mirror plane or making the mirrored half its
//! own.
//!
//! An edit keeps the id of every element it does not take away, and gives
//! what it adds ids no element has had, so that names built from them stay
//! put (see [`crate::cage`]).

use std::collections::{BTreeMap, BTreeSet, HashMap};

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::cage::{Cage, CageFace, CageVertex, Mirror, Mesh, edge_key, face_key, vertex_key};

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

    pub fn key(self) -> String {
        match self {
            Element::Vertex(v) => vertex_key(v),
            Element::Edge(a, b) => edge_key(a, b),
            Element::Face(f) => face_key(f),
        }
    }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
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
                        let norm = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt();
                        points.push(p.map(|x| radius * x / norm));
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
        if vertices.is_empty() {
            return None;
        }
        let mut sum = [0.0; 3];
        for &v in vertices {
            let at = self.vertex(v).ok()?.at;
            for c in 0..3 {
                sum[c] += at[c];
            }
        }
        Some(sum.map(|s| s / vertices.len() as f64))
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

    /// Creases the edges `edges`, or smooths them again.
    pub fn set_crease(&mut self, edges: &[[u32; 2]], sharp: bool) {
        for &[a, b] in edges {
            let edge = [a.min(b), a.max(b)];
            self.creases.retain(|&c| c != edge);
            if sharp {
                self.creases.push(edge);
            }
        }
        self.creases.sort();
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
        self.creases.retain(|c| edges.contains(c));
    }

    /// Deletes the faces `faces`, and every vertex and crease that leaves
    /// without a face.
    pub fn delete_faces(&mut self, faces: &BTreeSet<u32>) {
        self.faces.retain(|f| !faces.contains(&f.id));
        self.tidy();
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

    /// Extrudes the faces `faces` `distance` along their normals: they move
    /// out, keeping their ids, and a new quad joins each edge on the
    /// region's rim to its moved copy. A vertex inside the region just
    /// moves; one on its rim is left behind for the faces around, and a
    /// new one moves. With a `mirror`, what is on its plane stays on it,
    /// and an open edge along the plane gets no quad, which would lie in
    /// it.
    pub fn extrude(&mut self, faces: &BTreeSet<u32>, distance: f64, mirror: Mirror) -> GeopResult<()> {
        if faces.is_empty() {
            return Err(GeopError::new("extrude: no face selected"));
        }
        let region: Vec<CageFace> = faces
            .iter()
            .map(|&f| self.face(f).cloned())
            .collect::<GeopResult<_>>()?;
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
        let rim: Vec<(u32, u32)> = directed
            .iter()
            .copied()
            .filter(|&(a, b)| !directed.contains(&(b, a)))
            .collect();
        let mut normal: BTreeMap<u32, [f64; 3]> = BTreeMap::new();
        for face in &region {
            let n = self.normal(face)?;
            for &v in &face.vertices {
                let sum = normal.entry(v).or_insert([0.0; 3]);
                for i in 0..3 {
                    sum[i] += n[i];
                }
            }
        }
        let on_plane = |cage: &Cage, v: u32| -> GeopResult<bool> {
            let at = cage.vertex(v)?.at;
            Ok(mirror.axis().is_some_and(|a| at[a] == 0.0))
        };
        let mut offset: BTreeMap<u32, [f64; 3]> = BTreeMap::new();
        for (&v, n) in &normal {
            let mut n = *n;
            if on_plane(self, v)? {
                n[mirror.axis().expect("on a plane")] = 0.0;
            }
            let length = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if length == 0.0 || !length.is_finite() {
                return Err(GeopError::new(format!(
                    "extrude: the faces around vertex {} face no one way",
                    vertex_key(v)
                )));
            }
            offset.insert(v, n.map(|c| distance * c / length));
        }
        let rim_vertices: BTreeSet<u32> = rim.iter().flat_map(|&(a, b)| [a, b]).collect();
        let mut moved: BTreeMap<u32, u32> = BTreeMap::new();
        for (&v, d) in &offset {
            let at = self.vertex(v)?.at;
            let to = [at[0] + d[0], at[1] + d[1], at[2] + d[2]];
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
            if open && on_plane(self, a)? && on_plane(self, b)? {
                continue;
            }
            let id = self.fresh_id();
            self.faces.push(CageFace {
                id,
                vertices: vec![a, b, moved[&b], moved[&a]],
            });
        }
        // A crease whose edge only the region had — inside it, or open
        // along the mirror plane — moves out with it; one on its rim stays
        // where the region was.
        let edges = self.edges();
        let to = |v: u32| moved.get(&v).copied().unwrap_or(v);
        for crease in &mut self.creases {
            if !edges.contains(crease) {
                let (a, b) = (to(crease[0]), to(crease[1]));
                *crease = [a.min(b), a.max(b)];
            }
        }
        self.tidy();
        Ok(())
    }

    /// Inserts an edge loop across the edge between `a` and `b`: through
    /// the middle of every edge of its ring — the edges reached by
    /// crossing quads from one side to the opposite one, until the ring
    /// closes, or meets the cage's boundary or a face that is not a quad —
    /// splitting each of those quads in two.
    pub fn insert_loop(&mut self, a: u32, b: u32) -> GeopResult<()> {
        let mut owner: HashMap<(u32, u32), usize> = HashMap::new();
        for (i, face) in self.faces.iter().enumerate() {
            let n = face.vertices.len();
            for k in 0..n {
                owner.insert((face.vertices[k], face.vertices[(k + 1) % n]), i);
            }
        }
        if !owner.contains_key(&(a, b)) && !owner.contains_key(&(b, a)) {
            return Err(GeopError::new(format!(
                "insert loop: the cage has no edge {}",
                edge_key(a, b)
            )));
        }
        let undirected = |x: u32, y: u32| [x.min(y), x.max(y)];
        let mut ring: Vec<[u32; 2]> = vec![undirected(a, b)];
        // Per quad split: its index and the two ring edges it runs across.
        let mut splits: Vec<usize> = Vec::new();
        for start in [(a, b), (b, a)] {
            let mut edge = start;
            while let Some(&f) = owner.get(&edge) {
                let face = &self.faces[f].vertices;
                if face.len() != 4 || splits.contains(&f) {
                    break;
                }
                splits.push(f);
                let k = face.iter().position(|&v| v == edge.0).expect("on its face");
                let across = (face[(k + 2) % 4], face[(k + 3) % 4]);
                let key = undirected(across.0, across.1);
                if ring.contains(&key) {
                    break;
                }
                ring.push(key);
                edge = (across.1, across.0);
            }
        }
        let mut middle: HashMap<[u32; 2], u32> = HashMap::new();
        for &[x, y] in &ring {
            let (p, q) = (self.vertex(x)?.at, self.vertex(y)?.at);
            let id = self.fresh_id();
            self.vertices.push(CageVertex {
                id,
                at: [0, 1, 2].map(|c| (p[c] + q[c]) / 2.0),
            });
            middle.insert([x, y], id);
            if self.is_crease(x, y) {
                self.set_crease(&[[x, y]], false);
                self.set_crease(&[[x, id], [id, y]], true);
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
        for f in splits {
            let vertices = self.faces[f].vertices.clone();
            let cuts: Vec<usize> = (0..vertices.len())
                .filter(|&i| middle.values().any(|&m| m == vertices[i]))
                .collect();
            let [i, j] = cuts[..] else {
                return Err(GeopError::new(format!(
                    "insert loop: face {} is crossed by the loop {} times",
                    face_key(self.faces[f].id),
                    cuts.len()
                )));
            };
            let first: Vec<u32> = vertices[i..=j].to_vec();
            let second: Vec<u32> = vertices[j..].iter().chain(&vertices[..=i]).copied().collect();
            self.faces[f].vertices = first;
            let id = self.fresh_id();
            self.faces.push(CageFace {
                id,
                vertices: second,
            });
        }
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
                if self.is_crease(a, b) {
                    self.set_crease(&[[a, id], [id, b]], true);
                }
            }
        }
        let position: HashMap<u32, f64> = self.vertices.iter().map(|v| (v.id, side(v.at))).collect();
        let mut kept = Vec::new();
        for face in &self.faces {
            let n = face.vertices.len();
            let mut ring = Vec::with_capacity(n + 2);
            for k in 0..n {
                let (x, y) = (face.vertices[k], face.vertices[(k + 1) % n]);
                ring.push(x);
                if let Some(&m) = crossing.get(&[x.min(y), x.max(y)]) {
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
            let mut run = Vec::new();
            let mut i = (first + 1) % m;
            while at(ring[i]) >= 0.0 {
                run.push(ring[i]);
                i = (i + 1) % m;
            }
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
        let creases: Vec<[u32; 2]> = mesh.sharp.iter().map(|&(a, b)| [ids[a], ids[b]]).collect();
        cage.set_crease(&creases, true);
        Ok(cage)
    }
}
