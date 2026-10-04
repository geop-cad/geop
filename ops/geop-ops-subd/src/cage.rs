//! [`Cage`]: the control mesh a subdivision surface is the limit of, as a
//! step's arguments hold it — vertices, faces through them, and the edges
//! creased sharp — and [`Mesh`], the same mesh indexed, mirrored if asked
//! to, and checked to be one a limit surface can be built from.
//!
//! Every vertex and face has an id of its own, kept through every edit, so
//! that what the limit surface is made of can be named after the cage
//! element it comes from (see [`crate::limit`]): `v3` and `f7`, and, on the
//! mirrored half, `v3m` and `f7m`.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

/// A vertex of the cage: where it is.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CageVertex {
    pub id: u32,
    pub at: [f64; 3],
}

/// A face of the cage: its vertices, counter-clockwise seen from outside.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct CageFace {
    pub id: u32,
    pub vertices: Vec<u32>,
}

/// A control cage: a mesh of faces of any number of sides — quads mostly,
/// triangles and n-gons allowed — with the edges named in `creases` kept
/// sharp. Ids are unique across vertices and faces, and `next_id` is the
/// next one no element has had.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Cage {
    pub vertices: Vec<CageVertex>,
    pub faces: Vec<CageFace>,
    /// Edges kept sharp, each by its two vertices, the smaller id first.
    #[serde(default)]
    pub creases: Vec<[u32; 2]>,
    pub next_id: u32,
}

/// The plane the cage is mirrored in, if any: `X` is the plane `x = 0` of
/// the world. The cage holds one half; the other is its mirror image,
/// joined to it along the vertices that lie on the plane.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mirror {
    #[default]
    None,
    X,
    Y,
    Z,
}

impl Mirror {
    /// The coordinate the plane zeroes.
    pub fn axis(self) -> Option<usize> {
        match self {
            Mirror::None => None,
            Mirror::X => Some(0),
            Mirror::Y => Some(1),
            Mirror::Z => Some(2),
        }
    }
}

/// The name of the vertex `id`: `v3`.
pub fn vertex_key(id: u32) -> String {
    format!("v{id}")
}

/// The name of the face `id`: `f7`.
pub fn face_key(id: u32) -> String {
    format!("f{id}")
}

/// The name of the edge between vertices `a` and `b`: `e3-5`, the smaller
/// id first.
pub fn edge_key(a: u32, b: u32) -> String {
    let (a, b) = (a.min(b), a.max(b));
    format!("e{a}-{b}")
}

impl Cage {
    /// The vertex `id`.
    pub fn vertex(&self, id: u32) -> GeopResult<&CageVertex> {
        self.vertices
            .iter()
            .find(|v| v.id == id)
            .ok_or_else(|| GeopError::new(format!("the cage has no vertex {}", vertex_key(id))))
    }

    pub fn vertex_mut(&mut self, id: u32) -> GeopResult<&mut CageVertex> {
        self.vertices
            .iter_mut()
            .find(|v| v.id == id)
            .ok_or_else(|| GeopError::new(format!("the cage has no vertex {}", vertex_key(id))))
    }

    /// The face `id`.
    pub fn face(&self, id: u32) -> GeopResult<&CageFace> {
        self.faces
            .iter()
            .find(|f| f.id == id)
            .ok_or_else(|| GeopError::new(format!("the cage has no face {}", face_key(id))))
    }

    /// An id no element has had yet.
    pub fn fresh_id(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Every edge, each by its two vertices, the smaller id first, in order.
    pub fn edges(&self) -> BTreeSet<[u32; 2]> {
        self.faces
            .iter()
            .flat_map(|f| {
                let n = f.vertices.len();
                (0..n).map(move |k| {
                    let (a, b) = (f.vertices[k], f.vertices[(k + 1) % n]);
                    [a.min(b), a.max(b)]
                })
            })
            .collect()
    }

    pub fn is_crease(&self, a: u32, b: u32) -> bool {
        self.creases.contains(&[a.min(b), a.max(b)])
    }
}

/// The cage as the limit surface is built from it: indexed, the mirror
/// image added, every element named.
#[derive(Clone, Debug)]
pub struct Mesh {
    /// Per vertex: its name (`v3`, `v3m`) and where it is.
    pub vertex_names: Vec<String>,
    pub positions: Vec<[f64; 3]>,
    /// Per face: its name (`f7`, `f7m`) and its vertices, counter-clockwise
    /// seen from outside.
    pub face_names: Vec<String>,
    pub faces: Vec<Vec<usize>>,
    /// The edges kept sharp, the smaller index first.
    pub sharp: BTreeSet<(usize, usize)>,
}

impl Mesh {
    /// `cage`, and, if `mirror` names a plane, its mirror image in it,
    /// joined to it at the vertices on the plane. Fails, naming the
    /// elements, for a cage no limit surface can be built from (see
    /// [`Topology::new`]), and for a mirrored one with a face lying in the
    /// plane, or reaching across it.
    pub fn new(cage: &Cage, mirror: Mirror) -> GeopResult<Self> {
        let mut index: HashMap<u32, usize> = HashMap::new();
        let mut mesh = Mesh {
            vertex_names: Vec::new(),
            positions: Vec::new(),
            face_names: Vec::new(),
            faces: Vec::new(),
            sharp: BTreeSet::new(),
        };
        for v in &cage.vertices {
            if !v.at.iter().all(|c| c.is_finite()) {
                return Err(GeopError::new(format!(
                    "vertex {} is not at a finite point: {:?}",
                    vertex_key(v.id),
                    v.at
                )));
            }
            if index.insert(v.id, mesh.positions.len()).is_some() {
                return Err(GeopError::new(format!(
                    "the cage has two vertices {}",
                    vertex_key(v.id)
                )));
            }
            mesh.vertex_names.push(vertex_key(v.id));
            mesh.positions.push(v.at);
        }
        let mut face_ids = BTreeSet::new();
        for f in &cage.faces {
            if !face_ids.insert(f.id) {
                return Err(GeopError::new(format!(
                    "the cage has two faces {}",
                    face_key(f.id)
                )));
            }
            let vertices = f
                .vertices
                .iter()
                .map(|id| {
                    index.get(id).copied().ok_or_else(|| {
                        GeopError::new(format!(
                            "face {} runs through vertex {}, which the cage does not have",
                            face_key(f.id),
                            vertex_key(*id)
                        ))
                    })
                })
                .collect::<GeopResult<Vec<_>>>()?;
            mesh.face_names.push(face_key(f.id));
            mesh.faces.push(vertices);
        }
        for &[a, b] in &cage.creases {
            match (index.get(&a), index.get(&b)) {
                (Some(&i), Some(&j)) => {
                    mesh.sharp.insert((i.min(j), i.max(j)));
                }
                _ => {
                    return Err(GeopError::new(format!(
                        "crease {} runs between vertices the cage does not have",
                        edge_key(a, b)
                    )));
                }
            }
        }
        if let Some(axis) = mirror.axis() {
            mesh.add_mirror_image(axis)?;
        }
        Ok(mesh)
    }

    /// Adds the mirror image in the plane where coordinate `axis` is zero,
    /// sharing the vertices exactly on it.
    fn add_mirror_image(&mut self, axis: usize) -> GeopResult<()> {
        let plane = ["x", "y", "z"][axis];
        let n = self.positions.len();
        let mut image = vec![0; n];
        for i in 0..n {
            let mut at = self.positions[i];
            if at[axis] == 0.0 {
                image[i] = i;
                continue;
            }
            if at[axis] < 0.0 {
                return Err(GeopError::new(format!(
                    "vertex {} is on the far side of the mirror plane {plane} = 0: the cage holds the half where {plane} >= 0",
                    self.vertex_names[i]
                )));
            }
            at[axis] = -at[axis];
            image[i] = self.positions.len();
            self.vertex_names.push(format!("{}m", self.vertex_names[i]));
            self.positions.push(at);
        }
        let faces = self.faces.len();
        for f in 0..faces {
            if self.faces[f].iter().all(|&v| image[v] == v) {
                return Err(GeopError::new(format!(
                    "face {} lies in the mirror plane {plane} = 0",
                    self.face_names[f]
                )));
            }
            let mirrored: Vec<usize> = self.faces[f].iter().rev().map(|&v| image[v]).collect();
            self.face_names.push(format!("{}m", self.face_names[f]));
            self.faces.push(mirrored);
        }
        let sharp: Vec<(usize, usize)> = self.sharp.iter().copied().collect();
        for (a, b) in sharp {
            let (i, j) = (image[a], image[b]);
            self.sharp.insert((i.min(j), i.max(j)));
        }
        Ok(())
    }

    /// The name of the edge between vertices `a` and `b`, as a cage
    /// element: `e3-5`, or, on the mirrored half, `e3m-5m`.
    pub fn edge_name(&self, a: usize, b: usize) -> String {
        let (a, b) = (a.min(b), a.max(b));
        format!(
            "e{}-{}",
            &self.vertex_names[a][1..],
            &self.vertex_names[b][1..]
        )
    }
}

/// How the faces of a mesh fit together: which face runs along each
/// directed edge, and the faces around each vertex in order.
#[derive(Clone, Debug)]
pub struct Topology {
    /// `(a, b)`: the face that runs from `a` to `b`, and the corner of it
    /// at `a`.
    pub directed: HashMap<(usize, usize), (usize, usize)>,
    /// Per vertex: the faces around it, each with its corner there, in
    /// order — each next one across the edge to the previous one's
    /// preceding corner — and whether they close up all round, rather than
    /// meeting the mesh's boundary on both ends.
    pub fans: Vec<(Vec<(usize, usize)>, bool)>,
}

impl Topology {
    /// The topology of `faces` over `vertex_count` vertices. Fails, naming
    /// the elements with `vertex_names` and `face_names`, for a mesh no
    /// limit surface can be built from: a face of fewer than three
    /// vertices, or through one twice; faces turned against each other
    /// (an edge run the same way twice), or more than two meeting at an
    /// edge; faces around a vertex that do not make one fan; a vertex of no
    /// face; an inner vertex of fewer than three faces.
    pub fn new(
        vertex_count: usize,
        faces: &[Vec<usize>],
        vertex_names: &[String],
        face_names: &[String],
    ) -> GeopResult<Self> {
        let mut directed = HashMap::new();
        let mut corners: Vec<Vec<(usize, usize)>> = vec![Vec::new(); vertex_count];
        for (f, face) in faces.iter().enumerate() {
            let n = face.len();
            if n < 3 {
                return Err(GeopError::new(format!(
                    "face {} has {n} vertices: a face needs at least three",
                    face_names[f]
                )));
            }
            for k in 0..n {
                let (a, b) = (face[k], face[(k + 1) % n]);
                if face[..k].contains(&a) {
                    return Err(GeopError::new(format!(
                        "face {} runs through vertex {} twice",
                        face_names[f], vertex_names[a]
                    )));
                }
                if let Some((g, _)) = directed.insert((a, b), (f, k)) {
                    return Err(GeopError::new(format!(
                        "faces {} and {} both run from {} to {}: they are turned against each other, or more than two faces meet at that edge",
                        face_names[g], face_names[f], vertex_names[a], vertex_names[b]
                    )));
                }
                corners[a].push((f, k));
            }
        }
        let mut fans = Vec::with_capacity(vertex_count);
        for (v, around) in corners.iter().enumerate() {
            if around.is_empty() {
                return Err(GeopError::new(format!(
                    "vertex {} belongs to no face",
                    vertex_names[v]
                )));
            }
            let next = |&(f, k): &(usize, usize)| faces[f][(k + 1) % faces[f].len()];
            let prev = |&(f, k): &(usize, usize)| {
                let n = faces[f].len();
                faces[f][(k + n - 1) % n]
            };
            // A fan that meets the boundary starts at the face with no
            // neighbour across its edge to the next corner.
            let start = around
                .iter()
                .find(|c| !directed.contains_key(&(next(c), v)))
                .copied();
            let closed = start.is_none();
            let mut fan = vec![start.unwrap_or(around[0])];
            loop {
                let last = fan[fan.len() - 1];
                match directed.get(&(v, prev(&last))) {
                    Some(&c) if c == fan[0] => break,
                    Some(&c) if fan.len() < around.len() => fan.push(c),
                    _ => break,
                }
            }
            if fan.len() != around.len() {
                return Err(GeopError::new(format!(
                    "the faces around vertex {} do not make one fan: the surface pinches there",
                    vertex_names[v]
                )));
            }
            if closed && fan.len() < 3 {
                return Err(GeopError::new(format!(
                    "inner vertex {} belongs to {} faces: it needs at least three",
                    vertex_names[v],
                    fan.len()
                )));
            }
            fans.push((fan, closed));
        }
        Ok(Self { directed, fans })
    }

    /// Whether only one face runs along the edge between `a` and `b`.
    pub fn is_boundary(&self, a: usize, b: usize) -> bool {
        self.directed.contains_key(&(a, b)) != self.directed.contains_key(&(b, a))
    }

    /// Every edge, the smaller index first, with its faces.
    pub fn edges(&self) -> BTreeMap<(usize, usize), Vec<usize>> {
        let mut edges: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (&(a, b), &(f, _)) in &self.directed {
            edges.entry((a.min(b), a.max(b))).or_default().push(f);
        }
        for faces in edges.values_mut() {
            faces.sort();
        }
        edges
    }
}

impl Mesh {
    /// The mesh's topology, checked (see [`Topology::new`]), and checked to
    /// be in one piece.
    pub fn topology(&self) -> GeopResult<Topology> {
        if self.faces.is_empty() {
            return Err(GeopError::new("the cage has no faces"));
        }
        let topology = Topology::new(
            self.positions.len(),
            &self.faces,
            &self.vertex_names,
            &self.face_names,
        )?;
        let mut seen = vec![false; self.positions.len()];
        let mut stack = vec![0];
        seen[0] = true;
        while let Some(v) = stack.pop() {
            for &(f, _) in &topology.fans[v].0 {
                for &w in &self.faces[f] {
                    if !std::mem::replace(&mut seen[w], true) {
                        stack.push(w);
                    }
                }
            }
        }
        if let Some(apart) = seen.iter().position(|s| !s) {
            return Err(GeopError::new(format!(
                "the cage falls apart: vertex {} is not connected to vertex {}",
                self.vertex_names[apart], self.vertex_names[0]
            )));
        }
        for &(a, b) in &self.sharp {
            if !topology.directed.contains_key(&(a, b)) && !topology.directed.contains_key(&(b, a))
            {
                return Err(GeopError::new(format!(
                    "crease {} is no edge of the cage",
                    self.edge_name(a, b)
                )));
            }
        }
        Ok(topology)
    }
}
