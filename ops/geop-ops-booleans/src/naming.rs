//! How a boolean names what it creates.
//!
//! Everything is named after what it was made from, in terms of the names
//! the two operands had *before* the boolean — their *origins*. A piece of an
//! edge split in two has the edge's origin, and so does a piece of that
//! piece; likewise for faces. With `N` the boolean's [`Namer`]:
//!
//! | entity | name |
//! |---|---|
//! | the result solid | `N` |
//! | vertex where edges `E1 < E2` cross | `N(E1,E2,i,n)`: the `i`-th of their `n` crossings, counted along `E1` |
//! | vertex where edge `E` pierces face `F` | `N(E,F,i,n)`: the `i`-th of the `n` piercings, counted along `E` |
//! | piece of edge `E` starting at vertex `V` | `N(E,V)` — the piece at `E`'s own start keeps the name `E` |
//! | edge traced along faces `F1 < F2` from vertex `P` to `Q` (`P < Q`) | `N(F1,F2,P,Q)`, with a trailing `,k` only if several such edges join the same `P` and `Q` |
//! | piece of face `F` split off along edge `E` | `N(F,E)` — the other piece keeps the name `F` |
//!
//! `<` is the order of the names as strings, so each name is independent of
//! which operand came first and of the order the algorithm found things in.
//! The one exception is the trailing `,k` of a traced edge: two intersection
//! branches with the same ends on the same faces (an arc and its complement)
//! are told apart only by when they were traced.
//!
//! A crossing's `i` and `n` are only known once every crossing of those two
//! entities has been found, and every other name above builds on vertex
//! names. So [`BooleanNaming`] hands out a provisional name while the
//! boolean runs, remembers what each entity was made from, and
//! [`BooleanNaming::finish`] settles all names at once when it is done.

use std::collections::{BTreeMap, HashMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_topology::{Body, EdgeId, FaceId, VertexId};
use geop_ops::{Namer, Part, RefId};

/// What a created entity was made from — everything its final name derives
/// from (see the module docs).
enum Recipe<S: Scalar> {
    /// A vertex where `along` (an edge's origin) meets `other` (an edge's or
    /// a face's origin), at parameter `t` of `along`.
    Crossing {
        vertex: VertexId,
        along: String,
        other: String,
        t: S,
    },
    /// A piece of an edge with origin `origin`, starting at `start`.
    EdgePiece {
        edge: EdgeId,
        origin: String,
        start: VertexId,
    },
    /// An edge traced along faces with origins `faces`, between `ends`.
    Trace {
        edge: EdgeId,
        faces: [String; 2],
        ends: [VertexId; 2],
    },
    /// A piece of a face with origin `origin`, split off along `edge`.
    FacePiece {
        face: FaceId,
        origin: String,
        edge: EdgeId,
    },
}

/// The naming bookkeeping of one boolean, see the module docs.
pub struct BooleanNaming<S: Scalar> {
    namer: Namer,
    edge_origin: HashMap<EdgeId, String>,
    face_origin: HashMap<FaceId, String>,
    recipes: Vec<Recipe<S>>,
}

impl<S: Scalar> BooleanNaming<S> {
    /// Starts naming a boolean of `bodies`, remembering the names their
    /// edges and faces have now as those entities' origins.
    pub fn new(part: &Part<S>, namer: &Namer, bodies: &[Body]) -> GeopResult<Self> {
        let model = part.topology();
        let name = |id: RefId| {
            part.name_of(id)
                .map(str::to_string)
                .ok_or_else(|| GeopError::new(format!("boolean: operand entity {id} has no name")))
        };
        let mut edge_origin = HashMap::new();
        let mut face_origin = HashMap::new();
        for &body in bodies {
            for edge in model.iter_body_edges(body)? {
                edge_origin.insert(edge, name(edge.into())?);
            }
            for face in model.body_faces(body)? {
                face_origin.insert(face, name(face.into())?);
            }
        }
        Ok(Self {
            namer: namer.clone(),
            edge_origin,
            face_origin,
            recipes: Vec::new(),
        })
    }

    /// The name for the next entity, until [`BooleanNaming::finish`]
    /// replaces it. `~` never appears in a final name.
    pub fn provisional(&self) -> String {
        self.namer.name(&[&format!("~{}", self.recipes.len())])
    }

    pub fn edge_origin(&self, edge: EdgeId) -> GeopResult<&str> {
        self.edge_origin
            .get(&edge)
            .map(String::as_str)
            .ok_or_else(|| {
                GeopError::new(format!("boolean naming: edge {edge} has no known origin"))
            })
    }

    /// Every edge's origin, see [`BooleanNaming::edge_origin`].
    pub fn edge_origins(&self) -> &HashMap<EdgeId, String> {
        &self.edge_origin
    }

    /// Every face's origin, see [`BooleanNaming::face_origin`].
    pub fn face_origins(&self) -> &HashMap<FaceId, String> {
        &self.face_origin
    }

    pub fn face_origin(&self, face: FaceId) -> GeopResult<&str> {
        self.face_origin
            .get(&face)
            .map(String::as_str)
            .ok_or_else(|| {
                GeopError::new(format!("boolean naming: face {face} has no known origin"))
            })
    }

    /// Records that `vertex` was created where edge `edge_a` (at parameter
    /// `t_a`) crosses edge `edge_b` (at `t_b`).
    pub fn edge_crossing(
        &mut self,
        vertex: VertexId,
        (edge_a, t_a): (EdgeId, S),
        (edge_b, t_b): (EdgeId, S),
    ) -> GeopResult<()> {
        let a = self.edge_origin(edge_a)?.to_string();
        let b = self.edge_origin(edge_b)?.to_string();
        let ((along, t), other) = if a <= b { ((a, t_a), b) } else { ((b, t_b), a) };
        self.recipes.push(Recipe::Crossing {
            vertex,
            along,
            other,
            t,
        });
        Ok(())
    }

    /// Records that `vertex` was created where `edge` (at parameter `t`)
    /// pierces `face`, or where an intersection branch with `face` leaves
    /// it.
    pub fn piercing(
        &mut self,
        vertex: VertexId,
        edge: EdgeId,
        t: S,
        face: FaceId,
    ) -> GeopResult<()> {
        let along = self.edge_origin(edge)?.to_string();
        let other = self.face_origin(face)?.to_string();
        self.recipes.push(Recipe::Crossing {
            vertex,
            along,
            other,
            t,
        });
        Ok(())
    }

    /// Records that `new_edge` was split off `edge` at `start`; it inherits
    /// `edge`'s origin.
    pub fn edge_split(
        &mut self,
        edge: EdgeId,
        new_edge: EdgeId,
        start: VertexId,
    ) -> GeopResult<()> {
        let origin = self.edge_origin(edge)?.to_string();
        self.edge_origin.insert(new_edge, origin.clone());
        self.recipes.push(Recipe::EdgePiece {
            edge: new_edge,
            origin,
            start,
        });
        Ok(())
    }

    /// Records that `edge` was traced along `face_a` and `face_b` between
    /// `ends`.
    pub fn trace(
        &mut self,
        edge: EdgeId,
        face_a: FaceId,
        face_b: FaceId,
        ends: [VertexId; 2],
    ) -> GeopResult<()> {
        let faces = [
            self.face_origin(face_a)?.to_string(),
            self.face_origin(face_b)?.to_string(),
        ];
        self.recipes.push(Recipe::Trace { edge, faces, ends });
        Ok(())
    }

    /// Records that splicing `edge` into `face` split off `new_face`, if it
    /// did; `new_face` inherits `face`'s origin.
    pub fn face_split(
        &mut self,
        face: FaceId,
        edge: EdgeId,
        new_face: Option<FaceId>,
    ) -> GeopResult<()> {
        let Some(new_face) = new_face else {
            return Ok(());
        };
        let origin = self.face_origin(face)?.to_string();
        self.face_origin.insert(new_face, origin.clone());
        self.recipes.push(Recipe::FacePiece {
            face: new_face,
            origin,
            edge,
        });
        Ok(())
    }

    /// Gives every entity created so far its final name, see the module
    /// docs. Entities that were created and have since been deleted again
    /// (an edge piece merged into a coincident edge, say) are skipped.
    pub fn finish(self, part: &mut Part<S>) -> GeopResult<()> {
        let n = &self.namer;
        let ctx = |e: GeopError| e.with_context(format!("naming the entities of {}", n.root()));
        let alive = |part: &Part<S>, id: RefId| part.name_of(id).is_some();
        let name_of = |part: &Part<S>, id: RefId| -> GeopResult<String> {
            part.name_of(id).map(str::to_string).ok_or_else(|| {
                ctx(GeopError::new(format!(
                    "{id} is referred to by a name being built, but no longer exists"
                )))
            })
        };

        // Vertices first: every other name is built from vertex names.
        let mut crossings: BTreeMap<(&str, &str), Vec<(VertexId, S)>> = BTreeMap::new();
        for recipe in &self.recipes {
            if let Recipe::Crossing {
                vertex,
                along,
                other,
                t,
            } = recipe
                && alive(part, (*vertex).into())
            {
                crossings
                    .entry((along.as_str(), other.as_str()))
                    .or_default()
                    .push((*vertex, *t));
            }
        }
        for ((along, other), mut vertices) in crossings {
            // Distinct crossings of one edge have disjoint parameters, so
            // any point of each interval orders them.
            vertices.sort_by(|a, b| a.1.to_f64().total_cmp(&b.1.to_f64()));
            let count = vertices.len().to_string();
            for (i, (vertex, _)) in vertices.iter().enumerate() {
                part.rename(*vertex, n.name(&[along, other, &i.to_string(), &count]))
                    .map_err(ctx)?;
            }
        }

        let mut traces: BTreeMap<String, Vec<EdgeId>> = BTreeMap::new();
        for recipe in &self.recipes {
            match recipe {
                Recipe::EdgePiece {
                    edge,
                    origin,
                    start,
                } if alive(part, (*edge).into()) => {
                    let start = name_of(part, (*start).into())?;
                    part.rename(*edge, n.name(&[origin, &start])).map_err(ctx)?;
                }
                Recipe::Trace { edge, faces, ends } if alive(part, (*edge).into()) => {
                    let mut faces = faces.clone();
                    faces.sort();
                    let mut ends = [
                        name_of(part, ends[0].into())?,
                        name_of(part, ends[1].into())?,
                    ];
                    ends.sort();
                    traces
                        .entry(n.name(&[&faces[0], &faces[1], &ends[0], &ends[1]]))
                        .or_default()
                        .push(*edge);
                }
                _ => {}
            }
        }
        for (name, edges) in traces {
            if let [edge] = edges[..] {
                part.rename(edge, name).map_err(ctx)?;
            } else {
                for (k, edge) in edges.into_iter().enumerate() {
                    // `name` ends in `)`: append the index as one more
                    // argument.
                    let indexed = format!("{},{k})", &name[..name.len() - 1]);
                    part.rename(edge, indexed).map_err(ctx)?;
                }
            }
        }

        // Faces last: they are named after edges.
        for recipe in &self.recipes {
            if let Recipe::FacePiece { face, origin, edge } = recipe
                && alive(part, (*face).into())
            {
                let edge = name_of(part, (*edge).into())?;
                part.rename(*face, n.name(&[origin, &edge])).map_err(ctx)?;
            }
        }
        Ok(())
    }
}
