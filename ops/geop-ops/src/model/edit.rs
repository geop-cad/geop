//! Forwarding wrappers for every `Model` edit operator, exactly like
//! `euler.rs`: each registers what it creates under the caller's name and
//! forgets what it deletes.

use std::collections::HashSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::{
    Body, Edge, EdgeId, FaceId, ShellId, SolidId, Vertex, VertexId,
    build::{BodySpec, BuiltBody},
};

use super::Part;

impl<S: Scalar> Part<S> {
    /// Forwards to [`geop_core_topology::Model::insert_vertex`]: a vertex on
    /// nothing yet, about to be split into an edge (see
    /// [`Part::split_edge_at_vertex`]).
    pub fn insert_vertex(
        &mut self,
        point: Vector3<S>,
        name: impl Into<String>,
    ) -> GeopResult<VertexId> {
        let vertex = self.store.topology_mut().insert_vertex(Vertex { point });
        self.store.insert_name(vertex, name)?;
        Ok(vertex)
    }

    /// Forwards to [`geop_core_topology::Model::insert_edge`]: an edge on no
    /// face yet, about to be spliced into its faces (see
    /// [`Part::splice_edge_into_face`]).
    pub fn insert_edge(&mut self, edge: Edge<S>, name: impl Into<String>) -> GeopResult<EdgeId> {
        let edge = self.store.topology_mut().insert_edge(edge);
        self.store.insert_name(edge, name)?;
        Ok(edge)
    }

    /// Forwards to [`geop_core_topology::Model::merge_vertex`], forgetting
    /// the name of the vertex it deletes. The survivor keeps its own name.
    pub fn merge_vertex(
        &mut self,
        vertex_into_id: VertexId,
        vertex_deleted_id: VertexId,
    ) -> GeopResult<()> {
        self.store
            .topology_mut()
            .merge_vertex(vertex_into_id, vertex_deleted_id)?;
        self.store.remove_name(vertex_deleted_id);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::merge_edge`], forgetting the
    /// name of the edge it deletes. The survivor keeps its own name.
    pub fn merge_edge(
        &mut self,
        edge_into_id: EdgeId,
        edge_deleted_id: EdgeId,
        reversed: bool,
    ) -> GeopResult<()> {
        self.store
            .topology_mut()
            .merge_edge(edge_into_id, edge_deleted_id, reversed)?;
        self.store.remove_name(edge_deleted_id);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::reverse_face`]. Creates and
    /// deletes nothing.
    pub fn reverse_face(&mut self, face_id: FaceId) -> GeopResult<()> {
        self.store.topology_mut().reverse_face(face_id)
    }

    /// Forwards to [`geop_core_topology::Model::splice_edge_into_face`].
    ///
    /// Whether that creates a face depends on how the edge's ends sit on the
    /// face's boundary, which a caller imprinting a traced curve cannot know
    /// beforehand — so `new_face_name` is the name for the face *if* one is
    /// created, and unused otherwise.
    pub fn splice_edge_into_face(
        &mut self,
        edge_id: EdgeId,
        face_id: FaceId,
        max_nodes: usize,
        min_subdivision_size: S,
        new_face_name: impl Into<String>,
    ) -> GeopResult<Option<FaceId>> {
        let new_face = self.store.topology_mut().splice_edge_into_face(
            edge_id,
            face_id,
            max_nodes,
            min_subdivision_size,
        )?;
        if let Some(face) = new_face {
            self.store.insert_name(face, new_face_name)?;
        }
        Ok(new_face)
    }

    /// Forwards to [`geop_core_topology::Model::split_edge_at_vertex`],
    /// naming the new edge it creates: the (vertex -> end) segment, while
    /// `edge_id` keeps the (start -> vertex) one and its name.
    pub fn split_edge_at_vertex(
        &mut self,
        edge_id: EdgeId,
        edge_t: S,
        vertex_id: VertexId,
        max_nodes: usize,
        min_subdivision_size: S,
        new_edge_name: impl Into<String>,
    ) -> GeopResult<EdgeId> {
        let new_edge = self.store.topology_mut().split_edge_at_vertex(
            edge_id,
            edge_t,
            vertex_id,
            max_nodes,
            min_subdivision_size,
        )?;
        self.store.insert_name(new_edge, new_edge_name)?;
        Ok(new_edge)
    }

    /// Forwards to [`geop_core_topology::Model::build_body`], naming
    /// everything it builds after `names`, index for index. Fails, leaving
    /// the part unchanged, if a name is missing, repeated or taken.
    pub fn build_body(&mut self, spec: BodySpec<S>, names: BodyNames) -> GeopResult<BuiltBody> {
        let counts = [
            (names.vertices.len(), spec.vertices.len(), "vertices"),
            (names.edges.len(), spec.edges.len(), "edges"),
            (names.faces.len(), spec.faces.len(), "faces"),
        ];
        if let Some((n, m, what)) = counts.iter().find(|(n, m, _)| n != m) {
            return Err(GeopError::new(format!(
                "Part::build_body: {n} names for {m} {what}"
            )));
        }
        if names.solid.is_some() != spec.solid {
            return Err(GeopError::new(format!(
                "Part::build_body: a solid name {:?} for a body that {} a solid",
                names.solid,
                if spec.solid { "is" } else { "is not" }
            )));
        }
        let all = names
            .vertices
            .iter()
            .chain(&names.edges)
            .chain(&names.faces)
            .chain(&names.solid);
        let mut seen = HashSet::new();
        for name in all {
            if !seen.insert(name) || self.store.id_of(name).is_some() {
                return Err(GeopError::new(format!(
                    "Part::build_body: the name {name:?} is taken"
                )));
            }
        }
        let built = self.store.topology_mut().build_body(spec)?;
        for (&id, name) in built.vertices.iter().zip(names.vertices) {
            self.store.insert_name(id, name)?;
        }
        for (&id, name) in built.edges.iter().zip(names.edges) {
            self.store.insert_name(id, name)?;
        }
        for (&id, name) in built.faces.iter().zip(names.faces) {
            self.store.insert_name(id, name)?;
        }
        if let (Some(id), Some(name)) = (built.solid, names.solid) {
            self.store.insert_name(id, name)?;
        }
        Ok(built)
    }

    /// Copies `faces`, with every edge and vertex they use, into a body of
    /// their own that shares nothing with the originals (see
    /// [`geop_core_topology::Model::body_spec`]): a solid named `solid`, if
    /// given, or else a sheet. The copy of each entity named `X` is named
    /// `rename(X)`.
    pub fn copy_faces(
        &mut self,
        faces: &[FaceId],
        solid: Option<String>,
        rename: impl Fn(&str) -> String,
    ) -> GeopResult<BuiltBody> {
        let (spec, sources) = self.topology().body_spec(faces, solid.is_some())?;
        let copied = |ids: Vec<super::RefId>| -> GeopResult<Vec<String>> {
            ids.into_iter()
                .map(|id| {
                    let name = self.name_of(id).ok_or_else(|| {
                        GeopError::new(format!("Part::copy_faces: {id} has no name"))
                    })?;
                    Ok(rename(name))
                })
                .collect()
        };
        let names = BodyNames {
            vertices: copied(sources.vertices.iter().map(|&v| v.into()).collect())?,
            edges: copied(sources.edges.iter().map(|&e| e.into()).collect())?,
            faces: copied(sources.faces.iter().map(|&f| f.into()).collect())?,
            solid,
        };
        self.build_body(spec, names)
    }

    /// Copies the whole of `body` — a solid, its shells kept apart, or a
    /// sheet — into a body of its own that shares nothing with it. The copy
    /// of each entity named `X`, the solid included, is named `rename(X)`.
    pub fn copy_body(
        &mut self,
        body: Body,
        rename: impl Fn(&str) -> String,
    ) -> GeopResult<BuiltBody> {
        let (spec, names) = self.body_record(body)?;
        self.build_body(spec, names.renamed(rename))
    }

    /// The whole of `body` — a solid, its shells kept apart, or a sheet —
    /// as a body to build again (see [`Part::build_body`]), with the names
    /// of its entities: what [`Part::copy_body`] builds, kept to build
    /// later.
    pub fn body_record(&self, body: Body) -> GeopResult<(BodySpec<S>, BodyNames)> {
        let faces = self.topology().body_faces(body)?;
        let name = |id: super::RefId| -> GeopResult<String> {
            self.name_of(id)
                .map(str::to_string)
                .ok_or_else(|| GeopError::new(format!("Part::body_record: {id} has no name")))
        };
        let solid = match body {
            Body::Solid(solid) => Some(name(solid.into())?),
            Body::Sheet(_) => None,
        };
        let (mut spec, sources) = self.topology().body_spec(&faces, solid.is_some())?;
        // `body_spec` puts every face into one shell; a solid with a void
        // has more, which the record keeps.
        let mut shells: Vec<(ShellId, Vec<usize>)> = Vec::new();
        for (index, &face) in sources.faces.iter().enumerate() {
            let shell = self.topology().get_face(face)?.shell;
            match shells.iter_mut().find(|(s, _)| *s == shell) {
                Some((_, members)) => members.push(index),
                None => shells.push((shell, vec![index])),
            }
        }
        spec.shells = shells.into_iter().map(|(_, members)| members).collect();
        let names = BodyNames {
            vertices: sources
                .vertices
                .iter()
                .map(|&v| name(v.into()))
                .collect::<GeopResult<_>>()?,
            edges: sources
                .edges
                .iter()
                .map(|&e| name(e.into()))
                .collect::<GeopResult<_>>()?,
            faces: sources
                .faces
                .iter()
                .map(|&f| name(f.into()))
                .collect::<GeopResult<_>>()?,
            solid,
        };
        Ok((spec, names))
    }

    /// Forwards to [`geop_core_topology::Model::transform_body`]. Creates
    /// and deletes nothing, so every entity keeps its name.
    pub fn transform_body(&mut self, body: Body, motion: &Motion<S>) -> GeopResult<()> {
        self.store.topology_mut().transform_body(body, motion)
    }

    /// Forwards to [`geop_core_topology::Model::assemble_solid`], naming the
    /// solid it creates (if any) and forgetting the names of everything it
    /// deletes.
    pub fn assemble_solid(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
        solid_name: impl Into<String>,
    ) -> GeopResult<Option<SolidId>> {
        let solid = self.store.topology_mut().assemble_solid(consumed, keep)?;
        self.forget_dead_names();
        if let Some(solid) = solid {
            self.store.insert_name(solid, solid_name)?;
        }
        Ok(solid)
    }

    /// Forwards to [`geop_core_topology::Model::assemble_sheet`], forgetting
    /// the names of everything it deletes.
    pub fn assemble_sheet(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
    ) -> GeopResult<Option<ShellId>> {
        let sheet = self.store.topology_mut().assemble_sheet(consumed, keep)?;
        self.forget_dead_names();
        Ok(sheet)
    }

    /// Forwards to [`geop_core_topology::Model::merge_solids`], forgetting
    /// the name of the solid it deletes.
    pub fn merge_solids(&mut self, into: SolidId, from: SolidId) -> GeopResult<()> {
        self.store.topology_mut().merge_solids(into, from)?;
        self.store.remove_name(from);
        Ok(())
    }
}

/// The names [`Part::build_body`] gives what it builds: one per vertex, edge
/// and face of the [`BodySpec`], index for index, and the solid's, if it
/// builds one.
#[derive(Clone, Debug, Default)]
pub struct BodyNames {
    pub vertices: Vec<String>,
    pub edges: Vec<String>,
    pub faces: Vec<String>,
    pub solid: Option<String>,
}

impl BodyNames {
    /// The same names, each `X` as `rename(X)`.
    pub fn renamed(&self, rename: impl Fn(&str) -> String) -> BodyNames {
        let all = |names: &[String]| names.iter().map(|n| rename(n)).collect();
        BodyNames {
            vertices: all(&self.vertices),
            edges: all(&self.edges),
            faces: all(&self.faces),
            solid: self.solid.as_deref().map(&rename),
        }
    }
}
