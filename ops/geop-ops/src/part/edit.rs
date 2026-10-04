//! Forwarding wrappers for every `Model` edit operator, exactly like
//! `euler.rs`: each registers what it creates under the caller's name and
//! forgets what it deletes.

use std::collections::HashSet;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
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
        let vertex = self.topology.insert_vertex(Vertex { point });
        self.names.insert(vertex, name)?;
        Ok(vertex)
    }

    /// Forwards to [`geop_core_topology::Model::insert_edge`]: an edge on no
    /// face yet, about to be spliced into its faces (see
    /// [`Part::splice_edge_into_face`]).
    pub fn insert_edge(&mut self, edge: Edge<S>, name: impl Into<String>) -> GeopResult<EdgeId> {
        let edge = self.topology.insert_edge(edge);
        self.names.insert(edge, name)?;
        Ok(edge)
    }

    /// Forwards to [`geop_core_topology::Model::merge_vertex`], forgetting
    /// the name of the vertex it deletes. The survivor keeps its own name.
    pub fn merge_vertex(
        &mut self,
        vertex_into_id: VertexId,
        vertex_deleted_id: VertexId,
    ) -> GeopResult<()> {
        self.topology
            .merge_vertex(vertex_into_id, vertex_deleted_id)?;
        self.names.remove(vertex_deleted_id);
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
        self.topology
            .merge_edge(edge_into_id, edge_deleted_id, reversed)?;
        self.names.remove(edge_deleted_id);
        Ok(())
    }

    /// Forwards to [`geop_core_topology::Model::reverse_face`]. Creates and
    /// deletes nothing.
    pub fn reverse_face(&mut self, face_id: FaceId) -> GeopResult<()> {
        self.topology.reverse_face(face_id)
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
        let new_face = self.topology.splice_edge_into_face(
            edge_id,
            face_id,
            max_nodes,
            min_subdivision_size,
        )?;
        if let Some(face) = new_face {
            self.names.insert(face, new_face_name)?;
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
        let new_edge = self.topology.split_edge_at_vertex(
            edge_id,
            edge_t,
            vertex_id,
            max_nodes,
            min_subdivision_size,
        )?;
        self.names.insert(new_edge, new_edge_name)?;
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
            if !seen.insert(name) || self.names.id_of(name).is_some() {
                return Err(GeopError::new(format!(
                    "Part::build_body: the name {name:?} is taken"
                )));
            }
        }
        let built = self.topology.build_body(spec)?;
        for (&id, name) in built.vertices.iter().zip(names.vertices) {
            self.names.insert(id, name)?;
        }
        for (&id, name) in built.edges.iter().zip(names.edges) {
            self.names.insert(id, name)?;
        }
        for (&id, name) in built.faces.iter().zip(names.faces) {
            self.names.insert(id, name)?;
        }
        if let (Some(id), Some(name)) = (built.solid, names.solid) {
            self.names.insert(id, name)?;
        }
        Ok(built)
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
        let solid = self.topology.assemble_solid(consumed, keep)?;
        self.forget_dead_names();
        if let Some(solid) = solid {
            self.names.insert(solid, solid_name)?;
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
        let sheet = self.topology.assemble_sheet(consumed, keep)?;
        self.forget_dead_names();
        Ok(sheet)
    }

    /// Forwards to [`geop_core_topology::Model::merge_solids`], forgetting
    /// the name of the solid it deletes.
    pub fn merge_solids(&mut self, into: SolidId, from: SolidId) -> GeopResult<()> {
        self.topology.merge_solids(into, from)?;
        self.names.remove(from);
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
