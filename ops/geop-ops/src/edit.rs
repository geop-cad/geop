//! Forwarding wrappers for every `Model` edit operator, exactly like
//! `euler.rs`: each registers what it creates under the caller's name and
//! forgets what it deletes.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};
use geop_core_topology::{Edge, EdgeId, FaceId, SolidId, Vertex, VertexId};

use crate::part::Part;

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

    /// Forwards to [`geop_core_topology::Model::assemble_solid`], naming the
    /// solid it creates (if any) and forgetting the names of everything it
    /// deletes.
    pub fn assemble_solid(
        &mut self,
        consumed: &[SolidId],
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

    /// Forwards to [`geop_core_topology::Model::merge_solids`], forgetting
    /// the name of the solid it deletes.
    pub fn merge_solids(&mut self, into: SolidId, from: SolidId) -> GeopResult<()> {
        self.topology.merge_solids(into, from)?;
        self.names.remove(from);
        Ok(())
    }
}
