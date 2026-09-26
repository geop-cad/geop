use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, Face, FaceId, Sense, Shell, ShellId, Solid,
    SolidId, Vertex, VertexId,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use super::Model;

impl<S: Scalar> Model<S> {
    pub fn get_vertex(&self, id: VertexId) -> GeopResult<&Vertex<S>> {
        self.vertices
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("vertex with id {} does not exist", id.0)))
    }

    pub fn get_vertex_mut(&mut self, id: VertexId) -> GeopResult<&mut Vertex<S>> {
        self.vertices
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("vertex with id {} does not exist", id.0)))
    }

    pub fn get_edge(&self, id: EdgeId) -> GeopResult<&Edge<S>> {
        self.edges
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("edge with id {} does not exist", id.0)))
    }

    pub fn get_edge_mut(&mut self, id: EdgeId) -> GeopResult<&mut Edge<S>> {
        self.edges
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("edge with id {} does not exist", id.0)))
    }

    pub fn get_coedge(&self, id: CoedgeId) -> GeopResult<&Coedge<S>> {
        self.coedges
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("coedge with id {} does not exist", id.0)))
    }

    pub fn get_coedge_mut(&mut self, id: CoedgeId) -> GeopResult<&mut Coedge<S>> {
        self.coedges
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("coedge with id {} does not exist", id.0)))
    }

    pub fn get_face(&self, id: FaceId) -> GeopResult<&Face<S>> {
        self.faces
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("face with id {} does not exist", id.0)))
    }

    pub fn get_face_mut(&mut self, id: FaceId) -> GeopResult<&mut Face<S>> {
        self.faces
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("face with id {} does not exist", id.0)))
    }

    pub fn get_shell(&self, id: ShellId) -> GeopResult<&Shell> {
        self.shells
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("shell with id {} does not exist", id.0)))
    }

    pub fn get_shell_mut(&mut self, id: ShellId) -> GeopResult<&mut Shell> {
        self.shells
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("shell with id {} does not exist", id.0)))
    }

    pub fn get_solid(&self, id: SolidId) -> GeopResult<&Solid> {
        self.solids
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("solid with id {} does not exist", id.0)))
    }

    pub fn get_solid_mut(&mut self, id: SolidId) -> GeopResult<&mut Solid> {
        self.solids
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("solid with id {} does not exist", id.0)))
    }

    pub fn coedge_start_vertex_id(&self, coedge: CoedgeId) -> GeopResult<VertexId> {
        let coedge = self.get_coedge(coedge)?;
        match coedge.geometry {
            CoedgeGeometry::Edge(edge_id) => {
                let edge = self.get_edge(edge_id)?;
                Ok(match coedge.sense {
                    Sense::Forward => edge.start_vertex,
                    Sense::Reversed => edge.end_vertex,
                })
            }
            // Degenerate: sits at one vertex the whole way, so "start" and
            // "end" are the same.
            CoedgeGeometry::Vertex(vertex_id) => Ok(vertex_id),
        }
    }

    pub fn coedge_start_vertex(&self, coedge: CoedgeId) -> GeopResult<&Vertex<S>> {
        let vertex_id = self.coedge_start_vertex_id(coedge)?;
        self.get_vertex(vertex_id)
    }

    pub fn coedge_end_vertex_id(&self, coedge: CoedgeId) -> GeopResult<VertexId> {
        let coedge = self.get_coedge(coedge)?;
        match coedge.geometry {
            CoedgeGeometry::Edge(edge_id) => {
                let edge = self.get_edge(edge_id)?;
                Ok(match coedge.sense {
                    Sense::Forward => edge.end_vertex,
                    Sense::Reversed => edge.start_vertex,
                })
            }
            CoedgeGeometry::Vertex(vertex_id) => Ok(vertex_id),
        }
    }

    pub fn coedge_end_vertex(&self, coedge: CoedgeId) -> GeopResult<&Vertex<S>> {
        let vertex_id = self.coedge_end_vertex_id(coedge)?;
        self.get_vertex(vertex_id)
    }
}
