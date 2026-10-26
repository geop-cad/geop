use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, Face, FaceId, Sense, Shell, ShellId, Solid,
    SolidId, Vertex, VertexId, WireId,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    vector::Vector3,
};

use super::Model;

impl Model {
    pub fn get_vertex(&self, id: VertexId) -> GeopResult<&Vertex> {
        self.vertices
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("vertex with id {} does not exist", id.0)))
    }

    pub fn get_vertex_mut(&mut self, id: VertexId) -> GeopResult<&mut Vertex> {
        self.vertices
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("vertex with id {} does not exist", id.0)))
    }

    pub fn get_edge(&self, id: EdgeId) -> GeopResult<&Edge> {
        self.edges
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("edge with id {} does not exist", id.0)))
    }

    pub fn get_edge_mut(&mut self, id: EdgeId) -> GeopResult<&mut Edge> {
        self.edges
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("edge with id {} does not exist", id.0)))
    }

    pub fn get_coedge(&self, id: CoedgeId) -> GeopResult<&Coedge> {
        self.coedges
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("coedge with id {} does not exist", id.0)))
    }

    pub fn get_coedge_mut(&mut self, id: CoedgeId) -> GeopResult<&mut Coedge> {
        self.coedges
            .get_mut(&id)
            .ok_or_else(|| GeopError::new(format!("coedge with id {} does not exist", id.0)))
    }

    pub fn get_face(&self, id: FaceId) -> GeopResult<&Face> {
        self.faces
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("face with id {} does not exist", id.0)))
    }

    pub fn get_face_mut(&mut self, id: FaceId) -> GeopResult<&mut Face> {
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

    /// The wire `edge` is an edge of, if any.
    pub fn wire_of_edge(&self, edge: EdgeId) -> Option<WireId> {
        self.wires
            .iter()
            .find(|(_, w)| w.edges.contains(&edge))
            .map(|(&id, _)| id)
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

    pub fn coedge_start_vertex(&self, coedge: CoedgeId) -> GeopResult<&Vertex> {
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

    pub fn coedge_end_vertex(&self, coedge: CoedgeId) -> GeopResult<&Vertex> {
        let vertex_id = self.coedge_end_vertex_id(coedge)?;
        self.get_vertex(vertex_id)
    }

    /// The unit normal of `face`'s surface at its own vertex `vertex`: at
    /// the `(u, v)` its loop gives the vertex — where a coedge of it
    /// starting there starts — not at one searched for by the vertex's
    /// point. Where the patch's parametrization degenerates at the vertex —
    /// a SubD patch whose two sides leave its corner along one line — a
    /// whole valley of `(u, v)` lies within a search's reach of the point,
    /// and the search runs out of nodes; the loop knows which `(u, v)` it
    /// is. An error if no coedge of the face starts there.
    pub fn normal_at_vertex(&self, face: FaceId, vertex: VertexId) -> GeopResult<Vector3> {
        for coedge in self.iterate_face_coedges(face) {
            if self.coedge_start_vertex_id(coedge)? != vertex {
                continue;
            }
            let pcurve = &self.get_coedge(coedge)?.pcurve;
            let uv = pcurve.evaluate(pcurve.domain().0)?;
            return self.get_face(face)?.surface.normal(uv[0], uv[1]);
        }
        Err(GeopError::new(format!(
            "no coedge of face {face} starts at vertex {vertex}: it is no vertex of the face"
        )))
    }
}
