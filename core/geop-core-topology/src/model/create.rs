use crate::{
    Coedge, CoedgeId, Edge, EdgeId, Face, FaceId, Shell, ShellId, Solid, SolidId, Vertex, VertexId,
    Wire, WireId,
};

use super::Model;

impl Model {
    pub fn insert_vertex(&mut self, v: Vertex) -> VertexId {
        let id = VertexId(self.fresh_id());
        self.vertices.insert(id, v);
        id
    }

    pub fn insert_edge(&mut self, e: Edge) -> EdgeId {
        let id = EdgeId(self.fresh_id());
        self.edges.insert(id, e);
        id
    }

    pub fn insert_coedge(&mut self, c: Coedge) -> CoedgeId {
        let id = CoedgeId(self.fresh_id());
        self.coedges.insert(id, c);
        id
    }

    pub fn insert_face(&mut self, f: Face) -> FaceId {
        let id = FaceId(self.fresh_id());
        self.faces.insert(id, f);
        id
    }

    pub fn insert_shell(&mut self, s: Shell) -> ShellId {
        let id = ShellId(self.fresh_id());
        self.shells.insert(id, s);
        id
    }

    pub fn insert_solid(&mut self, s: Solid) -> SolidId {
        let id = SolidId(self.fresh_id());
        self.solids.insert(id, s);
        id
    }

    pub fn insert_wire(&mut self, w: Wire) -> WireId {
        let id = WireId(self.fresh_id());
        self.wires.insert(id, w);
        id
    }
}
