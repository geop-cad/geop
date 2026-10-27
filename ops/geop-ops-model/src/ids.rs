use geop_core_topology::{EdgeId, FaceId, SolidId, VertexId};

/// Every kind of entity a [`super::NamedModel`] names: the topology
/// entities a user can pick in isolation (a coedge or shell never is).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum RefId {
    Vertex(VertexId),
    Edge(EdgeId),
    Face(FaceId),
    Solid(SolidId),
}

impl std::fmt::Display for RefId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RefId::Vertex(id) => write!(f, "{id}"),
            RefId::Edge(id) => write!(f, "{id}"),
            RefId::Face(id) => write!(f, "{id}"),
            RefId::Solid(id) => write!(f, "{id}"),
        }
    }
}

impl From<VertexId> for RefId {
    fn from(id: VertexId) -> Self {
        RefId::Vertex(id)
    }
}

impl From<EdgeId> for RefId {
    fn from(id: EdgeId) -> Self {
        RefId::Edge(id)
    }
}

impl From<FaceId> for RefId {
    fn from(id: FaceId) -> Self {
        RefId::Face(id)
    }
}

impl From<SolidId> for RefId {
    fn from(id: SolidId) -> Self {
        RefId::Solid(id)
    }
}
