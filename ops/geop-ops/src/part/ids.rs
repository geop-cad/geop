use geop_core_topology::{EdgeId, FaceId, SolidId, VertexId};

/// A 64-bit FNV-1a hash of `name`. Fixed by its definition, so the same
/// name gives the same id in every build and run of a program.
fn hash_of(name: &str) -> u64 {
    name.bytes().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
    })
}

/// A [`crate::Part`]'s own id for one of its sketches.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct SketchId(u64);

impl SketchId {
    /// The id of the entity named `name`: derived from the name alone, so
    /// that it does not depend on how many entities were added before it.
    /// Two names with the same id are refused by the [`super::NameRegistry`].
    pub(super) fn named(name: &str) -> Self {
        Self(hash_of(name))
    }
}

impl std::fmt::Display for SketchId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "SketchId({})", self.0)
    }
}

/// A [`crate::Part`]'s own id for one of its 3-D sketches.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Sketch3dId(u64);

impl Sketch3dId {
    /// The id of the entity named `name`: derived from the name alone, so
    /// that it does not depend on how many entities were added before it.
    /// Two names with the same id are refused by the [`super::NameRegistry`].
    pub(super) fn named(name: &str) -> Self {
        Self(hash_of(name))
    }
}

impl std::fmt::Display for Sketch3dId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Sketch3dId({})", self.0)
    }
}

/// A [`crate::Part`]'s own id for one of its datums.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct DatumId(u64);

impl DatumId {
    /// The id of the entity named `name`: derived from the name alone, so
    /// that it does not depend on how many entities were added before it.
    /// Two names with the same id are refused by the [`super::NameRegistry`].
    pub(super) fn named(name: &str) -> Self {
        Self(hash_of(name))
    }
}

impl std::fmt::Display for DatumId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "DatumId({})", self.0)
    }
}

/// A [`crate::Part`]'s own id for one of the parts placed in it (see
/// [`crate::part::Instance`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct InstanceId(u64);

impl InstanceId {
    /// The id of the entity named `name`: derived from the name alone, so
    /// that it does not depend on how many entities were added before it.
    /// Two names with the same id are refused by the [`super::NameRegistry`].
    pub(super) fn named(name: &str) -> Self {
        Self(hash_of(name))
    }
}

impl std::fmt::Display for InstanceId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "InstanceId({})", self.0)
    }
}

/// Every kind of entity a [`crate::Part`] names: the topology entities a user
/// can pick in isolation (a coedge or shell never is), its sketches — planar
/// and 3-D — its datums and the parts placed in it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum RefId {
    Vertex(VertexId),
    Edge(EdgeId),
    Face(FaceId),
    Solid(SolidId),
    Sketch(SketchId),
    Sketch3d(Sketch3dId),
    Datum(DatumId),
    Instance(InstanceId),
}

impl std::fmt::Display for RefId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RefId::Vertex(id) => write!(f, "{id}"),
            RefId::Edge(id) => write!(f, "{id}"),
            RefId::Face(id) => write!(f, "{id}"),
            RefId::Solid(id) => write!(f, "{id}"),
            RefId::Sketch(id) => write!(f, "{id}"),
            RefId::Sketch3d(id) => write!(f, "{id}"),
            RefId::Datum(id) => write!(f, "{id}"),
            RefId::Instance(id) => write!(f, "{id}"),
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
impl From<SketchId> for RefId {
    fn from(id: SketchId) -> Self {
        RefId::Sketch(id)
    }
}
impl From<Sketch3dId> for RefId {
    fn from(id: Sketch3dId) -> Self {
        RefId::Sketch3d(id)
    }
}
impl From<DatumId> for RefId {
    fn from(id: DatumId) -> Self {
        RefId::Datum(id)
    }
}
impl From<InstanceId> for RefId {
    fn from(id: InstanceId) -> Self {
        RefId::Instance(id)
    }
}
