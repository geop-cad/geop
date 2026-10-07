//! [`Part`]: the topology, sketches and datums of a part, the parts placed
//! in it and the mates between them, each named, and the names themselves
//! ([`NameRegistry`], [`Namer`]) — changed only through `Part`'s own
//! methods, so no entity is ever without a name.

use std::{
    any::Any,
    sync::{
        Arc,
        atomic::{AtomicU64, Ordering},
    },
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{CoordinateSystem, Datum, DatumKind},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::Model;

mod cache;
mod cells;
mod datum;
mod describe;
mod edit;
mod euler;
mod extension;
mod feature;
mod ids;
mod instance;
mod mesh;
mod names;
mod resolve;
mod sketch;
mod sketch3d;
mod state;

pub use cells::{Access, Cell, Log};
pub use describe::{EdgeDescription, FaceDescription, InstanceDescription, PartDescription};
pub use edit::BodyNames;
pub use extension::{Annotation, Extension};
pub use feature::{BooleanOp, Feature, FeatureFaces, FeatureTool};
pub use ids::{DatumId, InstanceId, RefId, Sketch3dId, SketchId};
pub use instance::Instance;
pub use mesh::SolidMeshes;
pub use names::{NameRegistry, Namer, operation_of, validate_operation_id};
pub use sketch::PlacedSketch;
pub use state::{ParamValue, State, pose_parameter};

/// A complete, editable CAD part: its boundary-representation topology, the
/// sketches — planar and 3-D — and datums used to build it — starting with the frame
/// [`ORIGIN`] — the other parts placed in it and the mates holding those
/// together, and a name for every one of those entities.
///
/// A part placed in another is a part too, so a part is a tree: an
/// assembly of assemblies, as deep as its program files place each other
/// (see [`crate::program::Library`]).
///
/// The fields are private: the only way to change a part is through its
/// methods, each of which forwards straight to the identically named
/// [`Model`] operation, registers every vertex/edge/face/solid it created
/// under the name the caller supplied, and forgets the name of every one it
/// deleted. That keeps the invariant [`Part::check_names`] checks — every
/// entity has exactly one name, and every name one entity — true by
/// construction rather than by each caller's diligence.
#[derive(Clone)]
pub struct Part<S: Scalar> {
    /// Everything a step reads and writes (see [`Cell`]).
    store: cells::Store<S>,
    /// What was worked out of it once placed (see [`Part::view`]).
    cache: cache::Cache<S>,
    /// Which build it is (see [`Part::revision`]).
    revision: u64,
}

/// The revision the next part built gets (see [`Part::revision`]).
static NEXT_REVISION: AtomicU64 = AtomicU64::new(1);

/// The name of the frame datum every part starts with: the world's origin
/// and axes, and the three planes between them (see
/// [`geop_core_math::primitives::DatumComponent`]).
pub const ORIGIN: &str = "origin";

impl<S: Scalar> Part<S> {
    /// A part with nothing in it but the frame [`ORIGIN`].
    pub fn new() -> Self {
        let mut part = Self {
            store: cells::Store::new(),
            cache: cache::Cache::new(),
            revision: 0,
        };
        part.renew_revision();
        let origin = Datum {
            kind: DatumKind::Frame,
            frame: CoordinateSystem::world_at(Vector3::zero()),
        };
        part.add_datum(origin, ORIGIN)
            .expect("a new part has no names taken");
        part
    }

    /// Which build of a part it is: a new part, and each part a program's
    /// step builds (see [`crate::program::ProgramRunner`]), has a revision
    /// no other has, and a copy keeps it. What is worked out from a part —
    /// a drawing's views — can be kept for as long as its revision is the
    /// same.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Gives it a revision of its own: it was built anew.
    pub(crate) fn renew_revision(&mut self) {
        self.revision = NEXT_REVISION.fetch_add(1, Ordering::Relaxed);
    }

    /// The part's topology, to query. Changing it goes through `Part`'s own
    /// methods, so that names stay in sync.
    pub fn topology(&self) -> &Model<S> {
        self.store.topology()
    }

    pub fn names(&self) -> &NameRegistry {
        self.store.names()
    }

    pub fn name_of(&self, id: impl Into<RefId>) -> Option<&str> {
        self.store.name_of(id.into())
    }

    pub fn id_of(&self, name: &str) -> Option<RefId> {
        self.store.id_of(name)
    }

    /// Gives `id` the name `new_name` instead — see [`NameRegistry::rename`]
    /// for the one situation this is for.
    pub fn rename(&mut self, id: impl Into<RefId>, new_name: impl Into<String>) -> GeopResult<()> {
        self.store.rename(id.into(), new_name)
    }

    fn exists(&self, id: RefId) -> bool {
        match id {
            RefId::Vertex(id) => self.topology().vertices.contains_key(&id),
            RefId::Edge(id) => self.topology().edges.contains_key(&id),
            RefId::Face(id) => self.topology().faces.contains_key(&id),
            RefId::Solid(id) => self.topology().solids.contains_key(&id),
            RefId::Sketch(id) => self.store.sketches().contains_key(&id),
            RefId::Sketch3d(id) => self.store.sketches3d().contains_key(&id),
            RefId::Datum(id) => self.store.datums().contains_key(&id),
            RefId::Instance(id) => self.store.instance(id).is_some(),
        }
    }

    /// Records `data` on the solid named `solid`: what the operation family
    /// that built it needs to know about it beyond its topology — which of
    /// a sheet-metal body's faces are bends, say — for the operations of
    /// that family to read back ([`Part::body_data`]). Replaces whatever
    /// was recorded on it before.
    ///
    /// The record belongs to that one solid: an operation that consumes the
    /// solid — any that does not know the record — builds a solid of
    /// another name, which has none.
    pub fn set_body_data<T: Any + Send + Sync>(&mut self, solid: &str, data: T) -> GeopResult<()> {
        self.solid_id(solid)?;
        self.store
            .body_data_mut()
            .insert(solid.to_string(), Arc::new(data));
        Ok(())
    }

    /// What was recorded on the solid named `solid` (see
    /// [`Part::set_body_data`]), if it is of type `T` and the solid still
    /// exists.
    pub fn body_data<T: Any>(&self, solid: &str) -> Option<&T> {
        self.solid_id(solid).ok()?;
        self.store.body_data().get(solid)?.downcast_ref()
    }

    /// Forgets the name of every entity that no longer exists — for an
    /// operation that deletes by reachability rather than one id at a time
    /// (see [`Model::assemble_solid`]).
    pub(crate) fn forget_dead_names(&mut self) {
        let alive: std::collections::HashSet<RefId> = self
            .names()
            .iter()
            .map(|(id, _)| id)
            .filter(|&id| self.exists(id))
            .collect();
        self.store.retain_names(|id| alive.contains(&id));
        let names = self.store.names();
        let dead: Vec<String> = self
            .store
            .body_data()
            .keys()
            .filter(|solid| !matches!(names.id_of(solid), Some(RefId::Solid(_))))
            .cloned()
            .collect();
        for solid in dead {
            self.store.body_data_mut().remove(&solid);
        }
    }

    /// Checks the invariant every method keeps: every vertex, edge, face,
    /// solid, sketch, datum and instance has a name, and every name belongs
    /// to one of them.
    pub fn check_names(&self) -> GeopResult<()> {
        let topology = self.topology();
        let entities = topology
            .vertices
            .keys()
            .map(|&id| RefId::from(id))
            .chain(topology.edges.keys().map(|&id| id.into()))
            .chain(topology.faces.keys().map(|&id| id.into()))
            .chain(topology.solids.keys().map(|&id| id.into()))
            .chain(self.store.sketches().keys().map(|&id| id.into()))
            .chain(self.store.sketches3d().keys().map(|&id| id.into()))
            .chain(self.store.datums().keys().map(|&id| id.into()))
            .chain(self.store.instances().keys().map(|&id| id.into()));
        let unnamed: Vec<String> = entities
            .filter(|&id| self.name_of(id).is_none())
            .map(|id| id.to_string())
            .collect();
        let dead: Vec<&str> = self
            .names()
            .iter()
            .filter(|&(id, _)| !self.exists(id))
            .map(|(_, name)| name)
            .collect();
        if unnamed.is_empty() && dead.is_empty() {
            Ok(())
        } else {
            Err(GeopError::new(format!(
                "Part::check_names: unnamed entities {unnamed:?}, names of deleted entities {dead:?}"
            )))
        }
    }
}

impl<S: Scalar> Default for Part<S> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::scal_in_f64::ScalInF64, vector::Vector3};
    use geop_core_sketch::Sketch;

    use super::*;

    fn origin() -> Vector3<ScalInF64> {
        Vector3::from_array([ScalInF64::from_f64(0.0); 3])
    }

    /// `mvfs` creates a vertex, a face and a solid; each must be given a
    /// name, and `kvfs` undoing it erases all three names again.
    #[test]
    fn mvfs_and_kvfs_keep_names_in_sync() {
        let mut part = Part::<ScalInF64>::new();
        let (vertex, face, solid) = part.mvfs(origin(), "v0", "f0", "s0").unwrap();

        assert_eq!(part.name_of(vertex), Some("v0"));
        assert_eq!(part.name_of(face), Some("f0"));
        assert_eq!(part.name_of(solid), Some("s0"));
        assert_eq!(part.id_of("v0"), Some(RefId::Vertex(vertex)));
        part.check_names().unwrap();

        part.kvfs(solid).unwrap();

        assert_eq!(part.name_of(vertex), None);
        assert_eq!(part.name_of(face), None);
        assert_eq!(part.name_of(solid), None);
        assert!(part.topology().vertices.is_empty());
        assert!(part.topology().faces.is_empty());
        assert!(part.topology().solids.is_empty());
        part.check_names().unwrap();
    }

    /// Reusing a name that's already taken is rejected.
    #[test]
    fn duplicate_name_is_rejected() {
        let mut part = Part::<ScalInF64>::new();
        part.mvfs(origin(), "v0", "f0", "s0").unwrap();

        let err = part.mvfs(origin(), "v1", "f1", "s0");
        assert!(err.is_err());
    }

    /// A sketch is named like any other entity, and removing it forgets the
    /// name.
    #[test]
    fn sketches_are_named() {
        let mut part = Part::<ScalInF64>::new();
        let placed = PlacedSketch {
            plane: geop_core_math::primitives::CoordinateSystem::try_new(
                origin(),
                Vector3::from_array([ScalInF64::ONE, ScalInF64::ZERO, ScalInF64::ZERO]),
                Vector3::from_array([ScalInF64::ZERO, ScalInF64::ONE, ScalInF64::ZERO]),
                Vector3::from_array([ScalInF64::ZERO, ScalInF64::ZERO, ScalInF64::ONE]),
            )
            .unwrap(),
            sketch: Sketch::new(),
        };
        let id = part.add_sketch(placed, "sketch0").unwrap();
        assert_eq!(part.name_of(id), Some("sketch0"));
        assert_eq!(part.sketch_id("sketch0").unwrap(), id);
        part.check_names().unwrap();

        part.remove_sketch(id).unwrap();
        assert_eq!(part.name_of(id), None);
        assert!(part.sketch(id).is_err());
    }

    /// An id comes from the entity's name alone, whatever was added before
    /// it, while iteration stays in the order of adding.
    #[test]
    fn ids_come_from_names_and_iteration_keeps_the_order_added() {
        let datum = || Datum {
            kind: DatumKind::Frame,
            frame: CoordinateSystem::world_at(origin()),
        };
        let mut a = Part::<ScalInF64>::new();
        let (x, y) = (
            a.add_datum(datum(), "x").unwrap(),
            a.add_datum(datum(), "y").unwrap(),
        );
        let mut b = Part::<ScalInF64>::new();
        let y_first = b.add_datum(datum(), "y").unwrap();
        let x_second = b.add_datum(datum(), "x").unwrap();

        assert_eq!((x, y), (x_second, y_first));
        let order = |part: &Part<ScalInF64>| -> Vec<String> {
            part.datums()
                .map(|(id, _)| part.name_of(id).unwrap().to_string())
                .collect()
        };
        assert_eq!(order(&a), [ORIGIN, "x", "y"]);
        assert_eq!(order(&b), [ORIGIN, "y", "x"]);
        assert!(a.add_datum(datum(), "x").is_err());
    }

    /// A provisional name can be settled, but not onto a name in use.
    #[test]
    fn rename_settles_a_provisional_name() {
        let mut part = Part::<ScalInF64>::new();
        let (vertex, _, _) = part.mvfs(origin(), "v~0", "f0", "s0").unwrap();
        assert!(part.rename(vertex, "f0").is_err());
        part.rename(vertex, "v0").unwrap();
        assert_eq!(part.name_of(vertex), Some("v0"));
        assert_eq!(part.id_of("v~0"), None);
    }
}
