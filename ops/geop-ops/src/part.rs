use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Datum,
    scalars::Scalar,
};
use geop_core_topology::Model;

use crate::ids::{DatumId, RefId, SketchId};
use crate::names::NameRegistry;
use crate::sketch::PlacedSketch;

/// A complete, editable CAD part: its boundary-representation topology, the
/// sketches and datums used to build it, and a name for every one of those
/// entities.
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
    pub(crate) topology: Model<S>,
    pub(crate) names: NameRegistry,
    pub(crate) sketches: BTreeMap<SketchId, PlacedSketch<S>>,
    pub(crate) datums: BTreeMap<DatumId, Datum<S>>,
    /// The next sketch or datum id: ids count up in the order they are
    /// added, so iterating either map goes oldest first.
    next_id: u64,
}

impl<S: Scalar> Part<S> {
    pub fn new() -> Self {
        Self {
            topology: Model::new(),
            names: NameRegistry::new(),
            sketches: BTreeMap::new(),
            datums: BTreeMap::new(),
            next_id: 1,
        }
    }

    /// The part's topology, to query. Changing it goes through `Part`'s own
    /// methods, so that names stay in sync.
    pub fn topology(&self) -> &Model<S> {
        &self.topology
    }

    pub fn names(&self) -> &NameRegistry {
        &self.names
    }

    pub fn name_of(&self, id: impl Into<RefId>) -> Option<&str> {
        self.names.name_of(id)
    }

    pub fn id_of(&self, name: &str) -> Option<RefId> {
        self.names.id_of(name)
    }

    /// Gives `id` the name `new_name` instead — see [`NameRegistry::rename`]
    /// for the one situation this is for.
    pub fn rename(&mut self, id: impl Into<RefId>, new_name: impl Into<String>) -> GeopResult<()> {
        self.names.rename(id, new_name)
    }

    /// A sketch or datum id no entity has had yet.
    pub(crate) fn fresh_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    fn exists(&self, id: RefId) -> bool {
        match id {
            RefId::Vertex(id) => self.topology.vertices.contains_key(&id),
            RefId::Edge(id) => self.topology.edges.contains_key(&id),
            RefId::Face(id) => self.topology.faces.contains_key(&id),
            RefId::Solid(id) => self.topology.solids.contains_key(&id),
            RefId::Sketch(id) => self.sketches.contains_key(&id),
            RefId::Datum(id) => self.datums.contains_key(&id),
        }
    }

    /// Forgets the name of every entity that no longer exists — for an
    /// operation that deletes by reachability rather than one id at a time
    /// (see [`Model::assemble_solid`]).
    pub(crate) fn forget_dead_names(&mut self) {
        let alive: std::collections::HashSet<RefId> = self
            .names
            .iter()
            .map(|(id, _)| id)
            .filter(|&id| self.exists(id))
            .collect();
        self.names.retain(|id| alive.contains(&id));
    }

    /// Checks the invariant every method keeps: every vertex, edge, face,
    /// solid, sketch and datum has a name, and every name belongs to one of
    /// them.
    pub fn check_names(&self) -> GeopResult<()> {
        let topology = &self.topology;
        let entities = topology
            .vertices
            .keys()
            .map(|&id| RefId::from(id))
            .chain(topology.edges.keys().map(|&id| id.into()))
            .chain(topology.faces.keys().map(|&id| id.into()))
            .chain(topology.solids.keys().map(|&id| id.into()))
            .chain(self.sketches.keys().map(|&id| id.into()))
            .chain(self.datums.keys().map(|&id| id.into()));
        let unnamed: Vec<String> = entities
            .filter(|&id| self.names.name_of(id).is_none())
            .map(|id| id.to_string())
            .collect();
        let dead: Vec<&str> = self
            .names
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
    use crate::PlacedSketch;

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
