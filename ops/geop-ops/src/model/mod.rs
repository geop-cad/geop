//! [`NamedModel`]: a part's topology with a name for every entity in it
//! ([`NameRegistry`], [`Namer`]) — changed only through its own methods,
//! so no entity is ever without a name.

use std::{borrow::Cow, sync::Arc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_program::{
    part::Retrieve,
    target::{Revisable, TargetReference},
};
use geop_core_topology::Model;

mod datum;
mod describe;
mod edit;
mod euler;
mod feature;
mod ids;
mod mesh;
mod names;
mod records;
mod resolve;
mod sketch;
mod state;
mod threads;

pub use datum::{datum, datum_name};
pub use describe::{EdgeDescription, FaceDescription, PartDescription};
pub use edit::BodyNames;
pub use feature::{BooleanOp, Feature, FeatureFaces, FeatureTool, ToolReach};
pub use ids::RefId;
pub use mesh::SolidMeshes;
pub use names::{
    LONGEST_ARGUMENT, NameRegistry, Namer, digest, operation_of, origin_of, validate_operation_id,
};
pub use records::Records;
pub use sketch::{PlacedSketch, PlacedSketch3d, sketch, sketch_name, sketch3d, sketch3d_name};
pub use state::{ParamValue, pose_parameter};
pub use threads::{CosmeticThread, Threads, draw_threads, threads};

use crate::{Design, operation::Input};

/// A part's boundary-representation topology and the name of every vertex,
/// edge, face and solid in it: what the steps of a program build, one
/// revision each (see [`model`]).
///
/// The fields are private: the only way to change the topology is through
/// its methods, each of which forwards straight to the identically named
/// [`Model`] operation, registers every entity it created under the name
/// the caller supplied, and forgets the name of every one it deleted. That
/// keeps the invariant [`NamedModel::check_names`] checks — every entity
/// has exactly one name, and every name one entity — true by construction
/// rather than by each caller's diligence.
///
/// Both halves share their structure with the revisions made from them, so
/// a revision costs what it changes, and comparing two revisions what tells
/// them apart.
#[derive(Clone, Debug)]
pub struct NamedModel<S: Scalar> {
    topology: Model<S>,
    names: NameRegistry,
    /// What each step that combined tools with a solid did, in the order
    /// the steps ran (see [`Feature`]).
    features: Vec<(String, Arc<Feature<S>>)>,
}

/// Equal when the topology and the names are, and the features are the very
/// ones recorded: a feature recorded again counts as changed, however alike.
impl<S: Scalar + PartialEq> PartialEq for NamedModel<S> {
    fn eq(&self, other: &Self) -> bool {
        self.topology == other.topology
            && self.names == other.names
            && self.features.len() == other.features.len()
            && (self.features.iter().zip(&other.features))
                .all(|((a, f), (b, g))| a == b && Arc::ptr_eq(f, g))
    }
}

/// The model a part's steps build: defined by the first that builds any
/// topology, revised by every later one.
pub fn model() -> TargetReference<NamedModel<Design>> {
    TargetReference::new("model")
}

/// The model as `part` has it: empty, before the first step that builds
/// any topology — where only what needs none, the origin, resolves.
pub fn model_or_empty<'a>(
    part: &mut impl Retrieve<'a, Input, ParamValue>,
) -> Cow<'a, NamedModel<Design>> {
    match part.retrieve_target(&model()) {
        Ok(model) => Cow::Borrowed(model),
        Err(_) => Cow::Owned(NamedModel::new()),
    }
}

impl<S: Scalar + PartialEq> Revisable for NamedModel<S> {
    fn to_revise(&self) -> Self {
        self.clone()
    }
}

impl<S: Scalar> Default for NamedModel<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Scalar> NamedModel<S> {
    /// A model with nothing in it.
    pub fn new() -> Self {
        Self {
            topology: Model::new(),
            names: NameRegistry::new(),
            features: Vec::new(),
        }
    }

    /// The topology, to query. Changing it goes through the model's own
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

    /// The name of `id` — an error, naming the id, if it has none.
    pub fn name(&self, id: impl Into<RefId>) -> GeopResult<&str> {
        let id = id.into();
        (self.name_of(id)).ok_or_else(|| GeopError::new(format!("{id} has no name")))
    }

    pub fn id_of(&self, name: &str) -> Option<RefId> {
        self.names.id_of(name)
    }

    /// The vertices, edges and faces `text` — a validation report, which
    /// knows only ids — mentions, as `id = name`, sorted.
    pub fn names_mentioned(&self, text: &str) -> Vec<String> {
        let model = &self.topology;
        let mentioned = |forms: [String; 3]| forms.iter().any(|f| text.contains(f.as_str()));
        let mut names = Vec::new();
        let mut name =
            |id: RefId| names.push(format!("{id} = {}", self.name_of(id).unwrap_or("?")));
        for &id in model.vertices.keys() {
            if mentioned([
                format!("{id}"),
                format!("vertex {} ", id.0),
                format!("vertex {}'", id.0),
            ]) {
                name(id.into());
            }
        }
        for &id in model.edges.keys() {
            if mentioned([
                format!("{id}"),
                format!("edge {} ", id.0),
                format!("edge {}'", id.0),
            ]) {
                name(id.into());
            }
        }
        for &id in model.faces.keys() {
            if mentioned([
                format!("{id}"),
                format!("face {} ", id.0),
                format!("face {},", id.0),
            ]) {
                name(id.into());
            }
        }
        names.sort();
        names
    }

    /// Gives `id` the name `new_name` instead — see [`NameRegistry::rename`]
    /// for the one situation this is for.
    pub fn rename(&mut self, id: impl Into<RefId>, new_name: impl Into<String>) -> GeopResult<()> {
        self.names.rename(id, new_name)
    }

    fn exists(&self, id: RefId) -> bool {
        let topology = &self.topology;
        match id {
            RefId::Vertex(id) => topology.vertices.contains_key(&id),
            RefId::Edge(id) => topology.edges.contains_key(&id),
            RefId::Face(id) => topology.faces.contains_key(&id),
            RefId::Solid(id) => topology.solids.contains_key(&id),
        }
    }

    /// Forgets the name of every entity that no longer exists — for an
    /// operation that deletes by reachability rather than one id at a time
    /// (see [`Model::assemble_solid`]).
    pub(crate) fn forget_dead_names(&mut self) {
        let dead: Vec<RefId> = (self.names.iter())
            .map(|(id, _)| id)
            .filter(|&id| !self.exists(id))
            .collect();
        for id in dead {
            self.names.remove(id);
        }
    }

    /// Checks the invariant every method keeps: every vertex, edge, face
    /// and solid has a name, and every name belongs to one of them.
    pub fn check_names(&self) -> GeopResult<()> {
        let topology = &self.topology;
        let entities = (topology.vertices.keys().map(|&id| RefId::from(id)))
            .chain(topology.edges.keys().map(|&id| id.into()))
            .chain(topology.faces.keys().map(|&id| id.into()))
            .chain(topology.solids.keys().map(|&id| id.into()));
        let unnamed: Vec<String> = entities
            .filter(|&id| self.name_of(id).is_none())
            .map(|id| id.to_string())
            .collect();
        let dead: Vec<&str> = (self.names.iter())
            .filter(|&(id, _)| !self.exists(id))
            .map(|(_, name)| name)
            .collect();
        if unnamed.is_empty() && dead.is_empty() {
            Ok(())
        } else {
            Err(GeopError::new(format!(
                "NamedModel::check_names: unnamed entities {unnamed:?}, names of deleted entities {dead:?}"
            )))
        }
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::ScalInF64, vector::Vector3};

    use super::*;

    fn origin() -> Vector3<ScalInF64> {
        Vector3::zero()
    }

    /// `mvfs` creates a vertex, a face and a solid; each must be given a
    /// name, and `kvfs` undoing it erases all three names again.
    #[test]
    fn mvfs_and_kvfs_keep_names_in_sync() {
        let mut model = NamedModel::<ScalInF64>::new();
        let (vertex, face, solid) = model.mvfs(origin(), "v0", "f0", "s0").unwrap();
        assert_eq!(model.name_of(vertex), Some("v0"));
        assert_eq!(model.name_of(face), Some("f0"));
        assert_eq!(model.name_of(solid), Some("s0"));
        assert_eq!(model.id_of("v0"), Some(RefId::Vertex(vertex)));
        model.check_names().unwrap();

        model.kvfs(solid).unwrap();
        assert_eq!(model.name_of(vertex), None);
        assert_eq!(model.name_of(face), None);
        assert_eq!(model.name_of(solid), None);
        assert!(model.topology().vertices.is_empty());
        model.check_names().unwrap();
    }

    /// Reusing a name that's already taken is rejected.
    #[test]
    fn duplicate_name_is_rejected() {
        let mut model = NamedModel::<ScalInF64>::new();
        model.mvfs(origin(), "v0", "f0", "s0").unwrap();
        assert!(model.mvfs(origin(), "v1", "f1", "s0").is_err());
    }

    /// A provisional name can be settled, but not onto a name in use.
    #[test]
    fn rename_settles_a_provisional_name() {
        let mut model = NamedModel::<ScalInF64>::new();
        let (vertex, _, _) = model.mvfs(origin(), "v~0", "f0", "s0").unwrap();
        assert!(model.rename(vertex, "f0").is_err());
        model.rename(vertex, "v0").unwrap();
        assert_eq!(model.name_of(vertex), Some("v0"));
        assert_eq!(model.id_of("v~0"), None);
    }

    /// A revision shares what it leaves untouched, and comparing it with
    /// the model it was made from tells whether it changed.
    #[test]
    fn a_revision_is_equal_until_changed() {
        let mut model = NamedModel::<ScalInF64>::new();
        model.mvfs(origin(), "v0", "f0", "s0").unwrap();
        let mut revision = model.to_revise();
        assert_eq!(revision, model);
        revision.mvfs(origin(), "v1", "f1", "s1").unwrap();
        assert_ne!(revision, model);
    }
}
