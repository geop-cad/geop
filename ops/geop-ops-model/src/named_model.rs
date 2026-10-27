//! [`NamedModel`]: a part's topology with a name for every entity in it
//! ([`NameRegistry`], [`Namer`]) — changed only through its own methods,
//! so no entity is ever without a name.

use std::sync::Arc;

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::ElementReference,
    kept::Keeper,
    target::{Target, TargetReference},
};
use geop_core_topology::Model;

use crate::{Feature, ids::RefId, names::NameRegistry};

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
pub struct NamedModel {
    pub(crate) topology: Model,
    pub(crate) names: NameRegistry,
    /// What each step that combined tools with a solid did, in the order
    /// the steps ran (see [`Feature`]).
    pub(crate) features: Vec<(String, Arc<Feature>)>,
}

/// Every step that builds into the model produces it from the last; it is
/// kept between processes, exactly, its entities shared between revisions
/// ([`crate::kept`]).
impl Target for NamedModel {
    const CHAIN: bool = true;
    const KEEPER: Option<Keeper<Self>> = Some(Keeper {
        write: crate::kept::write,
        read: crate::kept::read,
    });
}

/// Equal when the topology, the names and the features are: a model built
/// again into the same value is the same model, however it was built — what
/// a version rebuilt unchanged, and a part built from scratch, are compared
/// by. A feature shared with the other model is equal without a look.
impl PartialEq for NamedModel {
    fn eq(&self, other: &Self) -> bool {
        self.topology == other.topology
            && self.names == other.names
            && self.features.len() == other.features.len()
            && (self.features.iter().zip(&other.features))
                .all(|((a, f), (b, g))| a == b && (Arc::ptr_eq(f, g) || f == g))
    }
}

/// The model a part's steps build: defined by the first that builds any
/// topology, revised by every later one.
pub fn model() -> TargetReference<NamedModel> {
    TargetReference::new("model")
}

/// The element `name` of the model, of the kind `K`: what a step's
/// arguments refer to it by.
pub fn element<K>(name: impl Into<String>) -> ElementReference<NamedModel, K> {
    ElementReference::new(model(), name)
}

impl Default for NamedModel {
    fn default() -> Self {
        Self::new()
    }
}

impl NamedModel {
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
    pub fn topology(&self) -> &Model {
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

    /// Gives `id` back the name `earlier` of what it turned out to be — a
    /// face a boolean only trimmed, though its splits left it a piece's
    /// name — its name so far standing for it too, so that what referred
    /// to it by that still finds it.
    pub fn restore_name(&mut self, id: impl Into<RefId>, earlier: &str) -> GeopResult<()> {
        let id = id.into();
        let current = self.name(id)?.to_string();
        self.names.rename(id, earlier)?;
        self.names.alias(current, earlier)
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
    use geop_core_math::vector::Vector3;
    use geop_core_topology::{Body, VertexId, build::BodySpec};

    use super::*;
    use crate::{BodyNames, testing::unit_cube};

    /// A free point named `name` at the origin: a wire of the vertex alone.
    fn point(model: &mut NamedModel, name: &str) -> GeopResult<VertexId> {
        let mut spec = BodySpec::new(false);
        spec.vertices.push(Vector3::zero());
        let names = BodyNames {
            vertices: vec![name.to_string()],
            edges: Vec::new(),
            faces: Vec::new(),
            solid: None,
        };
        Ok(model.build_body(spec, names)?.vertices[0])
    }

    /// Building names every entity it builds, and deleting a body forgets
    /// the names of all of its entities again.
    #[test]
    fn building_and_deleting_keep_names_in_sync() {
        let mut model = NamedModel::new();
        let vertex = point(&mut model, "v0").unwrap();
        assert_eq!(model.name_of(vertex), Some("v0"));
        assert_eq!(model.id_of("v0"), Some(RefId::Vertex(vertex)));
        model.check_names().unwrap();

        let mut cube = unit_cube();
        cube.check_names().unwrap();
        let RefId::Solid(solid) = cube.id_of("s0").unwrap() else {
            panic!("s0 names a solid");
        };
        cube.delete_bodies(&[Body::Solid(solid)]).unwrap();
        assert_eq!(cube.id_of("v0"), None);
        assert_eq!(cube.id_of("f0"), None);
        assert_eq!(cube.id_of("s0"), None);
        assert!(cube.topology().vertices.is_empty());
        cube.check_names().unwrap();
    }

    /// Reusing a name that's already taken is rejected.
    #[test]
    fn duplicate_name_is_rejected() {
        let mut model = NamedModel::new();
        point(&mut model, "v0").unwrap();
        assert!(point(&mut model, "v0").is_err());
    }

    /// A provisional name can be settled, but not onto a name in use.
    #[test]
    fn rename_settles_a_provisional_name() {
        let mut model = NamedModel::new();
        let vertex = point(&mut model, "v~0").unwrap();
        point(&mut model, "v1").unwrap();
        assert!(model.rename(vertex, "v1").is_err());
        model.rename(vertex, "v0").unwrap();
        assert_eq!(model.name_of(vertex), Some("v0"));
        assert_eq!(model.id_of("v~0"), None);
    }

    /// A revision shares what it leaves untouched, and comparing it with
    /// the model it was made from tells whether it changed.
    #[test]
    fn a_revision_is_equal_until_changed() {
        let mut model = NamedModel::new();
        point(&mut model, "v0").unwrap();
        let mut revision = model.clone();
        assert_eq!(revision, model);
        point(&mut revision, "v1").unwrap();
        assert_ne!(revision, model);
    }
}
