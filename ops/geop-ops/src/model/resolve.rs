//! Looking an entity up by its name instead of its id — what every
//! operation that refers to existing entities by name is built on.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_topology::{EdgeId, FaceId, SolidId, VertexId};

use super::NamedModel;
use super::ids::RefId;

impl<S: Scalar> NamedModel<S> {
    /// `name`'s id, checked to be the particular kind `extract` accepts.
    fn named<T>(
        &self,
        name: &str,
        kind: &str,
        extract: impl Fn(RefId) -> Option<T>,
    ) -> GeopResult<T> {
        let id = self.names.id_of(name).ok_or_else(|| {
            // The names of the same kind that the same step made: usually
            // what was meant, if a later step renamed or split it.
            let step = name.split_once(',').map_or(name, |(head, _)| head);
            let step = step.split_once('(').map_or(step, |(_, id)| id);
            let mut similar: Vec<&str> = self
                .names()
                .iter()
                .map(|(_, n)| n)
                .filter(|n| n.contains(step))
                .collect();
            similar.sort_unstable();
            similar.truncate(12);
            GeopError::new(format!(
                "no entity is named {name:?} (names of {step:?}: {similar:?})"
            ))
        })?;
        extract(id).ok_or_else(|| GeopError::new(format!("{name:?} names {id}, not a {kind}")))
    }

    pub fn vertex_id(&self, name: &str) -> GeopResult<VertexId> {
        self.named(name, "vertex", |r| match r {
            RefId::Vertex(id) => Some(id),
            _ => None,
        })
    }

    pub fn edge_id(&self, name: &str) -> GeopResult<EdgeId> {
        self.named(name, "edge", |r| match r {
            RefId::Edge(id) => Some(id),
            _ => None,
        })
    }

    pub fn face_id(&self, name: &str) -> GeopResult<FaceId> {
        self.named(name, "face", |r| match r {
            RefId::Face(id) => Some(id),
            _ => None,
        })
    }

    pub fn solid_id(&self, name: &str) -> GeopResult<SolidId> {
        self.named(name, "solid", |r| match r {
            RefId::Solid(id) => Some(id),
            _ => None,
        })
    }

    /// Every solid's name, oldest first.
    pub fn solid_names(&self) -> Vec<String> {
        let mut solids: Vec<SolidId> = self.topology().solids.keys().copied().collect();
        solids.sort_by_key(|s| s.0);
        solids
            .into_iter()
            .filter_map(|s| self.name_of(s).map(str::to_string))
            .collect()
    }

    /// The name of every face standing on its own — of a sheet, part of no
    /// solid — oldest first.
    pub fn sheet_face_names(&self) -> Vec<String> {
        let model = self.topology();
        let mut faces: Vec<FaceId> = model
            .faces
            .iter()
            .filter(|(_, f)| {
                model
                    .shells
                    .get(&f.shell)
                    .is_some_and(|s| s.solid.is_none())
            })
            .map(|(&id, _)| id)
            .collect();
        faces.sort_by_key(|f| f.0);
        faces
            .into_iter()
            .filter_map(|f| self.name_of(f).map(str::to_string))
            .collect()
    }
}
