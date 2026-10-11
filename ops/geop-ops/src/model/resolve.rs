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

    /// The name of the solid `face` is part of. An error for a face of a
    /// sheet, naming the face and saying `why` it has to be a solid's.
    pub fn solid_name_of_face(&self, face: FaceId, why: &str) -> GeopResult<String> {
        let Some(solid) = self.topology().solid_of_face(face)? else {
            return Err(GeopError::new(format!(
                "face {:?} stands on its own: {why}",
                self.name(face)?
            )));
        };
        Ok(self.name(solid)?.to_string())
    }

    /// Every solid's name, oldest first.
    pub fn solid_names(&self) -> Vec<String> {
        // The arenas are ordered by id, the order of creation.
        self.topology()
            .solids
            .keys()
            .filter_map(|&s| self.name_of(s).map(str::to_string))
            .collect()
    }

    /// The newest solid's name, if there is any: what a field picking a
    /// solid starts out holding.
    pub fn newest_solid(&self) -> Option<String> {
        self.solid_names().pop()
    }

    /// The name of every face standing on its own — of a sheet, part of no
    /// solid — oldest first.
    pub fn sheet_face_names(&self) -> Vec<String> {
        let model = self.topology();
        model
            .faces
            .iter()
            .filter(|(_, f)| {
                model
                    .shells
                    .get(&f.shell)
                    .is_some_and(|s| s.solid.is_none())
            })
            .filter_map(|(&f, _)| self.name_of(f).map(str::to_string))
            .collect()
    }
}
