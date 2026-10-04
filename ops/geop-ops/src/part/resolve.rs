//! Looking an entity up by its name instead of its id — what every
//! operation that refers to existing entities by name is built on.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_topology::{CoedgeId, EdgeId, FaceId, Sense, SolidId, VertexId};

use super::Part;
use super::ids::{DatumId, InstanceId, RefId, SketchId};

impl<S: Scalar> Part<S> {
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
                .names
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

    pub fn sketch_id(&self, name: &str) -> GeopResult<SketchId> {
        self.named(name, "sketch", |r| match r {
            RefId::Sketch(id) => Some(id),
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

    /// Every sketch's name, oldest first.
    pub fn sketch_names(&self) -> Vec<String> {
        self.sketches()
            .filter_map(|(id, _)| self.name_of(id).map(str::to_string))
            .collect()
    }

    pub fn datum_id(&self, name: &str) -> GeopResult<DatumId> {
        self.named(name, "datum", |r| match r {
            RefId::Datum(id) => Some(id),
            _ => None,
        })
    }

    pub fn instance_id(&self, name: &str) -> GeopResult<InstanceId> {
        self.named(name, "placed part", |r| match r {
            RefId::Instance(id) => Some(id),
            _ => None,
        })
    }

    /// The unique coedge of `edge_name` lying on `face_name`. An edge shared
    /// by two distinct faces has exactly one coedge per face, so this pair
    /// identifies it unambiguously — except for a connector edge
    /// [`geop_core_topology::Model::mve`] or
    /// [`geop_core_topology::Model::mekr`] mints, both of whose coedges sit
    /// on the very same face; resolve one of those with
    /// [`Part::coedge_id_with_sense`] instead.
    pub fn coedge_id(&self, edge_name: &str, face_name: &str) -> GeopResult<CoedgeId> {
        let edge = self.edge_id(edge_name)?;
        let face = self.face_id(face_name)?;
        let mut on_face = self
            .topology
            .coedges_of_edge(edge)
            .into_iter()
            .filter(|&c| {
                self.topology
                    .get_coedge(c)
                    .map(|co| co.face == face)
                    .unwrap_or(false)
            });
        let found = on_face.next().ok_or_else(|| {
            GeopError::new(format!(
                "edge {edge_name:?} has no coedge on face {face_name:?}"
            ))
        })?;
        if on_face.next().is_some() {
            return Err(GeopError::new(format!(
                "edge {edge_name:?} has more than one coedge on face {face_name:?} — use coedge_id_with_sense to disambiguate"
            )));
        }
        Ok(found)
    }

    /// The coedge of `edge_name` on `face_name` with sense `sense` — needed
    /// only for a connector edge whose two coedges both sit on one face (see
    /// [`Part::coedge_id`]'s own doc comment), where the plain `(edge, face)`
    /// pair is ambiguous.
    pub fn coedge_id_with_sense(
        &self,
        edge_name: &str,
        face_name: &str,
        sense: Sense,
    ) -> GeopResult<CoedgeId> {
        let edge = self.edge_id(edge_name)?;
        let face = self.face_id(face_name)?;
        self.topology
            .coedges_of_edge(edge)
            .into_iter()
            .find(|&c| {
                self.topology
                    .get_coedge(c)
                    .map(|co| co.face == face && co.sense == sense)
                    .unwrap_or(false)
            })
            .ok_or_else(|| {
                GeopError::new(format!(
                    "edge {edge_name:?} has no {sense:?} coedge on face {face_name:?}"
                ))
            })
    }
}
