//! Looking an entity up by its name instead of its id — what every
//! operation that refers to existing entities by name is built on.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_topology::{CoedgeId, EdgeId, FaceId, Sense, SolidId, VertexId};

use crate::ids::{DatumId, RefId, SketchId};
use crate::part::Part;

impl<S: Scalar> Part<S> {
    /// `name`'s id, checked to be the particular kind `extract` accepts.
    fn named<T>(
        &self,
        name: &str,
        kind: &str,
        extract: impl Fn(RefId) -> Option<T>,
    ) -> GeopResult<T> {
        let id = self
            .names
            .id_of(name)
            .ok_or_else(|| GeopError::new(format!("no entity is named {name:?}")))?;
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

    pub fn datum_id(&self, name: &str) -> GeopResult<DatumId> {
        self.named(name, "datum", |r| match r {
            RefId::Datum(id) => Some(id),
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
