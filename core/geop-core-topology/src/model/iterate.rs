use std::collections::HashSet;

use crate::{
    Body, CoedgeGeometry, CoedgeId, EdgeId, FaceId, Sense, SolidId, VertexId,
    boundary::{BoundaryIndex, BoundaryType},
};
use geop_core_math::geop_error::{GeopError, GeopResult};

use super::Model;

impl Model {
    /// The (unordered) coedges referencing a given edge. An edge shared by a
    /// single manifold face pair has exactly two.
    pub fn coedges_of_edge(&self, edge: EdgeId) -> Vec<CoedgeId> {
        self.coedges
            .iter()
            .filter(|(_, c)| c.geometry == CoedgeGeometry::Edge(edge))
            .map(|(id, _)| *id)
            .collect()
    }

    /// Every coedge of the loop anchored at `anchor`, in traversal order,
    /// following `next` back around to `anchor`. Does not itself bound how
    /// many coedges it yields: a corrupted `next` chain that never returns
    /// to `anchor` makes this iterate forever, which callers checking that
    /// invariant (see `validation::two_way_references`) detect with a
    /// `.take(n)` of their own.
    pub fn iterate_loop_coedges(&self, anchor: CoedgeId) -> impl Iterator<Item = CoedgeId> + '_ {
        std::iter::successors(Some(anchor), move |c| {
            let next = self.coedges[c].next;
            (next != anchor).then_some(next)
        })
    }

    /// The corners of `face_id`'s boundaries, outer loop first, each its
    /// vertex and where it is — what an error shows of a face, to tell which
    /// curves were imprinted into it, by vertices a named model can name
    /// (`names_mentioned`).
    pub fn face_corners(&self, face_id: FaceId) -> GeopResult<Vec<Vec<(VertexId, [f64; 3])>>> {
        let at = |v: VertexId| -> GeopResult<(VertexId, [f64; 3])> {
            let p = self.get_vertex(v)?.point;
            Ok((v, p.to_f64_array()))
        };
        self.get_face(face_id)?
            .boundaries()
            .map(|boundary| match boundary {
                BoundaryType::Vertex(v) => Ok(vec![at(v)?]),
                BoundaryType::Loop(anchor) => self
                    .iterate_loop_coedges(anchor)
                    .map(|c| at(self.coedge_start_vertex_id(c)?))
                    .collect(),
            })
            .collect()
    }

    /// Every coedge of every boundary loop of `face_id` (its outer loop and
    /// any holes) — a bare `Vertex` boundary contributes none.
    pub fn iterate_face_coedges(&self, face_id: FaceId) -> impl Iterator<Item = CoedgeId> + '_ {
        self.faces[&face_id]
            .boundaries()
            .filter_map(|boundary| match boundary {
                BoundaryType::Loop(anchor) => Some(anchor),
                BoundaryType::Vertex(_) => None,
            })
            .flat_map(|anchor| self.iterate_loop_coedges(anchor))
    }

    /// The edge of every edge-backed coedge of `face_id`'s boundaries, in
    /// loop order, outer loop first: a seam the face runs along twice comes
    /// twice.
    pub fn face_edges(&self, face_id: FaceId) -> impl Iterator<Item = EdgeId> + '_ {
        self.iterate_face_coedges(face_id)
            .filter_map(|c| match self.coedges[&c].geometry {
                CoedgeGeometry::Edge(edge) => Some(edge),
                CoedgeGeometry::Vertex(_) => None,
            })
    }

    /// The edges ending at `vertex`, sorted by id.
    pub fn edges_at_vertex(&self, vertex: VertexId) -> Vec<EdgeId> {
        self.edges
            .iter()
            .filter(|(_, e)| e.start_vertex == vertex || e.end_vertex == vertex)
            .map(|(&id, _)| id)
            .collect()
    }

    /// The faces whose boundaries run through `vertex` (along an edge
    /// starting there), sorted by id.
    pub fn faces_at_vertex(&self, vertex: VertexId) -> GeopResult<Vec<FaceId>> {
        let mut faces = Vec::new();
        for (&id, coedge) in &self.coedges {
            if coedge.edge().is_ok() && self.coedge_start_vertex_id(id)? == vertex {
                faces.push(coedge.face);
            }
        }
        faces.sort();
        faces.dedup();
        Ok(faces)
    }

    /// The two faces of `edge`: the one its forward coedge bounds (on the
    /// edge's left) first, the one running against it second. `None`
    /// unless the edge joins two faces.
    pub fn faces_of_edge(&self, edge: EdgeId) -> Option<[FaceId; 2]> {
        let mut forward = None;
        let mut reversed = None;
        for c in self.coedges_of_edge(edge) {
            let coedge = &self.coedges[&c];
            match coedge.sense {
                Sense::Forward => forward = Some(coedge.face),
                Sense::Reversed => reversed = Some(coedge.face),
            }
        }
        Some([forward?, reversed?])
    }

    /// The geometry of every coedge of `body`'s faces, in order.
    fn body_coedge_geometry(
        &self,
        body: impl Into<Body>,
    ) -> GeopResult<impl Iterator<Item = CoedgeGeometry> + '_> {
        Ok(self
            .body_faces(body)?
            .into_iter()
            .flat_map(|face| self.iterate_face_coedges(face))
            .map(|c| self.coedges[&c].geometry))
    }

    /// The distinct vertices referenced by `body`'s faces. Borrows `self`
    /// for as long as it lives, so a caller holding one cannot also call a
    /// `&mut self` method like `merge_vertex` on the same model: the borrow
    /// checker, not the caller, keeps it from going stale.
    pub fn iter_body_vertices(
        &self,
        body: impl Into<Body>,
    ) -> GeopResult<impl Iterator<Item = VertexId> + '_> {
        let mut seen = HashSet::new();
        Ok(self
            .body_coedge_geometry(body)?
            .flat_map(|geometry| match geometry {
                CoedgeGeometry::Edge(edge_id) => {
                    let edge = &self.edges[&edge_id];
                    vec![edge.start_vertex, edge.end_vertex]
                }
                CoedgeGeometry::Vertex(vertex_id) => vec![vertex_id],
            })
            .filter(move |&v| seen.insert(v)))
    }

    /// The distinct edges referenced by `body`'s faces, borrowing `self`
    /// like [`Self::iter_body_vertices`].
    pub fn iter_body_edges(
        &self,
        body: impl Into<Body>,
    ) -> GeopResult<impl Iterator<Item = EdgeId> + '_> {
        let mut seen = HashSet::new();
        Ok(self
            .body_coedge_geometry(body)?
            .filter_map(|geometry| match geometry {
                CoedgeGeometry::Edge(edge_id) => Some(edge_id),
                CoedgeGeometry::Vertex(_) => None,
            })
            .filter(move |&e| seen.insert(e)))
    }

    /// Which of `face_id`'s boundaries contains `coedge` — its outer loop or
    /// one of its holes — found by traversing each `Loop` boundary's `.next`
    /// chain. A face's boundaries are disjoint loops, so at most one can
    /// contain any given coedge.
    ///
    /// Returns [`BoundaryIndex`] rather than a bare position, because callers
    /// invariably need to know whether they found the outer loop: joining two
    /// holes, joining a hole to the outer loop, and joining two points of the
    /// outer loop are three different restructurings (see
    /// `Model::splice_edge_into_face`).
    pub fn find_boundary_containing(
        &self,
        face_id: FaceId,
        coedge: CoedgeId,
    ) -> GeopResult<BoundaryIndex> {
        let face = &self.faces[&face_id];
        let contains = |boundary: BoundaryType| match boundary {
            BoundaryType::Loop(anchor) => self.iterate_loop_coedges(anchor).any(|c| c == coedge),
            BoundaryType::Vertex(_) => false,
        };
        if contains(face.outer) {
            return Ok(BoundaryIndex::Outer);
        }
        face.holes
            .iter()
            .position(|&h| contains(h))
            .map(BoundaryIndex::Hole)
            .ok_or_else(|| {
                GeopError::new(format!(
                    "Model::find_boundary_containing: could not find a boundary containing coedge={coedge} on face={face_id}"
                ))
            })
    }

    /// Drop the hole `index` names from `face_id`.
    ///
    /// Errors on [`BoundaryIndex::Outer`]: a face without an outer boundary
    /// is not a face. An operation that genuinely consumes a face's outer
    /// loop is deleting the face, and must say so rather than leaving one
    /// behind with nothing bounding it.
    pub fn remove_boundary(&mut self, face_id: FaceId, index: BoundaryIndex) -> GeopResult<()> {
        let BoundaryIndex::Hole(i) = index else {
            return Err(GeopError::new(format!(
                "Model::remove_boundary: refusing to remove face={face_id}'s outer boundary — a face must always have exactly one"
            )));
        };
        let face = self.get_face_mut(face_id)?;
        if i >= face.holes.len() {
            return Err(GeopError::new(format!(
                "Model::remove_boundary: face={face_id} has no hole at index {i}"
            )));
        }
        face.holes.remove(i);
        Ok(())
    }

    /// Every `FaceId` across every shell of `solid_id`.
    pub fn solid_faces(&self, solid_id: SolidId) -> GeopResult<Vec<FaceId>> {
        self.body_faces(solid_id)
    }

    /// Every `FaceId` of `body`: across every shell of a solid, or of the
    /// one shell a sheet is.
    pub fn body_faces(&self, body: impl Into<Body>) -> GeopResult<Vec<FaceId>> {
        let shells = match body.into() {
            Body::Solid(solid) => self.get_solid(solid)?.shells.clone(),
            Body::Sheet(shell) => vec![shell],
        };
        let mut faces = Vec::new();
        for shell_id in shells {
            faces.extend(self.get_shell(shell_id)?.faces.iter().copied());
        }
        Ok(faces)
    }

    /// The solid `face_id` is part of; `None` for a face of a sheet.
    pub fn solid_of_face(&self, face_id: FaceId) -> GeopResult<Option<SolidId>> {
        Ok(match self.body_of_face(face_id)? {
            Body::Solid(solid) => Some(solid),
            Body::Sheet(_) => None,
        })
    }

    /// The body `face_id` is part of: its shell's solid, or its shell, if
    /// that is a sheet.
    pub fn body_of_face(&self, face_id: FaceId) -> GeopResult<Body> {
        let shell_id = self.get_face(face_id)?.shell;
        Ok(match self.get_shell(shell_id)?.solid {
            Some(solid) => Body::Solid(solid),
            None => Body::Sheet(shell_id),
        })
    }
}
