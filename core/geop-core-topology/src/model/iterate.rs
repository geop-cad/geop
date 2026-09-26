use crate::{
    CoedgeGeometry, CoedgeId, EdgeId, FaceId, SolidId, VertexId,
    boundary::{BoundaryIndex, BoundaryType},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use super::Model;

/// Walks a single `Loop` boundary, in traversal order, by following `next`
/// from `anchor` back around to itself. Does not itself bound how many
/// coedges it will yield — a corrupted `next` chain that never returns to
/// `anchor` makes this iterate forever, which is exactly what callers
/// checking that invariant (see `validation::two_way_references`) want to be
/// able to detect via a `.take(n)` of their own; well-formed loops always
/// terminate on their own.
struct LoopCoedges<'a, S: Scalar> {
    model: &'a Model<S>,
    anchor: CoedgeId,
    cursor: Option<CoedgeId>,
}

impl<'a, S: Scalar> Iterator for LoopCoedges<'a, S> {
    type Item = CoedgeId;

    fn next(&mut self) -> Option<CoedgeId> {
        let current = self.cursor?;
        let next = self.model.coedges[&current].next;
        self.cursor = if next == self.anchor {
            None
        } else {
            Some(next)
        };
        Some(current)
    }
}

/// Walks every coedge of every `Loop` boundary of a face, across all of them
/// in order — a bare `Vertex` boundary contributes nothing (there are no
/// coedges yet).
struct FaceCoedges<'a, S: Scalar> {
    model: &'a Model<S>,
    boundaries: std::vec::IntoIter<BoundaryType>,
    current: Option<LoopCoedges<'a, S>>,
}

impl<'a, S: Scalar> Iterator for FaceCoedges<'a, S> {
    type Item = CoedgeId;

    fn next(&mut self) -> Option<CoedgeId> {
        loop {
            if let Some(loop_iter) = &mut self.current {
                if let Some(id) = loop_iter.next() {
                    return Some(id);
                }
                self.current = None;
            }
            let boundary = self.boundaries.next()?;
            if let BoundaryType::Loop(anchor) = boundary {
                self.current = Some(LoopCoedges {
                    model: self.model,
                    anchor,
                    cursor: Some(anchor),
                });
            }
        }
    }
}

/// Walks the distinct vertices referenced by a solid's faces, borrowing
/// `model` for its whole lifetime — so a caller holding one of these live
/// cannot also call a `&mut self` method like `merge_vertex` on the same
/// model; the borrow checker enforces it rather than relying on a caller to
/// remember that a `Vec` snapshot goes stale the moment the model mutates.
struct SolidVertices<'a, S: Scalar> {
    model: &'a Model<S>,
    faces: std::vec::IntoIter<FaceId>,
    coedges: std::vec::IntoIter<CoedgeId>,
    seen: std::collections::HashSet<VertexId>,
    pending: std::collections::VecDeque<VertexId>,
}

impl<'a, S: Scalar> Iterator for SolidVertices<'a, S> {
    type Item = VertexId;

    fn next(&mut self) -> Option<VertexId> {
        loop {
            if let Some(vertex_id) = self.pending.pop_front() {
                if self.seen.insert(vertex_id) {
                    return Some(vertex_id);
                }
                continue;
            }

            let coedge_id = match self.coedges.next() {
                Some(coedge_id) => coedge_id,
                None => {
                    let face_id = self.faces.next()?;
                    self.coedges = self
                        .model
                        .iterate_face_coedges(face_id)
                        .collect::<Vec<_>>()
                        .into_iter();
                    continue;
                }
            };

            match self.model.coedges[&coedge_id].geometry {
                CoedgeGeometry::Edge(edge_id) => {
                    let edge = &self.model.edges[&edge_id];
                    self.pending.push_back(edge.start_vertex);
                    self.pending.push_back(edge.end_vertex);
                }
                CoedgeGeometry::Vertex(vertex_id) => {
                    self.pending.push_back(vertex_id);
                }
            }
        }
    }
}

/// Walks the distinct edges referenced by a solid's faces — see
/// [`SolidVertices`]'s own doc comment for why this borrows `model` instead
/// of returning an owned snapshot.
struct SolidEdges<'a, S: Scalar> {
    model: &'a Model<S>,
    faces: std::vec::IntoIter<FaceId>,
    coedges: std::vec::IntoIter<CoedgeId>,
    seen: std::collections::HashSet<EdgeId>,
}

impl<'a, S: Scalar> Iterator for SolidEdges<'a, S> {
    type Item = EdgeId;

    fn next(&mut self) -> Option<EdgeId> {
        loop {
            let coedge_id = match self.coedges.next() {
                Some(coedge_id) => coedge_id,
                None => {
                    let face_id = self.faces.next()?;
                    self.coedges = self
                        .model
                        .iterate_face_coedges(face_id)
                        .collect::<Vec<_>>()
                        .into_iter();
                    continue;
                }
            };

            if let CoedgeGeometry::Edge(edge_id) = self.model.coedges[&coedge_id].geometry {
                if self.seen.insert(edge_id) {
                    return Some(edge_id);
                }
            }
        }
    }
}

impl<S: Scalar> Model<S> {
    /// The (unordered) coedges referencing a given edge. An edge shared by a
    /// single manifold face pair has exactly two.
    pub fn coedges_of_edge(&self, edge: EdgeId) -> Vec<CoedgeId> {
        self.coedges
            .iter()
            .filter(|(_, c)| c.geometry == CoedgeGeometry::Edge(edge))
            .map(|(id, _)| *id)
            .collect()
    }

    /// Every coedge of the loop anchored at `anchor`, in traversal order.
    pub fn iterate_loop_coedges(&self, anchor: CoedgeId) -> impl Iterator<Item = CoedgeId> + '_ {
        LoopCoedges {
            model: self,
            anchor,
            cursor: Some(anchor),
        }
    }

    /// Every coedge of every boundary loop of `face_id` (its outer loop and
    /// any holes).
    pub fn iterate_face_coedges(&self, face_id: FaceId) -> impl Iterator<Item = CoedgeId> + '_ {
        FaceCoedges {
            model: self,
            boundaries: self.faces[&face_id]
                .boundaries()
                .collect::<Vec<_>>()
                .into_iter(),
            current: None,
        }
    }

    /// The distinct vertices referenced by `solid_id`'s faces, as an
    /// iterator borrowing `self` — see [`SolidVertices`]'s own doc comment.
    pub fn iter_solid_vertices(
        &self,
        solid_id: SolidId,
    ) -> GeopResult<impl Iterator<Item = VertexId> + '_> {
        let faces = self.solid_faces(solid_id)?;
        Ok(SolidVertices {
            model: self,
            faces: faces.into_iter(),
            coedges: Vec::new().into_iter(),
            seen: std::collections::HashSet::new(),
            pending: std::collections::VecDeque::new(),
        })
    }

    /// The distinct edges referenced by `solid_id`'s faces, as an iterator
    /// borrowing `self` — see [`SolidVertices`]'s own doc comment.
    pub fn iter_solid_edges(
        &self,
        solid_id: SolidId,
    ) -> GeopResult<impl Iterator<Item = EdgeId> + '_> {
        let faces = self.solid_faces(solid_id)?;
        Ok(SolidEdges {
            model: self,
            faces: faces.into_iter(),
            coedges: Vec::new().into_iter(),
            seen: std::collections::HashSet::new(),
        })
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
    /// loop is deleting the face, and must say so (see `kef`) rather than
    /// leaving one behind with nothing bounding it.
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
        let solid = self.get_solid(solid_id)?;
        let mut faces = Vec::new();
        for &shell_id in &solid.shells {
            faces.extend(self.get_shell(shell_id)?.faces.iter().copied());
        }
        Ok(faces)
    }
}
