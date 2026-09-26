use std::collections::HashSet;

use crate::{
    CoedgeGeometry, CoedgeId, EdgeId, FaceId, Model, Shell, Solid, SolidId, VertexId,
    boundary::BoundaryType,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    /// Replace the solids `consumed` by one new solid of a single shell made
    /// of `keep`, which must be faces of those solids. Every other face they
    /// owned is deleted, and so is everything no face reaches any more.
    ///
    /// What a boolean does last, once it has decided which faces survive.
    /// `Ok(None)` (and no solid) when `keep` is empty: the result is empty,
    /// which is an answer rather than a failure.
    ///
    /// Deletion is by reachability rather than by tracking what was split or
    /// re-homed on the way here, so no dangling id can be left behind — and
    /// it only ever deletes from what `consumed` owned: faces of any other
    /// solid in the model are none of this operation's business.
    pub fn assemble_solid(
        &mut self,
        consumed: &[SolidId],
        keep: &[FaceId],
    ) -> GeopResult<Option<SolidId>> {
        let mut consumed_faces: HashSet<FaceId> = HashSet::new();
        for &solid in consumed {
            consumed_faces.extend(self.solid_faces(solid)?);
        }
        if let Some(face) = keep.iter().find(|f| !consumed_faces.contains(f)) {
            return Err(GeopError::new(format!(
                "Model::assemble_solid: face {face} is to be kept, but belongs to none of the consumed solids {consumed:?}"
            )));
        }
        for &solid in consumed {
            for shell in self.get_solid(solid)?.shells.clone() {
                self.shells.remove(&shell);
            }
            self.solids.remove(&solid);
        }

        let kept: HashSet<FaceId> = keep.iter().copied().collect();
        self.faces
            .retain(|id, _| !consumed_faces.contains(id) || kept.contains(id));

        if keep.is_empty() {
            self.prune_unreachable();
            return Ok(None);
        }

        let shell_id = self.insert_shell(Shell {
            faces: keep.to_vec(),
            solid: SolidId(0),
        });
        let solid_id = self.insert_solid(Solid {
            shells: vec![shell_id],
        });
        self.get_shell_mut(shell_id)?.solid = solid_id;
        for &face_id in keep {
            self.get_face_mut(face_id)?.shell = shell_id;
        }

        self.prune_unreachable();
        Ok(Some(solid_id))
    }

    /// Move every shell of `from` into `into`, deleting `from`: one solid of
    /// both bodies. Only valid for bodies that don't touch — nothing is
    /// intersected, so two overlapping ones would make a solid whose shells
    /// cross; combining those is a boolean union's job.
    pub fn merge_solids(&mut self, into: SolidId, from: SolidId) -> GeopResult<()> {
        if into == from {
            return Err(GeopError::new(format!(
                "Model::merge_solids: refusing to merge solid {into} into itself"
            )));
        }
        self.get_solid(into)?;
        let shells = self.get_solid(from)?.shells.clone();
        for &shell in &shells {
            self.get_shell_mut(shell)?.solid = into;
        }
        self.solids.remove(&from);
        self.get_solid_mut(into)?.shells.extend(shells);
        Ok(())
    }

    /// Drop everything no longer reachable from a face: the coedges no face
    /// owns, then edges and vertices nothing refers to any more.
    fn prune_unreachable(&mut self) {
        let live_faces: Vec<FaceId> = self.faces.keys().copied().collect();
        let live_coedges: HashSet<CoedgeId> = live_faces
            .iter()
            .flat_map(|&f| self.iterate_face_coedges(f).collect::<Vec<_>>())
            .collect();
        self.coedges.retain(|id, _| live_coedges.contains(id));

        let live_edges: HashSet<EdgeId> = self
            .coedges
            .values()
            .filter_map(|c| c.edge().ok())
            .collect();
        self.edges.retain(|id, _| live_edges.contains(id));

        let mut live_vertices: HashSet<VertexId> = self
            .edges
            .values()
            .flat_map(|e| [e.start_vertex, e.end_vertex])
            .collect();
        for coedge in self.coedges.values() {
            if let CoedgeGeometry::Vertex(v) = coedge.geometry {
                live_vertices.insert(v);
            }
        }
        for face in self.faces.values() {
            for boundary in face.boundaries() {
                if let BoundaryType::Vertex(v) = boundary {
                    live_vertices.insert(v);
                }
            }
        }
        self.vertices.retain(|id, _| live_vertices.contains(id));
    }
}
