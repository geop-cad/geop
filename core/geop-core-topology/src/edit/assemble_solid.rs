use std::collections::HashSet;

use crate::{
    Body, CoedgeGeometry, CoedgeId, EdgeId, FaceId, Model, Shell, ShellId, Solid, SolidId,
    VertexId, boundary::BoundaryType,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    /// Replace the bodies `consumed` by one new solid of a single shell made
    /// of `keep`, which must be faces of those bodies. Every other face they
    /// owned is deleted, and so is everything no face reaches any more.
    ///
    /// What a boolean does last, once it has decided which faces survive.
    /// `Ok(None)` (and no solid) when `keep` is empty: the result is empty,
    /// which is an answer rather than a failure — and also how a body is
    /// deleted outright.
    ///
    /// Deletion is by reachability rather than by tracking what was split or
    /// re-homed on the way here, so no dangling id can be left behind — and
    /// it only ever deletes from what `consumed` owned: faces of any other
    /// body in the model are none of this operation's business.
    pub fn assemble_solid(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
    ) -> GeopResult<Option<SolidId>> {
        let Some(shell_id) = self.assemble_shell(consumed, keep)? else {
            return Ok(None);
        };
        let solid_id = self.insert_solid(Solid {
            shells: vec![shell_id],
        });
        self.get_shell_mut(shell_id)?.solid = Some(solid_id);
        Ok(Some(solid_id))
    }

    /// Like [`Model::assemble_solid`], but `keep` becomes a sheet — faces
    /// standing on their own — rather than a solid: what is left of a sheet
    /// once it is trimmed.
    pub fn assemble_sheet(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
    ) -> GeopResult<Option<ShellId>> {
        self.assemble_shell(consumed, keep)
    }

    /// [`Model::assemble_solid`]'s work, short of making the shell of `keep`
    /// a solid's.
    fn assemble_shell(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
    ) -> GeopResult<Option<ShellId>> {
        let mut consumed_faces: HashSet<FaceId> = HashSet::new();
        for &body in consumed {
            consumed_faces.extend(self.body_faces(body)?);
        }
        if let Some(face) = keep.iter().find(|f| !consumed_faces.contains(f)) {
            return Err(GeopError::new(format!(
                "Model::assemble_solid: face {face} is to be kept, but belongs to none of the consumed bodies {consumed:?}"
            )));
        }
        for &body in consumed {
            match body {
                Body::Solid(solid) => {
                    for shell in self.get_solid(solid)?.shells.clone() {
                        self.shells.remove(&shell);
                    }
                    self.solids.remove(&solid);
                }
                Body::Sheet(shell) => {
                    self.shells.remove(&shell);
                }
            }
        }

        let kept: HashSet<FaceId> = keep.iter().copied().collect();
        self.faces
            .retain(|id, _| !consumed_faces.contains(id) || kept.contains(id));

        let shell = (!keep.is_empty()).then(|| {
            self.insert_shell(Shell {
                faces: keep.to_vec(),
                solid: None,
            })
        });
        if let Some(shell_id) = shell {
            for &face_id in keep {
                self.get_face_mut(face_id)?.shell = shell_id;
            }
        }
        self.prune_unreachable();
        Ok(shell)
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
            self.get_shell_mut(shell)?.solid = Some(into);
        }
        self.solids.remove(&from);
        self.get_solid_mut(into)?.shells.extend(shells);
        Ok(())
    }

    /// Drop everything no longer reachable from a face or a wire: the
    /// coedges no face owns, then edges and vertices nothing refers to any
    /// more.
    pub(crate) fn prune_unreachable(&mut self) {
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
            .chain(self.wires.values().flat_map(|w| w.edges.iter().copied()))
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
        for wire in self.wires.values() {
            live_vertices.extend(wire.vertices.iter().copied());
        }
        self.vertices.retain(|id, _| live_vertices.contains(id));
    }
}
