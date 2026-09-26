use crate::{
    CoedgeId, Model,
    boundary::{BoundaryIndex, BoundaryType},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Kill the edge added by `mer`, undoing it: splices the ring `mer` moved
    // onto its `existing_face_id` back into the ring it split off from,
    // removing the boundary entry `mer` added there (unlike `kef`, the face
    // itself is not deleted — `mer` never created it, it may have other
    // boundaries of its own). `coedge_backward` and `coedge_forward` must be
    // exactly the pair `mer` itself returned (the closing edge's two
    // coedges, on the original face and on `existing_face_id` respectively).
    pub fn ker(
        self: &mut Model<S>,
        coedge_backward: CoedgeId,
        coedge_forward: CoedgeId,
    ) -> GeopResult<()> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::ker(coedge_backward={coedge_backward}, coedge_forward={coedge_forward})"
            ))
        };

        let cb = self.get_coedge(coedge_backward)?.clone();
        let ca = self.get_coedge(coedge_forward)?.clone();
        if ca.edge().with_context(&ctx)? != cb.edge().with_context(&ctx)? {
            return Err(ctx(GeopError::new(
                "coedge_backward and coedge_forward must belong to the same edge",
            )));
        }
        if ca.face == cb.face {
            return Err(ctx(GeopError::new(
                "coedge_backward and coedge_forward must belong to different faces (use kemr to merge two rings of the same face)",
            )));
        }

        let moved_face_id = ca.face;
        let survivor_face_id = cb.face;
        let remove_index = self
            .find_boundary_containing(moved_face_id, coedge_forward)
            .with_context(&ctx)?;
        let keep_index = self
            .find_boundary_containing(survivor_face_id, coedge_backward)
            .with_context(&ctx)?;

        // Captured before the splice removes `coedge_forward`: if the ring
        // being merged away is the *only* loop on `moved_face_id`, that face
        // is left with no loop at all, and a face must still have exactly one
        // outer boundary. It becomes a bare-vertex boundary at the vertex the
        // ring detached from — precisely inverting `mer`, which promotes a
        // bare-vertex boundary to the ring that arrives on it.
        let detach_vertex = self
            .coedge_start_vertex_id(coedge_forward)
            .with_context(&ctx)?;

        let survivor = ca.prev;

        // ca and cb each sit between one ring's root and the other ring's
        // prev, so removing them reconnects across: ca's prev links up with
        // cb's next, and cb's prev links up with ca's next — merging both
        // rings into one.
        self.coedges.get_mut(&ca.prev).unwrap().next = cb.next;
        self.coedges.get_mut(&cb.next).unwrap().prev = ca.prev;
        self.coedges.get_mut(&cb.prev).unwrap().next = ca.next;
        self.coedges.get_mut(&ca.next).unwrap().prev = cb.prev;
        self.coedges.remove(&coedge_forward);
        self.coedges.remove(&coedge_backward);
        self.edges.remove(&ca.edge().with_context(&ctx)?);

        // Every coedge that was on moved_face_id now belongs to the merged
        // ring on survivor_face_id.
        for c in self.iterate_loop_coedges(survivor).collect::<Vec<_>>() {
            self.coedges.get_mut(&c).unwrap().face = survivor_face_id;
        }

        // The surviving boundary keeps whichever role it already had, and the
        // ring that merged into it is gone. `remove_boundary` rejects an
        // attempt to consume an outer loop, which is the honest failure for a
        // caller that has asked to merge away the very loop bounding a face.
        self.faces
            .get_mut(&survivor_face_id)
            .unwrap()
            .set_boundary(keep_index, BoundaryType::Loop(survivor));
        match remove_index {
            BoundaryIndex::Outer => {
                self.get_face_mut(moved_face_id).with_context(&ctx)?.outer =
                    BoundaryType::Vertex(detach_vertex);
            }
            BoundaryIndex::Hole(_) => {
                self.remove_boundary(moved_face_id, remove_index)
                    .with_context(&ctx)?;
            }
        }

        Ok(())
    }
}
