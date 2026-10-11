use crate::{EdgeId, FaceId, Model, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    /// Kill the edge added by `mef`, merging `killed_face` back into the
    /// face on the other side of `edge`: the absorbed face's ring is
    /// spliced back into the surviving face's ring (undoing the split),
    /// and `killed_face` is removed from the model and from its shell's
    /// face list. `edge`'s two coedges must currently lie on two
    /// *different* faces, one of which is `killed_face`.
    pub fn kef(self: &mut Model<S>, edge: EdgeId, killed_face: FaceId) -> GeopResult<()> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::kef(edge={edge}, killed_face={killed_face})"
            ))
        };

        let refs = self.coedges_of_edge(edge);
        if refs.len() != 2 {
            return Err(ctx(GeopError::new("edge must have exactly two coedges")));
        }
        let (c0_id, c1_id) = (refs[0], refs[1]);
        let c0_face = self.get_coedge(c0_id)?.face;
        if c0_face == self.get_coedge(c1_id)?.face {
            return Err(ctx(GeopError::new(
                "edge's coedges must belong to different faces (use ker to merge two rings of the same face)",
            )));
        }

        let absorbed_face_id = killed_face;
        let absorbed_face = self.get_face(absorbed_face_id)?.clone();

        let (ca_id, cb_id) = if c0_face == killed_face {
            (c1_id, c0_id)
        } else {
            (c0_id, c1_id)
        };
        let ca = self.get_coedge(ca_id)?.clone();

        let surviving_face_id = ca.face;
        let keep_index = self
            .find_boundary_containing(surviving_face_id, ca_id)
            .map_err(|_| {
                ctx(GeopError::new(
                    "could not find the ring containing the surviving coedge",
                ))
            })?;

        let survivor = ca.prev;

        self.remove_edge_between(ca_id, cb_id).with_context(&ctx)?;

        // Every coedge that was on the absorbed face now belongs to the
        // merged ring on the surviving face.
        for c in self.iterate_loop_coedges(survivor).collect::<Vec<_>>() {
            self.coedges.get_mut(&c).unwrap().face = surviving_face_id;
        }

        // The absorbed face's holes are still holes of the merged material —
        // they describe regions removed from a patch that now belongs to the
        // surviving face, so they have to come across. Dropping them (as
        // deleting the face outright would) silently fills them in.
        let absorbed_holes = absorbed_face.holes.clone();
        let surviving = self.faces.get_mut(&surviving_face_id).unwrap();
        surviving.set_boundary(keep_index, BoundaryType::Loop(survivor));
        surviving.holes.extend(absorbed_holes);
        self.faces.remove(&absorbed_face_id);
        if let Some(shell) = self.shells.get_mut(&absorbed_face.shell) {
            shell.faces.retain(|&f| f != absorbed_face_id);
        }

        Ok(())
    }
}
