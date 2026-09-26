use crate::{CoedgeId, Model, argument_validation::validate_same_loop, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Kill the edge added by mekr, splitting the single loop it merged back into two
    // loops (rings) and adding a boundary entry for the newly separated ring. ca_id
    // and cb_id are mekr's own two coedges (its returned coedge_a and coedge_b) and
    // must currently lie on the same loop of the same face (the shape mekr leaves
    // behind).
    pub fn kemr(self: &mut Model<S>, ca_id: CoedgeId, cb_id: CoedgeId) -> GeopResult<()> {
        let ctx =
            |e: GeopError| e.with_context(format!("Model::kemr(ca_id={ca_id}, cb_id={cb_id})"));

        let ca = self.get_coedge(ca_id)?.clone();
        let cb = self.get_coedge(cb_id)?.clone();
        if ca.edge().with_context(&ctx)? != cb.edge().with_context(&ctx)? {
            return Err(ctx(GeopError::new(
                "ca_id and cb_id must belong to the same edge",
            )));
        }
        if ca.face != cb.face {
            return Err(ctx(GeopError::new(
                "ca_id and cb_id must belong to the same face",
            )));
        }
        validate_same_loop(&self, ca_id, cb_id).with_context(&ctx)?;

        let face_id = ca.face;
        let ring_index = self
            .find_boundary_containing(face_id, ca_id)
            .with_context(&ctx)?;

        let ring1 = cb.next;
        let ring2 = ca.next;

        // ca and cb each sit between one root coedge and the other loop's prev, so
        // removing them reconnects across: ca's prev links up with cb's next, and
        // cb's prev links up with ca's next.
        self.coedges.get_mut(&ca.prev).unwrap().next = cb.next;
        self.coedges.get_mut(&cb.next).unwrap().prev = ca.prev;
        self.coedges.get_mut(&cb.prev).unwrap().next = ca.next;
        self.coedges.get_mut(&ca.next).unwrap().prev = cb.prev;
        self.coedges.remove(&ca_id);
        self.coedges.remove(&cb_id);
        self.edges.remove(&ca.edge().with_context(&ctx)?);

        // The loop that was cut in two keeps its own role — outer stays
        // outer, a hole stays that hole — and the second ring it split off
        // becomes a new hole. Splitting a loop can only ever *add* an inner
        // boundary: the material still ends at the same outermost ring.
        let face = self.faces.get_mut(&face_id).unwrap();
        face.set_boundary(ring_index, BoundaryType::Loop(ring1));
        face.holes.push(BoundaryType::Loop(ring2));

        Ok(())
    }
}
