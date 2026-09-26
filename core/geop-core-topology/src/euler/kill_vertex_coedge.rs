use crate::{CoedgeGeometry, CoedgeId, Model, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    /// Undo [`Model::add_vertex_coedge`]: splice `coedge` back out of its
    /// loop. `coedge` must be `Vertex`-backed (i.e. one `add_vertex_coedge`
    /// itself returned) — nothing is removed from `V`, `E`, or `F` here
    /// either, matching `add_vertex_coedge`'s own no-op effect on them.
    pub fn kill_vertex_coedge(self: &mut Model<S>, coedge: CoedgeId) -> GeopResult<()> {
        let ctx =
            |e: GeopError| e.with_context(format!("Model::kill_vertex_coedge(coedge={coedge})"));

        let ce = self.get_coedge(coedge)?.clone();
        if !matches!(ce.geometry, CoedgeGeometry::Vertex(_)) {
            return Err(ctx(GeopError::new(
                "coedge is edge-backed, not vertex-backed — use kve instead",
            )));
        }

        self.coedges.get_mut(&ce.prev).unwrap().next = ce.next;
        self.coedges.get_mut(&ce.next).unwrap().prev = ce.prev;
        self.coedges.remove(&coedge);

        let face = self.faces.get_mut(&ce.face).unwrap();
        for b in face.boundaries_mut() {
            if *b == BoundaryType::Loop(coedge) {
                *b = BoundaryType::Loop(ce.next);
            }
        }

        Ok(())
    }
}
