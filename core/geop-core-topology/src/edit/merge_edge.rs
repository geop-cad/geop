use crate::{CoedgeGeometry, EdgeId, Model};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    /// Merges `edge_deleted_id` into `edge_into_id`: repoints every coedge
    /// tracing the deleted edge to the surviving one, then drops the
    /// now-unreferenced edge. `reversed` says whether the deleted edge ran
    /// start<->end the *other* way round relative to the surviving one — if
    /// so, every repointed coedge's `sense` is flipped so it keeps tracing
    /// the same physical direction it always did.
    ///
    /// TODO: `edge_into_id`'s own curve currently stays exactly as it was —
    /// it should instead be widened (unioned) to certainly contain the
    /// deleted edge's curve too, so the kept edge's geometry honestly
    /// reflects both original edges' combined tolerance instead of silently
    /// favoring whichever one happened to survive.
    pub fn merge_edge(
        &mut self,
        edge_into_id: EdgeId,
        edge_deleted_id: EdgeId,
        reversed: bool,
    ) -> GeopResult<()> {
        // Same self-merge hazard as `merge_vertex` — always a caller bug
        // (typically: acting on a stale edge snapshot), rejected rather than
        // silently absorbed.
        if edge_into_id == edge_deleted_id {
            return Err(GeopError::new(format!(
                "Model::merge_edge: edge_into_id and edge_deleted_id are both {edge_into_id} — refusing to merge an edge into itself"
            )));
        }

        for coedge in self.coedges.values_mut() {
            if coedge.geometry == CoedgeGeometry::Edge(edge_deleted_id) {
                coedge.geometry = CoedgeGeometry::Edge(edge_into_id);
                if reversed {
                    coedge.sense = coedge.sense.opposite();
                }
            }
        }

        self.edges.remove(&edge_deleted_id);

        Ok(())
    }
}
