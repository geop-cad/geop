use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Model, Sense, VertexId,
    argument_validation::validate_pcurve_start_and_end,
};
use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    /// Splice a single degenerate coedge — backed directly by `vertex` (see
    /// [`CoedgeGeometry::Vertex`]), not a real edge — into `after`'s own
    /// loop, right after it. `pcurve` must both start and end at `vertex`'s
    /// own position, as mapped through `after`'s face's surface (it's
    /// meant to sweep some parameter-space range while sitting at that one
    /// 3-D point the whole way, e.g. tracing a pole row's full angular
    /// span).
    ///
    /// Unlike every other euler operator here, this isn't one: it doesn't
    /// touch `V`, `E`, or `F` (`vertex` already exists, and no edge is
    /// created), so there's no Euler–Poincaré invariant for it to need to
    /// preserve — it's pure loop bookkeeping, letting a face's own
    /// boundary legitimately pass through an already-shared vertex without
    /// requiring a real (and, for a single-face-only detour, otherwise
    /// unpaired) edge to carry a pcurve.
    pub fn add_vertex_coedge(
        self: &mut Model<S>,
        after: CoedgeId,
        vertex: VertexId,
        pcurve: NurbCurve2D<S>,
    ) -> GeopResult<CoedgeId> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::add_vertex_coedge(after={after}, vertex={vertex}, pcurve={pcurve})"
            ))
        };

        let ce = self.get_coedge(after)?.clone();
        let face = self.get_face(ce.face)?.clone();
        let p = self.get_vertex(vertex)?.point;

        validate_pcurve_start_and_end(&face.surface, &pcurve, &p, &p).with_context(&ctx)?;

        let new_coedge = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Vertex(vertex),
            sense: Sense::Forward,
            pcurve,
            next: ce.next,
            prev: after,
            face: ce.face,
        });

        self.coedges.get_mut(&after).unwrap().next = new_coedge;
        self.coedges.get_mut(&ce.next).unwrap().prev = new_coedge;

        Ok(new_coedge)
    }
}
