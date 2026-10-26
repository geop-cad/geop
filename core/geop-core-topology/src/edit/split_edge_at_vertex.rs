use crate::{Coedge, CoedgeGeometry, Edge, EdgeId, Model, Sense, VertexId};
use geop_core_geometry::contains::curve::curve_could_contain;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
};

impl Model {
    /// Splits `edge_id` at `edge_t` into two edges joined at the pre-existing
    /// `vertex_id` (which must already coincide with `edge.curve.evaluate(edge_t)`),
    /// and splits every coedge tracing `edge_id` to match. `edge_id` itself
    /// keeps the first (start -> vertex) segment; the new edge holding the
    /// second (vertex -> end) segment, in the same direction, is returned.
    ///
    /// `max_nodes`/`min_subdivision_size` bound the BFS search used to locate
    /// each coedge's own pcurve parameter for the split (see
    /// `curve_could_contain`).
    pub fn split_edge_at_vertex(
        &mut self,
        edge_id: EdgeId,
        edge_t: Scalar,
        vertex_id: VertexId,
        max_nodes: usize,
        min_subdivision_size: Scalar,
    ) -> GeopResult<EdgeId> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::split_edge_at_vertex(edge_id={edge_id}, edge_t={edge_t}, vertex_id={vertex_id}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
            ))
        };

        let edge = self.get_edge(edge_id).with_context(&ctx)?.clone();
        let vertex_point = self.get_vertex(vertex_id).with_context(&ctx)?.point;

        // Checked for the 3-D curve *and* every coedge's pcurve before any
        // mutation — splitting the edge and only then finding a pcurve that
        // cannot be split would leave the model half-updated.
        //
        // Each parameter is Newton-refined against the point it is meant to
        // name, and the curve is cut at the sharp middle of that: where in
        // the refined enclosure is a free choice, and cutting at all of it
        // made every later piece of the edge wider than the last (see
        // `NurbCurve::split_parameter_at_point`, which also checks that the
        // curve reaches the vertex at the parameter chosen). A parameter
        // that could be the curve's end is refused there: the vertex is
        // then already an endpoint, which the caller rules out before
        // asking (see `find_piercing_crossing`), so reaching here with one
        // is a caller bug, reported rather than quietly ignored.
        let edge_t = edge
            .curve
            .split_parameter_at_point(edge_t, &vertex_point)
            .with_context(&ctx)?;
        // Where the pieces will meet: a point within the vertex, and the one
        // every pcurve is cut at too. Cut each at a point of the vertex of
        // its own and the pieces of the edge and of its pcurves end apart,
        // by as much as the vertex is wide — `1.4e-13` where a cubic crosses
        // a plane at its inflection, and the pcurve's end then maps off the
        // edge's.
        let split_point = edge.curve.evaluate(edge_t).with_context(&ctx)?;
        let mut pcurve_splits = Vec::new();
        for coedge_id in self.coedges_of_edge(edge_id) {
            let coedge = self.get_coedge(coedge_id).with_context(&ctx)?.clone();
            let coedge_ctx = |e: GeopError| {
                let (t0, t1) = coedge.pcurve.domain();
                e.with_context(format!(
                    "coedge_id={coedge_id}, face={}, sense={:?}, pcurve_domain=({t0}, {t1})",
                    coedge.face, coedge.sense,
                ))
            };
            let surface = &self.get_face(coedge.face).with_context(&ctx)?.surface;
            // The pcurve's own parameter range doesn't share the edge curve's
            // `t` values, so the split point is relocated geometrically:
            // locate the split point in the face's `(u, v)` space, then project
            // that `(u, v)` back onto the coedge's own pcurve.
            // Located, not merely isolated: the containment search alone
            // pins `(u, v)` only to its own subdivision tolerance, far too
            // wide to serve as the *target* of the pcurve refinement below
            // (a wide target makes a wide residual, a wide step, and a
            // pcurve parameter no narrower than it started).
            let (u, v) = surface
                .locate(&split_point, max_nodes, min_subdivision_size)
                .with_context(&ctx)
                .with_context(&coedge_ctx)?
                .ok_or_else(|| {
                    ctx(coedge_ctx(GeopError::new(format!(
                        "could not locate the split point {split_point:?} of vertex {vertex_id} (at {vertex_point:?}) on face {}'s surface",
                        coedge.face
                    ))))
                })?;
            let uv = Vector2::from_array([u, v]);
            let uv_ctx = |e: GeopError| e.with_context(format!("found uv=({u}, {v})"));
            let pcurve_t =
                curve_could_contain(&coedge.pcurve, &uv, max_nodes, min_subdivision_size)
                    .with_context(&ctx)
                    .with_context(&coedge_ctx)
                    .with_context(&uv_ctx)?
                    .ok_or_else(|| {
                        ctx(coedge_ctx(uv_ctx(GeopError::new(format!(
                            "could not locate vertex {vertex_id} on coedge {coedge_id}'s pcurve"
                        )))))
                    })?;
            let pcurve_t = coedge
                .pcurve
                .split_parameter_at_point(pcurve_t, &uv)
                .with_context(&ctx)
                .with_context(&coedge_ctx)
                .with_context(&uv_ctx)?;
            pcurve_splits.push((coedge_id, pcurve_t));
        }

        let (new_curve_1, new_curve_2) = edge.curve.split(edge_t).with_context(&ctx)?;
        // Reuse `edge_id` for the first (start -> vertex) segment, shortened
        // in place, and only allocate a new edge for the second segment.
        let existing_edge = self.get_edge_mut(edge_id).with_context(&ctx)?;
        existing_edge.curve = new_curve_1;
        existing_edge.end_vertex = vertex_id;
        let edge_1 = edge_id;
        let edge_2 = self.insert_edge(Edge {
            curve: new_curve_2,
            start_vertex: vertex_id,
            end_vertex: edge.end_vertex,
        });

        for (coedge_id, pcurve_t) in pcurve_splits {
            let coedge = self.get_coedge(coedge_id).with_context(&ctx)?.clone();

            let coedge_ctx = |e: GeopError| {
                let (t0, t1) = coedge.pcurve.domain();
                e.with_context(format!(
                    "coedge_id={coedge_id}, face={}, sense={:?}, pcurve_domain=({t0}, {t1})",
                    coedge.face, coedge.sense,
                ))
            };

            let (pcurve_left, pcurve_right) = coedge
                .pcurve
                .split(pcurve_t)
                .with_context(&ctx)
                .with_context(&coedge_ctx)?;
            // `pcurve_left`/`pcurve_right` are in the coedge's own traversal
            // order (its domain's low end is its own start), which for a
            // `Reversed` coedge runs from `edge_2` back to `edge_1`.
            let (first_edge, first_pcurve, second_edge, second_pcurve) = match coedge.sense {
                Sense::Forward => (edge_1, pcurve_left, edge_2, pcurve_right),
                Sense::Reversed => (edge_2, pcurve_left, edge_1, pcurve_right),
            };

            let new_coedge_id = self.insert_coedge(Coedge {
                geometry: CoedgeGeometry::Edge(second_edge),
                sense: coedge.sense,
                pcurve: second_pcurve,
                next: coedge.next,
                prev: coedge_id,
                face: coedge.face,
            });

            let existing = self.get_coedge_mut(coedge_id).with_context(&ctx)?;
            existing.geometry = CoedgeGeometry::Edge(first_edge);
            existing.pcurve = first_pcurve;
            existing.next = new_coedge_id;

            self.get_coedge_mut(coedge.next).with_context(&ctx)?.prev = new_coedge_id;
        }

        Ok(edge_2)
    }
}
