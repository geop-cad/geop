/// Newton iterations for the `(u, v)` foot-point refinement. Like
/// `NurbCurve::refine_parameter_at_point`'s own count, this only affects how
/// tightly an already-isolated answer is pinned down.
const NEWTON_ITERATIONS: usize = 20;

use crate::{Coedge, CoedgeGeometry, Edge, EdgeId, Model, Sense, VertexId};
use geop_core_geometry::contains::{curve::curve_could_contain, surface::surface_could_contain};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector2,
};

impl<S: Scalar> Model<S> {
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
        edge_t: S,
        vertex_id: VertexId,
        max_nodes: usize,
        min_subdivision_size: S,
    ) -> GeopResult<EdgeId> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::split_edge_at_vertex(edge_id={edge_id}, edge_t={edge_t}, vertex_id={vertex_id}, max_nodes={max_nodes}, min_subdivision_size={min_subdivision_size})"
            ))
        };

        let edge = self.get_edge(edge_id).with_context(&ctx)?.clone();
        let vertex_point = self.get_vertex(vertex_id).with_context(&ctx)?.point;

        let curve_point = edge.curve.evaluate(edge_t).with_context(&ctx)?;
        if !curve_point.could_be_equal(&vertex_point) {
            return Err(ctx(GeopError::new(format!(
                "edge {edge_id} at t={edge_t} evaluates to {curve_point}, which does not coincide with vertex {vertex_id} at {vertex_point}"
            ))));
        }

        // A split parameter that isn't *definitely* strictly inside its own
        // curve's domain could be that domain's bound — meaning the vertex is
        // already an endpoint of that curve, so there is no interior split to
        // make. That is the caller's job to rule out before asking (see
        // `find_piercing_crossing`, which skips exactly these), so reaching
        // here with one is a caller bug and is reported rather than quietly
        // ignored: silently doing nothing would leave the caller believing an
        // edge had been split when it had not.
        //
        // Checked for the 3-D curve *and* every coedge's pcurve before any
        // mutation — splitting the edge and only then finding a pcurve that
        // cannot be split would leave the model half-updated.
        //
        // Both parameters are Newton-refined against the point they are meant
        // to name before anything else looks at them (see
        // `NurbCurve::refine_parameter_at_point`). A subdivision search only
        // pins a parameter down to its own tolerance, and `split` cannot take
        // a parameter that wide — Boehm insertion amplifies it without bound.
        // The refinement is what makes the parameter narrow enough to split
        // at *and* still an honest enclosure, replacing the `sharpen` that
        // used to stand here and silently moved the split to the interval's
        // midpoint instead of the located point. Validation below therefore
        // tests exactly the value the split will use.
        let (curve_lo, curve_hi) = edge.curve.domain();
        let edge_t = edge
            .curve
            .refine_parameter_at_point(edge_t, &vertex_point)
            .with_context(&ctx)?;
        if !edge_t.definitely_greater(curve_lo) || !edge_t.definitely_less(curve_hi) {
            return Err(ctx(GeopError::new(format!(
                "edge_t={edge_t:?} is not strictly inside the curve domain ({curve_lo:?}, {curve_hi:?}), so vertex {vertex_id} is an endpoint of edge {edge_id} rather than interior to it — there is nothing to split"
            ))));
        }
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
            // locate the vertex in the face's `(u, v)` space, then project
            // that `(u, v)` back onto the coedge's own pcurve.
            let (u, v) = surface_could_contain(
                surface,
                &vertex_point,
                max_nodes,
                min_subdivision_size,
            )
            .with_context(&ctx)
            .with_context(&coedge_ctx)?
            .ok_or_else(|| {
                ctx(coedge_ctx(GeopError::new(format!(
                    "could not locate vertex {vertex_id} (at {vertex_point}) on face {}'s surface",
                    coedge.face
                ))))
            })?;
            // `surface_could_contain` isolates the `(u, v)` to its own
            // subdivision tolerance, which is far too wide to serve as the
            // *target* of the pcurve refinement below: a wide target makes a
            // wide residual, which makes a wide step, and the refined pcurve
            // parameter comes back no narrower than it started. So refine the
            // target first, by the same subdivide-to-isolate-then-Newton
            // split of labour — `project` polishes the foot point and keeps
            // its last iterate unsharpened, so the result is narrow and still
            // honest. Intersected with the box the search proved it lies in,
            // and falling back to that box if Newton left it.
            let refined = surface
                .project(vertex_point, u.sharpen(), v.sharpen(), NEWTON_ITERATIONS)
                .with_context(&ctx)
                .with_context(&coedge_ctx)?;
            let (u, v) = if refined.0.could_be_equal(u) && refined.1.could_be_equal(v) {
                (u.intersect(refined.0), v.intersect(refined.1))
            } else {
                (u, v)
            };
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
            let (pcurve_lo, pcurve_hi) = coedge.pcurve.domain();
            let pcurve_t = coedge
                .pcurve
                .refine_parameter_at_point(pcurve_t, &uv)
                .with_context(&ctx)
                .with_context(&coedge_ctx)?;
            if !pcurve_t.definitely_greater(pcurve_lo) || !pcurve_t.definitely_less(pcurve_hi) {
                return Err(ctx(coedge_ctx(GeopError::new(format!(
                    "pcurve_t={pcurve_t:?} is not strictly inside coedge {coedge_id}'s pcurve domain ({pcurve_lo:?}, {pcurve_hi:?}), so vertex {vertex_id} is an endpoint of that pcurve rather than interior to it — there is nothing to split"
                )))));
            }
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
