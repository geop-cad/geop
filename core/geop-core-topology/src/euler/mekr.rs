use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, Model, Sense,
    argument_validation::{
        validate_curve_start_and_end, validate_different_loop, validate_pcurve_start_and_end,
    },
};
use geop_core_geometry::nurb_curve::{NurbCurve2D, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Make a new edge from coedge1's end vertex to coedge2's start vertex, merging
    // the two different loops (rings) that coedge1 and coedge2 belong to into a
    // single loop rooted at coedge1. Both coedges must belong to the same face, but
    // to different boundary loops of it; the boundary entry for coedge2's ring is
    // killed since it becomes part of coedge1's ring. pcurve is required to go from
    // coedge1's end vertex to coedge2's start vertex, as measured through the
    // face's surface.
    // returns (new edge, coedge_a: end1 -> start2, coedge_b: start2 -> end1)
    pub fn mekr(
        self: &mut Model<S>,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
    ) -> GeopResult<(EdgeId, CoedgeId, CoedgeId)> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::mekr(
    coedge1={coedge1}
    coedge2={coedge2}
    curve={curve}
    pcurve={pcurve}
)"
            ))
        };

        let ce1 = self.get_coedge(coedge1)?.clone();
        let ce2 = self.get_coedge(coedge2)?.clone();
        if ce1.face != ce2.face {
            return Err(ctx(GeopError::new(
                "coedge1 and coedge2 must belong to the same face",
            )));
        }
        validate_different_loop(&self, coedge1, coedge2).with_context(&ctx)?;
        let face = self.get_face(ce1.face)?.clone();

        let start_id = self.coedge_end_vertex_id(coedge1)?;
        let end_id = self.coedge_start_vertex_id(coedge2)?;
        let start = self.get_vertex(start_id)?.clone();
        let end = self.get_vertex(end_id)?.clone();

        validate_pcurve_start_and_end(&face.surface, &pcurve, &start.point, &end.point)
            .with_context(&ctx)?;
        validate_curve_start_and_end(&curve, &start.point, &end.point).with_context(&ctx)?;

        // The ring containing coedge2 disappears into coedge1's ring, so find its
        // boundary entry now, before the splice below makes every ring look the same.
        let killed_ring_index = self
            .find_boundary_containing(ce1.face, coedge2)
            .with_context(&ctx)?;

        let edge_id = self.insert_edge(Edge {
            curve,
            start_vertex: start_id,
            end_vertex: end_id,
        });

        let reversed_pcurve = pcurve.reverse();
        let next1 = ce1.next;
        let prev2 = ce2.prev;

        // coedge_a: end1 -> start2, spliced in right after coedge1.
        let coedge_a = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Forward,
            pcurve,
            next: coedge2,
            prev: coedge1,
            face: ce1.face,
        });

        // coedge_b: start2 -> end1, spliced in right after prev2.
        let coedge_b = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Reversed,
            pcurve: reversed_pcurve,
            next: next1,
            prev: prev2,
            face: ce1.face,
        });

        self.coedges.get_mut(&coedge1).unwrap().next = coedge_a;
        self.coedges.get_mut(&coedge2).unwrap().prev = coedge_a;
        self.coedges.get_mut(&prev2).unwrap().next = coedge_b;
        self.coedges.get_mut(&next1).unwrap().prev = coedge_b;

        // The absorbed ring must be a hole: joining an inner ring to the loop
        // around it is what `mekr` does, and the outer boundary is what
        // survives that join. `remove_boundary` rejects the other case rather
        // than leaving a face with nothing bounding it.
        // Not via `ctx`: that closure borrows `curve`/`pcurve`, which have
        // since been moved into the new edge and coedges.
        self.remove_boundary(ce1.face, killed_ring_index)
            .with_context(&|e: GeopError| {
                e.with_context(format!(
                    "mekr(coedge1={coedge1}, coedge2={coedge2}): absorbing the ring containing coedge2"
                ))
            })?;

        Ok((edge_id, coedge_a, coedge_b))
    }
}
