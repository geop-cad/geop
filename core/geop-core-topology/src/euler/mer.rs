use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, FaceId, Model, Sense,
    argument_validation::{
        validate_curve_start_and_end, validate_pcurve_start_and_end, validate_same_loop,
    },
    boundary::BoundaryType,
};
use geop_core_geometry::nurb_curve::{NurbCurve2D, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
};

impl<S: Scalar> Model<S> {
    // Like `mef`, but instead of creating a brand new face for the ring
    // being split off, moves it onto `existing_face_id` — an already-existing
    // face, as an additional boundary of its own — leaving the rest of the
    // ring (the "old ring") right where it was, on whichever face
    // coedge1/coedge2 currently belong to. Useful for e.g. growing a hole's
    // boundary directly on a real, already-built face while its own mirror
    // ring is grown as scratch topology on a placeholder, then moved onto
    // that placeholder for further work (see `geop_ops_extrude_revolve::extrude`).
    // Curve and pcurve (validated against `existing_face_id`'s current
    // surface) must go from coedge1's end vertex to coedge2's start vertex.
    // PCurve reversed (on the original face) must go from coedge2's start
    // vertex to coedge1's end vertex.
    // Coedge1 and coedge2 must belong to the same loop of the same face.
    // returns (new edge, new coedge on the original face, new coedge on
    // existing_face_id)
    pub fn mer(
        self: &mut Model<S>,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        existing_face_id: FaceId,
    ) -> GeopResult<(EdgeId, CoedgeId, CoedgeId)> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::mer(
                coedge1={coedge1}
                coedge2={coedge2}
                curve={curve}
                pcurve={pcurve}
                pcurve_reversed={pcurve_reversed}
                existing_face_id={existing_face_id}"
            ))
        };

        let ce1 = self.get_coedge(coedge1)?.clone();
        let ce2 = self.get_coedge(coedge2)?.clone();
        validate_same_loop(self, coedge1, coedge2)?;
        let old_face_id = ce1.face;
        let old_face = self.get_face(old_face_id)?.clone();
        let existing_face = self.get_face(existing_face_id)?.clone();

        let start = self
            .get_vertex(self.coedge_end_vertex_id(coedge1)?)?
            .clone()
            .point;
        let end = self
            .get_vertex(self.coedge_start_vertex_id(coedge2)?)?
            .clone()
            .point;

        validate_pcurve_start_and_end(&existing_face.surface, &pcurve, &start, &end)
            .with_context("initial arg validation")
            .with_context(&ctx)?;
        validate_pcurve_start_and_end(&old_face.surface, &pcurve_reversed, &end, &start)
            .with_context("initial arg validation")
            .with_context(&ctx)?;
        validate_curve_start_and_end(&curve, &start, &end)
            .with_context("initial arg validation")
            .with_context(&ctx)?;

        // Find which of `old_face`'s boundaries is the one being split, by
        // index, before any pointers are mutated — see `mef` for why this
        // has to happen now rather than by walking post-mutation pointers.
        let old_boundary_idx = self
            .find_boundary_containing(old_face_id, coedge1)
            .with_context(&ctx)?;

        // All coedges between coedge1 and coedge2 (inclusive) will end up on
        // existing_face_id, so validate their pcurves against its surface.
        let mut ring_members = vec![coedge2];
        while *ring_members.last().unwrap() != coedge1 {
            ring_members.push(self.get_coedge(*ring_members.last().unwrap())?.next);
        }
        for &member in &ring_members {
            let c = self.get_coedge(member)?.clone();
            let sp = self.coedge_start_vertex(member)?.point;
            let ep = self.coedge_end_vertex(member)?.point;
            validate_pcurve_start_and_end(&existing_face.surface, &c.pcurve, &sp, &ep)
                .with_context(with_context!(
                    "reassigning coedge {member} to existing_face_id"
                ))
                .with_context(&ctx)?;
        }

        // Create the new edge and coedges
        let edge_id = self.insert_edge(Edge {
            curve,
            start_vertex: self.coedge_end_vertex_id(coedge1)?,
            end_vertex: self.coedge_start_vertex_id(coedge2)?,
        });
        let next1 = ce1.next;
        let prev2 = ce2.prev;

        // coedge_forward: coedge1.end -> coedge2.start
        let coedge_forward = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Forward,
            pcurve: pcurve,
            next: coedge2,
            prev: coedge1,
            face: ce1.face, // fixed up below, once moved onto existing_face_id
        });

        // coedge_backward: coedge2.start -> coedge1.end
        let coedge_backward = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Reversed,
            pcurve: pcurve_reversed,
            next: next1,
            prev: prev2,
            face: ce1.face,
        });

        self.coedges.get_mut(&coedge1).unwrap().next = coedge_forward;
        self.coedges.get_mut(&coedge2).unwrap().prev = coedge_forward;
        self.coedges.get_mut(&prev2).unwrap().next = coedge_backward;
        self.coedges.get_mut(&next1).unwrap().prev = coedge_backward;

        // Add the split-off ring as a new boundary of existing_face_id.
        // Where the ring lands depends on what already bounds the face it
        // moves to. A ring can only be a *hole* of a face that already has a
        // loop around it; a face whose boundary is still a bare vertex has no
        // such loop, so the arriving ring becomes the loop that bounds it —
        // the same promotion `mve` performs when an edge first grows off a
        // lone vertex. Splitting a ring off within a face that is already
        // bounded leaves its outer extent unchanged, so there it is a hole.
        let existing = self.faces.get_mut(&existing_face_id).unwrap();
        match existing.outer {
            BoundaryType::Vertex(_) => existing.outer = BoundaryType::Loop(coedge2),
            BoundaryType::Loop(_) => existing.holes.push(BoundaryType::Loop(coedge2)),
        }

        // reassign the coedge faces
        self.coedges.get_mut(&coedge_forward).unwrap().face = existing_face_id;
        for &member in &ring_members {
            self.coedges.get_mut(&member).unwrap().face = existing_face_id;
        }

        // The boundary we found above (by index, pre-mutation) is the one
        // that just split into the ring now owned by `existing_face_id` and
        // this old ring (whose coedges' `.face` was never touched) — repoint
        // it at the old ring via `next1`.
        self.faces
            .get_mut(&old_face_id)
            .unwrap()
            .set_boundary(old_boundary_idx, BoundaryType::Loop(next1));

        Ok((edge_id, coedge_backward, coedge_forward))
    }
}
