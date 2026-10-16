use crate::{CoedgeId, EdgeId, FaceId, Model, boundary::BoundaryType};
use geop_core_geometry::nurb_curve::{NurbCurve2D, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Like `mef`, but instead of creating a brand new face for the ring
    // being split off, moves it onto `existing_face_id` — an already-existing
    // face, as an additional boundary of its own — leaving the rest of the
    // ring (the "old ring") right where it was, on whichever face
    // coedge1/coedge2 currently belong to. Useful for e.g. growing a hole's
    // boundary directly on a real, already-built face while its own mirror
    // ring is grown as scratch topology on a placeholder, then moved onto
    // that placeholder for further work.
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

        let existing_surface = self.get_face(existing_face_id)?.surface.clone();
        let (old_boundary_idx, ring_members) = self
            .check_ring_split(
                coedge1,
                coedge2,
                &curve,
                &pcurve,
                &pcurve_reversed,
                &existing_surface,
            )
            .with_context(&ctx)?;
        let old_face_id = self.get_coedge(coedge1)?.face;
        let next1 = self.get_coedge(coedge1)?.next;
        let (edge_id, coedge_forward, coedge_backward) =
            self.insert_edge_between(coedge1, coedge2, curve, pcurve, pcurve_reversed)?;

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
