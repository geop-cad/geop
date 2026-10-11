use crate::{CoedgeId, EdgeId, Face, FaceId, Model, boundary::BoundaryType};
use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Create a new face by inserting an edge between
    // coedge1's end vertex and coedge2's start vertex.
    // Curve and pcurve (on the new face) must go from coedge1's end vertex to coedge2's start vertex.
    // PCurve reversed (on the existing face) must go from coedge2's start vertex to coedge1's end vertex.
    // Coedge1 and coedge2 must belong to the same loop of the same face.
    // The new face will have the edge and coedge 1 and 2 on the boundary.
    // returns (new edge, new face, new coedge on the existing face)
    pub fn mef(
        self: &mut Model<S>,
        coedge1: CoedgeId,
        coedge2: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        new_surface: NurbSurface3D<S>,
    ) -> GeopResult<(EdgeId, FaceId, CoedgeId, CoedgeId)> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::mef(
                coedge1={coedge1}
                coedge2={coedge2}
                curve={curve}
                pcurve={pcurve}
                pcurve_reversed={pcurve_reversed}
                new_surface={new_surface}"
            ))
        };

        let (old_boundary_idx, ring_members) = self
            .check_ring_split(
                coedge1,
                coedge2,
                &curve,
                &pcurve,
                &pcurve_reversed,
                &new_surface,
            )
            .with_context(&ctx)?;
        let old_face_id = self.get_coedge(coedge1)?.face;
        let shell = self.get_face(old_face_id)?.shell;
        let next1 = self.get_coedge(coedge1)?.next;
        let (edge_id, coedge_forward, coedge_backward) =
            self.insert_edge_between(coedge1, coedge2, curve, pcurve, pcurve_reversed)?;

        // Create the new face
        let new_face_id = self.insert_face(Face {
            surface: new_surface,
            outer: BoundaryType::Loop(coedge2),
            holes: Vec::new(),
            shell,
        });
        self.shells.get_mut(&shell).unwrap().faces.push(new_face_id);

        // reassign the coedge faces
        self.coedges.get_mut(&coedge_forward).unwrap().face = new_face_id;
        for &member in &ring_members {
            self.coedges.get_mut(&member).unwrap().face = new_face_id;
        }

        // The boundary we found above (by index, pre-mutation) is the one
        // that just split into the new ring (now owned by `new_face_id`,
        // already given its own boundary entry above) and this old ring
        // (whose coedges' `.face` was never touched) — repoint it at the old
        // ring via `next1`.
        // Splitting a face's *outer* loop makes two faces, each bounded by
        // one of the halves — so the new face's outer loop is the new ring
        // and the old face keeps its own role. Splitting a *hole* instead
        // carves a new face out of that hole's interior: the new face is
        // bounded by the ring that came off, and the old face's hole
        // continues to be a hole. Either way the boundary that split keeps
        // its kind, which is what `set_boundary` expresses.
        //
        // Holes of the old face are left where they are; `mef` has no way to
        // know which side of the new edge each falls on. Callers that split a
        // face carrying holes must reclassify them afterwards (see
        // `Model::reclassify_holes`).
        self.faces
            .get_mut(&old_face_id)
            .unwrap()
            .set_boundary(old_boundary_idx, BoundaryType::Loop(next1));

        Ok((edge_id, new_face_id, coedge_forward, coedge_backward))
    }
}
