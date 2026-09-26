use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, Face, FaceId, Model, Sense,
    argument_validation::{
        validate_curve_start_and_end, validate_pcurve_start_and_end, validate_same_loop,
    },
    boundary::BoundaryType,
};
use geop_core_geometry::{
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    with_context,
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

        let ce1 = self.get_coedge(coedge1)?.clone();
        let ce2 = self.get_coedge(coedge2)?.clone();
        validate_same_loop(self, coedge1, coedge2)?;
        let old_face_id = ce1.face;
        let old_face = self.get_face(old_face_id)?.clone();

        let start = self
            .get_vertex(self.coedge_end_vertex_id(coedge1)?)?
            .clone()
            .point;
        let end = self
            .get_vertex(self.coedge_start_vertex_id(coedge2)?)?
            .clone()
            .point;

        validate_pcurve_start_and_end(&new_surface, &pcurve, &start, &end)
            .with_context("initial arg validation")
            .with_context(&ctx)?;
        validate_pcurve_start_and_end(&old_face.surface, &pcurve_reversed, &end, &start)
            .with_context("initial arg validation")
            .with_context(&ctx)?;
        validate_curve_start_and_end(&curve, &start, &end)
            .with_context("initial arg validation")
            .with_context(&ctx)?;

        // Find which of `old_face`'s boundaries is the one being split (i.e.
        // the loop coedge1/coedge2 belong to), by index — done now, before
        // any pointers are mutated below, so the traversal walks the still
        // fully-intact original ring and unambiguously reaches `coedge1`
        // regardless of where that boundary's own anchor happens to sit in
        // it. Works for any number of boundaries (e.g. holes): exactly one
        // can structurally contain `coedge1`, since a face's boundaries are
        // disjoint loops.
        let old_boundary_idx = self
            .find_boundary_containing(old_face_id, coedge1)
            .with_context(&ctx)?;

        // All coedges between coedge1 and coedge2 (inclusive) will end up on the new face, so validate their pcurves against the new surface.
        let mut ring_members = vec![coedge2];
        while *ring_members.last().unwrap() != coedge1 {
            ring_members.push(self.get_coedge(*ring_members.last().unwrap())?.next);
        }
        for &member in &ring_members {
            let c = self.get_coedge(member)?.clone();
            let sp = self.coedge_start_vertex(member)?.point;
            let ep = self.coedge_end_vertex(member)?.point;
            validate_pcurve_start_and_end(&new_surface, &c.pcurve, &sp, &ep)
                .with_context(with_context!("reassigning coedge {member} to new face"))
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
            face: ce1.face, // fixed up below, once the new face exists
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

        // Create the new face
        let new_face_id = self.insert_face(Face {
            surface: new_surface,
            outer: BoundaryType::Loop(coedge2),
            holes: Vec::new(),
            shell: old_face.shell,
        });
        self.shells
            .get_mut(&old_face.shell)
            .unwrap()
            .faces
            .push(new_face_id);

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
