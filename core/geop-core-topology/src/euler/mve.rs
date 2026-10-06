use crate::{
    Coedge, CoedgeGeometry, CoedgeId, Edge, EdgeId, FaceId, Model, Sense, Vertex, VertexId,
    argument_validation::{validate_curve_start_and_end, validate_pcurve_start_and_end},
    boundary::BoundaryType,
};
use geop_core_geometry::nurb_curve::{NurbCurve2D, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

impl<S: Scalar> Model<S> {
    // Add a new edge right after the coedge.
    // Curve and pcurve must go from the coedge's end vertex to the new vertex `p`.
    // PCurve reversed must go from the new vertex `p` to the coedge's end vertex.
    // returns (new vertex, coedge_forward -> p, coedge_reversed <- p, new edge)
    pub fn mve(
        self: &mut Model<S>,
        coedge: CoedgeId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        p: Vector3<S>,
    ) -> GeopResult<(VertexId, CoedgeId, CoedgeId, EdgeId)> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "Model::mve(
    coedge={coedge}
    curve={curve}
    pcurve={pcurve}
    pcurve_reversed={pcurve_reversed}
    p={p}
)"
            ))
        };
        let ce = self.get_coedge(coedge)?.clone();
        let face = self.get_face(ce.face)?.clone();

        let ce_end_id = self.coedge_end_vertex_id(coedge)?;
        let ce_end = self.get_vertex(ce_end_id)?.clone();

        validate_pcurve_start_and_end(&face.surface, &pcurve, &ce_end.point, &p)
            .with_context(&ctx)?;
        validate_pcurve_start_and_end(&face.surface, &pcurve_reversed, &p, &ce_end.point)
            .with_context(&ctx)?;
        validate_curve_start_and_end(&curve, &ce_end.point, &p).with_context(&ctx)?;

        let v = self.insert_vertex(Vertex { point: p });
        let edge_id = self.insert_edge(Edge {
            curve,
            start_vertex: ce_end_id,
            end_vertex: v,
        });

        let coedge_forward = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Forward,
            pcurve,
            next: CoedgeId(0), // set later
            prev: coedge,
            face: ce.face,
        });

        let coedge_reversed = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Reversed,
            pcurve: pcurve_reversed,
            next: ce.next,     // set later
            prev: CoedgeId(0), // set later
            face: ce.face,
        });

        self.coedges.get_mut(&coedge_forward).unwrap().next = coedge_reversed;
        self.coedges.get_mut(&coedge_reversed).unwrap().prev = coedge_forward;
        self.coedges.get_mut(&coedge).unwrap().next = coedge_forward;
        self.coedges.get_mut(&ce.next).unwrap().prev = coedge_reversed;

        Ok((v, coedge_forward, coedge_reversed, edge_id))
    }

    // Add a new edge between the vertex and the new vertex `p`.
    // Curve and pcurve must go from the existing vertex to the new vertex `p`.
    // PCurve reversed must go from the new vertex `p` to the existing vertex.
    // returns (new vertex, coedge_forward -> p, coedge_reversed <- p, new edge)
    pub fn mve_from_vertex(
        self: &mut Model<S>,
        face_id: FaceId,
        vertex: VertexId,
        curve: NurbCurve3D<S>,
        pcurve: NurbCurve2D<S>,
        pcurve_reversed: NurbCurve2D<S>,
        p: Vector3<S>,
    ) -> GeopResult<(VertexId, CoedgeId, CoedgeId, EdgeId)> {
        let face = self.get_face(face_id)?.clone();
        let ce_end = self.get_vertex(vertex)?.clone();
        validate_pcurve_start_and_end(&face.surface, &pcurve, &ce_end.point, &p)?;
        validate_pcurve_start_and_end(&face.surface, &pcurve_reversed, &p, &ce_end.point)?;
        validate_curve_start_and_end(&curve, &ce_end.point, &p)?;

        let v = self.insert_vertex(Vertex { point: p });
        let edge_id = self.insert_edge(Edge {
            curve,
            start_vertex: vertex,
            end_vertex: v,
        });

        let coedge_forward = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Forward,
            pcurve,
            next: CoedgeId(0), // set later
            prev: CoedgeId(0), // set later
            face: face_id,
        });

        let coedge_reversed = self.insert_coedge(Coedge {
            geometry: CoedgeGeometry::Edge(edge_id),
            sense: Sense::Reversed,
            pcurve: pcurve_reversed,
            next: CoedgeId(0), // set later
            prev: CoedgeId(0), // set later
            face: face_id,
        });

        self.coedges.get_mut(&coedge_forward).unwrap().next = coedge_reversed;
        self.coedges.get_mut(&coedge_reversed).unwrap().prev = coedge_forward;
        self.coedges.get_mut(&coedge_forward).unwrap().prev = coedge_reversed;
        self.coedges.get_mut(&coedge_reversed).unwrap().next = coedge_forward;

        // remove this vertex from the face boundary if it was there
        let face = self.get_face_mut(face_id)?;
        let is_this_vertex =
            |b: &BoundaryType| matches!(b, BoundaryType::Vertex(v_id) if *v_id == vertex);
        if is_this_vertex(&face.outer) {
            face.outer = BoundaryType::Loop(coedge_reversed);
        } else if let Some(hole) = face.holes.iter_mut().find(|b| is_this_vertex(b)) {
            *hole = BoundaryType::Loop(coedge_reversed);
        }

        Ok((v, coedge_forward, coedge_reversed, edge_id))
    }
}
