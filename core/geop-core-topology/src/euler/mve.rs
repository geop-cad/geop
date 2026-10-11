use crate::{CoedgeId, EdgeId, FaceId, Model, VertexId, boundary::BoundaryType};
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
        let ctx = |e: GeopError| e.with_context(format!("Model::mve(coedge={coedge}, p={p})"));
        let ce = self.get_coedge(coedge).with_context(&ctx)?.clone();
        let from = self.coedge_end_vertex_id(coedge).with_context(&ctx)?;
        let (v, forward, reversed, edge_id) = self
            .insert_spur(ce.face, from, curve, pcurve, pcurve_reversed, p)
            .with_context(&ctx)?;

        self.coedges.get_mut(&forward).unwrap().prev = coedge;
        self.coedges.get_mut(&reversed).unwrap().next = ce.next;
        self.coedges.get_mut(&coedge).unwrap().next = forward;
        self.coedges.get_mut(&ce.next).unwrap().prev = reversed;

        Ok((v, forward, reversed, edge_id))
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
        let (v, forward, reversed, edge_id) = self
            .insert_spur(face_id, vertex, curve, pcurve, pcurve_reversed, p)
            .with_context(&|e: GeopError| {
                e.with_context(format!(
                    "Model::mve_from_vertex(face_id={face_id}, vertex={vertex}, p={p})"
                ))
            })?;

        self.coedges.get_mut(&forward).unwrap().prev = reversed;
        self.coedges.get_mut(&reversed).unwrap().next = forward;

        // remove this vertex from the face boundary if it was there
        let face = self.get_face_mut(face_id)?;
        let is_this_vertex =
            |b: &BoundaryType| matches!(b, BoundaryType::Vertex(v_id) if *v_id == vertex);
        if is_this_vertex(&face.outer) {
            face.outer = BoundaryType::Loop(reversed);
        } else if let Some(hole) = face.holes.iter_mut().find(|b| is_this_vertex(b)) {
            *hole = BoundaryType::Loop(reversed);
        }

        Ok((v, forward, reversed, edge_id))
    }
}
