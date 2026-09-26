use crate::{FaceId, Model, Vertex, VertexId, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

impl<S: Scalar> Model<S> {
    // Add a new bare vertex at `point` as a fresh boundary of `face_id` (e.g. to
    // later grow a hole loop off of via `mve_from_vertex`). Returns the new vertex.
    pub fn mvr(self: &mut Model<S>, face_id: FaceId, point: Vector3<S>) -> GeopResult<VertexId> {
        let ctx =
            |e: GeopError| e.with_context(format!("Model::mvr(face_id={face_id}, point={point})"));

        self.get_face(face_id).with_context(&ctx)?;

        let vertex_id = self.insert_vertex(Vertex { point });
        self.faces
            .get_mut(&face_id)
            .unwrap()
            .holes
            .push(BoundaryType::Vertex(vertex_id));

        Ok(vertex_id)
    }
}
