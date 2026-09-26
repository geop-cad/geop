use crate::{FaceId, Model, VertexId, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Kill the bare-vertex boundary added by mvr: vertex must be a bare-vertex
    // boundary (no edges yet) of face_id.
    pub fn kvr(self: &mut Model<S>, face_id: FaceId, vertex: VertexId) -> GeopResult<()> {
        let ctx = |e: GeopError| {
            e.with_context(format!("Model::kvr(face_id={face_id}, vertex={vertex})"))
        };

        let face = self.get_face(face_id).with_context(&ctx)?;
        // `mvr` adds a bare vertex as a *hole*, and only a hole can be
        // removed — killing a face's outer boundary would leave it unbounded,
        // so a vertex sitting there is not something `kvr` can undo.
        let hole_idx = face
            .holes
            .iter()
            .position(|&b| b == BoundaryType::Vertex(vertex))
            .ok_or_else(|| {
                ctx(GeopError::new(
                    "vertex is not a bare-vertex hole boundary of face_id",
                ))
            })?;

        self.faces.get_mut(&face_id).unwrap().holes.remove(hole_idx);
        self.vertices.remove(&vertex);

        Ok(())
    }
}
