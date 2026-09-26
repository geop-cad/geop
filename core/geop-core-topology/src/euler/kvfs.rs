use crate::{Model, SolidId, boundary::BoundaryType};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

impl<S: Scalar> Model<S> {
    // Kill the vertex, face and solid created by mvfs. solid must still be in its
    // freshly-made-vfs shape: a single void-free shell with a single face whose only
    // boundary is a bare vertex (no edges yet).
    pub fn kvfs(self: &mut Model<S>, solid: SolidId) -> GeopResult<()> {
        let ctx = |e: GeopError| e.with_context(format!("Model::kvfs(solid={solid})"));

        let s = self.get_solid(solid)?.clone();
        if s.shells.len() != 1 {
            return Err(ctx(GeopError::new("solid must have exactly one shell")));
        }
        let shell_id = s.shells[0];
        let shell = self.get_shell(shell_id)?.clone();
        if shell.faces.len() != 1 {
            return Err(ctx(GeopError::new("shell must have exactly one face")));
        }
        let face_id = shell.faces[0];
        let face = self.get_face(face_id)?.clone();
        if !face.holes.is_empty() {
            return Err(ctx(GeopError::new("face must have no holes")));
        }
        let vertex_id = match face.outer {
            BoundaryType::Vertex(v) => v,
            BoundaryType::Loop(_) => {
                return Err(ctx(GeopError::new("face boundary must be a bare vertex")));
            }
        };

        self.vertices.remove(&vertex_id);
        self.faces.remove(&face_id);
        self.shells.remove(&shell_id);
        self.solids.remove(&solid);

        Ok(())
    }
}
