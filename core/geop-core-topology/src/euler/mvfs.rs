use crate::{
    Face, FaceId, Model, Shell, ShellId, SolidId, Vertex, VertexId, boundary::BoundaryType,
};
use geop_core_geometry::nurb_surface::NurbSurface3D;
use geop_core_math::{scalars::Scalar, vector::Vector3};

impl<S: Scalar> Model<S> {
    // Create a new solid with a single face and vertex at `point`. Returns the new vertex, face, and solid id.
    pub fn mvfs(self: &mut Model<S>, point: Vector3<S>) -> (VertexId, FaceId, SolidId) {
        let vertex_id = self.insert_vertex(Vertex { point });

        let face_id = self.insert_face(Face {
            surface: NurbSurface3D::everything(),
            outer: BoundaryType::Vertex(vertex_id),
            holes: Vec::new(),
            shell: ShellId(0), // set later
        });
        let shell_id = self.insert_shell(Shell {
            faces: vec![face_id],
            solid: SolidId(0), // set later
        });
        let solid_id = self.insert_solid(crate::Solid {
            shells: vec![shell_id],
        });
        // backwards linking
        self.faces.get_mut(&face_id).unwrap().shell = shell_id;
        self.shells.get_mut(&shell_id).unwrap().solid = solid_id;
        (vertex_id, face_id, solid_id)
    }
}
