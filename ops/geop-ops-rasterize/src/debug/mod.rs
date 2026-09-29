//! Rendering a model for a person to look at while debugging: a
//! [`PrimitiveScene`] of points, lines, triangles and labels, saved as an
//! interactive HTML file — the shape as rasterized ([`RasterizedModel::scene`])
//! or its raw topology, every entity labelled ([`rasterize_topology`]).

mod color;
mod line;
mod scene;
mod topology;

pub use color::Color10;
pub use line::Line;
pub use scene::{PrimitiveScene, PrimitiveSceneRecorder};
pub use topology::rasterize_topology;

use geop_core_math::scalars::Scalar;
use geop_core_topology::FaceId;

use crate::RasterizedModel;

impl<S: Scalar> RasterizedModel<S> {
    /// As a scene: vertices dark gray, edges gray, each face in the color
    /// `face_color` gives it.
    pub fn scene(&self, face_color: impl Fn(FaceId) -> Color10) -> PrimitiveScene<S> {
        let mut scene = PrimitiveScene::new();
        for &point in self.vertices.values() {
            scene.add_point(point, Color10::DarkGray);
        }
        for polyline in self.edges.values() {
            scene.add_polyline(polyline, Color10::Gray);
        }
        for (&id, triangles) in &self.faces {
            for t in triangles {
                scene.add_triangle(t.clone(), face_color(id));
            }
        }
        scene
    }
}
