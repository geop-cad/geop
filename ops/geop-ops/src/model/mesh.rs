//! A model's solids as triangle meshes, for files that hold meshes (STL).

use geop_core_math::{geop_error::GeopResult, primitives::TriangleFace, scalars::Scalar};
use geop_ops_rasterize::rasterize;

use super::NamedModel;

/// Solids, each by name with its triangles.
pub type SolidMeshes<S> = Vec<(String, Vec<TriangleFace<S>>)>;

impl<S: Scalar> NamedModel<S> {
    /// Every solid, by name, as triangles, meshed `quality` fine (see
    /// [`rasterize`]), in name order so a file written from them does not
    /// depend on how the model stores them.
    pub fn solid_meshes(&self, quality: usize) -> GeopResult<SolidMeshes<S>> {
        let model = self.topology();
        let raster = rasterize(model, quality)?;
        let mut out = Vec::new();
        for &solid in model.solids.keys() {
            let mut faces = model.solid_faces(solid)?;
            faces.sort_by_key(|f| f.0);
            let triangles = faces
                .iter()
                .filter_map(|f| raster.faces.get(f))
                .flatten()
                .cloned()
                .collect();
            let name = self.name_of(solid).unwrap_or_default();
            out.push((name.to_string(), triangles));
        }
        out.sort_by(|a, b| a.0.cmp(&b.0));
        Ok(out)
    }
}
