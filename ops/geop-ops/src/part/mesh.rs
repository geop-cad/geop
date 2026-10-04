//! A part's solids as triangle meshes, for files that hold meshes (STL): its
//! own and those of the parts placed in it, however deep, each moved to
//! where it is placed.

use std::{collections::HashMap, sync::Arc};

use geop_core_math::{
    geop_error::GeopResult,
    primitives::{Pose, TriangleFace},
    scalars::Scalar,
};
use geop_ops_rasterize::rasterize;

use super::{Component, Part};
use crate::operation::INSTANCE_SEPARATOR;

/// Solids, each by name with its triangles.
pub type SolidMeshes<S> = Vec<(String, Vec<TriangleFace<S>>)>;

impl<S: Scalar> Part<S> {
    /// Every solid of the part and of the parts placed in it, by name — a
    /// placed part's behind its instance's name — as triangles in the part's
    /// frame, meshed `quality` fine (see [`rasterize`]), in name order so a
    /// file written from them does not depend on how the part stores them.
    /// A component placed many times is meshed once.
    pub fn solid_meshes(&self, quality: usize) -> GeopResult<SolidMeshes<S>> {
        let mut out = Vec::new();
        let own = self.own_solid_meshes(quality)?;
        self.placed_solid_meshes(
            &own,
            &Pose::identity(),
            "",
            quality,
            &mut HashMap::new(),
            &mut out,
        )?;
        out.sort_by(|a, b| a.0.cmp(&b.0));
        Ok(out)
    }

    /// Every solid of the part itself — not of the parts placed in it — by
    /// name, as triangles in its own frame.
    fn own_solid_meshes(&self, quality: usize) -> GeopResult<SolidMeshes<S>> {
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
        Ok(out)
    }

    /// The part's own solids `own`, and those of the parts placed in it,
    /// moved by `pose` and named behind `prefix`, into `out`. `meshed` keeps
    /// each component's own solids, by the component.
    fn placed_solid_meshes(
        &self,
        own: &SolidMeshes<S>,
        pose: &Pose<S>,
        prefix: &str,
        quality: usize,
        meshed: &mut HashMap<*const Component<S>, Arc<SolidMeshes<S>>>,
        out: &mut SolidMeshes<S>,
    ) -> GeopResult<()> {
        let motion = pose.motion();
        let place = |t: &TriangleFace<S>| TriangleFace {
            a: motion.apply(&t.a),
            b: motion.apply(&t.b),
            c: motion.apply(&t.c),
            normal: motion.rotate(&t.normal),
            vertex_normals: t.vertex_normals.map(|ns| ns.map(|n| motion.rotate(&n))),
        };
        for (name, triangles) in own {
            out.push((
                format!("{prefix}{name}"),
                triangles.iter().map(place).collect(),
            ));
        }
        for (id, instance) in self.instances() {
            let name = self.name_of(id).unwrap_or_default();
            let prefix = format!("{prefix}{name}{INSTANCE_SEPARATOR}");
            let key = Arc::as_ptr(&instance.component);
            if !meshed.contains_key(&key) {
                meshed.insert(key, Arc::new(instance.part().own_solid_meshes(quality)?));
            }
            let inner = meshed[&key].clone();
            instance.part().placed_solid_meshes(
                &inner,
                &pose.compose(&instance.pose),
                &prefix,
                quality,
                meshed,
                out,
            )?;
        }
        Ok(())
    }
}
