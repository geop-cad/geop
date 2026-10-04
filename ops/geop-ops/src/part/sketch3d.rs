//! A [`Part`]'s 3-D sketches: points and curves in space, named like any
//! other entity — the paths sweeps run along, and the rails that guide
//! them.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_sketch::space::Sketch3d;

use super::Part;
use super::ids::Sketch3dId;
use crate::Design;

impl<S: Scalar> Part<S> {
    /// Adds `sketch` to the part under `name` — solved, with the geometry
    /// of its references handed in (see [`Sketch3d::references`]). Fails,
    /// leaving the part unchanged, if `name` is already taken.
    pub fn add_sketch3d(
        &mut self,
        sketch: Sketch3d<Design>,
        name: impl Into<String>,
    ) -> GeopResult<Sketch3dId> {
        let id = Sketch3dId(self.fresh_id());
        self.names.insert(id, name)?;
        self.sketches3d.insert(id, sketch);
        Ok(id)
    }

    pub fn sketch3d(&self, id: Sketch3dId) -> GeopResult<&Sketch3d<Design>> {
        self.sketches3d
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("Part has no 3-D sketch {id}")))
    }

    /// Every 3-D sketch, in the order they were added.
    pub fn sketches3d(&self) -> impl Iterator<Item = (Sketch3dId, &Sketch3d<Design>)> {
        self.sketches3d.iter().map(|(&id, s)| (id, s))
    }
}
