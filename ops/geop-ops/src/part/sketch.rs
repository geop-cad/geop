//! A [`Part`]'s sketches: named like any other entity, and stored with the
//! plane they were placed on.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
};
use geop_core_sketch::Sketch;

use super::Part;
use super::ids::SketchId;

/// A sketch together with the plane it lies on: `plane.u`/`plane.v` are the
/// sketch's `x`/`y` (unit, orthogonal), `plane.w = u x v` its unit normal.
///
/// The plane is resolved once, when the sketch is added — a sketch drawn on a
/// face stays where that face *was*, even after a later operation reshapes or
/// consumes the face.
#[derive(Clone, Debug)]
pub struct PlacedSketch<S: Scalar> {
    pub plane: CoordinateSystem<S>,
    pub sketch: Sketch,
}

impl<S: Scalar> Part<S> {
    /// Adds `placed` to the part under `name`. Fails, leaving the part
    /// unchanged, if `name` is already taken.
    pub fn add_sketch(
        &mut self,
        placed: PlacedSketch<S>,
        name: impl Into<String>,
    ) -> GeopResult<SketchId> {
        let id = SketchId(self.fresh_id());
        self.names.insert(id, name)?;
        self.sketches.insert(id, placed);
        Ok(id)
    }

    pub fn remove_sketch(&mut self, id: SketchId) -> GeopResult<()> {
        if self.sketches.remove(&id).is_none() {
            return Err(GeopError::new(format!("Part has no sketch {id}")));
        }
        self.names.remove(id);
        Ok(())
    }

    pub fn sketch(&self, id: SketchId) -> GeopResult<&PlacedSketch<S>> {
        self.sketches
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("Part has no sketch {id}")))
    }

    /// Every sketch, in the order they were added.
    pub fn sketches(&self) -> impl Iterator<Item = (SketchId, &PlacedSketch<S>)> {
        self.sketches.iter().map(|(&id, s)| (id, s))
    }
}
