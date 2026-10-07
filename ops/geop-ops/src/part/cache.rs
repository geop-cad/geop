//! What is worked out from a [`Part`] once it is placed — in another part,
//! or in a bill of materials — and kept for as long as the part is: its
//! view, the box around it and its solids' mass properties. A part placed
//! a hundred times is shared (see [`super::Instance`]), so each is
//! worked out once, not once per instance.

use std::{
    collections::HashMap,
    sync::{Mutex, OnceLock, PoisonError},
};

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};
use geop_core_topology::{SolidId, mass::MassProperties};

use super::Part;
use crate::ui::PartView;

/// What is kept of a part (see the module). A copy of a part starts with
/// none: it is a part about to be changed, and what was worked out of the
/// original would be stale.
pub(super) struct Cache<S: Scalar> {
    view: OnceLock<PartView<S>>,
    bounds: OnceLock<Option<[Vector3<S>; 2]>>,
    /// Its solids' mass properties, of density one, as far as asked for.
    mass: Mutex<HashMap<SolidId, MassProperties<S>>>,
}

impl<S: Scalar> Cache<S> {
    pub(super) fn new() -> Self {
        Self {
            view: OnceLock::new(),
            bounds: OnceLock::new(),
            mass: Mutex::new(HashMap::new()),
        }
    }
}

impl<S: Scalar> Clone for Cache<S> {
    fn clone(&self) -> Self {
        Self::new()
    }
}

impl<S: Scalar> Part<S> {
    /// The mass properties of its solid `solid`, of density one (see
    /// [`geop_core_topology::Model::mass_properties`]): integrated the
    /// first time they are asked for, and kept for as long as the part is —
    /// every bill of materials, mass report and robot description of it,
    /// however often it is placed, integrates it once.
    pub fn mass_properties(&self, solid: SolidId) -> GeopResult<MassProperties<S>> {
        let kept = || {
            self.cache
                .mass
                .lock()
                .unwrap_or_else(PoisonError::into_inner)
        };
        if let Some(&properties) = kept().get(&solid) {
            return Ok(properties);
        }
        let properties = self.topology.mass_properties(solid)?;
        kept().insert(solid, properties);
        Ok(properties)
    }

    /// The box around every vertex of the part and of the parts placed in
    /// it, as placed — `None` for a part with none: found once, and kept
    /// for as long as the part is (see [`crate::assembly`]).
    pub fn bounds(&self) -> Option<[Vector3<S>; 2]> {
        *self
            .cache
            .bounds
            .get_or_init(|| crate::assembly::bounds(self))
    }

    /// The part as drawn, in its own frame: rasterized once, the first time
    /// it is asked for, and kept for as long as the part is. Without its
    /// sketches: what a part was drawn with is its own business, not that
    /// of the parts it is placed in.
    pub fn view(&self) -> GeopResult<&PartView<S>> {
        if let Some(view) = self.cache.view.get() {
            return Ok(view);
        }
        let mut view = PartView::of(self)?;
        view.sketches.clear();
        Ok(self.cache.view.get_or_init(|| view))
    }
}
