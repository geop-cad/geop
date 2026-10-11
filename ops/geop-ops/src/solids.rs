//! The solids a part shows, wherever they are placed — its own, and those
//! of the parts placed in it, however deep — each of its own part's
//! material: what weighing a part, checking it for interference, ordering
//! it and simulating it read.
//!
//! Lengths are millimetres and densities kg/m³ (see
//! [`crate::parameters::Material`]), so a mass is in kilograms.

use std::collections::HashMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::{Field, Scalar},
};
use geop_core_topology::{SolidId, mass::MassProperties};

use crate::{
    Design, NamedModel,
    model::model,
    operation::{BuildView, INSTANCE_SEPARATOR},
    placed::{MaterialOf, about, placed_parts},
};

/// A solid of a part, or of a part placed in it: named as the part names it
/// (`pin/extrude(pin)` for one of the part placed as `pin`), where it is.
pub struct PlacedSolid<'a> {
    pub name: String,
    /// The part it is a solid of — whose material it is made of.
    pub view: BuildView<'a>,
    pub model: &'a NamedModel<Design>,
    pub solid: SolidId,
    /// Where its part is placed; `None` for the part's own solids.
    pub pose: Option<Pose<Design>>,
}

impl PlacedSolid<'_> {
    /// What it is made of: its part's material.
    pub fn material(&self) -> MaterialOf {
        about(self.view).material()
    }

    /// Its mass properties, where it is, of its part's material — its
    /// shape's taken from `shapes`.
    pub fn mass_properties(&self, shapes: &mut Shapes) -> GeopResult<MassProperties<Design>> {
        let ctx = |e: GeopError| e.with_context(format!("the mass of {}", self.name));
        let per_mm3 = Design::from_f64(self.material().density)
            .div(Design::from_f64(1e9))
            .with_context(&ctx)?;
        let own = shapes.of(self).with_context(&ctx)?.with_density(per_mm3);
        match &self.pose {
            Some(pose) => own.placed(pose).with_context(&ctx),
            None => Ok(own),
        }
    }
}

/// The mass properties of solids' shapes — of unit density, each in its
/// own part's frame — each integrated once, however often its part is
/// placed.
#[derive(Default)]
pub struct Shapes(HashMap<(*const NamedModel<Design>, SolidId), MassProperties<Design>>);

impl Shapes {
    /// The mass properties of the shape of `solid`.
    pub fn of(&mut self, solid: &PlacedSolid) -> GeopResult<MassProperties<Design>> {
        let key = (std::ptr::from_ref(solid.model), solid.solid);
        if let Some(found) = self.0.get(&key) {
            return Ok(*found);
        }
        let found = solid.model.topology().mass_properties(solid.solid)?;
        self.0.insert(key, found);
        Ok(found)
    }
}

/// The solids of the part `view` shows itself — not of the parts placed in
/// it — the oldest first, each named behind `prefix` and placed at `pose`.
fn own<'a>(
    view: BuildView<'a>,
    prefix: &str,
    pose: Option<Pose<Design>>,
) -> GeopResult<Vec<PlacedSolid<'a>>> {
    // A part with no model has no solids.
    let Ok(model) = view.retrieve_target(&model()) else {
        return Ok(Vec::new());
    };
    (model.solid_names().into_iter())
        .map(|name| {
            Ok(PlacedSolid {
                solid: model.solid_id(&name)?,
                name: format!("{prefix}{name}"),
                view,
                model,
                pose,
            })
        })
        .collect()
}

/// The solids of the part `view` shows itself, the oldest first: not those
/// of the parts placed in it.
pub fn own_solids(view: BuildView<'_>) -> GeopResult<Vec<PlacedSolid<'_>>> {
    own(view, "", None)
}

/// Every solid of the part `view` shows and of the parts placed in it,
/// however deep, each where it is: the part's own first, then those of each
/// part placed, in the order they were placed.
pub fn placed_solids(view: BuildView<'_>) -> GeopResult<Vec<PlacedSolid<'_>>> {
    let mut out = own_solids(view)?;
    for placed in placed_parts(view)? {
        let prefix = format!("{}{INSTANCE_SEPARATOR}", placed.name);
        let inner = placed.located.instance.component.view();
        out.extend(own(inner, &prefix, Some(placed.located.world))?);
    }
    Ok(out)
}
