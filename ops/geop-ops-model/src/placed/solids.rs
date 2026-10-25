//! The solids a part shows, wherever they are placed — its own, and those
//! of the parts placed in it, however deep — each of its own part's
//! material: what weighing a part, checking it for interference, ordering
//! it and simulating it read.
//!
//! What a step says of a solid beyond its shape (see [`crate::notes`]) is
//! read here: a solid only drawn is no body.
//!
//! Lengths are millimetres and densities kg/m³ (see
//! [`crate::parameters::Material`]), so a mass is in kilograms.

use geop_core_program::{placed::Located, reader::Reader, values::Design};
use std::collections::HashMap;

use crate::{
    NamedModel, notes_of,
    parameters::{MaterialOf, Properties, read_properties},
    placed::placed_parts,
    read_model,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::{Field, Scalar},
};
use geop_core_topology::{SolidId, mass::MassProperties};

/// A solid of a part, or of a part placed in it: named as the part names it
/// (`/pin:extrude(pin)` for one of the part placed as `pin`, see
/// [`Located::key`]), where it is.
pub struct PlacedSolid<'a> {
    pub name: String,
    /// The path of the part placed it is of: empty for the part's own.
    pub instance: String,
    /// The part it is a solid of — whose material it is made of.
    pub reader: Reader<'a>,
    pub model: std::borrow::Cow<'a, NamedModel<Design>>,
    pub solid: SolidId,
    /// Where its part is placed; `None` for the part's own solids.
    pub pose: Option<Pose<Design>>,
}

impl PlacedSolid<'_> {
    /// The thickness of the sheet it is bent from (mm), for a sheet-metal
    /// body (see [`crate::SolidNote::thickness`]).
    pub fn thickness(&self) -> Option<f64> {
        let own = self.model.name_of(self.solid)?;
        notes_of(&self.reader)?.get(own)?.thickness
    }

    /// What it is made of: its part's material.
    pub fn material(&self) -> MaterialOf {
        (read_properties(&self.reader).map(|properties| properties.material()))
            .unwrap_or_else(|_| Properties::default().material())
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
        let key = (std::ptr::from_ref(&*solid.model), solid.solid);
        if let Some(found) = self.0.get(&key) {
            return Ok(*found);
        }
        let found = solid.model.topology().mass_properties(solid.solid)?;
        self.0.insert(key, found);
        Ok(found)
    }
}

/// The solids of the part `reader` reads itself — not of the parts placed in
/// it, and none only drawn (see [`crate::SolidNote::drawn_only`]) — the oldest
/// first, each named behind `prefix` and placed at `pose`: as the part
/// placed there at the path `instance` has them.
pub fn own_solids_at<'a>(
    reader: &Reader<'a>,
    instance: &str,
    pose: Option<Pose<Design>>,
) -> GeopResult<Vec<PlacedSolid<'a>>> {
    let model = read_model(reader)?;
    let notes = notes_of(reader);
    let drawn_only =
        |name: &str| (notes.and_then(|notes| notes.get(name))).is_some_and(|note| note.drawn_only);
    (model.solid_names().into_iter())
        .filter(|name| !drawn_only(name))
        .map(|name| {
            Ok(PlacedSolid {
                solid: model.solid_id(&name)?,
                name: Located::in_instance(instance, name.as_str()).key(),
                instance: instance.to_string(),
                reader: reader.clone(),
                model: model.clone(),
                pose,
            })
        })
        .collect()
}

/// The bodies of the part `reader` reads itself, the oldest first: not those
/// of the parts placed in it.
pub fn own_solids<'a>(reader: &Reader<'a>) -> GeopResult<Vec<PlacedSolid<'a>>> {
    own_solids_at(reader, "", None)
}

/// Every body of the part `reader` reads and of the parts placed in it,
/// however deep — every solid but those only drawn — each where it is:
/// the part's own first, then those of each part placed, in the order they
/// were placed.
pub fn placed_solids<'a>(reader: &Reader<'a>) -> GeopResult<Vec<PlacedSolid<'a>>> {
    let mut out = own_solids(reader)?;
    for placed in placed_parts(reader)? {
        let inner = placed.located.part.clone();
        out.extend(own_solids_at(
            &inner,
            &placed.name,
            Some(placed.located.world),
        )?);
    }
    Ok(out)
}
