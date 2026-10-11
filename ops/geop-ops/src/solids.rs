//! The solids a part shows, wherever they are placed — its own, and those
//! of the parts placed in it, however deep — each of its own part's
//! material: what weighing a part, checking it for interference, ordering
//! it and simulating it read.
//!
//! What a step says of a solid it built beyond its shape is kept as the
//! part's target [`solid_notes`]: a solid only drawn, which is no body
//! (a harness' swept bundle), the thickness of a sheet-metal body, the
//! wires ordered in a bundle's place. Whoever weighs, checks or orders a
//! part reads them here, knowing no step that writes them.
//!
//! Lengths are millimetres and densities kg/m³ (see
//! [`crate::parameters::Material`]), so a mass is in kilograms.

use std::collections::{BTreeMap, HashMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::{Field, Scalar},
};
use geop_core_program::target::{Revisable, TargetReference};
use geop_core_topology::{SolidId, mass::MassProperties};

use crate::{
    Design, NamedModel,
    model::model,
    operation::{BuildView, INSTANCE_SEPARATOR},
    placed::{MaterialOf, about, placed_parts},
};

/// What a step says of a solid it built beyond its shape (see
/// [`solid_notes`]).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SolidNote {
    /// Whether it only shows something that is no body — a harness'
    /// swept bundle — so that it is neither weighed, checked for
    /// interference nor ordered as one (see [`placed_solids`]).
    pub drawn_only: bool,
    /// The thickness of the sheet it is bent from (mm), for a sheet-metal
    /// body or a flat pattern of one.
    pub thickness: Option<f64>,
    /// The wires ordered in its place, each cut to length: a harness'.
    pub wires: Vec<CutWire<Design>>,
}

/// A wire cut to length, as a bill of materials orders it.
#[derive(Clone, Debug, PartialEq)]
pub struct CutWire<S: Scalar> {
    pub name: String,
    /// Its colour, `#rrggbb`.
    pub colour: String,
    /// Its outer diameter, insulation included.
    pub diameter: f64,
    /// Its American wire gauge, if it was given by one.
    pub gauge: Option<i32>,
    /// The length to cut it to. An enclosure, as every measured length is.
    pub cut_length: S,
}

/// What the steps of a part said of its solids, by the solids' names: the
/// value of its target [`solid_notes`], which every step that says
/// something of a solid it built revises.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SolidNotes(BTreeMap<String, SolidNote>);

/// What the steps of the part said of its solids.
pub fn solid_notes() -> TargetReference<SolidNotes> {
    TargetReference::new("solid_notes")
}

/// A part has few notes: a revision copies them.
impl Revisable for SolidNotes {
    fn to_revise(&self) -> Self {
        self.clone()
    }
}

impl SolidNotes {
    /// The note on the solid `solid`, made empty if there is none yet.
    pub fn note_mut(&mut self, solid: impl Into<String>) -> &mut SolidNote {
        self.0.entry(solid.into()).or_default()
    }

    /// Forgets the note on the solid `solid`: one a step consumed.
    pub fn remove(&mut self, solid: &str) {
        self.0.remove(solid);
    }

    /// The note on the solid `solid`, if there is one.
    pub fn get(&self, solid: &str) -> Option<&SolidNote> {
        self.0.get(solid)
    }

    /// Every note, by its solid's name, in the order of the names.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &SolidNote)> {
        self.0.iter().map(|(name, note)| (name.as_str(), note))
    }
}

/// What the steps of the part `view` shows said of its solids: nothing, if
/// none said anything.
pub fn notes_of<'a>(view: BuildView<'a>) -> Option<&'a SolidNotes> {
    view.retrieve_target(&solid_notes()).ok()
}

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
    /// The thickness of the sheet it is bent from (mm), for a sheet-metal
    /// body (see [`SolidNote::thickness`]).
    pub fn thickness(&self) -> Option<f64> {
        let own = self.model.name_of(self.solid)?;
        notes_of(self.view)?.get(own)?.thickness
    }

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
/// it, and none only drawn (see [`SolidNote::drawn_only`]) — the oldest
/// first, each named behind `prefix` and placed at `pose`: as the part
/// placed there with the name `prefix` ends in has them.
pub fn own_solids_at<'a>(
    view: BuildView<'a>,
    prefix: &str,
    pose: Option<Pose<Design>>,
) -> GeopResult<Vec<PlacedSolid<'a>>> {
    // A part with no model has no solids.
    let Ok(model) = view.retrieve_target(&model()) else {
        return Ok(Vec::new());
    };
    let notes = notes_of(view);
    let drawn_only =
        |name: &str| (notes.and_then(|notes| notes.get(name))).is_some_and(|note| note.drawn_only);
    (model.solid_names().into_iter())
        .filter(|name| !drawn_only(name))
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

/// The bodies of the part `view` shows itself, the oldest first: not those
/// of the parts placed in it.
pub fn own_solids(view: BuildView<'_>) -> GeopResult<Vec<PlacedSolid<'_>>> {
    own_solids_at(view, "", None)
}

/// Every body of the part `view` shows and of the parts placed in it,
/// however deep — every solid but those only drawn — each where it is:
/// the part's own first, then those of each part placed, in the order they
/// were placed.
pub fn placed_solids(view: BuildView<'_>) -> GeopResult<Vec<PlacedSolid<'_>>> {
    let mut out = own_solids(view)?;
    for placed in placed_parts(view)? {
        let prefix = format!("{}{INSTANCE_SEPARATOR}", placed.name);
        let inner = placed.located.instance.component.view();
        out.extend(own_solids_at(inner, &prefix, Some(placed.located.world))?);
    }
    Ok(out)
}
