//! What a step says of a solid it built beyond its shape, kept as the
//! part's target [`solid_notes`]: a solid only drawn, which is no body (a
//! harness' swept bundle), the thickness of a sheet-metal body, the wires
//! ordered in a bundle's place. Whoever weighs, checks or orders a part
//! reads them here, knowing no step that writes them.

use std::collections::BTreeMap;

use geop_core_math::scalars::Scalar;
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::Reader,
    target::TargetReference,
    values::Design,
};

/// What a step says of a solid it built beyond its shape (see
/// [`solid_notes`]).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SolidNote {
    /// Whether it only shows something that is no body — a harness'
    /// swept bundle — so that it is neither weighed, checked for
    /// interference nor ordered as one .
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

/// Read whole.
impl Kind for SolidNotes {
    const NAME: &'static str = "solid_notes";
}

impl HasElements<SolidNotes> for SolidNotes {
    type Element<'a> = &'a SolidNotes;
    const SINGLETON: Option<&'static str> = Some("solid_notes");

    fn element(&self, name: &str) -> geop_core_math::geop_error::GeopResult<&SolidNotes> {
        whole(self, name)
    }
}

/// What the steps of the part `reader` reads said of its solids: nothing,
/// if none said anything.
pub fn notes_of<'a>(reader: &Reader<'a>) -> Option<&'a SolidNotes> {
    reader.defines(&solid_notes()).then_some(())?;
    reader
        .retrieve_element(&ElementReference::<SolidNotes, SolidNotes>::whole(
            solid_notes(),
        ))
        .ok()
}
