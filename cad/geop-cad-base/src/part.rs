//! A part as the editor composes it from what each crate gives: the part
//! as drawn ([`draw`], every crate drawing its own targets — see
//! [`geop_ops_assembly::draw`]) and the structure an editor lists
//! ([`structure`]).

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::{ElementReference, PickedElement},
    reader::Reader,
    values::Design,
};
pub use geop_ops_assembly::draw;
use geop_ops_assembly::placed_parts;
use geop_ops_datums::{ORIGIN, read_datums};
use geop_ops_model::read_model;
use geop_ops_sketch::PlacedSketch;
use geop_ops_view::listed_by_steps;
use serde::Serialize;

/// An entry of the structure of the part drawn, by name: what [`structure`]
/// lists.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StructureItem {
    /// What it is: `datum`, `sketch`, `solid`, `face` (standing on its own,
    /// of no solid), `part` (placed), or what a step lists it as.
    pub kind: String,
    pub name: String,
    /// Whether it is shown, if it is drawn at all: what an editor's
    /// visibility switches. None for what a step lists, which is not drawn.
    pub visible: Option<bool>,
    /// What it refers to: the elements a mate is between, lit while it is
    /// pointed at.
    pub elements: Vec<PickedElement>,
}

/// What the part `reader` reads has beyond the faces of its solids, each
/// shown or not as `hidden` says: its datums, sketches, solids, faces
/// standing on their own and the parts placed in it — and whatever its
/// steps list.
pub fn structure(reader: &Reader, hidden: &[String]) -> GeopResult<Vec<StructureItem>> {
    let item = |kind: &str, name: &str| StructureItem {
        kind: kind.to_string(),
        name: name.to_string(),
        visible: Some(!hidden.iter().any(|h| h == name)),
        elements: Vec::new(),
    };
    let model = read_model(reader)?;
    let mut items = vec![item("datum", ORIGIN)];
    items.extend((read_datums(reader)?.names().into_iter()).map(|n| item("datum", n)));
    for sketch in reader.targets_of::<PlacedSketch<Design>>() {
        let placed =
            reader.retrieve_element(&ElementReference::<_, PlacedSketch<Design>>::whole(sketch))?;
        items.push(item("sketch", &placed.name));
    }
    items.extend((model.solid_names().iter()).map(|n| item("solid", n)));
    items.extend((model.sheet_face_names().iter()).map(|n| item("face", n)));
    let placed = placed_parts(reader).unwrap_or_default();
    items.extend(
        (placed.iter())
            .filter(|p| p.parent.is_none())
            .map(|p| item("part", &p.name)),
    );
    items.extend(
        listed_by_steps(reader)
            .into_iter()
            .map(|listed| StructureItem {
                kind: listed.kind,
                name: listed.name,
                visible: None,
                elements: listed.elements,
            }),
    );
    Ok(items)
}
