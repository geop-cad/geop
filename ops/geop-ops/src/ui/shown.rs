//! What a step shows of the part besides its model, for an editor to
//! collect without knowing the step's operation: annotations drawn on the
//! part ([`annotations`]) and entries of its structure ([`listing`]) — each
//! a target of the step's own, which the editor finds by its type.

use geop_core_math::geop_error::GeopResult;
use geop_core_program::target::TargetReference;
use serde::Serialize;

use super::ViewAnnotation;
use crate::{
    EntityRef, ORIGIN,
    operation::{BuildReader, BuildView, Lookup, Seen},
    placed::placed_parts,
};

/// What a step draws on the part besides its topology — cosmetic threads —
/// the value of its target [`annotations`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Annotations(pub Vec<ViewAnnotation>);

/// What the step `step` draws on the part: defined by a step that draws
/// something, and drawn by [`super::PartView::of`] for every step that
/// does.
pub fn annotations(step: &str) -> TargetReference<Annotations> {
    TargetReference::new(format!("annotations:{step}"))
}

/// Everything the steps `view` sees draw on the part, step by step.
pub fn annotations_of(view: BuildView) -> Vec<ViewAnnotation> {
    (view.targets_of::<Annotations>().into_iter())
        .flat_map(|(_, drawn)| drawn.0.iter().cloned())
        .collect()
}

/// An entry of the part's structure a step adds beside what every part has
/// (see [`StructureItem`]): a mate, of kind `mate`.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Listed {
    /// What it is — the group a structure tree lists it in: `mate`.
    pub kind: String,
    pub name: String,
    /// What it refers to: the entities a mate is between, lit while it is
    /// pointed at.
    pub entities: Vec<EntityRef>,
}

/// The entries of the part's structure a step adds: the value of its
/// target [`listing`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Listing(pub Vec<Listed>);

/// The entries of the part's structure the step `step` adds.
pub fn listing(step: &str) -> TargetReference<Listing> {
    TargetReference::new(format!("listing:{step}"))
}

/// The generator of a [`listing`] that is the entries it is given: what a
/// step that knows its entries from its arguments defines it with.
pub fn listed(_: &mut BuildReader, listing: &Listing) -> GeopResult<Listing> {
    Ok(listing.clone())
}

/// An entry of the structure of the part drawn, by name: what
/// [`structure`] lists.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct StructureItem {
    /// What it is: `datum`, `sketch`, `solid`, `face` (standing on its own,
    /// of no solid), `part` (placed), or what a step lists it as
    /// ([`Listed::kind`]).
    pub kind: String,
    pub name: String,
    /// Whether it is shown, if it is drawn at all: what an editor's
    /// visibility switches. None for what a step lists, which is not drawn.
    pub visible: Option<bool>,
    /// What it refers to (see [`Listed::entities`]).
    pub entities: Vec<EntityRef>,
}

/// What the part `view` shows has beyond the faces of its solids, each
/// shown or not as `hidden` says: its datums, sketches, solids, faces
/// standing on their own and the parts placed in it — and whatever its
/// steps list ([`listing`]).
pub fn structure(view: BuildView, hidden: &[String]) -> Vec<StructureItem> {
    let seen = Seen::new(view);
    let item = |kind: &str, name: &str| StructureItem {
        kind: kind.to_string(),
        name: name.to_string(),
        visible: Some(!hidden.iter().any(|h| h == name)),
        entities: Vec::new(),
    };
    let model = seen.model();
    let mut items = vec![item("datum", ORIGIN)];
    items.extend((seen.datums().into_iter()).map(|(n, _)| item("datum", n)));
    items.extend((seen.sketch_names().iter()).map(|n| item("sketch", n)));
    items.extend((model.solid_names().iter()).map(|n| item("solid", n)));
    items.extend((model.sheet_face_names().iter()).map(|n| item("face", n)));
    let placed = placed_parts(view).unwrap_or_default();
    items.extend(
        (placed.iter())
            .filter(|p| p.parent.is_none())
            .map(|p| item("part", &p.name)),
    );
    items.extend(
        (view.targets_of::<Listing>().into_iter())
            .flat_map(|(_, listing)| &listing.0)
            .map(|listed| StructureItem {
                kind: listed.kind.clone(),
                name: listed.name.clone(),
                visible: None,
                entities: listed.entities.clone(),
            }),
    );
    items
}
