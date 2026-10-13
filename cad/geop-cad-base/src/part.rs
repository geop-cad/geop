//! A part as the editor composes it from what each crate gives: the run's
//! given step ([`given`]) — what the program says of its part, the files
//! around it, the parts it can place, the origin — the part as drawn
//! ([`draw`]), every crate drawing its own targets, and the structure an
//! editor lists ([`structure`]).

use std::rc::Rc;

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{element::ElementReference, program_runner::Given};
use geop_ops_assembly::{GivenLibrary, Library, define_library, draw_instances, placed_parts};
use geop_ops_datums::{ORIGIN, define_datums, read_datums};
use geop_ops_model::{Design, read_model};
use geop_ops_parameters::{
    About, Files, PartReader, Shared, StateValue,
    about::{define_about, define_files},
    read_about,
};
use geop_ops_sketch::PlacedSketch;
use geop_ops_view::{PartView, annotations_of, listed_by_steps};
use geop_core_program::element::PickedElement;
use serde::Serialize;

/// What a run of a program is given besides its numbers: what the program
/// says of its part, `about`; the files around it and the parts it can
/// place, `library`; and the origin, which every part has.
pub fn given(about: About, library: Rc<dyn Library>) -> Given<Design, StateValue> {
    let files: Shared<dyn Files> = Shared(library.clone());
    let library = GivenLibrary(library);
    Rc::new(move |part| {
        define_about(part, &about)?;
        define_files(part, &files)?;
        define_library(part, &library)?;
        define_datums(part)
    })
}

/// The part `reader` reads, drawn: its model, its datums — the origin
/// first — its sketches, what its steps draw on it and the parts placed in
/// it, in its colour.
pub fn draw(reader: &PartReader) -> GeopResult<PartView> {
    let mut view = PartView::new();
    read_model(reader)?.draw("model", &mut view)?;
    read_datums(reader)?.draw(&mut view)?;
    for target in reader.targets_of::<PlacedSketch<Design>>() {
        let name = target.name().to_string();
        let placed = reader.retrieve_element(&ElementReference::<_, PlacedSketch<Design>>::whole(target))?;
        placed.draw(&name, &mut view)?;
    }
    view.annotations = annotations_of(reader);
    draw_instances(reader, &mut view)?;
    view.color = read_about(reader)?.color().map(str::to_string);
    Ok(view.finish())
}

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
pub fn structure(reader: &PartReader, hidden: &[String]) -> GeopResult<Vec<StructureItem>> {
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
        let placed = reader.retrieve_element(&ElementReference::<_, PlacedSketch<Design>>::whole(sketch))?;
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
    items.extend(listed_by_steps(reader).into_iter().map(|listed| StructureItem {
        kind: listed.kind,
        name: listed.name,
        visible: None,
        elements: listed.elements,
    }));
    Ok(items)
}
