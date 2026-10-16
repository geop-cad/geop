//! Sketches as an operation of a program: [`AddSketch`] places a sketch on
//! a plane of the part ([`SketchPlane`]), and edits it —
//! drawing with tools that snap by constraints, constraining, dragging (see
//! [`editor`]) — with reference geometry: its own origin and axes, and what
//! it projects of the part (see [`references`]). The sketch it draws is a
//! target of the part ([`placed`]), and what other steps build on.

mod add_sketch;
mod constraints;
pub mod editor;
pub mod fields;
mod geometry;
mod icons;
pub mod placed;
#[cfg(test)]
mod program_tests;
pub mod references;

pub use add_sketch::{AddSketch, AddSketchArgs, SketchPlane};
pub use placed::{
    PlacedSketch, SketchRef, chain_of_pieces, piece_names, shape_profiles, sketch, sketch_element,
    whole_sketch,
};

use geop_core_editor::{
    registry::{Drawer, Registry, StructureItem},
    view::PartView,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{element::ElementReference, reader::Reader, values::Design};

/// Draws a part's sketches, each as the target it is, and lists them.
pub struct SketchDrawer;

impl Drawer for SketchDrawer {
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()> {
        for target in reader.targets_of::<PlacedSketch<Design>>() {
            let name = target.name().to_string();
            let reference = ElementReference::<_, PlacedSketch<Design>>::whole(target);
            reader.retrieve_element(&reference)?.draw(&name, view)?;
        }
        Ok(())
    }

    fn structure(&self, reader: &Reader) -> GeopResult<Vec<StructureItem>> {
        (reader.targets_of::<PlacedSketch<Design>>().into_iter())
            .map(|target| {
                let reference = ElementReference::<_, PlacedSketch<Design>>::whole(target);
                let placed = reader.retrieve_element(&reference)?;
                Ok(StructureItem::drawn("sketch", "Sketches", &placed.name))
            })
            .collect()
    }
}

/// Adds what sketches offer an editor to `r`: the step adding one, and a
/// part's sketches drawn and listed ([`SketchDrawer`]).
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(AddSketch)?.drawer(SketchDrawer)
}

/// A sketch as a program holds it.
pub type Sketch = geop_core_sketch::Sketch<Design>;
/// A sketch constraint as a program holds it.
pub type Constraint = geop_core_sketch::Constraint<Design>;
/// A sketch curve's kind as a program holds it.
pub type CurveKind = geop_core_sketch::CurveKind<Design>;
