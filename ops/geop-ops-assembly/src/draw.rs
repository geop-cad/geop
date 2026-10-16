//! [`AssemblyDrawer`]: the parts placed in a part, drawn as instances of
//! their components and listed by name.

use geop_core_editor::{
    registry::{Drawer, StructureItem},
    view::PartView,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{placed::Located, reader::Reader};

use crate::placed::{draw_instances, drawn, placed_parts};

/// Draws the parts placed in a part, however deep, each as an instance of
/// its component where the state has it (see [`draw_instances`]), and lists
/// those placed in the part itself.
pub struct AssemblyDrawer;

impl Drawer for AssemblyDrawer {
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()> {
        draw_instances(reader, view)
    }

    fn structure(&self, reader: &Reader) -> GeopResult<Vec<StructureItem>> {
        let placed = placed_parts(reader).unwrap_or_default();
        Ok((placed.iter())
            .filter(|p| p.parent.is_none())
            .map(|p| StructureItem::drawn("part", "Placed parts", &p.name))
            .collect())
    }

    /// The datums of the parts placed: theirs, not the part's, shown only
    /// while a part is being placed, for its mates to pick.
    fn hidden(&self, reader: &Reader) -> Vec<String> {
        let Ok(placed) = placed_parts(reader) else {
            return Vec::new();
        };
        (placed.iter())
            .filter_map(|part| Some((part, drawn(reader, &part.name).ok()?)))
            .flat_map(|(part, drawn)| {
                (drawn.view.datums.iter())
                    .map(|d| Located::in_instance(part.name.as_str(), d.name.as_str()).key())
            })
            .collect()
    }
}
