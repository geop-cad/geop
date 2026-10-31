//! [`AssemblyDrawer`]: the parts placed in a part, drawn as instances of
//! their components and listed by name.

use geop_core_editor::{
    registry::{Drawer, StructureItem},
    view::PartView,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    placed::{Located, split_instance_path},
    reader::Reader,
};

use geop_ops_model::placed::{drawn, placed_parts};

use crate::placed::draw_instances;

/// Draws the parts placed in a part, however deep, each as an instance of
/// its component where the state has it (see [`draw_instances`]), and lists
/// them all, however deep, by their paths.
pub struct AssemblyDrawer;

impl Drawer for AssemblyDrawer {
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()> {
        draw_instances(reader, view)
    }

    /// Each part placed in it, however deep — each before those placed in
    /// it, by its path, `neck/head` — said beside it which file it is from,
    /// unless its name says so already.
    fn structure(&self, reader: &Reader) -> GeopResult<Vec<StructureItem>> {
        let placed = placed_parts(reader).unwrap_or_default();
        Ok((placed.iter())
            .map(|p| {
                let item = StructureItem::drawn("part", "Placed parts", &p.name);
                let file = p.located.instance.file.as_str();
                let file = file.rsplit(['/', ':']).next().unwrap_or(file);
                let stem = file.strip_suffix(".geop").unwrap_or(file);
                match stem == split_instance_path(&p.name).1 {
                    true => item,
                    false => item.detailed(stem.to_string()),
                }
            })
            .collect())
    }

    /// The datums of the parts placed: theirs, not the part's, shown only
    /// while a field waits to pick one — a mate's entities.
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
