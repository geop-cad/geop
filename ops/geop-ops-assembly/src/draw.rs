//! [`AssemblyDrawer`]: the parts placed in a part, drawn as instances of
//! their components and listed by name.

use geop_core_editor::{
    registry::{Drawer, StructureItem},
    view::PartView,
};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::{placed::Located, reader::Reader};

use geop_ops_model::{
    parameters::instances,
    placed::{drawn, placed_parts},
};
use std::collections::BTreeMap;

use crate::placed::draw_instances;

/// Draws the parts placed in a part, however deep, each as an instance of
/// its component where the state has it (see [`draw_instances`]), and lists
/// those placed in the part itself.
pub struct AssemblyDrawer;

impl Drawer for AssemblyDrawer {
    fn draw(&self, reader: &Reader, view: &mut PartView) -> GeopResult<()> {
        draw_instances(reader, view)
    }

    /// Each part placed in it, called after its file too: `base: part2`.
    fn structure(&self, reader: &Reader) -> GeopResult<Vec<StructureItem>> {
        let placed = placed_parts(reader).unwrap_or_default();
        let files: BTreeMap<&str, &str> = (instances(reader).unwrap_or_default().into_iter())
            .map(|(name, instance)| (name, instance.file.as_str()))
            .collect();
        Ok((placed.iter())
            .filter(|p| p.parent.is_none())
            .map(|p| {
                let item = StructureItem::drawn("part", "Placed parts", &p.name);
                let Some(file) = files.get(p.name.as_str()) else {
                    return item;
                };
                let file = file.rsplit('/').next().unwrap_or(file);
                let stem = file.strip_suffix(".geop").unwrap_or(file);
                item.labelled(format!("{stem}: {}", p.name))
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
