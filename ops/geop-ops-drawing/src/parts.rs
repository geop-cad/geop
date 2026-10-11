//! [`parts_list`]: the bill of materials a drawing lists, from the part's
//! own (see [`geop_ops_bom`]).

use geop_core_math::geop_error::GeopResult;
use geop_ops::operation::BuildView;
use geop_ops_bom::{LineKind, Structure, bom};

use crate::{DrawingArgs, PartsListLine};

/// The bill of materials of the part `view` shows, as a drawing lists it
/// (see [`PartsListLine`]): flat, each part and wire once with its count.
/// Empty if `args` asks for none.
pub fn parts_list(view: BuildView, args: &DrawingArgs) -> GeopResult<Vec<PartsListLine>> {
    if !args.bom {
        return Ok(Vec::new());
    }
    let bom = bom(view, Structure::Flat)?;
    Ok((bom.lines.into_iter())
        .map(|line| PartsListLine {
            item: line.item,
            quantity: line.quantity,
            name: line.name,
            designation: line.designation.unwrap_or_default(),
            material: match line.kind {
                LineKind::Part { material, .. } => material,
                LineKind::Wire { colour, .. } => format!("wire, {colour}"),
            },
            placements: line.placements,
        })
        .collect())
}
