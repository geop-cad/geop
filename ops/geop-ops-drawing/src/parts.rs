//! [`parts_list`]: the bill of materials a drawing lists, from the part's
//! own (see [`geop_ops_bom`]).

use geop_core_math::geop_error::GeopResult;
use geop_core_program::reader::Reader;
use geop_ops_model::placed::bom::{LineKind, Structure, bom};

use crate::{DrawingArgs, PartsListLine};

/// The bill of materials of the part `reader` reads, the program of the
/// file `file`, as a drawing lists it (see [`PartsListLine`]): flat, each
/// part and wire once with its count. Empty if `args` asks for none.
pub fn parts_list(
    reader: &Reader,
    file: &str,
    args: &DrawingArgs,
) -> GeopResult<Vec<PartsListLine>> {
    if !args.bom {
        return Ok(Vec::new());
    }
    let bom = bom(reader, file, Structure::Flat)?;
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
