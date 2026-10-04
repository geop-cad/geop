//! 2-D engineering drawings of a part: projected views with hidden lines
//! and silhouettes ([`hidden_lines`]), laid out on a sheet with dimensions,
//! centre marks and a title block ([`sheet`]), written as SVG or DXF, and
//! the [`Drawing`] operation of a program, which describes one.

pub mod drawing;
pub mod dxf;
pub mod hidden_lines;
pub mod operation;
pub mod scene;
pub mod section;
pub mod sheet;
pub mod silhouette;
pub mod svg;
pub mod view;

pub use drawing::{Dimension, DrawingArgs, PartsListLine, Projection, SheetSize, compose};
pub use dxf::to_dxf;
pub use hidden_lines::{LineKind, ProjectedView, ViewLine, ViewOptions, project_view};
pub use operation::Drawing;
pub use svg::to_svg;
pub use view::{ViewFrame, ViewKind};

use geop_core_math::scalars::Scalar;

/// Bounds how hard a containment or intersection search tries: effort, not
/// what an answer means.
pub(crate) const MAX_NODES: usize = 20_000;

/// Where a subdivision search hands over (see `AGENTS.md`).
pub(crate) fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(1e-7)
}

/// A file format a drawing is written in.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Format {
    Svg,
    Dxf,
}

impl Format {
    /// The format a file name's extension names, if it names one.
    pub fn of_path(path: &str) -> Option<Format> {
        let ext = path.rsplit('.').next()?.to_ascii_lowercase();
        match ext.as_str() {
            "svg" => Some(Format::Svg),
            "dxf" => Some(Format::Dxf),
            _ => None,
        }
    }

    pub fn extension(self) -> &'static str {
        match self {
            Format::Svg => "svg",
            Format::Dxf => "dxf",
        }
    }
}

/// `part`'s drawing as `args` describe it, dated `date`, with `parts` its
/// bill of materials if `args` asks for one, written as `format`.
pub fn render<S: Scalar>(
    part: &geop_ops::Part<S>,
    args: &DrawingArgs,
    date: &str,
    parts: &[PartsListLine],
    format: Format,
) -> geop_core_math::geop_error::GeopResult<String> {
    let sheet = compose(part, args, date, parts)?;
    Ok(match format {
        Format::Svg => to_svg(&sheet),
        Format::Dxf => to_dxf(&sheet),
    })
}

#[cfg(test)]
mod tests;
#[cfg(test)]
mod writer_tests;
