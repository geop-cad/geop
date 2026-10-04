//! 2-D engineering drawings of a part: projected views with hidden lines
//! and silhouettes ([`hidden_lines`]), laid out on a sheet with dimensions,
//! centre marks and a title block ([`sheet`]), written as SVG or DXF, and
//! the [`Drawing`] operation of a program, which describes one.

pub mod hidden_lines;
pub mod silhouette;
pub mod view;

pub use hidden_lines::{LineKind, ProjectedView, ViewLine, ViewOptions, project_view};
pub use view::{ViewFrame, ViewKind};

use geop_core_math::scalars::Scalar;

/// Bounds how hard a containment or intersection search tries: effort, not
/// what an answer means.
pub(crate) const MAX_NODES: usize = 20_000;

/// Where a subdivision search hands over (see `AGENTS.md`).
pub(crate) fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(1e-7)
}

#[cfg(test)]
mod tests;
