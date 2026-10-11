//! 2-D engineering drawings of a part: projected views with hidden lines
//! and silhouettes ([`hidden_lines`]) — of an assembly, of every part
//! placed, where it is placed ([`scene`]) — laid out on a sheet with
//! dimensions, centre marks, a title block and a ballooned bill of
//! materials (see [`geop_ops::output::sheet`]); the [`Drawing`] operation
//! of a program, which describes one; and the [`DrawingExporter`], which
//! writes it as SVG or DXF.

pub mod annotation;
pub mod drawing;
mod export;
pub mod hidden_lines;
pub mod operation;
mod parts;
pub mod scene;
pub mod section;
pub mod silhouette;
pub mod view;

pub use annotation::{Along, Annotation, Candidate, EdgeShape, Leader, Pickable, Target};
pub use drawing::{
    DrawingArgs, Layout, PartsListLine, Projected, Projection, SheetSize, annotated, arrange,
    compose, layout, project,
};
pub use export::{DrawingExporter, DrawingOptions, drawing, drawing_steps};
pub use hidden_lines::{LineKind, ProjectedView, ViewLine, ViewOptions, project_view};
pub use operation::{Drawing, DrawingSession};
pub use parts::parts_list;
pub use view::{DrawnView, ViewFrame, ViewKind};

use geop_core_math::scalars::Scalar;

/// Bounds how hard a containment or intersection search tries: effort, not
/// what an answer means.
pub(crate) const MAX_NODES: usize = 20_000;

/// Where a subdivision search hands over (see `AGENTS.md`).
pub(crate) fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(1e-7)
}

#[cfg(test)]
mod annotation_tests;
#[cfg(test)]
mod testing;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod writer_tests;
