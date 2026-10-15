//! Boolean operations on solids.
//!
//! `remesh` imprints the two solids' boundaries onto each other (splitting
//! edges at coincidences, tracing face x face intersection curves) so that
//! `boolean` can classify and keep the right faces for union, intersection,
//! or difference. Both work on a `geop_ops_model::NamedModel`, and `naming`
//! describes the stable names they give to everything they create.
//! `split` cuts a solid into pieces with a sheet, imprinting it the same way.
//! `imprint` divides one face along the curves where sheets cross it.
//! `operation` makes them the [`Boolean`] and [`Split`] operations of a
//! program, and [`Combine`]s a solid another operation built with one the
//! model has — the whole of it, or as far as an [`Extent`] says (see
//! [`extent`]).
//! `scenes` is test-scene scaffolding, built from basic shapes, shared by
//! both modules' tests.

pub mod boolean;
pub mod extent;
mod icons;
pub mod imprint;
pub mod naming;
pub mod operation;
pub mod remesh;
#[cfg(test)]
mod render_test;
#[cfg(test)]
mod scenes;
pub mod split;
pub mod trim;

pub use extent::Extent;
pub use operation::{Boolean, BooleanArgs, Combine, SolidRef, Split, SplitArgs, Tool, solid};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: combining bodies and
/// splitting them.
pub fn register(r: Registry) -> GeopResult<Registry> {
    r.operation(Boolean)?.operation(Split)
}
