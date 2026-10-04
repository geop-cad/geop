//! Boolean operations on solids.
//!
//! `remesh` imprints the two solids' boundaries onto each other (splitting
//! edges at coincidences, tracing face x face intersection curves) so that
//! `boolean` can classify and keep the right faces for union, intersection,
//! or difference. Both work on a `geop_ops::Part`, and `naming`
//! describes the stable names they give to everything they create.
//! `split` cuts a solid into pieces with a sheet, imprinting it the same way.
//! `imprint` divides one face along the curves where sheets cross it.
//! `operation` makes them the [`Boolean`] and [`Split`] operations of a
//! program, and [`Combine`]s a solid another operation built with one the
//! part has.
//! `scenes` is test-scene scaffolding, built from basic shapes, shared by
//! both modules' tests.

pub mod boolean;
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

pub use operation::{Boolean, BooleanArgs, Combine, Split, SplitArgs, Tool};
