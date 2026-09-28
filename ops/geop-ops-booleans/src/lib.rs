//! Boolean operations on solids.
//!
//! `remesh` imprints the two solids' boundaries onto each other (splitting
//! edges at coincidences, tracing face x face intersection curves) so that
//! `boolean` can classify and keep the right faces for union, intersection,
//! or difference. Both work on a `geop_ops::Part`, and `naming`
//! describes the stable names they give to everything they create.
//! `operation` makes them the [`Boolean`] operation of a program, and
//! [`Combine`]s a solid another operation built with one the part has.
//! `scenes` is test-scene scaffolding, built from basic shapes, shared by
//! both modules' tests.

pub mod boolean;
pub mod naming;
pub mod operation;
pub mod remesh;
mod render_test;
#[cfg(test)]
mod scenes;

pub use operation::{Boolean, BooleanArgs, Combine};
