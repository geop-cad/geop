//! Boolean operations on solids.
//!
//! [`remesh`] imprints the two solids' boundaries onto each other (splitting
//! edges at coincidences, tracing face x face intersection curves) so that
//! [`boolean`] can classify and keep the right faces for union,
//! intersection, or difference. Both work on a `geop_ops_model::NamedModel`,
//! and `naming` describes the stable names they give to everything they
//! create. [`split`] cuts a solid into pieces with a sheet, imprinting it the
//! same way. [`imprint_on_face`] divides one face along the curves where
//! sheets cross it. [`combine_tools`] combines the tools a step built as its
//! `geop_ops_model::Combine` says: the whole of each, or only up to the next
//! face of the solid it meets ([`boolean_up_to_next`], [`trim_up_to_next`]).
//! `scenes` is test-scene scaffolding, built from basic shapes, shared by
//! the modules' tests.

mod boolean;
mod combine;
mod imprint;
mod naming;
#[cfg(test)]
mod regression_tests;
mod remesh;
#[cfg(test)]
mod render_test;
#[cfg(test)]
mod scenes;
mod split;
mod trim;

pub use boolean::{Reach, boolean, boolean_up_to_next, piece_up_to_next};
pub use combine::combine_tools;
pub use imprint::imprint_on_face;
pub use remesh::{Origins, RemeshParams, remesh};
pub use split::split;
pub use trim::trim_up_to_next;
