//! Boolean operations on solids.
//!
//! `remesh` imprints the two solids' boundaries onto each other (splitting
//! edges at coincidences, tracing face x face intersection curves) so that
//! `boolean` can classify and keep the right faces for union, intersection,
//! or difference. Both work on a `geop_core_part::Part`, and `naming`
//! describes the stable names they give to everything they create. `scenes` is reusable test-scene scaffolding, built purely
//! from `basic_shapes`, shared by both modules' tests.

pub mod boolean;
pub mod naming;
pub mod remesh;
mod render_test;
pub mod scenes;
