//! Extruding and revolving profiles into B-rep solids, and the [`Extrude`]
//! and [`Revolve`] operations built on them.
//!
//! With the `test-shapes` feature — which only other crates' tests turn on
//! — [`shapes`] also builds basic solids directly: cubes, cylinders,
//! spheres, the fixtures those tests are written against.

pub mod common;

pub mod extrude;
pub mod operation;
pub mod revolve;
#[cfg(any(test, feature = "test-shapes"))]
pub mod shapes;

pub use operation::{Extrude, ExtrudeArgs, Revolve, RevolveArgs};
