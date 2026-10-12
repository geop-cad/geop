//! Extruding, revolving and sweeping profiles along paths into B-rep solids
//! and sheets — all sweeps (see [`sweep`], [`path_sweep`]) — lofting
//! through several profiles by skinning them (see [`loft`]), and the
//! [`Extrude`], [`Revolve`], [`Sweep`] and [`Loft`] operations built on
//! them.
//!
//! With the `testing` feature — which only other crates' tests turn on —
//! `shapes` also builds basic solids directly: cubes, cylinders, spheres,
//! the fixtures those tests are written against; and `testing` draws the
//! sketches and pushes the steps their programs start from.

pub mod common;

pub mod extrude;
mod icons;
pub mod loft;
pub mod operation;
pub mod path_sweep;
mod plain;
#[cfg(test)]
mod program_tests;
pub mod revolve;
#[cfg(any(test, feature = "testing"))]
pub mod shapes;
pub mod skin;
pub mod sweep;
#[cfg(any(test, feature = "testing"))]
pub mod testing;

pub use geop_ops_booleans::Extent;
pub use operation::{
    Extents, Extrude, ExtrudeArgs, Loft, LoftArgs, Output, Revolve, RevolveArgs, Sweep, SweepArgs,
};
pub use path_sweep::Orientation;
