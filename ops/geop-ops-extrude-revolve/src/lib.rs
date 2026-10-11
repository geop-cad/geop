//! Extruding, revolving and sweeping profiles along paths into B-rep solids
//! and sheets — all sweeps (see [`sweep`], [`path_sweep`]) — lofting
//! through several profiles by skinning them (see [`loft`]), and the
//! [`Extrude`], [`Revolve`], [`Sweep`] and [`Loft`] operations built on
//! them.
//!
//! With the `test-shapes` feature — which only other crates' tests turn on
//! — `shapes` also builds basic solids directly: cubes, cylinders,
//! spheres, the fixtures those tests are written against.

pub mod common;

pub mod extrude;
mod icons;
pub mod loft;
pub mod operation;
pub mod path_sweep;
mod plain;
pub mod revolve;
#[cfg(any(test, feature = "test-shapes"))]
pub mod shapes;
pub mod sweep;

pub use geop_ops_booleans::Extent;
pub use operation::{
    Extents, Extrude, ExtrudeArgs, Loft, LoftArgs, Output, Revolve, RevolveArgs, Sweep, SweepArgs,
};
pub use path_sweep::Orientation;
