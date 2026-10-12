//! Surface modelling: faces standing on their own — sheets — built from
//! edges, offset, thickened, joined, trimmed and extended, as operations of
//! a program (see [`geop_ops::operation`]):
//!
//! - [`BoundarySurface`]: the ruled face between two edges, or a closed
//!   loop of edges filled — a flat face for a flat loop, a Coons patch for
//!   four edges, optionally tangent to flat faces along them, and a patch
//!   of quadrilaterals around a center for any other number.
//! - [`NetworkSurface`]: the face through a network of curves crossing
//!   each other, `u` curves and `v` curves, interpolating every one.
//! - [`OffsetSurface`]: faces copied a distance along their normals.
//! - [`Thicken`]: a sheet made a solid of one thickness, on either side of
//!   it or on both.
//! - [`Knit`]: sheets whose edges meet joined into one, and into a solid
//!   once they close up.
//! - [`TrimSurface`]: a sheet cut back to one side of a face.
//! - [`ExtendSurface`]: a face standing on its own carried on past one of
//!   its edges.
//!
//! What a sheet is offset and thickened with is the shell's machinery
//! ([`geop_ops_shell::shell`]): a sheet thickened is a sheet shelled.

pub mod boundary;
pub mod extend;
mod icons;
pub mod knit;
pub mod network;
pub mod offset;
#[cfg(test)]
mod program_tests;
pub mod thicken;
pub mod trim;

pub use boundary::{BoundarySurface, BoundarySurfaceArgs};
pub use extend::{ExtendSurface, ExtendSurfaceArgs};
pub use knit::{Knit, KnitArgs};
pub use network::{NetworkSurface, NetworkSurfaceArgs};
pub use offset::{OffsetSurface, OffsetSurfaceArgs};
pub use thicken::{Thicken, ThickenArgs, ThickenSide};
pub use trim::{TrimKeep, TrimSurface, TrimSurfaceArgs};

/// A sheet to build, and the names of what it is made of.
pub type SheetSpec<S> = (geop_core_topology::build::BodySpec<S>, geop_ops::BodyNames);
