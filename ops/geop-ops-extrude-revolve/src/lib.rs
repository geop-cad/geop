//! Constructors for basic B-rep solids and shapes, and the [`Extrude`] and
//! [`Revolve`] operations built on them.

pub mod common;

pub mod cube;
pub mod cylinder;
pub mod extrude;
pub mod figure8_profile;
pub mod operation;
pub mod revolve;
pub mod sphere;
// pub mod torus;

pub use cube::cube_solid;
pub use operation::{Extrude, ExtrudeArgs, Revolve, RevolveArgs};
