//! Constructors for basic B-rep solids and shapes.

pub mod common;

pub mod cube;
pub mod cylinder;
pub mod extrude;
pub mod figure8_profile;
pub mod revolve;
pub mod sphere;
// pub mod torus;

pub use cube::cube_solid;
