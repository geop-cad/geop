//! Basic solids built directly from profiles and surface patches — cubes,
//! cylinders, spheres, a figure-eight prism — for tests to be written
//! against. Only built for tests: with `test-shapes`, or in this crate's
//! own.

pub mod cube;
pub mod cylinder;
pub mod figure8_profile;
pub mod sphere;

pub use cube::cube_solid;
