//! Blending a solid's edges: rounding them ([`Fillet`]) and bevelling them
//! ([`Chamfer`]).
//!
//! `blend` builds, for every edge, the tool that holds the material between
//! the edge and its blend face — swept from the edge's cross-section, along
//! a straight edge or around a circular one — and cuts it away with the
//! boolean difference of `geop_ops_booleans`. `operation` makes that the
//! [`Fillet`] and [`Chamfer`] operations of a program.

pub mod blend;
pub mod operation;

pub use blend::BlendShape;
pub use operation::{Chamfer, ChamferArgs, Fillet, FilletArgs};

#[cfg(test)]
mod tests;
