//! Sweeping planar profiles into B-rep solids and sheets: extruding them
//! along a straight path ([`extrude`]), revolving them around an axis
//! ([`revolve`], [`screw`]), carrying them along a chain of curves
//! ([`sweep_along`]) — all one sweep along a [`Path`] (see [`sweep`]) — and
//! lofting through several of them ([`loft`]). Every one of them skins a
//! grid of sections, as a blend's tool does too ([`skin`]).
//!
//! An algorithm here takes geometry — [`Profile`](geop_ops_model::Profile)s
//! in a plane's coordinates, [`Chain`](geop_ops_model::Chain)s, planes —
//! and builds into a [`NamedModel`](geop_ops_model::NamedModel). Resolving
//! a sketch or a picked edge into that geometry, and combining what is
//! built with a solid, is the operations' part (`geop-ops-extrude-revolve`).
//!
//! With the `testing` feature — which only other crates' tests turn on —
//! [`shapes`] also builds basic solids directly: cubes, cylinders, spheres,
//! the fixtures those tests are written against.

mod common;
mod extrude;
mod loft;
mod path_sweep;
mod plain;
#[cfg(test)]
mod rasterize_tests;
mod revolve;
#[cfg(any(test, feature = "testing"))]
pub mod shapes;
mod skin;
mod sweep;

pub use common::{circle_at, polygon, polyline};
pub use extrude::{Level, drafted, extrude, extrusion};
pub use loft::{Section, loft, mark};
pub use path_sweep::{
    Control, Orientation, along_chain, profile_centre, starting_near, sweep_along,
};
pub use revolve::{revolution, revolve, revolve_at_oriented, screw, screw_path};
pub use skin::{Glue, Skinned, Strip, StripCap, StripGeometry, skin};
pub use sweep::{Frame, Path, Span, SweepLoop, sweep, sweep_drawn};
