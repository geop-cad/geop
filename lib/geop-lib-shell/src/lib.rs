//! Rebuilding a solid face by face: some of its faces put on new surfaces,
//! everything they reach moved to where the surfaces now meet
//! ([`rebuilt`], [`replace_faces`]) — what a draft, a face moved, offset or
//! replaced, and a face deleted and healed over ([`delete_faces`]) do.
//! Shelling: a solid hollowed out to walls of one thickness, open where
//! faces are taken away ([`shell()`]); a sheet thickened into a solid
//! ([`thicken`]) or offset ([`offset_faces`]). How the hollow is built,
//! and what it does not do yet, is told in `shell.rs`.
//!
//! What rebuilding needs, for others that do it: room around a surface
//! ([`extended`]), a pcurve on it ([`pcurve`]), and whether a curve lies on
//! the surfaces it should ([`surface_missed`]).

mod delete;
mod rebuild;
#[cfg(test)]
mod regression_tests;
mod shell;

pub use delete::{Heal, delete_faces};
pub use rebuild::{Smooth, rebuilt, replace_faces, tangent_to_kept};
pub use shell::{extended, offset_faces, pcurve, shell, surface_missed, thicken};
