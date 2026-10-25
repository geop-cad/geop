//! Shelling: a solid hollowed out to walls of one thickness, open where
//! faces are taken away ([`shell()`]); a sheet thickened into a solid
//! ([`thicken`]) or offset ([`offset_faces`]). How the hollow is built, and
//! what it does not do yet, is told in `shell.rs`.
//!
//! What rebuilding a solid face by face needs, for others that do it (a
//! draft): room around a surface ([`extended`]), a pcurve on it
//! ([`pcurve`]), and whether a curve lies on the surfaces it should
//! ([`surface_missed`]).

mod shell;
#[cfg(test)]
mod regression_tests;

pub use shell::{extended, offset_faces, pcurve, shell, surface_missed, thicken};
