//! Holes and threads, the way a hole wizard makes them: the [`Hole`]
//! operation drills simple, counterbored, countersunk and tapped holes at
//! points on a planar face, sized by ISO tables ([`iso`]) or by hand, each a
//! revolved solid cut away (`hole`); the [`Thread`] operation puts an ISO
//! metric thread on a cylindrical face, recorded as a cosmetic thread or
//! modelled by sweeping its profile along a helix (`thread`).

mod hole;
mod icons;
pub mod iso;
pub mod operation;
#[cfg(test)]
mod pattern_tests;
#[cfg(test)]
mod program_tests;
#[cfg(any(test, feature = "testing"))]
pub mod testing;
mod thread;

pub use operation::{
    FaceRef, Hole, HoleArgs, HoleCentre, HoleKind, PlaneRef, Standard, Thread, ThreadArgs,
};
