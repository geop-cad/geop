//! Holes and threads, the way a hole wizard makes them: the [`Hole`]
//! operation drills simple, counterbored, countersunk and tapped holes at
//! points on a planar face, sized by ISO tables ([`iso`]) or by hand, each a
//! revolved solid cut away ([`hole`]); the [`Thread`] operation puts an ISO
//! metric thread on a cylindrical face, recorded as a cosmetic thread or
//! modelled by sweeping its profile along a helix ([`thread`]).

pub mod hole;
pub mod iso;
pub mod operation;
pub mod thread;

pub use operation::{Hole, HoleArgs, HoleKind, Standard, Thread, ThreadArgs};
