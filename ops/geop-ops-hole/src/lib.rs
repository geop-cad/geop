//! Holes and threads, the way a hole wizard makes them: the [`Hole`]
//! operation drills simple, counterbored, countersunk and tapped holes at
//! points on a planar face, sized by ISO tables ([`iso`]) or by hand, or
//! tapped for an NPT pipe thread ([`pipe`]) and reamed to its taper, each a
//! revolved solid cut away (`hole`); the [`Thread`] operation puts an ISO
//! metric or a pipe thread on a round face, straight or tapered, recorded as
//! a cosmetic thread or modelled by sweeping its profile along a helix
//! (`thread`).

#[cfg(test)]
mod editor_tests;
#[cfg(any(test, feature = "examples"))]
pub mod examples;
mod hole;
mod icons;
pub mod iso;
pub mod operation;
#[cfg(test)]
mod pattern_tests;
pub mod pipe;
#[cfg(test)]
mod program_tests;
#[cfg(test)]
mod regression_tests;
#[cfg(test)]
mod testing;
mod thread;

pub use operation::{
    FaceRef, Hole, HoleArgs, HoleCentre, HoleKind, PlaneRef, Standard, Thread, ThreadArgs,
};

use geop_core_editor::registry::Registry;
use geop_core_math::geop_error::GeopResult;

/// Adds what this crate offers an editor to `r`: holes and threads.
pub fn register(r: Registry) -> GeopResult<Registry> {
    let r = r.operation(Hole)?.operation(Thread)?;
    #[cfg(any(test, feature = "examples"))]
    let r = r.examples(examples::set())?;
    Ok(r)
}
