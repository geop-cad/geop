//! The default effort of the subdivision searches and the Newton
//! refinement after them. These bound how hard a search tries, never what
//! an answer means (see `AGENTS.md`): an operation that needs another
//! value says so where it uses it, with its reason.

use geop_core_math::scalars::Scalar;

/// Node budget for a subdivision search (`contains`, `intersection`).
pub const MAX_NODES: usize = 20_000;

/// Where a subdivision search hands over to Newton (see `AGENTS.md`'s
/// "Subdivide to isolate, then Newton to refine").
pub const MIN_SUBDIVISION_SIZE: f64 = 1e-7;

/// Newton steps projecting a point onto a surface.
pub const PROJECT_ITERATIONS: usize = 20;

/// [`MIN_SUBDIVISION_SIZE`] as a scalar.
pub fn min_subdivision_size<S: Scalar>() -> S {
    S::from_f64(MIN_SUBDIVISION_SIZE)
}
