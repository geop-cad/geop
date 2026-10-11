//! Solvers: finding where functions vanish, and the constraint systems built
//! on them.
//!
//! - [`linear`]: dense linear algebra in honest enclosures — the linear
//!   solves, eliminations, null spaces and inverses every other solver
//!   builds on.
//! - [`newton`]: Newton refinement of a root a subdivision search has
//!   isolated.
//! - [`least_squares`]: constrained Levenberg–Marquardt over any scalar.
//! - [`system`]: constraint systems on blocks of variables — numbers,
//!   poses — solved by the driver, checked, measured for freedom and
//!   conflicts, and their solution enclosed: the engine behind sketches and
//!   the state of programs alike. A solve holds the constraints exactly,
//!   and prefers pulls and staying put among where they hold.

pub mod least_squares;
pub mod linear;
pub mod newton;
pub mod system;
