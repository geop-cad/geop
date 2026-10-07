//! Solvers: finding where functions vanish, and the constraint systems built
//! on them.
//!
//! - [`interval_newton`]: a verified Gauss-Newton-Krawczyk contraction.
//! - [`least_squares`]: constrained Levenberg–Marquardt over any scalar.
//! - [`system`]: parameters and residuals, solved by the two above; the one
//!   engine behind sketches, assemblies and the parameters of programs.

pub mod interval_newton;
pub mod least_squares;
pub mod system;
