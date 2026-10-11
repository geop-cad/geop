//! Solvers: finding where functions vanish, and the constraint systems built
//! on them.
//!
//! - [`linear`]: dense linear algebra in honest enclosures — the linear
//!   solves, eliminations, null spaces and inverses every other solver
//!   builds on.
//! - [`iterate`]: fixed-point iteration from sharp seeds, keeping the
//!   honest answer beside the seed — the loop of every Newton-type method.
//! - [`bisect`]: bisection of a bracket, in plain numbers or sharp scalars.
//! - [`minimize`]: damped descent, golden-section search and alternating
//!   projections for a point that is a free choice.
//! - [`probe`]: a sequence of trial values — a step halved, a reach
//!   doubled — probed until one decides.
//! - [`refine`]: adaptive refinement, a resolution made finer until what is
//!   built at it is good enough.
//! - [`subdivide`]: subdivision search, the global method that finds every
//!   solution in a region within a node budget.
//! - [`newton`]: Newton refinement of a root a subdivision search has
//!   isolated.
//! - [`least_squares`]: constrained Levenberg–Marquardt over any scalar.
//! - [`system`]: constraint systems on blocks of variables — numbers,
//!   poses — solved by the driver, checked, measured for freedom and
//!   conflicts, and their solution enclosed: the engine behind sketches and
//!   the state of programs alike. A solve holds the constraints exactly,
//!   and prefers pulls and staying put among where they hold.

pub mod bisect;
pub mod iterate;
pub mod least_squares;
pub mod linear;
pub mod minimize;
pub mod newton;
pub mod probe;
pub mod refine;
pub mod subdivide;
pub mod system;
