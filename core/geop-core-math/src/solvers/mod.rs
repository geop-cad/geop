//! Solvers: finding where functions vanish, and the constraint systems built
//! on them.
//!
//! - [`linear`]: dense linear algebra in honest enclosures — the linear
//!   solves, eliminations, null spaces and inverses every other solver
//!   builds on.
//! - [`iterate`]: fixed-point iteration from sharp seeds, keeping the
//!   honest answer beside the seed — the loop of every Newton-type method.
//! - [`bisect`]: bisection of a bracket, in plain numbers or sharp scalars,
//!   and the scan of samples for the brackets of a sign change.
//! - [`minimize`]: damped descent to a point that is a free choice.
//! - [`probe`]: a sequence of trial values — a step halved, a reach
//!   doubled — probed until one decides.
//! - [`refine`]: adaptive refinement, a resolution made finer until what is
//!   built at it is good enough.
//! - [`subdivide`]: subdivision search, the global method that finds every
//!   solution in a region within a node budget.
//! - [`repeat`]: a step repeated until it says it is done, within a
//!   budget — worklists rewritten until nothing is left to rewrite, marches
//!   stepped on until they arrive, passes until one changes nothing.
//! - [`walk`]: a chain followed node by node until it closes, ends or runs
//!   into itself, and a graph flooded from some nodes to all it reaches.
//! - [`newton`]: Newton refinement of a root a subdivision search has
//!   isolated, and Newton's method on one unknown from a seed.
//! - [`least_squares`]: constrained Levenberg–Marquardt.
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
pub mod repeat;
pub mod subdivide;
pub mod system;
pub mod walk;

thread_local! {
    static WORK: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

/// How much dense linear algebra this thread has done, ever: per
/// factorisation of a matrix of `rows` rows over `columns` unknowns — a
/// step of [`least_squares::minimize`], an elimination of
/// [`linear::eliminate`] — `rows × columns²`, what it costs. A test bounds
/// the work of a solve by it, rather than timing it: a dense system grows
/// as the cube of its size.
pub fn work() -> u64 {
    WORK.get()
}

/// Counts a factorisation of `rows` rows over `columns` unknowns (see
/// [`work`]).
fn factorised(rows: usize, columns: usize) {
    let cost = (rows as u64).saturating_mul((columns as u64).saturating_pow(2));
    WORK.set(WORK.get().saturating_add(cost));
}
