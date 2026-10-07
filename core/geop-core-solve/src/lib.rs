//! Solving constraints: the one engine behind sketches, assemblies and the
//! parameters of programs.
//!
//! A [`System`] has **parameters** — numbers, and poses of rigid bodies
//! ([`Param`]) — and **residuals** ([`Residual`]): functions of a few
//! parameters that are zero exactly when what they stand for holds. Solving
//! moves the parameters that are free until every residual vanishes,
//! changing them as little as it can.
//!
//! - **Residuals are lengths.** Each is in units of length — a
//!   dimensionless one is multiplied by the system's characteristic size —
//!   so none dominates another by its choice of units, and one relative
//!   tolerance decides when a residual holds.
//! - **Residuals are constraints.** A solve minimizes what it is asked to
//!   prefer — pulls, and staying put — among the configurations where every
//!   residual holds, by constrained Levenberg–Marquardt
//!   ([`geop_core_math::least_squares`]): the residuals hold exactly, and no
//!   preference can buy itself a little violation of them. What the
//!   residuals leave free and nothing pulls stays exactly where it is.
//! - **Honest enclosures throughout.** Every residual is computed as a
//!   [`geop_core_math::dual::Dual`] over the system's scalar — any
//!   [`geop_core_math::scalars::Scalar`] — so its gradient is exact and the
//!   `sqrt`, `sin` and `PI` it needs are enclosed rather than rounded; the
//!   minimizer takes a step only where it definitely helps.
//! - **Increments.** The variables are increments from where the
//!   parameters are when a solve starts: a number's added to it, a pose's —
//!   a translation and a turn about its body's center — composed with it
//!   (see [`Placed`]). A solve that changes nothing leaves everything exactly
//!   where it was, and a pose has no singular configurations to cross.
//! - **Pulls.** What a drag pulls where — a number towards a value, a body
//!   towards a pose, a point of a body towards a point ([`Pull`]) — is a
//!   preference: what is pulled follows exactly as far as the residuals
//!   allow, and they win wherever they disagree.
//!
//! [`mates`] builds systems of rigid bodies and the constraints between them
//! on this; `geop-core-sketch` builds a sketch's.

mod linalg;
pub mod mates;
mod memory;
mod placed;
mod system;

pub use placed::Placed;
pub use system::{
    EncloseError, Mobility, Param, Phase, Pull, RELATIVE_TOLERANCE, Report, Residual, System, Value,
};
