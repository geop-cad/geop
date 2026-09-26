//! Verified interval Gauss-Newton (a.k.a. Gauss-Newton-Krawczyk) contraction
//! for over-determined, zero-residual-at-the-root systems `F: R² → Rᴹ`.
//!
//! This is the linear-algebra core of the technique described in the
//! `geop-core-geometry` curve-curve-intersection algorithm that uses it
//! ([`geop_core_geometry::intersection::curve_curve_intersect_gnk`], not
//! visible from here — this crate has no dependency on it): two unknowns
//! `(s, t)`, `M` residual equations (`M = 3` for a plain curve-curve
//! coincidence `C1(s) - C2(t) = 0` in R³, `M = 6` once a tangential root is
//! deflated with the cross-product condition `C1'(s) × C2'(t) = 0`). The
//! shape of the math is identical either way — only `M` changes — so it
//! lives here once, generic over `M`, rather than being duplicated per
//! deflation stage.
//!
//! # Why Gauss-Newton, not a square Krawczyk operator
//!
//! A genuine curve intersection is exactly-determined in a geometric sense
//! (one point on each curve) but over-determined algebraically (2 unknowns,
//! `M > 2` equations) — there is no square Jacobian to invert. The standard
//! fix is the normal equations: approximate the Moore-Penrose pseudoinverse
//! `Y ≈ (JᵀJ)⁻¹Jᵀ` (a `2×M` matrix) and run Krawczyk with `Y` in place of
//! `J⁻¹`:
//!
//! ```text
//! K(X) = x̂ − Y·F(x̂) + (I − Y·J(X))·(X − x̂)
//! ```
//!
//! Because a real intersection has `F(x*) = 0` exactly (not merely
//! minimized, as in a least-squares fit), Neumaier's convergence theory for
//! verified interval Gauss-Newton on zero-residual over-determined systems
//! applies directly: the contraction stays quadratic, exactly as for a
//! square Krawczyk step.
//!
//! # What `X` and `x̂` are here
//!
//! Every enclosure in this codebase is carried by the scalar type itself
//! (`ScalInF64`/`ScalInFPA64` *are* `[lo, hi]` intervals — see
//! `scalars::Scalar`), so a "box" in `(s, t)` is just a `Vector<S, 2>`, one
//! interval component per unknown. `x̂` is the (sharp) midpoint of that box
//! — a single evaluation point standing in for "the current best guess",
//! exactly as `Scalar::sharpen`/`Scalar::midpoint` are used elsewhere for
//! the same purpose (see `Scalar::sharpen`'s own doc comment). `J`, `J(X)`
//! are `Matrix<S, M, 2>` — `M` rows (one per residual component), 2 columns
//! (`∂/∂s`, `∂/∂t`).

use crate::{
    geop_error::{GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::Vector,
};

/// Outcome of one [`gauss_newton_krawczyk_step`] contraction.
pub struct KrawczykStep<S: Scalar> {
    /// `K(X) ∩ X` — still an honest enclosure of every root of `F` in the
    /// incoming `X`, tightened whenever `K(X)` and `X` actually overlap.
    /// Meaningless (equal to the incoming `X`, unchanged) when `empty` is
    /// `true` — the whole point of that flag is to tell the two cases apart.
    pub contracted: Vector<S, 2>,
    /// `K(X) ⊆ X` held: the rigorous existence-*and*-uniqueness certificate
    /// (Krawczyk's classical guarantee) — `X` contains exactly one root of
    /// `F`, and it lies in `contracted`.
    pub verified: bool,
    /// `K(X)` and `X` did not even overlap on some component: a rigorous
    /// *non*-existence proof — `X` contains no root of `F` at all. Distinct
    /// from `!verified`, which merely means "inconclusive" (neither proved
    /// nor disproved) — the caller must check this before trusting
    /// `contracted`.
    pub empty: bool,
}

/// One Gauss-Newton-Krawczyk contraction of the box `x_box = (s_box, t_box)`
/// against `F(s, t) ∈ Rᴹ`.
///
/// - `x_hat`: the evaluation point (sharp midpoint of `x_box`).
/// - `f_hat`: `F(x̂)`.
/// - `jac_hat`: `J(x̂)`, `M×2` — evaluated at the sharp point `x̂` (this is
///   what makes `JᵀJ` an ordinary — not interval — `2×2` system, solvable
///   by plain Gaussian elimination).
/// - `jac_box`: `J(X)`, `M×2` — the same Jacobian, but evaluated as an
///   *enclosure* over the whole incoming box (interval arithmetic over
///   `x_box`, not just the midpoint) — this is what lets the contraction
///   step be rigorous rather than merely a numerical guess.
///
/// Errs only when `JᵀJ` (built from the *sharp* `jac_hat`) is singular —
/// tangential/rank-deficient curves at `x̂`, Cauchy-Schwarz equality in
/// `|C1'|²|C2'|² = (C1'·C2')²` — a fixed, structural signal to the caller
/// that this system needs deflating (see the module doc comment) rather
/// than a transient numerical hiccup to retry.
pub fn gauss_newton_krawczyk_step<S: Scalar, const M: usize>(
    x_hat: Vector<S, 2>,
    f_hat: Vector<S, M>,
    jac_hat: Matrix<S, M, 2>,
    x_box: Vector<S, 2>,
    jac_box: Matrix<S, M, 2>,
) -> GeopResult<KrawczykStep<S>> {
    let jac_hat_t = jac_hat.transpose();
    let jtj = jac_hat_t.mul_mat(&jac_hat);

    // Gauss-Newton step at x̂: solve (JᵀJ) δ = Jᵀ F(x̂), so x̂ − δ is the usual
    // normal-equations correction (`Y·F(x̂)` with `Y = (JᵀJ)⁻¹Jᵀ`).
    let jtf = jac_hat_t.mul_vec(&f_hat);
    let delta = solve_linear_system(&jtj, &jtf).with_context(
        "gauss_newton_krawczyk_step: JᵀJ singular at x̂ (rank-deficient Jacobian — likely tangential)",
    )?;
    let x_center = x_hat.sub(&delta);

    // Y·J(X) = (JᵀJ)⁻¹ (Jᵀ J(X)), a 2×2 matrix, computed column-by-column
    // (each column is itself a (JᵀJ)⁻¹·(interval vector) solve).
    let jt_jbox = jac_hat_t.mul_mat(&jac_box);
    let col0 = solve_linear_system(&jtj, &jt_jbox.col(0))?;
    let col1 = solve_linear_system(&jtj, &jt_jbox.col(1))?;
    let yjx = Matrix::from_columns([col0, col1]);

    let imyjx = Matrix::<S, 2, 2>::identity().sub(&yjx);
    let term2 = imyjx.mul_vec(&x_box.sub(&x_hat));

    let k = x_center.add(&term2);

    let overlaps = [k[0].could_be_equal(x_box[0]), k[1].could_be_equal(x_box[1])];
    let empty = !overlaps[0] || !overlaps[1];
    let verified = !empty && k[0].is_subset_of(x_box[0]) && k[1].is_subset_of(x_box[1]);
    let contracted = if empty {
        x_box
    } else {
        Vector::from_array([k[0].intersect(x_box[0]), k[1].intersect(x_box[1])])
    };

    Ok(KrawczykStep {
        contracted,
        verified,
        empty,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::for_all_scalars;

    /// `F(s, t) = [s − 1, t − 2, 0]`: a trivial, exactly-determined-in-
    /// disguise linear system (the third residual component is identically
    /// zero everywhere, so it contributes to `JᵀJ` conditioning without
    /// ever perturbing the root). Root at `(1, 2)`.
    fn f_and_jac<S: Scalar>(s: S, t: S) -> (Vector<S, 3>, Matrix<S, 3, 2>) {
        let f = Vector::from_array([s.sub(S::ONE), t.sub(S::TWO), S::ZERO]);
        let jac = Matrix::from_rows([[S::ONE, S::ZERO], [S::ZERO, S::ONE], [S::ZERO, S::ZERO]]);
        (f, jac)
    }

    fn check_contracts_and_verifies_box_containing_root<S: Scalar>() {
        let x_box = Vector::from_array([
            S::from_f64(0.0).union(S::from_f64(3.0)),
            S::from_f64(1.0).union(S::from_f64(4.0)),
        ]);
        let x_hat = Vector::from_array([x_box[0].midpoint(), x_box[1].midpoint()]);
        let (f_hat, jac_hat) = f_and_jac(x_hat[0], x_hat[1]);
        // Linear system: J(X) is the same constant Jacobian everywhere.
        let jac_box = jac_hat;

        let step = gauss_newton_krawczyk_step(x_hat, f_hat, jac_hat, x_box, jac_box).unwrap();
        assert!(!step.empty);
        assert!(step.verified, "linear system should verify in one step");
        assert!(step.contracted[0].could_be_equal(S::ONE));
        assert!(step.contracted[1].could_be_equal(S::TWO));
    }
    #[test]
    fn contracts_and_verifies_box_containing_root() {
        for_all_scalars!(check_contracts_and_verifies_box_containing_root);
    }

    fn check_proves_empty_for_box_missing_root<S: Scalar>() {
        // Root is at (1, 2); this box doesn't come near it.
        let x_box = Vector::from_array([
            S::from_f64(10.0).union(S::from_f64(11.0)),
            S::from_f64(10.0).union(S::from_f64(11.0)),
        ]);
        let x_hat = Vector::from_array([x_box[0].midpoint(), x_box[1].midpoint()]);
        let (f_hat, jac_hat) = f_and_jac(x_hat[0], x_hat[1]);
        let jac_box = jac_hat;

        let step = gauss_newton_krawczyk_step(x_hat, f_hat, jac_hat, x_box, jac_box).unwrap();
        assert!(step.empty, "box far from the root should be proven empty");
    }
    #[test]
    fn proves_empty_for_box_missing_root() {
        for_all_scalars!(check_proves_empty_for_box_missing_root);
    }

    fn check_singular_jacobian_errs<S: Scalar>() {
        // Both columns identical -> JᵀJ singular (rank-1), the tangential
        // signal `gauss_newton_krawczyk_step` is documented to error on.
        let x_hat = Vector::from_array([S::ZERO, S::ZERO]);
        let x_box = Vector::from_array([S::ZERO.union(S::ONE), S::ZERO.union(S::ONE)]);
        let f_hat = Vector::from_array([S::ONE, S::ONE, S::ONE]);
        let jac_hat = Matrix::from_rows([[S::ONE, S::ONE], [S::ONE, S::ONE], [S::ONE, S::ONE]]);
        let jac_box = jac_hat;

        assert!(gauss_newton_krawczyk_step(x_hat, f_hat, jac_hat, x_box, jac_box).is_err());
    }
    #[test]
    fn singular_jacobian_errs() {
        for_all_scalars!(check_singular_jacobian_errs);
    }
}
