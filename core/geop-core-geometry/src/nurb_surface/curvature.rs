use geop_core_math::{geop_error::GeopResult, scalars::Scalar};

use super::NurbSurface;

impl<S: Scalar> NurbSurface<S, 4> {
    /// A conservative radius of curvature at `(u, v)`, or `None` if the
    /// surface is (locally) flat in both parametric directions.
    ///
    /// Computed straight from the surface's own second partial derivatives
    /// — no history, no finite differences — via the normal curvature along
    /// each parametric direction, `κ_a = (S_aa · n) / |S_a|²` for `a ∈ {u,
    /// v}` (the diagonal terms of the second fundamental form divided by the
    /// diagonal terms of the first). This is the *exact* normal curvature in
    /// direction `a` only when `Su ⊥ Sv`; the general formula also needs the
    /// mixed partial `Suv` and the off-diagonal metric term `F = Su·Sv` to
    /// handle an arbitrary direction. Every surface this crate actually
    /// constructs has orthogonal parametric directions by construction —
    /// flat bilinear box/cap faces (`Su`, `Sv` are the patch's two edge
    /// directions) and `revolve`'s ruled patches (axial `u` is always
    /// perpendicular to the circular `v`) — so the approximation is exact
    /// for our surfaces, not just a rough heuristic.
    ///
    /// The returned radius is `1 / max(|κ_u|, |κ_v|)`: the tighter of the
    /// two bends, so a caller sizing steps off of it stays conservative.
    pub fn curvature_radius(&self, u: S, v: S) -> GeopResult<Option<S>> {
        let (su, sv) = self.derivatives(u, v)?;
        let (suu, svv) = self.second_derivatives(u, v)?;

        let normal = match su.prod_cross(&sv).normalize() {
            Ok(n) => n,
            Err(_) => return Ok(None),
        };

        let su_len_sq = su.prod_dot(&su);
        let sv_len_sq = sv.prod_dot(&sv);

        let kappa_u = if su_len_sq.could_be_equal(S::ZERO) {
            S::ZERO
        } else {
            suu.prod_dot(&normal).div(su_len_sq)?.abs()
        };
        let kappa_v = if sv_len_sq.could_be_equal(S::ZERO) {
            S::ZERO
        } else {
            svv.prod_dot(&normal).div(sv_len_sq)?.abs()
        };

        let kappa = if kappa_u.definitely_greater(kappa_v) {
            kappa_u
        } else {
            kappa_v
        };

        if kappa.could_be_equal(S::ZERO) {
            Ok(None)
        } else {
            Ok(Some(S::ONE.div(kappa)?.abs()))
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_surface::NurbSurface;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(z),
            S::from_f64(w),
        ])
    }

    /// A flat bilinear patch has zero curvature everywhere: no radius limit.
    fn check_flat_patch_has_no_curvature_radius<S: Scalar>() {
        let f = S::from_f64;
        let s = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();

        assert!(s.curvature_radius(f(0.5), f(0.5)).unwrap().is_none());
    }
    #[test]
    fn flat_patch_has_no_curvature_radius() {
        for_all_scalars!(check_flat_patch_has_no_curvature_radius);
    }
}
