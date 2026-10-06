use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

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
    /// handle an arbitrary direction (see [`Self::mean_curvature`]). Every surface this crate actually
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
        let [suu, _, svv] = self.second_derivatives(u, v)?;

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

    /// A radius of curvature at `(u, v)` along the tangent direction
    /// `along` — a unit vector in the tangent plane, the way a curve on the
    /// surface runs — or `None` if the surface runs straight that way: the
    /// normal curvature `κ = (L a² + 2 M a b + N b²)`, where `along = a Su +
    /// b Sv` and `L, M, N` are the second fundamental form (`Suu·n`,
    /// `Suv·n`, `Svv·n`). Exact for any parametrization.
    ///
    /// A curve on the surface turns, as far as the surface makes it, by this
    /// — where [`Self::curvature_radius`], the tightest bend in *any*
    /// direction, says how a curve circling there would. At a cone's apex the
    /// tightest bend has the radius of the circle round the axis, which goes
    /// to nothing; a generator running into the apex does not bend at all.
    ///
    /// An error where `Su × Sv` vanishes (a pole), and where `along` leaves
    /// the tangent plane's reach.
    pub fn curvature_radius_along(&self, u: S, v: S, along: &Vector3<S>) -> GeopResult<Option<S>> {
        let (su, sv) = self.derivatives(u, v)?;
        let [suu, suv, svv] = self.second_derivatives(u, v)?;
        let n = su.prod_cross(&sv).normalize()?;
        let (e, f, g) = (su.prod_dot(&su), su.prod_dot(&sv), sv.prod_dot(&sv));
        let det = e.mul(g).sub(f.mul(f));
        let (t_u, t_v) = (along.prod_dot(&su), along.prod_dot(&sv));
        let a = g.mul(t_u).sub(f.mul(t_v)).div(det)?;
        let b = e.mul(t_v).sub(f.mul(t_u)).div(det)?;
        let kappa = suu
            .prod_dot(&n)
            .mul(a)
            .mul(a)
            .add(S::TWO.mul(suv.prod_dot(&n)).mul(a).mul(b))
            .add(svv.prod_dot(&n).mul(b).mul(b))
            .abs();
        if kappa.could_be_equal(S::ZERO) {
            Ok(None)
        } else {
            Ok(Some(S::ONE.div(kappa)?.abs()))
        }
    }

    /// The mean curvature at `(u, v)`, signed against the normal
    /// `normalize(Su × Sv)`: half the trace of the second fundamental form
    /// relative to the first,
    /// `H = (L G − 2 M F + N E) / (2 (E G − F²))`, with `E, F, G` the
    /// first fundamental form and `L, M, N` the second (`Suu·n`, `Suv·n`,
    /// `Svv·n`). Unlike [`Self::curvature_radius`] it is exact for any
    /// parametrization, orthogonal or not.
    ///
    /// An error where `Su × Sv` vanishes (a pole): the surface may well have
    /// a curvature there, but not one these partials can express.
    pub fn mean_curvature(&self, u: S, v: S) -> GeopResult<S> {
        let (su, sv) = self.derivatives(u, v)?;
        let [suu, suv, svv] = self.second_derivatives(u, v)?;
        let cross = su.prod_cross(&sv);
        let n = cross.normalize()?;
        let (e, f, g) = (su.prod_dot(&su), su.prod_dot(&sv), sv.prod_dot(&sv));
        let (l, m, nn) = (suu.prod_dot(&n), suv.prod_dot(&n), svv.prod_dot(&n));
        l.mul(g)
            .sub(S::TWO.mul(m).mul(f))
            .add(nn.mul(e))
            .div(S::TWO.mul(cross.norm_sq()))
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

    /// The saddle `S(u, v) = (u, v, uv)`: its only second partial is the
    /// mixed one, so it alone gives the mean curvature
    /// `H = -uv / (1 + u² + v²)^(3/2)`.
    fn check_saddle_mean_curvature<S: Scalar>() {
        let f = S::from_f64;
        let s = NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 1., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let (u, v): (f64, f64) = (0.5, 0.25);
        let expected = -u * v / (1.0 + u * u + v * v).powf(1.5);
        let h = s.mean_curvature(f(u), f(v)).unwrap();
        assert!(h.could_be_equal(f(expected)), "{h:?} vs {expected}");
    }
    #[test]
    fn saddle_mean_curvature() {
        for_all_scalars!(check_saddle_mean_curvature);
    }

    /// A rational quarter cylinder of radius 2 has `|H| = 1 / 4` everywhere,
    /// where the weights make its parametrization uneven too.
    fn check_cylinder_mean_curvature<S: Scalar>() {
        let f = S::from_f64;
        let (r, w) = (2.0, std::f64::consts::FRAC_1_SQRT_2);
        let mut cps = Vec::new();
        for (x, y, wi) in [(1., 0., 1.), (1., 1., w), (0., 1., 1.)] {
            for z in [0., 1.] {
                cps.push(pt(x * r * wi, y * r * wi, z * wi, wi));
            }
        }
        let s = NurbSurface::try_new(
            2,
            1,
            cps,
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        for (u, v) in [(0.1, 0.2), (0.5, 0.5), (0.8, 0.9)] {
            let h = s.mean_curvature(f(u), f(v)).unwrap();
            assert!(h.abs().could_be_equal(f(0.25)), "{h:?} at ({u}, {v})");
        }
    }
    #[test]
    fn cylinder_mean_curvature() {
        for_all_scalars!(check_cylinder_mean_curvature);
    }

    /// A rational quarter cylinder of radius 2 bends round its axis with
    /// radius 2 and runs straight along it — and every direction between
    /// bends with a radius of `2 / sin² θ` — where the tightest bend, by
    /// [`NurbSurface::curvature_radius`], is the one round it.
    fn check_a_cylinder_bends_as_its_direction_says<S: Scalar>() {
        use geop_core_math::vector::Vector3;
        let f = S::from_f64;
        let (r, w) = (2.0, std::f64::consts::FRAC_1_SQRT_2);
        let mut cps = Vec::new();
        for (x, y, wi) in [(1., 0., 1.), (1., 1., w), (0., 1., 1.)] {
            for z in [0., 1.] {
                cps.push(pt(x * r * wi, y * r * wi, z * wi, wi));
            }
        }
        let s = NurbSurface::try_new(
            2,
            1,
            cps,
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let (u, v) = (f(0.5), f(0.5));
        // At u = 1/2 the surface point is at 45 degrees: round is along
        // (-sin, cos, 0), the axis along z.
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let direction = |x: f64, y: f64, z: f64| Vector3::from_array([f(x), f(y), f(z)]);
        let axis = s
            .curvature_radius_along(u, v, &direction(0.0, 0.0, 1.0))
            .unwrap();
        assert!(axis.is_none(), "{axis:?}");
        let round = s
            .curvature_radius_along(u, v, &direction(-h, h, 0.0))
            .unwrap()
            .expect("a bend");
        assert!(round.could_be_equal(f(r)), "{round:?}");
        // Halfway between: sin² of 45 degrees is one half.
        let between = s
            .curvature_radius_along(u, v, &direction(-0.5, 0.5, h))
            .unwrap()
            .expect("a bend");
        assert!(between.could_be_equal(f(2.0 * r)), "{between:?}");
        let tightest = s.curvature_radius(u, v).unwrap().expect("a bend");
        assert!(tightest.could_be_equal(f(r)), "{tightest:?}");
    }
    #[test]
    fn a_cylinder_bends_as_its_direction_says() {
        for_all_scalars!(check_a_cylinder_bends_as_its_direction_says);
    }
}
