use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use super::NurbCurve;

impl<S: Scalar> NurbCurve<S, 4> {
    /// Cartesian tangent vector `C'(t)` (not normalized).
    ///
    /// Evaluates the homogeneous derivative curve from [`NurbCurve::derivative`]
    /// and applies the quotient rule `C'(t) = (A'(t) − w'(t)·C(t)) / w(t)`,
    /// where `(A(t), w(t))` is `self`'s own homogeneous point at `t`.
    pub fn tangent(&self, t: S) -> GeopResult<Vector3<S>> {
        let pos = self.evaluate(t)?;

        let span = self.find_knot_span(t)?;
        let hw = self.de_boor(t, span);
        let w = hw[3];

        let deriv = self.derivative()?;
        let dspan = deriv.find_knot_span(t)?;
        let dhw = deriv.de_boor(t, dspan);

        let mut result = Vector3::zero();
        for c in 0..3 {
            result[c] = dhw[c].sub(dhw[3].mul(pos[c])).div(w)?;
        }
        Ok(result)
    }

    /// Cartesian second derivative `C''(t)` (not normalized).
    ///
    /// Applies the quotient rule twice: `C''(t) = (A''(t) − 2·w'(t)·C'(t) −
    /// w''(t)·C(t)) / w(t)`, where `(A(t), w(t))` is `self`'s own homogeneous
    /// point at `t` and `A''(t), w''(t)` come from differentiating the
    /// homogeneous curve twice via [`NurbCurve::derivative`].
    ///
    /// A curve of degree `< 2` has no well-defined second derivative of its
    /// *homogeneous* representation (differentiating twice underflows the
    /// degree) — but geometrically a degree-`0`/`1` curve is a straight
    /// line, whose Cartesian second derivative is genuinely zero, so that's
    /// what's returned instead of an error.
    pub fn second_derivative(&self, t: S) -> GeopResult<Vector3<S>> {
        if self.degree < 2 {
            return Ok(Vector3::zero());
        }

        let pos = self.evaluate(t)?;
        let tan = self.tangent(t)?;

        let span = self.find_knot_span(t)?;
        let w = self.de_boor(t, span)[3];

        let d1 = self.derivative()?;
        let d1span = d1.find_knot_span(t)?;
        let wp = d1.de_boor(t, d1span)[3];

        let d2 = d1.derivative()?;
        let d2span = d2.find_knot_span(t)?;
        let d2hw = d2.de_boor(t, d2span);
        let wpp = d2hw[3];

        let mut result = Vector3::zero();
        for c in 0..3 {
            result[c] = d2hw[c]
                .sub(wp.mul(S::TWO).mul(tan[c]))
                .sub(wpp.mul(pos[c]))
                .div(w)?;
        }
        Ok(result)
    }
}

impl<S: Scalar> NurbCurve<S, 3> {
    /// Cartesian tangent vector `C'(t)` (not normalized), for a 2-D pcurve.
    ///
    /// Same quotient-rule derivation as the 3-D [`NurbCurve::<S, 4>::tangent`].
    pub fn tangent(&self, t: S) -> GeopResult<Vector2<S>> {
        let pos = self.evaluate(t)?;

        let span = self.find_knot_span(t)?;
        let hw = self.de_boor(t, span);
        let w = hw[2];

        let deriv = self.derivative()?;
        let dspan = deriv.find_knot_span(t)?;
        let dhw = deriv.de_boor(t, dspan);

        let mut result = Vector2::zero();
        for c in 0..2 {
            result[c] = dhw[c].sub(dhw[2].mul(pos[c])).div(w)?;
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_curve::NurbCurve;
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

    fn check_line_tangent_is_constant_direction<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            1,
            vec![pt(0., 0., 0., 1.), pt(2., 0., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();

        for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let tan = c.tangent(f(t)).unwrap();
            assert!(tan[0].could_be_equal(S::TWO));
            assert!(tan[1].could_be_equal(S::ZERO));
            assert!(tan[2].could_be_equal(S::ZERO));
        }
    }
    #[test]
    fn line_tangent_is_constant_direction() {
        for_all_scalars!(check_line_tangent_is_constant_direction);
    }

    /// Quadratic Bézier `(0,0,0) -> (1,1,0) -> (2,0,0)`: tangent at t=0.5
    /// (the apex) should be purely horizontal (dy/dt = 0), pointing +x.
    fn check_quadratic_apex_tangent_is_horizontal<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            2,
            vec![pt(0., 0., 0., 1.), pt(1., 1., 0., 1.), pt(2., 0., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();

        let tan = c.tangent(f(0.5)).unwrap();
        assert!(tan[0].definitely_greater(S::ZERO));
        assert!(tan[1].could_be_equal(S::ZERO));
        assert!(tan[2].could_be_equal(S::ZERO));
    }
    #[test]
    fn quadratic_apex_tangent_is_horizontal() {
        for_all_scalars!(check_quadratic_apex_tangent_is_horizontal);
    }

    /// At the start of that same Bézier, the tangent should point up and to
    /// the right (+x, +y), matching the initial control-polygon direction.
    fn check_quadratic_start_tangent_direction<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            2,
            vec![pt(0., 0., 0., 1.), pt(1., 1., 0., 1.), pt(2., 0., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();

        let tan = c.tangent(f(0.0)).unwrap();
        assert!(tan[0].definitely_greater(S::ZERO));
        assert!(tan[1].definitely_greater(S::ZERO));
    }
    #[test]
    fn quadratic_start_tangent_direction() {
        for_all_scalars!(check_quadratic_start_tangent_direction);
    }

    // ── 2-D (pcurve) tangent ──────────────────────────────────────────────────

    fn pt2<S: Scalar>(x: f64, y: f64, w: f64) -> geop_core_math::vector::Vector3<S> {
        geop_core_math::vector::Vector3::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(w),
        ])
    }

    fn check_line_2d_tangent_is_constant_direction<S: Scalar>() {
        let f = S::from_f64;
        let c: crate::nurb_curve::NurbCurve2D<S> = NurbCurve::try_new(
            1,
            vec![pt2(0., 0., 1.), pt2(2., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();

        for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let tan = c.tangent(f(t)).unwrap();
            assert!(tan[0].could_be_equal(S::TWO));
            assert!(tan[1].could_be_equal(S::ZERO));
        }
    }
    #[test]
    fn line_2d_tangent_is_constant_direction() {
        for_all_scalars!(check_line_2d_tangent_is_constant_direction);
    }

    /// Quadratic Bézier `(0,0) -> (1,1) -> (2,0)` in 2-D: tangent at t=0.5
    /// (the apex) should be purely horizontal (dy/dt = 0), pointing +x.
    fn check_quadratic_2d_apex_tangent_is_horizontal<S: Scalar>() {
        let f = S::from_f64;
        let c: crate::nurb_curve::NurbCurve2D<S> = NurbCurve::try_new(
            2,
            vec![pt2(0., 0., 1.), pt2(1., 1., 1.), pt2(2., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();

        let tan = c.tangent(f(0.5)).unwrap();
        assert!(tan[0].definitely_greater(S::ZERO));
        assert!(tan[1].could_be_equal(S::ZERO));
    }
    #[test]
    fn quadratic_2d_apex_tangent_is_horizontal() {
        for_all_scalars!(check_quadratic_2d_apex_tangent_is_horizontal);
    }
}
