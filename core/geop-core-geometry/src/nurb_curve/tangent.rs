use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3},
};

use super::NurbCurve;
use crate::spline::{centered, find_spans, homogeneous_derivatives, rational_derivatives};

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// The Cartesian point and its derivatives up to order `n` at `t`:
    /// `[C(t), C'(t), …, C⁽ⁿ⁾(t)]`, with `C = D − 1`. Evaluated directly,
    /// without building any derivative curve ([`homogeneous_derivatives`],
    /// then the quotient rule [`rational_derivatives`]).
    ///
    /// The derivatives are taken twice and intersected, as
    /// [`NurbCurve::evaluate`] is: from the control points as they are, and
    /// relative to one of them ([`centered`]). They do not depend on where
    /// the curve lies; the first's width does, far from the origin, and the
    /// second costs a rounding that near it is the wider one.
    ///
    /// Over every knot span `t` reaches, united (see [`find_spans`]).
    fn cartesian_derivatives<const C: usize>(
        &self,
        t: S,
        n: usize,
    ) -> GeopResult<Vec<Vector<S, C>>> {
        let p = self.degree;
        let last = self.control_points.len() - 1;
        let mut out: Option<Vec<Vector<S, C>>> = None;
        for (span, piece) in find_spans(p, &self.knot_vector, last, t)? {
            let local = &self.control_points[span - p..=span];
            let at = |points: &[Vector<S, D>]| {
                rational_derivatives::<S, D, C>(&homogeneous_derivatives(
                    p,
                    &self.knot_vector,
                    points,
                    span,
                    piece,
                    n,
                ))
            };
            let mut derivatives = at(local)?;
            let relative = at(&centered(local).0)?;
            for (d, r) in derivatives.iter_mut().zip(&relative).skip(1) {
                for c in 0..C {
                    d[c] = d[c].intersect(r[c]);
                }
            }
            out = Some(match out {
                Some(other) => other
                    .iter()
                    .zip(&derivatives)
                    .map(|(a, b)| a.union(b))
                    .collect(),
                None => derivatives,
            });
        }
        out.ok_or_else(|| GeopError::new(format!("NurbCurve::tangent(t={t:?}): no knot span")))
    }
}

impl<S: Scalar> NurbCurve<S, 4> {
    /// Cartesian tangent vector `C'(t)` (not normalized).
    pub fn tangent(&self, t: S) -> GeopResult<Vector3<S>> {
        Ok(self.cartesian_derivatives::<3>(t, 1)?[1])
    }

    /// Cartesian second derivative `C''(t)` (not normalized). Nonzero for a
    /// rational degree-1 curve too: a line with unequal weights is not
    /// traversed at constant speed.
    pub fn second_derivative(&self, t: S) -> GeopResult<Vector3<S>> {
        Ok(self.cartesian_derivatives::<3>(t, 2)?[2])
    }
}

impl<S: Scalar> NurbCurve<S, 3> {
    /// Cartesian tangent vector `C'(t)` (not normalized), for a 2-D pcurve.
    pub fn tangent(&self, t: S) -> GeopResult<Vector2<S>> {
        Ok(self.cartesian_derivatives::<2>(t, 1)?[1])
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

    /// Degree-1 line with weights 1 and 2: `x(t) = 2t / (1 + t)`, so
    /// `x' = 2 / (1+t)²` and `x'' = −4 / (1+t)³` — nonzero although the
    /// curve is straight.
    fn check_rational_line_derivatives<S: Scalar>() {
        let f = S::from_f64;
        let c = NurbCurve::try_new(
            1,
            vec![pt(0., 0., 0., 1.), pt(2., 0., 0., 2.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        for t in [0., 0.3, 1.] {
            let d1 = c.tangent(f(t)).unwrap();
            let d2 = c.second_derivative(f(t)).unwrap();
            assert!(d1[0].could_be_equal(f(2. / (1. + t).powi(2))), "{d1:?}");
            assert!(d2[0].could_be_equal(f(-4. / (1. + t).powi(3))), "{d2:?}");
            assert!(d1[1].could_be_equal(S::ZERO) && d2[1].could_be_equal(S::ZERO));
        }
    }
    #[test]
    fn rational_line_derivatives() {
        for_all_scalars!(check_rational_line_derivatives);
    }

    /// On the exact rational unit quarter circle `|C| = 1`, so
    /// `C·C' = 0` and `C·C'' + |C'|² = 0` for every `t`.
    fn check_quarter_circle_derivatives<S: Scalar>() {
        let f = S::from_f64;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let c = NurbCurve::try_new(
            2,
            vec![pt(1., 0., 0., 1.), pt(w, w, 0., w), pt(0., 1., 0., 1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        for t in [0., 0.2, 0.5, 0.9, 1.] {
            let p = c.evaluate(f(t)).unwrap();
            let d1 = c.tangent(f(t)).unwrap();
            let d2 = c.second_derivative(f(t)).unwrap();
            assert!(p.prod_dot(&d1).could_be_equal(S::ZERO), "t={t}");
            assert!(
                p.prod_dot(&d2)
                    .add(d1.prod_dot(&d1))
                    .could_be_equal(S::ZERO),
                "t={t}: {d1:?} {d2:?}"
            );
            assert!(d1.norm().definitely_greater(S::ZERO));
        }
    }
    #[test]
    fn quarter_circle_derivatives() {
        for_all_scalars!(check_quarter_circle_derivatives);
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
