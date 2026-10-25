use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::NurbSurface;
use crate::spline::{centered, de_boor, find_span, nearer_end};

/// The index, among the `(p + 1) × (q + 1)` control points acting on
/// `(u, v)` in `u` index major order, of the corner nearest `(u, v)`: the
/// one to evaluate relative to (see [`centered`]).
pub(super) fn corner_nearest(
    surface: &NurbSurface<4>,
    (span_u, u): (usize, Scalar),
    (span_v, v): (usize, Scalar),
) -> usize {
    let (p, q) = (surface.degree_u, surface.degree_v);
    let i = if nearer_end(&surface.knot_vector_u, span_u, u) {
        p
    } else {
        0
    };
    let j = if nearer_end(&surface.knot_vector_v, span_v, v) {
        q
    } else {
        0
    };
    i * (q + 1) + j
}

// ── 3-D surface ──────────────────────────────────────────────────────────────

impl NurbSurface<4> {
    /// Evaluate the surface at `(u, v)`, returning a 3-D Cartesian point.
    ///
    /// Relative to one of the control points acting there ([`centered`]),
    /// so an interval `(u, v)` gives a point as wide as the patch is, not as
    /// far from the origin as it lies.
    pub fn evaluate(&self, u: Scalar, v: Scalar) -> GeopResult<Vector3> {
        let (p, q) = (self.degree_u, self.degree_v);
        let nv = self.num_v;
        let span_u = find_span(p, &self.knot_vector_u, self.num_u - 1, u)?;
        let span_v = find_span(q, &self.knot_vector_v, nv - 1, v)?;

        // The `(p + 1) × (q + 1)` control points acting on the span, `u`
        // index major; each column is evaluated at `u`, then those at `v`.
        let local: Vec<Vector<4>> = (span_u - p..=span_u)
            .flat_map(|i| (span_v - q..=span_v).map(move |j| i * nv + j))
            .map(|k| self.control_points[k])
            .collect();
        let at = corner_nearest(self, (span_u, u), (span_v, v));
        let (local, origin) = centered(&local, at);
        let cols: Vec<Vector<4>> = (0..=q)
            .map(|j| {
                let col: Vec<Vector<4>> = (0..=p).map(|i| local[i * (q + 1) + j]).collect();
                de_boor(p, &self.knot_vector_u[span_u - p..], &col, u, p)
            })
            .collect();
        let hw = de_boor(q, &self.knot_vector_v[span_v - q..], &cols, v, q);

        let w = hw[3];
        if w.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new("weight is zero at evaluation point"));
        }
        let inv_w = Scalar::ONE.div(w)?;
        let mut result = Vector3::zero();
        for c in 0..3 {
            result[c] = origin[c].add(hw[c].mul(inv_w));
        }
        Ok(result)
    }
}

#[cfg(test)]
mod tests {
    use super::super::NurbSurface;
    use crate::test_fixtures::{bilinear, hom};
    use geop_core_math::scalars::Scalar;

    #[test]
    fn bilinear_corner_00() {
        let s = bilinear();
        let p = s.evaluate(Scalar::ZERO, Scalar::ZERO).unwrap();
        assert!(p[0].could_be_equal(Scalar::ZERO));
        assert!(p[1].could_be_equal(Scalar::ZERO));
        assert!(p[2].could_be_equal(Scalar::ZERO));
    }

    #[test]
    fn bilinear_corner_10() {
        let s = bilinear();
        let p = s.evaluate(Scalar::ONE, Scalar::ZERO).unwrap();
        assert!(p[0].could_be_equal(Scalar::ONE));
        assert!(p[1].could_be_equal(Scalar::ZERO));
    }

    #[test]
    fn bilinear_corner_01() {
        let s = bilinear();
        let p = s.evaluate(Scalar::ZERO, Scalar::ONE).unwrap();
        assert!(p[0].could_be_equal(Scalar::ZERO));
        assert!(p[1].could_be_equal(Scalar::ONE));
    }

    #[test]
    fn bilinear_corner_11() {
        let s = bilinear();
        let p = s.evaluate(Scalar::ONE, Scalar::ONE).unwrap();
        assert!(p[0].could_be_equal(Scalar::ONE));
        assert!(p[1].could_be_equal(Scalar::ONE));
    }

    #[test]
    fn bilinear_center() {
        let s = bilinear();
        let p = s
            .evaluate(Scalar::from_f64(0.5), Scalar::from_f64(0.5))
            .unwrap();
        assert!(p[0].could_be_equal(Scalar::from_f64(0.5)));
        assert!(p[1].could_be_equal(Scalar::from_f64(0.5)));
        assert!(p[2].could_be_equal(Scalar::ZERO));
    }

    #[test]
    fn bilinear_mid_u_edge() {
        let s = bilinear();
        let p = s.evaluate(Scalar::from_f64(0.5), Scalar::ZERO).unwrap();
        assert!(p[0].could_be_equal(Scalar::from_f64(0.5)));
        assert!(p[1].could_be_equal(Scalar::ZERO));
    }

    #[test]
    fn out_of_domain_u_returns_err() {
        let s = bilinear();
        assert!(
            s.evaluate(Scalar::from_f64(-0.1), Scalar::from_f64(0.5))
                .is_err()
        );
        assert!(
            s.evaluate(Scalar::from_f64(1.1), Scalar::from_f64(0.5))
                .is_err()
        );
    }

    #[test]
    fn out_of_domain_v_returns_err() {
        let s = bilinear();
        assert!(
            s.evaluate(Scalar::from_f64(0.5), Scalar::from_f64(-0.1))
                .is_err()
        );
        assert!(
            s.evaluate(Scalar::from_f64(0.5), Scalar::from_f64(1.1))
                .is_err()
        );
    }

    #[test]
    fn quadratic_u_midpoint() {
        let f = Scalar::from_f64;
        let s = NurbSurface::try_new(
            2,
            1,
            vec![
                hom(0., 0., 0., 1.),
                hom(0., 1., 0., 1.),
                hom(0.5, 0., 1., 1.),
                hom(0.5, 1., 1., 1.),
                hom(1., 0., 0., 1.),
                hom(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let p = s.evaluate(f(0.5), f(0.)).unwrap();
        assert!(p[0].could_be_equal(f(0.5)));
        assert!(p[1].could_be_equal(f(0.)));
        assert!(p[2].could_be_equal(f(0.5)));
    }

    /// A quarter of a cylinder's cap (radius 3, at height 50), evaluated
    /// over a wide parameter box near its centre: every point of it is at
    /// height 50, and so is the enclosure — as narrow as at the origin.
    /// Evaluated as `A / W` in absolute coordinates it spanned `[38, 94]`
    /// in fixed point, and a cylinder 50 up along its own axis was found
    /// to overlap its own other cap.
    #[test]
    fn a_cap_far_from_the_origin_stays_flat() {
        let f = Scalar::from_f64;
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let z = 50.0;
        // u round the quarter circle, v from the centre out.
        let corners = [(1.0, 0.0, 1.0), (1.0, 1.0, h), (0.0, 1.0, 1.0)];
        let points = corners
            .iter()
            .flat_map(|&(x, y, w)| {
                [
                    hom(0.0, 0.0, z * w, w),
                    hom(3.0 * x * w, 3.0 * y * w, z * w, w),
                ]
            })
            .collect();
        let s = NurbSurface::try_new(
            2,
            1,
            points,
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let (u, v) = (f(0.33).union(f(0.83)), f(0.0005).union(f(0.0008)));
        for (u, v) in [(u, v), (f(0.5), f(0.5))] {
            let p = s.evaluate(u, v).unwrap();
            assert!(p[2].could_be_equal(f(z)), "{p:?}");
            assert!(p[2].width().to_f64() < 1e-6, "{p:?}");
        }
    }

    #[test]
    fn everything_matches_any_point() {
        let s = NurbSurface::<4>::everything();
        let p = s
            .evaluate(Scalar::from_f64(0.5), Scalar::from_f64(-3.0))
            .unwrap();
        assert!(p[0].could_be_equal(Scalar::from_f64(123.456)));
        assert!(p[1].could_be_equal(Scalar::from_f64(-9.0)));
        assert!(p[2].could_be_equal(Scalar::ZERO));
    }
}
