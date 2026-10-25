//! The signed area a closed loop of 2-D curves encloses, integrated from
//! the curves themselves ([`loop_signed_area`]) rather than from a polygon
//! sampled along them.
//!
//! By Green's theorem the area a closed loop encloses is
//!
//! ```text
//! A = ½ ∮ (u v' − v u') dt,
//! ```
//!
//! positive counter-clockwise, and each curve of the loop adds its own part
//! ([`NurbCurve::area_term`]). On a span where the weights are all the same
//! number the curve is a polynomial of its degree `p`, the integrand one of
//! degree `2p − 1`, and the Gauss–Legendre rule of `p` points integrates it
//! exactly: the result is proven, with no truncation error. A rational
//! curve, an arc say, is integrated by adaptive Gauss–Kronrod, its
//! truncation error estimated (see [`geop_core_math::quadrature`]).
//!
//! A polygon of a fixed number of samples per curve has neither: the gap
//! between it and the curve is not part of its area's enclosure, so a thin
//! loop, or one of many spans, could come out with the wrong sign, and
//! definitely so.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    quadrature::{MAX_POLYNOMIAL_DEGREE, Quadrature, integrate, integrate_polynomial},
    scalars::Scalar,
    vector::Vector2,
};

use super::NurbCurve2D;

/// How hard the integral along a rational curve tries.
const RATIONAL: Quadrature = Quadrature {
    relative_tolerance: 1e-10,
    max_panels: 64,
};

impl<S: Scalar> NurbCurve2D<S> {
    /// `½ ∫ ((u − o_u) v' − (v − o_v) u') dt` over the curve, `o` the
    /// `origin`: its part of the signed area of a closed loop it belongs to
    /// (see the module docs). The parts of one loop must be measured from
    /// the same origin; their sum then does not depend on it.
    ///
    /// The curve is moved to `origin` once, so that every point of the
    /// integrand comes with its tangent from one pass over the spline.
    pub fn area_term(&self, origin: &Vector2<S>) -> GeopResult<S> {
        let local = self.translate(Vector2::from_array([origin[0].neg(), origin[1].neg()]));
        let integrand = |t: S| -> GeopResult<Vec<S>> {
            let [p, d] = local.cartesian_derivatives::<2>(t, 1)?[..] else {
                unreachable!("a point and its first derivative");
            };
            Ok(vec![p[0].mul(d[1]).sub(p[1].mul(d[0]))])
        };
        let breaks = self.breakpoints();
        let degree = (2 * self.degree).saturating_sub(1);
        let integral = if self.is_polynomial() && degree <= MAX_POLYNOMIAL_DEGREE {
            integrate_polynomial(integrand, &breaks, 1, degree)?
        } else {
            integrate(integrand, &breaks, 1, &RATIONAL)?
        };
        integral.value[0].div(S::TWO)
    }

    /// Whether every weight is the same number — the same enclosure, not
    /// merely overlapping ones — so that the curve is a polynomial spline.
    fn is_polynomial(&self) -> bool {
        let w = self.control_points[0][2];
        let same = |a: S| a.is_subset_of(w) && w.is_subset_of(a);
        self.control_points.iter().all(|c| same(c[2]))
    }
}

/// The signed area the closed loop `pcurves` encloses, in order: positive
/// counter-clockwise, negative clockwise (see the module docs).
///
/// Measured from the loop's first point, sharpened: where the parts are
/// measured from is a free choice, and one near the loop keeps the large
/// correlated products of a loop far from the origin out of the integrand.
pub fn loop_signed_area<'a, S: Scalar + 'a>(
    pcurves: impl IntoIterator<Item = &'a NurbCurve2D<S>>,
) -> GeopResult<S> {
    let mut pcurves = pcurves.into_iter().peekable();
    let first = pcurves
        .peek()
        .ok_or_else(|| GeopError::new("loop_signed_area: a loop of no curves"))?;
    let origin = first.ends()?.0.sharpen();
    pcurves.try_fold(S::ZERO, |area, pcurve| {
        Ok(area.add(pcurve.area_term(&origin)?))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::{for_all_scalars, scalars::Scalar};

    use crate::nurb_curve::sqrt2_over_2;

    fn v2<S: Scalar>(u: f64, v: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(u), S::from_f64(v)])
    }

    /// The unit square, counter-clockwise and then clockwise, far from the
    /// origin: an area of exactly one, of either sign.
    fn check_square_area_is_exact<S: Scalar>() {
        let corners = [(1000., 500.), (1001., 500.), (1001., 501.), (1000., 501.)];
        let ccw: Vec<NurbCurve2D<S>> = (0..4)
            .map(|k| {
                let (a, b) = (corners[k], corners[(k + 1) % 4]);
                NurbCurve2D::line(v2(a.0, a.1), v2(b.0, b.1)).unwrap()
            })
            .collect();
        let area = loop_signed_area(&ccw).unwrap();
        assert!(area.could_be_equal(S::ONE), "{area:?}");
        assert!(area.definitely_greater(S::ZERO), "{area:?}");
        let cw: Vec<NurbCurve2D<S>> = ccw.iter().rev().map(|c| c.reverse()).collect();
        let area = loop_signed_area(&cw).unwrap();
        assert!(area.could_be_equal(S::ONE.neg()), "{area:?}");
    }
    #[test]
    fn square_area_is_exact() {
        for_all_scalars!(check_square_area_is_exact);
    }

    /// A disc of four rational quarter arcs encloses `π r²`: within the
    /// enclosure, which takes in the quadrature's estimated error.
    fn check_disc_area<S: Scalar>() {
        let r = 2.0;
        let w = sqrt2_over_2::<S>();
        let at = |k: usize| {
            let (c, s) = [(1., 0.), (0., 1.), (-1., 0.), (0., -1.)][k % 4];
            v2::<S>(3. + r * c, -1. + r * s)
        };
        let center = v2::<S>(3., -1.);
        let quarters: Vec<NurbCurve2D<S>> = (0..4)
            .map(|k| {
                let mid = at(k).add(&at(k + 1)).sub(&center);
                NurbCurve2D::arc(at(k), mid, at(k + 1), w).unwrap()
            })
            .collect();
        let area = loop_signed_area(&quarters).unwrap();
        let exact = std::f64::consts::PI * r * r;
        assert!(
            area.could_be_equal(S::from_f64(exact)),
            "{area:?}, expected {exact}"
        );
    }
    #[test]
    fn disc_area() {
        for_all_scalars!(check_disc_area);
    }
}
