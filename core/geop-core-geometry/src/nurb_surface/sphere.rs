//! The piece of a sphere bounded by three great arcs
//! ([`NurbSurface3D::spherical_triangle`]): what rounds a corner where three
//! fillets of one radius meet.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::NurbSurface3D;

/// The binomial coefficient `n` choose `k`.
fn binomial(n: usize, k: usize) -> i64 {
    (0..k).fold(1i64, |c, i| c * (n - i) as i64 / (i + 1) as i64)
}

/// The Bernstein coefficients of the product of the polynomials whose
/// Bernstein coefficients are `a` (scalars) and `b` (homogeneous points):
/// of degree the sum of theirs. A coefficient made of one term of factor
/// one — at both ends — is taken as it is: multiplying by an exact one
/// would only widen it by rounding.
fn product<S: Scalar>(a: &[S], b: &[Vector4<S>]) -> GeopResult<Vec<Vector4<S>>> {
    let (m, n) = (a.len() - 1, b.len() - 1);
    (0..=m + n)
        .map(|k| {
            let mut sum: Option<Vector4<S>> = None;
            for i in k.saturating_sub(n)..=k.min(m) {
                let j = k - i;
                let factor =
                    S::from_ratio(binomial(m, i) * binomial(n, j), binomial(m + n, k))?.mul(a[i]);
                let term = b[j].prod_scalar(factor);
                sum = Some(match sum {
                    None => term,
                    Some(s) => s.add(&term),
                });
            }
            Ok(sum.expect("at least one term"))
        })
        .collect()
}

/// The Bernstein coefficients `a` raised by `by` degrees: the same
/// polynomial. Its two end coefficients stay exactly as they are.
fn elevate<S: Scalar>(a: &[Vector4<S>], by: usize) -> GeopResult<Vec<Vector4<S>>> {
    let m = a.len() - 1;
    (0..=m + by)
        .map(|k| {
            if k == 0 {
                return Ok(a[0]);
            }
            if k == m + by {
                return Ok(a[m]);
            }
            let mut sum: Option<Vector4<S>> = None;
            for i in k.saturating_sub(by)..=k.min(m) {
                let factor =
                    S::from_ratio(binomial(m, i) * binomial(by, k - i), binomial(m + by, k))?;
                let term = a[i].prod_scalar(factor);
                sum = Some(match sum {
                    None => term,
                    Some(s) => s.add(&term),
                });
            }
            Ok(sum.expect("at least one term"))
        })
        .collect()
}

/// `p` with weight one, homogeneous.
fn h<S: Scalar>(p: &Vector3<S>) -> Vector4<S> {
    Vector4::from_array([p[0], p[1], p[2], S::ONE])
}

impl<S: Scalar> NurbSurface3D<S> {
    /// The piece of the sphere of `radius` around `center` bounded by the
    /// great arcs between its points `corners`, each less than a half turn
    /// from the others: exactly, a rational patch of degree 4 along `u`
    /// and 2 along `v`.
    ///
    /// `v = 0` is the great arc from the first corner (`u = 0`) to the
    /// second (`u = 1`), in its usual form — the rational quadratic whose
    /// middle control point is where the arc's end tangents meet, weighted
    /// by the cosine of half its angle (degree-raised, which leaves it the
    /// same curve). Every `u`-line is the great arc from that arc's point to
    /// the third corner, which `v = 1` collapses to: a pole. So `u = 0` and
    /// `u = 1` are the great arcs from the first and the second corner to
    /// the third, again each in its usual form. The normal faces out of the
    /// sphere where the corners run counter-clockwise, seen from outside.
    ///
    /// Why it is exact: the great arc from a point `B` to `P`, with `b = B -
    /// C` and `p = P - C`, has weights `(1, cos(t/2), 1)`, `t` the angle
    /// between them — or any `(a, m, c)` with `m^2 / (a c) = (1 + cos t) /
    /// 2`. Along the base arc `B(u) = X(u) / beta(u)`, `X` and `beta`
    /// quadratic, `1 + cos t = gamma / (beta r^2)` with `gamma = beta r^2 +
    /// (X - beta C) · p`, also quadratic. Then the row `(X, beta)`, `tau
    /// (C delta + r^2 (X + beta p), gamma)` (`delta = gamma - beta r^2`, the
    /// middle point where the tangents meet, weighted `tau gamma`), `2 r^2
    /// tau^2 gamma (P, 1)` has that ratio for every `u` and any `tau(u)`;
    /// `tau` linear, chosen so that both side arcs come out in their usual
    /// form, makes the row polynomial of degree at most 4.
    pub fn spherical_triangle(
        center: &Vector3<S>,
        radius: S,
        corners: [Vector3<S>; 3],
    ) -> GeopResult<Self> {
        let r2 = radius.mul(radius);
        let [p1, p2, p3] = &corners;
        let (b1, b2, p) = (p1.sub(center), p2.sub(center), p3.sub(center));
        // The base arc, in its usual form.
        let one_plus_cos = S::ONE.add(b1.prod_dot(&b2).div(r2)?);
        if !one_plus_cos.definitely_greater(S::ZERO) {
            return Err(GeopError::new(format!(
                "NurbSurface::spherical_triangle: corners {p1:?} and {p2:?} could lie across the sphere from each other"
            )));
        }
        let middle = center.add(&b1.add(&b2).prod_scalar(S::ONE.div(one_plus_cos)?));
        let w = one_plus_cos.div(S::TWO)?.sqrt()?;
        let base = [
            h(p1),
            Vector4::from_array([middle[0].mul(w), middle[1].mul(w), middle[2].mul(w), w]),
            h(p2),
        ];
        // The middle column before `tau`, and `gamma`, coefficient by
        // coefficient: both are linear in the base's control points.
        let mut gamma = Vec::with_capacity(3);
        let mut inner = Vec::with_capacity(3);
        for q in &base {
            let x = Vector3::from_array([q[0], q[1], q[2]]);
            let beta = q[3];
            let delta = x.sub(&center.prod_scalar(beta)).prod_dot(&p);
            let g = beta.mul(r2).add(delta);
            if !g.definitely_greater(S::ZERO) {
                return Err(GeopError::new(format!(
                    "NurbSurface::spherical_triangle: the base arc from {p1:?} to {p2:?} could reach across the sphere from {p3:?}"
                )));
            }
            let m = center
                .prod_scalar(delta)
                .add(&x.add(&p.prod_scalar(beta)).prod_scalar(r2));
            inner.push(Vector4::from_array([m[0], m[1], m[2], g]));
            gamma.push(g);
        }
        // `tau` at both ends, so that `2 r^2 tau^2 gamma` is the base's end
        // weight, one: each side arc in its usual form.
        let tau = |g: S| S::ONE.div(S::TWO.mul(r2).mul(g).sqrt()?);
        let tau = [tau(gamma[0])?, tau(gamma[2])?];
        let tau_sq = [tau[0].mul(tau[0]), tau[0].mul(tau[1]), tau[1].mul(tau[1])];
        let pole_weights = product(
            &tau_sq,
            &gamma
                .iter()
                .map(|&g| h(p3).prod_scalar(g))
                .collect::<Vec<_>>(),
        )?;
        let columns = [
            elevate(&base, 2)?,
            elevate(&product(&tau, &inner)?, 1)?,
            pole_weights
                .iter()
                .map(|q| q.prod_scalar(S::TWO.mul(r2)))
                .collect(),
        ];
        let control_points = (0..5)
            .flat_map(|i| columns.iter().map(move |c| c[i]))
            .collect();
        let knots = |degree: usize| {
            let mut k = vec![S::ZERO; degree + 1];
            k.extend(vec![S::ONE; degree + 1]);
            k
        };
        NurbSurface3D::try_new(4, 2, control_points, knots(4), knots(2))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector3};

    use super::super::NurbSurface3D;

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// Every point of the patch lies on the sphere, its corners are where
    /// they were asked to be, and its sides lie on the great circles
    /// through them — for an octant and for a lopsided triangle.
    fn check_lies_on_the_sphere<S: Scalar>() {
        let center = v3::<S>(1.0, -2.0, 0.5);
        let radius = S::from_f64(0.7);
        let on = |x: f64, y: f64, z: f64| {
            let d = v3::<S>(x, y, z).normalize().unwrap();
            center.add(&d.prod_scalar(radius))
        };
        for corners in [
            [on(1.0, 0.0, 0.0), on(0.0, 1.0, 0.0), on(0.0, 0.0, 1.0)],
            [on(1.0, 0.2, 0.1), on(-0.3, 1.0, 0.4), on(0.2, 0.5, 1.0)],
        ] {
            let s = NurbSurface3D::spherical_triangle(&center, radius, corners).unwrap();
            for i in 0..=6 {
                for j in 0..=6 {
                    let (u, v) = (S::from_f64(i as f64 / 6.0), S::from_f64(j as f64 / 6.0));
                    let p = s.evaluate(u, v).unwrap();
                    let r = p.sub(&center).norm();
                    assert!(r.could_be_equal(radius), "({i}, {j}): radius {r:?}");
                }
            }
            let at = |u: f64, v: f64| s.evaluate(S::from_f64(u), S::from_f64(v)).unwrap();
            assert!(at(0.0, 0.0).could_be_equal(&corners[0]));
            assert!(at(1.0, 0.0).could_be_equal(&corners[1]));
            assert!(at(0.3, 1.0).could_be_equal(&corners[2]));
            // A side lies on its great circle: in the plane through the
            // center and its two ends.
            for (a, b, side) in [
                (0, 1, (None, Some(0.0))),
                (0, 2, (Some(0.0), None)),
                (1, 2, (Some(1.0), None)),
            ] {
                let n = corners[a].sub(&center).prod_cross(&corners[b].sub(&center));
                for k in 0..=4 {
                    let t = k as f64 / 4.0;
                    let p = match side {
                        (Some(u), None) => at(u, t),
                        (None, Some(v)) => at(t, v),
                        _ => unreachable!(),
                    };
                    assert!(p.sub(&center).prod_dot(&n).could_be_equal(S::ZERO));
                }
            }
            // Counter-clockwise seen from outside: the normal faces out.
            let (u, v) = (S::from_f64(0.4), S::from_f64(0.3));
            let p = s.evaluate(u, v).unwrap();
            assert!(
                s.normal(u, v)
                    .unwrap()
                    .prod_dot(&p.sub(&center))
                    .definitely_greater(S::ZERO)
            );
        }
    }
    #[test]
    fn lies_on_the_sphere() {
        for_all_scalars!(check_lies_on_the_sphere);
    }
}
