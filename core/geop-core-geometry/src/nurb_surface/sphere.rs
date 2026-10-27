//! The piece of a sphere bounded by three great arcs
//! ([`NurbSurface3D::spherical_triangle`]): what rounds a corner where three
//! fillets of one radius meet.

use geop_core_math::{
    combinatorics::binomial,
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::NurbSurface3D;

/// The Bernstein coefficients of the product of the polynomials whose
/// Bernstein coefficients are `a` (scalars) and `b` (homogeneous points):
/// of degree the sum of theirs. A coefficient made of one term of factor
/// one — at both ends — is taken as it is: multiplying by an exact one
/// would only widen it by rounding.
fn product(a: &[Scalar], b: &[Vector4]) -> GeopResult<Vec<Vector4>> {
    let (m, n) = (a.len() - 1, b.len() - 1);
    (0..=m + n)
        .map(|k| {
            let mut sum: Option<Vector4> = None;
            let first = k.saturating_sub(n);
            for (i, &ai) in a.iter().enumerate().take(k.min(m) + 1).skip(first) {
                let j = k - i;
                let factor =
                    Scalar::from_ratio(binomial(m, i) * binomial(n, j), binomial(m + n, k))?
                        .mul(ai);
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
fn elevate(a: &[Vector4], by: usize) -> GeopResult<Vec<Vector4>> {
    let m = a.len() - 1;
    (0..=m + by)
        .map(|k| {
            if k == 0 {
                return Ok(a[0]);
            }
            if k == m + by {
                return Ok(a[m]);
            }
            let mut sum: Option<Vector4> = None;
            let first = k.saturating_sub(by);
            for (i, ai) in a.iter().enumerate().take(k.min(m) + 1).skip(first) {
                let factor =
                    Scalar::from_ratio(binomial(m, i) * binomial(by, k - i), binomial(m + by, k))?;
                let term = ai.prod_scalar(factor);
                sum = Some(match sum {
                    None => term,
                    Some(s) => s.add(&term),
                });
            }
            Ok(sum.expect("at least one term"))
        })
        .collect()
}

impl NurbSurface3D {
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
    ///
    /// The rows multiply the center and the corners by weights and by
    /// each other, so it is built about the center's middle, a free choice
    /// of origin, and moved back by one rounding: built where it lies, a
    /// corner of a block at `(1e4, 1e4, 0)` came out with control points
    /// `8e-4` wide, and at `1e5` with no normal left at all.
    pub fn spherical_triangle(
        center: &Vector3,
        radius: Scalar,
        corners: [Vector3; 3],
    ) -> GeopResult<Self> {
        let origin = center.sharpen();
        let center = &center.sub(&origin);
        let corners = corners.map(|c| c.sub(&origin));
        let r2 = radius.mul(radius);
        let [p1, p2, p3] = &corners;
        let (b1, b2, p) = (p1.sub(center), p2.sub(center), p3.sub(center));
        // The base arc, in its usual form.
        let one_plus_cos = Scalar::ONE.add(b1.prod_dot(&b2).div(r2)?);
        if !one_plus_cos.definitely_greater(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "NurbSurface::spherical_triangle: corners {p1:?} and {p2:?} could lie across the sphere from each other"
            )));
        }
        let middle = center.add(&b1.add(&b2).prod_scalar(Scalar::ONE.div(one_plus_cos)?));
        let w = one_plus_cos.div(Scalar::TWO)?.sqrt()?;
        let base = [
            p1.homogeneous(Scalar::ONE),
            middle.homogeneous(w),
            p2.homogeneous(Scalar::ONE),
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
            if !g.definitely_greater(Scalar::ZERO) {
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
        let tau = |g: Scalar| Scalar::ONE.div(Scalar::TWO.mul(r2).mul(g).sqrt()?);
        let tau = [tau(gamma[0])?, tau(gamma[2])?];
        let tau_sq = [tau[0].mul(tau[0]), tau[0].mul(tau[1]), tau[1].mul(tau[1])];
        let pole_weights = product(
            &tau_sq,
            &gamma
                .iter()
                .map(|&g| p3.homogeneous(Scalar::ONE).prod_scalar(g))
                .collect::<Vec<_>>(),
        )?;
        let columns = [
            elevate(&base, 2)?,
            elevate(&product(&tau, &inner)?, 1)?,
            pole_weights
                .iter()
                .map(|q| q.prod_scalar(Scalar::TWO.mul(r2)))
                .collect(),
        ];
        let control_points = (0..5)
            .flat_map(|i| columns.iter().map(move |c| c[i]))
            .collect();
        let knots = |degree: usize| {
            let mut k = vec![Scalar::ZERO; degree + 1];
            k.extend(vec![Scalar::ONE; degree + 1]);
            k
        };
        Ok(NurbSurface3D::try_new(4, 2, control_points, knots(4), knots(2))?.translate(origin))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use super::super::NurbSurface3D;

    fn v3(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([
            Scalar::from_f64(x),
            Scalar::from_f64(y),
            Scalar::from_f64(z),
        ])
    }

    /// A corner's ball far from the origin, as a block placed at
    /// `(1e5, -1e5, 0)` has with every edge rounded: its patch is as
    /// narrow as its corners allow, and has a normal. Built where it lies,
    /// its weighted control points came out `1e-5` wide here.
    #[test]
    fn far_from_the_origin_is_as_narrow() {
        let radius = Scalar::from_f64(2.0);
        for [x, y, z] in [[2.0, 2.0, 2.0], [1e5 + 2.0, -1e5 + 2.0, 2.0]] {
            let center = v3(x, y, z);
            let corners = [v3(x - 2.0, y, z), v3(x, y - 2.0, z), v3(x, y, z - 2.0)];
            let s = NurbSurface3D::spherical_triangle(&center, radius, corners).unwrap();
            let widest = (s.control_points.iter())
                .flat_map(|p| (0..4).map(move |c| p[c].width().to_f64()))
                .fold(0.0, f64::max);
            assert!(widest < 1e-9, "at {center:?}: {widest:e}");
            let (u, v) = (Scalar::from_f64(0.4), Scalar::from_f64(0.3));
            assert!(s.normal(u, v).is_ok(), "at {center:?}");
        }
    }

    /// Every point of the patch lies on the sphere, its corners are where
    /// they were asked to be, and its sides lie on the great circles
    /// through them — for an octant and for a lopsided triangle.
    #[test]
    fn lies_on_the_sphere() {
        let center = v3(1.0, -2.0, 0.5);
        let radius = Scalar::from_f64(0.7);
        let on = |x: f64, y: f64, z: f64| {
            let d = v3(x, y, z).normalize().unwrap();
            center.add(&d.prod_scalar(radius))
        };
        for corners in [
            [on(1.0, 0.0, 0.0), on(0.0, 1.0, 0.0), on(0.0, 0.0, 1.0)],
            [on(1.0, 0.2, 0.1), on(-0.3, 1.0, 0.4), on(0.2, 0.5, 1.0)],
        ] {
            let s = NurbSurface3D::spherical_triangle(&center, radius, corners).unwrap();
            for i in 0..=6 {
                for j in 0..=6 {
                    let (u, v) = (
                        Scalar::from_f64(i as f64 / 6.0),
                        Scalar::from_f64(j as f64 / 6.0),
                    );
                    let p = s.evaluate(u, v).unwrap();
                    let r = p.sub(&center).norm();
                    assert!(r.could_be_equal(radius), "({i}, {j}): radius {r:?}");
                }
            }
            let at = |u: f64, v: f64| {
                s.evaluate(Scalar::from_f64(u), Scalar::from_f64(v))
                    .unwrap()
            };
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
                    assert!(p.sub(&center).prod_dot(&n).could_be_equal(Scalar::ZERO));
                }
            }
            // Counter-clockwise seen from outside: the normal faces out.
            let (u, v) = (Scalar::from_f64(0.4), Scalar::from_f64(0.3));
            let p = s.evaluate(u, v).unwrap();
            assert!(
                s.normal(u, v)
                    .unwrap()
                    .prod_dot(&p.sub(&center))
                    .definitely_greater(Scalar::ZERO)
            );
        }
    }
}
