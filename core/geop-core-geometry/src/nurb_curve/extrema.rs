//! Where a curve's coordinate along a direction is stationary: its extremes
//! along that direction, and the turning points of its projection.
//!
//! For a rational piece `C = H / W`, the coordinate `A / W` along a
//! direction `n` (`A = n · H`) has derivative `(A' W - A W') / W²`. The
//! weights are positive, so its sign is that of the numerator
//! `N = A' W - A W'`, a polynomial. On a Bézier piece of degree `p`, `A'`
//! has degree `p - 1` and `W` degree `p`, and their product's Bernstein
//! coefficients follow from the factors' by
//! `B_i^m B_j^n = C(m, i) C(n, j) / C(m + n, i + j) B_{i+j}^{m+n}` — no
//! evaluation, no sampling. Its roots are then isolated by subdividing the
//! coefficients (de Casteljau at the middle): a piece whose coefficients all
//! have one definite sign has none (the convex hull property).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

use super::NurbCurve;

/// `n` choose `k`, exactly.
fn binomial(n: usize, k: usize) -> i64 {
    (0..k).fold(1i64, |acc, i| acc * (n - i) as i64 / (i + 1) as i64)
}

/// The Bernstein coefficients (degree `2p - 1`) of `A' W - A W'`, up to the
/// positive factor `p / (b - a)`, for a Bézier piece of degree `p` with
/// coordinate coefficients `a` and weights `w`.
fn derivative_numerator<S: Scalar>(a: &[S], w: &[S]) -> GeopResult<Vec<S>> {
    let p = a.len() - 1;
    let m = 2 * p - 1;
    let mut n = vec![S::ZERO; m + 1];
    for i in 0..p {
        let (da, dw) = (a[i + 1].sub(a[i]), w[i + 1].sub(w[i]));
        for j in 0..=p {
            let factor = S::from_ratio(
                binomial(p - 1, i) * binomial(p, j),
                binomial(m, i + j),
            )?;
            let term = da.mul(w[j]).sub(dw.mul(a[j]));
            n[i + j] = n[i + j].add(factor.mul(term));
        }
    }
    Ok(n)
}

/// The Bernstein coefficients of the two halves of a polynomial, split at
/// the middle of its interval (de Casteljau).
fn halves<S: Scalar>(coefficients: &[S]) -> GeopResult<(Vec<S>, Vec<S>)> {
    let mut row = coefficients.to_vec();
    let mut left = Vec::with_capacity(row.len());
    let mut right = Vec::with_capacity(row.len());
    left.push(row[0]);
    right.push(row[row.len() - 1]);
    while row.len() > 1 {
        row = row
            .windows(2)
            .map(|pair| pair[0].add(pair[1]).div(S::TWO))
            .collect::<GeopResult<_>>()?;
        left.push(row[0]);
        right.push(row[row.len() - 1]);
    }
    right.reverse();
    Ok((left, right))
}

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// Every parameter at which the curve's coordinate along `direction`
    /// is stationary — its tangent perpendicular to `direction` — at an
    /// isolated point, as a parameter box: the curve's extremes along
    /// `direction`, and where its projection onto a plane containing
    /// `direction` turns back on itself. Sorted, and each box at most
    /// `min_subdivision_size` wide unless several merged.
    ///
    /// A stretch along which the coordinate is constant (a straight piece
    /// perpendicular to `direction`) has no isolated stationary point and
    /// contributes none. A box is a candidate, not a proof: it is where the
    /// search stopped being able to exclude one. Exhausting `max_nodes` is an
    /// error, since the result would be incomplete. `C` must be `D - 1`.
    pub fn stationary_parameters<const C: usize>(
        &self,
        direction: &Vector<S, C>,
        max_nodes: usize,
        min_subdivision_size: S,
    ) -> GeopResult<Vec<S>> {
        if C + 1 != D {
            return Err(GeopError::new(format!(
                "stationary_parameters: a direction of {C} components for a curve of {} \
                 dimensions",
                D - 1
            )));
        }
        let mut boxes: Vec<(S, S)> = Vec::new();
        let mut explored = 0usize;
        for piece in self.bezier_pieces()? {
            let p = piece.degree;
            if p == 0 || piece.control_points.len() != p + 1 {
                continue;
            }
            let a: Vec<S> = piece
                .control_points
                .iter()
                .map(|q| direction.prod_dot(&q.head::<C>()))
                .collect();
            let w: Vec<S> = piece.control_points.iter().map(|q| q[D - 1]).collect();
            let (lo, hi) = piece.domain();
            let mut stack = vec![(derivative_numerator(&a, &w)?, lo, hi)];
            while let Some((coefficients, lo, hi)) = stack.pop() {
                explored += 1;
                if explored > max_nodes {
                    return Err(GeopError::new(format!(
                        "stationary_parameters: exhausted max_nodes={max_nodes} along \
                         {direction:?}; the result would be incomplete"
                    )));
                }
                let positive = coefficients.iter().all(|c| c.definitely_greater(S::ZERO));
                let negative = coefficients.iter().all(|c| c.definitely_less(S::ZERO));
                let constant = coefficients.iter().all(|c| c.could_be_equal(S::ZERO));
                if positive || negative || constant {
                    continue;
                }
                if !hi.sub(lo).definitely_greater(min_subdivision_size) {
                    boxes.push((lo, hi));
                    continue;
                }
                // Where to cut is the search's own free choice.
                let mid = lo.add(hi).div(S::TWO)?.sharpen();
                let (left, right) = halves(&coefficients)?;
                stack.push((right, mid, hi));
                stack.push((left, lo, mid));
            }
        }
        boxes.sort_by(|x, y| x.0.to_f64().total_cmp(&y.0.to_f64()));
        let mut merged: Vec<(S, S)> = Vec::new();
        for (lo, hi) in boxes {
            match merged.last_mut() {
                Some(last) if !lo.definitely_greater(last.1) => last.1 = last.1.max(hi),
                _ => merged.push((lo, hi)),
            }
        }
        Ok(merged.into_iter().map(|(lo, hi)| lo.union(hi)).collect())
    }
}

#[cfg(test)]
mod tests {
    use crate::{nurb_curve::NurbCurve, shape::Arc, shape::Circle};
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// A unit circle in the xy plane is extreme along x at `(±1, 0)` and
    /// along y at `(0, ±1)`, and nowhere along z.
    fn check_circle_extremes<S: Scalar>() {
        let circle = Arc {
            circle: Circle {
                center: v3(0.0, 0.0, 0.0),
                normal: v3(0.0, 0.0, 1.0),
                radius: S::ONE,
            },
            start: v3(1.0, 0.0, 0.0),
            end: v3(1.0, 0.0, 0.0),
        }
        .to_curve()
        .unwrap();
        let min = S::from_f64(1e-9);
        for k in 0..2 {
            let mut direction = v3::<S>(0.0, 0.0, 0.0);
            direction[k] = S::ONE;
            let ts = circle.stationary_parameters(&direction, 1000, min).unwrap();
            let values: Vec<S> = ts
                .iter()
                .map(|t| circle.evaluate(t.midpoint()).unwrap()[k])
                .collect();
            assert!(values.iter().all(|v| v.abs().could_be_equal(S::ONE)), "{values:?}");
            assert!(values.iter().any(|v| v.could_be_equal(S::ONE)), "{values:?}");
            assert!(values.iter().any(|v| v.could_be_equal(S::ONE.neg())), "{values:?}");
        }
        let ts = circle
            .stationary_parameters(&v3::<S>(0.0, 0.0, 1.0), 1000, min)
            .unwrap();
        assert!(ts.is_empty(), "{ts:?}");
    }
    #[test]
    fn circle_extremes() {
        for_all_scalars!(check_circle_extremes);
    }

    /// A straight line has no isolated stationary point along any
    /// direction, including one it is perpendicular to.
    fn check_line_has_none<S: Scalar>() {
        let line = NurbCurve::try_new(
            1,
            vec![
                Vector4::from_array([S::ZERO, S::ZERO, S::ZERO, S::ONE]),
                Vector4::from_array([S::ZERO, S::TWO, S::ZERO, S::ONE]),
            ],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        let min = S::from_f64(1e-9);
        for k in 0..3 {
            let mut direction = v3::<S>(0.0, 0.0, 0.0);
            direction[k] = S::ONE;
            assert!(line.stationary_parameters(&direction, 100, min).unwrap().is_empty());
        }
    }
    #[test]
    fn line_has_none() {
        for_all_scalars!(check_line_has_none);
    }
}
