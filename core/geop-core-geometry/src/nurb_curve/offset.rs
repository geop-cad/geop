//! How far a curve fitted to a plane curve's offset is from it, proven.
//!
//! The offset of a plane NURBS curve `S` by `d`, `O = S + d N` with `N` its
//! unit normal, is no NURBS: a spline fitted to it only comes near it. How
//! near is bounded here, box by box over the fit's domain, by the mean
//! value form of the miss `e = F − O`: its value in the middle of the box,
//! plus its derivative over the box times the half width. That derivative
//! is the fit's less the offset's, `O' = S' + d N'` with
//! `N' = (S''⊥ |S'|² − S'⊥ (S'·S'')) / |S'|³`, all enclosed over the box:
//! the bound is honest wherever it is, and tight once the boxes are small
//! against the fit's own error.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector2,
};

use super::NurbCurve2D;

/// How many times a box of the fit's domain is split in two at most: the
/// bound converges with the square of the box's width.
const MISS_SPLITS: usize = 12;

/// `v` turned a quarter counter-clockwise.
fn left(v: Vector2) -> Vector2 {
    Vector2::from_array([v[1].neg(), v[0]])
}

impl NurbCurve2D {
    /// A bound on how far `fit`, over its domain, is from this curve's
    /// offset by `distance` to its left — `fit` at the start of its domain
    /// standing for the offset of this curve at `t0`, at its end for that
    /// at `t1`, affinely between — proven over boxes of `fit`'s domain,
    /// each split in two while its bound exceeds `tolerance` (at most
    /// [`MISS_SPLITS`] times; then the bound is only wider) and its middle
    /// is not definitely further off than that. Fails where the curve has
    /// no tangent.
    pub fn offset_miss(
        &self,
        distance: Scalar,
        (t0, t1): (Scalar, Scalar),
        fit: &NurbCurve2D,
        tolerance: Scalar,
    ) -> GeopResult<Scalar> {
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "NurbCurve2D::offset_miss(distance={distance:?}, t=({t0:?}, {t1:?}))"
            ))
        };
        let (s0, s1) = fit.domain();
        let rate = t1.sub(t0).div(s1.sub(s0)).map_err(ctx)?;
        let along = |s: Scalar| t0.add(s.sub(s0).mul(rate));
        let offset_at = |t: Scalar| -> GeopResult<Vector2> {
            let normal = left(self.tangent(t)?).normalize()?;
            Ok(self.evaluate(t)?.add(&normal.prod_scalar(distance)))
        };
        // The bound over the box from `a` to `b`, and whether its middle
        // alone is off by more than `tolerance`: then splitting it cannot
        // bring it under.
        let miss_over = |a: Scalar, b: Scalar| -> GeopResult<(Scalar, bool)> {
            let middle = a.add(b).div(Scalar::TWO)?.sharpen();
            let here = fit.evaluate(middle)?.sub(&offset_at(along(middle))?);
            let s = a.union(b);
            let t = along(s);
            let (first, second) = (self.tangent(t)?, self.second_derivative(t)?);
            let speed_sq = first.prod_dot(&first);
            let speed = speed_sq.sqrt()?;
            let turning = left(second)
                .prod_scalar(speed_sq)
                .sub(&left(first).prod_scalar(first.prod_dot(&second)))
                .prod_scalar(Scalar::ONE.div(speed_sq.mul(speed))?);
            let slope = fit
                .tangent(s)?
                .sub(&first.add(&turning.prod_scalar(distance)).prod_scalar(rate));
            let half = b.sub(a).div(Scalar::TWO)?;
            let reach = half.neg().union(half);
            let e = here.add(&slope.prod_scalar(reach));
            let (x, y) = (e[0].abs(), e[1].abs());
            let off = here.prod_dot(&here).sqrt()?.definitely_greater(tolerance);
            Ok((x.mul(x).add(y.mul(y)).sqrt()?.upper(), off))
        };
        let breaks = fit.breakpoints();
        let mut stack: Vec<(Scalar, Scalar, usize)> =
            breaks.windows(2).map(|w| (w[0], w[1], 0)).collect();
        let mut worst = Scalar::ZERO;
        while let Some((a, b, depth)) = stack.pop() {
            // Over a box too wide its enclosures are too: the speed could be
            // zero over it, and the bound is not decided there yet.
            let miss = match miss_over(a, b) {
                Err(_) if depth < MISS_SPLITS => None,
                miss => Some(miss.map_err(ctx)?),
            };
            if miss.is_none_or(|(m, off)| m.definitely_greater(tolerance) && !off)
                && depth < MISS_SPLITS
            {
                let middle = a.add(b).div(Scalar::TWO).map_err(ctx)?.sharpen();
                stack.push((a, middle, depth + 1));
                stack.push((middle, b, depth + 1));
                continue;
            }
            worst = worst.max(miss.expect("decided at the last split").0);
        }
        Ok(worst)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_fixtures::knots;

    fn h(x: f64, y: f64, w: f64) -> geop_core_math::vector::Vector3 {
        geop_core_math::vector::Vector3::from_array([x * w, y * w, w].map(Scalar::from_f64))
    }

    /// A quarter circle of radius 2 and its offset by 0.5 inward, the
    /// quarter circle of radius 1.5, exactly: missed by no more than the
    /// tolerance; and the same offset by 0.6, by 0.1 at least.
    #[test]
    fn an_arc_offset_is_missed_by_no_more_than_it_is_off() {
        let r2 = std::f64::consts::FRAC_1_SQRT_2;
        let arc = |r: f64| {
            NurbCurve2D::try_new(
                2,
                vec![h(r, 0., 1.), h(r, r, r2), h(0., r, 1.)],
                knots(&[0., 0., 0., 1., 1., 1.]),
            )
            .unwrap()
        };
        let domain = (Scalar::ZERO, Scalar::ONE);
        let tolerance = Scalar::from_f64(1e-6);
        let exact = arc(2.).offset_miss(Scalar::from_f64(0.5), domain, &arc(1.5), tolerance);
        let exact = exact.unwrap().to_f64();
        assert!(exact < 1e-6, "{exact}");
        let off = arc(2.)
            .offset_miss(Scalar::from_f64(0.6), domain, &arc(1.5), tolerance)
            .unwrap()
            .to_f64();
        assert!(off >= 0.1, "{off}");
    }
}
