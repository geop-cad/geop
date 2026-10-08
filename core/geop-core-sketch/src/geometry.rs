//! The geometry of sketch entities, generic over [`Scalar`] so every formula
//! serves both the residuals (differentiated with [`geop_core_math::dual::Dual`])
//! and plain evaluation (profiles, rendering).
//!
//! An arc is stored as `(start, end, sweep)`. Its signed curvature follows
//! from those as `k = 2 sin(sweep / 2) / |end - start|`, so `(start, end,
//! sweep)` and `(start, end, curvature)` describe the same arc — but only the
//! sweep is smooth through a straight arc (`k = 0`) and a half circle, and
//! only the sweep tells a major arc from the minor arc with the same
//! curvature. The formulas below are written in the half sweep `θ` and never
//! divide by `sin θ` where avoidable, so a nearly straight arc stays
//! well-conditioned.
//!
//! Every division and square root here is fallible — [`Scalar::div`] and
//! [`Scalar::sqrt`] refuse a divisor or radicand that could be zero/negative
//! — so a degenerate configuration (a zero-length chord, a collapsed arc)
//! surfaces as a [`GeopResult`] error instead of silently producing an
//! `inf`/`nan` that would then have to be caught downstream.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};

/// A circular arc from `s` to `e`, turning counter-clockwise by `2 * half`
/// (clockwise if negative).
#[derive(Clone, Copy, Debug)]
pub struct Arc<T> {
    pub s: Vector2<T>,
    pub e: Vector2<T>,
    pub half: T,
}

impl<T: Scalar> Arc<T> {
    pub fn chord(&self) -> Vector2<T> {
        self.e.sub(&self.s)
    }
    pub fn chord_length(&self) -> GeopResult<T> {
        self.chord().try_norm()
    }
    pub fn chord_mid(&self) -> Vector2<T> {
        self.s.add(&self.e).prod_scalar(half::<T>())
    }
    /// Unit normal to the chord, pointing to its left: the side the center is
    /// on for a counter-clockwise minor arc.
    pub fn left(&self) -> GeopResult<Vector2<T>> {
        Ok(self.chord().normalize()?.perp())
    }
    pub fn curvature(&self) -> GeopResult<T> {
        T::TWO.mul(self.half.sin()).div(self.chord_length()?)
    }
    /// `|radius|`. Infinite for a straight arc.
    pub fn radius(&self) -> GeopResult<T> {
        self.chord_length()?.div(T::TWO.mul(self.half.sin().abs()))
    }
    /// Center: `chord_mid + left * (L / 2) cot(half)`. Infinitely far for a
    /// straight arc, so only for constraints that are meaningless there
    /// anyway (concentricity, tangency to a circle).
    pub fn center(&self) -> GeopResult<Vector2<T>> {
        let d = self
            .chord_length()?
            .mul(half::<T>())
            .mul(self.half.cos())
            .div(self.half.sin())?;
        Ok(self.chord_mid().add(&self.left()?.prod_scalar(d)))
    }
    /// The point halfway along the arc: `chord_mid - left * (L / 2) tan(half / 2)`.
    pub fn arc_mid(&self) -> GeopResult<Vector2<T>> {
        let tan_quarter = self.half.sin().div(T::ONE.add(self.half.cos()))?;
        let sagitta = self.chord_length()?.mul(half::<T>()).mul(tan_quarter);
        Ok(self.chord_mid().sub(&self.left()?.prod_scalar(sagitta)))
    }
    /// Signed distance-like residual of `p` against the arc's full circle,
    /// finite and smooth for every `half` including a straight arc.
    ///
    /// With `q = p - chord_mid`, the circle is `|q|^2 - 2 d (left . q) - L^2/4
    /// = 0` for `d` the center's offset along `left`. Multiplying by the
    /// curvature `k` (and using `k d = cos(half)`) gives
    /// `G = k |q|^2 - 2 cos(half) (left . q) - k L^2 / 4`, which is a line's
    /// equation at `k = 0`. Near the circle `G ≈ ±2 dist`, so `G / 2` is a
    /// distance.
    pub fn circle_residual(&self, p: Vector2<T>) -> GeopResult<T> {
        let q = p.sub(&self.chord_mid());
        let k = self.curvature()?;
        let l = self.chord_length()?;
        let g = k
            .mul(q.norm_sq())
            .sub(T::TWO.mul(self.half.cos()).mul(self.left()?.prod_dot(&q)))
            .sub(k.mul(l).mul(l).mul(half::<T>().mul(half::<T>())));
        Ok(g.mul(half::<T>()))
    }
    /// Unit tangent at `s`, in the direction of travel.
    pub fn tangent_start(&self) -> GeopResult<Vector2<T>> {
        let c = self.chord().normalize()?;
        Ok(c.rotate(self.half.cos(), self.half.sin().neg()))
    }
    /// Unit tangent at `e`, in the direction of travel.
    pub fn tangent_end(&self) -> GeopResult<Vector2<T>> {
        let c = self.chord().normalize()?;
        Ok(c.rotate(self.half.cos(), self.half.sin()))
    }
    /// Arc length `L * half / sin(half)`.
    ///
    /// Where `sin(half)` could be zero — a straight arc — the quotient is not
    /// even defined, and the series `x / sin x = 1 + x²/6 + 7x⁴/360 + R(x)`
    /// stands in, finite with a finite slope through `x = 0`. Its
    /// coefficients are all positive and shrink by about `π²` each, so for
    /// `|x| ≤ 1` the rest `R` lies in `[0, x⁶/300]` and its slope in
    /// `6x⁵ [0, 1/300]`: the term enclosing both is added, so the series is as
    /// honest an enclosure as the quotient.
    pub fn length(&self) -> GeopResult<T> {
        let l = self.chord_length()?;
        let h = self.half;
        let sin = h.sin();
        if sin.definitely_not_equal(T::ZERO) || !h.abs().definitely_less(T::ONE) {
            return l.mul(h).div(sin);
        }
        let ratio = |num, den| T::from_ratio(num, den).expect("a positive denominator");
        let h2 = h.mul(h);
        let h4 = h2.mul(h2);
        let rest = T::ZERO.union(ratio(1, 300)).mul(h4).mul(h2);
        Ok(l.mul(
            T::ONE
                .add(h2.mul(ratio(1, 6)))
                .add(h4.mul(ratio(7, 360)))
                .add(rest),
        ))
    }
}

/// `1/2`.
fn half<T: Scalar>() -> T {
    T::ONE.div(T::TWO).expect("2 is not zero")
}

/// Signed distance of `p` from the line through `a` and `b`, positive on its
/// left.
pub fn line_distance<T: Scalar>(a: Vector2<T>, b: Vector2<T>, p: Vector2<T>) -> GeopResult<T> {
    let d = b.sub(&a);
    d.prod_cross(&p.sub(&a)).div(d.try_norm()?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::scalars::scal_in_f64::ScalInF64;

    fn vec2(x: ScalInF64, y: ScalInF64) -> Vector2<ScalInF64> {
        Vector2::from_array([x, y])
    }

    fn quarter() -> Arc<ScalInF64> {
        Arc {
            s: vec2(ScalInF64::from_f64(1.0), ScalInF64::from_f64(0.0)),
            e: vec2(ScalInF64::from_f64(0.0), ScalInF64::from_f64(1.0)),
            half: ScalInF64::from_f64(std::f64::consts::FRAC_PI_4),
        }
    }

    fn close(a: Vector2<ScalInF64>, b: [f64; 2]) -> bool {
        (a[0].to_f64() - b[0]).abs() < 1e-12 && (a[1].to_f64() - b[1]).abs() < 1e-12
    }

    #[test]
    fn quarter_circle_has_unit_radius_around_origin() {
        let a = quarter();
        assert!(close(a.center().unwrap(), [0.0, 0.0]));
        assert!((a.radius().unwrap().to_f64() - 1.0).abs() < 1e-12);
        assert!((a.curvature().unwrap().to_f64() - 1.0).abs() < 1e-12);
        let s = std::f64::consts::FRAC_1_SQRT_2;
        assert!(close(a.arc_mid().unwrap(), [s, s]));
        assert!(close(a.tangent_start().unwrap(), [0.0, 1.0]));
        assert!(close(a.tangent_end().unwrap(), [-1.0, 0.0]));
        assert!((a.length().unwrap().to_f64() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!(
            a.circle_residual(vec2(ScalInF64::from_f64(-s), ScalInF64::from_f64(-s)))
                .unwrap()
                .to_f64()
                .abs()
                < 1e-12
        );
        // Outside the circle by 1: residual ≈ distance (exactly `(ρ² - 1)/2`).
        assert!(
            (a.circle_residual(vec2(ScalInF64::from_f64(2.0), ScalInF64::from_f64(0.0)))
                .unwrap()
                .to_f64()
                - 1.5)
                .abs()
                < 1e-12
        );
    }

    #[test]
    fn major_arc_center_is_right_of_chord() {
        let a = Arc {
            half: ScalInF64::from_f64(3.0 * std::f64::consts::FRAC_PI_4),
            ..quarter()
        };
        assert!(close(a.center().unwrap(), [1.0, 1.0]));
        assert!((a.length().unwrap().to_f64() - 3.0 * std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn straight_arc_is_its_chord() {
        let a = Arc {
            half: ScalInF64::from_f64(0.0),
            ..quarter()
        };
        assert!((a.length().unwrap().to_f64() - 2f64.sqrt()).abs() < 1e-12);
        // On the chord's line: zero; to its left by `h`: `-h` (G ≈ -2 left·q).
        assert!(
            a.circle_residual(vec2(ScalInF64::from_f64(0.5), ScalInF64::from_f64(0.5)))
                .unwrap()
                .to_f64()
                .abs()
                < 1e-12
        );
        assert!(close(a.arc_mid().unwrap(), [0.5, 0.5]));
    }
}
