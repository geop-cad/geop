//! The geometry of sketch entities, generic over [`Scalar`] so every formula
//! serves both the residuals (differentiated with [`crate::dual::Dual`])
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

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};

/// A 2-D vector.
#[derive(Clone, Copy, Debug)]
pub struct V<T> {
    pub x: T,
    pub y: T,
}

// `add`/`sub`/`dot`/... mirror the kernel's own `Vector` API, so the same
// formulas read the same way here as they do there.
#[allow(clippy::should_implement_trait)]
impl<T: Scalar> V<T> {
    pub fn new(x: T, y: T) -> Self {
        V { x, y }
    }
    pub fn cst(p: [f64; 2]) -> Self {
        V::new(T::from_f64(p[0]), T::from_f64(p[1]))
    }
    pub fn add(self, o: Self) -> Self {
        V::new(self.x.add(o.x), self.y.add(o.y))
    }
    pub fn sub(self, o: Self) -> Self {
        V::new(self.x.sub(o.x), self.y.sub(o.y))
    }
    pub fn scale(self, s: T) -> Self {
        V::new(self.x.mul(s), self.y.mul(s))
    }
    pub fn dot(self, o: Self) -> T {
        self.x.mul(o.x).add(self.y.mul(o.y))
    }
    pub fn cross(self, o: Self) -> T {
        self.x.mul(o.y).sub(self.y.mul(o.x))
    }
    pub fn norm(self) -> GeopResult<T> {
        self.dot(self).sqrt()
    }
    /// Rotated 90 degrees counter-clockwise.
    pub fn perp(self) -> Self {
        V::new(T::ZERO.sub(self.y), self.x)
    }
    pub fn unit(self) -> GeopResult<Self> {
        Ok(self.scale(T::ONE.div(self.norm()?)?))
    }
    /// Rotated by the angle with cosine `c` and sine `s`.
    pub fn rotate(self, c: T, s: T) -> Self {
        V::new(
            self.x.mul(c).sub(self.y.mul(s)),
            self.x.mul(s).add(self.y.mul(c)),
        )
    }
    pub fn value(self) -> [f64; 2] {
        [self.x.to_f64(), self.y.to_f64()]
    }
}

/// A circular arc from `s` to `e`, turning counter-clockwise by `2 * half`
/// (clockwise if negative).
#[derive(Clone, Copy, Debug)]
pub struct Arc<T> {
    pub s: V<T>,
    pub e: V<T>,
    pub half: T,
}

impl<T: Scalar> Arc<T> {
    pub fn chord(&self) -> V<T> {
        self.e.sub(self.s)
    }
    pub fn chord_length(&self) -> GeopResult<T> {
        self.chord().norm()
    }
    pub fn chord_mid(&self) -> V<T> {
        self.s.add(self.e).scale(T::from_f64(0.5))
    }
    /// Unit normal to the chord, pointing to its left: the side the center is
    /// on for a counter-clockwise minor arc.
    pub fn left(&self) -> GeopResult<V<T>> {
        Ok(self.chord().unit()?.perp())
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
    pub fn center(&self) -> GeopResult<V<T>> {
        let d = self
            .chord_length()?
            .mul(T::from_f64(0.5))
            .mul(self.half.cos())
            .div(self.half.sin())?;
        Ok(self.chord_mid().add(self.left()?.scale(d)))
    }
    /// The point halfway along the arc: `chord_mid - left * (L / 2) tan(half / 2)`.
    pub fn arc_mid(&self) -> GeopResult<V<T>> {
        let tan_quarter = self.half.sin().div(T::ONE.add(self.half.cos()))?;
        let sagitta = self.chord_length()?.mul(T::from_f64(0.5)).mul(tan_quarter);
        Ok(self.chord_mid().sub(self.left()?.scale(sagitta)))
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
    pub fn circle_residual(&self, p: V<T>) -> GeopResult<T> {
        let q = p.sub(self.chord_mid());
        let k = self.curvature()?;
        let l = self.chord_length()?;
        let g = k
            .mul(q.dot(q))
            .sub(T::TWO.mul(self.half.cos()).mul(self.left()?.dot(q)))
            .sub(k.mul(l).mul(l).mul(T::from_f64(0.25)));
        Ok(g.mul(T::from_f64(0.5)))
    }
    /// Unit tangent at `s`, in the direction of travel.
    pub fn tangent_start(&self) -> GeopResult<V<T>> {
        let c = self.chord().unit()?;
        Ok(c.rotate(self.half.cos(), T::ZERO.sub(self.half.sin())))
    }
    /// Unit tangent at `e`, in the direction of travel.
    pub fn tangent_end(&self) -> GeopResult<V<T>> {
        let c = self.chord().unit()?;
        Ok(c.rotate(self.half.cos(), self.half.sin()))
    }
    /// Arc length `L * half / sin(half)`.
    pub fn length(&self) -> GeopResult<T> {
        let l = self.chord_length()?;
        let h = self.half;
        Ok(if h.to_f64().abs() < 1e-4 {
            // Series of `x / sin x`, to keep the derivative finite at 0.
            l.mul(
                T::ONE
                    .add(h.mul(h).mul(T::from_f64(1.0 / 6.0)))
                    .add(h.mul(h).mul(h).mul(h).mul(T::from_f64(7.0 / 360.0))),
            )
        } else {
            l.mul(h).div(h.sin())?
        })
    }
}

/// Signed distance of `p` from the line through `a` and `b`, positive on its
/// left.
pub fn line_distance<T: Scalar>(a: V<T>, b: V<T>, p: V<T>) -> GeopResult<T> {
    let d = b.sub(a);
    d.cross(p.sub(a)).div(d.norm()?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::scalars::scal_in_f64::ScalInF64;

    fn quarter() -> Arc<ScalInF64> {
        Arc {
            s: V::new(ScalInF64::from_f64(1.0), ScalInF64::from_f64(0.0)),
            e: V::new(ScalInF64::from_f64(0.0), ScalInF64::from_f64(1.0)),
            half: ScalInF64::from_f64(std::f64::consts::FRAC_PI_4),
        }
    }

    fn close(a: [f64; 2], b: [f64; 2]) -> bool {
        (a[0] - b[0]).abs() < 1e-12 && (a[1] - b[1]).abs() < 1e-12
    }

    #[test]
    fn quarter_circle_has_unit_radius_around_origin() {
        let a = quarter();
        assert!(close(a.center().unwrap().value(), [0.0, 0.0]));
        assert!((a.radius().unwrap().to_f64() - 1.0).abs() < 1e-12);
        assert!((a.curvature().unwrap().to_f64() - 1.0).abs() < 1e-12);
        let s = std::f64::consts::FRAC_1_SQRT_2;
        assert!(close(a.arc_mid().unwrap().value(), [s, s]));
        assert!(close(a.tangent_start().unwrap().value(), [0.0, 1.0]));
        assert!(close(a.tangent_end().unwrap().value(), [-1.0, 0.0]));
        assert!((a.length().unwrap().to_f64() - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        assert!(
            a.circle_residual(V::new(ScalInF64::from_f64(-s), ScalInF64::from_f64(-s)))
                .unwrap()
                .to_f64()
                .abs()
                < 1e-12
        );
        // Outside the circle by 1: residual ≈ distance (exactly `(ρ² - 1)/2`).
        assert!(
            (a.circle_residual(V::new(ScalInF64::from_f64(2.0), ScalInF64::from_f64(0.0)))
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
        assert!(close(a.center().unwrap().value(), [1.0, 1.0]));
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
            a.circle_residual(V::new(ScalInF64::from_f64(0.5), ScalInF64::from_f64(0.5)))
                .unwrap()
                .to_f64()
                .abs()
                < 1e-12
        );
        assert!(close(a.arc_mid().unwrap().value(), [0.5, 0.5]));
    }
}
