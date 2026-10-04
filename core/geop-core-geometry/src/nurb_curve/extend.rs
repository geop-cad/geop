//! Carrying a curve on past one of its ends: its end piece continued as the
//! polynomial (or, rational, the projected polynomial) it already is.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

use super::NurbCurve;
use crate::knot_insertion::{insert, is_clamped_at};

/// The control points of a Bézier piece over local parameters `[0, s]`,
/// for `points` its control points over `[0, 1]`: the left half of de
/// Casteljau's subdivision at `s`, which for `s > 1` extrapolates.
fn continued<S: Scalar, const D: usize>(points: &[Vector<S, D>], s: S) -> Vec<Vector<S, D>> {
    let mut level = points.to_vec();
    let mut out = vec![level[0]];
    for r in 1..points.len() {
        for i in 0..points.len() - r {
            level[i] = Vector::interpolate(&level[i], &level[i + 1], s);
        }
        out.push(level[0]);
    }
    out
}

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// The same curve carried on by `by` of its parameter past its end
    /// (`at_end`) or before its start: its end piece continued as the
    /// polynomial it is, so the curve stays as smooth as it was where it
    /// used to end. The domain grows by `by` on that side; the rest of the
    /// curve keeps its parametrization.
    ///
    /// The end piece is isolated as a Bézier piece by knot insertion, then
    /// subdivided at a parameter past its end — de Casteljau extrapolates
    /// as exactly as it interpolates, on the homogeneous control points, so
    /// a rational curve continues as itself: an arc continues round its
    /// circle. Fails for a knot vector not clamped at that end, and where
    /// the continued weights stop being positive — an arc carried round
    /// too far.
    pub fn extended(&self, at_end: bool, by: S) -> GeopResult<Self> {
        let p = self.degree;
        if !is_clamped_at(&self.knot_vector, p, !at_end) {
            return Err(GeopError::new(format!(
                "NurbCurve::extended: the knot vector {:?} is not clamped at the end to extend",
                self.knot_vector
            )));
        }
        if !by.definitely_greater(S::ZERO) {
            return Err(GeopError::new(format!(
                "NurbCurve::extended: extending by {by:?}, which is not greater than zero"
            )));
        }
        if !at_end {
            // Mirrored: the start continued is the end of the reversed
            // curve continued, which puts the domain's new start at the
            // reversed curve's new end.
            let (t0, t1) = self.domain();
            let reversed = self.reverse().extended(true, by)?;
            let mut out = reversed.reverse();
            // `reverse` keeps the domain `[t0, t1 + by]`; shift it to
            // `[t0 - by, t1]`.
            for k in &mut out.knot_vector {
                *k = k.sub(by);
            }
            debug_assert!(out.domain().1.could_be_equal(t1) && out.domain().0.could_be_equal(t0.sub(by)));
            return Ok(out);
        }
        let n = self.control_points.len();
        let end = self.knot_vector[n];
        // Where the end piece starts: the last knot before the end.
        let start = self.knot_vector[..n]
            .iter()
            .rev()
            .find(|k| k.definitely_less(end))
            .copied()
            .ok_or_else(|| GeopError::new("NurbCurve::extended: the curve has no span"))?;
        let mut knots = self.knot_vector.clone();
        let mut rows = [self.control_points.clone()];
        let multiplicity = knots.iter().filter(|k| k.could_be_equal(start)).count();
        for _ in multiplicity.min(p)..p {
            insert(&mut knots, &mut rows, p, start);
        }
        let [mut points] = rows;
        let m = points.len();
        let s = end.add(by).sub(start).div(end.sub(start))?;
        let piece = continued(&points[m - p - 1..], s);
        points.splice(m - p - 1.., piece);
        let len = knots.len();
        for k in &mut knots[len - p - 1..] {
            *k = end.add(by);
        }
        NurbCurve::try_new(p, points, knots)
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector4};

    use crate::nurb_curve::{NurbCurve, NurbCurve3D};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([x * w, y * w, z * w, w].map(S::from_f64))
    }

    /// A cubic with an interior knot, continued at either end: the same
    /// curve where it was, and on its own polynomial beyond.
    fn check_cubic_continues_as_itself<S: Scalar>() {
        let f = S::from_f64;
        let curve: NurbCurve3D<S> = NurbCurve::try_new(
            3,
            vec![
                pt(0., 0., 0., 1.),
                pt(1., 2., 0., 1.),
                pt(2., -1., 1., 1.),
                pt(3., 1., 0., 1.),
                pt(4., 0., 2., 1.),
            ],
            [0., 0., 0., 0., 0.5, 1., 1., 1., 1.].map(f).to_vec(),
        )
        .unwrap();
        let longer = curve.extended(true, f(0.25)).unwrap();
        let (t0, t1) = longer.domain();
        assert!(t0.could_be_equal(f(0.)) && t1.could_be_equal(f(1.25)));
        for i in 0..=8 {
            let t = S::from_ratio(i, 8).unwrap();
            assert!(longer.evaluate(t).unwrap().could_be_equal(&curve.evaluate(t).unwrap()));
        }
        // The last piece's polynomial, from the original piece's control
        // points at a parameter past it, by de Casteljau directly.
        let earlier = curve.extended(false, f(0.5)).unwrap();
        let (t0, t1) = earlier.domain();
        assert!(t0.could_be_equal(f(-0.5)) && t1.could_be_equal(f(1.)));
        for i in 0..=8 {
            let t = S::from_ratio(i, 8).unwrap();
            assert!(earlier.evaluate(t).unwrap().could_be_equal(&curve.evaluate(t).unwrap()));
        }
    }
    #[test]
    fn cubic_continues_as_itself() {
        for_all_scalars!(check_cubic_continues_as_itself);
    }

    /// A quarter circle continued at its end stays on its circle.
    fn check_arc_continues_round_its_circle<S: Scalar>() {
        let f = S::from_f64;
        let arc: NurbCurve3D<S> = NurbCurve::try_new(
            2,
            vec![
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., std::f64::consts::SQRT_2 / 2.0),
                pt(0., 1., 0., 1.),
            ],
            [0., 0., 0., 1., 1., 1.].map(f).to_vec(),
        )
        .unwrap();
        let longer = arc.extended(true, f(0.3)).unwrap();
        for i in 0..=13 {
            let t = S::from_ratio(i, 10).unwrap();
            let p = longer.evaluate(t).unwrap();
            assert!(p.norm_sq().could_be_equal(S::ONE), "{t:?}: {p:?}");
        }
    }
    #[test]
    fn arc_continues_round_its_circle() {
        for_all_scalars!(check_arc_continues_round_its_circle);
    }
}
