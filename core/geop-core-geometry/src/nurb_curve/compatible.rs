//! Making curves *compatible* — one degree, one knot vector — so that they
//! can be the rows of one surface, as skinning a surface through several
//! curves needs: raising a curve's degree and refining its knots, neither of
//! which changes the curve, and bringing a curve onto the domain `[0, 1]`.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

use super::NurbCurve;
use crate::{
    aabb::compute_aabb,
    knot_insertion::{insert, is_clamped_at},
};

/// The distinct knots strictly inside the domain of a curve of `degree`
/// with `num_points` control points, each with its multiplicity — knots
/// that could be equal counted as one, their enclosures united.
fn interior_knots<S: Scalar>(knots: &[S], degree: usize, num_points: usize) -> Vec<(S, usize)> {
    let (start, end) = (knots[degree], knots[num_points]);
    let mut out: Vec<(S, usize)> = Vec::new();
    for &k in &knots[degree + 1..num_points] {
        if !(k.definitely_greater(start) && k.definitely_less(end)) {
            continue;
        }
        match out.last_mut() {
            Some((u, m)) if u.could_be_equal(k) => {
                *u = u.union(k);
                *m += 1;
            }
            _ => out.push((k, 1)),
        }
    }
    out
}

impl<S: Scalar, const D: usize> NurbCurve<S, D> {
    /// The same curve reparametrized from its domain onto `[0, 1]`: its
    /// knots mapped linearly, its control points unchanged.
    pub fn with_unit_domain(&self) -> GeopResult<Self> {
        let (t0, t1) = self.domain();
        let span = t1.sub(t0);
        let knots = self
            .knot_vector
            .iter()
            .map(|&k| k.sub(t0).div(span))
            .collect::<GeopResult<Vec<S>>>()?;
        NurbCurve::try_new(self.degree, self.control_points.clone(), knots)
    }

    /// The same curve one degree higher, for a curve clamped at both ends.
    ///
    /// Cut into its Bézier pieces first — every interior knot inserted up to
    /// multiplicity `degree` — each piece is raised on its own:
    /// `Q_i = i/(p+1) P_{i-1} + (1 - i/(p+1)) P_i` on the homogeneous control
    /// points, exact for rational curves too. The pieces stay joined as they
    /// were, every interior knot now of multiplicity `degree + 1`.
    pub fn elevate_degree(&self) -> GeopResult<Self> {
        let p = self.degree;
        let n = self.control_points.len();
        if !(is_clamped_at(&self.knot_vector, p, true)
            && is_clamped_at(&self.knot_vector, p, false))
        {
            return Err(GeopError::new(format!(
                "elevate_degree: the knot vector {:?} is not clamped at both ends",
                self.knot_vector
            )));
        }
        let interior = interior_knots(&self.knot_vector, p, n);
        let mut knots = self.knot_vector.clone();
        let mut rows = [self.control_points.clone()];
        for &(u, m) in &interior {
            if m > p {
                return Err(GeopError::new(format!(
                    "elevate_degree: the knot {u:?} has multiplicity {m}, more than the degree {p}"
                )));
            }
            for _ in m..p {
                insert(&mut knots, &mut rows, p, u);
            }
        }
        let [pts] = rows;
        let pieces = interior.len() + 1;
        debug_assert_eq!(pts.len(), p * pieces + 1);
        let mut out: Vec<Vector<S, D>> = Vec::with_capacity((p + 1) * pieces + 1);
        out.push(pts[0]);
        for piece in pts.windows(p + 1).step_by(p.max(1)) {
            for i in 1..=p {
                let alpha = S::from_ratio(i as i64, (p + 1) as i64)?;
                out.push(Vector::interpolate(&piece[i], &piece[i - 1], alpha));
            }
            out.push(piece[p]);
        }
        let (start, end) = self.domain();
        let mut new_knots = vec![start; p + 2];
        for &(u, _) in &interior {
            new_knots.extend(std::iter::repeat_n(u, p + 1));
        }
        new_knots.extend(std::iter::repeat_n(end, p + 2));
        NurbCurve::try_new(p + 1, out, new_knots)
    }

    /// The same single-span (Bézier) curve with its end weights one, as a
    /// clamped curve's ends are in a profile — what a split of a rational
    /// curve, an arc, leaves otherwise.
    ///
    /// Scaling the `i`-th weight by `c^i` — every homogeneous control point
    /// with it — reparametrizes a rational Bézier curve without changing it,
    /// for any `c > 0`: `c^p = w_0 / w_p` brings both ends to one weight,
    /// and dividing by `w_0` makes that one. `c` is a free choice; the ends
    /// are divided by their own weights, so they come out at one whatever
    /// `c` rounded to. A curve whose ends already weigh exactly one is
    /// returned as it is.
    pub fn with_unit_end_weights(&self) -> GeopResult<Self> {
        let p = self.degree;
        let n = self.control_points.len();
        let weight = |i: usize| self.control_points[i][D - 1];
        let exactly_one = |w: S| w.is_subset_of(S::ONE) && S::ONE.is_subset_of(w);
        if exactly_one(weight(0)) && exactly_one(weight(n - 1)) {
            return Ok(self.clone());
        }
        if n != p + 1 {
            return Err(GeopError::new(format!(
                "with_unit_end_weights: a curve of {n} control points and degree {p} is not a single span, and its end weights {:?}, {:?} are not one",
                weight(0),
                weight(n - 1)
            )));
        }
        let c = (weight(0).to_f64() / weight(n - 1).to_f64()).powf(1.0 / p as f64);
        let control_points = (0..n)
            .map(|i| {
                let factor = if i == 0 {
                    S::ONE.div(weight(0))?
                } else if i == n - 1 {
                    S::ONE.div(weight(n - 1))?
                } else {
                    S::from_f64(c.powi(i as i32)).div(weight(0))?
                };
                Ok(self.control_points[i].prod_scalar(factor))
            })
            .collect::<GeopResult<Vec<_>>>()?;
        NurbCurve::try_new(p, control_points, self.knot_vector.clone())
    }

    /// `curves`, on one domain and clamped, made compatible: each raised to
    /// the highest degree among them, and given every knot any of them has,
    /// at the highest multiplicity any of them has it — so that they all end
    /// up on one knot vector. Every curve stays the curve it was.
    ///
    /// Knots of different curves that could be equal are taken as one, the
    /// union of their enclosures, which every curve then carries: an honest
    /// enclosure of each curve's own knot.
    pub fn compatible(curves: &[Self]) -> GeopResult<Vec<Self>> {
        let Some(first) = curves.first() else {
            return Ok(Vec::new());
        };
        let (start, end) = first.domain();
        let degree = curves.iter().map(|c| c.degree).max().unwrap_or(0);
        let mut out = Vec::with_capacity(curves.len());
        for curve in curves {
            let (s, e) = curve.domain();
            if !(s.could_be_equal(start) && e.could_be_equal(end)) {
                return Err(GeopError::new(format!(
                    "compatible: domains ({s:?}, {e:?}) and ({start:?}, {end:?}) differ"
                )));
            }
            let mut curve = curve.clone();
            while curve.degree < degree {
                curve = curve.elevate_degree()?;
            }
            out.push(curve);
        }

        let mut merged: Vec<(S, usize)> = Vec::new();
        for curve in &out {
            for (u, m) in interior_knots(&curve.knot_vector, degree, curve.control_points.len()) {
                match merged.iter_mut().find(|(v, _)| v.could_be_equal(u)) {
                    Some((v, k)) => {
                        *v = v.union(u);
                        *k = (*k).max(m);
                    }
                    None => merged.push((u, m)),
                }
            }
        }
        for curve in &mut out {
            let mut knots = curve.knot_vector.clone();
            let mut rows = [curve.control_points.clone()];
            for &(u, m) in &merged {
                let have = interior_knots(&knots, degree, rows[0].len())
                    .iter()
                    .find(|(v, _)| v.could_be_equal(u))
                    .map_or(0, |&(_, k)| k);
                for _ in have..m {
                    insert(&mut knots, &mut rows, degree, u);
                }
            }
            let [pts] = rows;
            curve.control_points = pts;
            curve.knot_vector = knots;
        }

        // One knot vector for all: each knot the union of every curve's.
        let len = out[0].knot_vector.len();
        if out.iter().any(|c| c.knot_vector.len() != len) {
            return Err(GeopError::new(format!(
                "compatible: knot vectors of different lengths after merging: {:?}",
                out.iter()
                    .map(|c| c.knot_vector.clone())
                    .collect::<Vec<_>>()
            )));
        }
        let common: Vec<S> = (0..len)
            .map(|k| {
                out.iter()
                    .map(|c| c.knot_vector[k])
                    .reduce(|a, b| a.union(b))
                    .expect("at least one curve")
            })
            .collect();
        for curve in &mut out {
            curve.knot_vector = common.clone();
            curve.aabb = compute_aabb(&curve.control_points);
        }
        Ok(out)
    }

    /// The chain `curves`, each starting where the one before ends, as one
    /// curve on `[0, 1]`, the `i`-th of `n` curves on `[i / n, (i + 1) / n]`:
    /// each brought onto its own unit domain with its ends weighted one (see
    /// [`Self::with_unit_end_weights`]) and raised to the highest degree
    /// among them, their knot vectors laid end to end, every joint a knot
    /// of full multiplicity. A joint's control point is the end of the curve
    /// before it — an error if the next curve could not start there. Every
    /// curve stays the curve it was, on its new piece of the domain.
    pub fn join(curves: &[Self]) -> GeopResult<Self> {
        let n = curves.len();
        if n == 0 {
            return Err(GeopError::new("join: no curves to join"));
        }
        let mut pieces = curves
            .iter()
            .map(|c| c.with_unit_domain()?.with_unit_end_weights())
            .collect::<GeopResult<Vec<_>>>()?;
        let degree = pieces.iter().map(|c| c.degree).max().unwrap_or(0);
        for piece in &mut pieces {
            while piece.degree < degree {
                *piece = piece.elevate_degree()?;
            }
        }
        if n == 1 {
            return Ok(pieces.remove(0));
        }
        let count = S::from_f64(n as f64);
        let mut points: Vec<Vector<S, D>> = Vec::new();
        let mut knots = vec![S::ZERO; degree + 1];
        for (i, piece) in pieces.iter().enumerate() {
            let len = piece.control_points.len();
            if let Some(end) = points.last() {
                let start = &piece.control_points[0];
                if !(0..D).all(|k| end[k].could_be_equal(start[k])) {
                    return Err(GeopError::new(format!(
                        "join: curve {i} starts at {start:?}, not where the one before ends, {end:?}"
                    )));
                }
                points.extend_from_slice(&piece.control_points[1..]);
            } else {
                points.extend_from_slice(&piece.control_points);
            }
            for &k in &piece.knot_vector[degree + 1..len] {
                knots.push(k.add(S::from_f64(i as f64)).div(count)?);
            }
            if i + 1 < n {
                let joint = S::from_ratio((i + 1) as i64, n as i64)?;
                knots.extend(std::iter::repeat_n(joint, degree));
            }
        }
        knots.extend(std::iter::repeat_n(S::ONE, degree + 1));
        NurbCurve::try_new(degree, points, knots)
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_curve::{NurbCurve, NurbCurve3D};
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x * w),
            S::from_f64(y * w),
            S::from_f64(z * w),
            S::from_f64(w),
        ])
    }

    fn quarter_arc<S: Scalar>() -> NurbCurve3D<S> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., std::f64::consts::SQRT_2 / 2.0),
                pt(0., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn cubic_spline<S: Scalar>() -> NurbCurve3D<S> {
        let f = S::from_f64;
        NurbCurve::try_new(
            3,
            vec![
                pt(0., 0., 0., 1.),
                pt(1., 2., 0., 1.),
                pt(2., -1., 1., 1.),
                pt(3., 1., 0., 1.),
                pt(4., 0., 2., 1.),
            ],
            vec![
                f(0.),
                f(0.),
                f(0.),
                f(0.),
                f(0.5),
                f(1.),
                f(1.),
                f(1.),
                f(1.),
            ],
        )
        .unwrap()
    }

    fn line<S: Scalar>() -> NurbCurve3D<S> {
        let f = S::from_f64;
        NurbCurve::try_new(
            1,
            vec![pt(0., 0., 0., 1.), pt(2., 1., 3., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn assert_same_curve<S: Scalar>(a: &NurbCurve3D<S>, b: &NurbCurve3D<S>) {
        for i in 0..=16 {
            let t = S::from_ratio(i, 16).unwrap();
            let (pa, pb) = (a.evaluate(t).unwrap(), b.evaluate(t).unwrap());
            assert!(pa.could_be_equal(&pb), "at {t:?}: {pa:?} vs {pb:?}");
        }
    }

    /// Raising the degree keeps the curve: a rational arc, and a cubic
    /// spline with an interior knot.
    fn check_elevate_degree_keeps_the_curve<S: Scalar>() {
        for curve in [quarter_arc::<S>(), cubic_spline(), line()] {
            let raised = curve.elevate_degree().unwrap();
            assert_eq!(raised.degree, curve.degree + 1);
            assert_same_curve(&curve, &raised);
            let twice = raised.elevate_degree().unwrap();
            assert_same_curve(&curve, &twice);
        }
    }
    #[test]
    fn elevate_degree_keeps_the_curve() {
        for_all_scalars!(check_elevate_degree_keeps_the_curve);
    }

    /// A line, an arc and a spline brought onto one degree and one knot
    /// vector, each still the curve it was.
    fn check_compatible_curves_share_degree_and_knots<S: Scalar>() {
        let curves = [line::<S>(), quarter_arc(), cubic_spline()];
        let out = NurbCurve::compatible(&curves).unwrap();
        for (before, after) in curves.iter().zip(&out) {
            assert_eq!(after.degree, 3);
            assert_eq!(after.control_points.len(), out[0].control_points.len());
            for (a, b) in after.knot_vector.iter().zip(&out[0].knot_vector) {
                assert!(a.is_subset_of(*b) && b.is_subset_of(*a));
            }
            assert_same_curve(before, after);
        }
    }
    #[test]
    fn compatible_curves_share_degree_and_knots() {
        for_all_scalars!(check_compatible_curves_share_degree_and_knots);
    }

    /// The second half of a split curve, brought onto `[0, 1]`.
    fn check_with_unit_domain_rescales_the_knots<S: Scalar>() {
        let curve = cubic_spline::<S>();
        let (_, right) = curve.split(S::from_f64(0.25)).unwrap();
        let unit = right.with_unit_domain().unwrap();
        let (t0, t1) = unit.domain();
        assert!(t0.could_be_equal(S::ZERO) && t1.could_be_equal(S::ONE));
        let mid = unit.evaluate(S::from_f64(0.5)).unwrap();
        assert!(mid.could_be_equal(&curve.evaluate(S::from_f64(0.625)).unwrap()));
    }
    #[test]
    fn with_unit_domain_rescales_the_knots() {
        for_all_scalars!(check_with_unit_domain_rescales_the_knots);
    }

    /// Half of a quarter arc, its end weights brought back to one: still
    /// the same arc.
    fn check_unit_end_weights_keep_the_curve<S: Scalar>() {
        let (left, _) = quarter_arc::<S>().split(S::from_f64(0.5)).unwrap();
        let left = left.with_unit_domain().unwrap();
        let unit = left.with_unit_end_weights().unwrap();
        for cp in [unit.control_points[0], unit.control_points[2]] {
            assert!(cp[3].could_be_equal(S::ONE));
        }
        // A different parametrization of the same points: every point of it
        // lies on the unit circle, and the ends are where they were.
        for i in 0..=8 {
            let p = unit.evaluate(S::from_ratio(i, 8).unwrap()).unwrap();
            assert!(p.norm_sq().could_be_equal(S::ONE), "{p:?}");
        }
        assert!(
            unit.evaluate(S::ONE)
                .unwrap()
                .could_be_equal(&left.evaluate(S::ONE).unwrap())
        );
    }
    #[test]
    fn unit_end_weights_keep_the_curve() {
        for_all_scalars!(check_unit_end_weights_keep_the_curve);
    }

    /// An arc and a line on from its end, joined: one quadratic on
    /// `[0, 1]`, the arc on its first half, the line on its second. A
    /// curve starting elsewhere does not join.
    fn check_join_lays_curves_end_to_end<S: Scalar>() {
        let f = S::from_f64;
        let on = NurbCurve::try_new(
            1,
            vec![pt(0., 1., 0., 1.), pt(-1., 1., 0., 1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap();
        let arc = quarter_arc::<S>();
        let joined = NurbCurve::join(&[arc.clone(), on.clone()]).unwrap();
        assert_eq!(joined.degree, 2);
        for i in 0..=8 {
            let t = S::from_ratio(i, 8).unwrap();
            let half = |k: f64| t.add(f(k)).div(S::TWO).unwrap();
            let (a, b) = (
                joined.evaluate(half(0.0)).unwrap(),
                arc.evaluate(t).unwrap(),
            );
            assert!(a.could_be_equal(&b), "{a:?} vs {b:?}");
            let (a, b) = (joined.evaluate(half(1.0)).unwrap(), on.evaluate(t).unwrap());
            assert!(a.could_be_equal(&b), "{a:?} vs {b:?}");
        }
        assert!(NurbCurve::join(&[arc, line()]).is_err());
    }
    #[test]
    fn join_lays_curves_end_to_end() {
        for_all_scalars!(check_join_lays_curves_end_to_end);
    }
}
