//! Two curves, one ending where the other starts, as one curve.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    vector::Vector,
};

use super::NurbCurve;
use crate::knot_insertion::is_clamped_at;

impl<const D: usize> NurbCurve<D> {
    /// This curve and `next`, which starts where this one ends, as one
    /// curve: the lower in degree raised to the other's, `next`'s knots
    /// carried on from where this one's end, its homogeneous control points
    /// scaled so its first weight is this one's last — the same rational
    /// curve — and the two joined by a knot of full multiplicity, so the
    /// joined curve is exactly the two pieces, however they meet there.
    /// The control point where they meet is the one both pieces had there:
    /// two enclosures of one point, so what they share.
    ///
    /// Fails for a knot vector not clamped where they meet, and for pieces
    /// that do not meet: their end and start control points apart.
    pub fn joined(&self, next: &Self) -> GeopResult<Self> {
        let (mut a, mut b) = (self.clone(), next.clone());
        while a.degree < b.degree {
            a = a.elevate_degree()?;
        }
        while b.degree < a.degree {
            b = b.elevate_degree()?;
        }
        let p = a.degree;
        if !is_clamped_at(&a.knot_vector, p, false) || !is_clamped_at(&b.knot_vector, p, true) {
            return Err(GeopError::new(format!(
                "NurbCurve::joined: the knot vectors {:?} and {:?} are not clamped where the curves meet",
                a.knot_vector, b.knot_vector
            )));
        }
        let last = *a.control_points.last().expect("a curve has control points");
        let first = b.control_points[0];
        let scale = last[D - 1].div(first[D - 1])?;
        let scaled: Vec<Vector<D>> = (b.control_points.iter())
            .map(|cp| cp.prod_scalar(scale))
            .collect();
        if !scaled[0].could_be_equal(&last) {
            return Err(GeopError::new(format!(
                "NurbCurve::joined: the curves do not meet: one ends at {last:?}, the other starts at {:?}",
                scaled[0]
            )));
        }
        let meet = Vector::from_array(std::array::from_fn(|k| last[k].intersect(scaled[0][k])));
        let (_, a1) = a.domain();
        let (b0, _) = b.domain();
        let shift = a1.sub(b0);
        let mut knots: Vec<_> = a.knot_vector[..a.knot_vector.len() - 1].to_vec();
        knots.extend(b.knot_vector[p + 1..].iter().map(|k| k.add(shift)));
        let mut points = a.control_points;
        *points.last_mut().expect("a curve has control points") = meet;
        points.extend_from_slice(&scaled[1..]);
        NurbCurve::try_new(p, points, knots)
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use crate::nurb_curve::NurbCurve3D;

    fn p(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    /// Two lines end to end joined: one curve through both, its middle at
    /// the corner where they met.
    #[test]
    fn joins_two_lines() {
        let a = NurbCurve3D::line(p(0., 0., 0.), p(1., 0., 0.)).unwrap();
        let b = NurbCurve3D::line(p(1., 0., 0.), p(1., 2., 0.)).unwrap();
        let joined = a.joined(&b).unwrap();
        let (t0, t1) = joined.domain();
        assert!(joined.evaluate(t0).unwrap().could_be_equal(&p(0., 0., 0.)));
        assert!(joined.evaluate(t1).unwrap().could_be_equal(&p(1., 2., 0.)));
        let (_, a1) = a.domain();
        assert!(joined.evaluate(a1).unwrap().could_be_equal(&p(1., 0., 0.)));
    }

    /// Curves that do not meet are refused.
    #[test]
    fn refuses_curves_apart() {
        let a = NurbCurve3D::line(p(0., 0., 0.), p(1., 0., 0.)).unwrap();
        let b = NurbCurve3D::line(p(2., 0., 0.), p(3., 0., 0.)).unwrap();
        assert!(a.joined(&b).is_err());
    }
}
