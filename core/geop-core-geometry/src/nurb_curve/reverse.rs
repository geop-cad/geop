use super::NurbCurve;

impl<const D: usize> NurbCurve<D> {
    /// This curve traversed in the opposite direction: same domain, same
    /// shape, but `reverse().evaluate(t) == evaluate(t_min + t_max - t)`.
    /// Reverses the control points and complements the knot vector
    /// (`u -> t_min + t_max - u`, then reversed) — the standard B-spline
    /// reversal construction.
    pub fn reverse(&self) -> Self {
        let (t_min, t_max) = self.domain();
        let sum = t_min.add(t_max);
        let control_points = self.control_points.iter().rev().copied().collect();
        let knot_vector = self.knot_vector.iter().rev().map(|&u| sum.sub(u)).collect();
        Self {
            degree: self.degree,
            control_points,
            knot_vector,
            // Same control points, just reordered — the bounding box they
            // enclose is unchanged.
            aabb: self.aabb,
        }
    }
}

impl NurbCurve<3> {
    /// This 2-D curve mirrored across the diagonal `x = y`: every point
    /// `(x, y)` becomes `(y, x)`, with the same parametrization. Mirroring
    /// flips orientation, so a counter-clockwise loop becomes clockwise.
    pub fn swap_xy(&self) -> Self {
        let control_points: Vec<_> = self
            .control_points
            .iter()
            .map(|cp| geop_core_math::vector::Vector3::from_array([cp[1], cp[0], cp[2]]))
            .collect();
        let aabb = crate::aabb::compute_aabb(&control_points);
        Self {
            degree: self.degree,
            control_points,
            knot_vector: self.knot_vector.clone(),
            aabb,
        }
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector2};

    use super::super::NurbCurve2D;

    #[test]
    fn reverse_of_line_evaluates_swapped() {
        let p0 = Vector2::from_array([Scalar::ZERO, Scalar::ZERO]);
        let p1 = Vector2::from_array([Scalar::ONE, Scalar::from_f64(2.0)]);
        let curve = NurbCurve2D::try_new(
            1,
            vec![
                geop_core_math::vector::Vector3::from_array([p0[0], p0[1], Scalar::ONE]),
                geop_core_math::vector::Vector3::from_array([p1[0], p1[1], Scalar::ONE]),
            ],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .unwrap();
        let reversed = curve.reverse();
        let (r0, r1) = reversed.domain();
        let (c0, c1) = curve.domain();
        assert!(r0.could_be_equal(c0) && r1.could_be_equal(c1));
        let a = reversed.evaluate(Scalar::ZERO).unwrap();
        let b = curve.evaluate(Scalar::ONE).unwrap();
        assert!(a.could_be_equal(&b));
        let a = reversed.evaluate(Scalar::ONE).unwrap();
        let b = curve.evaluate(Scalar::ZERO).unwrap();
        assert!(a.could_be_equal(&b));
    }
}
