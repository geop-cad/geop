use geop_core_math::{scalars::Scalar, vector::Vector3};

use super::NurbCurve;

impl<S: Scalar> NurbCurve<S, 4> {
    /// This 3-D curve shifted by `offset` — same shape and parametrization,
    /// every point moved by `offset`. Translates each (homogeneous) control
    /// point by `offset * weight`, leaving weights and the knot vector
    /// untouched.
    pub fn translate(&self, offset: Vector3<S>) -> Self {
        let control_points = self
            .control_points
            .iter()
            .map(|cp| {
                let w = cp[3];
                geop_core_math::vector::Vector4::from_array([
                    cp[0].add(offset[0].mul(w)),
                    cp[1].add(offset[1].mul(w)),
                    cp[2].add(offset[2].mul(w)),
                    w,
                ])
            })
            .collect();
        Self {
            degree: self.degree,
            control_points,
            knot_vector: self.knot_vector.clone(),
            // Every control point moved by exactly `offset`, so the box
            // moves by `offset` too — cheaper than rescanning the points.
            aabb: [
                self.aabb[0].add(offset[0]),
                self.aabb[1].add(offset[1]),
                self.aabb[2].add(offset[2]),
            ],
        }
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use super::super::NurbCurve3D;

    fn check_translate_shifts_evaluated_points<S: Scalar>() {
        let curve = NurbCurve3D::try_new(
            1,
            vec![
                geop_core_math::vector::Vector4::from_array([S::ZERO, S::ZERO, S::ZERO, S::ONE]),
                geop_core_math::vector::Vector4::from_array([S::ONE, S::ZERO, S::ZERO, S::ONE]),
            ],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        let offset = Vector3::from_array([S::from_f64(2.0), S::from_f64(3.0), S::from_f64(-1.0)]);
        let shifted = curve.translate(offset);
        for t in [0.0, 0.3, 1.0] {
            let a = shifted.evaluate(S::from_f64(t)).unwrap();
            let b = curve.evaluate(S::from_f64(t)).unwrap().add(&offset);
            assert!(a.could_be_equal(&b));
        }
    }
    #[test]
    fn translate_shifts_evaluated_points() {
        for_all_scalars!(check_translate_shifts_evaluated_points);
    }
}
