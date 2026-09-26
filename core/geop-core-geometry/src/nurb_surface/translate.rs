use geop_core_math::{scalars::Scalar, vector::Vector3};

use super::NurbSurface;

impl<S: Scalar> NurbSurface<S, 4> {
    /// This surface shifted by `offset` — same shape and parametrization,
    /// every point moved by `offset`. Translates each (homogeneous) control
    /// point by `offset * weight`, leaving weights and knot vectors
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
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            num_u: self.num_u,
            num_v: self.num_v,
            control_points,
            knot_vector_u: self.knot_vector_u.clone(),
            knot_vector_v: self.knot_vector_v.clone(),
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

    use super::super::NurbSurface3D;

    fn check_translate_shifts_evaluated_points<S: Scalar>() {
        let p = |x: f64, y: f64, z: f64| {
            geop_core_math::vector::Vector4::from_array([
                S::from_f64(x),
                S::from_f64(y),
                S::from_f64(z),
                S::ONE,
            ])
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![
                p(0.0, 0.0, 0.0),
                p(0.0, 1.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
            ],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap();
        let offset = Vector3::from_array([S::from_f64(2.0), S::from_f64(3.0), S::from_f64(-1.0)]);
        let shifted = surface.translate(offset);
        let a = shifted
            .evaluate(S::from_f64(0.3), S::from_f64(0.7))
            .unwrap();
        let b = surface
            .evaluate(S::from_f64(0.3), S::from_f64(0.7))
            .unwrap()
            .add(&offset);
        assert!(a.could_be_equal(&b));
    }
    #[test]
    fn translate_shifts_evaluated_points() {
        for_all_scalars!(check_translate_shifts_evaluated_points);
    }
}
