use geop_core_math::{
    primitives::Motion,
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

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

/// The homogeneous control point `cp = (w p, w)` moved by `motion`: an
/// isometry is affine, so it goes to `(w (A p + t), w) = (A (w p) + w t, w)`
/// without dividing by the weight. A weight of exactly one — every control
/// point of a polynomial curve or patch — multiplies nothing: rounding
/// `1 t` outward would only widen the point. For a translation `A` is the
/// identity and is not applied either, so the point moves by one addition
/// per coordinate.
pub(crate) fn transform_control_point<S: Scalar>(
    cp: &Vector4<S>,
    motion: &Motion<S>,
) -> Vector4<S> {
    let w = cp[3];
    let p = motion.rotate(&Vector3::from_array([cp[0], cp[1], cp[2]]));
    let mut t = motion.position();
    if !(w.is_sharp() && w.to_f64() == 1.0) {
        t = t.map(|x| x.mul(w));
    }
    Vector4::from_array([p[0].add(t[0]), p[1].add(t[1]), p[2].add(t[2]), w])
}

impl<S: Scalar> NurbCurve<S, 4> {
    /// This 3-D curve moved by `motion` — a rigid motion, a translation or
    /// a mirror — with the same parametrization: each control point moved
    /// (see [`transform_control_point`]), weights and knots untouched.
    pub fn transform(&self, motion: &Motion<S>) -> Self {
        let mut curve = Self {
            degree: self.degree,
            control_points: self
                .control_points
                .iter()
                .map(|cp| transform_control_point(cp, motion))
                .collect(),
            knot_vector: self.knot_vector.clone(),
            aabb: self.aabb,
        };
        curve.recompute_aabb();
        curve
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::for_all_scalars;
    use geop_core_math::{
        primitives::Pose,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

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

    /// A rational curve (a weight other than one) placed by a pose
    /// evaluates to the pose applied to its points.
    fn check_transform_moves_evaluated_points<S: Scalar>() {
        let curve = NurbCurve3D::try_new(
            2,
            vec![
                Vector4::from_array([S::ONE, S::ZERO, S::ZERO, S::ONE]),
                Vector4::from_array([
                    S::from_f64(0.5),
                    S::from_f64(0.5),
                    S::ZERO,
                    S::from_f64(0.5),
                ]),
                Vector4::from_array([S::ZERO, S::ONE, S::ZERO, S::ONE]),
            ],
            vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
        )
        .unwrap();
        let pose = Pose::from_euler(
            Vector3::from_array([1.0, -2.0, 0.5].map(S::from_f64)),
            [30.0, -10.0, 75.0].map(S::from_f64),
        )
        .unwrap();
        let placed = curve.transform(&pose.motion());
        for t in [0.0, 0.3, 0.8, 1.0] {
            let a = placed.evaluate(S::from_f64(t)).unwrap();
            let b = pose
                .motion()
                .apply(&curve.evaluate(S::from_f64(t)).unwrap());
            assert!(a.could_be_equal(&b));
        }
    }
    #[test]
    fn transform_moves_evaluated_points() {
        for_all_scalars!(check_transform_moves_evaluated_points);
    }
}
