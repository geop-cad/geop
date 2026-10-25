use geop_core_math::{
    geop_error::GeopResult,
    primitives::Motion,
    scalars::Scalar,
    vector::{Vector2, Vector3, Vector4},
};

use super::NurbCurve;

impl NurbCurve<4> {
    /// This 3-D curve shifted by `offset` — same shape and parametrization,
    /// every point moved by `offset`. Translates each (homogeneous) control
    /// point by `offset * weight`, leaving weights and the knot vector
    /// untouched.
    pub fn translate(&self, offset: Vector3) -> Self {
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

impl NurbCurve<3> {
    /// This 2-D curve shifted by `offset` (see [`NurbCurve::translate`] for
    /// 3-D). Shifted by an exact zero, it is the curve itself, not widened
    /// by adding zeros.
    pub fn translate(&self, offset: Vector2) -> Self {
        let zero = |x: Scalar| x.is_sharp() && x.could_be_equal(Scalar::ZERO);
        if zero(offset[0]) && zero(offset[1]) {
            return self.clone();
        }
        let control_points = self
            .control_points
            .iter()
            .map(|cp| {
                let w = cp[2];
                Vector3::from_array([cp[0].add(offset[0].mul(w)), cp[1].add(offset[1].mul(w)), w])
            })
            .collect();
        Self {
            degree: self.degree,
            control_points,
            knot_vector: self.knot_vector.clone(),
            aabb: [
                self.aabb[0].add(offset[0]),
                self.aabb[1].add(offset[1]),
                self.aabb[2],
            ],
        }
    }

    /// This 2-D curve as a 3-D one in the plane `z = 0`.
    pub fn lifted(&self) -> GeopResult<NurbCurve<4>> {
        NurbCurve::try_new(
            self.degree,
            self.control_points
                .iter()
                .map(|q| Vector4::from_array([q[0], q[1], Scalar::ZERO, q[2]]))
                .collect(),
            self.knot_vector.clone(),
        )
    }

    /// This 2-D curve as the planar 3-D curve `origin + x e1 + y e2` for
    /// `(x, y)` along it: a sketch's curve where its plane puts it. An
    /// affine map of the homogeneous control points `(w x, w y, w)` to
    /// `(w origin + (w x) e1 + (w y) e2, w)`, so it is exact for any NURBS
    /// and divides by no weight.
    pub fn embed(&self, origin: &Vector3, e1: &Vector3, e2: &Vector3) -> GeopResult<NurbCurve<4>> {
        let control_points = self
            .control_points
            .iter()
            .map(|cp| embed_point(cp, origin, e1, e2))
            .collect();
        NurbCurve::try_new(self.degree, control_points, self.knot_vector.clone())
    }
}

/// Homogeneous `(w (origin + x e1 + y e2), w)` for the homogeneous 2-D point
/// `cp = (w x, w y, w)`: linear in `cp`, so no division — the control point
/// [`NurbCurve::embed`] makes of `cp`.
pub fn embed_point(cp: &Vector3, origin: &Vector3, e1: &Vector3, e2: &Vector3) -> Vector4 {
    let p = origin
        .prod_scalar(cp[2])
        .add(&e1.prod_scalar(cp[0]))
        .add(&e2.prod_scalar(cp[1]));
    Vector4::from_array([p[0], p[1], p[2], cp[2]])
}

/// The homogeneous 3-D point `cp = (w p, w)` in the plane through `origin`
/// along the orthonormal `e1` and `e2`, homogeneous: `(w x, w y, w)` for
/// `x = (p - origin) . e1`, `y = (p - origin) . e2` — the inverse of
/// [`embed_point`] for a point of the plane, and like it without division.
pub fn flatten_point(cp: &Vector4, origin: &Vector3, e1: &Vector3, e2: &Vector3) -> Vector3 {
    let w = cp[3];
    let d = Vector3::from_array([cp[0], cp[1], cp[2]]).sub(&origin.prod_scalar(w));
    Vector3::from_array([d.prod_dot(e1), d.prod_dot(e2), w])
}

/// The homogeneous control point `cp = (w p, w)` moved by `motion`: an
/// isometry is affine, so it goes to `(w (A p + t), w) = (A (w p) + w t, w)`
/// without dividing by the weight. A weight of exactly one — every control
/// point of a polynomial curve or patch — multiplies nothing: rounding
/// `1 t` outward would only widen the point. For a translation `A` is the
/// identity and is not applied either, so the point moves by one addition
/// per coordinate.
pub(crate) fn transform_control_point(cp: &Vector4, motion: &Motion) -> Vector4 {
    let w = cp[3];
    let p = motion.rotate(&Vector3::from_array([cp[0], cp[1], cp[2]]));
    let mut t = motion.position();
    if !(w.is_sharp() && w.to_f64() == 1.0) {
        t = t.map(|x| x.mul(w));
    }
    Vector4::from_array([p[0].add(t[0]), p[1].add(t[1]), p[2].add(t[2]), w])
}

impl NurbCurve<4> {
    /// This 3-D curve moved by `motion` — a rigid motion, a translation or
    /// a mirror — with the same parametrization: each control point moved
    /// (see [`transform_control_point`]), weights and knots untouched.
    pub fn transform(&self, motion: &Motion) -> Self {
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
    use geop_core_math::{
        primitives::Pose,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use super::super::NurbCurve3D;

    #[test]
    fn translate_shifts_evaluated_points() {
        let curve = NurbCurve3D::try_new(
            1,
            vec![
                geop_core_math::vector::Vector4::from_array([
                    Scalar::ZERO,
                    Scalar::ZERO,
                    Scalar::ZERO,
                    Scalar::ONE,
                ]),
                geop_core_math::vector::Vector4::from_array([
                    Scalar::ONE,
                    Scalar::ZERO,
                    Scalar::ZERO,
                    Scalar::ONE,
                ]),
            ],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .unwrap();
        let offset = Vector3::from_array([
            Scalar::from_f64(2.0),
            Scalar::from_f64(3.0),
            Scalar::from_f64(-1.0),
        ]);
        let shifted = curve.translate(offset);
        for t in [0.0, 0.3, 1.0] {
            let a = shifted.evaluate(Scalar::from_f64(t)).unwrap();
            let b = curve.evaluate(Scalar::from_f64(t)).unwrap().add(&offset);
            assert!(a.could_be_equal(&b));
        }
    }

    /// A rational curve (a weight other than one) placed by a pose
    /// evaluates to the pose applied to its points.
    #[test]
    fn transform_moves_evaluated_points() {
        let curve = NurbCurve3D::try_new(
            2,
            vec![
                Vector4::from_array([Scalar::ONE, Scalar::ZERO, Scalar::ZERO, Scalar::ONE]),
                Vector4::from_array([
                    Scalar::from_f64(0.5),
                    Scalar::from_f64(0.5),
                    Scalar::ZERO,
                    Scalar::from_f64(0.5),
                ]),
                Vector4::from_array([Scalar::ZERO, Scalar::ONE, Scalar::ZERO, Scalar::ONE]),
            ],
            vec![
                Scalar::ZERO,
                Scalar::ZERO,
                Scalar::ZERO,
                Scalar::ONE,
                Scalar::ONE,
                Scalar::ONE,
            ],
        )
        .unwrap();
        let pose = Pose::from_euler(
            Vector3::from_array([1.0, -2.0, 0.5].map(Scalar::from_f64)),
            [30.0, -10.0, 75.0].map(Scalar::from_f64),
        )
        .unwrap();
        let placed = curve.transform(&pose.motion());
        for t in [0.0, 0.3, 0.8, 1.0] {
            let a = placed.evaluate(Scalar::from_f64(t)).unwrap();
            let b = pose
                .motion()
                .apply(&curve.evaluate(Scalar::from_f64(t)).unwrap());
            assert!(a.could_be_equal(&b));
        }
    }
}
