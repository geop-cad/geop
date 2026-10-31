use geop_core_math::{primitives::Motion, vector::Vector3};

use crate::{aabb::compute_aabb, nurb_curve::transform_control_point};

use super::NurbSurface;

impl NurbSurface<4> {
    /// This surface shifted by `offset` — same shape and parametrization,
    /// every point moved by `offset`. Translates each (homogeneous) control
    /// point by `offset * weight`, leaving weights and knot vectors
    /// untouched.
    pub fn translate(&self, offset: Vector3) -> Self {
        let control_points: Vec<_> = self
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
        let aabb = compute_aabb(&control_points);
        Self {
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            num_u: self.num_u,
            num_v: self.num_v,
            control_points,
            knot_vector_u: self.knot_vector_u.clone(),
            knot_vector_v: self.knot_vector_v.clone(),
            // Of the control points as moved, rounding and all: the box
            // moved by `offset` misses what rounding the move adds to a
            // point, and is not the box the same points read back have.
            aabb,
        }
    }

    /// This surface moved by `motion` — a rigid motion, a translation or a
    /// mirror — with the same parametrization, so every pcurve drawn on it
    /// still traces the same patch: each control point moved (see
    /// [`crate::nurb_curve::transform_control_point`]), weights and knots
    /// untouched.
    ///
    /// A mirror turns `Su x Sv` against the mirrored surface's material
    /// side: the normal of the result is the mirrored normal, negated. What
    /// the surface bounds has to be turned around for that (see
    /// `Model::transform_body`).
    pub fn transform(&self, motion: &Motion) -> Self {
        let mut surface = Self {
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            num_u: self.num_u,
            num_v: self.num_v,
            control_points: self
                .control_points
                .iter()
                .map(|cp| transform_control_point(cp, motion))
                .collect(),
            knot_vector_u: self.knot_vector_u.clone(),
            knot_vector_v: self.knot_vector_v.clone(),
            aabb: self.aabb,
        };
        surface.recompute_aabb();
        surface
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{primitives::Motion, scalars::Scalar, vector::Vector3};

    use super::super::NurbSurface3D;

    /// A saddle mirrored in a slanted plane: every point goes to its mirror
    /// image, and the normal to the mirrored normal, negated.
    #[test]
    fn mirror_moves_points_and_turns_normals() {
        let p = |x: f64, y: f64, z: f64| {
            geop_core_math::vector::Vector4::from_array([
                Scalar::from_f64(x),
                Scalar::from_f64(y),
                Scalar::from_f64(z),
                Scalar::ONE,
            ])
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![p(0., 0., 0.), p(0., 2., 1.), p(2., 0., 1.), p(2., 2., 0.)],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .unwrap();
        let v = |x: f64, y: f64, z: f64| Vector3::from_array([x, y, z].map(Scalar::from_f64));
        let mirror = Motion::mirror(&v(1.0, 0.5, -0.25), &v(1.0, 2.0, -0.5)).unwrap();
        let mirrored = surface.transform(&mirror);
        let (u, w) = (Scalar::from_f64(0.3), Scalar::from_f64(0.7));
        let a = mirrored.evaluate(u, w).unwrap();
        let b = mirror.apply(&surface.evaluate(u, w).unwrap());
        assert!(a.could_be_equal(&b), "{a:?} vs {b:?}");
        let n = mirrored.normal(u, w).unwrap();
        let m = mirror.rotate(&surface.normal(u, w).unwrap()).neg();
        assert!(n.could_be_equal(&m), "{n:?} vs {m:?}");
    }

    /// A surface shifted holds the box of its control points as shifted,
    /// rounding and all: the box a surface of the same points has — what
    /// one kept and read back has — and one that encloses every point.
    #[test]
    fn a_surface_shifted_boxes_its_points_as_shifted() {
        let p = |x: f64, y: f64, z: f64, w: f64| {
            geop_core_math::vector::Vector4::from_array(
                [x * w, y * w, z * w, w].map(Scalar::from_f64),
            )
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![
                p(0.1, 0., 0.3, 0.7),
                p(0., 2.2, 1., 1.3),
                p(1.9, 0., 1., 0.9),
                p(2., 2., 0.1, 1.),
            ],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .unwrap();
        let by = Vector3::from_array([1.7, -0.3, 1e3].map(Scalar::from_f64));
        let shifted = surface.translate(by);
        let mut boxed = shifted.clone();
        boxed.recompute_aabb();
        assert_eq!(shifted, boxed);
    }

    #[test]
    fn translate_shifts_evaluated_points() {
        let p = |x: f64, y: f64, z: f64| {
            geop_core_math::vector::Vector4::from_array([
                Scalar::from_f64(x),
                Scalar::from_f64(y),
                Scalar::from_f64(z),
                Scalar::ONE,
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
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .unwrap();
        let offset = Vector3::from_array([
            Scalar::from_f64(2.0),
            Scalar::from_f64(3.0),
            Scalar::from_f64(-1.0),
        ]);
        let shifted = surface.translate(offset);
        let a = shifted
            .evaluate(Scalar::from_f64(0.3), Scalar::from_f64(0.7))
            .unwrap();
        let b = surface
            .evaluate(Scalar::from_f64(0.3), Scalar::from_f64(0.7))
            .unwrap()
            .add(&offset);
        assert!(a.could_be_equal(&b));
    }

    /// A placed surface evaluates to the placed points of the original.
    #[test]
    fn place_moves_evaluated_points() {
        use geop_core_math::{primitives::Pose, scalars::Scalar};
        let p = |x: f64, y: f64, z: f64, w: f64| {
            geop_core_math::vector::Vector4::from_array(
                [x * w, y * w, z * w, w].map(Scalar::from_f64),
            )
        };
        let surface = NurbSurface3D::try_new(
            1,
            1,
            vec![
                p(0.0, 0.0, 0.0, 1.0),
                p(0.0, 1.0, 0.0, 2.0),
                p(1.0, 0.0, 1.0, 1.0),
                p(1.0, 1.0, 0.0, 1.0),
            ],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .unwrap();
        let pose = Pose::from_euler(
            Vector3::from_array([1.0, -2.0, 0.5].map(Scalar::from_f64)),
            [30.0, 10.0, -45.0].map(Scalar::from_f64),
        )
        .unwrap();
        let placed = surface.transform(&pose.motion());
        let (u, v) = (Scalar::from_f64(0.3), Scalar::from_f64(0.7));
        let a = placed.evaluate(u, v).unwrap();
        let b = pose.apply(&surface.evaluate(u, v).unwrap());
        assert!(a.could_be_equal(&b));
    }
}
