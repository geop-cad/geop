use crate::nurb_surface::NurbSurface3D;
use geop_core_math::{scalars::Scalar, vector::Vector3};

use super::NurbCurve;

impl NurbCurve<4> {
    /// The ruled surface swept out by translating this curve along
    /// `offset`: `Scalar(t, s) = self.evaluate(t) + s * offset`. Degree
    /// `(self.degree, 1)` — same knot vector as `self` in `t`, `s ∈ [0,
    /// 1]` — control points `[cp_i, cp_i + offset]` row-major per original
    /// control point `cp_i`.
    pub fn sweep(&self, offset: Vector3) -> NurbSurface3D {
        let shifted = self.translate(offset);
        let mut control_points = Vec::with_capacity(self.control_points.len() * 2);
        for i in 0..self.control_points.len() {
            control_points.push(self.control_points[i]);
            control_points.push(shifted.control_points[i]);
        }
        NurbSurface3D::try_new(
            self.degree,
            1,
            control_points,
            self.knot_vector.clone(),
            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE, Scalar::ONE],
        )
        .expect("sweeping a valid curve always yields a valid ruled surface")
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use super::super::NurbCurve3D;

    #[test]
    fn sweep_matches_curve_plus_offset() {
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
        let offset = Vector3::from_array([Scalar::ZERO, Scalar::ZERO, Scalar::from_f64(2.0)]);
        let surface = curve.sweep(offset);

        for t in [0.0, 0.4, 1.0] {
            let a = surface.evaluate(Scalar::from_f64(t), Scalar::ZERO).unwrap();
            let b = curve.evaluate(Scalar::from_f64(t)).unwrap();
            assert!(a.could_be_equal(&b));

            let a_top = surface.evaluate(Scalar::from_f64(t), Scalar::ONE).unwrap();
            let b_top = curve.evaluate(Scalar::from_f64(t)).unwrap().add(&offset);
            assert!(a_top.could_be_equal(&b_top));
        }
    }
}
