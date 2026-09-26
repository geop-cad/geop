use geop_core_math::{scalars::Scalar, vector::Vector};

use super::NurbSurface;

impl<S: Scalar, const D: usize> NurbSurface<S, D> {
    /// Mirror the `u` parametrization: the point at `u` moves to
    /// `u_lo + u_hi - u`, leaving `v` alone.
    ///
    /// The surface traces exactly the same set of points, but `Su` reverses,
    /// so `Su x Sv` — the normal — flips. That is the only way to turn a
    /// face's material side around in this kernel, since orientation lives in
    /// the parametrization rather than in a flag on the face.
    ///
    /// Mirroring rather than merely reordering matters: the domain is
    /// unchanged, so every pcurve drawn on this surface stays in range and
    /// only needs the same mirror applied to its own `u` coordinate (see
    /// `Model::reverse_face`).
    pub fn reverse_u(&self) -> Self {
        let (lo, hi) = self.domain_u();
        let span = lo.add(hi);

        // Knots run the other way and are mirrored about the domain, which
        // keeps them non-decreasing and keeps the domain itself fixed.
        let knot_vector_u: Vec<S> = self
            .knot_vector_u
            .iter()
            .rev()
            .map(|&k| span.sub(k))
            .collect();

        // Control points are stored u-major, so reversing the u index is a
        // reversal of whole rows of length `num_v`.
        let mut control_points: Vec<Vector<S, D>> = Vec::with_capacity(self.control_points.len());
        for i in (0..self.num_u).rev() {
            for j in 0..self.num_v {
                control_points.push(self.control_points[i * self.num_v + j]);
            }
        }

        NurbSurface {
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            num_u: self.num_u,
            num_v: self.num_v,
            control_points,
            knot_vector_u,
            knot_vector_v: self.knot_vector_v.clone(),
            // Same control points, just reordered — the bounding box they
            // enclose is unchanged.
            aabb: self.aabb,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_surface::NurbSurface3D;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    /// A non-planar bilinear patch, so `u` and `v` are genuinely independent.
    fn saddle<S: Scalar>() -> NurbSurface3D<S> {
        let p = |x: f64, y: f64, z: f64| {
            Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
        };
        NurbSurface3D::try_new(
            1,
            1,
            vec![p(0., 0., 0.), p(0., 2., 1.), p(2., 0., 1.), p(2., 2., 0.)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    /// The reversed surface passes through the same points, reached at the
    /// mirrored `u`.
    fn check_reverse_u_traces_the_same_surface<S: Scalar>() {
        let s = saddle::<S>();
        let r = s.reverse_u();
        let (lo, hi) = s.domain_u();
        assert!(r.domain_u().0.could_be_equal(lo) && r.domain_u().1.could_be_equal(hi));

        for &(u, v) in &[(0.25, 0.4), (0.5, 0.5), (0.8, 0.1)] {
            let (u, v) = (S::from_f64(u), S::from_f64(v));
            let original = s.evaluate(u, v).unwrap();
            let mirrored = r.evaluate(lo.add(hi).sub(u), v).unwrap();
            for c in 0..3 {
                assert!(
                    original[c].could_be_equal(mirrored[c]),
                    "coord {c}: {original:?} vs {mirrored:?}"
                );
            }
        }
    }
    #[test]
    fn reverse_u_traces_the_same_surface() {
        for_all_scalars!(check_reverse_u_traces_the_same_surface);
    }

    /// …and its normal points the other way, which is the whole point.
    fn check_reverse_u_flips_the_normal<S: Scalar>() {
        let s = saddle::<S>();
        let r = s.reverse_u();
        let (lo, hi) = s.domain_u();
        let (u, v) = (S::from_f64(0.3), S::from_f64(0.7));
        let n0 = s.normal(u, v).unwrap();
        let n1 = r.normal(lo.add(hi).sub(u), v).unwrap();
        for c in 0..3 {
            assert!(
                n0[c].could_be_equal(n1[c].neg()),
                "coord {c}: {n0:?} vs {n1:?}"
            );
        }
    }
    #[test]
    fn reverse_u_flips_the_normal() {
        for_all_scalars!(check_reverse_u_flips_the_normal);
    }
}
