use crate::nurb_curve::NurbCurve;
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

use super::{
    NurbSurface,
    evaluate::{de_boor, find_span},
};

impl<S: Scalar> NurbSurface<S, 4> {
    /// Isoparametric curve `u ↦ S(u, v)` at fixed `v`, as a `NurbCurve<S, 4>`.
    fn isocurve_u(&self, v: S) -> GeopResult<NurbCurve<S, 4>> {
        let span_v = find_span(self.degree_v, &self.knot_vector_v, self.num_v - 1, v)?;
        let mut pts = Vec::with_capacity(self.num_u);
        for i in 0..self.num_u {
            let col: Vec<_> = (0..self.num_v)
                .map(|j| self.control_points[i * self.num_v + j])
                .collect();
            pts.push(de_boor(self.degree_v, &self.knot_vector_v, &col, v, span_v));
        }
        NurbCurve::try_new(self.degree_u, pts, self.knot_vector_u.clone())
    }

    /// Isoparametric curve `v ↦ S(u, v)` at fixed `u`, as a `NurbCurve<S, 4>`.
    fn isocurve_v(&self, u: S) -> GeopResult<NurbCurve<S, 4>> {
        let span_u = find_span(self.degree_u, &self.knot_vector_u, self.num_u - 1, u)?;
        let mut pts = Vec::with_capacity(self.num_v);
        for j in 0..self.num_v {
            let row: Vec<_> = (0..self.num_u)
                .map(|i| self.control_points[i * self.num_v + j])
                .collect();
            pts.push(de_boor(self.degree_u, &self.knot_vector_u, &row, u, span_u));
        }
        NurbCurve::try_new(self.degree_v, pts, self.knot_vector_v.clone())
    }

    /// Partial derivatives `(∂S/∂u, ∂S/∂v)` at `(u, v)`.
    pub fn derivatives(&self, u: S, v: S) -> GeopResult<(Vector3<S>, Vector3<S>)> {
        let du = self.isocurve_u(v)?.tangent(u)?;
        let dv = self.isocurve_v(u)?.tangent(v)?;
        Ok((du, dv))
    }

    /// Pure second partial derivatives `(∂²S/∂u², ∂²S/∂v²)` at `(u, v)`.
    ///
    /// Each is obtained by fixing the *other* parameter, collapsing the
    /// surface to a 1-D isocurve, and differentiating that curve twice —
    /// exactly what "pure" (non-mixed) partials mean. The mixed partial
    /// `∂²S/∂u∂v` is deliberately not computed here (see
    /// [`curvature_radius`](super::curvature::curvature_radius) for why it's
    /// not needed for this crate's surfaces).
    pub(crate) fn second_derivatives(&self, u: S, v: S) -> GeopResult<(Vector3<S>, Vector3<S>)> {
        let duu = self.isocurve_u(v)?.second_derivative(u)?;
        let dvv = self.isocurve_v(u)?.second_derivative(v)?;
        Ok((duu, dvv))
    }

    /// Unit surface normal at `(u, v)`, `normalize(∂S/∂u × ∂S/∂v)`.
    ///
    /// Whether this points outward or inward for a given face depends on
    /// that surface's own `u`/`v` parametrization convention — it is each
    /// surface constructor's responsibility to pick the matching
    /// `Face::sense` (`Forward` if this normal is already outward,
    /// `Reversed` if it needs negating) so that callers can treat
    /// `Face::sense`-corrected `normal()` as reliably outward. See
    /// `box_solid`'s `FACE_DEFS` (`(P10 − P00) × (P01 − P00)`, `Forward`)
    /// and `revolve`/`sphere`'s patch constructors (natural normal is
    /// inward, hence `Reversed`) for both cases.
    pub fn normal(&self, u: S, v: S) -> GeopResult<Vector3<S>> {
        let (du, dv) = self.derivatives(u, v)?;
        du.prod_cross(&dv).normalize()
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_surface::NurbSurface;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(z),
            S::from_f64(w),
        ])
    }

    /// Flat unit patch in the xy-plane: normal should be constant +z everywhere.
    fn flat_xy<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn check_flat_surface_normal_is_constant_z<S: Scalar>() {
        let s = flat_xy::<S>();
        for &(u, v) in &[(0.0, 0.0), (0.5, 0.5), (1.0, 0.0), (0.2, 0.8)] {
            let n = s.normal(S::from_f64(u), S::from_f64(v)).unwrap();
            assert!(n[0].could_be_equal(S::ZERO));
            assert!(n[1].could_be_equal(S::ZERO));
            assert!(n[2].could_be_equal(S::ONE));
        }
    }
    #[test]
    fn flat_surface_normal_is_constant_z() {
        for_all_scalars!(check_flat_surface_normal_is_constant_z);
    }

    /// Non-planar bilinear "saddle" patch: corner heights 0,1,1,0 over
    /// x,y ∈ [0,2]. At the center (u=v=0.5) the surface is tangent to the
    /// z=0.5 plane, so the normal there should be purely +z.
    fn bent_surface<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 2., 1., 1.),
                pt(2., 0., 1., 1.),
                pt(2., 2., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    fn check_bent_surface_center_normal_is_z<S: Scalar>() {
        let s = bent_surface::<S>();
        let n = s.normal(S::from_f64(0.5), S::from_f64(0.5)).unwrap();
        assert!(n[0].could_be_equal(S::ZERO));
        assert!(n[1].could_be_equal(S::ZERO));
        assert!(n[2].definitely_greater(S::ZERO));
    }
    #[test]
    fn bent_surface_center_normal_is_z() {
        for_all_scalars!(check_bent_surface_center_normal_is_z);
    }
}
