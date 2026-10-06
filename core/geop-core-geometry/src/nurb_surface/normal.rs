use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::NurbSurface;
use crate::{
    knot_insertion::is_clamped_at,
    nurb_curve::dehomogenize,
    spline::{centered, find_span, homogeneous_derivatives, rational_derivatives},
};

impl<S: Scalar> NurbSurface<S, 4> {
    /// The homogeneous partial derivatives at `(u, v)`: the pure ones up to
    /// order `n`, `([A, A_u, A_uu, …], [A, A_v, A_vv, …])`, and the mixed
    /// `A_uv` where that order takes it in (`n >= 2`). A pure partial only differentiates along its own direction, so
    /// each is the curve case ([`homogeneous_derivatives`]) run across the
    /// local rows (columns), each first evaluated at `v` (`u`); the mixed one
    /// differentiates the rows once along `v`, then once across them along
    /// `u`. The `A` they start from is relative to a control point of the
    /// span, so only the derivatives are meaningful.
    #[allow(clippy::type_complexity)]
    fn homogeneous_partials(
        &self,
        u: S,
        v: S,
        n: usize,
    ) -> GeopResult<(Vec<Vector<S, 4>>, Vec<Vector<S, 4>>, Option<Vector<S, 4>>)> {
        let (p, q) = (self.degree_u, self.degree_v);
        let (ku, kv) = (&self.knot_vector_u, &self.knot_vector_v);
        let nv = self.num_v;
        let span_u = find_span(p, ku, self.num_u - 1, u)?;
        let span_v = find_span(q, kv, nv - 1, v)?;
        // Relative to a control point of the span (see [`centered`]): the
        // derivatives do not depend on where the patch lies, and that way
        // neither does their width.
        let local: Vec<Vector<S, 4>> = (span_u - p..=span_u)
            .flat_map(|i| (span_v - q..=span_v).map(move |j| i * nv + j))
            .map(|k| self.control_points[k])
            .collect();
        let (cp, _) = centered(&local);
        let at = |i: usize, j: usize| cp[i * (q + 1) + j];
        let mixed = n >= 2;
        // Each row at `v`, and along `v` where the mixed partial is asked.
        let (rows, rows_v): (Vec<Vector<S, 4>>, Vec<Vector<S, 4>>) = (0..=p)
            .map(|i| {
                let local: Vec<_> = (0..=q).map(|j| at(i, j)).collect();
                let d = homogeneous_derivatives(q, kv, &local, span_v, v, usize::from(mixed));
                (d[0], d.get(1).copied().unwrap_or_else(Vector::zero))
            })
            .unzip();
        let cols: Vec<Vector<S, 4>> = (0..=q)
            .map(|j| {
                let local: Vec<_> = (0..=p).map(|i| at(i, j)).collect();
                homogeneous_derivatives(p, ku, &local, span_u, u, 0)[0]
            })
            .collect();
        Ok((
            homogeneous_derivatives(p, ku, &rows, span_u, u, n),
            homogeneous_derivatives(q, kv, &cols, span_v, v, n),
            mixed.then(|| homogeneous_derivatives(p, ku, &rows_v, span_u, u, 1)[1]),
        ))
    }

    /// Partial derivatives `(∂S/∂u, ∂S/∂v)` at `(u, v)`.
    pub fn derivatives(&self, u: S, v: S) -> GeopResult<(Vector3<S>, Vector3<S>)> {
        let (du, dv, _) = self.homogeneous_partials(u, v, 1)?;
        Ok((rational_derivatives(&du)?[1], rational_derivatives(&dv)?[1]))
    }

    /// Second partial derivatives `[∂²S/∂u², ∂²S/∂u∂v, ∂²S/∂v²]` at
    /// `(u, v)`. The pure ones by the rational quotient rule along their own
    /// direction ([`rational_derivatives`]); the mixed one by differentiating
    /// `A = w S` once along each: `A_uv = w_uv S + w_u S_v + w_v S_u + w
    /// S_uv`.
    pub fn second_derivatives(&self, u: S, v: S) -> GeopResult<[Vector3<S>; 3]> {
        let (du, dv, a_uv) = self.homogeneous_partials(u, v, 2)?;
        let a_uv = a_uv.expect("the second order takes in the mixed partial");
        let (cu, cv) = (rational_derivatives(&du)?, rational_derivatives(&dv)?);
        let (w, w_u, w_v, w_uv) = (du[0][3], du[1][3], dv[1][3], a_uv[3]);
        let mut s_uv = Vector3::zero();
        for c in 0..3 {
            s_uv[c] = a_uv[c]
                .sub(w_uv.mul(cu[0][c]))
                .sub(w_u.mul(cv[1][c]))
                .sub(w_v.mul(cu[1][c]))
                .div(w)?;
        }
        Ok([cu[2], s_uv, cv[2]])
    }

    /// The boundary row at the start (`first`) or end of the `u` domain
    /// (`u_fixed`) or `v` domain, and the row next to it — `None` unless the
    /// knot vector is clamped there, so that the boundary row *is* the
    /// surface.
    fn boundary_rows(&self, u_fixed: bool, first: bool) -> Option<[Vec<Vector<S, 4>>; 2]> {
        let (nu, nv) = (self.num_u, self.num_v);
        let (knots, degree, len) = if u_fixed {
            (&self.knot_vector_u, self.degree_u, nu)
        } else {
            (&self.knot_vector_v, self.degree_v, nv)
        };
        if len < 2 || !is_clamped_at(knots, degree, first) {
            return None;
        }
        let row = |k: usize| -> Vec<Vector<S, 4>> {
            if u_fixed {
                self.control_points[k * nv..(k + 1) * nv].to_vec()
            } else {
                (0..nu).map(|i| self.control_points[i * nv + k]).collect()
            }
        };
        let (edge, next) = if first { (0, 1) } else { (len - 1, len - 2) };
        Some([row(edge), row(next)])
    }

    /// The normal at a pole: the boundary row at the start (`first`) or end
    /// of the `u` (`u_fixed`) or `v` domain, if it collapses to a single
    /// point `P`. `None` if that row is no pole.
    ///
    /// Leaving the pole, the derivative across the row is a positive
    /// combination of the *spokes* `E_i = X_i − P` to the next row's
    /// Cartesian control points `X_i` (the weights only scale each spoke by
    /// a positive factor). So the pole has a tangent plane exactly when the
    /// spokes are coplanar, and its normal is that plane's, oriented like
    /// `S_u × S_v` next to it by the spokes' turning `T = Σ E_i × E_{i+1}`.
    /// Spokes that are not coplanar — an apex, like a cone's — leave no
    /// single normal: an error, not an arbitrary pick among them.
    fn pole_normal(&self, u_fixed: bool, first: bool) -> Option<GeopResult<Vector3<S>>> {
        let [edge, next] = self.boundary_rows(u_fixed, first)?;
        let edge = dehomogenize::<S, 4, 3>(&edge);
        if !edge.iter().all(|p| p.could_be_equal(&edge[0])) {
            return None;
        }
        let pole = edge[0];
        let spokes: Vec<Vector3<S>> = dehomogenize::<S, 4, 3>(&next)
            .iter()
            .map(|x| x.sub(&pole))
            .collect();
        let turning = spokes
            .windows(2)
            .fold(Vector3::zero(), |acc: Vector3<S>, e| {
                acc.add(&e[0].prod_cross(&e[1]))
            });
        let result = turning.normalize().and_then(|t| {
            if !spokes
                .iter()
                .all(|e| e.prod_dot(&t).could_be_equal(S::ZERO))
            {
                return Err(GeopError::new(format!(
                    "the spokes {spokes:?} are not coplanar: an apex has no single normal"
                )));
            }
            // Next to a collapsed `v` row `S_u × S_v ≈ (v − v₀) S_uv × S_v`,
            // next to a collapsed `u` row `≈ (u − u₀) S_u × S_uv`; `S_uv`
            // turns the way the spokes do, and `v − v₀` (`u − u₀`) is
            // positive at the domain start.
            Ok(if u_fixed == first { t } else { t.neg() })
        });
        Some(result.with_context(&|e: GeopError| {
            e.with_context(format!(
                "NurbSurface::pole_normal(pole={pole:?}, collapsed {} row at the {})",
                if u_fixed { "u" } else { "v" },
                if first { "start" } else { "end" }
            ))
        }))
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
    ///
    /// **At a pole** — `(u, v)` touching a boundary row that collapses to a
    /// point — `S_u × S_v` vanishes, and the normal is that of the pole
    /// itself ([`Self::pole_normal`]), the same for every `u` (`v`) along
    /// the row. Anything else where the cross product vanishes has no
    /// normal and is an error.
    pub fn normal(&self, u: S, v: S) -> GeopResult<Vector3<S>> {
        let (du, dv) = self.derivatives(u, v)?;
        let regular = du.prod_cross(&dv).normalize();
        if regular.is_ok() {
            return regular;
        }
        let (u0, u1) = self.domain_u();
        let (v0, v1) = self.domain_v();
        for (u_fixed, t, (start, end)) in [(false, v, (v0, v1)), (true, u, (u0, u1))] {
            for (end_value, first) in [(start, true), (end, false)] {
                if t.could_be_equal(end_value)
                    && let Some(normal) = self.pole_normal(u_fixed, first)
                {
                    return normal;
                }
            }
        }
        regular
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

    // ── Poles ──────────────────────────────────────────────────────────────

    /// Quarter of a revolved patch: `u` sweeps the unit quarter arc at
    /// height `rim_z`, `v` runs linearly from the collapsed `v = 0` row at
    /// `(0, 0, apex_z)` out to the arc. `apex_z = rim_z` is a flat disc, any
    /// other a cone.
    fn revolved<S: Scalar>(apex_z: f64, rim_z: f64) -> NurbSurface<S, 4> {
        let f = S::from_f64;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let rim = [(1., 0., 1.), (1., 1., w), (0., 1., 1.)];
        let mut cps = Vec::new();
        for (x, y, wi) in rim {
            cps.push(pt(0., 0., apex_z * wi, wi));
            cps.push(pt(x * wi, y * wi, rim_z * wi, wi));
        }
        NurbSurface::try_new(
            2,
            1,
            cps,
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// A disc's centre has the disc's normal — for every `u` at once, and
    /// the same as just off the pole.
    fn check_disc_centre_normal<S: Scalar>() {
        let f = S::from_f64;
        let disc = revolved::<S>(0., 0.);
        let every_u = f(0.).union(f(1.));
        let n = disc.normal(every_u, f(0.)).unwrap();
        let off = disc.normal(f(0.5), f(0.5)).unwrap();
        assert!(n.could_be_equal(&off), "{n:?} vs {off:?}");
        assert!(
            n[2].abs().could_be_equal(S::ONE) && n[0].could_be_equal(S::ZERO),
            "{n:?}"
        );
    }
    #[test]
    fn disc_centre_normal() {
        for_all_scalars!(check_disc_centre_normal);
    }

    /// A cone's apex has no single normal: its spokes are not coplanar.
    fn check_cone_apex_has_no_normal<S: Scalar>() {
        let f = S::from_f64;
        let cone = revolved::<S>(1., 0.);
        assert!(cone.normal(f(0.).union(f(1.)), f(0.)).is_err());
        assert!(cone.normal(f(0.5), f(0.5)).is_ok());
    }
    #[test]
    fn cone_apex_has_no_normal() {
        for_all_scalars!(check_cone_apex_has_no_normal);
    }

    /// Exact rational sphere octant; `v = 1` is the pole (0, 0, 1) — a
    /// collapsed row at the *end* of the domain, where the limit's sign
    /// flips.
    fn check_sphere_pole_normal<S: Scalar>() {
        let f = S::from_f64;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let sphere = NurbSurface::try_new(
            2,
            2,
            vec![
                pt(1., 0., 0., 1.),
                pt(w, 0., w, w),
                pt(0., 0., 1., 1.),
                pt(w, w, 0., w),
                pt(0.5, 0.5, 0.5, 0.5),
                pt(0., 0., w, w),
                pt(0., 1., 0., 1.),
                pt(0., w, w, w),
                pt(0., 0., 1., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let n = sphere.normal(f(0.).union(f(1.)), f(1.)).unwrap();
        assert!(
            n[0].could_be_equal(S::ZERO) && n[1].could_be_equal(S::ZERO),
            "{n:?}"
        );
        // Same orientation as the regular normal just below the pole.
        let below = sphere.normal(f(0.5), f(0.9)).unwrap();
        assert!(
            n[2].mul(below[2]).definitely_greater(S::ZERO),
            "{n:?} vs {below:?}"
        );
    }
    #[test]
    fn sphere_pole_normal() {
        for_all_scalars!(check_sphere_pole_normal);
    }
}
