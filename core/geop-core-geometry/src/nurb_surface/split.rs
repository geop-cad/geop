use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector};

use super::NurbSurface;
use crate::{aabb::compute_aabb, knot_insertion};

/// A parameter direction of a tensor-product surface.
#[derive(Clone, Copy)]
enum Dir {
    U,
    V,
}

impl<S: Scalar, const D: usize> NurbSurface<S, D> {
    fn degree_in(&self, dir: Dir) -> usize {
        match dir {
            Dir::U => self.degree_u,
            Dir::V => self.degree_v,
        }
    }

    fn knots_in(&self, dir: Dir) -> &Vec<S> {
        match dir {
            Dir::U => &self.knot_vector_u,
            Dir::V => &self.knot_vector_v,
        }
    }

    /// The control net as rows running along `dir` (see `knot_insertion`):
    /// the `num_v` columns `P[·][j]` for `U`, the `num_u` rows `P[i][·]` for
    /// `V`.
    fn rows_along(&self, dir: Dir) -> Vec<Vec<Vector<S, D>>> {
        let (nu, nv) = (self.num_u, self.num_v);
        match dir {
            Dir::U => (0..nv)
                .map(|j| (0..nu).map(|i| self.control_points[i * nv + j]).collect())
                .collect(),
            Dir::V => self.control_points.chunks(nv).map(<[_]>::to_vec).collect(),
        }
    }

    /// This surface with the `dir` knots and rows replaced — the inverse of
    /// [`Self::rows_along`].
    fn with_rows(&self, dir: Dir, knots: Vec<S>, rows: Vec<Vec<Vector<S, D>>>) -> Self {
        let along = rows[0].len();
        let (num_u, num_v, control_points): (usize, usize, Vec<Vector<S, D>>) = match dir {
            Dir::U => (
                along,
                rows.len(),
                (0..along)
                    .flat_map(|i| rows.iter().map(move |col| col[i]))
                    .collect(),
            ),
            Dir::V => (rows.len(), along, rows.into_iter().flatten().collect()),
        };
        let (knot_vector_u, knot_vector_v) = match dir {
            Dir::U => (knots, self.knot_vector_v.clone()),
            Dir::V => (self.knot_vector_u.clone(), knots),
        };
        NurbSurface {
            degree_u: self.degree_u,
            degree_v: self.degree_v,
            num_u,
            num_v,
            aabb: compute_aabb(&control_points),
            control_points,
            knot_vector_u,
            knot_vector_v,
        }
    }

    fn split_in(&self, dir: Dir, t: S) -> GeopResult<(Self, Self)> {
        let mut knots = self.knots_in(dir).clone();
        let mut rows = self.rows_along(dir);
        let (right_knots, right_rows) =
            knot_insertion::split(&mut knots, &mut rows, self.degree_in(dir), t)?;
        Ok((
            self.with_rows(dir, knots, rows),
            self.with_rows(dir, right_knots, right_rows),
        ))
    }

    fn restrict_in(&self, dir: Dir, t0: S, t1: S) -> GeopResult<Self> {
        let mut knots = self.knots_in(dir).clone();
        let mut rows = self.rows_along(dir);
        knot_insertion::restrict(&mut knots, &mut rows, self.degree_in(dir), t0, t1)?;
        Ok(self.with_rows(dir, knots, rows))
    }

    /// Split at parameter `t` in the **u** direction.
    ///
    /// `t` must lie strictly inside the u domain.  Returns `(left, right)`.
    ///
    /// `t` is used exactly as given — **not** sharpened here, see `NurbCurve::split`'s own
    /// doc comment for why (this is the same Boehm-insertion construction,
    /// one dimension up: an unsharpened `t` carried into the new knot
    /// vector lets `alpha = (t - e) / (s - e)` blow up over repeated splits
    /// as `s - e` shrinks while `t`'s own width doesn't). Every current caller
    /// therefore sharpens its own midpoint before calling; a caller splitting
    /// at a *located* parameter must validate the sharpened value it is about
    /// to use, not the wide one it started from.
    pub fn split_u(&self, t: S) -> GeopResult<(NurbSurface<S, D>, NurbSurface<S, D>)> {
        self.split_in(Dir::U, t)
    }

    /// Split at parameter `t` in the **v** direction.
    ///
    /// `t` must lie strictly inside the v domain.  Returns `(left, right)`.
    /// `t` is used exactly as given — **not** sharpened here, same as [`Self::split_u`].
    pub fn split_v(&self, t: S) -> GeopResult<(NurbSurface<S, D>, NurbSurface<S, D>)> {
        self.split_in(Dir::V, t)
    }

    fn split_in_mid(&self, dir: Dir) -> GeopResult<(Self, Self)> {
        let knots = self.knots_in(dir);
        let n = match dir {
            Dir::U => self.num_u,
            Dir::V => self.num_v,
        };
        let (t0, t1) = (knots[self.degree_in(dir)], knots[n]);
        // A self-chosen subdivision point: any value in the interval cuts
        // it equally well, so sharpening loses no accuracy and keeps
        // repeated splits from compounding width (see AGENTS.md).
        let mid = t0.add(t1).div(S::TWO)?.sharpen();
        self.split_in(dir, mid)
    }

    /// Split at the midpoint of the **u** domain.
    pub fn split_u_mid(&self) -> GeopResult<(NurbSurface<S, D>, NurbSurface<S, D>)> {
        self.split_in_mid(Dir::U)
    }

    /// Split at the midpoint of the **v** domain.
    pub fn split_v_mid(&self) -> GeopResult<(NurbSurface<S, D>, NurbSurface<S, D>)> {
        self.split_in_mid(Dir::V)
    }

    /// This surface restricted to `u ∈ [u0, u1]`, `v ∈ [v0, v1]`, with the
    /// same cut rule as `NurbCurve::sub_curve`: only bounds strictly inside
    /// the domain are cut at, so the result covers at least the requested
    /// box ∩ domain. Bounds are used exactly as given and must be narrow.
    pub fn sub_surface(&self, (u0, u1): (S, S), (v0, v1): (S, S)) -> GeopResult<Self> {
        self.restrict_in(Dir::U, u0, u1)?
            .restrict_in(Dir::V, v0, v1)
    }
}

#[cfg(test)]
mod tests {
    use super::super::NurbSurface;
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

    fn bilinear<S: Scalar>() -> NurbSurface<S, 4> {
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

    fn check_split_u_junction_matches<S: Scalar>() {
        let orig = bilinear::<S>();
        let (left, right) = orig.split_u(S::from_f64(0.5)).unwrap();
        for v_val in [0.0_f64, 0.5, 1.0] {
            let v = S::from_f64(v_val);
            let o = orig.evaluate(S::from_f64(0.5), v).unwrap();
            let l = left.evaluate(S::from_f64(0.5), v).unwrap();
            let r = right.evaluate(S::from_f64(0.5), v).unwrap();
            for c in 0..3 {
                assert!(
                    o[c].could_be_equal(l[c]),
                    "split_u left mismatch v={v_val} c={c}"
                );
                assert!(
                    o[c].could_be_equal(r[c]),
                    "split_u right mismatch v={v_val} c={c}"
                );
            }
        }
    }
    #[test]
    fn split_u_junction_matches() {
        for_all_scalars!(check_split_u_junction_matches);
    }

    fn check_split_u_left_start_matches<S: Scalar>() {
        let orig = bilinear::<S>();
        let (left, _) = orig.split_u(S::from_f64(0.5)).unwrap();
        for v_val in [0.0_f64, 0.5, 1.0] {
            let v = S::from_f64(v_val);
            let o = orig.evaluate(S::ZERO, v).unwrap();
            let l = left.evaluate(S::ZERO, v).unwrap();
            for c in 0..3 {
                assert!(o[c].could_be_equal(l[c]), "c={c} v={v_val}");
            }
        }
    }
    #[test]
    fn split_u_left_start_matches() {
        for_all_scalars!(check_split_u_left_start_matches);
    }

    fn check_split_u_right_end_matches<S: Scalar>() {
        let orig = bilinear::<S>();
        let (_, right) = orig.split_u(S::from_f64(0.5)).unwrap();
        for v_val in [0.0_f64, 0.5, 1.0] {
            let v = S::from_f64(v_val);
            let o = orig.evaluate(S::ONE, v).unwrap();
            let r = right.evaluate(S::ONE, v).unwrap();
            for c in 0..3 {
                assert!(o[c].could_be_equal(r[c]), "c={c} v={v_val}");
            }
        }
    }
    #[test]
    fn split_u_right_end_matches() {
        for_all_scalars!(check_split_u_right_end_matches);
    }

    fn check_split_u_at_boundary_returns_err<S: Scalar>() {
        let s = bilinear::<S>();
        assert!(s.split_u(S::ZERO).is_err());
        assert!(s.split_u(S::ONE).is_err());
    }
    #[test]
    fn split_u_at_boundary_returns_err() {
        for_all_scalars!(check_split_u_at_boundary_returns_err);
    }

    fn check_split_v_junction_matches<S: Scalar>() {
        let orig = bilinear::<S>();
        let (left, right) = orig.split_v(S::from_f64(0.5)).unwrap();
        for u_val in [0.0_f64, 0.5, 1.0] {
            let u = S::from_f64(u_val);
            let o = orig.evaluate(u, S::from_f64(0.5)).unwrap();
            let l = left.evaluate(u, S::from_f64(0.5)).unwrap();
            let r = right.evaluate(u, S::from_f64(0.5)).unwrap();
            for c in 0..3 {
                assert!(
                    o[c].could_be_equal(l[c]),
                    "split_v left mismatch u={u_val} c={c}"
                );
                assert!(
                    o[c].could_be_equal(r[c]),
                    "split_v right mismatch u={u_val} c={c}"
                );
            }
        }
    }
    #[test]
    fn split_v_junction_matches() {
        for_all_scalars!(check_split_v_junction_matches);
    }

    fn check_split_v_left_start_matches<S: Scalar>() {
        let orig = bilinear::<S>();
        let (left, _) = orig.split_v(S::from_f64(0.5)).unwrap();
        for u_val in [0.0_f64, 0.5, 1.0] {
            let u = S::from_f64(u_val);
            let o = orig.evaluate(u, S::ZERO).unwrap();
            let l = left.evaluate(u, S::ZERO).unwrap();
            for c in 0..3 {
                assert!(o[c].could_be_equal(l[c]), "c={c} u={u_val}");
            }
        }
    }
    #[test]
    fn split_v_left_start_matches() {
        for_all_scalars!(check_split_v_left_start_matches);
    }

    fn check_split_v_right_end_matches<S: Scalar>() {
        let orig = bilinear::<S>();
        let (_, right) = orig.split_v(S::from_f64(0.5)).unwrap();
        for u_val in [0.0_f64, 0.5, 1.0] {
            let u = S::from_f64(u_val);
            let o = orig.evaluate(u, S::ONE).unwrap();
            let r = right.evaluate(u, S::ONE).unwrap();
            for c in 0..3 {
                assert!(o[c].could_be_equal(r[c]), "c={c} u={u_val}");
            }
        }
    }
    #[test]
    fn split_v_right_end_matches() {
        for_all_scalars!(check_split_v_right_end_matches);
    }

    fn check_split_v_at_boundary_returns_err<S: Scalar>() {
        let s = bilinear::<S>();
        assert!(s.split_v(S::ZERO).is_err());
        assert!(s.split_v(S::ONE).is_err());
    }
    #[test]
    fn split_v_at_boundary_returns_err() {
        for_all_scalars!(check_split_v_at_boundary_returns_err);
    }

    fn check_split_u_domains<S: Scalar>() {
        let orig = bilinear::<S>();
        let (left, right) = orig.split_u(S::from_f64(0.5)).unwrap();
        let (lu_min, lu_max) = left.domain_u();
        let (ru_min, ru_max) = right.domain_u();
        assert!(lu_min.could_be_equal(S::ZERO));
        assert!(lu_max.could_be_equal(S::from_f64(0.5)));
        assert!(ru_min.could_be_equal(S::from_f64(0.5)));
        assert!(ru_max.could_be_equal(S::ONE));
    }
    #[test]
    fn split_u_domains() {
        for_all_scalars!(check_split_u_domains);
    }

    fn check_split_v_domains<S: Scalar>() {
        let orig = bilinear::<S>();
        let (left, right) = orig.split_v(S::from_f64(0.5)).unwrap();
        let (lv_min, lv_max) = left.domain_v();
        let (rv_min, rv_max) = right.domain_v();
        assert!(lv_min.could_be_equal(S::ZERO));
        assert!(lv_max.could_be_equal(S::from_f64(0.5)));
        assert!(rv_min.could_be_equal(S::from_f64(0.5)));
        assert!(rv_max.could_be_equal(S::ONE));
    }
    #[test]
    fn split_v_domains() {
        for_all_scalars!(check_split_v_domains);
    }
}
