use geop_core_math::{
    combinatorics::binomial,
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::{NurbSurface, evaluate::corner_nearest};
use crate::{
    knot_insertion::is_clamped_at,
    nurb_curve::dehomogenize,
    spline::{
        centered, de_boor, find_span, find_spans, hodograph, homogeneous_derivatives,
        rational_derivatives,
    },
};

impl NurbSurface<4> {
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
        u: Scalar,
        v: Scalar,
        n: usize,
    ) -> GeopResult<(Vec<Vector<4>>, Vec<Vector<4>>, Option<Vector<4>>)> {
        let (p, q) = (self.degree_u, self.degree_v);
        let (ku, kv) = (&self.knot_vector_u, &self.knot_vector_v);
        let nv = self.num_v;
        let span_u = find_span(p, ku, self.num_u - 1, u)?;
        let span_v = find_span(q, kv, nv - 1, v)?;
        // Relative to a control point of the span (see [`centered`]): the
        // derivatives do not depend on where the patch lies, and that way
        // neither does their width.
        let local: Vec<Vector<4>> = (span_u - p..=span_u)
            .flat_map(|i| (span_v - q..=span_v).map(move |j| i * nv + j))
            .map(|k| self.control_points[k])
            .collect();
        let at = corner_nearest(self, (span_u, u), (span_v, v));
        let (cp, _) = centered(&local, at);
        let at = |i: usize, j: usize| cp[i * (q + 1) + j];
        let mixed = n >= 2;
        // Each row at `v`, and along `v` where the mixed partial is asked.
        let (rows, rows_v): (Vec<Vector<4>>, Vec<Vector<4>>) = (0..=p)
            .map(|i| {
                let local: Vec<_> = (0..=q).map(|j| at(i, j)).collect();
                let d = homogeneous_derivatives(q, kv, &local, span_v, v, usize::from(mixed));
                (d[0], d.get(1).copied().unwrap_or_else(Vector::zero))
            })
            .unzip();
        let cols: Vec<Vector<4>> = (0..=q)
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

    /// Every partial derivative up to `order` at `(u, v)`: `S[k][l]` is
    /// `∂^(k+l) S / ∂u^k ∂v^l` for `k + l <= order`, `S[0][0]` the point.
    /// Each homogeneous partial is the span's own net differenced `k` times
    /// along `u` and `l` times along `v` ([`hodograph`]), then evaluated;
    /// the Cartesian ones follow by the rational quotient rule (Piegl &
    /// Tiller algorithm A4.4). Relative to a control point of the span, as
    /// everything here is (see [`centered`]).
    ///
    /// Differenced first, then evaluated, so that an interval `(u, v)` box
    /// gives an enclosure as wide as the box times the next derivative.
    /// Evaluated first along `v` and differenced along `u` afterwards, as
    /// [`Self::second_derivatives`] does for its mixed partial, the rows'
    /// values over the box are independent intervals, and their third
    /// differences, divided by the span's width cubed, came out a hundred
    /// times wider than the partial itself.
    ///
    /// A box reaching over a knot is taken span by span, and the partials
    /// over the pieces united (see [`find_spans`]): one span's polynomial
    /// carried on over the next encloses nothing of the surface there, and
    /// a proof over the box (`NurbSurface3D::enclose_foot`) would prove
    /// nothing.
    pub fn partials(&self, u: Scalar, v: Scalar, order: usize) -> GeopResult<Vec<Vec<Vector3>>> {
        let (p, q) = (self.degree_u, self.degree_v);
        let us = find_spans(p, &self.knot_vector_u, self.num_u - 1, u)?;
        let vs = find_spans(q, &self.knot_vector_v, self.num_v - 1, v)?;
        let mut out: Option<Vec<Vec<Vector3>>> = None;
        for &u in &us {
            for &v in &vs {
                let piece = self.partials_in(u, v, order)?;
                out = Some(match out {
                    None => piece,
                    Some(all) => (all.iter().zip(&piece))
                        .map(|(a, b)| a.iter().zip(b).map(|(a, b)| a.union(b)).collect())
                        .collect(),
                });
            }
        }
        Ok(out.expect("a span at least"))
    }

    /// [`Self::partials`] over the part `u` of span `span_u` along `u` and
    /// `v` of `span_v` along `v`.
    fn partials_in(
        &self,
        (span_u, u): (usize, Scalar),
        (span_v, v): (usize, Scalar),
        order: usize,
    ) -> GeopResult<Vec<Vec<Vector3>>> {
        let (p, q) = (self.degree_u, self.degree_v);
        let (ku, kv) = (&self.knot_vector_u, &self.knot_vector_v);
        let nv = self.num_v;
        let local: Vec<Vector<4>> = (span_u - p..=span_u)
            .flat_map(|i| (span_v - q..=span_v).map(move |j| i * nv + j))
            .map(|k| self.control_points[k])
            .collect();
        let at = corner_nearest(self, (span_u, u), (span_v, v));
        let (cp, origin) = centered(&local, at);
        let (base_u, base_v) = (span_u - p, span_v - q);
        // The net differenced `l` times along `v`, as rows of fixed `u`.
        let mut along_v: Vec<Vec<Vec<Vector<4>>>> = vec![
            (0..=p)
                .map(|i| cp[i * (q + 1)..(i + 1) * (q + 1)].to_vec())
                .collect(),
        ];
        for l in 1..=order.min(q) {
            let previous = &along_v[l - 1];
            along_v.push(
                previous
                    .iter()
                    .map(|row| hodograph(q, kv, row, base_v, l))
                    .collect(),
            );
        }
        // `a[k][l]`, the homogeneous partial: each of those differenced `k`
        // times along `u`, evaluated at `(u, v)`.
        let mut a = vec![vec![Vector::<4>::zero(); order + 1]; order + 1];
        for k in 0..=order {
            for l in 0..=order - k {
                if k > p || l > q {
                    continue;
                }
                let mut net: Vec<Vec<Vector<4>>> = along_v[l].clone();
                for kk in 1..=k {
                    let columns: Vec<Vec<Vector<4>>> = (0..=q - l)
                        .map(|j| {
                            let column: Vec<Vector<4>> = net.iter().map(|row| row[j]).collect();
                            hodograph(p, ku, &column, base_u, kk)
                        })
                        .collect();
                    net = (0..columns[0].len())
                        .map(|i| columns.iter().map(|c| c[i]).collect())
                        .collect();
                }
                let rows: Vec<Vector<4>> = net
                    .iter()
                    .map(|row| de_boor(q - l, &kv[base_v + l..], row, v, q - l))
                    .collect();
                a[k][l] = de_boor(p - k, &ku[base_u + k..], &rows, u, p - k);
            }
        }
        let w = |k: usize, l: usize| a[k][l][3];
        let inverse = Scalar::ONE.div(w(0, 0))?;
        let mut s: Vec<Vec<Vector3>> = vec![Vec::new(); order + 1];
        for k in 0..=order {
            for l in 0..=order - k {
                let mut value = Vector3::from_array([a[k][l][0], a[k][l][1], a[k][l][2]]);
                let term =
                    |f: i64, wv: Scalar, sv: &Vector3| sv.prod_scalar(Scalar::from_i64(f).mul(wv));
                for j in 1..=l {
                    value = value.sub(&term(binomial(l, j), w(0, j), &s[k][l - j]));
                }
                for i in 1..=k {
                    value = value.sub(&term(binomial(k, i), w(i, 0), &s[k - i][l]));
                    for j in 1..=l {
                        value = value.sub(&term(
                            binomial(k, i) * binomial(l, j),
                            w(i, j),
                            &s[k - i][l - j],
                        ));
                    }
                }
                s[k].push(value.prod_scalar(inverse));
            }
        }
        s[0][0] = s[0][0].add(&Vector3::from_array([origin[0], origin[1], origin[2]]));
        Ok(s)
    }

    /// Partial derivatives `(∂Scalar/∂u, ∂Scalar/∂v)` at `(u, v)`.
    pub fn derivatives(&self, u: Scalar, v: Scalar) -> GeopResult<(Vector3, Vector3)> {
        let (du, dv, _) = self.homogeneous_partials(u, v, 1)?;
        Ok((rational_derivatives(&du)?[1], rational_derivatives(&dv)?[1]))
    }

    /// Second partial derivatives `[∂²S/∂u², ∂²S/∂u∂v, ∂²S/∂v²]` at
    /// `(u, v)`. The pure ones by the rational quotient rule along their own
    /// direction ([`rational_derivatives`]); the mixed one by differentiating
    /// `A = w Scalar` once along each: `A_uv = w_uv Scalar + w_u S_v + w_v S_u + w
    /// S_uv`.
    pub fn second_derivatives(&self, u: Scalar, v: Scalar) -> GeopResult<[Vector3; 3]> {
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
    fn boundary_rows(&self, u_fixed: bool, first: bool) -> Option<[Vec<Vector<4>>; 2]> {
        let (nu, nv) = (self.num_u, self.num_v);
        let (knots, degree, len) = if u_fixed {
            (&self.knot_vector_u, self.degree_u, nu)
        } else {
            (&self.knot_vector_v, self.degree_v, nv)
        };
        if len < 2 || !is_clamped_at(knots, degree, first) {
            return None;
        }
        let (edge, next) = if first { (0, 1) } else { (len - 1, len - 2) };
        Some([
            self.control_row(u_fixed, edge),
            self.control_row(u_fixed, next),
        ])
    }

    /// The point the boundary row at the start (`first`) or end of the
    /// `u` (`u_fixed`) or `v` domain collapses to, and the row next to it.
    /// `None` if that row is no pole. Every point of the row encloses the
    /// pole, so the pole is what they share: no wider than the narrowest.
    fn pole(&self, u_fixed: bool, first: bool) -> Option<(Vector3, Vec<Vector<4>>)> {
        let [edge, next] = self.boundary_rows(u_fixed, first)?;
        let edge = dehomogenize::<4, 3>(&edge);
        let shared = edge.iter().fold(edge[0], |shared, p| {
            Vector3::from_array(std::array::from_fn(|k| shared[k].intersect(p[k])))
        });
        edge.iter()
            .all(|p| p.could_be_equal(&shared))
            .then_some((shared, next))
    }

    /// The boundary rows `(u, v)` touches, as `(u_fixed, first)`.
    fn rows_touching(&self, u: Scalar, v: Scalar) -> Vec<(bool, bool)> {
        let ((u0, u1), (v0, v1)) = (self.domain_u(), self.domain_v());
        [
            (false, v, v0, true),
            (false, v, v1, false),
            (true, u, u0, true),
            (true, u, u1, false),
        ]
        .into_iter()
        .filter(|&(_, t, end, _)| t.could_be_equal(end))
        .map(|(u_fixed, _, _, first)| (u_fixed, first))
        .collect()
    }

    /// The pole `(u, v)` lies on: the point a boundary row it touches
    /// collapses to. `None` if it touches no pole.
    pub fn pole_at(&self, u: Scalar, v: Scalar) -> Option<Vector3> {
        self.rows_touching(u, v)
            .into_iter()
            .find_map(|(u_fixed, first)| self.pole(u_fixed, first))
            .map(|(pole, _)| pole)
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
    fn pole_normal(&self, u_fixed: bool, first: bool) -> Option<GeopResult<Vector3>> {
        let (pole, next) = self.pole(u_fixed, first)?;
        let spokes: Vec<Vector3> = dehomogenize::<4, 3>(&next)
            .iter()
            .map(|x| x.sub(&pole))
            .collect();
        let turning = spokes.windows(2).fold(Vector3::zero(), |acc: Vector3, e| {
            acc.add(&e[0].prod_cross(&e[1]))
        });
        let result = turning.normalize().and_then(|t| {
            if !spokes
                .iter()
                .all(|e| e.prod_dot(&t).could_be_equal(Scalar::ZERO))
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

    /// Whether the boundary row at the start (`first`) or end of the `u`
    /// (`u_fixed`) or `v` domain is an apex: a pole with no single normal,
    /// its spokes not in one plane, like a cone's tip.
    pub(crate) fn is_apex(&self, u_fixed: bool, first: bool) -> bool {
        matches!(self.pole_normal(u_fixed, first), Some(Err(_)))
    }

    /// Unit surface normal at `(u, v)`, `normalize(∂Scalar/∂u × ∂Scalar/∂v)`.
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
    pub fn normal(&self, u: Scalar, v: Scalar) -> GeopResult<Vector3> {
        let (du, dv) = self.derivatives(u, v)?;
        let regular = du
            .prod_cross(&dv)
            .normalize()
            .with_context(&|e: GeopError| {
                e.with_context(format!(
                    "NurbSurface::normal({:?}): S_u = {du:?}, S_v = {dv:?}",
                    (u, v)
                ))
            });
        if regular.is_ok() {
            return regular;
        }
        for (u_fixed, first) in self.rows_touching(u, v) {
            if let Some(normal) = self.pole_normal(u_fixed, first) {
                return normal;
            }
        }
        regular
    }

    /// The unit normal of this surface at `point`, which lies on it: at
    /// its `(u, v)` there ([`Self::locate`]). An error if the point is not
    /// on the surface.
    pub fn normal_at(
        &self,
        point: &Vector3,
        max_nodes: usize,
        min_subdivision_size: Scalar,
    ) -> GeopResult<Vector3> {
        let (u, v) = self
            .locate(point, max_nodes, min_subdivision_size)?
            .ok_or_else(|| GeopError::new(format!("{point:?} is not on the surface")))?;
        self.normal(u, v)
    }
}

#[cfg(test)]
mod tests {
    use crate::nurb_surface::NurbSurface;
    use crate::test_fixtures::{hom, revolved};
    use geop_core_math::scalars::Scalar;

    /// `partials` agrees with `derivatives` and `second_derivatives` at a
    /// point, and over a box a thousandth wide is about that much wider
    /// than at its centre, up to third order: a rational patch, its net
    /// refined.
    #[test]
    fn partials_are_tight_over_a_box() {
        let surface = revolved(1., 0.).refined([4, 4]).unwrap();
        let (u, v) = (Scalar::from_f64(0.3), Scalar::from_f64(0.6));
        let at = surface.partials(u, v, 3).unwrap();
        let (su, sv) = surface.derivatives(u, v).unwrap();
        let [suu, suv, svv] = surface.second_derivatives(u, v).unwrap();
        for (got, expected) in [
            (at[1][0], su),
            (at[0][1], sv),
            (at[2][0], suu),
            (at[1][1], suv),
            (at[0][2], svv),
        ] {
            assert!(got.could_be_equal(&expected), "{got:?} vs {expected:?}");
        }
        assert!(at[0][0].could_be_equal(&surface.evaluate(u, v).unwrap()));
        // Over the box, each partial is wider by about the box's width
        // times the next one, `|S^(k+1)| h`: never a hundred times that.
        let h = 1e-3;
        let over = surface
            .partials(
                u.add(Scalar::new(-h / 2., h / 2.)),
                v.add(Scalar::new(-h / 2., h / 2.)),
                3,
            )
            .unwrap();
        let mut table = String::new();
        let mut worst: f64 = 0.;
        for k in 0..=3 {
            for l in 0..=3 - k {
                let width = (0..3)
                    .map(|c| over[k][l][c].width().to_f64())
                    .fold(0., f64::max);
                let size = (0..3)
                    .map(|c| at[k][l][c].abs().to_f64())
                    .fold(0., f64::max);
                table += &format!("S[{k}][{l}]: {size:.3e} wide {width:.3e}\n");
                worst = worst.max(width / h);
            }
        }
        // Measured: at most 24 h, the third partial along `u`. Formed by
        // evaluating rows first, the third partials were a hundred times
        // wider.
        assert!(worst < 50., "{table}");
    }

    /// Over a box reaching over a knot, every partial encloses its values
    /// on both sides of it: a surface's span is not carried on over the
    /// next one. A net joined at a knot only by its position there, bent
    /// at that joint, so that its derivatives jump, which one span's
    /// polynomial carried on misses.
    #[test]
    fn partials_over_a_knot_enclose_both_spans() {
        let surface = revolved(1., 0.).refined([4, 1]).unwrap();
        // Raised, its knots are joints of position only; bent at one.
        let mut surface = surface.elevate_degree(true).unwrap();
        let nv = surface.num_v;
        let mid = surface.degree_u;
        for j in 0..nv {
            let cp = &mut surface.control_points[mid * nv + j];
            cp[2] = cp[2].add(cp[3].mul(Scalar::from_f64(0.3)));
        }
        let knot = surface.knot_vector_u[surface.degree_u + 1];
        let box_u = knot
            .sub(Scalar::from_f64(0.05))
            .union(knot.add(Scalar::from_f64(0.05)));
        let v = Scalar::from_f64(0.4);
        let over = surface.partials(box_u, v, 2).unwrap();
        for i in 0..=10 {
            let u = Scalar::interpolate(
                box_u.lower(),
                box_u.upper(),
                Scalar::from_ratio(i, 10).unwrap(),
            );
            let at = surface.partials(u, v, 2).unwrap();
            for (k, l) in [(0, 0), (1, 0), (0, 1), (2, 0), (1, 1), (0, 2)] {
                assert!(
                    at[k][l].could_be_equal(&over[k][l]),
                    "S[{k}][{l}] at {u:?}: {:?}, over the box {:?}",
                    at[k][l],
                    over[k][l]
                );
            }
        }
    }

    /// Flat unit patch in the xy-plane: normal should be constant +z everywhere.
    fn flat_xy() -> NurbSurface<4> {
        let f = Scalar::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                hom(0., 0., 0., 1.),
                hom(0., 1., 0., 1.),
                hom(1., 0., 0., 1.),
                hom(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    #[test]
    fn flat_surface_normal_is_constant_z() {
        let s = flat_xy();
        for &(u, v) in &[(0.0, 0.0), (0.5, 0.5), (1.0, 0.0), (0.2, 0.8)] {
            let n = s.normal(Scalar::from_f64(u), Scalar::from_f64(v)).unwrap();
            assert!(n[0].could_be_equal(Scalar::ZERO));
            assert!(n[1].could_be_equal(Scalar::ZERO));
            assert!(n[2].could_be_equal(Scalar::ONE));
        }
    }

    /// Non-planar bilinear "saddle" patch: corner heights 0,1,1,0 over
    /// x,y ∈ [0,2]. At the center (u=v=0.5) the surface is tangent to the
    /// z=0.5 plane, so the normal there should be purely +z.
    fn bent_surface() -> NurbSurface<4> {
        let f = Scalar::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                hom(0., 0., 0., 1.),
                hom(0., 2., 1., 1.),
                hom(2., 0., 1., 1.),
                hom(2., 2., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    #[test]
    fn bent_surface_center_normal_is_z() {
        let s = bent_surface();
        let n = s
            .normal(Scalar::from_f64(0.5), Scalar::from_f64(0.5))
            .unwrap();
        assert!(n[0].could_be_equal(Scalar::ZERO));
        assert!(n[1].could_be_equal(Scalar::ZERO));
        assert!(n[2].definitely_greater(Scalar::ZERO));
    }

    // ── Poles ──────────────────────────────────────────────────────────────

    /// A hair off a disc's centre — the foot point of a point a shell moves
    /// onto the disc's axis — the normal is still the disc's: `S_u` is of
    /// the order of the hair, and its width that of the control points
    /// relative to the pole, an ulp of their coordinates. Evaluated
    /// relative to the far corner of the patch, as it once was, `S_u` was
    /// as wide as the patch. (Nearer the pole than a few hundred ulps, the
    /// width wins: there the normal is the pole's, see `pole_normal`.)
    #[test]
    fn disc_normal_a_hair_off_the_pole() {
        let f = Scalar::from_f64;
        let disc = revolved(2., 2.);
        for (u, v) in [(0., 5.4e-14), (1., 5.4e-14), (0.3, 1e-10)] {
            let n = disc.normal(f(u), f(v)).unwrap();
            let (su, sv) = disc.derivatives(f(u), f(v)).unwrap();
            assert!(
                n[2].abs().could_be_equal(Scalar::ONE) && n[0].width().to_f64() < 0.1,
                "at {:?}: {n:?}, S_u = {su:?}, S_v = {sv:?}",
                (u, v)
            );
        }
    }

    /// A disc's centre has the disc's normal — for every `u` at once, and
    /// the same as just off the pole.
    #[test]
    fn disc_centre_normal() {
        let f = Scalar::from_f64;
        let disc = revolved(0., 0.);
        let every_u = f(0.).union(f(1.));
        let n = disc.normal(every_u, f(0.)).unwrap();
        let off = disc.normal(f(0.5), f(0.5)).unwrap();
        assert!(n.could_be_equal(&off), "{n:?} vs {off:?}");
        assert!(
            n[2].abs().could_be_equal(Scalar::ONE) && n[0].could_be_equal(Scalar::ZERO),
            "{n:?}"
        );
    }

    /// A cone's apex has no single normal: its spokes are not coplanar.
    #[test]
    fn cone_apex_has_no_normal() {
        let f = Scalar::from_f64;
        let cone = revolved(1., 0.);
        assert!(cone.normal(f(0.).union(f(1.)), f(0.)).is_err());
        assert!(cone.normal(f(0.5), f(0.5)).is_ok());
    }

    /// Exact rational sphere octant; `v = 1` is the pole (0, 0, 1) — a
    /// collapsed row at the *end* of the domain, where the limit's sign
    /// flips.
    #[test]
    fn sphere_pole_normal() {
        let f = Scalar::from_f64;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let sphere = NurbSurface::try_new(
            2,
            2,
            vec![
                hom(1., 0., 0., 1.),
                hom(w, 0., w, w),
                hom(0., 0., 1., 1.),
                hom(w, w, 0., w),
                hom(0.5, 0.5, 0.5, 0.5),
                hom(0., 0., w, w),
                hom(0., 1., 0., 1.),
                hom(0., w, w, w),
                hom(0., 0., 1., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let n = sphere.normal(f(0.).union(f(1.)), f(1.)).unwrap();
        assert!(
            n[0].could_be_equal(Scalar::ZERO) && n[1].could_be_equal(Scalar::ZERO),
            "{n:?}"
        );
        // Same orientation as the regular normal just below the pole.
        let below = sphere.normal(f(0.5), f(0.9)).unwrap();
        assert!(
            n[2].mul(below[2]).definitely_greater(Scalar::ZERO),
            "{n:?} vs {below:?}"
        );
    }
}
