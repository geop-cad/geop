//! Surfaces spanned by curves: the ruled surface between two curves
//! ([`NurbSurface3D::ruled`]), the Coons patch bounded by four
//! ([`NurbSurface3D::coons`]), and such a patch made tangent to planes along
//! its sides ([`NurbSurface3D::tangent_to_planes`]).
//!
//! Both constructions work on the homogeneous control points, so they are
//! exact for rational curves too: a patch's side *is* the curve it was
//! built from, control point for control point, not an approximation of it.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::NurbSurface3D;
use crate::{
    nurb_curve::{NurbCurve, NurbCurve3D},
    shape::Plane,
};

/// The Greville abscissae of a B-spline of `degree` with `n` control points
/// over `knots`: the coefficients with which its basis reproduces the
/// linear function `t` — so a linear blend between two rows of control
/// points, laid across that basis, is exactly the blend.
fn greville<S: Scalar>(knots: &[S], degree: usize, n: usize) -> GeopResult<Vec<S>> {
    let p = S::from_i64(degree as i64);
    (0..n)
        .map(|i| {
            knots[i + 1..=i + degree]
                .iter()
                .fold(S::ZERO, |sum, &k| sum.add(k))
                .div(p)
        })
        .collect()
}

fn exactly_one<S: Scalar>(w: S) -> bool {
    w.is_subset_of(S::ONE) && S::ONE.is_subset_of(w)
}

/// The point the homogeneous control point `p` stands for.
fn point<S: Scalar>(p: &Vector4<S>) -> GeopResult<Vector3<S>> {
    Ok(Vector3::from_array([
        p[0].div(p[3])?,
        p[1].div(p[3])?,
        p[2].div(p[3])?,
    ]))
}

/// `p` with weight `w`, homogeneous — taken as it is where the weight is
/// exactly one: multiplying by it would only widen it by rounding.
fn weighted<S: Scalar>(p: &Vector3<S>, w: S) -> Vector4<S> {
    if exactly_one(w) {
        Vector4::from_array([p[0], p[1], p[2], w])
    } else {
        Vector4::from_array([p[0].mul(w), p[1].mul(w), p[2].mul(w), w])
    }
}

/// `a` and `b` on the domain `[0, 1]`, made compatible (see
/// [`NurbCurve::compatible`]).
fn compatible_pair<S: Scalar>(
    a: &NurbCurve3D<S>,
    b: &NurbCurve3D<S>,
) -> GeopResult<(NurbCurve3D<S>, NurbCurve3D<S>)> {
    let mut pair = NurbCurve::compatible(&[a.with_unit_domain()?, b.with_unit_domain()?])?;
    let b = pair.pop().expect("two curves");
    let a = pair.pop().expect("two curves");
    Ok((a, b))
}

/// One side of a patch, as [`NurbSurface3D::coons`] numbers its curves:
/// `v` at its lowest, `u` at its highest, `v` at its highest and `u` at its
/// lowest, running round the patch.
const SIDES: usize = 4;

impl<S: Scalar> NurbSurface3D<S> {
    /// The ruled surface between `a`, at `v = 0`, and `b`, at `v = 1`: the
    /// straight lines joining their points of equal parameter, both brought
    /// onto `[0, 1]`, which is the surface's `u`. Its sides along `u` are
    /// the two curves exactly.
    pub fn ruled(a: &NurbCurve3D<S>, b: &NurbCurve3D<S>) -> GeopResult<Self> {
        let (a, b) = compatible_pair(a, b)?;
        let control_points = a
            .control_points
            .iter()
            .zip(&b.control_points)
            .flat_map(|(p, q)| [*p, *q])
            .collect();
        NurbSurface3D::try_new(
            a.degree,
            1,
            control_points,
            a.knot_vector.clone(),
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
    }

    /// The bilinearly blended Coons patch bounded by `sides`, which run
    /// round it end to end: from the corner at `(0, 0)` along `v = 0` to
    /// `(1, 0)`, up `u = 1` to `(1, 1)`, back along `v = 1` to `(0, 1)` and
    /// down `u = 0` to the start — counter-clockwise in `(u, v)`, so the
    /// patch's normal is the one the loop winds around.
    ///
    /// The patch is the sum of the ruled surfaces between opposite sides,
    /// less the bilinear patch of the corners, each laid on one control net
    /// (the Greville abscissae turn a linear blend into control points
    /// exactly): opposite sides are made compatible, and each blend is
    /// taken of the homogeneous control points, so rational sides are fine.
    /// Its border rows are the sides' control points themselves — that is
    /// what the blend gives there exactly; taking them as they are only
    /// leaves out the rounding of adding and taking away the same corners —
    /// so the patch interpolates its sides exactly, and its corners are the
    /// union of the two sides meeting there.
    ///
    /// The sides must end where the next one starts, with their end weights
    /// one (or a single span, which can be brought to it): two sides meeting
    /// at a corner must agree on its weight for the blend to meet them both.
    pub fn coons(sides: [&NurbCurve3D<S>; SIDES]) -> GeopResult<Self> {
        let unit = |c: &NurbCurve3D<S>| c.with_unit_domain()?.with_unit_end_weights();
        let bottom = unit(sides[0])?;
        let right = unit(sides[1])?;
        let top = unit(&sides[2].reverse())?;
        let left = unit(&sides[3].reverse())?;
        let first = |c: &NurbCurve3D<S>| point(&c.control_points[0]);
        let last = |c: &NurbCurve3D<S>| point(&c.control_points[c.control_points.len() - 1]);
        for (k, (end, start)) in [
            (last(&bottom)?, first(&right)?),
            (last(&right)?, last(&top)?),
            (first(&top)?, last(&left)?),
            (first(&left)?, first(&bottom)?),
        ]
        .into_iter()
        .enumerate()
        {
            if !end.could_be_equal(&start) {
                return Err(GeopError::new(format!(
                    "NurbSurface::coons: side {k} ends at {end:?}, but side {} starts at {start:?}",
                    (k + 1) % SIDES
                )));
            }
        }
        let (bottom, top) = compatible_pair(&bottom, &top)?;
        let (left, right) = compatible_pair(&left, &right)?;
        let (nu, nv) = (bottom.control_points.len(), left.control_points.len());
        let eta = greville(&bottom.knot_vector, bottom.degree, nu)?;
        let xi = greville(&left.knot_vector, left.degree, nv)?;
        let (b, t) = (&bottom.control_points, &top.control_points);
        let (l, r) = (&left.control_points, &right.control_points);
        let p00 = b[0].union(&l[0]);
        let p10 = b[nu - 1].union(&r[0]);
        let p01 = t[0].union(&l[nv - 1]);
        let p11 = t[nu - 1].union(&r[nv - 1]);
        let rational = [b, t, l, r]
            .iter()
            .any(|row| row.iter().any(|p| !exactly_one(p[3])));

        let mut control_points = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            for j in 0..nv {
                let mut p = match (i, j) {
                    (0, 0) => p00,
                    (0, _) if j == nv - 1 => p01,
                    (_, 0) if i == nu - 1 => p10,
                    _ if i == nu - 1 && j == nv - 1 => p11,
                    (_, 0) => b[i],
                    (0, _) => l[j],
                    _ if j == nv - 1 => t[i],
                    _ if i == nu - 1 => r[j],
                    _ => {
                        let across = Vector4::interpolate(&b[i], &t[i], xi[j]);
                        let along = Vector4::interpolate(&l[j], &r[j], eta[i]);
                        let corners = Vector4::interpolate(
                            &Vector4::interpolate(&p00, &p01, xi[j]),
                            &Vector4::interpolate(&p10, &p11, xi[j]),
                            eta[i],
                        );
                        across.add(&along).sub(&corners)
                    }
                };
                if !rational {
                    // Every weight is one, and so is every blend of them:
                    // the arithmetic above would only round it.
                    p[3] = S::ONE;
                }
                control_points.push(p);
            }
        }
        NurbSurface3D::try_new(
            bottom.degree,
            left.degree,
            control_points,
            bottom.knot_vector.clone(),
            left.knot_vector.clone(),
        )
    }

    /// The same surface one degree higher in `u` (`along_u`) or in `v`, for
    /// knots clamped at both ends: each row of control points along that
    /// direction raised as a curve (see [`NurbCurve::elevate_degree`]).
    pub fn elevate_degree(&self, along_u: bool) -> GeopResult<Self> {
        let (nu, nv) = (self.num_u, self.num_v);
        let (degree, knots, rows, len) = if along_u {
            (self.degree_u, &self.knot_vector_u, nv, nu)
        } else {
            (self.degree_v, &self.knot_vector_v, nu, nv)
        };
        let index = |row: usize, k: usize| if along_u { k * nv + row } else { row * nv + k };
        let raised = (0..rows)
            .map(|row| {
                let points = (0..len).map(|k| self.control_points[index(row, k)]).collect();
                NurbCurve::try_new(degree, points, knots.clone())?.elevate_degree()
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let new_len = raised[0].control_points.len();
        let new_knots = raised[0].knot_vector.clone();
        let (num_u, num_v) = if along_u {
            (new_len, nv)
        } else {
            (nu, new_len)
        };
        let control_points = (0..num_u)
            .flat_map(|i| (0..num_v).map(move |j| (i, j)))
            .map(|(i, j)| {
                if along_u {
                    raised[j].control_points[i]
                } else {
                    raised[i].control_points[j]
                }
            })
            .collect();
        if along_u {
            NurbSurface3D::try_new(
                degree + 1,
                self.degree_v,
                control_points,
                new_knots,
                self.knot_vector_v.clone(),
            )
        } else {
            NurbSurface3D::try_new(
                self.degree_u,
                degree + 1,
                control_points,
                self.knot_vector_u.clone(),
                new_knots,
            )
        }
    }

    /// The same patch made tangent along each side `k` (numbered as
    /// [`NurbSurface3D::coons`] numbers them) that has a `planes[k] =
    /// Some((plane, away))` to that plane, the patch leaving the side
    /// towards `away` — a face on that plane ending at the side, the patch
    /// continuing it smoothly.
    ///
    /// The side itself must lie in the plane already. Its next row of
    /// control points is then moved into the plane, each point along the
    /// plane's normal: the derivative across a side is a combination of
    /// that row less the side, so with both in the plane, the patch's
    /// tangent plane is that plane all along the side — exactly, rational
    /// or not. Where two such rows cross, the point goes onto the line the
    /// two planes meet in. The patch is first raised in degree until each
    /// side moved has a row of its own, away from the opposite side's.
    ///
    /// The ends of that row are the second control points of the sides
    /// meeting this one: they are curves the patch must keep, so they are
    /// not moved, and must lie in the plane — a side leaving the plane at
    /// a corner contradicts the tangency, and is refused. So is a row that
    /// does not head towards `away`: the patch would fold back over the
    /// face it is to continue.
    pub fn tangent_to_planes(
        &self,
        planes: &[Option<(Plane<S>, Vector3<S>)>; SIDES],
    ) -> GeopResult<Self> {
        let rows_needed = |a: usize, b: usize| match (planes[a].is_some(), planes[b].is_some()) {
            (true, true) => 4,
            (true, false) | (false, true) => 3,
            (false, false) => 0,
        };
        let mut surface = self.clone();
        while surface.num_v < rows_needed(0, 2) {
            surface = surface.elevate_degree(false)?;
        }
        while surface.num_u < rows_needed(1, 3) {
            surface = surface.elevate_degree(true)?;
        }
        let (nu, nv) = (surface.num_u, surface.num_v);
        let at = |i: usize, j: usize| i * nv + j;
        // Per side: whether `(i, j)` is in the row next to it, and the
        // control point on the side that row leaves from.
        let next_row = |k: usize, i: usize, j: usize| -> Option<(usize, usize)> {
            match k {
                0 if j == 1 => Some((i, 0)),
                1 if i == nu - 2 => Some((nu - 1, j)),
                2 if j == nv - 2 => Some((i, nv - 1)),
                3 if i == 1 => Some((0, j)),
                _ => None,
            }
        };
        let original = surface.control_points.clone();
        for (k, side) in planes.iter().enumerate() {
            let Some((plane, _)) = side else { continue };
            let on_side = (0..nu)
                .flat_map(|i| (0..nv).map(move |j| (i, j)))
                .filter(|&(i, j)| match k {
                    0 => j == 0,
                    1 => i == nu - 1,
                    2 => j == nv - 1,
                    _ => i == 0,
                });
            for (i, j) in on_side {
                let p = point(&original[at(i, j)])?;
                if !plane.signed_distance(&p).could_be_equal(S::ZERO) {
                    return Err(GeopError::new(format!(
                        "NurbSurface::tangent_to_planes: side {k} leaves its plane at control point {p:?}"
                    )));
                }
            }
        }
        for i in 0..nu {
            for j in 0..nv {
                let constraints: Vec<(usize, &Plane<S>, &Vector3<S>, (usize, usize))> = (0..SIDES)
                    .filter_map(|k| {
                        let (plane, away) = planes[k].as_ref()?;
                        next_row(k, i, j).map(|base| (k, plane, away, base))
                    })
                    .collect();
                if constraints.is_empty() {
                    continue;
                }
                let on_border = i == 0 || j == 0 || i == nu - 1 || j == nv - 1;
                let cp = original[at(i, j)];
                let p = point(&cp)?;
                let moved = if on_border {
                    // The second control point of a side meeting this one:
                    // kept, and so it must be in the plane already.
                    for &(k, plane, _, _) in &constraints {
                        if !plane.signed_distance(&p).could_be_equal(S::ZERO) {
                            return Err(GeopError::new(format!(
                                "NurbSurface::tangent_to_planes: the side meeting side {k} at its corner leaves the plane there, at {p:?}: the patch cannot be tangent to the plane and keep that side"
                            )));
                        }
                    }
                    p
                } else {
                    match constraints.as_slice() {
                        [(_, plane, _, _)] => plane.project(&p),
                        [(_, a, _, _), (_, b, _, _)] => {
                            if a.normal.prod_cross(&b.normal).norm_sq().could_be_equal(S::ZERO) {
                                // One plane, met along two sides.
                                a.project(&p)
                            } else {
                                a.intersect_plane(b)?.project(&p)
                            }
                        }
                        _ => {
                            return Err(GeopError::new(format!(
                                "NurbSurface::tangent_to_planes: control point ({i}, {j}) is in the rows of {} sides",
                                constraints.len()
                            )));
                        }
                    }
                };
                for &(k, _, away, (bi, bj)) in &constraints {
                    let base = point(&original[at(bi, bj)])?;
                    if !moved.sub(&base).prod_dot(away).definitely_greater(S::ZERO) {
                        return Err(GeopError::new(format!(
                            "NurbSurface::tangent_to_planes: at control point ({i}, {j}) the patch does not leave side {k} away from the face it continues: it would fold back over it"
                        )));
                    }
                }
                if !on_border {
                    surface.control_points[at(i, j)] = weighted(&moved, cp[3]);
                }
            }
        }
        surface.recompute_aabb();
        Ok(surface)
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use crate::{
        nurb_curve::{NurbCurve, NurbCurve3D},
        nurb_surface::NurbSurface3D,
        shape::Plane,
    };

    fn v<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([x * w, y * w, z * w, w].map(S::from_f64))
    }

    fn line<S: Scalar>(a: [f64; 3], b: [f64; 3]) -> NurbCurve3D<S> {
        NurbCurve::try_new(
            1,
            vec![pt(a[0], a[1], a[2], 1.0), pt(b[0], b[1], b[2], 1.0)],
            vec![S::ZERO, S::ZERO, S::ONE, S::ONE],
        )
        .unwrap()
    }

    /// The quarter circle in the plane `x = x0` from `(x0, 0, 0)` round
    /// the center `(x0, 0, 1)` to `(x0, 1, 1)`: it leaves the plane `z = 0`
    /// along `y`.
    fn rising_arc<S: Scalar>(x0: f64) -> NurbCurve3D<S> {
        let f = S::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                pt(x0, 0., 0., 1.),
                pt(x0, 1., 0., std::f64::consts::SQRT_2 / 2.0),
                pt(x0, 1., 1., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// A cubic from the origin to `(1, 0, 0)`, with an interior knot,
    /// bending out of the plane `z = 0` by `lift`.
    fn wavy<S: Scalar>(lift: f64) -> NurbCurve3D<S> {
        let f = S::from_f64;
        NurbCurve::try_new(
            3,
            vec![
                pt(0., 0., 0., 1.),
                pt(0.3, -0.2, 0.4 * lift, 1.),
                pt(0.6, 0.3, -0.2 * lift, 1.),
                pt(0.8, -0.1, 0.3 * lift, 1.),
                pt(1., 0., 0., 1.),
            ],
            vec![
                f(0.),
                f(0.),
                f(0.),
                f(0.),
                f(0.5),
                f(1.),
                f(1.),
                f(1.),
                f(1.),
            ],
        )
        .unwrap()
    }

    fn assert_on<S: Scalar>(surface: &NurbSurface3D<S>, curve: &NurbCurve3D<S>, side: usize) {
        for k in 0..=8 {
            let s = S::from_ratio(k, 8).unwrap();
            let (u, v) = match side {
                0 => (s, S::ZERO),
                1 => (S::ONE, s),
                2 => (S::ONE.sub(s), S::ONE),
                _ => (S::ZERO, S::ONE.sub(s)),
            };
            let on_surface = surface.evaluate(u, v).unwrap();
            let unit = curve.with_unit_domain().unwrap();
            let on_curve = unit.evaluate(s).unwrap();
            assert!(
                on_surface.could_be_equal(&on_curve),
                "side {side} at {s:?}: {on_surface:?} vs {on_curve:?}"
            );
        }
    }

    /// A Coons patch of a wavy cubic, two quarter circles and a line: its
    /// border rows are the sides' own control points, and it runs along
    /// each side wherever it is evaluated.
    fn check_coons_interpolates_its_sides<S: Scalar>() {
        let sides = [
            wavy::<S>(1.0),
            line([1., 0., 0.], [1., 1., 1.]),
            line([1., 1., 1.], [0., 1., 1.]),
            rising_arc::<S>(0.).reverse(),
        ];
        let surface = NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]]).unwrap();
        // The cubic, on the patch's `u` knots, is its first row.
        let nv = surface.num_v;
        for (i, cp) in wavy::<S>(1.0).control_points.iter().enumerate() {
            let row = surface.control_points[i * nv];
            assert!(row.could_be_equal(cp), "{i}: {row:?} vs {cp:?}");
        }
        // The arc, raised to the cubic's degree... is the first column,
        // evaluated: every side lies on the patch.
        for (k, side) in sides.iter().enumerate() {
            assert_on(&surface, side, k);
        }
    }
    #[test]
    fn coons_interpolates_its_sides() {
        for_all_scalars!(check_coons_interpolates_its_sides);
    }

    /// Four lines of a square make the square itself, flat.
    fn check_coons_of_a_square_is_flat<S: Scalar>() {
        let c = [[0., 0., 0.], [2., 0., 0.], [2., 1., 0.], [0., 1., 0.]];
        let sides: Vec<_> = (0..4).map(|k| line::<S>(c[k], c[(k + 1) % 4])).collect();
        let surface = NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]]).unwrap();
        let plane = surface.as_plane().unwrap().expect("flat");
        assert!(plane.normal.could_be_equal(&v(0., 0., 1.)), "{plane:?}");
    }
    #[test]
    fn coons_of_a_square_is_flat() {
        for_all_scalars!(check_coons_of_a_square_is_flat);
    }

    /// Sides that do not meet are refused.
    fn check_coons_needs_a_closed_loop<S: Scalar>() {
        let a = line::<S>([0., 0., 0.], [1., 0., 0.]);
        let b = line::<S>([1., 0., 0.], [1., 1., 0.]);
        let c = line::<S>([1., 1., 0.], [0., 1., 0.]);
        let d = line::<S>([0., 1.5, 0.], [0., 0., 0.]);
        assert!(NurbSurface3D::coons([&a, &b, &c, &d]).is_err());
    }
    #[test]
    fn coons_needs_a_closed_loop() {
        for_all_scalars!(check_coons_needs_a_closed_loop);
    }

    /// The ruled surface between two curves runs along both.
    fn check_ruled_runs_along_both<S: Scalar>() {
        let a = wavy::<S>(1.0);
        let b = rising_arc::<S>(2.0);
        let surface = NurbSurface3D::ruled(&a, &b).unwrap();
        for k in 0..=8 {
            let s = S::from_ratio(k, 8).unwrap();
            let pa = a.evaluate(s).unwrap();
            let pb = b.evaluate(s).unwrap();
            assert!(surface.evaluate(s, S::ZERO).unwrap().could_be_equal(&pa));
            assert!(surface.evaluate(s, S::ONE).unwrap().could_be_equal(&pb));
        }
    }
    #[test]
    fn ruled_runs_along_both() {
        for_all_scalars!(check_ruled_runs_along_both);
    }

    /// A patch rising from a wavy curve in the plane `z = 0` between two
    /// quarter circles that leave it along `y`: made tangent to the plane
    /// along its bottom side, its normal there is the plane's, all along
    /// the side — and its sides are where they were.
    fn check_tangent_to_a_plane<S: Scalar>() {
        // The top bends up and down: the blend alone does not keep the
        // plane's tangent.
        let top = {
            let f = S::from_f64;
            NurbCurve::try_new(
                3,
                vec![
                    pt(0., 1., 1., 1.),
                    pt(0.3, 1., 1.5, 1.),
                    pt(0.6, 1., 0.6, 1.),
                    pt(0.8, 1., 1.3, 1.),
                    pt(1., 1., 1., 1.),
                ],
                [0., 0., 0., 0., 0.5, 1., 1., 1., 1.].map(f).to_vec(),
            )
            .unwrap()
            .reverse()
        };
        let sides = [
            wavy::<S>(0.0),
            rising_arc::<S>(1.),
            top,
            rising_arc::<S>(0.).reverse(),
        ];
        let coons = NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]]).unwrap();
        let plane = Plane::try_new(v(0., 0., 0.), v(0., 0., 1.)).unwrap();
        let away = v(0., 1., 0.);
        let bends = (0..=8).any(|k| {
            let u = S::from_ratio(k, 8).unwrap();
            let n = coons.normal(u, S::ZERO).unwrap();
            !n.prod_cross(&v(0., 0., 1.)).norm_sq().could_be_equal(S::ZERO)
        });
        assert!(bends, "the Coons patch alone is not tangent to the plane");
        let surface = coons
            .tangent_to_planes(&[Some((plane, away)), None, None, None])
            .unwrap();
        for (k, side) in sides.iter().enumerate() {
            assert_on(&surface, side, k);
        }
        for k in 0..=8 {
            let u = S::from_ratio(k, 8).unwrap();
            let n = surface.normal(u, S::ZERO).unwrap();
            assert!(
                n.prod_cross(&v(0., 0., 1.))
                    .norm_sq()
                    .could_be_equal(S::ZERO),
                "at {u:?}: {n:?}"
            );
        }
        // Straight sides leaving the plane at an angle cannot be kept.
        let steep = [
            line::<S>([0., 0., 0.], [1., 0., 0.]),
            line([1., 0., 0.], [1., 1., 1.]),
            line([1., 1., 1.], [0., 1., 1.]),
            line([0., 1., 1.], [0., 0., 0.]),
        ];
        let coons = NurbSurface3D::coons([&steep[0], &steep[1], &steep[2], &steep[3]]).unwrap();
        let plane = Plane::try_new(v(0., 0., 0.), v(0., 0., 1.)).unwrap();
        assert!(
            coons
                .tangent_to_planes(&[Some((plane, away)), None, None, None])
                .is_err()
        );
    }
    #[test]
    fn tangent_to_a_plane() {
        for_all_scalars!(check_tangent_to_a_plane);
    }
}
