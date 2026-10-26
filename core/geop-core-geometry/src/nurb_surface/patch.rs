//! Surfaces spanned by curves: the ruled surface between two curves
//! ([`NurbSurface3D::ruled`]), the Coons patch bounded by four
//! ([`NurbSurface3D::coons`]), and such a patch made tangent to planes along
//! its sides to the faces it continues ([`NurbSurface3D::tangent_to`]).
//!
//! Both constructions work on the homogeneous control points, so they are
//! exact for rational curves too: a patch's side *is* the curve it was
//! built from, control point for control point, not an approximation of it.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::{
        linear::solve_pivoted,
        repeat::{Step, repeat},
    },
    vector::{Vector, Vector3, Vector4},
};

use super::NurbSurface3D;
use crate::{
    nurb_curve::{NurbCurve, NurbCurve3D, basis_funs, bezier_knots},
    shape::Plane,
    spline::find_span,
};

/// The Greville abscissae of a B-spline of `degree` with `n` control points
/// over `knots`: the coefficients with which its basis reproduces the
/// linear function `t` — so a linear blend between two rows of control
/// points, laid across that basis, is exactly the blend.
fn greville(knots: &[Scalar], degree: usize, n: usize) -> GeopResult<Vec<Scalar>> {
    let p = Scalar::from_i64(degree as i64);
    (0..n)
        .map(|i| {
            knots[i + 1..=i + degree]
                .iter()
                .fold(Scalar::ZERO, |sum, &k| sum.add(k))
                .div(p)
        })
        .collect()
}

/// `a` and `b` on the domain `[0, 1]`, made compatible (see
/// [`NurbCurve::compatible`]).
fn compatible_pair(a: &NurbCurve3D, b: &NurbCurve3D) -> GeopResult<(NurbCurve3D, NurbCurve3D)> {
    let mut pair = NurbCurve::compatible(&[a.with_unit_domain()?, b.with_unit_domain()?])?;
    let b = pair.pop().expect("two curves");
    let a = pair.pop().expect("two curves");
    Ok((a, b))
}

/// One side of a patch, as [`NurbSurface3D::coons`] numbers its curves:
/// `v` at its lowest, `u` at its highest, `v` at its highest and `u` at its
/// lowest, running round the patch.
const SIDES: usize = 4;

impl NurbSurface3D {
    /// The ruled surface between `a`, at `v = 0`, and `b`, at `v = 1`: the
    /// straight lines joining their points of equal parameter, both brought
    /// onto `[0, 1]`, which is the surface's `u`. Its sides along `u` are
    /// the two curves exactly.
    pub fn ruled(a: &NurbCurve3D, b: &NurbCurve3D) -> GeopResult<Self> {
        let (a, b) = compatible_pair(a, b)?;
        Self::ruled_compatible(&a, &b)
    }

    /// The ruled surface between `a`, at `v = 0`, and `b`, at `v = 1`, which
    /// are already alike — of one degree, as many control points and knots —
    /// taken as they are: its `u` runs over `a`'s knots, not brought onto
    /// `[0, 1]`. Exact, since its sides along `u` are the curves' own control
    /// points.
    pub fn ruled_compatible(a: &NurbCurve3D, b: &NurbCurve3D) -> GeopResult<Self> {
        if a.degree != b.degree
            || a.control_points.len() != b.control_points.len()
            || a.knot_vector.len() != b.knot_vector.len()
        {
            return Err(GeopError::new(format!(
                "NurbSurface::ruled_compatible: the curves are not alike: {a:?} and {b:?}"
            )));
        }
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
            bezier_knots(1),
        )
    }

    /// The bilinear patch with corners `p00` at `(u, v) = (0, 0)`, `p10` at
    /// `(1, 0)`, `p11` at `(1, 1)` and `p01` at `(0, 1)`, over `[0, 1]²` —
    /// a plane's, where the four lie in one.
    pub fn bilinear(p00: Vector3, p10: Vector3, p11: Vector3, p01: Vector3) -> GeopResult<Self> {
        NurbSurface3D::try_new(
            1,
            1,
            [p00, p01, p10, p11]
                .map(|p| p.homogeneous(Scalar::ONE))
                .to_vec(),
            bezier_knots(1),
            bezier_knots(1),
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
    pub fn coons(sides: [&NurbCurve3D; SIDES]) -> GeopResult<Self> {
        let unit = |c: &NurbCurve3D| c.with_unit_domain()?.with_unit_end_weights();
        let bottom = unit(sides[0])?;
        let right = unit(sides[1])?;
        let top = unit(&sides[2].reverse())?;
        let left = unit(&sides[3].reverse())?;
        let first = |c: &NurbCurve3D| c.start();
        let last = |c: &NurbCurve3D| c.end();
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
            .any(|row| row.iter().any(|p| !p[3].is_exactly(Scalar::ONE)));

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
                    p[3] = Scalar::ONE;
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
                let points = (0..len)
                    .map(|k| self.control_points[index(row, k)])
                    .collect();
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

    /// The same surface carried on by `by` of its `u` (`along_u`) or `v`
    /// parameter past the end (`at_end`) or before the start of that
    /// domain: each row of control points along that direction continued
    /// as a curve (see [`NurbCurve::extended`]), so the surface goes on as
    /// the polynomial it is there, as smooth as it was.
    pub fn extended(&self, along_u: bool, at_end: bool, by: Scalar) -> GeopResult<Self> {
        let (nu, nv) = (self.num_u, self.num_v);
        let (degree, knots, rows, len) = if along_u {
            (self.degree_u, &self.knot_vector_u, nv, nu)
        } else {
            (self.degree_v, &self.knot_vector_v, nu, nv)
        };
        let index = |row: usize, k: usize| if along_u { k * nv + row } else { row * nv + k };
        let longer = (0..rows)
            .map(|row| {
                let points = (0..len)
                    .map(|k| self.control_points[index(row, k)])
                    .collect();
                NurbCurve::try_new(degree, points, knots.clone())?.extended(at_end, by)
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let new_len = longer[0].control_points.len();
        let new_knots = longer[0].knot_vector.clone();
        let (num_u, num_v) = if along_u {
            (new_len, nv)
        } else {
            (nu, new_len)
        };
        let control_points = (0..num_u)
            .flat_map(|i| (0..num_v).map(move |j| (i, j)))
            .map(|(i, j)| {
                if along_u {
                    longer[j].control_points[i]
                } else {
                    longer[i].control_points[j]
                }
            })
            .collect();
        if along_u {
            NurbSurface3D::try_new(
                degree,
                self.degree_v,
                control_points,
                new_knots,
                self.knot_vector_v.clone(),
            )
        } else {
            NurbSurface3D::try_new(
                self.degree_u,
                degree,
                control_points,
                self.knot_vector_u.clone(),
                new_knots,
            )
        }
    }

    /// The same patch made tangent along each side `k` (numbered as
    /// [`NurbSurface3D::coons`] numbers them) that has a `sides[k]`, to the
    /// face it names — a face ending at the side, the patch continuing it
    /// smoothly.
    ///
    /// The patch is first raised in degree until each such side has a row
    /// of its own next to it, away from the opposite side's. Each control
    /// point of that row is then moved, along the face's normal, into the
    /// face's tangent plane where the side is at that point's Greville
    /// abscissa: the derivative across a side is a combination of that row
    /// less the side, so a row in the tangent plane makes the patch tangent
    /// there. Where two such rows cross, the point goes onto the line the
    /// two tangent planes meet in.
    ///
    /// Along a flat face the tangent plane is one plane, and the patch is
    /// tangent to it exactly — rational or not. Along a curved one it
    /// turns, and no NURBS row follows it exactly: the row is set so that
    /// the patch leaves the face at [`TILT`] towards the face's normal at
    /// each of those abscissae — where an approximate tangency would dip
    /// through the face as often as not, a crease of a millionth of a
    /// radian keeps the patch on one side of it — and checked between them
    /// to stay within twice that, on that side, the patch refined along the
    /// side while it does not.
    ///
    /// The ends of that row are the second control points of the sides
    /// meeting this one: they are curves the patch must keep, so they are
    /// not moved, and must lie in the tangent plane at the corner — a side
    /// leaving it there contradicts the tangency, and is refused. So is a
    /// row that does not head away from the face: the patch would fold back
    /// over the face it is to continue.
    pub fn tangent_to(&self, sides: &[Option<Neighbour<'_>>; SIDES]) -> GeopResult<Self> {
        let curved = |k: usize| sides[k].as_ref().is_some_and(|s| !s.flat);
        let mut base = self.clone();
        let mut strays = String::new();
        for _ in 0..=MOST_REFINEMENTS {
            let patch = base.tangent_rows(sides)?;
            match patch.strays(sides)? {
                None => return Ok(patch),
                Some(at) => strays = at,
            }
            let along = |even: bool| (0..SIDES).any(|k| curved(k) && (k % 2 == 0) == even);
            base = base.refined([1 + usize::from(along(true)), 1 + usize::from(along(false))])?;
        }
        Err(GeopError::new(format!(
            "NurbSurface::tangent_to: refined {MOST_REFINEMENTS} times along its curved neighbours, the patch still does not leave them at {TILT:e} radians: {strays}"
        )))
    }

    /// The patch's rows next to the sides of `sides` set as
    /// [`NurbSurface3D::tangent_to`] says, once, at its knots.
    fn tangent_rows(&self, sides: &[Option<Neighbour<'_>>; SIDES]) -> GeopResult<Self> {
        let rows_needed = |a: usize, b: usize| match (sides[a].is_some(), sides[b].is_some()) {
            (true, true) => 4,
            (true, false) | (false, true) => 3,
            (false, false) => 0,
        };
        let mut surface = self.clone();
        // Raised along `v` for the sides 0 and 2, along `u` for 1 and 3,
        // until there are as many rows as the faces need. Each raise adds
        // a row at least.
        for (along_u, needed) in [(false, rows_needed(0, 2)), (true, rows_needed(1, 3))] {
            repeat(needed + 1, || -> GeopResult<_> {
                let rows = if along_u {
                    surface.num_u
                } else {
                    surface.num_v
                };
                if rows >= needed {
                    return Ok(Step::Done(()));
                }
                surface = surface.elevate_degree(along_u)?;
                Ok(Step::Again)
            })?
            .done("raising the patch's degree for its tangent planes")?;
        }
        let (nu, nv) = (surface.num_u, surface.num_v);
        let at = |i: usize, j: usize| i * nv + j;
        // Per side: the face's normal and the way away from it at each
        // position along the side, where it is at that position's Greville
        // abscissa.
        let mut planes: [Vec<(Vector3, Vector3)>; SIDES] = Default::default();
        for (k, side) in sides.iter().enumerate() {
            let Some(side) = side else { continue };
            let rows = SideRows::of(&surface, k);
            for s in rows.abscissae(&surface)? {
                planes[k].push((side.at)(rows.fraction(&surface, s)?)?);
            }
            if side.flat {
                let plane = Plane::try_new(surface.evaluate_corner(&rows)?, planes[k][0].0)?;
                for i in 0..rows.len() {
                    let p = surface.control_points[rows.index(i).0].dehomogenized()?;
                    if !plane.signed_distance(&p).could_be_equal(Scalar::ZERO) {
                        return Err(GeopError::new(format!(
                            "NurbSurface::tangent_to: side {k} leaves the plane of its flat neighbour at control point {p:?}"
                        )));
                    }
                }
            }
        }
        // Per side whose next row `(i, j)` is in: the side, the position
        // along it, and the control point on the side that row leaves from.
        let next_row = |k: usize, i: usize, j: usize| -> Option<(usize, (usize, usize))> {
            match k {
                0 if j == 1 => Some((i, (i, 0))),
                1 if i == nu - 2 => Some((j, (nu - 1, j))),
                2 if j == nv - 2 => Some((i, (i, nv - 1))),
                3 if i == 1 => Some((j, (0, j))),
                _ => None,
            }
        };
        let original = surface.control_points.clone();
        // Per side, the positions along it whose next row point two sides
        // decide.
        let mut shared: [Vec<bool>; SIDES] = Default::default();
        for k in 0..SIDES {
            shared[k] = vec![false; planes[k].len()];
        }
        for i in 0..nu {
            for j in 0..nv {
                // The side, its tangent plane through the point on the
                // side, the way away from it, and that point.
                type Constraint = (usize, usize, Plane, Vector3, Vector3);
                let mut constraints: Vec<Constraint> = Vec::new();
                for k in 0..SIDES {
                    let Some((position, (bi, bj))) = next_row(k, i, j) else {
                        continue;
                    };
                    if sides[k].is_none() {
                        continue;
                    }
                    let (normal, away) = planes[k][position];
                    let base = original[at(bi, bj)].dehomogenized()?;
                    constraints.push((k, position, Plane::try_new(base, normal)?, away, base));
                }
                if constraints.is_empty() {
                    continue;
                }
                if let [(a, pa, ..), (b, pb, ..)] = constraints.as_slice() {
                    shared[*a][*pa] = true;
                    shared[*b][*pb] = true;
                }
                let on_border = i == 0 || j == 0 || i == nu - 1 || j == nv - 1;
                let cp = original[at(i, j)];
                let p = cp.dehomogenized()?;
                let moved = if on_border {
                    // The second control point of a side meeting this one:
                    // kept, and so it must be in the plane already.
                    for (k, _, plane, ..) in &constraints {
                        if !plane.signed_distance(&p).could_be_equal(Scalar::ZERO) {
                            return Err(GeopError::new(format!(
                                "NurbSurface::tangent_to: the side meeting side {k} at its corner leaves the face's tangent plane there, at {p:?}: the patch cannot be tangent to the face and keep that side"
                            )));
                        }
                    }
                    p
                } else {
                    match constraints.as_slice() {
                        [(_, _, plane, ..)] => plane.project(&p),
                        [(_, _, a, ..), (_, _, b, ..)] => {
                            if a.normal
                                .prod_cross(&b.normal)
                                .norm_sq()
                                .could_be_equal(Scalar::ZERO)
                            {
                                // One plane, met along two sides.
                                a.project(&p)
                            } else {
                                a.intersect_plane(b)?.project(&p)
                            }
                        }
                        _ => {
                            return Err(GeopError::new(format!(
                                "NurbSurface::tangent_to: control point ({i}, {j}) is in the rows of {} sides",
                                constraints.len()
                            )));
                        }
                    }
                };
                for (k, _, _, away, base) in &constraints {
                    if !moved
                        .sub(base)
                        .prod_dot(away)
                        .definitely_greater(Scalar::ZERO)
                    {
                        return Err(GeopError::new(format!(
                            "NurbSurface::tangent_to: at control point ({i}, {j}) the patch does not leave side {k} away from the face it continues: it would fold back over it"
                        )));
                    }
                }
                if !on_border {
                    surface.control_points[at(i, j)] = moved.homogeneous(cp[3]);
                }
            }
        }
        for (k, side) in sides.iter().enumerate() {
            if side.as_ref().is_some_and(|s| !s.flat) {
                surface.lean(k, &planes[k], &shared[k])?;
            }
        }
        surface.recompute_aabb();
        Ok(surface)
    }

    /// The point where side `rows` starts.
    fn evaluate_corner(&self, rows: &SideRows) -> GeopResult<Vector3> {
        self.control_points[rows.index(0).0].dehomogenized()
    }

    /// The row next to side `k` moved along the face's normals `planes`
    /// (per position along the side, at its Greville abscissa) so that the
    /// patch leaves the face at [`TILT`] towards its normal at each of those
    /// abscissae — but where the row's ends are, which the sides meeting
    /// this one keep, and the points `shared` with another side's row: a
    /// collocation, linear in how far each point moves.
    ///
    /// Where the patch is at `S(s)` on the side, the derivative into it is
    /// a positive multiple of `Σ_i N_i(s) (w1_i (p1_i - S) - w0_i (p0_i -
    /// S))` over the side's points `p0` and the next row's `p1`, weighted —
    /// which moving `p1_i` by `c_i n_i` changes by `N_i(s) w1_i c_i n_i`.
    /// How far each point moves is a free choice, sharpened.
    fn lean(&mut self, k: usize, planes: &[(Vector3, Vector3)], shared: &[bool]) -> GeopResult<()> {
        let rows = SideRows::of(self, k);
        let (degree, knots) = rows.knots(self);
        let len = rows.len();
        let abscissae = rows.abscissae(self)?;
        let free: Vec<usize> = (1..len - 1).filter(|&i| !shared[i]).collect();
        if free.is_empty() {
            return Ok(());
        }
        let tilt = Scalar::from_f64(TILT);
        let point = |i: usize, row: usize| -> GeopResult<(Vector3, Scalar)> {
            let cp = self.control_points[if row == 0 {
                rows.index(i).0
            } else {
                rows.index(i).1
            }];
            Ok((cp.dehomogenized()?, cp[3]))
        };
        let mut a = Vec::with_capacity(free.len());
        let mut rhs = Vec::with_capacity(free.len());
        for &j in &free {
            let s = abscissae[j];
            let (u, v) = rows.uv(self, s);
            let on_side = self.evaluate(u, v)?;
            let normal = planes[j].0;
            let span = find_span(degree, knots, len - 1, s)?;
            let funs = basis_funs(span, s, degree, knots);
            let mut into = Vector3::zero();
            for (f, &n) in funs.iter().enumerate() {
                let i = span - degree + f;
                let ((p0, w0), (p1, w1)) = (point(i, 0)?, point(i, 1)?);
                let term = p1
                    .sub(&on_side)
                    .prod_scalar(w1)
                    .sub(&p0.sub(&on_side).prod_scalar(w0));
                into = into.add(&term.prod_scalar(n));
            }
            let mut row = vec![Scalar::ZERO; free.len()];
            for (f, &n) in funs.iter().enumerate() {
                let i = span - degree + f;
                if let Some(c) = free.iter().position(|&x| x == i) {
                    row[c] = n.mul(point(i, 1)?.1).mul(planes[i].0.prod_dot(&normal));
                }
            }
            a.push(row);
            let target = tilt.mul(into.norm());
            rhs.push(Vector::from_array([target.sub(into.prod_dot(&normal))]));
        }
        let moves = solve_pivoted(a, rhs).map_err(|e| {
            e.with_context(format!(
                "NurbSurface::tangent_to: leaning the row next to side {k} off its curved neighbour"
            ))
        })?;
        let moved = (free.iter().zip(&moves))
            .map(|(&i, by)| {
                let (p, w) = point(i, 1)?;
                Ok((
                    i,
                    p.add(&planes[i].0.prod_scalar(by[0]))
                        .sharpen()
                        .homogeneous(w),
                ))
            })
            .collect::<GeopResult<Vec<_>>>()?;
        for (i, cp) in moved {
            self.control_points[rows.index(i).1] = cp;
        }
        Ok(())
    }

    /// Where the patch strays, along a side of `sides` whose face is
    /// curved, from leaving that face as [`NurbSurface3D::lean`] has it:
    /// away from the face, towards its normal at no more than twice
    /// [`TILT`]. Checked at each Greville abscissa and halfway between.
    fn strays(&self, sides: &[Option<Neighbour<'_>>; SIDES]) -> GeopResult<Option<String>> {
        let tilt = Scalar::from_f64(2.0 * TILT);
        for (k, side) in sides.iter().enumerate() {
            let Some(side) = side.as_ref().filter(|s| !s.flat) else {
                continue;
            };
            let rows = SideRows::of(self, k);
            let abscissae = rows.abscissae(self)?;
            let mut checked = Vec::with_capacity(2 * abscissae.len());
            for pair in abscissae.windows(2) {
                checked.push(pair[0]);
                checked.push(pair[0].add(pair[1]).div(Scalar::TWO)?.sharpen());
            }
            for s in checked.into_iter().skip(1) {
                let (u, v) = rows.uv(self, s);
                let point = self.evaluate(u, v)?;
                let (normal, away) = (side.at)(rows.fraction(self, s)?)?;
                let into = rows.across(self, s)?;
                let lean = into.prod_dot(&normal);
                let ok = into.prod_dot(&away).definitely_greater(Scalar::ZERO)
                    && lean.definitely_greater(Scalar::ZERO)
                    && lean.definitely_less(tilt.mul(into.norm()));
                if !ok {
                    let angle = lean.div(into.norm())?;
                    return Ok(Some(format!(
                        "along side {k} at {s:?}, at {point:?}, it leaves the face at {angle:?} radians towards its normal"
                    )));
                }
            }
        }
        Ok(None)
    }
}

/// The angle, in radians, at which a patch made tangent to a curved face
/// leaves it towards its normal (see [`NurbSurface3D::tangent_to`]).
pub const TILT: f64 = 1e-6;

/// How often [`NurbSurface3D::tangent_to`] halves the patch's knot spans
/// along a curved neighbour, at most: how hard it tries to follow the
/// face's turning tangent plane, not what following it means.
const MOST_REFINEMENTS: usize = 6;

/// A face a patch is made tangent to along one of its sides (see
/// [`NurbSurface3D::tangent_to`]).
pub struct Neighbour<'a> {
    /// At the fraction `t` of the way along the side, as it runs round the
    /// patch: the face's unit normal there, and the way in its tangent
    /// plane away from it, across the side.
    pub at: &'a dyn Fn(Scalar) -> GeopResult<(Vector3, Vector3)>,
    /// Whether its tangent plane is one plane all along the side: a flat
    /// face.
    pub flat: bool,
}

/// Side `k` of a patch's control net, as [`NurbSurface3D::coons`] numbers
/// them: its control points, and those of the row next to it.
struct SideRows {
    k: usize,
    nu: usize,
    nv: usize,
}

impl SideRows {
    fn of(surface: &NurbSurface3D, k: usize) -> Self {
        Self {
            k,
            nu: surface.num_u,
            nv: surface.num_v,
        }
    }

    /// Whether the side runs along `u`.
    fn along_u(&self) -> bool {
        self.k.is_multiple_of(2)
    }

    /// How many control points it has.
    fn len(&self) -> usize {
        if self.along_u() { self.nu } else { self.nv }
    }

    /// The index of its control point at position `i`, and of the next
    /// row's.
    fn index(&self, i: usize) -> (usize, usize) {
        let (nu, nv) = (self.nu, self.nv);
        let at = |a: usize, b: usize| a * nv + b;
        match self.k {
            0 => (at(i, 0), at(i, 1)),
            1 => (at(nu - 1, i), at(nu - 2, i)),
            2 => (at(i, nv - 1), at(i, nv - 2)),
            _ => (at(0, i), at(1, i)),
        }
    }

    /// The degree and the knots along the side.
    fn knots<'s>(&self, surface: &'s NurbSurface3D) -> (usize, &'s [Scalar]) {
        if self.along_u() {
            (surface.degree_u, &surface.knot_vector_u)
        } else {
            (surface.degree_v, &surface.knot_vector_v)
        }
    }

    /// The Greville abscissae of its control points.
    fn abscissae(&self, surface: &NurbSurface3D) -> GeopResult<Vec<Scalar>> {
        let (degree, knots) = self.knots(surface);
        greville(knots, degree, self.len())
    }

    /// The patch's `(u, v)` at `s` along the side.
    fn uv(&self, surface: &NurbSurface3D, s: Scalar) -> (Scalar, Scalar) {
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        match self.k {
            0 => (s, v0),
            1 => (u1, s),
            2 => (s, v1),
            _ => (u0, s),
        }
    }

    /// How far `s` is along the side, as a fraction, the way the side runs
    /// round the patch: with its parameter for the sides 0 and 1, against
    /// it for 2 and 3.
    fn fraction(&self, surface: &NurbSurface3D, s: Scalar) -> GeopResult<Scalar> {
        let (lo, hi) = if self.along_u() {
            surface.domain_u()
        } else {
            surface.domain_v()
        };
        let from = if self.k < 2 { s.sub(lo) } else { hi.sub(s) };
        from.div(hi.sub(lo))
    }

    /// The patch's derivative across the side at `s`, into the patch.
    fn across(&self, surface: &NurbSurface3D, s: Scalar) -> GeopResult<Vector3> {
        let (u, v) = self.uv(surface, s);
        let (du, dv) = surface.derivatives(u, v)?;
        Ok(match self.k {
            0 => dv,
            1 => du.neg(),
            2 => dv.neg(),
            _ => du,
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::test_fixtures::{line, weighted};
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    use crate::{
        nurb_curve::{NurbCurve, NurbCurve3D},
        nurb_surface::{Neighbour, NurbSurface3D},
        shape::Plane,
    };

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    /// The quarter circle in the plane `x = x0` from `(x0, 0, 0)` round
    /// the center `(x0, 0, 1)` to `(x0, 1, 1)`: it leaves the plane `z = 0`
    /// along `y`.
    fn rising_arc(x0: f64) -> NurbCurve3D {
        let f = Scalar::from_f64;
        NurbCurve::try_new(
            2,
            vec![
                weighted(x0, 0., 0., 1.),
                weighted(x0, 1., 0., std::f64::consts::SQRT_2 / 2.0),
                weighted(x0, 1., 1., 1.),
            ],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// A cubic from the origin to `(1, 0, 0)`, with an interior knot,
    /// bending out of the plane `z = 0` by `lift`.
    fn wavy(lift: f64) -> NurbCurve3D {
        let f = Scalar::from_f64;
        NurbCurve::try_new(
            3,
            vec![
                weighted(0., 0., 0., 1.),
                weighted(0.3, -0.2, 0.4 * lift, 1.),
                weighted(0.6, 0.3, -0.2 * lift, 1.),
                weighted(0.8, -0.1, 0.3 * lift, 1.),
                weighted(1., 0., 0., 1.),
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

    fn assert_on(surface: &NurbSurface3D, curve: &NurbCurve3D, side: usize) {
        for k in 0..=8 {
            let s = Scalar::from_ratio(k, 8).unwrap();
            let (u, v) = match side {
                0 => (s, Scalar::ZERO),
                1 => (Scalar::ONE, s),
                2 => (Scalar::ONE.sub(s), Scalar::ONE),
                _ => (Scalar::ZERO, Scalar::ONE.sub(s)),
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
    #[test]
    fn coons_interpolates_its_sides() {
        let sides = [
            wavy(1.0),
            line([1., 0., 0.], [1., 1., 1.]),
            line([1., 1., 1.], [0., 1., 1.]),
            rising_arc(0.).reverse(),
        ];
        let surface = NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]]).unwrap();
        // The cubic, on the patch's `u` knots, is its first row.
        let nv = surface.num_v;
        for (i, cp) in wavy(1.0).control_points.iter().enumerate() {
            let row = surface.control_points[i * nv];
            assert!(row.could_be_equal(cp), "{i}: {row:?} vs {cp:?}");
        }
        // The arc, raised to the cubic's degree... is the first column,
        // evaluated: every side lies on the patch.
        for (k, side) in sides.iter().enumerate() {
            assert_on(&surface, side, k);
        }
    }

    /// Four lines of a square make the square itself, flat.
    #[test]
    fn coons_of_a_square_is_flat() {
        let c = [[0., 0., 0.], [2., 0., 0.], [2., 1., 0.], [0., 1., 0.]];
        let sides: Vec<_> = (0..4).map(|k| line(c[k], c[(k + 1) % 4])).collect();
        let surface = NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]]).unwrap();
        let plane = surface.as_plane().unwrap().expect("flat");
        assert!(plane.normal.could_be_equal(&v(0., 0., 1.)), "{plane:?}");
    }

    /// Sides that do not meet are refused.
    #[test]
    fn coons_needs_a_closed_loop() {
        let a = line([0., 0., 0.], [1., 0., 0.]);
        let b = line([1., 0., 0.], [1., 1., 0.]);
        let c = line([1., 1., 0.], [0., 1., 0.]);
        let d = line([0., 1.5, 0.], [0., 0., 0.]);
        assert!(NurbSurface3D::coons([&a, &b, &c, &d]).is_err());
    }

    /// The ruled surface between two curves runs along both.
    #[test]
    fn ruled_runs_along_both() {
        let a = wavy(1.0);
        let b = rising_arc(2.0);
        let surface = NurbSurface3D::ruled(&a, &b).unwrap();
        for k in 0..=8 {
            let s = Scalar::from_ratio(k, 8).unwrap();
            let pa = a.evaluate(s).unwrap();
            let pb = b.evaluate(s).unwrap();
            assert!(
                surface
                    .evaluate(s, Scalar::ZERO)
                    .unwrap()
                    .could_be_equal(&pa)
            );
            assert!(
                surface
                    .evaluate(s, Scalar::ONE)
                    .unwrap()
                    .could_be_equal(&pb)
            );
        }
    }

    /// A patch rising from a wavy curve in the plane `z = 0` between two
    /// quarter circles that leave it along `y`: made tangent to the plane
    /// along its bottom side, its normal there is the plane's, all along
    /// the side — and its sides are where they were.
    #[test]
    fn tangent_to_a_plane() {
        // The top bends up and down: the blend alone does not keep the
        // plane's tangent.
        let top = {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                3,
                vec![
                    weighted(0., 1., 1., 1.),
                    weighted(0.3, 1., 1.5, 1.),
                    weighted(0.6, 1., 0.6, 1.),
                    weighted(0.8, 1., 1.3, 1.),
                    weighted(1., 1., 1., 1.),
                ],
                [0., 0., 0., 0., 0.5, 1., 1., 1., 1.].map(f).to_vec(),
            )
            .unwrap()
            .reverse()
        };
        let sides = [wavy(0.0), rising_arc(1.), top, rising_arc(0.).reverse()];
        let coons = NurbSurface3D::coons([&sides[0], &sides[1], &sides[2], &sides[3]]).unwrap();
        let plane = Plane::try_new(v(0., 0., 0.), v(0., 0., 1.)).unwrap();
        let away = v(0., 1., 0.);
        let bends = (0..=8).any(|k| {
            let u = Scalar::from_ratio(k, 8).unwrap();
            let n = coons.normal(u, Scalar::ZERO).unwrap();
            !n.prod_cross(&v(0., 0., 1.))
                .norm_sq()
                .could_be_equal(Scalar::ZERO)
        });
        assert!(bends, "the Coons patch alone is not tangent to the plane");
        let normal = plane.normal;
        let at = |_: Scalar| Ok((normal, away));
        let flat = || {
            Some(Neighbour {
                at: &at,
                flat: true,
            })
        };
        let surface = coons.tangent_to(&[flat(), None, None, None]).unwrap();
        for (k, side) in sides.iter().enumerate() {
            assert_on(&surface, side, k);
        }
        for k in 0..=8 {
            let u = Scalar::from_ratio(k, 8).unwrap();
            let n = surface.normal(u, Scalar::ZERO).unwrap();
            assert!(
                n.prod_cross(&v(0., 0., 1.))
                    .norm_sq()
                    .could_be_equal(Scalar::ZERO),
                "at {u:?}: {n:?}"
            );
        }
        // Straight sides leaving the plane at an angle cannot be kept.
        let steep = [
            line([0., 0., 0.], [1., 0., 0.]),
            line([1., 0., 0.], [1., 1., 1.]),
            line([1., 1., 1.], [0., 1., 1.]),
            line([0., 1., 1.], [0., 0., 0.]),
        ];
        let coons = NurbSurface3D::coons([&steep[0], &steep[1], &steep[2], &steep[3]]).unwrap();
        assert!(coons.tangent_to(&[flat(), None, None, None]).is_err());
    }

    /// A bilinear patch through four corners, not in one plane, has them
    /// at the corners of its domain.
    #[test]
    fn bilinear_has_its_corners() {
        let corners = [
            v(0.3, -1.2, 2.5),
            v(4.1, 0.7, -0.3),
            v(2.2, 3.3, 1.1),
            v(-1.5, 2.0, 0.6),
        ];
        let surface =
            NurbSurface3D::bilinear(corners[0], corners[1], corners[2], corners[3]).unwrap();
        for (p, (u, w)) in corners.iter().zip([(0, 0), (1, 0), (1, 1), (0, 1)]) {
            let at = surface
                .evaluate(Scalar::from_i64(u), Scalar::from_i64(w))
                .unwrap();
            assert!(at.could_be_equal(p), "at ({u}, {w}): {at:?}, not {p:?}");
        }
    }
}
