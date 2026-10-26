//! The offset of a NURBS surface, as a NURBS surface that encloses it:
//! fitted, and widened by a bound on how far the fit is from the offset
//! that is proven over the whole domain, from the control points' hulls of
//! polynomials, not sampled. Every surface the same way: a plane, a
//! cylinder or a free-form blend.
//!
//! The offset `O = S + d N` keeps `S`'s parametrization. It is fitted as
//! `S` plus a shift `D` in `S`'s own rational basis (refined, raised to at
//! least cubic): `D`'s homogeneous control points interpolate `W d N`, `W`
//! the basis's weight, at the Greville points. `S + D` is then a NURBS
//! surface with `S`'s weights, its control points `S`'s moved by `D`'s.
//! Where the offset is itself such a surface — a plane's, a sphere's — the
//! fit is it, to the rounding.
//!
//! **The bound.** What the fit misses is `E = D − d N`. Over one span
//! `S = A / w` is a ratio of polynomials, `D = e / w`, and the normal is
//! `N = m / |m|` for the polynomial field `m` of `net::normal_field`, a
//! pole's factor divided out of it. The square root is the one thing that
//! is not polynomial, so it is fitted too: a polynomial `g ≈ 1 / |m|` over
//! each box. Then
//!
//! ```text
//! E = (e − d w m g) / w + d m g (1 − 1 / √r),    r = g² (m·m),
//! ```
//!
//! and `|m g| = √r`, so `|E| ≤ |F| / w + |d| |√r − 1|` with `F` the
//! polynomial `e − d w m g`. Every one of `F`, `w` and `r` is bounded over
//! the box by its Bernstein coefficients' hull. A box whose bound is wider
//! than [`OFFSET_DRIFT`] is split in four, and the hulls close in on the
//! polynomials' ranges as the square of the box's size. Every control
//! point is then widened by the largest bound: the basis is non-negative
//! and sums to one, so the surface is widened by exactly that everywhere.
//!
//! **Where the offset is regular.** Along a principal direction the offset
//! scales the surface by `1 − d κ`: regular where both factors are
//! positive, collapsed where one of them is not, anywhere, and folded where
//! that changes. In the tangent basis `S_u, S_v` the factors are the
//! eigenvalues of `I⁻¹ (I − d II)`, so regular means `I − d II` positive
//! definite, and so `G = w⁴ (I − d II)`, `G_ab = P_a·P_b − d (w² / |m|)
//! J_ab`, with the polynomials `P_a = w A_a − w_a A = w² S_a` and
//! `J_ab = (w A_ab − w_ab A)·m`: its first entry and its determinant
//! positive. With `1 / |m|` enclosed over the box by `g / √r`, each entry
//! is a polynomial with an interval factor, and its sign is its
//! coefficients': positive, negative, or undecided, and the box is split.
//! At a pole `P_t` along the collapsed side vanishes with the factor, and
//! the entries are divided by it, as `m` is.
//!
//! A box-by-box Taylor form over the surface's partials came first, and
//! took over a minute for a rounded box's corner patches: interval
//! derivatives of the normal over a box are as wide as their terms, a
//! hundred times the result. The hulls of polynomials have no such
//! dependency.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::{
    NurbSurface3D,
    net::{Net, Side, dot, normal_field, poles, product, scale},
};
use crate::{nurb_curve::solve_interpolation_system, spline::breakpoints};

/// How far a fitted offset may drift from the true offset, per coordinate,
/// before its spans are split finer, and how small each box's bound is
/// made (see the module docs). It bounds effort and how wide the offset
/// comes out, not whether it encloses the true one: what the fit misses is
/// padded on, however much that is.
const OFFSET_DRIFT: f64 = 1e-6;

/// The most pieces each span of the surface is split into for the fit:
/// the budget [`OFFSET_DRIFT`] is pursued with.
const MAX_OFFSET_PIECES: usize = 32;

/// The most boxes the bound is sought over, all spans together, before the
/// offset is refused as not enclosed.
const MAX_OFFSET_BOXES: usize = 1 << 14;

/// How often a span's box is split in four before whether the offset is
/// regular over it has to be answered: one still undecided then could
/// fold. Bounds effort, not the answer: a box over which the offset is
/// regular, or collapsed, everywhere is found so long before, unless a
/// factor `1 − d κ` comes within the box's width of zero.
const REGULAR_SPLITS: usize = 6;

/// How often a span's box is split in four while the bound on what the fit
/// misses there is wider than [`OFFSET_DRIFT`]; then it is taken as it is.
/// Bounds effort and how wide the offset may come out, not whether it
/// encloses the true one.
const BOUND_SPLITS: usize = 4;

/// The degree in each direction of the polynomial fitted to `1 / |m|` over
/// a box: its error falls as the box's size to this plus one.
const INVERSE_DEGREE: usize = 3;

/// The polynomials over one span of the surface and of a shift fitted onto
/// its offset (see the module docs): `S`'s weight `w`, the normal field
/// `m`, a pole's factor divided out, and the shift's point part `e`,
/// `D = e / w`.
struct Fit {
    w: Net<1>,
    m: Net<3>,
    e: Net<3>,
}

/// The polynomials over one span of the surface that tell where its offset
/// is regular: `P_a·P_b` and `J_ab` for `uu`, `uv`, `vv`, each divided by a
/// pole's factor as often as it vanishes there, with `w` and `m`.
struct Shape {
    w: Net<1>,
    m: Net<3>,
    first: [Net<1>; 3],
    second: [Net<1>; 3],
}

/// `net` divided `times` times by the factor vanishing along `side`; an
/// error if it does not vanish there.
fn divide<const D: usize>(net: &Net<D>, side: Side, times: usize) -> GeopResult<Net<D>> {
    let mut out = net.clone();
    for _ in 0..times {
        out = out.divided(side)?.ok_or_else(|| {
            GeopError::new(format!(
                "a polynomial of the surface does not vanish along its pole at {side:?}"
            ))
        })?;
    }
    Ok(out)
}

/// The point part and the weight of the Bézier patch `patch`, centred,
/// and its normal field with its poles' factors divided out, and those
/// poles.
fn parts(patch: &NurbSurface3D) -> GeopResult<(Net<3>, Net<1>, Net<3>, Vec<Side>)> {
    let (a, w) = Net::of(patch, true)?.split();
    let sides = poles(patch)?;
    let mut m = normal_field(patch)?;
    for &side in &sides {
        m = divide(&m, side, 1)?;
    }
    Ok((a, w, m, sides))
}

/// `g ≈ 1 / |m|` over the box of `m` (see [`inverse_root`]), and the hull
/// of `r = g² (m·m)`, definitely positive: `None` where either cannot be
/// had over the box.
fn inverse_length(m: &Net<3>) -> GeopResult<Option<(Net<1>, Scalar)>> {
    let squared = m.product(m, dot)?;
    let Some(g) = inverse_root(&squared)? else {
        return Ok(None);
    };
    let r = g.product(&g, product)?.product(&squared, product)?.hull()[0];
    Ok(r.definitely_greater(Scalar::ZERO).then_some((g, r)))
}

impl Fit {
    fn new(surface: &NurbSurface3D, shift: &NurbSurface3D) -> GeopResult<Self> {
        let (_, w, m, _) = parts(surface)?;
        let (e, _) = Net::of(shift, false)?.split();
        Ok(Fit { w, m, e })
    }

    /// What the fit misses over the box `u` × `v` (see the module docs),
    /// per coordinate: `|e − d w m g| / w + |d| |√r − 1|`. `None` where the
    /// box is too wide to bound it.
    fn missed(
        &self,
        distance: Scalar,
        u: (Scalar, Scalar),
        v: (Scalar, Scalar),
    ) -> GeopResult<Option<[Scalar; 3]>> {
        let w = self.w.restricted(u, v)?;
        let m = self.m.restricted(u, v)?;
        let w_range = w.hull()[0];
        let Some((g, r)) = inverse_length(&m)? else {
            return Ok(None);
        };
        if !w_range.definitely_greater(Scalar::ZERO) {
            return Ok(None);
        }
        let f = self
            .e
            .restricted(u, v)?
            .minus(
                &w.product(&m, scale)?
                    .product(&g, |x, y| x.prod_scalar(y[0]))?
                    .scaled(distance),
            )?
            .hull();
        let stray = distance.abs().mul(r.sqrt()?.sub(Scalar::ONE).abs());
        let mut pad = [Scalar::ZERO; 3];
        for k in 0..3 {
            pad[k] = f[k].abs().div(w_range)?.add(stray).upper();
        }
        Ok(Some(pad))
    }
}

impl Shape {
    fn new(patch: &NurbSurface3D) -> GeopResult<Self> {
        let (a, w, m, sides) = parts(patch)?;
        // `P_a = w A_a − w_a A` and `w A_ab − w_ab A`.
        let along = |t: bool| -> GeopResult<Net<3>> {
            w.product(&a.derivative(t)?, scale)?
                .minus(&w.derivative(t)?.product(&a, scale)?)
        };
        let (p_u, p_v) = (along(true)?, along(false)?);
        let across = |s: bool, t: bool| -> GeopResult<Net<1>> {
            let a_st = a.derivative(s)?.derivative(t)?;
            let w_st = w.derivative(s)?.derivative(t)?;
            w.product(&a_st, scale)?
                .minus(&w_st.product(&a, scale)?)?
                .product(&m, dot)
        };
        let mut first = [
            p_u.product(&p_u, dot)?,
            p_u.product(&p_v, dot)?,
            p_v.product(&p_v, dot)?,
        ];
        let mut second = [
            across(true, true)?,
            across(true, false)?,
            across(false, false)?,
        ];
        // Along a pole's side `P_t` vanishes and the tangent across it does
        // not: an entry with `t` twice has its factor twice, with `t` once,
        // once — in `I` and in `II` alike.
        for &side in &sides {
            let t = if side.u_fixed { 2 } else { 0 };
            first[t] = divide(&first[t], side, 2)?;
            second[t] = divide(&second[t], side, 2)?;
            first[1] = divide(&first[1], side, 1)?;
            second[1] = divide(&second[1], side, 1)?;
        }
        Ok(Shape {
            w,
            m,
            first,
            second,
        })
    }

    /// Whether the offset by `distance` is regular over the box `u` × `v`
    /// (see the module docs): `Some(true)` everywhere, `Some(false)`
    /// nowhere, `None` undecided.
    fn regular(
        &self,
        distance: Scalar,
        u: (Scalar, Scalar),
        v: (Scalar, Scalar),
    ) -> GeopResult<Option<bool>> {
        let w = self.w.restricted(u, v)?;
        let Some((g, r)) = inverse_length(&self.m.restricted(u, v)?)? else {
            return Ok(None);
        };
        // `G_ab = P_a·P_b − (d / √r) w² g J_ab`.
        let c = w.product(&w, product)?.product(&g, product)?;
        let factor = distance.div(r.sqrt()?)?;
        let entry = |k: usize| -> GeopResult<Net<1>> {
            let second = c.product(&self.second[k].restricted(u, v)?, product)?;
            self.first[k]
                .restricted(u, v)?
                .minus(&second.scaled(factor))
        };
        let (g11, g12, g22) = (entry(0)?, entry(1)?, entry(2)?);
        let det = g11
            .product(&g22, product)?
            .minus(&g12.product(&g12, product)?)?
            .hull()[0];
        let g11 = g11.hull()[0];
        Ok(
            if g11.definitely_greater(Scalar::ZERO) && det.definitely_greater(Scalar::ZERO) {
                Some(true)
            } else if g11.definitely_less(Scalar::ZERO) || det.definitely_less(Scalar::ZERO) {
                Some(false)
            } else {
                None
            },
        )
    }
}

/// The boxes of `surface`'s spans, each split in four until `decide` says
/// something of it — told, after `splits` splits, that it is the last
/// chance; what it said of each, in turn, to `take`. Refused where it says
/// nothing then, or past [`MAX_OFFSET_BOXES`] boxes, naming the box.
fn over_boxes<T, S>(
    surface: &NurbSurface3D,
    what: &str,
    splits: usize,
    mut prepare: impl FnMut(((Scalar, Scalar), (Scalar, Scalar))) -> GeopResult<S>,
    mut decide: impl FnMut(&S, (Scalar, Scalar), (Scalar, Scalar), bool) -> GeopResult<Option<T>>,
    mut take: impl FnMut(T, (Scalar, Scalar), (Scalar, Scalar)),
) -> GeopResult<()> {
    let bu = breakpoints(&surface.knot_vector_u, surface.degree_u, surface.num_u);
    let bv = breakpoints(&surface.knot_vector_v, surface.degree_v, surface.num_v);
    let half = Scalar::from_f64(0.5);
    let mut boxes = 0usize;
    for su in bu.windows(2) {
        for sv in bv.windows(2) {
            let span = ((su[0], su[1]), (sv[0], sv[1]));
            let piece = prepare(span)
                .map_err(|e| e.with_context(format!("the span u={:?}, v={:?}", span.0, span.1)))?;
            let mut stack = vec![(span.0, span.1, 0usize)];
            while let Some((u, v, depth)) = stack.pop() {
                boxes += 1;
                if boxes > MAX_OFFSET_BOXES {
                    return Err(GeopError::new(format!(
                        "{what} was not decided within {MAX_OFFSET_BOXES} boxes; the last one u={u:?}, v={v:?}"
                    )));
                }
                let last = depth >= splits;
                if let Some(said) = decide(&piece, u, v, last)
                    .map_err(|e| e.with_context(format!("{what} over u={u:?}, v={v:?}")))?
                {
                    take(said, u, v);
                    continue;
                }
                if last {
                    return Err(GeopError::new(format!(
                        "{what} could not be decided over u={u:?}, v={v:?}, split {splits} times"
                    )));
                }
                let (cu, cv) = (
                    Scalar::fraction(u.0, u.1, half),
                    Scalar::fraction(v.0, v.1, half),
                );
                for a in [(u.0, cu), (cu, u.1)] {
                    for b in [(v.0, cv), (cv, v.1)] {
                        stack.push((a, b, depth + 1));
                    }
                }
            }
        }
    }
    Ok(())
}

/// A polynomial of degree [`INVERSE_DEGREE`] over `squared`'s box that
/// interpolates `1 / √squared` at a grid of its points, its coefficients
/// sharp (a fit is a free choice; what it misses is bounded by the
/// caller). `None` where `squared` could be zero at one of them.
fn inverse_root(squared: &Net<1>) -> GeopResult<Option<Net<1>>> {
    let n = INVERSE_DEGREE;
    let grid = |(lo, hi): (Scalar, Scalar)| -> GeopResult<Vec<Scalar>> {
        (0..=n)
            .map(|k| match k {
                // The ends as they are: a fraction of one past the end
                // rounds out of the box.
                0 => Ok(lo),
                k if k == n => Ok(hi),
                k => Ok(Scalar::fraction(
                    lo,
                    hi,
                    Scalar::from_ratio(k as i64, n as i64)?,
                )),
            })
            .collect()
    };
    let knots = |(lo, hi): (Scalar, Scalar)| {
        let mut k = vec![lo; n + 1];
        k.extend(vec![hi; n + 1]);
        k
    };
    let (gu, gv) = (grid(squared.u)?, grid(squared.v)?);
    let (ku, kv) = (knots(squared.u), knots(squared.v));
    let mut rows = Vec::with_capacity(n + 1);
    for &u in &gu {
        let mut values = Vec::with_capacity(n + 1);
        for &v in &gv {
            let q = squared.value(u, v)?[0];
            if !q.definitely_greater(Scalar::ZERO) {
                return Ok(None);
            }
            values.push(Vector::from_array([Scalar::ONE.div(q.sqrt()?)?]));
        }
        rows.push(solve_interpolation_system(&gv, &kv, n, &values)?);
    }
    let mut c = vec![vec![Vector::zero(); n + 1]; n + 1];
    for j in 0..=n {
        let column: Vec<Vector<1>> = rows.iter().map(|row| row[j]).collect();
        for (i, x) in solve_interpolation_system(&gu, &ku, n, &column)?
            .into_iter()
            .enumerate()
        {
            c[i][j] = Vector::from_array([x[0].sharpen()]);
        }
    }
    Ok(Some(Net {
        c,
        u: squared.u,
        v: squared.v,
    }))
}

impl NurbSurface3D {
    /// The surface `distance` along its normal, fitted and widened to
    /// enclose it (see the module docs), with the same parametrization;
    /// `None` where it collapses (see [`NurbSurface3D::offset`]). Fails
    /// where the surface has no normal, or the offset folds.
    pub(super) fn fitted_offset(&self, distance: Scalar) -> GeopResult<Option<Self>> {
        let ctx = |e: GeopError| e.with_context(format!("fitted_offset(distance={distance:?})"));
        let mut base = self.clone();
        while base.degree_u < 3 {
            base = base.elevate_degree(true).map_err(ctx)?;
        }
        while base.degree_v < 3 {
            base = base.elevate_degree(false).map_err(ctx)?;
        }
        // Regular, collapsed or folded is the surface's and the distance's
        // to say, over its own spans, before anything is fitted.
        let (mut regular, mut collapsed) = (None, None);
        over_boxes(
            self,
            "whether the offset is regular",
            REGULAR_SPLITS,
            |(u, v)| Shape::new(&self.sub_surface(u, v)?),
            |shape, u, v, last| {
                let said = shape.regular(distance, u, v)?;
                if said.is_none() && last {
                    return Err(GeopError::new(format!(
                        "the offset by {distance:?} could fold over itself over u={u:?}, v={v:?}: a factor 1 − d κ could be zero there"
                    )));
                }
                Ok(said)
            },
            |said, u, v| {
                if said {
                    regular = Some((u, v));
                } else {
                    collapsed = Some((u, v));
                }
            },
        )
        .map_err(ctx)?;
        match (regular, collapsed) {
            (Some(at), Some(not)) => {
                return Err(ctx(GeopError::new(format!(
                    "the offset by {distance:?} folds over itself: it is regular over u={:?}, v={:?}, but passes a centre of curvature over u={:?}, v={:?}",
                    at.0, at.1, not.0, not.1
                ))));
            }
            (None, Some(_)) => return Ok(None),
            _ => {}
        }
        // How finely to fit is a matter of effort, so it is chosen from a
        // look at the fit between its points; whether the fit encloses the
        // offset is the bound's to say, over every box, after.
        let mut pieces = 1;
        let (net, shift) = loop {
            let net = base.refined(pieces).map_err(ctx)?;
            let shift = net.fitted_shift(self, distance).map_err(ctx)?;
            if pieces < MAX_OFFSET_PIECES && !shift.fits(self, distance).map_err(ctx)? {
                pieces *= 2;
                continue;
            }
            break (net, shift);
        };
        let drift = Scalar::from_f64(OFFSET_DRIFT);
        let mut pad = [Scalar::ZERO; 3];
        over_boxes(
            &shift,
            "the bound on what the fit misses",
            BOUND_SPLITS,
            |(u, v)| Fit::new(&self.sub_surface(u, v)?, &shift.sub_surface(u, v)?),
            // Split while the bound is wider than the drift, and take it
            // as it is once splitting no longer pays: it is honest either
            // way, only wider.
            |fit, u, v, last| {
                Ok(fit
                    .missed(distance, u, v)?
                    .filter(|p| last || p.iter().all(|k| !k.definitely_greater(drift))))
            },
            |missed, _, _| {
                for k in 0..3 {
                    pad[k] = pad[k].max(missed[k]);
                }
            },
        )
        .map_err(|e| ctx(e).with_context(format!("each span split into {pieces} for the fit")))?;
        Ok(Some(net.shifted(&shift, &pad)))
    }

    /// Whether this shift is within half [`OFFSET_DRIFT`] of `surface`'s offset
    /// by `distance` halfway between its Greville points, in the middle of
    /// each span: a look, to choose how finely to fit, not a bound.
    fn fits(&self, surface: &NurbSurface3D, distance: Scalar) -> GeopResult<bool> {
        let drift = Scalar::from_f64(OFFSET_DRIFT / 2.);
        let middles = |knots: &[Scalar], degree: usize, n: usize| -> GeopResult<Vec<Scalar>> {
            let b = breakpoints(knots, degree, n);
            b.windows(2)
                .flat_map(|w| [1, 2, 3].map(|k| (w[0], w[1], k)))
                .map(|(a, b, k)| Ok(Scalar::fraction(a, b, Scalar::from_ratio(k, 4)?)))
                .collect()
        };
        for u in middles(&self.knot_vector_u, self.degree_u, self.num_u)? {
            for v in middles(&self.knot_vector_v, self.degree_v, self.num_v)? {
                let e = self
                    .evaluate(u, v)?
                    .sub(&surface.normal(u, v)?.prod_scalar(distance));
                if (0..3).any(|k| e[k].abs().definitely_greater(drift)) {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    /// The shift that moves this surface, as a net to fit on, onto the
    /// offset of `surface` by `distance`: in this net's rational basis, its
    /// homogeneous control points interpolating `W d N` at the Greville
    /// points, `W` the net's weight there. Its control points are sharp: a
    /// fit is a free choice, what it misses is bounded afterwards.
    fn fitted_shift(&self, surface: &NurbSurface3D, distance: Scalar) -> GeopResult<Self> {
        let greville = |knots: &[Scalar], degree: usize, n: usize| -> GeopResult<Vec<Scalar>> {
            (0..n)
                .map(|i| {
                    let sum = knots[i + 1..=i + degree]
                        .iter()
                        .fold(Scalar::ZERO, |a, &k| a.add(k));
                    Ok(sum.div(Scalar::from_i64(degree as i64))?.sharpen())
                })
                .collect()
        };
        let (nu, nv) = (self.num_u, self.num_v);
        let gu = greville(&self.knot_vector_u, self.degree_u, nu)?;
        let gv = greville(&self.knot_vector_v, self.degree_v, nv)?;
        // Along `v` for each `u` of the grid, then along `u`.
        let mut rows = Vec::with_capacity(nu);
        for &u in &gu {
            let values = gv
                .iter()
                .map(|&v| {
                    let normal = surface.normal(u, v)?;
                    Ok(normal.prod_scalar(distance.mul(self.weight(u, v)?)))
                })
                .collect::<GeopResult<Vec<Vector3>>>()?;
            rows.push(solve_interpolation_system(
                &gv,
                &self.knot_vector_v,
                self.degree_v,
                &values,
            )?);
        }
        let mut shifts = vec![Vector3::zero(); nu * nv];
        for j in 0..nv {
            let column: Vec<Vector3> = rows.iter().map(|row| row[j]).collect();
            let solved =
                solve_interpolation_system(&gu, &self.knot_vector_u, self.degree_u, &column)?;
            for (i, s) in solved.into_iter().enumerate() {
                shifts[i * nv + j] = s;
            }
        }
        let control_points = shifts
            .iter()
            .zip(&self.control_points)
            .map(|(s, cp)| {
                Vector::from_array([s[0].sharpen(), s[1].sharpen(), s[2].sharpen(), cp[3]])
            })
            .collect();
        NurbSurface3D::try_new(
            self.degree_u,
            self.degree_v,
            control_points,
            self.knot_vector_u.clone(),
            self.knot_vector_v.clone(),
        )
    }

    /// This net moved by `shift` (homogeneous control points added, the
    /// weights kept) and widened by `pad` in each coordinate.
    fn shifted(&self, shift: &NurbSurface3D, pad: &[Scalar; 3]) -> Self {
        let control_points = self
            .control_points
            .iter()
            .zip(&shift.control_points)
            .map(|(cp, s)| {
                let w = cp[3];
                let mut out = *cp;
                for k in 0..3 {
                    let moved = cp[k].add(s[k]);
                    let d = pad[k].mul(w);
                    out[k] = moved.sub(d).union(moved.add(d));
                }
                out
            })
            .collect();
        let mut out = self.clone();
        out.control_points = control_points;
        out.recompute_aabb();
        out
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    use super::*;
    use crate::test_fixtures::{hom, knots};

    /// A bicubic bump over the unit square: corners down, middle up by
    /// `height`.
    fn bump(height: f64) -> NurbSurface3D {
        let z = |i: usize, j: usize| {
            let edge = |k: usize| k == 0 || k == 3;
            if edge(i) || edge(j) { 0. } else { height }
        };
        let control_points: Vec<Vector4> = (0..4)
            .flat_map(|i| (0..4).map(move |j| (i, j)))
            .map(|(i, j)| hom(i as f64 / 3., j as f64 / 3., z(i, j), 1.))
            .collect();
        let k = knots(&[0., 0., 0., 0., 1., 1., 1., 1.]);
        NurbSurface3D::try_new(3, 3, control_points, k.clone(), k).unwrap()
    }

    /// Whether `offset` encloses `surface` moved `distance` along its
    /// normal at a grid of `(u, v)` that is not the fit's, and is at most
    /// `width` wide.
    fn assert_encloses(surface: &NurbSurface3D, offset: &NurbSurface3D, distance: f64, width: f64) {
        for a in 0..=7 {
            for b in 0..=7 {
                let (u, v) = (
                    Scalar::from_ratio(a, 7).unwrap(),
                    Scalar::from_ratio(b, 7).unwrap(),
                );
                let expected = surface.evaluate(u, v).unwrap().add(
                    &surface
                        .normal(u, v)
                        .unwrap()
                        .prod_scalar(Scalar::from_f64(distance)),
                );
                let got = offset.evaluate(u, v).unwrap();
                assert!(
                    got.could_be_equal(&expected),
                    "at ({a}, {b}): {got:?}, expected {expected:?}"
                );
            }
        }
        for cp in &offset.control_points {
            for k in 0..3 {
                assert!(cp[k].width().to_f64() < width, "{cp:?}");
            }
        }
    }

    /// The fit alone, before it is bounded: its error off the Greville
    /// points falls as the spans are split finer: on a bump this steep,
    /// four to six times for each halving, still short of the sixteen of a
    /// cubic's asymptote.
    #[test]
    fn the_fit_converges() {
        let surface = bump(0.5);
        let distance = Scalar::from_f64(0.1);
        let error = |pieces: usize| {
            let net = surface.refined(pieces).unwrap();
            let shift = net.fitted_shift(&surface, distance).unwrap();
            let mut worst: f64 = 0.;
            for a in 0..=13 {
                for b in 0..=13 {
                    let (u, v) = (
                        Scalar::from_ratio(a, 13).unwrap(),
                        Scalar::from_ratio(b, 13).unwrap(),
                    );
                    let e = shift
                        .evaluate(u, v)
                        .unwrap()
                        .sub(&surface.normal(u, v).unwrap().prod_scalar(distance));
                    worst = (0..3).map(|c| e[c].abs().to_f64()).fold(worst, f64::max);
                }
            }
            worst
        };
        let errors: Vec<f64> = [1, 2, 4, 8].map(error).to_vec();
        assert!(errors.windows(2).all(|w| w[1] < w[0] / 3.), "{errors:?}");
    }

    /// A gentle bump, the fast case.
    #[test]
    fn a_free_form_bump_is_offset_both_ways() {
        let surface = bump(0.1);
        for d in [0.05, -0.05] {
            let offset = surface.offset(Scalar::from_f64(d)).unwrap().unwrap();
            assert_encloses(&surface, &offset, d, 5e-5);
        }
    }

    /// A steep bump: its normal turns fast, so its fit takes more spans
    /// and its bound more boxes, seconds of them.
    #[test]
    #[ignore = "slow: 7 s — run with `cargo test -- --ignored`; `a_free_form_bump_is_offset_both_ways` is the fast case"]
    fn a_steep_free_form_bump_is_offset_both_ways() {
        let surface = bump(0.5);
        for d in [0.1, -0.1] {
            let offset = surface.offset(Scalar::from_f64(d)).unwrap().unwrap();
            assert_encloses(&surface, &offset, d, 5e-5);
        }
    }

    /// A rational surface fitted, not taken exactly: a torus patch, whose
    /// offset the fit has to enclose like any other.
    #[test]
    #[ignore = "slow: 10 s — run with `cargo test -- --ignored`"]
    fn a_rational_patch_is_fitted() {
        let r2 = std::f64::consts::FRAC_1_SQRT_2;
        let ring = [(2., 0., 1.), (2., 2., r2), (0., 2., 1.)];
        let tube = [(1., 0., 1.), (1., 1., r2), (0., 1., 1.)];
        let control_points: Vec<Vector4> = ring
            .iter()
            .flat_map(|&(x, y, wr)| {
                tube.iter().map(move |&(r, z, wt)| {
                    let (x, y) = (x * (1. + r) / 2., y * (1. + r) / 2.);
                    let w = wr * wt;
                    hom(x * w, y * w, z * w, w)
                })
            })
            .collect();
        let k = knots(&[0., 0., 0., 1., 1., 1.]);
        let torus = NurbSurface3D::try_new(2, 2, control_points, k.clone(), k).unwrap();
        for d in [0.25, -0.25] {
            let fitted = torus.fitted_offset(Scalar::from_f64(d)).unwrap().unwrap();
            assert_encloses(&torus, &fitted, d, 5e-5);
        }
    }

    /// A patch with a pole: a sphere's octant, its rows meeting at the top.
    /// Its normal there is no `S_u × S_v`, which vanishes; the normal
    /// field with the pole's factor divided out is (see `net`). Fitted,
    /// not taken as a revolution.
    #[test]
    #[ignore = "slow: 8 s — run with `cargo test -- --ignored`"]
    fn a_patch_with_a_pole_is_fitted() {
        let sphere = crate::test_fixtures::sphere_octant();
        for d in [0.2, -0.2] {
            let fitted = sphere.fitted_offset(Scalar::from_f64(d)).unwrap().unwrap();
            assert_encloses(&sphere, &fitted, d, 5e-5);
        }
    }

    /// A sphere's octant offset inward past its centre: both of its
    /// curvatures pass, so `O_u × O_v` keeps its side, and only `I − d II`
    /// being negative definite tells it collapses.
    #[test]
    fn a_sphere_offset_past_its_centre_collapses() {
        let sphere = crate::test_fixtures::sphere_octant();
        assert!(
            sphere
                .fitted_offset(Scalar::from_f64(-1.5))
                .unwrap()
                .is_none()
        );
    }

    /// A bump bent tighter than the distance it is offset into: the offset
    /// passes a centre of curvature and folds over. Refused.
    #[test]
    fn an_offset_past_a_centre_of_curvature_is_refused() {
        let error = bump(2.).offset(Scalar::from_f64(-0.5)).unwrap_err();
        assert!(error.root_message().contains("fold"), "{error:?}");
    }
}
