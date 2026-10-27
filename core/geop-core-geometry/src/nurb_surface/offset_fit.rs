//! The offset of a NURBS surface, as a NURBS surface that encloses it:
//! fitted, and widened by a bound on how far the fit is from the offset
//! that is proven over the whole domain, from the control points' hulls of
//! polynomials, not sampled. Every surface the same way: a plane, a
//! cylinder or a free-form blend.
//!
//! The offset `O = S + d N` keeps `S`'s parametrization. It is fitted as
//! `S` plus a shift `D` in `S`'s own rational basis — first as it is, and
//! where that cannot follow the offset, refined and raised to cubic: `D`'s
//! homogeneous control points interpolate `W d N`, `W` the basis's weight,
//! at the Greville points. `S + D` is then a NURBS surface with `S`'s
//! weights, its control points `S`'s moved by `D`'s. Where `W N` is itself
//! a polynomial of the basis — a plane's, a cylinder's, a cone's, a
//! sphere's, a torus's — the fit is the offset, to the rounding, and the
//! net keeps its degrees and its layout.
//!
//! **The bound.** What the fit misses is `E = D − d N`. Over one span
//! `S = A / w` is a ratio of polynomials, `D = e / w`, and the normal is
//! `N = m / |m|` for the polynomial field `m` of `net::normal_field`, a
//! pole's factor divided out of it. Split `D = a N + b` with `b ⟂ N`:
//!
//! ```text
//! |b|² = |e × m|² / (w² m·m),    a² − d² = (e·e − d² w²) / w² − |b|²,
//! ```
//!
//! and where `a` has the sign of `d` (`d (e·m) > 0`), `|a − d| = |a² − d²|
//! / |a + d| ≤ (|e·e − d² w²| / w² + |b|²) / |d|`. So `|E| ≤ |a − d| + |b|`
//! is bounded by polynomials alone: each ratio over a box by the largest
//! ratio of the Bernstein coefficients of its numerator and denominator,
//! the sign of `e·m` by its coefficients' hull. Where the fit is exact,
//! `e × m` and `e·e − d² w²` are zero as polynomials, and the bound is the
//! rounding. A box whose bound is wider than
//! [`OFFSET_DRIFT`] is split in four, and the hulls close in on the
//! polynomials' ranges as the square of the box's size. Every control
//! point is then widened by the largest bound over the boxes its basis
//! function covers: the basis is non-negative and sums to one, so the
//! surface is widened by at least the bound everywhere.
//!
//! **Where the offset is regular.** Along a principal direction the offset
//! scales the surface by `1 − d κ`: regular where both factors are
//! positive, collapsed where one of them is not, anywhere, and folded where
//! that changes. Where it is nowhere regular — collapsed, or a factor as
//! near zero as the boxes resolve, as offsetting a cylinder by its radius
//! makes it a line — the offset is no surface either way. In the tangent
//! basis `S_u, S_v` the factors are the eigenvalues of `I⁻¹ (I − d II)`, so regular means `I − d II` positive
//! definite, and so `G = w⁴ (I − d II)`, `G_ab = P_a·P_b − d (w² / |m|)
//! J_ab`, with the polynomials `P_a = w A_a − w_a A = w² S_a` and
//! `J_ab = (w A_ab − w_ab A)·m`: its first entry and its determinant
//! positive. `1 / |m|` is enclosed over a box by the hull of `m·m`, and
//! each entry is a polynomial with an interval factor, whose sign is its
//! coefficients': positive, negative, or undecided, and the box is split.
//!
//! At a pole `P_t` along the collapsed side vanishes with the factor, and
//! the entries are divided by it, as `m` is: twice where `J_tt` vanishes
//! twice too, a smooth pole like a sphere's, and once at an apex, a
//! cone's, where the curvature round it grows as one over the distance.
//! Along a side where the *offset* closes to a point (see `offset`), `G`
//! has a null vector, the side's own direction: there the entries that
//! vanish are taken by the mean value theorem, their derivative across the
//! side, and the determinant divided by the distance from it.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::{
    NurbSurface3D,
    net::{Net, Side, cross, dot, normal_field, poles, product, scale},
    offset::Closing,
};
use crate::{nurb_curve::solve_interpolation_system, spline::breakpoints};

/// How far a fitted offset may drift from the true offset before its spans
/// are split finer, and how small each box's bound is made (see the module
/// docs). It bounds effort and how wide the offset comes out, not whether
/// it encloses the true one: what the fit misses is padded on, however
/// much that is.
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

/// How many splits in a row that do not halve how far from zero a box's
/// entries could be leave whether the offset is regular there undecided:
/// effort, like [`REGULAR_SPLITS`]. One was too few: by a cone's closing
/// side the entries narrow slowly at first, then decide.
const STALLED_SPLITS: usize = 3;

/// The degree in each direction of the polynomial fitted to `1 / |m|` over
/// a box: its error falls as the box's size to this plus one.
const INVERSE_DEGREE: usize = 3;

/// How often a span's box is split in four while the bound on what the fit
/// misses there is wider than [`OFFSET_DRIFT`]; then it is taken as it is.
/// Bounds effort and how wide the offset may come out, not whether it
/// encloses the true one.
const BOUND_SPLITS: usize = 4;

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
/// pole's factor as often as both vanish there, with `w` and `m`; and what
/// `G_uv²` is multiplied by in the determinant where `G_tt` was divided by
/// an apex's factor once only, not twice.
struct Shape {
    w: Net<1>,
    m: Net<3>,
    first: [Net<1>; 3],
    second: [Net<1>; 3],
    twist: Option<Net<1>>,
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

/// The factor `t` (`1 − t`) that vanishes along `side`, over the box of
/// `like`, as a polynomial of degree one across it.
fn side_factor(side: Side, like: &Net<1>) -> Net<1> {
    let one = Vector::from_array([Scalar::ONE]);
    let zero = Vector::from_array([Scalar::ZERO]);
    let across = if side.at_end {
        vec![one, zero]
    } else {
        vec![zero, one]
    };
    let c = if side.u_fixed {
        across.into_iter().map(|x| vec![x]).collect()
    } else {
        vec![across]
    };
    Net {
        c,
        u: like.u,
        v: like.v,
    }
}

impl Fit {
    fn new(surface: &NurbSurface3D, shift: &NurbSurface3D) -> GeopResult<Self> {
        let (_, w, m, _) = parts(surface)?;
        let (e, _) = Net::of(shift, false)?.split();
        Ok(Fit { w, m, e })
    }

    /// What the fit misses over the box `u` × `v` (see the module docs):
    /// `|b| + (|e·e − d² w²| / w² + |b|²) / |d|`, with `|b|² = |e × m|² /
    /// (w² m·m)`. `None` where the box is too wide to bound it.
    fn missed(
        &self,
        distance: Scalar,
        u: (Scalar, Scalar),
        v: (Scalar, Scalar),
    ) -> GeopResult<Option<Scalar>> {
        let w = self.w.restricted(u, v)?;
        let m = self.m.restricted(u, v)?;
        let e = self.e.restricted(u, v)?;
        let along = e.product(&m, dot)?.hull()[0].mul(distance);
        if !along.definitely_greater(Scalar::ZERO) {
            return Ok(None);
        }
        let w_squared = w.product(&w, product)?;
        let across = e.product(&m, cross)?;
        let Some(b_squared) = ratio_bound(
            &across.product(&across, dot)?,
            &w_squared.product(&m.product(&m, dot)?, product)?,
        )?
        else {
            return Ok(None);
        };
        let Some(length) = ratio_bound(
            &e.product(&e, dot)?
                .minus(&w_squared.scaled(distance.mul(distance)))?,
            &w_squared,
        )?
        else {
            return Ok(None);
        };
        let b = b_squared.sqrt()?;
        let along = length.add(b_squared).div(distance.abs())?;
        Ok(Some(b.add(along).upper()))
    }
}

/// A bound on `|num| / den` over their box, for `den` positive there: the
/// largest ratio of their Bernstein coefficients at a common degree, as
/// `Σ B_i a_i ≤ max_i (a_i / d_i) Σ B_i d_i` for positive `d_i`. Unlike the
/// ratio of their hulls, it does not take the numerator's largest value
/// over the denominator's least where the two are far apart, as across a
/// span of weights that fall off. `None` where a coefficient of `den` is
/// not definitely positive.
fn ratio_bound(num: &Net<1>, den: &Net<1>) -> GeopResult<Option<Scalar>> {
    let (p, q) = (
        num.c.len().max(den.c.len()) - 1,
        num.c[0].len().max(den.c[0].len()) - 1,
    );
    let (num, den) = (num.raised(p, q)?, den.raised(p, q)?);
    let mut bound = Scalar::ZERO;
    for (a, d) in num.c.iter().flatten().zip(den.c.iter().flatten()) {
        if !d[0].definitely_greater(Scalar::ZERO) {
            return Ok(None);
        }
        bound = bound.max(a[0].abs().upper().div(d[0])?.upper());
    }
    Ok(Some(bound))
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
        // not: `P_t·P_t` has its factor twice, `P_t·P_s` once. `J_tt` has it
        // twice at a smooth pole, once at an apex; `J_ts` once.
        let mut twist: Option<Net<1>> = None;
        for &side in &sides {
            let t = if side.u_fixed { 2 } else { 0 };
            let once = divide(&second[t], side, 1)?;
            let (twice, times) = match once.divided(side)? {
                Some(twice) => (twice, 2),
                None => (once, 1),
            };
            second[t] = twice;
            first[t] = divide(&first[t], side, times)?;
            second[1] = second[1].divided(side)?.ok_or_else(|| {
                GeopError::new(format!(
                    "the surface twists at its apex at {side:?}: its offset is not regular there"
                ))
            })?;
            first[1] = divide(&first[1], side, 1)?;
            if times == 1 {
                // `det G / t = G'_tt G_ss − t G'_ts²`.
                let factor = side_factor(side, &first[1]);
                twist = Some(match twist {
                    Some(f) => f.product(&factor, product)?,
                    None => factor,
                });
            }
        }
        Ok(Shape {
            w,
            m,
            first,
            second,
            twist,
        })
    }

    /// Whether the offset by `distance` is regular over the box `u` × `v`
    /// (see the module docs): `Some(true)` everywhere, `Some(false)`
    /// nowhere, `None` undecided — and how far from zero the entries it
    /// decides by could be there, the larger of their hulls' magnitudes. `closes` is the side of the span where
    /// the offset closes to a point, if it does there; `flipped`, whether
    /// the span lies past an apex the surface was carried on through (see
    /// `offset`).
    fn regular(
        &self,
        distance: Scalar,
        u: (Scalar, Scalar),
        v: (Scalar, Scalar),
        closes: Option<Side>,
        flipped: bool,
    ) -> GeopResult<(Option<bool>, f64)> {
        let w = self.w.restricted(u, v)?;
        let m = self.m.restricted(u, v)?;
        // The entries with `1 / |m|` as a polynomial `g / √r` fitted over
        // the box (see [`inverse_length`]).
        let Some((fitted, r)) = inverse_length(&m)? else {
            return Ok((None, f64::INFINITY));
        };
        let factor = distance.div(r.sqrt()?)?;
        let c = w.product(&w, product)?.product(&fitted, product)?;
        let g = (0..3)
            .map(|i| {
                self.first[i].restricted(u, v)?.minus(
                    &c.product(&self.second[i].restricted(u, v)?, product)?
                        .scaled(factor),
                )
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let twist = self
            .twist
            .as_ref()
            .map(|t| t.restricted(u, v))
            .transpose()?;
        let (t, s) = match closes {
            Some(side) if side.u_fixed => (2, 0),
            _ => (0, 2),
        };
        // Past an apex the offset keeps its way round where the surface
        // turns over: one factor `1 − d κ` changes sign with it there.
        let verdict = |diagonal: Scalar, det: Scalar| {
            let det = if flipped { det.neg() } else { det };
            let size = diagonal
                .abs()
                .upper()
                .to_f64()
                .max(det.abs().upper().to_f64());
            let said = if diagonal.definitely_greater(Scalar::ZERO)
                && det.definitely_greater(Scalar::ZERO)
            {
                Some(true)
            } else if diagonal.definitely_less(Scalar::ZERO) || det.definitely_less(Scalar::ZERO) {
                Some(false)
            } else {
                None
            };
            (said, size)
        };
        let squared = g[1].product(&g[1], product)?;
        let squared = match &twist {
            Some(t) => squared.product(t, product)?,
            None => squared,
        };
        let det = g[0].product(&g[2], product)?.minus(&squared)?.hull()[0];
        let said = verdict(g[s].hull()[0], det);
        let Some(side) = closes.filter(|_| said.0.is_none()) else {
            return Ok(said);
        };
        // The offset's row along `side` is a point: `G e_t = 0` there, `t`
        // the side's own direction. So by the mean value theorem `G_tt = x
        // G'_tt` and `G_ts = x G'_ts`, `x` the distance across from the
        // side (negative from the end), the primes enclosing the
        // derivatives across over the box carried on to the side, and
        // `det G / x = G'_tt G_ss − G'_ts G_ts`. Undecided as its entries
        // are near the side, the box is decided so.
        let across = side.u_fixed;
        let (eu, ev) = match (side.u_fixed, side.at_end) {
            (true, false) => ((self.w.u.0, u.1), v),
            (true, true) => ((u.0, self.w.u.1), v),
            (false, false) => (u, (self.w.v.0, v.1)),
            (false, true) => (u, (v.0, self.w.v.1)),
        };
        let w = self.w.restricted(eu, ev)?;
        let m = self.m.restricted(eu, ev)?;
        // `G_ab = P_a·P_b − d ρ K_ab`, `K_ab = w² g J_ab`, `ρ = 1 / √r`
        // and `r = g² (m·m)` near one, so `∂ρ = −ρ³ ∂r / 2` is small.
        let Some((fitted, _)) = inverse_length(&m)? else {
            return Ok((None, f64::INFINITY));
        };
        let r = fitted
            .product(&fitted, product)?
            .product(&m.product(&m, dot)?, product)?;
        let rho = Scalar::ONE.div(r.hull()[0].sqrt()?)?;
        let drho = rho
            .mul(rho)
            .mul(rho)
            .mul(r.derivative(across)?.hull()[0])
            .div(Scalar::TWO)?
            .neg();
        let c = w.product(&w, product)?.product(&fitted, product)?;
        let derivative = |i: usize| -> GeopResult<Scalar> {
            let k = c.product(&self.second[i].restricted(eu, ev)?, product)?;
            Ok(self.first[i]
                .restricted(eu, ev)?
                .derivative(across)?
                .minus(&k.derivative(across)?.scaled(distance.mul(rho)))?
                .minus(&k.scaled(distance.mul(drho)))?
                .hull()[0])
        };
        let x = if side.at_end {
            Scalar::ONE.neg()
        } else {
            Scalar::ONE
        };
        let ts = g[1].hull()[0];
        let ts = match &twist {
            Some(f) => ts.mul(f.hull()[0]),
            None => ts,
        };
        let det = x.mul(
            derivative(t)?
                .mul(g[s].hull()[0])
                .sub(derivative(1)?.mul(ts)),
        );
        Ok(verdict(g[s].hull()[0], det))
    }
}

/// What [`over_boxes`]' `decide` says of a box: something to take, or to
/// split it, telling its quarters what it found there.
enum Said<T, W> {
    Is(T),
    Split(W),
}

/// The boxes of `surface`'s spans, each split in four until `decide` says
/// something of it — told, after `splits` splits, that it is the last
/// chance, and what it found of the box it was split from; what it said of
/// each, in turn, to `take`. Refused where it says nothing then, or past
/// [`MAX_OFFSET_BOXES`] boxes, naming the box.
fn over_boxes<T, S, W: Clone>(
    surface: &NurbSurface3D,
    what: &str,
    splits: usize,
    mut prepare: impl FnMut(((Scalar, Scalar), (Scalar, Scalar))) -> GeopResult<S>,
    mut decide: impl FnMut(
        &S,
        (Scalar, Scalar),
        (Scalar, Scalar),
        bool,
        Option<&W>,
    ) -> GeopResult<Said<T, W>>,
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
            let mut stack: Vec<(_, _, usize, Option<W>)> = vec![(span.0, span.1, 0usize, None)];
            while let Some((u, v, depth, parent)) = stack.pop() {
                boxes += 1;
                if boxes > MAX_OFFSET_BOXES {
                    return Err(GeopError::new(format!(
                        "{what} was not decided within {MAX_OFFSET_BOXES} boxes; the last one u={u:?}, v={v:?}"
                    )));
                }
                let last = depth >= splits;
                let found = match decide(&piece, u, v, last, parent.as_ref())
                    .map_err(|e| e.with_context(format!("{what} over u={u:?}, v={v:?}")))?
                {
                    Said::Is(said) => {
                        take(said, u, v);
                        continue;
                    }
                    Said::Split(found) => found,
                };
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
                        stack.push((a, b, depth + 1, Some(found.clone())));
                    }
                }
            }
        }
    }
    Ok(())
}

/// A surface offset, and how: `distance` along its normal, the normal
/// continued through the apex `closing` names, if it has one inside (see
/// [`Closing`]).
struct Offsetting<'a> {
    surface: &'a NurbSurface3D,
    distance: Scalar,
    closing: Option<&'a Closing>,
}

impl Offsetting<'_> {
    /// Whether the parameter across the closing side, `x`, lies past the
    /// apex inside, on the closing side's side of it: where the surface
    /// is its own continuation through the apex, its normal the other way.
    fn flipped(&self, u: Scalar, v: Scalar) -> bool {
        let Some(Closing {
            side,
            apex: Some(apex),
        }) = self.closing
        else {
            return false;
        };
        let x = if side.u_fixed { u } else { v };
        if side.at_end {
            x.definitely_greater(*apex)
        } else {
            x.definitely_less(*apex)
        }
    }

    /// The distance to offset by over the box `u` × `v`, inside one span:
    /// against the surface's own normal past an apex inside.
    fn distance_over(&self, u: (Scalar, Scalar), v: (Scalar, Scalar)) -> GeopResult<Scalar> {
        Ok(if self.flipped_over(u, v)? {
            self.distance.neg()
        } else {
            self.distance
        })
    }

    /// Whether the box `u` × `v`, inside one span, lies past an apex inside.
    fn flipped_over(&self, u: (Scalar, Scalar), v: (Scalar, Scalar)) -> GeopResult<bool> {
        let mid = |(a, b): (Scalar, Scalar)| a.add(b).div(Scalar::TWO);
        Ok(self.flipped(mid(u)?, mid(v)?))
    }

    /// The shift `d N` at `(u, v)`, the normal continued through an apex
    /// inside: at the apex itself its limit along the parameter line, from
    /// the side that is not flipped.
    fn shift(&self, u: Scalar, v: Scalar) -> GeopResult<Vector3> {
        if let Some(Closing {
            side,
            apex: Some(apex),
        }) = self.closing
        {
            let x = if side.u_fixed { u } else { v };
            if x.could_be_equal(*apex) {
                // Next to a collapsed `v` row `S_u ≈ (v − v₀) S_uv`, next to
                // a collapsed `u` row `S_v ≈ (u − u₀) S_uv`.
                let s = self.surface.partials(u, v, 2)?;
                let limit = if side.u_fixed {
                    s[1][0].prod_cross(&s[1][1])
                } else {
                    s[1][1].prod_cross(&s[0][1])
                };
                // The side not flipped is after the apex for a closing
                // side at the start.
                let limit = if side.at_end { limit.neg() } else { limit };
                return Ok(limit.normalize()?.prod_scalar(self.distance));
            }
        }
        let normal = self.surface.normal(u, v)?;
        Ok(normal.prod_scalar(if self.flipped(u, v) {
            self.distance.neg()
        } else {
            self.distance
        }))
    }

    /// The side the span `u` × `v` touches where the offset closes.
    fn closes_over(&self, u: (Scalar, Scalar), v: (Scalar, Scalar)) -> Option<Side> {
        let side = self.closing?.side;
        let (x, (lo, hi)) = if side.u_fixed {
            (u, self.surface.domain_u())
        } else {
            (v, self.surface.domain_v())
        };
        let touches = if side.at_end {
            x.1.could_be_equal(hi)
        } else {
            x.0.could_be_equal(lo)
        };
        touches.then_some(side)
    }
}

impl NurbSurface3D {
    /// Whether the surface has a normal everywhere: its normal field, a
    /// pole's factor divided out, definitely not zero over every span, each
    /// split in four up to [`REGULAR_SPLITS`] times. `false` where it
    /// vanishes, or could: a patch carried on so far that it folds back on
    /// itself, its offset no surface.
    pub fn has_normal_everywhere(&self) -> GeopResult<bool> {
        let mut everywhere = true;
        over_boxes(
            self,
            "whether the surface has a normal",
            REGULAR_SPLITS,
            |(u, v)| Ok(parts(&self.sub_surface(u, v)?)?.2),
            |m, u, v, last, _: Option<&()>| {
                let m = m.restricted(u, v)?;
                let squared = m.product(&m, dot)?.hull()[0];
                Ok(if squared.definitely_greater(Scalar::ZERO) {
                    Said::Is(true)
                } else if last {
                    Said::Is(false)
                } else {
                    Said::Split(())
                })
            },
            |has, _, _| everywhere &= has,
        )?;
        Ok(everywhere)
    }

    /// The surface `distance` along its normal, fitted and widened to
    /// enclose it (see the module docs), with the same parametrization;
    /// `None` where it collapses (see [`NurbSurface3D::offset`]). Fails
    /// where the surface has no normal, or the offset folds. `closing`
    /// says where the surface was moved so that its offset closes at an
    /// apex.
    pub(super) fn fitted_offset(
        &self,
        distance: Scalar,
        closing: Option<&Closing>,
    ) -> GeopResult<Option<Self>> {
        let ctx = |e: GeopError| e.with_context(format!("fitted_offset(distance={distance:?})"));
        let offsetting = Offsetting {
            surface: self,
            distance,
            closing,
        };
        // Regular, collapsed or folded is the surface's and the distance's
        // to say, over its own spans, before anything is fitted.
        let (mut regular, mut collapsed, mut undecided) = (None, None, None);
        over_boxes(
            self,
            "whether the offset is regular",
            REGULAR_SPLITS,
            |(u, v)| {
                Ok((
                    Shape::new(&self.sub_surface(u, v)?)?,
                    offsetting.distance_over(u, v)?,
                    offsetting.flipped_over(u, v)?,
                    offsetting.closes_over(u, v),
                ))
            },
            |(shape, distance, flipped, closes), u, v, last, parent: Option<&(f64, usize)>| {
                let (said, size) = shape.regular(*distance, u, v, *closes, *flipped)?;
                // How many splits in a row have not halved how far from zero
                // the box's entries could be.
                let stalled = match parent {
                    Some(&(p, n)) if p <= 2.0 * size => n + 1,
                    _ => 0,
                };
                Ok(match said {
                    Some(said) => Said::Is(Some(said)),
                    // Undecided at the last split, or where splitting the
                    // box has stopped narrowing how far from zero its
                    // entries could be, [`STALLED_SPLITS`] times in a row: a
                    // factor `1 − d κ` as near zero as the arithmetic
                    // resolves, as a cylinder's offset by its radius has
                    // all over, which no split decides.
                    None if last || stalled >= STALLED_SPLITS => Said::Is(None),
                    None => Said::Split((size, stalled)),
                })
            },
            |said, u, v| match said {
                Some(true) => regular = Some((u, v)),
                Some(false) => collapsed = Some((u, v)),
                None => undecided = Some((u, v)),
            },
        )
        .map_err(ctx)?;
        match (regular, collapsed, undecided) {
            (Some(at), Some(not), _) => {
                return Err(ctx(GeopError::new(format!(
                    "the offset by {distance:?} folds over itself: it is regular over u={:?}, v={:?}, but passes a centre of curvature over u={:?}, v={:?}",
                    at.0, at.1, not.0, not.1
                ))));
            }
            (Some(at), None, Some((u, v))) => {
                return Err(ctx(GeopError::new(format!(
                    "the offset by {distance:?} could fold over itself over u={u:?}, v={v:?}: a factor 1 − d κ could be zero there, and it is regular over u={:?}, v={:?}",
                    at.0, at.1
                ))));
            }
            // Nowhere regular: collapsed, or as near a curve as can be told.
            (None, ..) => return Ok(None),
            _ => {}
        }
        // How finely to fit is a matter of effort, so it is chosen from a
        // look at the fit between its points; whether the fit encloses the
        // offset is the bound's to say, over every box, after. The
        // surface's own net first: where it can follow the offset, the
        // offset keeps its degrees and its layout.
        let own = self.fitted_shift(&offsetting).map_err(ctx)?;
        let (net, shift) = if own.fits(&offsetting).map_err(ctx)? {
            (self.clone(), own)
        } else {
            let mut base = self.clone();
            while base.degree_u < 3 {
                base = base.elevate_degree(true).map_err(ctx)?;
            }
            while base.degree_v < 3 {
                base = base.elevate_degree(false).map_err(ctx)?;
            }
            let mut pieces = 1;
            loop {
                let net = base.refined([pieces, pieces]).map_err(ctx)?;
                let shift = net.fitted_shift(&offsetting).map_err(ctx)?;
                if pieces < MAX_OFFSET_PIECES && !shift.fits(&offsetting).map_err(ctx)? {
                    pieces *= 2;
                    continue;
                }
                break (net, shift);
            }
        };
        let drift = Scalar::from_f64(OFFSET_DRIFT);
        let mut bounds = Vec::new();
        over_boxes(
            &shift,
            "the bound on what the fit misses",
            BOUND_SPLITS,
            |(u, v)| {
                Ok((
                    Fit::new(&self.sub_surface(u, v)?, &shift.sub_surface(u, v)?)?,
                    offsetting.distance_over(u, v)?,
                ))
            },
            // Split while the bound is wider than the drift, and take it
            // as it is once splitting no longer pays: it is honest either
            // way, only wider.
            |(fit, distance), u, v, last, _: Option<&()>| {
                Ok(
                    match fit
                        .missed(*distance, u, v)?
                        .filter(|p| last || !p.definitely_greater(drift))
                    {
                        Some(missed) => Said::Is(missed),
                        None => Said::Split(()),
                    },
                )
            },
            |missed, u, v| bounds.push((u, v, missed)),
        )
        .map_err(|e| ctx(e).with_context(format!("the shift fitted on {net:?}")))?;
        let offset = net.shifted(&shift, &bounds);
        if let Some(closing) = closing
            && !poles(&offset)?.contains(&closing.side)
        {
            return Err(ctx(GeopError::new(format!(
                "the offset of the apex does not close in a point along its side {:?}: the offsets of the surface round it do not meet in one point",
                closing.side
            ))));
        }
        Ok(Some(offset))
    }

    /// Whether this shift is within half [`OFFSET_DRIFT`] of the offset
    /// halfway between its Greville points, in the middle of each span: a
    /// look, to choose how finely to fit, not a bound.
    fn fits(&self, offsetting: &Offsetting) -> GeopResult<bool> {
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
                let e = self.evaluate(u, v)?.sub(&offsetting.shift(u, v)?);
                if (0..3).any(|k| e[k].abs().definitely_greater(drift)) {
                    return Ok(false);
                }
            }
        }
        Ok(true)
    }

    /// The shift that moves this surface, as a net to fit on, onto the
    /// offset: in this net's rational basis, its homogeneous control points
    /// interpolating `W d N` at the Greville points, `W` the net's weight
    /// there. Its control points are sharp: a fit is a free choice, what it
    /// misses is bounded afterwards.
    fn fitted_shift(&self, offsetting: &Offsetting) -> GeopResult<Self> {
        let greville = |knots: &[Scalar], degree: usize, n: usize| -> GeopResult<Vec<Scalar>> {
            (0..n)
                .map(|i| {
                    let sum = knots[i + 1..=i + degree]
                        .iter()
                        .fold(Scalar::ZERO, |a, &k| a.add(k));
                    // Inside the domain: a division by the degree rounds
                    // out of it at an end.
                    let mean = sum.div(Scalar::from_i64(degree as i64))?.sharpen();
                    Ok(mean.max(knots[degree]).min(knots[n]))
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
                    let shift = offsetting.shift(u, v).map_err(|e| {
                        e.with_context(format!("the shift at the Greville point {:?}", (u, v)))
                    })?;
                    Ok(shift.prod_scalar(self.weight(u, v)?))
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
    /// weights kept) and widened in each coordinate by the largest of the
    /// `bounds` over the boxes its basis function is not zero on: wherever
    /// the surface is evaluated, every control point it is made of is then
    /// widened by at least the bound there, and the basis sums to one. A
    /// span carried on far, its weights small, widens only the points it
    /// is made of.
    #[allow(clippy::type_complexity)]
    fn shifted(
        &self,
        shift: &NurbSurface3D,
        bounds: &[((Scalar, Scalar), (Scalar, Scalar), Scalar)],
    ) -> Self {
        let support = |knots: &[Scalar], degree: usize, i: usize, (a, b): (Scalar, Scalar)| {
            knots[i].definitely_less(b) && a.definitely_less(knots[i + degree + 1])
        };
        let control_points = self
            .control_points
            .iter()
            .zip(&shift.control_points)
            .enumerate()
            .map(|(index, (cp, s))| {
                let (i, j) = (index / self.num_v, index % self.num_v);
                let pad = bounds
                    .iter()
                    .filter(|(u, v, _)| {
                        support(&self.knot_vector_u, self.degree_u, i, *u)
                            && support(&self.knot_vector_v, self.degree_v, j, *v)
                    })
                    .fold(Scalar::ZERO, |pad, (_, _, missed)| pad.max(*missed));
                let w = cp[3];
                let mut out = *cp;
                let d = pad.mul(w);
                for k in 0..3 {
                    let moved = cp[k].add(s[k]);
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
            let net = surface.refined([pieces, pieces]).unwrap();
            let offsetting = Offsetting {
                surface: &surface,
                distance,
                closing: None,
            };
            let shift = net.fitted_shift(&offsetting).unwrap();
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
            let fitted = torus
                .fitted_offset(Scalar::from_f64(d), None)
                .unwrap()
                .unwrap();
            assert_encloses(&torus, &fitted, d, 5e-5);
        }
    }

    /// A patch with a pole: a sphere's octant, its rows meeting at the top.
    /// Its normal there is no `S_u × S_v`, which vanishes; the normal
    /// field with the pole's factor divided out is (see `net`). Fitted,
    /// not taken as a revolution.
    #[test]
    fn a_patch_with_a_pole_is_fitted() {
        let sphere = crate::test_fixtures::sphere_octant();
        for d in [0.2, -0.2] {
            let fitted = sphere
                .fitted_offset(Scalar::from_f64(d), None)
                .unwrap()
                .unwrap();
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
                .fitted_offset(Scalar::from_f64(-1.5), None)
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
