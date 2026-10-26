//! The offset of any NURBS surface, as a NURBS surface that encloses it:
//! fitted, and widened by a bound on how far the fit is from the offset
//! that is proven over the whole domain, not sampled.
//!
//! The offset `O = S + d N` keeps `S`'s parametrization. It is fitted as
//! `S` plus a shift `D` in `S`'s own rational basis (refined, raised to at
//! least cubic): `D`'s homogeneous control points interpolate `W d N`, `W`
//! the basis's weight, at the Greville points. `S + D` is then a NURBS
//! surface with `S`'s weights, its control points `S`'s moved by `D`'s.
//!
//! What the fit misses, `E = D - d N`, is bounded box by box over each
//! span by its Taylor form to second order, `E(c) + ∇E(c) δ + ½ δᵀ H_E(B) δ`:
//! the value and gradient at the box's centre `c`, the Hessian over the box
//! `B`. The Hessian takes the normal's second derivatives, from the
//! normal field as a polynomial (see `net`), whose partials over a box
//! ([`NurbSurface3D::partials`]) are as tight as a box allows, and which a
//! pole's factor is divided out of: at a pole the parametrization says
//! nothing, the surface is fine. Of the bound, only the Hessian's part
//! overestimates, and it shrinks as the cube of the box's size; a box
//! where it is more than [`OFFSET_DRIFT`] is split. Every control point is
//! then widened by the largest bound: the basis is non-negative and sums
//! to one, so the surface is widened by exactly that everywhere, and
//! encloses the offset at every `(u, v)`.
//!
//! A first-order (mean value) form was tried and is not enough: its part
//! over the box shrinks only as the square of the box's size, and a bump
//! needed more than a million boxes to bound to a millionth.
//!
//! The same boxes tell whether the offset folds: past a centre of
//! curvature, `O_u × O_v` turns against `S`'s normal. That is refused.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::{
    NurbSurface3D,
    net::{Net, Side, cross, normal_field, poles, scale},
};
use crate::{nurb_curve::solve_interpolation_system, spline::breakpoints};

/// How far a fitted offset may drift from the true offset, per coordinate,
/// before its spans are split finer, and how small each box's overestimate
/// is made (see the module docs). It bounds effort and how wide the offset
/// comes out, not whether it encloses the true one: what the fit misses is
/// padded on, however much that is.
const OFFSET_DRIFT: f64 = 1e-5;

/// The most pieces each span of the surface is split into for the fit:
/// the budget [`OFFSET_DRIFT`] is pursued with.
const MAX_OFFSET_PIECES: usize = 32;

/// The most boxes the bound is sought over, all spans together, before the
/// offset is refused as not enclosed. How many it takes goes with the
/// surface's area in `(u, v)` over the box a bound settles at, not with how
/// finely it is fitted.
const MAX_OFFSET_BOXES: usize = 1 << 18;

/// What a fit misses, per coordinate: the bound over every box, which the
/// control points are widened by.
struct Missed {
    pad: [Scalar; 3],
}

/// The normal's first and second derivatives over a box or at a point,
/// from those of a field `n` in its direction (see [`normal_field`]).
struct Normals {
    n: Vector3,
    /// `[N_u, N_v]`.
    first: [Vector3; 2],
    /// `[N_uu, N_uv, N_vv]`.
    second: [Vector3; 3],
}

impl Normals {
    /// From the partials `m[k][l]` of a field `n` in the normal's direction
    /// up to second order (see [`NurbSurface3D::partials`]). With
    /// `L = |n|`, `N = n / L` and `α_a = N·n_a / L`: `N_a = n_a / L − α_a N`,
    /// and `N_ab = (n_ab − N (N·n_ab)) / L − α_b N_a − α_a N_b − (N_a·N_b) N`.
    /// Written in `N_a`, not expanded in `n_a`: the expanded terms are each
    /// a hundred times the result on a bump, and over a box their widths,
    /// not their values, are what is left of them.
    fn of(m: &[Vec<Vector3>]) -> GeopResult<Self> {
        let n = m[0][0];
        let n_a = [m[1][0], m[0][1]];
        let n_ab = [m[2][0], m[1][1], m[0][2]];
        let inverse = Scalar::ONE.div(n.norm())?;
        let unit = n.prod_scalar(inverse);
        // `α_a = N·n_a / L`, the rate `L` grows at.
        let alpha = [0, 1].map(|a| unit.prod_dot(&n_a[a]).mul(inverse));
        let first = [0, 1].map(|a| n_a[a].prod_scalar(inverse).sub(&unit.prod_scalar(alpha[a])));
        let second = [(0, 0, 0), (0, 1, 1), (1, 1, 2)].map(|(a, b, ab)| {
            n_ab[ab]
                .sub(&unit.prod_scalar(unit.prod_dot(&n_ab[ab])))
                .prod_scalar(inverse)
                .sub(&first[a].prod_scalar(alpha[b]))
                .sub(&first[b].prod_scalar(alpha[a]))
                .sub(&unit.prod_scalar(first[a].prod_dot(&first[b])))
        });
        Ok(Normals { n, first, second })
    }
}

/// Where a side of a span collapses to a pole, the offset's derivative
/// along that side divided by the factor that vanishes there (see
/// `Span::over_box`): `t = (w A_t − w_t A) / factor`, so `S_t / factor =
/// t / w²`, and `r = (n_t × n) / factor`, so `N_t / factor = (n × r) /
/// |n|³`.
struct PoleForm {
    side: Side,
    along_u: bool,
    t: NurbSurface3D,
    r: NurbSurface3D,
}

/// One span of a surface and of the shift fitted onto its offset, as what
/// the bound evaluates over its boxes.
struct Span {
    surface: NurbSurface3D,
    moved: NurbSurface3D,
    /// Where a side of the span is a pole, the normal field as a
    /// polynomial, the pole's factor divided out (see [`normal_field`]).
    /// Elsewhere `S_u × S_v` itself, from the surface's partials: of a
    /// third of the degree, it is the tighter over a box.
    normal: Option<NurbSurface3D>,
    poles: Vec<PoleForm>,
    u: (Scalar, Scalar),
    v: (Scalar, Scalar),
}

impl Span {
    fn new(surface: NurbSurface3D, moved: NurbSurface3D) -> GeopResult<Self> {
        let (a, w) = Net::of(&surface)?.split();
        let sides = poles(&surface)?;
        let mut field = normal_field(&surface)?;
        // Without a pole the field is only needed for the forms below.
        for &side in &sides {
            field = field.divided(side)?.ok_or_else(|| {
                GeopError::new(format!(
                    "the normal field does not vanish along the pole at {side:?}"
                ))
            })?;
        }
        let mut forms = Vec::new();
        for &side in &sides {
            let along_u = !side.u_fixed;
            let (a_t, w_t) = (a.derivative(along_u)?, w.derivative(along_u)?);
            let q = w.product(&a_t, scale)?.minus(&w_t.product(&a, scale)?);
            let p = field.derivative(along_u)?.product(&field, cross)?;
            let divided = |net: Net<3>| -> GeopResult<NurbSurface3D> {
                net.divided(side)?
                    .ok_or_else(|| {
                        GeopError::new(format!(
                            "a derivative along the pole at {side:?} does not vanish there"
                        ))
                    })?
                    .surface()
            };
            forms.push(PoleForm {
                side,
                along_u,
                t: divided(q)?,
                r: divided(p)?,
            });
        }
        Ok(Span {
            u: surface.domain_u(),
            v: surface.domain_v(),
            normal: if sides.is_empty() {
                None
            } else {
                Some(field.surface()?)
            },
            surface,
            moved,
            poles: forms,
        })
    }

    /// Over the box `u`, `v`, each with how far it reaches from its centre
    /// (`δu`, `δv`): the Hessian's part of the bound on what `moved` misses
    /// of `surface`'s offset by `distance`, `½ δᵀ H_E(B) δ` (see the module
    /// docs), where it is within [`OFFSET_DRIFT`] and the offset definitely
    /// does not fold over the box. Where the box is too wide to say — a
    /// normal or a weight that could be zero over it, a part too wide, a
    /// fold undecided — which way to split it: along `u` where its part
    /// is the larger, `None` where neither is.
    ///
    /// At a pole both `O_t` along the collapsed side and the factor
    /// vanish, so `O_t × O_s` does, and whether the offset folds is told
    /// by `O_t` divided by the factor (see [`PoleForm`]).
    fn over_box(
        &self,
        distance: Scalar,
        (u, du): (Scalar, Scalar),
        (v, dv): (Scalar, Scalar),
    ) -> Bounded {
        let attempt = || -> GeopResult<Bounded> {
            let s = self.surface.partials(u, v, 1)?;
            let m = self.field(u, v)?;
            let normals = Normals::of(&m)?;
            // Past a centre of curvature the offset turns against the
            // surface's normal.
            // The surface's tangents and the offset's, `O_t = S_t + d N_t`;
            // along a pole's side both divided by the factor vanishing there.
            let mut tangents = [s[1][0], s[0][1]];
            let mut o = [
                s[1][0].add(&normals.first[0].prod_scalar(distance)),
                s[0][1].add(&normals.first[1].prod_scalar(distance)),
            ];
            for form in self.poles.iter().filter(|f| self.touches(f.side, u, v)) {
                let w = self.surface.weight(u, v)?;
                let n = normals.n;
                let cube = n.norm().mul(n.norm_sq());
                let tangent = form
                    .t
                    .evaluate(u, v)?
                    .prod_scalar(Scalar::ONE.div(w.mul(w))?);
                let along = tangent.add(
                    &n.prod_cross(&form.r.evaluate(u, v)?)
                        .prod_scalar(distance.div(cube)?),
                );
                let k = if form.along_u { 0 } else { 1 };
                tangents[k] = tangent;
                o[k] = along;
            }
            // `O_t = (I − d W) S_t`: the offset is regular where both of
            // `I − d W`'s eigenvalues, `1 − d κ`, are positive — its
            // determinant and its trace, each over `|n|²`. The determinant
            // alone misses an offset past both centres of curvature, a
            // sphere's turned inside out.
            let n = normals.n;
            let facing = o[0].prod_cross(&o[1]).prod_dot(&n);
            let trace = o[0]
                .prod_cross(&tangents[1])
                .add(&tangents[0].prod_cross(&o[1]))
                .prod_dot(&n);
            if facing.definitely_less(Scalar::ZERO) || trace.definitely_less(Scalar::ZERO) {
                return Ok(Bounded::Folds);
            }
            let d = self.moved.partials(u, v, 2)?;
            let hessian = [d[2][0], d[1][1], d[0][2]];
            let e_ab: Vec<Vector3> = (0..3)
                .map(|i| hessian[i].sub(&normals.second[i].prod_scalar(distance)))
                .collect();
            let square = |t: Scalar| {
                let reach = t.abs().upper();
                Scalar::ZERO.union(reach.mul(reach))
            };
            let (uu, vv, uv) = (square(du), square(dv), du.mul(dv));
            let half = Scalar::from_f64(0.5);
            let drift = Scalar::from_f64(OFFSET_DRIFT);
            let (mut along_u, mut along_v) = (Scalar::ZERO, Scalar::ZERO);
            let mut curved = [Scalar::ZERO; 3];
            let mut within =
                facing.definitely_greater(Scalar::ZERO) && trace.definitely_greater(Scalar::ZERO);
            for k in 0..3 {
                let (pu, pv, puv) = (
                    e_ab[0][k].mul(uu).mul(half),
                    e_ab[2][k].mul(vv).mul(half),
                    e_ab[1][k].mul(uv),
                );
                along_u = Scalar::max(along_u, pu.abs().upper());
                along_v = Scalar::max(along_v, pv.abs().upper());
                curved[k] = pu.add(pv).add(puv);
                within &= !curved[k].abs().upper().definitely_greater(drift);
            }
            if within {
                return Ok(Bounded::Within(curved));
            }
            let why = format!(
                "facing {facing:?}, trace {trace:?}, the Hessian's part {curved:?}, the shift's second partials {hessian:?}, the normal's {:?}",
                normals.second,
            );
            Ok(Bounded::Split(
                if along_u.definitely_greater(along_v) {
                    Some(true)
                } else if along_v.definitely_greater(along_u) {
                    Some(false)
                } else {
                    None
                },
                why,
            ))
        };
        attempt().unwrap_or_else(|e| Bounded::Split(None, format!("{e:?}")))
    }

    /// The partials of a field in the normal's direction up to second
    /// order at `(u, v)`, a point or a box: `n = S_u × S_v` and its
    /// derivatives from the surface's partials up to third order, or, in a
    /// span with a pole, the polynomial field. Only there: of three times
    /// the degree, it is the wider over a box — but `S_u × S_v` near the
    /// pole is wider still, as small as the distance to it and divided by.
    fn field(&self, u: Scalar, v: Scalar) -> GeopResult<Vec<Vec<Vector3>>> {
        if let Some(normal) = &self.normal {
            return normal.partials(u, v, 2);
        }
        let s = self.surface.partials(u, v, 3)?;
        let (su, sv) = (s[1][0], s[0][1]);
        let (suu, suv, svv) = (s[2][0], s[1][1], s[0][2]);
        let (suuu, suuv, suvv, svvv) = (s[3][0], s[2][1], s[1][2], s[0][3]);
        let two = Scalar::TWO;
        Ok(vec![
            vec![
                su.prod_cross(&sv),
                suv.prod_cross(&sv).add(&su.prod_cross(&svv)),
                suvv.prod_cross(&sv)
                    .add(&suv.prod_cross(&svv).prod_scalar(two))
                    .add(&su.prod_cross(&svvv)),
            ],
            vec![
                suu.prod_cross(&sv).add(&su.prod_cross(&suv)),
                suuv.prod_cross(&sv)
                    .add(&suu.prod_cross(&svv))
                    .add(&su.prod_cross(&suvv)),
            ],
            vec![
                suuu.prod_cross(&sv)
                    .add(&suu.prod_cross(&suv).prod_scalar(two))
                    .add(&su.prod_cross(&suuv)),
            ],
        ])
    }

    /// Whether the box `u` × `v` reaches `side` of the span.
    fn touches(&self, side: Side, u: Scalar, v: Scalar) -> bool {
        let (t, (lo, hi)) = if side.u_fixed {
            (u, self.u)
        } else {
            (v, self.v)
        };
        if side.at_end {
            t.could_be_equal(hi)
        } else {
            t.could_be_equal(lo)
        }
    }
}

impl NurbSurface3D {
    /// The surface `distance` along its normal, fitted and widened to
    /// enclose it (see the module docs), with the same parametrization.
    /// Fails where the surface has no normal, or the offset folds.
    pub(super) fn fitted_offset(&self, distance: Scalar) -> GeopResult<Self> {
        let ctx = |e: GeopError| e.with_context(format!("fitted_offset(distance={distance:?})"));
        let mut base = self.clone();
        while base.degree_u < 3 {
            base = base.elevate_degree(true).map_err(ctx)?;
        }
        while base.degree_v < 3 {
            base = base.elevate_degree(false).map_err(ctx)?;
        }
        // How finely to fit is a matter of effort, so it is chosen from a
        // look at the fit between its points; whether the fit encloses the
        // offset is the bound's to say, over every box, after.
        let mut pieces = 1;
        loop {
            let net = base.refined(pieces).map_err(ctx)?;
            let shift = net.fitted_shift(self, distance).map_err(ctx)?;
            if pieces < MAX_OFFSET_PIECES && !shift.fits(self, distance).map_err(ctx)? {
                pieces *= 2;
                continue;
            }
            let missed = self.missed_by(&shift, distance).map_err(|e| {
                ctx(e).with_context(format!("each span split into {pieces} for the fit"))
            })?;
            return Ok(net.shifted(&shift, &missed.pad));
        }
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

    /// What `shift` misses of this surface's offset by `distance` — see the
    /// module docs — over each span of the shift, box by box. Refused where
    /// it takes more than [`MAX_OFFSET_BOXES`] boxes.
    fn missed_by(&self, shift: &NurbSurface3D, distance: Scalar) -> GeopResult<Missed> {
        let mut missed = Missed {
            pad: [Scalar::ZERO; 3],
        };
        let bu = breakpoints(&shift.knot_vector_u, shift.degree_u, shift.num_u);
        let bv = breakpoints(&shift.knot_vector_v, shift.degree_v, shift.num_v);
        let half = Scalar::from_f64(0.5);
        let mut boxes = 0usize;
        for su in bu.windows(2) {
            for sv in bv.windows(2) {
                // One polynomial piece each, so a box's derivatives are
                // that piece's over it.
                let span = ((su[0], su[1]), (sv[0], sv[1]));
                let piece = Span::new(
                    self.sub_surface(span.0, span.1)?,
                    shift.sub_surface(span.0, span.1)?,
                )?;
                let mut stack = vec![span];
                let mut undecided = String::new();
                while let Some(((u0, u1), (v0, v1))) = stack.pop() {
                    boxes += 1;
                    if boxes > MAX_OFFSET_BOXES {
                        return Err(GeopError::new(format!(
                            "the offset's error over the span u={:?}, v={:?} was not bounded within {MAX_OFFSET_BOXES} boxes; the last box split: {undecided}",
                            span.0, span.1
                        )));
                    }
                    let (cu, cv) = (
                        Scalar::fraction(u0, u1, half),
                        Scalar::fraction(v0, v1, half),
                    );
                    // At the centre, sharp: a surface with no normal there
                    // has no offset.
                    let at = piece.field(cu, cv)?;
                    let normals = Normals::of(&at).map_err(|e| {
                        e.with_context(format!(
                            "the surface has no normal at u={cu:?}, v={cv:?}: its offset is not supported there"
                        ))
                    })?;
                    let moved_at = piece.moved.partials(cu, cv, 1)?;
                    let unit = normals.n.normalize()?;
                    let e = moved_at[0][0].sub(&unit.prod_scalar(distance));
                    let e_u = moved_at[1][0].sub(&normals.first[0].prod_scalar(distance));
                    let e_v = moved_at[0][1].sub(&normals.first[1].prod_scalar(distance));
                    let (du, dv) = (u0.union(u1).sub(cu), v0.union(v1).sub(cv));
                    let split_u = match piece.over_box(
                        distance,
                        (u0.union(u1), du),
                        (v0.union(v1), dv),
                    ) {
                        Bounded::Within(curved) => {
                            for k in 0..3 {
                                let first = e[k].add(e_u[k].mul(du)).add(e_v[k].mul(dv));
                                missed.pad[k] =
                                    Scalar::max(missed.pad[k], first.add(curved[k]).abs().upper());
                            }
                            continue;
                        }
                        Bounded::Folds => {
                            return Err(GeopError::new(format!(
                                "the offset by {distance:?} folds over itself near u={cu:?}, v={cv:?}: it passes a centre of curvature of the surface"
                            )));
                        }
                        Bounded::Split(along_u, why) => {
                            undecided = format!("u=({u0:?}, {u1:?}), v=({v0:?}, {v1:?}): {why}");
                            along_u.unwrap_or_else(|| u1.sub(u0).definitely_greater(v1.sub(v0)))
                        }
                    };
                    if split_u {
                        stack.push(((u0, cu), (v0, v1)));
                        stack.push(((cu, u1), (v0, v1)));
                    } else {
                        stack.push(((u0, u1), (v0, cv)));
                        stack.push(((u0, u1), (cv, v1)));
                    }
                }
            }
        }
        Ok(missed)
    }
}

/// What a box of the bound on a fit's error came to (see
/// `NurbSurface3D::over_box`).
enum Bounded {
    /// The Hessian's part of the bound over the box, per coordinate.
    Within([Scalar; 3]),
    /// The offset definitely folds over the box.
    Folds,
    /// Undecided: to split along `u`, along `v`, or the longer side, and
    /// why.
    Split(Option<bool>, String),
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
            let fitted = torus.fitted_offset(Scalar::from_f64(d)).unwrap();
            assert_encloses(&torus, &fitted, d, 5e-5);
        }
    }

    /// The normal's derivatives (`Normals`) agree with the normal's
    /// differences across a small step.
    #[test]
    fn normal_derivatives_match_differences() {
        let surface = bump(0.5);
        let (u, v) = (0.3, 0.6);
        let h = 1e-4;
        let n = |a: f64, b: f64| {
            let n = surface
                .normal(Scalar::from_f64(a), Scalar::from_f64(b))
                .unwrap();
            [0, 1, 2].map(|c| n[c].to_f64())
        };
        let at = Normals::of(
            &normal_field(&surface)
                .unwrap()
                .surface()
                .unwrap()
                .partials(Scalar::from_f64(u), Scalar::from_f64(v), 2)
                .unwrap(),
        )
        .unwrap();
        let close = |got: &Vector3, expected: [f64; 3], what: &str| {
            for c in 0..3 {
                assert!(
                    (got[c].to_f64() - expected[c]).abs() < 1e-4,
                    "{what}: {got:?} vs {expected:?}"
                );
            }
        };
        let diff = |f: &dyn Fn(usize) -> f64| [0, 1, 2].map(f);
        let (p, m) = (n(u + h, v), n(u - h, v));
        let (q, r) = (n(u, v + h), n(u, v - h));
        let o = n(u, v);
        close(&at.first[0], diff(&|c| (p[c] - m[c]) / (2. * h)), "N_u");
        close(&at.first[1], diff(&|c| (q[c] - r[c]) / (2. * h)), "N_v");
        close(
            &at.second[0],
            diff(&|c| (p[c] - 2. * o[c] + m[c]) / (h * h)),
            "N_uu",
        );
        close(
            &at.second[2],
            diff(&|c| (q[c] - 2. * o[c] + r[c]) / (h * h)),
            "N_vv",
        );
        let (pp, pm, mp, mm) = (
            n(u + h, v + h),
            n(u + h, v - h),
            n(u - h, v + h),
            n(u - h, v - h),
        );
        close(
            &at.second[1],
            diff(&|c| (pp[c] - pm[c] - mp[c] + mm[c]) / (4. * h * h)),
            "N_uv",
        );
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
            let fitted = sphere.fitted_offset(Scalar::from_f64(d)).unwrap();
            assert_encloses(&sphere, &fitted, d, 5e-5);
        }
    }

    /// A sphere's octant offset inward past its centre: both of its
    /// curvatures pass, `O_u × O_v` keeps its side, and only the trace of
    /// `I − d W` tells it is turned inside out.
    #[test]
    fn a_sphere_offset_past_its_centre_folds() {
        let sphere = crate::test_fixtures::sphere_octant();
        let error = sphere.fitted_offset(Scalar::from_f64(-1.5)).unwrap_err();
        assert!(error.root_message().contains("folds"), "{error:?}");
    }

    /// A bump bent tighter than the distance it is offset into: the offset
    /// passes a centre of curvature and folds over. Refused.
    #[test]
    fn an_offset_past_a_centre_of_curvature_is_refused() {
        let error = bump(2.).offset(Scalar::from_f64(-0.5)).unwrap_err();
        assert!(error.root_message().contains("folds"), "{error:?}");
    }
}
