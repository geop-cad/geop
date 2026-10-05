//! Curve–curve intersection by per-axis fat line clipping — the design is
//! `curve_curve.md` next to this file; it reuses the clip of
//! `contains/curve.md` and the search schedule of `contains/surface.md`.
//!
//! For curves `A(s) = H_A / W_A` and `B(t) = H_B / W_B` with positive
//! weights, `A_k(s) = B_k(t)` iff
//! `g_k(s, t) = H_{A,k}(s) W_B(t) - W_A(s) H_{B,k}(t) = 0`, a polynomial
//! tensor-product spline in the *independent* parameters `s`, `t` with
//! coefficients `d_ij = P_{i,k} Q_{j,w} - P_{i,w} Q_{j,k}` — no division, no
//! degree elevation. Its zeros are clipped in both directions
//! ([`clip_tensor`]), exactly as a surface's are in `contains::surface`.
//!
//! Assumes no coincident arcs (`curve_curve.md`): the result is a list of
//! paired `(s, t)` boxes, and reaching some count of them means nothing.

use std::collections::VecDeque;

use super::{
    Intersections,
    coincidence::{self, Hit, Overlap},
};
use crate::nurb_surface::clamp;
use crate::{
    aabb::{aabb_could_overlap, curve_could_meet_aabb},
    contains::curve::curve_could_contain,
    fat_line::{
        Stalled, carried_width, chord, clip_tensor, extent, greville_abscissae, restriction,
        spanning, stalled,
    },
    knot_insertion::pinned_clamped_end,
    nurb_curve::{NurbCurve, ParameterRefinable, dehomogenize},
};
use geop_core_math::{
    disjoint_set::DisjointSet,
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::Vector,
};

/// Clip the pair: a box `[ŝ, t̂]` inside both domains enclosing every
/// `(s, t)` with `A(s) = B(t)`, or `None` if some equation proves there is
/// none. `NurbCurve::try_new` guarantees positive weights, which the cross
/// multiplication needs.
///
/// The equations are combinations `g_n = n · (H_A W_B - W_A H_B)` along
/// free-choice directions ([`spanning`]): the `C - 1` directions
/// perpendicular to `B`'s chord (nearly independent of `t`, so they pin `s`)
/// and those perpendicular to `A`'s (pinning `t`). Together they span
/// space, so a pair satisfying all of them is a solution. If they don't —
/// a closed segment without a chord, or two collinear curves — the
/// coordinate axes are added.
fn clip<S: Scalar, const D: usize, const C: usize>(
    a: &NurbCurve<S, D>,
    b: &NurbCurve<S, D>,
) -> GeopResult<Option<[S; 2]>> {
    let mut hats = [a.domain_as_scalar(), b.domain_as_scalar()];
    let (na, nb) = (a.control_points.len(), b.control_points.len());
    let greville = [
        greville_abscissae(&a.knot_vector, a.degree, na)?,
        greville_abscissae(&b.knot_vector, b.degree, nb)?,
    ];

    let mut dirs = chord::<S, D, C>(&b.control_points)
        .orthonormal_complement()
        .unwrap_or_default();
    dirs.extend(
        chord::<S, D, C>(&a.control_points)
            .orthonormal_complement()
            .unwrap_or_default(),
    );

    let w = D - 1;
    let mut d = Vec::with_capacity(na * nb);
    for n in spanning(dirs) {
        let nb_pts: Vec<S> = b
            .control_points
            .iter()
            .map(|q| n.prod_dot(&q.head()))
            .collect();
        d.clear();
        for p in &a.control_points {
            let np = n.prod_dot(&p.head());
            d.extend(
                b.control_points
                    .iter()
                    .zip(&nb_pts)
                    .map(|(q, &nq)| np.mul(q[w]).sub(p[w].mul(nq))),
            );
        }
        if !clip_tensor(&d, &[na, nb], &greville, &mut hats) {
            return Ok(None);
        }
    }
    Ok(Some(hats))
}

/// If the clip pinned `s` (`t`) exactly onto a clamped end of its curve,
/// every solution has that curve's *endpoint* as its spatial point, so the
/// rest is point containment in the other curve (the "boundary evaluation"
/// of `contains/surface.md` §3). Returns the endpoint and whether it is `A`'s.
fn pinned_endpoint<S: Scalar, const D: usize, const C: usize>(
    a: &NurbCurve<S, D>,
    b: &NurbCurve<S, D>,
    hats: [S; 2],
) -> Option<(Vector<S, C>, bool)> {
    let endpoint = |curve: &NurbCurve<S, D>, first: bool| {
        let cp = if first {
            curve.control_points[0]
        } else {
            curve.control_points[curve.control_points.len() - 1]
        };
        dehomogenize::<S, D, C>(&[cp])[0]
    };
    let pinned = |curve: &NurbCurve<S, D>, hat: S| {
        pinned_clamped_end(
            hat,
            &curve.knot_vector,
            curve.control_points.len(),
            curve.degree,
        )
    };
    if let Some(first) = pinned(a, hats[0]) {
        return Some((endpoint(a, first), true));
    }
    if let Some(first) = pinned(b, hats[1]) {
        return Some((endpoint(b, first), false));
    }
    None
}

/// All `(s, t)` with `curve_a(s) = curve_b(t)`, as paired parameter boxes
/// (`curve_curve.md`), for curves that do **not** overlap along an arc — see
/// [`curve_curve_intersect`] for the wrapper that handles overlaps.
/// Breadth-first over pairs of subcurves:
///
/// - the cached AABBs and the [`clip`] are necessary conditions — failing
///   either rejects the pair;
/// - if one parameter is pinned onto a clamped end, the other curve is
///   searched for that endpoint with [`curve_could_contain`];
/// - while the clip keeps shrinking the pair, both curves are restricted to
///   it ([`crate::fat_line::restriction`]). Rebuilding the coefficients
///   after a restriction is what couples the directions: clipping `t`
///   shrinks the rows unioned for `s`;
/// - once clipping stalls ([`crate::fat_line::stalled`]), a pair whose
///   segments' extents are both within `min_subdivision_size` has converged
///   and reports its segments' domains — a *candidate*, not an existence
///   proof; any other has one curve bisected. `min_subdivision_size` thus
///   bounds only bisection, never a pair clipping is still shrinking.
///
/// Boxes that overlap are merged by union into one unresolved cluster
/// (`DisjointSet`), never averaged. Exhausting `max_nodes` is an error —
/// the result would be incomplete, and neither "empty" nor "coincident" may
/// be read into it.
pub fn curve_curve_crossings<S: Scalar, const D: usize, const C: usize>(
    curve_a: &NurbCurve<S, D>,
    curve_b: &NurbCurve<S, D>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Vec<(S, S)>>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    let mut queue: VecDeque<(NurbCurve<S, D>, NurbCurve<S, D>)> = VecDeque::new();
    queue.push_back((curve_a.clone(), curve_b.clone()));
    let mut explored = 0usize;
    let mut solutions: DisjointSet<(S, S)> = DisjointSet::new();

    while let Some((a, b)) = queue.pop_front() {
        if explored >= max_nodes {
            return Err(GeopError::new(format!(
                "curve_curve_crossings: exhausted max_nodes={max_nodes} with {} \
                 pairs pending; the result would be incomplete",
                queue.len() + 1
            )));
        }
        explored += 1;

        if !aabb_could_overlap(&a.aabb, &b.aabb, C) {
            continue;
        }
        let Some(hats) = clip::<S, D, C>(&a, &b)? else {
            continue;
        };

        if let Some((point, on_a)) = pinned_endpoint::<S, D, C>(&a, &b, hats) {
            let (other, free) = if on_a { (&b, hats[1]) } else { (&a, hats[0]) };
            let budget = max_nodes - explored;
            if let Some(t) = curve_could_contain(other, &point, budget, min_subdivision_size)? {
                if t.could_be_equal(free) {
                    let free = free.intersect(t);
                    solutions.insert(if on_a {
                        (hats[0], free)
                    } else {
                        (free, hats[1])
                    });
                }
            }
            continue;
        }

        let ranges = [a.domain(), b.domain()];
        if let Some(bounds) = restriction(&hats, &ranges)? {
            if let (Ok(ra), Ok(rb)) = (
                a.sub_curve(bounds[0].0, bounds[0].1),
                b.sub_curve(bounds[1].0, bounds[1].1),
            ) {
                queue.push_back((ra, rb));
                continue;
            }
        }

        let sizes = [
            extent([a.control_points.clone()]),
            extent([b.control_points.clone()]),
        ];
        let carried = carried_width(&a.control_points).max(carried_width(&b.control_points));
        let order = match stalled(&ranges, &sizes, carried, min_subdivision_size) {
            Stalled::Converged => {
                // The pieces' whole domains, not the tighter clip: pieces the
                // search could not separate — a tangency converges on a chain
                // of adjacent ones — must merge into one unresolved cluster,
                // and adjacent domains share an endpoint where clips need not
                // touch. A transversal crossing loses nothing: restriction has
                // already cut its pieces down to the clip.
                solutions.insert((a.domain_as_scalar(), b.domain_as_scalar()));
                continue;
            }
            Stalled::Bisect(order) => order,
        };
        let children = order.iter().find_map(|&dir| {
            if dir == 0 {
                let (l, r) = a.split_mid().ok()?;
                Some([(l, b.clone()), (r, b.clone())])
            } else {
                let (l, r) = b.split_mid().ok()?;
                Some([(a.clone(), l), (a.clone(), r)])
            }
        });
        match children {
            Some(children) => queue.extend(children),
            // Nothing left to cut or split: what's here is the candidate.
            None => solutions.insert((hats[0], hats[1])),
        }
    }

    Ok(solutions.into_vec())
}

/// Every stretch of `a` lying on `b` (see [`coincidence`]), with
/// `b`'s parameter as the partner: candidates are `a`'s ends found on `b`
/// and `b`'s ends found on `a`.
pub(crate) fn curve_curve_overlaps<S: Scalar, const D: usize, const C: usize>(
    a: &NurbCurve<S, D>,
    b: &NurbCurve<S, D>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Vec<Overlap<S, S>>>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    let on = |curve: &NurbCurve<S, D>, other: &NurbCurve<S, D>, t: S| -> GeopResult<Option<S>> {
        let point = other.evaluate_cartesian(t)?;
        curve_could_contain(curve, &point, max_nodes, min_subdivision_size)
    };
    let (a0, a1) = a.domain();
    let (b0, b1) = b.domain();
    let mut candidates = Vec::new();
    for s in [a0, a1] {
        if let Some(t) = on(b, a, s)? {
            candidates.push(Hit { t: s, partner: t });
        }
    }
    for t in [b0, b1] {
        if let Some(s) = on(a, b, t)? {
            candidates.push(Hit { t: s, partner: t });
        }
    }
    coincidence::find_overlaps(candidates, |s| on(b, a, s))
}

/// Overlaps of `a` with `b`, and the isolated crossings away from them (the
/// clipping search run on each stretch of `a` between overlaps).
pub fn curve_curve_overlaps_and_crossings<S: Scalar, const D: usize, const C: usize>(
    a: &NurbCurve<S, D>,
    b: &NurbCurve<S, D>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<(Vec<Overlap<S, S>>, Vec<(S, S)>)>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    let overlaps = curve_curve_overlaps(a, b, max_nodes, min_subdivision_size)?;
    if overlaps.is_empty() {
        let crossings = curve_curve_crossings(a, b, max_nodes, min_subdivision_size)?;
        return Ok((overlaps, crossings));
    }
    let mut crossings = Vec::new();
    for (lo, hi) in coincidence::gaps(a.domain(), &overlaps) {
        let piece = a.sub_curve(lo, hi)?;
        crossings.extend(curve_curve_crossings(
            &piece,
            b,
            max_nodes,
            min_subdivision_size,
        )?);
    }
    Ok((overlaps, crossings))
}

/// Points where `curve_a` crosses — or, overlapping along an arc, coincides
/// with — `curve_b`, under the [`Intersections`] contract.
///
/// Overlaps are found directly ([`curve_curve_overlaps`]: ends of each curve
/// located on the other, then one midpoint probe per candidate stretch)
/// instead of being inferred from a search hitting `max_solutions`. Then:
///
/// - no overlap: [`Intersections::Found`] with the clipping search's
///   crossings, at most `max_solutions`;
/// - an overlap: [`Intersections::Coincident`] with, in this order and up to
///   `max_solutions` in total, the overlaps' end points, the isolated
///   crossings on the rest of `curve_a`, and points spread evenly over the
///   overlaps.
///
/// Exhausting `max_nodes` in any sub-search is an error.
pub fn curve_curve_intersect<S: Scalar, const D: usize, const C: usize>(
    curve_a: &NurbCurve<S, D>,
    curve_b: &NurbCurve<S, D>,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Intersections<(S, S)>>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    // Disjoint bounding boxes rule out crossings and overlaps alike, before
    // any candidate probe runs — a straight segment's own extent, not its
    // box (see `curve_could_meet_aabb`).
    if max_solutions == 0
        || !curve_could_meet_aabb(curve_a, &curve_b.aabb, C)
        || !curve_could_meet_aabb(curve_b, &curve_a.aabb, C)
    {
        return Ok(Intersections::Found(vec![]));
    }
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "curve_curve_intersect(curve_a={curve_a:?}, curve_b={curve_b:?}, max_nodes={max_nodes}, \
             min_subdivision_size={min_subdivision_size:?})"
        ))
    };
    let (overlaps, crossings) =
        curve_curve_overlaps_and_crossings(curve_a, curve_b, max_nodes, min_subdivision_size)
            .with_context(&ctx)?;
    let samples = coincidence::samples(&overlaps, max_solutions, |s| {
        let point = curve_a.evaluate_cartesian(s)?;
        curve_could_contain(curve_b, &point, max_nodes, min_subdivision_size)
    })?;
    Ok(coincidence::assemble(
        &overlaps,
        crossings,
        samples,
        max_solutions,
    ))
}

// The Krawczyk-verified refinement below (`TangentialDeflation`,
// `plain_krawczyk_step`, and a Krawczyk-based `refine_crossing`) is
// disabled for now — wiring it into `refine_crossing` regressed the
// `geop-ops-booleans` remesh test suite's runtime (more iterations and
// extra `evaluate`/`tangent`/`second_derivative` calls per refinement, in a
// hot path called for every candidate edge/edge and edge/face crossing).
// The math itself (`geop_core_math::interval_newton`, `geop_core_math::matrix`)
// and the `Intersections` enum are unaffected and stay in active use; only
// this file's *use* of Krawczyk for polishing a crossing is reverted to the
// original plain (unverified) Gauss-Newton below. Kept here, commented out,
// rather than deleted, in case it's worth revisiting with a cheaper
// convergence check.
//
// use geop_core_math::interval_newton::{KrawczykStep, gauss_newton_krawczyk_step};
//
// /// Cross-product tangential deflation for [`refine_crossing`] — see
// /// `geop_core_math::interval_newton`'s own doc comment for the Krawczyk math
// /// this feeds into, and the module-level rationale for why a tangential
// /// contact (parallel tangents, Cauchy-Schwarz equality) leaves the plain
// /// system's Jacobian `J = [A'(s), -B'(t)]` rank-deficient: deflating to
// /// `G = [A(s)-B(t); A'(s)×B'(t)]` recovers full rank generically, without
// /// adding a search dimension.
// ///
// /// Only 3-D curves (`C = 3`) have a cross product to build `G` from; 2-D
// /// pcurves (`C = 2`) get a no-op impl below — [`refine_crossing`]'s
// /// unconditional "never returns a worse enclosure than it was given"
// /// guarantee still holds there, it just can't rescue a tangential contact
// /// the same way a 3-D one can (same as any other refinement stall).
// pub trait TangentialDeflation<S: Scalar>: Sized {
//     /// One Gauss-Newton-Krawczyk step on the deflated system, evaluated at
//     /// `x_hat` (sharp point) and enclosed over `x_box`, or `None` if
//     /// deflation isn't available here (2-D) or is itself singular (contact
//     /// of higher order than this deflates for).
//     fn deflated_krawczyk_step(
//         &self,
//         other: &Self,
//         x_hat: Vector<S, 2>,
//         x_box: Vector<S, 2>,
//     ) -> Option<KrawczykStep<S>>;
// }
//
// impl<S: Scalar> TangentialDeflation<S> for NurbCurve<S, 4> {
//     fn deflated_krawczyk_step(
//         &self,
//         other: &Self,
//         x_hat: Vector<S, 2>,
//         x_box: Vector<S, 2>,
//     ) -> Option<KrawczykStep<S>> {
//         let pa = self.evaluate(x_hat[0]).ok()?;
//         let pb = other.evaluate(x_hat[1]).ok()?;
//         let ta = self.tangent(x_hat[0]).ok()?;
//         let tb = other.tangent(x_hat[1]).ok()?;
//         let saa = self.second_derivative(x_hat[0]).ok()?;
//         let sbb = other.second_derivative(x_hat[1]).ok()?;
//
//         let ta_box = self.tangent(x_box[0]).ok()?;
//         let tb_box = other.tangent(x_box[1]).ok()?;
//         let saa_box = self.second_derivative(x_box[0]).ok()?;
//         let sbb_box = other.second_derivative(x_box[1]).ok()?;
//
//         let f = pa.sub(&pb);
//         let cross_hat = ta.prod_cross(&tb);
//         let f_hat = Vector::from_array([f[0], f[1], f[2], cross_hat[0], cross_hat[1], cross_hat[2]]);
//
//         // ∂G/∂s = [A'(s); A''(s)×B'(t)], ∂G/∂t = [-B'(t); A'(s)×B''(t)].
//         let tb_neg = tb.neg();
//         let d_cross_ds_hat = saa.prod_cross(&tb);
//         let d_cross_dt_hat = ta.prod_cross(&sbb);
//         let jac_hat = Matrix::from_rows([
//             [ta[0], tb_neg[0]],
//             [ta[1], tb_neg[1]],
//             [ta[2], tb_neg[2]],
//             [d_cross_ds_hat[0], d_cross_dt_hat[0]],
//             [d_cross_ds_hat[1], d_cross_dt_hat[1]],
//             [d_cross_ds_hat[2], d_cross_dt_hat[2]],
//         ]);
//
//         let tb_box_neg = tb_box.neg();
//         let d_cross_ds_box = saa_box.prod_cross(&tb_box);
//         let d_cross_dt_box = ta_box.prod_cross(&sbb_box);
//         let jac_box = Matrix::from_rows([
//             [ta_box[0], tb_box_neg[0]],
//             [ta_box[1], tb_box_neg[1]],
//             [ta_box[2], tb_box_neg[2]],
//             [d_cross_ds_box[0], d_cross_dt_box[0]],
//             [d_cross_ds_box[1], d_cross_dt_box[1]],
//             [d_cross_ds_box[2], d_cross_dt_box[2]],
//         ]);
//
//         gauss_newton_krawczyk_step(x_hat, f_hat, jac_hat, x_box, jac_box).ok()
//     }
// }
//
// impl<S: Scalar> TangentialDeflation<S> for NurbCurve<S, 3> {
//     fn deflated_krawczyk_step(
//         &self,
//         _other: &Self,
//         _x_hat: Vector<S, 2>,
//         _x_box: Vector<S, 2>,
//     ) -> Option<KrawczykStep<S>> {
//         None
//     }
// }
//
// /// One [`gauss_newton_krawczyk_step`] on the plain system
// /// `F(s, t) = A(s) - B(t) ∈ Rᶜ`, generic over `C` via [`ParameterRefinable`].
// fn plain_krawczyk_step<S: Scalar, const D: usize, const C: usize>(
//     curve_a: &NurbCurve<S, D>,
//     curve_b: &NurbCurve<S, D>,
//     x_hat: Vector<S, 2>,
//     x_box: Vector<S, 2>,
// ) -> Option<KrawczykStep<S>>
// where
//     NurbCurve<S, D>: ParameterRefinable<S, C>,
// {
//     let pa = curve_a.evaluate_cartesian(x_hat[0]).ok()?;
//     let pb = curve_b.evaluate_cartesian(x_hat[1]).ok()?;
//     let da = curve_a.tangent_cartesian(x_hat[0]).ok()?;
//     let db = curve_b.tangent_cartesian(x_hat[1]).ok()?;
//     let da_box = curve_a.tangent_cartesian(x_box[0]).ok()?;
//     let db_box = curve_b.tangent_cartesian(x_box[1]).ok()?;
//
//     let f_hat = pa.sub(&pb);
//     let db_neg = db.neg();
//     let mut jac_hat = Matrix::<S, C, 2>::zero();
//     let db_box_neg = db_box.neg();
//     let mut jac_box = Matrix::<S, C, 2>::zero();
//     for c in 0..C {
//         jac_hat[(c, 0)] = da[c];
//         jac_hat[(c, 1)] = db_neg[c];
//         jac_box[(c, 0)] = da_box[c];
//         jac_box[(c, 1)] = db_box_neg[c];
//     }
//
//     gauss_newton_krawczyk_step(x_hat, f_hat, jac_hat, x_box, jac_box).ok()
// }
//
// /// Krawczyk-verified version of `refine_crossing` -- see the module comment
// /// above for why this is currently disabled.
// pub fn refine_crossing_krawczyk<S: Scalar, const D: usize, const C: usize>(
//     curve_a: &NurbCurve<S, D>,
//     curve_b: &NurbCurve<S, D>,
//     t_a: S,
//     t_b: S,
// ) -> (S, S)
// where
//     NurbCurve<S, D>: ParameterRefinable<S, C> + TangentialDeflation<S>,
// {
//     let (a_lo, a_hi) = curve_a.domain();
//     let (b_lo, b_hi) = curve_b.domain();
//
//     let mut x_box = Vector::from_array([t_a, t_b]);
//     let mut deflated = false;
//
//     for _ in 0..20 {
//         let x_hat_raw = Vector::from_array([x_box[0].midpoint(), x_box[1].midpoint()]);
//         let x_hat = Vector::from_array([
//             clamp(x_hat_raw[0], a_lo, a_hi),
//             clamp(x_hat_raw[1], b_lo, b_hi),
//         ]);
//
//         let step = match plain_krawczyk_step::<S, D, C>(curve_a, curve_b, x_hat, x_box) {
//             Some(step) if !deflated => step,
//             _ => {
//                 deflated = true;
//                 match curve_a.deflated_krawczyk_step(curve_b, x_hat, x_box) {
//                     Some(step) => step,
//                     None => break,
//                 }
//             }
//         };
//
//         if step.empty {
//             break;
//         }
//
//         let stalled = !step.contracted[0].width().definitely_less(x_box[0].width())
//             && !step.contracted[1].width().definitely_less(x_box[1].width());
//         x_box = step.contracted;
//         if stalled {
//             break;
//         }
//     }
//
//     if !x_box[0].could_be_equal(t_a) || !x_box[1].could_be_equal(t_b) {
//         return (t_a, t_b);
//     }
//     (t_a.intersect(x_box[0]), t_b.intersect(x_box[1]))
// }

/// Newton iterations for [`refine_crossing`]. See `curve_surface`'s own
/// constant — this only affects how tightly an isolated answer is pinned down.
const REFINE_ITERATIONS: usize = 12;

/// Polish one isolated `(t_a, t_b)` — as returned by [`curve_curve_intersect`]
/// — by Gauss-Newton on `A(t_a) - B(t_b) = 0`.
///
/// Two unknowns against `C` equations, so this solves the normal equations
/// `(JᵀJ)δ = -JᵀF` with `J = [A'(t_a), -B'(t_b)]`. See
/// `curve_surface::refine_crossing` for why subdivision and Newton are split
/// this way, and why this is opt-in rather than applied to everything the
/// search returns.
///
/// Infallible by construction: anything that stops Newton — parallel tangents
/// making `JᵀJ` singular, an iterate leaving a domain, a refined box disjoint
/// from the one subdivision proved the solution lies in — returns the incoming
/// box unchanged. Refinement can only tighten, never fail.
///
/// (The Krawczyk-verified version above this is currently disabled — see the
/// module comment near the top of the file — so this is plain, unverified
/// Newton, same as before that work: it tightens an already-isolated crossing
/// but doesn't itself certify existence/uniqueness or handle a tangential
/// contact any better than stalling on it.)
pub fn refine_crossing<S: Scalar, const D: usize, const C: usize>(
    curve_a: &NurbCurve<S, D>,
    curve_b: &NurbCurve<S, D>,
    t_a: S,
    t_b: S,
) -> (S, S)
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    let (a_lo, a_hi) = curve_a.domain();
    let (b_lo, b_hi) = curve_b.domain();
    let (mut ta, mut tb) = (t_a.sharpen(), t_b.sharpen());

    for iteration in 0..REFINE_ITERATIONS {
        let (Ok(pa), Ok(pb)) = (
            curve_a.evaluate_cartesian(ta),
            curve_b.evaluate_cartesian(tb),
        ) else {
            return (t_a, t_b);
        };
        let (Ok(da), Ok(db)) = (curve_a.tangent_cartesian(ta), curve_b.tangent_cartesian(tb))
        else {
            return (t_a, t_b);
        };

        let f = pa.sub(&pb);
        let m = Matrix::from_rows([
            [da.prod_dot(&da), da.prod_dot(&db).neg()],
            [da.prod_dot(&db).neg(), db.prod_dot(&db)],
        ]);
        let rhs = Vector::from_array([da.prod_dot(&f).neg(), db.prod_dot(&f)]);
        let Ok(delta) = solve_linear_system(&m, &rhs) else {
            return (t_a, t_b);
        };

        let next = [ta.add(delta[0]), tb.add(delta[1])];
        let next = if iteration + 1 == REFINE_ITERATIONS {
            next
        } else {
            [next[0].sharpen(), next[1].sharpen()]
        };
        ta = clamp(next[0], a_lo, a_hi);
        tb = clamp(next[1], b_lo, b_hi);
    }

    if !ta.could_be_equal(t_a) || !tb.could_be_equal(t_b) {
        return (t_a, t_b);
    }
    let (a_ref, b_ref) = (t_a.intersect(ta), t_b.intersect(tb));
    // The refined box must still be able to hold a crossing — see
    // `curve_surface::refine_crossing`: plain Newton can return a narrow box
    // that provably misses where its root is not isolated and regular.
    let holds_a_crossing = match (
        curve_a.evaluate_cartesian(a_ref),
        curve_b.evaluate_cartesian(b_ref),
    ) {
        (Ok(pa), Ok(pb)) => pa.could_be_equal(&pb),
        _ => false,
    };
    if !holds_a_crossing {
        return (t_a, t_b);
    }
    (a_ref, b_ref)
}

#[cfg(test)]
mod tests {
    use super::curve_curve_crossings;
    use crate::nurb_curve::NurbCurve;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    const MAX: usize = 5000;
    const EPS: f64 = 1e-6;

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x * w),
            S::from_f64(y * w),
            S::from_f64(z * w),
            S::from_f64(w),
        ])
    }

    fn knots<S: Scalar>(k: &[f64]) -> Vec<S> {
        k.iter().map(|&x| S::from_f64(x)).collect()
    }

    fn line<S: Scalar>(a: [f64; 3], b: [f64; 3]) -> NurbCurve<S, 4> {
        NurbCurve::try_new(
            1,
            vec![pt(a[0], a[1], a[2], 1.), pt(b[0], b[1], b[2], 1.)],
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    /// Exact rational quarter circle of radius 1 in the xy-plane.
    fn quarter_circle<S: Scalar>() -> NurbCurve<S, 4> {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        NurbCurve::try_new(
            2,
            vec![pt(1., 0., 0., 1.), pt(1., 1., 0., w), pt(0., 1., 0., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap()
    }

    fn solve<S: Scalar>(a: &NurbCurve<S, 4>, b: &NurbCurve<S, 4>) -> Vec<(S, S)> {
        curve_curve_crossings::<S, 4, 3>(a, b, MAX, S::from_f64(EPS)).unwrap()
    }

    /// Every returned pair's two points agree, and there are `n` of them.
    fn assert_solutions<S: Scalar>(
        a: &NurbCurve<S, 4>,
        b: &NurbCurve<S, 4>,
        n: usize,
    ) -> Vec<(S, S)> {
        let sols = solve(a, b);
        assert_eq!(sols.len(), n, "{sols:?}");
        sols
    }

    /// `curve_curve.md` §3: projections alone can't contract this pair.
    fn check_crossing_diagonals<S: Scalar>() {
        let a = line::<S>([0., 0., 0.], [1., 1., 0.]);
        let b = line::<S>([0., 1., 0.], [1., 0., 0.]);
        let sols = assert_solutions(&a, &b, 1);
        let half = S::from_f64(0.5);
        assert!(
            sols[0].0.could_be_equal(half) && sols[0].1.could_be_equal(half),
            "{sols:?}"
        );
    }
    #[test]
    fn crossing_diagonals() {
        for_all_scalars!(check_crossing_diagonals);
    }

    fn check_circle_meets_line_once<S: Scalar>() {
        // x = y meets the arc at (√½, √½), parameter ½ by symmetry.
        let sols = assert_solutions(&quarter_circle::<S>(), &line([0., 0., 0.], [1., 1., 0.]), 1);
        assert!(sols[0].0.could_be_equal(S::from_f64(0.5)), "{sols:?}");
    }
    #[test]
    fn circle_meets_line_once() {
        for_all_scalars!(check_circle_meets_line_once);
    }

    fn check_circle_meets_chord_twice<S: Scalar>() {
        // x + y = 1.15 meets the arc at x = (1.15 ± √0.6775) / 2 ≈ 0.987 and
        // 0.163; the chord spans x ∈ [0.15, 1], so both.
        let chord = line::<S>([1., 0.15, 0.], [0.15, 1., 0.]);
        assert_solutions(&quarter_circle::<S>(), &chord, 2);
    }
    #[test]
    fn circle_meets_chord_twice() {
        for_all_scalars!(check_circle_meets_chord_twice);
    }

    /// Shared endpoint: the kernel's most common case (edges meeting at a
    /// vertex).
    fn check_shared_endpoint<S: Scalar>() {
        let a = line::<S>([0., 0., 0.], [1., 0., 0.]);
        let b = line::<S>([1., 0., 0.], [1., 1., 1.]);
        let sols = assert_solutions(&a, &b, 1);
        assert!(
            sols[0].0.could_be_equal(S::ONE) && sols[0].1.could_be_equal(S::ZERO),
            "{sols:?}"
        );
    }
    #[test]
    fn shared_endpoint() {
        for_all_scalars!(check_shared_endpoint);
    }

    fn check_skew_lines_miss<S: Scalar>() {
        let a = line::<S>([0., 0., 0.], [1., 1., 0.]);
        let b = line::<S>([0., 1., 0.01], [1., 0., 0.01]);
        assert_solutions(&a, &b, 0);
        assert_solutions(
            &quarter_circle::<S>(),
            &line([0., 0., 0.], [0.5, 0.5, 0.]),
            0,
        );
    }
    #[test]
    fn skew_lines_miss() {
        for_all_scalars!(check_skew_lines_miss);
    }

    fn check_budget_exhaustion_is_an_error<S: Scalar>() {
        let a = line::<S>([0., 0., 0.], [1., 1., 0.]);
        let b = line::<S>([0., 1., 0.], [1., 0., 0.]);
        assert!(curve_curve_crossings::<S, 4, 3>(&a, &b, 1, S::from_f64(EPS)).is_err());
    }
    #[test]
    fn budget_exhaustion_is_an_error() {
        for_all_scalars!(check_budget_exhaustion_is_an_error);
    }

    // ── The coincidence-handling wrapper ─────────────────────────────────────

    use super::curve_curve_intersect;
    use crate::intersection::Intersections;

    fn wrap<S: Scalar>(a: &NurbCurve<S, 4>, b: &NurbCurve<S, 4>) -> Intersections<(S, S)> {
        curve_curve_intersect::<S, 4, 3>(a, b, 5, MAX, S::from_f64(EPS)).unwrap()
    }

    /// Both ends of the arc lie on its chord, but the arc leaves it: the
    /// midpoint probe rules the candidate stretch out.
    fn check_arc_with_ends_on_chord_is_not_coincident<S: Scalar>() {
        let r = wrap(&quarter_circle::<S>(), &line([1., 0., 0.], [0., 1., 0.]));
        assert!(!r.is_coincident(), "{r:?}");
        assert_eq!(r.len(), 2, "{r:?}");
    }
    #[test]
    fn arc_with_ends_on_chord_is_not_coincident() {
        for_all_scalars!(check_arc_with_ends_on_chord_is_not_coincident);
    }

    fn check_shared_vertex_is_not_coincident<S: Scalar>() {
        let r = wrap(
            &line::<S>([0., 0., 0.], [1., 0., 0.]),
            &line([1., 0., 0.], [1., 1., 1.]),
        );
        assert!(!r.is_coincident() && r.len() == 1, "{r:?}");
    }
    #[test]
    fn shared_vertex_is_not_coincident() {
        for_all_scalars!(check_shared_vertex_is_not_coincident);
    }

    /// The overlap's ends come first: `a` at s = ½ (where `b` starts) and
    /// s = 1 (`a`'s end, `b` at t = ½).
    fn check_partial_overlap_reports_its_ends<S: Scalar>() {
        let a = line::<S>([0., 0., 0.], [1., 0., 0.]);
        let b = line::<S>([0.5, 0., 0.], [1.5, 0., 0.]);
        let r = wrap(&a, &b);
        assert!(r.is_coincident(), "{r:?}");
        let v = r.as_slice();
        let half = S::from_f64(0.5);
        assert!(
            v[0].0.could_be_equal(half) && v[0].1.could_be_equal(S::ZERO),
            "{v:?}"
        );
        assert!(
            v[1].0.could_be_equal(S::ONE) && v[1].1.could_be_equal(half),
            "{v:?}"
        );
        for (s, _) in v {
            assert!(!s.definitely_less(half), "{v:?}");
        }
    }
    #[test]
    fn partial_overlap_reports_its_ends() {
        for_all_scalars!(check_partial_overlap_reports_its_ends);
    }

    /// Same arc, split differently: coincident over the shared quarter.
    fn check_arc_pieces_overlap<S: Scalar>() {
        let arc = quarter_circle::<S>();
        let (left, _) = arc.split(S::from_f64(0.75)).unwrap();
        let (_, right) = arc.split(S::from_f64(0.25)).unwrap();
        let r = wrap(&left, &right);
        assert!(r.is_coincident(), "{r:?}");
        for (s, t) in r.as_slice() {
            assert!(s.could_be_equal(*t), "same curve, same parameter: {r:?}");
        }
    }
    #[test]
    fn arc_pieces_overlap() {
        for_all_scalars!(check_arc_pieces_overlap);
    }

    /// The test suite of the old `curve_curve` search, run unchanged against the
    /// coincidence-handling wrapper — the drop-in contract it must keep.
    mod old_suite {
        use super::super::curve_curve_intersect;
        use crate::intersection::curve_curve::refine_crossing;
        use crate::nurb_curve::NurbCurve;
        use geop_core_math::for_all_scalars;
        use geop_core_math::{
            scalars::Scalar,
            vector::{Vector3, Vector4},
        };

        const MAX_NODES: usize = 2000;

        fn ptc<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
            Vector4::from_array([
                S::from_f64(x),
                S::from_f64(y),
                S::from_f64(z),
                S::from_f64(w),
            ])
        }

        /// Horizontal line along the x axis, y=0.3, x ∈ [0,1].
        fn horizontal_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0., 0.3, 0., 1.), ptc(1., 0.3, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Vertical line along the y axis at x=0.5, y ∈ [-1,1] -- crosses
        /// `horizontal_line` once at (0.5, 0.3, 0).
        fn vertical_crossing_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.5, -1., 0., 1.), ptc(0.5, 1., 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Vertical line along the y axis at x=2.0, y ∈ [-1,1] -- never crosses
        /// `horizontal_line` (x ∈ [0,1]).
        fn vertical_missing_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(2.0, -1., 0., 1.), ptc(2.0, 1., 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Quadratic Bézier dipping below y=0.3 and back, crossing
        /// `horizontal_line` twice. x=0.3 is deliberately not the midpoint of
        /// `horizontal_line`'s x range [0,1], avoiding the "both halves always
        /// survive" tie pathology that exact midpoints trigger.
        fn double_dip_curve<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                2,
                vec![
                    ptc(0.3, 1.0, 0., 1.),
                    ptc(0.5, -2.0, 0., 1.),
                    ptc(0.7, 1.0, 0., 1.),
                ],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Straight line lying *on* `horizontal_line` (same y=0.3, z=0), spanning
        /// only part of its x range: from (0.5, 0.3, 0) to (1.5, 0.3, 0). The
        /// overlap with `horizontal_line` (x ∈ [0,1]) is x ∈ [0.5, 1].
        fn coincident_overlap_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.5, 0.3, 0., 1.), ptc(1.5, 0.3, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Straight line lying *on* `horizontal_line` exactly (same domain,
        /// x ∈ [0,1], y=0.3, z=0) — fully coincident, not just partially.
        fn full_coincident_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0., 0.3, 0., 1.), ptc(1., 0.3, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        const EPS: f64 = 1e-6;

        // ── Single crossing ───────────────────────────────────────────────────────

        fn check_single_crossing_curves_have_one_solution<S: Scalar>() {
            let a = horizontal_line::<S>();
            let b = vertical_crossing_line::<S>();
            let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result.len(), 1);
        }
        #[test]
        fn single_crossing_curves_have_one_solution() {
            for_all_scalars!(check_single_crossing_curves_have_one_solution);
        }

        // ── No crossing ───────────────────────────────────────────────────────────

        fn check_curves_missing_each_other_have_no_solution<S: Scalar>() {
            let a = horizontal_line::<S>();
            let b = vertical_missing_line::<S>();
            let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(result.is_empty());
        }
        #[test]
        fn curves_missing_each_other_have_no_solution() {
            for_all_scalars!(check_curves_missing_each_other_have_no_solution);
        }

        // ── Budget ────────────────────────────────────────────────────────────────

        fn check_max_solutions_zero_returns_empty<S: Scalar>() {
            let a = horizontal_line::<S>();
            let b = vertical_crossing_line::<S>();
            let result = curve_curve_intersect(&a, &b, 0, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(result.is_empty());
        }
        #[test]
        fn max_solutions_zero_returns_empty() {
            for_all_scalars!(check_max_solutions_zero_returns_empty);
        }

        fn check_max_nodes_exhausted_errors<S: Scalar>() {
            // Adapted: see the same test in `curve_surface`'s copy — the
            // coincident pair no longer overruns a tiny budget, so a pair
            // with two crossings that genuinely needs more nodes stands in.
            let a = horizontal_line::<S>();
            let b = double_dip_curve::<S>();
            let result = curve_curve_intersect(&a, &b, 1000, 1, S::from_f64(EPS));
            assert!(result.is_err());
        }
        #[test]
        fn max_nodes_exhausted_errors() {
            for_all_scalars!(check_max_nodes_exhausted_errors);
        }

        // ── Two crossings ─────────────────────────────────────────────────────────

        fn check_two_crossings_found_when_budget_allows<S: Scalar>() {
            let a = horizontal_line::<S>();
            let b = double_dip_curve::<S>();
            let result = curve_curve_intersect(&a, &b, 2, MAX_NODES, S::from_f64(EPS))
                .unwrap()
                .into_vec();
            assert_eq!(result.len(), 2);
            assert!(
                !result[0].0.could_be_equal(result[1].0),
                "the two crossings should remain distinct"
            );
        }
        #[test]
        fn two_crossings_found_when_budget_allows() {
            for_all_scalars!(check_two_crossings_found_when_budget_allows);
        }

        fn check_max_solutions_one_caps_at_one_even_with_two_crossings<S: Scalar>() {
            let a = horizontal_line::<S>();
            let b = double_dip_curve::<S>();
            let result = curve_curve_intersect(&a, &b, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result.len(), 1);
        }
        #[test]
        fn max_solutions_one_caps_at_one_even_with_two_crossings() {
            for_all_scalars!(check_max_solutions_one_caps_at_one_even_with_two_crossings);
        }

        // ── min_subdivision_size controls precision ────────────────────────────

        fn check_min_subdivision_size_controls_precision<S: Scalar>() {
            let a = horizontal_line::<S>();
            let b = vertical_crossing_line::<S>();
            let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(1e-3))
                .unwrap()
                .into_vec();
            assert_eq!(result.len(), 1);

            let (t_a, _) = result[0];
            assert!(
                t_a.sub(S::from_f64(0.5))
                    .abs()
                    .could_be_less(S::from_f64(1e-2))
            );
        }
        #[test]
        fn min_subdivision_size_controls_precision() {
            for_all_scalars!(check_min_subdivision_size_controls_precision);
        }

        // ── Coincident overlap: must terminate ───────────────────────────────────

        fn check_coincident_overlap_terminates<S: Scalar + 'static>() {
            let a = horizontal_line::<S>();
            let b = coincident_overlap_line::<S>();

            // A single dive must converge to exactly one result.
            let result_one = curve_curve_intersect(&a, &b, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result_one.len(), 1);

            // Asking for more solutions still terminates, with at most that many
            // (possibly fewer after merging) segments along the overlap, and
            // each solution lying within the overlapping x ∈ [0.5, 1] range.
            let result_many =
                curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);

            let lower_bound = S::from_f64(0.5 - EPS);
            for &(t_a, _) in result_many.as_slice() {
                assert!(t_a.could_be_greater(lower_bound));
            }
        }
        #[test]
        fn coincident_overlap_terminates() {
            for_all_scalars!(check_coincident_overlap_terminates);
        }

        // ── Coincident: an evenly-spread solution count, not just 1-or-cap ──────

        fn check_full_coincidence_reaches_max_solutions<S: Scalar>() {
            let a = horizontal_line::<S>();
            let b = full_coincident_line::<S>();
            // Two curves coincident over their *entire* shared domain, with a
            // generous node budget, should reliably reach the requested
            // solution count via the evenly-spread search -- and be reported
            // via the explicit `Coincident` variant, not just inferred from
            // hitting the length cap.
            let result = curve_curve_intersect(&a, &b, 5, 5000, S::from_f64(1e-3)).unwrap();
            assert!(result.is_coincident());
            assert_eq!(result.len(), 5);
        }
        #[test]
        fn full_coincidence_reaches_max_solutions() {
            for_all_scalars!(check_full_coincidence_reaches_max_solutions);
        }

        // ── 2-D (D=3) pcurve intersection — the motivating use case ──────────────

        fn pt2<S: Scalar>(x: f64, y: f64) -> Vector3<S> {
            Vector3::from_array([S::from_f64(x), S::from_f64(y), S::ONE])
        }

        /// Horizontal 2-D segment y=0.3, x ∈ [0,1].
        fn horizontal_line_2d<S: Scalar>() -> crate::nurb_curve::NurbCurve2D<S> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![pt2(0., 0.3), pt2(1., 0.3)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Vertical 2-D segment x=0.5, y ∈ [-1,1] — crosses the horizontal line
        /// once at (0.5, 0.3).
        fn vertical_crossing_line_2d<S: Scalar>() -> crate::nurb_curve::NurbCurve2D<S> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![pt2(0.5, -1.), pt2(0.5, 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        fn check_single_crossing_2d<S: Scalar>() {
            let a = horizontal_line_2d::<S>();
            let b = vertical_crossing_line_2d::<S>();
            let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(EPS))
                .unwrap()
                .into_vec();
            assert_eq!(result.len(), 1);
            let (t_a, _) = result[0];
            let hit = a.evaluate(t_a).unwrap();
            assert!(
                hit[0]
                    .sub(S::from_f64(0.5))
                    .abs()
                    .could_be_less(S::from_f64(1e-3))
            );
            assert!(
                hit[1]
                    .sub(S::from_f64(0.3))
                    .abs()
                    .could_be_less(S::from_f64(1e-3))
            );
        }
        #[test]
        fn single_crossing_2d() {
            for_all_scalars!(check_single_crossing_2d);
        }

        // ── Hard-to-intersect curves: quartic tangencies ─────────────────────────
        //
        // Both curves below are built by converting a monomial `(2t-1)^n` (in
        // `x = 2t-1`, over the standard `t ∈ [0,1]` NURBS domain) to its exact
        // Bernstein/Bezier control points, so `y` is *exactly* `x^n` along the
        // curve, not merely close to it — an honest, analytically-known worst
        // case rather than an approximation of one.

        /// Degree-2 Bezier tracing `y = x^2` exactly (`x = 2t-1`, `t ∈ [0,1]`),
        /// touching `y = 0` at `t = 0.5` with **order-2** contact (an ordinary
        /// parabola-tangent-to-a-line case) — the case one level of
        /// cross-product deflation is built to resolve.
        fn quadratic_tangent_to_x_axis<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                2,
                vec![
                    ptc(-1., 1., 0., 1.),
                    ptc(0., -1., 0., 1.),
                    ptc(1., 1., 0., 1.),
                ],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Degree-4 Bezier tracing `y = x^4` exactly (`x = 2t-1`, `t ∈ [0,1]`),
        /// touching `y = 0` at `t = 0.5` with **order-4** contact — flatter than
        /// cross-product deflation (which resolves order-2) can fully
        /// regularize: at the touch point both the tangent cross product *and*
        /// its first derivative vanish (`y = 16s^4` near `s = t-0.5` has
        /// `y'' = 192s^2 = 0` at `s = 0` too). The deliberately hard case: does
        /// refinement stay *sound* (never claims a narrower, wrong answer) when
        /// it cannot fully converge, rather than just being tight when it can.
        fn quartic_tangent_to_x_axis<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                4,
                vec![
                    ptc(-1., 1., 0., 1.),
                    ptc(-0.5, -1., 0., 1.),
                    ptc(0., 1., 0., 1.),
                    ptc(0.5, -1., 0., 1.),
                    ptc(1., 1., 0., 1.),
                ],
                vec![
                    f(0.),
                    f(0.),
                    f(0.),
                    f(0.),
                    f(0.),
                    f(1.),
                    f(1.),
                    f(1.),
                    f(1.),
                    f(1.),
                ],
            )
            .unwrap()
        }

        /// The x axis, x ∈ [-1,1] — the common tangent line for both curves
        /// above, touched at (0,0,0).
        fn x_axis_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(-1., 0., 0., 1.), ptc(1., 0., 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// An order-2 tangency: with Krawczyk-verified deflation currently
        /// disabled (see the module comment near the top of the file),
        /// `refine_crossing` is plain Newton, whose Jacobian is *also* singular
        /// right at a tangential contact — so this is a soundness check, same
        /// shape as the order-4 case below, not a tightness one. (Once deflation
        /// is re-enabled, this is exactly the case it's meant to resolve to
        /// machine/fixed-point precision instead.)
        fn check_refine_crossing_stays_sound_for_order_2_tangency<S: Scalar>() {
            let a = quadratic_tangent_to_x_axis::<S>();
            let b = x_axis_line::<S>();

            let found = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(1e-3))
                .unwrap()
                .into_vec();
            assert_eq!(
                found.len(),
                1,
                "a single tangential touch, not a crossing pair"
            );
            let (t_a, t_b) = found[0];

            // Soundness: the search's own (loose) box must already bracket the
            // true touch point.
            assert!(t_a.could_be_equal(S::from_f64(0.5)));
            assert!(t_b.could_be_equal(S::from_f64(0.5)));

            let (ra, rb) = refine_crossing(&a, &b, t_a, t_b);

            // Soundness: refinement must still contain the true parameter.
            assert!(ra.could_be_equal(S::from_f64(0.5)));
            assert!(rb.could_be_equal(S::from_f64(0.5)));
            // Never worse than the input: refinement can only tighten (or leave
            // it unchanged, which is what happens here since plain Newton's own
            // Jacobian is singular at this tangency too).
            assert!(ra.is_subset_of(t_a));
            assert!(rb.is_subset_of(t_b));
        }
        #[test]
        fn refine_crossing_stays_sound_for_order_2_tangency() {
            for_all_scalars!(check_refine_crossing_stays_sound_for_order_2_tangency);
        }

        /// Order-4 tangency: deflation's own Jacobian is *also* singular right
        /// at the touch point, so refinement is expected to stall -- the
        /// requirement under test is that it stays sound (still encloses the
        /// true parameter, never claims a narrower box than it actually proved)
        /// rather than that it achieves full precision.
        fn check_refine_crossing_stays_sound_for_order_4_tangency<S: Scalar>() {
            let a = quartic_tangent_to_x_axis::<S>();
            let b = x_axis_line::<S>();

            let found = curve_curve_intersect(&a, &b, 5, MAX_NODES, S::from_f64(1e-3))
                .unwrap()
                .into_vec();
            assert!(!found.is_empty(), "the touch point must still be found");

            for (t_a, t_b) in found {
                // Soundness of the search itself.
                assert!(t_a.could_be_equal(S::from_f64(0.5)));
                assert!(t_b.could_be_equal(S::from_f64(0.5)));

                let (ra, rb) = refine_crossing(&a, &b, t_a, t_b);

                // The refinement guarantee under test: whatever comes back
                // still encloses the true touch point (order-4 flatness may
                // well mean it comes back completely unchanged -- that is a
                // pass, not a failure, per `refine_crossing`'s own "can only
                // tighten, never fail" contract).
                assert!(
                    ra.could_be_equal(S::from_f64(0.5)),
                    "refine_crossing must never lose the true root: got {ra:?}"
                );
                assert!(rb.could_be_equal(S::from_f64(0.5)));
                // And never claim a box the search didn't already prove.
                assert!(ra.is_subset_of(t_a));
                assert!(rb.is_subset_of(t_b));
            }
        }
        #[test]
        fn refine_crossing_stays_sound_for_order_4_tangency() {
            for_all_scalars!(check_refine_crossing_stays_sound_for_order_4_tangency);
        }

        /// A transversal crossing very close to a quartic's flat spot (two
        /// distinct roots of `x^4 = 0.0001`, i.e. `x = ±0.1`, extremely close
        /// together and ill-conditioned near `x=0`) — not tangential at all,
        /// but numerically adversarial: checks the search still separates and
        /// soundly encloses both nearby crossings instead of merging or losing
        /// one.
        fn quartic_minus_epsilon<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            // y = x^4 - 0.0001, same control-point construction as
            // `quartic_tangent_to_x_axis` with every y-coordinate shifted down
            // by the constant 0.0001 (Bezier control points are affine in the
            // curve's own coordinates, so a constant shift is just a shift of
            // every control point's y).
            let dy = 0.0001;
            NurbCurve::try_new(
                4,
                vec![
                    ptc(-1., 1. - dy, 0., 1.),
                    ptc(-0.5, -1. - dy, 0., 1.),
                    ptc(0., 1. - dy, 0., 1.),
                    ptc(0.5, -1. - dy, 0., 1.),
                    ptc(1., 1. - dy, 0., 1.),
                ],
                vec![
                    f(0.),
                    f(0.),
                    f(0.),
                    f(0.),
                    f(0.),
                    f(1.),
                    f(1.),
                    f(1.),
                    f(1.),
                    f(1.),
                ],
            )
            .unwrap()
        }

        fn check_close_transversal_crossings_near_quartic_flat_spot<S: Scalar>() {
            let a = quartic_minus_epsilon::<S>();
            let b = x_axis_line::<S>();

            let found = curve_curve_intersect(&a, &b, 4, MAX_NODES, S::from_f64(1e-4))
                .unwrap()
                .into_vec();
            assert_eq!(
                found.len(),
                2,
                "two distinct, separated crossings near x=±0.1"
            );
            assert!(
                !found[0].0.could_be_equal(found[1].0),
                "the two nearby crossings must remain distinct"
            );

            // x = 2t-1 = ±0.1 -> t = 0.45 or t = 0.55.
            for &(t_a, t_b) in &found {
                let near_left = t_a
                    .sub(S::from_f64(0.45))
                    .abs()
                    .could_be_less(S::from_f64(1e-2));
                let near_right = t_a
                    .sub(S::from_f64(0.55))
                    .abs()
                    .could_be_less(S::from_f64(1e-2));
                assert!(near_left || near_right, "crossing at unexpected t={t_a:?}");

                let (ra, _) = refine_crossing(&a, &b, t_a, t_b);
                assert!(ra.is_subset_of(t_a), "refinement must only ever tighten");
            }
        }
        #[test]
        fn close_transversal_crossings_near_quartic_flat_spot() {
            for_all_scalars!(check_close_transversal_crossings_near_quartic_flat_spot);
        }
    }
}
