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

use super::{
    Intersections,
    coincidence::{self, Hit, Overlap},
};
use crate::{
    aabb::{aabb_could_overlap, curve_could_meet_aabb},
    contains::curve::curve_could_contain,
    fat_line::{
        Stalled, carried_width, chord, clip_tensor, extent, greville_abscissae, restriction,
        spanning, stalled,
    },
    nurb_curve::{NurbCurve, ParameterRefinable},
};
use geop_core_math::{
    disjoint_set::DisjointSet,
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::Matrix,
    scalars::Scalar,
    solvers::{
        linear::solve_linear_system,
        newton,
        subdivide::{Order, Searched, Split, subdivide},
    },
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
fn clip<const D: usize, const C: usize>(
    a: &NurbCurve<D>,
    b: &NurbCurve<D>,
) -> GeopResult<Option<[Scalar; 2]>> {
    let mut hats = [a.domain_as_scalar(), b.domain_as_scalar()];
    let (na, nb) = (a.control_points.len(), b.control_points.len());
    let greville = [
        greville_abscissae(&a.knot_vector, a.degree, na)?,
        greville_abscissae(&b.knot_vector, b.degree, nb)?,
    ];

    let mut dirs = chord::<D, C>(&b.control_points)
        .orthonormal_complement()
        .unwrap_or_default();
    dirs.extend(
        chord::<D, C>(&a.control_points)
            .orthonormal_complement()
            .unwrap_or_default(),
    );

    let w = D - 1;
    let mut d = Vec::with_capacity(na * nb);
    for n in spanning(dirs) {
        let nb_pts: Vec<Scalar> = b
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
pub fn curve_curve_crossings<const D: usize, const C: usize>(
    curve_a: &NurbCurve<D>,
    curve_b: &NurbCurve<D>,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Vec<(Scalar, Scalar)>>
where
    NurbCurve<D>: ParameterRefinable<C>,
{
    let mut solutions: DisjointSet<(Scalar, Scalar)> = DisjointSet::new();
    let root = (curve_a.clone(), curve_b.clone());
    let searched = subdivide(
        [root],
        Order::BreadthFirst,
        max_nodes,
        |(a, b), budget| -> GeopResult<_> {
            if !aabb_could_overlap(&a.aabb, &b.aabb, C) {
                return Ok(Split::Done);
            }
            let Some(hats) = clip::<D, C>(&a, &b)? else {
                return Ok(Split::Done);
            };

            // If the clip pinned `s` (`t`) exactly onto a clamped end of its
            // curve, every solution has that curve's endpoint as its spatial
            // point, so the rest is point containment in the other curve (the
            // "boundary evaluation" of `contains/surface.md` §3).
            let pinned = (a.pinned_end::<C>(hats[0]).map(|p| (p, true)))
                .or_else(|| b.pinned_end::<C>(hats[1]).map(|p| (p, false)));
            if let Some((point, on_a)) = pinned {
                let (other, free) = if on_a { (&b, hats[1]) } else { (&a, hats[0]) };
                if let Some(t) = curve_could_contain(other, &point, budget, min_subdivision_size)?
                    && t.could_be_equal(free)
                {
                    let free = free.intersect(t);
                    solutions.insert(if on_a {
                        (hats[0], free)
                    } else {
                        (free, hats[1])
                    });
                }
                return Ok(Split::Done);
            }

            let ranges = [a.domain(), b.domain()];
            let sizes = [
                extent([a.control_points.clone()]),
                extent([b.control_points.clone()]),
            ];
            let carried = carried_width(&a.control_points).max(carried_width(&b.control_points));
            if let Some(bounds) = restriction(&hats, &ranges, &sizes, carried)?
                && let (Ok(ra), Ok(rb)) = (
                    a.sub_curve(bounds[0].0, bounds[0].1),
                    b.sub_curve(bounds[1].0, bounds[1].1),
                )
            {
                return Ok(Split::Restrict((ra, rb)));
            }

            let order = match stalled(&ranges, &sizes, carried, min_subdivision_size) {
                Stalled::Converged => {
                    // The pieces' whole domains, not the tighter clip: pieces the
                    // search could not separate — a tangency converges on a chain
                    // of adjacent ones — must merge into one unresolved cluster,
                    // and adjacent domains share an endpoint where clips need not
                    // touch. A transversal crossing loses nothing: restriction has
                    // already cut its pieces down to the clip.
                    solutions.insert((a.domain_as_scalar(), b.domain_as_scalar()));
                    return Ok(Split::Done);
                }
                Stalled::Bisect(order) => order,
            };
            let children = order.iter().find_map(|&dir| {
                if dir == 0 {
                    let (l, r) = a.split_mid().ok()?;
                    Some(((l, b.clone()), (r, b.clone())))
                } else {
                    let (l, r) = b.split_mid().ok()?;
                    Some(((a.clone(), l), (a.clone(), r)))
                }
            });
            Ok(match children {
                Some((first, second)) => Split::Halves(first, second),
                // Nothing left to cut or split: what's here is the candidate.
                None => {
                    solutions.insert((hats[0], hats[1]));
                    Split::Done
                }
            })
        },
    )?;
    if let Searched::Exhausted { pending } = searched {
        return Err(GeopError::new(format!(
            "curve_curve_crossings: exhausted max_nodes={max_nodes} with {pending} \
             pairs pending; the result would be incomplete"
        )));
    }

    Ok(solutions.into_vec())
}

/// Where `other(t)` lies on `curve`, if it does: its parameter there.
fn point_on<const D: usize, const C: usize>(
    curve: &NurbCurve<D>,
    other: &NurbCurve<D>,
    t: Scalar,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Option<Scalar>>
where
    NurbCurve<D>: ParameterRefinable<C>,
{
    let point = other.evaluate_cartesian(t)?;
    curve_could_contain(curve, &point, max_nodes, min_subdivision_size)
}

/// Every stretch of `a` lying on `b` (see [`coincidence`]), with
/// `b`'s parameter as the partner: candidates are `a`'s ends found on `b`
/// and `b`'s ends found on `a`.
pub(crate) fn curve_curve_overlaps<const D: usize, const C: usize>(
    a: &NurbCurve<D>,
    b: &NurbCurve<D>,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Vec<Overlap<Scalar>>>
where
    NurbCurve<D>: ParameterRefinable<C>,
{
    let on = |curve: &NurbCurve<D>, other: &NurbCurve<D>, t: Scalar| {
        point_on(curve, other, t, max_nodes, min_subdivision_size)
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

/// What [`curve_curve_overlaps_and_crossings`] finds: the overlaps, and the
/// parameter pairs of the isolated crossings.
pub type OverlapsAndCrossings = (Vec<Overlap<Scalar>>, Vec<(Scalar, Scalar)>);

/// Overlaps of `a` with `b`, and the isolated crossings away from them (the
/// clipping search run on each stretch of `a` between overlaps).
pub fn curve_curve_overlaps_and_crossings<const D: usize, const C: usize>(
    a: &NurbCurve<D>,
    b: &NurbCurve<D>,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<OverlapsAndCrossings>
where
    NurbCurve<D>: ParameterRefinable<C>,
{
    let overlaps = curve_curve_overlaps(a, b, max_nodes, min_subdivision_size)?;
    let crossings = coincidence::crossings_between(a, &overlaps, |piece| {
        curve_curve_crossings(piece, b, max_nodes, min_subdivision_size)
    })?;
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
pub fn curve_curve_intersect<const D: usize, const C: usize>(
    curve_a: &NurbCurve<D>,
    curve_b: &NurbCurve<D>,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Intersections<(Scalar, Scalar)>>
where
    NurbCurve<D>: ParameterRefinable<C>,
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
        point_on(curve_b, curve_a, s, max_nodes, min_subdivision_size)
    })?;
    Ok(coincidence::assemble(
        &overlaps,
        crossings,
        samples,
        max_solutions,
    ))
}

/// Polish one isolated `(t_a, t_b)` — as returned by [`curve_curve_intersect`]
/// — by Gauss-Newton on `A(t_a) - B(t_b) = 0` ([`newton::refine`]).
///
/// Two unknowns against `C` equations, so each step solves the normal
/// equations `(JᵀJ)δ = -JᵀF` with `J = [A'(t_a), -B'(t_b)]`; parallel
/// tangents make `JᵀJ` singular, and leave the box as it is. So does a
/// refined box that provably holds no crossing — see
/// `curve_surface::refine_crossing`.
pub fn refine_crossing<const D: usize, const C: usize>(
    curve_a: &NurbCurve<D>,
    curve_b: &NurbCurve<D>,
    t_a: Scalar,
    t_b: Scalar,
) -> (Scalar, Scalar)
where
    NurbCurve<D>: ParameterRefinable<C>,
{
    let refined = newton::refine(
        [t_a, t_b],
        [curve_a.domain(), curve_b.domain()],
        |[ta, tb]| {
            let pa = curve_a.evaluate_cartesian(ta).ok()?;
            let pb = curve_b.evaluate_cartesian(tb).ok()?;
            let da = curve_a.tangent_cartesian(ta).ok()?;
            let db = curve_b.tangent_cartesian(tb).ok()?;
            let f = pa.sub(&pb);
            let m = Matrix::from_rows([
                [da.prod_dot(&da), da.prod_dot(&db).neg()],
                [da.prod_dot(&db).neg(), db.prod_dot(&db)],
            ]);
            let rhs = Vector::from_array([da.prod_dot(&f).neg(), db.prod_dot(&f)]);
            let delta = solve_linear_system(&m, &rhs).ok()?;
            Some([ta.add(delta[0]), tb.add(delta[1])])
        },
    );
    let [a_ref, b_ref] = refined;
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
    use crate::test_fixtures::{knots, line, weighted};
    use geop_core_math::scalars::Scalar;

    const MAX: usize = 5000;
    const MIN_SUBDIVISION_SIZE: f64 = 1e-6;

    /// Exact rational quarter circle of radius 1 in the xy-plane.
    fn quarter_circle() -> NurbCurve<4> {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        NurbCurve::try_new(
            2,
            vec![
                weighted(1., 0., 0., 1.),
                weighted(1., 1., 0., w),
                weighted(0., 1., 0., 1.),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap()
    }

    fn solve(a: &NurbCurve<4>, b: &NurbCurve<4>) -> Vec<(Scalar, Scalar)> {
        curve_curve_crossings::<4, 3>(a, b, MAX, Scalar::from_f64(MIN_SUBDIVISION_SIZE)).unwrap()
    }

    /// Every returned pair's two points agree, and there are `n` of them.
    fn assert_solutions(a: &NurbCurve<4>, b: &NurbCurve<4>, n: usize) -> Vec<(Scalar, Scalar)> {
        let sols = solve(a, b);
        assert_eq!(sols.len(), n, "{sols:?}");
        sols
    }

    /// `curve_curve.md` §3: projections alone can't contract this pair.
    #[test]
    fn crossing_diagonals() {
        let a = line([0., 0., 0.], [1., 1., 0.]);
        let b = line([0., 1., 0.], [1., 0., 0.]);
        let sols = assert_solutions(&a, &b, 1);
        let half = Scalar::from_f64(0.5);
        assert!(
            sols[0].0.could_be_equal(half) && sols[0].1.could_be_equal(half),
            "{sols:?}"
        );
    }

    #[test]
    fn circle_meets_line_once() {
        // x = y meets the arc at (√½, √½), parameter ½ by symmetry.
        let sols = assert_solutions(&quarter_circle(), &line([0., 0., 0.], [1., 1., 0.]), 1);
        assert!(sols[0].0.could_be_equal(Scalar::from_f64(0.5)), "{sols:?}");
    }

    #[test]
    fn circle_meets_chord_twice() {
        // x + y = 1.15 meets the arc at x = (1.15 ± √0.6775) / 2 ≈ 0.987 and
        // 0.163; the chord spans x ∈ [0.15, 1], so both.
        let chord = line([1., 0.15, 0.], [0.15, 1., 0.]);
        assert_solutions(&quarter_circle(), &chord, 2);
    }

    /// Shared endpoint: the kernel's most common case (edges meeting at a
    /// vertex).
    #[test]
    fn shared_endpoint() {
        let a = line([0., 0., 0.], [1., 0., 0.]);
        let b = line([1., 0., 0.], [1., 1., 1.]);
        let sols = assert_solutions(&a, &b, 1);
        assert!(
            sols[0].0.could_be_equal(Scalar::ONE) && sols[0].1.could_be_equal(Scalar::ZERO),
            "{sols:?}"
        );
    }

    #[test]
    fn skew_lines_miss() {
        let a = line([0., 0., 0.], [1., 1., 0.]);
        let b = line([0., 1., 0.01], [1., 0., 0.01]);
        assert_solutions(&a, &b, 0);
        assert_solutions(&quarter_circle(), &line([0., 0., 0.], [0.5, 0.5, 0.]), 0);
    }

    #[test]
    fn budget_exhaustion_is_an_error() {
        let a = line([0., 0., 0.], [1., 1., 0.]);
        let b = line([0., 1., 0.], [1., 0., 0.]);
        assert!(
            curve_curve_crossings::<4, 3>(&a, &b, 1, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                .is_err()
        );
    }

    // ── The coincidence-handling wrapper ─────────────────────────────────────

    use super::curve_curve_intersect;
    use crate::intersection::Intersections;

    fn wrap(a: &NurbCurve<4>, b: &NurbCurve<4>) -> Intersections<(Scalar, Scalar)> {
        curve_curve_intersect::<4, 3>(a, b, 5, MAX, Scalar::from_f64(MIN_SUBDIVISION_SIZE)).unwrap()
    }

    /// Both ends of the arc lie on its chord, but the arc leaves it: the
    /// midpoint probe rules the candidate stretch out.
    #[test]
    fn arc_with_ends_on_chord_is_not_coincident() {
        let r = wrap(&quarter_circle(), &line([1., 0., 0.], [0., 1., 0.]));
        assert!(!r.is_coincident(), "{r:?}");
        assert_eq!(r.len(), 2, "{r:?}");
    }

    #[test]
    fn shared_vertex_is_not_coincident() {
        let r = wrap(
            &line([0., 0., 0.], [1., 0., 0.]),
            &line([1., 0., 0.], [1., 1., 1.]),
        );
        assert!(!r.is_coincident() && r.len() == 1, "{r:?}");
    }

    /// The overlap's ends come first: `a` at s = ½ (where `b` starts) and
    /// s = 1 (`a`'s end, `b` at t = ½).
    #[test]
    fn partial_overlap_reports_its_ends() {
        let a = line([0., 0., 0.], [1., 0., 0.]);
        let b = line([0.5, 0., 0.], [1.5, 0., 0.]);
        let r = wrap(&a, &b);
        assert!(r.is_coincident(), "{r:?}");
        let v = r.as_slice();
        let half = Scalar::from_f64(0.5);
        assert!(
            v[0].0.could_be_equal(half) && v[0].1.could_be_equal(Scalar::ZERO),
            "{v:?}"
        );
        assert!(
            v[1].0.could_be_equal(Scalar::ONE) && v[1].1.could_be_equal(half),
            "{v:?}"
        );
        for (s, _) in v {
            assert!(!s.definitely_less(half), "{v:?}");
        }
    }

    /// Same arc, split differently: coincident over the shared quarter.
    #[test]
    fn arc_pieces_overlap() {
        let arc = quarter_circle();
        let (left, _) = arc.split(Scalar::from_f64(0.75)).unwrap();
        let (_, right) = arc.split(Scalar::from_f64(0.25)).unwrap();
        let r = wrap(&left, &right);
        assert!(r.is_coincident(), "{r:?}");
        for (s, t) in r.as_slice() {
            assert!(s.could_be_equal(*t), "same curve, same parameter: {r:?}");
        }
    }

    /// The test suite of the old `curve_curve` search, run unchanged against the
    /// coincidence-handling wrapper — the drop-in contract it must keep.
    mod old_suite {
        use super::super::curve_curve_intersect;
        use crate::intersection::curve_curve::refine_crossing;
        use crate::nurb_curve::NurbCurve;
        use geop_core_math::{
            scalars::Scalar,
            vector::{Vector3, Vector4},
        };

        const MAX_NODES: usize = 2000;

        fn ptc(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
            Vector4::from_array([
                Scalar::from_f64(x),
                Scalar::from_f64(y),
                Scalar::from_f64(z),
                Scalar::from_f64(w),
            ])
        }

        /// Horizontal line along the x axis, y=0.3, x ∈ [0,1].
        fn horizontal_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0., 0.3, 0., 1.), ptc(1., 0.3, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Vertical line along the y axis at x=0.5, y ∈ [-1,1] -- crosses
        /// `horizontal_line` once at (0.5, 0.3, 0).
        fn vertical_crossing_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.5, -1., 0., 1.), ptc(0.5, 1., 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Vertical line along the y axis at x=2.0, y ∈ [-1,1] -- never crosses
        /// `horizontal_line` (x ∈ [0,1]).
        fn vertical_missing_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
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
        fn double_dip_curve() -> NurbCurve<4> {
            let f = Scalar::from_f64;
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
        fn coincident_overlap_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.5, 0.3, 0., 1.), ptc(1.5, 0.3, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Straight line lying *on* `horizontal_line` exactly (same domain,
        /// x ∈ [0,1], y=0.3, z=0) — fully coincident, not just partially.
        fn full_coincident_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0., 0.3, 0., 1.), ptc(1., 0.3, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        const MIN_SUBDIVISION_SIZE: f64 = 1e-6;

        // ── Single crossing ───────────────────────────────────────────────────────

        #[test]
        fn single_crossing_curves_have_one_solution() {
            let a = horizontal_line();
            let b = vertical_crossing_line();
            let result =
                curve_curve_intersect(&a, &b, 5, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap();
            assert_eq!(result.len(), 1);
        }

        // ── No crossing ───────────────────────────────────────────────────────────

        #[test]
        fn curves_missing_each_other_have_no_solution() {
            let a = horizontal_line();
            let b = vertical_missing_line();
            let result =
                curve_curve_intersect(&a, &b, 5, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap();
            assert!(result.is_empty());
        }

        // ── Budget ────────────────────────────────────────────────────────────────

        #[test]
        fn max_solutions_zero_returns_empty() {
            let a = horizontal_line();
            let b = vertical_crossing_line();
            let result =
                curve_curve_intersect(&a, &b, 0, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap();
            assert!(result.is_empty());
        }

        #[test]
        fn max_nodes_exhausted_errors() {
            // Adapted: see the same test in `curve_surface`'s copy — the
            // coincident pair no longer overruns a tiny budget, so a pair
            // with two crossings that genuinely needs more nodes stands in.
            let a = horizontal_line();
            let b = double_dip_curve();
            let result =
                curve_curve_intersect(&a, &b, 1000, 1, Scalar::from_f64(MIN_SUBDIVISION_SIZE));
            assert!(result.is_err());
        }

        // ── Two crossings ─────────────────────────────────────────────────────────

        #[test]
        fn two_crossings_found_when_budget_allows() {
            let a = horizontal_line();
            let b = double_dip_curve();
            let result =
                curve_curve_intersect(&a, &b, 2, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap()
                    .into_vec();
            assert_eq!(result.len(), 2);
            assert!(
                !result[0].0.could_be_equal(result[1].0),
                "the two crossings should remain distinct"
            );
        }

        #[test]
        fn max_solutions_one_caps_at_one_even_with_two_crossings() {
            let a = horizontal_line();
            let b = double_dip_curve();
            let result =
                curve_curve_intersect(&a, &b, 1, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap();
            assert_eq!(result.len(), 1);
        }

        // ── min_subdivision_size controls precision ────────────────────────────

        #[test]
        fn min_subdivision_size_controls_precision() {
            let a = horizontal_line();
            let b = vertical_crossing_line();
            let result = curve_curve_intersect(&a, &b, 5, MAX_NODES, Scalar::from_f64(1e-3))
                .unwrap()
                .into_vec();
            assert_eq!(result.len(), 1);

            let (t_a, _) = result[0];
            assert!(
                t_a.sub(Scalar::from_f64(0.5))
                    .abs()
                    .could_be_less(Scalar::from_f64(1e-2))
            );
        }

        // ── Coincident overlap: must terminate ───────────────────────────────────

        #[test]
        fn coincident_overlap_terminates() {
            let a = horizontal_line();
            let b = coincident_overlap_line();

            // A single dive must converge to exactly one result.
            let result_one =
                curve_curve_intersect(&a, &b, 1, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap();
            assert_eq!(result_one.len(), 1);

            // Asking for more solutions still terminates, with at most that many
            // (possibly fewer after merging) segments along the overlap, and
            // each solution lying within the overlapping x ∈ [0.5, 1] range.
            let result_many =
                curve_curve_intersect(&a, &b, 5, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);

            let lower_bound = Scalar::from_f64(0.5 - MIN_SUBDIVISION_SIZE);
            for &(t_a, _) in result_many.as_slice() {
                assert!(t_a.could_be_greater(lower_bound));
            }
        }

        // ── Coincident: an evenly-spread solution count, not just 1-or-cap ──────

        #[test]
        fn full_coincidence_reaches_max_solutions() {
            let a = horizontal_line();
            let b = full_coincident_line();
            // Two curves coincident over their *entire* shared domain, with a
            // generous node budget, should reliably reach the requested
            // solution count via the evenly-spread search -- and be reported
            // via the explicit `Coincident` variant, not just inferred from
            // hitting the length cap.
            let result = curve_curve_intersect(&a, &b, 5, 5000, Scalar::from_f64(1e-3)).unwrap();
            assert!(result.is_coincident());
            assert_eq!(result.len(), 5);
        }

        // ── 2-D (D=3) pcurve intersection — the motivating use case ──────────────

        fn pt2(x: f64, y: f64) -> Vector3 {
            Vector3::from_array([Scalar::from_f64(x), Scalar::from_f64(y), Scalar::ONE])
        }

        /// Horizontal 2-D segment y=0.3, x ∈ [0,1].
        fn horizontal_line_2d() -> crate::nurb_curve::NurbCurve2D {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![pt2(0., 0.3), pt2(1., 0.3)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Vertical 2-D segment x=0.5, y ∈ [-1,1] — crosses the horizontal line
        /// once at (0.5, 0.3).
        fn vertical_crossing_line_2d() -> crate::nurb_curve::NurbCurve2D {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![pt2(0.5, -1.), pt2(0.5, 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        #[test]
        fn single_crossing_2d() {
            let a = horizontal_line_2d();
            let b = vertical_crossing_line_2d();
            let result =
                curve_curve_intersect(&a, &b, 5, MAX_NODES, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                    .unwrap()
                    .into_vec();
            assert_eq!(result.len(), 1);
            let (t_a, _) = result[0];
            let hit = a.evaluate(t_a).unwrap();
            assert!(
                hit[0]
                    .sub(Scalar::from_f64(0.5))
                    .abs()
                    .could_be_less(Scalar::from_f64(1e-3))
            );
            assert!(
                hit[1]
                    .sub(Scalar::from_f64(0.3))
                    .abs()
                    .could_be_less(Scalar::from_f64(1e-3))
            );
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
        fn quadratic_tangent_to_x_axis() -> NurbCurve<4> {
            let f = Scalar::from_f64;
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
        fn quartic_tangent_to_x_axis() -> NurbCurve<4> {
            let f = Scalar::from_f64;
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
        fn x_axis_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(-1., 0., 0., 1.), ptc(1., 0., 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// An order-2 tangency: `refine_crossing` is plain Newton, whose
        /// Jacobian is singular right at a tangential contact — so this is a
        /// soundness check, same shape as the order-4 case below, not a
        /// tightness one.
        #[test]
        fn refine_crossing_stays_sound_for_order_2_tangency() {
            let a = quadratic_tangent_to_x_axis();
            let b = x_axis_line();

            let found = curve_curve_intersect(&a, &b, 5, MAX_NODES, Scalar::from_f64(1e-3))
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
            assert!(t_a.could_be_equal(Scalar::from_f64(0.5)));
            assert!(t_b.could_be_equal(Scalar::from_f64(0.5)));

            let (ra, rb) = refine_crossing(&a, &b, t_a, t_b);

            // Soundness: refinement must still contain the true parameter.
            assert!(ra.could_be_equal(Scalar::from_f64(0.5)));
            assert!(rb.could_be_equal(Scalar::from_f64(0.5)));
            // Never worse than the input: refinement can only tighten (or leave
            // it unchanged, which is what happens here since plain Newton's own
            // Jacobian is singular at this tangency too).
            assert!(ra.is_subset_of(t_a));
            assert!(rb.is_subset_of(t_b));
        }

        /// Order-4 tangency: deflation's own Jacobian is *also* singular right
        /// at the touch point, so refinement is expected to stall -- the
        /// requirement under test is that it stays sound (still encloses the
        /// true parameter, never claims a narrower box than it actually proved)
        /// rather than that it achieves full precision.
        #[test]
        fn refine_crossing_stays_sound_for_order_4_tangency() {
            let a = quartic_tangent_to_x_axis();
            let b = x_axis_line();

            let found = curve_curve_intersect(&a, &b, 5, MAX_NODES, Scalar::from_f64(1e-3))
                .unwrap()
                .into_vec();
            assert!(!found.is_empty(), "the touch point must still be found");

            for (t_a, t_b) in found {
                // Soundness of the search itself.
                assert!(t_a.could_be_equal(Scalar::from_f64(0.5)));
                assert!(t_b.could_be_equal(Scalar::from_f64(0.5)));

                let (ra, rb) = refine_crossing(&a, &b, t_a, t_b);

                // The refinement guarantee under test: whatever comes back
                // still encloses the true touch point (order-4 flatness may
                // well mean it comes back completely unchanged -- that is a
                // pass, not a failure, per `refine_crossing`'s own "can only
                // tighten, never fail" contract).
                assert!(
                    ra.could_be_equal(Scalar::from_f64(0.5)),
                    "refine_crossing must never lose the true root: got {ra:?}"
                );
                assert!(rb.could_be_equal(Scalar::from_f64(0.5)));
                // And never claim a box the search didn't already prove.
                assert!(ra.is_subset_of(t_a));
                assert!(rb.is_subset_of(t_b));
            }
        }

        /// A transversal crossing very close to a quartic's flat spot (two
        /// distinct roots of `x^4 = 0.0001`, i.e. `x = ±0.1`, extremely close
        /// together and ill-conditioned near `x=0`) — not tangential at all,
        /// but numerically adversarial: checks the search still separates and
        /// soundly encloses both nearby crossings instead of merging or losing
        /// one.
        fn quartic_minus_epsilon() -> NurbCurve<4> {
            let f = Scalar::from_f64;
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

        #[test]
        fn close_transversal_crossings_near_quartic_flat_spot() {
            let a = quartic_minus_epsilon();
            let b = x_axis_line();

            let found = curve_curve_intersect(&a, &b, 4, MAX_NODES, Scalar::from_f64(1e-4))
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
                    .sub(Scalar::from_f64(0.45))
                    .abs()
                    .could_be_less(Scalar::from_f64(1e-2));
                let near_right = t_a
                    .sub(Scalar::from_f64(0.55))
                    .abs()
                    .could_be_less(Scalar::from_f64(1e-2));
                assert!(near_left || near_right, "crossing at unexpected t={t_a:?}");

                let (ra, _) = refine_crossing(&a, &b, t_a, t_b);
                assert!(ra.is_subset_of(t_a), "refinement must only ever tighten");
            }
        }
    }
}
