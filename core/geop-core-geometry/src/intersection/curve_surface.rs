//! Curve–surface intersection by per-axis fat line clipping — the design is
//! `curve_surface.md` next to this file; it is `curve_curve` with one more
//! parameter.
//!
//! For a curve `C = H_C / W_C` and a surface `Scalar = H_S / W_S` with positive
//! weights, `C_k(t) = S_k(u, v)` iff
//! `g_k(t, u, v) = H_{C,k}(t) W_S(u, v) - W_C(t) H_{Scalar,k}(u, v) = 0`, a
//! polynomial tensor-product spline in three independent parameters with
//! coefficients `d_ijl = P_{i,k} Q_{jl,w} - P_{i,w} Q_{jl,k}`. Its zeros are
//! clipped in all three directions ([`clip_tensor`]) — nine clips per box.
//!
//! Assumes no arc of the curve lies on the surface (`curve_surface.md`): the
//! result is a list of paired `(t, (u, v))` boxes, and reaching some count
//! of them means nothing. Works on the untrimmed patch.

use super::{
    Intersections,
    coincidence::{self, Hit, Overlap},
};
use crate::{
    aabb::{aabb_could_overlap, curve_could_meet_aabb},
    contains::surface::{
        boundary_curve, collapsed_boundary, surface_could_contain, surface_extents,
    },
    fat_line::{
        Stalled, carried_width, chord, clip_tensor, extent, greville_abscissae, restriction,
        spanning, stalled,
    },
    intersection::curve_curve,
    nurb_curve::{NurbCurve, dehomogenize},
    nurb_surface::NurbSurface,
    search::{MAX_NODES, min_subdivision_size},
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
    vector::{Vector, Vector2},
};

/// Clip the pair: a box `[t̂, û, v̂]` inside the domains enclosing every
/// `(t, u, v)` with `C(t) = Scalar(u, v)`, or `None` if some equation proves
/// there is none. Both objects' weights are positive by construction
/// (`curve_surface.md` §1), which the cross multiplication needs.
///
/// The equations are combinations `g_n = n · (H_C W_S - W_C H_S)` along
/// free-choice directions ([`spanning`]), from the curve's chord `c` and
/// the patch's mean edge directions `e_u`, `e_v`: the patch normal
/// `e_u × e_v` (nearly independent of `u, v`, so it pins `t`), `c × e_v`
/// (pins `u`) and `c × e_u` (pins `v`). If they don't span space — a curve
/// without a chord, or lying in the patch's plane, which makes all three
/// parallel — the coordinate axes are added.
fn clip(c: &NurbCurve<4>, s: &NurbSurface<4>) -> GeopResult<Option<[Scalar; 3]>> {
    let (u0, u1) = s.domain_u();
    let (v0, v1) = s.domain_v();
    let mut hats = [c.domain_as_scalar(), u0.union(u1), v0.union(v1)];
    let (nc, nu, nv) = (c.control_points.len(), s.num_u, s.num_v);
    let greville = [
        greville_abscissae(&c.knot_vector, c.degree, nc)?,
        greville_abscissae(&s.knot_vector_u, s.degree_u, nu)?,
        greville_abscissae(&s.knot_vector_v, s.degree_v, nv)?,
    ];

    let cp = &s.control_points;
    let corners = dehomogenize::<4, 3>(&[cp[0], cp[nv - 1], cp[(nu - 1) * nv], cp[nu * nv - 1]]);
    let (p00, p01, p10, p11) = (corners[0], corners[1], corners[2], corners[3]);
    let e_u = p10.add(&p11).sub(&p00.add(&p01)).sharpen();
    let e_v = p01.add(&p11).sub(&p00.add(&p10)).sharpen();
    let c_dir = chord::<4, 3>(&c.control_points);
    let dirs = [
        e_u.prod_cross(&e_v),
        c_dir.prod_cross(&e_v),
        c_dir.prod_cross(&e_u),
    ]
    .into_iter()
    .filter_map(|n| n.normalize().ok())
    .collect();

    let mut d = Vec::with_capacity(nc * nu * nv);
    for n in spanning(dirs) {
        let ns: Vec<Scalar> = cp.iter().map(|q| n.prod_dot(&q.head())).collect();
        d.clear();
        for p in &c.control_points {
            let np = n.prod_dot(&p.head());
            d.extend(
                s.control_points
                    .iter()
                    .zip(&ns)
                    .map(|(q, &nq)| np.mul(q[3]).sub(p[3].mul(nq))),
            );
        }
        if !clip_tensor(&d, &[nc, nu, nv], &greville, &mut hats) {
            return Ok(None);
        }
    }
    Ok(Some(hats))
}

/// The "boundary evaluation" of `contains/surface.md` §3, one dimension up:
/// if the clip pinned a parameter exactly onto a clamped end, the problem
/// loses that dimension. `t` pinned → the curve's endpoint must lie on the
/// surface ([`surface_could_contain`]); `u` or `v` pinned → the
/// curve must meet that boundary row of the surface
/// ([`curve_curve::curve_curve_crossings`]). Returns the boxes found, or
/// `None` if nothing is pinned.
fn solve_pinned(
    c: &NurbCurve<4>,
    s: &NurbSurface<4>,
    hats: [Scalar; 3],
    budget: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Option<Vec<[Scalar; 3]>>> {
    let overlapping = |found: [Scalar; 3]| -> Option<[Scalar; 3]> {
        (0..3)
            .all(|i| found[i].could_be_equal(hats[i]))
            .then(|| std::array::from_fn(|i| hats[i].intersect(found[i])))
    };

    if let Some(end) = c.pinned_end::<3>(hats[0]) {
        let found = surface_could_contain(s, &end, budget, min_subdivision_size)?;
        return Ok(Some(
            found
                .and_then(|(u, v)| overlapping([hats[0], u, v]))
                .into_iter()
                .collect(),
        ));
    }

    if let Some((boundary, along_v)) = collapsed_boundary(s, hats[1], hats[2]) {
        let pairs =
            curve_curve::curve_curve_crossings::<4, 3>(c, &boundary, budget, min_subdivision_size)?;
        return Ok(Some(
            pairs
                .into_iter()
                .filter_map(|(t, w)| {
                    overlapping(if along_v {
                        [t, hats[1], w]
                    } else {
                        [t, w, hats[2]]
                    })
                })
                .collect(),
        ));
    }
    Ok(None)
}

/// All `(t, (u, v))` with `curve(t) = surface(u, v)`, as paired parameter
/// boxes (`curve_surface.md`), for a curve with no arc lying on the surface —
/// see [`curve_surface_intersect`] for the wrapper that handles that. The same search as
/// [`curve_curve::curve_curve_crossings`], over (curve segment, patch)
/// pairs with three parameter directions: AABB and [`clip`] rejection, a
/// pinned parameter handed down one dimension ([`solve_pinned`]),
/// restriction while the clip keeps shrinking the pair
/// ([`crate::fat_line::restriction`]), and once it stalls, convergence when
/// both objects' extents are within `min_subdivision_size` or fair
/// bisection otherwise ([`crate::fat_line::stalled`]).
///
/// Overlapping boxes merge by union into one unresolved cluster, never an
/// average. Exhausting `max_nodes` is an error — the result would be
/// incomplete.
pub fn curve_surface_crossings(
    curve: &NurbCurve<4>,
    surface: &NurbSurface<4>,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Vec<(Scalar, Vector2)>> {
    let mut solutions: DisjointSet<(Scalar, Vector2)> = DisjointSet::new();
    let mut insert = |[t, u, v]: [Scalar; 3]| solutions.insert((t, Vector2::from_array([u, v])));

    let root = (curve.clone(), surface.clone());
    let searched = subdivide(
        [root],
        Order::BreadthFirst,
        max_nodes,
        |(c, s), budget| -> GeopResult<_> {
            if !aabb_could_overlap(&c.aabb, &s.aabb, 3) {
                return Ok(Split::Done);
            }
            let Some(hats) = clip(&c, &s)? else {
                return Ok(Split::Done);
            };

            if let Some(found) = solve_pinned(&c, &s, hats, budget, min_subdivision_size)? {
                found.into_iter().for_each(&mut insert);
                return Ok(Split::Done);
            }

            let ranges = [c.domain(), s.domain_u(), s.domain_v()];
            let [size_u, size_v] = surface_extents(&s);
            let sizes = [extent([c.control_points.clone()]), size_u, size_v];
            let carried = carried_width(&c.control_points).max(carried_width(&s.control_points));
            if let Some(b) = restriction(&hats, &ranges, &sizes, carried)?
                && let (Ok(rc), Ok(rs)) = (c.sub_curve(b[0].0, b[0].1), s.sub_surface(b[1], b[2]))
            {
                return Ok(Split::Restrict((rc, rs)));
            }

            let order = match stalled(&ranges, &sizes, carried, min_subdivision_size) {
                Stalled::Converged => {
                    // The pieces' whole domains, not the tighter clip, so pieces
                    // the search could not separate merge — see
                    // `curve_curve_crossings`.
                    let (u0, u1) = s.domain_u();
                    let (v0, v1) = s.domain_v();
                    insert([c.domain_as_scalar(), u0.union(u1), v0.union(v1)]);
                    return Ok(Split::Done);
                }
                Stalled::Bisect(order) => order,
            };
            let children = order.iter().find_map(|&dir| match dir {
                0 => {
                    let (l, r) = c.split_mid().ok()?;
                    Some(((l, s.clone()), (r, s.clone())))
                }
                1 => {
                    let (l, r) = s.split_u_mid().ok()?;
                    Some(((c.clone(), l), (c.clone(), r)))
                }
                _ => {
                    let (l, r) = s.split_v_mid().ok()?;
                    Some(((c.clone(), l), (c.clone(), r)))
                }
            });
            Ok(match children {
                Some((first, second)) => Split::Halves(first, second),
                // Nothing left to cut or split: what's here is the candidate.
                None => {
                    insert(hats);
                    Split::Done
                }
            })
        },
    )?;
    if let Searched::Exhausted { pending } = searched {
        return Err(GeopError::new(format!(
            "curve_surface_crossings: exhausted max_nodes={max_nodes} with {pending} \
             pairs pending; the result would be incomplete"
        )));
    }

    Ok(solutions.into_vec())
}

/// Where `curve(t)` lies on `surface`, if it does: its `(u, v)` there.
fn point_on(
    curve: &NurbCurve<4>,
    surface: &NurbSurface<4>,
    t: Scalar,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Option<Vector2>> {
    let point = curve.evaluate(t)?;
    Ok(
        surface_could_contain(surface, &point, max_nodes, min_subdivision_size)?
            .map(|(u, v)| Vector2::from_array([u, v])),
    )
}

/// Every stretch of `curve` lying on `surface` (see [`coincidence`]), with
/// the surface parameters as the partner. Candidates are the curve's ends
/// found on the surface, and where the curve meets the patch's four
/// boundary curves — through [`curve_curve::curve_curve_overlaps_and_crossings`],
/// so a curve running *along* a boundary contributes that stretch's ends.
/// A boundary that isn't clamped (so its control row isn't the surface
/// there) contributes no candidates.
pub fn curve_surface_overlaps(
    curve: &NurbCurve<4>,
    surface: &NurbSurface<4>,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Vec<Overlap<Vector2>>> {
    let on_surface = |t: Scalar| point_on(curve, surface, t, max_nodes, min_subdivision_size);

    let mut candidates = Vec::new();
    let (t0, t1) = curve.domain();
    for t in [t0, t1] {
        if let Some(uv) = on_surface(t)? {
            candidates.push(Hit { t, partner: uv });
        }
    }

    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    for (u_fixed, first, fixed) in [
        (true, true, u0),
        (true, false, u1),
        (false, true, v0),
        (false, false, v1),
    ] {
        let Some(boundary) = boundary_curve(surface, u_fixed, first) else {
            continue;
        };
        let uv = |w: Scalar| Vector2::from_array(if u_fixed { [fixed, w] } else { [w, fixed] });
        let (overlaps, crossings) = curve_curve::curve_curve_overlaps_and_crossings::<4, 3>(
            curve,
            &boundary,
            max_nodes,
            min_subdivision_size,
        )?;
        let ends = overlaps
            .iter()
            .flat_map(|o| [o.start, o.end])
            .map(|h| (h.t, h.partner));
        for (t, w) in ends.chain(crossings) {
            candidates.push(Hit { t, partner: uv(w) });
        }
    }

    coincidence::find_overlaps(candidates, on_surface)
}

/// Points where `curve` crosses — or, lying on it along an arc, coincides
/// with — `surface`, under the [`Intersections`] contract.
///
/// Overlaps are found directly ([`curve_surface_overlaps`]) instead of being
/// inferred from a search hitting `max_solutions`. Without one, the result
/// is [`Intersections::Found`] with the clipping search's crossings (at most
/// `max_solutions`). With one, it is [`Intersections::Coincident`] with, in
/// this order and up to `max_solutions` in total: the overlaps' end points,
/// the isolated crossings on the rest of the curve, and points spread evenly
/// over the overlaps. Works on the untrimmed patch. Exhausting `max_nodes`
/// in any sub-search is an error.
pub fn curve_surface_intersect(
    curve: &NurbCurve<4>,
    surface: &NurbSurface<4>,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Intersections<(Scalar, Vector2)>> {
    // Disjoint bounding boxes rule out crossings and overlaps alike, before
    // any candidate probe runs — a straight segment's own extent, not its
    // box (see `curve_could_meet_aabb`).
    if max_solutions == 0 || !curve_could_meet_aabb(curve, &surface.aabb, 3) {
        return Ok(Intersections::Found(vec![]));
    }
    let ctx = |e: GeopError| {
        e.with_context(format!(
            "curve_surface_intersect(curve={curve:?}, surface={surface:?}, max_nodes={max_nodes}, \
             min_subdivision_size={min_subdivision_size:?})"
        ))
    };
    let overlaps = curve_surface_overlaps(curve, surface, max_nodes, min_subdivision_size)
        .with_context(&ctx)?;
    let crossings = coincidence::crossings_between(curve, &overlaps, |piece| {
        curve_surface_crossings(piece, surface, max_nodes, min_subdivision_size)
    })
    .with_context(&|e: GeopError| ctx(e.with_context("crossing search")))?;
    let samples = coincidence::samples(&overlaps, crossings.len(), max_solutions, |t| {
        point_on(curve, surface, t, max_nodes, min_subdivision_size)
    })?;
    Ok(coincidence::assemble(
        &overlaps,
        crossings,
        samples,
        max_solutions,
    ))
}

/// Polish one isolated `(t, uv)` — as returned by [`curve_surface_intersect`]
/// — by Newton on `C(t) - Scalar(u, v) = 0`, three equations in the three unknowns
/// `t`, `u`, `v` ([`newton::refine`]).
///
/// `curve_surface_intersect` deliberately does *not* apply this to
/// everything it returns — most callers only need to know where and how
/// many crossings there are, and refining changes results they already
/// agree with. Call it when the parameter is about to be *used* as a split
/// point, where the width genuinely matters: a point evaluated at a wide
/// crossing is just as wide, which is how it becomes a fat vertex and a fat
/// sub-curve.
///
/// The refined box must still be able to hold a crossing: the curve and
/// the surface evaluated over it have to overlap. Newton only assumes an
/// isolated, regular root; where that fails — a curve lying on the
/// surface's extension beyond the patch, so the iterate slides along it and
/// gets clamped at the patch's edge — it can return a narrow box that
/// provably misses, and the honest incoming box is what is kept.
pub fn refine_crossing(
    curve: &NurbCurve<4>,
    surface: &NurbSurface<4>,
    t: Scalar,
    uv: Vector2,
) -> (Scalar, Vector2) {
    let refined = newton::refine(
        [t, uv[0], uv[1]],
        [curve.domain(), surface.domain_u(), surface.domain_v()],
        |[tt, uu, vv]| {
            let c = curve.evaluate(tt).ok()?;
            let s = surface.evaluate(uu, vv).ok()?;
            let ct = curve.tangent(tt).ok()?;
            let (su, sv) = surface.derivatives(uu, vv).ok()?;
            let mut a = [[Scalar::ZERO; 3]; 3];
            let mut b = [Scalar::ZERO; 3];
            for row in 0..3 {
                a[row] = [ct[row], su[row].neg(), sv[row].neg()];
                b[row] = s[row].sub(c[row]);
            }
            let delta = solve_linear_system(&Matrix::from_rows(a), &Vector::from_array(b)).ok()?;
            Some([tt.add(delta[0]), uu.add(delta[1]), vv.add(delta[2])])
        },
    );
    let [t_ref, u_ref, v_ref] = refined;
    let holds_a_crossing = match (curve.evaluate(t_ref), surface.evaluate(u_ref, v_ref)) {
        (Ok(c), Ok(s)) => c.could_be_equal(&s),
        _ => false,
    };
    if !holds_a_crossing {
        return (t, uv);
    }
    (t_ref, Vector2::from_array([u_ref, v_ref]))
}

/// Every point where `curve` crosses `surface`, each refined
/// ([`refine_crossing`]) and with where it is on the surface: what the
/// curve and the surface give there intersected, both enclosures of one
/// point — at a pole, where the collapsed parameter says nothing, the pole.
/// A curve lying on the surface along an arc is refused: where it meets the
/// surface is no point.
pub fn crossing_points(
    curve: &NurbCurve<4>,
    surface: &NurbSurface<4>,
) -> GeopResult<Vec<(Vector<3>, Vector2)>> {
    let hits = match curve_surface_intersect(curve, surface, 8, MAX_NODES, min_subdivision_size())?
    {
        Intersections::Found(hits) => hits,
        Intersections::Coincident(_) => {
            return Err(GeopError::new(
                "the curve lies on the surface: where it crosses it is no point",
            ));
        }
    };
    let mut points = Vec::with_capacity(hits.len());
    for (t, uv) in hits {
        let (t, uv) = refine_crossing(curve, surface, t, uv);
        let ctx = |e: GeopError| {
            e.with_context(format!(
                "the curve crossing the surface at t={t:?}, uv={uv:?}, the surface's domain {:?} x {:?}",
                surface.domain_u(),
                surface.domain_v()
            ))
        };
        let on_curve = curve.evaluate(t).map_err(ctx)?;
        let on_surface = match surface.pole_at(uv[0], uv[1]) {
            Some(pole) => pole,
            None => surface.evaluate(uv[0], uv[1]).map_err(ctx)?,
        };
        if on_curve.could_be_equal(&on_surface) {
            let point = Vector::from_array(std::array::from_fn(|k| {
                on_curve[k].intersect(on_surface[k])
            }));
            points.push((point, uv));
        }
    }
    Ok(points)
}

#[cfg(test)]
mod tests {
    use super::curve_surface_crossings;
    use crate::test_fixtures::{knots, line, sphere_octant, weighted};
    use crate::{nurb_curve::NurbCurve, nurb_surface::NurbSurface};
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    const MAX: usize = 5000;
    const MIN_SUBDIVISION_SIZE: f64 = 1e-6;

    /// The unit square `Scalar(u, v) = (u, v, 0)`.
    fn plane() -> NurbSurface<4> {
        NurbSurface::try_new(
            1,
            1,
            vec![
                weighted(0., 0., 0., 1.),
                weighted(0., 1., 0., 1.),
                weighted(1., 0., 0., 1.),
                weighted(1., 1., 0., 1.),
            ],
            knots(&[0., 0., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    fn solve(c: &NurbCurve<4>, s: &NurbSurface<4>) -> Vec<(Scalar, [Scalar; 2])> {
        curve_surface_crossings(c, s, MAX, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
            .unwrap()
            .into_iter()
            .map(|(t, uv)| (t, [uv[0], uv[1]]))
            .collect()
    }

    /// `curve_surface.md` §3's example: the clips isolate the root directly.
    #[test]
    fn vertical_line_through_plane() {
        let sols = solve(&line([0.25, 0.75, -1.], [0.25, 0.75, 1.]), &plane());
        assert_eq!(sols.len(), 1, "{sols:?}");
        let (t, [u, v]) = sols[0];
        let f = Scalar::from_f64;
        assert!(t.could_be_equal(f(0.5)) && u.could_be_equal(f(0.25)) && v.could_be_equal(f(0.75)));
    }

    #[test]
    fn oblique_line_through_plane() {
        let sols = solve(&line([0., 0., -1.], [1., 0.5, 1.]), &plane());
        assert_eq!(sols.len(), 1, "{sols:?}");
        let (t, [u, v]) = sols[0];
        let f = Scalar::from_f64;
        assert!(t.could_be_equal(f(0.5)) && u.could_be_equal(f(0.5)) && v.could_be_equal(f(0.25)));
    }

    /// A line through the sphere's center pierces the octant once, at
    /// `(1, 1, 1) / √3`.
    #[test]
    fn line_pierces_sphere() {
        let sols = solve(&line([0., 0., 0.], [1., 1., 1.]), &sphere_octant());
        assert_eq!(sols.len(), 1, "{sols:?}");
        let r = 1.0 / 3f64.sqrt();
        assert!(sols[0].0.could_be_equal(Scalar::from_f64(r)), "{sols:?}");
    }

    /// Ending exactly on the surface — a curve meeting a face at its vertex.
    #[test]
    fn endpoint_on_surface() {
        let sols = solve(&line([0.3, 0.6, 1.], [0.3, 0.6, 0.]), &plane());
        assert_eq!(sols.len(), 1, "{sols:?}");
        assert!(sols[0].0.could_be_equal(Scalar::ONE), "{sols:?}");
    }

    /// Through the pole: every `u` is a preimage.
    #[test]
    fn line_through_pole() {
        let sols = solve(&line([0., 0., 0.5], [0., 0., 1.5]), &sphere_octant());
        assert!(!sols.is_empty(), "{sols:?}");
        for (t, [_, v]) in &sols {
            assert!(
                t.could_be_equal(Scalar::from_f64(0.5)) && v.could_be_equal(Scalar::ONE),
                "{sols:?}"
            );
        }
    }

    #[test]
    fn misses() {
        assert!(solve(&line([0., 0., 0.1], [1., 1., 0.5]), &plane()).is_empty());
        assert!(solve(&line([0., 0., 0.], [0.5, 0.5, 0.5]), &sphere_octant()).is_empty());
        assert!(solve(&line([2., 2., -1.], [2., 2., 1.]), &plane()).is_empty());
    }

    #[test]
    fn budget_exhaustion_is_an_error() {
        let c = line([0., 0., -1.], [1., 0.5, 1.]);
        assert!(
            curve_surface_crossings(&c, &plane(), 1, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                .is_err()
        );
    }

    // ── The coincidence-handling wrapper ─────────────────────────────────────

    use super::curve_surface_intersect;
    use crate::intersection::Intersections;
    use geop_core_math::vector::Vector2;

    fn wrap(c: &NurbCurve<4>, s: &NurbSurface<4>) -> Intersections<(Scalar, Vector2)> {
        curve_surface_intersect(c, s, 5, MAX, Scalar::from_f64(MIN_SUBDIVISION_SIZE)).unwrap()
    }

    /// The equator lies on the sphere octant's `v = 0` boundary.
    #[test]
    fn equator_on_sphere_is_coincident() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let equator = NurbCurve::try_new(
            2,
            vec![
                weighted(1., 0., 0., 1.),
                weighted(1., 1., 0., w),
                weighted(0., 1., 0., 1.),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        let r = wrap(&equator, &sphere_octant());
        assert!(r.is_coincident() && r.len() == 5, "{r:?}");
        for (_, uv) in r.as_slice() {
            assert!(uv[1].could_be_equal(Scalar::ZERO), "{r:?}");
        }
    }

    /// An edge running along the face's `v = 0` edge.
    #[test]
    fn curve_along_patch_boundary() {
        let r = wrap(&line([0.2, 0., 0.], [0.8, 0., 0.]), &plane());
        assert!(r.is_coincident(), "{r:?}");
        for (_, uv) in r.as_slice() {
            assert!(uv[1].could_be_equal(Scalar::ZERO), "{r:?}");
        }
    }

    /// Enters the patch through `u = 0` at t = ½ and lies on it to its end:
    /// those are the overlap's ends, reported first.
    #[test]
    fn line_partly_on_plane() {
        let r = wrap(&line([-0.5, 0.5, 0.], [0.5, 0.5, 0.]), &plane());
        assert!(r.is_coincident(), "{r:?}");
        let v = r.as_slice();
        let half = Scalar::from_f64(0.5);
        assert!(
            v[0].0.could_be_equal(half) && v[0].1[0].could_be_equal(Scalar::ZERO),
            "{v:?}"
        );
        assert!(
            v[1].0.could_be_equal(Scalar::ONE) && v[1].1[0].could_be_equal(half),
            "{v:?}"
        );
        for (t, _) in v {
            assert!(!t.definitely_less(half), "{v:?}");
        }
    }

    /// A line lying on the plane, asked for three points: its two ends and,
    /// of the points spread over it, the one there is room for — its
    /// middle, the point furthest from both ends, not the first of three
    /// spread a quarter apart. A boolean asks whether an edge enters a face
    /// at these points; a blend's contact 7.6e-7 long was judged by its
    /// ends and a point a quarter along, all within a pcurve's width of the
    /// boundary, and never imprinted
    /// (`rounding_the_trailing_edge_of_a_lofted_blade` in the fillet).
    #[test]
    fn the_point_room_is_left_for_is_the_middle_of_an_overlap() {
        let r = curve_surface_intersect(
            &line([0.5, 0.2, 0.], [0.5, 0.8, 0.]),
            &plane(),
            3,
            MAX,
            Scalar::from_f64(1e-4),
        )
        .unwrap();
        assert!(r.is_coincident() && r.len() == 3, "{r:?}");
        let middle = |t: &Scalar| (t.to_f64() - 0.5).abs() < 1e-12;
        assert!(r.as_slice().iter().any(|(t, _)| middle(t)), "{r:?}");
    }

    /// Both ends on the plane, the middle above it: two crossings, not an
    /// overlap.
    #[test]
    fn bump_with_ends_on_plane_is_not_coincident() {
        let bump = NurbCurve::try_new(
            2,
            vec![
                weighted(0.2, 0.5, 0., 1.),
                weighted(0.5, 0.5, 0.6, 1.),
                weighted(0.8, 0.5, 0., 1.),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        let r = wrap(&bump, &plane());
        assert!(!r.is_coincident() && r.len() == 2, "{r:?}");
    }

    /// A revolved cylinder's top cap: the spoke of one quadrant against the
    /// neighbouring quadrant's patch, which collapses to the centre along
    /// its whole `v = 0` row. Coplanar, touching only at that pole: every
    /// clip direction built from the patch's plane is its normal, and every
    /// `u`-piece contains the pole. Exhausted the node budget before the
    /// equations were required to span space and bisection went by spatial
    /// extent.
    #[test]
    fn spoke_touching_cap_quadrant_at_its_pole() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pole = weighted(0., 0., 2., 1.);
        let quadrant = NurbSurface::try_new(
            2,
            1,
            vec![
                pole,
                weighted(1., 0., 2., 1.),
                pole,
                weighted(1., -1., 2., w),
                pole,
                weighted(0., -1., 2., 1.),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        let spoke = line([0., 1., 2.], [0., 0., 2.]);
        let r =
            curve_surface_intersect(&spoke, &quadrant, 7, 5000, Scalar::from_f64(1e-4)).unwrap();
        assert!(!r.is_coincident() && !r.is_empty(), "{r:?}");
        for (t, uv) in r.as_slice() {
            assert!(
                t.could_be_equal(Scalar::ONE) && uv[1].could_be_equal(Scalar::ZERO),
                "{r:?}"
            );
        }
    }

    /// The same far from the origin, in the layout a revolve gives a cap:
    /// the bottom cap of `revolved_cylinder_far_from_the_origin_is_valid`'s
    /// `Y` case, radius 3 in the plane `y = -500` around `(1000, -500, 50)`,
    /// rim along `v = 0` and pole along `v = 1`, its homogeneous coordinates
    /// rounded as the revolve rounds them (`w x`). The
    /// spoke runs on from the quadrant's `u = 0` side through the pole.
    ///
    /// In fixed point, the weight's rounding times 1000 makes the patch carry
    /// a width of about 5e-7, and clipping stalls near the pole with the
    /// spoke and `v` at `min_subdivision_size` but, at a speed of 3, still
    /// wider in space than it. Bisection then split `u`, whose extent was
    /// already below that carried width, into thousands of pieces each still
    /// containing the pole, and exhausted the node budget (see
    /// `fat_line::stalled`).
    #[test]
    fn spoke_touching_cap_quadrant_at_its_pole_far_from_the_origin() {
        let f = Scalar::from_f64;
        let w = f(std::f64::consts::FRAC_1_SQRT_2);
        let at = |x: f64, z: f64, w: Scalar| {
            let p = [x, -500., z].map(|c| f(c).mul(w));
            Vector4::from_array([p[0], p[1], p[2], w])
        };
        let quadrant = NurbSurface::try_new(
            2,
            1,
            vec![
                at(1000., 53., Scalar::ONE),
                at(1000., 50., Scalar::ONE),
                at(997., 53., w),
                at(1000., 50., w),
                at(997., 50., Scalar::ONE),
                at(1000., 50., Scalar::ONE),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        let spoke = line([1000., -500., 47.], [1000., -500., 50.]);
        let r =
            curve_surface_intersect(&spoke, &quadrant, 7, 5000, Scalar::from_f64(1e-4)).unwrap();
        assert!(!r.is_coincident() && !r.is_empty(), "{r:?}");
        for (t, uv) in r.as_slice() {
            assert!(
                t.could_be_equal(Scalar::ONE) && uv[1].could_be_equal(Scalar::ONE),
                "{r:?}"
            );
        }
    }

    /// A blended bar's cap, 461 from the origin, as the press handle's
    /// rounded rim leaves it: the spoke of the cap's shrunken quarter, 2 long
    /// over the domain `[0, 2/3]`, against the neighbouring quadrant of
    /// radius 3, whose pole is its `v = 0` row, which the spoke runs on from
    /// along `u = 1`. The enclosures are the ones the blend built.
    #[test]
    fn blended_spoke_touching_cap_quadrant_at_its_pole() {
        let i = |lo: f64, hi: f64| Scalar::from_f64(lo).union(Scalar::from_f64(hi));
        let x = i(460.99999999999983, 461.00000000000017);
        let tiny = |e: f64| i(-e, e);
        let w = i(0.7071067811865475, 0.7071067811865477);
        let centre = Vector4::from_array([
            x,
            tiny(6.675221575521606e-308),
            tiny(6.675221575521606e-308),
            Scalar::ONE,
        ]);
        let quadrant = NurbSurface::try_new(
            2,
            1,
            vec![
                centre,
                Vector4::from_array([
                    x,
                    tiny(1.7800590868057623e-307),
                    i(-3.0000000000000027, -2.999999999999998),
                    Scalar::ONE,
                ]),
                Vector4::from_array([
                    i(325.97622612699826, 325.9762261269986),
                    tiny(4.720094441974079e-308),
                    tiny(4.720094441974079e-308),
                    w,
                ]),
                Vector4::from_array([
                    i(325.97622612699826, 325.9762261269986),
                    i(-2.121320343559646, -2.1213203435596406),
                    i(-2.121320343559646, -2.1213203435596406),
                    w,
                ]),
                centre,
                Vector4::from_array([
                    x,
                    i(-3.0000000000000027, -2.999999999999998),
                    tiny(1.7800590868057623e-307),
                    Scalar::ONE,
                ]),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        let two_thirds = i(0.6666666666619685, 0.6666666666713679);
        let spoke = NurbCurve::try_new(
            1,
            vec![
                centre,
                Vector4::from_array([
                    i(460.99999999999955, 461.00000000000045),
                    i(1.9999999999859022, 2.000000000014108),
                    tiny(1.4092134437327347e-307),
                    i(0.9999999999999999, 1.0000000000000002),
                ]),
            ],
            vec![Scalar::ZERO, Scalar::ZERO, two_thirds, two_thirds],
        )
        .unwrap();
        let r =
            curve_surface_intersect(&spoke, &quadrant, 7, 5000, Scalar::from_f64(1e-4)).unwrap();
        assert!(!r.is_coincident() && !r.is_empty(), "{r:?}");
        for (t, uv) in r.as_slice() {
            assert!(
                t.could_be_equal(Scalar::ZERO) && uv[1].could_be_equal(Scalar::ZERO),
                "{r:?}"
            );
        }
    }

    /// The test suite of the old `curve_surface` search, run unchanged against the
    /// coincidence-handling wrapper — the drop-in contract it must keep.
    mod old_suite {
        use super::super::curve_surface_intersect;
        use crate::{nurb_curve::NurbCurve, nurb_surface::NurbSurface};
        use geop_core_math::{scalars::Scalar, vector::Vector4};

        const MAX_NODES: usize = 2000;

        fn ptc(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
            Vector4::from_array([
                Scalar::from_f64(x),
                Scalar::from_f64(y),
                Scalar::from_f64(z),
                Scalar::from_f64(w),
            ])
        }

        fn pts(x: f64, y: f64, z: f64) -> Vector4 {
            Vector4::from_array([
                Scalar::from_f64(x),
                Scalar::from_f64(y),
                Scalar::from_f64(z),
                Scalar::ONE,
            ])
        }

        /// Flat unit patch in the xy-plane (z = 0), x,y ∈ [0,1].
        fn flat_xy() -> NurbSurface<4> {
            let f = Scalar::from_f64;
            NurbSurface::try_new(
                1,
                1,
                vec![
                    pts(0., 0., 0.),
                    pts(0., 1., 0.),
                    pts(1., 0., 0.),
                    pts(1., 1., 0.),
                ],
                vec![f(0.), f(0.), f(1.), f(1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Straight line crossing `flat_xy` once at (0.5, 0.5, 0).
        fn vertical_crossing_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.5, 0.5, -1., 1.), ptc(0.5, 0.5, 1., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Straight line entirely above `flat_xy` (z ∈ [1, 2]) — never crosses.
        fn line_above_surface() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.5, 0.5, 1., 1.), ptc(0.5, 0.5, 2., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Quadratic Bézier dipping below z=0 and back, crossing `flat_xy` twice.
        /// y = 0.3 is deliberately not the midpoint of flat_xy's y range [0,1],
        /// avoiding the "both halves always survive" tie pathology that exact
        /// midpoints trigger (see `crossing_vyz` in `surface_surface.rs`).
        fn double_dip_curve() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                2,
                vec![
                    ptc(0.2, 0.3, 1.0, 1.),
                    ptc(0.5, 0.3, -2.0, 1.),
                    ptc(0.8, 0.3, 1.0, 1.),
                ],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Straight line lying *in* the `flat_xy` plane (z = 0), spanning part of
        /// its footprint: from (0.2, 0.5, 0) to (0.8, 0.5, 0).
        fn coplanar_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.2, 0.5, 0., 1.), ptc(0.8, 0.5, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Coplanar line spanning x ∈ [-0.5, 0.5] at y=0.5 — only the x ∈ [0, 0.5]
        /// half (t ∈ [0.5, 1]) overlaps `flat_xy`'s footprint (x,y ∈ [0,1]).
        fn partial_overlap_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(-0.5, 0.5, 0., 1.), ptc(0.5, 0.5, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Coplanar line spanning x ∈ [-1, 2] at y=0.5 — much larger than
        /// `flat_xy`'s x-extent [0,1], extending beyond it on both sides. Only
        /// x ∈ [0,1] (t ∈ [1/3, 2/3]) overlaps the surface.
        fn oversized_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(-1.0, 0.5, 0., 1.), ptc(2.0, 0.5, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Coplanar line spanning x ∈ [0,1] at y=0.5 — exactly matches
        /// `flat_xy`'s x-extent.
        fn full_width_line() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0., 0.5, 0., 1.), ptc(1., 0.5, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Homogeneous control point with weight `w`, given its Cartesian
        /// position `(x, y, z)`.
        fn ptw(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
            Vector4::from_array([
                Scalar::from_f64(x * w),
                Scalar::from_f64(y * w),
                Scalar::from_f64(z * w),
                Scalar::from_f64(w),
            ])
        }

        /// Non-planar bilinear "saddle" patch: corner heights 0,1,1,0 over
        /// x,y ∈ [0,2]. Its v=0.5 ridge line sits at z = 0.5.
        fn bent_surface() -> NurbSurface<4> {
            let f = Scalar::from_f64;
            NurbSurface::try_new(
                1,
                1,
                vec![
                    pts(0., 0., 0.),
                    pts(0., 2., 1.),
                    pts(2., 0., 1.),
                    pts(2., 2., 0.),
                ],
                vec![f(0.), f(0.), f(1.), f(1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Quadratic Bézier running along `bent_surface`'s ridge line (y = 1),
        /// dipping from z=-1 up to z=3 and back to z=-1 -- crossing the ridge's
        /// z=0.5 height at two distinct points (t ≈ 0.25 and t ≈ 0.75).
        fn bent_curve() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            NurbCurve::try_new(
                2,
                vec![
                    ptc(0.2, 1.0, -1.0, 1.),
                    ptc(1.0, 1.0, 3.0, 1.),
                    ptc(1.8, 1.0, -1.0, 1.),
                ],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Degree-(2,2) rational patch covering one octant of the unit sphere
        /// (x,y,z >= 0), built by revolving a quarter-circle meridian (in the
        /// xz-plane) by a quarter turn around the z axis. Its v=0 edge is exactly
        /// `equator_quarter_circle`.
        fn sphere_octant_patch() -> NurbSurface<4> {
            let f = Scalar::from_f64;
            let w = 1.0 / 2.0_f64.sqrt();
            NurbSurface::try_new(
                2,
                2,
                vec![
                    // u = 0 (azimuth 0deg)
                    ptw(1., 0., 0., 1.),
                    ptw(1., 0., 1., w),
                    ptw(0., 0., 1., 1.),
                    // u = 1 (azimuth 45deg)
                    ptw(1., 1., 0., w),
                    ptw(1., 1., 1., 0.5),
                    ptw(0., 0., 1., w),
                    // u = 2 (azimuth 90deg)
                    ptw(0., 1., 0., 1.),
                    ptw(0., 1., 1., w),
                    ptw(0., 0., 1., 1.),
                ],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Quarter circle from (1,0,0) to (0,1,0) in the xy-plane -- exactly the
        /// v=0 edge of `sphere_octant_patch`, i.e. coincident with that surface.
        fn equator_quarter_circle() -> NurbCurve<4> {
            let f = Scalar::from_f64;
            let w = 1.0 / 2.0_f64.sqrt();
            NurbCurve::try_new(
                2,
                vec![ptw(1., 0., 0., 1.), ptw(1., 1., 0., w), ptw(0., 1., 0., 1.)],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            )
            .unwrap()
        }

        const MIN_SUBDIVISION_SIZE: f64 = 1e-2;

        // ── Single crossing ───────────────────────────────────────────────────────

        #[test]
        fn single_crossing_curve_has_one_solution() {
            let curve = vertical_crossing_line();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                5,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert_eq!(result.len(), 1);
        }

        // ── No crossing ───────────────────────────────────────────────────────────

        #[test]
        fn curve_missing_surface_has_no_solution() {
            let curve = line_above_surface();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                5,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert!(result.is_empty());
        }

        // ── Budget ────────────────────────────────────────────────────────────────

        #[test]
        fn max_solutions_zero_returns_empty() {
            let curve = vertical_crossing_line();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                0,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert!(result.is_empty());
        }

        #[test]
        fn max_nodes_exhausted_errors() {
            // Adapted: the old search could only see coincidence as endless
            // subdivision, so a coincident pair was the way to overrun a tiny
            // budget. The wrapper resolves that pair directly (and correctly,
            // as `Coincident`) within it, so the invariant — running out of
            // budget is an error, never a truncated result — is exercised on
            // a two-crossing pair that genuinely needs more nodes than that.
            let curve = double_dip_curve();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                1000,
                1,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            );
            assert!(result.is_err());
        }

        // ── Two crossings ─────────────────────────────────────────────────────────

        #[test]
        fn two_crossings_found_when_budget_allows() {
            let curve = double_dip_curve();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                2,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap()
            .into_vec();
            assert_eq!(result.len(), 2);
            assert!(
                !result[0].0.could_be_equal(result[1].0),
                "the two crossings should remain distinct after unification"
            );
        }

        #[test]
        fn max_solutions_one_caps_at_one_even_with_two_crossings() {
            let curve = double_dip_curve();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                1,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert_eq!(result.len(), 1);
        }

        // ── min_subdivision_size controls precision ────────────────────────────

        #[test]
        fn min_subdivision_size_controls_precision() {
            let curve = vertical_crossing_line();
            let surf = flat_xy();
            let result =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, Scalar::from_f64(1e-3))
                    .unwrap()
                    .into_vec();
            assert_eq!(result.len(), 1);

            let (_, uv) = result[0];
            assert!(
                uv[0]
                    .sub(Scalar::from_f64(0.5))
                    .abs()
                    .could_be_less(Scalar::from_f64(1e-2))
            );
            assert!(
                uv[1]
                    .sub(Scalar::from_f64(0.5))
                    .abs()
                    .could_be_less(Scalar::from_f64(1e-2))
            );
        }

        // ── Coplanar curve: must terminate ───────────────────────────────────────

        #[test]
        fn coplanar_curve_terminates() {
            let curve = coplanar_line();
            let surf = flat_xy();

            // A single dive must converge to exactly one result.
            let result_one = curve_surface_intersect(
                &curve,
                &surf,
                1,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert_eq!(result_one.len(), 1);

            // Asking for more solutions still terminates, with at most that many
            // (possibly fewer after unification) segments along the coplanar overlap.
            let result_many = curve_surface_intersect(
                &curve,
                &surf,
                5,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);
        }

        // ── Coincident: an evenly-spread solution count, not just 1-or-cap ──────

        #[test]
        fn coincident_curve_reaches_max_solutions() {
            let curve = full_width_line();
            let surf = flat_xy();
            // A curve running the *entire* width of the surface it's coincident
            // with, with a generous node budget, should reliably reach the
            // requested solution count via the evenly-spread search — this is
            // exactly the property `max_solutions` saturating is meant to
            // signal "coincident" to a caller in the first place.
            let result =
                curve_surface_intersect(&curve, &surf, 5, 5000, Scalar::from_f64(1e-3)).unwrap();
            assert!(result.is_coincident());
            assert_eq!(result.len(), 5);
        }

        // ── Partial overlap: only part of the curve lies over the surface ───────

        #[test]
        fn partial_overlap_coplanar_line() {
            let curve = partial_overlap_line();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                5,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert!(!result.is_empty());
            assert!(result.len() <= 5);

            // Every solution must lie within the overlapping half of the curve
            // (x >= 0, i.e. t >= 0.5), up to a small tolerance.
            let lower_bound = Scalar::from_f64(0.5 - MIN_SUBDIVISION_SIZE);
            for &(t, _) in result.as_slice() {
                assert!(!t.definitely_less(lower_bound));
            }
        }

        // ── Curve much larger than the surface ───────────────────────────────────

        #[test]
        fn curve_larger_than_surface_terminates() {
            let curve = oversized_line();
            let surf = flat_xy();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                5,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert!(!result.is_empty());
            assert!(result.len() <= 5);

            // Every solution must lie within the overlapping middle third of the
            // curve (x ∈ [0,1], i.e. t ∈ [1/3, 2/3]), up to a small tolerance.
            let lower_bound = Scalar::from_f64(1.0 / 3.0 - MIN_SUBDIVISION_SIZE);
            let upper_bound = Scalar::from_f64(2.0 / 3.0 + MIN_SUBDIVISION_SIZE);
            for &(t, _) in result.as_slice() {
                assert!(!t.definitely_less(lower_bound));
                assert!(!t.definitely_greater(upper_bound));
            }
        }

        // ── Curve exactly the same size as the surface ──────────────────────────

        #[test]
        fn curve_same_size_as_surface_terminates() {
            let curve = full_width_line();
            let surf = flat_xy();

            let result_one = curve_surface_intersect(
                &curve,
                &surf,
                1,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert_eq!(result_one.len(), 1);

            let result_many = curve_surface_intersect(
                &curve,
                &surf,
                5,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);
        }

        // ── Bent surface / bent curve: distinct crossings ────────────────────────

        #[test]
        fn bent_surface_bent_curve_two_distinct_crossings() {
            let curve = bent_curve();
            let surf = bent_surface();
            let result = curve_surface_intersect(
                &curve,
                &surf,
                4,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap()
            .into_vec();
            assert_eq!(result.len(), 2);
            assert!(
                !result[0].0.could_be_equal(result[1].0),
                "the two crossings of a bent curve through a bent surface should be distinct"
            );
        }

        // ── Coincident circle on a spherical patch: must terminate ───────────────

        #[test]
        fn coincident_circle_on_sphere_patch_terminates() {
            let curve = equator_quarter_circle();
            let surf = sphere_octant_patch();

            let result_one = curve_surface_intersect(
                &curve,
                &surf,
                1,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert_eq!(result_one.len(), 1);

            let result_many = curve_surface_intersect(
                &curve,
                &surf,
                5,
                MAX_NODES,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE),
            )
            .unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);
        }
    }
}
