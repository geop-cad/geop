//! Curve–surface intersection by per-axis fat line clipping — the design is
//! `curve_surface.md` next to this file; it is `curve_curve` with one more
//! parameter.
//!
//! For a curve `C = H_C / W_C` and a surface `S = H_S / W_S` with positive
//! weights, `C_k(t) = S_k(u, v)` iff
//! `g_k(t, u, v) = H_{C,k}(t) W_S(u, v) - W_C(t) H_{S,k}(u, v) = 0`, a
//! polynomial tensor-product spline in three independent parameters with
//! coefficients `d_ijl = P_{i,k} Q_{jl,w} - P_{i,w} Q_{jl,k}`. Its zeros are
//! clipped in all three directions ([`clip_tensor`]) — nine clips per box.
//!
//! Assumes no arc of the curve lies on the surface (`curve_surface.md`): the
//! result is a list of paired `(t, (u, v))` boxes, and reaching some count
//! of them means nothing. Works on the untrimmed patch.

use std::collections::VecDeque;

use super::{
    Intersections,
    coincidence::{self, Hit, Overlap},
};
use crate::nurb_surface::clamp;
use crate::{
    aabb::aabb_could_overlap,
    contains::surface::{
        boundary_curve, collapsed_boundary, surface_could_contain, surface_extents,
        weights_positive,
    },
    fat_line::{
        Plan, carried_width, clip_tensor, converged, directions, extent, greville_abscissae, plan,
    },
    intersection::curve_curve,
    knot_insertion::pinned_clamped_end,
    nurb_curve::{NurbCurve, dehomogenize},
    nurb_surface::NurbSurface,
};
use geop_core_math::{
    disjoint_set::DisjointSet,
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, solve_linear_system},
    scalars::Scalar,
    vector::{Vector, Vector2},
};

/// Clip the pair: a box `[t̂, û, v̂]` inside the domains enclosing every
/// `(t, u, v)` with `C(t) = S(u, v)`, or `None` if some equation proves
/// there is none. The curve's weights are positive by construction; the
/// surface's are checked (`curve_surface.md` §1) — without them the whole
/// box is returned (no information) and the search subdivides.
///
/// The equations are combinations `g_n = n · (H_C W_S - W_C H_S)` along
/// free-choice directions ([`directions`]), from the curve's chord `c` and
/// the patch's mean edge directions `e_u`, `e_v`: the patch normal
/// `e_u × e_v` (nearly independent of `u, v`, so it pins `t`), `c × e_v`
/// (pins `u`) and `c × e_u` (pins `v`). If they don't span space — a curve
/// without a chord, or lying in the patch's plane, which makes all three
/// parallel — the coordinate axes are added.
fn clip<S: Scalar>(c: &NurbCurve<S, 4>, s: &NurbSurface<S, 4>) -> GeopResult<Option<[S; 3]>> {
    let (u0, u1) = s.domain_u();
    let (v0, v1) = s.domain_v();
    let mut hats = [c.domain_as_scalar(), u0.union(u1), v0.union(v1)];
    if !weights_positive(s) {
        return Ok(Some(hats));
    }
    let (nc, nu, nv) = (c.control_points.len(), s.num_u, s.num_v);
    let greville = [
        greville_abscissae(&c.knot_vector, c.degree, nc)?,
        greville_abscissae(&s.knot_vector_u, s.degree_u, nu)?,
        greville_abscissae(&s.knot_vector_v, s.degree_v, nv)?,
    ];

    let at = |i: usize, j: usize| directions::cartesian::<S, 4, 3>(&s.control_points[i * nv + j]);
    let add = |a: [f64; 3], b: [f64; 3]| std::array::from_fn(|k| a[k] + b[k]);
    let chord = directions::sub(
        directions::cartesian::<S, 4, 3>(&c.control_points[nc - 1]),
        directions::cartesian::<S, 4, 3>(&c.control_points[0]),
    );
    let e_u = directions::sub(
        add(at(nu - 1, 0), at(nu - 1, nv - 1)),
        add(at(0, 0), at(0, nv - 1)),
    );
    let e_v = directions::sub(
        add(at(0, nv - 1), at(nu - 1, nv - 1)),
        add(at(0, 0), at(nu - 1, 0)),
    );
    let dirs: Vec<[f64; 3]> = [
        directions::cross(e_u, e_v),
        directions::cross(chord, e_v),
        directions::cross(chord, e_u),
    ]
    .into_iter()
    .filter_map(directions::unit)
    .collect();
    let dirs = directions::spanning(dirs);

    let mut d = Vec::with_capacity(nc * nu * nv);
    for n in dirs {
        let n = directions::sharp::<S, 3>(n);
        let ns: Vec<S> = s
            .control_points
            .iter()
            .map(|q| directions::dot(&n, q))
            .collect();
        d.clear();
        for p in &c.control_points {
            let np = directions::dot(&n, p);
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
fn solve_pinned<S: Scalar>(
    c: &NurbCurve<S, 4>,
    s: &NurbSurface<S, 4>,
    hats: [S; 3],
    budget: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<Vec<[S; 3]>>> {
    let overlapping = |found: [S; 3]| -> Option<[S; 3]> {
        (0..3)
            .all(|i| found[i].could_be_equal(hats[i]))
            .then(|| std::array::from_fn(|i| hats[i].intersect(found[i])))
    };

    if let Some(first) =
        pinned_clamped_end(hats[0], &c.knot_vector, c.control_points.len(), c.degree)
    {
        let cp = if first {
            c.control_points[0]
        } else {
            c.control_points[c.control_points.len() - 1]
        };
        let Ok(end) = dehomogenize::<S, 4, 3>(&[cp]) else {
            return Ok(None);
        };
        let found = surface_could_contain(s, &end[0], budget, min_subdivision_size)?;
        return Ok(Some(
            found
                .and_then(|(u, v)| overlapping([hats[0], u, v]))
                .into_iter()
                .collect(),
        ));
    }

    if let Some((boundary, along_v)) = collapsed_boundary(s, hats[1], hats[2]) {
        let pairs = curve_curve::curve_curve_crossings::<S, 4, 3>(
            c,
            &boundary,
            budget,
            min_subdivision_size,
        )?;
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
/// pairs with three parameter directions: AABB and [`clip`] rejection,
/// convergence once both objects' extents are within `min_subdivision_size`
/// ([`crate::fat_line::converged`]), a pinned parameter handed down one dimension
/// ([`solve_pinned`]), and otherwise restriction or fair bisection per
/// [`crate::fat_line::plan`].
///
/// Overlapping boxes merge by union into one unresolved cluster, never an
/// average. Exhausting `max_nodes` is an error — the result would be
/// incomplete.
pub fn curve_surface_crossings<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surface: &NurbSurface<S, 4>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Vec<(S, Vector2<S>)>> {
    let mut queue: VecDeque<(NurbCurve<S, 4>, NurbSurface<S, 4>)> = VecDeque::new();
    queue.push_back((curve.clone(), surface.clone()));
    let mut explored = 0usize;
    let mut solutions: DisjointSet<(S, Vector2<S>)> = DisjointSet::new();
    let mut insert = |[t, u, v]: [S; 3]| solutions.insert((t, Vector2::from_array([u, v])));

    while let Some((c, s)) = queue.pop_front() {
        if explored >= max_nodes {
            return Err(GeopError::new(format!(
                "curve_surface_crossings: exhausted max_nodes={max_nodes} with {} \
                 pairs pending; the result would be incomplete",
                queue.len() + 1
            )));
        }
        explored += 1;

        if !aabb_could_overlap(&c.aabb, &s.aabb, 3) {
            continue;
        }
        let Some(hats) = clip(&c, &s)? else {
            continue;
        };

        let [size_u, size_v] = surface_extents(&s);
        let sizes = [extent([c.control_points.clone()]), size_u, size_v];
        let carried = carried_width(&c.control_points).max(carried_width(&s.control_points));
        if converged(&sizes, carried, min_subdivision_size) {
            // The pieces' whole domains, not the tighter clip, so pieces the
            // search could not separate merge — see `curve_curve_crossings`.
            let (u0, u1) = s.domain_u();
            let (v0, v1) = s.domain_v();
            insert([c.domain_as_scalar(), u0.union(u1), v0.union(v1)]);
            continue;
        }

        if let Some(found) = solve_pinned(&c, &s, hats, max_nodes - explored, min_subdivision_size)?
        {
            found.into_iter().for_each(&mut insert);
            continue;
        }

        let ranges = [c.domain(), s.domain_u(), s.domain_v()];
        let order = match plan(&hats, &ranges, &sizes, min_subdivision_size)? {
            Plan::Restrict(b) => match (c.sub_curve(b[0].0, b[0].1), s.sub_surface(b[1], b[2])) {
                (Ok(rc), Ok(rs)) => {
                    queue.push_back((rc, rs));
                    continue;
                }
                _ => vec![0, 1, 2],
            },
            Plan::Bisect(order) => order,
        };
        let children = order.iter().find_map(|&dir| match dir {
            0 => {
                let (l, r) = c.split_mid().ok()?;
                Some([(l, s.clone()), (r, s.clone())])
            }
            1 => {
                let (l, r) = s.split_u_mid().ok()?;
                Some([(c.clone(), l), (c.clone(), r)])
            }
            _ => {
                let (l, r) = s.split_v_mid().ok()?;
                Some([(c.clone(), l), (c.clone(), r)])
            }
        });
        match children {
            Some(children) => queue.extend(children),
            // Nothing left to cut or split: what's here is the candidate.
            None => insert(hats),
        }
    }

    Ok(solutions.into_vec())
}

/// Every stretch of `curve` lying on `surface` (see [`coincidence`]), with
/// the surface parameters as the partner. Candidates are the curve's ends
/// found on the surface, and where the curve meets the patch's four
/// boundary curves — through [`curve_curve::curve_curve_overlaps_and_crossings`],
/// so a curve running *along* a boundary contributes that stretch's ends.
/// A boundary that isn't clamped (so its control row isn't the surface
/// there) contributes no candidates.
pub(crate) fn curve_surface_overlaps<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surface: &NurbSurface<S, 4>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Vec<Overlap<S, Vector2<S>>>> {
    let on_surface = |t: S| -> GeopResult<Option<Vector2<S>>> {
        let point = curve.evaluate(t)?;
        Ok(
            surface_could_contain(surface, &point, max_nodes, min_subdivision_size)?
                .map(|(u, v)| Vector2::from_array([u, v])),
        )
    };

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
        let uv = |w: S| Vector2::from_array(if u_fixed { [fixed, w] } else { [w, fixed] });
        let (overlaps, crossings) = curve_curve::curve_curve_overlaps_and_crossings::<S, 4, 3>(
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
/// with — `surface`: the drop-in counterpart of
/// [`super::curve_surface_bisect::curve_surface_intersect`], with the same
/// signature and [`Intersections`] contract.
///
/// Overlaps are found directly ([`curve_surface_overlaps`]) instead of being
/// inferred from a search hitting `max_solutions`. Without one, the result
/// is [`Intersections::Found`] with the clipping search's crossings (at most
/// `max_solutions`). With one, it is [`Intersections::Coincident`] with, in
/// this order and up to `max_solutions` in total: the overlaps' end points,
/// the isolated crossings on the rest of the curve, and points spread evenly
/// over the overlaps. Works on the untrimmed patch. Exhausting `max_nodes`
/// in any sub-search is an error.
pub fn curve_surface_intersect<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surface: &NurbSurface<S, 4>,
    max_solutions: usize,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Intersections<(S, Vector2<S>)>> {
    // Disjoint bounding boxes rule out crossings and overlaps alike, before
    // any candidate probe runs.
    if max_solutions == 0 || !aabb_could_overlap(&curve.aabb, &surface.aabb, 3) {
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
    let crossings = if overlaps.is_empty() {
        curve_surface_crossings(curve, surface, max_nodes, min_subdivision_size).with_context(
            &|e: GeopError| ctx(e.with_context("no overlap found; crossing search")),
        )?
    } else {
        let mut crossings = Vec::new();
        for (lo, hi) in coincidence::gaps(curve.domain(), &overlaps) {
            let piece = curve.sub_curve(lo, hi)?;
            crossings.extend(curve_surface_crossings(
                &piece,
                surface,
                max_nodes,
                min_subdivision_size,
            )?);
        }
        crossings
    };
    let samples = coincidence::samples(&overlaps, max_solutions, |t| {
        let point = curve.evaluate(t)?;
        Ok(
            surface_could_contain(surface, &point, max_nodes, min_subdivision_size)?
                .map(|(u, v)| Vector2::from_array([u, v])),
        )
    })?;
    Ok(coincidence::assemble(
        &overlaps,
        crossings,
        samples,
        max_solutions,
    ))
}

/// Newton iterations for [`refine`]. Quadratic convergence makes a handful
/// plenty; this cannot change which solutions are found, only how tightly an
/// already-isolated one is pinned down.
const REFINE_ITERATIONS: usize = 12;

/// Polish one isolated `(t, uv)` — as returned by [`curve_surface_intersect`]
/// — by Newton on `C(t) - S(u, v) = 0`, three equations in the three unknowns
/// `t`, `u`, `v`.
///
/// Subdivision and Newton divide the work: subdivision is the global method,
/// reliably finding and separating every solution and recognizing coincidence
/// even for a partial overlap, but converging only one bit per split; Newton
/// cannot find anything but polishes an isolated solution quadratically.
/// `curve_surface_intersect` deliberately does *not* apply this to everything
/// it returns — most callers only need to know where and how many crossings
/// there are, and refining changes results they already agree with. Call it
/// when the parameter is about to be *used* as a split point, where the width
/// genuinely matters: `NurbCurve::split` cannot absorb a
/// `min_subdivision_size`-wide parameter (Boehm insertion amplifies it without
/// bound), and a point evaluated at one is just as wide — which is how a wide
/// crossing becomes a fat vertex and a fat sub-curve.
///
/// Every iterate but the last is sharpened, which is legitimate: it is only a
/// seed for the next step. The final step is left unsharpened, so the returned
/// widths honestly state how well the crossing is determined (see "Sharpen
/// only where the value is a free choice" in `AGENTS.md`).
///
/// Infallible by construction: the incoming box is already a valid enclosure,
/// so anything that stops Newton — a singular Jacobian at a tangential
/// crossing or a pole, an iterate leaving the domain, a refined box disjoint
/// from the one subdivision proved the solution lies in — just returns that
/// box unchanged. Refinement can only tighten, never fail.
pub fn refine_crossing<S: Scalar>(
    curve: &NurbCurve<S, 4>,
    surface: &NurbSurface<S, 4>,
    t: S,
    uv: Vector2<S>,
) -> (S, Vector2<S>) {
    let (t_lo, t_hi) = curve.domain();
    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();

    let (mut tt, mut uu, mut vv) = (t.sharpen(), uv[0].sharpen(), uv[1].sharpen());
    for iteration in 0..REFINE_ITERATIONS {
        let (Ok(c), Ok(s)) = (curve.evaluate(tt), surface.evaluate(uu, vv)) else {
            return (t, uv);
        };
        let (Ok(ct), Ok((su, sv))) = (curve.tangent(tt), surface.derivatives(uu, vv)) else {
            return (t, uv);
        };

        let mut a = [[S::ZERO; 3]; 3];
        let mut b = [S::ZERO; 3];
        for row in 0..3 {
            a[row] = [ct[row], su[row].neg(), sv[row].neg()];
            b[row] = s[row].sub(c[row]);
        }
        let Ok(delta) = solve_linear_system(&Matrix::from_rows(a), &Vector::from_array(b)) else {
            return (t, uv);
        };

        let next = [tt.add(delta[0]), uu.add(delta[1]), vv.add(delta[2])];
        let next = if iteration + 1 == REFINE_ITERATIONS {
            next
        } else {
            [next[0].sharpen(), next[1].sharpen(), next[2].sharpen()]
        };
        tt = clamp(next[0], t_lo, t_hi);
        uu = clamp(next[1], u_lo, u_hi);
        vv = clamp(next[2], v_lo, v_hi);
    }

    if !tt.could_be_equal(t) || !uu.could_be_equal(uv[0]) || !vv.could_be_equal(uv[1]) {
        return (t, uv);
    }
    let (t_ref, u_ref, v_ref) = (t.intersect(tt), uv[0].intersect(uu), uv[1].intersect(vv));
    // The refined box must still be able to hold a crossing: the curve and
    // the surface evaluated over it have to overlap. Newton only assumes an
    // isolated, regular root; where that fails — a curve lying on the
    // surface's extension beyond the patch, so the iterate slides along it
    // and gets clamped at the patch's edge — it can return a narrow box that
    // provably misses, and the honest incoming box is what must be kept.
    let holds_a_crossing = match (curve.evaluate(t_ref), surface.evaluate(u_ref, v_ref)) {
        (Ok(c), Ok(s)) => c.could_be_equal(&s),
        _ => false,
    };
    if !holds_a_crossing {
        return (t, uv);
    }
    (t_ref, Vector2::from_array([u_ref, v_ref]))
}

#[cfg(test)]
mod tests {
    use super::curve_surface_crossings;
    use crate::{nurb_curve::NurbCurve, nurb_surface::NurbSurface};
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

    /// The unit square `S(u, v) = (u, v, 0)`.
    fn plane<S: Scalar>() -> NurbSurface<S, 4> {
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., 1.),
            ],
            knots(&[0., 0., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    /// Exact rational sphere octant; `v = 1` is the pole (0, 0, 1).
    fn sphere_octant<S: Scalar>() -> NurbSurface<S, 4> {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let circle = [(1., 0., 1.), (1., 1., w), (0., 1., 1.)];
        let cps = circle
            .iter()
            .flat_map(|&(x, y, wu)| {
                circle
                    .iter()
                    .map(move |&(r, z, wv)| pt(x * r, y * r, z, wu * wv))
            })
            .collect();
        let k = knots(&[0., 0., 0., 1., 1., 1.]);
        NurbSurface::try_new(2, 2, cps, k.clone(), k).unwrap()
    }

    fn solve<S: Scalar>(c: &NurbCurve<S, 4>, s: &NurbSurface<S, 4>) -> Vec<(S, [S; 2])> {
        curve_surface_crossings(c, s, MAX, S::from_f64(EPS))
            .unwrap()
            .into_iter()
            .map(|(t, uv)| (t, [uv[0], uv[1]]))
            .collect()
    }

    /// `curve_surface.md` §3's example: the clips isolate the root directly.
    fn check_vertical_line_through_plane<S: Scalar>() {
        let sols = solve(&line::<S>([0.25, 0.75, -1.], [0.25, 0.75, 1.]), &plane());
        assert_eq!(sols.len(), 1, "{sols:?}");
        let (t, [u, v]) = sols[0];
        let f = S::from_f64;
        assert!(t.could_be_equal(f(0.5)) && u.could_be_equal(f(0.25)) && v.could_be_equal(f(0.75)));
    }
    #[test]
    fn vertical_line_through_plane() {
        for_all_scalars!(check_vertical_line_through_plane);
    }

    fn check_oblique_line_through_plane<S: Scalar>() {
        let sols = solve(&line::<S>([0., 0., -1.], [1., 0.5, 1.]), &plane());
        assert_eq!(sols.len(), 1, "{sols:?}");
        let (t, [u, v]) = sols[0];
        let f = S::from_f64;
        assert!(t.could_be_equal(f(0.5)) && u.could_be_equal(f(0.5)) && v.could_be_equal(f(0.25)));
    }
    #[test]
    fn oblique_line_through_plane() {
        for_all_scalars!(check_oblique_line_through_plane);
    }

    /// A line through the sphere's center pierces the octant once, at
    /// `(1, 1, 1) / √3`.
    fn check_line_pierces_sphere<S: Scalar>() {
        let sols = solve(&line::<S>([0., 0., 0.], [1., 1., 1.]), &sphere_octant());
        assert_eq!(sols.len(), 1, "{sols:?}");
        let r = 1.0 / 3f64.sqrt();
        assert!(sols[0].0.could_be_equal(S::from_f64(r)), "{sols:?}");
    }
    #[test]
    fn line_pierces_sphere() {
        for_all_scalars!(check_line_pierces_sphere);
    }

    /// Ending exactly on the surface — a curve meeting a face at its vertex.
    fn check_endpoint_on_surface<S: Scalar>() {
        let sols = solve(&line::<S>([0.3, 0.6, 1.], [0.3, 0.6, 0.]), &plane());
        assert_eq!(sols.len(), 1, "{sols:?}");
        assert!(sols[0].0.could_be_equal(S::ONE), "{sols:?}");
    }
    #[test]
    fn endpoint_on_surface() {
        for_all_scalars!(check_endpoint_on_surface);
    }

    /// Through the pole: every `u` is a preimage.
    fn check_line_through_pole<S: Scalar>() {
        let sols = solve(&line::<S>([0., 0., 0.5], [0., 0., 1.5]), &sphere_octant());
        assert!(!sols.is_empty(), "{sols:?}");
        for (t, [_, v]) in &sols {
            assert!(
                t.could_be_equal(S::from_f64(0.5)) && v.could_be_equal(S::ONE),
                "{sols:?}"
            );
        }
    }
    #[test]
    fn line_through_pole() {
        for_all_scalars!(check_line_through_pole);
    }

    fn check_misses<S: Scalar>() {
        assert!(solve(&line::<S>([0., 0., 0.1], [1., 1., 0.5]), &plane()).is_empty());
        assert!(solve(&line::<S>([0., 0., 0.], [0.5, 0.5, 0.5]), &sphere_octant()).is_empty());
        assert!(solve(&line::<S>([2., 2., -1.], [2., 2., 1.]), &plane()).is_empty());
    }
    #[test]
    fn misses() {
        for_all_scalars!(check_misses);
    }

    fn check_budget_exhaustion_is_an_error<S: Scalar>() {
        let c = line::<S>([0., 0., -1.], [1., 0.5, 1.]);
        assert!(curve_surface_crossings(&c, &plane(), 1, S::from_f64(EPS)).is_err());
    }
    #[test]
    fn budget_exhaustion_is_an_error() {
        for_all_scalars!(check_budget_exhaustion_is_an_error);
    }

    // ── The coincidence-handling wrapper ─────────────────────────────────────

    use super::curve_surface_intersect;
    use crate::intersection::Intersections;
    use geop_core_math::vector::Vector2;

    fn wrap<S: Scalar>(
        c: &NurbCurve<S, 4>,
        s: &NurbSurface<S, 4>,
    ) -> Intersections<(S, Vector2<S>)> {
        curve_surface_intersect(c, s, 5, MAX, S::from_f64(EPS)).unwrap()
    }

    /// The equator lies on the sphere octant's `v = 0` boundary.
    fn check_equator_on_sphere_is_coincident<S: Scalar>() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let equator = NurbCurve::try_new(
            2,
            vec![pt(1., 0., 0., 1.), pt(1., 1., 0., w), pt(0., 1., 0., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        let r = wrap(&equator, &sphere_octant::<S>());
        assert!(r.is_coincident() && r.len() == 5, "{r:?}");
        for (_, uv) in r.as_slice() {
            assert!(uv[1].could_be_equal(S::ZERO), "{r:?}");
        }
    }
    #[test]
    fn equator_on_sphere_is_coincident() {
        for_all_scalars!(check_equator_on_sphere_is_coincident);
    }

    /// An edge running along the face's `v = 0` edge.
    fn check_curve_along_patch_boundary<S: Scalar>() {
        let r = wrap(&line::<S>([0.2, 0., 0.], [0.8, 0., 0.]), &plane());
        assert!(r.is_coincident(), "{r:?}");
        for (_, uv) in r.as_slice() {
            assert!(uv[1].could_be_equal(S::ZERO), "{r:?}");
        }
    }
    #[test]
    fn curve_along_patch_boundary() {
        for_all_scalars!(check_curve_along_patch_boundary);
    }

    /// Enters the patch through `u = 0` at t = ½ and lies on it to its end:
    /// those are the overlap's ends, reported first.
    fn check_line_partly_on_plane<S: Scalar>() {
        let r = wrap(&line::<S>([-0.5, 0.5, 0.], [0.5, 0.5, 0.]), &plane());
        assert!(r.is_coincident(), "{r:?}");
        let v = r.as_slice();
        let half = S::from_f64(0.5);
        assert!(
            v[0].0.could_be_equal(half) && v[0].1[0].could_be_equal(S::ZERO),
            "{v:?}"
        );
        assert!(
            v[1].0.could_be_equal(S::ONE) && v[1].1[0].could_be_equal(half),
            "{v:?}"
        );
        for (t, _) in v {
            assert!(!t.definitely_less(half), "{v:?}");
        }
    }
    #[test]
    fn line_partly_on_plane() {
        for_all_scalars!(check_line_partly_on_plane);
    }

    /// Both ends on the plane, the middle above it: two crossings, not an
    /// overlap.
    fn check_bump_with_ends_on_plane_is_not_coincident<S: Scalar>() {
        let bump = NurbCurve::try_new(
            2,
            vec![
                pt(0.2, 0.5, 0., 1.),
                pt(0.5, 0.5, 0.6, 1.),
                pt(0.8, 0.5, 0., 1.),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        let r = wrap(&bump, &plane::<S>());
        assert!(!r.is_coincident() && r.len() == 2, "{r:?}");
    }
    #[test]
    fn bump_with_ends_on_plane_is_not_coincident() {
        for_all_scalars!(check_bump_with_ends_on_plane_is_not_coincident);
    }

    /// A revolved cylinder's top cap: the spoke of one quadrant against the
    /// neighbouring quadrant's patch, which collapses to the centre along
    /// its whole `v = 0` row. Coplanar, touching only at that pole: every
    /// clip direction built from the patch's plane is its normal, and every
    /// `u`-piece contains the pole. Exhausted the node budget before the
    /// equations were required to span space and bisection went by spatial
    /// extent.
    fn check_spoke_touching_cap_quadrant_at_its_pole<S: Scalar>() {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let pole = pt(0., 0., 2., 1.);
        let quadrant = NurbSurface::try_new(
            2,
            1,
            vec![
                pole,
                pt(1., 0., 2., 1.),
                pole,
                pt(1., -1., 2., w),
                pole,
                pt(0., -1., 2., 1.),
            ],
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        let spoke = line::<S>([0., 1., 2.], [0., 0., 2.]);
        let r = curve_surface_intersect(&spoke, &quadrant, 7, 5000, S::from_f64(1e-4)).unwrap();
        assert!(!r.is_coincident() && !r.is_empty(), "{r:?}");
        for (t, uv) in r.as_slice() {
            assert!(
                t.could_be_equal(S::ONE) && uv[1].could_be_equal(S::ZERO),
                "{r:?}"
            );
        }
    }
    #[test]
    fn spoke_touching_cap_quadrant_at_its_pole() {
        for_all_scalars!(check_spoke_touching_cap_quadrant_at_its_pole);
    }

    /// The test suite of the old `curve_surface` search, run unchanged against the
    /// coincidence-handling wrapper — the drop-in contract it must keep.
    mod old_suite {
        use super::super::curve_surface_intersect;
        use crate::{nurb_curve::NurbCurve, nurb_surface::NurbSurface};
        use geop_core_math::for_all_scalars;
        use geop_core_math::{scalars::Scalar, vector::Vector4};

        const MAX_NODES: usize = 2000;

        fn ptc<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
            Vector4::from_array([
                S::from_f64(x),
                S::from_f64(y),
                S::from_f64(z),
                S::from_f64(w),
            ])
        }

        fn pts<S: Scalar>(x: f64, y: f64, z: f64) -> Vector4<S> {
            Vector4::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z), S::ONE])
        }

        /// Flat unit patch in the xy-plane (z = 0), x,y ∈ [0,1].
        fn flat_xy<S: Scalar>() -> NurbSurface<S, 4> {
            let f = S::from_f64;
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
        fn vertical_crossing_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.5, 0.5, -1., 1.), ptc(0.5, 0.5, 1., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Straight line entirely above `flat_xy` (z ∈ [1, 2]) — never crosses.
        fn line_above_surface<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
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
        fn double_dip_curve<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
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
        fn coplanar_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0.2, 0.5, 0., 1.), ptc(0.8, 0.5, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Coplanar line spanning x ∈ [-0.5, 0.5] at y=0.5 — only the x ∈ [0, 0.5]
        /// half (t ∈ [0.5, 1]) overlaps `flat_xy`'s footprint (x,y ∈ [0,1]).
        fn partial_overlap_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
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
        fn oversized_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(-1.0, 0.5, 0., 1.), ptc(2.0, 0.5, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Coplanar line spanning x ∈ [0,1] at y=0.5 — exactly matches
        /// `flat_xy`'s x-extent.
        fn full_width_line<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            NurbCurve::try_new(
                1,
                vec![ptc(0., 0.5, 0., 1.), ptc(1., 0.5, 0., 1.)],
                vec![f(0.), f(0.), f(1.), f(1.)],
            )
            .unwrap()
        }

        /// Homogeneous control point with weight `w`, given its Cartesian
        /// position `(x, y, z)`.
        fn ptw<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
            Vector4::from_array([
                S::from_f64(x * w),
                S::from_f64(y * w),
                S::from_f64(z * w),
                S::from_f64(w),
            ])
        }

        /// Non-planar bilinear "saddle" patch: corner heights 0,1,1,0 over
        /// x,y ∈ [0,2]. Its v=0.5 ridge line sits at z = 0.5.
        fn bent_surface<S: Scalar>() -> NurbSurface<S, 4> {
            let f = S::from_f64;
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
        fn bent_curve<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
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
        fn sphere_octant_patch<S: Scalar>() -> NurbSurface<S, 4> {
            let f = S::from_f64;
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
        fn equator_quarter_circle<S: Scalar>() -> NurbCurve<S, 4> {
            let f = S::from_f64;
            let w = 1.0 / 2.0_f64.sqrt();
            NurbCurve::try_new(
                2,
                vec![ptw(1., 0., 0., 1.), ptw(1., 1., 0., w), ptw(0., 1., 0., 1.)],
                vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
            )
            .unwrap()
        }

        const EPS: f64 = 1e-2;

        // ── Single crossing ───────────────────────────────────────────────────────

        fn check_single_crossing_curve_has_one_solution<S: Scalar>() {
            let curve = vertical_crossing_line::<S>();
            let surf = flat_xy::<S>();
            let result =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result.len(), 1);
        }
        #[test]
        fn single_crossing_curve_has_one_solution() {
            for_all_scalars!(check_single_crossing_curve_has_one_solution);
        }

        // ── No crossing ───────────────────────────────────────────────────────────

        fn check_curve_missing_surface_has_no_solution<S: Scalar>() {
            let curve = line_above_surface::<S>();
            let surf = flat_xy::<S>();
            let result =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(result.is_empty());
        }
        #[test]
        fn curve_missing_surface_has_no_solution() {
            for_all_scalars!(check_curve_missing_surface_has_no_solution);
        }

        // ── Budget ────────────────────────────────────────────────────────────────

        fn check_max_solutions_zero_returns_empty<S: Scalar>() {
            let curve = vertical_crossing_line::<S>();
            let surf = flat_xy::<S>();
            let result =
                curve_surface_intersect(&curve, &surf, 0, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(result.is_empty());
        }
        #[test]
        fn max_solutions_zero_returns_empty() {
            for_all_scalars!(check_max_solutions_zero_returns_empty);
        }

        fn check_max_nodes_exhausted_errors<S: Scalar>() {
            // Adapted: the old search could only see coincidence as endless
            // subdivision, so a coincident pair was the way to overrun a tiny
            // budget. The wrapper resolves that pair directly (and correctly,
            // as `Coincident`) within it, so the invariant — running out of
            // budget is an error, never a truncated result — is exercised on
            // a two-crossing pair that genuinely needs more nodes than that.
            let curve = double_dip_curve::<S>();
            let surf = flat_xy::<S>();
            let result = curve_surface_intersect(&curve, &surf, 1000, 1, S::from_f64(EPS));
            assert!(result.is_err());
        }
        #[test]
        fn max_nodes_exhausted_errors() {
            for_all_scalars!(check_max_nodes_exhausted_errors);
        }

        // ── Two crossings ─────────────────────────────────────────────────────────

        fn check_two_crossings_found_when_budget_allows<S: Scalar>() {
            let curve = double_dip_curve::<S>();
            let surf = flat_xy::<S>();
            let result = curve_surface_intersect(&curve, &surf, 2, MAX_NODES, S::from_f64(EPS))
                .unwrap()
                .into_vec();
            assert_eq!(result.len(), 2);
            assert!(
                !result[0].0.could_be_equal(result[1].0),
                "the two crossings should remain distinct after unification"
            );
        }
        #[test]
        fn two_crossings_found_when_budget_allows() {
            for_all_scalars!(check_two_crossings_found_when_budget_allows);
        }

        fn check_max_solutions_one_caps_at_one_even_with_two_crossings<S: Scalar>() {
            let curve = double_dip_curve::<S>();
            let surf = flat_xy::<S>();
            let result =
                curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result.len(), 1);
        }
        #[test]
        fn max_solutions_one_caps_at_one_even_with_two_crossings() {
            for_all_scalars!(check_max_solutions_one_caps_at_one_even_with_two_crossings);
        }

        // ── min_subdivision_size controls precision ────────────────────────────

        fn check_min_subdivision_size_controls_precision<S: Scalar>() {
            let curve = vertical_crossing_line::<S>();
            let surf = flat_xy::<S>();
            let result = curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(1e-3))
                .unwrap()
                .into_vec();
            assert_eq!(result.len(), 1);

            let (_, uv) = result[0];
            assert!(
                uv[0]
                    .sub(S::from_f64(0.5))
                    .abs()
                    .could_be_less(S::from_f64(1e-2))
            );
            assert!(
                uv[1]
                    .sub(S::from_f64(0.5))
                    .abs()
                    .could_be_less(S::from_f64(1e-2))
            );
        }
        #[test]
        fn min_subdivision_size_controls_precision() {
            for_all_scalars!(check_min_subdivision_size_controls_precision);
        }

        // ── Coplanar curve: must terminate ───────────────────────────────────────

        fn check_coplanar_curve_terminates<S: Scalar>() {
            let curve = coplanar_line::<S>();
            let surf = flat_xy::<S>();

            // A single dive must converge to exactly one result.
            let result_one =
                curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result_one.len(), 1);

            // Asking for more solutions still terminates, with at most that many
            // (possibly fewer after unification) segments along the coplanar overlap.
            let result_many =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);
        }
        #[test]
        fn coplanar_curve_terminates() {
            for_all_scalars!(check_coplanar_curve_terminates);
        }

        // ── Coincident: an evenly-spread solution count, not just 1-or-cap ──────

        fn check_coincident_curve_reaches_max_solutions<S: Scalar>() {
            let curve = full_width_line::<S>();
            let surf = flat_xy::<S>();
            // A curve running the *entire* width of the surface it's coincident
            // with, with a generous node budget, should reliably reach the
            // requested solution count via the evenly-spread search — this is
            // exactly the property `max_solutions` saturating is meant to
            // signal "coincident" to a caller in the first place.
            let result =
                curve_surface_intersect(&curve, &surf, 5, 5000, S::from_f64(1e-3)).unwrap();
            assert!(result.is_coincident());
            assert_eq!(result.len(), 5);
        }
        #[test]
        fn coincident_curve_reaches_max_solutions() {
            for_all_scalars!(check_coincident_curve_reaches_max_solutions);
        }

        // ── Partial overlap: only part of the curve lies over the surface ───────

        fn check_partial_overlap_coplanar_line<S: Scalar>() {
            let curve = partial_overlap_line::<S>();
            let surf = flat_xy::<S>();
            let result =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(!result.is_empty());
            assert!(result.len() <= 5);

            // Every solution must lie within the overlapping half of the curve
            // (x >= 0, i.e. t >= 0.5), up to a small tolerance.
            let lower_bound = S::from_f64(0.5 - EPS);
            for &(t, _) in result.as_slice() {
                assert!(!t.definitely_less(lower_bound));
            }
        }
        #[test]
        fn partial_overlap_coplanar_line() {
            for_all_scalars!(check_partial_overlap_coplanar_line);
        }

        // ── Curve much larger than the surface ───────────────────────────────────

        fn check_curve_larger_than_surface_terminates<S: Scalar>() {
            let curve = oversized_line::<S>();
            let surf = flat_xy::<S>();
            let result =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(!result.is_empty());
            assert!(result.len() <= 5);

            // Every solution must lie within the overlapping middle third of the
            // curve (x ∈ [0,1], i.e. t ∈ [1/3, 2/3]), up to a small tolerance.
            let lower_bound = S::from_f64(1.0 / 3.0 - EPS);
            let upper_bound = S::from_f64(2.0 / 3.0 + EPS);
            for &(t, _) in result.as_slice() {
                assert!(!t.definitely_less(lower_bound));
                assert!(!t.definitely_greater(upper_bound));
            }
        }
        #[test]
        fn curve_larger_than_surface_terminates() {
            for_all_scalars!(check_curve_larger_than_surface_terminates);
        }

        // ── Curve exactly the same size as the surface ──────────────────────────

        fn check_curve_same_size_as_surface_terminates<S: Scalar>() {
            let curve = full_width_line::<S>();
            let surf = flat_xy::<S>();

            let result_one =
                curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result_one.len(), 1);

            let result_many =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);
        }
        #[test]
        fn curve_same_size_as_surface_terminates() {
            for_all_scalars!(check_curve_same_size_as_surface_terminates);
        }

        // ── Bent surface / bent curve: distinct crossings ────────────────────────

        fn check_bent_surface_bent_curve_two_distinct_crossings<S: Scalar>() {
            let curve = bent_curve::<S>();
            let surf = bent_surface::<S>();
            let result = curve_surface_intersect(&curve, &surf, 4, MAX_NODES, S::from_f64(EPS))
                .unwrap()
                .into_vec();
            assert_eq!(result.len(), 2);
            assert!(
                !result[0].0.could_be_equal(result[1].0),
                "the two crossings of a bent curve through a bent surface should be distinct"
            );
        }
        #[test]
        fn bent_surface_bent_curve_two_distinct_crossings() {
            for_all_scalars!(check_bent_surface_bent_curve_two_distinct_crossings);
        }

        // ── Coincident circle on a spherical patch: must terminate ───────────────

        fn check_coincident_circle_on_sphere_patch_terminates<S: Scalar>() {
            let curve = equator_quarter_circle::<S>();
            let surf = sphere_octant_patch::<S>();

            let result_one =
                curve_surface_intersect(&curve, &surf, 1, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert_eq!(result_one.len(), 1);

            let result_many =
                curve_surface_intersect(&curve, &surf, 5, MAX_NODES, S::from_f64(EPS)).unwrap();
            assert!(!result_many.is_empty());
            assert!(result_many.len() <= 5);
        }
        #[test]
        fn coincident_circle_on_sphere_patch_terminates() {
            for_all_scalars!(check_coincident_circle_on_sphere_patch_terminates);
        }
    }
}
