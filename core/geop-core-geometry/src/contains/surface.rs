//! Surface/point containment by per-axis fat line clipping — the design is
//! `surface.md` next to this file, the curve version it extends is
//! `curve.md` / [`super::curve`].
//!
//! Each Cartesian axis `k` gives a polynomial tensor-product spline
//! `g_k(u, v) = H_k(u, v) - p_k W(u, v)` whose zeros are exactly where that
//! coordinate matches. Its control net is collapsed onto each parameter
//! axis by interval union over the other index, and the resulting 1-D
//! envelopes are clipped with the same fat line as curves
//! ([`clip_tensor`]). Intersecting over the axes shrinks the patch towards
//! the solution in both directions at once.

use crate::{
    aabb::aabb_could_contain,
    fat_line::{
        Stalled, carried_width, clip_tensor, extent, greville_abscissae, restriction, stalled,
    },
    knot_insertion::{is_clamped_at, pinned_clamped_end},
    nurb_curve::NurbCurve,
    nurb_surface::NurbSurface,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    solvers::subdivide::{Order, Searched, Split, subdivide},
    vector::Vector3,
};

use super::curve::curve_could_contain;

/// Spatial extent of `patch` along `u` and along `v` (see
/// [`crate::fat_line::extent`]): the longest control-polygon row running in
/// each direction.
pub(crate) fn surface_extents(patch: &NurbSurface<4>) -> [f64; 2] {
    let (nu, nv) = (patch.num_u, patch.num_v);
    let cp = &patch.control_points;
    [
        extent((0..nv).map(|j| (0..nu).map(move |i| cp[i * nv + j]))),
        extent(cp.chunks(nv).map(|row| row.iter().copied())),
    ]
}

/// Clip `patch` against `point` (`surface.md` §§1–3): a box `(û, v̂)` inside
/// the patch's domain enclosing every `(u, v)` with `Scalar(u, v) = point`, or
/// `None` if some axis proves there is none. `NurbSurface::try_new`
/// guarantees positive weights, so `W > 0` and the cross-multiplied
/// equations have exactly the surface's zeros.
fn clip(patch: &NurbSurface<4>, point: &Vector3) -> GeopResult<Option<(Scalar, Scalar)>> {
    let (u0, u1) = patch.domain_u();
    let (v0, v1) = patch.domain_v();
    let mut hats = [u0.union(u1), v0.union(v1)];
    let (nu, nv) = (patch.num_u, patch.num_v);
    let greville = [
        greville_abscissae(&patch.knot_vector_u, patch.degree_u, nu)?,
        greville_abscissae(&patch.knot_vector_v, patch.degree_v, nv)?,
    ];
    let mut d = Vec::with_capacity(nu * nv);
    for k in 0..3 {
        // `d[i * nv + j] = P_ij,k - p_k w_ij`, straight from the homogeneous
        // net: no division.
        d.clear();
        d.extend(
            patch
                .control_points
                .iter()
                .map(|q| q[k].sub(point[k].mul(q[3]))),
        );
        if !clip_tensor(&d, &[nu, nv], &greville, &mut hats) {
            return Ok(None);
        }
    }
    Ok(Some((hats[0], hats[1])))
}

/// The boundary of `patch` at the start (`first`) or end of its `u`
/// domain (`u_fixed`) or `v` domain, as a curve along the other parameter —
/// the first/last control row, which *is* the surface there when the knot
/// vector is clamped at that end. `None` if it isn't.
pub fn boundary_curve(patch: &NurbSurface<4>, u_fixed: bool, first: bool) -> Option<NurbCurve<4>> {
    let end = |n: usize| if first { 0 } else { n - 1 };
    if u_fixed {
        if !is_clamped_at(&patch.knot_vector_u, patch.degree_u, first) {
            return None;
        }
        let row = patch.control_row(true, end(patch.num_u));
        NurbCurve::try_new(patch.degree_v, row, patch.knot_vector_v.clone()).ok()
    } else {
        if !is_clamped_at(&patch.knot_vector_v, patch.degree_v, first) {
            return None;
        }
        let column = patch.control_row(false, end(patch.num_v));
        NurbCurve::try_new(patch.degree_u, column, patch.knot_vector_u.clone()).ok()
    }
}

/// "A collapsed domain is handled as a boundary evaluation" (`surface.md`
/// §3): if the clip pinned `u` (or `v`) exactly onto a clamped domain end,
/// every solution lies on that boundary row, which *is* a NURBS curve.
/// Returns it, and whether it runs along `v` (the `u` direction collapsed).
pub(crate) fn collapsed_boundary(
    patch: &NurbSurface<4>,
    u_hat: Scalar,
    v_hat: Scalar,
) -> Option<(NurbCurve<4>, bool)> {
    let (nu, nv) = (patch.num_u, patch.num_v);
    if let Some(first) = pinned_clamped_end(u_hat, &patch.knot_vector_u, nu, patch.degree_u) {
        return boundary_curve(patch, true, first).map(|c| (c, true));
    }
    if let Some(first) = pinned_clamped_end(v_hat, &patch.knot_vector_v, nv, patch.degree_v) {
        return boundary_curve(patch, false, first).map(|c| (c, false));
    }
    None
}

/// Every `(u, v)` at which `surface` could pass through `point`
/// (`surface.md` §§4–5): the componentwise union of every converged patch's
/// clipped `(u, v)` box, exploring breadth-first.
///
/// Per patch:
/// - the cached AABB and the [`clip`] are necessary conditions — failing
///   either rejects the patch;
/// - if a direction collapsed onto a clamped domain end, the boundary row is
///   searched with [`curve_could_contain`];
/// - while the clip keeps shrinking the patch, it is restricted to the clip
///   ([`crate::fat_line::restriction`]) — narrowing `v` tightens the next `u`
///   projection and vice versa;
/// - once clipping stalls ([`crate::fat_line::stalled`]), a patch whose
///   extent along both directions (control-polygon lengths, which bound a
///   folded patch where corner chords don't) is within `min_subdivision_size`
///   has converged and is reported with its clipped box; any other is
///   bisected along its spatially longest direction.
///
/// `min_subdivision_size` only bounds bisection: a patch that clipping keeps
/// shrinking is never reported early, so a point merely within
/// `min_subdivision_size` of the surface is still rejected.
///
/// `None` means every part of the domain was rejected. Running out of
/// `max_nodes` is never read as "not contained" (nor as "contained"): it is
/// an error, since the search is incomplete. The boundary curve search gets
/// the remaining budget as its own.
pub fn surface_could_contain(
    surface: &NurbSurface<4>,
    point: &Vector3,
    max_nodes: usize,
    min_subdivision_size: Scalar,
) -> GeopResult<Option<(Scalar, Scalar)>> {
    let mut solution: Option<(Scalar, Scalar)> = None;
    let mut report = |(u, v): (Scalar, Scalar)| {
        solution = Some(match solution {
            Some((su, sv)) => (su.union(u), sv.union(v)),
            None => (u, v),
        });
    };

    let searched = subdivide(
        [surface.clone()],
        Order::BreadthFirst,
        max_nodes,
        |patch, budget| -> GeopResult<_> {
            if !aabb_could_contain(&patch.aabb, point) {
                return Ok(Split::Done);
            }
            let Some((u_hat, v_hat)) = clip(&patch, point)? else {
                return Ok(Split::Done);
            };

            if let Some((boundary, along_v)) = collapsed_boundary(&patch, u_hat, v_hat) {
                if let Some(t) =
                    curve_could_contain(&boundary, point, budget, min_subdivision_size)?
                {
                    let (fixed, free) = if along_v {
                        (u_hat, v_hat)
                    } else {
                        (v_hat, u_hat)
                    };
                    if t.could_be_equal(free) {
                        let free = free.intersect(t);
                        report(if along_v {
                            (fixed, free)
                        } else {
                            (free, fixed)
                        });
                    }
                }
                return Ok(Split::Done);
            }

            let ranges = [patch.domain_u(), patch.domain_v()];
            let sizes = surface_extents(&patch);
            // The query point's own width counts too, like a second object's.
            let point_width = (0..3)
                .map(|k| point[k].width().to_f64())
                .fold(0.0, f64::max);
            let carried = carried_width(&patch.control_points).max(point_width);
            if let Some(b) = restriction(&[u_hat, v_hat], &ranges, &sizes, carried)?
                && let Ok(restricted) = patch.sub_surface(b[0], b[1])
            {
                return Ok(Split::Restrict(restricted));
            }

            let order = match stalled(&ranges, &sizes, carried, min_subdivision_size) {
                Stalled::Converged => {
                    report((u_hat, v_hat));
                    return Ok(Split::Done);
                }
                Stalled::Bisect(order) => order,
            };
            let halves = order.iter().find_map(|&dir| {
                if dir == 0 {
                    patch.split_u_mid()
                } else {
                    patch.split_v_mid()
                }
                .ok()
            });
            if let Some((left, right)) = halves {
                return Ok(Split::Halves(left, right));
            }
            // Nothing left to cut or split: what's here is the answer.
            report((u_hat, v_hat));
            Ok(Split::Done)
        },
    )?;
    if let Searched::Exhausted { pending } = searched {
        return Err(GeopError::new(format!(
            "surface_could_contain (clipping): exhausted max_nodes={max_nodes} with {pending} \
             patches pending; the result would be incomplete"
        )));
    }

    Ok(solution)
}

#[cfg(test)]
mod tests {
    use super::surface_could_contain;
    use crate::nurb_surface::NurbSurface;
    use crate::test_fixtures::{knots, sphere_octant, weighted};
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    const MAX: usize = 2000;
    const MIN_SUBDIVISION_SIZE: f64 = 1e-4;

    fn v3(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([
            Scalar::from_f64(x),
            Scalar::from_f64(y),
            Scalar::from_f64(z),
        ])
    }

    /// Bilinear patch with one corner lifted to z=1.
    fn lifted() -> NurbSurface<4> {
        NurbSurface::try_new(
            1,
            1,
            vec![
                weighted(0., 0., 0., 1.),
                weighted(0., 1., 0., 1.),
                weighted(1., 0., 0., 1.),
                weighted(1., 1., 1., 1.),
            ],
            knots(&[0., 0., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    /// Exact rational quarter cylinder: radius 1 around z, u around, v up.
    fn quarter_cylinder() -> NurbSurface<4> {
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let circle = [(1., 0., 1.), (1., 1., w), (0., 1., 1.)];
        let cps = circle
            .iter()
            .flat_map(|&(x, y, wi)| [weighted(x, y, 0., wi), weighted(x, y, 1., wi)])
            .collect();
        NurbSurface::try_new(
            2,
            1,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    /// Bicubic 5×5 net with interior knots in both directions, wavy in z.
    fn wavy() -> NurbSurface<4> {
        let cps = (0..5)
            .flat_map(|i| {
                (0..5).map(move |j| {
                    let z = ((i * 3 + j * 5) % 7) as f64 / 7.0 - 0.5;
                    weighted(i as f64, j as f64, z, 1.0)
                })
            })
            .collect();
        let k = knots(&[0., 0., 0., 0., 0.4, 1., 1., 1., 1.]);
        NurbSurface::try_new(3, 3, cps, k.clone(), k).unwrap()
    }

    /// The point at `(u, v)` is found, and the result contains `(u, v)`.
    fn assert_contains_at(s: &NurbSurface<4>, u: f64, v: f64) {
        let (u, v) = (Scalar::from_f64(u), Scalar::from_f64(v));
        let p = s.evaluate(u, v).unwrap();
        let (ru, rv) = surface_could_contain(s, &p, MAX, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
            .unwrap()
            .unwrap_or_else(|| panic!("point at ({u:?}, {v:?}) not found"));
        assert!(
            ru.could_be_equal(u) && rv.could_be_equal(v),
            "({ru:?}, {rv:?}) misses ({u:?}, {v:?})"
        );
    }

    #[test]
    fn contains_grid() {
        for s in [lifted(), quarter_cylinder(), sphere_octant(), wavy()] {
            for u in [0., 0.3, 0.4, 0.75, 1.] {
                for v in [0., 0.2, 0.4, 0.5, 1.] {
                    assert_contains_at(&s, u, v);
                }
            }
        }
    }

    /// The pole has a whole interval of preimages: every `u` at `v = 1`.
    #[test]
    fn pole_keeps_every_u() {
        let (u, v) = surface_could_contain(
            &sphere_octant(),
            &v3(0., 0., 1.),
            MAX,
            Scalar::from_f64(MIN_SUBDIVISION_SIZE),
        )
        .unwrap()
        .unwrap();
        assert!(
            u.could_be_equal(Scalar::ZERO) && u.could_be_equal(Scalar::ONE),
            "{u:?}"
        );
        assert!(v.could_be_equal(Scalar::ONE), "{v:?}");
    }

    #[test]
    fn misses_off_surface_points() {
        let miss = |s: &NurbSurface<4>, p: Vector3| {
            surface_could_contain(s, &p, MAX, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
                .unwrap()
                .is_none()
        };
        assert!(miss(&lifted(), v3(0.5, 0.5, 5.)));
        assert!(miss(&lifted(), v3(0.5, 0.5, 0.3)));
        // Inside the cylinder's control hull, off the surface.
        assert!(miss(&quarter_cylinder(), v3(0.8, 0.5, 0.5)));
        assert!(miss(&sphere_octant(), v3(0.5, 0.5, 0.5)));
        assert!(miss(&wavy(), v3(2., 2., 3.)));
    }

    /// `min_subdivision_size` bounds bisection, not accuracy: a point off the
    /// sphere by far less than it is still rejected, because clipping keeps
    /// shrinking the patch until it proves the miss.
    #[test]
    fn misses_points_closer_than_min_subdivision_size() {
        let r = (1. + 1e-10) / 3f64.sqrt();
        let found = surface_could_contain(
            &sphere_octant(),
            &v3(r, r, r),
            MAX,
            Scalar::from_f64(MIN_SUBDIVISION_SIZE),
        )
        .unwrap();
        assert!(found.is_none(), "{found:?}");
    }

    /// An exhausted budget is an incomplete search: an error, never "not
    /// contained".
    #[test]
    fn zero_budget_is_an_error() {
        let s = lifted();
        assert!(
            surface_could_contain(
                &s,
                &v3(0.5, 0.5, 5.),
                0,
                Scalar::from_f64(MIN_SUBDIVISION_SIZE)
            )
            .is_err()
        );
    }

    /// Clipping narrows a transversal hit below `min_subdivision_size`.
    #[test]
    fn result_is_tight() {
        let s = wavy();
        let p = s
            .evaluate(Scalar::from_f64(0.3), Scalar::from_f64(0.7))
            .unwrap();
        let (u, v) = surface_could_contain(&s, &p, MAX, Scalar::from_f64(MIN_SUBDIVISION_SIZE))
            .unwrap()
            .unwrap();
        assert!(
            u.width()
                .definitely_less(Scalar::from_f64(MIN_SUBDIVISION_SIZE)),
            "{u:?}"
        );
        assert!(
            v.width()
                .definitely_less(Scalar::from_f64(MIN_SUBDIVISION_SIZE)),
            "{v:?}"
        );
    }
}
