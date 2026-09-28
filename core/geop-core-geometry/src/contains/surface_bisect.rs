//! The previous surface/point containment — convex hull test and
//! bisection — kept only as the baseline for
//! `examples/surface_contains_bench.rs`. The kernel uses [`super::surface`].

use std::collections::VecDeque;

use crate::{aabb::aabb_could_contain, nurb_surface::NurbSurface};
use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};

/// Folds `(u, v)` into the running `(u, v)` solution, unioning each
/// component independently — see `curve::union_domain`'s own doc comment for
/// why folding in every converged patch (instead of returning the first)
/// matters.
fn union_uv<S: Scalar>(solution: Option<(S, S)>, uv: (S, S)) -> (S, S) {
    match solution {
        Some((eu, ev)) => (eu.union(uv.0), ev.union(uv.1)),
        None => uv,
    }
}

/// BFS over subdivisions of `surface`, exploring every node up to the
/// `max_nodes` budget (never stopping early at the first hit) and returning
/// the union of every converged patch's own `(u, v)` domain (each axis as a
/// single unsharp interval scalar spanning that patch, not a numeric
/// midpoint — see `patch_uv`) — a patch converges once its convex hull could
/// contain `point` and its
/// maximum span (max of the u-edge and v-edge of its control net) is no
/// longer definitely greater than `min_subdivision_size`. `None` if no patch
/// converged within budget.
///
/// Exploring to completion (rather than returning on the first match)
/// matters for the same reason as `curve_bisect::curve_could_contain`: more than one patch
/// can independently converge on `point` (e.g. near a seam, a pole, or
/// simply because the point is close to more than one subdivision
/// boundary), and stopping early would silently narrow the answer to
/// whichever one the BFS happened to visit first.
///
/// At each step the patch is split along the longer of its two parameter-domain
/// dimensions, keeping the BFS balanced.
pub fn surface_could_contain<S: Scalar>(
    surface: &NurbSurface<S, 4>,
    point: &Vector3<S>,
    max_nodes: usize,
    min_subdivision_size: S,
) -> GeopResult<Option<(S, S)>> {
    let mut queue: VecDeque<NurbSurface<S, 4>> = VecDeque::new();
    queue.push_back(surface.clone());

    let mut explored = 0usize;
    let mut solution: Option<(S, S)> = None;

    while let Some(patch) = queue.pop_front() {
        if explored >= max_nodes {
            break;
        }
        explored += 1;

        // The whole patch domain, unioned into a single (necessarily
        // unsharp) interval per axis — like `NurbCurve::domain_as_scalar` —
        // rather than its numeric midpoint, so a converged patch's genuine
        // remaining uncertainty (up to `min_subdivision_size`) propagates
        // through as interval width instead of being silently collapsed
        // into one (possibly off-curve) point.
        let patch_uv = || -> (S, S) {
            let (u_min, u_max) = patch.domain_u();
            let (v_min, v_max) = patch.domain_v();
            (u_min.union(u_max), v_min.union(v_max))
        };

        // Cheap prefilter: see `contains::curve_bisect::curve_could_contain`'s
        // identical structure — the cached bounding box is far quicker to
        // compare than building a convex hull and running GJK, and just as
        // sound.
        if !aabb_could_contain(&patch.aabb, point) {
            continue;
        }

        let hull = patch.convex_hull();

        if !hull.could_contain(point) {
            continue;
        }

        // Patch could contain the point — is it small enough?
        let nu = patch.num_u();
        let nv = patch.num_v();
        let p00 = hull.points[0];
        let pn0 = hull.points[(nu - 1) * nv];
        let p0m = hull.points[nv - 1];
        let u_size = pn0.sub(&p00).norm();
        let v_size = p0m.sub(&p00).norm();
        let max_size = if v_size.definitely_greater(u_size) {
            v_size
        } else {
            u_size
        };
        if !max_size.definitely_greater(min_subdivision_size) {
            solution = Some(union_uv(solution, patch_uv()));
            continue;
        }

        // Split along the longer dimension.
        let (left, right) = if v_size.definitely_greater(u_size) {
            let (v_min, v_max) = patch.domain_v();
            // A self-chosen subdivision point: any value in the interval cuts
            // it equally well, so sharpening loses no accuracy and keeps
            // repeated splits from compounding width (see AGENTS.md).
            let mid_v = v_min.add(v_max).div(S::TWO)?.sharpen();
            match patch.split_v(mid_v) {
                Ok(halves) => halves,
                Err(_) => {
                    solution = Some(union_uv(solution, patch_uv()));
                    continue;
                }
            }
        } else {
            let (u_min, u_max) = patch.domain_u();
            let mid_u = u_min.add(u_max).div(S::TWO)?.sharpen();
            match patch.split_u(mid_u) {
                Ok(halves) => halves,
                Err(_) => {
                    solution = Some(union_uv(solution, patch_uv()));
                    continue;
                }
            }
        };

        queue.push_back(left);
        queue.push_back(right);
    }

    Ok(solution)
}

/// Boolean negation of [`surface_could_contain`].
pub fn surface_definitely_not_contains<S: Scalar>(
    surface: &NurbSurface<S, 4>,
    point: &Vector3<S>,
    max_nodes: usize,
    epsilon: S,
) -> GeopResult<bool> {
    Ok(surface_could_contain(surface, point, max_nodes, epsilon)?.is_none())
}

#[cfg(test)]
mod tests {
    use super::{surface_could_contain, surface_definitely_not_contains};
    use crate::nurb_surface::NurbSurface;
    use geop_core_math::for_all_scalars;
    use geop_core_math::{
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    fn pt<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([
            S::from_f64(x),
            S::from_f64(y),
            S::from_f64(z),
            S::from_f64(w),
        ])
    }

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    const MAX: usize = 2000;
    const EPS: f64 = 1e-3;

    fn flat_patch<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 0., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    /// One corner lifted to z=1.
    fn lifted_patch<S: Scalar>() -> NurbSurface<S, 4> {
        let f = S::from_f64;
        NurbSurface::try_new(
            1,
            1,
            vec![
                pt(0., 0., 0., 1.),
                pt(0., 1., 0., 1.),
                pt(1., 0., 0., 1.),
                pt(1., 1., 1., 1.),
            ],
            vec![f(0.), f(0.), f(1.), f(1.)],
            vec![f(0.), f(0.), f(1.), f(1.)],
        )
        .unwrap()
    }

    // ── On-surface points ─────────────────────────────────────────────────────

    fn check_flat_surface_contains_corner<S: Scalar>() {
        let s = flat_patch::<S>();
        let p = s.evaluate(S::ZERO, S::ZERO).unwrap();
        assert!(
            surface_could_contain(&s, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn flat_surface_contains_corner() {
        for_all_scalars!(check_flat_surface_contains_corner);
    }

    fn check_flat_surface_contains_center<S: Scalar>() {
        let s = flat_patch::<S>();
        let p = s.evaluate(S::from_f64(0.5), S::from_f64(0.5)).unwrap();
        assert!(
            surface_could_contain(&s, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn flat_surface_contains_center() {
        for_all_scalars!(check_flat_surface_contains_center);
    }

    fn check_flat_surface_contains_midedge<S: Scalar>() {
        let s = flat_patch::<S>();
        let p = s.evaluate(S::from_f64(0.5), S::ZERO).unwrap();
        assert!(
            surface_could_contain(&s, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn flat_surface_contains_midedge() {
        for_all_scalars!(check_flat_surface_contains_midedge);
    }

    fn check_lifted_surface_contains_corner<S: Scalar>() {
        let s = lifted_patch::<S>();
        let p = s.evaluate(S::ZERO, S::ZERO).unwrap();
        assert!(
            surface_could_contain(&s, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn lifted_surface_contains_corner() {
        for_all_scalars!(check_lifted_surface_contains_corner);
    }

    fn check_lifted_surface_contains_midpoint<S: Scalar>() {
        let s = lifted_patch::<S>();
        let p = s.evaluate(S::from_f64(0.5), S::from_f64(0.5)).unwrap();
        assert!(
            surface_could_contain(&s, &p, MAX, S::from_f64(EPS))
                .unwrap()
                .is_some()
        );
    }
    #[test]
    fn lifted_surface_contains_midpoint() {
        for_all_scalars!(check_lifted_surface_contains_midpoint);
    }

    // ── Off-surface points ────────────────────────────────────────────────────

    fn check_flat_surface_excludes_above<S: Scalar>() {
        let s = flat_patch::<S>();
        assert!(
            surface_definitely_not_contains(&s, &v3(0.5, 0.5, 5.), MAX, S::from_f64(EPS)).unwrap()
        );
    }
    #[test]
    fn flat_surface_excludes_above() {
        for_all_scalars!(check_flat_surface_excludes_above);
    }

    fn check_flat_surface_excludes_below<S: Scalar>() {
        let s = flat_patch::<S>();
        assert!(
            surface_definitely_not_contains(&s, &v3(0.5, 0.5, -5.), MAX, S::from_f64(EPS)).unwrap()
        );
    }
    #[test]
    fn flat_surface_excludes_below() {
        for_all_scalars!(check_flat_surface_excludes_below);
    }

    fn check_flat_surface_excludes_outside_uv<S: Scalar>() {
        let s = flat_patch::<S>();
        assert!(
            surface_definitely_not_contains(&s, &v3(5., 5., 0.), MAX, S::from_f64(EPS)).unwrap()
        );
    }
    #[test]
    fn flat_surface_excludes_outside_uv() {
        for_all_scalars!(check_flat_surface_excludes_outside_uv);
    }

    fn check_lifted_surface_excludes_far_above<S: Scalar>() {
        let s = lifted_patch::<S>();
        assert!(
            surface_definitely_not_contains(&s, &v3(0.5, 0.5, 10.), MAX, S::from_f64(EPS)).unwrap()
        );
    }
    #[test]
    fn lifted_surface_excludes_far_above() {
        for_all_scalars!(check_lifted_surface_excludes_far_above);
    }

    fn check_lifted_surface_excludes_outside_uv<S: Scalar>() {
        let s = lifted_patch::<S>();
        assert!(
            surface_definitely_not_contains(&s, &v3(5., 5., 0.), MAX, S::from_f64(EPS)).unwrap()
        );
    }
    #[test]
    fn lifted_surface_excludes_outside_uv() {
        for_all_scalars!(check_lifted_surface_excludes_outside_uv);
    }

    // ── Budget ────────────────────────────────────────────────────────────────

    fn check_zero_budget_always_false<S: Scalar>() {
        let s = flat_patch::<S>();
        let p = s.evaluate(S::from_f64(0.5), S::from_f64(0.5)).unwrap();
        assert!(
            surface_could_contain(&s, &p, 0, S::from_f64(EPS))
                .unwrap()
                .is_none()
        );
    }
    #[test]
    fn zero_budget_always_false() {
        for_all_scalars!(check_zero_budget_always_false);
    }
}
