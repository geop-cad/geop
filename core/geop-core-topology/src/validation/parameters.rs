use geop_core_math::scalars::Scalar;

/// Shared tuning knobs for the whole-model validation checks — how many
/// points to sample along a curve, and the budgets passed down to the
/// underlying BFS/DFS geometric searches.
#[derive(Clone, Copy, Debug)]
pub struct ValidationParameters<S: Scalar> {
    /// How many points to sample along a curve for sampling-based checks.
    pub sample_count: usize,
    /// Node budget for the BFS containment searches (see
    /// `contains::curve::curve_could_contain`), and for the pairwise
    /// `curve_curve_intersect`/`curve_surface_intersect` searches in
    /// `disjointness_check` (which now error, rather than silently
    /// returning a truncated result, if they exhaust it — see
    /// `curve_surface_intersect`'s own doc comment).
    pub max_nodes: usize,
    /// Convergence tolerance for those same searches. Empirically, the
    /// pairwise `curve_curve_intersect`/`curve_surface_intersect` searches
    /// need a *much* looser tolerance than a single-curve
    /// `curve_could_contain` query to reliably converge within `max_nodes`:
    /// `1e-7` can fail to resolve a clean, unambiguous case at all; `1e-5`
    /// resolves most cases but not a near-tangential edge/face pair (e.g.
    /// two adjacent flat walls of an `extruded_cylinder`'s polygon
    /// approximation meeting at a shallow dihedral angle, or an edge ending
    /// exactly at a `revolve`d cap's own pole) — each halving of the
    /// tolerance costs roughly one extra subdivision *level*, and a
    /// near-tangential pair's hull-overlap pruning barely discriminates at
    /// all (see `curve_surface_intersect`'s own doc comment), so that one
    /// extra level can mean an order of magnitude more nodes. `1e-4` is
    /// loose enough to resolve every case in this crate's own basic-shape
    /// validations within `max_nodes`, while still being far tighter than
    /// any genuine geometric feature these checks care about.
    pub min_subdivision_size: S,
    /// `max_solutions` passed to `curve_curve_intersect` for every edge x
    /// edge pair: how many points to return at most. Coincidence is reported
    /// directly (`Intersections::Coincident`), not inferred from reaching
    /// this count.
    pub max_edge_edge_intersection_samples: usize,
    /// Same idea as `max_edge_edge_intersection_samples`, but for
    /// `curve_surface_intersect` on every edge x face pair.
    pub max_edge_face_intersection_samples: usize,
    /// How many random (point-on-face-a, point-on-face-b) starting pairs
    /// `face_face_numerical_intersection` tries per face pair.
    pub face_face_sample_count: usize,
    /// How many alternating-projection Newton rounds each of those starting
    /// pairs gets to converge in.
    pub face_face_newton_iterations: usize,
    /// How many random points, and how many random ray directions per
    /// point, `validate_manifold`'s ray-direction-consistency check tries
    /// per shell.
    pub manifold_ray_sample_count: usize,
    /// Seeds every random draw in `validate_manifold`'s
    /// ray-direction-consistency check.
    pub manifold_seed: u64,
}

impl<S: Scalar> Default for ValidationParameters<S> {
    fn default() -> Self {
        Self {
            sample_count: 17,
            max_nodes: 5000,
            min_subdivision_size: S::from_f64(1e-4),
            max_edge_edge_intersection_samples: 17,
            max_edge_face_intersection_samples: 7,
            face_face_sample_count: 20,
            face_face_newton_iterations: 20,
            manifold_ray_sample_count: 10,
            manifold_seed: 0x5EED_1234_5678_9ABC,
        }
    }
}
