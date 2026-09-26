use crate::{scalars::Scalar, vector::Vector};

/// Maximum number of GJK iterations before falling back to the conservative
/// "could overlap" answer.
const MAX_ITERS: usize = 64;

/// Upper bound on a GJK simplex's size, comfortably above what it ever
/// actually reaches: Johnson's subalgorithm ([`reduce_simplex`]) keeps it at
/// `N + 1` points between iterations (the most that can be affinely
/// independent in `R^N`), and `N` is 1, 2 or 3 everywhere this crate calls
/// `could_overlap` — so the simplex never holds more than `N + 2 = 5` points
/// even transiently, right after a push and before the following reduction.
///
/// Fixing this bound turns every `Vec`/heap allocation in this module's
/// GJK inner loop (`closest_point_on_simplex`'s per-subset index/point
/// lists, `solve_barycentric`'s Gram matrix and solution) into a plain
/// stack array. Profiling `geop-ops-booleans`' render tests found roughly a
/// fifth of all CPU cycles going to `malloc`/`free`, almost entirely
/// traced to these two functions — called for every subset of every
/// simplex of every GJK iteration of every convex-hull-overlap check in the
/// curve-curve/curve-surface intersection search, i.e. an enormous number
/// of times for objects this small.
const MAX_SIMPLEX: usize = 8;

/// [`solve_barycentric`]'s bordered Gram system is `(m + 1) x (m + 1)` for
/// an `m`-point subset, `m <= MAX_SIMPLEX`.
const MAX_GRAM: usize = MAX_SIMPLEX + 1;

/// The point of `points` with the largest dot product with `d`.
fn farthest_point<S: Scalar, const N: usize>(
    points: &[Vector<S, N>],
    d: &Vector<S, N>,
) -> Vector<S, N> {
    let mut best = points[0];
    let mut best_dot = best.prod_dot(d);
    for p in &points[1..] {
        let dp = p.prod_dot(d);
        if dp.definitely_greater(best_dot) {
            best = *p;
            best_dot = dp;
        }
    }
    best
}

/// Support point of the Minkowski difference `a - b` in direction `d`.
fn support<S: Scalar, const N: usize>(
    a: &[Vector<S, N>],
    b: &[Vector<S, N>],
    d: &Vector<S, N>,
) -> Vector<S, N> {
    farthest_point(a, d).sub(&farthest_point(b, &d.neg()))
}

/// True if `d` is a *definite* separating axis for point sets `a` and `b`:
/// every point of `a` has a dot product with `d` that is definitely less
/// than every point of `b`'s.
///
/// This checks all pairs rather than comparing against a single
/// [`farthest_point`], because under interval arithmetic a near-degenerate
/// (tiny) `d` can make `farthest_point`'s `definitely_greater` comparisons
/// unable to resolve the true maximum — it would then silently return an
/// arbitrary tied candidate, and a single-point check against that candidate
/// could falsely "confirm" separation along an axis that doesn't actually
/// separate the hulls.
fn separates<S: Scalar, const N: usize>(
    a: &[Vector<S, N>],
    b: &[Vector<S, N>],
    d: &Vector<S, N>,
) -> bool {
    a.iter().all(|pa| {
        let da = pa.prod_dot(d);
        b.iter().all(|pb| da.definitely_less(pb.prod_dot(d)))
    })
}

/// True if `a` and `b` are definitely separated along one of the `N`
/// coordinate axes, in either direction.
///
/// GJK's iterative search direction `d` can become a tiny vector with a huge
/// *relative* interval width (e.g. inherited from accumulated subdivision
/// rounding), making `farthest_point`'s comparisons along `d` unable to
/// resolve anything and the simplex reduction collapse `d` towards the
/// origin — at which point `could_overlap` conservatively gives up and
/// reports "could overlap". Checking the axis-aligned directions directly
/// against the input coordinates sidesteps that amplification entirely: the
/// coordinates themselves carry only their own (small) interval widths, so a
/// real gap between the point sets along any axis is still detected.
fn axis_separates<S: Scalar, const N: usize>(a: &[Vector<S, N>], b: &[Vector<S, N>]) -> bool {
    for axis in 0..N {
        let mut e = Vector::<S, N>::zero();
        e[axis] = S::ONE;
        if separates(a, b, &e) || separates(b, a, &e) {
            return true;
        }
    }
    false
}

/// Solve the bordered Gram system for the barycentric coordinates of the
/// point on the affine hull of `pts` closest to the origin (Johnson's
/// subalgorithm):
///
/// ```text
/// [ G   1 ] [λ]   [0]
/// [ 1ᵀ  0 ] [μ] = [1]
/// ```
///
/// where `G_ij = pts[i] · pts[j]`. Returns `None` if the system is singular
/// (e.g. `pts` are affinely dependent or coincide) — solved via Gaussian
/// elimination without pivoting, sized `(m+1) x (m+1)` for `m = pts.len()`.
///
/// Returns `(lambda, m)`: the first `m` entries of `lambda` are the
/// solution, the rest unused padding — a fixed-capacity stack buffer (see
/// [`MAX_SIMPLEX`]) standing in for what used to be a heap-allocated `Vec`.
fn solve_barycentric<S: Scalar, const N: usize>(
    pts: &[Vector<S, N>],
) -> Option<([S; MAX_SIMPLEX], usize)> {
    let m = pts.len();
    if m == 1 {
        let mut lambda = [S::ZERO; MAX_SIMPLEX];
        lambda[0] = S::ONE;
        return Some((lambda, 1));
    }

    let n = m + 1;
    let mut a = [[S::ZERO; MAX_GRAM]; MAX_GRAM];
    for i in 0..m {
        for j in 0..m {
            a[i][j] = pts[i].prod_dot(&pts[j]);
        }
        a[i][m] = S::ONE;
        a[m][i] = S::ONE;
    }

    let mut rhs = [S::ZERO; MAX_GRAM];
    rhs[m] = S::ONE;

    for col in 0..n {
        let pivot = a[col][col];
        for row in (col + 1)..n {
            let factor = a[row][col].div(pivot).ok()?;
            for c in col..n {
                a[row][c] = a[row][c].sub(factor.mul(a[col][c]));
            }
            rhs[row] = rhs[row].sub(factor.mul(rhs[col]));
        }
    }

    let mut x = [S::ZERO; MAX_GRAM];
    for row in (0..n).rev() {
        let mut sum = rhs[row];
        for col in (row + 1)..n {
            sum = sum.sub(a[row][col].mul(x[col]));
        }
        x[row] = sum.div(a[row][row]).ok()?;
    }

    let mut lambda = [S::ZERO; MAX_SIMPLEX];
    lambda[..m].copy_from_slice(&x[..m]);
    Some((lambda, m))
}

/// Try every non-empty subset of `simplex`, solve each for barycentric
/// coordinates via [`solve_barycentric`], and return the point (and its
/// winning subset, as indices into `simplex`) of smallest `norm_sq()` among
/// subsets whose coordinates are all non-negative (i.e. genuine faces of the
/// simplex). `None` if every subset is singular or has a negative
/// coordinate (fully degenerate simplex).
///
/// When the winning subset is the entire simplex (`N+1` affinely
/// independent points spanning all of `R^N`), its affine hull is all of
/// `R^N`, so the solved point is always the origin itself — this is what
/// detects full simplex enclosure (replacing the old tetrahedron-only
/// terminal case) without any size-specific logic.
/// Returns `(point, indices, count)`: `indices[..count]` are the winning
/// subset's positions in `simplex`, in a fixed-capacity stack buffer (see
/// [`MAX_SIMPLEX`]) rather than a heap-allocated `Vec`.
fn closest_point_on_simplex<S: Scalar, const N: usize>(
    simplex: &[Vector<S, N>],
) -> Option<(Vector<S, N>, [usize; MAX_SIMPLEX], usize)> {
    let k = simplex.len();
    debug_assert!(k <= MAX_SIMPLEX, "GJK simplex exceeded MAX_SIMPLEX");
    let mut best: Option<(Vector<S, N>, [usize; MAX_SIMPLEX], usize, S)> = None;

    for mask in 1..(1u32 << k) {
        let mut indices = [0usize; MAX_SIMPLEX];
        let mut count = 0;
        for i in 0..k {
            if mask & (1 << i) != 0 {
                indices[count] = i;
                count += 1;
            }
        }
        let mut pts = [Vector::<S, N>::zero(); MAX_SIMPLEX];
        for j in 0..count {
            pts[j] = simplex[indices[j]];
        }

        let Some((lambda, m)) = solve_barycentric(&pts[..count]) else {
            continue;
        };
        if (0..m).any(|i| lambda[i].definitely_less(S::ZERO)) {
            continue;
        }

        let mut point = Vector::<S, N>::zero();
        for i in 0..m {
            point = point.add(&pts[i].prod_scalar(lambda[i]));
        }
        let dist = point.norm_sq();

        let is_better = match &best {
            Some((_, _, _, best_dist)) => dist.definitely_less(*best_dist),
            None => true,
        };
        if is_better {
            best = Some((point, indices, count, dist));
        }
    }

    best.map(|(p, idx, count, _)| (p, idx, count))
}

/// Reduces `simplex[..*len]` towards the origin via Johnson's subalgorithm,
/// updating `len` and the search direction `d`. Returns `true` if the
/// origin is enclosed by (or lies on) the simplex.
fn reduce_simplex<S: Scalar, const N: usize>(
    simplex: &mut [Vector<S, N>; MAX_SIMPLEX],
    len: &mut usize,
    d: &mut Vector<S, N>,
) -> bool {
    match closest_point_on_simplex(&simplex[..*len]) {
        Some((point, indices, count)) => {
            if point.norm_sq().could_be_equal(S::ZERO) {
                return true;
            }
            let mut reduced = [Vector::<S, N>::zero(); MAX_SIMPLEX];
            for i in 0..count {
                reduced[i] = simplex[indices[i]];
            }
            *simplex = reduced;
            *len = count;
            *d = point.neg();
            false
        }
        // Every subset was singular or rejected — fully degenerate simplex.
        // Conservatively report overlap, matching this crate's convention
        // for "couldn't determine, assume the more permissive answer" (see
        // e.g. `contains/surface.rs`'s `Err(_) => Ok(true)`).
        None => true,
    }
}

/// GJK overlap test: true if the convex hulls of point sets `a` and `b` could
/// intersect or touch.
///
/// Both `a` and `b` must be non-empty. The convex hull of each set is *not*
/// computed explicitly; GJK works directly off the support function of the
/// point sets.
pub fn could_overlap<S: Scalar, const N: usize>(a: &[Vector<S, N>], b: &[Vector<S, N>]) -> bool {
    if axis_separates(a, b) {
        return false;
    }

    let mut d = b[0].sub(&a[0]);
    if d.norm_sq().could_be_equal(S::ZERO) {
        return true;
    }

    let mut simplex = [Vector::<S, N>::zero(); MAX_SIMPLEX];
    simplex[0] = support(a, b, &d);
    let mut len = 1;
    d = simplex[0].neg();
    if d.norm_sq().could_be_equal(S::ZERO) {
        return true;
    }

    for _ in 0..MAX_ITERS {
        if separates(a, b, &d) {
            return false;
        }
        let new_pt = support(a, b, &d);
        // `reduce_simplex` always shrinks back to at most `N + 1` points
        // before the next push (see `MAX_SIMPLEX`'s own doc comment), so
        // this never actually saturates — the fallback is defensive, not
        // load-bearing.
        if len >= MAX_SIMPLEX {
            return true;
        }
        simplex[len] = new_pt;
        len += 1;

        if reduce_simplex(&mut simplex, &mut len, &mut d) {
            return true;
        }
        if d.norm_sq().could_be_equal(S::ZERO) {
            return true;
        }
    }

    // Exceeded the iteration budget without a definite answer — conservatively
    // report that the hulls could overlap.
    true
}

/// Negation of [`could_overlap`].
pub fn definitely_no_overlap<S: Scalar, const N: usize>(
    a: &[Vector<S, N>],
    b: &[Vector<S, N>],
) -> bool {
    !could_overlap(a, b)
}

#[cfg(test)]
mod tests {
    use super::{could_overlap, definitely_no_overlap};
    use crate::{
        for_all_scalars,
        scalars::{ScalInF64, Scalar},
        vector::{Vector, Vector2, Vector3},
    };

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    fn v1<S: Scalar>(x: f64) -> Vector<S, 1> {
        Vector::from_array([S::from_f64(x)])
    }

    fn iv3(xlo: f64, xhi: f64, ylo: f64, yhi: f64, zlo: f64, zhi: f64) -> Vector3<ScalInF64> {
        Vector3::from_array([
            ScalInF64::new(xlo, xhi),
            ScalInF64::new(ylo, yhi),
            ScalInF64::new(zlo, zhi),
        ])
    }

    /// Regression test for a false `definitely_no_overlap` found while
    /// subdividing two near-collinear, coplanar curve segments to a tight
    /// `epsilon`. The two hulls below genuinely overlap (hull_a's
    /// near-degenerate y-interval sits inside hull_b's y-range, and their
    /// x-ranges overlap), but a single-point GJK termination check picked an
    /// arbitrary tied "farthest point" for a near-zero search direction and
    /// concluded separation along an axis that didn't actually separate them.
    #[test]
    fn near_collinear_coplanar_hulls_that_overlap() {
        let a0 = iv3(
            0.3537597655932039,
            0.3537597656567957,
            0.2999999999730407,
            0.3000000000269586,
            -4.450147717414304e-308,
            4.450147717414304e-308,
        );
        let a1 = iv3(
            0.3540039062411684,
            0.35400390625883144,
            0.2999999999925169,
            0.30000000000748256,
            -4.450147717125399e-308,
            4.450147717125399e-308,
        );

        let b0 = iv3(
            0.3539306639795137,
            0.3539306641454851,
            0.30010940872548025,
            0.3001094088661847,
            -4.450147718057801e-308,
            4.450147718057801e-308,
        );
        let b1 = iv3(
            0.3539367674404758,
            0.3539367677157728,
            0.3000425434435929,
            0.3000425436769378,
            -4.450147718745072e-308,
            4.450147718745072e-308,
        );
        let b2 = iv3(
            0.35394287092364923,
            0.3539428712638493,
            0.2999756837684811,
            0.2999756840567862,
            -4.450147719153063e-308,
            4.450147719153063e-308,
        );

        let a = vec![a0, a1];
        let b = vec![b0, b1, b2];

        assert!(could_overlap(&a, &b));
        assert!(!definitely_no_overlap(&a, &b));
    }

    /// Axis-aligned unit cube centered at `(cx, cy, cz)`.
    fn cube<S: Scalar>(cx: f64, cy: f64, cz: f64, half: f64) -> Vec<Vector3<S>> {
        let mut pts = Vec::with_capacity(8);
        for &dx in &[-half, half] {
            for &dy in &[-half, half] {
                for &dz in &[-half, half] {
                    pts.push(v3(cx + dx, cy + dy, cz + dz));
                }
            }
        }
        pts
    }

    fn check_overlapping_cubes_could_overlap<S: Scalar>() {
        let a = cube::<S>(0., 0., 0., 1.);
        let b = cube::<S>(0.5, 0., 0., 1.);
        assert!(could_overlap(&a, &b));
        assert!(!definitely_no_overlap(&a, &b));
    }
    #[test]
    fn overlapping_cubes_could_overlap() {
        for_all_scalars!(check_overlapping_cubes_could_overlap);
    }

    fn check_separated_cubes_no_overlap<S: Scalar>() {
        let a = cube::<S>(0., 0., 0., 1.);
        let b = cube::<S>(10., 0., 0., 1.);
        assert!(definitely_no_overlap(&a, &b));
        assert!(!could_overlap(&a, &b));
    }
    #[test]
    fn separated_cubes_no_overlap() {
        for_all_scalars!(check_separated_cubes_no_overlap);
    }

    fn check_touching_cubes_could_overlap<S: Scalar>() {
        // Cubes of half-extent 1 centered 2 apart touch exactly at one face.
        let a = cube::<S>(0., 0., 0., 1.);
        let b = cube::<S>(2., 0., 0., 1.);
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn touching_cubes_could_overlap() {
        for_all_scalars!(check_touching_cubes_could_overlap);
    }

    fn check_nested_point_inside_cube_overlaps<S: Scalar>() {
        let cube = cube::<S>(0., 0., 0., 1.);
        let point = vec![v3::<S>(0.25, -0.25, 0.5)];
        assert!(could_overlap(&cube, &point));
    }
    #[test]
    fn nested_point_inside_cube_overlaps() {
        for_all_scalars!(check_nested_point_inside_cube_overlaps);
    }

    fn check_point_outside_cube_no_overlap<S: Scalar>() {
        let cube = cube::<S>(0., 0., 0., 1.);
        let point = vec![v3::<S>(5., 5., 5.)];
        assert!(definitely_no_overlap(&cube, &point));
    }
    #[test]
    fn point_outside_cube_no_overlap() {
        for_all_scalars!(check_point_outside_cube_no_overlap);
    }

    fn check_identical_cubes_overlap<S: Scalar>() {
        let a = cube::<S>(0., 0., 0., 1.);
        let b = cube::<S>(0., 0., 0., 1.);
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn identical_cubes_overlap() {
        for_all_scalars!(check_identical_cubes_overlap);
    }

    /// Two triangles (degenerate, 2-D hulls embedded in 3-D) that cross.
    fn check_crossing_triangles_overlap<S: Scalar>() {
        let a = vec![
            v3::<S>(-1., 0., 0.),
            v3::<S>(1., 0., 0.),
            v3::<S>(0., 1., 0.),
        ];
        let b = vec![
            v3::<S>(0., -1., 0.),
            v3::<S>(0., 1., 0.),
            v3::<S>(1., -1., 0.),
        ];
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn crossing_triangles_overlap() {
        for_all_scalars!(check_crossing_triangles_overlap);
    }

    /// Two single points: overlap iff they coincide.
    fn check_single_points<S: Scalar>() {
        let a = vec![v3::<S>(1., 2., 3.)];
        let b = vec![v3::<S>(1., 2., 3.)];
        assert!(could_overlap(&a, &b));

        let c = vec![v3::<S>(1., 2., 3.0001)];
        assert!(definitely_no_overlap(&a, &c));
    }
    #[test]
    fn single_points() {
        for_all_scalars!(check_single_points);
    }

    /// A segment passing through a cube overlaps it.
    fn check_segment_through_cube_overlaps<S: Scalar>() {
        let cube = cube::<S>(0., 0., 0., 1.);
        let segment = vec![v3::<S>(-5., 0., 0.), v3::<S>(5., 0., 0.)];
        assert!(could_overlap(&cube, &segment));
    }
    #[test]
    fn segment_through_cube_overlaps() {
        for_all_scalars!(check_segment_through_cube_overlaps);
    }

    /// A segment that misses the cube entirely.
    fn check_segment_missing_cube_no_overlap<S: Scalar>() {
        let cube = cube::<S>(0., 0., 0., 1.);
        let segment = vec![v3::<S>(-5., 5., 5.), v3::<S>(5., 5., 5.)];
        assert!(definitely_no_overlap(&cube, &segment));
    }
    #[test]
    fn segment_missing_cube_no_overlap() {
        for_all_scalars!(check_segment_missing_cube_no_overlap);
    }

    // ── Degenerate cases: collinear / coplanar / duplicate points ────────────

    /// Two overlapping segments on the x-axis: [0,1] and [0.5,1.5].
    fn check_collinear_segments_overlap<S: Scalar>() {
        let a = vec![v3::<S>(0., 0., 0.), v3::<S>(1., 0., 0.)];
        let b = vec![v3::<S>(0.5, 0., 0.), v3::<S>(1.5, 0., 0.)];
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn collinear_segments_overlap() {
        for_all_scalars!(check_collinear_segments_overlap);
    }

    /// Two collinear segments on the x-axis touching only at a shared endpoint.
    fn check_collinear_segments_touching<S: Scalar>() {
        let a = vec![v3::<S>(0., 0., 0.), v3::<S>(1., 0., 0.)];
        let b = vec![v3::<S>(1., 0., 0.), v3::<S>(2., 0., 0.)];
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn collinear_segments_touching() {
        for_all_scalars!(check_collinear_segments_touching);
    }

    /// Two collinear segments on the x-axis with a gap between them.
    fn check_collinear_segments_separated<S: Scalar>() {
        let a = vec![v3::<S>(0., 0., 0.), v3::<S>(1., 0., 0.)];
        let b = vec![v3::<S>(2., 0., 0.), v3::<S>(3., 0., 0.)];
        assert!(definitely_no_overlap(&a, &b));
    }
    #[test]
    fn collinear_segments_separated() {
        for_all_scalars!(check_collinear_segments_separated);
    }

    /// Two perpendicular segments (each collinear/degenerate on its own)
    /// that cross at the origin.
    fn check_crossing_collinear_segments_overlap<S: Scalar>() {
        let a = vec![v3::<S>(-1., 0., 0.), v3::<S>(1., 0., 0.)];
        let b = vec![v3::<S>(0., -1., 0.), v3::<S>(0., 1., 0.)];
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn crossing_collinear_segments_overlap() {
        for_all_scalars!(check_crossing_collinear_segments_overlap);
    }

    /// Two parallel collinear segments offset along y: never overlap.
    fn check_parallel_collinear_segments_no_overlap<S: Scalar>() {
        let a = vec![v3::<S>(0., 0., 0.), v3::<S>(1., 0., 0.)];
        let b = vec![v3::<S>(0., 1., 0.), v3::<S>(1., 1., 0.)];
        assert!(definitely_no_overlap(&a, &b));
    }
    #[test]
    fn parallel_collinear_segments_no_overlap() {
        for_all_scalars!(check_parallel_collinear_segments_no_overlap);
    }

    /// A degenerate hull made of 3+ collinear points (e.g. a straight NURBS
    /// curve segment's control points) vs a point inside/outside its span.
    fn check_collinear_hull_vs_point<S: Scalar>() {
        let a = vec![
            v3::<S>(0., 0., 0.),
            v3::<S>(0.5, 0., 0.),
            v3::<S>(1., 0., 0.),
        ];
        // Inside the span.
        assert!(could_overlap(&a, &[v3::<S>(0.3, 0., 0.)]));
        // On the line, but outside the span.
        assert!(definitely_no_overlap(&a, &[v3::<S>(2., 0., 0.)]));
        // Off the line entirely.
        assert!(definitely_no_overlap(&a, &[v3::<S>(0.3, 1., 0.)]));
    }
    #[test]
    fn collinear_hull_vs_point() {
        for_all_scalars!(check_collinear_hull_vs_point);
    }

    /// Two coplanar (z=0) squares that overlap and that don't.
    fn check_coplanar_squares<S: Scalar>() {
        let square = |cx: f64, cy: f64| -> Vec<Vector3<S>> {
            vec![
                v3(cx, cy, 0.),
                v3(cx + 1., cy, 0.),
                v3(cx, cy + 1., 0.),
                v3(cx + 1., cy + 1., 0.),
            ]
        };
        let a = square(0., 0.);
        let overlapping = square(0.5, 0.5);
        let separated = square(5., 5.);
        assert!(could_overlap(&a, &overlapping));
        assert!(definitely_no_overlap(&a, &separated));
    }
    #[test]
    fn coplanar_squares() {
        for_all_scalars!(check_coplanar_squares);
    }

    /// A point set collapsed entirely to a single location (every point
    /// identical) — degenerate to a 0-D hull.
    fn check_degenerate_repeated_point_hull<S: Scalar>() {
        let a = vec![v3::<S>(1., 2., 3.); 4];
        let same = vec![v3::<S>(1., 2., 3.); 3];
        let elsewhere = vec![v3::<S>(1., 2., 3.0001); 2];
        assert!(could_overlap(&a, &same));
        assert!(definitely_no_overlap(&a, &elsewhere));
    }
    #[test]
    fn degenerate_repeated_point_hull() {
        for_all_scalars!(check_degenerate_repeated_point_hull);
    }

    /// A "surface patch" hull collapsed onto a single edge (two pairs of
    /// duplicate control points), as happens when a degenerate NURBS patch
    /// folds to a line. Should still behave like the underlying segment.
    fn check_degenerate_edge_hull_vs_cube<S: Scalar>() {
        // Control net: (0,0,0), (0,0,0), (1,0,0), (1,0,0) — a folded patch
        // whose hull is just the segment from (0,0,0) to (1,0,0).
        let folded = vec![
            v3::<S>(0., 0., 0.),
            v3::<S>(0., 0., 0.),
            v3::<S>(1., 0., 0.),
            v3::<S>(1., 0., 0.),
        ];
        let overlapping_cube = cube::<S>(0., 0., 0., 1.);
        let far_cube = cube::<S>(10., 0., 0., 1.);
        assert!(could_overlap(&folded, &overlapping_cube));
        assert!(definitely_no_overlap(&folded, &far_cube));
    }
    #[test]
    fn degenerate_edge_hull_vs_cube() {
        for_all_scalars!(check_degenerate_edge_hull_vs_cube);
    }

    // ── N=2 (planar) analogues — the motivating use case for genericity ─────

    fn square2<S: Scalar>(cx: f64, cy: f64, half: f64) -> Vec<Vector2<S>> {
        vec![
            v2(cx - half, cy - half),
            v2(cx + half, cy - half),
            v2(cx - half, cy + half),
            v2(cx + half, cy + half),
        ]
    }

    fn check_overlapping_squares_2d<S: Scalar>() {
        let a = square2::<S>(0., 0., 1.);
        let b = square2::<S>(0.5, 0., 1.);
        assert!(could_overlap(&a, &b));
        assert!(!definitely_no_overlap(&a, &b));
    }
    #[test]
    fn overlapping_squares_2d() {
        for_all_scalars!(check_overlapping_squares_2d);
    }

    fn check_separated_squares_2d<S: Scalar>() {
        let a = square2::<S>(0., 0., 1.);
        let b = square2::<S>(10., 0., 1.);
        assert!(definitely_no_overlap(&a, &b));
        assert!(!could_overlap(&a, &b));
    }
    #[test]
    fn separated_squares_2d() {
        for_all_scalars!(check_separated_squares_2d);
    }

    fn check_touching_squares_2d<S: Scalar>() {
        let a = square2::<S>(0., 0., 1.);
        let b = square2::<S>(2., 0., 1.);
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn touching_squares_2d() {
        for_all_scalars!(check_touching_squares_2d);
    }

    fn check_point_inside_triangle_2d<S: Scalar>() {
        let tri = vec![v2::<S>(0., 0.), v2::<S>(2., 0.), v2::<S>(0., 2.)];
        assert!(could_overlap(&tri, &[v2::<S>(0.5, 0.5)]));
        assert!(definitely_no_overlap(&tri, &[v2::<S>(5., 5.)]));
    }
    #[test]
    fn point_inside_triangle_2d() {
        for_all_scalars!(check_point_inside_triangle_2d);
    }

    /// Two segments crossing like an X.
    fn check_crossing_segments_2d<S: Scalar>() {
        let a = vec![v2::<S>(-1., -1.), v2::<S>(1., 1.)];
        let b = vec![v2::<S>(-1., 1.), v2::<S>(1., -1.)];
        assert!(could_overlap(&a, &b));
    }
    #[test]
    fn crossing_segments_2d() {
        for_all_scalars!(check_crossing_segments_2d);
    }

    fn check_collinear_segments_2d<S: Scalar>() {
        let overlapping_a = vec![v2::<S>(0., 0.), v2::<S>(1., 0.)];
        let overlapping_b = vec![v2::<S>(0.5, 0.), v2::<S>(1.5, 0.)];
        assert!(could_overlap(&overlapping_a, &overlapping_b));

        let touching_a = vec![v2::<S>(0., 0.), v2::<S>(1., 0.)];
        let touching_b = vec![v2::<S>(1., 0.), v2::<S>(2., 0.)];
        assert!(could_overlap(&touching_a, &touching_b));

        let separated_a = vec![v2::<S>(0., 0.), v2::<S>(1., 0.)];
        let separated_b = vec![v2::<S>(2., 0.), v2::<S>(3., 0.)];
        assert!(definitely_no_overlap(&separated_a, &separated_b));

        let parallel_a = vec![v2::<S>(0., 0.), v2::<S>(1., 0.)];
        let parallel_b = vec![v2::<S>(0., 1.), v2::<S>(1., 1.)];
        assert!(definitely_no_overlap(&parallel_a, &parallel_b));
    }
    #[test]
    fn collinear_segments_2d() {
        for_all_scalars!(check_collinear_segments_2d);
    }

    fn check_degenerate_repeated_point_hull_2d<S: Scalar>() {
        let a = vec![v2::<S>(1., 2.); 4];
        let same = vec![v2::<S>(1., 2.); 3];
        let elsewhere = vec![v2::<S>(1., 2.0001); 2];
        assert!(could_overlap(&a, &same));
        assert!(definitely_no_overlap(&a, &elsewhere));
    }
    #[test]
    fn degenerate_repeated_point_hull_2d() {
        for_all_scalars!(check_degenerate_repeated_point_hull_2d);
    }

    // ── N=1 sanity check — confirms true dimension-genericity ───────────────

    fn check_intervals_1d<S: Scalar>() {
        let a = vec![v1::<S>(0.), v1::<S>(1.)];
        let overlapping = vec![v1::<S>(0.5), v1::<S>(1.5)];
        let separated = vec![v1::<S>(2.), v1::<S>(3.)];
        assert!(could_overlap(&a, &overlapping));
        assert!(definitely_no_overlap(&a, &separated));

        let point_inside = vec![v1::<S>(0.5)];
        let point_outside = vec![v1::<S>(5.)];
        assert!(could_overlap(&a, &point_inside));
        assert!(definitely_no_overlap(&a, &point_outside));
    }
    #[test]
    fn intervals_1d() {
        for_all_scalars!(check_intervals_1d);
    }
}
