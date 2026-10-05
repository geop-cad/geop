//! B-spline basis helpers shared by `NurbCurve` and `NurbSurface`: knot span
//! lookup, de Boor evaluation, and basis function derivatives — all in
//! homogeneous space, before any division by the weight.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

/// The distinct knots strictly inside the domain of a curve of `degree`
/// with `num_points` control points, each with its multiplicity — knots
/// that could be equal counted as one, their enclosures united.
pub(crate) fn interior_knots<S: Scalar>(
    knots: &[S],
    degree: usize,
    num_points: usize,
) -> Vec<(S, usize)> {
    let (start, end) = (knots[degree], knots[num_points]);
    let mut out: Vec<(S, usize)> = Vec::new();
    for &k in &knots[degree + 1..num_points] {
        if !(k.definitely_greater(start) && k.definitely_less(end)) {
            continue;
        }
        match out.last_mut() {
            Some((u, m)) if u.could_be_equal(k) => {
                *u = u.union(k);
                *m += 1;
            }
            _ => out.push((k, 1)),
        }
    }
    out
}

/// Where a spline of `degree` with `num_points` control points stops being
/// one polynomial piece: the ends of its domain, and every distinct knot
/// strictly between them — what an integration or a sampling of it should
/// start its panels at.
pub(crate) fn breakpoints<S: Scalar>(knots: &[S], degree: usize, num_points: usize) -> Vec<S> {
    let mut out = vec![knots[degree]];
    out.extend(
        interior_knots(knots, degree, num_points)
            .into_iter()
            .map(|(k, _)| k),
    );
    out.push(knots[num_points]);
    out
}

/// The knot spans `t` could lie in, each with the part of `t` within it.
///
/// One span and `t` itself — [`find_span`]'s — unless `t` is an interval
/// reaching past that span. One span's polynomial evaluated over all of
/// such a `t` extrapolates it across the others, which encloses nothing of
/// the spline there: a cubic of a curve fitted through 48 points, evaluated
/// over its whole domain, came out 25 times as wide as the curve. So then
/// it is every span the interval reaches, with the piece of `t` in it, and
/// a curve's evaluation is the union of the pieces'.
///
/// A surface's is not yet: `NurbSurface::evaluate` and its derivatives
/// still extrapolate [`find_span`]'s span. Uniting its pieces the same way
/// left `chamfer_cylinder_flat_side`'s boolean with a face straddling the
/// cylinder's cap — something downstream leans on the extrapolated width,
/// and needs finding before surfaces can follow.
pub(crate) fn find_spans<S: Scalar>(
    degree: usize,
    knots: &[S],
    n: usize,
    t: S,
) -> GeopResult<Vec<(usize, S)>> {
    let span = find_span(degree, knots, n, t)?;
    // Wholly within the span found, its ends included: its polynomial holds
    // over all of `t`.
    let inside =
        !t.lower().could_be_less(knots[span]) && !t.upper().could_be_greater(knots[span + 1]);
    if t.is_sharp() || inside {
        return Ok(vec![(span, t)]);
    }
    let pieces: Vec<(usize, S)> = (degree..=n)
        .filter(|&k| knots[k].could_be_less(knots[k + 1]))
        .filter(|&k| !t.definitely_less(knots[k]) && !t.definitely_greater(knots[k + 1]))
        .map(|k| (k, t.intersect(knots[k].union(knots[k + 1]))))
        .collect();
    if pieces.is_empty() {
        return Ok(vec![(span, t)]);
    }
    Ok(pieces)
}

/// The knot span containing `t`: the last index `k` in `[degree, n]` with
/// `knots[k] <= t < knots[k+1]`, where `n + 1` is the number of control
/// points.
pub(crate) fn find_span<S: Scalar>(
    degree: usize,
    knots: &[S],
    n: usize,
    t: S,
) -> GeopResult<usize> {
    let p = degree;
    if t.definitely_less(knots[p]) || t.definitely_greater(knots[n + 1]) {
        return Err(GeopError::new(format!(
            "parameter t={} out of domain [{}, {}]",
            t,
            knots[p],
            knots[n + 1]
        )));
    }
    if !t.definitely_less(knots[n + 1]) {
        for k in (p..=n).rev() {
            if knots[k].definitely_less(knots[n + 1]) {
                return Ok(k);
            }
        }
        return Ok(p);
    }
    for k in p..=n {
        if !t.definitely_less(knots[k]) && t.definitely_less(knots[k + 1]) {
            return Ok(k);
        }
    }
    Err(GeopError::new("could not find knot span"))
}

/// The homogeneous control points `local` moved so that the first one's
/// Cartesian position sits at the origin, and that position (in the first
/// `D − 1` components; the weight slot is zero).
///
/// Evaluating a rational spline at an interval parameter computes `A / W`
/// from two enclosures that interval arithmetic cannot correlate, so the
/// quotient's width grows with `|A|`: with the patch's distance from the
/// origin, not with its size. A planar cap at height 50 came out spanning
/// `[38, 94]` in height for a `u` box that it does not depend on at all.
/// Evaluated relative to a point of its own, the spline's width depends on
/// its extent only (and on nothing along a direction it is flat in), and
/// adding the origin back costs a single rounding. Which point is a free
/// choice; a sharp one costs nothing, and one of the points acting on the
/// span keeps the shifted ones as small as the patch.
pub(crate) fn centered<S: Scalar, const D: usize>(
    local: &[Vector<S, D>],
) -> (Vec<Vector<S, D>>, Vector<S, D>) {
    let first = local[0];
    let inv_w = S::ONE
        .div(first[D - 1])
        .expect("weights are definitely positive by construction");
    let mut origin = Vector::<S, D>::zero();
    for c in 0..D - 1 {
        // A placeholder's coordinates are unbounded and have no middle:
        // there the origin stays where it is.
        let at = first[c].mul(inv_w);
        origin[c] = if at.is_finite() {
            at.sharpen()
        } else {
            S::ZERO
        };
    }
    let moved = local
        .iter()
        .map(|p| {
            let mut q = *p;
            for c in 0..D - 1 {
                q[c] = p[c].sub(p[D - 1].mul(origin[c]));
            }
            q
        })
        .collect();
    (moved, origin)
}

/// De Boor triangular recursion over `points` at `t` in knot span `span`.
pub(crate) fn de_boor<S: Scalar, const D: usize>(
    degree: usize,
    knots: &[S],
    points: &[Vector<S, D>],
    t: S,
    span: usize,
) -> Vector<S, D> {
    let p = degree;
    let mut d: Vec<Vector<S, D>> = (0..=p).map(|j| points[span - p + j]).collect();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = span - p + j;
            let denom = knots[i + p - r + 1].sub(knots[i]);
            let alpha = if denom.could_be_equal(S::ZERO) {
                S::ZERO
            } else {
                t.sub(knots[i]).div(denom).unwrap_or(S::ZERO)
            };
            d[j] = Vector::interpolate(&d[j - 1], &d[j], alpha);
        }
    }
    d[p]
}

/// The homogeneous point and its derivatives up to order `n` at `t`:
/// `[A(t), A'(t), …, A⁽ⁿ⁾(t)]`, from the `degree + 1` control points
/// `local` that act on knot span `span` (`local[j]` is control point
/// `span − degree + j`).
///
/// Each derivative is a spline of one degree less whose control points are
/// the scaled differences `(p−k+1)(Q_{i+1} − Q_i) / (u_{i+p+1} − u_{i+k})`
/// (the hodograph); only the ones acting on `span` are formed, and each is
/// evaluated by de Boor. Keeping de Boor's convex combinations — rather
/// than summing basis function derivatives, where `t` recurs in every
/// term — is what keeps the enclosures tight for an interval `t`.
pub(crate) fn homogeneous_derivatives<S: Scalar, const D: usize>(
    degree: usize,
    knots: &[S],
    local: &[Vector<S, D>],
    span: usize,
    t: S,
    n: usize,
) -> Vec<Vector<S, D>> {
    let p = degree;
    let base = span - p;
    let mut points = local.to_vec();
    let mut out = Vec::with_capacity(n + 1);
    for k in 0..=n {
        if k > p {
            out.push(Vector::zero());
            continue;
        }
        if k > 0 {
            // Degree `p − k + 1` → `p − k`: point `j` is `Q_{base + j}`.
            let scale = S::from_i64((p - k + 1) as i64);
            points = (0..=p - k)
                .map(|j| {
                    let den = knots[base + j + p + 1].sub(knots[base + j + k]);
                    // A repeated knot contributes nothing (`0/0 = 0`).
                    match scale.div(den) {
                        Ok(f) => points[j + 1].sub(&points[j]).prod_scalar(f),
                        Err(_) => Vector::zero(),
                    }
                })
                .collect();
        }
        // The `k`-th derivative's knots are `knots[k..]`; shifting them by
        // `base` makes `points` its control points from index 0.
        out.push(de_boor(p - k, &knots[base + k..], &points, t, p - k));
    }
    out
}

/// `binom(n, k)`, for the small degrees splines here have.
fn binomial(n: usize, k: usize) -> i64 {
    (0..k).fold(1i64, |acc, i| acc * (n - i) as i64 / (i as i64 + 1))
}

/// Cartesian derivatives `[C, C', …, C⁽ⁿ⁾]` (`C = D − 1` components) from the
/// homogeneous ones `[(A, W), (A', W'), …]` along one parameter: differentiating
/// `A = W·C` `k` times gives the rational quotient rule
/// `C⁽ᵏ⁾ = (A⁽ᵏ⁾ − Σ_{i=1..k} binom(k, i) W⁽ⁱ⁾ C⁽ᵏ⁻ⁱ⁾) / W`
/// (Piegl & Tiller algorithm A4.2).
pub(crate) fn rational_derivatives<S: Scalar, const D: usize, const C: usize>(
    homogeneous: &[Vector<S, D>],
) -> GeopResult<Vec<Vector<S, C>>> {
    debug_assert_eq!(C + 1, D, "Cartesian dimension must be D - 1");
    let w = homogeneous[0][D - 1];
    let mut out: Vec<Vector<S, C>> = Vec::with_capacity(homogeneous.len());
    for (k, a) in homogeneous.iter().enumerate() {
        let mut v = Vector::<S, C>::zero();
        for c in 0..C {
            v[c] = a[c];
        }
        for i in 1..=k {
            let f = S::from_i64(binomial(k, i)).mul(homogeneous[i][D - 1]);
            for c in 0..C {
                v[c] = v[c].sub(f.mul(out[k - i][c]));
            }
        }
        for c in 0..C {
            v[c] = v[c].div(w)?;
        }
        out.push(v);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::{de_boor, find_span, homogeneous_derivatives};
    use geop_core_math::for_all_scalars;
    use geop_core_math::{scalars::Scalar, vector::Vector};

    /// The whole-curve hodograph: degree `p − 1`, knots without the ends.
    fn hodograph<S: Scalar>(
        p: usize,
        knots: &[S],
        c: &[Vector<S, 1>],
    ) -> (Vec<S>, Vec<Vector<S, 1>>) {
        let pts = (0..c.len() - 1)
            .map(|i| {
                let scale = S::from_i64(p as i64)
                    .div(knots[i + p + 1].sub(knots[i + 1]))
                    .unwrap();
                c[i + 1].sub(&c[i]).prod_scalar(scale)
            })
            .collect();
        (knots[1..knots.len() - 1].to_vec(), pts)
    }

    /// On a cubic with interior knots, the local derivatives must agree with
    /// evaluating the whole-curve hodograph (and its hodograph) by de Boor.
    fn check_local_derivatives_match_hodograph<S: Scalar>() {
        let f = S::from_f64;
        let p = 3;
        let knots: Vec<S> = [0., 0., 0., 0., 0.3, 0.6, 1., 1., 1., 1.].map(f).to_vec();
        let c: Vec<Vector<S, 1>> = [0., 2., -1., 2., 0., 1.]
            .map(|x| Vector::from_array([f(x)]))
            .to_vec();
        let (k1, h1) = hodograph(p, &knots, &c);
        let (k2, h2) = hodograph(p - 1, &k1, &h1);
        for t in [0., 0.1, 0.3, 0.45, 0.6, 0.8, 1.] {
            let t = f(t);
            let span = find_span(p, &knots, c.len() - 1, t).unwrap();
            let ders = homogeneous_derivatives(p, &knots, &c[span - p..=span], span, t, 3);
            let at = |deg: usize, k: &[S], pts: &[Vector<S, 1>]| {
                de_boor(deg, k, pts, t, find_span(deg, k, pts.len() - 1, t).unwrap())[0]
            };
            assert!(ders[0][0].could_be_equal(at(p, &knots, &c)), "t={t:?}");
            assert!(
                ders[1][0].could_be_equal(at(p - 1, &k1, &h1)),
                "t={t:?}: {:?}",
                ders[1]
            );
            assert!(
                ders[2][0].could_be_equal(at(p - 2, &k2, &h2)),
                "t={t:?}: {:?}",
                ders[2]
            );
        }
    }
    #[test]
    fn local_derivatives_match_hodograph() {
        for_all_scalars!(check_local_derivatives_match_hodograph);
    }
}
