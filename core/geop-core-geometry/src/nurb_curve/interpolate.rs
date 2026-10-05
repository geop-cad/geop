//! General-purpose NURBS curve interpolation through an ordered sequence of
//! points (e.g. a `TracedCurve`'s marched polyline), in either 2-D or 3-D.
//!
//! Follows the standard global-interpolation recipe (Piegl & Tiller, "The
//! NURBS Book", §9.2.1): chord-length parameterization, the knot-averaging
//! technique, then one linear solve per coordinate for control points that
//! pass exactly through the given points at those parameters.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector, Vector2, Vector3, Vector4},
};

use super::{NurbCurve, ParameterRefinable};
use crate::spline::find_span;

/// Uniform parameterization: `t[0] = 0`, `t[last] = 1`, interior values
/// evenly spaced regardless of the spacing of `points`.
fn uniform_params<S: Scalar>(m: usize) -> Vec<S> {
    let mut t = vec![S::ZERO; m];
    t[m - 1] = S::ONE;
    for (k, tk) in t.iter_mut().enumerate().take(m - 1).skip(1) {
        *tk = S::from_ratio(k as i64, (m - 1) as i64).unwrap();
    }
    t
}

/// Chord-length parameterization (Piegl & Tiller eq. 9.5): each point gets
/// the fraction of the polyline's total length that precedes it.
///
/// The parameter values are a free choice — any strictly increasing sequence
/// yields an interpolant through the same points — but not an innocent one:
/// they decide the *speed* the curve is asked to travel between them, and a
/// spline forced to cover a long span and a short one in equal parameter time
/// overshoots on the long one. That is not hypothetical here. A traced
/// intersection curve marches in even strides and then makes one final,
/// arbitrarily-sized leg onto the vertex it terminates at; parameterized
/// uniformly, the resulting edge left the true intersection by 7.1e-4 over
/// that last leg — seven times the width the rest of the kernel is allowed to
/// carry, and enough that the same arc traced from its two ends produced two
/// curves 1.3e-4 apart, which no containment search can then recognise as the
/// same curve. Under chord length the same data reproduces the arc to about
/// its own step size to the fourth power, as cubic interpolation should.
///
/// Falls back to [`uniform_params`] when the polyline has a repeated point (or
/// no length at all): chord length would hand those two points the same
/// parameter, and the collocation matrix built from it is singular.
fn chord_length_params<S: Scalar, const C: usize>(points: &[Vector<S, C>]) -> Vec<S> {
    let m = points.len();
    let mut cumulative = vec![S::ZERO; m];
    for k in 1..m {
        let chord = points[k].sub(&points[k - 1]).norm();
        if chord.could_be_equal(S::ZERO) {
            return uniform_params(m);
        }
        cumulative[k] = cumulative[k - 1].add(chord);
    }
    let total = cumulative[m - 1];
    if total.could_be_equal(S::ZERO) {
        return uniform_params(m);
    }
    let mut t = vec![S::ZERO; m];
    t[m - 1] = S::ONE;
    for k in 1..(m - 1) {
        // Sharpened because a parameter value is a free choice, not an
        // answer: any increasing sequence interpolates the same points, so
        // the width the chord lengths carry says nothing about where the
        // curve is and only propagates into every basis function below.
        t[k] = match cumulative[k].div(total) {
            Ok(x) => x.sharpen(),
            Err(_) => return uniform_params(m),
        };
    }
    t
}

/// Knot vector via the averaging technique: `degree + 1` repeated knots at
/// each end, interior knots the running average of `degree` consecutive
/// parameter values — guarantees every knot span contains at least one
/// parameter value, which is what keeps the collocation matrix below
/// nonsingular.
fn averaging_knots<S: Scalar>(t: &[S], degree: usize) -> Vec<S> {
    let m = t.len();
    let n = m - 1;
    let p = degree;
    let mut knots = vec![S::ZERO; m + p + 1];
    for i in 0..=p {
        knots[i] = S::ZERO;
        let last = knots.len() - 1 - i;
        knots[last] = S::ONE;
    }
    let p_s = S::from_i64(p as i64);
    for j in 1..=(n - p) {
        let mut sum = S::ZERO;
        for &tk in &t[j..(j + p)] {
            sum = sum.add(tk);
        }
        knots[j + p] = sum.div(p_s).unwrap();
    }
    knots
}

/// All `degree + 1` nonzero basis function values at `t`, for the span
/// found by `find_span` (Piegl & Tiller algorithm A2.2).
fn basis_funs<S: Scalar>(span: usize, t: S, degree: usize, knots: &[S]) -> Vec<S> {
    let p = degree;
    let mut n = vec![S::ZERO; p + 1];
    n[0] = S::ONE;
    let mut left = vec![S::ZERO; p + 1];
    let mut right = vec![S::ZERO; p + 1];

    for j in 1..=p {
        left[j] = t.sub(knots[span + 1 - j]);
        right[j] = knots[span + j].sub(t);
        let mut saved = S::ZERO;
        for r in 0..j {
            let denom = right[r + 1].add(left[j - r]);
            let temp = if denom.could_be_equal(S::ZERO) {
                S::ZERO
            } else {
                n[r].div(denom).unwrap_or(S::ZERO)
            };
            n[r] = saved.add(right[r + 1].mul(temp));
            saved = left[j - r].mul(temp);
        }
        n[j] = saved;
    }
    n
}

/// Solve the `m x m` interpolation system (one row per data point, one
/// column per control point) for the control points, via Gaussian
/// elimination without pivoting. Safe without pivoting because the
/// collocation matrix of a B-spline basis evaluated at parameters chosen by
/// `averaging_knots` is totally positive and nonsingular (Piegl & Tiller
/// §9.2.1) — the same reason the reference algorithm doesn't pivot either.
///
/// The matrix is banded: row `k` is nonzero only in the `degree + 1`
/// columns of its parameter's knot span, and the spans never decrease down
/// the rows. Elimination without pivoting keeps the band, so it runs within
/// it: below a pivot only the rows whose band reaches its column, and in
/// them only the columns the pivot row's band reaches. Outside the band
/// every factor and every entry is an exact zero, and subtracting zero
/// times zero changes nothing but the enclosure — outward rounding widened
/// each entry by an ulp per pivot above it, and the dense elimination took
/// `m³ / 3` steps for what the band holds in `m (degree + 1)²`.
///
/// Generic over the dimension `C` of `points`, which are solved for as they
/// are: Cartesian points (see [`with_unit_weights`]) or homogeneous ones.
fn solve_interpolation_system<S: Scalar, const C: usize>(
    t: &[S],
    knots: &[S],
    degree: usize,
    points: &[Vector<S, C>],
) -> GeopResult<Vec<Vector<S, C>>> {
    let m = points.len();
    let p = degree;

    // Row `k`'s band: columns `first[k] ..= last[k]`.
    let mut a = vec![vec![S::ZERO; m]; m];
    let mut first = vec![0; m];
    let mut last = vec![0; m];
    for (k, &tk) in t.iter().enumerate() {
        let span = find_span(p, knots, m - 1, tk)?;
        let funs = basis_funs(span, tk, p, knots);
        for (j, &val) in funs.iter().enumerate() {
            a[k][span - p + j] = val;
        }
        (first[k], last[k]) = (span - p, span);
    }
    if first.windows(2).any(|w| w[1] < w[0]) || last.windows(2).any(|w| w[1] < w[0]) {
        return Err(GeopError::new(format!(
            "NurbCurve::interpolate: parameters {t:?} do not run through the knot spans in order"
        )));
    }

    let mut rhs: Vec<Vector<S, C>> = points.to_vec();
    for col in 0..m {
        let pivot = a[col][col];
        for row in (col + 1)..m {
            if first[row] > col {
                break;
            }
            let factor = a[row][col].div(pivot)?;
            for c in col..=last[col] {
                a[row][c] = a[row][c].sub(factor.mul(a[col][c]));
            }
            for c in 0..C {
                rhs[row][c] = rhs[row][c].sub(factor.mul(rhs[col][c]));
            }
        }
    }

    let mut ctrl = vec![Vector::<S, C>::zero(); m];
    for row in (0..m).rev() {
        let mut sum = rhs[row];
        for col in (row + 1)..=last[row] {
            let coef = a[row][col];
            for c in 0..C {
                sum[c] = sum[c].sub(coef.mul(ctrl[col][c]));
            }
        }
        for c in 0..C {
            ctrl[row][c] = sum[c].div(a[row][row])?;
        }
    }
    Ok(ctrl)
}

/// The Cartesian points `points` as homogeneous control points of weight
/// one (`D = C + 1`).
fn with_unit_weights<S: Scalar, const C: usize, const D: usize>(
    points: Vec<Vector<S, C>>,
) -> Vec<Vector<S, D>> {
    points
        .into_iter()
        .map(|p| {
            let mut v = Vector::<S, D>::zero();
            for c in 0..C {
                v[c] = p[c];
            }
            v[C] = S::ONE;
            v
        })
        .collect()
}

/// Where, within interval `i` of `intervals` between consecutive samples, a
/// caller of `interpolate_enclosing` should locate its true points: these
/// fractions of the way from one sample to the next.
///
/// One point, in the middle, for an interior interval — that is where a
/// cubic interpolant's drift peaks, and it keeps the extra work at one
/// projection per interval. The two end intervals get seven, at eighths:
/// next to a clamped end the drift is lopsided and several times larger than
/// inside, so it sets the width of the whole enclosure, and its peak lies
/// wherever the end conditions put it. Measured at quarters, a smooth hump
/// can peak between the points by ~10% of its height: on a traced
/// plane × cylinder branch that left the branch 1.36e-8 from its curve,
/// widened to 1.25e-8 (see
/// `cylinder_joined_over_a_hole` in `geop-cad-base`). At eighths the
/// shortfall is ~2%, and on a circle sampled in 8 intervals a single midpoint
/// there left a true point unenclosed.
pub fn true_point_fractions(i: usize, intervals: usize) -> &'static [(i64, i64)] {
    if i == 0 || i + 1 == intervals {
        &[(1, 8), (1, 4), (3, 8), (1, 2), (5, 8), (3, 4), (7, 8)]
    } else {
        &[(1, 2)]
    }
}

/// Shared core of `NurbCurve::interpolate` / `interpolate_enclosing` for both
/// the 2-D and 3-D cases: fit a NURBS curve of degree `degree` (clamped to at
/// least 1 and to at most `points.len() - 1`) that passes exactly through
/// `points`, then — given `between`, true points inside each interval — widen
/// it to enclose the true curve too (see [`widen_to_enclose`]).
fn interpolate<S: Scalar, const C: usize, const D: usize>(
    points: &[Vector<S, C>],
    between: Option<&[Vec<Vector<S, C>>]>,
    degree: usize,
) -> GeopResult<NurbCurve<S, D>>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    if points.len() < 2 {
        return Err(GeopError::new(
            "NurbCurve::interpolate: need at least 2 points",
        ));
    }

    let m = points.len();
    let p = degree.max(1).min(m - 1);

    // With true points to enclose, the interpolant runs through the samples'
    // centres — which points it passes through is a free choice — and the
    // samples' own width is enclosed below like any other true point.
    // Solving through interval samples instead would amplify their width
    // through the elimination.
    let centres: Vec<Vector<S, C>>;
    let through = if between.is_some() {
        centres = points.iter().map(|q| q.sharpen()).collect();
        &centres[..]
    } else {
        points
    };

    let t = chord_length_params(through);
    let knots = averaging_knots(&t, p);
    let control_points = with_unit_weights(solve_interpolation_system(&t, &knots, p, through)?);

    let mut curve = NurbCurve::try_new(p, control_points, knots)?;
    if let Some(between) = between {
        if between.len() != m - 1 {
            return Err(GeopError::new(format!(
                "NurbCurve::interpolate_enclosing: expected true points for each of the {} \
                 intervals between the {m} samples, got {}",
                m - 1,
                between.len()
            )));
        }
        // Every sample (whole, not just its centre) at its own parameter, and
        // each true point between samples evenly through its interval in the
        // interpolant's parameter — the `j`th of `k` at `(j + 1) / (k + 1)` of
        // the way. Where a check starts is a free choice, and a sharp one.
        let mut checks: Vec<(S, Vector<S, C>)> =
            t.iter().copied().zip(points.iter().copied()).collect();
        for (w, qs) in t.windows(2).zip(between) {
            for (j, &q) in qs.iter().enumerate() {
                let frac = S::from_ratio(j as i64 + 1, qs.len() as i64 + 1)?;
                checks.push((S::interpolate(w[0], w[1], frac).sharpen(), q));
            }
        }
        widen_to_enclose(&mut curve, &checks)?;
    }
    Ok(curve)
}

/// Widen `curve` until it encloses every check's point — and, as far as the
/// checks measure it, the whole true curve.
///
/// Each check is first moved to the curve's parameter nearest its point by
/// one Gauss–Newton step from its initial guess. The interpolant and the true
/// curve are parameterized differently, so comparing both at "the same
/// fraction" of an interval would mostly measure a shift *along* the curve,
/// which is no drift at all; one step removes that to first order, cheaply.
/// Where the check is made is a free choice, so the stepped parameter is
/// sharpened.
///
/// There, per coordinate, it measures how far the check's point sticks out
/// of the curve's own enclosure, and every control point is widened by the
/// largest of those, `±d` in each
/// coordinate (times its weight, in homogeneous form). The basis is
/// non-negative and sums to one, so that widens the curve everywhere by
/// exactly `d`: every check is enclosed, and so is the true curve wherever it
/// strays no further than the worst check saw. One uniform `d` rather than a
/// local one keeps it cheap and keeps a locally tight neighbour from diluting
/// the width between checks, where the drift is least measured.
fn widen_to_enclose<S: Scalar, const C: usize, const D: usize>(
    curve: &mut NurbCurve<S, D>,
    checks: &[(S, Vector<S, C>)],
) -> GeopResult<()>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    let pad = enclosing_pad(curve, checks)?;
    widen(curve, &pad);
    Ok(())
}

/// How far, per coordinate, `curve` has to be widened to enclose every
/// check's point (see [`widen_to_enclose`]).
fn enclosing_pad<S: Scalar, const C: usize, const D: usize>(
    curve: &NurbCurve<S, D>,
    checks: &[(S, Vector<S, C>)],
) -> GeopResult<[S; C]>
where
    NurbCurve<S, D>: ParameterRefinable<S, C>,
{
    let (lo, hi) = curve.domain();
    let mut pad = [S::ZERO; C];
    for &(guess, q) in checks {
        let position = curve.evaluate_cartesian(guess)?;
        let tangent = curve.tangent_cartesian(guess)?;
        let step = position
            .sub(&q)
            .prod_dot(&tangent)
            .div(tangent.prod_dot(&tangent))
            .unwrap_or(S::ZERO);
        let tau = guess.sub(step).sharpen();
        let tau = if tau.definitely_less(lo) {
            lo
        } else if tau.definitely_greater(hi) {
            hi
        } else {
            tau
        };
        // How far `q` sticks out of the curve's enclosure there, on either
        // side — what the curve must grow by to hold it. Not `|C(τ) - q|`,
        // which would count the width both already carry.
        let on_curve = curve.evaluate_cartesian(tau)?;
        for k in 0..C {
            let above = q[k].upper().sub(on_curve[k].upper()).upper();
            let below = on_curve[k].lower().sub(q[k].lower()).upper();
            pad[k] = pad[k].union(above).union(below).upper();
        }
    }
    Ok(pad)
}

/// `curve` widened by `±pad[k]` in each coordinate `k`, everywhere: every
/// control point by that much (times its weight, in homogeneous form) —
/// the basis is non-negative and sums to one.
fn widen<S: Scalar, const C: usize, const D: usize>(curve: &mut NurbCurve<S, D>, pad: &[S; C]) {
    for cp in &mut curve.control_points {
        let w = cp[D - 1];
        for k in 0..C {
            let d = pad[k].mul(w);
            cp[k] = cp[k].sub(d).union(cp[k].add(d));
        }
    }
    curve.recompute_aabb();
}

/// Checks `params` are parameters to interpolate `m` values at: as many,
/// strictly increasing, from exactly 0 to exactly 1.
fn check_params<S: Scalar>(params: &[S], m: usize) -> GeopResult<()> {
    let increasing = params.windows(2).all(|w| w[0].definitely_less(w[1]));
    let ends = params
        .first()
        .is_some_and(|t| t.is_sharp() && t.could_be_equal(S::ZERO))
        && params
            .last()
            .is_some_and(|t| t.is_sharp() && t.could_be_equal(S::ONE));
    if params.len() != m || m < 2 || !increasing || !ends {
        return Err(GeopError::new(format!(
            "NurbCurve::interpolate_homogeneous: {m} values need as many parameters, strictly increasing from 0 to 1, not {params:?}"
        )));
    }
    Ok(())
}

/// Solves `a x = rhs` for the `n x n` matrix `a` and `C` right-hand sides
/// by Gaussian elimination, pivoting on the entry of largest magnitude in
/// each column. Which row to pivot on is a free choice — every pivot that
/// cannot be zero gives the same enclosed solution — and the largest is the
/// well-conditioned one. Fails if no entry of a column definitely differs
/// from zero.
fn solve_pivoted<S: Scalar, const C: usize>(
    mut a: Vec<Vec<S>>,
    mut rhs: Vec<Vector<S, C>>,
) -> GeopResult<Vec<Vector<S, C>>> {
    let n = a.len();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&i, &j| {
                a[i][col]
                    .abs()
                    .to_f64()
                    .total_cmp(&a[j][col].abs().to_f64())
            })
            .expect("a column has rows");
        if a[pivot][col].could_be_equal(S::ZERO) {
            return Err(GeopError::new(format!(
                "solve_pivoted: column {col} has no entry that is definitely not zero"
            )));
        }
        a.swap(col, pivot);
        rhs.swap(col, pivot);
        for row in (col + 1)..n {
            let factor = a[row][col].div(a[col][col])?;
            for c in col..n {
                a[row][c] = a[row][c].sub(factor.mul(a[col][c]));
            }
            rhs[row] = rhs[row].sub(&rhs[col].prod_scalar(factor));
        }
    }
    let mut x = vec![Vector::<S, C>::zero(); n];
    for row in (0..n).rev() {
        let mut sum = rhs[row];
        for col in (row + 1)..n {
            sum = sum.sub(&x[col].prod_scalar(a[row][col]));
        }
        x[row] = sum.prod_scalar(S::ONE.div(a[row][row])?);
    }
    Ok(x)
}

impl<S: Scalar> NurbCurve<S, 4> {
    /// The cubic spline through `points`, in order: twice continuously
    /// differentiable, parameterized by chord length (see
    /// [`chord_length_params`]) with a knot at every point, so it bends
    /// only as much as passing through them needs.
    ///
    /// Each end leaves along its tangent `start` / arrives along `end` if
    /// given — a direction, whose length does not matter — and is free
    /// otherwise: straight there, its second derivative zero (a natural
    /// spline). How fast it leaves along a given tangent is a free choice
    /// of the spline's shape; it is the speed the parameterization travels
    /// at anyway, the polyline's length per unit of parameter, worked out
    /// from the points' midpoints so that the same points always give the
    /// same spline.
    ///
    /// Two points and no tangents give the straight line between them.
    pub fn cubic_spline(
        points: &[Vector3<S>],
        start: Option<Vector3<S>>,
        end: Option<Vector3<S>>,
    ) -> GeopResult<Self> {
        let m = points.len();
        if m < 2 {
            return Err(GeopError::new(
                "NurbCurve::cubic_spline: need at least 2 points",
            ));
        }
        let t = chord_length_params(points);
        // Knots `0 0 0 0 t1 .. t(m-2) 1 1 1 1`: `m + 2` control points.
        let mut knots = vec![S::ZERO; 4];
        knots.extend_from_slice(&t[1..m - 1]);
        knots.extend([S::ONE; 4]);
        let n = m + 2;
        let speed = S::from_f64(
            points
                .windows(2)
                .map(|w| w[1].sub(&w[0]).norm().to_f64())
                .sum(),
        );
        let mut a = vec![vec![S::ZERO; n]; n];
        let mut rhs = vec![Vector3::zero(); n];
        // Through the ends.
        a[0][0] = S::ONE;
        rhs[0] = points[0];
        a[n - 1][n - 1] = S::ONE;
        rhs[n - 1] = points[m - 1];
        // Through the points between, at their parameters.
        for k in 1..m - 1 {
            let span = find_span(3, &knots, n - 1, t[k])?;
            for (j, value) in basis_funs(span, t[k], 3, &knots).into_iter().enumerate() {
                a[k + 1][span - 3 + j] = value;
            }
            rhs[k + 1] = points[k];
        }
        // The end conditions, as rows over the first (last) three control
        // points: `C'(0) = 3 (P1 - P0) / u4`, `C''(0) ∝ (P2 - P1) / u5 -
        // (P1 - P0) / u4`, and mirrored at the end.
        let (u4, u5) = (knots[4], knots[5]);
        let (v4, v5) = (S::ONE.sub(knots[n - 1]), S::ONE.sub(knots[n - 2]));
        let mut condition = |row: usize,
                             tangent: Option<Vector3<S>>,
                             h1: S,
                             h2: S,
                             ends: [usize; 3],
                             sign: S|
         -> GeopResult<()> {
            let [p0, p1, p2] = ends;
            match tangent {
                Some(direction) => {
                    let derivative = direction.normalize()?.prod_scalar(speed.mul(sign));
                    a[row][p1] = S::ONE;
                    a[row][p0] = S::ONE.neg();
                    rhs[row] = derivative.prod_scalar(h1.div(S::from_i64(3))?);
                }
                None => {
                    let (k1, k2) = (S::ONE.div(h1)?, S::ONE.div(h2)?);
                    a[row][p2] = k2;
                    a[row][p1] = k1.add(k2).neg();
                    a[row][p0] = k1;
                }
            }
            Ok(())
        };
        condition(1, start, u4, u5, [0, 1, 2], S::ONE)?;
        // At the end, the tangent points back along the spline from its
        // last control point.
        condition(n - 2, end, v4, v5, [n - 1, n - 2, n - 3], S::ONE.neg())?;
        let control_points = solve_pivoted(a, rhs)?
            .into_iter()
            .map(|p| Vector4::from_array([p[0], p[1], p[2], S::ONE]))
            .collect();
        NurbCurve::try_new(3, control_points, knots)
    }

    /// Fit a 3-D NURBS curve exactly through `points` — see `interpolate`
    /// above. Between the points it is only an approximation of whatever
    /// curve they were sampled from; use [`Self::interpolate_enclosing`]
    /// when the result has to *enclose* that curve.
    pub fn interpolate(points: &[Vector3<S>], degree: usize) -> GeopResult<Self> {
        interpolate::<S, 3, 4>(points, None, degree)
    }

    /// Fit a 3-D NURBS curve through samples `points` of some true curve
    /// (through their centres; their full width is enclosed), widened so it
    /// also encloses that curve between them: `between[i]` are
    /// points of the true curve strictly between `points[i]` and
    /// `points[i + 1]`, in order and roughly evenly spaced (see
    /// [`true_point_fractions`] for the usual choice of where).
    ///
    /// An interpolant drifts from the curve it was sampled from between the
    /// samples. That drift is a genuine uncertainty about where the curve is,
    /// so it belongs in the curve's interval width: otherwise a point on the
    /// true curve tests as *not* on its interpolant, and every containment or
    /// intersection question asked of it is answered for the wrong curve. A
    /// true point per interval measures it where it is largest; the
    /// enclosure is exact there and as good as that measurement in between.
    pub fn interpolate_enclosing(
        points: &[Vector3<S>],
        between: &[Vec<Vector3<S>>],
        degree: usize,
    ) -> GeopResult<Self> {
        interpolate::<S, 3, 4>(points, Some(between), degree)
    }

    /// The rational curve of `degree` whose homogeneous control points
    /// interpolate the homogeneous points `values` — `(w p, w)`, the point
    /// `p` of weight `w` — at the strictly increasing `params`, from 0 to 1,
    /// on the knots their averages give: it passes through each `p` at its
    /// parameter. Curves interpolated at the same parameters share one knot
    /// vector, so they are the rows of a surface skinned through curves
    /// whose control points are the values (see
    /// [`crate::nurb_surface::NurbSurface::try_new`]), its iso-curve at each
    /// parameter exactly the curve there.
    ///
    /// The values are solved for as they are: interval values carry their
    /// width into the control points (sharpen a value that is a free choice
    /// first). An error if a control point's weight comes out not positive.
    pub fn interpolate_homogeneous(
        values: &[Vector4<S>],
        params: &[S],
        degree: usize,
    ) -> GeopResult<Self> {
        check_params(params, values.len())?;
        let p = degree.max(1).min(values.len() - 1);
        let knots = averaging_knots(params, p);
        let control_points = solve_interpolation_system(params, &knots, p, values)?;
        NurbCurve::try_new(p, control_points, knots)
    }

    /// How far, per coordinate, this curve has to be widened (see
    /// [`Self::widen`]) to enclose every check's point: `(t, q)`, the true
    /// point `q`, near the curve at about `t` — exactly as
    /// [`Self::interpolate_enclosing`] widens an interpolant to enclose the
    /// curve it was sampled from.
    pub fn enclosing_pad(&self, checks: &[(S, Vector3<S>)]) -> GeopResult<[S; 3]> {
        enclosing_pad(self, checks)
    }

    /// This curve widened by `±pad[k]` in each coordinate `k`, everywhere.
    pub fn widen(&mut self, pad: &[S; 3]) {
        widen(self, pad)
    }
}

impl<S: Scalar> NurbCurve<S, 3> {
    /// Fit a 2-D NURBS curve exactly through `points` — see the 3-D
    /// [`NurbCurve::interpolate`].
    pub fn interpolate(points: &[Vector2<S>], degree: usize) -> GeopResult<Self> {
        interpolate::<S, 2, 3>(points, None, degree)
    }

    /// The 2-D counterpart of the 3-D [`NurbCurve::interpolate_enclosing`].
    pub fn interpolate_enclosing(
        points: &[Vector2<S>],
        between: &[Vec<Vector2<S>>],
        degree: usize,
    ) -> GeopResult<Self> {
        interpolate::<S, 2, 3>(points, Some(between), degree)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geop_core_math::for_all_scalars;

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    /// A clamped interpolating spline must reproduce its first and last data
    /// points *exactly*, not merely closely: the end rows of the collocation
    /// matrix are `[1, 0, ...]` and `[..., 0, 1]`, so the end control points
    /// are the end data points. Callers rely on this to make a fitted curve
    /// start and end on the topological vertices it was built between — a
    /// traced intersection edge whose endpoint drifts even slightly no longer
    /// matches its own `start_vertex`/`end_vertex` and fails validation.
    fn check_interpolate_reproduces_endpoints<S: Scalar>() {
        // Deliberately not axis-aligned or evenly spaced, so no coordinate
        // is reproduced by accident.
        let points = vec![
            v3::<S>(0.5, 0.5, 0.5),
            v3::<S>(0.4713, 0.5219, 0.4102),
            v3::<S>(0.4402, 0.5411, 0.3301),
            v3::<S>(0.4001, 0.5502, 0.2604),
            v3::<S>(0.5, 0.5, 0.2),
        ];
        for degree in [1, 2, 3] {
            let curve = NurbCurve::<S, 4>::interpolate(&points, degree).unwrap();
            let (t0, t1) = curve.domain();
            let start = curve.evaluate(t0).unwrap();
            let end = curve.evaluate(t1).unwrap();
            assert!(
                start.could_be_equal(&points[0]),
                "degree {degree}: start {start:?} != {:?}",
                points[0]
            );
            assert!(
                end.could_be_equal(points.last().unwrap()),
                "degree {degree}: end {end:?} != {:?}",
                points.last().unwrap()
            );
        }
    }

    #[test]
    fn interpolate_reproduces_endpoints() {
        for_all_scalars!(check_interpolate_reproduces_endpoints);
    }

    /// Same parabola-fitting check as `check_interpolate_curve_matches_samples`,
    /// but through the 2-D (`Vector2`) code path.
    fn check_interpolate_2d_curve_matches_samples<S: Scalar>() {
        let n = 20;
        let points: Vec<_> = (0..=n)
            .map(|i| {
                let x = i as f64 / n as f64;
                v2(x, x * x)
            })
            .collect();

        let curve = NurbCurve::<S, 3>::interpolate(&points, 3).unwrap();

        let (t0, t1) = curve.domain();
        let mid_t = t0.add(t1.sub(t0).mul(S::from_f64(0.5)));
        let p = curve.evaluate(mid_t).unwrap();
        assert!(
            p[1].sub(p[0].mul(p[0]))
                .abs()
                .could_be_less(S::from_f64(1e-3))
        );
    }
    #[test]
    fn interpolate_2d_curve_matches_samples() {
        for_all_scalars!(check_interpolate_2d_curve_matches_samples);
    }

    /// Densely-sampled straight line: evaluating the fitted curve anywhere
    /// should reproduce the corresponding point on the line.
    fn check_interpolate_line<S: Scalar>() {
        let n = 50;
        let points: Vec<_> = (0..=n).map(|i| v3(i as f64 / n as f64, 0.0, 0.0)).collect();

        let curve = NurbCurve::<S, 4>::interpolate(&points, 3).unwrap();

        let p = curve.evaluate(S::from_f64(0.5)).unwrap();
        assert!(p[0].could_be_equal(S::from_f64(0.5)));
        assert!(p[1].could_be_equal(S::ZERO));
        assert!(p[2].could_be_equal(S::ZERO));
    }
    #[test]
    fn interpolate_line() {
        for_all_scalars!(check_interpolate_line);
    }

    /// A right-angle "L" polyline: the fitted curve must still pass through
    /// every original vertex.
    fn check_interpolate_passes_through_corner<S: Scalar>() {
        let mut points = Vec::new();
        for i in 0..=10 {
            points.push(v3(i as f64 / 10.0, 0.0, 0.0));
        }
        for i in 1..=10 {
            points.push(v3(1.0, i as f64 / 10.0, 0.0));
        }

        let curve = NurbCurve::<S, 4>::interpolate(&points, 3).unwrap();

        let (t0, t1) = curve.domain();
        let start = curve.evaluate(t0).unwrap();
        let end = curve.evaluate(t1).unwrap();
        assert!(start[0].could_be_equal(S::ZERO));
        assert!(start[1].could_be_equal(S::ZERO));
        assert!(end[0].could_be_equal(S::ONE));
        assert!(end[1].could_be_equal(S::ONE));
    }
    #[test]
    fn interpolate_passes_through_corner() {
        for_all_scalars!(check_interpolate_passes_through_corner);
    }

    /// Sampled points on a parabola `y = x^2`: fit with degree 3 and confirm
    /// the curve reproduces intermediate sample points closely (a non-linear
    /// curve needs its interior control points, unlike the straight-line
    /// case).
    fn check_interpolate_curve_matches_samples<S: Scalar>() {
        let n = 20;
        let points: Vec<_> = (0..=n)
            .map(|i| {
                let x = i as f64 / n as f64;
                v3(x, x * x, 0.0)
            })
            .collect();

        let curve = NurbCurve::<S, 4>::interpolate(&points, 3).unwrap();

        let (t0, t1) = curve.domain();
        let mid_t = t0.add(t1.sub(t0).mul(S::from_f64(0.5)));
        let p = curve.evaluate(mid_t).unwrap();
        // The fitted point must lie on the parabola.
        assert!(
            p[1].sub(p[0].mul(p[0]))
                .abs()
                .could_be_less(S::from_f64(1e-3))
        );
    }
    #[test]
    fn interpolate_curve_matches_samples() {
        for_all_scalars!(check_interpolate_curve_matches_samples);
    }

    /// Points of a true curve *between* the samples: an exact interpolant
    /// through a few samples of a circle misses them (its drift isn't in its
    /// width), the enclosing one contains every one of them.
    fn check_interpolate_enclosing_contains_the_true_curve<S: Scalar>() {
        use crate::contains::curve::curve_could_contain;
        let circle = |a: f64| v3::<S>(a.cos(), a.sin(), 0.);
        let n = 8;
        let angle = |i: usize, of: usize| std::f64::consts::FRAC_PI_2 * i as f64 / of as f64;
        let points: Vec<_> = (0..=n).map(|i| circle(angle(i, n))).collect();
        let between: Vec<Vec<_>> = (0..n)
            .map(|i| {
                true_point_fractions(i, n)
                    .iter()
                    .map(|&(a, b)| circle(angle(i * b as usize + a as usize, n * b as usize)))
                    .collect()
            })
            .collect();

        let exact = NurbCurve::<S, 4>::interpolate(&points, 3).unwrap();
        let enclosing = NurbCurve::<S, 4>::interpolate_enclosing(&points, &between, 3).unwrap();
        let eps = S::from_f64(1e-6);
        let dense: Vec<_> = (0..=200).map(|i| circle(angle(i, 200))).collect();
        let found = |c: &NurbCurve<S, 4>| {
            dense
                .iter()
                .filter(|q| curve_could_contain(c, q, 5000, eps).unwrap().is_some())
                .count()
        };
        assert!(
            found(&exact) < dense.len(),
            "the exact interpolant should drift"
        );
        let missed: Vec<(usize, Result<Option<S>, String>)> = dense
            .iter()
            .enumerate()
            .map(|(i, q)| {
                (
                    i,
                    curve_could_contain(&enclosing, q, 5000, eps).map_err(|e| format!("{e}")),
                )
            })
            .filter(|(_, r)| !matches!(r, Ok(Some(_))))
            .collect();
        assert!(missed.is_empty(), "true points not enclosed: {missed:?}");
    }
    #[test]
    fn interpolate_enclosing_contains_the_true_curve() {
        for_all_scalars!(check_interpolate_enclosing_contains_the_true_curve);
    }

    /// A cubic spline passes through every point, leaves and arrives along
    /// the tangents it is given, and is straight at a free end.
    fn check_cubic_spline_meets_its_conditions<S: Scalar>() {
        let points = [
            v3::<S>(0.0, 0.0, 0.0),
            v3(1.0, 0.5, 0.2),
            v3(2.0, 0.3, 1.0),
            v3(2.5, 1.5, 1.2),
        ];
        let parallel = |a: Vector3<S>, b: Vector3<S>| {
            let c = a.prod_cross(&b);
            (0..3).all(|k| c[k].abs().to_f64() < 1e-9) && a.prod_dot(&b).to_f64() > 0.0
        };
        let start = v3(0.0, 0.0, 1.0);
        let curve = NurbCurve::<S, 4>::cubic_spline(&points, Some(start), None).unwrap();
        let (t0, t1) = curve.domain();
        // Through every point: the ends exactly, the rest at their knots.
        assert!(curve.evaluate(t0).unwrap().could_be_equal(&points[0]));
        assert!(curve.evaluate(t1).unwrap().could_be_equal(&points[3]));
        for (k, &p) in points.iter().enumerate().take(3).skip(1) {
            let at = curve.knot_vector[3 + k];
            let on = curve.evaluate(at).unwrap();
            assert!(
                on.could_be_equal(&p),
                "point {k}: {on:?} at {at:?}, not {p:?}"
            );
        }
        assert!(parallel(curve.tangent(t0).unwrap(), start));
        // Free at the end: no second derivative.
        let bend = curve.second_derivative(t1).unwrap();
        assert!((0..3).all(|k| bend[k].could_be_equal(S::ZERO)), "{bend:?}");

        let end = v3(1.0, 0.0, 0.0);
        let curve = NurbCurve::<S, 4>::cubic_spline(&points, None, Some(end)).unwrap();
        assert!(parallel(curve.tangent(S::ONE).unwrap(), end));
        let bend = curve.second_derivative(S::ZERO).unwrap();
        assert!((0..3).all(|k| bend[k].could_be_equal(S::ZERO)), "{bend:?}");
    }
    #[test]
    fn cubic_spline_meets_its_conditions() {
        for_all_scalars!(check_cubic_spline_meets_its_conditions);
    }

    /// Two points and no tangents: the line between them.
    fn check_cubic_spline_of_two_points_is_straight<S: Scalar>() {
        let (a, b) = (v3::<S>(0.0, 1.0, 2.0), v3(3.0, 1.0, -2.0));
        let curve = NurbCurve::<S, 4>::cubic_spline(&[a, b], None, None).unwrap();
        let middle = curve.evaluate(S::from_f64(0.5)).unwrap();
        assert!(middle.could_be_equal(&v3(1.5, 1.0, 0.0)), "{middle:?}");
    }
    #[test]
    fn cubic_spline_of_two_points_is_straight() {
        for_all_scalars!(check_cubic_spline_of_two_points_is_straight);
    }
}
