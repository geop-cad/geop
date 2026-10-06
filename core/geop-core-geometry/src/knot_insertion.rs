//! Knot insertion shared by `NurbCurve` and `NurbSurface` splitting.
//!
//! Everything here works on *rows*: sequences of control points along the
//! direction a knot is inserted in, all sharing one knot vector. A curve is
//! a single row; a surface split in `u` is its `num_v` columns, split in `v`
//! its `num_u` rows. Knots and alphas depend only on the knot vector, so
//! they are computed once and applied to every row.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector,
};

/// Boehm single-knot insertion of `t_bar` into span `span`, in place.
fn insert_once<S: Scalar, const D: usize>(
    knots: &mut Vec<S>,
    rows: &mut [Vec<Vector<S, D>>],
    degree: usize,
    span: usize,
    t_bar: S,
) {
    let p = degree;
    // New points: Q_i = P_i for i <= span - p, Q_i = interp(P_{i-1}, P_i,
    // α_i) for span - p < i <= span, Q_{i+1} = P_i for i >= span. Duplicating
    // P_span first and then overwriting downwards reads every P_{i-1}/P_i
    // before it is replaced. The α_i use the knots *before* insertion.
    let alphas: Vec<S> = ((span - p + 1)..=span)
        .map(|i| {
            let denom = knots[i + p].sub(knots[i]);
            if denom.could_be_equal(S::ZERO) {
                S::ONE
            } else {
                t_bar.sub(knots[i]).div(denom).unwrap_or(S::ONE)
            }
        })
        .collect();
    for pts in rows.iter_mut() {
        pts.insert(span + 1, pts[span]);
        for (k, &alpha) in alphas.iter().enumerate().rev() {
            let i = span - p + 1 + k;
            pts[i] = Vector::interpolate(&pts[i - 1], &pts[i], alpha);
        }
    }
    knots.insert(span + 1, t_bar);
}

fn find_insertion_span<S: Scalar>(knots: &[S], n: usize, degree: usize, t_bar: S) -> usize {
    for k in (degree..=n).rev() {
        if !t_bar.definitely_less(knots[k]) && t_bar.definitely_less(knots[k + 1]) {
            return k;
        }
    }
    degree
}

/// Insert `t` once into every row, in place: Boehm insertion into the span
/// `t` falls in — after every knot `t` could equal, so inserting an
/// existing knot raises its multiplicity.
pub(crate) fn insert<S: Scalar, const D: usize>(
    knots: &mut Vec<S>,
    rows: &mut [Vec<Vector<S, D>>],
    degree: usize,
    t: S,
) {
    let span = find_insertion_span(knots, rows[0].len() - 1, degree, t);
    insert_once(knots, rows, degree, span, t);
}

/// True if `knots` (for `num_points` control points of `degree`) is a single
/// Bézier span `[a; p+1] ++ [b; p+1]`. "Same knot" means identical
/// enclosures (`is_subset_of` both ways), stricter than the
/// `could_be_equal` multiplicity count.
fn is_bezier_span<S: Scalar>(knots: &[S], num_points: usize, degree: usize) -> bool {
    let p = degree;
    let same = |x: S, y: S| x.is_subset_of(y) && y.is_subset_of(x);
    num_points == p + 1
        && knots[..=p].iter().all(|&k| same(k, knots[0]))
        && knots[p + 1..].iter().all(|&k| same(k, knots[p + 1]))
}

/// True if the knot vector is clamped at its start (`first = true`) or end:
/// the domain end knot has multiplicity `degree + 1`, so the first / last
/// control point (row) *is* the curve (surface) there.
pub(crate) fn is_clamped_at<S: Scalar>(knots: &[S], degree: usize, first: bool) -> bool {
    let same = |x: S, y: S| x.is_subset_of(y) && y.is_subset_of(x);
    let ends = if first {
        &knots[..=degree]
    } else {
        &knots[knots.len() - degree - 1..]
    };
    ends.iter().all(|&k| same(k, ends[0]))
}

/// If `hat` pins a parameter *exactly* (sharp) onto one end of a domain
/// that is clamped there, which end: `Some(true)` for the start. Then every
/// solution lies on that end's boundary row, which is the object itself
/// there — the "boundary evaluation" of `contains/surface.md` §3.
pub(crate) fn pinned_clamped_end<S: Scalar>(
    hat: S,
    knots: &[S],
    num_points: usize,
    degree: usize,
) -> Option<bool> {
    if !hat.is_sharp() {
        return None;
    }
    let (start, end) = domain(knots, num_points, degree);
    [(start, true), (end, false)]
        .into_iter()
        .find(|&(t, first)| hat.could_be_equal(t) && is_clamped_at(knots, degree, first))
        .map(|(_, first)| first)
}

/// Fast path of [`insert_to_full_multiplicity`] for a single Bézier span:
/// de Casteljau subdivision. Inserting `t` `p + 1` times into
/// `[a; p+1] ++ [b; p+1]` is exactly de Casteljau with `α = (t - a) / (b - a)`
/// at every level, so this is the same geometry with one division instead of
/// Boehm's ~p(p+1)/2 — and without Boehm's later-level alphas like
/// `(t - t) / (b - t)`, which interval arithmetic can only widen. `None`
/// (use Boehm) if the knots aren't a Bézier span, or `b - a` could be zero.
fn split_bezier_span<S: Scalar, const D: usize>(
    knots: &mut Vec<S>,
    rows: &mut [Vec<Vector<S, D>>],
    degree: usize,
    t: S,
) -> Option<usize> {
    let p = degree;
    if !is_bezier_span(knots, rows[0].len(), p) {
        return None;
    }
    let (a, b) = (knots[p], knots[p + 1]);
    let alpha = t.sub(a).div(b.sub(a)).ok()?;

    for pts in rows.iter_mut() {
        // Triangle row `tri`; its first entry after level r is the left
        // piece's r-th control point, its last live entry the right piece's
        // (p-r)-th.
        let mut tri = pts.clone();
        let mut left = Vec::with_capacity(2 * p + 2);
        let mut right = vec![tri[p]];
        left.push(tri[0]);
        for r in 1..=p {
            for i in 0..=p - r {
                tri[i] = Vector::interpolate(&tri[i], &tri[i + 1], alpha);
            }
            left.push(tri[0]);
            right.push(tri[p - r]);
        }
        right.reverse();
        left.extend(right);
        *pts = left;
    }

    knots.clear();
    knots.extend(std::iter::repeat_n(a, p + 1));
    knots.extend(std::iter::repeat_n(t, p + 1));
    knots.extend(std::iter::repeat_n(b, p + 1));
    Some(p + 1)
}

/// Insert `t` (strictly inside the domain) until it has multiplicity
/// `degree + 1`, in place, and return the index `k` at which every row
/// separates: the left piece is `row[..k]` / `knots[..k + degree + 1]`, the
/// right piece `row[k..]` / `knots[k..]`.
fn insert_to_full_multiplicity<S: Scalar, const D: usize>(
    knots: &mut Vec<S>,
    rows: &mut [Vec<Vector<S, D>>],
    degree: usize,
    t: S,
) -> GeopResult<usize> {
    let s: usize = knots.iter().filter(|&&ui| ui.could_be_equal(t)).count();
    if s == 0
        && let Some(k) = split_bezier_span(knots, rows, degree, t)
    {
        return Ok(k);
    }
    for _ in 0..(degree + 1).saturating_sub(s) {
        let span = find_insertion_span(knots, rows[0].len() - 1, degree, t);
        insert_once(knots, rows, degree, span, t);
    }
    knots
        .iter()
        .position(|&ui| ui.could_be_equal(t))
        .ok_or_else(|| GeopError::new("knot not found after insertion"))
}

/// Domain `(knots[p], knots[n + 1])` of rows of `num_points` points.
fn domain<S: Scalar>(knots: &[S], num_points: usize, degree: usize) -> (S, S) {
    (knots[degree], knots[num_points])
}

/// Control point rows: one `Vec` of points per row of a tensor-product grid.
pub(crate) type Rows<S, const D: usize> = Vec<Vec<Vector<S, D>>>;

/// Split every row at `t`, which must be strictly inside the domain.
/// Returns the right piece's `(knots, rows)`; `knots`/`rows` are left
/// holding the left piece.
pub(crate) fn split<S: Scalar, const D: usize>(
    knots: &mut Vec<S>,
    rows: &mut [Vec<Vector<S, D>>],
    degree: usize,
    t: S,
) -> GeopResult<(Vec<S>, Rows<S, D>)> {
    let (start, end) = domain(knots, rows[0].len(), degree);
    if !t.definitely_greater(start) || !t.definitely_less(end) {
        return Err(GeopError::new(
            "split parameter must be strictly inside the parameter domain",
        ));
    }
    let k = insert_to_full_multiplicity(knots, rows, degree, t)?;
    let right_rows = rows.iter_mut().map(|pts| pts.split_off(k)).collect();
    let right_knots = knots[k..].to_vec();
    knots.truncate(k + degree + 1);
    Ok((right_knots, right_rows))
}

/// Restrict every row to `[t0, t1]` in one pass: each bound that lies
/// strictly inside the domain (`definitely_greater` the start /
/// `definitely_less` the end) is inserted to full multiplicity and the
/// outside is dropped; a bound that doesn't is not cut at, so the result
/// always covers at least `[t0, t1] ∩ domain`.
pub(crate) fn restrict<S: Scalar, const D: usize>(
    knots: &mut Vec<S>,
    rows: &mut [Vec<Vector<S, D>>],
    degree: usize,
    t0: S,
    t1: S,
) -> GeopResult<()> {
    let (start, end) = domain(knots, rows[0].len(), degree);
    if t0.definitely_greater(start) && t0.definitely_less(end) {
        let k = insert_to_full_multiplicity(knots, rows, degree, t0)?;
        knots.drain(..k);
        for pts in rows.iter_mut() {
            pts.drain(..k);
        }
    }
    let (start, end) = domain(knots, rows[0].len(), degree);
    if t1.definitely_greater(start) && t1.definitely_less(end) {
        let k = insert_to_full_multiplicity(knots, rows, degree, t1)?;
        knots.truncate(k + degree + 1);
        for pts in rows.iter_mut() {
            pts.truncate(k);
        }
    }
    Ok(())
}
