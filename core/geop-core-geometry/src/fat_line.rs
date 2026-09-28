//! Fat line clipping of a spline function against zero — "Clip B" of
//! `contains/curve.md`, shared by the curve (`contains::curve`) and surface
//! (`contains::surface`) searches, and the intersection searches.

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector};

use crate::nurb_curve::dehomogenize;

/// Greville abscissae `ξ_i = (u_{i+1} + … + u_{i+p}) / p` of the `count`
/// control points of degree `p` on knot vector `u`: the first coordinates
/// that make `(ξ_i, d_i)` the control polygon of the graph
/// `(t, Σ d_i N_i(t))` (linear precision, `Σ ξ_i N_i(t) = t`). `None` for
/// degree 0, which has no such polygon.
pub(crate) fn greville_abscissae<S: Scalar>(
    u: &[S],
    p: usize,
    count: usize,
) -> GeopResult<Option<Vec<S>>> {
    if p == 0 {
        return Ok(None);
    }
    let p_s = S::from_i64(p as i64);
    let mut xi = Vec::with_capacity(count);
    for i in 0..count {
        let sum = u[i + 1..=i + p].iter().fold(S::ZERO, |acc, &k| acc.add(k));
        xi.push(sum.div(p_s)?);
    }
    Ok(Some(xi))
}

/// Enclosure of the zeros of the spline whose graph has control polygon
/// `(xi[i], d[i])`, or `None` if it definitely has none.
///
/// Fat line: the chord `ℓ` from the first to the last graph control point,
/// fattened by `Δ`, the union of every control point's *signed vertical*
/// offset from it. The whole graph lies in the band `ℓ(t) + Δ` (convex hull
/// property), so a zero needs `ℓ(t) ∈ -Δ`, i.e.
/// `t ∈ ξ_0 - (d_0 + Δ) / s` for the chord's slope `s`. Evaluated in
/// interval arithmetic, that expression encloses every real instantiation
/// of the (interval) inputs — losing the correlation between `d_0`, `s` and
/// `Δ` only widens it.
pub(crate) fn fat_line_zeros<S: Scalar>(xi: &[S], d: &[S]) -> Option<S> {
    let n = d.len() - 1;
    let (xi0, d0) = (xi[0], d[0]);

    let Ok(slope) = d[n].sub(d0).div(xi[n].sub(xi0)) else {
        // Chord of zero parametric length (all knots coincide): no line to
        // clip with, only the plain range test is left.
        let range = d.iter().skip(1).fold(d0, |acc, &di| acc.union(di));
        return range.could_be_equal(S::ZERO).then_some(S::ENTIRE);
    };

    let band = xi
        .iter()
        .zip(d)
        .map(|(&xi_i, &d_i)| d_i.sub(d0.add(slope.mul(xi_i.sub(xi0)))))
        .reduce(|acc, off| acc.union(off))
        .unwrap_or(S::ZERO);

    // Over the domain `ℓ` stays between `d_0` and `d_n`, so the band there
    // is covered by `hull(d_0, d_n) + Δ`. Missing zero proves there is no
    // root — and this also decides the case the division below can't: a
    // chord that could be horizontal.
    if !d0.union(d[n]).add(band).could_be_equal(S::ZERO) {
        return None;
    }
    match d0.add(band).div(slope) {
        Ok(q) => Some(xi0.sub(q)),
        // `s` could be zero: the band does reach the axis, but a horizontal
        // band does so everywhere — no information from this axis.
        Err(_) => Some(S::ENTIRE),
    }
}

/// Clip the zeros of a tensor-product spline in `dims.len()` parameters
/// (`surface.md` §2): `d` holds its coefficients row-major over `dims`
/// (last index fastest), `greville[dir]` the abscissae of direction `dir`
/// (`None` for degree 0), and `hats[dir]` the current enclosure of that
/// parameter, narrowed in place. `false` if the spline definitely has no
/// zero in the box.
///
/// Each direction's coefficients are collapsed by union over every other
/// index: with the other parameters fixed, the spline's coefficients along
/// `dir` are convex combinations of those, so the union envelope encloses
/// every slice and its fat line clip ([`fat_line_zeros`]) keeps every zero.
/// Intersecting per-row clips instead would be wrong — a zero of the
/// tensor need not be a zero of any row.
pub(crate) fn clip_tensor<S: Scalar>(
    d: &[S],
    dims: &[usize],
    greville: &[Option<Vec<S>>],
    hats: &mut [S],
) -> bool {
    // Range test: the whole graph lies within the coefficients' hull. Implied
    // by any clip below, but a direction of degree 0 has none.
    let Some(range) = d.iter().copied().reduce(|a, x| a.union(x)) else {
        return true;
    };
    if !range.could_be_equal(S::ZERO) {
        return false;
    }

    let mut envelopes: Vec<Vec<Option<S>>> = dims.iter().map(|&n| vec![None; n]).collect();
    let mut idx = vec![0usize; dims.len()];
    for &x in d {
        for (env, &i) in envelopes.iter_mut().zip(&idx) {
            env[i] = Some(env[i].map_or(x, |a| a.union(x)));
        }
        for dir in (0..dims.len()).rev() {
            idx[dir] += 1;
            if idx[dir] < dims[dir] {
                break;
            }
            idx[dir] = 0;
        }
    }

    for ((env, xi), hat) in envelopes.iter().zip(greville).zip(hats.iter_mut()) {
        let Some(xi) = xi else { continue };
        let env: Vec<S> = env.iter().map(|e| e.unwrap_or(S::ZERO)).collect();
        // Both constraints hold at once, so they intersect — but `intersect`
        // of disjoint enclosures returns an input rather than an empty set,
        // so disjointness is checked first.
        match fat_line_zeros(xi, &env) {
            Some(z) if z.could_be_equal(*hat) => *hat = hat.intersect(z),
            _ => return false,
        }
    }
    true
}

/// The restriction step of the clipping searches' shared schedule
/// (`surface.md` §4): the per-direction bounds to restrict the box to, or
/// `None` if clipping has stalled.
///
/// A direction is cut when its clip `hats[dir]` removed at least 20% of its
/// domain `ranges[dir]` (the usual Bézier clipping rule) *and* one of the
/// clip's sharp outer bounds lies strictly inside the domain. If any
/// direction is cut, all such cuts are made at once; a direction that isn't
/// gets its own domain ends, which cut nothing.
///
/// This comes *before* any convergence test ([`stalled`]): while clipping
/// still shrinks a box, the box is not the resolution limit of anything, and
/// stopping there would report a piece clipping was about to reject.
pub(crate) fn restriction<S: Scalar>(
    hats: &[S],
    ranges: &[(S, S)],
) -> GeopResult<Option<Vec<(S, S)>>> {
    let min_progress = S::from_ratio(4, 5)?;
    let inside = |t: S, (a, b): (S, S)| t.definitely_greater(a) && t.definitely_less(b);
    let cut: Vec<bool> = (0..hats.len())
        .map(|dir| {
            let hat = hats[dir];
            let (a, b) = ranges[dir];
            hat.width().definitely_less(a.union(b).width().mul(min_progress))
                && (inside(hat.lower(), ranges[dir]) || inside(hat.upper(), ranges[dir]))
        })
        .collect();
    if !cut.iter().any(|&c| c) {
        return Ok(None);
    }
    Ok(Some(
        (0..hats.len())
            .map(|dir| {
                if cut[dir] {
                    (hats[dir].lower(), hats[dir].upper())
                } else {
                    ranges[dir]
                }
            })
            .collect(),
    ))
}

/// What a clipping search does with a box on which clipping has stalled
/// (no [`restriction`], or it could not be made).
pub(crate) enum Stalled {
    /// The box is at the resolution the data has: report it.
    Converged,
    /// Bisect, trying directions in this order. Empty means nothing is left
    /// to split, so the box is reported as it is.
    Bisect(Vec<usize>),
}

/// The rest of the shared schedule, for a box clipping could not shrink:
/// either several solutions (or a tangency) share it, or it has reached the
/// resolution its data has.
///
/// It has converged once each spatial [`extent`] in `sizes` is no longer
/// definitely greater than `min_subdivision_size` — or than `carried`, the
/// interval width the geometry being compared carries (see
/// [`carried_width`]), whichever is larger. `carried` is the widest of *all*
/// objects in the comparison: resolving one object finer than another's
/// width answers nothing new — every piece within that width stays a
/// candidate — and only multiplies them.
///
/// Measured on the extents — the part of a piece that splitting shrinks —
/// and not on its bounding box, which also holds that carried width. An
/// interpolated curve honestly widened by its drift
/// (`NurbCurve::interpolate_enclosing`) has a box no split can bring under
/// that width; and once its extent is under it, splitting further cannot
/// sharpen the answer either — every sub-piece still carries the full width —
/// it only multiplies the pieces that survive. So subdivision stops at the
/// resolution the data actually has. `min_subdivision_size` thus bounds only
/// how far *bisection* goes, never how far clipping goes: only effort
/// depends on it, never what a converged answer claims.
///
/// Otherwise the box is bisected, preferring the direction with the largest
/// *spatial* extent `sizes[dir]`: parameter widths from differently scaled
/// domains aren't comparable (`curve_surface.md` §3), and a direction with no
/// extent at all — `u` along a pole row — gains nothing from a split, while
/// every piece of it survives, so it is skipped, as is one whose domain is
/// no wider than `min_subdivision_size`.
pub(crate) fn stalled<S: Scalar>(
    ranges: &[(S, S)],
    sizes: &[f64],
    carried: f64,
    min_subdivision_size: S,
) -> Stalled {
    let resolution = S::from_f64(carried).union(min_subdivision_size).upper();
    if sizes
        .iter()
        .all(|&e| !S::from_f64(e).definitely_greater(resolution))
    {
        return Stalled::Converged;
    }
    let mut order: Vec<usize> = (0..ranges.len())
        .filter(|&dir| {
            let (a, b) = ranges[dir];
            sizes[dir] > 0.0 && a.union(b).width().definitely_greater(min_subdivision_size)
        })
        .collect();
    order.sort_by(|&a, &b| sizes[b].total_cmp(&sizes[a]));
    Stalled::Bisect(order)
}

/// The interval width `points` carry: the widest Cartesian coordinate of any
/// of them. What no subdivision of the object they define can shrink.
pub(crate) fn carried_width<S: Scalar, const D: usize>(points: &[Vector<S, D>]) -> f64 {
    points
        .iter()
        .flat_map(|q| {
            let w = q[D - 1].to_f64();
            (0..D - 1).map(move |k| q[k].width().to_f64() / w)
        })
        .fold(0.0, f64::max)
}

/// Spatial extent of an object along one parameter direction, for choosing
/// which direction to bisect: the largest control-polygon length over the
/// `rows` running along it. By the convex hull property this bounds how far
/// the object reaches in that direction (corner chords don't: a closed or
/// folded row has a zero chord), and it is exactly zero along a collapsed
/// row. A free choice, so plain `f64`.
pub(crate) fn extent<S: Scalar, const D: usize>(
    rows: impl IntoIterator<Item = impl IntoIterator<Item = Vector<S, D>>>,
) -> f64 {
    rows.into_iter()
        .map(|row| {
            let pts: Vec<[f64; 3]> = row
                .into_iter()
                .map(|q| {
                    let w = q[D - 1].to_f64();
                    std::array::from_fn(|k| if k < D - 1 { q[k].to_f64() / w } else { 0.0 })
                })
                .collect();
            pts.windows(2)
                .map(|p| {
                    (0..3)
                        .map(|k| (p[1][k] - p[0][k]).powi(2))
                        .sum::<f64>()
                        .sqrt()
                })
                .sum::<f64>()
        })
        .fold(0.0, f64::max)
}

/// Directions for *combining* the per-axis equations. For any fixed `n`,
/// `g_n = Σ_k n_k g_k` vanishes wherever every `g_k` does, so clipping it is
/// exactly as sound as clipping the axes — and `n` is a free choice, so it
/// is used sharp. Choosing `n` perpendicular to how the *other* object
/// varies (its [`chord`]) makes `g_n` nearly independent of that object's
/// parameters, so collapsing them by union loses little: the classic Bézier
/// clipping choice. It recovers exactly the coupling the axis projections
/// lose (`intersection/curve_curve.md` §3's diagonal example gives `2s - 1`
/// and `2t - 1`).
///
/// Returns the unit vectors `dirs`, sharpened, plus the coordinate axes if
/// `dirs` don't span space well: the equations along them must together
/// pin down every coordinate, or a configuration where they all coincide —
/// a curve lying in a patch's plane makes the normal and both `c × e`
/// parallel — clips nothing at all. Conditioning is a free choice here (it
/// affects only how well the equations clip, never what they prove), so a
/// well-conditioned residual of ½ in Gram–Schmidt is simply required.
pub(crate) fn spanning<S: Scalar, const C: usize>(
    mut dirs: Vec<Vector<S, C>>,
) -> Vec<Vector<S, C>> {
    let quarter = S::from_f64(0.25);
    let mut basis: Vec<Vector<S, C>> = Vec::new();
    for d in &dirs {
        let e = basis
            .iter()
            .fold(*d, |e, b| e.sub(&b.prod_scalar(e.prod_dot(b))));
        if e.norm_sq().definitely_greater(quarter) {
            basis.extend(e.normalize().ok().map(|e| e.sharpen()));
        }
    }
    if basis.len() < C {
        dirs.extend((0..C).map(Vector::axis));
    }
    dirs.into_iter().map(|d| d.sharpen()).collect()
}

/// The sharp Cartesian chord from the first to the last of homogeneous
/// `control_points` — only ever used to *choose* a direction ([`spanning`]),
/// so sharpening is a free choice.
pub(crate) fn chord<S: Scalar, const D: usize, const C: usize>(
    control_points: &[Vector<S, D>],
) -> Vector<S, C> {
    let ends =
        dehomogenize::<S, D, C>(&[control_points[0], control_points[control_points.len() - 1]]);
    ends[1].sub(&ends[0]).sharpen()
}
