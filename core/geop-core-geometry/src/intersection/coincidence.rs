//! Coincidence handling around the clipping searches (`curve_curve`,
//! `curve_surface`), which themselves assume no coincident arcs.
//!
//! A *primary* curve (parameter `t`) overlaps another object — a curve, or a
//! surface — along stretches of `t`. For single-piece rational geometry,
//! such a stretch can only begin or end where one object's boundary is: an
//! end of the primary curve lying on the other object, or a point of the
//! primary curve on the other object's boundary (the other curve's ends; the
//! surface's four boundary curves). Collecting those *candidates* and
//! testing the midpoint between each consecutive pair therefore finds every
//! overlap, without a search that could only ever see an overlap as "too
//! many solutions":
//!
//! - candidates sorted by `t`; ones that could be equal merge (a shared
//!   vertex seen from both sides is one candidate);
//! - between consecutive candidates, the primary curve either lies on the
//!   other object throughout or nowhere in the interior — so one probe at the
//!   midpoint decides it.
//!
//! The midpoint probe is the one place this is a test rather than a proof:
//! a curve touching the other object exactly at two candidates *and* the
//! midpoint between them, while leaving it in between, is read as an overlap.

use geop_core_math::{
    disjoint_set::{DisjointSet, Mergeable},
    geop_error::GeopResult,
    scalars::Scalar,
};

use super::Intersections;
use crate::nurb_curve::NurbCurve;

/// A point of the primary curve at parameter `t` that lies on the other
/// object at `partner` (its parameter(s) there).
#[derive(Clone, Copy, Debug)]
pub struct Hit<P> {
    pub t: Scalar,
    pub partner: P,
}

/// A stretch `[start.t, end.t]` of the primary curve lying on the other
/// object.
#[derive(Clone, Copy, Debug)]
pub struct Overlap<P> {
    pub start: Hit<P>,
    pub end: Hit<P>,
}

/// Every overlap bounded by `candidates` (see the module doc). `probe(t)`
/// locates the primary curve's point at a sharp `t` on the other object.
/// Adjacent overlaps sharing a candidate are joined into one.
pub(crate) fn find_overlaps<P: Mergeable>(
    mut candidates: Vec<Hit<P>>,
    mut probe: impl FnMut(Scalar) -> GeopResult<Option<P>>,
) -> GeopResult<Vec<Overlap<P>>> {
    // Sorting is only an order to visit them in: any order of candidates
    // that could be equal is fine, since those merge.
    candidates.sort_by(|a, b| a.t.to_f64().total_cmp(&b.t.to_f64()));
    let mut merged: Vec<Hit<P>> = Vec::with_capacity(candidates.len());
    for c in candidates {
        match merged.last_mut() {
            Some(last) if last.t.could_be_equal(c.t) => {
                last.t = last.t.union(c.t);
                last.partner = last.partner.union(&c.partner);
            }
            _ => merged.push(c),
        }
    }

    let mut overlaps: Vec<Overlap<P>> = Vec::new();
    let mut joins_previous = false;
    for pair in merged.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        // A free choice strictly between the two candidates.
        let mid = a.t.upper().add(b.t.lower()).div(Scalar::TWO)?.sharpen();
        if probe(mid)?.is_some() {
            match overlaps.last_mut() {
                Some(last) if joins_previous => last.end = b,
                _ => overlaps.push(Overlap { start: a, end: b }),
            }
            joins_previous = true;
        } else {
            joins_previous = false;
        }
    }
    Ok(overlaps)
}

/// The isolated crossings of the primary `curve` away from `overlaps`:
/// `search` run on the whole curve if there are none, else on each stretch
/// between them — cut at the overlaps' sharp *outer* bounds, and only where
/// definitely longer than a point.
pub(crate) fn crossings_between<const D: usize, P, R>(
    curve: &NurbCurve<D>,
    overlaps: &[Overlap<P>],
    mut search: impl FnMut(&NurbCurve<D>) -> GeopResult<Vec<R>>,
) -> GeopResult<Vec<R>> {
    if overlaps.is_empty() {
        return search(curve);
    }
    let (start, end) = curve.domain();
    let mut bounds = vec![start];
    for o in overlaps {
        bounds.extend([o.start.t.lower(), o.end.t.upper()]);
    }
    bounds.push(end);
    let mut crossings = Vec::new();
    for gap in bounds.chunks(2) {
        if gap[0].definitely_less(gap[1]) {
            let piece = curve.sub_curve(gap[0], gap[1])?;
            crossings.extend(search(&piece).map_err(|e| {
                e.with_context(format!(
                    "on the stretch {:?} of the curve",
                    (gap[0], gap[1])
                ))
            })?);
        }
    }
    Ok(crossings)
}

/// Points spread evenly over the overlaps, by length: as many as
/// [`assemble`] has room for under `max_solutions` after the overlaps' ends
/// and the `crossings` it puts first. `probe` as in [`find_overlaps`]; a
/// sample it cannot place is skipped.
///
/// As many as there is room for, not `max_solutions` cut short: spread for
/// more than are kept, the ones kept were all at the start of the first
/// overlap. Three points of a blend's contact 7.6e-7 long, both ends and one
/// a quarter along, were the three a boolean judged it by, and each was
/// within a pcurve's width of the edge it continues, so it was never found
/// to enter the face it lies in. The middle is the point furthest from both.
pub(crate) fn samples<P>(
    overlaps: &[Overlap<P>],
    crossings: usize,
    max_solutions: usize,
    mut probe: impl FnMut(Scalar) -> GeopResult<Option<P>>,
) -> GeopResult<Vec<Hit<P>>> {
    let count = max_solutions.saturating_sub(2 * overlaps.len() + crossings);
    let spans: Vec<(Scalar, Scalar)> = overlaps
        .iter()
        .map(|o| (o.start.t.upper(), o.end.t.lower()))
        .collect();
    let lengths: Vec<f64> = spans
        .iter()
        .map(|(a, b)| (b.to_f64() - a.to_f64()).max(0.0))
        .collect();
    let total: f64 = lengths.iter().sum();
    let mut out = Vec::new();
    for ((lo, hi), len) in spans.into_iter().zip(lengths) {
        let n = if total > 0.0 {
            ((count as f64) * len / total).ceil() as usize
        } else {
            count
        };
        for j in 1..=n {
            // Free choices inside the overlap.
            let frac = Scalar::from_ratio(j as i64, (n + 1) as i64)?;
            let t = Scalar::fraction(lo, hi, frac);
            if let Some(partner) = probe(t)? {
                out.push(Hit { t, partner });
            }
        }
    }
    Ok(out)
}

/// The wrapper's result, with the old `Intersections` contract:
/// `Coincident` iff an overlap was found, carrying the overlaps' end points
/// first (where the objects part — the points that matter for splitting),
/// then the isolated crossings elsewhere, then the spread `samples`, merged
/// (`DisjointSet`) and capped at `max_solutions`. Without overlaps it is
/// `Found` with the first `max_solutions` crossings.
pub(crate) fn assemble<P: Mergeable>(
    overlaps: &[Overlap<P>],
    crossings: Vec<(Scalar, P)>,
    samples: Vec<Hit<P>>,
    max_solutions: usize,
) -> Intersections<(Scalar, P)> {
    let mut set: DisjointSet<(Scalar, P)> = DisjointSet::new();
    let ends = overlaps.iter().flat_map(|o| [o.start, o.end]);
    let points = ends
        .map(|h| (h.t, h.partner))
        .chain(crossings)
        .chain(samples.into_iter().map(|h| (h.t, h.partner)));
    for p in points {
        if set.len() >= max_solutions {
            break;
        }
        set.insert(p);
    }
    let mut v = set.into_vec();
    v.truncate(max_solutions);
    if overlaps.is_empty() {
        Intersections::Found(v)
    } else {
        Intersections::Coincident(v)
    }
}
