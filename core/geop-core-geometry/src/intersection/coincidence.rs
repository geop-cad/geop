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

/// A point of the primary curve at parameter `t` that lies on the other
/// object at `partner` (its parameter(s) there).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Hit<S: Scalar, P> {
    pub t: S,
    pub partner: P,
}

/// A stretch `[start.t, end.t]` of the primary curve lying on the other
/// object.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Overlap<S: Scalar, P> {
    pub start: Hit<S, P>,
    pub end: Hit<S, P>,
}

/// Every overlap bounded by `candidates` (see the module doc). `probe(t)`
/// locates the primary curve's point at a sharp `t` on the other object.
/// Adjacent overlaps sharing a candidate are joined into one.
pub(crate) fn find_overlaps<S: Scalar, P: Mergeable>(
    mut candidates: Vec<Hit<S, P>>,
    mut probe: impl FnMut(S) -> GeopResult<Option<P>>,
) -> GeopResult<Vec<Overlap<S, P>>> {
    // Sorting is only an order to visit them in: any order of candidates
    // that could be equal is fine, since those merge.
    candidates.sort_by(|a, b| a.t.to_f64().total_cmp(&b.t.to_f64()));
    let mut merged: Vec<Hit<S, P>> = Vec::with_capacity(candidates.len());
    for c in candidates {
        match merged.last_mut() {
            Some(last) if last.t.could_be_equal(c.t) => {
                last.t = last.t.union(c.t);
                last.partner = last.partner.union(&c.partner);
            }
            _ => merged.push(c),
        }
    }

    let mut overlaps: Vec<Overlap<S, P>> = Vec::new();
    let mut joins_previous = false;
    for pair in merged.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        // A free choice strictly between the two candidates.
        let mid = a.t.upper().add(b.t.lower()).div(S::TWO)?.sharpen();
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

/// The primary curve's domain `domain` minus every overlap: the stretches
/// where only isolated crossings can occur, as sharp `(lo, hi)` bounds cut
/// at the overlaps' *outer* bounds. Stretches that aren't definitely longer
/// than a point are dropped.
pub(crate) fn gaps<S: Scalar, P>(domain: (S, S), overlaps: &[Overlap<S, P>]) -> Vec<(S, S)> {
    let mut out = Vec::new();
    let mut lo = domain.0;
    for o in overlaps {
        out.push((lo, o.start.t.lower()));
        lo = o.end.t.upper();
    }
    out.push((lo, domain.1));
    out.retain(|&(a, b)| a.definitely_less(b));
    out
}

/// `count` points spread evenly over the overlaps, by length. `probe` as in
/// [`find_overlaps`]; a sample it cannot place is skipped.
pub(crate) fn samples<S: Scalar, P>(
    overlaps: &[Overlap<S, P>],
    count: usize,
    mut probe: impl FnMut(S) -> GeopResult<Option<P>>,
) -> GeopResult<Vec<Hit<S, P>>> {
    let spans: Vec<(S, S)> = overlaps
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
            let frac = S::from_ratio(j as i64, (n + 1) as i64)?;
            let t = lo.add(hi.sub(lo).mul(frac)).sharpen();
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
pub(crate) fn assemble<S: Scalar, P: Mergeable>(
    overlaps: &[Overlap<S, P>],
    crossings: Vec<(S, P)>,
    samples: Vec<Hit<S, P>>,
    max_solutions: usize,
) -> Intersections<(S, P)> {
    let mut set: DisjointSet<(S, P)> = DisjointSet::new();
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
