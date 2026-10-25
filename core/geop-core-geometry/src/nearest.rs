//! [`least_distance`]: the least distance between two sets of points — a
//! point, curves, surfaces, trimmed faces — and two points attaining it,
//! by branch and bound over pieces of them.
//!
//! # How
//!
//! A [`Piece`] is part of one of the sets: a span of a curve, a patch of a
//! surface. It knows points whose convex hull holds all of it (its control
//! points: every weight is positive, so a rational piece lies in their
//! hull too), a point it attains, and how to cut itself in two. For a pair
//! of pieces the hulls give a lower bound on the distance between them,
//! and the attained points an upper bound on the least distance overall.
//! Pairs are taken least lower bound first; a pair whose lower bound is
//! definitely above the best distance attained holds nothing nearer and is
//! dropped, and any other is cut, its larger piece in two. The search is
//! done once the attained distance is not definitely above the least lower
//! bound left: nothing can then be shown to be nearer.
//!
//! The lower bound is the separation of the two hulls along the direction
//! between them, or of their boxes, whichever is more. Along the right
//! direction it is off by the square of a piece's size, not by the size
//! itself, as a box's is: near a minimum that keeps a handful of pairs
//! alive at each level, rather than a number growing with every level.
//!
//! # What is returned
//!
//! Two attained points, and the least distance as an enclosure: from the
//! least lower bound of every pair still standing to the distance between
//! the two points. Pieces stop being cut once they are smaller than the
//! caller's `handoff` (see "Subdivide to isolate, then Newton to refine"
//! in `AGENTS.md`), and the search stops at `max_nodes` pairs; either only
//! leaves the enclosure wider, never wrong. The pairs left at the handoff
//! are returned too, for a caller to refine each with Newton.
//!
//! A piece is also left as it is once its points spread no further than
//! they are wide ([`resolved`]). Its hull points are enclosures, and the
//! bound from them is off by their width as well as by the piece's size:
//! cutting halves the size, never the width. A span of a traced curve, its
//! control points `2e-6` wide, was cut until its domain was two ULPs wide
//! because its box never came under the handoff; the distance it bounds
//! is as wide as its data, and no cut could make it sharper.

use std::{cmp::Ordering, collections::BinaryHeap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
};

use crate::{
    contains::surface::surface_extents,
    nurb_curve::{NurbCurve3D, dehomogenize},
    nurb_surface::NurbSurface3D,
};

/// Part of a set a distance is measured to (see the module docs).
pub trait Piece: Sized {
    /// Where on the whole a point of the piece is: its parameters, say.
    type At: Clone;

    /// Points whose convex hull holds every point of the piece.
    fn hull(&self) -> GeopResult<Vec<Vector3>>;

    /// A point of the piece and where it is — if one is known to belong to
    /// the set: a patch of a trimmed face may hold none of the face.
    fn point(&self) -> GeopResult<Option<(Self::At, Vector3)>>;

    /// What is left of the piece cut in two — a half known to hold nothing
    /// of the set left out — or `None` once it is within `handoff` in
    /// size.
    fn split(&self, handoff: Scalar) -> GeopResult<Option<Vec<Self>>>;
}

/// The least distance between two sets, as [`least_distance`] finds it.
#[derive(Clone, Debug)]
pub struct Nearest<A, B> {
    /// The point of the first set, and where on it.
    pub a: (A, Vector3),
    /// The point of the second set, and where on it.
    pub b: (B, Vector3),
    /// The least distance between the sets: at least the search's lower
    /// bound, at most the distance between `a` and `b`.
    pub distance: Scalar,
    /// Where on each set the pairs left at the handoff attain a point:
    /// where else a least distance could be, for Newton to refine.
    pub candidates: Vec<(A, B)>,
}

/// A pair of pieces waiting to be searched, with what bounds it.
struct Pair<PA: Piece, PB: Piece> {
    a: PA,
    b: PB,
    hull_a: Vec<Vector3>,
    hull_b: Vec<Vector3>,
    /// A lower bound on the distance between the two pieces.
    lower: Scalar,
}

impl<PA: Piece, PB: Piece> Pair<PA, PB> {
    fn new(a: PA, b: PB) -> GeopResult<Self> {
        let (hull_a, hull_b) = (a.hull()?, b.hull()?);
        let lower = separation(&hull_a, &hull_b)?;
        Ok(Self {
            a,
            b,
            hull_a,
            hull_b,
            lower,
        })
    }

    /// Which pair to take first: the least lower bound. An order only,
    /// never an answer.
    fn key(&self) -> f64 {
        self.lower.lower().to_f64()
    }
}

impl<PA: Piece, PB: Piece> PartialEq for Pair<PA, PB> {
    fn eq(&self, other: &Self) -> bool {
        self.key() == other.key()
    }
}
impl<PA: Piece, PB: Piece> Eq for Pair<PA, PB> {}
impl<PA: Piece, PB: Piece> PartialOrd for Pair<PA, PB> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<PA: Piece, PB: Piece> Ord for Pair<PA, PB> {
    /// Reversed: [`BinaryHeap`] pops its greatest.
    fn cmp(&self, other: &Self) -> Ordering {
        other.key().total_cmp(&self.key())
    }
}

/// The box around `points`: per axis, the union of their coordinates.
fn bounds(points: &[Vector3]) -> [Scalar; 3] {
    std::array::from_fn(|k| {
        points
            .iter()
            .map(|p| p[k])
            .reduce(|a, b| a.union(b))
            .unwrap_or(Scalar::ENTIRE)
    })
}

/// A lower bound on the distance between any point of the hull of `a` and
/// any of the hull of `b`: the more of how far apart their boxes are, and
/// how far apart they are along the direction from one box's centre to
/// the other's. Which direction is a free choice; any gives a bound.
fn separation(a: &[Vector3], b: &[Vector3]) -> GeopResult<Scalar> {
    let (box_a, box_b) = (bounds(a), bounds(b));
    // Apart on an axis by how much the one box's lower end is above the
    // other's upper end, if it definitely is.
    let gap = |x: Scalar, y: Scalar| {
        let ahead = y.lower().sub(x.upper());
        let behind = x.lower().sub(y.upper());
        if ahead.definitely_greater(Scalar::ZERO) {
            ahead
        } else if behind.definitely_greater(Scalar::ZERO) {
            behind
        } else {
            Scalar::ZERO
        }
    };
    let boxes = (0..3)
        .map(|k| gap(box_a[k], box_b[k]))
        .fold(Scalar::ZERO, |sum, g| sum.add(g.mul(g)))
        .sqrt()?;

    let centre = |b: &[Scalar; 3]| Vector3::from_array(b.map(|x| x.midpoint()));
    let d = centre(&box_b).sub(&centre(&box_a)).sharpen();
    let along = |points: &[Vector3]| -> Scalar {
        points
            .iter()
            .map(|p| p.prod_dot(&d))
            .reduce(|x, y| x.union(y))
            .unwrap_or(Scalar::ENTIRE)
    };
    let apart = along(b).lower().sub(along(a).upper());
    let length = d.norm();
    let directed =
        if apart.definitely_greater(Scalar::ZERO) && length.definitely_greater(Scalar::ZERO) {
            apart.div(length.upper())?.lower()
        } else {
            Scalar::ZERO
        };
    Ok(if directed.to_f64() > boxes.lower().to_f64() {
        directed
    } else {
        boxes.lower()
    })
}

/// Whether cutting a piece whose hull is `points` cannot tighten its bound:
/// the points' midpoints spread, on no axis, definitely further than the
/// widest of them is wide. A cut halves the spread and keeps the width,
/// so the bound is then as sharp as the piece's enclosures can make it.
fn resolved(points: &[Vector3]) -> bool {
    let midpoints: Vec<Vector3> = points
        .iter()
        .map(|p| Vector3::from_array(p.to_array().map(|x| x.midpoint())))
        .collect();
    let spread = bounds(&midpoints);
    (0..3).all(|k| {
        let widest = points
            .iter()
            .map(|p| p[k].width())
            .fold(
                Scalar::ZERO,
                |m, w| if w.to_f64() > m.to_f64() { w } else { m },
            );
        !spread[k].width().definitely_greater(widest)
    })
}

/// The least distance attained so far, and the two points attaining it.
type Best<A, B> = Option<(Scalar, (A, Vector3), (B, Vector3))>;

/// Keeps the points `a` and `b` attain as the best pair, if they attain
/// points and are nearer. Which of two attained pairs to keep is a free
/// choice.
fn offer<PA: Piece, PB: Piece>(best: &mut Best<PA::At, PB::At>, a: &PA, b: &PB) -> GeopResult<()> {
    let (Some(pa), Some(pb)) = (a.point()?, b.point()?) else {
        return Ok(());
    };
    let d = pa.1.sub(&pb.1).norm();
    if best
        .as_ref()
        .is_none_or(|(e, ..)| d.midpoint().to_f64() < e.midpoint().to_f64())
    {
        *best = Some((d, pa, pb));
    }
    Ok(())
}

/// Whether `pair` definitely holds nothing nearer than `best`.
fn beyond<PA: Piece, PB: Piece>(pair: &Pair<PA, PB>, best: &Best<PA::At, PB::At>) -> bool {
    best.as_ref()
        .is_some_and(|(d, ..)| pair.lower.definitely_greater(*d))
}

/// The least distance between the set `a` is pieces of and the set `b` is
/// pieces of (see the module docs).
///
/// Fails only if neither set attains a point in any piece searched.
pub fn least_distance<PA: Piece + Clone, PB: Piece + Clone>(
    a: Vec<PA>,
    b: Vec<PB>,
    handoff: Scalar,
    max_nodes: usize,
) -> GeopResult<Nearest<PA::At, PB::At>> {
    let mut best: Best<PA::At, PB::At> = None;

    let mut pending = BinaryHeap::new();
    for pa in &a {
        for pb in &b {
            offer(&mut best, pa, pb)?;
            pending.push(Pair::new(pa.clone(), pb.clone())?);
        }
    }
    let mut leaves: Vec<Pair<PA, PB>> = Vec::new();
    let mut visited = 0usize;
    while let Some(pair) = pending.pop() {
        if beyond(&pair, &best) {
            // Every pair waiting has a lower bound at least this one's.
            pending.clear();
            break;
        }
        // Nothing waiting is definitely nearer than what is attained: the
        // enclosure from this lower bound to the attained distance is as
        // tight as the numbers tell. Touching faces end here, where every
        // pair of their patches could be at distance zero.
        let converged = best
            .as_ref()
            .is_some_and(|(d, ..)| !d.definitely_greater(pair.lower));
        if converged || visited >= max_nodes {
            pending.push(pair);
            break;
        }
        visited += 1;
        // Cut the larger piece of the pair, or the other if it cannot be:
        // a piece is not cut below the handoff, nor once its data resolves
        // nothing finer (see the module docs).
        let size = |hull: &[Vector3]| -> f64 {
            bounds(hull)
                .iter()
                .map(|x| x.width().to_f64().powi(2))
                .sum::<f64>()
        };
        let a_first = size(&pair.hull_a) >= size(&pair.hull_b);
        let cut_a = |pair: &Pair<PA, PB>| -> GeopResult<Option<Vec<(PA, PB)>>> {
            if resolved(&pair.hull_a) {
                return Ok(None);
            }
            Ok(pair
                .a
                .split(handoff)?
                .map(|halves| halves.into_iter().map(|h| (h, pair.b.clone())).collect()))
        };
        let cut_b = |pair: &Pair<PA, PB>| -> GeopResult<Option<Vec<(PA, PB)>>> {
            if resolved(&pair.hull_b) {
                return Ok(None);
            }
            Ok(pair
                .b
                .split(handoff)?
                .map(|halves| halves.into_iter().map(|h| (pair.a.clone(), h)).collect()))
        };
        let children = if a_first {
            match cut_a(&pair)? {
                Some(c) => Some(c),
                None => cut_b(&pair)?,
            }
        } else {
            match cut_b(&pair)? {
                Some(c) => Some(c),
                None => cut_a(&pair)?,
            }
        };
        let Some(children) = children else {
            leaves.push(pair);
            continue;
        };
        for (ca, cb) in children {
            offer(&mut best, &ca, &cb)?;
            let child = Pair::new(ca, cb)?;
            if !beyond(&child, &best) {
                pending.push(child);
            }
        }
    }

    let (attained, pa, pb) = best.ok_or_else(|| {
        GeopError::new("least_distance: no piece of one of the sets holds a point of it")
    })?;
    // Nothing nearer than the least lower bound of what is still standing.
    let standing = leaves.iter().chain(pending.iter());
    let lower = standing
        .map(|p| p.lower.lower())
        .reduce(|x, y| if y.to_f64() < x.to_f64() { y } else { x })
        .unwrap_or(attained);
    let mut candidates = Vec::new();
    for leaf in leaves
        .iter()
        .filter(|leaf| !leaf.lower.definitely_greater(attained))
    {
        if let (Some(a), Some(b)) = (leaf.a.point()?, leaf.b.point()?) {
            candidates.push((a.0, b.0));
        }
    }
    Ok(Nearest {
        a: pa,
        b: pb,
        distance: lower.lower().union(attained),
        candidates,
    })
}

/// A single point: a set of one piece that cannot be cut.
#[derive(Clone, Debug)]
pub struct PointPiece(pub Vector3);

impl Piece for PointPiece {
    type At = ();

    fn hull(&self) -> GeopResult<Vec<Vector3>> {
        Ok(vec![self.0])
    }

    fn point(&self) -> GeopResult<Option<((), Vector3)>> {
        Ok(Some(((), self.0)))
    }

    fn split(&self, _: Scalar) -> GeopResult<Option<Vec<Self>>> {
        Ok(None)
    }
}

/// A span of a 3-D curve: a piece of the curve's points, `At` its
/// parameter.
#[derive(Clone, Debug)]
pub struct CurveSpan(pub NurbCurve3D);

impl Piece for CurveSpan {
    type At = Scalar;

    fn hull(&self) -> GeopResult<Vec<Vector3>> {
        Ok(dehomogenize::<4, 3>(&self.0.control_points))
    }

    fn point(&self) -> GeopResult<Option<(Scalar, Vector3)>> {
        let t = self.0.mid_parameter()?;
        Ok(Some((t, self.0.evaluate(t)?)))
    }

    fn split(&self, handoff: Scalar) -> GeopResult<Option<Vec<Self>>> {
        if !self.0.size()?.definitely_greater(handoff) {
            return Ok(None);
        }
        let (a, b) = self.0.split_mid().with_context(&|e: GeopError| {
            e.with_context(format!(
                "CurveSpan::split(domain={:?}, handoff={handoff:?}, control_points={:?})",
                self.0.domain(),
                self.0.control_points
            ))
        })?;
        Ok(Some(vec![CurveSpan(a), CurveSpan(b)]))
    }
}

/// A patch of a surface: a piece of the surface's points, `At` its
/// parameters.
#[derive(Clone, Debug)]
pub struct SurfacePatch(pub NurbSurface3D);

impl SurfacePatch {
    /// The patch in two, across the direction it is longer in.
    pub fn halves(&self) -> GeopResult<(Self, Self)> {
        let [along_u, along_v] = surface_extents(&self.0);
        let (a, b) = if along_u >= along_v {
            self.0.split_u_mid()?
        } else {
            self.0.split_v_mid()?
        };
        Ok((SurfacePatch(a), SurfacePatch(b)))
    }

    /// The middle of its domain, sharp: a free choice of a point of it.
    pub fn middle(&self) -> GeopResult<(Scalar, Scalar)> {
        let mid = |(lo, hi): (Scalar, Scalar)| -> GeopResult<Scalar> {
            Ok(lo.add(hi).div(Scalar::TWO)?.sharpen())
        };
        Ok((mid(self.0.domain_u())?, mid(self.0.domain_v())?))
    }
}

impl Piece for SurfacePatch {
    type At = (Scalar, Scalar);

    fn hull(&self) -> GeopResult<Vec<Vector3>> {
        Ok(dehomogenize::<4, 3>(&self.0.control_points))
    }

    fn point(&self) -> GeopResult<Option<((Scalar, Scalar), Vector3)>> {
        let (u, v) = self.middle()?;
        Ok(Some(((u, v), self.0.evaluate(u, v)?)))
    }

    fn split(&self, handoff: Scalar) -> GeopResult<Option<Vec<Self>>> {
        if !self.0.size()?.definitely_greater(handoff) {
            return Ok(None);
        }
        let (a, b) = self.halves()?;
        Ok(Some(vec![a, b]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurb_curve::NurbCurve;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    /// A piece of the segment `[lo, hi]` of the `x` axis.
    #[derive(Clone)]
    struct Segment(Scalar, Scalar);

    impl Piece for Segment {
        type At = Scalar;

        fn hull(&self) -> GeopResult<Vec<Vector3>> {
            let at = |x: Scalar| Vector3::from_array([x, Scalar::ZERO, Scalar::ZERO]);
            Ok(vec![at(self.0), at(self.1)])
        }

        fn point(&self) -> GeopResult<Option<(Scalar, Vector3)>> {
            let x = self.0.add(self.1).div(Scalar::TWO)?.sharpen();
            Ok(Some((
                x,
                Vector3::from_array([x, Scalar::ZERO, Scalar::ZERO]),
            )))
        }

        fn split(&self, handoff: Scalar) -> GeopResult<Option<Vec<Self>>> {
            if !self.1.sub(self.0).definitely_greater(handoff) {
                return Ok(None);
            }
            let mid = self.0.add(self.1).div(Scalar::TWO)?.sharpen();
            Ok(Some(vec![Segment(self.0, mid), Segment(mid, self.1)]))
        }
    }

    /// A span of the rim of a hole drilled by a boolean, traced and fitted:
    /// its control points are as wide as the trace, `2e-6` in `y`,
    /// `2e-12` in `z`, beside a sharp edge of the same top, measured to two
    /// points of the bottom, one below. The sharp edge attains the distance
    /// one sharply; the rim's lower bound is one less its width in `z`, so
    /// the pair is neither beyond the best nor converged. Its box can never
    /// shrink below its points' widths, so cutting it never brought its
    /// size under the handoff, and the search cut it until its domain was
    /// two ULPs wide and could not be split. The distance is one, enclosed
    /// as wide as the data is.
    #[test]
    fn traced_span_beside_a_sharp_edge_above_two_points() {
        let cp = |x: [f64; 2], y: [f64; 2], z: [f64; 2], w: [f64; 2]| {
            Vector4::from_array([
                Scalar::new(x[0], x[1]),
                Scalar::new(y[0], y[1]),
                Scalar::new(z[0], z[1]),
                Scalar::new(w[0], w[1]),
            ])
        };
        let span = NurbCurve::try_new(
            3,
            vec![
                cp(
                    [1.0502014289205366, 1.0502015605016914],
                    [1.3968357901928796, 1.3968380515018717],
                    [0.9999999999990593, 1.0000000000009528],
                    [0.9999999999999375, 1.0000000000000744],
                ),
                cp(
                    [1.0502014289205344, 1.0502015605016937],
                    [1.396835790192877, 1.3968380515018743],
                    [0.9999999999990575, 1.000000000000955],
                    [0.9999999999999357, 1.0000000000000766],
                ),
                cp(
                    [1.050201428920532, 1.0502015605016959],
                    [1.3968357901928743, 1.396838051501877],
                    [0.9999999999990558, 1.0000000000009572],
                    [0.9999999999999339, 1.0000000000000788],
                ),
                cp(
                    [1.0502014289205295, 1.050201560501698],
                    [1.3968357901928716, 1.3968380515018797],
                    [0.999999999999054, 1.0000000000009595],
                    [0.9999999999999322, 1.000000000000081],
                ),
            ],
            [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
                .map(Scalar::from_f64)
                .to_vec(),
        )
        .unwrap();
        let at = |p: [f64; 3]| Vector3::from_array(p.map(Scalar::from_f64));
        let edge = crate::test_fixtures::line([4.0, 4.0, 1.0], [5.0, 5.0, 1.0]);
        let nearest = least_distance(
            vec![
                PointPiece(at([1.0502015, 1.3968369, 0.0])),
                PointPiece(at([4.5, 4.5, 0.0])),
            ],
            vec![CurveSpan(span), CurveSpan(edge)],
            Scalar::from_f64(1e-7),
            10_000,
        )
        .unwrap();
        assert!(
            nearest.distance.could_be_equal(Scalar::ONE),
            "{:?}",
            nearest.distance
        );
        assert!(
            !nearest
                .distance
                .width()
                .definitely_greater(Scalar::from_f64(1e-5)),
            "{:?}",
            nearest.distance
        );
    }

    /// A point above the middle of a segment is its height from it, and the
    /// enclosure of the least distance holds the true one. Where on the
    /// segment is only as sharp as the square root of that: the distance is
    /// flat at its least, which is why callers refine the point by Newton.
    #[test]
    fn point_above_a_segment() {
        let target = Vector3::from_array([0.3, 2.0, 0.0].map(Scalar::from_f64));
        let nearest = least_distance(
            vec![PointPiece(target)],
            vec![Segment(Scalar::from_f64(-1.0), Scalar::ONE)],
            Scalar::from_f64(1e-6),
            10_000,
        )
        .unwrap();
        assert!(
            nearest.distance.could_be_equal(Scalar::TWO),
            "{:?}",
            nearest.distance
        );
        assert!(
            !nearest
                .distance
                .width()
                .definitely_greater(Scalar::from_f64(1e-6)),
            "{:?}",
            nearest.distance
        );
    }
}
