//! A 3-D sketch's drawn curves as the edges of a wire. A point two drawn
//! curves pass through — an end of one, a point a spline is drawn through,
//! an arc's middle point, or points made one by a
//! [`Constraint3d::Coincident`](super::Constraint3d::Coincident) — is one
//! vertex of the wire, and every curve passing through it is cut there: the
//! wire is a graph whose edges meet only at its vertices, which is what the
//! kernel requires of a body's edges. Curves that meet anywhere else are
//! refused by name, since a crossing that is no point of the sketch is one
//! the user cannot see.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_geometry::{
    intersection::curve_curve::curve_curve_intersect,
    nurb_curve::{NurbCurve, NurbCurve3D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::Vector3,
    with_context,
};

use super::{
    CurveKind3d, Enclosure3d, Sketch3d,
    curves::{arc_pieces, halvings},
    geometry::Arc3,
};
use crate::{CurveId, PointId};

/// Budgets for the crossing search of every pair of drawn curves: how many
/// crossings one pair may have, how hard a single search tries, and the
/// subdivision size it stops isolating at — one part in `MIN_SUBDIVISION`
/// of a curve's domain.
const MAX_CROSSINGS: usize = 16;
const MAX_NODES: usize = 4000;
const MIN_SUBDIVISION: i64 = 1_000_000;

/// An edge of the wire a 3-D sketch is built as: the drawn curve `curve`
/// from its point `start` to its point `end` — the whole curve, or, `cut`,
/// one of the pieces it is cut into at the points it shares with other
/// drawn curves. Points are the lowest id of their class (see
/// [`Sketch3d::point_classes`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WireEdge {
    pub curve: CurveId,
    pub start: PointId,
    pub end: PointId,
    pub cut: bool,
}

impl Sketch3d {
    /// The indices of the points of `curve` between its ends that another
    /// drawn curve passes through too: where the wire cuts it.
    fn junctions(
        &self,
        curve: CurveId,
        class: &BTreeMap<PointId, PointId>,
    ) -> GeopResult<Vec<usize>> {
        let points = self.curve(curve)?.points();
        let others: BTreeSet<PointId> = self
            .drawn_curves()
            .filter(|(id, _)| **id != curve)
            .flat_map(|(_, c)| c.points())
            .map(|p| class[&p])
            .collect();
        Ok((1..points.len().saturating_sub(1))
            .filter(|&k| others.contains(&class[&points[k]]))
            .collect())
    }

    /// The edges of the wire (see the module docs): oldest curve first, each
    /// curve's from its start to its end.
    pub fn wire_edges(&self) -> GeopResult<Vec<WireEdge>> {
        let class = self.point_classes();
        let mut edges = Vec::new();
        for (&curve, c) in self.drawn_curves() {
            let points = c.points();
            if points.len() < 2 {
                return Err(GeopError::new(format!(
                    "the drawn curve {curve} has no ends"
                )));
            }
            let junctions = self.junctions(curve, &class)?;
            let cut = !junctions.is_empty();
            let stations: Vec<PointId> = (std::iter::once(0).chain(junctions))
                .chain(std::iter::once(points.len() - 1))
                .map(|k| class[&points[k]])
                .collect();
            edges.extend(stations.windows(2).map(|w| WireEdge {
                curve,
                start: w[0],
                end: w[1],
                cut,
            }));
        }
        Ok(edges)
    }

    /// The edges of `curve` (see [`Sketch3d::wire_edges`]) as NURBS on
    /// `[0, 1]`, in order, built from `geometry`: the whole curve, or its
    /// pieces between the points it is cut at — a spline split at the knot
    /// of each such point, an arc rebuilt from its middle point to each end.
    /// An edge of several pieces (an arc's quarters) is joined into one
    /// curve; an edge of one piece is that piece as it is, its weights
    /// included: a spline's weights of one come out of each knot inserted
    /// an ulp wide, which is honest and bars nothing.
    pub fn wire_edge_nurbs(
        &self,
        curve: CurveId,
        geometry: &Enclosure3d,
    ) -> GeopResult<Vec<NurbCurve3D>> {
        let ctx = with_context!("3-D sketch curve {curve} as the wire's edges");
        let edge = |pieces: &[NurbCurve3D]| match pieces {
            [one] => one.with_unit_domain(),
            many => NurbCurve::join(many),
        };
        let junctions = self.junctions(curve, &self.point_classes())?;
        let whole = self.curve_nurbs(curve, geometry).with_context(ctx)?;
        if junctions.is_empty() {
            return Ok(vec![edge(&whole).with_context(ctx)?]);
        }
        let at = |p: &PointId| geometry.points[p];
        let pieces: Vec<Vec<NurbCurve3D>> = match &self.curve(curve)?.kind {
            CurveKind3d::Spline { .. } => {
                let [spline] = whole.as_slice() else {
                    return Err(GeopError::new("a spline is one NURBS")).with_context(ctx);
                };
                // Knots `0 0 0 0 t1 .. t(m-2) 1 1 1 1`: the point `k` is
                // passed at the knot `3 + k` (see `NurbCurve::cubic_spline`).
                let knots = spline.knot_vector.clone();
                let mut rest = spline.clone();
                let mut pieces = Vec::new();
                for k in junctions {
                    let (left, right) = rest.split(knots[3 + k]).with_context(ctx)?;
                    pieces.push(vec![left]);
                    rest = right;
                }
                pieces.push(vec![rest]);
                pieces
            }
            CurveKind3d::Arc {
                start,
                through,
                end,
            } => {
                let drawn = Arc3 {
                    s: self.points[start].at,
                    m: self.points[through].at,
                    e: self.points[end].at,
                };
                let arc = Arc3 {
                    s: at(start),
                    m: at(through),
                    e: at(end),
                };
                let normal = arc.normal().with_context(ctx)?;
                let halves = [
                    ((&arc.s, &arc.m), (&drawn.s, &drawn.m)),
                    ((&arc.m, &arc.e), (&drawn.m, &drawn.e)),
                ];
                halves
                    .into_iter()
                    .map(|((p, q), (drawn_p, drawn_q))| {
                        arc_pieces(
                            p,
                            q,
                            &normal,
                            arc.half_sweep_between(p, q)?,
                            halvings(drawn.half_sweep_between(drawn_p, drawn_q)?.0)?,
                        )
                    })
                    .collect::<GeopResult<_>>()
                    .with_context(ctx)?
            }
            CurveKind3d::Line { .. } | CurveKind3d::Reference => {
                return Err(GeopError::new(format!(
                    "curve {curve} has no points between its ends to be cut at"
                )))
                .with_context(ctx);
            }
        };
        (pieces.iter().map(|p| edge(p)))
            .collect::<GeopResult<_>>()
            .with_context(ctx)
    }

    /// Every point of the sketch that is one — the lowest id of its class
    /// (see [`Sketch3d::point_classes`]) — and where it is: the enclosure of
    /// all the points made one with it, at `geometry`.
    pub fn point_enclosures(&self, geometry: &Enclosure3d) -> BTreeMap<PointId, Vector3> {
        let mut out: BTreeMap<PointId, Vector3> = BTreeMap::new();
        for (p, &rep) in &self.point_classes() {
            let at = geometry.points[p];
            out.entry(rep)
                .and_modify(|e| *e = e.union(&at))
                .or_insert(at);
        }
        out
    }

    /// Checks the wire's edges meet only at its vertices (see the module
    /// docs): two points of the sketch are nowhere at one place unless they
    /// are one point, and two drawn curves meet only at points both pass
    /// through. Says which points or curves do otherwise, and where.
    pub fn check_wire(&self, geometry: &Enclosure3d) -> GeopResult<()> {
        let enclosures = self.point_enclosures(geometry);
        let points: Vec<(&PointId, &Vector3)> = enclosures.iter().collect();
        for (i, (p, at)) in points.iter().enumerate() {
            for (q, other) in &points[i + 1..] {
                if at.could_be_equal(other) {
                    return Err(GeopError::new(format!(
                        "points {p} and {q} are both at {at:?}, but are two points of the sketch: \
                         draw through one of them, or make them coincident"
                    )));
                }
            }
        }
        let class = self.point_classes();
        let min_subdivision = Scalar::from_ratio(1, MIN_SUBDIVISION)?;
        let curves: Vec<(CurveId, BTreeSet<PointId>, NurbCurve3D)> = (self.drawn_curves())
            .map(|(&id, c)| {
                let nurbs = NurbCurve::join(&self.curve_nurbs(id, geometry)?)?;
                Ok((id, c.points().iter().map(|p| class[p]).collect(), nurbs))
            })
            .collect::<GeopResult<_>>()?;
        for (i, (a, a_points, a_curve)) in curves.iter().enumerate() {
            for (b, b_points, b_curve) in &curves[i + 1..] {
                let ctx = with_context!("curves {a} and {b} of the 3-D sketch");
                let hits = curve_curve_intersect(
                    a_curve,
                    b_curve,
                    MAX_CROSSINGS,
                    MAX_NODES,
                    min_subdivision,
                )
                .with_context(ctx)?;
                if hits.is_coincident() {
                    return Err(GeopError::new(format!(
                        "curves {a} and {b} run along each other"
                    )));
                }
                for (ta, tb) in hits.into_vec() {
                    let at = (a_curve.evaluate(ta).with_context(ctx)?)
                        .union(&b_curve.evaluate(tb).with_context(ctx)?);
                    let passes = |p: &PointId| enclosures[p].could_be_equal(&at);
                    if a_points.intersection(b_points).any(passes) {
                        continue;
                    }
                    let of =
                        |points: &BTreeSet<PointId>| points.iter().copied().find(|p| passes(p));
                    return Err(match (of(a_points), of(b_points)) {
                        (Some(p), _) => GeopError::new(format!(
                            "curves {a} and {b} cross at point {p} of curve {a}, which curve {b} does \
                             not pass through: draw curve {b} through it"
                        )),
                        (None, Some(p)) => GeopError::new(format!(
                            "curves {a} and {b} cross at point {p} of curve {b}, which curve {a} does \
                             not pass through: draw curve {a} through it"
                        )),
                        (None, None) => GeopError::new(format!(
                            "curves {a} and {b} cross at {at:?}, where neither has a point: draw both \
                             through a point there"
                        )),
                    });
                }
            }
        }
        Ok(())
    }
}
