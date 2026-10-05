//! Turning a solved sketch into profiles: closed loops of curves grouped into
//! regions (an outer loop with its holes), and those loops into the NURBS
//! curves that extrude and revolve consume.
//!
//! **Connectivity is structural.** Two curves are joined exactly when they
//! share an endpoint (the same point, or points joined by
//! [`crate::Constraint::Coincident`]) — never because their ends happen to
//! be close. Curves hanging off a loop (open chains, e.g. a helper line) are
//! ignored; curves that branch (three or more meeting at a point) are an
//! error, since the regions they bound are ambiguous.
//!
//! **Nesting and orientation are answered on the real curves.** Which loop
//! lies inside which, and which way a loop winds, both come from casting a
//! ray and counting crossings with the kernel's own
//! [`curve_curve_intersect`] — the same thing
//! `geop_core_topology::contains::face::loops_contain` does for a face's
//! trim, on the same NURBS curves the profile is made of. Polylines are
//! sampled from those curves too, for drawing.

use crate::sketch::{CurveId, CurveKind, Enclosure, PointId, Sketch};
use geop_core_geometry::{
    intersection::curve_curve_intersect,
    nurb_curve::{NurbCurve, NurbCurve2D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    vector::{Vector2, Vector3},
    with_context,
};
use std::collections::BTreeMap;

/// Budgets for the ray casts in [`loop_contains`]: how many crossings one
/// ray may find with one curve, how hard a single search tries, and the
/// subdivision size it stops isolating at — one part in `MIN_SUBDIVISION`
/// of a curve's domain.
const MAX_CROSSINGS: usize = 16;
const MAX_NODES: usize = 4000;
const MIN_SUBDIVISION: i64 = 1_000_000;
/// How many directions a ray cast tries before giving up on a probe point.
const MAX_RAY_ATTEMPTS: usize = 32;

/// One curve of a loop, traversed forwards or backwards.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProfileEdge {
    pub curve: CurveId,
    pub reversed: bool,
}

/// A closed chain of curves, each ending where the next starts.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProfileLoop {
    pub edges: Vec<ProfileEdge>,
}

/// Curves joined end to end, in order (see [`Sketch::chain`]).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Chain {
    pub edges: Vec<ProfileEdge>,
    /// Its last curve ends where its first starts — or it is a circle.
    pub closed: bool,
}

/// An area of the sketch: an outer loop (counter-clockwise) and the holes in
/// it (clockwise).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Region {
    pub outer: ProfileLoop,
    pub holes: Vec<ProfileLoop>,
}

/// What a sketch draws to sweep: the one area its curves enclose, or — for
/// a sweep into a sheet only — the one open chain they form, enclosing
/// nothing.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Shape {
    Region(Region),
    Chain(ProfileLoop),
}

/// Samples per span of a curved piece, for drawing.
const SAMPLES: usize = 16;

impl<S: Scalar> Sketch<S> {
    /// The one area the sketch's curves enclose, with its holes.
    ///
    /// One sketch is one area to extrude or revolve: a sketch of several
    /// separate areas is an error — each would be a body of its own, so each
    /// belongs in a sketch of its own.
    pub fn region(&self) -> GeopResult<Region> {
        let mut regions = self.regions()?;
        match regions.len() {
            1 => Ok(regions.remove(0)),
            n => Err(GeopError::new(format!(
                "sketch has {n} separate areas, but one sketch is one area to sweep: draw each in a sketch of its own"
            ))),
        }
    }

    /// What the sketch draws to sweep (see [`Shape`]): its one area, or, if
    /// its curves enclose nothing, the one chain they form.
    pub fn shape(&self) -> GeopResult<Shape> {
        self.validate()?;
        if self.loops()?.is_empty() {
            Ok(Shape::Chain(self.sweep_chain()?))
        } else {
            Ok(Shape::Region(self.region()?))
        }
    }

    /// The one open chain the sketch's non-construction curves form (see
    /// [`Sketch::chain`]). Fails unless they form exactly one, unbranched.
    fn sweep_chain(&self) -> GeopResult<ProfileLoop> {
        let curves: Vec<CurveId> = self
            .curves
            .iter()
            .filter(|(_, c)| !c.construction && c.endpoints().is_some())
            .map(|(&id, _)| id)
            .collect();
        if curves.is_empty() {
            return Err(GeopError::new("sketch has no curves to sweep"));
        }
        let ctx = with_context!(
            "one sketch is one profile to sweep: draw each chain in a sketch of its own"
        );
        let chain = self.chain(&curves).with_context(ctx)?;
        Ok(ProfileLoop { edges: chain.edges })
    }

    /// `curves` in order as one chain, each joined to the next at an end
    /// point, none branching: an open chain walked from its end at the
    /// lower point id, a closed one from its first curve given — or a lone
    /// circle.
    pub fn chain(&self, curves: &[CurveId]) -> GeopResult<Chain> {
        let mut ids: Vec<CurveId> = Vec::new();
        for &c in curves {
            self.curve(c)?;
            if !ids.contains(&c) {
                ids.push(c);
            }
        }
        let Some(&first) = ids.first() else {
            return Err(GeopError::new("no curves to make a chain of"));
        };
        if let Some(&circle) = ids
            .iter()
            .find(|c| matches!(self.curves[c].kind, CurveKind::Circle { .. }))
        {
            if ids.len() > 1 {
                return Err(GeopError::new(format!(
                    "circle {circle} is a chain of its own, apart from the other curves"
                )));
            }
            return Ok(Chain {
                edges: vec![ProfileEdge {
                    curve: circle,
                    reversed: false,
                }],
                closed: true,
            });
        }
        let class = self.point_classes();
        let ends = |c: CurveId| {
            let (s, e) = self.curves[&c].endpoints().expect("no circle");
            (class[&s], class[&e])
        };
        let mut at: BTreeMap<PointId, Vec<CurveId>> = BTreeMap::new();
        for &c in &ids {
            let (s, e) = ends(c);
            at.entry(s).or_default().push(c);
            at.entry(e).or_default().push(c);
        }
        if let Some((p, cs)) = at.iter().find(|(_, cs)| cs.len() > 2) {
            return Err(GeopError::new(format!(
                "curves branch at point {p}: {} curves meet there",
                cs.len()
            )));
        }
        // An open chain starts at its end at the lower point id.
        let start = at.iter().find(|(_, cs)| cs.len() == 1).map(|(&p, cs)| {
            let c = cs[0];
            (c, ends(c).0 != p)
        });
        let closed = start.is_none();
        let (mut c, mut reversed) = start.unwrap_or((first, false));
        let mut edges = vec![ProfileEdge { curve: c, reversed }];
        loop {
            let (s, e) = ends(c);
            let end = if reversed { s } else { e };
            let Some(&next) = at[&end]
                .iter()
                .find(|&&k| !edges.iter().any(|edge| edge.curve == k))
            else {
                break;
            };
            reversed = ends(next).1 == end;
            c = next;
            edges.push(ProfileEdge { curve: c, reversed });
        }
        if let Some(apart) = ids
            .iter()
            .find(|&&k| !edges.iter().any(|edge| edge.curve == k))
        {
            return Err(GeopError::new(format!(
                "the curves form separate chains: {apart} is not joined to {}",
                edges[0].curve
            )));
        }
        Ok(Chain { edges, closed })
    }

    /// The chain through `curve`: it, and on from each of its ends the
    /// line or arc joined there — as long as exactly one other line or arc
    /// of its kind (profile or construction) is.
    pub fn chain_through(&self, curve: CurveId) -> GeopResult<Vec<CurveId>> {
        let construction = self.curve(curve)?.construction;
        let Some((s, e)) = self.curves[&curve].endpoints() else {
            return Ok(vec![curve]);
        };
        let class = self.point_classes();
        let kin: Vec<(CurveId, PointId, PointId)> = self
            .curves
            .iter()
            .filter(|(_, c)| c.construction == construction)
            .filter(|(_, c)| matches!(c.kind, CurveKind::Line { .. } | CurveKind::Arc { .. }))
            .filter_map(|(&id, c)| c.endpoints().map(|(s, e)| (id, class[&s], class[&e])))
            .collect();
        let others_at = |p: PointId, not: CurveId| -> Vec<(CurveId, PointId)> {
            kin.iter()
                .filter(|&&(id, s, e)| id != not && (s == p || e == p))
                .map(|&(id, s, e)| (id, if s == p { e } else { s }))
                .collect()
        };
        let mut chain = vec![curve];
        for from in [class[&e], class[&s]] {
            let (mut c, mut p) = (curve, from);
            while let [(next, far)] = others_at(p, c)[..] {
                if chain.contains(&next) {
                    break;
                }
                chain.push(next);
                (c, p) = (next, far);
            }
        }
        Ok(chain)
    }

    /// Every region bounded by the sketch's non-construction curves.
    pub fn regions(&self) -> GeopResult<Vec<Region>> {
        self.validate()?;
        let loops = self.loops()?;
        if loops.is_empty() {
            return Err(GeopError::new(
                "sketch has no closed profile: its curves do not form a loop",
            ));
        }
        // Nesting and winding are questions about the sketch as drawn.
        let drawn = Enclosure::as_drawn(self);
        let curves: Vec<Vec<NurbCurve2D<S>>> = loops
            .iter()
            .map(|l| {
                Ok(l.to_nurbs(self, &drawn)?
                    .into_iter()
                    .map(|p| p.curve)
                    .collect())
            })
            .collect::<GeopResult<_>>()?;
        let extent = extent_of(&curves)?;
        let counter_clockwise: Vec<bool> = curves
            .iter()
            .map(|c| turns_counter_clockwise(c, extent))
            .collect::<GeopResult<_>>()?;

        // containers[i]: the loops loop `i` lies inside. A point of loop `i`
        // serves as the probe — loops of one profile never cross, so a point
        // on one is either inside another or outside it.
        let mut containers: Vec<Vec<usize>> = Vec::with_capacity(loops.len());
        for i in 0..loops.len() {
            let probe = midpoint(&curves[i][0])?;
            let mut inside = Vec::new();
            for (j, other) in curves.iter().enumerate() {
                if j != i && loop_contains(other, &probe, extent)? {
                    inside.push(j);
                }
            }
            containers.push(inside);
        }

        let mut regions: Vec<(usize, Region)> = Vec::new();
        for i in (0..loops.len()).filter(|&i| containers[i].len().is_multiple_of(2)) {
            let outer = if counter_clockwise[i] {
                loops[i].clone()
            } else {
                loops[i].reversed()
            };
            regions.push((
                i,
                Region {
                    outer,
                    holes: Vec::new(),
                },
            ));
        }
        for i in (0..loops.len()).filter(|&i| !containers[i].len().is_multiple_of(2)) {
            let depth = containers[i].len();
            let parent = *containers[i]
                .iter()
                .find(|&&j| containers[j].len() == depth - 1)
                .expect("an odd-depth loop lies directly inside an even-depth one");
            let hole = if counter_clockwise[i] {
                loops[i].reversed()
            } else {
                loops[i].clone()
            };
            regions
                .iter_mut()
                .find(|(j, _)| *j == parent)
                .unwrap()
                .1
                .holes
                .push(hole);
        }
        Ok(regions.into_iter().map(|(_, r)| r).collect())
    }

    /// Every closed loop of non-construction curves.
    fn loops(&self) -> GeopResult<Vec<ProfileLoop>> {
        let class = self.point_classes();
        let mut loops = Vec::new();
        // Open curves between two distinct points: the edges of a graph on
        // the point classes.
        let mut edges: Vec<(CurveId, PointId, PointId)> = Vec::new();
        for (&id, curve) in &self.curves {
            if curve.construction {
                continue;
            }
            match curve.endpoints() {
                None => loops.push(ProfileLoop {
                    edges: vec![ProfileEdge {
                        curve: id,
                        reversed: false,
                    }],
                }),
                Some((s, e)) if class[&s] == class[&e] => {
                    if !matches!(curve.kind, CurveKind::Spline { .. }) {
                        return Err(GeopError::new(format!(
                            "curve {id} starts and ends at the same point"
                        )));
                    }
                    loops.push(ProfileLoop {
                        edges: vec![ProfileEdge {
                            curve: id,
                            reversed: false,
                        }],
                    });
                }
                Some((s, e)) => edges.push((id, class[&s], class[&e])),
            }
        }

        // Drop open chains: repeatedly remove edges at points of degree 1.
        let mut alive = vec![true; edges.len()];
        let mut degree: BTreeMap<PointId, usize> = BTreeMap::new();
        for &(_, a, b) in &edges {
            *degree.entry(a).or_default() += 1;
            *degree.entry(b).or_default() += 1;
        }
        loop {
            let mut changed = false;
            for (k, &(_, a, b)) in edges.iter().enumerate() {
                if alive[k] && (degree[&a] == 1 || degree[&b] == 1) {
                    alive[k] = false;
                    *degree.get_mut(&a).unwrap() -= 1;
                    *degree.get_mut(&b).unwrap() -= 1;
                    changed = true;
                }
            }
            if !changed {
                break;
            }
        }
        if let Some((p, d)) = degree.iter().find(|(_, d)| **d > 2) {
            return Err(GeopError::new(format!(
                "profile curves branch at point {p}: {d} curves meet there"
            )));
        }

        // Every remaining point has degree 2: walk the cycles.
        let mut used = vec![false; edges.len()];
        for start in 0..edges.len() {
            if !alive[start] || used[start] {
                continue;
            }
            let mut lp = Vec::new();
            let (mut k, mut at_end) = (start, false);
            loop {
                used[k] = true;
                let (id, a, b) = edges[k];
                lp.push(ProfileEdge {
                    curve: id,
                    reversed: at_end,
                });
                let next_point = if at_end { a } else { b };
                let Some(next) = (0..edges.len()).find(|&j| {
                    alive[j] && !used[j] && (edges[j].1 == next_point || edges[j].2 == next_point)
                }) else {
                    break;
                };
                at_end = edges[next].2 == next_point;
                k = next;
            }
            loops.push(ProfileLoop { edges: lp });
        }
        Ok(loops)
    }
}

/// Where a joint of a profile comes from in the sketch: a sketch point, or a
/// point a sketch curve had to be split at to become NURBS pieces (see
/// [`ProfileLoop::to_nurbs`]) — `Split { curve, index }` is where piece
/// `index` of `curve` starts, counted in the curve's own direction. A circle
/// has no points of its own, so all of its joints are splits, `index = 0`
/// included.
///
/// This is design data, stable across edits of the sketch that keep the
/// curve — what a topological name of anything built from the joint is made
/// from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum ProfileJoint {
    Point(PointId),
    Split { curve: CurveId, index: usize },
}

/// `p3` for a sketch point, `c5@1` for where piece 1 of curve 5 starts.
impl std::fmt::Display for ProfileJoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProfileJoint::Point(p) => write!(f, "{p}"),
            ProfileJoint::Split { curve, index } => write!(f, "{curve}@{index}"),
        }
    }
}

/// One NURBS piece of a profile loop, and where it comes from in the sketch:
/// piece `index` of sketch curve `source` (counted in the curve's own
/// direction), running from joint `start` to joint `end` in the loop's
/// direction.
#[derive(Clone, Debug)]
pub struct ProfilePiece<S: Scalar> {
    pub curve: NurbCurve2D<S>,
    pub source: CurveId,
    pub index: usize,
    pub start: ProfileJoint,
    pub end: ProfileJoint,
}

impl<S: Scalar> ProfilePiece<S> {
    /// The piece's stable name within its sketch: `c5` for a curve's first
    /// (often only) piece, `c5#1` for the next.
    pub fn name(&self) -> String {
        if self.index == 0 {
            format!("{}", self.source)
        } else {
            format!("{}#{}", self.source, self.index)
        }
    }
}

impl ProfileLoop {
    /// The same loop traversed the other way.
    pub fn reversed(&self) -> ProfileLoop {
        ProfileLoop {
            edges: self
                .edges
                .iter()
                .rev()
                .map(|e| ProfileEdge {
                    curve: e.curve,
                    reversed: !e.reversed,
                })
                .collect(),
        }
    }

    /// The loop as NURBS pieces, each on the domain `[0, 1]`, each starting
    /// exactly where the previous one ends, with the sketch's points placed
    /// at `positions` (the sketch's own positions, or a rigid,
    /// orientation-preserving motion of them).
    ///
    /// Arcs are split into pieces of at most a quarter turn — a rational
    /// quadratic's middle weight `cos(sweep / 2)` must stay positive — and
    /// circles into four quarters. A loop of a single piece (a closed spline)
    /// is split in two, so every loop has at least two joints; an open chain
    /// of one piece stays one. Each piece
    /// records which sketch curve it is part of and which joints it runs
    /// between (see [`ProfilePiece`]).
    ///
    /// Also used for an open chain (a revolve profile): the pieces then
    /// simply don't close up, and the last one's `end` is the chain's end.
    pub fn to_nurbs<D: Scalar, S: Scalar>(
        &self,
        sketch: &Sketch<D>,
        geometry: &Enclosure<S>,
    ) -> GeopResult<Vec<ProfilePiece<S>>> {
        let mut out = Vec::new();
        for edge in &self.edges {
            let curve = edge.curve;
            let ctx = with_context!("converting sketch curve {curve} to NURBS");
            let pieces = curve_nurbs(sketch, geometry, curve).with_context(ctx)?;
            let (first, last) = curve_joints(sketch, curve).with_context(ctx)?;
            let n = pieces.len();
            // Joint `j`, in the curve's own direction: where piece `j` starts
            // (`j = n` is the curve's end).
            let joint = |j: usize| match j {
                0 => first,
                j if j == n => last,
                index => ProfileJoint::Split { curve, index },
            };
            let pieces = pieces
                .into_iter()
                .enumerate()
                .map(|(index, c)| ProfilePiece {
                    curve: c,
                    source: curve,
                    index,
                    start: joint(index),
                    end: joint(index + 1),
                });
            if edge.reversed {
                out.extend(pieces.rev().map(|p| ProfilePiece {
                    curve: p.curve.reverse(),
                    start: p.end,
                    end: p.start,
                    ..p
                }));
            } else {
                out.extend(pieces);
            }
        }
        if let [only] = &out[..]
            && only.start == only.end
        {
            // A closed curve of a single piece: split it at its middle, which
            // becomes the curve's joint 1 whichever way the loop runs.
            let (a, b) = only.curve.split(S::ONE.div(S::TWO)?)?;
            let middle = ProfileJoint::Split {
                curve: only.source,
                index: 1,
            };
            // Indices count in the curve's own direction, so a reversed loop
            // meets the curve's second half first.
            let (first, second) = if self.edges[0].reversed {
                (1, 0)
            } else {
                (0, 1)
            };
            out = vec![
                ProfilePiece {
                    curve: a.with_unit_domain()?,
                    source: only.source,
                    index: first,
                    start: only.start,
                    end: middle,
                },
                ProfilePiece {
                    curve: b.with_unit_domain()?,
                    source: only.source,
                    index: second,
                    start: middle,
                    end: only.end,
                },
            ];
        }
        Ok(out)
    }

    /// A dense polyline along the loop, as drawn — for drawing.
    pub fn polyline<S: Scalar>(&self, sketch: &Sketch<S>) -> GeopResult<Vec<Vector2<S>>> {
        let mut out = Vec::new();
        for edge in &self.edges {
            let mut pts = curve_polyline(sketch, edge.curve)?;
            if edge.reversed {
                pts.reverse();
            }
            // Each edge's last sample is the next edge's first.
            pts.pop();
            out.extend(pts);
        }
        Ok(out)
    }
}

/// The joints at a sketch curve's start and end: its end points, or for a
/// circle, which has none, its seam (joint 0 of its own split points).
fn curve_joints<S: Scalar>(
    sketch: &Sketch<S>,
    curve: CurveId,
) -> GeopResult<(ProfileJoint, ProfileJoint)> {
    Ok(match sketch.curve(curve)?.endpoints() {
        Some((s, e)) => (ProfileJoint::Point(s), ProfileJoint::Point(e)),
        None => {
            let seam = ProfileJoint::Split { curve, index: 0 };
            (seam, seam)
        }
    })
}

/// Points along a curve as drawn, from its start to its end (a circle
/// starts and ends at angle 0) — sampled from its NURBS pieces, dense enough
/// for drawing.
pub fn curve_polyline<S: Scalar>(
    sketch: &Sketch<S>,
    curve: CurveId,
) -> GeopResult<Vec<Vector2<S>>> {
    let pieces = curve_nurbs(sketch, &Enclosure::as_drawn(sketch), curve)?;
    let mut out = Vec::new();
    for piece in &pieces {
        let spans = if piece.degree == 1 {
            1
        } else {
            SAMPLES * (piece.control_points.len() - 1)
        };
        // Each piece's last sample is the next piece's first.
        out.pop();
        for i in 0..=spans {
            out.push(piece.evaluate(S::from_ratio(i as i64, spans as i64)?)?);
        }
    }
    Ok(out)
}

/// Degree of a spline with `n` control points.
fn spline_degree(n: usize) -> usize {
    3.min(n - 1)
}

/// Clamped uniform knot vector on `[0, 1]`.
fn spline_knots<S: Scalar>(n: usize, degree: usize) -> GeopResult<Vec<S>> {
    let spans = n - degree;
    let mut knots = vec![S::ZERO; degree + 1];
    for i in 1..spans {
        knots.push(S::from_ratio(i as i64, spans as i64)?);
    }
    knots.extend(std::iter::repeat_n(S::ONE, degree + 1));
    Ok(knots)
}

/// Homogeneous control point `(w x, w y, w)`.
fn hom<S: Scalar>(p: Vector2<S>, w: S) -> Vector3<S> {
    Vector3::from_array([p[0].mul(w), p[1].mul(w), w])
}

/// The knots `[0; n] ++ [1; n]` of a single Bézier span on `[0, 1]`.
fn bezier_knots<S: Scalar>(n: usize) -> Vec<S> {
    let mut knots = vec![S::ZERO; n];
    knots.extend(std::iter::repeat_n(S::ONE, n));
    knots
}

/// A rational quadratic from `p0` to `p2` through the tangent intersection
/// `m`, with middle weight `w`.
fn conic<S: Scalar>(
    p0: Vector2<S>,
    m: Vector2<S>,
    p2: Vector2<S>,
    w: S,
) -> GeopResult<NurbCurve2D<S>> {
    NurbCurve::try_new(
        2,
        vec![hom(p0, S::ONE), hom(m, w), hom(p2, S::ONE)],
        bezier_knots(3),
    )
}

fn line<S: Scalar>(p0: Vector2<S>, p1: Vector2<S>) -> GeopResult<NurbCurve2D<S>> {
    NurbCurve::try_new(1, vec![hom(p0, S::ONE), hom(p1, S::ONE)], bezier_knots(2))
}

/// `v` turned a quarter counter-clockwise.
fn perpendicular<S: Scalar>(v: Vector2<S>) -> Vector2<S> {
    Vector2::from_array([v[1].neg(), v[0]])
}

/// `v` turned counter-clockwise by the angle whose cosine and sine are
/// `cos` and `sin`.
fn rotated<S: Scalar>(v: Vector2<S>, cos: S, sin: S) -> Vector2<S> {
    Vector2::from_array([
        v[0].mul(cos).sub(v[1].mul(sin)),
        v[0].mul(sin).add(v[1].mul(cos)),
    ])
}

/// One sketch curve as NURBS pieces from its start to its end, built from
/// `geometry` — how many pieces, from the sketch as drawn, which is design
/// data, so the pieces and their names do not depend on how precisely the
/// geometry is known.
pub fn curve_nurbs<D: Scalar, S: Scalar>(
    sketch: &Sketch<D>,
    geometry: &Enclosure<S>,
    curve: CurveId,
) -> GeopResult<Vec<NurbCurve2D<S>>> {
    let at = |p: &PointId| geometry.points[p];
    let param = || geometry.params[&curve];
    let one_half = S::ONE.div(S::TWO)?;
    match &sketch.curve(curve)?.kind {
        CurveKind::Line { start, end } => Ok(vec![line(at(start), at(end))?]),
        CurveKind::Arc { start, end, sweep } => {
            let (s, e) = (at(start), at(end));
            if sweep.is_sharp() && sweep.could_be_equal(D::ZERO) {
                return Ok(vec![line(s, e)?]);
            }
            // As many pieces as quarter turns the sweep spans, one that could
            // be exactly `k` of them spanning `k`: counted on the sweep as
            // drawn, which is design data, so the pieces' names do not
            // depend on how precisely it is known.
            let quarter = D::PI.div(D::TWO)?;
            let pieces = (1..=4)
                .find(|&k| {
                    !sweep
                        .abs()
                        .definitely_greater(quarter.mul(D::from_i64(k as i64)))
                })
                .unwrap_or(4);
            let delta = param().div(S::from_i64(pieces as i64))?;
            let half = delta.div(S::TWO)?;
            // Piece boundaries: the start turned about the center by
            // multiples of `delta`. Only needed for more than one piece,
            // where the arc turns by more than a quarter and so has a center
            // at a moderate distance.
            let mut ends = vec![s];
            if pieces > 1 {
                let whole = param().div(S::TWO)?;
                let chord = e.sub(&s);
                let length = chord.norm();
                let left = perpendicular(chord.normalize()?);
                let mid = s.add(&e).prod_scalar(one_half);
                let offset = length.mul(one_half).mul(whole.cos()).div(whole.sin())?;
                let center = mid.add(&left.prod_scalar(offset));
                ends.extend((1..pieces).map(|j| {
                    let angle = delta.mul(S::from_i64(j as i64));
                    center.add(&rotated(s.sub(&center), angle.cos(), angle.sin()))
                }));
            }
            ends.push(e);
            ends.windows(2)
                .map(|w| {
                    // Tangent intersection from the chord alone: `(L/2)
                    // tan(δ/2)` to the chord's right (its left for a
                    // clockwise arc). Exact for any radius, including a
                    // nearly straight arc whose center is far away.
                    let chord = w[1].sub(&w[0]);
                    let length = chord.norm();
                    let left = perpendicular(chord.normalize()?);
                    let bulge = length.mul(one_half).mul(half.sin()).div(half.cos())?;
                    let mid = w[0].add(&w[1]).prod_scalar(one_half);
                    let m = mid.sub(&left.prod_scalar(bulge));
                    conic(w[0], m, w[1], half.cos())
                })
                .collect()
        }
        CurveKind::Circle { center, .. } => {
            let c = at(center);
            let r = param();
            let point = |x: S, y: S| c.add(&Vector2::from_array([x, y]));
            let q = [
                point(r, S::ZERO),
                point(S::ZERO, r),
                point(r.neg(), S::ZERO),
                point(S::ZERO, r.neg()),
            ];
            let corners = [
                point(r, r),
                point(r.neg(), r),
                point(r.neg(), r.neg()),
                point(r, r.neg()),
            ];
            // cos(45°), honestly: √2 / 2 is irrational.
            let w = one_half.sqrt()?;
            (0..4)
                .map(|j| conic(q[j], corners[j], q[(j + 1) % 4], w))
                .collect()
        }
        CurveKind::Spline {
            control_points,
            shape,
        } => {
            let n = control_points.len();
            let Some(shape) = shape else {
                let degree = spline_degree(n);
                return Ok(vec![NurbCurve::try_new(
                    degree,
                    control_points.iter().map(|p| hom(at(p), S::ONE)).collect(),
                    spline_knots(n, degree)?,
                )?]);
            };
            // Taken from elsewhere, on knots of its own: brought onto
            // `[0, 1]`, like every other piece.
            let curve = NurbCurve::try_new(
                shape.degree,
                control_points
                    .iter()
                    .zip(&shape.weights)
                    .map(|(p, w)| hom(at(p), w.cast()))
                    .collect(),
                shape.knots.iter().map(|k| k.cast()).collect(),
            )?;
            Ok(vec![curve.with_unit_domain()?])
        }
    }
}

// ── containment, on the curves themselves ───────────────────────────────────

/// How far `loops`' control points spread: the diagonal of their bounding
/// box — how long a ray has to be to leave every loop behind, and the
/// length the probe offsets below are measured against.
fn extent_of<S: Scalar>(loops: &[Vec<NurbCurve2D<S>>]) -> GeopResult<S> {
    let points: Vec<Vector2<S>> = loops
        .iter()
        .flatten()
        .flat_map(|c| c.control_points.iter())
        .map(|cp| {
            Ok(Vector2::from_array([
                cp[0].div(cp[2])?.sharpen(),
                cp[1].div(cp[2])?.sharpen(),
            ]))
        })
        .collect::<GeopResult<_>>()?;
    let span = |k: usize| {
        let hi = points.iter().map(|p| p[k]).reduce(S::max);
        let lo = points.iter().map(|p| p[k]).reduce(S::min);
        match (hi, lo) {
            (Some(hi), Some(lo)) => hi.sub(lo),
            _ => S::ZERO,
        }
    };
    let extent = span(0).mul(span(0)).add(span(1).mul(span(1))).sqrt()?;
    if !extent.definitely_greater(S::ZERO) {
        return Err(GeopError::new("the profile's loops have no extent"));
    }
    Ok(extent)
}

/// The point halfway along `curve` — a probe, chosen freely, so sharp.
fn midpoint<S: Scalar>(curve: &NurbCurve2D<S>) -> GeopResult<Vector2<S>> {
    let (t0, t1) = curve.domain();
    Ok(curve.evaluate(t0.add(t1).div(S::TWO)?.sharpen())?.sharpen())
}

/// A segment from `from` in direction `dir`, long enough to leave a profile
/// of size `extent` behind.
fn ray<S: Scalar>(from: &Vector2<S>, dir: &Vector2<S>, extent: S) -> GeopResult<NurbCurve2D<S>> {
    let to = from
        .add(&dir.prod_scalar(S::from_i64(3).mul(extent)))
        .sharpen();
    line(*from, to)
}

/// Is `probe` inside the closed loop `curves`?
///
/// Ray casting, counting crossings with [`curve_curve_intersect`] — the
/// kernel's own search on the loop's own curves, so a hole's boundary is
/// followed exactly rather than through a polygon standing in for it. A ray
/// that grazes a curve's endpoint (a crossing shared by two curves, so
/// ambiguous to count), runs along a curve, or exhausts a search's budget
/// says nothing reliable, and the next direction is tried instead; the
/// directions walk the golden angle, so a handful of them are spread evenly
/// around the circle without ever repeating. A direction is a free choice,
/// so it is sharp.
fn loop_contains<S: Scalar>(
    curves: &[NurbCurve2D<S>],
    probe: &Vector2<S>,
    extent: S,
) -> GeopResult<bool> {
    let min_subdivision = S::from_ratio(1, MIN_SUBDIVISION)?;
    // The golden angle, `π (3 - √5)`.
    let golden = S::PI.mul(S::from_i64(3).sub(S::from_i64(5).sqrt()?));
    let mut last_rejection = String::new();
    'attempt: for k in 0..MAX_RAY_ATTEMPTS {
        let angle = golden.mul(S::from_i64(k as i64));
        let dir = Vector2::from_array([angle.cos(), angle.sin()]).sharpen();
        let ray = ray(probe, &dir, extent)?;
        let mut crossings = 0usize;
        for curve in curves {
            let hits =
                match curve_curve_intersect(&ray, curve, MAX_CROSSINGS, MAX_NODES, min_subdivision)
                {
                    Ok(hits) if !hits.is_coincident() => hits.into_vec(),
                    Ok(_) => {
                        last_rejection = format!("the ray runs along {curve:?}");
                        continue 'attempt;
                    }
                    Err(e) => {
                        last_rejection =
                            format!("the ray against {curve:?} did not converge: {e:?}");
                        continue 'attempt;
                    }
                };
            let (t0, t1) = curve.domain();
            for (along_ray, along_curve) in hits {
                if !along_ray.definitely_greater(S::ZERO) {
                    // At the probe itself: it lies on this loop, which the
                    // caller already knows (it picked a point of another
                    // one), so it is not a crossing of anything.
                    continue;
                }
                let from_start = along_curve.sub(t0).abs();
                let from_end = along_curve.sub(t1).abs();
                if !from_start.definitely_greater(min_subdivision)
                    || !from_end.definitely_greater(min_subdivision)
                {
                    last_rejection =
                        format!("the ray grazes an end of {curve:?} at {along_curve:?}");
                    continue 'attempt;
                }
                crossings += 1;
            }
        }
        return Ok(!crossings.is_multiple_of(2));
    }
    Err(GeopError::new(format!(
        "could not classify {probe:?} against a loop: every ray direction was ambiguous, \
         the last because {last_rejection}"
    )))
}

/// Does `curves` wind counter-clockwise (material on its left)?
///
/// Asked the same way as everything else here: step off the loop's first
/// curve to either side of its midpoint and see which side is inside. The
/// step starts at a fraction of the profile's extent and halves until the
/// two sides genuinely disagree — that disagreement is the property the
/// answer rests on, so it is verified rather than assumed, and a loop with a
/// feature narrower than the first step simply takes another halving.
fn turns_counter_clockwise<S: Scalar>(curves: &[NurbCurve2D<S>], extent: S) -> GeopResult<bool> {
    let curve = &curves[0];
    let (t0, t1) = curve.domain();
    let mid = t0.add(t1).div(S::TWO)?.sharpen();
    let point = curve.evaluate(mid)?;
    let tangent = curve.tangent(mid)?;
    let left = Vector2::from_array([tangent[1].neg(), tangent[0]]);
    let mut step = extent.div(S::from_i64(64))?;
    for _ in 0..24 {
        let at = |sign: S| point.add(&left.prod_scalar(step.mul(sign))).sharpen();
        let inside_left = loop_contains(curves, &at(S::ONE), extent)?;
        let inside_right = loop_contains(curves, &at(S::ONE.neg()), extent)?;
        if inside_left != inside_right {
            return Ok(inside_left);
        }
        step = step.div(S::TWO)?;
    }
    Err(GeopError::new(format!(
        "could not tell which way {curve:?} winds: both sides of it classify the same way"
    )))
}
