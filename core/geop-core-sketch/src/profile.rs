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

use crate::sketch::{CurveId, CurveKind, Enclosure, PointId, Sketch, SplineShape};
use geop_core_geometry::{
    aabb::union_aabb,
    intersection::curve_curve_intersect,
    nurb_curve::{NurbCurve, NurbCurve2D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::Scalar,
    solvers::{
        probe::{Probe, attempts, halve},
        repeat,
        walk::walk_by,
    },
    vector::Vector2,
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

impl Sketch {
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
        let (c, reversed) = start.unwrap_or((first, false));
        // On from each curve's end to the other curve there, until there is
        // none, or it is one already in the chain.
        let edges = walk_by(
            ProfileEdge { curve: c, reversed },
            |edge| edge.curve,
            |edge| {
                let (s, e) = ends(edge.curve);
                let end = if edge.reversed { s } else { e };
                Ok::<_, GeopError>(at[&end].iter().find(|&&k| k != edge.curve).map(|&next| {
                    ProfileEdge {
                        curve: next,
                        reversed: ends(next).1 == end,
                    }
                }))
            },
        )?
        .nodes;
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
            // On from each curve to the one other joined at its far end, as
            // far as the chain has not come already.
            let on = walk_by(
                (curve, from),
                |&(c, _)| c,
                |&(c, p)| {
                    Ok::<_, GeopError>(match others_at(p, c)[..] {
                        [(next, far)] => Some((next, far)),
                        _ => None,
                    })
                },
            )?;
            let new: Vec<CurveId> = (on.nodes[1..].iter())
                .map(|&(c, _)| c)
                .take_while(|c| !chain.contains(c))
                .collect();
            chain.extend(new);
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
        let curves: Vec<Vec<NurbCurve2D>> = loops
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
                Some((s, e)) if class[&s] != class[&e] => edges.push((id, class[&s], class[&e])),
                Some(_) if !matches!(curve.kind, CurveKind::Spline { .. }) => {
                    return Err(GeopError::new(format!(
                        "curve {id} starts and ends at the same point"
                    )));
                }
                // A closed curve is a loop by itself.
                _ => loops.push(ProfileLoop {
                    edges: vec![ProfileEdge {
                        curve: id,
                        reversed: false,
                    }],
                }),
            }
        }

        // Drop open chains: repeatedly remove edges at points of degree 1.
        let mut alive = vec![true; edges.len()];
        let mut degree: BTreeMap<PointId, usize> = BTreeMap::new();
        for &(_, a, b) in &edges {
            *degree.entry(a).or_default() += 1;
            *degree.entry(b).or_default() += 1;
        }
        let _ = repeat::resolve((0..edges.len()).collect(), |&k| {
            let (_, a, b) = edges[k];
            if degree[&a] != 1 && degree[&b] != 1 {
                return Err(());
            }
            alive[k] = false;
            *degree.get_mut(&a).unwrap() -= 1;
            *degree.get_mut(&b).unwrap() -= 1;
            Ok(())
        });
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
            // On to an edge not walked yet at each edge's far end.
            let around = walk_by(
                (start, false),
                |&(k, _)| k,
                |&(k, at_end)| {
                    used[k] = true;
                    let (_, a, b) = edges[k];
                    let next_point = if at_end { a } else { b };
                    Ok::<_, GeopError>(
                        (0..edges.len())
                            .find(|&j| {
                                alive[j]
                                    && !used[j]
                                    && (edges[j].1 == next_point || edges[j].2 == next_point)
                            })
                            .map(|next| (next, edges[next].2 == next_point)),
                    )
                },
            )?;
            let lp = (around.nodes.iter())
                .map(|&(k, at_end)| ProfileEdge {
                    curve: edges[k].0,
                    reversed: at_end,
                })
                .collect();
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

/// One NURBS piece `curve` of a chain of a sketch's curves — a profile
/// loop, or a 3-D sketch's path — and where it comes from in the sketch:
/// piece `index` of sketch curve `source` (counted in the curve's own
/// direction), running from joint `start` to joint `end` in the chain's
/// direction.
#[derive(Clone, Debug)]
pub struct Piece<C> {
    pub curve: C,
    pub source: CurveId,
    pub index: usize,
    pub start: ProfileJoint,
    pub end: ProfileJoint,
}

/// A piece of a planar sketch's profile loop.
pub type ProfilePiece = Piece<NurbCurve2D>;

impl<C> Piece<C> {
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
    pub fn to_nurbs(&self, sketch: &Sketch, geometry: &Enclosure) -> GeopResult<Vec<ProfilePiece>> {
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
            let (a, b) = only.curve.split(Scalar::ONE.div(Scalar::TWO)?)?;
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
    pub fn polyline(&self, sketch: &Sketch) -> GeopResult<Vec<Vector2>> {
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
fn curve_joints(sketch: &Sketch, curve: CurveId) -> GeopResult<(ProfileJoint, ProfileJoint)> {
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
pub fn curve_polyline(sketch: &Sketch, curve: CurveId) -> GeopResult<Vec<Vector2>> {
    let pieces = curve_nurbs(sketch, &Enclosure::as_drawn(sketch), curve)?;
    polyline(&pieces, NurbCurve2D::evaluate)
}

/// Points along `pieces`, a chain of curves on `[0, 1]` each starting where
/// the one before ends, as `evaluate` gives them: a straight piece by its
/// ends, any other by `SAMPLES` per span of its control points — dense
/// enough to draw it.
pub(crate) fn polyline<const D: usize, P>(
    pieces: &[NurbCurve<D>],
    evaluate: impl Fn(&NurbCurve<D>, Scalar) -> GeopResult<P>,
) -> GeopResult<Vec<P>> {
    let mut out = Vec::new();
    for piece in pieces {
        let spans = if piece.degree == 1 {
            1
        } else {
            SAMPLES * (piece.control_points.len() - 1)
        };
        // Each piece's last sample is the next piece's first.
        out.pop();
        for i in 0..=spans {
            out.push(evaluate(
                piece,
                Scalar::from_ratio(i as i64, spans as i64)?,
            )?);
        }
    }
    Ok(out)
}

/// Degree of a spline with `n` control points.
fn spline_degree(n: usize) -> usize {
    3.min(n - 1)
}

/// Clamped uniform knot vector on `[0, 1]`.
fn spline_knots(n: usize, degree: usize) -> GeopResult<Vec<Scalar>> {
    let spans = n - degree;
    let mut knots = vec![Scalar::ZERO; degree + 1];
    for i in 1..spans {
        knots.push(Scalar::from_ratio(i as i64, spans as i64)?);
    }
    knots.extend(std::iter::repeat_n(Scalar::ONE, degree + 1));
    Ok(knots)
}

/// One sketch curve as NURBS pieces from its start to its end, built from
/// `geometry` — how many pieces, from the sketch as drawn, which is design
/// data, so the pieces and their names do not depend on how precisely the
/// geometry is known.
pub fn curve_nurbs(
    sketch: &Sketch,
    geometry: &Enclosure,
    curve: CurveId,
) -> GeopResult<Vec<NurbCurve2D>> {
    let at = |p: &PointId| geometry.points[p];
    let param = || geometry.params[&curve];
    let one_half = Scalar::ONE.div(Scalar::TWO)?;
    match &sketch.curve(curve)?.kind {
        CurveKind::Line { start, end } => Ok(vec![NurbCurve2D::line(at(start), at(end))?]),
        CurveKind::Arc { start, end, sweep } => {
            let (s, e) = (at(start), at(end));
            if sweep.is_sharp() && sweep.could_be_equal(Scalar::ZERO) {
                return Ok(vec![NurbCurve2D::line(s, e)?]);
            }
            // As many pieces as quarter turns the sweep spans, one that could
            // be exactly `k` of them spanning `k`: counted on the sweep as
            // drawn, which is design data, so the pieces' names do not
            // depend on how precisely it is known.
            let quarter = Scalar::PI.div(Scalar::TWO)?;
            let pieces = (1..=4)
                .find(|&k| {
                    !sweep
                        .abs()
                        .definitely_greater(quarter.mul(Scalar::from_i64(k as i64)))
                })
                .unwrap_or(4);
            let delta = param().div(Scalar::from_i64(pieces as i64))?;
            let half = delta.div(Scalar::TWO)?;
            // Piece boundaries: the start turned about the center by
            // multiples of `delta`. Only needed for more than one piece,
            // where the arc turns by more than a quarter and so has a center
            // at a moderate distance.
            let mut ends = vec![s];
            if pieces > 1 {
                let whole = param().div(Scalar::TWO)?;
                let chord = e.sub(&s);
                let length = chord.norm();
                let left = chord.normalize()?.perp();
                let mid = s.add(&e).prod_scalar(one_half);
                let offset = length.mul(one_half).mul(whole.cos()).div(whole.sin())?;
                let center = mid.add(&left.prod_scalar(offset));
                ends.extend((1..pieces).map(|j| {
                    let angle = delta.mul(Scalar::from_i64(j as i64));
                    center.add(&s.sub(&center).rotate(angle.cos(), angle.sin()))
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
                    let left = chord.normalize()?.perp();
                    let bulge = length.mul(one_half).mul(half.sin()).div(half.cos())?;
                    let mid = w[0].add(&w[1]).prod_scalar(one_half);
                    let m = mid.sub(&left.prod_scalar(bulge));
                    NurbCurve2D::arc(w[0], m, w[1], half.cos())
                })
                .collect()
        }
        CurveKind::Circle { center, .. } => {
            let c = at(center);
            let r = param();
            let point = |x: Scalar, y: Scalar| c.add(&Vector2::from_array([x, y]));
            let q = [
                point(r, Scalar::ZERO),
                point(Scalar::ZERO, r),
                point(r.neg(), Scalar::ZERO),
                point(Scalar::ZERO, r.neg()),
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
                .map(|j| NurbCurve2D::arc(q[j], corners[j], q[(j + 1) % 4], w))
                .collect()
        }
        CurveKind::Spline {
            control_points,
            shape,
        } => {
            let points: Vec<Vector2> = control_points.iter().map(at).collect();
            Ok(vec![spline_nurbs(&points, shape.as_ref())?])
        }
    }
}

/// The point at `t` of a spline of `n` control points and the shape
/// `shape` — and its derivative there, if `derivative` — as a sum of its
/// control points: each one whose basis function's support reaches `t`,
/// by its index, with the rational basis function's value and derivative
/// at `t`. Constant numbers, for the weights and the knots are design data:
/// a point of a spline is linear in its control points. Read off the
/// spline of each control point alone at one, the rest at zero.
pub(crate) fn basis_at(
    n: usize,
    shape: Option<&SplineShape>,
    t: Scalar,
    derivative: bool,
) -> GeopResult<Vec<(usize, Scalar, Scalar)>> {
    let one_hot = |j: usize| -> Vec<Vector2> {
        (0..n)
            .map(|k| {
                let one = if k == j { Scalar::ONE } else { Scalar::ZERO };
                Vector2::from_array([one, Scalar::ZERO])
            })
            .collect()
    };
    let whole = spline_nurbs(&one_hot(0), shape)?;
    let (degree, knots) = (whole.degree, &whole.knot_vector);
    let mut out = Vec::new();
    for j in 0..n {
        if t.definitely_less(knots[j]) || t.definitely_greater(knots[j + degree + 1]) {
            continue;
        }
        let spline = spline_nurbs(&one_hot(j), shape)?;
        let value = spline.evaluate(t)?[0];
        let slope = if derivative {
            spline.tangent(t)?[0]
        } else {
            Scalar::ZERO
        };
        out.push((j, value, slope));
    }
    Ok(out)
}

/// The spline of the control points `points` and the shape `shape` (see
/// [`CurveKind::Spline`]), on `[0, 1]`.
pub(crate) fn spline_nurbs(
    points: &[Vector2],
    shape: Option<&SplineShape>,
) -> GeopResult<NurbCurve2D> {
    let n = points.len();
    let Some(shape) = shape else {
        let degree = spline_degree(n);
        return NurbCurve::try_new(
            degree,
            points.iter().map(|p| p.homogeneous(Scalar::ONE)).collect(),
            spline_knots(n, degree)?,
        );
    };
    // Taken from elsewhere, on knots of its own: brought onto `[0, 1]`,
    // like every other piece.
    let curve = NurbCurve::try_new(
        shape.degree,
        points
            .iter()
            .zip(&shape.weights)
            .map(|(p, &w)| p.homogeneous(w))
            .collect(),
        shape.knots.clone(),
    )?;
    curve.with_unit_domain()
}

// ── containment, on the curves themselves ───────────────────────────────────

/// How far `loops`' control points spread: the diagonal of their bounding
/// box — how long a ray has to be to leave every loop behind, and the
/// length the probe offsets below are measured against.
fn extent_of(loops: &[Vec<NurbCurve2D>]) -> GeopResult<Scalar> {
    let bounds = union_aabb(loops.iter().flatten().map(NurbCurve::aabb));
    let span = |k: usize| bounds.map_or(Scalar::ZERO, |b| b[k].upper().sub(b[k].lower()));
    let extent = span(0).mul(span(0)).add(span(1).mul(span(1))).sqrt()?;
    if !extent.definitely_greater(Scalar::ZERO) {
        return Err(GeopError::new("the profile's loops have no extent"));
    }
    Ok(extent)
}

/// The point halfway along `curve` — a probe, chosen freely, so sharp.
fn midpoint(curve: &NurbCurve2D) -> GeopResult<Vector2> {
    Ok(curve.evaluate(curve.mid_parameter()?)?.sharpen())
}

/// A segment from `from` in direction `dir`, long enough to leave a profile
/// of size `extent` behind.
fn ray(from: &Vector2, dir: &Vector2, extent: Scalar) -> GeopResult<NurbCurve2D> {
    let to = from
        .add(&dir.prod_scalar(Scalar::from_i64(3).mul(extent)))
        .sharpen();
    NurbCurve2D::line(*from, to)
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
fn loop_contains(curves: &[NurbCurve2D], probe: &Vector2, extent: Scalar) -> GeopResult<bool> {
    let min_subdivision = Scalar::from_ratio(1, MIN_SUBDIVISION)?;
    // The golden angle, `π (3 - √5)`.
    let golden = Scalar::PI.mul(Scalar::from_i64(3).sub(Scalar::from_i64(5).sqrt()?));
    let mut last_rejection = String::new();
    // Directions tried in turn until one is clear.
    let found = attempts(MAX_RAY_ATTEMPTS, |&k| -> GeopResult<_> {
        let angle = golden.mul(Scalar::from_i64(k as i64));
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
                        return Ok(Probe::Next);
                    }
                    Err(e) => {
                        last_rejection =
                            format!("the ray against {curve:?} did not converge: {e:?}");
                        return Ok(Probe::Next);
                    }
                };
            let (t0, t1) = curve.domain();
            for (along_ray, along_curve) in hits {
                if !along_ray.definitely_greater(Scalar::ZERO) {
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
                    return Ok(Probe::Next);
                }
                crossings += 1;
            }
        }
        Ok(Probe::Found(!crossings.is_multiple_of(2)))
    })?;
    if let Some(inside) = found {
        return Ok(inside);
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
fn turns_counter_clockwise(curves: &[NurbCurve2D], extent: Scalar) -> GeopResult<bool> {
    let curve = &curves[0];
    let mid = curve.mid_parameter()?;
    let point = curve.evaluate(mid)?;
    let tangent = curve.tangent(mid)?;
    let left = tangent.perp();
    let found = halve(
        extent.div(Scalar::from_i64(64))?,
        24,
        |&step| -> GeopResult<_> {
            let at = |sign: Scalar| point.add(&left.prod_scalar(step.mul(sign))).sharpen();
            let inside_left = loop_contains(curves, &at(Scalar::ONE), extent)?;
            let inside_right = loop_contains(curves, &at(Scalar::ONE.neg()), extent)?;
            Ok(if inside_left != inside_right {
                Probe::Found(inside_left)
            } else {
                Probe::Next
            })
        },
    )?;
    found.ok_or_else(|| {
        GeopError::new(format!(
            "could not tell which way {curve:?} winds: both sides of it classify the same way"
        ))
    })
}
