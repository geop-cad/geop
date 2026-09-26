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
//! trim, on the same NURBS curves the profile is made of. Polylines here are
//! for drawing only.

use crate::sketch::{CurveId, CurveKind, PointId, Positions, Sketch};
use geop_core_geometry::{
    intersection::curve_curve_intersect,
    nurb_curve::{NurbCurve, NurbCurve2D},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    scalars::{Field, Ring, Scalar, scal_in_f64::ScalInF64},
    vector::Vector3,
    with_context,
};
use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, SQRT_2};

/// The scalar the containment tests below run in. A sketch is `f64` design
/// data, and these questions are about the sketch's own geometry, so nothing
/// wider is called for — but they still go through the kernel's interval
/// scalar, because that is what its searches are written against and what
/// makes "could this be a graze?" answerable at all.
type F = ScalInF64;

/// Budgets for the ray casts in [`loop_contains`]: how many crossings one
/// ray may find with one curve, how hard a single search tries, and the
/// subdivision size it stops isolating at.
const MAX_CROSSINGS: usize = 16;
const MAX_NODES: usize = 4000;
const MIN_SUBDIVISION: f64 = 1e-6;
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

/// An area of the sketch: an outer loop (counter-clockwise) and the holes in
/// it (clockwise).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Region {
    pub outer: ProfileLoop,
    pub holes: Vec<ProfileLoop>,
}

/// Samples per quarter turn of an arc, and per spline, for nesting tests.
const SAMPLES: usize = 16;

impl Sketch {
    /// Every region bounded by the sketch's non-construction curves.
    pub fn regions(&self) -> GeopResult<Vec<Region>> {
        self.validate()?;
        let loops = self.loops()?;
        if loops.is_empty() {
            return Err(GeopError::new(
                "sketch has no closed profile: its curves do not form a loop",
            ));
        }
        let positions = self.positions();
        let curves: Vec<Vec<NurbCurve2D<F>>> = loops
            .iter()
            .map(|l| {
                Ok(l.to_nurbs::<F>(self, &positions)?
                    .into_iter()
                    .map(|p| p.curve)
                    .collect())
            })
            .collect::<GeopResult<_>>()?;
        let extent = extent_of(&curves);
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
                if j != i && loop_contains(other, probe, extent)? {
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
    /// is split in two, so every loop has at least two joints. Each piece
    /// records which sketch curve it is part of and which joints it runs
    /// between (see [`ProfilePiece`]).
    ///
    /// Also used for an open chain (a revolve profile): the pieces then
    /// simply don't close up, and the last one's `end` is the chain's end.
    pub fn to_nurbs<S: Scalar>(
        &self,
        sketch: &Sketch,
        positions: &Positions,
    ) -> GeopResult<Vec<ProfilePiece<S>>> {
        let mut out = Vec::new();
        for edge in &self.edges {
            let curve = edge.curve;
            let ctx = with_context!("converting sketch curve {curve} to NURBS");
            let pieces = edge_pieces(sketch, positions, curve).with_context(ctx)?;
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
        if let [only] = &out[..] {
            // A closed curve of a single piece: split it at its middle, which
            // becomes the curve's joint 1 whichever way the loop runs.
            let (a, b) = only.curve.split(S::from_f64(0.5))?;
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
                    curve: rescale_to_unit(a)?,
                    source: only.source,
                    index: first,
                    start: only.start,
                    end: middle,
                },
                ProfilePiece {
                    curve: rescale_to_unit(b)?,
                    source: only.source,
                    index: second,
                    start: middle,
                    end: only.end,
                },
            ];
        }
        Ok(out)
    }

    /// A dense polyline along the loop (for nesting tests and display).
    pub fn polyline(&self, sketch: &Sketch, positions: &Positions) -> Vec<[f64; 2]> {
        let mut out = Vec::new();
        for edge in &self.edges {
            let mut pts = curve_polyline(sketch, positions, edge.curve);
            if edge.reversed {
                pts.reverse();
            }
            // Each edge's last sample is the next edge's first.
            pts.pop();
            out.extend(pts);
        }
        out
    }
}

/// Center, radius and chord geometry of a circular arc, computed directly in
/// `f64`. [`crate::geometry::Arc`] carries the same formulas generically
/// over [`Scalar`] and stays fallible so it can cover a genuinely degenerate
/// arc (a divide-by-zero at `sweep = 0`) — every caller here already knows
/// `sweep != 0` (checked before an [`Arc`] is even built), so that fallible
/// machinery would only get in the way of tessellating an already-solved
/// sketch for display or NURBS-piece construction.
struct PlainArc {
    s: [f64; 2],
    e: [f64; 2],
    half: f64,
}

impl PlainArc {
    fn chord(&self) -> [f64; 2] {
        [self.e[0] - self.s[0], self.e[1] - self.s[1]]
    }
    fn chord_length(&self) -> f64 {
        let c = self.chord();
        c[0].hypot(c[1])
    }
    fn chord_mid(&self) -> [f64; 2] {
        [(self.s[0] + self.e[0]) * 0.5, (self.s[1] + self.e[1]) * 0.5]
    }
    /// Unit normal to the chord, pointing to its left.
    fn left(&self) -> [f64; 2] {
        let c = self.chord();
        let n = self.chord_length();
        [-c[1] / n, c[0] / n]
    }
    fn center(&self) -> [f64; 2] {
        let d = self.chord_length() * 0.5 * self.half.cos() / self.half.sin();
        let (m, l) = (self.chord_mid(), self.left());
        [m[0] + l[0] * d, m[1] + l[1] * d]
    }
    /// `|radius|`.
    fn radius(&self) -> f64 {
        self.chord_length() / (2.0 * self.half.sin().abs())
    }
}

fn pos(positions: &Positions, p: PointId) -> [f64; 2] {
    positions[&p]
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

fn arc_of(positions: &Positions, start: PointId, end: PointId, sweep: f64) -> PlainArc {
    PlainArc {
        s: pos(positions, start),
        e: pos(positions, end),
        half: sweep / 2.0,
    }
}

/// Points along a curve from its start to its end (a circle starts and ends
/// at angle 0), dense enough for display and nesting tests.
pub fn curve_polyline(sketch: &Sketch, positions: &Positions, curve: CurveId) -> Vec<[f64; 2]> {
    match &sketch.curves[&curve].kind {
        CurveKind::Line { start, end } => vec![positions[start], positions[end]],
        CurveKind::Arc { start, end, sweep } => {
            let arc = arc_of(positions, *start, *end, *sweep);
            if *sweep == 0.0 {
                return vec![positions[start], positions[end]];
            }
            let c = arc.center();
            let r = arc.radius();
            let a0 = (arc.s[1] - c[1]).atan2(arc.s[0] - c[0]);
            let n = SAMPLES * (1 + (sweep.abs() / FRAC_PI_2) as usize);
            let mut pts: Vec<[f64; 2]> = (0..=n)
                .map(|i| {
                    let a = a0 + sweep * i as f64 / n as f64;
                    [c[0] + r * a.cos(), c[1] + r * a.sin()]
                })
                .collect();
            pts[0] = positions[start];
            pts[n] = positions[end];
            pts
        }
        CurveKind::Circle { center, radius } => {
            let c = positions[center];
            let n = 4 * SAMPLES;
            (0..=n)
                .map(|i| {
                    let a = std::f64::consts::TAU * i as f64 / n as f64;
                    [c[0] + radius * a.cos(), c[1] + radius * a.sin()]
                })
                .collect()
        }
        CurveKind::Spline { control_points } => {
            let cps: Vec<[f64; 2]> = control_points.iter().map(|p| positions[p]).collect();
            let n = 2 * SAMPLES * cps.len();
            (0..=n)
                .map(|i| bspline_point(&cps, i as f64 / n as f64))
                .collect()
        }
    }
}

/// Degree of a spline with `n` control points.
fn spline_degree(n: usize) -> usize {
    3.min(n - 1)
}

/// Clamped uniform knot vector on `[0, 1]`.
fn spline_knots(n: usize, degree: usize) -> Vec<f64> {
    let spans = n - degree;
    let mut knots = vec![0.0; degree + 1];
    knots.extend((1..spans).map(|i| i as f64 / spans as f64));
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    knots
}

/// De Boor evaluation of the sketch's spline convention in `f64`.
fn bspline_point(cps: &[[f64; 2]], t: f64) -> [f64; 2] {
    let p = spline_degree(cps.len());
    let knots = spline_knots(cps.len(), p);
    // Span `k` with knots[k] <= t < knots[k + 1] (the last span at t = 1).
    let k = (p..cps.len()).rev().find(|&k| knots[k] <= t).unwrap_or(p);
    let mut d: Vec<[f64; 2]> = (0..=p).map(|j| cps[j + k - p]).collect();
    for r in 1..=p {
        for j in (r..=p).rev() {
            let i = j + k - p;
            let denom = knots[i + p + 1 - r] - knots[i];
            let alpha = if denom == 0.0 {
                0.0
            } else {
                (t - knots[i]) / denom
            };
            d[j] = [
                (1.0 - alpha) * d[j - 1][0] + alpha * d[j][0],
                (1.0 - alpha) * d[j - 1][1] + alpha * d[j][1],
            ];
        }
    }
    d[p]
}

/// Homogeneous control point `(w x, w y, w)`.
fn hom<S: Scalar>(p: [f64; 2], w: f64) -> Vector3<S> {
    Vector3::from_array([S::from_f64(p[0] * w), S::from_f64(p[1] * w), S::from_f64(w)])
}

fn unit_knots<S: Scalar>(knots: &[f64]) -> Vec<S> {
    knots.iter().map(|&k| S::from_f64(k)).collect()
}

/// A rational quadratic from `p0` to `p2` through the tangent intersection
/// `m`, with middle weight `w`.
fn conic<S: Scalar>(p0: [f64; 2], m: [f64; 2], p2: [f64; 2], w: f64) -> GeopResult<NurbCurve2D<S>> {
    NurbCurve::try_new(
        2,
        vec![hom(p0, 1.0), hom(m, w), hom(p2, 1.0)],
        unit_knots(&[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]),
    )
}

fn line<S: Scalar>(p0: [f64; 2], p1: [f64; 2]) -> GeopResult<NurbCurve2D<S>> {
    NurbCurve::try_new(
        1,
        vec![hom(p0, 1.0), hom(p1, 1.0)],
        unit_knots(&[0.0, 0.0, 1.0, 1.0]),
    )
}

/// One sketch curve as NURBS pieces from its start to its end.
fn edge_pieces<S: Scalar>(
    sketch: &Sketch,
    positions: &Positions,
    curve: CurveId,
) -> GeopResult<Vec<NurbCurve2D<S>>> {
    match &sketch.curve(curve)?.kind {
        CurveKind::Line { start, end } => Ok(vec![line(positions[start], positions[end])?]),
        CurveKind::Arc { start, end, sweep } => {
            let (s, e) = (positions[start], positions[end]);
            if *sweep == 0.0 {
                return Ok(vec![line(s, e)?]);
            }
            let arc = arc_of(positions, *start, *end, *sweep);
            let pieces = (sweep.abs() / FRAC_PI_2).ceil().max(1.0) as usize;
            let delta = sweep / pieces as f64;
            // Piece boundaries on the circle. Only needed for more than one
            // piece, where the arc turns by more than a quarter and so has a
            // center at a moderate distance.
            let mut ends = vec![s];
            if pieces > 1 {
                let c = arc.center();
                let r = arc.radius();
                let a0 = (s[1] - c[1]).atan2(s[0] - c[0]);
                ends.extend((1..pieces).map(|j| {
                    let a = a0 + delta * j as f64;
                    [c[0] + r * a.cos(), c[1] + r * a.sin()]
                }));
            }
            ends.push(e);
            ends.windows(2)
                .map(|w| {
                    // Tangent intersection from the chord alone: `(L/2)
                    // tan(δ/2)` to the chord's right (its left for a
                    // clockwise arc). Exact for any radius, including a
                    // nearly straight arc whose center is far away.
                    let piece = PlainArc {
                        s: w[0],
                        e: w[1],
                        half: delta / 2.0,
                    };
                    let bulge = piece.chord_length() * 0.5 * (delta / 2.0).tan();
                    let (cm, l) = (piece.chord_mid(), piece.left());
                    let m = [cm[0] - l[0] * bulge, cm[1] - l[1] * bulge];
                    conic(w[0], m, w[1], (delta / 2.0).cos())
                })
                .collect()
        }
        CurveKind::Circle { center, radius } => {
            let [cx, cy] = positions[center];
            let r = *radius;
            let q = [[cx + r, cy], [cx, cy + r], [cx - r, cy], [cx, cy - r]];
            let corners = [
                [cx + r, cy + r],
                [cx - r, cy + r],
                [cx - r, cy - r],
                [cx + r, cy - r],
            ];
            (0..4)
                .map(|j| conic(q[j], corners[j], q[(j + 1) % 4], SQRT_2 / 2.0))
                .collect()
        }
        CurveKind::Spline { control_points } => {
            let n = control_points.len();
            let degree = spline_degree(n);
            Ok(vec![NurbCurve::try_new(
                degree,
                control_points
                    .iter()
                    .map(|p| hom(positions[p], 1.0))
                    .collect(),
                unit_knots(&spline_knots(n, degree)),
            )?])
        }
    }
}

/// `curve` reparametrized from its domain onto `[0, 1]`.
fn rescale_to_unit<S: Scalar>(curve: NurbCurve2D<S>) -> GeopResult<NurbCurve2D<S>> {
    let (t0, t1) = curve.domain();
    let span = t1.sub(t0);
    let knots = curve
        .knot_vector
        .iter()
        .map(|&k| k.sub(t0).div(span))
        .collect::<GeopResult<Vec<S>>>()?;
    NurbCurve::try_new(curve.degree, curve.control_points, knots)
}

// ── containment, on the curves themselves ───────────────────────────────────

/// The largest distance any of `loops`' control points reaches from any
/// other: how long a ray has to be to leave every loop behind, and the
/// length the probe offsets below are measured against.
fn extent_of(loops: &[Vec<NurbCurve2D<F>>]) -> f64 {
    let points: Vec<[f64; 2]> = loops
        .iter()
        .flatten()
        .flat_map(|c| {
            c.control_points.iter().map(|cp| {
                let w = cp[2].to_f64();
                [cp[0].to_f64() / w, cp[1].to_f64() / w]
            })
        })
        .collect();
    let span = |k: usize| {
        let (lo, hi) = points
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                (lo.min(p[k]), hi.max(p[k]))
            });
        hi - lo
    };
    span(0).hypot(span(1)).max(1e-9)
}

/// The point halfway along `curve`, in plain coordinates.
fn midpoint(curve: &NurbCurve2D<F>) -> GeopResult<[f64; 2]> {
    let (t0, t1) = curve.domain();
    let p = curve.evaluate(t0.add(t1).div(F::TWO)?)?;
    Ok([p[0].to_f64(), p[1].to_f64()])
}

/// A segment from `from` in direction `dir`, long enough to leave a profile
/// of size `extent` behind.
fn ray(from: [f64; 2], dir: [f64; 2], extent: f64) -> GeopResult<NurbCurve2D<F>> {
    let length = 3.0 * extent;
    let to = [from[0] + dir[0] * length, from[1] + dir[1] * length];
    NurbCurve::try_new(
        1,
        vec![hom::<F>(from, 1.0), hom::<F>(to, 1.0)],
        unit_knots::<F>(&[0.0, 0.0, 1.0, 1.0]),
    )
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
/// around the circle without ever repeating.
fn loop_contains(curves: &[NurbCurve2D<F>], probe: [f64; 2], extent: f64) -> GeopResult<bool> {
    let min_subdivision = F::from_f64(MIN_SUBDIVISION);
    let mut last_rejection = String::new();
    'attempt: for k in 0..MAX_RAY_ATTEMPTS {
        let angle = k as f64 * 2.399_963_229_728_653;
        let ray = ray(probe, [angle.cos(), angle.sin()], extent)?;
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
                if !along_ray.definitely_greater(F::ZERO) {
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
fn turns_counter_clockwise(curves: &[NurbCurve2D<F>], extent: f64) -> GeopResult<bool> {
    let curve = &curves[0];
    let (t0, t1) = curve.domain();
    let mid = t0.add(t1).div(F::TWO)?;
    let point = curve.evaluate(mid)?;
    let tangent = curve.tangent(mid)?;
    let left = [F::ZERO.sub(tangent[1]).to_f64(), tangent[0].to_f64()];
    let mut step = extent / 64.0;
    for _ in 0..24 {
        let at = |sign: f64| {
            [
                point[0].to_f64() + left[0] * step * sign,
                point[1].to_f64() + left[1] * step * sign,
            ]
        };
        let inside_left = loop_contains(curves, at(1.0), extent)?;
        let inside_right = loop_contains(curves, at(-1.0), extent)?;
        if inside_left != inside_right {
            return Ok(inside_left);
        }
        step /= 2.0;
    }
    Err(GeopError::new(format!(
        "could not tell which way {curve:?} winds: both sides of it classify the same way"
    )))
}
