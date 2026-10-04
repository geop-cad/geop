//! 3-D sketches: points in space, and the lines, arcs and splines through
//! them — the paths sweeps, pipes, frames and wire routes run along, and the
//! rails a sweep is guided by.
//!
//! The same principles as the planar [`crate::Sketch`]:
//!
//! - **Points carry all positions.** Curves refer to points by
//!   [`PointId`], so curves sharing a point stay connected by construction;
//!   [`Constraint3d::Coincident`] joins separately drawn points into one.
//! - **What is given from outside is fixed.** A point at a vertex of the
//!   part or at a datum point is a *fixed* point, put where that is by
//!   whoever owns the sketch; an edge of the part a point is constrained
//!   onto is a [`CurveKind3d::Reference`] curve, its geometry handed in
//!   ([`Sketch3d::references`]). The solver never moves either.
//! - **Directions are given.** An axis or a straight edge a line is
//!   parallel to, or a tangent a spline leaves along, is a direction vector
//!   in the constraint, brought up to date by the sketch's owner.
//!
//! Curves:
//!
//! - a **line** between two points;
//! - a **circular arc** from a point, through a second, to a third — the
//!   arc lies in their plane;
//! - a **spline** through points: the natural cubic spline through them
//!   (see [`geop_core_geometry::nurb_curve::NurbCurve::cubic_spline`]),
//!   whose ends are free unless a [`Constraint3d::TangentTo`] gives them a
//!   direction or a [`Constraint3d::Tangent`] joins them smoothly to the
//!   curve before or after. Either is how the spline is *built*, not
//!   something the solver moves points for: a spline end tangent to a line
//!   leaves along the line.
//!
//! [`solve`](Sketch3d::solve) moves the points so the constraints hold,
//! [`enclose`](Sketch3d::enclose) encloses the exact solution, and
//! [`chains`](Sketch3d::chains) / [`Chain3d::to_nurbs`] turn the curves into
//! NURBS in space, chain by chain.

mod curves;
mod geometry;
mod solve;
#[cfg(test)]
mod tests;

pub use curves::{Chain3d, Piece3d};
pub use geometry::Arc3;
pub use solve::Solve3dReport;

use std::collections::{BTreeMap, BTreeSet};

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Scalar, as_f64},
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::{ConstraintId, CurveId, PointId};

fn is_false(b: &bool) -> bool {
    !*b
}

/// A point of a 3-D sketch.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Point3d<S: Scalar> {
    pub at: Vector3<S>,
    /// Given rather than solved for: a vertex of the part, a datum point.
    #[serde(default, skip_serializing_if = "is_false")]
    pub fixed: bool,
}

/// What a curve of a 3-D sketch is.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum CurveKind3d {
    Line {
        start: PointId,
        end: PointId,
    },
    /// The circular arc from `start` through `through` to `end`.
    Arc {
        start: PointId,
        through: PointId,
        end: PointId,
    },
    /// The cubic spline through `points`, in order.
    Spline {
        points: Vec<PointId>,
    },
    /// A curve given from outside — an edge of the part — whose geometry is
    /// in [`Sketch3d::references`]: only for points to be constrained onto.
    Reference,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Curve3d {
    #[serde(flatten)]
    pub kind: CurveKind3d,
    /// Takes part in constraints, but in no chain.
    #[serde(default, skip_serializing_if = "is_false")]
    pub construction: bool,
}

impl Curve3d {
    /// The points it is defined by.
    pub fn points(&self) -> Vec<PointId> {
        match &self.kind {
            CurveKind3d::Line { start, end } => vec![*start, *end],
            CurveKind3d::Arc {
                start,
                through,
                end,
            } => vec![*start, *through, *end],
            CurveKind3d::Spline { points } => points.clone(),
            CurveKind3d::Reference => Vec::new(),
        }
    }

    /// `(start, end)`; none for a reference curve, which has no points of
    /// the sketch.
    pub fn endpoints(&self) -> Option<(PointId, PointId)> {
        match &self.kind {
            CurveKind3d::Line { start, end } | CurveKind3d::Arc { start, end, .. } => {
                Some((*start, *end))
            }
            CurveKind3d::Spline { points } => Some((*points.first()?, *points.last()?)),
            CurveKind3d::Reference => None,
        }
    }

    /// Whether it is drawn — a line, an arc or a spline — rather than given.
    pub fn is_drawn(&self) -> bool {
        !matches!(self.kind, CurveKind3d::Reference)
    }
}

/// An end of a curve.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum End {
    Start,
    End,
}

/// A coordinate axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Coordinate {
    X,
    Y,
    Z,
}

impl Coordinate {
    pub const ALL: [Coordinate; 3] = [Coordinate::X, Coordinate::Y, Coordinate::Z];

    /// Its index in a vector.
    pub fn index(self) -> usize {
        match self {
            Coordinate::X => 0,
            Coordinate::Y => 1,
            Coordinate::Z => 2,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Coordinate::X => "x",
            Coordinate::Y => "y",
            Coordinate::Z => "z",
        }
    }
}

/// The constraints of a 3-D sketch. Lengths are in sketch units.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", bound = "S: Scalar")]
pub enum Constraint3d<S: Scalar> {
    /// `a` and `b` are the same point.
    Coincident { a: PointId, b: PointId },
    /// The coordinate `axis` of `point` is `value`.
    Coordinate {
        point: PointId,
        axis: Coordinate,
        #[serde(with = "as_f64")]
        value: S,
    },
    Distance {
        a: PointId,
        b: PointId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// The length of a line.
    Length {
        line: CurveId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// The radius of an arc.
    Radius {
        arc: CurveId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// Two lines are parallel.
    Parallel { a: CurveId, b: CurveId },
    /// A line runs along `direction`: an axis, a straight edge.
    ParallelTo {
        line: CurveId,
        direction: Vector3<S>,
    },
    /// An arc or a spline leaves (`end` its start) or arrives (its end)
    /// along `direction`, either way along it.
    TangentTo {
        curve: CurveId,
        end: End,
        direction: Vector3<S>,
    },
    /// `point` lies on `curve`: within a line or an arc's full circle, or
    /// on a reference curve.
    OnCurve { point: PointId, curve: CurveId },
    /// Two curves meeting at an end of each go on smoothly there.
    Tangent { a: CurveId, b: CurveId },
}

impl<S: Scalar> Constraint3d<S> {
    /// The points it refers to directly, curves aside.
    pub fn points(&self) -> Vec<PointId> {
        use Constraint3d::*;
        match *self {
            Coincident { a, b } | Distance { a, b, .. } => vec![a, b],
            Coordinate { point, .. } | OnCurve { point, .. } => vec![point],
            _ => Vec::new(),
        }
    }

    /// The curves it refers to.
    pub fn curves(&self) -> Vec<CurveId> {
        use Constraint3d::*;
        match *self {
            Length { line: c, .. }
            | Radius { arc: c, .. }
            | ParallelTo { line: c, .. }
            | TangentTo { curve: c, .. }
            | OnCurve { curve: c, .. } => vec![c],
            Parallel { a, b } | Tangent { a, b } => vec![a, b],
            Coincident { .. } | Coordinate { .. } | Distance { .. } => Vec::new(),
        }
    }

    /// The dimension it holds, if it holds one: what a dialog edits.
    pub fn value(&self) -> Option<S> {
        use Constraint3d::*;
        match *self {
            Coordinate { value, .. }
            | Distance { value, .. }
            | Length { value, .. }
            | Radius { value, .. } => Some(value),
            _ => None,
        }
    }

    /// Gives its dimension the value `v`; nothing for one without.
    pub fn set_value(&mut self, v: S) {
        use Constraint3d::*;
        match self {
            Coordinate { value, .. }
            | Distance { value, .. }
            | Length { value, .. }
            | Radius { value, .. } => *value = v,
            _ => {}
        }
    }
}

/// A 3-D sketch's geometry as the kernel builds on it: every point, solved
/// (see [`Sketch3d::enclose`]) or as drawn ([`Enclosure3d::as_drawn`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Enclosure3d<S: Scalar> {
    pub points: BTreeMap<PointId, Vector3<S>>,
}

impl<S: Scalar> Enclosure3d<S> {
    /// `sketch`'s points exactly as drawn, as enclosures in `S`.
    pub fn as_drawn<D: Scalar>(sketch: &Sketch3d<D>) -> Self {
        Enclosure3d {
            points: sketch
                .points
                .iter()
                .map(|(&id, p)| (id, p.at.map(|c| c.cast())))
                .collect(),
        }
    }
}

/// A 3-D constraint sketch (see the module docs). Ids are handed out once
/// and never reused, as in a planar [`crate::Sketch`].
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Sketch3d<S: Scalar> {
    pub points: BTreeMap<PointId, Point3d<S>>,
    pub curves: BTreeMap<CurveId, Curve3d>,
    pub constraints: BTreeMap<ConstraintId, Constraint3d<S>>,
    /// The id the next added entity gets: one counter for all three kinds.
    pub next_id: u64,
    /// The geometry of every [`CurveKind3d::Reference`] curve: given from
    /// outside, so not saved with the sketch — its owner hands it in before
    /// solving or building.
    #[serde(skip)]
    pub references: BTreeMap<CurveId, NurbCurve3D<S>>,
}

/// Two sketches are the same design: the same points, curves and
/// constraints — whatever geometry was handed in for their references.
impl<S: Scalar + PartialEq> PartialEq for Sketch3d<S> {
    fn eq(&self, other: &Self) -> bool {
        self.points == other.points
            && self.curves == other.curves
            && self.constraints == other.constraints
            && self.next_id == other.next_id
    }
}

impl<S: Scalar> Default for Sketch3d<S> {
    fn default() -> Self {
        Sketch3d {
            points: BTreeMap::new(),
            curves: BTreeMap::new(),
            constraints: BTreeMap::new(),
            next_id: 0,
            references: BTreeMap::new(),
        }
    }
}

impl<S: Scalar> Sketch3d<S> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn point(&self, id: PointId) -> GeopResult<&Point3d<S>> {
        self.points
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("3-D sketch has no point {id}")))
    }

    pub fn curve(&self, id: CurveId) -> GeopResult<&Curve3d> {
        self.curves
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("3-D sketch has no curve {id}")))
    }

    fn fresh_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    pub fn add_point(&mut self, at: Vector3<S>) -> PointId {
        let id = PointId(self.fresh_id());
        self.points.insert(id, Point3d { at, fixed: false });
        id
    }

    /// A point given from outside the sketch, at `at`.
    pub fn add_fixed_point(&mut self, at: Vector3<S>) -> PointId {
        let id = PointId(self.fresh_id());
        self.points.insert(id, Point3d { at, fixed: true });
        id
    }

    pub fn add_curve(&mut self, kind: CurveKind3d) -> CurveId {
        let id = CurveId(self.fresh_id());
        self.curves.insert(
            id,
            Curve3d {
                kind,
                construction: false,
            },
        );
        id
    }

    pub fn add_line(&mut self, start: PointId, end: PointId) -> CurveId {
        self.add_curve(CurveKind3d::Line { start, end })
    }

    pub fn add_arc(&mut self, start: PointId, through: PointId, end: PointId) -> CurveId {
        self.add_curve(CurveKind3d::Arc {
            start,
            through,
            end,
        })
    }

    pub fn add_spline(&mut self, points: Vec<PointId>) -> CurveId {
        self.add_curve(CurveKind3d::Spline { points })
    }

    /// A curve given from outside, `curve`, to constrain points onto.
    pub fn add_reference(&mut self, curve: NurbCurve3D<S>) -> CurveId {
        let id = self.add_curve(CurveKind3d::Reference);
        self.references.insert(id, curve);
        id
    }

    pub fn constrain(&mut self, constraint: Constraint3d<S>) -> ConstraintId {
        let id = ConstraintId(self.fresh_id());
        self.constraints.insert(id, constraint);
        id
    }

    /// Removes `points`, `curves` and `constraints`, and everything that
    /// depends on them: a curve on a removed point, a constraint on any
    /// removed entity, and a point nothing uses any more because of it.
    pub fn remove(&mut self, points: &[PointId], curves: &[CurveId], constraints: &[ConstraintId]) {
        let dead_curves: BTreeSet<CurveId> = self
            .curves
            .iter()
            .filter(|(id, c)| curves.contains(id) || c.points().iter().any(|p| points.contains(p)))
            .map(|(&id, _)| id)
            .collect();
        let dead_constraints: BTreeSet<ConstraintId> = self
            .constraints
            .iter()
            .filter(|(id, c)| {
                constraints.contains(id)
                    || c.points().iter().any(|p| points.contains(p))
                    || c.curves().iter().any(|k| dead_curves.contains(k))
            })
            .map(|(&id, _)| id)
            .collect();
        let mut used_before = BTreeSet::new();
        let mut used_after = BTreeSet::new();
        for (id, c) in &self.curves {
            for p in c.points() {
                used_before.insert(p);
                if !dead_curves.contains(id) {
                    used_after.insert(p);
                }
            }
        }
        for (id, c) in &self.constraints {
            for p in c.points() {
                used_before.insert(p);
                if !dead_constraints.contains(id) {
                    used_after.insert(p);
                }
            }
        }
        self.points.retain(|p, _| {
            !points.contains(p) && !(used_before.contains(p) && !used_after.contains(p))
        });
        self.curves.retain(|id, _| !dead_curves.contains(id));
        self.references.retain(|id, _| !dead_curves.contains(id));
        self.constraints
            .retain(|id, _| !dead_constraints.contains(id));
    }

    /// Union-find representative of every point under
    /// [`Constraint3d::Coincident`] — the lowest id of its class: points
    /// with the same representative are one point.
    pub fn point_classes(&self) -> BTreeMap<PointId, PointId> {
        fn find(parent: &mut BTreeMap<PointId, PointId>, i: PointId) -> PointId {
            let mut r = i;
            while parent[&r] != r {
                r = parent[&r];
            }
            let mut i = i;
            while parent[&i] != r {
                let next = parent[&i];
                parent.insert(i, r);
                i = next;
            }
            r
        }
        let mut parent: BTreeMap<PointId, PointId> =
            self.points.keys().map(|&id| (id, id)).collect();
        for c in self.constraints.values() {
            if let Constraint3d::Coincident { a, b } = *c {
                let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                parent.insert(ra.max(rb), ra.min(rb));
            }
        }
        self.points
            .keys()
            .map(|&id| (id, find(&mut parent, id)))
            .collect()
    }

    /// Which ends of `a` and `b` are one point, if any.
    pub fn shared_end(&self, a: CurveId, b: CurveId) -> GeopResult<Option<(End, End)>> {
        let (Some((a0, a1)), Some((b0, b1))) =
            (self.curve(a)?.endpoints(), self.curve(b)?.endpoints())
        else {
            return Ok(None);
        };
        let class = self.point_classes();
        let at = |p: PointId| class[&p];
        let pick = |e: End, s: PointId, t: PointId| if e == End::Start { s } else { t };
        Ok([
            (End::Start, End::Start),
            (End::Start, End::End),
            (End::End, End::Start),
            (End::End, End::End),
        ]
        .into_iter()
        .find(|&(ea, eb)| at(pick(ea, a0, a1)) == at(pick(eb, b0, b1))))
    }

    /// Checks every reference and every constraint's operands, so the
    /// solver and the curves can rely on them.
    pub fn validate(&self) -> GeopResult<()> {
        let max_id = [
            self.points.keys().last().map(|id| id.0),
            self.curves.keys().last().map(|id| id.0),
            self.constraints.keys().last().map(|id| id.0),
        ];
        if let Some(max_id) = max_id.into_iter().flatten().max()
            && max_id >= self.next_id
        {
            return Err(GeopError::new(format!(
                "3-D sketch uses id {max_id}, but its next_id is only {}",
                self.next_id
            )));
        }
        let class = self.point_classes();
        for (&i, curve) in &self.curves {
            for p in curve.points() {
                self.point(p)
                    .map_err(|e| e.with_context(format!("curve {i}")))?;
            }
            let distinct = |ps: &[PointId]| ps.windows(2).all(|w| class[&w[0]] != class[&w[1]]);
            match &curve.kind {
                CurveKind3d::Line { start, end } if class[start] == class[end] => {
                    return Err(GeopError::new(format!(
                        "line {i} starts and ends at the same point"
                    )));
                }
                CurveKind3d::Arc {
                    start,
                    through,
                    end,
                } if !distinct(&[*start, *through, *end]) || class[start] == class[end] => {
                    return Err(GeopError::new(format!(
                        "arc {i} needs three different points"
                    )));
                }
                CurveKind3d::Spline { points } if points.len() < 2 || !distinct(points) => {
                    return Err(GeopError::new(format!(
                        "spline {i} needs at least two points, each different from the one before"
                    )));
                }
                CurveKind3d::Reference if !self.references.contains_key(&i) => {
                    return Err(GeopError::new(format!(
                        "reference curve {i} has no geometry handed in"
                    )));
                }
                _ => {}
            }
        }
        // Each spline end gets its direction from one place at most.
        let mut directed: BTreeMap<(CurveId, End), ConstraintId> = BTreeMap::new();
        for (&k, c) in &self.constraints {
            self.validate_constraint(c)
                .map_err(|e| e.with_context(format!("constraint {k} = {c:?}")))?;
            for (curve, end) in self.spline_ends_directed(c)? {
                if let Some(other) = directed.insert((curve, end), k) {
                    return Err(GeopError::new(format!(
                        "the {end:?} of spline {curve} is given its direction twice, by constraints {other} and {k}: a spline end leaves along one direction"
                    )));
                }
            }
        }
        Ok(())
    }

    fn validate_constraint(&self, c: &Constraint3d<S>) -> GeopResult<()> {
        use Constraint3d::*;
        for p in c.points() {
            self.point(p)?;
        }
        let kind = |id: CurveId| self.curve(id).map(|c| &c.kind);
        let is_line = |id| Ok::<_, GeopError>(matches!(kind(id)?, CurveKind3d::Line { .. }));
        let need = |ok: bool, what: &str| {
            if ok {
                Ok(())
            } else {
                Err(GeopError::new(format!("operands must be {what}")))
            }
        };
        match c {
            Coincident { .. } | Coordinate { .. } | Distance { .. } => Ok(()),
            Length { line, .. } => need(is_line(*line)?, "a line"),
            Radius { arc, .. } => need(matches!(kind(*arc)?, CurveKind3d::Arc { .. }), "an arc"),
            Parallel { a, b } => need(is_line(*a)? && is_line(*b)?, "two lines"),
            ParallelTo { line, direction } => {
                need(is_line(*line)?, "a line")?;
                direction.normalize().map(|_| ())
            }
            TangentTo {
                curve, direction, ..
            } => {
                need(
                    matches!(
                        kind(*curve)?,
                        CurveKind3d::Arc { .. } | CurveKind3d::Spline { .. }
                    ),
                    "an arc or a spline",
                )?;
                direction.normalize().map(|_| ())
            }
            OnCurve { curve, .. } => need(
                !matches!(kind(*curve)?, CurveKind3d::Spline { .. }),
                "a point and a line, an arc or an edge (a point on a spline is not supported)",
            ),
            Tangent { a, b } => {
                need(
                    self.curve(*a)?.is_drawn() && self.curve(*b)?.is_drawn(),
                    "two drawn curves",
                )?;
                need(
                    self.shared_end(*a, *b)?.is_some(),
                    "two curves meeting at an end of each",
                )
            }
        }
    }

    /// The spline ends `c` gives a direction to: a [`Constraint3d::TangentTo`]
    /// its curve's end, a [`Constraint3d::Tangent`] the end of the spline that
    /// follows the other curve's direction (see [`Sketch3d::adopts`]).
    fn spline_ends_directed(&self, c: &Constraint3d<S>) -> GeopResult<Vec<(CurveId, End)>> {
        Ok(match *c {
            Constraint3d::TangentTo { curve, end, .. }
                if matches!(self.curve(curve)?.kind, CurveKind3d::Spline { .. }) =>
            {
                vec![(curve, end)]
            }
            Constraint3d::Tangent { a, b } => self.adopts(a, b)?.into_iter().collect(),
            _ => Vec::new(),
        })
    }

    /// For a [`Constraint3d::Tangent`] between `a` and `b`, which spline end
    /// takes the other curve's direction there, if one does: a spline's end
    /// meeting a line or an arc; of two splines, the newer one's (the
    /// higher id), so that no spline ever waits on itself. `None` where
    /// neither is a spline: the solver makes those two tangent.
    pub fn adopts(&self, a: CurveId, b: CurveId) -> GeopResult<Option<(CurveId, End)>> {
        let Some((ea, eb)) = self.shared_end(a, b)? else {
            return Ok(None);
        };
        let spline = |c: CurveId| -> GeopResult<bool> {
            Ok(matches!(self.curve(c)?.kind, CurveKind3d::Spline { .. }))
        };
        Ok(match (spline(a)?, spline(b)?) {
            (false, false) => None,
            (true, false) => Some((a, ea)),
            (false, true) => Some((b, eb)),
            (true, true) if a > b => Some((a, ea)),
            (true, true) => Some((b, eb)),
        })
    }
}
