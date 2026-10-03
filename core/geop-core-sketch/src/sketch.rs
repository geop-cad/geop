//! Sketch entities and constraints: design data, in any [`Scalar`] — the
//! values a file holds, read as sharp scalars and written as their
//! midpoints.
//!
//! Points are the only entities with positions of their own; lines, arcs and
//! splines reference points by [`PointId`], so two curves that share an
//! endpoint share the *same* point and stay connected by construction.
//! Separately drawn points are joined with [`Constraint::Coincident`], which
//! the solver treats the same way (the two points become one set of
//! variables), so connectivity is always structural — never inferred from
//! two positions happening to be close.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Scalar, as_f64},
    vector::Vector2,
};

use serde::{Deserialize, Serialize};

macro_rules! define_ids {
    ($($(#[$doc:meta])* $name:ident => $prefix:literal),* $(,)?) => {
        $(
            $(#[$doc])*
            #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize)]
            #[serde(transparent)]
            pub struct $name(pub u64);

            /// The id as it appears in a topological name, e.g. `p3`.
            impl std::fmt::Display for $name {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    write!(f, concat!($prefix, "{}"), self.0)
                }
            }

            impl<'de> Deserialize<'de> for $name {
                fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
                    d.deserialize_any(IdVisitor).map($name)
                }
            }
        )*
    };
}

/// Reads an id from a number, or from the string a JSON object key holds it
/// as. Serde's derived `u64` accepts the key string only when it knows the
/// target type up front, not when the map was first buffered — as it is
/// inside a `#[serde(flatten)]`ed or internally tagged container, where a
/// sketch naturally ends up.
struct IdVisitor;

impl serde::de::Visitor<'_> for IdVisitor {
    type Value = u64;

    fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        f.write_str("a non-negative integer id, as a number or a string")
    }

    fn visit_u64<E: serde::de::Error>(self, v: u64) -> Result<u64, E> {
        Ok(v)
    }

    fn visit_i64<E: serde::de::Error>(self, v: i64) -> Result<u64, E> {
        u64::try_from(v).map_err(|_| E::custom(format!("id {v} is negative")))
    }

    fn visit_str<E: serde::de::Error>(self, v: &str) -> Result<u64, E> {
        v.parse()
            .map_err(|_| E::custom(format!("{v:?} is not an integer id")))
    }
}

define_ids!(
    /// Key of [`Sketch::points`].
    PointId => "p",
    /// Key of [`Sketch::curves`].
    CurveId => "c",
    /// Key of [`Sketch::constraints`].
    ConstraintId => "k",
);

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Point<S: Scalar> {
    #[serde(with = "as_f64")]
    pub x: S,
    #[serde(with = "as_f64")]
    pub y: S,
}

impl<S: Scalar> Point<S> {
    pub fn xy(&self) -> Vector2<S> {
        Vector2::from_array([self.x, self.y])
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", bound = "S: Scalar")]
pub enum CurveKind<S: Scalar> {
    Line {
        start: PointId,
        end: PointId,
    },
    /// A circular arc from `start` to `end`, turning counter-clockwise by
    /// `sweep` radians (clockwise if negative, `|sweep| < 2π`). Equivalent to
    /// giving its curvature `2 sin(sweep / 2) / |end - start|` — see
    /// [`crate::geometry`] for why the sweep is what is stored and solved.
    Arc {
        start: PointId,
        end: PointId,
        #[serde(with = "as_f64")]
        sweep: S,
    },
    Circle {
        center: PointId,
        #[serde(with = "as_f64")]
        radius: S,
    },
    /// A clamped, uniform, non-rational B-spline of degree
    /// `min(3, control_points.len() - 1)` through its first and last control
    /// points.
    Spline {
        control_points: Vec<PointId>,
    },
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Curve<S: Scalar> {
    #[serde(flatten)]
    pub kind: CurveKind<S>,
    /// Construction geometry takes part in constraints but not in profiles
    /// (e.g. a revolve axis or a symmetry line).
    #[serde(default)]
    pub construction: bool,
}

impl<S: Scalar> Curve<S> {
    /// The points this curve is defined by.
    pub fn points(&self) -> Vec<PointId> {
        match &self.kind {
            CurveKind::Line { start, end } | CurveKind::Arc { start, end, .. } => {
                vec![*start, *end]
            }
            CurveKind::Circle { center, .. } => vec![*center],
            CurveKind::Spline { control_points } => control_points.clone(),
        }
    }

    /// `(start, end)` for an open curve, `None` for a circle.
    pub fn endpoints(&self) -> Option<(PointId, PointId)> {
        match &self.kind {
            CurveKind::Line { start, end } | CurveKind::Arc { start, end, .. } => {
                Some((*start, *end))
            }
            CurveKind::Circle { .. } => None,
            CurveKind::Spline { control_points } => {
                Some((control_points[0], *control_points.last()?))
            }
        }
    }
}

/// The typical CAD sketch constraints. Distances and lengths are in sketch
/// units, angles in radians.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", bound = "S: Scalar")]
pub enum Constraint<S: Scalar> {
    /// `a` and `b` are the same point.
    Coincident {
        a: PointId,
        b: PointId,
    },
    /// `point` lies on `curve` (a line's infinite extension, an arc's full
    /// circle, or a circle).
    PointOnCurve {
        point: PointId,
        curve: CurveId,
    },
    Horizontal {
        line: CurveId,
    },
    Vertical {
        line: CurveId,
    },
    Parallel {
        a: CurveId,
        b: CurveId,
    },
    Perpendicular {
        a: CurveId,
        b: CurveId,
    },
    /// Two lines on one infinite line.
    Collinear {
        a: CurveId,
        b: CurveId,
    },
    /// Two curves meet tangentially: at a shared endpoint if they have one
    /// (lines, arcs and splines), else a line touching a circle/arc, or two
    /// circles/arcs touching each other.
    Tangent {
        a: CurveId,
        b: CurveId,
    },
    /// Equal length (two lines) or equal radius (two circles/arcs).
    Equal {
        a: CurveId,
        b: CurveId,
    },
    /// Two circles/arcs share a center.
    Concentric {
        a: CurveId,
        b: CurveId,
    },
    /// `point` is the midpoint of a line or arc.
    Midpoint {
        point: PointId,
        curve: CurveId,
    },
    /// `a` and `b` are mirror images across `line`.
    Symmetric {
        a: PointId,
        b: PointId,
        line: CurveId,
    },
    /// `point` stays at `(x, y)`.
    Fix {
        point: PointId,
        #[serde(with = "as_f64")]
        x: S,
        #[serde(with = "as_f64")]
        y: S,
    },
    Distance {
        a: PointId,
        b: PointId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// `b.x - a.x = value`.
    DistanceX {
        a: PointId,
        b: PointId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// `b.y - a.y = value`.
    DistanceY {
        a: PointId,
        b: PointId,
        #[serde(with = "as_f64")]
        value: S,
    },
    PointLineDistance {
        point: PointId,
        line: CurveId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// Length of a line or arc.
    Length {
        curve: CurveId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// Radius of a circle or arc.
    Radius {
        curve: CurveId,
        #[serde(with = "as_f64")]
        value: S,
    },
    /// The counter-clockwise angle from line `a`'s direction to line `b`'s.
    Angle {
        a: CurveId,
        b: CurveId,
        #[serde(with = "as_f64")]
        value: S,
    },
}

impl<S: Scalar> Constraint<S> {
    /// The points this constraint refers to directly (curves aside).
    pub fn points(&self) -> Vec<PointId> {
        use Constraint::*;
        match *self {
            Coincident { a, b }
            | Distance { a, b, .. }
            | DistanceX { a, b, .. }
            | DistanceY { a, b, .. }
            | Symmetric { a, b, .. } => vec![a, b],
            PointOnCurve { point, .. }
            | Midpoint { point, .. }
            | Fix { point, .. }
            | PointLineDistance { point, .. } => vec![point],
            _ => Vec::new(),
        }
    }

    /// The curves this constraint refers to.
    pub fn curves(&self) -> Vec<CurveId> {
        use Constraint::*;
        match *self {
            PointOnCurve { curve, .. }
            | Midpoint { curve, .. }
            | Length { curve, .. }
            | Radius { curve, .. } => vec![curve],
            Horizontal { line }
            | Vertical { line }
            | PointLineDistance { line, .. }
            | Symmetric { line, .. } => vec![line],
            Parallel { a, b }
            | Perpendicular { a, b }
            | Collinear { a, b }
            | Tangent { a, b }
            | Equal { a, b }
            | Concentric { a, b }
            | Angle { a, b, .. } => vec![a, b],
            _ => Vec::new(),
        }
    }
}

/// Every point's `[x, y]`, by [`PointId`]: the sketch's own positions, as
/// drawn (see [`Sketch::positions`]).
pub type Positions<S> = BTreeMap<PointId, Vector2<S>>;

/// A sketch's geometry as the kernel builds on it: every point, and every
/// arc's sweep and circle's radius. Solved (see [`Sketch::enclose`]), each
/// encloses the exact solution of the sketch's constraints; as drawn (see
/// [`Enclosure::as_drawn`]), each is the sketch's own value, sharp. A rigid
/// motion of either is one too (see [`crate::ProfileLoop::to_nurbs`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Enclosure<S: Scalar> {
    pub points: BTreeMap<PointId, Vector2<S>>,
    /// An arc's sweep, a circle's radius.
    pub params: BTreeMap<CurveId, S>,
}

impl<S: Scalar> Enclosure<S> {
    /// `sketch`'s geometry exactly as drawn — in the scalar type `S`, the
    /// sketch's own values as enclosures there (see [`Scalar::cast`]).
    pub fn as_drawn<D: Scalar>(sketch: &Sketch<D>) -> Self {
        Enclosure {
            points: sketch
                .points
                .iter()
                .map(|(&id, p)| (id, p.xy().map(|c| c.cast())))
                .collect(),
            params: sketch
                .curves
                .iter()
                .filter_map(|(&id, c)| match c.kind {
                    CurveKind::Arc { sweep, .. } => Some((id, sweep.cast())),
                    CurveKind::Circle { radius, .. } => Some((id, radius.cast())),
                    _ => None,
                })
                .collect(),
        }
    }
}

/// A constraint sketch.
///
/// Every entity is keyed by a stable id rather than stored by position: an id
/// is handed out once, by the `add_*` methods, and never reused — not even
/// after the entity is removed. That is what lets anything outside the sketch
/// (a constraint, a profile, a topological name of the solid extruded from
/// it) keep referring to "that line" while the sketch is edited around it.
/// The maps are ordered, so a serialized sketch is deterministic and diffs
/// line by line.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Sketch<S: Scalar> {
    pub points: BTreeMap<PointId, Point<S>>,
    pub curves: BTreeMap<CurveId, Curve<S>>,
    pub constraints: BTreeMap<ConstraintId, Constraint<S>>,
    /// The id the next added entity gets; greater than every id in use. One
    /// counter for all three kinds, so an id `add_*` hands out is unique
    /// across the sketch.
    pub next_id: u64,
}

impl<S: Scalar> Default for Sketch<S> {
    fn default() -> Self {
        Sketch {
            points: BTreeMap::new(),
            curves: BTreeMap::new(),
            constraints: BTreeMap::new(),
            next_id: 0,
        }
    }
}

impl<S: Scalar> Sketch<S> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn point(&self, id: PointId) -> GeopResult<&Point<S>> {
        self.points
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("sketch has no point {id}")))
    }

    pub fn curve(&self, id: CurveId) -> GeopResult<&Curve<S>> {
        self.curves
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("sketch has no curve {id}")))
    }

    /// Every point's `[x, y]`.
    pub fn positions(&self) -> Positions<S> {
        self.points.iter().map(|(&id, p)| (id, p.xy())).collect()
    }

    fn fresh_id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    /// Takes the id `id` for a new entity: `taken` says whether an entity of
    /// its kind already has it. Ids only need to be unique per kind — `p0`
    /// and `c0` are never confused — but the counter stays above all of
    /// them.
    fn claim_id(&mut self, id: u64, taken: bool) -> GeopResult<()> {
        if taken {
            return Err(GeopError::new(format!("sketch id {id} is already in use")));
        }
        self.next_id = self.next_id.max(id + 1);
        Ok(())
    }

    /// Adds `point` under the id `id` rather than a fresh one — for reading
    /// back a sketch whose ids were chosen already, so that what refers to
    /// them keeps doing so. Fails if `id` is taken.
    pub fn insert_point(&mut self, id: PointId, point: Point<S>) -> GeopResult<()> {
        self.claim_id(id.0, self.points.contains_key(&id))?;
        self.points.insert(id, point);
        Ok(())
    }

    /// Like [`Sketch::insert_point`], for a curve.
    pub fn insert_curve(&mut self, id: CurveId, curve: Curve<S>) -> GeopResult<()> {
        self.claim_id(id.0, self.curves.contains_key(&id))?;
        self.curves.insert(id, curve);
        Ok(())
    }

    pub fn add_point(&mut self, x: S, y: S) -> PointId {
        let id = PointId(self.fresh_id());
        self.points.insert(id, Point { x, y });
        id
    }

    fn add_curve(&mut self, kind: CurveKind<S>) -> CurveId {
        let id = CurveId(self.fresh_id());
        self.curves.insert(
            id,
            Curve {
                kind,
                construction: false,
            },
        );
        id
    }

    pub fn add_line(&mut self, start: PointId, end: PointId) -> CurveId {
        self.add_curve(CurveKind::Line { start, end })
    }

    /// An arc from `start` to `end`, turning counter-clockwise by `sweep`
    /// radians (clockwise if negative).
    pub fn add_arc(&mut self, start: PointId, end: PointId, sweep: S) -> CurveId {
        self.add_curve(CurveKind::Arc { start, end, sweep })
    }

    pub fn add_circle(&mut self, center: PointId, radius: S) -> CurveId {
        self.add_curve(CurveKind::Circle { center, radius })
    }

    pub fn add_spline(&mut self, control_points: Vec<PointId>) -> CurveId {
        self.add_curve(CurveKind::Spline { control_points })
    }

    pub fn set_construction(&mut self, curve: CurveId, construction: bool) {
        if let Some(c) = self.curves.get_mut(&curve) {
            c.construction = construction;
        }
    }

    pub fn constrain(&mut self, constraint: Constraint<S>) -> ConstraintId {
        let id = ConstraintId(self.fresh_id());
        self.constraints.insert(id, constraint);
        id
    }

    /// Removes `points`, `curves` and `constraints`, and everything that
    /// depends on them: a curve on a removed point, a constraint on any
    /// removed entity, and a point nothing uses any more because of it — a
    /// lone point drawn on purpose stays. Everything else keeps its id.
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
        self.constraints
            .retain(|id, _| !dead_constraints.contains(id));
    }

    /// Check every reference and every constraint's operand kinds, so the
    /// solver and profile code can rely on them.
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
                "sketch uses id {max_id}, but its next_id is only {}: new entities would reuse ids",
                self.next_id
            )));
        }
        for (&i, curve) in &self.curves {
            for p in curve.points() {
                self.point(p)
                    .map_err(|e| e.with_context(format!("curve {i}")))?;
            }
            match &curve.kind {
                CurveKind::Line { start, end } | CurveKind::Arc { start, end, .. }
                    if start == end =>
                {
                    return Err(GeopError::new(format!(
                        "curve {i} starts and ends at the same point"
                    )));
                }
                CurveKind::Arc { sweep, .. }
                    if !sweep.is_finite() || !sweep.abs().definitely_less(S::TWO.mul(S::PI)) =>
                {
                    return Err(GeopError::new(format!(
                        "arc {i} has sweep {sweep:?}, which is not definitely within (-2π, 2π)"
                    )));
                }
                CurveKind::Spline { control_points } if control_points.len() < 2 => {
                    return Err(GeopError::new(format!(
                        "spline {i} needs at least 2 control points"
                    )));
                }
                _ => {}
            }
        }
        for (&i, c) in &self.constraints {
            self.validate_constraint(c)
                .map_err(|e| e.with_context(format!("constraint {i} = {c:?}")))?;
        }
        Ok(())
    }

    fn validate_constraint(&self, c: &Constraint<S>) -> GeopResult<()> {
        use Constraint::*;
        for p in c.points() {
            self.point(p)?;
        }
        let kind = |id: CurveId| self.curve(id).map(|c| &c.kind);
        let is_line = |id| Ok::<_, GeopError>(matches!(kind(id)?, CurveKind::Line { .. }));
        let is_round = |id| {
            Ok::<_, GeopError>(matches!(
                kind(id)?,
                CurveKind::Arc { .. } | CurveKind::Circle { .. }
            ))
        };
        let need = |ok: bool, what: &str| {
            if ok {
                Ok(())
            } else {
                Err(GeopError::new(format!("operands must be {what}")))
            }
        };
        match c {
            Coincident { .. }
            | Distance { .. }
            | DistanceX { .. }
            | DistanceY { .. }
            | Fix { .. } => Ok(()),
            PointOnCurve { curve, .. } => need(
                !matches!(kind(*curve)?, CurveKind::Spline { .. }),
                "a point and a line, arc or circle",
            ),
            Horizontal { line } | Vertical { line } => need(is_line(*line)?, "a line"),
            Parallel { a, b }
            | Perpendicular { a, b }
            | Collinear { a, b }
            | Angle { a, b, .. } => need(is_line(*a)? && is_line(*b)?, "two lines"),
            Tangent { a, b } => {
                let shared = self.shared_endpoint(*a, *b)?.is_some();
                let ok = shared
                    && !(is_line(*a)? && is_line(*b)?)
                    && !matches!(kind(*a)?, CurveKind::Circle { .. })
                    && !matches!(kind(*b)?, CurveKind::Circle { .. })
                    || !shared
                        && (is_round(*a)? && (is_round(*b)? || is_line(*b)?)
                            || is_line(*a)? && is_round(*b)?);
                need(
                    ok,
                    "curves sharing an endpoint (not two lines), a line and a circle/arc, or two circles/arcs",
                )
            }
            Equal { a, b } => need(
                is_line(*a)? && is_line(*b)? || is_round(*a)? && is_round(*b)?,
                "two lines or two circles/arcs",
            ),
            Concentric { a, b } => need(is_round(*a)? && is_round(*b)?, "two circles/arcs"),
            Midpoint { curve, .. } => need(
                matches!(
                    kind(*curve)?,
                    CurveKind::Line { .. } | CurveKind::Arc { .. }
                ),
                "a point and a line or arc",
            ),
            Symmetric { line, .. } => need(is_line(*line)?, "two points and a line"),
            PointLineDistance { line, .. } => need(is_line(*line)?, "a point and a line"),
            Length { curve, .. } => need(
                matches!(
                    kind(*curve)?,
                    CurveKind::Line { .. } | CurveKind::Arc { .. }
                ),
                "a line or arc",
            ),
            Radius { curve, .. } => need(is_round(*curve)?, "a circle or arc"),
        }
    }

    /// Union-find representative of every point under
    /// [`Constraint::Coincident`]: points with the same representative are
    /// one point.
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
            if let Constraint::Coincident { a, b } = *c {
                let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                // The lower id represents the class, so the representative
                // does not depend on constraint order.
                let (lo, hi) = (ra.min(rb), ra.max(rb));
                parent.insert(hi, lo);
            }
        }
        self.points
            .keys()
            .map(|&id| (id, find(&mut parent, id)))
            .collect()
    }

    /// Every point the constraints put on the infinite line through the line
    /// `line`: its endpoints, points constrained onto it
    /// ([`Constraint::PointOnCurve`], a [`Constraint::Midpoint`] of it, an
    /// endpoint of a [`Constraint::Collinear`] partner), and any point
    /// coincident with one of those.
    ///
    /// Structural, like all connectivity here: a point that merely happens
    /// to lie on the line is not reported.
    pub fn on_line(&self, line: CurveId) -> GeopResult<BTreeSet<PointId>> {
        let CurveKind::Line { start, end } = self.curve(line)?.kind else {
            return Err(GeopError::new(format!("curve {line} is not a line")));
        };
        let class = self.point_classes();
        let mut on = BTreeSet::new();
        let mut mark = |p: PointId| {
            on.insert(class[&p]);
        };
        mark(start);
        mark(end);
        for c in self.constraints.values() {
            match *c {
                Constraint::PointOnCurve { point, curve }
                | Constraint::Midpoint { point, curve }
                    if curve == line =>
                {
                    mark(point)
                }
                Constraint::Collinear { a, b } if a == line || b == line => {
                    let other = if a == line { b } else { a };
                    for p in self.curve(other)?.points() {
                        mark(p);
                    }
                }
                _ => {}
            }
        }
        Ok(class
            .iter()
            .filter(|(_, rep)| on.contains(rep))
            .map(|(&p, _)| p)
            .collect())
    }

    /// Which ends `(a_at_end, b_at_end)` of open curves `a` and `b` are the
    /// same point, if any (`false` = start, `true` = end).
    pub fn shared_endpoint(&self, a: CurveId, b: CurveId) -> GeopResult<Option<(bool, bool)>> {
        let (Some((a0, a1)), Some((b0, b1))) =
            (self.curve(a)?.endpoints(), self.curve(b)?.endpoints())
        else {
            return Ok(None);
        };
        let class = self.point_classes();
        let same = |p: PointId, q: PointId| class[&p] == class[&q];
        Ok([(false, false), (false, true), (true, false), (true, true)]
            .into_iter()
            .find(|&(ea, eb)| same(if ea { a1 } else { a0 }, if eb { b1 } else { b0 })))
    }
}
