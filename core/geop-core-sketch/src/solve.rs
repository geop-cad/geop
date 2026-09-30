//! Solving a sketch: every constraint contributes residuals that are zero
//! exactly when it holds, and [`crate::bfgs::minimize`] drives the sum of
//! their squares to zero.
//!
//! **Variables.** Each class of coincident points (see
//! [`Sketch::point_classes`]) is one `(x, y)` pair — coincidence is not a
//! residual at all, it removes two degrees of freedom by construction. Each
//! arc adds its half sweep, each circle its radius.
//!
//! **Units.** Every residual is a length: dimensionless ones (angles,
//! parallelism) are multiplied by the sketch's characteristic size, so no
//! constraint kind dominates the objective just by its choice of units, and
//! the convergence test is a single relative length.
//!
//! **Floating point variables, interval-scalar residuals.** A sketch is
//! design intent, not a geometric claim: the solved positions are the
//! designer's free choice, entering the kernel as exact inputs through
//! [`crate::profile`]. So [`bfgs::minimize`] itself still walks plain `f64`
//! variables and its tolerances only decide when to stop iterating — but
//! each residual along the way is computed as a [`Dual<ScalInF64>`], so the
//! `sin`/`sqrt`/`PI` a geometric formula can't help but need are honestly
//! enclosed rather than quietly rounded away. A residual that becomes
//! genuinely undecidable (a division or square root at the edge of its
//! domain) is treated as "this point is infeasible" — the same outcome a
//! plain `f64` computing `inf`/`nan` there would already have produced.
use crate::{
    bfgs::{BfgsOptions, minimize},
    dual::{Dual, MAX_LOCAL_VARS},
    geometry::{Arc, V, line_distance},
    point::P2,
    sketch::{Constraint, ConstraintId, CurveId, CurveKind, Enclosure, PointId, Sketch},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Ring, Scalar, scal_in_f64::ScalInF64},
    vector::Vector2,
};

/// The scalar every residual is computed in: an honest enclosure, not a bare
/// `f64` — see the module docs.
type S = ScalInF64;

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Maps sketch entities to solver variables.
struct Layout {
    /// Index of the `x` variable of each point (its `y` follows).
    point_var: BTreeMap<PointId, usize>,
    /// The arc's half sweep or the circle's radius, for every arc and circle.
    curve_var: BTreeMap<CurveId, usize>,
    n: usize,
}

impl Layout {
    fn new(sketch: &Sketch) -> Self {
        let class = sketch.point_classes();
        let mut n = 0;
        let mut class_var = BTreeMap::new();
        for rep in class.values() {
            class_var.entry(*rep).or_insert_with(|| {
                n += 2;
                n - 2
            });
        }
        let point_var = class.iter().map(|(&p, rep)| (p, class_var[rep])).collect();
        let curve_var = sketch
            .curves
            .iter()
            .filter(|(_, c)| matches!(c.kind, CurveKind::Arc { .. } | CurveKind::Circle { .. }))
            .map(|(&id, _)| {
                n += 1;
                (id, n - 1)
            })
            .collect();
        Layout {
            point_var,
            curve_var,
            n,
        }
    }

    /// The current variable values, read from the sketch.
    fn read(&self, sketch: &Sketch) -> Vec<f64> {
        let mut x = vec![0.0; self.n];
        // Iterate in reverse so a class takes its representative's (lowest
        // id) position.
        for (id, p) in sketch.points.iter().rev() {
            x[self.point_var[id]] = p.x;
            x[self.point_var[id] + 1] = p.y;
        }
        for (id, c) in &sketch.curves {
            match c.kind {
                CurveKind::Arc { sweep, .. } => x[self.curve_var[id]] = sweep / 2.0,
                CurveKind::Circle { radius, .. } => x[self.curve_var[id]] = radius,
                _ => {}
            }
        }
        x
    }

    /// Write variable values back into the sketch.
    fn write(&self, sketch: &mut Sketch, x: &[f64]) {
        for (id, p) in sketch.points.iter_mut() {
            p.x = x[self.point_var[id]];
            p.y = x[self.point_var[id] + 1];
        }
        for (id, c) in sketch.curves.iter_mut() {
            match &mut c.kind {
                CurveKind::Arc { sweep, .. } => *sweep = 2.0 * x[self.curve_var[id]],
                CurveKind::Circle { radius, .. } => *radius = x[self.curve_var[id]].abs(),
                _ => {}
            }
        }
    }

    /// The variables a curve depends on.
    fn curve_vars(&self, sketch: &Sketch, c: CurveId, out: &mut Vec<usize>) {
        for p in sketch.curves[&c].points() {
            out.extend([self.point_var[&p], self.point_var[&p] + 1]);
        }
        out.extend(self.curve_var.get(&c));
    }
}

/// Read-only view of the variables for evaluating one constraint, with that
/// constraint's own variables `seeded` as [`Scalar`] values.
struct Geo<'a, T> {
    sketch: &'a Sketch,
    layout: &'a Layout,
    x: &'a [f64],
    seeded: &'a [(usize, T)],
}

impl<T: Scalar> Geo<'_, T> {
    fn var(&self, i: usize) -> T {
        self.seeded
            .iter()
            .find(|(j, _)| *j == i)
            .map(|(_, t)| *t)
            .unwrap_or_else(|| T::from_f64(self.x[i]))
    }
    fn point(&self, p: PointId) -> V<T> {
        let i = self.layout.point_var[&p];
        V::new(self.var(i), self.var(i + 1))
    }
    fn line(&self, c: CurveId) -> (V<T>, V<T>) {
        match &self.sketch.curves[&c].kind {
            CurveKind::Line { start, end } => (self.point(*start), self.point(*end)),
            k => unreachable!("validated to be a line: {k:?}"),
        }
    }
    fn arc(&self, c: CurveId) -> Option<Arc<T>> {
        match &self.sketch.curves[&c].kind {
            CurveKind::Arc { start, end, .. } => Some(Arc {
                s: self.point(*start),
                e: self.point(*end),
                half: self.var(self.layout.curve_var[&c]),
            }),
            _ => None,
        }
    }
    /// `(center, radius)` of a circle or arc.
    fn round(&self, c: CurveId) -> GeopResult<(V<T>, T)> {
        Ok(match &self.sketch.curves[&c].kind {
            CurveKind::Circle { center, .. } => (
                self.point(*center),
                self.var(self.layout.curve_var[&c]).abs(),
            ),
            CurveKind::Arc { .. } => {
                let a = self.arc(c).unwrap();
                (a.center()?, a.radius()?)
            }
            k => unreachable!("validated to be a circle or arc: {k:?}"),
        })
    }
    /// Unit tangent of an open curve at its start (`at_end = false`) or end,
    /// in the direction of travel.
    fn end_tangent(&self, c: CurveId, at_end: bool) -> GeopResult<V<T>> {
        Ok(match &self.sketch.curves[&c].kind {
            CurveKind::Line { .. } => {
                let (s, e) = self.line(c);
                e.sub(s).unit()?
            }
            CurveKind::Arc { .. } => {
                let a = self.arc(c).unwrap();
                if at_end {
                    a.tangent_end()?
                } else {
                    a.tangent_start()?
                }
            }
            CurveKind::Spline { control_points } => {
                let n = control_points.len();
                let (p, q) = if at_end {
                    (control_points[n - 2], control_points[n - 1])
                } else {
                    (control_points[0], control_points[1])
                };
                self.point(q).sub(self.point(p)).unit()?
            }
            k => unreachable!("validated to be an open curve: {k:?}"),
        })
    }
}

/// How a [`Constraint::Tangent`] is enforced, decided once per solve from
/// the operands' connectivity and their starting configuration.
#[derive(Clone, Copy, Debug)]
enum TangentMode {
    /// The curves share an endpoint: their tangents there are parallel.
    Endpoint { a_end: bool, b_end: bool },
    /// Line `line` touches circle/arc `round`.
    LineRound { line: CurveId, round: CurveId },
    /// Two circles/arcs touch, from inside or outside — whichever is closer
    /// to holding when the solve starts, so a solve never flips the
    /// configuration the designer drew.
    RoundRound { internal: bool },
}

/// One constraint, ready to evaluate.
struct Prepared<'a> {
    constraint: &'a Constraint,
    vars: Vec<usize>,
    tangent: Option<TangentMode>,
}

/// Everything needed to evaluate the objective, fixed for one solve.
struct Problem<'a> {
    sketch: &'a Sketch,
    layout: Layout,
    constraints: Vec<Prepared<'a>>,
    /// Characteristic length of the sketch; see the module docs.
    scale: f64,
    /// Extra residuals `weight * (point - target)` pulling points towards a
    /// cursor while dragging.
    drags: Vec<(PointId, P2, f64)>,
}

impl<'a> Problem<'a> {
    fn new(sketch: &'a Sketch) -> GeopResult<Self> {
        sketch.validate()?;
        let layout = Layout::new(sketch);
        let x0 = layout.read(sketch);

        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for p in sketch.points.values() {
            lo = [lo[0].min(p.x), lo[1].min(p.y)];
            hi = [hi[0].max(p.x), hi[1].max(p.y)];
        }
        let diagonal = crate::point::dist(lo, hi);
        let radii = sketch.curves.values().filter_map(|c| match c.kind {
            CurveKind::Circle { radius, .. } => Some(2.0 * radius.abs()),
            _ => None,
        });
        let scale = radii.fold(diagonal, f64::max);
        let scale = if scale.is_finite() && scale > 0.0 {
            scale
        } else {
            1.0
        };

        let mut constraints = Vec::new();
        for c in sketch.constraints.values() {
            use Constraint::*;
            let mut vars = Vec::new();
            let pt = |p: &PointId, vars: &mut Vec<usize>| {
                vars.extend([layout.point_var[p], layout.point_var[p] + 1])
            };
            match c {
                // Coincidence is built into the layout.
                Coincident { .. } => continue,
                Fix { point, .. } => pt(point, &mut vars),
                Distance { a, b, .. } | DistanceX { a, b, .. } | DistanceY { a, b, .. } => {
                    pt(a, &mut vars);
                    pt(b, &mut vars);
                }
                PointOnCurve { point, curve }
                | Midpoint { point, curve }
                | PointLineDistance {
                    point, line: curve, ..
                } => {
                    pt(point, &mut vars);
                    layout.curve_vars(sketch, *curve, &mut vars);
                }
                Symmetric { a, b, line } => {
                    pt(a, &mut vars);
                    pt(b, &mut vars);
                    layout.curve_vars(sketch, *line, &mut vars);
                }
                Horizontal { line: curve }
                | Vertical { line: curve }
                | Length { curve, .. }
                | Radius { curve, .. } => layout.curve_vars(sketch, *curve, &mut vars),
                Parallel { a, b }
                | Perpendicular { a, b }
                | Collinear { a, b }
                | Tangent { a, b }
                | Equal { a, b }
                | Concentric { a, b }
                | Angle { a, b, .. } => {
                    layout.curve_vars(sketch, *a, &mut vars);
                    layout.curve_vars(sketch, *b, &mut vars);
                }
            }
            vars.sort_unstable();
            vars.dedup();
            if vars.len() > MAX_LOCAL_VARS {
                return Err(GeopError::new(format!(
                    "constraint {c:?} depends on {} variables, more than the supported {MAX_LOCAL_VARS}",
                    vars.len()
                )));
            }
            let tangent = match c {
                Tangent { a, b } => Some(Self::tangent_mode(sketch, &layout, &x0, *a, *b)?),
                _ => None,
            };
            constraints.push(Prepared {
                constraint: c,
                vars,
                tangent,
            });
        }

        Ok(Problem {
            sketch,
            layout,
            constraints,
            scale,
            drags: Vec::new(),
        })
    }

    fn tangent_mode(
        sketch: &Sketch,
        layout: &Layout,
        x: &[f64],
        a: CurveId,
        b: CurveId,
    ) -> GeopResult<TangentMode> {
        if let Some((a_end, b_end)) = sketch.shared_endpoint(a, b)? {
            return Ok(TangentMode::Endpoint { a_end, b_end });
        }
        let is_line = |c: CurveId| matches!(sketch.curves[&c].kind, CurveKind::Line { .. });
        if is_line(a) {
            return Ok(TangentMode::LineRound { line: a, round: b });
        }
        if is_line(b) {
            return Ok(TangentMode::LineRound { line: b, round: a });
        }
        let geo = Geo::<S> {
            sketch,
            layout,
            x,
            seeded: &[],
        };
        let ((ca, ra), (cb, rb)) = (geo.round(a)?, geo.round(b)?);
        let d = ca.sub(cb).norm()?.to_f64();
        let (ra, rb) = (ra.to_f64(), rb.to_f64());
        Ok(TangentMode::RoundRound {
            internal: (d - (ra - rb).abs()).abs() < (d - (ra + rb)).abs(),
        })
    }

    fn residuals<T: Scalar>(&self, p: &Prepared, geo: &Geo<T>, out: &mut Vec<T>) -> GeopResult<()> {
        use Constraint::*;
        let scale = T::from_f64(self.scale);
        match *p.constraint {
            Coincident { .. } => {}
            Fix { point, x, y } => {
                let q = geo.point(point);
                out.extend([q.x.sub(T::from_f64(x)), q.y.sub(T::from_f64(y))]);
            }
            Distance { a, b, value } => out.push(
                geo.point(b)
                    .sub(geo.point(a))
                    .norm()?
                    .sub(T::from_f64(value)),
            ),
            DistanceX { a, b, value } => {
                out.push(geo.point(b).x.sub(geo.point(a).x).sub(T::from_f64(value)))
            }
            DistanceY { a, b, value } => {
                out.push(geo.point(b).y.sub(geo.point(a).y).sub(T::from_f64(value)))
            }
            PointOnCurve { point, curve } => {
                let q = geo.point(point);
                out.push(match &self.sketch.curves[&curve].kind {
                    CurveKind::Line { .. } => {
                        let (s, e) = geo.line(curve);
                        line_distance(s, e, q)?
                    }
                    CurveKind::Arc { .. } => geo.arc(curve).unwrap().circle_residual(q)?,
                    _ => {
                        let (c, r) = geo.round(curve)?;
                        q.sub(c).norm()?.sub(r)
                    }
                });
            }
            Midpoint { point, curve } => {
                let q = geo.point(point);
                let m = match geo.arc(curve) {
                    Some(a) => a.arc_mid()?,
                    None => {
                        let (s, e) = geo.line(curve);
                        s.add(e).scale(T::from_f64(0.5))
                    }
                };
                out.extend([q.x.sub(m.x), q.y.sub(m.y)]);
            }
            Symmetric { a, b, line } => {
                let (pa, pb) = (geo.point(a), geo.point(b));
                let (s, e) = geo.line(line);
                let mid = pa.add(pb).scale(T::from_f64(0.5));
                out.push(line_distance(s, e, mid)?);
                out.push(pb.sub(pa).dot(e.sub(s).unit()?));
            }
            PointLineDistance { point, line, value } => {
                let (s, e) = geo.line(line);
                out.push(
                    line_distance(s, e, geo.point(point))?
                        .abs()
                        .sub(T::from_f64(value)),
                );
            }
            Horizontal { line } => {
                let (s, e) = geo.line(line);
                out.push(e.y.sub(s.y));
            }
            Vertical { line } => {
                let (s, e) = geo.line(line);
                out.push(e.x.sub(s.x));
            }
            Parallel { a, b } | Perpendicular { a, b } | Angle { a, b, .. } => {
                let (sa, ea) = geo.line(a);
                let (sb, eb) = geo.line(b);
                let (ua, ub) = (ea.sub(sa).unit()?, eb.sub(sb).unit()?);
                let (cross, dot) = (ua.cross(ub), ua.dot(ub));
                // sin(angle(a, b) - target), in units of length.
                let r = match *p.constraint {
                    Parallel { .. } => cross,
                    Perpendicular { .. } => dot,
                    Angle { value, .. } => cross
                        .mul(T::from_f64(value.cos()))
                        .sub(dot.mul(T::from_f64(value.sin()))),
                    _ => unreachable!(),
                };
                out.push(r.mul(scale));
            }
            Collinear { a, b } => {
                let (sa, ea) = geo.line(a);
                let (sb, eb) = geo.line(b);
                out.push(line_distance(sa, ea, sb)?);
                out.push(line_distance(sa, ea, eb)?);
            }
            Tangent { a, b } => match p.tangent.unwrap() {
                TangentMode::Endpoint { a_end, b_end } => {
                    let (ta, tb) = (geo.end_tangent(a, a_end)?, geo.end_tangent(b, b_end)?);
                    out.push(ta.cross(tb).mul(scale));
                }
                TangentMode::LineRound { line, round } => {
                    let (s, e) = geo.line(line);
                    let (c, r) = geo.round(round)?;
                    out.push(line_distance(s, e, c)?.abs().sub(r));
                }
                TangentMode::RoundRound { internal } => {
                    let ((ca, ra), (cb, rb)) = (geo.round(a)?, geo.round(b)?);
                    let d = ca.sub(cb).norm()?;
                    out.push(if internal {
                        d.sub(ra.sub(rb).abs())
                    } else {
                        d.sub(ra.add(rb))
                    });
                }
            },
            Equal { a, b } => {
                let size = |c: CurveId| -> GeopResult<T> {
                    Ok(match &self.sketch.curves[&c].kind {
                        CurveKind::Line { .. } => {
                            let (s, e) = geo.line(c);
                            e.sub(s).norm()?
                        }
                        _ => geo.round(c)?.1,
                    })
                };
                out.push(size(a)?.sub(size(b)?));
            }
            Concentric { a, b } => {
                let (ca, cb) = (geo.round(a)?.0, geo.round(b)?.0);
                out.extend([ca.x.sub(cb.x), ca.y.sub(cb.y)]);
            }
            Length { curve, value } => {
                let length = match geo.arc(curve) {
                    Some(a) => a.length()?,
                    None => {
                        let (s, e) = geo.line(curve);
                        e.sub(s).norm()?
                    }
                };
                out.push(length.sub(T::from_f64(value)));
            }
            Radius { curve, value } => out.push(match geo.arc(curve) {
                // `2 R |sin θ| - L` rather than `L / (2 |sin θ|) - R`: the
                // same zero set, but finite for a nearly straight arc.
                Some(a) => T::from_f64(2.0 * value)
                    .mul(a.half.sin().abs())
                    .sub(a.chord_length()?),
                None => geo.round(curve)?.1.sub(T::from_f64(value)),
            }),
        }
        Ok(())
    }

    /// Objective `Σ r²` and its gradient. A residual that hits a degenerate
    /// division/sqrt (see the module docs) makes the whole point infeasible
    /// — `f = ∞` — exactly as a plain `f64` computing `inf`/`nan` there would
    /// already have made the line search back off.
    fn objective(&self, x: &[f64]) -> (f64, Vec<f64>) {
        let mut f = 0.0;
        let mut g = vec![0.0; x.len()];
        let mut rs = Vec::new();
        for p in &self.constraints {
            let seeded: Vec<(usize, Dual<S>)> = p
                .vars
                .iter()
                .enumerate()
                .map(|(slot, &i)| (i, Dual::var(S::from_f64(x[i]), slot)))
                .collect();
            let geo = Geo {
                sketch: self.sketch,
                layout: &self.layout,
                x,
                seeded: &seeded,
            };
            rs.clear();
            if self.residuals(p, &geo, &mut rs).is_err() {
                return (f64::INFINITY, vec![0.0; x.len()]);
            }
            for r in &rs {
                let rv = r.v.to_f64();
                f += rv * rv;
                for (slot, &i) in p.vars.iter().enumerate() {
                    g[i] += 2.0 * rv * r.d[slot].to_f64();
                }
            }
        }
        for &(point, target, weight) in &self.drags {
            let i = self.layout.point_var[&point];
            for k in 0..2 {
                let r = weight * (x[i + k] - target[k]);
                f += r * r;
                g[i + k] += 2.0 * weight * r;
            }
        }
        (f, g)
    }

    /// Every constraint residual (no drags), and its Jacobian row by row. A
    /// constraint whose residual hits a degenerate division/sqrt at `x` (see
    /// the module docs) contributes no rows — [`Problem::report`] catches
    /// the same failure independently and lists it as unsatisfied.
    fn jacobian(&self, x: &[f64]) -> (Vec<f64>, Vec<Vec<f64>>) {
        let mut values = Vec::new();
        let mut rows = Vec::new();
        let mut rs = Vec::new();
        for p in &self.constraints {
            let seeded: Vec<(usize, Dual<S>)> = p
                .vars
                .iter()
                .enumerate()
                .map(|(slot, &i)| (i, Dual::var(S::from_f64(x[i]), slot)))
                .collect();
            let geo = Geo {
                sketch: self.sketch,
                layout: &self.layout,
                x,
                seeded: &seeded,
            };
            rs.clear();
            if self.residuals(p, &geo, &mut rs).is_err() {
                continue;
            }
            for r in &rs {
                values.push(r.v.to_f64());
                let mut row = vec![0.0; x.len()];
                for (slot, &i) in p.vars.iter().enumerate() {
                    row[i] += r.d[slot].to_f64();
                }
                rows.push(row);
            }
        }
        (values, rows)
    }

    fn minimize(&self, x: Vec<f64>, max_iterations: usize) -> (Vec<f64>, usize) {
        let tol = RELATIVE_TOLERANCE * self.scale;
        let r = minimize(
            |x| self.objective(x),
            x,
            BfgsOptions {
                max_iterations,
                f_tolerance: (0.01 * tol).powi(2),
                g_tolerance: 0.0,
            },
        );
        (r.x, r.iterations)
    }
}

/// A constraint counts as satisfied once its residual is within this
/// fraction of the sketch's size.
const RELATIVE_TOLERANCE: f64 = 1e-9;

/// Newton steps polishing a solution before it is enclosed: each squares
/// the error, so a handful take BFGS's `RELATIVE_TOLERANCE` to rounding.
const POLISH_STEPS: usize = 6;

/// How many candidate boxes [`Problem::enclose`] tries, each twice as wide
/// as the last one needed: how hard it tries, never what a verified box
/// means.
const ENCLOSE_ATTEMPTS: usize = 24;

/// The outcome of a solve.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolveReport {
    /// Every constraint holds (to [`RELATIVE_TOLERANCE`] of the sketch size).
    pub converged: bool,
    /// Largest remaining constraint residual, in sketch units.
    pub max_residual: f64,
    pub iterations: usize,
    /// Remaining degrees of freedom: variables minus independent constraints.
    pub dof: usize,
    /// Per point: whether it can still move without violating a constraint.
    pub free_points: BTreeMap<PointId, bool>,
    /// Per curve: whether any of its points or its own sweep/radius can still
    /// change.
    pub free_curves: BTreeMap<CurveId, bool>,
    /// The constraints left unsatisfied (conflicting or unreachable), empty
    /// when `converged`.
    pub failed_constraints: Vec<ConstraintId>,
}

impl Sketch {
    /// The sketch's geometry as the kernel builds on it (see [`Enclosure`]):
    /// an enclosure of the exact solution of its constraints near the
    /// solved positions, each variable the constraints leave free exactly
    /// as drawn (see [`Problem::enclose`]).
    ///
    /// A sketch whose constraints are not met is no solution of them, and is
    /// built exactly as drawn: that is all there is to build. Fails if the
    /// solution cannot be proven — a sketch meeting its constraints only at
    /// a singular configuration.
    pub fn enclose<T: Scalar>(&self) -> GeopResult<Enclosure<T>> {
        let ctx = |e: GeopError| e.with_context("enclosing the sketch's solution");
        let problem = Problem::new(self).map_err(ctx)?;
        let x = problem.layout.read(self);
        if !problem.report(&x, 0).converged {
            return Ok(Enclosure::as_drawn(self));
        }
        let x = problem.enclose(&x).map_err(ctx)?;
        let to_t = |v: S| T::from_f64(v.lower().to_f64()).union(T::from_f64(v.upper().to_f64()));
        let layout = &problem.layout;
        Ok(Enclosure {
            points: layout
                .point_var
                .iter()
                .map(|(&p, &i)| (p, Vector2::from_array([to_t(x[i]), to_t(x[i + 1])])))
                .collect(),
            params: self
                .curves
                .iter()
                .filter_map(|(&id, c)| {
                    let v = x[*layout.curve_var.get(&id)?];
                    Some(match c.kind {
                        CurveKind::Arc { .. } => (id, to_t(v.mul(S::TWO))),
                        _ => (id, to_t(v.abs())),
                    })
                })
                .collect(),
        })
    }

    /// Move every point (and arc sweep, circle radius) so all constraints
    /// hold, changing the sketch as little as the constraints allow.
    ///
    /// The sketch is updated even if the solve does not converge, to the
    /// closest configuration found — the report says which constraints could
    /// not be met.
    pub fn solve(&mut self) -> GeopResult<SolveReport> {
        self.solve_with_drag(&[])
    }

    /// Like [`Sketch::solve`], while pulling each `(point, target)` towards
    /// its target as far as the constraints allow — interactive dragging.
    pub fn solve_with_drag(&mut self, drags: &[(PointId, P2)]) -> GeopResult<SolveReport> {
        let mut problem = Problem::new(self)?;
        let x0 = problem.layout.read(self);
        let mut iterations = 0;
        let mut x = x0;
        if !drags.is_empty() {
            // Pull softly first, then solve the constraints alone from there:
            // a dragged point follows the cursor exactly when it is free to,
            // and the constraints win wherever they disagree with it.
            problem.drags = drags.iter().map(|&(p, t)| (p, t, 0.1)).collect();
            let (x1, it) = problem.minimize(x, 200);
            problem.drags.clear();
            x = x1;
            iterations += it;
        }
        let (x, it) = problem.minimize(x, 2000);
        iterations += it;

        let report = problem.report(&x, iterations);
        let layout = problem.layout;
        layout.write(self, &x);
        Ok(report)
    }
}

impl Problem<'_> {
    /// Every constraint residual over the box `x`, and its gradient with
    /// respect to every variable: an enclosure of both, for every point of
    /// the box. Fails where a residual is undecidable somewhere in it.
    fn rows_over(&self, x: &[S]) -> GeopResult<Vec<(S, Vec<S>)>> {
        let mid: Vec<f64> = x.iter().map(|v| v.to_f64()).collect();
        let mut out = Vec::new();
        let mut rs = Vec::new();
        for p in &self.constraints {
            let seeded: Vec<(usize, Dual<S>)> = p
                .vars
                .iter()
                .enumerate()
                .map(|(slot, &i)| (i, Dual::var(x[i], slot)))
                .collect();
            let geo = Geo {
                sketch: self.sketch,
                layout: &self.layout,
                x: &mid,
                seeded: &seeded,
            };
            rs.clear();
            self.residuals(p, &geo, &mut rs)?;
            for r in &rs {
                let mut row = vec![S::ZERO; x.len()];
                for (slot, &i) in p.vars.iter().enumerate() {
                    row[i] = row[i].add(r.d[slot]);
                }
                out.push((r.v, row));
            }
        }
        Ok(out)
    }

    /// An enclosure of the exact solution near `x`, a solution of the
    /// constraints to the solver's tolerance.
    ///
    /// The constraints determine some variables and leave the rest free
    /// (see [`eliminate`]). The free ones are the designer's choice, and stay
    /// exactly as drawn. The determined ones are first polished by Newton's
    /// method on the independent constraints, then enclosed by the Krawczyk
    /// test: for a box `X` around the polished `x̃`, with `Y` any
    /// approximate inverse of the Jacobian at `x̃`,
    ///
    /// ```text
    /// K(X) = x̃ - Y f(x̃) + (I - Y J(X)) (X - x̃)
    /// ```
    ///
    /// and `K(X) ⊆ X` proves `X` holds a solution — in interval arithmetic,
    /// so the proof is rigorous. The box is widened until the test passes;
    /// its width is then how precisely the constraints pin the solution
    /// down, and `K(X) ∩ X` is returned. Fails if no box passes: a singular
    /// Jacobian — a tangency the constraints only just meet, say — leaves the
    /// solution unproven, and nothing narrower than that is honest.
    ///
    /// Redundant constraints (a rectangle's fourth side, say) are left out
    /// of the test: they hold wherever the independent ones do, when the
    /// sketch is consistent, which a converged solve says it is.
    fn enclose(&self, x: &[f64]) -> GeopResult<Vec<S>> {
        let sharp = |x: &[f64]| -> Vec<S> { x.iter().map(|&v| S::from_f64(v)).collect() };
        let mids = |rows: &[(S, Vec<S>)]| -> Vec<Vec<f64>> {
            rows.iter()
                .map(|(_, d)| d.iter().map(|v| v.to_f64()).collect())
                .collect()
        };
        let n = x.len();
        let rows = self.rows_over(&sharp(x))?;
        let pivots = eliminate(mids(&rows), n).pivots;
        if pivots.is_empty() {
            return Ok(sharp(x));
        }
        let (cols, picked): (Vec<usize>, Vec<usize>) = pivots.into_iter().unzip();
        let m = cols.len();
        let square = |rows: &[(S, Vec<S>)]| -> Vec<Vec<f64>> {
            picked
                .iter()
                .map(|&r| cols.iter().map(|&c| rows[r].1[c].to_f64()).collect())
                .collect()
        };
        let singular = || GeopError::new("the sketch's constraints are singular at its solution");

        // Newton, on the determined variables: every iterate is only a seed
        // for the next, and the last one only the box's center — a free
        // choice (see `AGENTS.md`).
        let mut xt = x.to_vec();
        for _ in 0..POLISH_STEPS {
            let rows = self.rows_over(&sharp(&xt))?;
            let y = inverse(&square(&rows)).ok_or_else(singular)?;
            for (k, &c) in cols.iter().enumerate() {
                let step: f64 = (0..m).map(|j| y[k][j] * rows[picked[j]].0.to_f64()).sum();
                xt[c] -= step;
            }
        }

        let rows = self.rows_over(&sharp(&xt))?;
        let y = inverse(&square(&rows)).ok_or_else(singular)?;
        let yf: Vec<S> = (0..m)
            .map(|k| {
                (0..m).fold(S::ZERO, |sum, j| {
                    sum.add(S::from_f64(y[k][j]).mul(rows[picked[j]].0))
                })
            })
            .collect();
        let center: Vec<f64> = cols.iter().map(|&c| xt[c]).collect();
        // Half-widths of the candidate box: at least what the Newton step
        // from `x̃` still reaches, and a rounding step either side.
        let mut radius: Vec<f64> = (0..m)
            .map(|k| {
                let reach = yf[k]
                    .lower()
                    .to_f64()
                    .abs()
                    .max(yf[k].upper().to_f64().abs());
                let ulp = center[k].next_up() - center[k];
                reach.max(ulp)
            })
            .collect();
        for _ in 0..ENCLOSE_ATTEMPTS {
            let candidate: Vec<S> = (0..m)
                .map(|k| {
                    let (c, r) = (center[k], 2.0 * radius[k]);
                    S::from_f64(c - r).union(S::from_f64(c + r))
                })
                .collect();
            let mut boxed = sharp(&xt);
            for (k, &c) in cols.iter().enumerate() {
                boxed[c] = candidate[k];
            }
            let over = self.rows_over(&boxed)?;
            let offset: Vec<S> = (0..m)
                .map(|k| candidate[k].sub(S::from_f64(center[k])))
                .collect();
            let krawczyk: Vec<S> = (0..m)
                .map(|k| {
                    let spread = (0..m).fold(S::ZERO, |sum, l| {
                        // Row `k` of `I - Y J(X)`, column `l`.
                        let yj = (0..m).fold(S::ZERO, |sum, j| {
                            sum.add(S::from_f64(y[k][j]).mul(over[picked[j]].1[cols[l]]))
                        });
                        let identity = if k == l { S::ONE } else { S::ZERO };
                        sum.add(identity.sub(yj).mul(offset[l]))
                    });
                    S::from_f64(center[k]).sub(yf[k]).add(spread)
                })
                .collect();
            if (0..m).all(|k| krawczyk[k].is_subset_of(candidate[k])) {
                for (k, &c) in cols.iter().enumerate() {
                    boxed[c] = krawczyk[k].intersect(candidate[k]);
                }
                return Ok(boxed);
            }
            for k in 0..m {
                let reach = krawczyk[k].sub(S::from_f64(center[k]));
                let reach = reach
                    .lower()
                    .to_f64()
                    .abs()
                    .max(reach.upper().to_f64().abs());
                radius[k] = radius[k].max(reach);
                if !radius[k].is_finite() {
                    return Err(singular());
                }
            }
        }
        Err(GeopError::new(format!(
            "could not enclose the sketch's solution: no box around it passed the Krawczyk test in {ENCLOSE_ATTEMPTS} attempts"
        )))
    }

    fn report(&self, x: &[f64], iterations: usize) -> SolveReport {
        let tol = RELATIVE_TOLERANCE * self.scale;
        let (values, rows) = self.jacobian(x);
        let max_residual = values.iter().fold(0.0f64, |m, r| m.max(r.abs()));

        let mut failed_constraints = Vec::new();
        let mut rs = Vec::new();
        let mut prepared = self.constraints.iter();
        for (&i, c) in &self.sketch.constraints {
            if matches!(c, Constraint::Coincident { .. }) {
                continue;
            }
            let p = prepared.next().unwrap();
            let geo = Geo::<S> {
                sketch: self.sketch,
                layout: &self.layout,
                x,
                seeded: &[],
            };
            rs.clear();
            if self.residuals(p, &geo, &mut rs).is_err() {
                failed_constraints.push(i);
                continue;
            }
            if rs.iter().any(|r| !r.is_finite() || r.to_f64().abs() > tol) {
                failed_constraints.push(i);
            }
        }

        let free_vars = free_variables(rows, self.layout.n);
        let dof = free_vars.1;
        let free_vars = free_vars.0;
        let free_points = self
            .layout
            .point_var
            .iter()
            .map(|(&p, &i)| (p, free_vars[i] || free_vars[i + 1]))
            .collect();
        let free_curves = self
            .sketch
            .curves
            .keys()
            .map(|&c| {
                let mut vars = Vec::new();
                self.layout.curve_vars(self.sketch, c, &mut vars);
                (c, vars.iter().any(|&i| free_vars[i]))
            })
            .collect();

        SolveReport {
            converged: failed_constraints.is_empty(),
            max_residual,
            iterations,
            dof,
            free_points,
            free_curves,
            failed_constraints,
        }
    }
}

/// The outcome of Gauss-Jordan elimination (see [`eliminate`]).
struct Elimination {
    /// The rows, reduced: row `k` has a one at `pivots[k].0` and zeros at
    /// every other pivot column.
    rows: Vec<Vec<f64>>,
    /// Per pivot: its column, and the index of the row it was among the
    /// rows eliminated.
    pivots: Vec<(usize, usize)>,
}

/// Gauss-Jordan elimination of `rows` (each over `n` variables) with partial
/// pivoting; a pivot counts as zero below a fixed fraction of the largest
/// entry. Which constraints are independent and which variables they
/// determine is a classification, not a claim: where it matters for
/// correctness, [`Problem::enclose`] verifies what it is used for.
fn eliminate(mut rows: Vec<Vec<f64>>, n: usize) -> Elimination {
    let largest = rows.iter().flatten().fold(0.0f64, |m, v| m.max(v.abs()));
    let zero = 1e-9 * largest.max(1e-300);
    let mut origin: Vec<usize> = (0..rows.len()).collect();
    let mut pivots = Vec::new();
    let mut r = 0;
    for col in 0..n {
        let Some(best) =
            (r..rows.len()).max_by(|&a, &b| rows[a][col].abs().total_cmp(&rows[b][col].abs()))
        else {
            break;
        };
        if rows[best][col].abs() <= zero {
            continue;
        }
        rows.swap(r, best);
        origin.swap(r, best);
        let pivot = rows[r][col];
        for v in &mut rows[r] {
            *v /= pivot;
        }
        for i in 0..rows.len() {
            if i != r && rows[i][col] != 0.0 {
                let factor = rows[i][col];
                let (pivot_row, row) = if i < r {
                    let (lo, hi) = rows.split_at_mut(r);
                    (&hi[0], &mut lo[i])
                } else {
                    let (lo, hi) = rows.split_at_mut(i);
                    (&lo[r], &mut hi[0])
                };
                for (v, p) in row.iter_mut().zip(pivot_row) {
                    *v -= factor * p;
                }
            }
        }
        pivots.push((col, origin[r]));
        r += 1;
    }
    Elimination { rows, pivots }
}

/// Which of `n` variables can move to first order without changing any
/// residual (those with a nonzero component in the Jacobian's null space),
/// and the null space's dimension. This only classifies entities for
/// display — the solve itself does not depend on it.
fn free_variables(rows: Vec<Vec<f64>>, n: usize) -> (Vec<bool>, usize) {
    let Elimination { rows, pivots } = eliminate(rows, n);
    // Null space basis: one vector per non-pivot column `f`, with `v_f = 1`
    // and `v_{pivots[i].0} = -rows[i][f]`.
    let mut free = vec![false; n];
    let is_pivot: Vec<bool> = (0..n).map(|c| pivots.iter().any(|p| p.0 == c)).collect();
    for f in (0..n).filter(|&c| !is_pivot[c]) {
        free[f] = true;
        for (i, &(pc, _)) in pivots.iter().enumerate() {
            if rows[i][f].abs() > 1e-7 {
                free[pc] = true;
            }
        }
    }
    (free, n - pivots.len())
}

/// The inverse of the square matrix `a`, by Gauss-Jordan elimination with
/// partial pivoting — `None` if it is singular.
fn inverse(a: &[Vec<f64>]) -> Option<Vec<Vec<f64>>> {
    let m = a.len();
    let mut rows: Vec<Vec<f64>> = a
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut row = row.clone();
            row.extend((0..m).map(|j| if i == j { 1.0 } else { 0.0 }));
            row
        })
        .collect();
    let Elimination { rows, pivots } = eliminate(std::mem::take(&mut rows), 2 * m);
    if pivots.len() < m || pivots.iter().any(|p| p.0 >= m) {
        return None;
    }
    Some(rows.into_iter().map(|row| row[m..].to_vec()).collect())
}
