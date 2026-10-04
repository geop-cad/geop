//! Solving a 3-D sketch with the solver every system of the kernel shares
//! ([`geop_core_solve`]), the way a planar sketch is solved (see
//! [`crate::solve`]): every constraint a [`Residual`] of the sketch's
//! variables, each a length, computed as a [`geop_core_math::dual::Dual`] of
//! the sketch's scalar so its gradient is exact and its value an enclosure.
//!
//! **Variables.** Each class of coincident points (see
//! [`Sketch3d::point_classes`]) is one `(x, y, z)`. A point on a line or on
//! a reference curve adds where along it — its parameter there — so lying
//! on it is three plain equations, `p = C(t)`, rather than a distance whose
//! square root has no slope at its solution. A point on an arc needs no
//! parameter: it lies in the arc's plane and as far from its center as its
//! ends.
//!
//! **Directions.** "Parallel to `d`" is two equations, not one: the cross
//! product of the two unit directions, measured along two vectors across
//! `d` (see [`across`]) — a cross product's length, like a distance, has no
//! slope where it vanishes.
//!
//! What is structural is no residual at all: coincidence (one class of
//! points), and a spline end's direction, which the spline is built with
//! (see [`Sketch3d::adopts`]).

use std::collections::BTreeMap;

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Ring, Scalar, as_f64},
    vector::Vector3,
};
use geop_core_solve::{Param, Phase, Pull, Residual, System, Value};
use serde::{Deserialize, Serialize};

use super::{
    Constraint3d, CurveKind3d, Enclosure3d, End, Sketch3d,
    geometry::{Arc3, across, length, unit},
};
use crate::{ConstraintId, CurveId, PointId};

/// The most variables one constraint depends on: a tangency between two
/// arcs, five points.
const MAX_LOCAL_VARS: usize = 16;

/// Samples along a reference curve to seed where on it a point is from,
/// and Newton steps polishing the nearest.
const SEED_SAMPLES: usize = 64;
const SEED_NEWTON_STEPS: usize = 8;

type Dual<S> = geop_core_math::dual::Dual<S, MAX_LOCAL_VARS>;

/// Maps the sketch's entities to solver variables.
struct Layout {
    /// The index of each point's `x` (its `y` and `z` follow).
    point_var: BTreeMap<PointId, usize>,
    /// The parameter of each [`Constraint3d::OnCurve`] on a line or a
    /// reference curve.
    param_var: BTreeMap<ConstraintId, usize>,
    /// Per variable: whether the solver may change it.
    free: Vec<bool>,
}

impl Layout {
    fn new<S: Scalar>(sketch: &Sketch3d<S>) -> Self {
        let class = sketch.point_classes();
        let mut free = Vec::new();
        let mut class_var = BTreeMap::new();
        for rep in class.values() {
            class_var.entry(*rep).or_insert_with(|| {
                free.extend([true; 3]);
                free.len() - 3
            });
        }
        let point_var: BTreeMap<PointId, usize> =
            class.iter().map(|(&p, rep)| (p, class_var[rep])).collect();
        for (id, p) in &sketch.points {
            if p.fixed {
                free[point_var[id]..point_var[id] + 3].fill(false);
            }
        }
        let mut param_var = BTreeMap::new();
        for (&id, c) in &sketch.constraints {
            if let Constraint3d::OnCurve { curve, .. } = c
                && matches!(
                    sketch.curves[curve].kind,
                    CurveKind3d::Line { .. } | CurveKind3d::Reference
                )
            {
                free.push(true);
                param_var.insert(id, free.len() - 1);
            }
        }
        Layout {
            point_var,
            param_var,
            free,
        }
    }

    /// Per variable, its index among the free ones.
    fn offsets(&self) -> Vec<Option<usize>> {
        let mut n = 0;
        self.free
            .iter()
            .map(|&free| {
                free.then(|| {
                    n += 1;
                    n - 1
                })
            })
            .collect()
    }

    /// The points' variables, read from the sketch: a class where its fixed
    /// point is, else where its representative (lowest id) is. Where things
    /// are is a free choice, sharp.
    fn read<S: Scalar>(&self, sketch: &Sketch3d<S>, x: &mut [S]) {
        let points = sketch.points.iter().rev();
        for (id, p) in points
            .clone()
            .filter(|(_, p)| !p.fixed)
            .chain(points.filter(|(_, p)| p.fixed))
        {
            let i = self.point_var[id];
            x[i..i + 3].copy_from_slice(&p.at.to_array());
        }
    }

    /// Writes the points back, all but the fixed ones.
    fn write<S: Scalar>(&self, sketch: &mut Sketch3d<S>, x: &[S]) {
        for (id, p) in sketch.points.iter_mut().filter(|(_, p)| !p.fixed) {
            let i = self.point_var[id];
            p.at = Vector3::from_array([x[i], x[i + 1], x[i + 2]]);
        }
    }

    fn point_vars(&self, p: PointId, out: &mut Vec<usize>) {
        let i = self.point_var[&p];
        out.extend([i, i + 1, i + 2]);
    }
}

/// One constraint, ready to evaluate.
struct Prepared<'a, S: Scalar> {
    id: ConstraintId,
    constraint: &'a Constraint3d<S>,
    vars: Vec<usize>,
    /// Two vectors across the direction a "parallel" is measured from (see
    /// [`across`]).
    across: Option<[Vector3<S>; 2]>,
    /// The reference curve a point lies on, in the residuals' scalar.
    reference: Option<NurbCurve3D<Dual<S>>>,
}

/// Everything a solve evaluates, fixed for one solve.
struct Problem<'a, S: Scalar> {
    sketch: &'a Sketch3d<S>,
    layout: Layout,
    constraints: Vec<Prepared<'a, S>>,
    /// The variables where the solve starts: the points as drawn, each
    /// parameter where its point is nearest its curve.
    start: Vec<S>,
    /// The sketch's characteristic length: the diagonal of its points' box,
    /// at least one.
    scale: S,
}

/// The point `p` of `x`.
fn point_of<T: Scalar>(layout: &Layout, x: &[T], p: PointId) -> Vector3<T> {
    let i = layout.point_var[&p];
    Vector3::from_array([x[i], x[i + 1], x[i + 2]])
}

/// The sketch's curve `c` as an arc, at the variables `x`; `None` for
/// another kind.
fn arc_of<S: Scalar, T: Scalar>(
    sketch: &Sketch3d<S>,
    layout: &Layout,
    x: &[T],
    c: CurveId,
) -> Option<Arc3<T>> {
    match sketch.curves[&c].kind {
        CurveKind3d::Arc {
            start,
            through,
            end,
        } => Some(Arc3 {
            s: point_of(layout, x, start),
            m: point_of(layout, x, through),
            e: point_of(layout, x, end),
        }),
        _ => None,
    }
}

/// The ends of the line `c`, at the variables `x`.
fn line_of<S: Scalar, T: Scalar>(
    sketch: &Sketch3d<S>,
    layout: &Layout,
    x: &[T],
    c: CurveId,
) -> (Vector3<T>, Vector3<T>) {
    match sketch.curves[&c].kind {
        CurveKind3d::Line { start, end } => (point_of(layout, x, start), point_of(layout, x, end)),
        ref k => unreachable!("validated to be a line: {k:?}"),
    }
}

/// The unit tangent of the line or arc `c` at its end `end`, in its own
/// direction, at the variables `x`.
fn tangent_of<S: Scalar, T: Scalar>(
    sketch: &Sketch3d<S>,
    layout: &Layout,
    x: &[T],
    c: CurveId,
    end: End,
) -> GeopResult<Vector3<T>> {
    match arc_of(sketch, layout, x, c) {
        Some(arc) => arc.tangent(end == End::End),
        None => {
            let (s, e) = line_of(sketch, layout, x, c);
            unit(&e.sub(&s))
        }
    }
}

impl<'a, S: Scalar> Problem<'a, S> {
    fn new(sketch: &'a Sketch3d<S>) -> GeopResult<Self> {
        sketch.validate()?;
        let layout = Layout::new(sketch);
        let mut start = vec![S::ZERO; layout.free.len()];
        layout.read(sketch, &mut start);

        let span = |k: usize| {
            let values = || sketch.points.values().map(move |p| p.at[k]);
            let hi = values().reduce(S::max).unwrap_or(S::ZERO);
            let lo = values().reduce(S::min).unwrap_or(S::ZERO);
            hi.sub(lo)
        };
        let diagonal = (0..3)
            .map(|k| span(k).mul(span(k)))
            .fold(S::ZERO, S::add)
            .sqrt()?;
        let scale = if diagonal.is_finite() && diagonal.definitely_greater(S::ONE) {
            diagonal
        } else {
            S::ONE
        };

        let mut constraints = Vec::new();
        for (&id, c) in &sketch.constraints {
            use Constraint3d::*;
            let mut vars = Vec::new();
            let mut across_ = None;
            let mut reference = None;
            let curve_vars = |c: CurveId, vars: &mut Vec<usize>| {
                for p in sketch.curves[&c].points() {
                    layout.point_vars(p, vars);
                }
            };
            match c {
                Coincident { a, b } if sketch.points[a].fixed && sketch.points[b].fixed => {}
                // Built into the layout.
                Coincident { .. } => continue,
                Coordinate { point, .. } => layout.point_vars(*point, &mut vars),
                Distance { a, b, .. } => {
                    layout.point_vars(*a, &mut vars);
                    layout.point_vars(*b, &mut vars);
                }
                Length { line: c, .. } | Radius { arc: c, .. } => curve_vars(*c, &mut vars),
                Parallel { a, b } => {
                    curve_vars(*a, &mut vars);
                    curve_vars(*b, &mut vars);
                    let (s, e) = line_of(sketch, &layout, &start, *a);
                    across_ = Some(across(&e.sub(&s))?);
                }
                ParallelTo { line, direction } => {
                    curve_vars(*line, &mut vars);
                    across_ = Some(across(direction)?);
                }
                // A spline is built along it.
                TangentTo { curve, .. }
                    if matches!(sketch.curves[curve].kind, CurveKind3d::Spline { .. }) =>
                {
                    continue;
                }
                TangentTo {
                    curve, direction, ..
                } => {
                    curve_vars(*curve, &mut vars);
                    across_ = Some(across(direction)?);
                }
                OnCurve { point, curve } => {
                    layout.point_vars(*point, &mut vars);
                    curve_vars(*curve, &mut vars);
                    if let Some(&t) = layout.param_var.get(&id) {
                        vars.push(t);
                        let p = point_of(&layout, &start, *point);
                        let seed = match &sketch.curves[curve].kind {
                            CurveKind3d::Line { .. } => {
                                let (s, e) = line_of(sketch, &layout, &start, *curve);
                                let d = e.sub(&s);
                                p.sub(&s).prod_dot(&d).div(d.norm_sq())?
                            }
                            _ => {
                                let curve = &sketch.references[curve];
                                reference = Some(NurbCurve::try_new(
                                    curve.degree,
                                    curve
                                        .control_points
                                        .iter()
                                        .map(|cp| cp.map(Dual::cst))
                                        .collect(),
                                    curve.knot_vector.iter().map(|&k| Dual::cst(k)).collect(),
                                )?);
                                nearest_parameter(curve, &p)?
                            }
                        };
                        // Where to start looking from: a free choice, sharp.
                        start[t] = S::from_f64(seed.to_f64());
                    }
                }
                Tangent { a, b } => {
                    if sketch.adopts(*a, *b)?.is_some() {
                        continue;
                    }
                    curve_vars(*a, &mut vars);
                    curve_vars(*b, &mut vars);
                    let (ea, _) = sketch.shared_end(*a, *b)?.expect("validated");
                    across_ = Some(across(&tangent_of(sketch, &layout, &start, *a, ea)?)?);
                }
            }
            vars.sort_unstable();
            vars.dedup();
            if vars.len() > MAX_LOCAL_VARS {
                return Err(GeopError::new(format!(
                    "constraint {id} depends on {} variables, more than the supported {MAX_LOCAL_VARS}",
                    vars.len()
                )));
            }
            constraints.push(Prepared {
                id,
                constraint: c,
                vars,
                across: across_,
                reference,
            });
        }
        Ok(Problem {
            sketch,
            layout,
            constraints,
            start,
            scale,
        })
    }

    /// The residuals of `p` at the variables `x` — only those it depends on
    /// are read.
    fn residuals(
        &self,
        p: &Prepared<'_, S>,
        x: &[Dual<S>],
        out: &mut Vec<Dual<S>>,
    ) -> GeopResult<()> {
        use Constraint3d::*;
        let c = Dual::cst;
        let scale = c(self.scale);
        let (sketch, layout) = (self.sketch, &self.layout);
        let point = |q: PointId| point_of(layout, x, q);
        // `a` parallel to `b`, two lengths (see the module docs).
        let parallel = |a: &Vector3<Dual<S>>, b: &Vector3<Dual<S>>, out: &mut Vec<Dual<S>>| {
            let cross = a.prod_cross(b);
            for u in p.across.as_ref().expect("prepared") {
                out.push(cross.prod_dot(&u.map(c)).mul(scale));
            }
        };
        match *p.constraint {
            Coincident { a, b } => {
                let d = sketch.points[&b].at.sub(&sketch.points[&a].at);
                out.extend(d.to_array().map(c));
            }
            Coordinate {
                point: q,
                axis,
                value,
            } => {
                out.push(point(q)[axis.index()].sub(c(value)));
            }
            Distance { a, b, value } => out.push(length(&point(b).sub(&point(a)))?.sub(c(value))),
            Length { line, value } => {
                let (s, e) = line_of(sketch, layout, x, line);
                out.push(length(&e.sub(&s))?.sub(c(value)));
            }
            Radius { arc, value } => {
                let arc = arc_of(sketch, layout, x, arc).expect("validated");
                out.push(arc.radius()?.sub(c(value)));
            }
            Parallel { a, b } => {
                let (sa, ea) = line_of(sketch, layout, x, a);
                let (sb, eb) = line_of(sketch, layout, x, b);
                parallel(&unit(&ea.sub(&sa))?, &unit(&eb.sub(&sb))?, out);
            }
            ParallelTo { line, direction } => {
                let (s, e) = line_of(sketch, layout, x, line);
                parallel(&unit(&e.sub(&s))?, &unit(&direction.map(c))?, out);
            }
            TangentTo {
                curve,
                end,
                direction,
            } => {
                let t = tangent_of(sketch, layout, x, curve, end)?;
                parallel(&t, &unit(&direction.map(c))?, out);
            }
            OnCurve { point: q, curve } => {
                let q = point(q);
                let on = match (&sketch.curves[&curve].kind, layout.param_var.get(&p.id)) {
                    (CurveKind3d::Arc { .. }, _) => {
                        let arc = arc_of(sketch, layout, x, curve).expect("an arc");
                        let center = arc.center()?;
                        out.push(q.sub(&arc.m).prod_dot(&arc.normal()?));
                        out.push(length(&q.sub(&center))?.sub(arc.radius()?));
                        return Ok(());
                    }
                    (CurveKind3d::Line { .. }, Some(&t)) => {
                        let (s, e) = line_of(sketch, layout, x, curve);
                        s.add(&e.sub(&s).prod_scalar(x[t]))
                    }
                    (_, Some(&t)) => p.reference.as_ref().expect("prepared").evaluate(x[t])?,
                    (k, None) => unreachable!("a point on {k:?} has a parameter"),
                };
                out.extend(q.sub(&on).to_array());
            }
            Tangent { a, b } => {
                let (ea, eb) = sketch.shared_end(a, b)?.expect("validated");
                let ta = tangent_of(sketch, layout, x, a, ea)?;
                let tb = tangent_of(sketch, layout, x, b, eb)?;
                parallel(&ta, &tb, out);
            }
        }
        Ok(())
    }

    fn residuals_of(&self) -> Vec<SketchResidual<'_, 'a, S>> {
        self.constraints
            .iter()
            .map(|prepared| SketchResidual {
                problem: self,
                prepared,
            })
            .collect()
    }

    fn system<'r>(
        &self,
        residuals: &'r [SketchResidual<'_, 'a, S>],
        x: &[S],
    ) -> System<'r, S, MAX_LOCAL_VARS> {
        System {
            params: x.iter().map(|&v| Param::Scalar(v)).collect(),
            free: self.layout.free.clone(),
            residuals: residuals
                .iter()
                .map(|r| r as &dyn Residual<S, MAX_LOCAL_VARS>)
                .collect(),
            scale: self.scale,
        }
    }

    /// How the sketch stands at the variables `system` has.
    fn report(
        &self,
        system: &System<'_, S, MAX_LOCAL_VARS>,
        iterations: usize,
        phases: Vec<Phase<S>>,
    ) -> GeopResult<Solve3dReport<S>> {
        let solved = system.report()?;
        let failed_constraints = solved
            .failed
            .iter()
            .map(|&i| self.constraints[i].id)
            .collect::<Vec<_>>();
        let (free_offsets, dof) = system.free_variables();
        let free_vars: Vec<bool> = self
            .layout
            .offsets()
            .iter()
            .map(|o| o.is_some_and(|o| free_offsets[o]))
            .collect();
        let free_points = self
            .layout
            .point_var
            .iter()
            .map(|(&p, &i)| (p, free_vars[i..i + 3].iter().any(|&f| f)))
            .collect();
        Ok(Solve3dReport {
            converged: failed_constraints.is_empty(),
            max_residual: solved.max_residual,
            iterations,
            phases,
            dof,
            free_points,
            failed_constraints,
        })
    }
}

/// The parameter of `curve` whose point is nearest `p`: the nearest of
/// evenly spaced samples, polished by Newton's method on `(C(t) - p) .
/// C'(t) = 0` — where a solve starts looking from, a free choice, worked
/// out in plain numbers. Polished, a point that lies on the curve is
/// found where it is, and a solve does not move it along the curve to
/// meet a parameter merely near its own.
fn nearest_parameter<S: Scalar>(curve: &NurbCurve3D<S>, p: &Vector3<S>) -> GeopResult<S> {
    let (t0, t1) = curve.domain();
    let (t0, t1) = (t0.to_f64(), t1.to_f64());
    let mut best = (f64::INFINITY, t0);
    for i in 0..=SEED_SAMPLES {
        let t = t0 + (t1 - t0) * i as f64 / SEED_SAMPLES as f64;
        let d = curve.evaluate(S::from_f64(t))?.sub(p).norm_sq().to_f64();
        if d < best.0 {
            best = (d, t);
        }
    }
    let plain = |v: Vector3<S>| v.to_array().map(|c| c.to_f64());
    let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| a[k] * b[k]).sum::<f64>();
    let mut t = best.1;
    for _ in 0..SEED_NEWTON_STEPS {
        let at = S::from_f64(t);
        let off = plain(curve.evaluate(at)?.sub(p));
        let (d1, d2) = (plain(curve.tangent(at)?), plain(curve.second_derivative(at)?));
        let slope = dot(d1, d1) + dot(off, d2);
        if !(slope > 0.0) {
            break;
        }
        t = (t - dot(off, d1) / slope).clamp(t0, t1);
    }
    Ok(S::from_f64(t))
}

/// The variables of a system of a sketch, as they are now.
fn values<S: Scalar>(system: &System<'_, S, MAX_LOCAL_VARS>) -> Vec<S> {
    system
        .params
        .iter()
        .map(|p| match p {
            Param::Scalar(v) => *v,
            Param::Pose { .. } => unreachable!("a sketch's variables are numbers"),
        })
        .collect()
}

/// One constraint as a residual of the variables it depends on.
struct SketchResidual<'p, 'a, S: Scalar> {
    problem: &'p Problem<'a, S>,
    prepared: &'p Prepared<'a, S>,
}

impl<S: Scalar> Residual<S, MAX_LOCAL_VARS> for SketchResidual<'_, '_, S> {
    fn params(&self) -> &[usize] {
        &self.prepared.vars
    }

    fn eval(&self, values: &[Value<Dual<S>>], out: &mut Vec<Dual<S>>) -> GeopResult<()> {
        // Only the variables it depends on are read: the rest are zero.
        let mut x = vec![Dual::cst(S::ZERO); self.problem.layout.free.len()];
        for (&i, v) in self.prepared.vars.iter().zip(values) {
            x[i] = v.scalar()?;
        }
        self.problem.residuals(self.prepared, &x, out)
    }
}

/// The outcome of solving a 3-D sketch.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Solve3dReport<S: Scalar> {
    /// Every constraint holds, to [`geop_core_solve::RELATIVE_TOLERANCE`] of
    /// the sketch's size.
    pub converged: bool,
    /// The largest remaining residual, a length: an upper bound.
    #[serde(with = "as_f64")]
    pub max_residual: S,
    pub iterations: usize,
    pub phases: Vec<Phase<S>>,
    /// Degrees of freedom left.
    pub dof: usize,
    /// Per point: whether it can still move.
    pub free_points: BTreeMap<PointId, bool>,
    /// The constraints that do not hold.
    pub failed_constraints: Vec<ConstraintId>,
}

impl<S: Scalar> Sketch3d<S> {
    /// Moves the points so every constraint holds, as little as they allow.
    /// The sketch is moved even if they cannot all hold, as near as they
    /// come — the report says which do not.
    pub fn solve(&mut self) -> GeopResult<Solve3dReport<S>> {
        self.solve_with_drag(&[])
    }

    /// Like [`Sketch3d::solve`], pulling each `(point, target)` towards its
    /// target as far as the constraints allow: dragging.
    pub fn solve_with_drag(
        &mut self,
        drags: &[(PointId, Vector3<S>)],
    ) -> GeopResult<Solve3dReport<S>> {
        let problem = Problem::new(self)?;
        let residuals = problem.residuals_of();
        let mut system = problem.system(&residuals, &problem.start);
        let pulls: Vec<Pull<S>> = drags
            .iter()
            .flat_map(|&(point, target)| {
                let i = problem.layout.point_var[&point];
                (0..3).map(move |k| Pull::Scalar {
                    param: i + k,
                    target: target[k],
                })
            })
            .collect();
        let solved = system.solve(&pulls)?;
        let report = problem.report(&system, solved.iterations, solved.phases)?;
        let x = values(&system);
        problem.layout.write(self, &x);
        Ok(report)
    }

    /// Which constraints hold where the points are now, and what can still
    /// move — without moving anything.
    pub fn check(&self) -> GeopResult<Solve3dReport<S>> {
        let problem = Problem::new(self)?;
        let residuals = problem.residuals_of();
        let system = problem.system(&residuals, &problem.start);
        problem.report(&system, 0, Vec::new())
    }

    /// The sketch's geometry as the kernel builds on it: an enclosure of
    /// the exact solution of its constraints near the solved points, every
    /// coordinate they leave free exactly as drawn (see
    /// [`System::enclose`]). A sketch whose constraints do not hold is built
    /// as drawn. Fails where the solution cannot be proven.
    pub fn enclose<T: Scalar>(&self) -> GeopResult<Enclosure3d<T>> {
        let ctx = |e: GeopError| e.with_context("enclosing the 3-D sketch's solution");
        let problem = Problem::new(self).map_err(ctx)?;
        let residuals = problem.residuals_of();
        let system = problem.system(&residuals, &problem.start);
        if !problem.report(&system, 0, Vec::new())?.converged {
            return Ok(Enclosure3d::as_drawn(self));
        }
        let enclosed = system.enclose().map_err(ctx)?;
        let layout = &problem.layout;
        let x: Vec<S> = problem
            .start
            .iter()
            .zip(layout.offsets())
            .map(|(&given, offset)| offset.map_or(given, |o| enclosed[o]))
            .collect();
        Ok(Enclosure3d {
            points: layout
                .point_var
                .iter()
                .map(|(&p, &i)| {
                    (
                        p,
                        Vector3::from_array([x[i], x[i + 1], x[i + 2]]).map(|c| c.cast()),
                    )
                })
                .collect(),
        })
    }
}
