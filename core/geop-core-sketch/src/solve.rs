//! Solving a sketch: every constraint contributes residuals that are zero
//! exactly when it holds — a [`Residual`] of the sketch's variables — and
//! the solver every system of the kernel shares ([`geop_core_math::solvers::system`])
//! drives the sum of their squares to zero, pulls a dragged point, and
//! encloses the exact solution.
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
//! **Honest enclosures.** Everything is computed in the sketch's own scalar
//! — any [`Scalar`] — each residual as a [`geop_core_math::dual::Dual`] of
//! it, so the `sin`/`sqrt`/`PI` a geometric formula can't help but need are
//! enclosed rather than quietly rounded away. A residual that becomes
//! genuinely undecidable (a division or square root at the edge of its
//! domain) makes that point infeasible.
use crate::{
    geometry::{Arc, line_distance},
    sketch::{Constraint, ConstraintId, CurveId, CurveKind, Enclosure, PointId, Sketch},
};
use geop_core_math::solvers::system::{self, Mobility, Param, Phase, Pull, Residual, Value};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Ring, Scalar, as_f64},
    vector::Vector2,
};

/// The most variables a single constraint may depend on. The largest
/// constraints (tangency or equality between two arcs) touch two arcs of
/// five variables each.
const MAX_LOCAL_VARS: usize = 10;

/// A residual with its gradient with respect to its constraint's own
/// variables.
type Dual<S> = geop_core_math::dual::Dual<S, MAX_LOCAL_VARS>;

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Maps sketch entities to solver variables.
struct Layout {
    /// Index of the `x` variable of each point (its `y` follows).
    point_var: BTreeMap<PointId, usize>,
    /// The arc's half sweep or the circle's radius, for every arc and circle.
    curve_var: BTreeMap<CurveId, usize>,
    /// Per variable: whether the solver may change it — not for a class of
    /// points one of which is fixed, nor for a fixed curve's own parameter.
    free: Vec<bool>,
}

impl Layout {
    fn new<S: Scalar>(sketch: &Sketch<S>) -> Self {
        let class = sketch.point_classes();
        let mut free = Vec::new();
        let mut class_var = BTreeMap::new();
        for rep in class.values() {
            class_var.entry(*rep).or_insert_with(|| {
                free.extend([true, true]);
                free.len() - 2
            });
        }
        let point_var: BTreeMap<PointId, usize> =
            class.iter().map(|(&p, rep)| (p, class_var[rep])).collect();
        for (id, p) in &sketch.points {
            if p.fixed {
                free[point_var[id]] = false;
                free[point_var[id] + 1] = false;
            }
        }
        let curve_var = sketch
            .curves
            .iter()
            .filter(|(_, c)| matches!(c.kind, CurveKind::Arc { .. } | CurveKind::Circle { .. }))
            .map(|(&id, c)| {
                free.push(!c.fixed);
                (id, free.len() - 1)
            })
            .collect();
        Layout {
            point_var,
            curve_var,
            free,
        }
    }

    fn n(&self) -> usize {
        self.free.len()
    }

    /// Per variable, its index among the free ones: how the solver's own
    /// vectors are laid out.
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

    /// The current variable values, read from the sketch. Where things are
    /// is the sketch's state — a free choice — so the variables are sharp,
    /// and so is what [`Layout::write`] writes back. A class of points with
    /// a fixed one is where that one is.
    fn read<S: Scalar>(&self, sketch: &Sketch<S>) -> GeopResult<Vec<S>> {
        let mut x = vec![S::ZERO; self.n()];
        // Iterate in reverse so a class takes its representative's (lowest
        // id) position — or, after, its fixed point's.
        let points = sketch.points.iter().rev();
        for (id, p) in points
            .clone()
            .filter(|(_, p)| !p.fixed)
            .chain(points.filter(|(_, p)| p.fixed))
        {
            x[self.point_var[id]] = p.x;
            x[self.point_var[id] + 1] = p.y;
        }
        for (id, c) in &sketch.curves {
            match c.kind {
                CurveKind::Arc { sweep, .. } => {
                    x[self.curve_var[id]] = sweep.div(S::TWO)?.sharpen()
                }
                CurveKind::Circle { radius, .. } => x[self.curve_var[id]] = radius,
                _ => {}
            }
        }
        Ok(x)
    }

    /// Write variable values back into the sketch — all but what is fixed,
    /// which stays as given.
    fn write<S: Scalar>(&self, sketch: &mut Sketch<S>, x: &[S]) {
        for (id, p) in sketch.points.iter_mut().filter(|(_, p)| !p.fixed) {
            p.x = x[self.point_var[id]];
            p.y = x[self.point_var[id] + 1];
        }
        for (id, c) in sketch.curves.iter_mut().filter(|(_, c)| !c.fixed) {
            match &mut c.kind {
                CurveKind::Arc { sweep, .. } => {
                    *sweep = S::TWO.mul(x[self.curve_var[id]]).sharpen()
                }
                CurveKind::Circle { radius, .. } => *radius = x[self.curve_var[id]].abs().sharpen(),
                _ => {}
            }
        }
    }

    /// The variables a curve depends on.
    fn curve_vars<S: Scalar>(&self, sketch: &Sketch<S>, c: CurveId, out: &mut Vec<usize>) {
        for p in sketch.curves[&c].points() {
            out.extend([self.point_var[&p], self.point_var[&p] + 1]);
        }
        out.extend(self.curve_var.get(&c));
    }
}

/// Read-only view of the variables for evaluating one constraint, with that
/// constraint's own variables `seeded` as [`Scalar`] values.
struct Geo<'a, S: Scalar, T> {
    sketch: &'a Sketch<S>,
    layout: &'a Layout,
    seeded: &'a [(usize, T)],
}

impl<S: Scalar, T: Scalar> Geo<'_, S, T> {
    fn var(&self, i: usize) -> T {
        self.seeded
            .iter()
            .find(|(j, _)| *j == i)
            .map(|(_, t)| *t)
            .expect("every variable a constraint reads is seeded")
    }
    fn point(&self, p: PointId) -> Vector2<T> {
        let i = self.layout.point_var[&p];
        Vector2::from_array([self.var(i), self.var(i + 1)])
    }
    fn line(&self, c: CurveId) -> (Vector2<T>, Vector2<T>) {
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
    fn round(&self, c: CurveId) -> GeopResult<(Vector2<T>, T)> {
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
    fn end_tangent(&self, c: CurveId, at_end: bool) -> GeopResult<Vector2<T>> {
        Ok(match &self.sketch.curves[&c].kind {
            CurveKind::Line { .. } => {
                let (s, e) = self.line(c);
                e.sub(&s).normalize()?
            }
            CurveKind::Arc { .. } => {
                let a = self.arc(c).unwrap();
                if at_end {
                    a.tangent_end()?
                } else {
                    a.tangent_start()?
                }
            }
            CurveKind::Spline { control_points, .. } => {
                let n = control_points.len();
                let (p, q) = if at_end {
                    (control_points[n - 2], control_points[n - 1])
                } else {
                    (control_points[0], control_points[1])
                };
                self.point(q).sub(&self.point(p)).normalize()?
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
struct Prepared<'a, S: Scalar> {
    id: ConstraintId,
    constraint: &'a Constraint<S>,
    vars: Vec<usize>,
    tangent: Option<TangentMode>,
}

/// Whether `a` and `b` are both fixed points: then a coincidence between
/// them is no merging of variables — neither can move — but a fact about
/// where they were given, which holds or does not.
fn both_fixed<S: Scalar>(sketch: &Sketch<S>, a: PointId, b: PointId) -> bool {
    sketch.points[&a].fixed && sketch.points[&b].fixed
}

/// Everything needed to evaluate the objective, fixed for one solve.
struct Problem<'a, S: Scalar> {
    sketch: &'a Sketch<S>,
    layout: Layout,
    constraints: Vec<Prepared<'a, S>>,
    /// Characteristic length of the sketch; see the module docs.
    scale: S,
}

impl<'a, S: Scalar> Problem<'a, S> {
    fn new(sketch: &'a Sketch<S>) -> GeopResult<Self> {
        sketch.validate()?;
        let layout = Layout::new(sketch);
        let x0 = layout.read(sketch)?;

        // The diagonal of the points' bounding box, or a circle's diameter
        // if that is larger — one, where the sketch has no size at all.
        let span = |k: usize| {
            let values = || sketch.points.values().map(move |p| p.xy()[k]);
            let hi = values().reduce(S::max).unwrap_or(S::ZERO);
            let lo = values().reduce(S::min).unwrap_or(S::ZERO);
            hi.sub(lo)
        };
        let diagonal = span(0).mul(span(0)).add(span(1).mul(span(1))).sqrt()?;
        let scale = sketch
            .curves
            .values()
            .filter_map(|c| match c.kind {
                CurveKind::Circle { radius, .. } => Some(S::TWO.mul(radius.abs())),
                _ => None,
            })
            .fold(diagonal, S::max);
        let scale = if scale.is_finite() && scale.definitely_greater(S::ZERO) {
            scale
        } else {
            S::ONE
        };

        let mut constraints = Vec::new();
        for (&id, c) in &sketch.constraints {
            use Constraint::*;
            let mut vars = Vec::new();
            let pt = |p: &PointId, vars: &mut Vec<usize>| {
                vars.extend([layout.point_var[p], layout.point_var[p] + 1])
            };
            match c {
                // Coincidence of two fixed points is a fact about where
                // they are, checked as given.
                Coincident { a, b } if both_fixed(sketch, *a, *b) => {}
                // Any other coincidence is built into the layout.
                Coincident { .. } => continue,
                Fix { point, .. } => pt(point, &mut vars),
                Distance { a, b, .. } | DistanceX { a, b, .. } | DistanceY { a, b, .. } => {
                    pt(a, &mut vars);
                    pt(b, &mut vars);
                }
                PointOnCurve { point, curve }
                | Midpoint { point, curve }
                | Center { point, curve }
                | PointLineDistance {
                    point, line: curve, ..
                } => {
                    pt(point, &mut vars);
                    layout.curve_vars(sketch, *curve, &mut vars);
                }
                Symmetric { a, b, line } | Moved { a, b, by: line } => {
                    pt(a, &mut vars);
                    pt(b, &mut vars);
                    layout.curve_vars(sketch, *line, &mut vars);
                }
                // Only the two sweeps.
                EqualSweep { a, b } => vars.extend([layout.curve_var[a], layout.curve_var[b]]),
                Horizontal { line: curve }
                | Vertical { line: curve }
                | Length { curve, .. }
                | Radius { curve, .. }
                | Diameter { curve, .. } => layout.curve_vars(sketch, *curve, &mut vars),
                Parallel { a, b }
                | Perpendicular { a, b }
                | Collinear { a, b }
                | Tangent { a, b }
                | Equal { a, b }
                | Concentric { a, b }
                | Offset { a, b, .. }
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
                id,
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
        })
    }

    fn tangent_mode(
        sketch: &Sketch<S>,
        layout: &Layout,
        x: &[S],
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
        let seeded: Vec<(usize, S)> = x.iter().copied().enumerate().collect();
        let geo = Geo {
            sketch,
            layout,
            seeded: &seeded,
        };
        let ((ca, ra), (cb, rb)) = (geo.round(a)?, geo.round(b)?);
        let d = ca.sub(&cb).try_norm()?;
        let internal = d.sub(ra.sub(rb).abs()).abs();
        let external = d.sub(ra.add(rb)).abs();
        Ok(TangentMode::RoundRound {
            internal: internal.definitely_less(external),
        })
    }

    fn residuals(
        &self,
        p: &Prepared<'_, S>,
        geo: &Geo<'_, S, Dual<S>>,
        out: &mut Vec<Dual<S>>,
    ) -> GeopResult<()> {
        use Constraint::*;
        let c = Dual::cst;
        let scale = c(self.scale);
        let half = c(S::ONE.div(S::TWO)?);
        match *p.constraint {
            Coincident { a, b } => {
                let (pa, pb) = (&self.sketch.points[&a], &self.sketch.points[&b]);
                out.extend([c(pb.x.sub(pa.x)), c(pb.y.sub(pa.y))]);
            }
            Fix { point, x, y } => {
                let q = geo.point(point);
                out.extend([q[0].sub(c(x)), q[1].sub(c(y))]);
            }
            Distance { a, b, value } => {
                out.push(geo.point(b).sub(&geo.point(a)).try_norm()?.sub(c(value)))
            }
            DistanceX { a, b, value } => {
                out.push(geo.point(b)[0].sub(geo.point(a)[0]).sub(c(value)))
            }
            DistanceY { a, b, value } => {
                out.push(geo.point(b)[1].sub(geo.point(a)[1]).sub(c(value)))
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
                        q.sub(&c).try_norm()?.sub(r)
                    }
                });
            }
            Midpoint { point, curve } => {
                let q = geo.point(point);
                let m = match geo.arc(curve) {
                    Some(a) => a.arc_mid()?,
                    None => {
                        let (s, e) = geo.line(curve);
                        s.add(&e).prod_scalar(half)
                    }
                };
                out.extend([q[0].sub(m[0]), q[1].sub(m[1])]);
            }
            Center { point, curve } => {
                let (q, center) = (geo.point(point), geo.round(curve)?.0);
                out.extend([q[0].sub(center[0]), q[1].sub(center[1])]);
            }
            Symmetric { a, b, line } => {
                let (pa, pb) = (geo.point(a), geo.point(b));
                let (s, e) = geo.line(line);
                let mid = pa.add(&pb).prod_scalar(half);
                out.push(line_distance(s, e, mid)?);
                out.push(pb.sub(&pa).prod_dot(&e.sub(&s).normalize()?));
            }
            Moved { a, b, by } => {
                // The motion carrying `by`'s start `s` onto its end `e`:
                // `q ↦ e + R(q - s)`, `R` the turn by its sweep — none
                // along a line. Smooth through a straight arc, where it is
                // the shift along the chord.
                let from = geo.point(a);
                let image = match geo.arc(by) {
                    Some(arc) => {
                        let turn = arc.half.mul(c(S::TWO));
                        arc.e.add(&from.sub(&arc.s).rotate(turn.cos(), turn.sin()))
                    }
                    None => {
                        let (s, e) = geo.line(by);
                        e.add(&from.sub(&s))
                    }
                };
                let to = geo.point(b);
                out.extend([to[0].sub(image[0]), to[1].sub(image[1])]);
            }
            EqualSweep { a, b } => {
                let half = |k: CurveId| geo.var(self.layout.curve_var[&k]);
                out.push(half(b).sub(half(a)).mul(scale));
            }
            Offset { a, b, value } => match self.sketch.curves[&a].kind {
                CurveKind::Line { .. } => {
                    // Both ends of `b` the same distance from `a`'s line:
                    // parallel, and that distance `value`.
                    let (sa, ea) = geo.line(a);
                    let (sb, eb) = geo.line(b);
                    let (ds, de) = (line_distance(sa, ea, sb)?, line_distance(sa, ea, eb)?);
                    out.push(ds.sub(de));
                    out.push(ds.abs().sub(c(value)));
                }
                _ => {
                    let ((ca, ra), (cb, rb)) = (geo.round(a)?, geo.round(b)?);
                    out.extend([ca[0].sub(cb[0]), ca[1].sub(cb[1])]);
                    out.push(rb.sub(ra).abs().sub(c(value)));
                }
            },
            PointLineDistance { point, line, value } => {
                let (s, e) = geo.line(line);
                out.push(line_distance(s, e, geo.point(point))?.abs().sub(c(value)));
            }
            Horizontal { line } => {
                let (s, e) = geo.line(line);
                out.push(e[1].sub(s[1]));
            }
            Vertical { line } => {
                let (s, e) = geo.line(line);
                out.push(e[0].sub(s[0]));
            }
            Parallel { a, b } | Perpendicular { a, b } | Angle { a, b, .. } => {
                let (sa, ea) = geo.line(a);
                let (sb, eb) = geo.line(b);
                let (ua, ub) = (ea.sub(&sa).normalize()?, eb.sub(&sb).normalize()?);
                let (cross, dot) = (ua.prod_cross(&ub), ua.prod_dot(&ub));
                // sin(angle(a, b) - target), in units of length.
                let r = match *p.constraint {
                    Parallel { .. } => cross,
                    Perpendicular { .. } => dot,
                    Angle { value, .. } => cross.mul(c(value.cos())).sub(dot.mul(c(value.sin()))),
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
                    out.push(ta.prod_cross(&tb).mul(scale));
                }
                TangentMode::LineRound { line, round } => {
                    let (s, e) = geo.line(line);
                    let (c, r) = geo.round(round)?;
                    out.push(line_distance(s, e, c)?.abs().sub(r));
                }
                TangentMode::RoundRound { internal } => {
                    let ((ca, ra), (cb, rb)) = (geo.round(a)?, geo.round(b)?);
                    let d = ca.sub(&cb).try_norm()?;
                    out.push(if internal {
                        d.sub(ra.sub(rb).abs())
                    } else {
                        d.sub(ra.add(rb))
                    });
                }
            },
            Equal { a, b } => {
                let size = |c: CurveId| -> GeopResult<Dual<S>> {
                    Ok(match &self.sketch.curves[&c].kind {
                        CurveKind::Line { .. } => {
                            let (s, e) = geo.line(c);
                            e.sub(&s).try_norm()?
                        }
                        _ => geo.round(c)?.1,
                    })
                };
                out.push(size(a)?.sub(size(b)?));
            }
            Concentric { a, b } => {
                let (ca, cb) = (geo.round(a)?.0, geo.round(b)?.0);
                out.extend([ca[0].sub(cb[0]), ca[1].sub(cb[1])]);
            }
            Length { curve, value } => {
                let length = match geo.arc(curve) {
                    Some(a) => a.length()?,
                    None => {
                        let (s, e) = geo.line(curve);
                        e.sub(&s).try_norm()?
                    }
                };
                out.push(length.sub(c(value)));
            }
            Radius { curve, value } => out.push(match geo.arc(curve) {
                // `2 R |sin θ| - L` rather than `L / (2 |sin θ|) - R`: the
                // same zero set, but finite for a nearly straight arc.
                Some(a) => c(S::TWO.mul(value))
                    .mul(a.half.sin().abs())
                    .sub(a.chord_length()?),
                None => geo.round(curve)?.1.sub(c(value)),
            }),
            Diameter { curve, value } => out.push(match geo.arc(curve) {
                // As for the radius: `D |sin θ| - L`.
                Some(a) => c(value).mul(a.half.sin().abs()).sub(a.chord_length()?),
                None => geo.round(curve)?.1.sub(c(value.div(S::TWO)?)),
            }),
        }
        Ok(())
    }

    /// Every constraint, as a residual of the sketch's variables.
    fn residuals_of(&self) -> Vec<SketchResidual<'_, 'a, S>> {
        self.constraints
            .iter()
            .map(|prepared| SketchResidual {
                problem: self,
                prepared,
            })
            .collect()
    }

    /// What a solve may do with each variable: change it as far as the
    /// constraints need if it is free, else not at all.
    fn mobility(&self) -> Vec<Mobility> {
        self.layout
            .free
            .iter()
            .map(|&f| if f { Mobility::Held } else { Mobility::Fixed })
            .collect()
    }
}

/// The variables `x` as the solver's parameters.
pub(crate) fn parameters<S: Scalar>(x: &[S]) -> Vec<Param<S>> {
    x.iter().map(|&v| Param::Scalar(v)).collect()
}

/// The residuals as the solver is given them.
pub(crate) fn solver_residuals<'r, S: Scalar, const N: usize, R: Residual<S, N>>(
    residuals: &'r [R],
) -> Vec<&'r dyn Residual<S, N>> {
    residuals.iter().map(|r| r as &dyn Residual<S, N>).collect()
}

/// The variables of a sketch, as the parameters are now.
pub(crate) fn values<S: Scalar>(params: &[Param<S>]) -> Vec<S> {
    params
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
        let seeded = self
            .prepared
            .vars
            .iter()
            .zip(values)
            .map(|(&i, v)| Ok((i, v.scalar()?)))
            .collect::<GeopResult<Vec<_>>>()?;
        let geo = Geo {
            sketch: self.problem.sketch,
            layout: &self.problem.layout,
            seeded: &seeded,
        };
        self.problem.residuals(self.prepared, &geo, out)
    }
}

/// The outcome of a solve.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct SolveReport<S: Scalar> {
    /// Every constraint holds (to [`geop_core_math::solvers::system::RELATIVE_TOLERANCE`] of
    /// the sketch size).
    pub converged: bool,
    /// Largest remaining constraint residual, in sketch units: an upper
    /// bound.
    #[serde(with = "as_f64")]
    pub max_residual: S,
    pub iterations: usize,
    /// Each minimization of the solve.
    pub phases: Vec<Phase<S>>,
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

impl<S: Scalar> Sketch<S> {
    /// The sketch's geometry as the kernel builds on it (see [`Enclosure`]):
    /// an enclosure of the exact solution of its constraints near the
    /// solved positions, each variable the constraints leave free exactly
    /// as drawn (see [`system::enclose`]).
    ///
    /// A sketch whose constraints are not met is no solution of them, and is
    /// built exactly as drawn: that is all there is to build. Fails if the
    /// solution cannot be proven — a sketch meeting its constraints only at
    /// a singular configuration.
    pub fn enclose<T: Scalar>(&self) -> GeopResult<Enclosure<T>> {
        let ctx = |e: GeopError| e.with_context("enclosing the sketch's solution");
        let problem = Problem::new(self).map_err(ctx)?;
        let residuals = problem.residuals_of();
        let params = parameters(&problem.layout.read(self)?);
        let solver = solver_residuals(&residuals);
        if !problem.report(&params, &solver, 0, Vec::new())?.converged {
            return Ok(Enclosure::as_drawn(self));
        }
        let enclosed = system::enclose(&params, &problem.mobility(), &solver, problem.scale)
            .map_err(|e| {
                e.named(|i| {
                    let c = &problem.constraints[i];
                    format!("{} {:?}", c.id, c.constraint)
                })
            })
            .map_err(ctx)?;
        let layout = &problem.layout;
        // What is fixed is as given; the rest as enclosed.
        let x: Vec<S> = values(&params)
            .into_iter()
            .zip(layout.offsets())
            .map(|(given, offset)| offset.map_or(given, |o| enclosed[o]))
            .collect();
        let to_t = |v: S| -> T { v.cast() };
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
    pub fn solve(&mut self) -> GeopResult<SolveReport<S>> {
        self.solve_with_drag(&[])
    }

    /// Like [`Sketch::solve`], while pulling each `(point, target)` towards
    /// its target as far as the constraints allow — interactive dragging.
    pub fn solve_with_drag(
        &mut self,
        drags: &[(PointId, Vector2<S>)],
    ) -> GeopResult<SolveReport<S>> {
        let problem = Problem::new(self)?;
        let residuals = problem.residuals_of();
        let start = parameters(&problem.layout.read(self)?);
        let solver = solver_residuals(&residuals);
        // A dragged point is pulled towards the cursor among the
        // configurations that meet the constraints: it follows the cursor
        // exactly where it is free to.
        let pulls: Vec<Pull<S>> = drags
            .iter()
            .flat_map(|&(point, target)| {
                let i = problem.layout.point_var[&point];
                (0..2).map(move |k| Pull::Scalar {
                    param: i + k,
                    target: target[k],
                })
            })
            .collect();
        let solved = system::solve(&start, &problem.mobility(), &solver, problem.scale, &pulls)?;
        let report = problem.report(
            &solved.params,
            &solver,
            solved.report.iterations,
            solved.report.phases,
        )?;
        // Where the solve put the points is the sketch's state, and a free
        // choice: sharp.
        let x: Vec<S> = values(&solved.params)
            .into_iter()
            .map(Scalar::sharpen)
            .collect();
        problem.layout.write(self, &x);
        Ok(report)
    }
}

impl<S: Scalar> Problem<'_, S> {
    /// How the sketch stands at the variables `params` are: which
    /// constraints hold, and what can still move.
    fn report(
        &self,
        params: &[Param<S>],
        residuals: &[&dyn Residual<S, MAX_LOCAL_VARS>],
        iterations: usize,
        phases: Vec<Phase<S>>,
    ) -> GeopResult<SolveReport<S>> {
        let solved = system::report(params, &self.mobility(), residuals, self.scale)?;
        let failed_constraints = solved
            .failed
            .iter()
            .map(|&i| self.constraints[i].id)
            .collect::<Vec<_>>();
        let (free_offsets, dof) =
            system::free_variables(params, &self.mobility(), residuals, self.scale);
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
        Ok(SolveReport {
            converged: failed_constraints.is_empty(),
            max_residual: solved.max_residual,
            iterations,
            phases,
            dof,
            free_points,
            free_curves,
            failed_constraints,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use geop_core_math::scalars::ScalInF64;

    use super::*;

    type S = ScalInF64;

    fn n(x: f64) -> S {
        S::from_f64(x)
    }

    /// At points drawn exactly, every residual and every slope of it is
    /// known to rounding: the solver takes a step only where the merit
    /// definitely drops, and a residual wider than its arithmetic needs
    /// would hide every step.
    #[test]
    fn residuals_are_sharp_where_the_points_are() {
        let mut s = Sketch::<S>::new();
        let a = s.add_point(n(-1.0), n(0.1));
        let b = s.add_point(n(1.2), n(-0.1));
        let c = s.add_point(n(0.1), n(1.5));
        let base = s.add_line(a, b);
        let left = s.add_line(c, a);
        let right = s.add_line(c, b);
        let m = s.add_point(n(0.0), n(0.0));
        let axis_top = s.add_point(n(0.0), n(2.0));
        let axis = s.add_line(m, axis_top);
        s.constrain(Constraint::Midpoint {
            point: m,
            curve: base,
        });
        s.constrain(Constraint::Symmetric { a, b, line: axis });
        s.constrain(Constraint::Equal { a: left, b: right });
        s.constrain(Constraint::Angle {
            a: left,
            b: right,
            value: n(PI / 3.0),
        });
        let problem = Problem::new(&s).unwrap();
        let residuals = problem.residuals_of();
        let params = parameters(&problem.layout.read(&s).unwrap());
        let solver = solver_residuals(&residuals);
        let n = params.len();
        let e = system::evaluate(
            &params,
            &problem.mobility(),
            &solver,
            problem.scale,
            &vec![S::ZERO; n],
            &[],
            false,
        )
        .unwrap();
        for (k, (v, row)) in e.sum.values.iter().zip(&e.sum.jacobian).enumerate() {
            for x in std::iter::once(v).chain(row) {
                let width = x.width().to_f64();
                assert!(
                    width <= 1e-12 * (1.0 + x.to_f64().abs()),
                    "residual {k}: {x:?} in {v:?}, {row:?}"
                );
            }
        }
    }

    /// A rectangle drawn a little crooked, its constraints saying it twice
    /// over: every corner square, and each pair of opposite sides parallel
    /// too. Its first corner held at the origin, its sides `2` and `1`
    /// long, and its bottom horizontal if `level`. Its sides.
    fn rectangle_said_twice(s: &mut Sketch<S>, level: bool) -> Vec<CurveId> {
        let p = [[0.0, 0.0], [2.05, 0.1], [1.9, 1.07], [-0.08, 0.95]]
            .map(|q| s.add_point(n(q[0]), n(q[1])));
        let l: Vec<CurveId> = (0..4).map(|i| s.add_line(p[i], p[(i + 1) % 4])).collect();
        s.constrain(Constraint::Fix {
            point: p[0],
            x: n(0.0),
            y: n(0.0),
        });
        for i in 0..4 {
            s.constrain(Constraint::Perpendicular {
                a: l[i],
                b: l[(i + 1) % 4],
            });
        }
        for i in 0..2 {
            s.constrain(Constraint::Parallel {
                a: l[i],
                b: l[i + 2],
            });
        }
        if level {
            s.constrain(Constraint::Horizontal { line: l[0] });
        }
        for (i, value) in [(0, 2.0), (1, 1.0)] {
            s.constrain(Constraint::Length {
                curve: l[i],
                value: n(value),
            });
        }
        l
    }

    /// The far corner of a rectangle as enclosed: where its second side
    /// ends.
    fn far_corner(s: &Sketch<S>, sides: &[CurveId], e: &Enclosure<S>) -> Vector2<S> {
        let (_, end) = s.curves[&sides[1]].endpoints().unwrap();
        e.points[&end]
    }

    /// Four right angles where three would do, and parallels besides: the
    /// redundant constraints hold wherever the others do, and the solution
    /// is proven — level, every coordinate determined; turned, the turn
    /// left as drawn.
    #[test]
    fn a_rectangle_said_twice_is_proven() {
        let mut s = Sketch::<S>::new();
        let sides = rectangle_said_twice(&mut s, true);
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        let e = s.enclose::<S>().unwrap();
        let corner = far_corner(&s, &sides, &e);
        assert!(
            corner[0].could_be_equal(n(2.0)) && corner[1].could_be_equal(n(1.0)),
            "{corner:?}"
        );
        assert!(corner[0].width().to_f64() < 1e-12, "{corner:?}");

        let mut s = Sketch::<S>::new();
        let sides = rectangle_said_twice(&mut s, false);
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 1, "{report:?}");
        let e = s.enclose::<S>().unwrap();
        let corner = far_corner(&s, &sides, &e);
        let diagonal = corner[0].mul(corner[0]).add(corner[1].mul(corner[1]));
        assert!(diagonal.could_be_equal(n(5.0)), "{diagonal:?}");
    }

    /// Two lengths of one rectangle a hundred-billionth apart: closer than
    /// the solver's tolerance, so the solve meets both, but they conflict,
    /// and proving the solution says so, naming both.
    #[test]
    fn conflicting_lengths_are_named() {
        let mut s = Sketch::<S>::new();
        let sides = rectangle_said_twice(&mut s, true);
        let top = s.constrain(Constraint::Length {
            curve: sides[2],
            value: n(2.0 + 1e-11),
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        let e = s.enclose::<S>().unwrap_err();
        let message = e.root_message();
        let bottom = s
            .constraints
            .iter()
            .find(|(_, c)| matches!(c, Constraint::Length { curve, .. } if *curve == sides[0]))
            .map(|(id, _)| *id)
            .unwrap();
        assert!(message.contains("conflict"), "{message}");
        for id in [top, bottom] {
            assert!(message.contains(&format!("{id} ")), "{id}: {message}");
        }
    }
}
