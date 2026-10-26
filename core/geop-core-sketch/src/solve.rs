//! Solving a sketch: every constraint contributes residuals that are zero
//! exactly when it holds — a term of the sketch's variables — and
//! [`geop_core_math::solvers::system`], on the driver every solve of the
//! kernel shares ([`geop_core_math::solvers::constrained`]), holds them,
//! pulls a dragged point, and encloses the exact solution.
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
    profile::basis_at,
    sketch::{Constraint, ConstraintId, CurveId, CurveKind, Enclosure, Point, PointId, Sketch},
    variables::{PointLayout, Residuals, SketchPoint, Standing, Variables},
};
use geop_core_math::solvers::system::{self, Phase};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Number, Scalar, as_f64},
    vector::Vector2,
};

/// The most variables a single constraint may depend on. The largest
/// constraint, a point of a spline's offset ([`Constraint::OffsetAt`]),
/// touches the four control points of two cubics a point depends on and
/// a circle's radius.
const MAX_LOCAL_VARS: usize = 17;

/// A residual with its gradient with respect to its constraint's own
/// variables.
type Dual = geop_core_math::dual::Dual<MAX_LOCAL_VARS>;

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Maps sketch entities to solver variables: each class of points a pair
/// (see [`PointLayout`]), then every arc's half sweep and circle's radius.
struct Layout {
    points: PointLayout<2>,
    /// The arc's half sweep or the circle's radius, for every arc and circle.
    curve_var: BTreeMap<CurveId, usize>,
}

impl SketchPoint<2> for Point {
    fn fixed(&self) -> bool {
        self.fixed
    }

    fn coordinates(&self) -> [Scalar; 2] {
        [self.x, self.y]
    }

    fn set_coordinates(&mut self, [x, y]: [Scalar; 2]) {
        (self.x, self.y) = (x, y);
    }
}

impl Layout {
    fn new(sketch: &Sketch) -> Self {
        let mut points = PointLayout::new(&sketch.points, &sketch.point_classes());
        let curve_var = sketch
            .curves
            .iter()
            .filter(|(_, c)| matches!(c.kind, CurveKind::Arc { .. } | CurveKind::Circle { .. }))
            .map(|(&id, c)| (id, points.push(!c.fixed)))
            .collect();
        Layout { points, curve_var }
    }

    /// The current variable values, read from the sketch: the points as
    /// [`PointLayout::read`] reads them, each curve's own variable as
    /// sharp as they.
    fn read(&self, sketch: &Sketch) -> GeopResult<Vec<Scalar>> {
        let mut x = vec![Scalar::ZERO; self.points.len()];
        self.points.read(&sketch.points, &mut x);
        for (id, c) in &sketch.curves {
            match c.kind {
                CurveKind::Arc { sweep, .. } => {
                    x[self.curve_var[id]] = sweep.div(Scalar::TWO)?.sharpen()
                }
                CurveKind::Circle { radius, .. } => x[self.curve_var[id]] = radius,
                _ => {}
            }
        }
        Ok(x)
    }

    /// Write variable values back into the sketch — all but what is fixed,
    /// which stays as given.
    fn write(&self, sketch: &mut Sketch, x: &[Scalar]) {
        self.points.write(&mut sketch.points, x);
        for (id, c) in sketch.curves.iter_mut().filter(|(_, c)| !c.fixed) {
            match &mut c.kind {
                CurveKind::Arc { sweep, .. } => {
                    *sweep = Scalar::TWO.mul(x[self.curve_var[id]]).sharpen()
                }
                CurveKind::Circle { radius, .. } => *radius = x[self.curve_var[id]].abs().sharpen(),
                _ => {}
            }
        }
    }

    /// The variables a curve depends on.
    fn curve_vars(&self, sketch: &Sketch, c: CurveId, out: &mut Vec<usize>) {
        for p in sketch.curves[&c].points() {
            self.points.point_vars(p, out);
        }
        out.extend(self.curve_var.get(&c));
    }
}

/// Read-only view of the variables for evaluating one constraint, with that
/// constraint's own variables `seeded` as [`Scalar`] values.
struct Geo<'a, T> {
    sketch: &'a Sketch,
    layout: &'a Layout,
    seeded: &'a [(usize, T)],
}

impl<T: Number> Geo<'_, T> {
    fn var(&self, i: usize) -> T {
        self.seeded
            .iter()
            .find(|(j, _)| *j == i)
            .map(|(_, t)| *t)
            .expect("every variable a constraint reads is seeded")
    }
    fn point(&self, p: PointId) -> Vector2<T> {
        let i = self.layout.points.point_var[&p];
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
struct Prepared<'a> {
    id: ConstraintId,
    constraint: &'a Constraint,
    vars: Vec<usize>,
    tangent: Option<TangentMode>,
    /// For a [`Constraint::OffsetAt`], the splines' points as sums of
    /// their control points.
    offset: Option<OffsetBasis>,
}

/// The points of a [`Constraint::OffsetAt`]: of `a`, with its derivative,
/// and of `b`, as sums of their control points (see [`spline_basis`]).
struct OffsetBasis {
    a: Vec<(PointId, Scalar, Scalar)>,
    b: Vec<(PointId, Scalar, Scalar)>,
}

/// The spline `curve`'s point at `t` — and its derivative there, if
/// `derivative` — as a sum of its control points (see [`basis_at`]).
fn spline_basis(
    sketch: &Sketch,
    curve: CurveId,
    t: Scalar,
    derivative: bool,
) -> GeopResult<Vec<(PointId, Scalar, Scalar)>> {
    let CurveKind::Spline {
        control_points,
        shape,
    } = &sketch.curve(curve)?.kind
    else {
        return Err(GeopError::new(format!("curve {curve} is no spline")));
    };
    Ok(
        basis_at(control_points.len(), shape.as_ref(), t, derivative)?
            .into_iter()
            .map(|(j, value, slope)| (control_points[j], value, slope))
            .collect(),
    )
}

/// Whether `a` and `b` are both fixed points: then a coincidence between
/// them is no merging of variables — neither can move — but a fact about
/// where they were given, which holds or does not.
fn both_fixed(sketch: &Sketch, a: PointId, b: PointId) -> bool {
    sketch.points[&a].fixed && sketch.points[&b].fixed
}

/// Everything needed to evaluate the objective, fixed for one solve.
struct Problem<'a> {
    sketch: &'a Sketch,
    layout: Layout,
    constraints: Vec<Prepared<'a>>,
    /// Characteristic length of the sketch; see the module docs.
    scale: Scalar,
}

impl<'a> Problem<'a> {
    fn new(sketch: &'a Sketch) -> GeopResult<Self> {
        sketch.validate()?;
        let layout = Layout::new(sketch);
        let x0 = layout.read(sketch)?;

        // The diagonal of the points' bounding box, or a circle's diameter
        // if that is larger — one, where the sketch has no size at all.
        let span = |k: usize| {
            let values = || sketch.points.values().map(move |p| p.xy()[k]);
            let hi = values().reduce(Scalar::max).unwrap_or(Scalar::ZERO);
            let lo = values().reduce(Scalar::min).unwrap_or(Scalar::ZERO);
            hi.sub(lo)
        };
        let diagonal = span(0).mul(span(0)).add(span(1).mul(span(1))).sqrt()?;
        let scale = sketch
            .curves
            .values()
            .filter_map(|c| match c.kind {
                CurveKind::Circle { radius, .. } => Some(Scalar::TWO.mul(radius.abs())),
                _ => None,
            })
            .fold(diagonal, Scalar::max);
        let scale = if scale.is_finite() && scale.definitely_greater(Scalar::ZERO) {
            scale
        } else {
            Scalar::ONE
        };

        let mut constraints = Vec::new();
        for (&id, c) in &sketch.constraints {
            use Constraint::*;
            let mut vars = Vec::new();
            let mut offset_basis = None;
            let pt = |p: &PointId, vars: &mut Vec<usize>| layout.points.point_vars(*p, vars);
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
                // The control points the two points depend on, and the
                // circle's radius alone.
                OffsetAt {
                    a, at, b, on, by, ..
                } => {
                    let basis = OffsetBasis {
                        a: spline_basis(sketch, *a, *at, true)?,
                        b: spline_basis(sketch, *b, *on, false)?,
                    };
                    for (p, ..) in basis.a.iter().chain(&basis.b) {
                        pt(p, &mut vars);
                    }
                    vars.push(layout.curve_var[by]);
                    offset_basis = Some(basis);
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
                offset: offset_basis,
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
        sketch: &Sketch,
        layout: &Layout,
        x: &[Scalar],
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
        let seeded: Vec<(usize, Scalar)> = x.iter().copied().enumerate().collect();
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
        p: &Prepared<'_>,
        geo: &Geo<'_, Dual>,
        out: &mut Vec<Dual>,
    ) -> GeopResult<()> {
        use Constraint::*;
        let c = Dual::cst;
        let scale = c(self.scale);
        let half = c(Scalar::ONE.div(Scalar::TWO)?);
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
                        let turn = arc.half.mul(c(Scalar::TWO));
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
            OffsetAt { by, across, .. } => {
                let basis = p.offset.as_ref().expect("prepared with its basis");
                let sum = |terms: &[(PointId, Scalar, Scalar)], slope: bool| {
                    terms.iter().fold(
                        Vector2::from_array([c(Scalar::ZERO); 2]),
                        |acc, &(q, v, d)| {
                            acc.add(&geo.point(q).prod_scalar(c(if slope { d } else { v })))
                        },
                    )
                };
                let tangent = sum(&basis.a, true).normalize()?;
                let normal = Vector2::from_array([tangent[1].neg(), tangent[0]]);
                let off = sum(&basis.b, false).sub(&sum(&basis.a, false));
                if !across {
                    out.push(off.prod_dot(&tangent));
                }
                let radius = geo.var(self.layout.curve_var[&by]).abs();
                out.push(off.prod_dot(&normal).abs().sub(radius));
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
                let size = |c: CurveId| -> GeopResult<Dual> {
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
                Some(a) => c(Scalar::TWO.mul(value))
                    .mul(a.half.sin().abs())
                    .sub(a.chord_length()?),
                None => geo.round(curve)?.1.sub(c(value)),
            }),
            Diameter { curve, value } => out.push(match geo.arc(curve) {
                // As for the radius: `D |sin θ| - L`.
                Some(a) => c(value).mul(a.half.sin().abs()).sub(a.chord_length()?),
                None => geo.round(curve)?.1.sub(c(value.div(Scalar::TWO)?)),
            }),
        }
        Ok(())
    }
}

impl<'a> Residuals<MAX_LOCAL_VARS> for Problem<'a> {
    fn offsets(&self) -> Vec<Option<usize>> {
        self.layout.points.offsets()
    }

    fn constraints(&self) -> usize {
        self.constraints.len()
    }

    fn reads(&self, c: usize) -> &[usize] {
        &self.constraints[c].vars
    }

    fn residuals(&self, c: usize, values: &[Dual], out: &mut Vec<Dual>) -> GeopResult<()> {
        let prepared = &self.constraints[c];
        let seeded: Vec<(usize, Dual)> = prepared
            .vars
            .iter()
            .copied()
            .zip(values.iter().copied())
            .collect();
        let geo = Geo {
            sketch: self.sketch,
            layout: &self.layout,
            seeded: &seeded,
        };
        Problem::residuals(self, prepared, &geo, out)
    }

    fn name(&self, c: usize) -> String {
        let c = &self.constraints[c];
        format!("{} {:?}", c.id, c.constraint)
    }

    fn scale(&self) -> Scalar {
        self.scale
    }
}

/// The planar sketch's system (see [`Variables`]).
type Planar<'p, 'a> = Variables<'p, Problem<'a>, MAX_LOCAL_VARS>;

/// The outcome of a solve.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolveReport {
    /// Every constraint holds (to
    /// [`geop_core_math::solvers::system::RELATIVE_TOLERANCE`] of the
    /// sketch size).
    pub converged: bool,
    /// Largest remaining constraint residual, in sketch units: an upper
    /// bound.
    #[serde(with = "as_f64")]
    pub max_residual: Scalar,
    pub iterations: usize,
    /// Each minimization of the solve.
    pub phases: Vec<Phase>,
    /// Remaining degrees of freedom: variables minus independent
    /// constraints — which are independent decided over the box an
    /// enclosure of the solution chose them over (see
    /// [`geop_core_math::solvers::system::System::freedom`]).
    pub dof: usize,
    /// Per point: whether it can still move without violating a constraint.
    pub free_points: BTreeMap<PointId, bool>,
    /// Per curve: whether any of its points or its own sweep/radius can still
    /// change.
    pub free_curves: BTreeMap<CurveId, bool>,
    /// The constraints left unsatisfied (conflicting or unreachable), empty
    /// when `converged`.
    pub failed_constraints: Vec<ConstraintId>,
    /// Why no true solution near where the solve ended is proven, if none
    /// is: the constraints that conflict there, by id, or what is singular
    /// (see [`Sketch::enclose`], which encloses it).
    #[serde(default)]
    pub unproven: Vec<String>,
}

impl Sketch {
    /// The sketch's geometry as the kernel builds on it (see [`Enclosure`]):
    /// an enclosure of the exact solution of its constraints near the
    /// solved positions, each variable the constraints leave free exactly
    /// as drawn (see [`System::enclose`]).
    ///
    /// A sketch whose constraints are not met is no solution of them, and is
    /// built exactly as drawn: that is all there is to build. Fails if the
    /// solution cannot be proven — a sketch meeting its constraints only at
    /// a singular configuration.
    pub fn enclose(&self) -> GeopResult<Enclosure> {
        let ctx = |e: GeopError| e.with_context("enclosing the sketch's solution");
        let problem = Problem::new(self).map_err(ctx)?;
        let params = problem.layout.read(self)?;
        let variables = Planar::new(&problem, params, Vec::new());
        let start = variables.start();
        if !variables.system().held(&start)?.converged {
            return Ok(Enclosure::as_drawn(self));
        }
        // What is fixed is as given; the rest as enclosed.
        let x = variables.enclosed(&start).map_err(ctx)?;
        let layout = &problem.layout;
        Ok(Enclosure {
            points: (layout.points.point_var.keys())
                .map(|&p| (p, Vector2::from_array(layout.points.point(&x, p))))
                .collect(),
            params: self
                .curves
                .iter()
                .filter_map(|(&id, c)| {
                    let v = x[*layout.curve_var.get(&id)?];
                    Some(match c.kind {
                        CurveKind::Arc { .. } => (id, v.mul(Scalar::TWO)),
                        _ => (id, v.abs()),
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
    pub fn solve_with_drag(&mut self, drags: &[(PointId, Vector2)]) -> GeopResult<SolveReport> {
        let problem = Problem::new(self)?;
        let start = problem.layout.read(self)?;
        let (variables, solved, x) =
            Planar::dragged(&problem, start, &problem.layout.points.point_var, drags)?;
        let report = variables.report(&solved.values, solved.phases, solved.enclosure)?;
        // Where the solve put the points is the sketch's state, and a free
        // choice: sharp.
        let x: Vec<Scalar> = x.into_iter().map(Scalar::sharpen).collect();
        problem.layout.write(self, &x);
        Ok(report)
    }
}

impl Planar<'_, '_> {
    /// How the sketch stands with the free variables at `blocks`: which
    /// constraints hold, and what can still move.
    fn report(
        &self,
        blocks: &[Scalar],
        phases: Vec<Phase>,
        solved: system::Enclosure<Scalar>,
    ) -> GeopResult<SolveReport> {
        let Standing {
            held,
            dof,
            free: free_vars,
            unproven,
        } = self.standing(blocks, Some(solved))?;
        let problem = self.problem;
        let failed_constraints = (held.failed.iter())
            .map(|&i| problem.constraints[i].id)
            .collect::<Vec<_>>();
        let points = &problem.layout.points;
        let free_points = (points.point_var.keys())
            .map(|&p| (p, points.point(&free_vars, p).contains(&true)))
            .collect();
        let free_curves = problem
            .sketch
            .curves
            .keys()
            .map(|&c| {
                let mut vars = Vec::new();
                problem.layout.curve_vars(problem.sketch, c, &mut vars);
                (c, vars.iter().any(|&i| free_vars[i]))
            })
            .collect();
        Ok(SolveReport {
            converged: failed_constraints.is_empty(),
            max_residual: held.max_residual,
            iterations: phases.iter().map(|p| p.iterations).sum(),
            phases,
            dof,
            free_points,
            free_curves,
            failed_constraints,
            unproven,
        })
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use geop_core_math::{
        scalars::Scalar,
        solvers::system::{Equations, Term},
    };

    use super::*;

    fn n(x: f64) -> Scalar {
        Scalar::from_f64(x)
    }

    /// At points drawn exactly, every residual and every slope of it is
    /// known to rounding: the solver takes a step only where the merit
    /// definitely drops, and a residual wider than its arithmetic needs
    /// would hide every step.
    #[test]
    fn residuals_are_sharp_where_the_points_are() {
        let mut s = Sketch::new();
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
        let params = problem.layout.read(&s).unwrap();
        let variables = Planar::new(&problem, params, Vec::new());
        let terms: Vec<Term> = (0..problem.constraints.len())
            .map(Term::Constraint)
            .collect();
        for (k, made) in variables
            .evaluate(&variables.start(), &terms)
            .iter()
            .enumerate()
        {
            let made = made.as_ref().unwrap();
            let (v, row) = (&made.values, &made.jacobian);
            for x in (v.iter()).chain(row.iter().flat_map(|(_, rows)| rows.iter().flatten())) {
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
    fn rectangle_said_twice(s: &mut Sketch, level: bool) -> Vec<CurveId> {
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
    fn far_corner(s: &Sketch, sides: &[CurveId], e: &Enclosure) -> Vector2 {
        let (_, end) = s.curves[&sides[1]].endpoints().unwrap();
        e.points[&end]
    }

    /// A profile between two edges of a part projected into the sketch, both
    /// on one straight side of it (the microduck's head): its closing line is
    /// made vertical, which its ends on the two edges already say. The
    /// edges' ends are given as the projection encloses them, and their
    /// rounding (`-25.000000000000004` against `-24.999999999999993`) tilts
    /// each a hair off vertical, so the vertical row is independent by a
    /// pivot of `1e-15` at sharp ends: taken as independent, nothing could
    /// be proven. Over the ends as enclosed, that pivot could be zero, and the
    /// vertical is the redundant constraint it is.
    #[test]
    fn a_line_closing_on_two_projected_edges_of_one_side_is_proven() {
        let given = |x: f64| Scalar::new(x - 1e-13, x + 1e-13);
        let mut s = Sketch::new();
        let fixed = |s: &mut Sketch, x: f64, y: f64| s.add_fixed_point(given(x), given(y));
        let upper = {
            let a = fixed(&mut s, -25.000000000000004, 14.52958965872012);
            let b = fixed(&mut s, -24.999999999999993, -11.926666666666662);
            s.add_line(a, b)
        };
        let lower = {
            let a = fixed(&mut s, -24.999999999999993, -62.64416256853966);
            let b = fixed(&mut s, -25.000000000000007, -11.926666666666666);
            s.add_line(a, b)
        };
        for c in [upper, lower] {
            let curve = s.curves.get_mut(&c).unwrap();
            (curve.fixed, curve.construction) = (true, true);
        }
        let p24 = s.add_point(n(-24.999999999999993), n(-19.650020988468217));
        let p26 = s.add_point(n(-13.56352402378605), n(-23.519644932457258));
        let p28 = s.add_point(n(-13.56352402378605), n(-35.413048566902255));
        let p36 = s.add_point(n(-24.999999999999996), n(-39.2826725108913));
        let l27 = s.add_line(p24, p26);
        let a29 = s.add_arc(p26, p28, n(-2.489060692764724));
        let l38 = s.add_line(p28, p36);
        let l40 = s.add_line(p36, p24);
        for c in [
            Constraint::PointOnCurve {
                point: p24,
                curve: upper,
            },
            Constraint::Tangent { a: l27, b: a29 },
            Constraint::PointOnCurve {
                point: p36,
                curve: lower,
            },
            Constraint::Tangent { a: a29, b: l38 },
            Constraint::Vertical { line: l40 },
            Constraint::Tangent { a: l38, b: a29 },
            Constraint::DistanceX {
                a: p26,
                b: p28,
                value: n(0.0),
            },
        ] {
            s.constrain(c);
        }
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        s.enclose().unwrap();
    }

    /// Four right angles where three would do, and parallels besides: the
    /// redundant constraints hold wherever the others do, and the solution
    /// is proven — level, every coordinate determined; turned, the turn
    /// left as drawn.
    #[test]
    fn a_rectangle_said_twice_is_proven() {
        let mut s = Sketch::new();
        let sides = rectangle_said_twice(&mut s, true);
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert!(report.unproven.is_empty(), "{report:?}");
        let e = s.enclose().unwrap();
        let corner = far_corner(&s, &sides, &e);
        assert!(
            corner[0].could_be_equal(n(2.0)) && corner[1].could_be_equal(n(1.0)),
            "{corner:?}"
        );
        assert!(corner[0].width().to_f64() < 1e-12, "{corner:?}");

        let mut s = Sketch::new();
        let sides = rectangle_said_twice(&mut s, false);
        let report = s.solve().unwrap();
        assert!(report.converged && report.dof == 1, "{report:?}");
        let e = s.enclose().unwrap();
        let corner = far_corner(&s, &sides, &e);
        let diagonal = corner[0].mul(corner[0]).add(corner[1].mul(corner[1]));
        assert!(diagonal.could_be_equal(n(5.0)), "{diagonal:?}");
    }

    /// Two lengths of one rectangle a hundred-billionth apart: closer than
    /// the solver's tolerance, so the solve meets both, but they conflict,
    /// and proving the solution says so, naming both.
    #[test]
    fn conflicting_lengths_are_named() {
        let mut s = Sketch::new();
        let sides = rectangle_said_twice(&mut s, true);
        let top = s.constrain(Constraint::Length {
            curve: sides[2],
            value: n(2.0 + 1e-11),
        });
        let report = s.solve().unwrap();
        assert!(report.converged, "{report:?}");
        // The solve says so too: no true solution is proven.
        let [why] = &report.unproven[..] else {
            panic!("{report:?}");
        };
        assert!(why.contains("conflict"), "{why}");
        let e = s.enclose().unwrap_err();
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
