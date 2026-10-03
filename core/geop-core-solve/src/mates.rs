//! Rigid bodies and the constraints between them — mates — as a
//! [`System`]: a body is a pose parameter, a constraint a residual between
//! a point, a line or a plane attached to each of two bodies, or to the
//! ground, which never moves. [`Assembly`] puts them together.

use geop_core_math::{
    dual,
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Ring, Scalar, as_f64},
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::{Param, Placed, Pull as ParamPull, Residual, System, Value};

/// Variables per body: a translation and a turn.
const BODY_VARS: usize = 6;

/// The dual numbers of a mate: two bodies' worth of variables.
pub const MATE_VARS: usize = 2 * BODY_VARS;

type Dual<S> = dual::Dual<S, MATE_VARS>;
type V<S> = Vector3<Dual<S>>;

fn cst<S: Scalar>(v: &Vector3<S>) -> V<S> {
    v.map(Dual::cst)
}

/// A piece of geometry a constraint holds, in the frame of the body it is
/// attached to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Geometry<S: Scalar> {
    Point {
        at: Vector3<S>,
    },
    /// The line through `point` along `direction` (any non-zero length).
    Line {
        point: Vector3<S>,
        direction: Vector3<S>,
    },
    /// The plane through `point` normal to `normal` (any non-zero length).
    Plane {
        point: Vector3<S>,
        normal: Vector3<S>,
    },
}

impl<S: Scalar> Geometry<S> {
    fn kind(&self) -> &'static str {
        match self {
            Geometry::Point { .. } => "a point",
            Geometry::Line { .. } => "a line",
            Geometry::Plane { .. } => "a plane",
        }
    }
}

/// Geometry attached to a body — by its index in [`Assembly::bodies`] — or
/// to the ground (`None`), which never moves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Feature<S: Scalar> {
    pub body: Option<usize>,
    pub geometry: Geometry<S>,
}

/// What a constraint asks of its two features.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", bound = "S: Scalar")]
pub enum Kind<S: Scalar> {
    /// They touch: two points coincide, a point lies on a line or a plane,
    /// two lines are one, a line lies in a plane, two planes are one
    /// (facing either way).
    Coincident,
    /// Two lines — axes, as a round edge or face has — are one.
    Concentric,
    /// Lines and planes run parallel: two lines, two planes' normals, a
    /// line along a plane.
    Parallel,
    /// Lines and planes stand square: two lines, two planes, a line along a
    /// plane's normal.
    Perpendicular,
    /// They are `value` apart: two points, a point and a line or a plane,
    /// or two parallel lines or planes, or a line parallel to a plane.
    Distance {
        #[serde(with = "as_f64")]
        value: S,
    },
    /// The lines (or planes' normals) meet at `value` degrees.
    Angle {
        #[serde(with = "as_f64")]
        value: S,
    },
}

impl<S: Scalar> Kind<S> {
    /// The same kind in the scalar type `T` (see [`Scalar::cast`]).
    pub fn cast<T: Scalar>(&self) -> Kind<T> {
        match *self {
            Kind::Coincident => Kind::Coincident,
            Kind::Concentric => Kind::Concentric,
            Kind::Parallel => Kind::Parallel,
            Kind::Perpendicular => Kind::Perpendicular,
            Kind::Distance { value } => Kind::Distance {
                value: value.cast(),
            },
            Kind::Angle { value } => Kind::Angle {
                value: value.cast(),
            },
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            Kind::Coincident => "Coincident",
            Kind::Concentric => "Concentric",
            Kind::Parallel => "Parallel",
            Kind::Perpendicular => "Perpendicular",
            Kind::Distance { .. } => "Distance",
            Kind::Angle { .. } => "Angle",
        }
    }
}

/// A constraint between two features.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Constraint<S: Scalar> {
    pub kind: Kind<S>,
    pub a: Feature<S>,
    pub b: Feature<S>,
}

impl<S: Scalar> Constraint<S> {
    /// Checks that its kind can hold between its features at all: a
    /// distance between two lines, say, but no angle at a point.
    pub fn validate(&self) -> GeopResult<()> {
        use Geometry::*;
        let (a, b) = (&self.a.geometry, &self.b.geometry);
        let directed = |g: &Geometry<S>| !matches!(g, Point { .. });
        let fits = match self.kind {
            Kind::Coincident | Kind::Distance { .. } => true,
            Kind::Concentric => matches!((a, b), (Line { .. }, Line { .. })),
            Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. } => directed(a) && directed(b),
        };
        if fits {
            Ok(())
        } else {
            Err(GeopError::new(format!(
                "a {} constraint cannot hold between {} and {}",
                self.kind.label().to_lowercase(),
                a.kind(),
                b.kind()
            )))
        }
    }
}

/// A rigid body: where it is, whether the constraints may move it, and the
/// point of it — in its own frame — that it turns about while solved: its
/// middle, ideally.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body<S: Scalar> {
    pub pose: Pose<S>,
    pub free: bool,
    pub center: Vector3<S>,
}

/// What a solve pulls a body towards (see the crate docs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pull<S: Scalar> {
    /// The whole body towards `target`.
    Pose { body: usize, target: Pose<S> },
    /// The body's point `local` (in its own frame) towards the world point
    /// `target` — a drag.
    Point {
        body: usize,
        local: Vector3<S>,
        target: Vector3<S>,
    },
}

/// The outcome of a solve.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct SolveReport<S: Scalar> {
    /// Every constraint holds (to [`crate::RELATIVE_TOLERANCE`] of the size).
    pub converged: bool,
    /// The largest remaining constraint residual, in units of length: an
    /// upper bound.
    #[serde(with = "as_f64")]
    pub max_residual: S,
    pub iterations: usize,
    /// Each minimization of the solve.
    pub phases: Vec<crate::Phase<S>>,
    /// The constraints left unsatisfied — conflicting, or unreachable from
    /// where the bodies started — by index; empty when `converged`.
    pub failed: Vec<usize>,
}

/// Bodies and the constraints between them.
#[derive(Clone, Debug, PartialEq)]
pub struct Assembly<S: Scalar> {
    pub bodies: Vec<Body<S>>,
    pub constraints: Vec<Constraint<S>>,
    /// Its characteristic size: what turns angles into lengths, and what
    /// the tolerance constraints are held to is relative to. At least 1.
    pub scale: S,
}

/// A feature, where it is during a solve.
enum World<S: Scalar> {
    Point(V<S>),
    Line(V<S>, V<S>),
    Plane(V<S>, V<S>),
}

impl<S: Scalar> World<S> {
    /// Its direction: a line's, a plane's normal.
    fn direction(&self) -> Option<&V<S>> {
        match self {
            World::Point(_) => None,
            World::Line(_, d) | World::Plane(_, d) => Some(d),
        }
    }
}

/// A constraint as a residual of the poses of the bodies it holds.
struct Mate<S: Scalar> {
    constraint: Constraint<S>,
    /// The bodies of its features — none for the ground — as parameters.
    params: Vec<usize>,
    scale: S,
}

impl<S: Scalar> Mate<S> {
    fn new(constraint: Constraint<S>, scale: S) -> Self {
        let mut params: Vec<usize> = [constraint.a.body, constraint.b.body]
            .into_iter()
            .flatten()
            .collect();
        params.dedup();
        Mate {
            constraint,
            params,
            scale,
        }
    }

    /// The body `body` places among `values`; `None` for the ground.
    fn placed<'v>(
        &self,
        values: &'v [Value<Dual<S>>],
        body: Option<usize>,
    ) -> GeopResult<Option<&'v Placed<Dual<S>>>> {
        let Some(body) = body else {
            return Ok(None);
        };
        let slot = self
            .params
            .iter()
            .position(|&p| p == body)
            .expect("its own bodies");
        Ok(Some(values[slot].pose()?))
    }

    /// `feature` in the world.
    fn world(feature: &Feature<S>, placed: Option<&Placed<Dual<S>>>) -> GeopResult<World<S>> {
        let point = |at: &Vector3<S>| -> V<S> {
            match placed {
                Some(p) => p.point(&cst(at)),
                None => cst(at),
            }
        };
        let direction = |d: &Vector3<S>| -> GeopResult<V<S>> {
            let d = match placed {
                Some(p) => p.direction(&cst(d)),
                None => cst(d),
            };
            d.normalize()
        };
        Ok(match &feature.geometry {
            Geometry::Point { at } => World::Point(point(at)),
            Geometry::Line {
                point: p,
                direction: d,
            } => World::Line(point(p), direction(d)?),
            Geometry::Plane {
                point: p,
                normal: n,
            } => World::Plane(point(p), direction(n)?),
        })
    }

    /// The residuals of `constraint`, with `a` and `b` its features in the
    /// world.
    fn residuals(
        &self,
        kind: Kind<S>,
        a: &World<S>,
        b: &World<S>,
        out: &mut Vec<Dual<S>>,
    ) -> GeopResult<()> {
        use World::*;
        let l = Dual::cst(self.scale);
        let vector = |v: V<S>, out: &mut Vec<Dual<S>>| out.extend(v.to_array());
        let parallel = |d: &V<S>, e: &V<S>, out: &mut Vec<Dual<S>>| {
            vector(d.prod_cross(e).prod_scalar(l), out)
        };
        // How far `p` is off the line through `q` along `d`, as a vector.
        let off_line = |p: &V<S>, q: &V<S>, d: &V<S>| p.sub(q).prod_cross(d);
        let along = |p: &V<S>, q: &V<S>, n: &V<S>| n.prod_dot(&p.sub(q));
        // A point-to-thing distance, as a residual and as a value.
        let gap = |a: &World<S>, b: &World<S>| -> Option<Dual<S>> {
            Some(match (a, b) {
                (Point(p), Point(q)) => p.sub(q).norm(),
                (Point(p), Line(q, d)) | (Line(q, d), Point(p)) => off_line(p, q, d).norm(),
                (Point(p), Plane(q, n)) | (Plane(q, n), Point(p)) => along(p, q, n).abs(),
                _ => return None,
            })
        };
        match kind {
            Kind::Coincident => match (a, b) {
                (Point(p), Point(q)) => vector(p.sub(q), out),
                (Point(p), Line(q, d)) | (Line(q, d), Point(p)) => vector(off_line(p, q, d), out),
                (Point(p), Plane(q, n)) | (Plane(q, n), Point(p)) => out.push(along(p, q, n)),
                (Line(p, d), Line(q, e)) => {
                    parallel(d, e, out);
                    vector(off_line(q, p, d), out);
                }
                (Line(p, d), Plane(q, n)) | (Plane(q, n), Line(p, d)) => {
                    out.push(d.prod_dot(n).mul(l));
                    out.push(along(p, q, n));
                }
                (Plane(p, n), Plane(q, m)) => {
                    parallel(n, m, out);
                    out.push(along(q, p, n));
                }
            },
            Kind::Concentric => match (a, b) {
                (Line(p, d), Line(q, e)) => {
                    parallel(d, e, out);
                    vector(off_line(q, p, d), out);
                }
                _ => unreachable!("validated"),
            },
            Kind::Parallel | Kind::Perpendicular => {
                let (d, e) = (
                    a.direction().expect("validated"),
                    b.direction().expect("validated"),
                );
                // A line along a plane runs square to its normal, and a line
                // square to a plane runs along its normal: mixed pairs swap.
                let mixed = matches!((a, b), (Line(..), Plane(..)) | (Plane(..), Line(..)));
                if matches!(kind, Kind::Parallel) != mixed {
                    parallel(d, e, out);
                } else {
                    out.push(d.prod_dot(e).mul(l));
                }
            }
            Kind::Distance { value } => {
                let value = Dual::cst(value);
                match (a, b) {
                    (Line(p, d), Line(q, e)) => {
                        parallel(d, e, out);
                        out.push(off_line(q, p, d).norm().sub(value));
                    }
                    (Line(p, d), Plane(q, n)) | (Plane(q, n), Line(p, d)) => {
                        out.push(d.prod_dot(n).mul(l));
                        out.push(along(p, q, n).abs().sub(value));
                    }
                    (Plane(p, n), Plane(q, m)) => {
                        parallel(n, m, out);
                        out.push(along(q, p, n).abs().sub(value));
                    }
                    _ => out.push(gap(a, b).expect("a point and anything").sub(value)),
                }
            }
            Kind::Angle { value } => {
                let (d, e) = (
                    a.direction().expect("validated"),
                    b.direction().expect("validated"),
                );
                let radians = value.mul(S::PI.div(S::from_i64(180))?);
                out.push(d.prod_dot(e).sub(Dual::cst(radians.cos())).mul(l));
                out.push(d.prod_cross(e).norm().sub(Dual::cst(radians.sin())).mul(l));
            }
        }
        Ok(())
    }
}

impl<S: Scalar> Residual<S, MATE_VARS> for Mate<S> {
    fn params(&self) -> &[usize] {
        &self.params
    }

    fn eval(&self, values: &[Value<Dual<S>>], out: &mut Vec<Dual<S>>) -> GeopResult<()> {
        let c = &self.constraint;
        let a = Self::world(&c.a, self.placed(values, c.a.body)?)?;
        let b = Self::world(&c.b, self.placed(values, c.b.body)?)?;
        self.residuals(c.kind, &a, &b, out)
    }
}

impl<S: Scalar> Assembly<S> {
    /// Checks every constraint can hold between its features at all, and
    /// refers to bodies there are.
    pub fn validate(&self) -> GeopResult<()> {
        for (i, c) in self.constraints.iter().enumerate() {
            for body in [c.a.body, c.b.body].into_iter().flatten() {
                if body >= self.bodies.len() {
                    return Err(GeopError::new(format!(
                        "constraint {i} refers to body {body}, but there are {} bodies",
                        self.bodies.len()
                    )));
                }
            }
            c.validate()
                .map_err(|e| e.with_context(format!("constraint {i}")))?;
        }
        Ok(())
    }

    fn mates(&self) -> Vec<Mate<S>> {
        self.constraints
            .iter()
            .map(|&c| Mate::new(c, self.scale))
            .collect()
    }

    /// The system of the bodies and `mates`.
    fn system<'m>(&self, mates: &'m [Mate<S>]) -> System<'m, S, MATE_VARS> {
        System {
            params: self
                .bodies
                .iter()
                .map(|b| Param::Pose {
                    pose: b.pose,
                    center: b.center,
                })
                .collect(),
            free: self.bodies.iter().map(|b| b.free).collect(),
            residuals: mates
                .iter()
                .map(|m| m as &dyn Residual<S, MATE_VARS>)
                .collect(),
            scale: self.scale,
        }
    }

    /// Moves the free bodies so every constraint holds, changing the poses
    /// as little as the constraints allow — pulled as `pulls` ask (see the
    /// crate docs).
    ///
    /// The bodies are moved even if the solve does not converge, to the
    /// closest configuration found — the report says which constraints
    /// could not be met.
    pub fn solve(&mut self, pulls: &[Pull<S>]) -> GeopResult<SolveReport<S>> {
        self.validate()?;
        let mates = self.mates();
        let mut system = self.system(&mates);
        let pulls: Vec<ParamPull<S>> = pulls
            .iter()
            .map(|p| match *p {
                Pull::Pose { body, target } => ParamPull::Pose {
                    param: body,
                    target,
                },
                Pull::Point {
                    body,
                    local,
                    target,
                } => ParamPull::Point {
                    param: body,
                    local,
                    target,
                },
            })
            .collect();
        let report = system.solve(&pulls)?;
        for (body, param) in self.bodies.iter_mut().zip(&system.params) {
            if let Param::Pose { pose, .. } = param {
                body.pose = *pose;
            }
        }
        Ok(SolveReport {
            converged: report.converged,
            max_residual: report.max_residual,
            iterations: report.iterations,
            phases: report.phases,
            failed: report.failed,
        })
    }

    /// Which constraints hold where the bodies are now.
    pub fn report(&self) -> GeopResult<SolveReport<S>> {
        let mates = self.mates();
        let report = self.system(&mates).report()?;
        Ok(SolveReport {
            converged: report.converged,
            max_residual: report.max_residual,
            iterations: 0,
            phases: Vec::new(),
            failed: report.failed,
        })
    }
}

#[cfg(test)]
#[path = "mates_tests.rs"]
mod tests;
