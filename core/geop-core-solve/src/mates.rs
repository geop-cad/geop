//! Rigid bodies and the constraints between them — mates — as a
//! [`System`]: a body is a pose parameter, a constraint a residual between
//! a point, a line or a plane attached to each of two bodies, or to the
//! ground, which never moves. Joints, and the couplings between them, are
//! mates too (see [`joints`]): their coordinates are numbers of the system.
//! [`Assembly`] puts them together.

use geop_core_math::{
    dual,
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Ring, Scalar, as_f64},
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::{Param, Placed, Pull as ParamPull, Residual, System, Value, linalg::rank};

mod joints;

pub use joints::{
    Connector, Coordinate, Coupling, CouplingKind, Joint, JointEnd, JointKind, Motion,
};
use joints::{CouplingResidual, JointResidual, from_variable, to_variable};

/// Variables per body: a translation and a turn.
const BODY_VARS: usize = 6;

/// The dual numbers of a mate: two bodies' worth of variables, and a
/// joint's two coordinates.
pub const MATE_VARS: usize = 2 * BODY_VARS + 2;

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
    /// Every mate holds (to [`crate::RELATIVE_TOLERANCE`] of the size).
    pub converged: bool,
    /// The largest remaining mate residual, in units of length: an upper
    /// bound.
    #[serde(with = "as_f64")]
    pub max_residual: S,
    pub iterations: usize,
    /// Each minimization of the solve.
    pub phases: Vec<crate::Phase<S>>,
    /// The mates left unsatisfied — conflicting, or unreachable from where
    /// the bodies started — by index among the constraints, then the
    /// joints, then the couplings (see [`Assembly::mates`]); empty when
    /// `converged`.
    pub failed: Vec<usize>,
    /// The joints' coordinates the solve stopped at one of their limits.
    pub at_limit: Vec<(usize, Motion)>,
}

/// How free the bodies of an assembly are, where they are now, to first
/// order.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Freedom {
    /// Per body, how many independent ways it can still move, the others
    /// moving with it as the mates need: 6 for one nothing holds, 1 for a
    /// crank, 0 for one held fast — or fixed.
    pub bodies: Vec<usize>,
    /// How many independent ways the assembly as a whole can move.
    pub total: usize,
}

/// Bodies, the constraints and joints between them, and the couplings
/// between the joints.
#[derive(Clone, Debug, PartialEq)]
pub struct Assembly<S: Scalar> {
    pub bodies: Vec<Body<S>>,
    pub constraints: Vec<Constraint<S>>,
    pub joints: Vec<Joint<S>>,
    pub couplings: Vec<Coupling<S>>,
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

/// The largest turn, in degrees, a held joint is driven by in one solve
/// (see [`Assembly::drive`]): how hard a solve tries, never what its answer
/// means.
const TURN_STEP: f64 = 30.0;

/// The residuals of an assembly, kept alive for a [`System`] borrowing
/// them, and where each joint's coordinates are among its parameters.
struct Residuals<S: Scalar> {
    mates: Vec<Mate<S>>,
    joints: Vec<JointResidual<S>>,
    couplings: Vec<CouplingResidual<S>>,
    /// Per joint and motion: the parameter its coordinate is, if its kind
    /// frees that motion.
    coordinates: Vec<[Option<usize>; 2]>,
}

impl<S: Scalar> Assembly<S> {
    /// An assembly of `bodies` and `constraints` alone, of size `scale`.
    pub fn new(bodies: Vec<Body<S>>, constraints: Vec<Constraint<S>>, scale: S) -> Self {
        Assembly {
            bodies,
            constraints,
            joints: Vec::new(),
            couplings: Vec::new(),
            scale,
        }
    }

    /// How many mates there are: constraints, joints and couplings — what
    /// [`SolveReport::failed`] counts in.
    pub fn mates(&self) -> usize {
        self.constraints.len() + self.joints.len() + self.couplings.len()
    }

    /// Checks every mate can hold between its features at all, and refers
    /// to bodies and joints there are.
    pub fn validate(&self) -> GeopResult<()> {
        let body_ok = |what: &str, i: usize, body: Option<usize>| match body {
            Some(b) if b >= self.bodies.len() => Err(GeopError::new(format!(
                "{what} {i} refers to body {b}, but there are {} bodies",
                self.bodies.len()
            ))),
            _ => Ok(()),
        };
        for (i, c) in self.constraints.iter().enumerate() {
            for body in [c.a.body, c.b.body] {
                body_ok("constraint", i, body)?;
            }
            c.validate()
                .map_err(|e| e.with_context(format!("constraint {i}")))?;
        }
        for (i, j) in self.joints.iter().enumerate() {
            for body in [j.a.body, j.b.body] {
                body_ok("joint", i, body)?;
            }
            j.validate()
                .map_err(|e| e.with_context(format!("joint {i}")))?;
        }
        for (i, c) in self.couplings.iter().enumerate() {
            c.validate(&self.joints)
                .map_err(|e| e.with_context(format!("coupling {i}")))?;
        }
        Ok(())
    }

    /// The residuals of every mate. The coordinates its joints' kinds free
    /// are parameters after the bodies', in order.
    fn residuals(&self) -> GeopResult<Residuals<S>> {
        let mut next = self.bodies.len();
        let coordinates: Vec<[Option<usize>; 2]> = self
            .joints
            .iter()
            .map(|j| {
                Motion::ALL.map(|m| {
                    j.kind.moves(m).then(|| {
                        next += 1;
                        next - 1
                    })
                })
            })
            .collect();
        let joints = self
            .joints
            .iter()
            .zip(&coordinates)
            .map(|(j, c)| JointResidual::new(*j, *c, self.scale))
            .collect::<GeopResult<_>>()?;
        let param = |joint: usize, motion: Motion| coordinates[joint][motion as usize];
        let couplings = self
            .couplings
            .iter()
            .map(|c| CouplingResidual::new(c, &self.joints, &param, self.scale))
            .collect::<GeopResult<_>>()?;
        Ok(Residuals {
            mates: self
                .constraints
                .iter()
                .map(|&c| Mate::new(c, self.scale))
                .collect(),
            joints,
            couplings,
            coordinates,
        })
    }

    /// The system of the bodies and the joints' coordinates, and the
    /// residuals of every mate.
    fn system<'m>(&self, residuals: &'m Residuals<S>) -> GeopResult<System<'m, S, MATE_VARS>> {
        let mut params: Vec<Param<S>> = self
            .bodies
            .iter()
            .map(|b| Param::Pose {
                pose: b.pose,
                center: b.center,
            })
            .collect();
        let mut free: Vec<bool> = self.bodies.iter().map(|b| b.free).collect();
        for (joint, coordinates) in self.joints.iter().zip(&residuals.coordinates) {
            for (k, motion) in Motion::ALL.into_iter().enumerate() {
                if coordinates[k].is_some() {
                    let c = joint.coordinate(motion);
                    params.push(Param::Scalar(to_variable(motion, c.value, self.scale)?));
                    free.push(!c.held);
                }
            }
        }
        let mut all: Vec<&dyn Residual<S, MATE_VARS>> = Vec::new();
        all.extend(residuals.mates.iter().map(|m| m as &dyn Residual<S, MATE_VARS>));
        all.extend(residuals.joints.iter().map(|m| m as &dyn Residual<S, MATE_VARS>));
        all.extend(
            residuals
                .couplings
                .iter()
                .map(|m| m as &dyn Residual<S, MATE_VARS>),
        );
        Ok(System {
            params,
            free,
            residuals: all,
            scale: self.scale,
        })
    }

    /// Moves the free bodies, and the joints' coordinates that are not
    /// held, so every mate holds, changing them as little as the mates
    /// allow — pulled as `pulls` ask (see the crate docs) — and keeping
    /// every coordinate within its limits.
    ///
    /// A limit is an inequality, which the solve keeps by an active set:
    /// solved without them first, every coordinate that ends up beyond one
    /// of its limits is held at that limit, and the solve starts over from
    /// where the bodies were — so a drag past a joint's limit stops the
    /// joint at it, and the rest of the drag moves what else can move. A
    /// coordinate held at a limit stays held for the rest of the solve; the
    /// next solve starts with it free again.
    ///
    /// The bodies are moved even if the solve does not converge, to the
    /// closest configuration found — the report says which mates could not
    /// be met.
    pub fn solve(&mut self, pulls: &[Pull<S>]) -> GeopResult<SolveReport<S>> {
        self.validate()?;
        self.seed()?;
        let start = self.clone();
        let mut at_limit: Vec<(usize, Motion, S)> = Vec::new();
        loop {
            *self = start.clone();
            for &(joint, motion, bound) in &at_limit {
                *self.joints[joint].coordinate_mut(motion) = Coordinate {
                    value: bound,
                    held: true,
                };
            }
            let report = self.drive(pulls)?;
            let beyond = self.beyond_limits();
            if beyond.is_empty() {
                // Held at a limit for this solve only.
                for &(joint, motion, _) in &at_limit {
                    self.joints[joint].coordinate_mut(motion).held =
                        start.joints[joint].coordinate(motion).held;
                }
                return Ok(SolveReport {
                    at_limit: at_limit.iter().map(|&(j, m, _)| (j, m)).collect(),
                    ..report
                });
            }
            at_limit.extend(beyond);
        }
    }

    /// The pose of `body`, or the ground's.
    fn pose_of(&self, body: Option<usize>) -> Pose<S> {
        body.map_or(Pose::identity(), |b| self.bodies[b].pose)
    }

    /// The angle a joint is turned to, measured (see [`Joint::measure`]) —
    /// of the angles a whole number of turns apart that put its bodies
    /// there, the one nearest `near`, in degrees.
    fn measured_turn(&self, joint: &Joint<S>, near: f64) -> f64 {
        let measured = joint.measure(|b| self.pose_of(b))[0];
        measured + 360.0 * ((near - measured) / 360.0).round()
    }

    /// Starts the free coordinates of every joint that does not hold where
    /// its bodies are (see [`Joint::measure`]) — an angle at the turn
    /// nearest its value. Where the bodies are is what the poses say; a
    /// coordinate nothing holds only follows them. A seed, so a free choice:
    /// the solve finds the exact values. A joint that holds is left exactly
    /// as it is.
    fn seed(&mut self) -> GeopResult<()> {
        let failed = self.report()?.failed;
        let nc = self.constraints.len();
        for i in 0..self.joints.len() {
            if !failed.contains(&(nc + i)) {
                continue;
            }
            let joint = self.joints[i];
            let [_, distance] = joint.measure(|b| self.pose_of(b));
            let turn = self.measured_turn(&joint, joint.angle.value.to_f64());
            for motion in joint.kind.motions() {
                let c = self.joints[i].coordinate_mut(motion);
                if !c.held {
                    c.value = S::from_f64(match motion {
                        Motion::Turn => turn,
                        Motion::Slide => distance,
                    });
                }
            }
        }
        Ok(())
    }

    /// A solve (see [`Assembly::solve_once`]) that drives every held angle
    /// further than [`TURN_STEP`] from where its bodies are there in steps
    /// no larger, the rest of the mechanism following each — a joint set
    /// half a turn away cannot be reached in one (see [`Joint::measure`]),
    /// and a linkage jumping there could fold over into another of its
    /// configurations. The pulls pull in the last step only.
    fn drive(&mut self, pulls: &[Pull<S>]) -> GeopResult<SolveReport<S>> {
        let targets: Vec<(usize, f64, S)> = self
            .joints
            .iter()
            .enumerate()
            .filter(|(_, j)| j.kind.moves(Motion::Turn) && j.angle.held)
            .map(|(i, j)| {
                let to = j.angle.value.to_f64();
                (i, self.measured_turn(j, to), j.angle.value)
            })
            .collect();
        let steps = targets
            .iter()
            .map(|&(_, from, to)| ((to.to_f64() - from).abs() / TURN_STEP).ceil() as usize)
            .max()
            .unwrap_or(0);
        for k in 1..steps {
            for &(i, from, to) in &targets {
                // Where a step goes is a free choice: any angle between.
                let at = from + (to.to_f64() - from) * k as f64 / steps as f64;
                self.joints[i].angle.value = S::from_f64(at);
            }
            self.solve_once(&[])?;
        }
        for &(i, _, to) in &targets {
            self.joints[i].angle.value = to;
        }
        self.solve_once(pulls)
    }

    /// The free coordinates beyond one of their limits, each with that
    /// limit.
    fn beyond_limits(&self) -> Vec<(usize, Motion, S)> {
        let mut beyond = Vec::new();
        for (i, joint) in self.joints.iter().enumerate() {
            for motion in joint.kind.motions() {
                let c = joint.coordinate(motion);
                if c.held {
                    continue;
                }
                match joint.kind.limits(motion) {
                    [Some(min), _] if c.value.definitely_less(min) => {
                        beyond.push((i, motion, min))
                    }
                    [_, Some(max)] if c.value.definitely_greater(max) => {
                        beyond.push((i, motion, max))
                    }
                    _ => {}
                }
            }
        }
        beyond
    }

    /// One solve, limits left out (see [`Assembly::solve`]).
    fn solve_once(&mut self, pulls: &[Pull<S>]) -> GeopResult<SolveReport<S>> {
        let residuals = self.residuals()?;
        let mut system = self.system(&residuals)?;
        let before = system.params.clone();
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
        for (joint, coordinates) in self.joints.iter_mut().zip(&residuals.coordinates) {
            for (k, motion) in Motion::ALL.into_iter().enumerate() {
                let Some(p) = coordinates[k] else {
                    continue;
                };
                // Moved only if the solve moved it: a coordinate it left
                // keeps its value exactly, not converted there and back.
                if let (Param::Scalar(v), Param::Scalar(was)) = (system.params[p], before[p])
                    && !(v.is_sharp() && was.is_sharp() && v.could_be_equal(was))
                {
                    // Where the solve put it is state, a free choice (see
                    // `System::minimize`): sharp.
                    joint.coordinate_mut(motion).value =
                        from_variable(motion, v, self.scale)?.sharpen();
                }
            }
        }
        Ok(SolveReport {
            converged: report.converged,
            max_residual: report.max_residual,
            iterations: report.iterations,
            phases: report.phases,
            failed: report.failed,
            at_limit: Vec::new(),
        })
    }

    /// Which mates hold where the bodies are now.
    pub fn report(&self) -> GeopResult<SolveReport<S>> {
        let residuals = self.residuals()?;
        let report = self.system(&residuals)?.report()?;
        Ok(SolveReport {
            converged: report.converged,
            max_residual: report.max_residual,
            iterations: 0,
            phases: Vec::new(),
            failed: report.failed,
            at_limit: Vec::new(),
        })
    }

    /// How free every body is where the bodies are now (see [`Freedom`]):
    /// the directions in which every mate's residuals stay put to first
    /// order — the Jacobian's null space — and of those, how many move each
    /// body.
    pub fn freedom(&self) -> GeopResult<Freedom> {
        self.validate()?;
        let residuals = self.residuals()?;
        let system = self.system(&residuals)?;
        let basis = system.null_space();
        let bodies = (0..self.bodies.len())
            .map(|body| match system.variables(body) {
                Some(vars) => rank(
                    basis.iter().map(|v| v[vars.clone()].to_vec()).collect(),
                    vars.len(),
                ),
                None => 0,
            })
            .collect();
        Ok(Freedom {
            bodies,
            total: basis.len(),
        })
    }

    /// The mates that cannot all hold together, by index as
    /// [`SolveReport::failed`] counts them: a smallest set — leave out any
    /// one, and the others hold — found by leaving out each mate in turn
    /// and keeping it out wherever the rest still conflict. Empty if every
    /// mate holds once solved.
    ///
    /// Each test is a solve from where the bodies are, so it costs as many
    /// solves as there are mates: for reporting a conflict, not for every
    /// drag.
    pub fn conflicting(&self) -> GeopResult<Vec<usize>> {
        self.validate()?;
        let holds = |keep: &[bool]| -> GeopResult<bool> {
            let mut subset = self.subset(keep);
            Ok(subset.solve(&[])?.converged)
        };
        let mut keep = vec![true; self.mates()];
        if holds(&keep)? {
            return Ok(Vec::new());
        }
        for i in 0..keep.len() {
            keep[i] = false;
            if holds(&keep)? {
                keep[i] = true;
            }
        }
        Ok((0..keep.len()).filter(|&i| keep[i]).collect())
    }

    /// The assembly with only the mates `keep` says — by index as
    /// [`SolveReport::failed`] counts them. A coupling of a joint left out
    /// is left out too.
    fn subset(&self, keep: &[bool]) -> Self {
        let (nc, nj) = (self.constraints.len(), self.joints.len());
        let mut index = vec![None; nj];
        let mut joints = Vec::new();
        for (i, joint) in self.joints.iter().enumerate() {
            if keep[nc + i] {
                index[i] = Some(joints.len());
                joints.push(*joint);
            }
        }
        Assembly {
            bodies: self.bodies.clone(),
            constraints: (0..nc)
                .filter(|&i| keep[i])
                .map(|i| self.constraints[i])
                .collect(),
            couplings: self
                .couplings
                .iter()
                .enumerate()
                .filter(|(i, _)| keep[nc + nj + i])
                .filter_map(|(_, c)| {
                    Some(Coupling {
                        kind: c.kind,
                        a: index[c.a]?,
                        b: index[c.b]?,
                    })
                })
                .collect(),
            joints,
            scale: self.scale,
        }
    }
}

#[cfg(test)]
#[path = "mates_tests.rs"]
mod tests;
