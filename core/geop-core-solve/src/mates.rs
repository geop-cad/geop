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

/// Two unit directions square to the unit vector `axis` and to each other:
/// of the axes `x`, `y`, `z`, the one that runs least along `axis`, made
/// square to it — a free choice, and the well-conditioned one — and `axis`
/// crossed with that.
pub(crate) fn across<S: Scalar>(axis: &Vector3<S>) -> GeopResult<[Vector3<S>; 2]> {
    let size = |k: usize| axis[k].abs().sharpen();
    let least = (1..3).fold(0, |best, k| {
        if size(k).definitely_less(size(best)) {
            k
        } else {
            best
        }
    });
    let reference = Vector3::axis(least);
    let first = reference
        .sub(&axis.prod_scalar(axis.prod_dot(&reference)))
        .normalize()
        .map_err(|e| e.with_context(format!("a direction square to {axis:?}")))?;
    Ok([first, axis.prod_cross(&first)])
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
    /// The free bodies a solve may have moved, by index: those of the
    /// groups it solved (see [`Assembly::solve`]). Every other body is
    /// exactly where it was.
    pub moved: Vec<usize>,
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

/// A line's direction or a plane's normal, where it is during a solve, and
/// two directions square to it and to each other.
struct Direction<S: Scalar> {
    along: V<S>,
    across: [V<S>; 2],
}

/// A feature, where it is during a solve.
enum World<S: Scalar> {
    Point(V<S>),
    Line(V<S>, Direction<S>),
    Plane(V<S>, Direction<S>),
}

impl<S: Scalar> World<S> {
    /// Its direction: a line's, a plane's normal.
    fn direction(&self) -> Option<&Direction<S>> {
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
        let rotate = |d: &Vector3<S>| -> V<S> {
            match placed {
                Some(p) => p.direction(&cst(d)),
                None => cst(d),
            }
        };
        // The directions square to it are chosen in the body's frame — a
        // free choice — and turn with the body.
        let direction = |d: &Vector3<S>| -> GeopResult<Direction<S>> {
            Ok(Direction {
                along: rotate(d).normalize()?,
                across: across(&d.normalize()?)?.map(|v| rotate(&v)),
            })
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
        // Lines and planes run parallel — facing either way — where the
        // one's direction has no part across the other's: two rows, of rank
        // two wherever they nearly are. A cross product, three rows, has
        // rank three while they are not quite parallel and two once they
        // are, and a row whose slope fades as the mate closes is one the
        // minimizer can neither drop nor resolve (a mated arm stalled on
        // it).
        let parallel = |d: &Direction<S>, e: &Direction<S>, out: &mut Vec<Dual<S>>| {
            out.extend(d.across.map(|x| x.prod_dot(&e.along).mul(l)))
        };
        // How far `p` is off the line through `q` along `d`: as two rows
        // across it, for the same reason, and as a distance.
        let off_line = |p: &V<S>, q: &V<S>, d: &Direction<S>, out: &mut Vec<Dual<S>>| {
            out.extend(d.across.map(|x| x.prod_dot(&p.sub(q))))
        };
        let distance_off_line =
            |p: &V<S>, q: &V<S>, d: &Direction<S>| p.sub(q).prod_cross(&d.along).norm();
        let along = |p: &V<S>, q: &V<S>, n: &Direction<S>| n.along.prod_dot(&p.sub(q));
        let dot = |d: &Direction<S>, e: &Direction<S>| d.along.prod_dot(&e.along).mul(l);
        // A point-to-thing distance.
        let gap = |a: &World<S>, b: &World<S>| -> Option<Dual<S>> {
            Some(match (a, b) {
                (Point(p), Point(q)) => p.sub(q).norm(),
                (Point(p), Line(q, d)) | (Line(q, d), Point(p)) => distance_off_line(p, q, d),
                (Point(p), Plane(q, n)) | (Plane(q, n), Point(p)) => along(p, q, n).abs(),
                _ => return None,
            })
        };
        match kind {
            Kind::Coincident => match (a, b) {
                (Point(p), Point(q)) => out.extend(p.sub(q).to_array()),
                (Point(p), Line(q, d)) | (Line(q, d), Point(p)) => off_line(p, q, d, out),
                (Point(p), Plane(q, n)) | (Plane(q, n), Point(p)) => out.push(along(p, q, n)),
                (Line(p, d), Line(q, e)) => {
                    parallel(d, e, out);
                    off_line(q, p, d, out);
                }
                (Line(p, d), Plane(q, n)) | (Plane(q, n), Line(p, d)) => {
                    out.push(dot(d, n));
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
                    off_line(q, p, d, out);
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
                    out.push(dot(d, e));
                }
            }
            Kind::Distance { value } => {
                let value = Dual::cst(value);
                match (a, b) {
                    (Line(p, d), Line(q, e)) => {
                        parallel(d, e, out);
                        out.push(distance_off_line(q, p, d).sub(value));
                    }
                    (Line(p, d), Plane(q, n)) | (Plane(q, n), Line(p, d)) => {
                        out.push(dot(d, n));
                        out.push(along(p, q, n).abs().sub(value));
                    }
                    (Plane(p, n), Plane(q, m)) => {
                        parallel(n, m, out);
                        out.push(along(q, p, n).abs().sub(value));
                    }
                    _ => out.push(gap(a, b).expect("a point and anything").sub(value)),
                }
            }
            // One condition, so one row: `d·e = cos θ`, divided by `sin θ`,
            // its slope there, so that the row is a length per turn like
            // every other. The length of a cross product, as a second row,
            // has no slope at all where the directions are parallel, and
            // the two rows together say one thing twice. At 0° or 180° the
            // condition is two — the directions parallel — and `d·e` has
            // no slope there either: those are held as parallel ones are.
            Kind::Angle { value } => {
                let (d, e) = (
                    a.direction().expect("validated"),
                    b.direction().expect("validated"),
                );
                let radians = value.mul(S::PI.div(S::from_i64(180))?);
                let sin = radians.sin();
                if sin.could_be_equal(S::ZERO) {
                    parallel(d, e, out);
                } else {
                    out.push(
                        d.along
                            .prod_dot(&e.along)
                            .sub(Dual::cst(radians.cos()))
                            .mul(Dual::cst(S::ONE.div(sin)?))
                            .mul(l),
                    );
                }
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

/// Free bodies and joints no mate ties to any others (see
/// [`Assembly::independent`]), and the mates that move them — by index.
#[derive(Default)]
struct Group {
    bodies: Vec<usize>,
    constraints: Vec<usize>,
    joints: Vec<usize>,
    couplings: Vec<usize>,
}

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
        all.extend(
            residuals
                .mates
                .iter()
                .map(|m| m as &dyn Residual<S, MATE_VARS>),
        );
        all.extend(
            residuals
                .joints
                .iter()
                .map(|m| m as &dyn Residual<S, MATE_VARS>),
        );
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

    /// The free bodies, and the joints with a free coordinate, in groups no
    /// mate ties together — each with the mates that move one of its bodies
    /// or coordinates, by index. A joint ties its free bodies, and a
    /// coupling its two joints, so their bodies too. A mate that moves
    /// nothing — none of its bodies free, no coordinate of it free — is in
    /// no group.
    fn independent(&self) -> Vec<Group> {
        let nb = self.bodies.len();
        // Union-find over the bodies, then the joints: each points towards
        // the root of its group.
        let mut root: Vec<usize> = (0..nb + self.joints.len()).collect();
        fn find(root: &mut [usize], mut b: usize) -> usize {
            while root[b] != b {
                root[b] = root[root[b]];
                b = root[b];
            }
            b
        }
        fn unite(root: &mut [usize], a: usize, b: usize) {
            let (a, b) = (find(root, a), find(root, b));
            root[a] = b;
        }
        let free = |bodies: [Option<usize>; 2]| {
            bodies
                .into_iter()
                .flatten()
                .filter(|&b| self.bodies[b].free)
        };
        for c in &self.constraints {
            let mut bodies = free([c.a.body, c.b.body]);
            if let (Some(a), Some(b)) = (bodies.next(), bodies.next()) {
                unite(&mut root, a, b);
            }
        }
        for (j, joint) in self.joints.iter().enumerate() {
            for b in free([joint.a.body, joint.b.body]) {
                unite(&mut root, nb + j, b);
            }
        }
        for c in &self.couplings {
            unite(&mut root, nb + c.a, nb + c.b);
        }
        // The group of each root that has a free body or coordinate.
        let mut group_of: Vec<Option<usize>> = vec![None; root.len()];
        let mut groups: Vec<Group> = Vec::new();
        let mut open = |node: usize, root: &mut [usize], groups: &mut Vec<Group>| {
            let r = find(root, node);
            *group_of[r].get_or_insert_with(|| {
                groups.push(Group::default());
                groups.len() - 1
            })
        };
        for (b, body) in self.bodies.iter().enumerate() {
            if body.free {
                let g = open(b, &mut root, &mut groups);
                groups[g].bodies.push(b);
            }
        }
        for (j, joint) in self.joints.iter().enumerate() {
            let moves = joint
                .kind
                .motions()
                .into_iter()
                .any(|m| !joint.coordinate(m).held);
            if moves {
                open(nb + j, &mut root, &mut groups);
            }
        }
        // Every mate in the group of what it moves, if that is one.
        for (i, c) in self.constraints.iter().enumerate() {
            if let Some(b) = free([c.a.body, c.b.body]).next()
                && let Some(g) = group_of[find(&mut root, b)]
            {
                groups[g].constraints.push(i);
            }
        }
        for j in 0..self.joints.len() {
            if let Some(g) = group_of[find(&mut root, nb + j)] {
                groups[g].joints.push(j);
            }
        }
        for (i, c) in self.couplings.iter().enumerate() {
            if let Some(g) = group_of[find(&mut root, nb + c.a)] {
                groups[g].couplings.push(i);
            }
        }
        groups
    }

    /// The assembly of `group` alone — the bodies its mates hold that are
    /// not free, held as they are — and, by index here, the body each of
    /// its bodies is.
    fn restricted(&self, group: &Group) -> (Assembly<S>, Vec<usize>) {
        let mut bodies: Vec<usize> = group.bodies.clone();
        let mut local = std::collections::HashMap::new();
        for (l, &b) in bodies.iter().enumerate() {
            local.insert(b, l);
        }
        let mut at = |b: Option<usize>| {
            b.map(|b| {
                *local.entry(b).or_insert_with(|| {
                    bodies.push(b);
                    bodies.len() - 1
                })
            })
        };
        let constraints = group
            .constraints
            .iter()
            .map(|&i| {
                let c = self.constraints[i];
                Constraint {
                    a: Feature {
                        body: at(c.a.body),
                        ..c.a
                    },
                    b: Feature {
                        body: at(c.b.body),
                        ..c.b
                    },
                    ..c
                }
            })
            .collect();
        let joints = group
            .joints
            .iter()
            .map(|&j| {
                let joint = self.joints[j];
                Joint {
                    a: JointEnd {
                        body: at(joint.a.body),
                        ..joint.a
                    },
                    b: JointEnd {
                        body: at(joint.b.body),
                        ..joint.b
                    },
                    ..joint
                }
            })
            .collect();
        let joint_at = |j: usize| {
            group
                .joints
                .iter()
                .position(|&k| k == j)
                .expect("a coupling's joints are in its group")
        };
        let couplings = group
            .couplings
            .iter()
            .map(|&i| {
                let c = self.couplings[i];
                Coupling {
                    a: joint_at(c.a),
                    b: joint_at(c.b),
                    ..c
                }
            })
            .collect();
        let assembly = Assembly {
            bodies: bodies.iter().map(|&b| self.bodies[b]).collect(),
            constraints,
            joints,
            couplings,
            scale: self.scale,
        };
        (assembly, bodies)
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
    /// Bodies no mate ties together move independently, so each group of
    /// them is solved on its own ([`Assembly::independent`]): a
    /// plate with hundreds of screws mated to it is hundreds of small
    /// solves, not one of hundreds of bodies. That changes nothing about
    /// the solution — what a solve minimizes is a sum over the groups, and
    /// the mates of one never involve another's bodies or coordinates. A
    /// group that nothing pulls and whose mates hold already is where its
    /// solve would leave it, and is not solved at all.
    ///
    /// The bodies are moved even if the solve does not converge, to the
    /// closest configuration found — the report says which mates could not
    /// be met. Its steps and phases are those of the groups solved side by
    /// side (see [`Assembly::independent`]): the most steps any took, and
    /// per phase, the worst.
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
                    [Some(min), _] if c.value.definitely_less(min) => beyond.push((i, motion, min)),
                    [_, Some(max)] if c.value.definitely_greater(max) => {
                        beyond.push((i, motion, max))
                    }
                    _ => {}
                }
            }
        }
        beyond
    }

    /// One solve, limits left out (see [`Assembly::solve`]): the groups no
    /// mate ties together (see [`Assembly::independent`]) one by one.
    fn solve_once(&mut self, pulls: &[Pull<S>]) -> GeopResult<SolveReport<S>> {
        let mut phases: Vec<crate::Phase<S>> = Vec::new();
        let mut iterations = 0;
        let mut moved = Vec::new();
        for group in self.independent() {
            let (mut part, bodies) = self.restricted(&group);
            let free = &group.bodies;
            let pulls: Vec<Pull<S>> = pulls
                .iter()
                .filter_map(|p| {
                    let local = |body: usize| free.iter().position(|&b| b == body);
                    Some(match *p {
                        Pull::Pose { body, target } => Pull::Pose {
                            body: local(body)?,
                            target,
                        },
                        Pull::Point {
                            body,
                            local: at,
                            target,
                        } => Pull::Point {
                            body: local(body)?,
                            local: at,
                            target,
                        },
                    })
                })
                .collect();
            if pulls.is_empty() && part.report()?.converged {
                continue;
            }
            let report = part.solve_together(&pulls)?;
            for (&b, body) in bodies.iter().zip(&part.bodies) {
                self.bodies[b].pose = body.pose;
            }
            for (&j, joint) in group.joints.iter().zip(&part.joints) {
                self.joints[j].angle = joint.angle;
                self.joints[j].distance = joint.distance;
            }
            moved.extend(free);
            iterations = iterations.max(report.iterations);
            for (k, phase) in report.phases.into_iter().enumerate() {
                match phases.get_mut(k) {
                    None => phases.push(phase),
                    Some(worst) => {
                        if phase.iterations > worst.iterations {
                            worst.stop = phase.stop;
                            worst.iterations = phase.iterations;
                        }
                        if phase.max_residual.could_be_greater(worst.max_residual) {
                            worst.max_residual = phase.max_residual;
                        }
                    }
                }
            }
        }
        moved.sort();
        Ok(SolveReport {
            iterations,
            phases,
            moved,
            ..self.report()?
        })
    }

    /// [`Assembly::solve_once`], every free body in one system.
    fn solve_together(&mut self, pulls: &[Pull<S>]) -> GeopResult<SolveReport<S>> {
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
            moved: (0..self.bodies.len())
                .filter(|&b| self.bodies[b].free)
                .collect(),
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
            moved: Vec::new(),
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
