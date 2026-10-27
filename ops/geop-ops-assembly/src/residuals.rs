//! What a constraint mate asks of the geometry of two placed parts, as
//! residuals: a point, a line or a plane of each — where its part puts it
//! ([`Geometry::placed`]) — and what they must be to each other
//! ([`Kind`]), each residual a length that vanishes where it holds
//! ([`residuals`]).
//!
//! Everything placed is computed in dual numbers ([`D`]), so that every
//! residual carries its derivatives (see `crate::mechanism`).
//!
//! A mate that holds a point on a curve or a face of the other part
//! ([`Kind::Path`], [`Kind::Tangent`]) asks where on it: the foot point's
//! parameters ([`Kind::feet`]) are values of the program's state, solved
//! with the poses, and its residuals are the follower's point less the foot
//! point ([`contact_residuals`]).

use geop_core_geometry::{nurb_curve::NurbCurve3D, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Motion, Pose},
    scalars::{Number, Scalar, as_f64},
    vector::Vector3,
    with_context,
};
use geop_ops_model::selection::{Role, Shape};
use serde::{Deserialize, Serialize};

pub use crate::solving::D;

/// Where a pose puts the points of a part and turns its directions — or,
/// with none, where they are: of a part that never moves.
pub(crate) struct Placing(Option<Motion<D>>);

impl Placing {
    pub(crate) fn new(pose: Option<&Pose<D>>) -> Self {
        Placing(pose.map(Pose::motion))
    }

    /// The point `at` of the part, where the part is.
    pub(crate) fn point(&self, at: &Vector3) -> Vector3<D> {
        self.moved(&at.map(D::cst))
    }

    /// The point `at` of the part, carrying derivatives of its own, where
    /// the part is.
    pub(crate) fn moved(&self, at: &Vector3<D>) -> Vector3<D> {
        match &self.0 {
            Some(m) => m.apply(at),
            None => *at,
        }
    }

    /// The direction `d` of the part, turned as the part is.
    pub(crate) fn turn(&self, d: &Vector3<D>) -> Vector3<D> {
        match &self.0 {
            Some(m) => m.rotate(d),
            None => *d,
        }
    }
}

/// The rows that keep `along` parallel to the direction `across` is
/// square to — facing either way: its parts along the two directions of
/// `across`, each scaled by the part's size `scale`.
///
/// Two rows, of rank two wherever they nearly are parallel. A cross
/// product, three rows, has rank three while they are not quite parallel
/// and two once they are, and a row whose slope fades as the mate closes
/// is one the minimizer can neither drop nor resolve (a 6-axis arm
/// stalled on it).
pub(crate) fn parallel(across: &[Vector3<D>; 2], along: &Vector3<D>, scale: D) -> [D; 2] {
    across.map(|x| x.prod_dot(along).mul(scale))
}

/// A point, a line or a plane of a part, in its own frame — or what a
/// point is kept on: a curve, a face's surface.
#[derive(Clone, Debug, PartialEq)]
pub enum Geometry {
    Point {
        at: Vector3,
    },
    /// The line through `point` along `direction` (any non-zero length).
    Line {
        point: Vector3,
        direction: Vector3,
    },
    /// The plane through `point` normal to `normal` (any non-zero length).
    Plane {
        point: Vector3,
        normal: Vector3,
    },
    /// A circle's center and its radius: what is kept that far off a face
    /// it rolls on.
    Rim {
        center: Vector3,
        radius: Scalar,
    },
    /// A curve: an edge's, a sketch's.
    Curve {
        curve: NurbCurve3D,
    },
    /// A face's surface, its normal out of its solid.
    Surface {
        surface: NurbSurface3D,
    },
}

impl Geometry {
    fn kind(&self) -> &'static str {
        match self {
            Geometry::Point { .. } => "a point",
            Geometry::Line { .. } => "a line",
            Geometry::Plane { .. } => "a plane",
            Geometry::Rim { .. } => "a circle",
            Geometry::Curve { .. } => "a curve",
            Geometry::Surface { .. } => "a face",
        }
    }

    /// What `shape` gives used as `role`, if it fits it: a point — the
    /// center of a circle is one — a line, an axis as a line, a plane.
    fn as_role(role: Role, shape: &Shape) -> Option<Geometry> {
        let line = |a: &geop_core_geometry::shape::Axis| Geometry::Line {
            point: a.point,
            direction: a.direction,
        };
        match role {
            Role::Point => (shape.point.as_ref())
                .or(shape.arc.as_ref().map(|a| &a.circle.center))
                .map(|at| Geometry::Point { at: *at }),
            Role::Line => shape.line.as_ref().map(line),
            Role::Round => shape.round.as_ref().map(line),
            Role::Plane => shape.plane.as_ref().map(|p| Geometry::Plane {
                point: *p.origin(),
                normal: *p.w(),
            }),
            _ => None,
        }
    }
}

/// What a constraint asks of its two features.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Kind {
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
        value: Scalar,
    },
    /// The lines (or planes' normals) meet at `value` degrees.
    Angle {
        #[serde(with = "as_f64")]
        value: Scalar,
    },
    /// The second entity keeps touching the first, a face, flat or curved:
    /// a circle — a roller's rim, a pin's — keeps its radius off the face,
    /// its center on the face's offset, a point stays on the face. A cam
    /// and its follower, a pin in a slot of any shape.
    Tangent,
    /// The second entity — a point, a circle's center — runs along the
    /// first, a curve: a path mate, a pin along a slot's center line.
    Path,
}

impl Kind {
    pub fn label(&self) -> &'static str {
        match self {
            Kind::Coincident => "Coincident",
            Kind::Concentric => "Concentric",
            Kind::Parallel => "Parallel",
            Kind::Perpendicular => "Perpendicular",
            Kind::Distance { .. } => "Distance",
            Kind::Angle { .. } => "Angle",
            Kind::Tangent => "Tangent",
            Kind::Path => "Path",
        }
    }

    /// The names of the parameters of the foot point on the first entity
    /// it is kept at, each a value of the state: the curve's `t`, the
    /// face's `u` and `v`. None for a kind that keeps no point on a curve
    /// or a face.
    pub fn feet(&self) -> &'static [&'static str] {
        match self {
            Kind::Path => &["t"],
            Kind::Tangent => &["u", "v"],
            _ => &[],
        }
    }

    /// What an entity of a constraint of this kind can be, in the order its
    /// geometry is taken (see [`Kind::geometry`]): for one that touches or
    /// keeps a distance, a point before a line before a plane before a round
    /// face's axis; for a concentric one, an axis; for one that turns, a
    /// direction.
    ///
    /// A flat face is its plane before it is anything round: the cap of a
    /// revolved cylinder is a disc, a surface of revolution round the
    /// cylinder's axis, and a part standing on it stands on the plane, not
    /// with the axis in it. Only a concentric mate wants the disc's axis.
    pub fn roles(&self) -> &'static [Role] {
        match self {
            Kind::Coincident | Kind::Distance { .. } => {
                &[Role::Point, Role::Line, Role::Plane, Role::Round]
            }
            Kind::Concentric => &[Role::Round, Role::Line],
            Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. } => {
                &[Role::Line, Role::Plane, Role::Round]
            }
            Kind::Tangent => &[Role::Circle, Role::Point],
            Kind::Path => &[Role::Curve, Role::Circle, Role::Point],
        }
    }

    /// The geometry the `k`th entity of a constraint of this kind
    /// contributes, `shape` — as the first of its [`Kind::roles`] it fits —
    /// and `surface`, the surface of a face, asked of a tangent's first
    /// only: of a tangent constraint, the
    /// face first and then the circle or the point that touches it; of a
    /// path, the curve and then the point that runs along it.
    pub fn geometry(
        &self,
        k: usize,
        shape: &Shape,
        surface: impl FnOnce() -> GeopResult<Option<NurbSurface3D>>,
    ) -> GeopResult<Option<Geometry>> {
        Ok(match (self, k) {
            (Kind::Tangent, 0) => surface()?.map(|surface| Geometry::Surface { surface }),
            (Kind::Tangent, _) => (shape.arc.as_ref())
                .map(|arc| Geometry::Rim {
                    center: arc.circle.center,
                    radius: arc.circle.radius,
                })
                .or_else(|| Geometry::as_role(Role::Point, shape)),
            (Kind::Path, 0) => (shape.curve.clone()).map(|curve| Geometry::Curve { curve }),
            (Kind::Path, _) => Geometry::as_role(Role::Point, shape),
            _ => (self.roles().iter()).find_map(|&role| Geometry::as_role(role, shape)),
        })
    }

    /// What the `k`th entity of a constraint of this kind has to be, in
    /// words, where it is not what [`Kind::roles`] says: of a tangent and of
    /// a path, each entity something of its own.
    pub fn needs(&self, k: usize) -> Option<&'static str> {
        match (self, k) {
            (Kind::Tangent, 0) => Some("a face"),
            (Kind::Tangent, _) => Some("a circle, an arc or a point"),
            (Kind::Path, 0) => Some("an edge or a curve"),
            (Kind::Path, _) => Some("a point or a circle"),
            _ => None,
        }
    }

    /// Checks that it can hold between `a` and `b` at all: a distance
    /// between two lines, say, but no angle at a point.
    pub fn fits(&self, a: &Geometry, b: &Geometry) -> GeopResult<()> {
        use Geometry::*;
        let directed = |g: &Geometry| !matches!(g, Point { .. });
        let flat = |g: &Geometry| matches!(g, Point { .. } | Line { .. } | Plane { .. });
        let fits = match self {
            Kind::Tangent => matches!(a, Surface { .. }) && matches!(b, Point { .. } | Rim { .. }),
            Kind::Path => matches!(a, Curve { .. }) && matches!(b, Point { .. }),
            _ if !(flat(a) && flat(b)) => false,
            Kind::Coincident | Kind::Distance { .. } => true,
            Kind::Concentric => matches!((a, b), (Line { .. }, Line { .. })),
            Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. } => directed(a) && directed(b),
        };
        let (a, b) = (a.kind(), b.kind());
        match fits {
            true => Ok(()),
            false => Err(GeopError::new(format!(
                "a {} constraint cannot hold between {} and {}",
                self.label().to_lowercase(),
                a,
                b
            ))),
        }
    }
}

/// A line's direction or a plane's normal where it is, and two directions
/// square to it and to each other.
pub struct Direction {
    along: Vector3<D>,
    across: [Vector3<D>; 2],
}

/// A feature where its part puts it.
pub enum World {
    Point(Vector3<D>),
    Line(Vector3<D>, Direction),
    Plane(Vector3<D>, Direction),
}

impl World {
    /// Its direction: a line's, a plane's normal.
    fn direction(&self) -> Option<&Direction> {
        match self {
            World::Point(_) => None,
            World::Line(_, d) | World::Plane(_, d) => Some(d),
        }
    }
}

impl Geometry {
    /// Where `pose` puts it — or, with none, where it is: of a part that
    /// never moves. The directions square to a line or a plane are chosen
    /// in the part's own frame — a free choice — and turn with it.
    pub fn placed(&self, pose: Option<&Pose<D>>) -> GeopResult<World> {
        let placing = Placing::new(pose);
        let direction = |d: &Vector3| -> GeopResult<Direction> {
            let d = d.map(D::cst).normalize()?;
            Ok(Direction {
                along: placing.turn(&d),
                across: d.across()?.map(|v| placing.turn(&v)),
            })
        };
        Ok(match self {
            Geometry::Point { at } => World::Point(placing.point(at)),
            Geometry::Line {
                point: p,
                direction: d,
            } => World::Line(placing.point(p), direction(d)?),
            Geometry::Plane {
                point: p,
                normal: n,
            } => World::Plane(placing.point(p), direction(n)?),
            Geometry::Rim { center, .. } => World::Point(placing.point(center)),
            Geometry::Curve { .. } | Geometry::Surface { .. } => {
                return Err(GeopError::new(format!(
                    "{} is placed by where on it its foot point is",
                    self.kind()
                )));
            }
        })
    }

    /// The point of a follower — a point, a circle's center — where `pose`
    /// puts it.
    fn follower(&self, pose: Option<&Pose<D>>) -> GeopResult<Vector3<D>> {
        match self {
            Geometry::Point { at } | Geometry::Rim { center: at, .. } => {
                Ok(Placing::new(pose).point(at))
            }
            _ => Err(GeopError::new(format!("{} follows nothing", self.kind()))),
        }
    }

    /// How far a follower keeps off what it touches: a circle its radius.
    fn radius(&self) -> Option<Scalar> {
        match self {
            Geometry::Rim { radius, .. } => Some(*radius),
            _ => None,
        }
    }

    /// The point of a curve at `feet`, `[t]`, or of a surface at `[u, v]`
    /// — off it by `offset` along its normal — in the part's own frame,
    /// with the derivatives the feet carry. Past the domain's end, which a
    /// solve may try on its way, the curve or surface goes on along its
    /// tangents there: the feet's limits keep a solution within (see
    /// [`Geometry::domains`]).
    fn foot(&self, feet: &[D], offset: Option<Scalar>) -> GeopResult<Vector3<D>> {
        match (self, feet) {
            (Geometry::Curve { curve }, [t]) => {
                let (lo, hi) = curve.domain();
                let at = t.v.clamped(lo, hi);
                let (p, dp) = (curve.evaluate(at)?, curve.tangent(at)?);
                let past = t.sub(D::cst(at));
                Ok(first_order(&p, &[(&dp, &past)]))
            }
            (Geometry::Surface { surface }, [u, v]) => {
                let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
                let (uc, vc) = (u.v.clamped(u0, u1), v.v.clamped(v0, v1));
                let (du, dv) = (u.sub(D::cst(uc)), v.sub(D::cst(vc)));
                let order = if offset.is_some() { 2 } else { 1 };
                let s = surface.partials(uc, vc, order)?;
                let at = first_order(&s[0][0], &[(&s[1][0], &du), (&s[0][1], &dv)]);
                let Some(offset) = offset else {
                    return Ok(at);
                };
                // The normal, its derivatives by the second partials: of
                // the tangents, each to first order.
                let su = first_order(&s[1][0], &[(&s[2][0], &du), (&s[1][1], &dv)]);
                let sv = first_order(&s[0][1], &[(&s[1][1], &du), (&s[0][2], &dv)]);
                let normal = su.prod_cross(&sv).normalize()?;
                Ok(at.add(&normal.prod_scalar(D::cst(offset))))
            }
            _ => Err(GeopError::new(format!(
                "{} has no foot point at {} parameters",
                self.kind(),
                feet.len()
            ))),
        }
    }

    /// The domains of its foot point's parameters: a curve's `t`, a
    /// surface's `u` and `v`.
    pub fn domains(&self) -> Vec<(Scalar, Scalar)> {
        match self {
            Geometry::Curve { curve } => vec![curve.domain()],
            Geometry::Surface { surface } => vec![surface.domain_u(), surface.domain_v()],
            _ => Vec::new(),
        }
    }
}

/// `value` plus each `(slope, by)` pair's slope times the step `by`, to
/// first order: a point of a curve or a surface, by its parameters' steps.
fn first_order(value: &Vector3, steps: &[(&Vector3, &D)]) -> Vector3<D> {
    Vector3::from_array(std::array::from_fn(|i| {
        (steps.iter()).fold(D::cst(value[i]), |sum, (slope, by)| {
            sum.add(by.mul(D::cst(slope[i])))
        })
    }))
}

/// The residuals of a constraint of `kind` that keeps the follower `b` on
/// `a` — a point on a curve, a circle or a point touching a face — at the
/// foot point `feet` (see [`Kind::feet`]), where `pa` and `pb` put them:
/// the follower's point less the foot point, three lengths.
pub fn contact_residuals(
    kind: &Kind,
    a: &Geometry,
    pa: Option<&Pose<D>>,
    b: &Geometry,
    pb: Option<&Pose<D>>,
    feet: &[D],
) -> GeopResult<Vec<D>> {
    let ctx = with_context!("a {} constraint", kind.label().to_lowercase());
    let foot = a.foot(feet, b.radius()).with_context(ctx)?;
    let on = Placing::new(pa).moved(&foot);
    Ok(b.follower(pb)
        .with_context(ctx)?
        .sub(&on)
        .to_array()
        .to_vec())
}

/// Where on `a` the foot point of the follower `b` is, where `pa` and `pb`
/// put them: of a curve its `t`, of a surface its `u` and `v`, nearest the
/// follower's point. A seed for a solve, which finds the true one: a free
/// choice.
pub fn seed_feet(
    a: &Geometry,
    pa: Option<&Pose>,
    b: &Geometry,
    pb: Option<&Pose>,
) -> GeopResult<Vec<Scalar>> {
    let at = match b {
        Geometry::Point { at } | Geometry::Rim { center: at, .. } => *at,
        _ => return Err(GeopError::new(format!("{} follows nothing", b.kind()))),
    };
    let world = pb.map_or(at, |pose| pose.apply(&at));
    let local = pa.map_or(world, |pose| pose.inverse().apply(&world));
    match a {
        Geometry::Curve { curve } => Ok(vec![curve.closest_point(&local)?.0]),
        Geometry::Surface { surface } => {
            let (u, v, _) = surface.closest_point(&local)?;
            Ok(vec![u, v])
        }
        _ => Err(GeopError::new(format!("{} has no foot point", a.kind()))),
    }
}

/// The residuals of a constraint of `kind` between `a` and `b`, where they
/// are, each a length — a direction's scaled by the part's size `scale`.
pub fn residuals(kind: &Kind, a: &World, b: &World, scale: D) -> GeopResult<Vec<D>> {
    use World::*;
    let l = scale;
    let mut out = Vec::new();
    // Lines and planes run parallel — facing either way — where the one's
    // direction has no part across the other's.
    let parallel = |d: &Direction, e: &Direction, out: &mut Vec<D>| {
        out.extend(self::parallel(&d.across, &e.along, l))
    };
    // How far `p` is off the line through `q` along `d`: as two rows across
    // it, for the same reason, and as a distance.
    let off_line = |p: &Vector3<D>, q: &Vector3<D>, d: &Direction, out: &mut Vec<D>| {
        out.extend(d.across.map(|x| x.prod_dot(&p.sub(q))))
    };
    let distance_off_line =
        |p: &Vector3<D>, q: &Vector3<D>, d: &Direction| p.sub(q).prod_cross(&d.along).norm();
    let along = |p: &Vector3<D>, q: &Vector3<D>, n: &Direction| n.along.prod_dot(&p.sub(q));
    let dot = |d: &Direction, e: &Direction| d.along.prod_dot(&e.along).mul(l);
    let directions = || -> GeopResult<(&Direction, &Direction)> {
        match (a.direction(), b.direction()) {
            (Some(d), Some(e)) => Ok((d, e)),
            _ => Err(GeopError::new(format!(
                "a {} constraint needs two directions",
                kind.label().to_lowercase()
            ))),
        }
    };
    match kind {
        Kind::Coincident => match (a, b) {
            (Point(p), Point(q)) => out.extend(p.sub(q).to_array()),
            (Point(p), Line(q, d)) | (Line(q, d), Point(p)) => off_line(p, q, d, &mut out),
            (Point(p), Plane(q, n)) | (Plane(q, n), Point(p)) => out.push(along(p, q, n)),
            (Line(p, d), Line(q, e)) => {
                parallel(d, e, &mut out);
                off_line(q, p, d, &mut out);
            }
            (Line(p, d), Plane(q, n)) | (Plane(q, n), Line(p, d)) => {
                out.push(dot(d, n));
                out.push(along(p, q, n));
            }
            (Plane(p, n), Plane(q, m)) => {
                parallel(n, m, &mut out);
                out.push(along(q, p, n));
            }
        },
        // Two lines share their axis where they coincide.
        Kind::Concentric => match (a, b) {
            (Line(..), Line(..)) => return residuals(&Kind::Coincident, a, b, scale),
            _ => return Err(GeopError::new("a concentric constraint needs two lines")),
        },
        Kind::Parallel | Kind::Perpendicular => {
            let (d, e) = directions()?;
            // A line along a plane runs square to its normal, and a line
            // square to a plane runs along its normal: mixed pairs swap.
            let mixed = matches!((a, b), (Line(..), Plane(..)) | (Plane(..), Line(..)));
            match matches!(kind, Kind::Parallel) != mixed {
                true => parallel(d, e, &mut out),
                false => out.push(dot(d, e)),
            }
        }
        Kind::Distance { value } => {
            let value = D::cst(*value);
            match (a, b) {
                (Line(p, d), Line(q, e)) => {
                    parallel(d, e, &mut out);
                    out.push(distance_off_line(q, p, d).sub(value));
                }
                (Line(p, d), Plane(q, n)) | (Plane(q, n), Line(p, d)) => {
                    out.push(dot(d, n));
                    out.push(along(p, q, n).abs().sub(value));
                }
                (Plane(p, n), Plane(q, m)) => {
                    parallel(n, m, &mut out);
                    out.push(along(q, p, n).abs().sub(value));
                }
                (Point(p), Point(q)) => out.push(p.sub(q).norm().sub(value)),
                (Point(p), Line(q, d)) | (Line(q, d), Point(p)) => {
                    out.push(distance_off_line(p, q, d).sub(value))
                }
                (Point(p), Plane(q, n)) | (Plane(q, n), Point(p)) => {
                    out.push(along(p, q, n).abs().sub(value))
                }
            }
        }
        // One condition, so one row: `d·e = cos θ`, divided by `sin θ`, its
        // slope there, so that the row is a length per turn like every
        // other. The length of a cross product, as a second row, has no
        // slope at all where the directions are parallel, and the two rows
        // together say one thing twice. At 0° or 180° the condition is two —
        // the directions parallel — and `d·e` has no slope there either:
        // those are held as parallel ones are.
        Kind::Angle { value } => {
            let (d, e) = directions()?;
            let radians = D::cst(*value).degrees_to_radians()?;
            let sin = radians.sin();
            if sin.could_be_equal(D::ZERO) {
                parallel(d, e, &mut out);
            } else {
                let cos = d.along.prod_dot(&e.along).sub(radians.cos());
                out.push(cos.mul(D::ONE.div(sin)?).mul(l));
            }
        }
        Kind::Tangent | Kind::Path => {
            return Err(GeopError::new(format!(
                "a {} constraint holds where its foot point is: see `contact_residuals`",
                kind.label().to_lowercase()
            )));
        }
    }
    Ok(out)
}
