//! What a constraint mate asks of the geometry of two placed parts, as
//! residuals: a point, a line or a plane of each — where its part puts it
//! ([`Geometry::placed`]) — and what they must be to each other
//! ([`Kind`]), each residual a length that vanishes where it holds
//! ([`residuals`]).
//!
//! Everything placed is computed in dual numbers ([`D`]), so that every
//! residual carries its derivatives (see `crate::mechanism`).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Motion, Pose},
    scalars::{Field, Ring, Scalar, as_f64},
    vector::Vector3,
};
use geop_core_program::values::Design;
use geop_ops_reference_geometry::{Role, Shape};
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
    pub(crate) fn point(&self, at: &Vector3<Design>) -> Vector3<D> {
        let at = at.map(D::cst);
        match &self.0 {
            Some(m) => m.apply(&at),
            None => at,
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

/// A point, a line or a plane of a part, in its own frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Geometry {
    Point {
        at: Vector3<Design>,
    },
    /// The line through `point` along `direction` (any non-zero length).
    Line {
        point: Vector3<Design>,
        direction: Vector3<Design>,
    },
    /// The plane through `point` normal to `normal` (any non-zero length).
    Plane {
        point: Vector3<Design>,
        normal: Vector3<Design>,
    },
}

impl Geometry {
    fn kind(&self) -> &'static str {
        match self {
            Geometry::Point { .. } => "a point",
            Geometry::Line { .. } => "a line",
            Geometry::Plane { .. } => "a plane",
        }
    }

    /// What `shape` gives used as `role`, if it fits it: a point — the
    /// center of a circle is one — a line, an axis as a line, a plane.
    fn as_role(role: Role, shape: &Shape) -> Option<Geometry> {
        let line = |a: &geop_core_geometry::shape::Axis<Design>| Geometry::Line {
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
        value: Design,
    },
    /// The lines (or planes' normals) meet at `value` degrees.
    Angle {
        #[serde(with = "as_f64")]
        value: Design,
    },
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
        }
    }

    /// What an entity of a constraint of this kind can be, in the order its
    /// geometry is taken (see [`Kind::geometry`]): for one that touches or
    /// keeps a distance, a point before a line before a plane; for a
    /// concentric one, an axis; for one that turns, a direction.
    pub fn roles(&self) -> &'static [Role] {
        match self {
            Kind::Coincident | Kind::Distance { .. } => {
                &[Role::Point, Role::Line, Role::Round, Role::Plane]
            }
            Kind::Concentric => &[Role::Round, Role::Line],
            Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. } => {
                &[Role::Line, Role::Round, Role::Plane]
            }
        }
    }

    /// The geometry `shape` contributes to a constraint of this kind: as
    /// the first of its [`Kind::roles`] it fits.
    pub fn geometry(&self, shape: &Shape) -> Option<Geometry> {
        (self.roles().iter()).find_map(|&role| Geometry::as_role(role, shape))
    }

    /// Checks that it can hold between `a` and `b` at all: a distance
    /// between two lines, say, but no angle at a point.
    pub fn fits(&self, a: &Geometry, b: &Geometry) -> GeopResult<()> {
        use Geometry::*;
        let directed = |g: &Geometry| !matches!(g, Point { .. });
        let fits = match self {
            Kind::Coincident | Kind::Distance { .. } => true,
            Kind::Concentric => matches!((a, b), (Line { .. }, Line { .. })),
            Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. } => directed(a) && directed(b),
        };
        match fits {
            true => Ok(()),
            false => Err(GeopError::new(format!(
                "a {} constraint cannot hold between {} and {}",
                self.label().to_lowercase(),
                a.kind(),
                b.kind()
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
        let direction = |d: &Vector3<Design>| -> GeopResult<Direction> {
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
        })
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
    }
    Ok(out)
}
