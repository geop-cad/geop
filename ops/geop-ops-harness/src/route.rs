//! The path a cable takes: through [`Waypoint`]s, in order, as a chain of
//! straight lines and circular arcs ([`RoutePath`]), and what it measures
//! ([`Measure`]): how long it is and how tightly it bends.
//!
//! # Why lines and arcs
//!
//! A cable is laid the way a pipe is bent: straight runs and bends of one
//! radius each. Between two waypoints, each with the direction the route
//! has there, the route is a *biarc* — two circular arcs, tangent to each
//! other where they meet and to the given directions at the ends, either
//! of them straight where the geometry is. That makes the whole route
//! tangent-continuous, and everything about it exact:
//!
//! - its **length** is the sum of chords and of radius times angle, an
//!   enclosure of the true length, not an approximation of it;
//! - its **bend radii** are the arcs' radii, so the minimum bend radius is
//!   checked exactly, arc by arc, and a violation names the two waypoints
//!   the arc lies between;
//! - the **bundle** swept along it is made of cylinders and pieces of tori,
//!   which the path sweep builds exactly (see
//!   [`geop_ops_extrude_revolve::path_sweep`]).
//!
//! # The biarc
//!
//! From `P0` heading `T0` to `P1` heading `T1` (unit vectors), the two arcs
//! meet at `J = (Q1 + Q2) / 2` with `Q1 = P0 + d T0` and `Q2 = P1 - d T1`,
//! the arcs' middle control points: the arc from `P0` to `J` has both legs
//! `d` long, as has the one from `J` to `P1`, which is what makes each a
//! circular arc. `|Q2 - Q1| = 2d` gives `d` as the positive root of
//!
//! ```text
//! 2 (T0·T1 - 1) d² - 2 v·(T0 + T1) d + v·v = 0,    v = P1 - P0,
//! ```
//!
//! which always has exactly one, since the leading coefficient is never
//! positive and the constant always is. Equal legs on both arcs is the
//! usual choice among the biarcs between two ends (it is a one-parameter
//! family); it is symmetric, and a straight route or a single arc comes out
//! as exactly that. Everything is computed in the scalars' own arithmetic,
//! so the arcs are honest enclosures, recognized as arcs by everything
//! downstream.
//!
//! # Directions
//!
//! A coordinate system says which way the cable leaves it: along its `z`
//! axis — out of a connector at the start, into one (against its `z` axis)
//! at the end, along it in between. A circular edge — a clip, a grommet —
//! says the axis the route passes along, and the route passes the way it
//! is going. Anything else — a vertex, a datum point — leaves the direction
//! free, and the route chooses it (see [`RoutePath::through`]).

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::DatumKind,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_ops::{
    Part,
    operation::{Aspects, EntityRef},
};
use geop_ops_extrude_revolve::{
    common::{arc3, line3},
    path_sweep::PathChain,
};

/// Which way the route is told to run through a [`Waypoint`].
#[derive(Clone, Debug)]
pub enum Heading<S: Scalar> {
    /// Not at all: the route chooses.
    Free,
    /// Along this axis, either way: the route passes the way it is going.
    Axis(Vector3<S>),
    /// This way: out of a connector, or along a coordinate system's `z`.
    Exit(Vector3<S>),
}

/// A point the route runs through, and which way.
#[derive(Clone, Debug)]
pub struct Waypoint<S: Scalar> {
    pub point: Vector3<S>,
    pub heading: Heading<S>,
    /// What the route calls it when it says something about it:
    /// `point 2 (clip/extrude(c,end))`.
    pub label: String,
}

impl<S: Scalar> Waypoint<S> {
    /// The `index`-th point the route runs through, `entity` of `part`: a
    /// coordinate system, its origin and `z` axis; a circular edge, its
    /// centre and axis; any other point, just where it is.
    pub fn of(part: &Part<S>, entity: &EntityRef, index: usize) -> GeopResult<Self> {
        let label = format!("point {} ({})", index + 1, entity.label());
        let ctx = with_context!("resolving the route's {label}");
        let unsupported = || {
            GeopError::new(format!(
                "route: {label} is {entity}: a route runs through points, coordinate systems (along their z axis) and circular edges (through their centre, along their axis)"
            ))
        };
        let (point, heading) = if let EntityRef::Datum { .. } = entity {
            let datum = entity.resolve_datum(part).with_context(ctx)?;
            let heading = match datum.kind {
                DatumKind::Point => Heading::Free,
                DatumKind::Frame => Heading::Exit(*datum.frame.w()),
                DatumKind::Axis | DatumKind::Plane => return Err(unsupported()),
            };
            (*datum.frame.origin(), heading)
        } else {
            let aspects = Aspects::of(entity, part).with_context(ctx)?;
            match (aspects.point, aspects.arc) {
                (Some(point), _) => (point, Heading::Free),
                (None, Some(arc)) => (arc.circle.center, Heading::Axis(arc.circle.normal)),
                (None, None) => return Err(unsupported()),
            }
        };
        Ok(Waypoint {
            point,
            heading,
            label,
        })
    }
}

/// A route: a chain of lines and arcs through waypoints, named for the
/// solid swept along it — `w{i}` the `i`-th waypoint, `m{i}` where the two
/// arcs between it and the next meet, `a{i}` the arc from `w{i}` to `m{i}`
/// (or the line from `w{i}` to `w{i+1}`, if both are straight) and `b{i}`
/// the arc from `m{i}` on.
#[derive(Clone, Debug)]
pub struct RoutePath<S: Scalar> {
    pub chain: PathChain<S>,
    /// Where each curve of the chain lies, as the route says it:
    /// `between point 1 (…) and point 2 (…)`.
    pub places: Vec<String>,
}

/// What a route measures.
#[derive(Clone, Debug)]
pub struct Measure<S: Scalar> {
    /// End to end.
    pub length: S,
    /// Every bend, in order.
    pub bends: Vec<Bend<S>>,
}

/// A bend of a route: the arc `curve` of its chain, of radius `radius`.
#[derive(Clone, Debug)]
pub struct Bend<S: Scalar> {
    pub curve: usize,
    pub radius: S,
}

impl<S: Scalar> Measure<S> {
    /// The tightest bend, if the route bends at all.
    pub fn tightest(&self) -> Option<&Bend<S>> {
        self.bends
            .iter()
            .min_by(|a, b| a.radius.to_f64().total_cmp(&b.radius.to_f64()))
    }

    /// The bends definitely tighter than `min_radius`. One that could be
    /// exactly as tight — a bend designed at the limit — is not.
    pub fn too_tight(&self, min_radius: f64) -> Vec<&Bend<S>> {
        let min = S::from_f64(min_radius);
        self.bends
            .iter()
            .filter(|b| b.radius.definitely_less(min))
            .collect()
    }
}

impl<S: Scalar> RoutePath<S> {
    /// The route through `waypoints`, in order (see the module docs). Where
    /// a waypoint leaves the direction free, the route chooses it: in
    /// between, it turns as much before as after, along the bisector of the
    /// chords to its neighbours; at an end, it bends into the next
    /// waypoint's direction in one arc — or runs straight, between two free
    /// ends.
    pub fn through(waypoints: &[Waypoint<S>]) -> GeopResult<Self> {
        let n = waypoints.len();
        if n < 2 {
            return Err(GeopError::new(format!(
                "route: pick the points the route runs through, two at least — {n} picked"
            )));
        }
        let tangents = tangents(waypoints)?;
        let label = |i: usize| &waypoints[i].label;
        let mut chain = PathChain {
            curves: Vec::new(),
            curve_names: Vec::new(),
            joint_names: Vec::new(),
        };
        let mut places = Vec::new();
        for i in 0..n - 1 {
            let ctx = with_context!("routing from {} to {}", label(i), label(i + 1));
            let (p0, p1) = (&waypoints[i].point, &waypoints[i + 1].point);
            let place = format!("between {} and {}", label(i), label(i + 1));
            let [first, second] =
                biarc(&place, p0, &tangents[i], p1, &tangents[i + 1]).with_context(ctx)?;
            let mut push = |curve: NurbCurve3D<S>, name: String, joint: String| {
                chain.curves.push(curve);
                chain.curve_names.push(name);
                chain.joint_names.push(joint);
                places.push(place.clone());
            };
            if first.straight && second.straight {
                push(line3(*p0, *p1)?, format!("a{i}"), format!("w{i}"));
            } else {
                push(first.curve, format!("a{i}"), format!("w{i}"));
                push(second.curve, format!("b{i}"), format!("m{i}"));
            }
        }
        chain.joint_names.push(format!("w{}", n - 1));
        Ok(RoutePath { chain, places })
    }

    /// How long the route is and where it bends how tightly. Its curves are
    /// lines and arcs — what [`RoutePath::through`] builds; a chain with
    /// any other curve is not measured.
    pub fn measure(&self) -> GeopResult<Measure<S>> {
        let mut length = S::ZERO;
        let mut bends = Vec::new();
        for (k, curve) in self.chain.curves.iter().enumerate() {
            let ctx = with_context!("measuring the route {}", self.places[k]);
            // An arc first: a flat one's control points could all lie on
            // its chord, but its length is not the chord's.
            if let Some(arc) = curve.as_arc().with_context(ctx)? {
                length = length.add(arc.length().with_context(ctx)?);
                bends.push(Bend {
                    curve: k,
                    radius: arc.circle.radius,
                });
            } else if curve.as_line().with_context(ctx)?.is_some() {
                let (t0, t1) = curve.domain();
                let chord = curve.evaluate(t1)?.sub(&curve.evaluate(t0)?);
                length = length.add(chord.norm());
            } else {
                return Err(GeopError::new(format!(
                    "route: the route {} is neither straight nor a circular arc, and only those are measured",
                    self.places[k]
                )));
            }
        }
        Ok(Measure { length, bends })
    }
}

/// The unit direction the route has at each waypoint (see
/// [`RoutePath::through`]).
fn tangents<S: Scalar>(waypoints: &[Waypoint<S>]) -> GeopResult<Vec<Vector3<S>>> {
    let n = waypoints.len();
    let label = |i: usize| &waypoints[i].label;
    let chords = (0..n - 1)
        .map(|i| {
            let d = waypoints[i + 1].point.sub(&waypoints[i].point);
            if d.norm_sq().could_be_equal(S::ZERO) {
                return Err(GeopError::new(format!(
                    "route: {} and {} could be one point: the route runs between points apart",
                    label(i),
                    label(i + 1)
                )));
            }
            d.normalize()
        })
        .collect::<GeopResult<Vec<_>>>()?;
    // Which way the route is going at `i`, as its neighbours say: along the
    // chords to either side.
    let going = |i: usize| match (i.checked_sub(1).map(|j| chords[j]), chords.get(i)) {
        (Some(a), Some(b)) => a.add(b),
        (Some(a), None) => a,
        (None, Some(b)) => *b,
        (None, None) => unreachable!("a route has two waypoints at least"),
    };
    let mut tangents: Vec<Option<Vector3<S>>> = Vec::with_capacity(n);
    for (i, waypoint) in waypoints.iter().enumerate() {
        tangents.push(match &waypoint.heading {
            Heading::Exit(d) if i + 1 == n => Some(d.neg().normalize()?),
            Heading::Exit(d) => Some(d.normalize()?),
            Heading::Axis(axis) => {
                let along = axis.prod_dot(&going(i));
                if along.definitely_greater(S::ZERO) {
                    Some(axis.normalize()?)
                } else if along.definitely_less(S::ZERO) {
                    Some(axis.neg().normalize()?)
                } else {
                    return Err(GeopError::new(format!(
                        "route: the axis of {} could lie square to the route, which then has no way to pass through it: move the points before or after it",
                        label(i)
                    )));
                }
            }
            Heading::Free if i == 0 || i + 1 == n => None,
            Heading::Free => {
                let bisector = going(i);
                if bisector.norm_sq().could_be_equal(S::ZERO) {
                    return Err(GeopError::new(format!(
                        "route: the route could turn right back at {}",
                        label(i)
                    )));
                }
                // A free choice, so sharp — before it is normalized: the
                // biarc takes its directions to be unit vectors, which a
                // unit vector sharpened afterwards no longer quite is.
                Some(bisector.sharpen().normalize()?)
            }
        });
    }
    // A free end bends into its neighbour's direction in one arc: it
    // leaves along that direction mirrored in the chord between them —
    // again a free choice.
    let mirrored = |t: &Vector3<S>, c: &Vector3<S>| {
        c.prod_scalar(S::TWO.mul(c.prod_dot(t)))
            .sub(t)
            .sharpen()
            .normalize()
    };
    if n == 2 && tangents.iter().all(Option::is_none) {
        return Ok(vec![chords[0]; 2]);
    }
    if tangents[0].is_none() {
        let next = tangents[1].expect("set: a free end with a free neighbour is a route of two");
        tangents[0] = Some(mirrored(&next, &chords[0])?);
    }
    if tangents[n - 1].is_none() {
        let before = tangents[n - 2].expect("set before");
        tangents[n - 1] = Some(mirrored(&before, &chords[n - 2])?);
    }
    Ok(tangents.into_iter().map(|t| t.expect("all set")).collect())
}

/// One of a biarc's two curves, and whether it is straight.
struct Piece<S: Scalar> {
    curve: NurbCurve3D<S>,
    straight: bool,
}

/// The biarc from `p0` heading `t0` to `p1` heading `t1` (see the module
/// docs), `place` being where it lies, as the route says it.
fn biarc<S: Scalar>(
    place: &str,
    p0: &Vector3<S>,
    t0: &Vector3<S>,
    p1: &Vector3<S>,
    t1: &Vector3<S>,
) -> GeopResult<[Piece<S>; 2]> {
    let v = p1.sub(p0);
    let a = S::TWO.mul(t0.prod_dot(t1).sub(S::ONE));
    let b = S::TWO.mul(v.prod_dot(&t0.add(t1))).neg();
    let c = v.norm_sq();
    let root = b.mul(b).sub(S::TWO.mul(S::TWO).mul(a).mul(c)).sqrt()?;
    // The positive root `2c / (sqrt(disc) - b)`, which needs no division by
    // `a`, zero when the two directions are parallel.
    let denominator = root.sub(b);
    let numbers = || format!("heading {t0:?} at {p0:?} and {t1:?} at {p1:?}");
    if !denominator.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "route: {place}, the route would have to loop round to arrive: add a waypoint between them"
        ))
        .with_context(numbers()));
    }
    let d = S::TWO.mul(c).div(denominator)?;
    let q1 = p0.add(&t0.prod_scalar(d));
    let q2 = p1.sub(&t1.prod_scalar(d));
    let joint = q1.add(&q2).prod_scalar(S::ONE.div(S::TWO)?);
    let turns_back = || {
        GeopError::new(format!(
            "route: {place}, the route would have to turn right back on itself: add a waypoint between them"
        ))
        .with_context(numbers())
    };
    match [piece(p0, &q1, &joint)?, piece(&joint, &q2, p1)?] {
        [Some(first), Some(second)] => Ok([first, second]),
        _ => Err(turns_back()),
    }
}

/// The arc from `a` to `b` whose legs meet at `q` — legs of one length — or
/// the line, if they could be straight on. None if they could turn right
/// back.
fn piece<S: Scalar>(
    a: &Vector3<S>,
    q: &Vector3<S>,
    b: &Vector3<S>,
) -> GeopResult<Option<Piece<S>>> {
    let (first, second) = (q.sub(a), b.sub(q));
    let dot = first.prod_dot(&second);
    if first
        .prod_cross(&second)
        .norm_sq()
        .could_be_equal(S::ZERO)
    {
        if !dot.definitely_greater(S::ZERO) {
            return Ok(None);
        }
        return Ok(Some(Piece {
            curve: line3(*a, *b)?,
            straight: true,
        }));
    }
    // The weight is the cosine of half the turn between the legs.
    let cos = dot.div(first.norm().mul(second.norm()))?;
    let weight = S::ONE.add(cos).div(S::TWO)?.sqrt()?;
    Ok(Some(Piece {
        curve: arc3(*a, *q, *b, weight)?,
        straight: false,
    }))
}
