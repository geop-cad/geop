//! Sweeping planar profiles along a path of curves (see [`crate::sweep`]):
//! the profile, where it is drawn, carried along the path by the rigid
//! motion that keeps it square to the path — a rotation-minimizing frame,
//! which for a planar path is the frame that keeps the path's plane normal
//! fixed.
//!
//! Every joint of the path is a station; every curve of it a span:
//!
//! - a **line** moves the profile straight on — a [`Span::Line`], exact;
//! - a **circular arc** turns it about the arc's axis — a [`Span::Arc`],
//!   exact: the wall is a piece of a torus, as a revolve sweeps it;
//! - **any other curve** — a spline — has no swept surface that is a
//!   NURBS, so it is approximated: sections of the profile at samples
//!   along the curve, joined by cubic Hermite interpolation of the frames
//!   carrying them — a [`Span::Spline`] skinning those sections.
//!
//! At a joint where the path is tangent-continuous the profile carries on.
//! Where two lines meet at an angle, the profile at the joint is mitred —
//! projected onto the plane bisecting the corner, along either line — so
//! the two straight walls meet there exactly. Any other corner, at an arc or
//! a spline, has no such joint, and is refused.
//!
//! Lines, arcs and corners carry the frames along exactly, in the scalars'
//! own arithmetic. Along a spline, which is approximated anyway, the frames
//! — the sections and the Hermite control frames between them — are a free
//! choice: the swept body is defined by them, every edge and face built
//! from the same frames, so it is consistent whatever they are exactly.
//! They are worked out in plain numbers and taken as sharp, the way a
//! subdivision point is.

use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::build::BuiltBody;
use geop_ops::{Namer, Part};

use crate::sweep::{Frame, Path, Span, SweepLoop, sweep};

/// A path to sweep along: a chain of 3-D curves, each on `[0, 1]` and
/// starting where the one before ends, named like a [`crate::common::Profile`]
/// — `joint_names[i]` where `curves[i]` starts; a closed path, ending where
/// it starts, has one joint per curve, an open one also its end.
#[derive(Clone, Debug)]
pub struct PathChain<S: Scalar> {
    pub curves: Vec<NurbCurve3D<S>>,
    pub curve_names: Vec<String>,
    pub joint_names: Vec<String>,
}

impl<S: Scalar> PathChain<S> {
    pub fn is_closed(&self) -> bool {
        self.joint_names.len() == self.curves.len()
    }

    /// The open chain run the other way; every name stays with its curve
    /// or joint.
    pub fn reversed(&self) -> Self {
        Self {
            curves: self.curves.iter().rev().map(|c| c.reverse()).collect(),
            curve_names: self.curve_names.iter().rev().cloned().collect(),
            joint_names: self.joint_names.iter().rev().cloned().collect(),
        }
    }

    /// The closed chain starting at its joint `k`.
    pub fn starting_at(&self, k: usize) -> Self {
        let mut out = self.clone();
        out.curves.rotate_left(k);
        out.curve_names.rotate_left(k);
        out.joint_names.rotate_left(k);
        out
    }

    /// The joints, where they are.
    pub fn joints(&self) -> GeopResult<Vec<Vector3<S>>> {
        let mut joints = self
            .curves
            .iter()
            .map(start_of)
            .collect::<GeopResult<Vec<_>>>()?;
        if !self.is_closed() {
            joints.push(end_of(self.curves.last().expect("a chain has curves"))?);
        }
        Ok(joints)
    }
}

/// Sections of a spline span per knot span of the path curve it follows.
const SECTIONS_PER_KNOT_SPAN: usize = 4;
/// Steps of the double reflection method between two sections.
const REFLECTION_STEPS: usize = 8;

// ── rigid motions of frames ─────────────────────────────────────────────────

/// A rotation about the unit `axis` by the angle with cosine `cos` and sine
/// `sin`.
#[derive(Clone, Copy, Debug)]
struct Rotation<S: Scalar> {
    axis: Vector3<S>,
    cos: S,
    sin: S,
}

impl<S: Scalar> Rotation<S> {
    /// The least rotation taking the direction `a` onto `b`, two directions
    /// that are definitely not parallel.
    fn between(a: &Vector3<S>, b: &Vector3<S>) -> GeopResult<Self> {
        let (a, b) = (a.normalize()?, b.normalize()?);
        let cross = a.prod_cross(&b);
        let sin = cross.norm();
        Ok(Rotation {
            axis: cross.normalize()?,
            cos: a.prod_dot(&b),
            sin,
        })
    }

    /// `x` turned (Rodrigues).
    fn apply(&self, x: &Vector3<S>) -> Vector3<S> {
        let along = self
            .axis
            .prod_scalar(self.axis.prod_dot(x).mul(S::ONE.sub(self.cos)));
        x.prod_scalar(self.cos)
            .add(&self.axis.prod_cross(x).prod_scalar(self.sin))
            .add(&along)
    }
}

/// `frame` moved by the rigid motion turning by `rotation` about `pivot`,
/// and then carrying `pivot` to `to`.
fn moved<S: Scalar>(
    frame: &Frame<S>,
    rotation: &Rotation<S>,
    pivot: &Vector3<S>,
    to: &Vector3<S>,
) -> Frame<S> {
    Frame {
        origin: to.add(&rotation.apply(&frame.origin.sub(pivot))),
        e1: rotation.apply(&frame.e1),
        e2: rotation.apply(&frame.e2),
    }
}

/// `frame` projected along `direction` onto the plane through `point` with
/// normal `normal`: an affine map, so the profile in it is the profile
/// projected.
fn projected<S: Scalar>(
    frame: &Frame<S>,
    direction: &Vector3<S>,
    point: &Vector3<S>,
    normal: &Vector3<S>,
) -> GeopResult<Frame<S>> {
    let k = S::ONE.div(direction.prod_dot(normal))?;
    let vector = |e: &Vector3<S>| e.sub(&direction.prod_scalar(e.prod_dot(normal).mul(k)));
    Ok(Frame {
        origin: frame
            .origin
            .sub(&direction.prod_scalar(frame.origin.sub(point).prod_dot(normal).mul(k))),
        e1: vector(&frame.e1),
        e2: vector(&frame.e2),
    })
}

/// The point a clamped curve starts at: its first control point.
fn start_of<S: Scalar>(curve: &NurbCurve3D<S>) -> GeopResult<Vector3<S>> {
    let cp = curve.control_points[0];
    Ok(Vector3::from_array([
        cp[0].div(cp[3])?,
        cp[1].div(cp[3])?,
        cp[2].div(cp[3])?,
    ]))
}

fn end_of<S: Scalar>(curve: &NurbCurve3D<S>) -> GeopResult<Vector3<S>> {
    start_of(&curve.reverse())
}

/// What a path curve is, as far as sweeping along it goes.
enum Kind<S: Scalar> {
    Line,
    /// A circular arc from `start` to `end`, its tangents meeting at
    /// `middle`, turning by `rotation`; `weight` is the cosine of half its
    /// angle.
    Arc {
        start: Vector3<S>,
        middle: Vector3<S>,
        end: Vector3<S>,
        rotation: Rotation<S>,
        weight: S,
    },
    Other,
}

/// Whether `curve` is a line, a circular arc, or something else: an arc is
/// a rational quadratic Bézier with legs of one length and its middle
/// weight the cosine of half the angle they turn by — checked on the
/// curve's own enclosures, so a curve that could be one is taken as one.
fn kind<S: Scalar>(curve: &NurbCurve3D<S>) -> GeopResult<Kind<S>> {
    let n = curve.control_points.len();
    if curve.degree == 1 && n == 2 {
        return Ok(Kind::Line);
    }
    if !(curve.degree == 2 && n == 3) {
        return Ok(Kind::Other);
    }
    let cp = &curve.control_points;
    let (p0, p2) = (start_of(curve)?, end_of(curve)?);
    let p1 = Vector3::from_array([
        cp[1][0].div(cp[1][3])?,
        cp[1][1].div(cp[1][3])?,
        cp[1][2].div(cp[1][3])?,
    ]);
    // Weights normalized so the ends have weight one: the middle one is
    // then `w1 / sqrt(w0 w2)`.
    let w = cp[1][3].div(cp[0][3].mul(cp[2][3]).sqrt()?)?;
    let (a, b) = (p1.sub(&p0), p2.sub(&p1));
    let (la, lb) = (a.norm_sq(), b.norm_sq());
    // The cosine of the turn between the legs: `2 w^2 - 1` for an arc.
    let cos = S::TWO.mul(w).mul(w).sub(S::ONE);
    let is_arc = la.could_be_equal(lb)
        && w.definitely_less(S::ONE)
        && w.definitely_greater(S::ZERO)
        && a.prod_dot(&b).div(la.mul(lb).sqrt()?)?.could_be_equal(cos);
    if !is_arc {
        return Ok(Kind::Other);
    }
    // The turn's sine, read off the legs as `|a x b| / (|a| |b|)`, stays as
    // sharp as the legs however flat the arc — unlike `2 w sqrt(1 - w^2)`,
    // whose slope grows without bound as `w` nears one.
    let cross = a.prod_cross(&b);
    Ok(Kind::Arc {
        start: p0,
        middle: p1,
        end: p2,
        rotation: Rotation {
            axis: cross.normalize()?,
            cos,
            sin: cross.norm().div(la.mul(lb).sqrt()?)?,
        },
        weight: w,
    })
}

/// Whether the path goes on smoothly at a joint — told from the tangents
/// arriving and leaving there, honestly: a joint that could be smooth is.
fn is_smooth<S: Scalar>(arriving: &Vector3<S>, leaving: &Vector3<S>) -> GeopResult<bool> {
    let c = arriving.prod_cross(leaving);
    let parallel = (0..3).all(|k| c[k].could_be_equal(S::ZERO));
    if parallel && arriving.prod_dot(leaving).could_be_less(S::ZERO) {
        return Err(GeopError::new(
            "path sweep: the path could turn right back on itself at a joint",
        ));
    }
    Ok(parallel)
}

/// The path along `chain` for a profile drawn in `plane`'s `(u, v)` (see
/// the module docs): the profile at the first station where it is drawn,
/// carried along from there. Its stations are named after the chain's
/// joints, its spans after its curves.
pub fn along_chain<S: Scalar>(
    chain: &PathChain<S>,
    plane: &CoordinateSystem<S>,
) -> GeopResult<Path<S>> {
    let n = chain.curves.len();
    let closed = chain.is_closed();
    if n == 0 || chain.curve_names.len() != n || chain.joint_names.len() != n + usize::from(!closed)
    {
        return Err(GeopError::new(format!(
            "path sweep: a path of {n} curves with {} curve names and {} joint names",
            chain.curve_names.len(),
            chain.joint_names.len()
        )));
    }
    let kinds = chain
        .curves
        .iter()
        .map(kind)
        .collect::<GeopResult<Vec<_>>>()?;
    // The tangents each curve leaves its start with and arrives at its end
    // with, and whether the path turns a corner where each curve starts.
    let leaving = chain
        .curves
        .iter()
        .map(|c| c.tangent(S::ZERO))
        .collect::<GeopResult<Vec<_>>>()?;
    let arriving = chain
        .curves
        .iter()
        .map(|c| c.tangent(S::ONE))
        .collect::<GeopResult<Vec<_>>>()?;
    let before = |k: usize| {
        if k > 0 {
            Some(k - 1)
        } else {
            closed.then_some(n - 1)
        }
    };
    let mut corner = vec![false; n];
    for k in 0..n {
        let Some(b) = before(k) else { continue };
        if !is_smooth(&arriving[b], &leaving[k])? {
            if !(matches!(kinds[b], Kind::Line) && matches!(kinds[k], Kind::Line)) {
                return Err(GeopError::new(format!(
                    "path sweep: the path turns a corner at {}, where only two lines can meet at an angle: make it tangent there",
                    chain.joint_names[k]
                )));
            }
            corner[k] = true;
        }
    }

    // Which way round the path runs, against the profile plane's normal.
    let along = plane.u().prod_cross(plane.v()).prod_dot(&leaving[0]);
    let along_normal = if along.definitely_greater(S::ZERO) {
        true
    } else if along.definitely_less(S::ZERO) {
        false
    } else {
        return Err(GeopError::new(
            "path sweep: the path could run along the profile's plane where it starts",
        ));
    };

    // `rigid` is the profile carried along so far, square to the path as
    // it was where it was drawn; the stations are it, or mitred: projected
    // onto the plane bisecting the corner, along `direction`.
    let mitre = |frame: &Frame<S>, k: usize, direction: &Vector3<S>| -> GeopResult<Frame<S>> {
        let b = before(k).expect("a corner has a curve before it");
        let bisector = arriving[b].normalize()?.add(&leaving[k].normalize()?);
        projected(frame, direction, &start_of(&chain.curves[k])?, &bisector)
    };
    let mut rigid = Frame {
        origin: *plane.origin(),
        e1: *plane.u(),
        e2: *plane.v(),
    };
    let mut stations: Vec<Frame<S>> = Vec::new();
    let mut spans: Vec<Span<S>> = Vec::new();
    for k in 0..n {
        let ctx = with_context!(
            "path sweep at {}, along {}",
            chain.joint_names[k],
            chain.curve_names[k]
        );
        if corner[k] && k == 0 {
            // A closed path starting at a corner: the profile is drawn
            // square to the first curve, and mitred along it.
            stations.push(mitre(&rigid, 0, &leaving[0]).with_context(ctx)?);
        } else if corner[k] {
            let b = k - 1;
            stations.push(mitre(&rigid, k, &arriving[b]).with_context(ctx)?);
            let joint = start_of(&chain.curves[k])?;
            let turn = Rotation::between(&arriving[b], &leaving[k]).with_context(ctx)?;
            rigid = moved(&rigid, &turn, &joint, &joint);
        } else {
            stations.push(rigid.clone());
        }
        let (span, end) = carry(&chain.curves[k], &kinds[k], &rigid).with_context(ctx)?;
        spans.push(span);
        rigid = end;
    }
    if !closed {
        stations.push(rigid);
    }
    Ok(Path {
        stations,
        spans,
        closed,
        along_normal,
        station_names: chain.joint_names.clone(),
        span_names: chain.curve_names.iter().cloned().map(Some).collect(),
    })
}

/// The span carrying the profile at `start` along `curve`, and the profile
/// where it arrives.
fn carry<S: Scalar>(
    curve: &NurbCurve3D<S>,
    kind: &Kind<S>,
    start: &Frame<S>,
) -> GeopResult<(Span<S>, Frame<S>)> {
    Ok(match kind {
        Kind::Line => {
            let step = end_of(curve)?.sub(&start_of(curve)?);
            let end = Frame {
                origin: start.origin.add(&step),
                ..start.clone()
            };
            (Span::Line, end)
        }
        Kind::Arc {
            start: from,
            middle: corner,
            end: to,
            rotation,
            weight,
        } => {
            // The rigid motion along the arc turns about its axis and takes
            // its start to its end: a point `from + r` goes to
            // `to + rotation(r)`. Measured from the arc's own start, not
            // from its centre, so a wide, flat arc's far-off centre never
            // multiplies the turn's width.
            let end = moved(start, rotation, from, to);
            // The middle control row of the arc each point travels: where
            // the tangents at its ends meet. That is linear in the point,
            // and the arc's own start travels the arc itself, whose
            // tangents meet at `corner`; an offset `e` from there adds its
            // part along the axis, and `1 / (1 + cos)` of the sum of its
            // part across and that part turned.
            let k = S::ONE.div(S::ONE.add(rotation.cos))?;
            let axis = &rotation.axis;
            let middle_vector = |e: &Vector3<S>| {
                let along = axis.prod_scalar(axis.prod_dot(e));
                let off = e.sub(&along);
                along.add(&off.add(&rotation.apply(&off)).prod_scalar(k))
            };
            let middle = Frame {
                origin: corner.add(&middle_vector(&start.origin.sub(from))),
                e1: middle_vector(&start.e1),
                e2: middle_vector(&start.e2),
            };
            (
                Span::Arc {
                    middle,
                    weight: *weight,
                },
                end,
            )
        }
        Kind::Other => spline_span(curve, start)?,
    })
}

// ── along a spline, in plain numbers ────────────────────────────────────────

type V = [f64; 3];

fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: V, k: f64) -> V {
    [a[0] * k, a[1] * k, a[2] * k]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn unit(a: V) -> V {
    scale(a, 1.0 / dot(a, a).sqrt())
}
fn plain<S: Scalar>(v: &Vector3<S>) -> V {
    [v[0].to_f64(), v[1].to_f64(), v[2].to_f64()]
}

/// A [`Frame`] in plain numbers, or a frame's derivative.
#[derive(Clone, Copy, Debug)]
struct Plain {
    origin: V,
    e1: V,
    e2: V,
}

impl Plain {
    fn of<S: Scalar>(frame: &Frame<S>) -> Self {
        Plain {
            origin: plain(&frame.origin),
            e1: plain(&frame.e1),
            e2: plain(&frame.e2),
        }
    }

    /// The frame, taken as sharp (see the module docs).
    fn frame<S: Scalar>(&self) -> Frame<S> {
        let v = |a: V| Vector3::from_array(a.map(S::from_f64));
        Frame {
            origin: v(self.origin),
            e1: v(self.e1),
            e2: v(self.e2),
        }
    }

    /// `self + h derivative`.
    fn step(&self, derivative: &Plain, h: f64) -> Self {
        Plain {
            origin: add(self.origin, scale(derivative.origin, h)),
            e1: add(self.e1, scale(derivative.e1, h)),
            e2: add(self.e2, scale(derivative.e2, h)),
        }
    }
}

/// The profile at `start` carried along the general `curve` (see the
/// module docs): sections at evenly spaced parameters, each frame carried
/// on from the one before by the double reflection method (Wang et al.,
/// 2008) — rotation minimizing, and exact for a planar curve — joined by
/// cubic Hermite interpolation, each frame's derivative the frame's
/// angular velocity `T x C'' / |C'|` applied to it.
fn spline_span<S: Scalar>(
    curve: &NurbCurve3D<S>,
    start: &Frame<S>,
) -> GeopResult<(Span<S>, Frame<S>)> {
    let knot_spans = curve.control_points.len() - curve.degree;
    let sections = SECTIONS_PER_KNOT_SPAN * knot_spans.max(1);
    let fine = sections * REFLECTION_STEPS;
    let at = |i: usize| S::from_f64(i as f64 / fine as f64);
    let tangent = |t: S| -> GeopResult<V> { Ok(unit(plain(&curve.tangent(t)?))) };
    let start = Plain::of(start);
    let c0 = plain(&curve.evaluate(S::ZERO)?);
    // The rotation carrying the start frame along, as the images of the
    // axes, and where on the curve it has got to.
    let mut axes: [V; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    let (mut x, mut t) = (c0, tangent(S::ZERO)?);
    let mut frames = Vec::with_capacity(sections + 1);
    let mut derivatives = Vec::with_capacity(sections + 1);
    for i in 0..=fine {
        if i > 0 {
            let (x1, t1) = (plain(&curve.evaluate(at(i))?), tangent(at(i))?);
            axes = axes.map(|a| reflect(a, x, x1, t, t1));
            (x, t) = (x1, t1);
        }
        if i % REFLECTION_STEPS != 0 {
            continue;
        }
        let turn = |v: V| {
            add(
                add(scale(axes[0], v[0]), scale(axes[1], v[1])),
                scale(axes[2], v[2]),
            )
        };
        let frame = Plain {
            origin: add(x, turn(sub(start.origin, c0))),
            e1: turn(start.e1),
            e2: turn(start.e2),
        };
        let d1 = plain(&curve.tangent(at(i))?);
        let d2 = plain(&curve.second_derivative(at(i))?);
        let omega = scale(cross(t, d2), 1.0 / dot(d1, d1).sqrt());
        derivatives.push(Plain {
            origin: add(d1, cross(omega, sub(frame.origin, x))),
            e1: cross(omega, frame.e1),
            e2: cross(omega, frame.e2),
        });
        frames.push(frame);
    }
    let h = 1.0 / sections as f64;
    let mut middle = Vec::new();
    for k in 0..sections {
        if k > 0 {
            middle.push(frames[k].frame());
        }
        middle.push(frames[k].step(&derivatives[k], h / 3.0).frame());
        middle.push(frames[k + 1].step(&derivatives[k + 1], -h / 3.0).frame());
    }
    let mut knots = vec![S::ZERO; 4];
    for k in 1..sections {
        knots.extend(std::iter::repeat_n(
            S::from_ratio(k as i64, sections as i64)?,
            3,
        ));
    }
    knots.extend(vec![S::ONE; 4]);
    Ok((
        Span::Spline {
            degree: 3,
            middle,
            knots,
        },
        frames[sections].frame(),
    ))
}

/// One step of the double reflection method, applied to the vector `v`:
/// the rotation carrying a frame at the point `x0` with unit tangent `t0`
/// on to `x1` and `t1` — a reflection in the plane bisecting `x0` and `x1`,
/// then one taking the reflected tangent onto `t1`.
fn reflect(v: V, x0: V, x1: V, t0: V, t1: V) -> V {
    let reflect_in = |n: V, x: V| {
        let c = dot(n, n);
        if c == 0.0 {
            x
        } else {
            sub(x, scale(n, 2.0 * dot(n, x) / c))
        }
    };
    let v1 = sub(x1, x0);
    let v2 = sub(t1, reflect_in(v1, t0));
    reflect_in(v2, reflect_in(v1, v))
}

/// Sweeps `loops` — the first the outer loop, counter-clockwise in
/// `plane`'s `(u, v)`, the rest holes in it, clockwise — along `chain`
/// (see [`along_chain`]): into a solid named `solid`, or, without one, into
/// sheets. Either way round, the solid comes out with its faces pointing
/// outwards.
///
/// Named after the profiles' curves `X` and joints `P` and the chain's
/// curves `C` and joints `J` (see [`sweep`]): the walls `N(X,C)`, their
/// edges at the joints `N(X,J)`, the edges between `N(P,C)`, the vertices
/// `N(P,J)`; for an open chain, the caps `N(start)` and `N(end)`.
pub fn sweep_along<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    solid: Option<&str>,
    chain: &PathChain<S>,
    plane: &CoordinateSystem<S>,
    loops: &[SweepLoop<S>],
) -> GeopResult<BuiltBody> {
    let path = along_chain(chain, plane)?;
    let loops: Vec<SweepLoop<S>> = if path.along_normal {
        loops.iter().map(SweepLoop::reversed).collect()
    } else {
        loops.to_vec()
    };
    sweep(part, namer, &path, &loops, solid)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{Profile, arc2, arc3, line3, polygon, polyline, sqrt2_over_2};
    use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve2D};
    use geop_core_math::for_all_scalars;
    use geop_core_math::vector::{Vector2, Vector4};
    use geop_core_topology::{
        Model,
        validation::{ValidationParameters, validate, validate_manifold},
    };

    fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
        Vector2::from_array([S::from_f64(x), S::from_f64(y)])
    }

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    /// A circle of radius `r` around the origin, as four quarter arcs,
    /// counter-clockwise.
    fn circle<S: Scalar>(r: f64) -> Vec<NurbCurve2D<S>> {
        let q = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
        (0..4)
            .map(|i| {
                let (a, b) = (q[i], q[(i + 1) % 4]);
                arc2(
                    v2(a.0, a.1),
                    v2(a.0 + b.0, a.1 + b.1),
                    v2(b.0, b.1),
                    sqrt2_over_2(),
                )
                .unwrap()
            })
            .collect()
    }

    fn square<S: Scalar>(half: f64) -> Vec<NurbCurve2D<S>> {
        polygon(&[
            v2(-half, -half),
            v2(half, -half),
            v2(half, half),
            v2(-half, half),
        ])
        .unwrap()
    }

    /// The plane through `origin` spanned by `u` and `v`.
    fn plane<S: Scalar>(origin: Vector3<S>, u: Vector3<S>, v: Vector3<S>) -> CoordinateSystem<S> {
        CoordinateSystem::try_new(origin, u, v, u.prod_cross(&v)).unwrap()
    }

    /// The `yz` plane through `origin`: square to a path along `x`.
    fn yz<S: Scalar>(origin: Vector3<S>) -> CoordinateSystem<S> {
        plane(origin, v3(0., 1., 0.), v3(0., 0., 1.))
    }

    fn chain<S: Scalar>(curves: Vec<NurbCurve3D<S>>, closed: bool) -> PathChain<S> {
        let n = curves.len();
        PathChain {
            curve_names: (0..n).map(|i| format!("k{i}")).collect(),
            joint_names: (0..n + usize::from(!closed))
                .map(|i| format!("j{i}"))
                .collect(),
            curves,
        }
    }

    fn assert_valid<S: Scalar>(model: &Model<S>) {
        let params = ValidationParameters::default();
        if let Err(e) = validate(&params, model) {
            panic!("{e:?}");
        }
        if let Err(e) = validate_manifold(&params, model) {
            panic!("{e:?}");
        }
    }

    /// Sweeps the closed `outer` loop in `plane` along `path` into a solid
    /// in a fresh part, and checks it is valid.
    fn swept<S: Scalar>(
        outer: Vec<NurbCurve2D<S>>,
        plane: &CoordinateSystem<S>,
        path: &PathChain<S>,
    ) -> Part<S> {
        let mut part = Part::<S>::new();
        let namer = Namer::new("sweep", "s").unwrap();
        sweep_along(
            &mut part,
            &namer,
            Some(&namer.root()),
            path,
            plane,
            &[SweepLoop::plain(Profile::closed(outer))],
        )
        .unwrap();
        part.check_names().unwrap();
        assert_valid(part.topology());
        part
    }

    /// Along a single line, a sweep is an extrude.
    fn check_sweep_along_a_line_is_a_box<S: Scalar>() {
        let path = chain(
            vec![line3(v3::<S>(0., 0., 0.), v3(2., 0., 0.)).unwrap()],
            false,
        );
        let part = swept(square(0.5), &yz(Vector3::zero()), &path);
        let model = part.topology();
        assert_eq!(model.faces.len(), 6);
        let far = part.vertex_id("sweep(s,p2,j1)").unwrap();
        assert!(
            model
                .get_vertex(far)
                .unwrap()
                .point
                .could_be_equal(&v3(2., 0.5, 0.5))
        );
    }
    #[test]
    fn sweep_along_a_line_is_a_box() {
        for_all_scalars!(check_sweep_along_a_line_is_a_box);
    }

    /// Two lines at a right angle: the profile is mitred at the corner, so
    /// the corner's vertices lie on the plane bisecting it.
    fn check_sweep_round_a_corner_is_mitred<S: Scalar>() {
        let path = chain(
            vec![
                line3(v3::<S>(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                line3(v3::<S>(2., 0., 0.), v3(2., 2., 0.)).unwrap(),
            ],
            false,
        );
        let part = swept(square(0.5), &yz(Vector3::zero()), &path);
        let model = part.topology();
        assert_eq!(model.faces.len(), 4 * 2 + 2);
        // The bisecting plane of the corner at (2, 0, 0): x - 2 = -y.
        for p in 0..4 {
            let v = part.vertex_id(&format!("sweep(s,p{p},j1)")).unwrap();
            let point = model.get_vertex(v).unwrap().point;
            assert!(
                point[0]
                    .sub(S::from_f64(2.0))
                    .could_be_equal(point[1].neg()),
                "{point:?}"
            );
        }
    }
    #[test]
    fn sweep_round_a_corner_is_mitred() {
        for_all_scalars!(check_sweep_round_a_corner_is_mitred);
    }

    /// A pipe: a line, a quarter bend, a line — tangent all along.
    fn check_sweep_a_pipe_through_a_bend<S: Scalar>() {
        let path = chain(
            vec![
                line3(v3::<S>(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                arc3(
                    v3(2., 0., 0.),
                    v3(3., 0., 0.),
                    v3(3., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
                line3(v3::<S>(3., 1., 0.), v3(3., 3., 0.)).unwrap(),
            ],
            false,
        );
        let part = swept(circle(0.3), &yz(Vector3::zero()), &path);
        let model = part.topology();
        assert_eq!(model.faces.len(), 4 * 3 + 2);
        // The far end of the pipe lies square to the last line, at y = 3.
        let end = part.face_id("sweep(s,end)").unwrap();
        assert!(model.iterate_face_coedges(end).all(|c| {
            model.coedge_start_vertex(c).unwrap().point[1].could_be_equal(S::from_f64(3.0))
        }));
    }
    #[test]
    fn sweep_a_pipe_through_a_bend() {
        for_all_scalars!(check_sweep_a_pipe_through_a_bend);
    }

    /// Round a closed circle: a torus, with no caps.
    fn check_sweep_round_a_circle_is_a_torus<S: Scalar>() {
        let r = 2.0;
        let q = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
        let arcs = (0..4)
            .map(|i| {
                let (a, b) = (q[i], q[(i + 1) % 4]);
                arc3(
                    v3::<S>(a.0, a.1, 0.),
                    v3(a.0 + b.0, a.1 + b.1, 0.),
                    v3(b.0, b.1, 0.),
                    sqrt2_over_2(),
                )
                .unwrap()
            })
            .collect();
        let path = chain(arcs, true);
        let profile_plane = plane(v3(2., 0., 0.), v3(1., 0., 0.), v3(0., 0., 1.));
        let part = swept(circle(0.5), &profile_plane, &path);
        let model = part.topology();
        assert_eq!(model.faces.len(), 16);
        for v in model.vertices.values() {
            // On the torus: (sqrt(x^2 + y^2) - 2)^2 + z^2 = 0.25.
            let (x, y, z) = (v.point[0], v.point[1], v.point[2]);
            let rho = x.mul(x).add(y.mul(y)).sqrt().unwrap();
            let d = rho.sub(S::from_f64(2.0));
            let off = d.mul(d).add(z.mul(z));
            assert!(off.could_be_equal(S::from_f64(0.25)), "{:?}", v.point);
        }
    }
    #[test]
    fn sweep_round_a_circle_is_a_torus() {
        for_all_scalars!(check_sweep_round_a_circle_is_a_torus);
    }

    /// Round a closed square: a frame, every corner mitred, the first one
    /// too.
    fn check_sweep_round_a_square_is_a_frame<S: Scalar>() {
        let corners = [(0., 0.), (4., 0.), (4., 4.), (0., 4.)];
        let lines = (0..4)
            .map(|i| {
                let (a, b) = (corners[i], corners[(i + 1) % 4]);
                line3(v3::<S>(a.0, a.1, 0.), v3(b.0, b.1, 0.)).unwrap()
            })
            .collect();
        let path = chain(lines, true);
        let part = swept(square(0.5), &yz(Vector3::zero()), &path);
        let model = part.topology();
        assert_eq!(model.faces.len(), 16);
        assert_eq!(model.vertices.len(), 16);
    }
    #[test]
    fn sweep_round_a_square_is_a_frame() {
        for_all_scalars!(check_sweep_round_a_square_is_a_frame);
    }

    /// Along a cubic spline: skinned sections, ending square to it.
    fn check_sweep_along_a_spline<S: Scalar>() {
        let f = S::from_f64;
        let p = |x: f64, y: f64| Vector4::from_array([f(x), f(y), f(0.), f(1.)]);
        let spline = NurbCurve::try_new(
            3,
            vec![p(0., 0.), p(1., 0.), p(2., 1.), p(3., 1.)],
            vec![f(0.), f(0.), f(0.), f(0.), f(1.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let path = chain(vec![spline], false);
        let part = swept(circle(0.2), &yz(Vector3::zero()), &path);
        let model = part.topology();
        assert_eq!(model.faces.len(), 4 + 2);
        // The spline ends heading along x again: the end cap lies at x = 3 —
        // as near as the frames along a spline, a sharp free choice worked
        // out in plain numbers, put it, which is not an enclosure of x = 3.
        let end = part.face_id("sweep(s,end)").unwrap();
        assert!(model.iterate_face_coedges(end).all(|c| {
            let x = model.coedge_start_vertex(c).unwrap().point[0].to_f64();
            (x - 3.0).abs() < 1e-9
        }));
    }
    #[test]
    fn sweep_along_a_spline() {
        for_all_scalars!(check_sweep_along_a_spline);
    }

    /// An open chain sweeps into a sheet.
    fn check_open_profile_sweeps_into_a_sheet<S: Scalar>() {
        let path = chain(
            vec![
                line3(v3::<S>(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                arc3(
                    v3(2., 0., 0.),
                    v3(3., 0., 0.),
                    v3(3., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
            ],
            false,
        );
        let mut part = Part::<S>::new();
        let namer = Namer::new("sweep", "s").unwrap();
        let profile = Profile::open(polyline(&[v2(-0.5, 0.), v2(0., 0.5), v2(0.5, 0.)]).unwrap());
        let built = sweep_along(
            &mut part,
            &namer,
            None,
            &path,
            &yz(Vector3::zero()),
            &[SweepLoop::plain(profile)],
        )
        .unwrap();
        assert!(built.solid.is_none());
        part.check_names().unwrap();
        if let Err(e) = validate(&ValidationParameters::default(), part.topology()) {
            panic!("{e:?}");
        }
        assert_eq!(part.topology().faces.len(), 4);
    }
    #[test]
    fn open_profile_sweeps_into_a_sheet() {
        for_all_scalars!(check_open_profile_sweeps_into_a_sheet);
    }

    /// A line meeting an arc at an angle has no mitre: refused.
    fn check_corner_at_an_arc_is_refused<S: Scalar>() {
        let path = chain(
            vec![
                line3(v3::<S>(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                arc3(
                    v3(2., 0., 0.),
                    v3(2., 1., 0.),
                    v3(1., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
            ],
            false,
        );
        let error = along_chain(&path, &yz(Vector3::<S>::zero())).unwrap_err();
        assert!(error.root_message().contains("corner"), "{error:?}");
    }
    #[test]
    fn corner_at_an_arc_is_refused() {
        for_all_scalars!(check_corner_at_an_arc_is_refused);
    }

    /// A profile in a plane the path runs along cannot be swept.
    fn check_profile_along_the_path_is_refused<S: Scalar>() {
        let path = chain(
            vec![line3(v3::<S>(0., 0., 0.), v3(2., 0., 0.)).unwrap()],
            false,
        );
        let xy = plane(Vector3::<S>::zero(), v3(1., 0., 0.), v3(0., 1., 0.));
        assert!(along_chain(&path, &xy).is_err());
    }
    #[test]
    fn profile_along_the_path_is_refused() {
        for_all_scalars!(check_profile_along_the_path_is_refused);
    }
}
