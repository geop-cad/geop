//! Sweeping planar profiles along a path of curves (see [`crate::sweep`]):
//! the profile, where it is drawn, carried along the path by the rigid
//! motion that keeps it square to the path — a rotation-minimizing frame,
//! which for a planar path is the frame that keeps the path's plane normal
//! fixed.
//!
//! Every joint of the path is a station; every curve of it a span:
//!
//! - a **line** moves the profile straight on — a [`Span::Line`], exact;
//! - a **circular arc** turns it about the arc's axis — a [`Span::arc`],
//!   exact: the wall is a piece of a torus, as a revolve sweeps it;
//! - **any other curve** — a spline — has no swept surface that is a
//!   NURBS, so it is approximated: sections of the profile at samples
//!   along the curve, joined by cubic Hermite interpolation of the frames
//!   carrying them — a [`Span::Curve`] skinning those sections.
//!
//! At a joint where the path is tangent-continuous the profile carries on.
//! Where two lines meet at an angle, the profile at the joint is mitred —
//! projected onto the plane bisecting the corner, along either line — so
//! the two straight walls meet there exactly. At any other corner, at an
//! arc or a spline, the walls either side are put on the surfaces the
//! profile sweeps along each curve, and the body rebuilt around them: they
//! meet where they cross (see [`sweep_along`]).
//!
//! Lines, arcs and corners carry the frames along exactly, in the scalars'
//! own arithmetic. Along a spline, which is approximated anyway, the frames
//! — the sections and the Hermite control frames between them — are a free
//! choice: the swept body is defined by them, every edge and face built
//! from the same frames, so it is consistent whatever they are exactly.
//! They are worked out in plain numbers and taken as sharp, the way a
//! subdivision point is.
//!
//! The profile can also change as it travels (see [`Control`]): twisted
//! and scaled evenly along the path, or turned and scaled by guide rails so
//! that its points on them follow them. No such sweep is a NURBS in general,
//! so every span of it is sampled as a spline's is — the sections the
//! carried frame with the profile mapped in it — except a line along which
//! the map changes linearly, which stays exact. And it can keep facing the
//! way it is drawn ([`Orientation::FixedNormal`]): then every point of it
//! travels a copy of the path, and every span is exact.

use std::collections::HashMap;

use geop_core_geometry::{
    intersection::crossing_points,
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
    search::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    plain::{P3, add, cross, cross2, dot, inverse2, normalize, scale, solve2, sub},
    primitives::CoordinateSystem,
    scalars::Scalar,
    solvers::newton::{Arrive, newton_1d},
    vector::{Vector2, Vector3, Vector4},
    with_context,
};
use geop_core_topology::{
    Curve2, Sense,
    build::{BodySpec, BuiltBody},
};
use geop_ops_model::{BodyNames, Chain, NamedModel, Namer, Profile};
use serde::{Deserialize, Serialize};

use crate::{
    loft::{centre, mark},
    plain::Plain,
    sweep::{Frame, Path, Span, SweepLoop, skinned, sweep_drawn, wall},
};

/// Which way the profile faces as it travels along the path.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Orientation {
    /// Square to the path as it was where it is drawn: carried along by a
    /// rotation minimizing frame, mitred where two lines meet.
    #[default]
    FollowPath,
    /// Facing the way it is drawn all along: moved along the path, never
    /// turned — the path has to keep running through its plane.
    FixedNormal,
}

/// How the profile changes as it travels, besides being carried along the
/// path (see [`sweep_along`]). Every section of the sweep is the profile
/// mapped about the path's start point in the profile's plane — its centre
/// — by a turn and a scale:
///
/// - **twist** and **scale** grow evenly with the length travelled, from
///   none to `twist` (radians, counter-clockwise in the profile's plane as
///   drawn) and from one to `end_scale`;
/// - **rails**, one or two, decide them instead: where each rail crosses a
///   section is where the profile's point it starts at goes. One rail turns
///   and scales the profile uniformly about the centre; two map it linearly,
///   which scales it differently along different directions. That is the
///   sweep following the path and its first guide: with
///   [`Orientation::FollowPath`] the section planes are square to the path,
///   and the rails turn the profile within them.
#[derive(Clone, Debug)]
pub struct Control {
    pub orientation: Orientation,
    pub twist: f64,
    pub end_scale: f64,
    pub rails: Vec<Chain<NurbCurve3D>>,
}

impl Default for Control {
    /// Carried along the path, unchanged.
    fn default() -> Self {
        Self {
            orientation: Orientation::FollowPath,
            twist: 0.0,
            end_scale: 1.0,
            rails: Vec::new(),
        }
    }
}

impl Control {
    /// Whether the profile only travels, neither twisted, scaled nor
    /// following rails — which needs no sampling (see the module docs).
    pub fn is_plain(&self) -> bool {
        self.twist == 0.0 && self.end_scale == 1.0 && self.rails.is_empty()
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
struct Rotation {
    axis: Vector3,
    cos: Scalar,
    sin: Scalar,
}

impl Rotation {
    /// The least rotation taking the direction `a` onto `b`, two directions
    /// that are definitely not parallel.
    fn between(a: &Vector3, b: &Vector3) -> GeopResult<Self> {
        let (a, b) = (a.normalize()?, b.normalize()?);
        let cross = a.prod_cross(&b);
        let sin = cross.norm();
        Ok(Rotation {
            axis: cross.normalize()?,
            cos: a.prod_dot(&b),
            sin,
        })
    }

    /// `x` turned.
    fn apply(&self, x: &Vector3) -> Vector3 {
        x.rotated_about(&self.axis, self.cos, self.sin)
    }
}

/// `frame` moved by the rigid motion turning by `rotation` about `pivot`,
/// and then carrying `pivot` to `to`.
fn moved(frame: &Frame, rotation: &Rotation, pivot: &Vector3, to: &Vector3) -> Frame {
    Frame {
        origin: to.add(&rotation.apply(&frame.origin.sub(pivot))),
        e1: rotation.apply(&frame.e1),
        e2: rotation.apply(&frame.e2),
    }
}

/// `frame` projected along `direction` onto the plane through `point` with
/// normal `normal`: an affine map, so the profile in it is the profile
/// projected.
fn projected(
    frame: &Frame,
    direction: &Vector3,
    point: &Vector3,
    normal: &Vector3,
) -> GeopResult<Frame> {
    let k = Scalar::ONE.div(direction.prod_dot(normal))?;
    let vector = |e: &Vector3| e.sub(&direction.prod_scalar(e.prod_dot(normal).mul(k)));
    Ok(Frame {
        origin: frame
            .origin
            .sub(&direction.prod_scalar(frame.origin.sub(point).prod_dot(normal).mul(k))),
        e1: vector(&frame.e1),
        e2: vector(&frame.e2),
    })
}

/// What a path curve is, as far as sweeping along it goes.
#[allow(clippy::large_enum_variant, reason = "decided once per path curve")]
enum Kind {
    Line,
    /// A circular arc from `start` to `end`, its tangents meeting at
    /// `middle`, turning by `rotation`; `weight` is the cosine of half its
    /// angle.
    Arc {
        start: Vector3,
        middle: Vector3,
        end: Vector3,
        rotation: Rotation,
        weight: Scalar,
    },
    Other,
}

/// Whether `curve` is a line, a circular arc, or something else: an arc is
/// a rational quadratic Bézier with legs of one length and its middle
/// weight the cosine of half the angle they turn by — checked on the
/// curve's own enclosures, so a curve that could be one is taken as one.
fn kind(curve: &NurbCurve3D) -> GeopResult<Kind> {
    let n = curve.control_points.len();
    if curve.degree == 1 && n == 2 {
        return Ok(Kind::Line);
    }
    if !(curve.degree == 2 && n == 3) {
        return Ok(Kind::Other);
    }
    let cp = &curve.control_points;
    let (p0, p2) = (curve.start()?, curve.end()?);
    let p1 = cp[1].dehomogenized()?;
    // Weights normalized so the ends have weight one: the middle one is
    // then `w1 / sqrt(w0 w2)`.
    let w = cp[1][3].div(cp[0][3].mul(cp[2][3]).sqrt()?)?;
    let (a, b) = (p1.sub(&p0), p2.sub(&p1));
    let (la, lb) = (a.norm_sq(), b.norm_sq());
    // The cosine of the turn between the legs: `2 w^2 - 1` for an arc.
    let cos = Scalar::TWO.mul(w).mul(w).sub(Scalar::ONE);
    let is_arc = la.could_be_equal(lb)
        && w.definitely_less(Scalar::ONE)
        && w.definitely_greater(Scalar::ZERO)
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
fn is_smooth(arriving: &Vector3, leaving: &Vector3) -> GeopResult<bool> {
    let c = arriving.prod_cross(leaving);
    let parallel = (0..3).all(|k| c[k].could_be_equal(Scalar::ZERO));
    if parallel && arriving.prod_dot(leaving).could_be_less(Scalar::ZERO) {
        return Err(GeopError::new(
            "path sweep: the path could turn right back on itself at a joint",
        ));
    }
    Ok(parallel)
}

/// A path along a chain (see [`followed`]), and what a sweep along it
/// still has to rebuild.
struct Followed {
    path: Path,
    /// Per span, the profile carried rigidly along it: where it starts and
    /// where it ends, unmitred.
    rigid: Vec<[Frame; 2]>,
    /// The joints where the path turns a corner at an arc or a spline: the
    /// walls either side run to the mitred section there, which is not
    /// where the swept walls meet (see [`sweep_along`]).
    rebuilt: Vec<usize>,
}

/// The path along `chain` for a profile drawn in `plane`'s `(u, v)` (see
/// the module docs), changing as `control` says: the profile at the first
/// station where it is drawn, carried along from there. Its stations are
/// named after the chain's joints, its spans after its curves.
fn followed(
    chain: &Chain<NurbCurve3D>,
    plane: &CoordinateSystem,
    control: &Control,
) -> GeopResult<Followed> {
    let follow = control.orientation == Orientation::FollowPath;
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
        .map(|c| c.tangent(Scalar::ZERO))
        .collect::<GeopResult<Vec<_>>>()?;
    let arriving = chain
        .curves
        .iter()
        .map(|c| c.tangent(Scalar::ONE))
        .collect::<GeopResult<Vec<_>>>()?;
    let before = |k: usize| {
        if k > 0 {
            Some(k - 1)
        } else {
            closed.then_some(n - 1)
        }
    };
    let mut corner = vec![false; n];
    let mut rebuilt = Vec::new();
    for k in 0..n {
        let Some(b) = before(k) else { continue };
        // A profile that never turns needs no mitre: it meets itself at
        // any corner.
        if !is_smooth(&arriving[b], &leaving[k])? && follow {
            if !(matches!(kinds[b], Kind::Line) && matches!(kinds[k], Kind::Line)) {
                if !control.is_plain() {
                    return Err(GeopError::new(format!(
                        "path sweep: the path turns a corner at {}, where only two lines can meet at an angle while the profile twists, scales or follows rails: make it tangent there",
                        chain.joint_names[k]
                    )));
                }
                rebuilt.push(k);
            }
            corner[k] = true;
        }
    }

    // Which way round the path runs, against the profile plane's normal.
    let along = plane.u().prod_cross(plane.v()).prod_dot(&leaving[0]);
    let along_normal = along.definite_sign().ok_or_else(|| {
        GeopError::new("path sweep: the path could run along the profile's plane where it starts")
    })?;
    let drawn = Frame {
        origin: *plane.origin(),
        e1: *plane.u(),
        e2: *plane.v(),
    };
    let done = |path: Path, rigid: Vec<[Frame; 2]>| Followed {
        path,
        rigid,
        rebuilt: rebuilt.clone(),
    };
    if !follow {
        let path = translated(chain, &drawn, along_normal)?;
        if control.is_plain() {
            return Ok(done(path, Vec::new()));
        }
        let starts = path.stations[..n].to_vec();
        let path = controlled(chain, &kinds, plane, &path, &starts, &corner, control)?;
        return Ok(done(path, Vec::new()));
    }

    // `rigid` is the profile carried along so far, square to the path as
    // it was where it was drawn; the stations are it, or mitred: projected
    // onto the plane bisecting the corner, along `direction`.
    let mitre = |frame: &Frame, k: usize, direction: &Vector3| -> GeopResult<Frame> {
        let b = before(k).expect("a corner has a curve before it");
        let bisector = arriving[b].normalize()?.add(&leaving[k].normalize()?);
        projected(frame, direction, &chain.curves[k].start()?, &bisector)
    };
    let mut rigid = drawn;
    let mut stations: Vec<Frame> = Vec::new();
    let mut spans: Vec<Span> = Vec::new();
    // The profile carried along, unmitred, where each curve starts, and
    // where it ends.
    let mut starts: Vec<Frame> = Vec::new();
    let mut rigid_spans: Vec<[Frame; 2]> = Vec::new();
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
            let joint = chain.curves[k].start()?;
            let turn = Rotation::between(&arriving[b], &leaving[k]).with_context(ctx)?;
            rigid = moved(&rigid, &turn, &joint, &joint);
        } else {
            stations.push(rigid.clone());
        }
        starts.push(rigid.clone());
        let (span, end) = carry(&chain.curves[k], &kinds[k], &rigid).with_context(ctx)?;
        spans.push(span);
        rigid_spans.push([rigid.clone(), end.clone()]);
        rigid = end;
    }
    if !closed {
        stations.push(rigid);
    }
    let path = Path {
        stations,
        spans,
        closed,
        along_normal,
        station_names: chain.joint_names.clone(),
        span_names: chain.curve_names.iter().cloned().map(Some).collect(),
    };
    if control.is_plain() {
        return Ok(done(path, rigid_spans));
    }
    let path = controlled(chain, &kinds, plane, &path, &starts, &corner, control)?;
    Ok(done(path, Vec::new()))
}

/// The path along `chain` for a profile placed by `drawn` that keeps facing
/// the way it was drawn (see [`Orientation::FixedNormal`]): the profile
/// moved along each curve, never turned. Every point of it travels a copy
/// of the curve, so every span is the curve's own degree, knots and weights,
/// its rows the profile moved to each control point — exact.
///
/// The path has to keep running through the profile's plane the way it
/// starts, or the walls would fold over: checked on the control points,
/// whose heights above the plane rise all the way along — which is enough,
/// since a NURBS of positive weights varies no more than its control
/// polygon.
fn translated(chain: &Chain<NurbCurve3D>, drawn: &Frame, along_normal: bool) -> GeopResult<Path> {
    let normal = drawn.e1.prod_cross(&drawn.e2);
    let normal = if along_normal { normal } else { normal.neg() };
    let start = chain.curves[0].start()?;
    let moved_to = |p: &Vector3| Frame {
        origin: drawn.origin.add(&p.sub(&start)),
        ..drawn.clone()
    };
    let mut height: Option<Scalar> = None;
    let mut stations = Vec::new();
    let mut spans = Vec::new();
    for (k, curve) in chain.curves.iter().enumerate() {
        // Its ends weighted one, as the stations are.
        let curve = &curve.with_unit_end_weights()?;
        let points = curve
            .control_points
            .iter()
            .map(Vector4::dehomogenized)
            .collect::<GeopResult<Vec<_>>>()?;
        for (i, p) in points.iter().enumerate() {
            let h = p.sub(&start).prod_dot(&normal);
            // Every curve after the first starts where the one before ends.
            if k > 0 && i == 0 {
                continue;
            }
            if i > 0 && !height.is_some_and(|before| h.definitely_greater(before)) {
                return Err(GeopError::new(format!(
                    "path sweep: with a fixed normal, the path has to keep running through the profile's plane, but along {} it could turn parallel to it or back",
                    chain.curve_names[k]
                )));
            }
            height = Some(h);
        }
        stations.push(moved_to(&points[0]));
        let n = points.len();
        spans.push(if curve.degree == 1 && n == 2 {
            Span::Line
        } else {
            Span::Curve {
                degree: curve.degree,
                knots: curve.knot_vector.clone(),
                middle: (1..n - 1)
                    .map(|i| {
                        let w = curve.control_points[i][3];
                        (
                            moved_to(&points[i]),
                            (!w.is_exactly(Scalar::ONE)).then_some(w),
                        )
                    })
                    .collect(),
            }
        });
    }
    let closed = chain.is_closed();
    if !closed {
        let last = chain.curves.last().expect("a chain has curves");
        stations.push(moved_to(&last.end()?));
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
fn carry(curve: &NurbCurve3D, kind: &Kind, start: &Frame) -> GeopResult<(Span, Frame)> {
    Ok(match kind {
        Kind::Line => {
            let step = curve.end()?.sub(&curve.start()?);
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
            let k = Scalar::ONE.div(Scalar::ONE.add(rotation.cos))?;
            let axis = &rotation.axis;
            let middle_vector = |e: &Vector3| {
                let along = axis.prod_scalar(axis.prod_dot(e));
                let off = e.sub(&along);
                along.add(&off.add(&rotation.apply(&off)).prod_scalar(k))
            };
            let middle = Frame {
                origin: corner.add(&middle_vector(&start.origin.sub(from))),
                e1: middle_vector(&start.e1),
                e2: middle_vector(&start.e2),
            };
            (Span::arc(middle, *weight), end)
        }
        Kind::Other => spline_span(curve, start)?,
    })
}

// ── along a spline, in plain numbers ────────────────────────────────────────

/// A frame's sample along a span: the frame, its derivative along the
/// span's parameter, and the point of the path it is at.
#[derive(Clone, Copy, Debug)]
struct Sample {
    frame: Plain,
    derivative: Plain,
    point: P3,
}

/// The profile at `start` carried along the general `curve` by a rotation
/// minimizing frame, at `fine + 1` evenly spaced parameters: each frame
/// carried on from the one before by the double reflection method (Wang et
/// al., 2008) — exact for a planar curve — its derivative the frame's
/// angular velocity `T x C'' / |C'|` applied to it.
fn reflected(curve: &NurbCurve3D, start: &Frame, fine: usize) -> GeopResult<Vec<Sample>> {
    let at = |i: usize| Scalar::from_f64(i as f64 / fine as f64);
    let tangent = |t: Scalar| -> GeopResult<P3> {
        normalize(curve.tangent(t)?.to_f64_array())
            .ok_or_else(|| GeopError::new(format!("path sweep: the path has no tangent at {t:?}")))
    };
    let start = Plain::of(start);
    let c0 = curve.evaluate(Scalar::ZERO)?.to_f64_array();
    // The rotation carrying the start frame along, as the images of the
    // axes, and where on the curve it has got to.
    let mut axes: [P3; 3] = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]];
    let (mut x, mut t) = (c0, tangent(Scalar::ZERO)?);
    let mut samples = Vec::with_capacity(fine + 1);
    for i in 0..=fine {
        if i > 0 {
            let (x1, t1) = (curve.evaluate(at(i))?.to_f64_array(), tangent(at(i))?);
            axes = axes.map(|a| reflect(a, x, x1, t, t1));
            (x, t) = (x1, t1);
        }
        let turn = |v: P3| {
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
        let d1 = curve.tangent(at(i))?.to_f64_array();
        let d2 = curve.second_derivative(at(i))?.to_f64_array();
        let omega = scale(cross(t, d2), 1.0 / dot(d1, d1).sqrt());
        samples.push(Sample {
            frame,
            derivative: Plain {
                origin: add(d1, cross(omega, sub(frame.origin, x))),
                e1: cross(omega, frame.e1),
                e2: cross(omega, frame.e2),
            },
            point: x,
        });
    }
    Ok(samples)
}

/// The cubic Hermite spline through `samples` taken as evenly spaced over
/// `[0, 1]` — the frames at its knots, the frames a third of a step along
/// their derivatives either side between them: the inner rows and knots of
/// a [`Span::Curve`] from the first sample's frame to the last's.
fn hermite(samples: &[Sample]) -> GeopResult<Span> {
    let sections = samples.len() - 1;
    let h = 1.0 / sections as f64;
    let mut middle = Vec::new();
    for k in 0..sections {
        if k > 0 {
            middle.push(samples[k].frame.frame());
        }
        middle.push(
            samples[k]
                .frame
                .step(&samples[k].derivative, h / 3.0)
                .frame(),
        );
        middle.push(
            samples[k + 1]
                .frame
                .step(&samples[k + 1].derivative, -h / 3.0)
                .frame(),
        );
    }
    let mut knots = vec![Scalar::ZERO; 4];
    for k in 1..sections {
        knots.extend(std::iter::repeat_n(
            Scalar::from_ratio(k as i64, sections as i64)?,
            3,
        ));
    }
    knots.extend(vec![Scalar::ONE; 4]);
    Ok(Span::Curve {
        degree: 3,
        knots,
        middle: middle.into_iter().map(|m| (m, None)).collect(),
    })
}

/// How many knot spans `curve` has.
fn knot_spans(curve: &NurbCurve3D) -> usize {
    (curve.control_points.len() - curve.degree).max(1)
}

/// The profile at `start` carried along the general `curve` (see the
/// module docs): sections at evenly spaced parameters (see [`reflected`]),
/// joined by cubic Hermite interpolation (see [`hermite`]).
fn spline_span(curve: &NurbCurve3D, start: &Frame) -> GeopResult<(Span, Frame)> {
    let sections = SECTIONS_PER_KNOT_SPAN * knot_spans(curve);
    let samples: Vec<Sample> = reflected(curve, start, sections * REFLECTION_STEPS)?
        .into_iter()
        .step_by(REFLECTION_STEPS)
        .collect();
    Ok((hermite(&samples)?, samples[sections].frame.frame()))
}

/// One step of the double reflection method, applied to the vector `v`:
/// the rotation carrying a frame at the point `x0` with unit tangent `t0`
/// on to `x1` and `t1` — a reflection in the plane bisecting `x0` and `x1`,
/// then one taking the reflected tangent onto `t1`.
fn reflect(v: P3, x0: P3, x1: P3, t0: P3, t1: P3) -> P3 {
    let reflect_in = |n: P3, x: P3| {
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

// ── twist, scale and guide rails ────────────────────────────────────────────

/// A linear map of the profile's plane, `[[a, b], [c, d]]` taking `(x, y)`
/// to `(a x + b y, c x + d y)`.
type Mat = [[f64; 2]; 2];

const IDENTITY: Mat = [[1.0, 0.0], [0.0, 1.0]];

/// Steps of Newton's method finding where a rail crosses a section, at most.
const RAIL_NEWTON_STEPS: usize = 50;

/// `frame` with the profile in it mapped by `m` about the profile point
/// `c` first: `p` goes where `c + m (p - c)` went.
fn compose(frame: &Plain, m: &Mat, c: [f64; 2]) -> Plain {
    let (a, b) = offset(m, c);
    Plain {
        origin: add(frame.origin, add(scale(frame.e1, a), scale(frame.e2, b))),
        e1: add(scale(frame.e1, m[0][0]), scale(frame.e2, m[1][0])),
        e2: add(scale(frame.e1, m[0][1]), scale(frame.e2, m[1][1])),
    }
}

/// The derivative of `compose(frame, m, c)`, given those of `frame` and `m`.
fn compose_derivative(frame: &Plain, d_frame: &Plain, m: &Mat, d_m: &Mat, c: [f64; 2]) -> Plain {
    let moved = compose(d_frame, m, c);
    let (da, db) = (
        -(d_m[0][0] * c[0] + d_m[0][1] * c[1]),
        -(d_m[1][0] * c[0] + d_m[1][1] * c[1]),
    );
    Plain {
        origin: add(moved.origin, add(scale(frame.e1, da), scale(frame.e2, db))),
        e1: add(
            moved.e1,
            add(scale(frame.e1, d_m[0][0]), scale(frame.e2, d_m[1][0])),
        ),
        e2: add(
            moved.e2,
            add(scale(frame.e1, d_m[0][1]), scale(frame.e2, d_m[1][1])),
        ),
    }
}

/// `c - m c`: where the profile's origin goes under the map `m` about `c`.
fn offset(m: &Mat, c: [f64; 2]) -> (f64, f64) {
    (
        c[0] - (m[0][0] * c[0] + m[0][1] * c[1]),
        c[1] - (m[1][0] * c[0] + m[1][1] * c[1]),
    )
}

/// [`compose`] on a frame of scalars: the map, a free choice made in plain
/// numbers, taken as sharp. The identity leaves the frame as it is.
fn compose_frame(frame: &Frame, m: &Mat, c: [f64; 2]) -> Frame {
    if *m == IDENTITY {
        return frame.clone();
    }
    let f = Scalar::from_f64;
    let (a, b) = offset(m, c);
    let combine = |x: f64, y: f64| frame.e1.prod_scalar(f(x)).add(&frame.e2.prod_scalar(f(y)));
    Frame {
        origin: frame.origin.add(&combine(a, b)),
        e1: combine(m[0][0], m[1][0]),
        e2: combine(m[0][1], m[1][1]),
    }
}

/// `x` in the coordinates of `frame`'s plane: the `(x, y)` whose point
/// `origin + x e1 + y e2` is nearest to it.
fn in_plane(frame: &Plain, x: P3) -> [f64; 2] {
    let d = sub(x, frame.origin);
    let (g11, g12, g22) = (
        dot(frame.e1, frame.e1),
        dot(frame.e1, frame.e2),
        dot(frame.e2, frame.e2),
    );
    solve2(
        [[g11, g12], [g12, g22]],
        [dot(d, frame.e1), dot(d, frame.e2)],
    )
}

/// A rail followed along the sweep: where along it the last section
/// crossed it, as a parameter running `0..n` over its `n` curves.
struct Tracked<'a> {
    rail: &'a Chain<NurbCurve3D>,
    chain: Chain<NurbCurve3D>,
    tau: f64,
}

impl<'a> Tracked<'a> {
    /// `rail`, run from its end on the profile's `plane` — an error if
    /// neither end could be on it.
    fn new(rail: &'a Chain<NurbCurve3D>, plane: &CoordinateSystem) -> GeopResult<Self> {
        if rail.is_closed() {
            return Err(GeopError::new(format!(
                "path sweep: the rail {} is a closed loop: a rail is an open chain of curves, starting on the profile's plane",
                rail.name
            )));
        }
        let Some(chain) = rail.starting_on(plane)? else {
            return Err(GeopError::new(format!(
                "path sweep: the rail {} starts on the profile's plane at neither end: draw it from a point of the profile",
                rail.name
            )));
        };
        Ok(Self {
            rail,
            chain,
            tau: 0.0,
        })
    }

    fn length(&self) -> f64 {
        self.chain.curves.len() as f64
    }

    /// The curve `tau` is on, and the parameter along it.
    fn local(&self, tau: f64) -> (usize, f64) {
        let i = (tau.floor() as usize).min(self.chain.curves.len() - 1);
        (i, tau - i as f64)
    }

    /// The rail at `tau`, and its derivative.
    fn at(&self, tau: f64) -> GeopResult<(P3, P3)> {
        let (i, t) = self.local(tau);
        let curve = &self.chain.curves[i];
        let t = Scalar::from_f64(t);
        Ok((
            curve.evaluate(t)?.to_f64_array(),
            curve.tangent(t)?.to_f64_array(),
        ))
    }

    /// Where the rail crosses the plane of `frame`, in its coordinates:
    /// Newton's method from where it crossed the last section, until its
    /// steps stop shrinking — where rounding takes over. An error if the
    /// rail runs along the section, ends before it, or the steps never stop
    /// shrinking.
    fn crossing(&mut self, frame: &Plain) -> GeopResult<[f64; 2]> {
        let normal = cross(frame.e1, frame.e2);
        // The rail reaches the last section (see `reaches_the_end`), so a
        // step past its end is a step past a crossing at the end.
        let (tau, arrived) = newton_1d(
            self.tau,
            (0.0, self.length()),
            RAIL_NEWTON_STEPS,
            Arrive::StepsStopShrinking,
            |tau| -> GeopResult<_> {
                let (x, d) = self.at(tau)?;
                let slope = dot(d, normal);
                if slope == 0.0 {
                    return Err(GeopError::new(format!(
                        "path sweep: the rail {} runs along a section of the sweep, at its parameter {tau:?}",
                        self.rail.name
                    )));
                }
                Ok((dot(sub(x, frame.origin), normal), slope))
            },
        )?;
        if !arrived {
            return Err(GeopError::new(format!(
                "path sweep: could not find where the rail {} crosses a section near its parameter {:?}",
                self.rail.name, self.tau
            )));
        }
        self.tau = tau;
        Ok(in_plane(frame, self.at(tau)?.0))
    }

    /// Checks the rail reaches the plane of the path's last station, `end`,
    /// which the path runs into along its normal if `along_normal`: its end
    /// could lie on or beyond it — decided on the exact station and rail, so
    /// that a rail ending exactly there counts as reaching it.
    fn reaches_the_end(&self, end: &Frame, along_normal: bool) -> GeopResult<()> {
        let joints = self.chain.joints()?;
        let normal = end.e1.prod_cross(&end.e2);
        let ahead = joints[joints.len() - 1].sub(&end.origin).prod_dot(&normal);
        let ahead = if along_normal { ahead } else { ahead.neg() };
        if ahead.definitely_less(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "path sweep: the rail {} ends before the path does: it has to reach the plane of every section, the last one too",
                self.rail.name
            )));
        }
        Ok(())
    }

    /// Whether the rail runs straight from `from` to `to`: both on one of
    /// its curves, a line.
    fn straight_between(&self, from: f64, to: f64) -> bool {
        let (i, _) = self.local(from);
        let curve = &self.chain.curves[i];
        to <= (i + 1) as f64 && curve.degree == 1 && curve.control_points.len() == 2
    }
}

/// The map the rails make of the profile about `c`, given where each rail
/// crossed the plane it is drawn in, `start`, and where it crosses this
/// section, `now`: about `c`, with one rail, the turn and uniform scale
/// taking its start where it is now; with two, the linear map taking both
/// starts where they are now.
fn rail_map(c: [f64; 2], start: &[[f64; 2]], now: &[[f64; 2]]) -> Mat {
    let rel = |p: [f64; 2]| sub(p, c);
    match start.len() {
        1 => {
            // (now - c) / (start - c), as complex numbers.
            let (q, r) = (rel(start[0]), rel(now[0]));
            let n = dot(q, q);
            let (a, b) = (dot(r, q) / n, cross2(q, r) / n);
            [[a, -b], [b, a]]
        }
        _ => {
            // [r0 r1] [q0 q1]^-1, the points as columns.
            let (q0, q1, r0, r1) = (rel(start[0]), rel(start[1]), rel(now[0]), rel(now[1]));
            let inv = inverse2([[q0[0], q1[0]], [q0[1], q1[1]]]);
            let r = [[r0[0], r1[0]], [r0[1], r1[1]]];
            std::array::from_fn(|i| {
                std::array::from_fn(|j| r[i][0] * inv[0][j] + r[i][1] * inv[1][j])
            })
        }
    }
}

/// The derivatives of `maps`, evenly spaced over `[0, 1]`, by finite
/// differences: central inside, of second order at the ends.
fn map_derivatives(maps: &[Mat]) -> Vec<Mat> {
    let n = maps.len() - 1;
    let h = 1.0 / n as f64;
    (0..=n)
        .map(|i| {
            std::array::from_fn(|r| {
                std::array::from_fn(|s| {
                    let m = |k: usize| maps[k][r][s];
                    if i == 0 {
                        (-3.0 * m(0) + 4.0 * m(1) - m(2)) / (2.0 * h)
                    } else if i == n {
                        (3.0 * m(n) - 4.0 * m(n - 1) + m(n - 2)) / (2.0 * h)
                    } else {
                        (m(i + 1) - m(i - 1)) / (2.0 * h)
                    }
                })
            })
        })
        .collect()
}

/// The frame `start` carried along curve `k` of the path at `fine + 1`
/// evenly spaced parameters of its span, the way the plain path carries it
/// (see [`carry`] and [`translated`]): moved straight along a line, turned
/// about an arc's axis, by a rotation minimizing frame along anything else
/// — or, keeping a fixed normal, only moved along the curve.
fn carried_samples(
    curve: &NurbCurve3D,
    kind: &Kind,
    start: &Frame,
    fine: usize,
    orientation: Orientation,
) -> GeopResult<Vec<Sample>> {
    let begin = Plain::of(start);
    let fraction = |i: usize| i as f64 / fine as f64;
    let still = |frame: Plain, d_origin: P3, point: P3| Sample {
        frame,
        derivative: Plain {
            origin: d_origin,
            e1: [0.0; 3],
            e2: [0.0; 3],
        },
        point,
    };
    if orientation == Orientation::FixedNormal {
        let c0 = curve.start()?.to_f64_array();
        return (0..=fine)
            .map(|i| {
                let t = Scalar::from_f64(fraction(i));
                let point = curve.evaluate(t)?.to_f64_array();
                let frame = Plain {
                    origin: add(begin.origin, sub(point, c0)),
                    ..begin
                };
                Ok(still(frame, curve.tangent(t)?.to_f64_array(), point))
            })
            .collect();
    }
    match kind {
        Kind::Line => {
            let c0 = curve.start()?.to_f64_array();
            let step = sub(curve.end()?.to_f64_array(), c0);
            Ok((0..=fine)
                .map(|i| {
                    let u = fraction(i);
                    let frame = Plain {
                        origin: add(begin.origin, scale(step, u)),
                        ..begin
                    };
                    still(frame, step, add(c0, scale(step, u)))
                })
                .collect())
        }
        Kind::Arc {
            start: p0,
            middle: corner,
            rotation,
            ..
        } => {
            let axis = rotation.axis.to_f64_array();
            let angle = rotation.sin.to_f64().atan2(rotation.cos.to_f64());
            // The centre lies square to the tangent at the start, towards
            // the turn (a velocity `axis x (p - centre)` points along the
            // tangent), a tangent leg's length over tan(angle / 2) away.
            let (p0, tangent) = (
                p0.to_f64_array(),
                sub(corner.to_f64_array(), p0.to_f64_array()),
            );
            let towards = cross(axis, tangent);
            let towards = scale(towards, 1.0 / dot(towards, towards).sqrt());
            let radius = dot(tangent, tangent).sqrt() / (angle / 2.0).tan();
            let center = add(p0, scale(towards, radius));
            let c0 = curve.start()?.to_f64_array();
            Ok((0..=fine)
                .map(|i| {
                    let (sin, cos) = (angle * fraction(i)).sin_cos();
                    // Rodrigues, about the axis through the centre.
                    let turn = |v: P3| {
                        add(
                            add(scale(v, cos), scale(cross(axis, v), sin)),
                            scale(axis, dot(axis, v) * (1.0 - cos)),
                        )
                    };
                    let frame = Plain {
                        origin: add(center, turn(sub(begin.origin, center))),
                        e1: turn(begin.e1),
                        e2: turn(begin.e2),
                    };
                    let spin = |v: P3| scale(cross(axis, v), angle);
                    Sample {
                        frame,
                        derivative: Plain {
                            origin: spin(sub(frame.origin, center)),
                            e1: spin(frame.e1),
                            e2: spin(frame.e2),
                        },
                        point: add(center, turn(sub(c0, center))),
                    }
                })
                .collect())
        }
        Kind::Other => reflected(curve, start, fine),
    }
}

/// The path along `chain` with the profile changing as `control` says (see
/// [`Control`]), from the plain one, `rigid`, carrying the profile the way
/// `control.orientation` says — its stations, mitred at corners, and
/// `starts`, the frame unmitred where each curve starts.
///
/// Each span is sampled: the rigid frame at evenly spaced parameters (see
/// [`carried_samples`]), the profile in it mapped about the path's start
/// point in the profile's plane by a twist and scale growing evenly with the
/// length travelled — or the map the rails make (see [`rail_map`]). A
/// station is its rigid station so mapped; between them, the sections are
/// joined by cubic Hermite interpolation, as along a spline, blended in so
/// that the span runs from station to station exactly, a mitre included.
/// A line along which the map grows linearly — no twist, and every rail
/// straight there — stays a line between its stations.
///
/// The map is a free choice made in plain numbers, taken as sharp, like
/// the frames along a spline: the swept body is defined by it, every edge
/// and face built from the same frames.
fn controlled(
    chain: &Chain<NurbCurve3D>,
    kinds: &[Kind],
    plane: &CoordinateSystem,
    rigid: &Path,
    starts: &[Frame],
    corner: &[bool],
    control: &Control,
) -> GeopResult<Path> {
    let n = chain.curves.len();
    let rails_given = !control.rails.is_empty();
    if chain.is_closed() {
        return Err(GeopError::new(
            "path sweep: a closed path sweeps a ring, which a twist, a scale or a rail would not close: sweep along an open path",
        ));
    }
    if control.rails.len() > 2 {
        return Err(GeopError::new(format!(
            "path sweep: {} rails given, but a sweep follows one or two",
            control.rails.len()
        )));
    }
    if rails_given && (control.twist != 0.0 || control.end_scale != 1.0) {
        return Err(GeopError::new(
            "path sweep: the rails decide how the profile turns and scales: no twist or scale with them",
        ));
    }
    if !(control.end_scale > 0.0 && control.end_scale.is_finite() && control.twist.is_finite()) {
        return Err(GeopError::new(format!(
            "path sweep: cannot scale the profile to {} times its size: the end scale is more than zero",
            control.end_scale
        )));
    }
    if rails_given && let Some(k) = (0..n).find(|&k| corner[k]) {
        return Err(GeopError::new(format!(
            "path sweep: the path turns a corner at {}, which the profile cannot follow rails round: make it tangent there",
            chain.joint_names[k]
        )));
    }

    // The profile turns and scales about the path's start, in its plane.
    let c = {
        let p = plane.to_uvw(&chain.curves[0].start()?);
        [p[0].to_f64(), p[1].to_f64()]
    };
    let mut rails = control
        .rails
        .iter()
        .map(|rail| Tracked::new(rail, plane))
        .collect::<GeopResult<Vec<_>>>()?;
    for rail in &rails {
        rail.reaches_the_end(&rigid.stations[n], rigid.along_normal)?;
    }
    // Where each rail starts, from the path's start, in the profile's plane:
    // apart from it — and, for two, not in line with it — or they cannot
    // say how the profile scales.
    let offsets: Vec<Vector3> = rails
        .iter()
        .map(|r| {
            Ok(plane
                .to_uvw(&r.chain.curves[0].start()?)
                .sub(&plane.to_uvw(&chain.curves[0].start()?)))
        })
        .collect::<GeopResult<_>>()?;
    let degenerate = match offsets.as_slice() {
        [] => false,
        [q] => q[0].could_be_equal(Scalar::ZERO) && q[1].could_be_equal(Scalar::ZERO),
        [q0, q1, ..] => q0[0]
            .mul(q1[1])
            .sub(q1[0].mul(q0[1]))
            .could_be_equal(Scalar::ZERO),
    };
    if degenerate {
        return Err(GeopError::new(format!(
            "path sweep: the rails {} could start in line with the path, so they cannot say how the profile scales: start them off it",
            control
                .rails
                .iter()
                .map(|r| r.name.as_str())
                .collect::<Vec<_>>()
                .join(" and ")
        )));
    }
    let rail_starts: Vec<[f64; 2]> = offsets
        .iter()
        .map(|q| [c[0] + q[0].to_f64(), c[1] + q[1].to_f64()])
        .collect();

    // Sections: as along a spline, and more for a twist or for rails.
    let lengths = chain
        .curves
        .iter()
        .map(|curve| Ok(curve.length()?.0.midpoint().to_f64()))
        .collect::<GeopResult<Vec<f64>>>()?;
    let total: f64 = lengths.iter().sum();
    let rail_sections = rails
        .iter()
        .map(|r| r.chain.curves.iter().map(knot_spans).sum::<usize>())
        .max()
        .unwrap_or(0)
        * SECTIONS_PER_KNOT_SPAN;
    // A section every eighth of a half turn of twist, at least.
    let turn_per_section = std::f64::consts::FRAC_PI_8;

    let mut samples: Vec<Vec<Sample>> = Vec::with_capacity(n);
    for k in 0..n {
        let curve = &chain.curves[k];
        let twist_sections =
            (control.twist.abs() * lengths[k] / total / turn_per_section).ceil() as usize;
        let sections = (SECTIONS_PER_KNOT_SPAN * knot_spans(curve))
            .max(twist_sections)
            .max(rail_sections);
        samples.push(carried_samples(
            curve,
            &kinds[k],
            &starts[k],
            sections * REFLECTION_STEPS,
            control.orientation,
        )?);
    }

    // How far along the path each sample is, as a fraction of its length.
    let mut along: Vec<Vec<f64>> = Vec::with_capacity(n);
    let mut travelled = 0.0;
    for span in &samples {
        let mut fractions = vec![travelled];
        for w in span.windows(2) {
            let d = sub(w[1].point, w[0].point);
            travelled += dot(d, d).sqrt();
            fractions.push(travelled);
        }
        along.push(fractions);
    }
    let along: Vec<Vec<f64>> = along
        .into_iter()
        .map(|f| f.into_iter().map(|s| s / travelled).collect())
        .collect();

    // The map at every sample, and where each rail is at either end of
    // every span.
    let mut maps: Vec<Vec<Mat>> = Vec::with_capacity(n);
    let mut rail_ends: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n);
    for k in 0..n {
        let mut span_maps = Vec::with_capacity(samples[k].len());
        let before: Vec<f64> = rails.iter().map(|r| r.tau).collect();
        for (i, sample) in samples[k].iter().enumerate() {
            let map = if i == 0 && k == 0 {
                IDENTITY
            } else if i == 0 {
                // The joint, as the span before ended.
                *maps[k - 1].last().expect("a span has samples")
            } else if rails_given {
                let now = rails
                    .iter_mut()
                    .map(|r| r.crossing(&sample.frame))
                    .collect::<GeopResult<Vec<_>>>()?;
                rail_map(c, &rail_starts, &now)
            } else {
                let s = along[k][i];
                let size = 1.0 + (control.end_scale - 1.0) * s;
                let (sin, cos) = (control.twist * s).sin_cos();
                [[size * cos, -size * sin], [size * sin, size * cos]]
            };
            let det = map[0][0] * map[1][1] - map[0][1] * map[1][0];
            if det <= 0.0 || det.is_nan() {
                return Err(GeopError::new(format!(
                    "path sweep: the rails squeeze the profile flat or turn it over along {}",
                    chain.curve_names[k]
                )));
            }
            span_maps.push(map);
        }
        maps.push(span_maps);
        rail_ends.push(rails.iter().zip(before).map(|(r, b)| (b, r.tau)).collect());
    }

    // The stations: the rigid ones, mapped.
    let stations: Vec<Frame> = (0..=n)
        .map(|k| {
            let map = if k < n {
                &maps[k][0]
            } else {
                maps[n - 1].last().expect("samples")
            };
            compose_frame(&rigid.stations[k], map, c)
        })
        .collect();

    let mut spans = Vec::with_capacity(n);
    for k in 0..n {
        let straight = matches!(kinds[k], Kind::Line)
            && control.twist == 0.0
            && rails
                .iter()
                .zip(&rail_ends[k])
                .all(|(r, &(from, to))| r.straight_between(from, to));
        if straight {
            spans.push(Span::Line);
            continue;
        }
        let d_maps = map_derivatives(&maps[k]);
        let last = samples[k].len() - 1;
        let mapped = |i: usize| compose(&samples[k][i].frame, &maps[k][i], c);
        // What the span has to be moved by at either end to run from
        // station to station: a mitre, or rounding.
        let to_start = Plain::of(&stations[k]).minus(&mapped(0));
        let to_end = Plain::of(&stations[k + 1]).minus(&mapped(last));
        let sections: Vec<Sample> = (0..=last)
            .step_by(REFLECTION_STEPS)
            .map(|i| {
                let u = i as f64 / last as f64;
                let sample = &samples[k][i];
                Sample {
                    frame: mapped(i).step(&to_start, 1.0 - u).step(&to_end, u),
                    derivative: compose_derivative(
                        &sample.frame,
                        &sample.derivative,
                        &maps[k][i],
                        &d_maps[i],
                        c,
                    )
                    .step(&to_start, -1.0)
                    .step(&to_end, 1.0),
                    point: sample.point,
                }
            })
            .collect();
        spans.push(hermite(&sections)?);
    }
    Ok(Path {
        stations,
        spans,
        ..rigid.clone()
    })
}

/// Refuses `loops` where they reach to or across the axis of a circular
/// arc of `chain`, swept along it as `path` carries them: turned about that
/// axis, the profile would sweep through itself — a bend tighter than the
/// profile is wide.
///
/// Where an arc starts, the profile's plane holds the arc's axis and its
/// radius there, so the profile is clear of the axis if every point of it
/// lies definitely on the radius's side. Each profile curve lies within the
/// hull of its control points, and of its quarters' — which hug it closely
/// enough that a profile is only refused where it comes within a hair of
/// the axis.
fn clear_of_bends(chain: &Chain<NurbCurve3D>, path: &Path, loops: &[SweepLoop]) -> GeopResult<()> {
    for (k, curve) in chain.curves.iter().enumerate() {
        let Kind::Arc {
            start,
            middle,
            rotation,
            ..
        } = kind(curve)?
        else {
            continue;
        };
        // Square to the tangent at the start, towards the turn, the tangent
        // leg's length over tan(angle / 2) = sin / (1 + cos) away.
        let leg = middle.sub(&start);
        let radius = leg
            .norm()
            .mul(Scalar::ONE.add(rotation.cos))
            .div(rotation.sin)?;
        let center = start.add(
            &rotation
                .axis
                .prod_cross(&leg)
                .normalize()?
                .prod_scalar(radius),
        );
        let station = &path.stations[k];
        let radial = curve.start()?.sub(&center);
        for profile in loops.iter().map(|l| &l.profile) {
            for (c, name) in profile.curves.iter().zip(&profile.curve_names) {
                let (a, b) = c.split_mid()?;
                let (a, b) = (a.split_mid()?, b.split_mid()?);
                for piece in [a.0, a.1, b.0, b.1] {
                    for cp in &piece.control_points {
                        let p = Vector2::from_array([cp[0].div(cp[2])?, cp[1].div(cp[2])?]);
                        let outward = station.point(&p).sub(&center).prod_dot(&radial);
                        if !outward.definitely_greater(Scalar::ZERO) {
                            return Err(GeopError::new(format!(
                                "path sweep: the profile's {name} reaches across the axis of the bend {}: make the bend's radius larger than the profile reaches from the path",
                                chain.curve_names[k]
                            )));
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

/// Where the profile `loops`, drawn in `plane`, is: the centre of its outer
/// loop's joints — a point to find the nearest end of the path from.
pub fn profile_centre(plane: &CoordinateSystem, loops: &[SweepLoop]) -> GeopResult<[f64; 3]> {
    let outer = &loops
        .first()
        .ok_or_else(|| GeopError::new("sweep: the profile has no curves"))?
        .profile;
    let starts = outer
        .curves
        .iter()
        .map(|curve| Ok(plane.uv_to_xyz(&curve.start()?).to_f64_array()))
        .collect::<GeopResult<Vec<_>>>()?;
    Ok(centre(&starts))
}

/// `chain` starting where it comes nearest to `centre`: an open one at its
/// nearer end, a closed one at its nearest joint. Which is a free choice of
/// where to start; the nearest is what the profile is drawn at.
pub fn starting_near(
    chain: Chain<NurbCurve3D>,
    centre: &[f64; 3],
) -> GeopResult<Chain<NurbCurve3D>> {
    let distance =
        |p: &Vector3| -> f64 { (0..3).map(|k| (p[k].to_f64() - centre[k]).powi(2)).sum() };
    let joints = chain.joints()?;
    Ok(if chain.is_closed() {
        let nearest = (0..joints.len())
            .min_by(|&a, &b| distance(&joints[a]).total_cmp(&distance(&joints[b])))
            .expect("a chain has joints");
        chain.starting_at(nearest)
    } else if distance(&joints[joints.len() - 1]) < distance(&joints[0]) {
        chain.reversed()
    } else {
        chain
    })
}

/// Rebuilds a body some of whose faces were put on new surfaces, around
/// them: [`geop_lib_shell::rebuilt`] and [`geop_lib_shell::room`], which
/// this crate cannot depend on (its tests sweep their solids), handed in by
/// the caller. `None` where the caller has none: a corner at an arc or a
/// spline is then refused.
#[derive(Clone, Copy, Debug)]
pub struct Rebuild {
    /// A new surface given room around it, as `rebuilt` asks of each.
    pub room: fn(NurbSurface3D) -> GeopResult<NurbSurface3D>,
    /// The body rebuilt around the faces marked replaced, and the vertices
    /// marked to reseat.
    pub rebuilt: fn(&BodySpec, &BodyNames, &[bool], &[bool]) -> GeopResult<BodySpec>,
}

/// Sweeps `loops` — the first the outer loop, counter-clockwise in
/// `plane`'s `(u, v)`, the rest holes in it, clockwise — along `chain`,
/// changing as `control` says (see [`followed`]): into a solid named
/// `solid`, or, without one, into sheets. Either way round, the solid comes
/// out with its faces pointing outwards.
///
/// Where the path turns a corner at an arc or a spline, the profile
/// projected onto the plane bisecting the corner lies on neither swept
/// wall, as it does between two lines. Each wall there is then put on the
/// surface the profile, carried rigidly, sweeps along its own curve, and
/// the body rebuilt around them (`rebuild`): the walls either side of the
/// corner meet where they cross. A joint of the profile is where its paths
/// either side cross the walls on the other side: for a planar path, one
/// point, since both paths run at the joint's height above the path's
/// plane; otherwise two, one each, joined by a piece of the crossing (see
/// [`apart_at_corner`]). Where the walls either side touch, the profile's
/// normal along the axis the path turns about, they cross in an X: a point
/// inside a curve of the profile is made a joint of it there, named
/// `X@J` for the curve `X` and the corner `J`, and is one vertex. Refused,
/// by name, for a path that twists, scales or follows rails round such a
/// corner.
///
/// Named after the profiles' curves `X` and joints `P` and the chain's
/// curves `C` and joints `J` (see [`crate::sweep::sweep`]): the walls `N(X,C)`, their
/// edges at the joints `N(X,J)`, the edges between `N(P,C)`, the vertices
/// `N(P,J)`; for an open chain, the caps `N(start)` and `N(end)`.
#[allow(clippy::too_many_arguments)]
pub fn sweep_along(
    model: &mut NamedModel,
    namer: &Namer,
    solid: Option<&str>,
    chain: &Chain<NurbCurve3D>,
    plane: &CoordinateSystem,
    loops: &[SweepLoop],
    control: &Control,
    rebuild: Option<Rebuild>,
) -> GeopResult<BuiltBody> {
    let along = followed(chain, plane, control)?;
    let path = &along.path;
    // Whether a bend is tighter than the profile reaches is a question
    // about the profile carried rigidly along the path: a sampled path's
    // stations no longer match the chain's curves one to one.
    let rigid = match control.is_plain() {
        true => None,
        false => Some(followed(chain, plane, &Control::default())?.path),
    };
    clear_of_bends(chain, rigid.as_ref().unwrap_or(path), loops)?;
    let Some(&first) = along.rebuilt.first() else {
        return sweep_drawn(model, namer, path, loops, solid);
    };
    let Some(rebuild) = rebuild else {
        return Err(GeopError::new(format!(
            "path sweep: the path turns a corner at {}, where only two lines can meet at an angle: make it tangent there",
            chain.joint_names[first]
        )));
    };
    let ctx = with_context!(
        "path sweep: rebuilding the walls at the corners {:?}",
        (along.rebuilt.iter())
            .map(|&k| &chain.joint_names[k])
            .collect::<Vec<_>>()
    );
    let drawn: Vec<SweepLoop> = if path.along_normal {
        loops.iter().map(SweepLoop::reversed).collect()
    } else {
        loops.to_vec()
    };
    // Cut at its corners, as [`crate::sweep::sweep`] cuts what it sweeps.
    let mut drawn = (drawn.iter())
        .map(SweepLoop::at_corners)
        .collect::<GeopResult<Vec<_>>>()
        .with_context(ctx)?;
    let n = chain.curves.len();
    let corner = |k: usize| -> GeopResult<Corner<'_>> {
        Ok(Corner {
            station: k,
            frame: &along.rigid[(k + n - 1) % n][1],
            axis: (chain.curves[(k + n - 1) % n].tangent(Scalar::ONE)?)
                .prod_cross(&chain.curves[k].tangent(Scalar::ZERO)?),
        })
    };
    // Where a wall before a corner touches the wall of its curve after it
    // — the profile's normal there along the axis the path turns about,
    // which the turn leaves as it is — the two cross in an X, not along a
    // curve. Inside a curve of the profile, that X would lie inside the
    // edge the corner's station gives it, where nothing can trace the edge
    // or tell whether its faces run on smoothly. A joint of the profile
    // there makes it a vertex, which the turn leaves where it is (see
    // [`apart_at_corner`]), and each edge of the curve ends at it.
    for &k in &along.rebuilt {
        let corner = corner(k)?;
        let frame = corner.frame;
        let across = Vector2::from_array([
            corner.axis.prod_dot(&frame.e1),
            corner.axis.prod_dot(&frame.e2),
        ]);
        for lp in &mut drawn {
            let mut touches = Vec::new();
            for (curve, name) in lp.profile.curves.iter().zip(&lp.profile.curve_names) {
                let (t0, t1) = curve.domain();
                let stationary = curve
                    .stationary_parameters(&across, MAX_NODES, min_subdivision_size())
                    .with_context(ctx)?;
                for t in stationary {
                    if t.could_be_equal(t0) || t.could_be_equal(t1) {
                        continue;
                    }
                    // The search isolates it in a box; the walls touch at
                    // one point of it, which Newton finds: a joint a hair
                    // off it would have the joint's paths graze the walls
                    // on the other side.
                    let (t, _) = newton_1d(
                        t.sharpen(),
                        (t0, t1),
                        PROJECT_ITERATIONS,
                        Arrive::AtZero,
                        |t| -> GeopResult<_> {
                            Ok((
                                curve.tangent(t)?.prod_dot(&across),
                                curve.second_derivative(t)?.prod_dot(&across),
                            ))
                        },
                    )
                    .with_context(ctx)?;
                    let joint = format!("{name}@{}", path.station_names[k]);
                    touches.push((curve.evaluate(t)?, joint));
                }
            }
            for (point, joint) in touches {
                let (marked, _) = mark(&lp.profile, &point, &joint).with_context(ctx)?;
                *lp = SweepLoop::plain(marked);
            }
        }
    }
    let sections = vec![drawn.clone(); path.stations.len()];
    let (mut spec, names) = skinned(namer, path, &sections, solid).with_context(ctx)?;
    // The walls of the spans either side of each such corner, as the
    // profile carried rigidly along each sweeps them.
    let mut swept: HashMap<String, NurbSurface3D> = HashMap::new();
    for &k in &along.rebuilt {
        for j in [(k + n - 1) % n, k] {
            let Some(span) = &path.span_names[j] else {
                continue;
            };
            let alone = Path {
                stations: along.rigid[j].to_vec(),
                spans: vec![path.spans[j].clone()],
                closed: false,
                along_normal: path.along_normal,
                station_names: vec!["from".into(), "to".into()],
                span_names: vec![Some(span.clone())],
            };
            for (l, lp) in drawn.iter().enumerate() {
                for (i, curve) in lp.profile.curve_names.iter().enumerate() {
                    let surface = wall(&alone, &drawn, (l, 0, i)).with_context(ctx)?;
                    swept.insert(namer.name(&[curve, span]), surface);
                }
            }
        }
    }
    let mut replaced = vec![false; spec.faces.len()];
    for (f, name) in names.faces.iter().enumerate() {
        if let Some(surface) = swept.remove(name) {
            spec.faces[f].surface = (rebuild.room)(surface).with_context(ctx)?;
            replaced[f] = true;
        }
    }
    let mut names = names;
    for &k in &along.rebuilt {
        for lp in &drawn {
            let closed = lp.profile.is_closed();
            let joints = lp.profile.curves.len() + usize::from(closed) - 1;
            for i in (0..joints).map(|i| i + usize::from(!closed)) {
                let corner = corner(k)?;
                apart_at_corner(&mut spec, &mut names, namer, path, &lp.profile, &corner, i)
                    .with_context(ctx)?;
            }
        }
    }
    let reseat = vec![false; spec.vertices.len()];
    let spec = (rebuild.rebuilt)(&spec, &names, &replaced, &reseat).with_context(ctx)?;
    model.build_body(spec, names).with_context(ctx)
}

/// A corner of the path at an arc or a spline: its station, the frame the
/// profile is carried to it in along the curve before it, and the axis the
/// path turns about there.
struct Corner<'a> {
    station: usize,
    frame: &'a Frame,
    axis: Vector3,
}

/// The vertex of joint `i` of `profile` at the corner at `corner.station` of
/// `path`, in `spec`, named by `names` — the walls either side on the
/// surfaces the profile sweeps along each curve, with room — taken apart
/// where the joint's paths either side cross the walls on the other side
/// at two points: one vertex where the path before the corner crosses, one
/// where the path after it does, and an edge between them, where a wall
/// before the corner crosses one after it.
///
/// Which of the walls after the corner the path before it crosses — that
/// of the curve arriving at the joint, or of the one leaving it — says
/// which comes first along the profile, and so which two walls the new
/// edge parts.
///
/// Where a wall before the corner touches the wall of its curve after it
/// at the joint — the profile's normal there along the axis the path turns
/// about, which the turn leaves as it is — the joint's paths cross the
/// walls on the other side tangentially, at no point a search can isolate:
/// the vertex stays one, where the smooth surface across the corner meets
/// the others (see [`Rebuild`]).
fn apart_at_corner(
    spec: &mut BodySpec,
    names: &mut BodyNames,
    namer: &Namer,
    path: &Path,
    profile: &Profile,
    corner: &Corner<'_>,
    i: usize,
) -> GeopResult<()> {
    let k = corner.station;
    let m = profile.curves.len();
    let arriving = (i + m - 1) % m;
    for tangent in [
        profile.curves[arriving].tangent(Scalar::ONE)?,
        profile.curves[i].tangent(Scalar::ZERO)?,
    ] {
        let normal =
            (corner.frame.e1.prod_scalar(tangent[1])).sub(&corner.frame.e2.prod_scalar(tangent[0]));
        let along = normal.prod_cross(&corner.axis);
        if (0..3).all(|c| along[c].could_be_equal(Scalar::ZERO)) {
            return Ok(());
        }
    }
    let before = (k + path.spans.len() - 1) % path.spans.len();
    let (Some(span_before), Some(span_after)) = (&path.span_names[before], &path.span_names[k])
    else {
        return Ok(());
    };
    let (joint, station) = (&profile.joint_names[i], &path.station_names[k]);
    let (arriving_name, leaving_name) = (&profile.curve_names[arriving], &profile.curve_names[i]);
    let index = |of: &[String], name: String| {
        of.iter()
            .position(|n| *n == name)
            .ok_or_else(|| GeopError::new(format!("path sweep: nothing named {name}")))
    };
    let v = index(&names.vertices, namer.name(&[joint, station]))?;
    let face = |curve: &str, span: &str| index(&names.faces, namer.name(&[curve, span]));
    let [before_in, before_out, after_in, after_out] = [
        face(arriving_name, span_before)?,
        face(leaving_name, span_before)?,
        face(arriving_name, span_after)?,
        face(leaving_name, span_after)?,
    ];
    let edge = |name: String| index(&names.edges, name);
    let path_after = edge(namer.name(&[joint, span_after]))?;
    let [station_in, station_out] = [
        edge(namer.name(&[arriving_name, station]))?,
        edge(namer.name(&[leaving_name, station]))?,
    ];
    // The joint's path along a span: the leaving curve's wall where the
    // curve starts, carried on. Where it crosses the walls on the other
    // side nearest the corner — each within its curve's own stretch of
    // it — and whether that is the arriving curve's wall.
    let mitred = spec.vertices[v];
    let crossing = |along: usize, walls: [usize; 2]| -> GeopResult<(Vector3, bool)> {
        let start = profile.curves[i].domain().0;
        let seam = spec.faces[along].surface.iso_curve(false, start)?;
        let mut best: Option<(Vector3, bool, f64)> = None;
        for (wall, curve) in walls.into_iter().zip([arriving, i]) {
            let (from, to) = profile.curves[curve].domain();
            for (point, uv) in crossing_points(&seam, &spec.faces[wall].surface)? {
                if uv[1].definitely_less(from) || uv[1].definitely_greater(to) {
                    continue;
                }
                let distance = point.sub(&mitred).norm_sq().to_f64();
                if best.as_ref().is_none_or(|(.., d)| distance < *d) {
                    best = Some((point, curve == arriving, distance));
                }
            }
        }
        best.map(|(p, arrives, _)| (p, arrives)).ok_or_else(|| {
            GeopError::new(format!(
                "the path of {joint} along {} does not cross the walls on the other side of the corner {station}",
                names.faces[along]
            ))
        })
    };
    let (on_before, before_arrives) = crossing(before_out, [after_in, after_out])?;
    let (on_after, after_arrives) = crossing(after_out, [before_in, before_out])?;
    if on_before.could_be_equal(&on_after) {
        return Ok(());
    }
    // The path before the corner crossing the arriving curve's wall after
    // it comes first: past it, the wall before is the leaving curve's, the
    // one after still the arriving one's.
    let first_before = match (before_arrives, after_arrives) {
        (true, false) => true,
        (false, true) => false,
        _ => {
            return Err(GeopError::new(format!(
                "the paths of {joint} either side of the corner {station} cross the walls on the other side at {on_before:?} and {on_after:?}, which does not tell which comes first along the profile: not supported"
            )));
        }
    };
    let around = spec.vertex_faces()?;
    // A line between the two vertices on each face around, run either way.
    let mut pcurves: HashMap<usize, [Curve2; 2]> = HashMap::new();
    for &(f, uv) in &around[v] {
        let surface = &spec.faces[f].surface;
        let at = |p: Vector3| -> GeopResult<Vector2> {
            let (u, w) = surface.project_in_domain(
                p,
                uv[0].sharpen(),
                uv[1].sharpen(),
                PROJECT_ITERATIONS,
            )?;
            Ok(Vector2::from_array([u, w]))
        };
        let (a, b) = (at(on_before)?, at(on_after)?);
        pcurves.insert(f, [NurbCurve2D::line(a, b)?, NurbCurve2D::line(b, a)?]);
    }
    let moved = if first_before {
        [(path_after, false), (station_out, false)]
    } else {
        [(path_after, false), (station_in, true)]
    };
    spec.vertices[v] = on_before;
    let line = NurbCurve3D::line(on_before, on_after)?;
    spec.split_vertex(v, &moved, on_after, line, |f, sense| {
        let [forward, reversed] = pcurves.get(&f).ok_or_else(|| {
            GeopError::new(format!("path sweep: face {f} is not around the corner"))
        })?;
        Ok(match sense {
            Sense::Forward => forward.clone(),
            Sense::Reversed => reversed.clone(),
        })
    })?;
    names.vertices[v] = namer.name(&[joint, station, span_before]);
    names
        .vertices
        .push(namer.name(&[joint, station, span_after]));
    names.edges.push(namer.name(&[joint, station]));
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::polyline;
    use crate::shapes::{circle, plane, square, v2, v3};
    use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve2D, sqrt2_over_2};
    use geop_ops_model::Profile;
    use geop_ops_model::testing::{assert_model_valid, assert_model_valid_sheets};

    /// The `yz` plane through `origin`: square to a path along `x`.
    fn yz(origin: Vector3) -> CoordinateSystem {
        plane(origin, v3(0., 1., 0.), v3(0., 0., 1.))
    }

    fn chain(curves: Vec<NurbCurve3D>, closed: bool) -> Chain<NurbCurve3D> {
        let n = curves.len();
        Chain {
            name: "path".to_string(),
            curve_names: (0..n).map(|i| format!("k{i}")).collect(),
            joint_names: (0..n + usize::from(!closed))
                .map(|i| format!("j{i}"))
                .collect(),
            curves,
        }
    }

    /// Sweeps the closed `outer` loop in `plane` along `path` into a solid
    /// in a fresh part, and checks it is valid.
    fn swept(
        outer: Vec<NurbCurve2D>,
        plane: &CoordinateSystem,
        path: &Chain<NurbCurve3D>,
    ) -> NamedModel {
        let mut model = NamedModel::new();
        let namer = Namer::new("sweep", "s").unwrap();
        sweep_along(
            &mut model,
            &namer,
            Some(&namer.root()),
            path,
            plane,
            &[SweepLoop::plain(Profile::closed(outer))],
            &Control::default(),
            None,
        )
        .unwrap();
        model.check_names().unwrap();
        assert_model_valid(model.topology());
        model
    }

    /// Along a single line, a sweep is an extrude.
    #[test]
    fn sweep_along_a_line_is_a_box() {
        let path = chain(
            vec![NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap()],
            false,
        );
        let model = swept(square(0.5), &yz(Vector3::zero()), &path);
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 6);
        let far = model.vertex_id("sweep(s,p2,j1)").unwrap();
        assert!(
            topology
                .get_vertex(far)
                .unwrap()
                .point
                .could_be_equal(&v3(2., 0.5, 0.5))
        );
    }

    /// Two lines at a right angle: the profile is mitred at the corner, so
    /// the corner's vertices lie on the plane bisecting it.
    #[test]
    fn sweep_round_a_corner_is_mitred() {
        let path = chain(
            vec![
                NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                NurbCurve3D::line(v3(2., 0., 0.), v3(2., 2., 0.)).unwrap(),
            ],
            false,
        );
        let model = swept(square(0.5), &yz(Vector3::zero()), &path);
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 4 * 2 + 2);
        // The bisecting plane of the corner at (2, 0, 0): x - 2 = -y.
        for p in 0..4 {
            let v = model.vertex_id(&format!("sweep(s,p{p},j1)")).unwrap();
            let point = topology.get_vertex(v).unwrap().point;
            assert!(
                point[0]
                    .sub(Scalar::from_f64(2.0))
                    .could_be_equal(point[1].neg()),
                "{point:?}"
            );
        }
    }

    /// A pipe: a line, a quarter bend, a line — tangent all along.
    #[test]
    fn sweep_a_pipe_through_a_bend() {
        let path = chain(
            vec![
                NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                NurbCurve3D::arc(
                    v3(2., 0., 0.),
                    v3(3., 0., 0.),
                    v3(3., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
                NurbCurve3D::line(v3(3., 1., 0.), v3(3., 3., 0.)).unwrap(),
            ],
            false,
        );
        let model = swept(circle(0.3), &yz(Vector3::zero()), &path);
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 4 * 3 + 2);
        // The far end of the pipe lies square to the last line, at y = 3.
        let end = model.face_id("sweep(s,end)").unwrap();
        assert!(topology.iterate_face_coedges(end).all(|c| {
            topology.coedge_start_vertex(c).unwrap().point[1].could_be_equal(Scalar::from_f64(3.0))
        }));
    }

    /// Round a closed circle: a torus, with no caps.
    #[test]
    fn sweep_round_a_circle_is_a_torus() {
        let r = 2.0;
        let q = [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)];
        let arcs = (0..4)
            .map(|i| {
                let (a, b) = (q[i], q[(i + 1) % 4]);
                NurbCurve3D::arc(
                    v3(a.0, a.1, 0.),
                    v3(a.0 + b.0, a.1 + b.1, 0.),
                    v3(b.0, b.1, 0.),
                    sqrt2_over_2(),
                )
                .unwrap()
            })
            .collect();
        let path = chain(arcs, true);
        let profile_plane = plane(v3(2., 0., 0.), v3(1., 0., 0.), v3(0., 0., 1.));
        let model = swept(circle(0.5), &profile_plane, &path);
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 16);
        for v in topology.vertices.values() {
            // On the torus: (sqrt(x^2 + y^2) - 2)^2 + z^2 = 0.25.
            let (x, y, z) = (v.point[0], v.point[1], v.point[2]);
            let rho = x.mul(x).add(y.mul(y)).sqrt().unwrap();
            let d = rho.sub(Scalar::from_f64(2.0));
            let off = d.mul(d).add(z.mul(z));
            assert!(off.could_be_equal(Scalar::from_f64(0.25)), "{:?}", v.point);
        }
    }

    /// Round a closed square: a frame, every corner mitred, the first one
    /// too.
    #[test]
    fn sweep_round_a_square_is_a_frame() {
        let corners = [(0., 0.), (4., 0.), (4., 4.), (0., 4.)];
        let lines = (0..4)
            .map(|i| {
                let (a, b) = (corners[i], corners[(i + 1) % 4]);
                NurbCurve3D::line(v3(a.0, a.1, 0.), v3(b.0, b.1, 0.)).unwrap()
            })
            .collect();
        let path = chain(lines, true);
        let model = swept(square(0.5), &yz(Vector3::zero()), &path);
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 16);
        assert_eq!(topology.vertices.len(), 16);
    }

    /// Along a cubic spline: skinned sections, ending square to it.
    #[test]
    fn sweep_along_a_spline() {
        let f = Scalar::from_f64;
        let p = |x: f64, y: f64| Vector4::from_array([f(x), f(y), f(0.), f(1.)]);
        let spline = NurbCurve::try_new(
            3,
            vec![p(0., 0.), p(1., 0.), p(2., 1.), p(3., 1.)],
            vec![f(0.), f(0.), f(0.), f(0.), f(1.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let path = chain(vec![spline], false);
        let model = swept(circle(0.2), &yz(Vector3::zero()), &path);
        let topology = model.topology();
        assert_eq!(topology.faces.len(), 4 + 2);
        // The spline ends heading along x again: the end cap lies at x = 3 —
        // as near as the frames along a spline, a sharp free choice worked
        // out in plain numbers, put it, which is not an enclosure of x = 3.
        let end = model.face_id("sweep(s,end)").unwrap();
        assert!(topology.iterate_face_coedges(end).all(|c| {
            let x = topology.coedge_start_vertex(c).unwrap().point[0].to_f64();
            (x - 3.0).abs() < 1e-9
        }));
    }

    /// An open chain sweeps into a sheet.
    #[test]
    fn open_profile_sweeps_into_a_sheet() {
        let path = chain(
            vec![
                NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                NurbCurve3D::arc(
                    v3(2., 0., 0.),
                    v3(3., 0., 0.),
                    v3(3., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
            ],
            false,
        );
        let mut model = NamedModel::new();
        let namer = Namer::new("sweep", "s").unwrap();
        let profile = Profile::open(polyline(&[v2(-0.5, 0.), v2(0., 0.5), v2(0.5, 0.)]).unwrap());
        let built = sweep_along(
            &mut model,
            &namer,
            None,
            &path,
            &yz(Vector3::zero()),
            &[SweepLoop::plain(profile)],
            &Control::default(),
            None,
        )
        .unwrap();
        assert!(built.solid.is_none());
        model.check_names().unwrap();
        assert_model_valid_sheets(model.topology());
        assert_eq!(model.topology().faces.len(), 4);
    }

    /// A line meeting an arc at an angle, with nothing to rebuild the walls
    /// there by (see [`Rebuild`]): refused.
    #[test]
    fn corner_at_an_arc_is_refused() {
        let path = chain(
            vec![
                NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                NurbCurve3D::arc(
                    v3(2., 0., 0.),
                    v3(2., 1., 0.),
                    v3(1., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
            ],
            false,
        );
        let error = sweep_along(
            &mut NamedModel::new(),
            &Namer::new("sweep", "s").unwrap(),
            None,
            &path,
            &yz(Vector3::zero()),
            &[SweepLoop::plain(Profile::closed(square(0.2)))],
            &Control::default(),
            None,
        )
        .unwrap_err();
        assert!(error.root_message().contains("corner"), "{error:?}");
    }

    /// A bend tighter than the profile reaches would sweep it through
    /// itself: refused, naming the bend.
    #[test]
    fn bend_tighter_than_the_profile_is_refused() {
        let path = chain(
            vec![
                NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                NurbCurve3D::arc(
                    v3(2., 0., 0.),
                    v3(2.2, 0., 0.),
                    v3(2.2, 0.2, 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
            ],
            false,
        );
        let mut model = NamedModel::new();
        let namer = Namer::new("sweep", "s").unwrap();
        let error = sweep_along(
            &mut model,
            &namer,
            Some(&namer.root()),
            &path,
            &yz(Vector3::zero()),
            &[SweepLoop::plain(Profile::closed(circle(0.3)))],
            &Control::default(),
            None,
        )
        .unwrap_err();
        assert!(error.root_message().contains("bend k1"), "{error:?}");
    }

    /// A profile in a plane the path runs along cannot be swept.
    #[test]
    fn profile_along_the_path_is_refused() {
        let path = chain(
            vec![NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap()],
            false,
        );
        let xy = plane(Vector3::zero(), v3(1., 0., 0.), v3(0., 1., 0.));
        assert!(followed(&path, &xy, &Control::default()).is_err());
    }

    // ── twist, scale, fixed normal and rails ────────────────────────────────

    /// Sweeps the closed `outer` loop in `plane` along `path`, changing as
    /// `control` says, into a solid in a fresh part, and checks it is valid.
    fn swept_with(
        outer: Vec<NurbCurve2D>,
        plane: &CoordinateSystem,
        path: &Chain<NurbCurve3D>,
        control: &Control,
    ) -> NamedModel {
        let mut model = NamedModel::new();
        let namer = Namer::new("sweep", "s").unwrap();
        sweep_along(
            &mut model,
            &namer,
            Some(&namer.root()),
            path,
            plane,
            &[SweepLoop::plain(Profile::closed(outer))],
            control,
            None,
        )
        .unwrap();
        model.check_names().unwrap();
        assert_model_valid(model.topology());
        model
    }

    fn rail(name: &str, curves: Vec<NurbCurve3D>) -> Chain<NurbCurve3D> {
        Chain {
            name: name.into(),
            ..chain(curves, false)
        }
    }

    fn with_rails(rails: Vec<Chain<NurbCurve3D>>) -> Control {
        Control {
            rails,
            ..Control::default()
        }
    }

    /// The straight path along `x` from the origin to `(length, 0, 0)`.
    fn along_x(length: f64) -> Chain<NurbCurve3D> {
        chain(
            vec![NurbCurve3D::line(v3(0., 0., 0.), v3(length, 0., 0.)).unwrap()],
            false,
        )
    }

    /// Whether the wall `name` of `model` runs straight along the path.
    fn straight(model: &NamedModel, name: &str) -> bool {
        let face = model.face_id(name).unwrap();
        model.topology().faces[&face].surface.degree_u == 1
    }

    /// Points of the wall `name` of `model` on a grid of its parameters.
    fn wall_points(model: &NamedModel, name: &str) -> Vec<Vector3> {
        let face = model.face_id(name).unwrap();
        let surface = &model.topology().faces[&face].surface;
        let mut points = Vec::new();
        for i in 0..=8 {
            for j in 0..=8 {
                let (u, v) = (
                    Scalar::from_f64(i as f64 / 8.0),
                    Scalar::from_f64(j as f64 / 8.0),
                );
                points.push(surface.evaluate(u, v).unwrap());
            }
        }
        points
    }

    /// The distance of `p` from the `x` axis.
    fn radius(p: &Vector3) -> Scalar {
        p[1].mul(p[1]).add(p[2].mul(p[2])).sqrt().unwrap()
    }

    /// The vertex `name` of `model`.
    fn vertex(model: &NamedModel, name: &str) -> Vector3 {
        let v = model.vertex_id(name).unwrap();
        model.topology().get_vertex(v).unwrap().point
    }

    /// Whether `value` could be `target`, but for the rounding of a
    /// sweep's sections — a free choice made in plain numbers, which puts
    /// them where they are worked out to go only that near.
    fn near(value: Scalar, target: f64) -> bool {
        const ROUNDING: f64 = 1e-12;
        value.lower().to_f64() - ROUNDING <= target && target <= value.upper().to_f64() + ROUNDING
    }

    fn near_point(p: &Vector3, target: [f64; 3]) -> bool {
        (0..3).all(|k| near(p[k], target[k]))
    }

    /// A circle along a line, its point on a straight rail drifting out:
    /// a cone, every wall still straight along the path — the radius at
    /// every point of every wall what the rail says at its `x`.
    #[test]
    fn one_rail_sweeps_a_cone() {
        let rails = vec![rail(
            "r",
            vec![NurbCurve3D::line(v3(0., 0.5, 0.), v3(2., 1., 0.)).unwrap()],
        )];
        let model = swept_with(
            circle(0.5),
            &yz(Vector3::zero()),
            &along_x(2.0),
            &with_rails(rails),
        );
        assert_eq!(model.topology().faces.len(), 4 + 2);
        for i in 0..4 {
            assert!(straight(&model, &format!("sweep(s,c{i},k0)")));
            for p in wall_points(&model, &format!("sweep(s,c{i},k0)")) {
                // The radius the rail says at this `x`, `0.5 + x / 4`.
                let wanted = p[0].mul(Scalar::from_f64(0.25)).add(Scalar::from_f64(0.5));
                assert!(near(radius(&p).sub(wanted), 0.0), "{p:?}");
            }
        }
        // The rail's end is where the profile's point on it went.
        let end = vertex(&model, "sweep(s,p0,j1)");
        assert!(near_point(&end, [2.0, 1.0, 0.0]), "{end:?}");
    }

    /// A circle along a line between two rails, one drifting out along `y`
    /// and the other in along `z`: an ellipse growing, its half axes what
    /// the rails say at every section.
    #[test]
    fn two_rails_scale_an_ellipse() {
        let rails = vec![
            rail(
                "wide",
                vec![NurbCurve3D::line(v3(0., 1., 0.), v3(2., 2., 0.)).unwrap()],
            ),
            rail(
                "flat",
                vec![NurbCurve3D::line(v3(0., 0., 1.), v3(2., 0., 0.5)).unwrap()],
            ),
        ];
        let model = swept_with(
            circle(1.0),
            &yz(Vector3::zero()),
            &along_x(2.0),
            &with_rails(rails),
        );
        for i in 0..4 {
            assert!(straight(&model, &format!("sweep(s,c{i},k0)")));
            for p in wall_points(&model, &format!("sweep(s,c{i},k0)")) {
                let f = Scalar::from_f64;
                let a = f(1.0).add(p[0].mul(f(0.5)));
                let b = f(1.0).sub(p[0].mul(f(0.25)));
                let (y, z) = (p[1].div(a).unwrap(), p[2].div(b).unwrap());
                assert!(near(y.mul(y).add(z.mul(z)), 1.0), "{p:?}");
            }
        }
        let top = vertex(&model, "sweep(s,p1,j1)");
        assert!(near_point(&top, [2.0, 0.0, 0.5]), "{top:?}");
    }

    /// A rail bowing out along a line: the walls are sections skinned
    /// together, each section — at every knot of a wall — the circle the
    /// rail says.
    #[test]
    fn curved_rail_shapes_the_sections() {
        let f = Scalar::from_f64;
        let p = |x: f64, y: f64| Vector4::from_array([f(x), f(y), f(0.), f(1.)]);
        let bow = NurbCurve::try_new(
            2,
            vec![p(0., 0.5), p(1., 1.5), p(2., 0.5)],
            vec![f(0.), f(0.), f(0.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let model = swept_with(
            circle(0.5),
            &yz(Vector3::zero()),
            &along_x(2.0),
            &with_rails(vec![rail("bow", vec![bow.clone()])]),
        );
        let face = model.face_id("sweep(s,c0,k0)").unwrap();
        let surface = &model.topology().faces[&face].surface;
        let (degree, knots) = (surface.degree_u, &surface.knot_vector_u);
        let sections = (degree + 1..knots.len() - degree - 1).step_by(degree);
        for k in sections {
            let u = knots[k];
            let q = surface.evaluate(u, Scalar::ZERO).unwrap();
            // Where the rail is at this `x`: it runs as `x = 2 t`, with
            // `y = 0.5 + 2 t (1 - t)`.
            let t = q[0].mul(Scalar::from_f64(0.5));
            let wanted = Scalar::from_f64(0.5).add(Scalar::TWO.mul(t).mul(Scalar::ONE.sub(t)));
            assert!(near(radius(&q).sub(wanted), 0.0), "{q:?}");
        }
    }

    /// A square bar along a line, twisted a quarter turn: its far end
    /// turned so, and half way along — a section of every wall — half so.
    #[test]
    fn twisted_square_bar() {
        let control = Control {
            twist: std::f64::consts::FRAC_PI_2,
            ..Control::default()
        };
        let model = swept_with(square(0.5), &yz(Vector3::zero()), &along_x(4.0), &control);
        assert_eq!(model.topology().faces.len(), 4 + 2);
        // The corner (0.5, 0.5) at the end, turned to (-0.5, 0.5).
        let end = vertex(&model, "sweep(s,p2,j1)");
        assert!(near_point(&end, [4.0, -0.5, 0.5]), "{end:?}");
        // Half way, turned an eighth: straight up, `sqrt(1/2)` out.
        let edge = model.edge_id("sweep(s,p2,k0)").unwrap();
        let curve = &model.topology().edges[&edge].curve;
        let middle = curve.evaluate(Scalar::from_f64(0.5)).unwrap();
        assert!(near_point(&middle, [2.0, 0.0, 0.5f64.sqrt()]), "{middle:?}");
    }

    /// Scaled to twice its size along a line: a frustum, straight walls.
    #[test]
    fn scaled_sweep_is_a_frustum() {
        let control = Control {
            end_scale: 2.0,
            ..Control::default()
        };
        let model = swept_with(circle(0.5), &yz(Vector3::zero()), &along_x(2.0), &control);
        for i in 0..4 {
            assert!(straight(&model, &format!("sweep(s,c{i},k0)")));
            for p in wall_points(&model, &format!("sweep(s,c{i},k0)")) {
                // The radius the rail says at this `x`, `0.5 + x / 4`.
                let wanted = p[0].mul(Scalar::from_f64(0.25)).add(Scalar::from_f64(0.5));
                assert!(near(radius(&p).sub(wanted), 0.0), "{p:?}");
            }
        }
    }

    /// Twisted and scaled through a bend: a valid solid, scaled at its end.
    #[test]
    fn twisted_and_scaled_through_a_bend() {
        let path = chain(
            vec![
                NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                NurbCurve3D::arc(
                    v3(2., 0., 0.),
                    v3(3., 0., 0.),
                    v3(3., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
            ],
            false,
        );
        let control = Control {
            twist: std::f64::consts::PI,
            end_scale: 0.5,
            ..Control::default()
        };
        let model = swept_with(square(0.3), &yz(Vector3::zero()), &path, &control);
        // The end lies square to the arc's end, at `y = 1`, half the size.
        let topology = model.topology();
        let end = model.face_id("sweep(s,end)").unwrap();
        for c in topology.iterate_face_coedges(end) {
            let p = topology.coedge_start_vertex(c).unwrap().point;
            assert!(near(p[1], 1.0), "{p:?}");
            let dx = p[0].sub(Scalar::from_f64(3.0));
            let off = dx.mul(dx).add(p[2].mul(p[2])).sqrt().unwrap();
            assert!(near(off, 0.15 * 2f64.sqrt()), "{p:?}");
        }
    }

    /// Keeping its normal along a slanted line and an arc: the profile only
    /// moves — exact, every vertex the profile's corner moved by the path.
    #[test]
    fn fixed_normal_moves_the_profile() {
        let path = chain(
            vec![
                NurbCurve3D::line(v3(0., 0., 0.), v3(1., 1., 0.)).unwrap(),
                NurbCurve3D::arc(
                    v3(1., 1., 0.),
                    v3(2., 2., 0.),
                    v3(3., 2., 0.),
                    Scalar::from_f64((std::f64::consts::PI / 8.0).cos()),
                )
                .unwrap(),
            ],
            false,
        );
        let control = Control {
            orientation: Orientation::FixedNormal,
            ..Control::default()
        };
        let model = swept_with(square(0.25), &yz(Vector3::zero()), &path, &control);
        let topology = model.topology();
        let v = model.vertex_id("sweep(s,p2,j2)").unwrap();
        assert!(
            topology
                .get_vertex(v)
                .unwrap()
                .point
                .could_be_equal(&v3(3., 2.25, 0.25))
        );
    }

    /// What a controlled sweep refuses, by name.
    #[test]
    fn controlled_sweeps_refuse_what_they_cannot_build() {
        let refused = |path: &Chain<NurbCurve3D>, control: Control, says: &str| {
            let error = followed(path, &yz(Vector3::zero()), &control)
                .err()
                .unwrap();
            assert!(error.root_message().contains(says), "{error:?}");
        };
        let straight = |name: &str, from: Vector3, to: Vector3| {
            rail(name, vec![NurbCurve3D::line(from, to).unwrap()])
        };
        // A rail off the profile's plane at both ends.
        refused(
            &along_x(2.0),
            with_rails(vec![straight("off", v3(0.5, 0.5, 0.), v3(2., 1., 0.))]),
            "off starts on the profile's plane at neither end",
        );
        // A rail with a twist.
        refused(
            &along_x(2.0),
            Control {
                twist: 1.0,
                ..with_rails(vec![straight("r", v3(0., 0.5, 0.), v3(2., 1., 0.))])
            },
            "no twist or scale with them",
        );
        // A rail shorter than the path.
        refused(
            &along_x(2.0),
            with_rails(vec![straight("short", v3(0., 0.5, 0.), v3(1., 1., 0.))]),
            "short ends before the path does",
        );
        // Two rails in line with the path.
        refused(
            &along_x(2.0),
            with_rails(vec![
                straight("a", v3(0., 0.5, 0.), v3(2., 1., 0.)),
                straight("b", v3(0., -0.5, 0.), v3(2., -1., 0.)),
            ]),
            "in line with the path",
        );
        // A twisted ring.
        let corners = [(0., 0.), (4., 0.), (4., 4.), (0., 4.)];
        let ring = chain(
            (0..4)
                .map(|i| {
                    let (a, b) = (corners[i], corners[(i + 1) % 4]);
                    NurbCurve3D::line(v3(a.0, a.1, 0.), v3(b.0, b.1, 0.)).unwrap()
                })
                .collect(),
            true,
        );
        refused(
            &ring,
            Control {
                twist: 1.0,
                ..Control::default()
            },
            "sweep along an open path",
        );
        // A path that turns parallel to the plane, with a fixed normal.
        let back = chain(
            vec![
                NurbCurve3D::arc(
                    v3(0., 0., 0.),
                    v3(1., 0., 0.),
                    v3(1., 1., 0.),
                    sqrt2_over_2(),
                )
                .unwrap(),
            ],
            false,
        );
        refused(
            &back,
            Control {
                orientation: Orientation::FixedNormal,
                ..Control::default()
            },
            "keep running through the profile's plane",
        );
    }

    /// Every rail shape — a line, an arc, a spline, a line running on into
    /// an arc — alone and paired with a straight one, with a circle and a
    /// square; and a twist and scale through bends of several turns: every
    /// sweep valid.
    #[test]
    #[ignore = "slow: rail shapes, twists and bends — run with `cargo test -- --ignored`"]
    fn rail_shapes() {
        let f = Scalar::from_f64;
        let p = |x: f64, y: f64| Vector4::from_array([f(x), f(y), f(0.), f(1.)]);
        let spline = NurbCurve::try_new(
            3,
            vec![p(0., 0.5), p(1., 0.3), p(2., 1.0), p(3., 0.8)],
            vec![f(0.), f(0.), f(0.), f(0.), f(1.), f(1.), f(1.), f(1.)],
        )
        .unwrap();
        let rails: Vec<(&str, Vec<NurbCurve3D>)> = vec![
            (
                "line",
                vec![NurbCurve3D::line(v3(0., 0.5, 0.), v3(3., 0.8, 0.)).unwrap()],
            ),
            (
                "arc",
                vec![
                    NurbCurve3D::arc(v3(0., 0.5, 0.), v3(1.5, 1.25, 0.), v3(3., 0.5, 0.), f(0.8))
                        .unwrap(),
                ],
            ),
            ("spline", vec![spline]),
            (
                "line_arc",
                vec![
                    NurbCurve3D::line(v3(0., 0.5, 0.), v3(1., 0.5, 0.)).unwrap(),
                    NurbCurve3D::arc(v3(1., 0.5, 0.), v3(2., 0.5, 0.), v3(3., 1.0, 0.), f(0.9))
                        .unwrap(),
                ],
            ),
        ];
        let partner = || {
            rail(
                "z",
                vec![NurbCurve3D::line(v3(0., 0., 0.5), v3(3., 0., 0.3)).unwrap()],
            )
        };
        for (name, curves) in rails {
            for outer in [circle(0.5), square(0.5)] {
                for two in [false, true] {
                    let mut guides = vec![rail(name, curves.clone())];
                    if two {
                        guides.push(partner());
                    }
                    let mut model = NamedModel::new();
                    let namer = Namer::new("sweep", "s").unwrap();
                    sweep_along(
                        &mut model,
                        &namer,
                        Some(&namer.root()),
                        &along_x(3.0),
                        &yz(Vector3::zero()),
                        &[SweepLoop::plain(Profile::closed(outer.clone()))],
                        &with_rails(guides),
                        None,
                    )
                    .unwrap_or_else(|e| panic!("{name}, two rails: {two}: {e:?}"));
                    assert_model_valid(model.topology());
                }
            }
        }
        // A line, then an arc turning by `degrees`, tangent to it.
        for degrees in [30.0f64, 90.0, 150.0] {
            let half = (degrees / 2.0).to_radians();
            let leg = half.tan();
            let turned = 2.0 * half;
            let path = chain(
                vec![
                    NurbCurve3D::line(v3(0., 0., 0.), v3(2., 0., 0.)).unwrap(),
                    NurbCurve3D::arc(
                        v3(2., 0., 0.),
                        v3(2. + leg, 0., 0.),
                        v3(2. + leg * (1. + turned.cos()), leg * turned.sin(), 0.),
                        f(half.cos()),
                    )
                    .unwrap(),
                ],
                false,
            );
            for twist in [0.0, 1.0, 4.0] {
                let control = Control {
                    twist,
                    end_scale: 1.5,
                    ..Control::default()
                };
                swept_with(square(0.3), &yz(Vector3::zero()), &path, &control);
            }
        }
    }
}
