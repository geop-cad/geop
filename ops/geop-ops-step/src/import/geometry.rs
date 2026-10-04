//! The geometry of a STEP file: points, placements, curves and surfaces as
//! the file defines them — in millimetres, placed where the assembly puts
//! them — and their conversion to the kernel's NURBS.
//!
//! What the file says is read into plain `f64` definitions ([`CurveDef`],
//! [`SurfaceDef`]): it is data, decided on (where a face wraps around, which
//! loop is outer) before anything exact is built from it. The NURBS built
//! from it are exact for every analytic type — lines, circles, ellipses,
//! planes, cylinders, cones, spheres, tori, surfaces of revolution and of
//! extrusion — not approximations.

use geop_core_geometry::{
    nurb_curve::{NurbCurve, NurbCurve3D},
    nurb_surface::{NurbSurface, NurbSurface3D},
    shape::{Arc, Axis, Circle},
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Pose, Quaternion},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::reader::{Args, Reader, unsupported};

pub type P3 = [f64; 3];

pub fn add(a: P3, b: P3) -> P3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub fn sub(a: P3, b: P3) -> P3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub fn scale(a: P3, s: f64) -> P3 {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub fn dot(a: P3, b: P3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub fn cross(a: P3, b: P3) -> P3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
pub fn norm(a: P3) -> f64 {
    dot(a, a).sqrt()
}
pub fn distance(a: P3, b: P3) -> f64 {
    norm(sub(a, b))
}

/// `a` scaled to unit length, or `None` if it has none.
pub fn normalize(a: P3) -> Option<P3> {
    let n = norm(a);
    (n > 0.0 && n.is_finite()).then(|| scale(a, 1.0 / n))
}

/// A right-handed orthonormal frame: `AXIS2_PLACEMENT_3D`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub origin: P3,
    pub x: P3,
    pub y: P3,
    pub z: P3,
}

impl Frame {
    /// The frame at `origin` with its `z` along `axis` and its `x` along
    /// `reference` made perpendicular to it — or along any perpendicular,
    /// if `reference` is parallel to `axis`.
    pub fn new(origin: P3, axis: P3, reference: P3) -> GeopResult<Self> {
        let z = normalize(axis)
            .ok_or_else(|| GeopError::new(format!("a placement's axis {axis:?} has no length")))?;
        let x = normalize(sub(reference, scale(z, dot(reference, z))))
            .or_else(|| {
                let other = if z[0].abs() < 0.9 {
                    [1.0, 0.0, 0.0]
                } else {
                    [0.0, 1.0, 0.0]
                };
                normalize(sub(other, scale(z, dot(other, z))))
            })
            .expect("a vector not along a unit axis has a perpendicular part");
        Ok(Self {
            origin,
            x,
            y: cross(z, x),
            z,
        })
    }

    /// `p`, given in this frame's coordinates.
    pub fn point(&self, p: P3) -> P3 {
        add(
            self.origin,
            add(
                scale(self.x, p[0]),
                add(scale(self.y, p[1]), scale(self.z, p[2])),
            ),
        )
    }

    /// `p` in this frame's coordinates.
    pub fn local(&self, p: P3) -> P3 {
        let d = sub(p, self.origin);
        [dot(d, self.x), dot(d, self.y), dot(d, self.z)]
    }
}

/// A rigid motion, as an assembly places a part: `p -> origin + axes p`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placement {
    /// The images of the unit vectors: the columns of the rotation.
    pub axes: [P3; 3],
    pub origin: P3,
}

impl Placement {
    pub const IDENTITY: Placement = Placement {
        axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        origin: [0.0; 3],
    };

    /// The motion taking the world frame to `frame`.
    pub fn of(frame: &Frame) -> Self {
        Placement {
            axes: [frame.x, frame.y, frame.z],
            origin: frame.origin,
        }
    }

    pub fn direction(&self, d: P3) -> P3 {
        add(
            scale(self.axes[0], d[0]),
            add(scale(self.axes[1], d[1]), scale(self.axes[2], d[2])),
        )
    }

    pub fn point(&self, p: P3) -> P3 {
        add(self.origin, self.direction(p))
    }

    /// This motion, then `outer`.
    pub fn then(&self, outer: &Placement) -> Placement {
        Placement {
            axes: self.axes.map(|a| outer.direction(a)),
            origin: outer.point(self.origin),
        }
    }

    pub fn inverse(&self) -> Placement {
        // The transpose of an orthonormal rotation is its inverse.
        let [a, b, c] = self.axes;
        let axes = [[a[0], b[0], c[0]], [a[1], b[1], c[1]], [a[2], b[2], c[2]]];
        let inverse = Placement {
            axes,
            origin: [0.0; 3],
        };
        Placement {
            axes,
            origin: scale(inverse.direction(self.origin), -1.0),
        }
    }
}

/// How the geometry of one representation is read: its units, its
/// uncertainty, and where the assembly places it.
#[derive(Clone, Copy, Debug)]
pub struct Scope {
    /// Millimetres per length unit of the file.
    pub length: f64,
    /// Radians per angle unit of the file.
    pub angle: f64,
    /// How far apart two points the file means as one may be, in
    /// millimetres: the file's own `distance_accuracy_value`.
    pub uncertainty: f64,
    pub place: Placement,
}

/// A curve as the file defines it.
#[derive(Clone, Debug)]
pub enum CurveDef {
    Line {
        origin: P3,
        direction: P3,
    },
    Circle {
        frame: Frame,
        radius: f64,
    },
    Ellipse {
        frame: Frame,
        a: f64,
        b: f64,
    },
    Nurbs(NurbsCurve),
    Polyline(Vec<P3>),
    /// A hyperbola or a parabola, in the plane of `frame`.
    Conic {
        frame: Frame,
        kind: ConicKind,
    },
}

#[derive(Clone, Copy, Debug)]
pub enum ConicKind {
    /// `a cosh u x + b sinh u y`.
    Hyperbola { a: f64, b: f64 },
    /// `focal (u^2 x + 2 u y)`.
    Parabola { focal: f64 },
}

#[derive(Clone, Debug)]
pub struct NurbsCurve {
    pub degree: usize,
    pub points: Vec<P3>,
    pub weights: Option<Vec<f64>>,
    pub knots: Vec<f64>,
}

/// A surface as the file defines it, and whether the NURBS it is built as
/// (see [`SurfaceDef::natural_normal_flipped`]) faces the other way.
#[derive(Clone, Debug)]
pub struct SurfaceDef {
    pub kind: SurfaceKind,
    /// Whether the surface's normal as the file defines it is opposite to
    /// that of the NURBS built from it: what decides, with a face's own
    /// `same_sense`, which way a face's NURBS has to run.
    pub flipped: bool,
}

#[derive(Clone, Debug)]
pub enum SurfaceKind {
    Plane(Frame),
    Revolved(Revolved),
    Nurbs(NurbsSurface),
    /// `curve` swept along `vector`: `S(u, v) = curve(u) + v vector`.
    Extrusion {
        curve: NurbsCurve,
        vector: P3,
    },
}

#[derive(Clone, Debug)]
pub struct NurbsSurface {
    pub degree_u: usize,
    pub degree_v: usize,
    pub num_u: usize,
    pub num_v: usize,
    /// Row-major, `u` the row.
    pub points: Vec<P3>,
    pub weights: Option<Vec<f64>>,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
}

/// A surface swept by a profile turning about the `z` of `frame`: the
/// profile lies in the half plane of `frame.x` at angle 0.
#[derive(Clone, Debug)]
pub struct Revolved {
    pub frame: Frame,
    pub profile: Profile,
    /// How close to the axis a point is on it, and has no angle: the
    /// file's uncertainty.
    pub on_axis: f64,
}

/// What a [`Revolved`] surface's profile is, as a function of its
/// parameter `v` in the half plane at angle 0: `(radius, height)` along
/// `frame.x` and `frame.z`.
#[derive(Clone, Debug)]
pub enum Profile {
    /// A straight line, `(rho + v drho, h + v dh)`: a cylinder's, a
    /// cone's — its apex where the radius is none — or any line's.
    Line {
        rho: f64,
        h: f64,
        drho: f64,
        dh: f64,
    },
    /// `(r cos v, r sin v)`, `v` from `-pi/2` to `pi/2`.
    Sphere { radius: f64 },
    /// A circle off the axis, `(rho + r cos v, h + r sin v)`: a torus'
    /// tube, or any circle's. One reaching across the axis (`rho <= r`)
    /// makes two sheets meeting at the poles where it crosses the axis: its
    /// arc on this side turned, `v` between the poles — an apple — and the
    /// arc past the axis turned, a lemon inside it (see
    /// [`Revolved::past_axis`]).
    Circle { rho: f64, h: f64, radius: f64 },
    /// A curve of its own: `v` its parameter.
    Curve(NurbsCurve),
}

impl Revolved {
    /// The angle of `p` about the axis, and its profile parameter where
    /// the profile is analytic — `None` for the angle of a point on the
    /// axis, which has none.
    pub fn chart(&self, p: P3) -> (Option<f64>, f64) {
        let [x, y, h] = self.frame.local(p);
        let rho = x.hypot(y);
        let angle = (rho > self.on_axis).then(|| y.atan2(x));
        let v = match &self.profile {
            Profile::Line {
                rho: r0,
                h: h0,
                drho,
                dh,
            } => {
                // A line crossing the axis — a cone's — reaches past its
                // apex to negative radii: a point there is the profile's at
                // the radius taken negative, half a turn round. Whichever
                // of the two lies on the line is the point's.
                let along =
                    |rho: f64| ((rho - r0) * drho + (h - h0) * dh) / (drho * drho + dh * dh);
                let off = |rho: f64| ((rho - r0) * dh - (h - h0) * drho).abs();
                if *drho != 0.0 && off(-rho) < off(rho) {
                    let angle = angle.map(|a| {
                        if a > 0.0 {
                            a - std::f64::consts::PI
                        } else {
                            a + std::f64::consts::PI
                        }
                    });
                    return (angle, along(-rho));
                }
                along(rho)
            }
            Profile::Curve(_) => h,
            Profile::Sphere { .. } => h.atan2(rho),
            Profile::Circle { rho: rc, h: hc, .. } => (h - hc).atan2(rho - rc),
        };
        (angle, v)
    }

    /// For a circle crossing the axis, how much nearer `p` lies to the
    /// sheet its arc past the axis turns into than to the one its arc on
    /// this side does: positive where `p` is on the sheet past the axis.
    pub fn past_axis(&self, p: P3) -> Option<f64> {
        let Profile::Circle {
            rho: rc,
            h: hc,
            radius,
        } = self.profile
        else {
            return None;
        };
        if rc > radius {
            return None;
        }
        let [x, y, h] = self.frame.local(p);
        let rho = x.hypot(y);
        let off = |rho: f64| ((rho - rc).hypot(h - hc) - radius).abs();
        Some(off(rho) - off(-rho))
    }

    /// The sheet the arc of a circle crossing the axis past it turns into,
    /// as a surface of its own: the circle mirrored in the axis, turned on
    /// this side. Its profile parameter runs the other way round the
    /// circle, so its normal is the other way round to the file's.
    pub fn mirrored(&self) -> Revolved {
        let profile = match self.profile {
            Profile::Circle { rho, h, radius } => Profile::Circle {
                rho: -rho,
                h,
                radius,
            },
            ref other => other.clone(),
        };
        Revolved {
            profile,
            ..self.clone()
        }
    }

    /// Whether the profile's parameter `v` goes once round, as a torus'
    /// does: then it is an angle too.
    pub fn v_is_angle(&self) -> bool {
        matches!(self.profile, Profile::Circle { rho, radius, .. } if rho > radius)
    }

    /// Where the profile meets the axis, by its parameter: the poles of the
    /// surface.
    pub fn poles(&self) -> Vec<f64> {
        match &self.profile {
            // A circle reaching the axis meets it where its radius is none,
            // either side of its far point from the axis: the surface is
            // the arc between, turned.
            Profile::Circle { rho, radius, .. } if rho <= radius => {
                let v = (-rho / radius).acos();
                vec![-v, v]
            }
            Profile::Circle { .. } | Profile::Curve(_) => Vec::new(),
            Profile::Line { drho, .. } if *drho == 0.0 => Vec::new(),
            Profile::Line { rho, drho, .. } => vec![-rho / drho],
            Profile::Sphere { .. } => {
                vec![-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2]
            }
        }
    }

    /// The profile's point at `v`, in the half plane at angle 0.
    pub fn profile_point(&self, v: f64) -> P3 {
        let (rho, h) = match &self.profile {
            Profile::Line { rho, h, drho, dh } => (rho + v * drho, h + v * dh),
            Profile::Sphere { radius } => (radius * v.cos(), radius * v.sin()),
            Profile::Circle { rho, h, radius } => (rho + radius * v.cos(), h + radius * v.sin()),
            Profile::Curve(_) => unreachable!("a curve profile has no analytic point"),
        };
        self.frame.point([rho, 0.0, h])
    }

    /// The profile from `v0` to `v1` as a curve, in the half plane at
    /// angle 0 — the whole curve, for a profile that is one.
    pub fn profile_curve<S: Scalar>(&self, v0: f64, v1: f64) -> GeopResult<NurbCurve3D<S>> {
        let s3 = |p: P3| Vector3::from_array(p.map(S::from_f64));
        match &self.profile {
            Profile::Line { .. } => line(&s3(self.profile_point(v0)), &s3(self.profile_point(v1))),
            Profile::Sphere { radius } | Profile::Circle { radius, .. } => {
                let center = match &self.profile {
                    Profile::Circle { rho, h, .. } => self.frame.point([*rho, 0.0, *h]),
                    _ => self.frame.origin,
                };
                // Turning from `x` towards `z` is right-handed about `-y`.
                Arc {
                    circle: Circle {
                        center: s3(center),
                        normal: s3(scale(self.frame.y, -1.0)),
                        radius: S::from_f64(*radius),
                    },
                    start: s3(self.profile_point(v0)),
                    end: s3(self.profile_point(v1)),
                }
                .to_curve()
            }
            Profile::Curve(curve) => curve.to_nurbs(),
        }
    }

    /// The parallel at the profile parameter `v` — the circle about the
    /// axis the profile's point there turns along — from the angle `from`
    /// counter-clockwise about the axis to `to`: all the way round where
    /// they are one.
    pub fn parallel<S: Scalar>(&self, v: f64, from: f64, to: f64) -> GeopResult<NurbCurve3D<S>> {
        let s3 = |p: P3| Vector3::from_array(p.map(S::from_f64));
        let p = self.profile_point(v);
        let [rho, _, h] = self.frame.local(p);
        let at =
            |angle: f64| -> GeopResult<Vector3<S>> { Ok(self.turn::<S>(angle)?.apply(&s3(p))) };
        let start = at(from)?;
        let end = if to == from { start } else { at(to)? };
        Arc {
            circle: Circle {
                center: s3(self.frame.point([0.0, 0.0, h])),
                normal: s3(self.frame.z),
                radius: S::from_f64(rho),
            },
            start,
            end,
        }
        .to_curve()
    }

    pub fn axis<S: Scalar>(&self) -> GeopResult<Axis<S>> {
        Axis::try_new(
            Vector3::from_array(self.frame.origin.map(S::from_f64)),
            Vector3::from_array(self.frame.z.map(S::from_f64)),
        )
    }

    /// The surface between the angles `from` and `to` and, where the
    /// profile is analytic, the profile parameters `v0` and `v1`.
    pub fn patch<S: Scalar>(
        &self,
        from: f64,
        to: f64,
        v0: f64,
        v1: f64,
    ) -> GeopResult<NurbSurface3D<S>> {
        NurbSurface::revolve(&self.profile_curve(v0, v1)?, &self.axis()?, from, to)
    }

    /// The motion turning by `angle` about the axis.
    pub fn turn<S: Scalar>(&self, angle: f64) -> GeopResult<Pose<S>> {
        let (half_sin, half_cos) = (angle / 2.0).sin_cos();
        let z = self.frame.z;
        let rotation = Quaternion::new(
            S::from_f64(half_cos),
            S::from_f64(z[0] * half_sin),
            S::from_f64(z[1] * half_sin),
            S::from_f64(z[2] * half_sin),
        )
        .normalized()?;
        let turned = Pose::new(Vector3::zero(), rotation)?;
        let o = Vector3::from_array(self.frame.origin.map(S::from_f64));
        Pose::new(o.sub(&turned.apply(&o)), rotation)
    }
}

/// The straight line from `a` to `b`, on `[0, 1]`.
pub fn line<S: Scalar>(a: &Vector3<S>, b: &Vector3<S>) -> GeopResult<NurbCurve3D<S>> {
    let h = |p: &Vector3<S>| Vector4::from_array([p[0], p[1], p[2], S::ONE]);
    NurbCurve::try_new(1, vec![h(a), h(b)], vec![S::ZERO, S::ZERO, S::ONE, S::ONE])
}

impl NurbsCurve {
    pub fn to_nurbs<S: Scalar>(&self) -> GeopResult<NurbCurve3D<S>> {
        let points = self
            .points
            .iter()
            .enumerate()
            .map(|(i, p)| homogeneous(*p, self.weights.as_ref().map(|w| w[i])))
            .collect();
        let curve = NurbCurve::try_new(
            self.degree,
            points,
            self.knots.iter().map(|&k| S::from_f64(k)).collect(),
        )?;
        clamped(curve)
    }
}

/// `curve` with its ends at its first and last control points, as every
/// curve of the kernel has: an unclamped (periodic) knot vector cut at the
/// ends of its domain.
fn clamped<S: Scalar>(curve: NurbCurve3D<S>) -> GeopResult<NurbCurve3D<S>> {
    let p = curve.degree;
    let k = &curve.knot_vector;
    let n = k.len();
    let is_clamped = (0..=p).all(|i| k[i].to_f64() == k[p].to_f64())
        && (0..=p).all(|i| k[n - 1 - i].to_f64() == k[n - 1 - p].to_f64());
    if is_clamped {
        return Ok(curve);
    }
    let (lo, hi) = curve.domain();
    curve.sub_curve(lo, hi)
}

fn homogeneous<S: Scalar>(p: P3, weight: Option<f64>) -> Vector4<S> {
    match weight {
        // A weight of one is taken as it is: multiplying by it would only
        // widen the point by rounding.
        None => Vector4::from_array([
            S::from_f64(p[0]),
            S::from_f64(p[1]),
            S::from_f64(p[2]),
            S::ONE,
        ]),
        Some(w) => {
            let w = S::from_f64(w);
            Vector4::from_array([
                S::from_f64(p[0]).mul(w),
                S::from_f64(p[1]).mul(w),
                S::from_f64(p[2]).mul(w),
                w,
            ])
        }
    }
}

impl NurbsSurface {
    pub fn to_nurbs<S: Scalar>(&self) -> GeopResult<NurbSurface3D<S>> {
        let points = self
            .points
            .iter()
            .enumerate()
            .map(|(i, p)| homogeneous(*p, self.weights.as_ref().map(|w| w[i])))
            .collect();
        let surface = NurbSurface::try_new(
            self.degree_u,
            self.degree_v,
            points,
            self.knots_u.iter().map(|&k| S::from_f64(k)).collect(),
            self.knots_v.iter().map(|&k| S::from_f64(k)).collect(),
        )?;
        let clamped_in = |knots: &[f64], p: usize| {
            let n = knots.len();
            (0..=p).all(|i| knots[i] == knots[p])
                && (0..=p).all(|i| knots[n - 1 - i] == knots[n - 1 - p])
        };
        if clamped_in(&self.knots_u, self.degree_u) && clamped_in(&self.knots_v, self.degree_v) {
            Ok(surface)
        } else {
            surface.sub_surface(surface.domain_u(), surface.domain_v())
        }
    }
}

impl CurveDef {
    /// The curve from `from` to `to` — points on it, the curve running from
    /// one to the other the way it runs — as the kernel builds an edge: a
    /// line or an arc ending exactly at those points, or the piece of a
    /// B-spline between where they are on it. `closed` asks for the whole
    /// way round, from a point back to itself.
    ///
    /// Where on a B-spline the points are is found by projecting them onto
    /// it; within `uncertainty` of an end of the curve, they are taken to be
    /// that end. That is the file's own statement of which points are one:
    /// the trim it writes at an end means the end.
    pub fn edge<S: Scalar>(
        &self,
        from: P3,
        to: P3,
        closed: bool,
        uncertainty: f64,
    ) -> GeopResult<NurbCurve3D<S>> {
        let s3 = |p: P3| Vector3::from_array(p.map(S::from_f64));
        match self {
            CurveDef::Line { .. } => {
                if closed {
                    return Err(GeopError::new(
                        "a straight edge cannot start and end at the same vertex",
                    ));
                }
                line(&s3(from), &s3(to))
            }
            CurveDef::Circle { frame, radius } => Arc {
                circle: Circle {
                    center: s3(frame.origin),
                    normal: s3(frame.z),
                    radius: S::from_f64(*radius),
                },
                start: s3(from),
                end: s3(to),
            }
            .to_curve(),
            CurveDef::Ellipse { frame, a, b } => {
                // The arc of the unit circle between the points' angles,
                // stretched onto the ellipse: an affine map of a rational
                // curve's control points is exact.
                let unit = |p: P3| {
                    let l = frame.local(p);
                    let (s, c) = (l[1] / b).atan2(l[0] / a).sin_cos();
                    Vector3::from_array([S::from_f64(c), S::from_f64(s), S::ZERO])
                };
                let (start, end) = (unit(from), unit(to));
                let end = if closed { start } else { end };
                let arc = Arc {
                    circle: Circle {
                        center: Vector3::zero(),
                        normal: Vector3::from_array([S::ZERO, S::ZERO, S::ONE]),
                        radius: S::ONE,
                    },
                    start,
                    end,
                }
                .to_curve()?;
                let (o, x, y) = (
                    s3(frame.origin),
                    s3(scale(frame.x, *a)),
                    s3(scale(frame.y, *b)),
                );
                let control_points = arc
                    .control_points
                    .iter()
                    .map(|cp| {
                        let w = cp[3];
                        let p = o
                            .prod_scalar(w)
                            .add(&x.prod_scalar(cp[0]))
                            .add(&y.prod_scalar(cp[1]));
                        Vector4::from_array([p[0], p[1], p[2], w])
                    })
                    .collect();
                NurbCurve::try_new(arc.degree, control_points, arc.knot_vector)
            }
            CurveDef::Nurbs(nurbs) => trim(nurbs.to_nurbs()?, from, to, closed, uncertainty),
            CurveDef::Conic { frame, kind } => {
                if closed {
                    return Err(GeopError::new(
                        "a hyperbola or parabola cannot close on itself",
                    ));
                }
                // One rational quadratic piece from `from` to `to`: its
                // middle control point where the tangents at the ends meet.
                let parameter = |p: P3| {
                    let l = frame.local(p);
                    match kind {
                        ConicKind::Hyperbola { b, .. } => (l[1] / b).asinh(),
                        ConicKind::Parabola { focal } => l[1] / (2.0 * focal),
                    }
                };
                let (u0, u1) = (parameter(from), parameter(to));
                let (middle, weight) = match kind {
                    ConicKind::Hyperbola { a, b } => {
                        let (m, h) = ((u0 + u1) / 2.0, (u1 - u0) / 2.0);
                        (
                            frame.point([a * m.cosh() / h.cosh(), b * m.sinh() / h.cosh(), 0.0]),
                            h.cosh(),
                        )
                    }
                    ConicKind::Parabola { focal } => {
                        (frame.point([focal * u0 * u1, focal * (u0 + u1), 0.0]), 1.0)
                    }
                };
                let control_points = vec![
                    homogeneous(from, None),
                    homogeneous(middle, (weight != 1.0).then_some(weight)),
                    homogeneous(to, None),
                ];
                NurbCurve::try_new(
                    2,
                    control_points,
                    vec![S::ZERO, S::ZERO, S::ZERO, S::ONE, S::ONE, S::ONE],
                )
            }
            CurveDef::Polyline(points) => {
                let n = points.len();
                if n < 2 {
                    return Err(GeopError::new("a polyline of fewer than two points"));
                }
                let mut knots = vec![S::ZERO];
                for i in 0..n {
                    knots.push(S::from_ratio(i as i64, n as i64 - 1)?);
                }
                knots.push(S::ONE);
                let control_points = points.iter().map(|p| homogeneous(*p, None)).collect();
                trim(
                    NurbCurve::try_new(1, control_points, knots)?,
                    from,
                    to,
                    closed,
                    uncertainty,
                )
            }
        }
    }
}

/// The piece of `curve` from `from` to `to` (see [`CurveDef::edge`]).
fn trim<S: Scalar>(
    curve: NurbCurve3D<S>,
    from: P3,
    to: P3,
    closed: bool,
    uncertainty: f64,
) -> GeopResult<NurbCurve3D<S>> {
    let (lo, hi) = curve.domain();
    let start = to_p3(&curve.evaluate(lo)?);
    let end = to_p3(&curve.evaluate(hi)?);
    let t0 = if distance(start, from) <= uncertainty {
        None
    } else {
        Some(parameter_of(&curve, from)?)
    };
    let t1 = if distance(end, to) <= uncertainty || (closed && t0.is_none()) {
        None
    } else {
        Some(parameter_of(&curve, to)?)
    };
    if t0.is_none() && t1.is_none() {
        return Ok(curve);
    }
    let a = t0.unwrap_or(lo);
    let b = t1.unwrap_or(hi);
    if !a.definitely_less(b) {
        return Err(GeopError::new(format!(
            "the edge from {from:?} to {to:?} runs against its curve, or round past the curve's own ends (at parameters {a:?} and {b:?} of {lo:?} to {hi:?})"
        )));
    }
    curve.sub_curve(a, b)
}

pub fn to_p3<S: Scalar>(p: &Vector3<S>) -> P3 {
    [p[0].to_f64(), p[1].to_f64(), p[2].to_f64()]
}

/// Where on `curve` the point `p` is: the parameter of its nearest point,
/// found by sampling and polished by Newton. A free choice of where to cut
/// the curve, so sharp.
pub fn parameter_of<S: Scalar>(curve: &NurbCurve3D<S>, p: P3) -> GeopResult<S> {
    let (lo, hi) = curve.domain();
    let (lo_f, hi_f) = (lo.to_f64(), hi.to_f64());
    let samples = 16 * curve.control_points.len().max(4);
    let mut best = (f64::INFINITY, lo_f);
    for i in 0..=samples {
        let t = lo_f + (hi_f - lo_f) * i as f64 / samples as f64;
        let d = distance(point_at(curve, t)?, p);
        if d < best.0 {
            best = (d, t);
        }
    }
    let target = Vector3::from_array(p.map(S::from_f64));
    let step = (hi_f - lo_f) / samples as f64;
    let seed = S::from_f64((best.1 - step).max(lo_f)).union(S::from_f64((best.1 + step).min(hi_f)));
    Ok(curve.refine_parameter_at_point(seed, &target)?.sharpen())
}

impl SurfaceDef {
    /// Whether this is a plane or a surface of revolution, whose patches
    /// are built to fit each face.
    pub fn revolved(&self) -> Option<&Revolved> {
        match &self.kind {
            SurfaceKind::Revolved(r) => Some(r),
            _ => None,
        }
    }
}

impl<'a> Reader<'a> {
    pub fn point(&self, scope: &Scope, id: u64) -> GeopResult<P3> {
        let args = self.args(id, "CARTESIAN_POINT")?;
        let c = args.reals(1)?;
        if c.len() != 3 {
            return Err(GeopError::new(format!(
                "#{id} CARTESIAN_POINT has {} coordinates, where 3 were expected",
                c.len()
            )));
        }
        Ok(scope.place.point(scale([c[0], c[1], c[2]], scope.length)))
    }

    pub fn direction(&self, scope: &Scope, id: u64) -> GeopResult<P3> {
        let args = self.args(id, "DIRECTION")?;
        let c = args.reals(1)?;
        if c.len() != 3 {
            return Err(GeopError::new(format!(
                "#{id} DIRECTION has {} components, where 3 were expected",
                c.len()
            )));
        }
        let d = normalize([c[0], c[1], c[2]])
            .ok_or_else(|| GeopError::new(format!("#{id} DIRECTION has no length")))?;
        Ok(scope.place.direction(d))
    }

    /// A `VECTOR`, its magnitude a length.
    pub fn vector(&self, scope: &Scope, id: u64) -> GeopResult<P3> {
        let args = self.args(id, "VECTOR")?;
        let d = self.direction(scope, args.reference(1)?)?;
        Ok(scale(d, args.real(2)? * scope.length))
    }

    pub fn frame(&self, scope: &Scope, id: u64) -> GeopResult<Frame> {
        let instance = self.instance(id)?;
        if instance.is("AXIS2_PLACEMENT_2D") {
            return Err(unsupported(
                id,
                instance,
                "a 2-D placement where a 3-D one is needed",
            ));
        }
        let args = self.args(id, "AXIS2_PLACEMENT_3D")?;
        let origin = self.point(scope, args.reference(1)?)?;
        let axis = if args.is_null(2) {
            scope.place.direction([0.0, 0.0, 1.0])
        } else {
            self.direction(scope, args.reference(2)?)?
        };
        let reference = if args.is_null(3) {
            scope.place.direction([1.0, 0.0, 0.0])
        } else {
            self.direction(scope, args.reference(3)?)?
        };
        Frame::new(origin, axis, reference)
            .map_err(|e| e.with_context(format!("#{id} AXIS2_PLACEMENT_3D")))
    }

    /// The curve `#id`, and whether the curve as used runs against the one
    /// it is defined by (a `TRIMMED_CURVE` whose sense disagrees).
    pub fn curve(&self, scope: &Scope, id: u64) -> GeopResult<(CurveDef, bool)> {
        let instance = self.instance(id)?;
        if instance.is("B_SPLINE_CURVE")
            || instance.is("B_SPLINE_CURVE_WITH_KNOTS")
            || instance.is("BEZIER_CURVE")
            || instance.is("UNIFORM_CURVE")
            || instance.is("QUASI_UNIFORM_CURVE")
            || instance.is("RATIONAL_B_SPLINE_CURVE")
        {
            return Ok((CurveDef::Nurbs(self.nurbs_curve(scope, id)?), false));
        }
        let (kind, args) = self
            .args_of_any(
                id,
                &[
                    "LINE",
                    "CIRCLE",
                    "ELLIPSE",
                    "TRIMMED_CURVE",
                    "SURFACE_CURVE",
                    "SEAM_CURVE",
                    "INTERSECTION_CURVE",
                    "POLYLINE",
                    "HYPERBOLA",
                    "PARABOLA",
                ],
            )
            .map_err(|_| unsupported(id, instance, "this kind of curve cannot be read"))?;
        Ok(match kind {
            "LINE" => (
                CurveDef::Line {
                    origin: self.point(scope, args.reference(1)?)?,
                    direction: self.vector(scope, args.reference(2)?)?,
                },
                false,
            ),
            "CIRCLE" => (
                CurveDef::Circle {
                    frame: self.frame(scope, args.reference(1)?)?,
                    radius: args.real(2)? * scope.length,
                },
                false,
            ),
            "ELLIPSE" => (
                CurveDef::Ellipse {
                    frame: self.frame(scope, args.reference(1)?)?,
                    a: args.real(2)? * scope.length,
                    b: args.real(3)? * scope.length,
                },
                false,
            ),
            "TRIMMED_CURVE" => {
                // The edge's own vertices say where it ends: only the basis
                // curve, and which way round it is taken, matter.
                let (basis, reversed) = self.curve(scope, args.reference(1)?)?;
                (basis, reversed != !args.logical(4)?)
            }
            "HYPERBOLA" => (
                CurveDef::Conic {
                    frame: self.frame(scope, args.reference(1)?)?,
                    kind: ConicKind::Hyperbola {
                        a: args.real(2)? * scope.length,
                        b: args.real(3)? * scope.length,
                    },
                },
                false,
            ),
            "PARABOLA" => (
                CurveDef::Conic {
                    frame: self.frame(scope, args.reference(1)?)?,
                    kind: ConicKind::Parabola {
                        focal: args.real(2)? * scope.length,
                    },
                },
                false,
            ),
            "POLYLINE" => (
                CurveDef::Polyline(
                    args.references(1)?
                        .into_iter()
                        .map(|p| self.point(scope, p))
                        .collect::<GeopResult<_>>()?,
                ),
                false,
            ),
            // A curve on surfaces: its 3-D curve. The curves it has on the
            // surfaces are fitted anew, as the kernel fits every pcurve.
            _ => self.curve(scope, args.reference(1)?)?,
        })
    }

    fn nurbs_curve(&self, scope: &Scope, id: u64) -> GeopResult<NurbsCurve> {
        let instance = self.instance(id)?;
        // A simple instance holds every supertype's parameters before its
        // own: name, degree, points, form, closed, self-intersecting.
        let (base, own): (Args, Option<Args>) = match instance {
            crate::part21::Instance::Simple(record) => {
                let args = Args { id, record };
                (args, Some(args))
            }
            crate::part21::Instance::Complex(_) => (self.args(id, "B_SPLINE_CURVE")?, None),
        };
        let offset = usize::from(own.is_some());
        let degree = base.integer(offset)? as usize;
        let points: Vec<P3> = base
            .references(offset + 1)?
            .into_iter()
            .map(|p| self.point(scope, p))
            .collect::<GeopResult<_>>()?;
        let n = points.len();
        let subtype = |name: &str| -> Option<Args> {
            match own {
                Some(args) if args.record.name == name => Some(args),
                Some(_) => None,
                None => instance.record(name).map(|record| Args { id, record }),
            }
        };
        // Parameters of a subtype start after the supertype's in a simple
        // instance, at the start of its own record in a complex one.
        let sub_offset = if own.is_some() { 6 } else { 0 };
        let knots = if let Some(args) = subtype("B_SPLINE_CURVE_WITH_KNOTS") {
            expand_knots(
                id,
                &args.integers(sub_offset)?,
                &args.reals(sub_offset + 1)?,
            )?
        } else if subtype("BEZIER_CURVE").is_some() {
            bezier_knots(id, n, degree)?
        } else if subtype("QUASI_UNIFORM_CURVE").is_some() {
            quasi_uniform_knots(n, degree)
        } else if subtype("UNIFORM_CURVE").is_some() {
            (0..n + degree + 1)
                .map(|i| i as f64 - degree as f64)
                .collect()
        } else {
            return Err(unsupported(
                id,
                instance,
                "a B-spline curve without a knot vector",
            ));
        };
        if knots.len() != n + degree + 1 {
            return Err(GeopError::new(format!(
                "#{id} {}: {} knots for {n} control points of degree {degree}, where {} were expected",
                instance.type_name(),
                knots.len(),
                n + degree + 1
            )));
        }
        let weights = match instance.record("RATIONAL_B_SPLINE_CURVE") {
            Some(record) => {
                let args = Args { id, record };
                let w = args.reals(if own.is_some() { 6 } else { 0 })?;
                check_weights(id, &w, n)?;
                Some(w)
            }
            None => None,
        };
        Ok(NurbsCurve {
            degree,
            points,
            weights,
            knots,
        })
    }

    pub fn surface(&self, scope: &Scope, id: u64) -> GeopResult<SurfaceDef> {
        let instance = self.instance(id)?;
        if instance.is("B_SPLINE_SURFACE")
            || instance.is("B_SPLINE_SURFACE_WITH_KNOTS")
            || instance.is("BEZIER_SURFACE")
            || instance.is("UNIFORM_SURFACE")
            || instance.is("QUASI_UNIFORM_SURFACE")
        {
            return Ok(SurfaceDef {
                kind: SurfaceKind::Nurbs(self.nurbs_surface(scope, id)?),
                flipped: false,
            });
        }
        let (kind, args) = self
            .args_of_any(
                id,
                &[
                    "PLANE",
                    "CYLINDRICAL_SURFACE",
                    "CONICAL_SURFACE",
                    "SPHERICAL_SURFACE",
                    "TOROIDAL_SURFACE",
                    "SURFACE_OF_REVOLUTION",
                    "SURFACE_OF_LINEAR_EXTRUSION",
                    "RECTANGULAR_TRIMMED_SURFACE",
                ],
            )
            .map_err(|_| unsupported(id, instance, "this kind of surface cannot be read"))?;
        let revolved = |profile: Profile| -> GeopResult<SurfaceDef> {
            Ok(SurfaceDef {
                kind: SurfaceKind::Revolved(Revolved {
                    frame: self.frame(scope, args.reference(1)?)?,
                    profile,
                    on_axis: scope.uncertainty,
                }),
                flipped: false,
            })
        };
        let length = |k: usize| -> GeopResult<f64> { Ok(args.real(k)? * scope.length) };
        let positive = |k: usize, what: &str| -> GeopResult<f64> {
            let value = length(k)?;
            if value > 0.0 {
                Ok(value)
            } else {
                Err(unsupported(
                    id,
                    instance,
                    &format!("its {what} {value} is not positive"),
                ))
            }
        };
        match kind {
            "PLANE" => Ok(SurfaceDef {
                kind: SurfaceKind::Plane(self.frame(scope, args.reference(1)?)?),
                flipped: false,
            }),
            "CYLINDRICAL_SURFACE" => revolved(Profile::Line {
                rho: positive(2, "radius")?,
                h: 0.0,
                drho: 0.0,
                dh: 1.0,
            }),
            "CONICAL_SURFACE" => {
                let angle = args.real(3)? * scope.angle;
                if !(angle > 0.0 && angle < std::f64::consts::FRAC_PI_2) {
                    return Err(unsupported(
                        id,
                        instance,
                        &format!(
                            "its half angle {angle} rad is not between none and a right angle"
                        ),
                    ));
                }
                revolved(Profile::Line {
                    rho: length(2)?,
                    h: 0.0,
                    drho: angle.tan(),
                    dh: 1.0,
                })
            }
            "SPHERICAL_SURFACE" => revolved(Profile::Sphere {
                radius: positive(2, "radius")?,
            }),
            "TOROIDAL_SURFACE" => {
                let major = positive(2, "major radius")?;
                let minor = positive(3, "minor radius")?;
                revolved(Profile::Circle {
                    rho: major,
                    h: 0.0,
                    radius: minor,
                })
            }
            "SURFACE_OF_REVOLUTION" => {
                let (curve, reversed) = self.curve(scope, args.reference(1)?)?;
                let axis = self.args(args.reference(2)?, "AXIS1_PLACEMENT")?;
                let origin = self.point(scope, axis.reference(1)?)?;
                let z = if axis.is_null(2) {
                    scope.place.direction([0.0, 0.0, 1.0])
                } else {
                    self.direction(scope, axis.reference(2)?)?
                };
                let radial = |p: P3| {
                    let d = sub(p, origin);
                    sub(d, scale(z, dot(d, z)))
                };
                let off_plane = || {
                    unsupported(
                        id,
                        instance,
                        "a surface of revolution whose curve does not lie in a plane through its axis",
                    )
                };
                // The file's surface is parametrized by the turn, then the
                // curve (ISO 10303-42), its normal the turn across the
                // curve's tangent; the NURBS' is the turn across the
                // profile's — so they agree where the profile runs the
                // curve's way.
                let (frame, profile, along) = match curve {
                    CurveDef::Nurbs(curve) => {
                        // Angle 0 is the half plane the profile lies in.
                        let off_axis = curve
                            .points
                            .iter()
                            .map(|&p| radial(p))
                            .max_by(|a, b| norm(*a).total_cmp(&norm(*b)))
                            .unwrap_or([0.0; 3]);
                        let frame = Frame::new(origin, z, off_axis)?;
                        for &p in &curve.points {
                            let l = frame.local(p);
                            if l[1].abs() > scope.uncertainty || l[0] < -scope.uncertainty {
                                return Err(unsupported(
                                    id,
                                    instance,
                                    "a surface of revolution whose curve does not lie in one half plane through its axis",
                                ));
                            }
                        }
                        (frame, Profile::Curve(curve), true)
                    }
                    CurveDef::Line {
                        origin: p0,
                        direction,
                    } => {
                        let d = normalize(direction).ok_or_else(off_plane)?;
                        let off_axis = if norm(radial(p0)) > scope.uncertainty {
                            radial(p0)
                        } else {
                            radial(add(p0, d))
                        };
                        let frame = Frame::new(origin, z, off_axis)?;
                        let (l0, l1) = (frame.local(p0), frame.local(add(p0, d)));
                        if l0[1].abs() > scope.uncertainty || l1[1].abs() > scope.uncertainty {
                            return Err(off_plane());
                        }
                        let profile = Profile::Line {
                            rho: l0[0],
                            h: l0[2],
                            drho: l1[0] - l0[0],
                            dh: l1[2] - l0[2],
                        };
                        (frame, profile, true)
                    }
                    CurveDef::Circle {
                        frame: circle,
                        radius,
                    } => {
                        let center = radial(circle.origin);
                        let on_axis = norm(center) <= scope.uncertainty;
                        let reference = if on_axis {
                            radial(add(circle.origin, circle.x))
                        } else {
                            center
                        };
                        let frame = Frame::new(origin, z, reference)?;
                        let c = frame.local(circle.origin);
                        let n = [
                            dot(circle.z, frame.x),
                            dot(circle.z, frame.y),
                            dot(circle.z, frame.z),
                        ];
                        if c[1].abs() > scope.uncertainty || n[0].abs() > 1e-9 || n[2].abs() > 1e-9
                        {
                            return Err(off_plane());
                        }
                        // Turning from `x` towards `z` is about `-y`.
                        let along = n[1] < 0.0;
                        if on_axis {
                            let frame = Frame {
                                origin: frame.point([0.0, 0.0, c[2]]),
                                ..frame
                            };
                            (frame, Profile::Sphere { radius }, along)
                        } else {
                            (
                                frame,
                                Profile::Circle {
                                    rho: c[0],
                                    h: c[2],
                                    radius,
                                },
                                along,
                            )
                        }
                    }
                    CurveDef::Ellipse { .. } | CurveDef::Polyline(_) | CurveDef::Conic { .. } => {
                        return Err(unsupported(
                            id,
                            instance,
                            "a surface of revolution of an ellipse, a conic or a polyline",
                        ));
                    }
                };
                Ok(SurfaceDef {
                    kind: SurfaceKind::Revolved(Revolved {
                        frame,
                        profile,
                        on_axis: scope.uncertainty,
                    }),
                    flipped: along == reversed,
                })
            }
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                let (curve, _) = self.curve(scope, args.reference(1)?)?;
                let curve = match curve {
                    CurveDef::Nurbs(nurbs) => nurbs,
                    _ => {
                        return Err(unsupported(
                            id,
                            instance,
                            "a surface of extrusion of a curve other than a B-spline",
                        ));
                    }
                };
                Ok(SurfaceDef {
                    kind: SurfaceKind::Extrusion {
                        curve,
                        vector: self.vector(scope, args.reference(2)?)?,
                    },
                    flipped: false,
                })
            }
            "RECTANGULAR_TRIMMED_SURFACE" => {
                // The faces' bounds trim it: only the basis surface, and
                // whether its normal is turned, matter.
                let mut basis = self.surface(scope, args.reference(1)?)?;
                let turned = args.logical(6)? != args.logical(7)?;
                basis.flipped ^= turned;
                Ok(basis)
            }
            _ => unreachable!("one of the kinds asked for"),
        }
    }

    fn nurbs_surface(&self, scope: &Scope, id: u64) -> GeopResult<NurbsSurface> {
        let instance = self.instance(id)?;
        let simple = matches!(instance, crate::part21::Instance::Simple(_));
        let base = if simple {
            match instance {
                crate::part21::Instance::Simple(record) => Args { id, record },
                _ => unreachable!(),
            }
        } else {
            self.args(id, "B_SPLINE_SURFACE")?
        };
        let offset = usize::from(simple);
        let degree_u = base.integer(offset)? as usize;
        let degree_v = base.integer(offset + 1)? as usize;
        let rows = base.list(offset + 2)?;
        let mut points = Vec::new();
        let mut num_v = None;
        for row in rows {
            let crate::part21::Value::List(row) = row else {
                return Err(GeopError::new(format!(
                    "#{id} {}: its control points are not a list of rows",
                    instance.type_name()
                )));
            };
            if *num_v.get_or_insert(row.len()) != row.len() {
                return Err(GeopError::new(format!(
                    "#{id} {}: its rows of control points differ in length",
                    instance.type_name()
                )));
            }
            for p in row {
                let crate::part21::Value::Ref(p) = p else {
                    return Err(GeopError::new(format!(
                        "#{id} {}: a control point is not a reference",
                        instance.type_name()
                    )));
                };
                points.push(self.point(scope, *p)?);
            }
        }
        let num_u = rows.len();
        let num_v = num_v.unwrap_or(0);
        let subtype = |name: &str| -> Option<Args> {
            if simple {
                (base.record.name == name).then_some(base)
            } else {
                instance.record(name).map(|record| Args { id, record })
            }
        };
        let sub_offset = if simple { 8 } else { 0 };
        let (knots_u, knots_v) = if let Some(args) = subtype("B_SPLINE_SURFACE_WITH_KNOTS") {
            (
                expand_knots(
                    id,
                    &args.integers(sub_offset)?,
                    &args.reals(sub_offset + 2)?,
                )?,
                expand_knots(
                    id,
                    &args.integers(sub_offset + 1)?,
                    &args.reals(sub_offset + 3)?,
                )?,
            )
        } else if subtype("BEZIER_SURFACE").is_some() {
            (
                bezier_knots(id, num_u, degree_u)?,
                bezier_knots(id, num_v, degree_v)?,
            )
        } else if subtype("QUASI_UNIFORM_SURFACE").is_some() {
            (
                quasi_uniform_knots(num_u, degree_u),
                quasi_uniform_knots(num_v, degree_v),
            )
        } else {
            return Err(unsupported(
                id,
                instance,
                "a B-spline surface without a knot vector",
            ));
        };
        for (knots, n, p, dir) in [
            (&knots_u, num_u, degree_u, "u"),
            (&knots_v, num_v, degree_v, "v"),
        ] {
            if knots.len() != n + p + 1 {
                return Err(GeopError::new(format!(
                    "#{id} {}: {} knots in {dir} for {n} control points of degree {p}, where {} were expected",
                    instance.type_name(),
                    knots.len(),
                    n + p + 1
                )));
            }
        }
        let weights = match instance.record("RATIONAL_B_SPLINE_SURFACE") {
            Some(record) => {
                let args = Args { id, record };
                let k = if simple { 8 } else { 0 };
                let mut w = Vec::new();
                for row in args.list(k)? {
                    let crate::part21::Value::List(row) = row else {
                        return Err(GeopError::new(format!(
                            "#{id}: weights are not a list of rows"
                        )));
                    };
                    for x in row {
                        w.push(super::reader::real(x).ok_or_else(|| {
                            GeopError::new(format!("#{id}: a weight is not a number"))
                        })?);
                    }
                }
                check_weights(id, &w, num_u * num_v)?;
                Some(w)
            }
            None => None,
        };
        Ok(NurbsSurface {
            degree_u,
            degree_v,
            num_u,
            num_v,
            points,
            weights,
            knots_u,
            knots_v,
        })
    }
}

fn check_weights(id: u64, weights: &[f64], n: usize) -> GeopResult<()> {
    if weights.len() != n {
        return Err(GeopError::new(format!(
            "#{id}: {} weights for {n} control points",
            weights.len()
        )));
    }
    if let Some(w) = weights.iter().find(|w| !(**w > 0.0)) {
        return Err(GeopError::new(format!(
            "#{id}: the weight {w} is not positive"
        )));
    }
    Ok(())
}

fn expand_knots(id: u64, multiplicities: &[i64], knots: &[f64]) -> GeopResult<Vec<f64>> {
    if multiplicities.len() != knots.len() {
        return Err(GeopError::new(format!(
            "#{id}: {} knot multiplicities for {} knots",
            multiplicities.len(),
            knots.len()
        )));
    }
    let mut out = Vec::new();
    for (&m, &k) in multiplicities.iter().zip(knots) {
        if m < 1 {
            return Err(GeopError::new(format!("#{id}: a knot multiplicity of {m}")));
        }
        if out.last().is_some_and(|&last| last > k) {
            return Err(GeopError::new(format!("#{id}: its knots decrease")));
        }
        out.extend(std::iter::repeat_n(k, m as usize));
    }
    Ok(out)
}

fn bezier_knots(id: u64, n: usize, degree: usize) -> GeopResult<Vec<f64>> {
    if degree == 0 || (n - 1) % degree != 0 {
        return Err(GeopError::new(format!(
            "#{id}: {n} control points do not make Bezier pieces of degree {degree}"
        )));
    }
    let pieces = (n - 1) / degree;
    let mut knots = vec![0.0; degree + 1];
    for k in 1..pieces {
        knots.extend(std::iter::repeat_n(k as f64, degree));
    }
    knots.extend(std::iter::repeat_n(pieces as f64, degree + 1));
    Ok(knots)
}

fn quasi_uniform_knots(n: usize, degree: usize) -> Vec<f64> {
    let mut knots = vec![0.0; degree + 1];
    let inner = n.saturating_sub(degree + 1);
    for k in 1..=inner {
        knots.push(k as f64);
    }
    knots.extend(std::iter::repeat_n((inner + 1) as f64, degree + 1));
    knots
}

/// The point of `curve` at the parameter `t`, chosen in `f64`: at a domain
/// end, the end itself, which `t` may miss by rounding.
pub fn point_at<S: Scalar>(curve: &NurbCurve3D<S>, t: f64) -> GeopResult<P3> {
    let (lo, hi) = curve.domain();
    let t = if t <= lo.to_f64() {
        lo
    } else if t >= hi.to_f64() {
        hi
    } else {
        S::from_f64(t)
    };
    Ok(to_p3(&curve.evaluate(t)?))
}
