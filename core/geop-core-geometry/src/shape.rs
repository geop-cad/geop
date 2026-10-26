//! Recognizing the elementary shapes a NURBS curve or surface can be — a
//! straight line, a circular arc, a plane, a surface of revolution — for
//! whatever needs to know *what* an entity is, not just where it is: a
//! reference axis built along an edge, a sketch placed on a face.
//!
//! Every test is a question about the control net, answered with the
//! kernel's three-valued comparisons: a shape is recognized when the net
//! *could* be exactly that shape — rounding can't rule it out — and never
//! because it is merely close to it. A curve that bends by a micron is not a
//! line, however short.
//!
//! The shapes themselves ([`Axis`], [`Circle`], [`Arc`], [`Plane`]) carry
//! the few constructions everything built on them needs: projecting onto
//! them, intersecting them.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
};

use crate::{
    nurb_curve::{NurbCurve, NurbCurve3D, dehomogenize},
    nurb_surface::NurbSurface3D,
};

/// Whether `v` could be the zero vector.
fn could_be_zero(v: &Vector3) -> bool {
    v.norm_sq().could_be_equal(Scalar::ZERO)
}

/// A straight line: through `point`, along the unit vector `direction`.
#[derive(Clone, Debug, PartialEq)]
pub struct Axis {
    pub point: Vector3,
    pub direction: Vector3,
}

impl Axis {
    /// The line through `point` along `direction`, which need not be unit
    /// length but must not be zero.
    pub fn try_new(point: Vector3, direction: Vector3) -> GeopResult<Self> {
        Ok(Self {
            point,
            direction: direction.normalize()?,
        })
    }

    /// The point of the line closest to `p`: the foot of the perpendicular
    /// from `p`.
    pub fn project(&self, p: &Vector3) -> Vector3 {
        let along = p.sub(&self.point).prod_dot(&self.direction);
        self.point.add(&self.direction.prod_scalar(along))
    }

    /// Whether `p` could lie on the line.
    pub fn could_contain(&self, p: &Vector3) -> bool {
        could_be_zero(&p.sub(&self.point).prod_cross(&self.direction))
    }

    /// Whether `other` could run parallel to this line, either way.
    pub fn could_be_parallel(&self, other: &Axis) -> bool {
        self.direction.could_be_parallel(&other.direction)
    }

    /// Where the two lines come closest: halfway between the point of each
    /// nearest the other — where they cross, if they do. Fails if they could
    /// be parallel: then every point is as near as any other.
    pub fn nearest(&self, other: &Axis) -> GeopResult<Vector3> {
        let n = self.direction.prod_cross(&other.direction);
        let n2 = n.norm_sq();
        if n2.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(
                "the lines are parallel, so no point of them is nearer the other than any other",
            ));
        }
        let r = other.point.sub(&self.point);
        let s = r.prod_cross(&other.direction).prod_dot(&n).div(n2)?;
        let t = r.prod_cross(&self.direction).prod_dot(&n).div(n2)?;
        let a = self.point.add(&self.direction.prod_scalar(s));
        let b = other.point.add(&other.direction.prod_scalar(t));
        Ok(Vector3::interpolate(&a, &b, Scalar::ONE.div(Scalar::TWO)?))
    }

    /// The line moved by `motion`.
    pub fn transform(&self, motion: &Motion) -> Self {
        Self {
            point: motion.apply(&self.point),
            direction: motion.rotate(&self.direction),
        }
    }
}

/// A circle: around `center`, in the plane normal to the unit vector
/// `normal`, of `radius`.
#[derive(Clone, Debug)]
pub struct Circle {
    pub center: Vector3,
    pub normal: Vector3,
    pub radius: Scalar,
}

impl Circle {
    /// The line through the center along the normal: what the circle turns
    /// around.
    pub fn axis(&self) -> Axis {
        Axis {
            point: self.center,
            direction: self.normal,
        }
    }

    /// The circle moved by `motion`.
    pub fn transform(&self, motion: &Motion) -> Self {
        Self {
            center: motion.apply(&self.center),
            normal: motion.rotate(&self.normal),
            radius: self.radius,
        }
    }

    /// Whether `p` could lie on the circle: in its plane, `radius` from
    /// its center.
    pub fn could_contain(&self, p: &Vector3) -> bool {
        let radial = p.sub(&self.center);
        radial.prod_dot(&self.normal).could_be_equal(Scalar::ZERO)
            && radial.norm().could_be_equal(self.radius)
    }

    fn could_be_equal(&self, other: &Circle) -> bool {
        self.center.could_be_equal(&other.center)
            && self.normal.could_be_equal(&other.normal)
            && self.radius.could_be_equal(other.radius)
    }

    /// The smallest circle description enclosing both — two enclosures of
    /// one circle, combined (see `AGENTS.md` on `union`).
    fn union(&self, other: &Circle) -> Circle {
        Circle {
            center: self.center.union(&other.center),
            normal: self.normal.union(&other.normal),
            radius: self.radius.union(other.radius),
        }
    }
}

/// A circular arc: the part of `circle` from `start` counter-clockwise
/// (seen with `circle.normal` towards the viewer) to `end` — all of it when
/// `start` and `end` coincide.
#[derive(Clone, Debug)]
pub struct Arc {
    pub circle: Circle,
    pub start: Vector3,
    pub end: Vector3,
}

impl Arc {
    /// The arc moved by `motion`. A mirror turns counter-clockwise into
    /// clockwise, so the mirrored arc runs from its end to its start.
    pub fn transform(&self, motion: &Motion) -> Self {
        let (start, end) = (motion.apply(&self.start), motion.apply(&self.end));
        let (start, end) = match motion.mirrors() {
            true => (end, start),
            false => (start, end),
        };
        Self {
            circle: self.circle.transform(motion),
            start,
            end,
        }
    }

    /// Whether the arc could be the whole circle.
    pub fn could_be_closed(&self) -> bool {
        self.start.could_be_equal(&self.end)
    }

    /// The angle the arc turns through, in radians, in `(0, 2 pi]`: an
    /// enclosure, for what is measured — its length ([`Arc::length`]).
    /// The whole turn for an arc that could be closed, as [`Arc::sweep`]
    /// takes it.
    pub fn angle(&self) -> GeopResult<Scalar> {
        if self.could_be_closed() {
            return Ok(Scalar::TWO.mul(Scalar::PI));
        }
        let c = &self.circle;
        let x = self.start.sub(&c.center);
        let e = self.end.sub(&c.center);
        let angle = c.normal.prod_dot(&x.prod_cross(&e)).atan2(x.prod_dot(&e));
        if angle.definitely_greater(Scalar::ZERO) {
            Ok(angle)
        } else if angle.definitely_less(Scalar::ZERO) {
            Ok(angle.add(Scalar::TWO.mul(Scalar::PI)))
        } else {
            Err(GeopError::new(format!(
                "Arc::angle: the arc could turn by nothing or all round, from {:?} to {:?} about {:?}",
                self.start, self.end, c.center
            )))
        }
    }

    /// How long the arc is: its radius times the angle it turns through.
    pub fn length(&self) -> GeopResult<Scalar> {
        Ok(self.circle.radius.mul(self.angle()?))
    }

    /// The angle the arc turns through, in radians, in `(0, 2 pi]`, as a
    /// plain `f64`: only ever used to *choose* a point along the arc (see
    /// [`Arc::point_at`]); what is measured takes [`Arc::angle`].
    pub fn sweep(&self) -> f64 {
        if self.could_be_closed() {
            return std::f64::consts::TAU;
        }
        let c = &self.circle;
        let x = self.start.sub(&c.center);
        let e = self.end.sub(&c.center);
        let cos = x.prod_dot(&e).to_f64();
        let sin = c.normal.prod_dot(&x.prod_cross(&e)).to_f64();
        let angle = sin.atan2(cos);
        if angle > 0.0 {
            angle
        } else {
            angle + std::f64::consts::TAU
        }
    }

    /// The point `fraction` of the way along the arc, by angle: `start` at
    /// 0, `end` at 1.
    ///
    /// The angle is computed in `f64` (see [`Arc::sweep`]) and taken as
    /// sharp — legitimately: which point is "0.3 of the way" is a choice
    /// the caller makes, and any angle within rounding of it serves that
    /// choice equally well. What the result has to be *honestly* is a point
    /// on the circle, and it is, since it is built from the circle itself.
    pub fn point_at(&self, fraction: f64) -> GeopResult<Vector3> {
        let c = &self.circle;
        let x = self.start.sub(&c.center);
        let y = c.normal.prod_cross(&x);
        let angle = Scalar::from_f64(fraction * self.sweep());
        Ok(c.center
            .add(&x.prod_scalar(angle.cos()))
            .add(&y.prod_scalar(angle.sin())))
    }

    /// The unit tangent at `p`, a point of the arc, pointing the way the arc
    /// runs.
    pub fn tangent_at(&self, p: &Vector3) -> GeopResult<Vector3> {
        self.circle
            .normal
            .prod_cross(&p.sub(&self.circle.center))
            .normalize()
    }

    /// The arc as a curve, the way the kernel builds every arc: rational
    /// quadratic pieces of at most a quarter turn each, on `[0, 1]`,
    /// running from `start` to `end` exactly.
    ///
    /// Where the pieces meet is a free choice (see [`Arc::point_at`]): any
    /// point of the circle serves. Each piece from `P0` to `P2` around the
    /// center `C`, turning through `phi`, has its middle control point
    /// where the tangents at its ends meet, `C + (P0 - C + P2 - C) / (1 +
    /// cos phi)`, weighted `cos(phi / 2) = sqrt((1 + cos phi) / 2)` — both
    /// read off the ends, so no angle is taken.
    pub fn to_curve(&self) -> GeopResult<NurbCurve3D> {
        let quarter = std::f64::consts::FRAC_PI_2;
        let pieces = ((self.sweep() / quarter).ceil() as usize).max(1);
        let mut points = vec![self.start];
        for k in 1..pieces {
            points.push(self.point_at(k as f64 / pieces as f64)?);
        }
        points.push(self.end);
        // Relative to a sharp point of the arc, a free choice, and added
        // back with one rounding: the middle control point is reached
        // through the centre, and far from the origin each rounding of a
        // coordinate costs its size. The weight is what multiplies the
        // far coordinate, so its width is kept to what the local figure
        // gives it.
        let origin = points[0].sharpen();
        let c = self.circle.center.sub(&origin);
        let r2 = self.circle.radius.mul(self.circle.radius);
        let mut control_points = vec![points[0].homogeneous(Scalar::ONE)];
        let mut knots = vec![Scalar::ZERO; 3];
        for k in 0..pieces {
            let (a, b) = (
                points[k].sub(&origin).sub(&c),
                points[k + 1].sub(&origin).sub(&c),
            );
            let one_plus_cos = Scalar::ONE.add(a.prod_dot(&b).div(r2)?);
            let middle = origin.add(&c.add(&a.add(&b).prod_scalar(Scalar::ONE.div(one_plus_cos)?)));
            let weight = one_plus_cos.div(Scalar::TWO)?.sqrt()?;
            control_points.push(middle.homogeneous(weight));
            control_points.push(points[k + 1].homogeneous(Scalar::ONE));
            if k + 1 < pieces {
                let knot = Scalar::from_ratio(k as i64 + 1, pieces as i64)?;
                knots.extend([knot, knot]);
            }
        }
        knots.extend([Scalar::ONE; 3]);
        NurbCurve::try_new(2, control_points, knots)
    }
}

/// A plane: through `point`, normal to the unit vector `normal`.
#[derive(Clone, Debug)]
pub struct Plane {
    pub point: Vector3,
    pub normal: Vector3,
}

impl Plane {
    /// The plane through `point` normal to `normal`, which need not be unit
    /// length but must not be zero.
    pub fn try_new(point: Vector3, normal: Vector3) -> GeopResult<Self> {
        Ok(Self {
            point,
            normal: normal.normalize()?,
        })
    }

    /// How far `p` lies in front of the plane (along the normal); negative
    /// behind it.
    pub fn signed_distance(&self, p: &Vector3) -> Scalar {
        p.sub(&self.point).prod_dot(&self.normal)
    }

    /// The point of the plane closest to `p`: the foot of the perpendicular
    /// from `p`.
    pub fn project(&self, p: &Vector3) -> Vector3 {
        p.sub(&self.normal.prod_scalar(self.signed_distance(p)))
    }

    /// Where `axis` pierces the plane. Fails if it could run parallel to it.
    pub fn intersect_axis(&self, axis: &Axis) -> GeopResult<Vector3> {
        let along = axis.direction.prod_dot(&self.normal);
        if along.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(
                "the line runs parallel to the plane, so it does not pierce it",
            ));
        }
        let t = self.signed_distance(&axis.point).div(along)?;
        Ok(axis.point.sub(&axis.direction.prod_scalar(t)))
    }

    /// The line the two planes meet in, running along `self.normal x
    /// other.normal`. Fails if they could be parallel.
    pub fn intersect_plane(&self, other: &Plane) -> GeopResult<Axis> {
        let direction = self.normal.prod_cross(&other.normal);
        let n2 = direction.norm_sq();
        if n2.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(
                "the planes are parallel, so they do not meet",
            ));
        }
        // The point of the line closest to the origin: the combination of
        // both normals that lies on both planes.
        let (d1, d2) = (
            self.point.prod_dot(&self.normal),
            other.point.prod_dot(&other.normal),
        );
        let point = other
            .normal
            .prod_cross(&direction)
            .prod_scalar(d1)
            .add(&direction.prod_cross(&self.normal).prod_scalar(d2))
            .prod_scalar(Scalar::ONE.div(n2)?);
        Axis::try_new(point, direction)
    }
}

// ── curves ────────────────────────────────────────────────────────────────────

impl NurbCurve3D {
    /// The line the curve runs along, if it is straight: every control point
    /// on the line from the first to the last, which is then the curve's
    /// direction. `None` for a curve that bends, or whose ends coincide.
    pub fn as_line(&self) -> GeopResult<Option<Axis>> {
        let points = dehomogenize::<4, 3>(&self.control_points);
        let (first, last) = (points[0], points[points.len() - 1]);
        let d = last.sub(&first);
        if could_be_zero(&d) {
            return Ok(None);
        }
        let axis = Axis::try_new(first, d)?;
        Ok(points.iter().all(|p| axis.could_contain(p)).then_some(axis))
    }

    /// The arc the curve traces, if it is a circular one: rational quadratic
    /// pieces, each an exact arc, all of one circle — how the kernel builds
    /// every arc and circle, and what splitting one leaves. `None` for any
    /// other curve.
    pub fn as_arc(&self) -> GeopResult<Option<Arc>> {
        if self.degree != 2 {
            return Ok(None);
        }
        let mut circle: Option<Circle> = None;
        for piece in self.bezier_pieces()? {
            let Some(c) = bezier_circle(&piece)? else {
                return Ok(None);
            };
            circle = Some(match circle {
                None => c,
                Some(prev) if prev.could_be_equal(&c) => prev.union(&c),
                Some(_) => return Ok(None),
            });
        }
        let Some(circle) = circle else {
            return Ok(None);
        };
        let (t0, t1) = self.domain();
        Ok(Some(Arc {
            circle,
            start: self.evaluate(t0)?,
            end: self.evaluate(t1)?,
        }))
    }
}

/// The circle a rational quadratic Bézier piece traces, if it is an exact
/// circular arc.
///
/// With end points `P0`, `P2`, middle control point `P1` and weights `w0`,
/// `w1`, `w2`, the piece is an arc exactly when `P1` is where the arc's end
/// tangents meet — equally far from both ends, `|P1 - P0| = |P1 - P2|` — and
/// the weight normalized to `w0 = w2 = 1`, `w1 / sqrt(w0 w2)`, is the cosine
/// of the angle `theta` between the chord and those tangents:
/// `cos theta = |P2 - P0| / (2 |P1 - P0|)`. Squared, so no root is taken.
fn bezier_circle(piece: &NurbCurve3D) -> GeopResult<Option<Circle>> {
    let [h0, h1, h2] = match piece.control_points.as_slice() {
        [a, b, c] => [*a, *b, *c],
        _ => return Ok(None),
    };
    let points = dehomogenize::<4, 3>(&[h0, h1, h2]);
    // Relative to a sharp point beside the piece, a free choice: far from
    // the origin every rounding of a coordinate costs its size, and the
    // centre is reached through a dozen of them. Added back at the end with
    // one rounding (see "Evaluate relative to the patch, not the origin").
    let origin = points[0].sharpen();
    let (p0, p1, p2) = (
        points[0].sub(&origin),
        points[1].sub(&origin),
        points[2].sub(&origin),
    );
    let (w0, w1, w2) = (h0[3], h1[3], h2[3]);
    let tangent = p1.sub(&p0).norm_sq();
    if !tangent.could_be_equal(p1.sub(&p2).norm_sq()) {
        return Ok(None);
    }
    let chord = p2.sub(&p0).norm_sq();
    let four = Scalar::TWO.add(Scalar::TWO);
    if !four
        .mul(w1)
        .mul(w1)
        .mul(tangent)
        .could_be_equal(w0.mul(w2).mul(chord))
    {
        return Ok(None);
    }
    // The center lies on the line from `P1` through the chord's midpoint
    // `M`, `|P1 - P0|^2 / |P1 - M|^2` times as far from `P1` as `M` is: the
    // triangle `P0 P1 center` has its right angle at `P0`.
    let m = p0.add(&p2).prod_scalar(Scalar::ONE.div(Scalar::TWO)?);
    let h = m.sub(&p1).norm_sq();
    if h.could_be_equal(Scalar::ZERO) {
        return Ok(None);
    }
    let center = p1.add(&m.sub(&p1).prod_scalar(tangent.div(h)?));
    let radial = p0.sub(&center);
    // The piece leaves `P0` towards `P1`: turning counter-clockwise about
    // `radial x (P1 - P0)`.
    let normal = radial.prod_cross(&p1.sub(&p0)).normalize()?;
    Ok(Some(Circle {
        center: origin.add(&center),
        normal,
        radius: radial.norm(),
    }))
}

// ── surfaces ──────────────────────────────────────────────────────────────────

impl NurbSurface3D {
    /// The plane the surface lies in, if it is flat: every control point on
    /// the plane through its middle, normal to it there — the normal pointing
    /// the way the surface's own does. `None` for a surface that bends.
    pub fn as_plane(&self) -> GeopResult<Option<Plane>> {
        let ((u0, u1), (v0, v1)) = (self.domain_u(), self.domain_v());
        let (u, v) = (u0.add(u1).div(Scalar::TWO)?, v0.add(v1).div(Scalar::TWO)?);
        let plane = Plane {
            point: self.evaluate(u, v)?,
            normal: self.normal(u, v)?,
        };
        let points = dehomogenize::<4, 3>(&self.control_points);
        Ok(points
            .iter()
            .all(|p| plane.signed_distance(p).could_be_equal(Scalar::ZERO))
            .then_some(plane))
    }

    /// The axis the surface turns around, if it is a surface of revolution —
    /// a cylinder, a cone, a sphere, a torus, a disc: one of its parameter
    /// directions sweeps circular arcs around a common axis. `None` for any
    /// other surface.
    ///
    /// Checked on the control net: every row of control points along that
    /// direction is an arc (see [`NurbCurve3D::as_arc`]) around the one
    /// axis, starting at the same angle, with weights proportional to every
    /// other row's — or a single point on the axis, as at a sphere's pole.
    /// Then every row turns through the same angles at the same parameters,
    /// and a blend of them across the other direction is the blended
    /// profile, turned: a surface of revolution.
    pub fn axis_of_revolution(&self) -> GeopResult<Option<Axis>> {
        for along_u in [true, false] {
            if let Some(axis) = self.revolution_along(along_u)? {
                return Ok(Some(axis));
            }
        }
        Ok(None)
    }

    /// The cylinder the surface lies on, if it is a piece of one: a
    /// surface of revolution (see [`NurbSurface3D::axis_of_revolution`])
    /// whose every row of control points is an arc of one radius — the same
    /// arc, then, moved along the axis — with no row collapsed to a pole.
    /// `None` for a cone, a sphere, a disc or any other surface.
    pub fn as_cylinder(&self) -> GeopResult<Option<Cylinder>> {
        for along_u in [true, false] {
            let Some(revolution) = self.revolution_rows(along_u)? else {
                continue;
            };
            if !revolution.poles.is_empty() {
                continue;
            }
            let first = revolution.radii[0];
            if revolution.radii.iter().all(|r| r.could_be_equal(first)) {
                let radius = revolution.radii.iter().fold(first, |a, &r| a.union(r));
                return Ok(Some(Cylinder {
                    axis: revolution.axis,
                    radius,
                }));
            }
        }
        Ok(None)
    }

    /// [`NurbSurface3D::axis_of_revolution`], for rows along `u` or along `v`.
    pub(crate) fn revolution_along(&self, along_u: bool) -> GeopResult<Option<Axis>> {
        Ok(self.revolution_rows(along_u)?.map(|r| r.axis))
    }

    /// [`NurbSurface3D::revolution_along`], with the radius of every row
    /// that is an arc and the point of every row that is a pole.
    fn revolution_rows(&self, along_u: bool) -> GeopResult<Option<Revolution>> {
        let (rows, len, degree, knots) = if along_u {
            (self.num_v, self.num_u, self.degree_u, &self.knot_vector_u)
        } else {
            (self.num_u, self.num_v, self.degree_v, &self.knot_vector_v)
        };
        let row = |j: usize| -> Vec<_> {
            (0..len)
                .map(|i| {
                    let index = if along_u {
                        i * self.num_v + j
                    } else {
                        j * self.num_v + i
                    };
                    self.control_points[index]
                })
                .collect()
        };
        // The first row that is an arc sets the axis, the angle every row
        // starts at, and the weights every row is proportional to.
        let mut reference: Option<(Arc, Vec<Scalar>)> = None;
        let mut poles = Vec::new();
        let mut radii = Vec::new();
        for j in 0..rows {
            let cps = row(j);
            let weights: Vec<Scalar> = cps.iter().map(|p| p[3]).collect();
            if let Some((_, reference_weights)) = &reference {
                let proportional = (0..len).all(|i| {
                    weights[i]
                        .mul(reference_weights[0])
                        .could_be_equal(reference_weights[i].mul(weights[0]))
                });
                if !proportional {
                    return Ok(None);
                }
            }
            let points = dehomogenize::<4, 3>(&cps);
            if points.iter().all(|p| p.could_be_equal(&points[0])) {
                poles.push(points[0]);
                continue;
            }
            let Some(arc) = NurbCurve::try_new(degree, cps, knots.clone())?.as_arc()? else {
                return Ok(None);
            };
            radii.push(arc.circle.radius);
            match &reference {
                None => reference = Some((arc, weights)),
                Some((first, _)) => {
                    let axis = first.circle.axis();
                    let same_angle = arc
                        .start
                        .sub(&arc.circle.center)
                        .normalize()?
                        .could_be_equal(&first.start.sub(&first.circle.center).normalize()?);
                    if !arc.circle.normal.could_be_equal(&axis.direction)
                        || !axis.could_contain(&arc.circle.center)
                        || !same_angle
                    {
                        return Ok(None);
                    }
                }
            }
        }
        let Some((first, _)) = reference else {
            return Ok(None);
        };
        let axis = first.circle.axis();
        Ok(poles
            .iter()
            .all(|p| axis.could_contain(p))
            .then_some(Revolution { axis, radii, poles }))
    }
}

/// What [`NurbSurface3D::revolution_rows`] finds a surface of revolution's
/// control rows to be: arcs around `axis` of `radii`, and `poles` on it.
struct Revolution {
    axis: Axis,
    radii: Vec<Scalar>,
    poles: Vec<Vector3>,
}

/// A cylinder: the points `radius` away from `axis`.
#[derive(Clone, Debug)]
pub struct Cylinder {
    pub axis: Axis,
    pub radius: Scalar,
}

#[cfg(test)]
mod tests {
    use crate::test_fixtures::knots;
    use geop_core_math::{
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use super::*;
    use crate::nurb_surface::NurbSurface;

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    /// `(x, y, z)` with weight `w`, homogeneous.
    fn h(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
        Vector4::from_array([x * w, y * w, z * w, w].map(Scalar::from_f64))
    }

    const R2: f64 = std::f64::consts::FRAC_1_SQRT_2;

    /// The unit circle around the origin in the xy plane, counter-clockwise
    /// from `(1, 0, 0)`: four quarter arcs, the kernel's usual form.
    fn unit_circle() -> NurbCurve3D {
        let corners = [
            (1., 0.),
            (1., 1.),
            (0., 1.),
            (-1., 1.),
            (-1., 0.),
            (-1., -1.),
            (0., -1.),
            (1., -1.),
            (1., 0.),
        ];
        let cps = corners
            .iter()
            .enumerate()
            .map(|(i, &(x, y))| h(x, y, 0., if i % 2 == 1 { R2 } else { 1. }))
            .collect();
        NurbCurve::try_new(
            2,
            cps,
            knots(&[0., 0., 0., 1., 1., 2., 2., 3., 3., 4., 4., 4.]),
        )
        .unwrap()
    }

    #[test]
    fn straight_curves_are_lines() {
        let line = NurbCurve::<4>::try_new(
            2,
            vec![h(0., 0., 0., 1.), h(1., 2., 2., 0.5), h(3., 6., 6., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        let axis = line.as_line().unwrap().expect("collinear control points");
        assert!(axis.direction.could_be_equal(&v(1. / 3., 2. / 3., 2. / 3.)));
        assert!(unit_circle().as_line().unwrap().is_none());
    }

    #[test]
    fn a_circle_is_recognized_with_its_turning_sense() {
        let arc = unit_circle().as_arc().unwrap().expect("a circle");
        assert!(arc.circle.center.could_be_equal(&v(0., 0., 0.)));
        assert!(arc.circle.normal.could_be_equal(&v(0., 0., 1.)));
        assert!(arc.circle.radius.could_be_equal(Scalar::ONE));
        assert!(arc.could_be_closed());
        assert!(arc.point_at(0.5).unwrap().could_be_equal(&v(-1., 0., 0.)));
        let ninety = arc.point_at(0.25).unwrap();
        assert!(ninety.could_be_equal(&v(0., 1., 0.)));
        assert!(
            arc.tangent_at(&ninety)
                .unwrap()
                .could_be_equal(&v(-1., 0., 0.))
        );
    }

    /// A piece split off an arc — as a boolean leaves one — is still that
    /// arc's circle, with its own ends.
    #[test]
    fn a_split_arc_is_still_an_arc() {
        let (left, _) = unit_circle().split(Scalar::from_f64(1.3)).unwrap();
        let arc = left.as_arc().unwrap().expect("still circular");
        assert!(arc.circle.radius.could_be_equal(Scalar::ONE));
        assert!(!arc.could_be_closed());
        assert!(arc.sweep() > std::f64::consts::FRAC_PI_2);
        assert!(arc.sweep() < std::f64::consts::PI);
        // Measured, the angle and the length are enclosures of the turn.
        let angle = arc.angle().unwrap();
        assert!(
            angle.could_be_equal(Scalar::from_f64(arc.sweep())),
            "{angle:?}"
        );
        let length = arc.length().unwrap();
        assert!(length.could_be_equal(angle), "{length:?}");
        assert!(length.width().to_f64() < 1e-5, "{length:?}");
        // Clockwise, seen from below: the same circle, the other normal.
        let reversed = left.reverse().as_arc().unwrap().expect("still circular");
        assert!(reversed.circle.normal.could_be_equal(&v(0., 0., -1.)));
    }

    /// A conic that is not a circle: the right corner, weighted wrong.
    #[test]
    fn other_conics_are_not_arcs() {
        let conic = NurbCurve::<4>::try_new(
            2,
            vec![h(1., 0., 0., 1.), h(1., 1., 0., 0.5), h(0., 1., 0., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        assert!(conic.as_arc().unwrap().is_none());
        let lopsided = NurbCurve::<4>::try_new(
            2,
            vec![h(1., 0., 0., 1.), h(1., 2., 0., R2), h(0., 1., 0., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        assert!(lopsided.as_arc().unwrap().is_none());
    }

    #[test]
    fn arcs_become_curves() {
        // Around (1, 1, 2), radius 2, turning about -z.
        let circle = Circle {
            center: v(1., 1., 2.),
            normal: v(0., 0., -1.),
            radius: Scalar::from_f64(2.),
        };
        let at = |angle: f64| v(1. + 2. * angle.cos(), 1. - 2. * angle.sin(), 2.);
        for (from, to) in [(0.3, 1.2), (0.3, 3.0), (-2.0, 3.5), (0.5, 0.5)] {
            let arc = Arc {
                circle: circle.clone(),
                start: at(from),
                end: at(to),
            };
            let curve = arc.to_curve().unwrap();
            let traced = curve.as_arc().unwrap().expect("an arc");
            assert!(traced.circle.could_be_equal(&circle), "{traced:?}");
            assert!(traced.start.could_be_equal(&arc.start));
            assert!(traced.end.could_be_equal(&arc.end));
            // It runs the way the arc does, all the way round for a circle:
            // its pieces turn alike, so it is halfway round halfway along.
            let sweep = if from == to {
                std::f64::consts::TAU
            } else {
                to - from
            };
            let mid = curve.evaluate(Scalar::from_f64(0.5)).unwrap();
            let expected = at(from + sweep / 2.);
            let off = (0..3)
                .map(|k| mid[k].sub(expected[k]).abs().to_f64())
                .fold(0., f64::max);
            assert!(off < 1e-7, "{mid:?} vs {expected:?}");
        }
    }

    /// A quarter of a cylinder of radius 2 around the z axis, `u` around it
    /// and `v` along it, 3 high.
    fn quarter_cylinder() -> NurbSurface3D {
        let ring = [(2., 0., 1.), (2., 2., R2), (0., 2., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, w)| [h(x, y, 0., w), h(x, y, 3., w)])
            .collect();
        NurbSurface::try_new(
            2,
            1,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    #[test]
    fn a_cylinder_turns_around_its_axis() {
        let cylinder = quarter_cylinder();
        let axis = cylinder.axis_of_revolution().unwrap().expect("a cylinder");
        assert!(axis.could_contain(&v(0., 0., 7.)));
        assert!(axis.direction.could_be_parallel(&v(0., 0., 1.)));
        assert!(cylinder.as_plane().unwrap().is_none());

        // Twisted: the top ring starts a little further round.
        let mut twisted = cylinder.clone();
        twisted.control_points[1] = h(2., 0.1, 3., 1.);
        assert!(twisted.axis_of_revolution().unwrap().is_none());

        let found = cylinder.as_cylinder().unwrap().expect("a cylinder");
        assert!(found.radius.could_be_equal(Scalar::from_f64(2.0)));
        assert!(found.axis.could_contain(&v(0., 0., -1.)));
        // Narrowing towards the top: a cone, which turns around the same
        // axis but is no cylinder.
        let mut cone = cylinder.clone();
        for (k, &(x, y, w)) in [(1., 0., 1.), (1., 1., R2), (0., 1., 1.)]
            .iter()
            .enumerate()
        {
            cone.control_points[2 * k + 1] = h(x, y, 3., w);
        }
        assert!(cone.axis_of_revolution().unwrap().is_some());
        assert!(cone.as_cylinder().unwrap().is_none());
    }

    #[test]
    fn flat_surfaces_are_planes() {
        let flat = NurbSurface::<4>::try_new(
            1,
            1,
            vec![
                h(0., 0., 1., 1.),
                h(0., 1., 1., 1.),
                h(1., 0., 1., 1.),
                h(1., 1., 1., 1.),
            ],
            knots(&[0., 0., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        let plane = flat.as_plane().unwrap().expect("flat");
        assert!(plane.normal.could_be_equal(&v(0., 0., 1.)));
        assert!(
            plane
                .signed_distance(&v(5., 5., 1.))
                .could_be_equal(Scalar::ZERO)
        );
        let mut bent = flat.clone();
        bent.control_points[3] = h(1., 1., 1.2, 1.);
        assert!(bent.as_plane().unwrap().is_none());
    }

    #[test]
    fn planes_and_axes_meet_where_they_should() {
        let z = Plane::try_new(v(0., 0., 2.), v(0., 0., 3.)).unwrap();
        let x = Plane::try_new(v(1., 0., 0.), v(1., 0., 0.)).unwrap();
        assert!(z.project(&v(4., 5., 6.)).could_be_equal(&v(4., 5., 2.)));
        let line = z.intersect_plane(&x).unwrap();
        assert!(line.could_contain(&v(1., 7., 2.)));
        assert!(line.direction.could_be_parallel(&v(0., 1., 0.)));
        let slanted = Axis::try_new(v(0., 0., 0.), v(1., 1., 1.)).unwrap();
        assert!(
            z.intersect_axis(&slanted)
                .unwrap()
                .could_be_equal(&v(2., 2., 2.))
        );
        assert!(
            slanted
                .project(&v(3., 0., 0.))
                .could_be_equal(&v(1., 1., 1.))
        );
        let crossing = Axis::try_new(v(2., 0., 2.), v(0., 1., 0.)).unwrap();
        assert!(
            slanted
                .nearest(&crossing)
                .unwrap()
                .could_be_equal(&v(2., 2., 2.))
        );
        // Missing each other: halfway along the common perpendicular, from
        // (2.5, 2.5, 2.5) to (2, 2.5, 3).
        let skew = Axis::try_new(v(2., 0., 3.), v(0., 1., 0.)).unwrap();
        assert!(
            slanted
                .nearest(&skew)
                .unwrap()
                .could_be_equal(&v(2.25, 2.5, 2.75))
        );
        assert!(slanted.nearest(&slanted).is_err());
        assert!(z.intersect_plane(&z).is_err());
    }
}
