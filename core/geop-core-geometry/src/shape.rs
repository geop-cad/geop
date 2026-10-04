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
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use crate::{
    nurb_curve::{NurbCurve, NurbCurve3D, dehomogenize},
    nurb_surface::NurbSurface3D,
};

/// Whether `v` could be the zero vector.
fn could_be_zero<S: Scalar>(v: &Vector3<S>) -> bool {
    v.norm_sq().could_be_equal(S::ZERO)
}

/// Whether the unit vectors `a` and `b` could be parallel, in either sense.
fn could_be_parallel<S: Scalar>(a: &Vector3<S>, b: &Vector3<S>) -> bool {
    could_be_zero(&a.prod_cross(b))
}

/// A straight line: through `point`, along the unit vector `direction`.
#[derive(Clone, Debug)]
pub struct Axis<S: Scalar> {
    pub point: Vector3<S>,
    pub direction: Vector3<S>,
}

impl<S: Scalar> Axis<S> {
    /// The line through `point` along `direction`, which need not be unit
    /// length but must not be zero.
    pub fn try_new(point: Vector3<S>, direction: Vector3<S>) -> GeopResult<Self> {
        Ok(Self {
            point,
            direction: direction.normalize()?,
        })
    }

    /// The point of the line closest to `p`: the foot of the perpendicular
    /// from `p`.
    pub fn project(&self, p: &Vector3<S>) -> Vector3<S> {
        let along = p.sub(&self.point).prod_dot(&self.direction);
        self.point.add(&self.direction.prod_scalar(along))
    }

    /// Whether `p` could lie on the line.
    pub fn could_contain(&self, p: &Vector3<S>) -> bool {
        could_be_zero(&p.sub(&self.point).prod_cross(&self.direction))
    }

    /// Whether `other` could run parallel to this line, either way.
    pub fn could_be_parallel(&self, other: &Axis<S>) -> bool {
        could_be_parallel(&self.direction, &other.direction)
    }

    /// Where the two lines come closest: halfway between the point of each
    /// nearest the other — where they cross, if they do. Fails if they could
    /// be parallel: then every point is as near as any other.
    pub fn nearest(&self, other: &Axis<S>) -> GeopResult<Vector3<S>> {
        let n = self.direction.prod_cross(&other.direction);
        let n2 = n.norm_sq();
        if n2.could_be_equal(S::ZERO) {
            return Err(GeopError::new(
                "the lines are parallel, so no point of them is nearer the other than any other",
            ));
        }
        let r = other.point.sub(&self.point);
        let s = r.prod_cross(&other.direction).prod_dot(&n).div(n2)?;
        let t = r.prod_cross(&self.direction).prod_dot(&n).div(n2)?;
        let a = self.point.add(&self.direction.prod_scalar(s));
        let b = other.point.add(&other.direction.prod_scalar(t));
        Ok(Vector3::interpolate(&a, &b, S::ONE.div(S::TWO)?))
    }
}

/// A circle: around `center`, in the plane normal to the unit vector
/// `normal`, of `radius`.
#[derive(Clone, Debug)]
pub struct Circle<S: Scalar> {
    pub center: Vector3<S>,
    pub normal: Vector3<S>,
    pub radius: S,
}

impl<S: Scalar> Circle<S> {
    /// The line through the center along the normal: what the circle turns
    /// around.
    pub fn axis(&self) -> Axis<S> {
        Axis {
            point: self.center,
            direction: self.normal,
        }
    }

    fn could_be_equal(&self, other: &Circle<S>) -> bool {
        self.center.could_be_equal(&other.center)
            && self.normal.could_be_equal(&other.normal)
            && self.radius.could_be_equal(other.radius)
    }

    /// The smallest circle description enclosing both — two enclosures of
    /// one circle, combined (see `AGENTS.md` on `union`).
    fn union(&self, other: &Circle<S>) -> Circle<S> {
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
pub struct Arc<S: Scalar> {
    pub circle: Circle<S>,
    pub start: Vector3<S>,
    pub end: Vector3<S>,
}

impl<S: Scalar> Arc<S> {
    /// Whether the arc could be the whole circle.
    pub fn could_be_closed(&self) -> bool {
        self.start.could_be_equal(&self.end)
    }

    /// The angle the arc turns through, in radians, in `(0, 2 pi]`.
    ///
    /// A plain `f64`: `Scalar` has no inverse trigonometry, and this is
    /// only ever used to *choose* a point along the arc (see
    /// [`Arc::point_at`]).
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
    pub fn point_at(&self, fraction: f64) -> GeopResult<Vector3<S>> {
        let c = &self.circle;
        let x = self.start.sub(&c.center);
        let y = c.normal.prod_cross(&x);
        let angle = S::from_f64(fraction * self.sweep());
        Ok(c.center
            .add(&x.prod_scalar(angle.cos()))
            .add(&y.prod_scalar(angle.sin())))
    }

    /// The unit tangent at `p`, a point of the arc, pointing the way the arc
    /// runs.
    pub fn tangent_at(&self, p: &Vector3<S>) -> GeopResult<Vector3<S>> {
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
    pub fn to_curve(&self) -> GeopResult<NurbCurve3D<S>> {
        let quarter = std::f64::consts::FRAC_PI_2;
        let pieces = ((self.sweep() / quarter).ceil() as usize).max(1);
        let mut points = vec![self.start];
        for k in 1..pieces {
            points.push(self.point_at(k as f64 / pieces as f64)?);
        }
        points.push(self.end);
        let c = &self.circle.center;
        let r2 = self.circle.radius.mul(self.circle.radius);
        // The ends of the pieces weigh one: taken as they are, not
        // multiplied by it, which would only widen them by rounding.
        let end = |p: &Vector3<S>| Vector4::from_array([p[0], p[1], p[2], S::ONE]);
        let middle_point =
            |p: &Vector3<S>, w: S| Vector4::from_array([p[0].mul(w), p[1].mul(w), p[2].mul(w), w]);
        let mut control_points = vec![end(&points[0])];
        let mut knots = vec![S::ZERO; 3];
        for k in 0..pieces {
            let (a, b) = (points[k].sub(c), points[k + 1].sub(c));
            let one_plus_cos = S::ONE.add(a.prod_dot(&b).div(r2)?);
            let middle = c.add(&a.add(&b).prod_scalar(S::ONE.div(one_plus_cos)?));
            let weight = one_plus_cos.div(S::TWO)?.sqrt()?;
            control_points.push(middle_point(&middle, weight));
            control_points.push(end(&points[k + 1]));
            if k + 1 < pieces {
                let knot = S::from_ratio(k as i64 + 1, pieces as i64)?;
                knots.extend([knot, knot]);
            }
        }
        knots.extend([S::ONE; 3]);
        NurbCurve::try_new(2, control_points, knots)
    }
}

/// A plane: through `point`, normal to the unit vector `normal`.
#[derive(Clone, Debug)]
pub struct Plane<S: Scalar> {
    pub point: Vector3<S>,
    pub normal: Vector3<S>,
}

impl<S: Scalar> Plane<S> {
    /// The plane through `point` normal to `normal`, which need not be unit
    /// length but must not be zero.
    pub fn try_new(point: Vector3<S>, normal: Vector3<S>) -> GeopResult<Self> {
        Ok(Self {
            point,
            normal: normal.normalize()?,
        })
    }

    /// How far `p` lies in front of the plane (along the normal); negative
    /// behind it.
    pub fn signed_distance(&self, p: &Vector3<S>) -> S {
        p.sub(&self.point).prod_dot(&self.normal)
    }

    /// The point of the plane closest to `p`: the foot of the perpendicular
    /// from `p`.
    pub fn project(&self, p: &Vector3<S>) -> Vector3<S> {
        p.sub(&self.normal.prod_scalar(self.signed_distance(p)))
    }

    /// Where `axis` pierces the plane. Fails if it could run parallel to it.
    pub fn intersect_axis(&self, axis: &Axis<S>) -> GeopResult<Vector3<S>> {
        let along = axis.direction.prod_dot(&self.normal);
        if along.could_be_equal(S::ZERO) {
            return Err(GeopError::new(
                "the line runs parallel to the plane, so it does not pierce it",
            ));
        }
        let t = self.signed_distance(&axis.point).div(along)?;
        Ok(axis.point.sub(&axis.direction.prod_scalar(t)))
    }

    /// The line the two planes meet in, running along `self.normal x
    /// other.normal`. Fails if they could be parallel.
    pub fn intersect_plane(&self, other: &Plane<S>) -> GeopResult<Axis<S>> {
        let direction = self.normal.prod_cross(&other.normal);
        let n2 = direction.norm_sq();
        if n2.could_be_equal(S::ZERO) {
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
            .prod_scalar(S::ONE.div(n2)?);
        Axis::try_new(point, direction)
    }
}

// ── curves ────────────────────────────────────────────────────────────────────

impl<S: Scalar> NurbCurve3D<S> {
    /// The line the curve runs along, if it is straight: every control point
    /// on the line from the first to the last, which is then the curve's
    /// direction. `None` for a curve that bends, or whose ends coincide.
    pub fn as_line(&self) -> GeopResult<Option<Axis<S>>> {
        let points = dehomogenize::<S, 4, 3>(&self.control_points);
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
    pub fn as_arc(&self) -> GeopResult<Option<Arc<S>>> {
        if self.degree != 2 {
            return Ok(None);
        }
        let mut circle: Option<Circle<S>> = None;
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

    /// The curve cut at every interior knot: its polynomial (or rational)
    /// pieces, in order.
    fn bezier_pieces(&self) -> GeopResult<Vec<Self>> {
        let end = self.domain().1;
        let interior = &self.knot_vector[self.degree + 1..self.control_points.len()];
        let mut rest = self.clone();
        let mut pieces = Vec::new();
        for &k in interior {
            if k.definitely_greater(rest.domain().0) && k.definitely_less(end) {
                let (left, right) = rest.split(k)?;
                pieces.push(left);
                rest = right;
            }
        }
        pieces.push(rest);
        Ok(pieces)
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
fn bezier_circle<S: Scalar>(piece: &NurbCurve3D<S>) -> GeopResult<Option<Circle<S>>> {
    let [h0, h1, h2] = match piece.control_points.as_slice() {
        [a, b, c] => [*a, *b, *c],
        _ => return Ok(None),
    };
    let points = dehomogenize::<S, 4, 3>(&[h0, h1, h2]);
    let (p0, p1, p2) = (points[0], points[1], points[2]);
    let (w0, w1, w2) = (h0[3], h1[3], h2[3]);
    let tangent = p1.sub(&p0).norm_sq();
    if !tangent.could_be_equal(p1.sub(&p2).norm_sq()) {
        return Ok(None);
    }
    let chord = p2.sub(&p0).norm_sq();
    let four = S::TWO.add(S::TWO);
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
    let m = p0.add(&p2).prod_scalar(S::ONE.div(S::TWO)?);
    let h = m.sub(&p1).norm_sq();
    if h.could_be_equal(S::ZERO) {
        return Ok(None);
    }
    let center = p1.add(&m.sub(&p1).prod_scalar(tangent.div(h)?));
    let radial = p0.sub(&center);
    // The piece leaves `P0` towards `P1`: turning counter-clockwise about
    // `radial x (P1 - P0)`.
    let normal = radial.prod_cross(&p1.sub(&p0)).normalize()?;
    Ok(Some(Circle {
        center,
        normal,
        radius: radial.norm(),
    }))
}

// ── surfaces ──────────────────────────────────────────────────────────────────

impl<S: Scalar> NurbSurface3D<S> {
    /// The plane the surface lies in, if it is flat: every control point on
    /// the plane through its middle, normal to it there — the normal pointing
    /// the way the surface's own does. `None` for a surface that bends.
    pub fn as_plane(&self) -> GeopResult<Option<Plane<S>>> {
        let ((u0, u1), (v0, v1)) = (self.domain_u(), self.domain_v());
        let (u, v) = (u0.add(u1).div(S::TWO)?, v0.add(v1).div(S::TWO)?);
        let plane = Plane {
            point: self.evaluate(u, v)?,
            normal: self.normal(u, v)?,
        };
        let points = dehomogenize::<S, 4, 3>(&self.control_points);
        Ok(points
            .iter()
            .all(|p| plane.signed_distance(p).could_be_equal(S::ZERO))
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
    pub fn axis_of_revolution(&self) -> GeopResult<Option<Axis<S>>> {
        for along_u in [true, false] {
            if let Some(axis) = self.revolution_along(along_u)? {
                return Ok(Some(axis));
            }
        }
        Ok(None)
    }

    /// [`NurbSurface3D::axis_of_revolution`], for rows along `u` or along `v`.
    pub(crate) fn revolution_along(&self, along_u: bool) -> GeopResult<Option<Axis<S>>> {
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
        let mut reference: Option<(Arc<S>, Vec<S>)> = None;
        let mut poles = Vec::new();
        for j in 0..rows {
            let cps = row(j);
            let weights: Vec<S> = cps.iter().map(|p| p[3]).collect();
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
            let points = dehomogenize::<S, 4, 3>(&cps);
            if points.iter().all(|p| p.could_be_equal(&points[0])) {
                poles.push(points[0]);
                continue;
            }
            let Some(arc) = NurbCurve::try_new(degree, cps, knots.clone())?.as_arc()? else {
                return Ok(None);
            };
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
        Ok(poles.iter().all(|p| axis.could_contain(p)).then_some(axis))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use super::*;
    use crate::nurb_surface::NurbSurface;

    fn v<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    /// `(x, y, z)` with weight `w`, homogeneous.
    fn h<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([x * w, y * w, z * w, w].map(S::from_f64))
    }

    fn knots<S: Scalar>(ks: &[f64]) -> Vec<S> {
        ks.iter().map(|&k| S::from_f64(k)).collect()
    }

    const R2: f64 = std::f64::consts::FRAC_1_SQRT_2;

    /// The unit circle around the origin in the xy plane, counter-clockwise
    /// from `(1, 0, 0)`: four quarter arcs, the kernel's usual form.
    fn unit_circle<S: Scalar>() -> NurbCurve3D<S> {
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

    fn check_line<S: Scalar>() {
        let line = NurbCurve::<S, 4>::try_new(
            2,
            vec![h(0., 0., 0., 1.), h(1., 2., 2., 0.5), h(3., 6., 6., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        let axis = line.as_line().unwrap().expect("collinear control points");
        assert!(axis.direction.could_be_equal(&v(1. / 3., 2. / 3., 2. / 3.)));
        assert!(unit_circle::<S>().as_line().unwrap().is_none());
    }
    #[test]
    fn straight_curves_are_lines() {
        for_all_scalars!(check_line);
    }

    fn check_circle<S: Scalar>() {
        let arc = unit_circle::<S>().as_arc().unwrap().expect("a circle");
        assert!(arc.circle.center.could_be_equal(&v(0., 0., 0.)));
        assert!(arc.circle.normal.could_be_equal(&v(0., 0., 1.)));
        assert!(arc.circle.radius.could_be_equal(S::ONE));
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
    #[test]
    fn a_circle_is_recognized_with_its_turning_sense() {
        for_all_scalars!(check_circle);
    }

    /// A piece split off an arc — as a boolean leaves one — is still that
    /// arc's circle, with its own ends.
    fn check_split_arc<S: Scalar>() {
        let (left, _) = unit_circle::<S>().split(S::from_f64(1.3)).unwrap();
        let arc = left.as_arc().unwrap().expect("still circular");
        assert!(arc.circle.radius.could_be_equal(S::ONE));
        assert!(!arc.could_be_closed());
        assert!(arc.sweep() > std::f64::consts::FRAC_PI_2);
        assert!(arc.sweep() < std::f64::consts::PI);
        // Clockwise, seen from below: the same circle, the other normal.
        let reversed = left.reverse().as_arc().unwrap().expect("still circular");
        assert!(reversed.circle.normal.could_be_equal(&v(0., 0., -1.)));
    }
    #[test]
    fn a_split_arc_is_still_an_arc() {
        for_all_scalars!(check_split_arc);
    }

    /// A conic that is not a circle: the right corner, weighted wrong.
    fn check_not_circle<S: Scalar>() {
        let conic = NurbCurve::<S, 4>::try_new(
            2,
            vec![h(1., 0., 0., 1.), h(1., 1., 0., 0.5), h(0., 1., 0., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        assert!(conic.as_arc().unwrap().is_none());
        let lopsided = NurbCurve::<S, 4>::try_new(
            2,
            vec![h(1., 0., 0., 1.), h(1., 2., 0., R2), h(0., 1., 0., 1.)],
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        assert!(lopsided.as_arc().unwrap().is_none());
    }
    #[test]
    fn other_conics_are_not_arcs() {
        for_all_scalars!(check_not_circle);
    }

    fn check_arc_curve<S: Scalar>() {
        // Around (1, 1, 2), radius 2, turning about -z.
        let circle = Circle {
            center: v(1., 1., 2.),
            normal: v(0., 0., -1.),
            radius: S::from_f64(2.),
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
            let mid = curve.evaluate(S::from_f64(0.5)).unwrap();
            let expected = at(from + sweep / 2.);
            let off = (0..3)
                .map(|k| mid[k].sub(expected[k]).abs().to_f64())
                .fold(0., f64::max);
            assert!(off < 1e-7, "{mid:?} vs {expected:?}");
        }
    }
    #[test]
    fn arcs_become_curves() {
        for_all_scalars!(check_arc_curve);
    }

    /// A quarter of a cylinder of radius 2 around the z axis, `u` around it
    /// and `v` along it, 3 high.
    fn quarter_cylinder<S: Scalar>() -> NurbSurface3D<S> {
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

    fn check_revolution<S: Scalar>() {
        let cylinder = quarter_cylinder::<S>();
        let axis = cylinder.axis_of_revolution().unwrap().expect("a cylinder");
        assert!(axis.could_contain(&v(0., 0., 7.)));
        assert!(could_be_parallel(&axis.direction, &v(0., 0., 1.)));
        assert!(cylinder.as_plane().unwrap().is_none());

        // Twisted: the top ring starts a little further round.
        let mut twisted = cylinder.clone();
        twisted.control_points[1] = h(2., 0.1, 3., 1.);
        assert!(twisted.axis_of_revolution().unwrap().is_none());
    }
    #[test]
    fn a_cylinder_turns_around_its_axis() {
        for_all_scalars!(check_revolution);
    }

    fn check_plane<S: Scalar>() {
        let flat = NurbSurface::<S, 4>::try_new(
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
                .could_be_equal(S::ZERO)
        );
        let mut bent = flat.clone();
        bent.control_points[3] = h(1., 1., 1.2, 1.);
        assert!(bent.as_plane().unwrap().is_none());
    }
    #[test]
    fn flat_surfaces_are_planes() {
        for_all_scalars!(check_plane);
    }

    fn check_constructions<S: Scalar>() {
        let z = Plane::<S>::try_new(v(0., 0., 2.), v(0., 0., 3.)).unwrap();
        let x = Plane::try_new(v(1., 0., 0.), v(1., 0., 0.)).unwrap();
        assert!(z.project(&v(4., 5., 6.)).could_be_equal(&v(4., 5., 2.)));
        let line = z.intersect_plane(&x).unwrap();
        assert!(line.could_contain(&v(1., 7., 2.)));
        assert!(could_be_parallel(&line.direction, &v(0., 1., 0.)));
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
    #[test]
    fn planes_and_axes_meet_where_they_should() {
        for_all_scalars!(check_constructions);
    }
}
