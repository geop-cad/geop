//! [`Ray`]: a half-line from a point in a direction, and how near it passes
//! to points, segments, triangles, planes and lines.

use serde::{Deserialize, Serialize};

use super::CoordinateSystem;
use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

/// The points `origin + t dir` for `t >= 0`. `dir` has unit length, so `t`
/// is a distance.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(try_from = "RayData<S>", bound = "S: Scalar")]
pub struct Ray<S: Scalar> {
    origin: Vector3<S>,
    dir: Vector3<S>,
}

/// A ray as it is written: any direction, normalized on reading.
#[derive(Deserialize)]
#[serde(bound(deserialize = "S: Scalar"))]
struct RayData<S: Scalar> {
    origin: Vector3<S>,
    dir: Vector3<S>,
}

impl<S: Scalar> TryFrom<RayData<S>> for Ray<S> {
    type Error = GeopError;

    fn try_from(data: RayData<S>) -> GeopResult<Self> {
        Ray::try_new(data.origin, data.dir)
    }
}

/// `t`, or zero if it is definitely negative: a distance along a ray, which
/// never lies behind its origin.
fn ahead<S: Scalar>(t: S) -> S {
    if t.definitely_less(S::ZERO) {
        S::ZERO
    } else {
        t
    }
}

impl<S: Scalar> Ray<S> {
    /// The ray from `origin` along `dir`, of any length. Fails if `dir`
    /// could be zero.
    pub fn try_new(origin: Vector3<S>, dir: Vector3<S>) -> GeopResult<Self> {
        let dir = dir
            .normalize()
            .map_err(|e| e.with_context(format!("Ray::try_new(origin={origin:?}, dir={dir:?})")))?;
        Ok(Self { origin, dir })
    }

    pub fn origin(&self) -> &Vector3<S> {
        &self.origin
    }

    /// Unit length.
    pub fn dir(&self) -> &Vector3<S> {
        &self.dir
    }

    /// The point `t` along the ray.
    pub fn at(&self, t: S) -> Vector3<S> {
        self.origin.add(&self.dir.prod_scalar(t))
    }

    /// How far along the ray it comes closest to `p`, never behind its
    /// origin.
    pub fn closest_to_point(&self, p: &Vector3<S>) -> S {
        ahead(p.sub(&self.origin).prod_dot(&self.dir))
    }

    /// How far the ray passes from `p`, and how far along it that is.
    pub fn distance_to_point(&self, p: &Vector3<S>) -> (S, S) {
        let t = self.closest_to_point(p);
        (self.at(t).sub(p).norm(), t)
    }

    /// How far the ray passes from the segment `a..b`, and how far along
    /// the ray that is.
    pub fn distance_to_segment(&self, a: &Vector3<S>, b: &Vector3<S>) -> (S, S) {
        let d = b.sub(a);
        let r = self.origin.sub(a);
        let e = d.norm_sq();
        if e.could_be_equal(S::ZERO) {
            return self.distance_to_point(a);
        }
        let b_ = self.dir.prod_dot(&d);
        let c = self.dir.prod_dot(&r);
        let f = d.prod_dot(&r);
        // `dir` has unit length, so `dir . dir = 1`: the ray and the
        // segment's line are parallel exactly where this vanishes.
        let denom = e.sub(b_.mul(b_));
        let t = if denom.could_be_equal(S::ZERO) {
            S::ZERO
        } else {
            ahead(b_.mul(f).sub(c.mul(e)).div(denom).unwrap_or(S::ZERO))
        };
        let u = b_.mul(t).add(f).div(e).unwrap_or(S::ZERO);
        let (u, t) = if u.definitely_less(S::ZERO) {
            (S::ZERO, self.closest_to_point(a))
        } else if u.definitely_greater(S::ONE) {
            (S::ONE, self.closest_to_point(b))
        } else {
            (u, t)
        };
        let on_segment = a.add(&d.prod_scalar(u));
        (self.at(t).sub(&on_segment).norm(), t)
    }

    /// How far along the ray it enters the triangle `a, b, c`, if it could
    /// (Möller–Trumbore).
    pub fn intersect_triangle(&self, a: &Vector3<S>, b: &Vector3<S>, c: &Vector3<S>) -> Option<S> {
        let e1 = b.sub(a);
        let e2 = c.sub(a);
        let h = self.dir.prod_cross(&e2);
        let det = e1.prod_dot(&h);
        if det.could_be_equal(S::ZERO) {
            return None;
        }
        let s = self.origin.sub(a);
        let q = s.prod_cross(&e1);
        let u = s.prod_dot(&h).div(det).ok()?;
        let v = self.dir.prod_dot(&q).div(det).ok()?;
        let t = e2.prod_dot(&q).div(det).ok()?;
        let outside = u.definitely_less(S::ZERO)
            || v.definitely_less(S::ZERO)
            || u.add(v).definitely_greater(S::ONE)
            || t.definitely_less(S::ZERO);
        (!outside).then_some(t)
    }

    /// Where the ray meets the plane through `origin` normal to `normal`,
    /// and how far along it: `None` if it could run along the plane, or
    /// points away from it.
    pub fn intersect_plane(
        &self,
        origin: &Vector3<S>,
        normal: &Vector3<S>,
    ) -> Option<(S, Vector3<S>)> {
        let denom = self.dir.prod_dot(normal);
        if denom.could_be_equal(S::ZERO) {
            return None;
        }
        let t = origin.sub(&self.origin).prod_dot(normal).div(denom).ok()?;
        (!t.definitely_less(S::ZERO)).then(|| (t, self.at(t)))
    }

    /// Where the ray meets `frame`'s `u`/`v` plane, in `u`/`v` coordinates,
    /// and how far along the ray: `None` if it could run along the plane,
    /// or points away from it.
    pub fn intersect_uv_plane(&self, frame: &CoordinateSystem<S>) -> Option<(S, Vector2<S>)> {
        let (t, p) = self.intersect_plane(frame.origin(), frame.w())?;
        Some((t, frame.to_uvw(&p).head()))
    }

    /// The parameter `s` of the point `at + s direction` of a line nearest
    /// the ray's line. `None` if the two could be parallel, where no point
    /// of the line is nearer than another.
    pub fn line_parameter(&self, at: &Vector3<S>, direction: &Vector3<S>) -> Option<S> {
        let w0 = at.sub(&self.origin);
        let b = direction.prod_dot(&self.dir);
        let denom = direction.norm_sq().sub(b.mul(b));
        if denom.could_be_equal(S::ZERO) {
            return None;
        }
        b.mul(self.dir.prod_dot(&w0))
            .sub(direction.prod_dot(&w0))
            .div(denom)
            .ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::for_all_scalars;

    fn v<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    /// Looking straight down `-z` from `(x, y, 10)`.
    fn down<S: Scalar>(x: f64, y: f64) -> Ray<S> {
        Ray::try_new(v(x, y, 10.0), v(0.0, 0.0, -2.0)).unwrap()
    }

    /// Whether `a` encloses `b`.
    fn close<S: Scalar>(a: S, b: f64) -> bool {
        a.could_be_equal(S::from_f64(b))
    }

    fn check_points_and_segments<S: Scalar>() {
        let (dist, t) = down::<S>(0.0, 0.3).distance_to_point(&v(0.0, 0.0, 1.0));
        assert!(close(dist, 0.3) && close(t, 9.0));
        // Past the end of the segment, its end is nearest.
        let (dist, _) =
            down::<S>(2.0, 0.0).distance_to_segment(&v(0.0, 0.0, 0.0), &v(1.0, 0.0, 0.0));
        assert!(close(dist, 1.0));
        let (dist, t) =
            down::<S>(0.5, 0.2).distance_to_segment(&v(0.0, 0.0, 0.0), &v(1.0, 0.0, 0.0));
        assert!(close(dist, 0.2) && close(t, 10.0));
    }

    #[test]
    fn points_and_segments() {
        for_all_scalars!(check_points_and_segments);
    }

    fn check_triangles_and_planes<S: Scalar>() {
        let (a, b, c) = (v::<S>(0.0, 0.0, 1.0), v(1.0, 0.0, 1.0), v(0.0, 1.0, 1.0));
        assert!(close(
            down::<S>(0.2, 0.2).intersect_triangle(&a, &b, &c).unwrap(),
            9.0
        ));
        assert!(down::<S>(0.8, 0.8).intersect_triangle(&a, &b, &c).is_none());
        let (t, p) = down::<S>(1.0, 2.0)
            .intersect_plane(&v(0.0, 0.0, -1.0), &v(0.0, 0.0, 1.0))
            .unwrap();
        assert!(close(t, 11.0) && close(p[1], 2.0));
        // Behind the ray, and along it.
        assert!(
            down::<S>(0.0, 0.0)
                .intersect_plane(&v(0.0, 0.0, 20.0), &v(0.0, 0.0, 1.0))
                .is_none()
        );
        assert!(
            down::<S>(0.0, 0.0)
                .intersect_plane(&v(0.0, 0.0, 0.0), &v(1.0, 0.0, 0.0))
                .is_none()
        );
    }

    #[test]
    fn triangles_and_planes() {
        for_all_scalars!(check_triangles_and_planes);
    }

    /// Seen from the side, the point of a line along `z` nearest the ray
    /// follows the ray's height — and looking straight along the line, no
    /// point of it is nearest.
    fn check_line_parameters<S: Scalar>() {
        let side = Ray::try_new(v::<S>(5.0, 0.0, 0.7), v(-1.0, 0.0, 0.0)).unwrap();
        let s = side
            .line_parameter(&v(0.0, 0.0, 0.0), &v(0.0, 0.0, 2.0))
            .unwrap();
        assert!(close(s, 0.35));
        assert!(
            down::<S>(0.0, 0.0)
                .line_parameter(&v(0.0, 0.0, 0.0), &v(0.0, 0.0, 1.0))
                .is_none()
        );
    }

    #[test]
    fn line_parameters() {
        for_all_scalars!(check_line_parameters);
    }
}
