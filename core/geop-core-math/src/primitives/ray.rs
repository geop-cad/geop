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
#[serde(try_from = "RayData")]
pub struct Ray {
    origin: Vector3,
    dir: Vector3,
}

/// A ray as it is written: any direction, normalized on reading.
#[derive(Deserialize)]
struct RayData {
    origin: Vector3,
    dir: Vector3,
}

impl TryFrom<RayData> for Ray {
    type Error = GeopError;

    fn try_from(data: RayData) -> GeopResult<Self> {
        Ray::try_new(data.origin, data.dir)
    }
}

/// `t`, or zero if it is definitely negative: a distance along a ray, which
/// never lies behind its origin.
fn ahead(t: Scalar) -> Scalar {
    if t.definitely_less(Scalar::ZERO) {
        Scalar::ZERO
    } else {
        t
    }
}

impl Ray {
    /// The ray from `origin` along `dir`, of any length. Fails if `dir`
    /// could be zero.
    pub fn try_new(origin: Vector3, dir: Vector3) -> GeopResult<Self> {
        let dir = dir
            .normalize()
            .map_err(|e| e.with_context(format!("Ray::try_new(origin={origin:?}, dir={dir:?})")))?;
        Ok(Self { origin, dir })
    }

    pub fn origin(&self) -> &Vector3 {
        &self.origin
    }

    /// Unit length.
    pub fn dir(&self) -> &Vector3 {
        &self.dir
    }

    /// The point `t` along the ray.
    pub fn at(&self, t: Scalar) -> Vector3 {
        self.origin.add(&self.dir.prod_scalar(t))
    }

    /// How far along the ray it comes closest to `p`, never behind its
    /// origin.
    pub fn closest_to_point(&self, p: &Vector3) -> Scalar {
        ahead(p.sub(&self.origin).prod_dot(&self.dir))
    }

    /// How far the ray passes from `p`, and how far along it that is.
    pub fn distance_to_point(&self, p: &Vector3) -> (Scalar, Scalar) {
        let t = self.closest_to_point(p);
        (self.at(t).sub(p).norm(), t)
    }

    /// How far the ray passes from the segment `a..b`, and how far along
    /// the ray that is.
    pub fn distance_to_segment(&self, a: &Vector3, b: &Vector3) -> (Scalar, Scalar) {
        let d = b.sub(a);
        let r = self.origin.sub(a);
        let e = d.norm_sq();
        if e.could_be_equal(Scalar::ZERO) {
            return self.distance_to_point(a);
        }
        let b_ = self.dir.prod_dot(&d);
        let c = self.dir.prod_dot(&r);
        let f = d.prod_dot(&r);
        // `dir` has unit length, so `dir . dir = 1`: the ray and the
        // segment's line are parallel exactly where this vanishes.
        let denom = e.sub(b_.mul(b_));
        let t = if denom.could_be_equal(Scalar::ZERO) {
            Scalar::ZERO
        } else {
            ahead(b_.mul(f).sub(c.mul(e)).div(denom).unwrap_or(Scalar::ZERO))
        };
        let u = b_.mul(t).add(f).div(e).unwrap_or(Scalar::ZERO);
        let (u, t) = if u.definitely_less(Scalar::ZERO) {
            (Scalar::ZERO, self.closest_to_point(a))
        } else if u.definitely_greater(Scalar::ONE) {
            (Scalar::ONE, self.closest_to_point(b))
        } else {
            (u, t)
        };
        let on_segment = a.add(&d.prod_scalar(u));
        (self.at(t).sub(&on_segment).norm(), t)
    }

    /// How far along the ray it enters the triangle `a, b, c`, if it could
    /// (Möller–Trumbore).
    pub fn intersect_triangle(&self, a: &Vector3, b: &Vector3, c: &Vector3) -> Option<Scalar> {
        let e1 = b.sub(a);
        let e2 = c.sub(a);
        let h = self.dir.prod_cross(&e2);
        let det = e1.prod_dot(&h);
        if det.could_be_equal(Scalar::ZERO) {
            return None;
        }
        let s = self.origin.sub(a);
        let q = s.prod_cross(&e1);
        let u = s.prod_dot(&h).div(det).ok()?;
        let v = self.dir.prod_dot(&q).div(det).ok()?;
        let t = e2.prod_dot(&q).div(det).ok()?;
        let outside = u.definitely_less(Scalar::ZERO)
            || v.definitely_less(Scalar::ZERO)
            || u.add(v).definitely_greater(Scalar::ONE)
            || t.definitely_less(Scalar::ZERO);
        (!outside).then_some(t)
    }

    /// Where the ray meets the plane through `origin` normal to `normal`,
    /// and how far along it: `None` if it could run along the plane, or
    /// points away from it.
    pub fn intersect_plane(&self, origin: &Vector3, normal: &Vector3) -> Option<(Scalar, Vector3)> {
        let denom = self.dir.prod_dot(normal);
        if denom.could_be_equal(Scalar::ZERO) {
            return None;
        }
        let t = origin.sub(&self.origin).prod_dot(normal).div(denom).ok()?;
        (!t.definitely_less(Scalar::ZERO)).then(|| (t, self.at(t)))
    }

    /// Where the ray meets `frame`'s `u`/`v` plane, in `u`/`v` coordinates,
    /// and how far along the ray: `None` if it could run along the plane,
    /// or points away from it.
    pub fn intersect_uv_plane(&self, frame: &CoordinateSystem) -> Option<(Scalar, Vector2)> {
        let (t, p) = self.intersect_plane(frame.origin(), frame.w())?;
        Some((t, frame.to_uv(&p)))
    }

    /// The parameter `s` of the point `at + s direction` of a line nearest
    /// the ray's line. `None` if the two could be parallel, where no point
    /// of the line is nearer than another.
    pub fn line_parameter(&self, at: &Vector3, direction: &Vector3) -> Option<Scalar> {
        let w0 = at.sub(&self.origin);
        let b = direction.prod_dot(&self.dir);
        let denom = direction.norm_sq().sub(b.mul(b));
        if denom.could_be_equal(Scalar::ZERO) {
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

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    /// Looking straight down `-z` from `(x, y, 10)`.
    fn down(x: f64, y: f64) -> Ray {
        Ray::try_new(v(x, y, 10.0), v(0.0, 0.0, -2.0)).unwrap()
    }

    /// Whether `a` encloses `b`.
    fn close(a: Scalar, b: f64) -> bool {
        a.could_be_equal(Scalar::from_f64(b))
    }

    #[test]
    fn points_and_segments() {
        let (dist, t) = down(0.0, 0.3).distance_to_point(&v(0.0, 0.0, 1.0));
        assert!(close(dist, 0.3) && close(t, 9.0));
        // Past the end of the segment, its end is nearest.
        let (dist, _) = down(2.0, 0.0).distance_to_segment(&v(0.0, 0.0, 0.0), &v(1.0, 0.0, 0.0));
        assert!(close(dist, 1.0));
        let (dist, t) = down(0.5, 0.2).distance_to_segment(&v(0.0, 0.0, 0.0), &v(1.0, 0.0, 0.0));
        assert!(close(dist, 0.2) && close(t, 10.0));
    }

    #[test]
    fn triangles_and_planes() {
        let (a, b, c) = (v(0.0, 0.0, 1.0), v(1.0, 0.0, 1.0), v(0.0, 1.0, 1.0));
        assert!(close(
            down(0.2, 0.2).intersect_triangle(&a, &b, &c).unwrap(),
            9.0
        ));
        assert!(down(0.8, 0.8).intersect_triangle(&a, &b, &c).is_none());
        let (t, p) = down(1.0, 2.0)
            .intersect_plane(&v(0.0, 0.0, -1.0), &v(0.0, 0.0, 1.0))
            .unwrap();
        assert!(close(t, 11.0) && close(p[1], 2.0));
        // Behind the ray, and along it.
        assert!(
            down(0.0, 0.0)
                .intersect_plane(&v(0.0, 0.0, 20.0), &v(0.0, 0.0, 1.0))
                .is_none()
        );
        assert!(
            down(0.0, 0.0)
                .intersect_plane(&v(0.0, 0.0, 0.0), &v(1.0, 0.0, 0.0))
                .is_none()
        );
    }

    /// Seen from the side, the point of a line along `z` nearest the ray
    /// follows the ray's height — and looking straight along the line, no
    /// point of it is nearest.
    #[test]
    fn line_parameters() {
        let side = Ray::try_new(v(5.0, 0.0, 0.7), v(-1.0, 0.0, 0.0)).unwrap();
        let s = side
            .line_parameter(&v(0.0, 0.0, 0.0), &v(0.0, 0.0, 2.0))
            .unwrap();
        assert!(close(s, 0.35));
        assert!(
            down(0.0, 0.0)
                .line_parameter(&v(0.0, 0.0, 0.0), &v(0.0, 0.0, 1.0))
                .is_none()
        );
    }
}
