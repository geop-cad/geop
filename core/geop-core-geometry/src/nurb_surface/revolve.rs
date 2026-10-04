use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::NurbSurface;
use crate::{nurb_curve::NurbCurve3D, shape::Axis};

impl<S: Scalar> NurbSurface<S, 4> {
    /// The surface `profile` sweeps turning about `axis` from the angle
    /// `from` to the angle `to`, in radians, right-handed about
    /// `axis.direction`: the profile as it is at angle 0, turned.
    ///
    /// `u` is the turn, on `[0, 1]`: rational quadratic pieces of equal
    /// angle, each at most a quarter turn, the way [`crate::shape::Arc`]
    /// builds an arc. `v` is the profile's own parameter. So `Su x Sv`
    /// points away from the axis where the profile runs along
    /// `axis.direction`.
    ///
    /// A profile control point on the axis stays put at every angle: its
    /// row collapses to one point, a pole of the parametrization.
    ///
    /// The angles of the joints between pieces are free choices, taken in
    /// `f64` as [`crate::shape::Arc::point_at`] does: any angle serves
    /// equally well, and each joint lies honestly on the circle it is built
    /// from. Fails unless `to` is more than `from`, by at most a full turn.
    pub fn revolve(
        profile: &NurbCurve3D<S>,
        axis: &Axis<S>,
        from: f64,
        to: f64,
    ) -> GeopResult<Self> {
        let sweep = to - from;
        if !(sweep > 0.0 && sweep <= std::f64::consts::TAU) {
            return Err(GeopError::new(format!(
                "NurbSurface::revolve: cannot turn from {from} to {to}: it must be more than none and at most a full turn"
            )));
        }
        let pieces = ((sweep / std::f64::consts::FRAC_PI_2).ceil() as usize).max(1);
        let step = sweep / pieces as f64;
        let angles: Vec<f64> = (0..=pieces).map(|k| from + step * k as f64).collect();
        // Each piece's middle control point is where the tangents at its
        // ends meet: the ends' radii summed, over `1 + cos(step)`, weighted
        // `cos(step / 2)`.
        let one_plus_cos = S::ONE.add(S::from_f64(step).cos());
        let middle_weight = S::from_f64(step / 2.0).cos();

        let num_v = profile.control_points.len();
        let mut columns: Vec<Vec<Vector4<S>>> = Vec::with_capacity(num_v);
        for cp in &profile.control_points {
            let w = cp[3];
            let q = Vector3::from_array([cp[0].div(w)?, cp[1].div(w)?, cp[2].div(w)?]);
            let center = axis.project(&q);
            let x = q.sub(&center);
            let y = axis.direction.prod_cross(&x);
            // The radius at `angle`. At angle 0 the profile point itself,
            // taken as it is: turning it by nothing would only widen it.
            let radius_at = |angle: f64| {
                if angle == 0.0 {
                    x
                } else {
                    let a = S::from_f64(angle);
                    x.prod_scalar(a.cos()).add(&y.prod_scalar(a.sin()))
                }
            };
            let homogeneous = |p: Vector3<S>, weight: S| {
                Vector4::from_array([p[0].mul(weight), p[1].mul(weight), p[2].mul(weight), weight])
            };
            let mut column = Vec::with_capacity(2 * pieces + 1);
            let mut previous = radius_at(angles[0]);
            column.push(if angles[0] == 0.0 {
                *cp
            } else {
                homogeneous(center.add(&previous), w)
            });
            for &angle in &angles[1..] {
                let next = radius_at(angle);
                let middle =
                    center.add(&previous.add(&next).prod_scalar(S::ONE.div(one_plus_cos)?));
                column.push(homogeneous(middle, w.mul(middle_weight)));
                column.push(homogeneous(center.add(&next), w));
                previous = next;
            }
            columns.push(column);
        }

        let mut control_points = Vec::with_capacity((2 * pieces + 1) * num_v);
        for i in 0..2 * pieces + 1 {
            for column in &columns {
                control_points.push(column[i]);
            }
        }
        let mut knots_u = vec![S::ZERO; 3];
        for k in 1..pieces {
            let knot = S::from_ratio(k as i64, pieces as i64)?;
            knots_u.extend([knot, knot]);
        }
        knots_u.extend([S::ONE; 3]);
        NurbSurface::try_new(
            2,
            profile.degree,
            control_points,
            knots_u,
            profile.knot_vector.clone(),
        )
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        for_all_scalars,
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

    use super::super::NurbSurface3D;
    use crate::{nurb_curve::NurbCurve3D, shape::Axis};

    fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
    }

    fn line<S: Scalar>(a: [f64; 3], b: [f64; 3]) -> NurbCurve3D<S> {
        let f = S::from_f64;
        let h = |c: [f64; 3]| Vector4::from_array([f(c[0]), f(c[1]), f(c[2]), S::ONE]);
        NurbCurve3D::try_new(1, vec![h(a), h(b)], vec![S::ZERO, S::ZERO, S::ONE, S::ONE]).unwrap()
    }

    /// A line beside the axis turns into a cylinder: every point at its
    /// radius, at the height it had, the normal pointing away from the axis.
    fn check_a_line_turns_into_a_cylinder<S: Scalar>() {
        let axis = Axis::try_new(v3(1.0, 2.0, 0.0), v3(0.0, 0.0, 1.0)).unwrap();
        let profile = line([3.0, 2.0, -1.0], [3.0, 2.0, 4.0]);
        for (from, to) in [(0.0, std::f64::consts::TAU), (0.5, 2.0), (-1.0, 3.0)] {
            let surface = NurbSurface3D::revolve(&profile, &axis, from, to).unwrap();
            for (u, v) in [(0.0, 0.0), (0.3, 0.5), (0.77, 1.0), (1.0, 0.25)] {
                let (u, v) = (S::from_f64(u), S::from_f64(v));
                let p = surface.evaluate(u, v).unwrap();
                let r = p.sub(&v3(1.0, 2.0, p[2].to_f64())).norm();
                assert!(r.could_be_equal(S::from_f64(2.0)), "radius {r:?}");
                let normal = surface.normal(u, v).unwrap();
                let outward = p.sub(&axis.project(&p)).normalize().unwrap();
                assert!(normal.prod_dot(&outward).definitely_greater(S::ZERO));
            }
            let start = surface.evaluate(S::ZERO, S::ZERO).unwrap();
            let (cos, sin) = (S::from_f64(from).cos(), S::from_f64(from).sin());
            let expected = Vector3::from_array([
                S::ONE.add(S::TWO.mul(cos)),
                S::TWO.add(S::TWO.mul(sin)),
                S::from_f64(-1.0),
            ]);
            assert!(
                start.could_be_equal(&expected),
                "{from}: {start:?} vs {expected:?}"
            );
        }
    }
    #[test]
    fn a_line_turns_into_a_cylinder() {
        for_all_scalars!(check_a_line_turns_into_a_cylinder);
    }

    /// A profile touching the axis collapses there: a pole.
    fn check_a_point_on_the_axis_is_a_pole<S: Scalar>() {
        let axis = Axis::try_new(v3(0.0, 0.0, 0.0), v3(0.0, 0.0, 1.0)).unwrap();
        let profile = line([0.0, 0.0, 2.0], [1.0, 0.0, 0.0]);
        let surface = NurbSurface3D::revolve(&profile, &axis, 0.0, 3.0).unwrap();
        for u in [0.0, 0.4, 1.0] {
            let apex = surface.evaluate(S::from_f64(u), S::ZERO).unwrap();
            assert!(apex.could_be_equal(&v3(0.0, 0.0, 2.0)));
        }
        assert!(NurbSurface3D::revolve(&profile, &axis, 1.0, 1.0).is_err());
        assert!(NurbSurface3D::revolve(&profile, &axis, 0.0, 7.0).is_err());
    }
    #[test]
    fn a_point_on_the_axis_is_a_pole() {
        for_all_scalars!(check_a_point_on_the_axis_is_a_pole);
    }
}
