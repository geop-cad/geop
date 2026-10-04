//! Offsetting a surface along its normal, exactly.
//!
//! The offset of a NURBS surface is in general not a NURBS surface at all.
//! It is one, with the very same parametrization, for the surfaces the
//! kernel's solids are mostly made of:
//!
//! - a plane, moved along its normal;
//! - a surface of revolution whose meridian — the profile turned around the
//!   axis — is a straight line (a cylinder, a cone, a flat ring) or a
//!   circular arc (a sphere, a torus): its offset turns the meridian's
//!   offset, which is again a line or a concentric arc.
//!
//! Everything else is refused rather than approximated.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::NurbSurface3D;
use crate::{
    nurb_curve::{NurbCurve, dehomogenize},
    shape::Axis,
};

/// Whether the surface's normal points along a direction, from `along`,
/// the two dotted: it must be definitely positive or definitely negative.
fn points_along<S: Scalar>(along: S) -> GeopResult<bool> {
    if along.definitely_greater(S::ZERO) {
        Ok(true)
    } else if along.definitely_less(S::ZERO) {
        Ok(false)
    } else {
        Err(GeopError::new(format!(
            "NurbSurface::offset: cannot tell which way the normal points: {along:?}"
        )))
    }
}

/// `distance` if `along`, else its opposite.
fn signed<S: Scalar>(distance: S, along: bool) -> S {
    if along { distance } else { distance.neg() }
}

/// The point `p` with weight `w`, homogeneous — taken as it is where the
/// weight is exactly one: multiplying by it would only widen it by
/// rounding.
fn homogeneous<S: Scalar>(p: [S; 3], w: S) -> Vector4<S> {
    if w.is_sharp() && w.could_be_equal(S::ONE) {
        Vector4::from_array([p[0], p[1], p[2], w])
    } else {
        Vector4::from_array([p[0].mul(w), p[1].mul(w), p[2].mul(w), w])
    }
}

impl<S: Scalar> NurbSurface3D<S> {
    /// The surface `distance` along its normal — against it for a negative
    /// distance — with the same parametrization: the offset of the point at
    /// `(u, v)` is at `(u, v)` of the result, so a trim loop's `(u, v)` stay
    /// meaningful on it.
    ///
    /// Exact for a plane and for a surface of revolution whose meridian is a
    /// line or a circular arc (see the module docs). Fails for any other
    /// surface, and where the offset would pass through itself: a cylinder
    /// shrunk past its axis, a sphere past its center.
    pub fn offset(&self, distance: S) -> GeopResult<Self> {
        if let Some(plane) = self.as_plane()? {
            return Ok(self.translate(plane.normal.prod_scalar(distance)));
        }
        for along_u in [true, false] {
            if let Some(axis) = self.revolution_along(along_u)? {
                return self.offset_revolution(&axis, along_u, distance);
            }
        }
        Err(GeopError::new(
            "NurbSurface::offset: only planes and surfaces of revolution with a straight or circular meridian have an exact offset",
        ))
    }

    /// [`NurbSurface3D::offset`] of a surface of revolution around `axis`
    /// whose control rows along `u` (`along_u`) or along `v` are arcs.
    ///
    /// Such a net is the meridian's control points `(h_j, r_j)` — height
    /// along the axis, distance from it — turned: the `i`-th point of row
    /// `j` is `o + h_j a + r_j D_i`, with `D_i` the `i`-th control point of
    /// the unit circle the rows share (corner points of it lie further out
    /// than 1), and its weight the meridian's times the circle's. That map
    /// is affine in `(h, r)` for each `i`, so turning the offset meridian's
    /// control points gives the offset surface. The net is checked to be
    /// exactly that, point for point, before anything is built from it.
    fn offset_revolution(&self, axis: &Axis<S>, along_u: bool, distance: S) -> GeopResult<Self> {
        let (rows, len) = if along_u {
            (self.num_v, self.num_u)
        } else {
            (self.num_u, self.num_v)
        };
        let index = |i: usize, j: usize| {
            if along_u {
                i * self.num_v + j
            } else {
                j * self.num_v + i
            }
        };
        let (o, a) = (axis.point, axis.direction);
        // A point's height along the axis, and the way from the axis to it.
        let meridian = |p: &Vector3<S>| {
            let d = p.sub(&o);
            let h = d.prod_dot(&a);
            (h, d.sub(&a.prod_scalar(h)))
        };
        let points = dehomogenize::<S, 4, 3>(&self.control_points);

        let mut h = Vec::with_capacity(rows);
        let mut r = Vec::with_capacity(rows);
        let mut circle: Option<Vec<Vector3<S>>> = None;
        for j in 0..rows {
            // A row's first control point is on its circle: the row is a
            // clamped arc.
            let (hj, radial) = meridian(&points[index(0, j)]);
            let r2 = radial.norm_sq();
            let rj = if r2.could_be_equal(S::ZERO) {
                S::ZERO
            } else {
                let rj = r2.sqrt()?;
                if circle.is_none() {
                    let inv = S::ONE.div(rj)?;
                    circle = Some(
                        (0..len)
                            .map(|i| meridian(&points[index(i, j)]).1.prod_scalar(inv))
                            .collect(),
                    );
                }
                rj
            };
            h.push(hj);
            r.push(rj);
        }
        let circle = circle.ok_or_else(|| {
            GeopError::new("NurbSurface::offset: every row of the surface is a pole")
        })?;
        let turned =
            |hj: S, rj: S, i: usize| o.add(&a.prod_scalar(hj)).add(&circle[i].prod_scalar(rj));
        for j in 0..rows {
            for i in 0..len {
                if !turned(h[j], r[j], i).could_be_equal(&points[index(i, j)]) {
                    return Err(GeopError::new(format!(
                        "NurbSurface::offset: control point ({i}, {j}) of the surface of revolution is not its meridian turned: {:?}",
                        points[index(i, j)]
                    )));
                }
            }
        }

        // The meridian, in the plane of `(h, r)`, weighted as the rows are.
        let weight = |j: usize| self.control_points[index(0, j)][3];
        let (degree, knots) = if along_u {
            (self.degree_v, self.knot_vector_v.clone())
        } else {
            (self.degree_u, self.knot_vector_u.clone())
        };
        let profile = NurbCurve::try_new(
            degree,
            (0..rows)
                .map(|j| homogeneous([h[j], r[j], S::ZERO], weight(j)))
                .collect(),
            knots,
        )?;

        // Which way the surface's own normal points, in the meridian plane
        // through the middle of the patch.
        let ((u0, u1), (v0, v1)) = (self.domain_u(), self.domain_v());
        let (um, vm) = (u0.add(u1).div(S::TWO)?, v0.add(v1).div(S::TWO)?);
        let x = self.evaluate(um, vm)?;
        let normal = self.normal(um, vm)?;
        let (hx, radial) = meridian(&x);
        let rx = radial.norm();
        let er = radial.normalize()?;
        let in_space = |dh: S, dr: S| a.prod_scalar(dh).add(&er.prod_scalar(dr));

        let offset: Vec<(S, S)> = if let Some(line) = profile.as_line()? {
            // The meridian's normal, turned to point the way the surface's
            // does: every control point moves `distance` along it.
            let (nh, nr) = (line.direction[1].neg(), line.direction[0]);
            let d = signed(distance, points_along(normal.prod_dot(&in_space(nh, nr)))?);
            let (dh, dr) = (nh.mul(d), nr.mul(d));
            (0..rows).map(|j| (h[j].add(dh), r[j].add(dr))).collect()
        } else if let Some(arc) = profile.as_arc()? {
            // The concentric arc: every control point scaled about the
            // center, by more where the normal points away from it.
            let (hc, rc) = (arc.circle.center[0], arc.circle.center[1]);
            let radius = arc.circle.radius;
            let outward = in_space(hx.sub(hc), rx.sub(rc));
            let scaled = radius.add(signed(distance, points_along(normal.prod_dot(&outward))?));
            if !scaled.definitely_greater(S::ZERO) {
                return Err(GeopError::new(format!(
                    "NurbSurface::offset: offsetting by {distance:?} shrinks the meridian's arc of radius {radius:?} past its center"
                )));
            }
            let k = scaled.div(radius)?;
            (0..rows)
                .map(|j| (hc.add(h[j].sub(hc).mul(k)), rc.add(r[j].sub(rc).mul(k))))
                .collect()
        } else {
            return Err(GeopError::new(
                "NurbSurface::offset: the surface of revolution's meridian is neither a line nor a circular arc",
            ));
        };
        if offset.iter().any(|(_, rj)| rj.definitely_less(S::ZERO)) {
            return Err(GeopError::new(format!(
                "NurbSurface::offset: offsetting by {distance:?} takes the surface of revolution across its axis"
            )));
        }

        let mut control_points = self.control_points.clone();
        for (j, &(hj, rj)) in offset.iter().enumerate() {
            for i in 0..len {
                let w = self.control_points[index(i, j)][3];
                control_points[index(i, j)] = homogeneous(turned(hj, rj, i).to_array(), w);
            }
        }
        Self::try_new(
            self.degree_u,
            self.degree_v,
            control_points,
            self.knot_vector_u.clone(),
            self.knot_vector_v.clone(),
        )
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector4};

    use super::*;

    const R2: f64 = std::f64::consts::FRAC_1_SQRT_2;

    fn h<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
        Vector4::from_array([x * w, y * w, z * w, w].map(S::from_f64))
    }

    fn knots<S: Scalar>(ks: &[f64]) -> Vec<S> {
        ks.iter().map(|&k| S::from_f64(k)).collect()
    }

    /// Whether `offset` is `surface` moved `distance` along its normal, at
    /// a grid of samples.
    fn assert_offset<S: Scalar>(
        surface: &NurbSurface3D<S>,
        offset: &NurbSurface3D<S>,
        distance: f64,
    ) {
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        for a in 0..=4 {
            for b in 0..=4 {
                let u = S::interpolate(u0, u1, S::from_f64(a as f64 / 4.0));
                let v = S::interpolate(v0, v1, S::from_f64(b as f64 / 4.0));
                let expected = surface.evaluate(u, v).unwrap().add(
                    &surface
                        .normal(u, v)
                        .unwrap()
                        .prod_scalar(S::from_f64(distance)),
                );
                let got = offset.evaluate(u, v).unwrap();
                assert!(
                    got.could_be_equal(&expected),
                    "at ({a}, {b}): {got:?}, expected {expected:?}"
                );
            }
        }
    }

    fn check_plane<S: Scalar>() {
        let flat = NurbSurface3D::<S>::try_new(
            1,
            1,
            vec![
                h(0., 0., 1., 1.),
                h(0., 2., 1.5, 1.),
                h(1., 0., 1., 1.),
                h(1., 2., 1.5, 1.),
            ],
            knots(&[0., 0., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        assert_offset(&flat, &flat.offset(S::from_f64(0.3)).unwrap(), 0.3);
        assert_offset(&flat, &flat.offset(S::from_f64(-0.3)).unwrap(), -0.3);
    }
    #[test]
    fn planes_move_along_their_normal() {
        for_all_scalars!(check_plane);
    }

    /// A quarter of a cylinder of radius 2 around the z axis, `u` around it
    /// and `v` along it, 3 high; its normal points away from the axis.
    fn quarter_cylinder<S: Scalar>() -> NurbSurface3D<S> {
        let ring = [(2., 0., 1.), (2., 2., R2), (0., 2., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, w)| [h(x, y, 0., w), h(x, y, 3., w)])
            .collect();
        NurbSurface3D::try_new(
            2,
            1,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap()
    }

    fn check_cylinder<S: Scalar>() {
        let cylinder = quarter_cylinder::<S>();
        for d in [0.5, -0.5, -1.5] {
            assert_offset(&cylinder, &cylinder.offset(S::from_f64(d)).unwrap(), d);
        }
        assert!(cylinder.offset(S::from_f64(-2.5)).is_err());
    }
    #[test]
    fn cylinders_change_radius() {
        for_all_scalars!(check_cylinder);
    }

    /// A quarter of a cone around the z axis, from radius 2 at z = 0 to
    /// radius 1 at z = 1.
    fn check_cone<S: Scalar>() {
        let ring = [(1., 0., 1.), (1., 1., R2), (0., 1., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, w)| [h(2. * x, 2. * y, 0., w), h(x, y, 1., w)])
            .collect();
        let cone = NurbSurface3D::<S>::try_new(
            2,
            1,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        assert_offset(&cone, &cone.offset(S::from_f64(-0.25)).unwrap(), -0.25);
    }
    #[test]
    fn cones_move_along_their_normal() {
        for_all_scalars!(check_cone);
    }

    /// An eighth of the sphere of radius 2 around the origin: `u` around
    /// the z axis, `v` from the equator up to the pole.
    fn check_sphere<S: Scalar>() {
        let meridian = [(2., 0., 1.), (2., 2., R2), (0., 2., 1.)];
        let ring = [(1., 0., 1.), (1., 1., R2), (0., 1., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, wr)| {
                meridian
                    .iter()
                    .map(move |&(r, z, wm)| h(r * x, r * y, z, wr * wm))
            })
            .collect();
        let sphere = NurbSurface3D::<S>::try_new(
            2,
            2,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        // Its natural normal: whichever way, the offset follows it.
        assert_offset(&sphere, &sphere.offset(S::from_f64(0.5)).unwrap(), 0.5);
        assert_offset(&sphere, &sphere.offset(S::from_f64(-0.5)).unwrap(), -0.5);
    }
    #[test]
    fn spheres_change_radius() {
        for_all_scalars!(check_sphere);
    }

    /// A quarter of a torus around the z axis, tube radius 1 around the
    /// circle of radius 3, the outer half of the tube.
    fn check_torus<S: Scalar>() {
        let tube = [(4., 0., 1.), (4., 1., R2), (3., 1., 1.)];
        let ring = [(1., 0., 1.), (1., 1., R2), (0., 1., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, wr)| {
                tube.iter()
                    .map(move |&(r, z, wt)| h(r * x, r * y, z, wr * wt))
            })
            .collect();
        let torus = NurbSurface3D::<S>::try_new(
            2,
            2,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        assert_offset(&torus, &torus.offset(S::from_f64(0.25)).unwrap(), 0.25);
        assert_offset(&torus, &torus.offset(S::from_f64(-0.25)).unwrap(), -0.25);
    }
    #[test]
    fn tori_change_tube_radius() {
        for_all_scalars!(check_torus);
    }

    fn check_freeform<S: Scalar>() {
        let mut bent = quarter_cylinder::<S>();
        bent.control_points[2] = h(2.5, 2., 0., R2);
        assert!(bent.offset(S::from_f64(0.1)).is_err());
    }
    #[test]
    fn freeform_surfaces_are_refused() {
        for_all_scalars!(check_freeform);
    }
}
