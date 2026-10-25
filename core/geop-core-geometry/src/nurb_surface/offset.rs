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
    vector::Vector3,
};

use super::NurbSurface3D;
use crate::{
    nurb_curve::{NurbCurve, dehomogenize},
    shape::Axis,
};

/// Whether the surface's normal points along a direction, from `along`,
/// the two dotted: it must be definitely positive or definitely negative.
fn points_along(along: Scalar) -> GeopResult<bool> {
    if along.definitely_greater(Scalar::ZERO) {
        Ok(true)
    } else if along.definitely_less(Scalar::ZERO) {
        Ok(false)
    } else {
        Err(GeopError::new(format!(
            "NurbSurface::offset: cannot tell which way the normal points: {along:?}"
        )))
    }
}

/// `distance` if `along`, else its opposite.
fn signed(distance: Scalar, along: bool) -> Scalar {
    if along { distance } else { distance.neg() }
}

impl NurbSurface3D {
    /// The surface `distance` along its normal — against it for a negative
    /// distance — with the same parametrization: the offset of the point at
    /// `(u, v)` is at `(u, v)` of the result, so a trim loop's `(u, v)` stay
    /// meaningful on it. A cone's apex, which has no normal, goes to the
    /// offset cone's apex, so a pole stays a pole; along the meridians
    /// from it the points then move a little along the meridian too.
    ///
    /// Exact for a plane and for a surface of revolution whose meridian is a
    /// line or a circular arc (see the module docs). Fails for any other
    /// surface, and where the offset would pass through itself: a cylinder
    /// shrunk past its axis, a sphere past its center.
    pub fn offset(&self, distance: Scalar) -> GeopResult<Self> {
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
    fn offset_revolution(&self, axis: &Axis, along_u: bool, distance: Scalar) -> GeopResult<Self> {
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
        let meridian = |p: &Vector3| {
            let d = p.sub(&o);
            let h = d.prod_dot(&a);
            (h, d.sub(&a.prod_scalar(h)))
        };
        let points = dehomogenize::<4, 3>(&self.control_points);

        let mut h = Vec::with_capacity(rows);
        let mut r = Vec::with_capacity(rows);
        let mut circle: Option<Vec<Vector3>> = None;
        for j in 0..rows {
            // A row's first control point is on its circle: the row is a
            // clamped arc.
            let (hj, radial) = meridian(&points[index(0, j)]);
            let r2 = radial.norm_sq();
            let rj = if r2.could_be_equal(Scalar::ZERO) {
                Scalar::ZERO
            } else {
                let rj = r2.sqrt()?;
                if circle.is_none() {
                    let inv = Scalar::ONE.div(rj)?;
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
        let turned = |hj: Scalar, rj: Scalar, i: usize| {
            o.add(&a.prod_scalar(hj)).add(&circle[i].prod_scalar(rj))
        };
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
                .map(|j| Vector3::from_array([h[j], r[j], Scalar::ZERO]).homogeneous(weight(j)))
                .collect(),
            knots,
        )?;

        // Which way the surface's own normal points, in the meridian plane
        // through the middle of the patch.
        let ((u0, u1), (v0, v1)) = (self.domain_u(), self.domain_v());
        let (um, vm) = (u0.add(u1).div(Scalar::TWO)?, v0.add(v1).div(Scalar::TWO)?);
        let x = self.evaluate(um, vm)?;
        let normal = self.normal(um, vm)?;
        let (hx, radial) = meridian(&x);
        let rx = radial.norm();
        let er = radial.normalize()?;
        let in_space = |dh: Scalar, dr: Scalar| a.prod_scalar(dh).add(&er.prod_scalar(dr));

        let offset: Vec<(Scalar, Scalar)> = if let Some(line) = profile.as_line()? {
            // The meridian's normal, turned to point the way the surface's
            // does: every control point moves `distance` along it, but an
            // apex. An apex has no normal, and its offset is the offset
            // cone's apex: where the moved meridian meets the axis, `d / r'`
            // along it for the meridian's unit direction `(h', r')`. Moved
            // along the meridian's normal, it would leave the axis, and the
            // offset would have a hole round it or pass through itself. A
            // disc's pole moves along the axis anyway (`dr` is zero).
            let (nh, nr) = (line.direction[1].neg(), line.direction[0]);
            let d = signed(distance, points_along(normal.prod_dot(&in_space(nh, nr)))?);
            let (dh, dr) = (nh.mul(d), nr.mul(d));
            (0..rows)
                .map(|j| {
                    if r[j].could_be_equal(Scalar::ZERO) && !dr.could_be_equal(Scalar::ZERO) {
                        Ok((h[j].sub(d.div(line.direction[1])?), Scalar::ZERO))
                    } else {
                        Ok((h[j].add(dh), r[j].add(dr)))
                    }
                })
                .collect::<GeopResult<_>>()?
        } else if let Some(arc) = profile.as_arc()? {
            // The concentric arc: every control point scaled about the
            // center, by more where the normal points away from it.
            let (hc, rc) = (arc.circle.center[0], arc.circle.center[1]);
            let radius = arc.circle.radius;
            let outward = in_space(hx.sub(hc), rx.sub(rc));
            let scaled = radius.add(signed(distance, points_along(normal.prod_dot(&outward))?));
            if !scaled.definitely_greater(Scalar::ZERO) {
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
        if offset
            .iter()
            .any(|(_, rj)| rj.definitely_less(Scalar::ZERO))
        {
            return Err(GeopError::new(format!(
                "NurbSurface::offset: offsetting by {distance:?} takes the surface of revolution across its axis"
            )));
        }

        let mut control_points = self.control_points.clone();
        for (j, &(hj, rj)) in offset.iter().enumerate() {
            for i in 0..len {
                let w = self.control_points[index(i, j)][3];
                control_points[index(i, j)] = turned(hj, rj, i).homogeneous(w);
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
    use crate::test_fixtures::knots;
    use geop_core_math::{scalars::Scalar, vector::Vector4};

    use super::*;

    const R2: f64 = std::f64::consts::FRAC_1_SQRT_2;

    fn h(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
        Vector4::from_array([x * w, y * w, z * w, w].map(Scalar::from_f64))
    }

    /// Whether `offset` is `surface` moved `distance` along its normal, at
    /// a grid of samples.
    fn assert_offset(surface: &NurbSurface3D, offset: &NurbSurface3D, distance: f64) {
        let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
        for a in 0..=4 {
            for b in 0..=4 {
                let u = Scalar::interpolate(u0, u1, Scalar::from_f64(a as f64 / 4.0));
                let v = Scalar::interpolate(v0, v1, Scalar::from_f64(b as f64 / 4.0));
                let expected = surface.evaluate(u, v).unwrap().add(
                    &surface
                        .normal(u, v)
                        .unwrap()
                        .prod_scalar(Scalar::from_f64(distance)),
                );
                let got = offset.evaluate(u, v).unwrap();
                assert!(
                    got.could_be_equal(&expected),
                    "at ({a}, {b}): {got:?}, expected {expected:?}"
                );
            }
        }
    }

    #[test]
    fn planes_move_along_their_normal() {
        let flat = NurbSurface3D::try_new(
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
        assert_offset(&flat, &flat.offset(Scalar::from_f64(0.3)).unwrap(), 0.3);
        assert_offset(&flat, &flat.offset(Scalar::from_f64(-0.3)).unwrap(), -0.3);
    }

    /// A quarter of a cylinder of radius 2 around the z axis, `u` around it
    /// and `v` along it, 3 high; its normal points away from the axis.
    fn quarter_cylinder() -> NurbSurface3D {
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

    #[test]
    fn cylinders_change_radius() {
        let cylinder = quarter_cylinder();
        for d in [0.5, -0.5, -1.5] {
            assert_offset(&cylinder, &cylinder.offset(Scalar::from_f64(d)).unwrap(), d);
        }
        assert!(cylinder.offset(Scalar::from_f64(-2.5)).is_err());
    }

    /// A quarter of a cone around the z axis, from radius 2 at z = 0 to
    /// radius 1 at z = 1.
    #[test]
    fn cones_move_along_their_normal() {
        let ring = [(1., 0., 1.), (1., 1., R2), (0., 1., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, w)| [h(2. * x, 2. * y, 0., w), h(x, y, 1., w)])
            .collect();
        let cone = NurbSurface3D::try_new(
            2,
            1,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        assert_offset(&cone, &cone.offset(Scalar::from_f64(-0.25)).unwrap(), -0.25);
    }

    /// An eighth of the sphere of radius 2 around the origin: `u` around
    /// the z axis, `v` from the equator up to the pole.
    #[test]
    fn spheres_change_radius() {
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
        let sphere = NurbSurface3D::try_new(
            2,
            2,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        // Its natural normal: whichever way, the offset follows it.
        assert_offset(&sphere, &sphere.offset(Scalar::from_f64(0.5)).unwrap(), 0.5);
        assert_offset(
            &sphere,
            &sphere.offset(Scalar::from_f64(-0.5)).unwrap(),
            -0.5,
        );
    }

    /// A quarter of a torus around the z axis, tube radius 1 around the
    /// circle of radius 3, the outer half of the tube.
    #[test]
    fn tori_change_tube_radius() {
        let tube = [(4., 0., 1.), (4., 1., R2), (3., 1., 1.)];
        let ring = [(1., 0., 1.), (1., 1., R2), (0., 1., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, wr)| {
                tube.iter()
                    .map(move |&(r, z, wt)| h(r * x, r * y, z, wr * wt))
            })
            .collect();
        let torus = NurbSurface3D::try_new(
            2,
            2,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 0., 1., 1., 1.]),
        )
        .unwrap();
        assert_offset(&torus, &torus.offset(Scalar::from_f64(0.25)).unwrap(), 0.25);
        assert_offset(
            &torus,
            &torus.offset(Scalar::from_f64(-0.25)).unwrap(),
            -0.25,
        );
    }

    /// A quarter of the cone of half-angle 45° with its apex at z = 1 and
    /// radius 1 at z = 0 (`v` from the apex to the rim): the apex goes to
    /// the offset cone's apex, `√2` times the distance along the axis, and
    /// the rim moves along the normal.
    #[test]
    fn a_cones_apex_goes_to_the_offset_cones_apex() {
        let cone = crate::test_fixtures::revolved(1., 0.);
        for d in [0.25, -0.25] {
            let offset = cone.offset(Scalar::from_f64(d)).unwrap();
            let rim = |s: &NurbSurface3D, u: f64| s.evaluate(Scalar::from_f64(u), Scalar::ONE).unwrap();
            let shift = rim(&offset, 0.5).sub(&rim(&cone, 0.5));
            let pole = offset.pole_at(Scalar::ZERO, Scalar::ZERO).unwrap();
            // The pole moves the way the rim moves along the axis.
            let up = shift[2].definitely_greater(Scalar::ZERO);
            let z = 1. + if up { 0.25 } else { -0.25 } * std::f64::consts::SQRT_2;
            let expected = Vector3::from_array([0., 0., z].map(Scalar::from_f64));
            assert!(pole.could_be_equal(&expected), "{d}: {pole:?}");
            for u in [0., 0.5, 1.] {
                let (u, v) = (Scalar::from_f64(u), Scalar::ONE);
                let expected = cone
                    .evaluate(u, v)
                    .unwrap()
                    .add(&cone.normal(u, v).unwrap().prod_scalar(Scalar::from_f64(d)));
                assert!(offset.evaluate(u, v).unwrap().could_be_equal(&expected), "{d}: {u:?}");
            }
        }
    }

    #[test]
    fn freeform_surfaces_are_refused() {
        let mut bent = quarter_cylinder();
        bent.control_points[2] = h(2.5, 2., 0., R2);
        assert!(bent.offset(Scalar::from_f64(0.1)).is_err());
    }
}
