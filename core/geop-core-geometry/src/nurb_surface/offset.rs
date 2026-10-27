//! Offsetting a surface along its normal.
//!
//! The offset of a NURBS surface is in general not a NURBS surface at all.
//! Every surface's offset is fitted in its own basis, and widened to
//! enclose the true one by a bound proven over the whole domain (see
//! `offset_fit`): exact to the rounding where the offset is a NURBS of that
//! basis, as a plane's, a cylinder's, a cone's, a sphere's or a torus's
//! is, and no branch on what kind of surface it is.
//!
//! **An apex.** A surface whose row collapses to a point with no tangent
//! plane — a cone's tip — has no normal there, and next to it its offset
//! is no surface the patch could take: on one side the offset rows pass
//! through each other across the axis (a fold), on the other they leave a
//! hole round it. A shell or a moved face wants the offset cone, closed in
//! its own apex. So the surface is first cut back to where its offset's
//! row closes to a point, or carried on through its apex, the continued
//! patch's normal continued with it, to there; and parametrized over its
//! old domain again, so that the apex stays where it was in `(u, v)`.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use super::{NurbSurface3D, net::Side};

/// How a surface was moved before it was offset so that its offset closes
/// at an apex: the side its offset's row closes along, and where the apex
/// itself is across that side, if the surface was carried on through it
/// (otherwise it was cut back short of it).
pub(super) struct Closing {
    pub side: Side,
    pub apex: Option<Scalar>,
}

/// How many secant steps the parameter where an offset closes is sought
/// with: it converges to the last bit in a handful, and stops where the
/// rest could be zero or a step repeats.
const CLOSING_STEPS: usize = 64;

impl NurbSurface3D {
    /// The surface `distance` along its normal — against it for a negative
    /// distance — with the same parametrization: the offset of the point at
    /// `(u, v)` is at `(u, v)` of the result, so a trim loop's `(u, v)` stay
    /// meaningful on it. An apex is the exception (see the module docs):
    /// it goes to the offset's apex, at the same `(u, v)`, and along the
    /// lines from it the points move along those lines too.
    ///
    /// `None` where the offset collapses: every point of the surface passes
    /// its centre of curvature, as a cylinder shrunk past its axis, a
    /// sphere or a torus's tube past its centre, so the offset has no
    /// area — a blend narrower than the walls a shell builds are thick.
    /// Fails where only part of it would, where the offset folds, and at an
    /// apex whose offset does not close in a point.
    pub fn offset(&self, distance: Scalar) -> GeopResult<Option<Self>> {
        if distance.is_zero() {
            return Ok(Some(self.clone()));
        }
        let ctx =
            |e: GeopError| e.with_context(format!("NurbSurface::offset(distance={distance:?})"));
        match self.closed(distance).map_err(ctx)? {
            Some((surface, closing)) => surface.fitted_offset(distance, Some(&closing)),
            None => self.fitted_offset(distance, None).map_err(|e| {
                // How its rows turn at its sides, which said it closes
                // nowhere inside.
                let turning: Vec<_> = (Side::ALL.iter())
                    .map(|side| {
                        let ((lo, hi), (a, b)) = if side.u_fixed {
                            (self.domain_u(), self.domain_v())
                        } else {
                            (self.domain_v(), self.domain_u())
                        };
                        let x = if side.at_end { hi } else { lo };
                        let middle = a.add(b).div(Scalar::TWO)?.sharpen();
                        self.row_turning(side.u_fixed, middle, x, distance, false)
                    })
                    .collect();
                e.with_context(format!(
                    "closing nowhere inside over u={:?}, v={:?}: its rows turn by {turning:?} at its sides",
                    self.domain_u(),
                    self.domain_v()
                ))
            }),
        }
    }

    /// The offset by `distance` of the row through `x` across the side
    /// (`across_u`: `x` is a `u`), on the parameter line through `middle`:
    /// `O_t·S_t = S_t·S_t − d N·S_tt`, `t` along the row, the normal
    /// turned round if `flipped`. Positive where the offset's row runs the
    /// way the surface's does, zero where it closes to a point, negative
    /// where it has turned over.
    fn row_turning(
        &self,
        across_u: bool,
        middle: Scalar,
        x: Scalar,
        distance: Scalar,
        flipped: bool,
    ) -> GeopResult<Scalar> {
        let (u, v) = if across_u { (x, middle) } else { (middle, x) };
        let s = self.partials(u, v, 2)?;
        let (along, bend) = if across_u {
            (s[0][1], s[0][2])
        } else {
            (s[1][0], s[2][0])
        };
        let normal = self.normal(u, v)?;
        let normal = if flipped { normal.neg() } else { normal };
        Ok(along
            .prod_dot(&along)
            .sub(distance.mul(normal.prod_dot(&bend))))
    }

    /// This surface cut back to, or carried on through an apex to, where
    /// its offset by `distance` closes in a point, over its own domain
    /// again (see the module docs). `None` where it closes nowhere.
    fn closed(&self, distance: Scalar) -> GeopResult<Option<(Self, Closing)>> {
        let apexes: Vec<Side> = Side::ALL
            .into_iter()
            .filter(|s| self.is_apex(s.u_fixed, !s.at_end))
            .collect();
        let (side, cut, longer) = match apexes.as_slice() {
            [] => match self.closes_inside(distance)? {
                Some((side, cut)) => (side, cut, None),
                None => return Ok(None),
            },
            [side] => {
                let (cut, longer) = self.closes_at_apex(*side, distance)?;
                (*side, cut, longer)
            }
            _ => {
                return Err(GeopError::new(format!(
                    "the surface has apexes at {apexes:?}: offsetting more than one is not supported"
                )));
            }
        };
        let across_u = side.u_fixed;
        let ((lo, hi), (other_lo, other_hi)) = if across_u {
            (self.domain_u(), self.domain_v())
        } else {
            (self.domain_v(), self.domain_u())
        };
        let far = if side.at_end { lo } else { hi };
        let (surface, apex_inside) = match &longer {
            Some((longer, apex)) => (longer, Some(*apex)),
            None => (self, None),
        };
        let (c0, c1) = if side.at_end { (far, cut) } else { (cut, far) };
        let whole = (other_lo, other_hi);
        let piece = if across_u {
            surface.sub_surface((c0, c1), whole)?
        } else {
            surface.sub_surface(whole, (c0, c1))?
        };
        // Over the old domain again: `c0 ↦ lo`, `c1 ↦ hi`, affinely.
        let scale = hi.sub(lo).div(c1.sub(c0))?;
        let map = |k: Scalar| lo.add(k.sub(c0).mul(scale));
        let relabel = |knots: &[Scalar], degree: usize| -> Vec<Scalar> {
            let len = knots.len();
            knots
                .iter()
                .enumerate()
                .map(|(i, &k)| {
                    if i <= degree {
                        lo
                    } else if i >= len - degree - 1 {
                        hi
                    } else {
                        map(k)
                    }
                })
                .collect()
        };
        let (knots_u, knots_v) = if across_u {
            (
                relabel(&piece.knot_vector_u, piece.degree_u),
                piece.knot_vector_v.clone(),
            )
        } else {
            (
                piece.knot_vector_u.clone(),
                relabel(&piece.knot_vector_v, piece.degree_v),
            )
        };
        // The apex inside is where the carried-on piece was joined on, a
        // knot: the relabelled knot itself, so that the Greville points
        // there find it.
        let apex_inside =
            match apex_inside {
                Some(apex) => {
                    let (old, new) = if across_u {
                        (&piece.knot_vector_u, &knots_u)
                    } else {
                        (&piece.knot_vector_v, &knots_v)
                    };
                    let i = old.iter().position(|k| k.could_be_equal(apex)).ok_or_else(|| {
                    GeopError::new(format!(
                        "the apex at {apex:?} is no knot of the surface carried on through it"
                    ))
                })?;
                    Some(new[i])
                }
                None => None,
            };
        let closed = Self::try_new(
            piece.degree_u,
            piece.degree_v,
            piece.control_points,
            knots_u,
            knots_v,
        )?;
        Ok(Some((
            closed,
            Closing {
                side,
                apex: apex_inside,
            },
        )))
    }

    /// Where the offset by `distance` of this surface, which has no apex,
    /// closes inside its domain: the side whose rows have turned over, and
    /// the parameter across it where the row closes. Asked of the row's
    /// turning ([`Self::row_turning`]) at the two sides, on the parameter
    /// line through the middle — a free choice of line; that the whole row
    /// closes there is checked on the offset. `None` where it does not
    /// change sign definitely between them: the offset closes nowhere, or
    /// everywhere (it collapses), or folds, which the fit finds.
    fn closes_inside(&self, distance: Scalar) -> GeopResult<Option<(Side, Scalar)>> {
        let mut found: Option<(Side, Scalar)> = None;
        for across_u in [true, false] {
            let ((lo, hi), (other_lo, other_hi)) = if across_u {
                (self.domain_u(), self.domain_v())
            } else {
                (self.domain_v(), self.domain_u())
            };
            let middle = other_lo.add(other_hi).div(Scalar::TWO)?.sharpen();
            let turning = |x: Scalar| self.row_turning(across_u, middle, x, distance, false);
            let (at_lo, at_hi) = (turning(lo)?, turning(hi)?);
            let positive = |k: Scalar| k.definitely_greater(Scalar::ZERO);
            let negative = |k: Scalar| k.definitely_less(Scalar::ZERO);
            let at_end = if positive(at_lo) && negative(at_hi) {
                true
            } else if positive(at_hi) && negative(at_lo) {
                false
            } else {
                continue;
            };
            // A bracket from where the row runs on to where it has turned
            // over, narrowed by false position (Illinois): where to split
            // it is a free choice.
            let (mut good, mut bad, mut k_good, mut k_bad) = if at_end {
                (lo, hi, at_lo, at_hi)
            } else {
                (hi, lo, at_hi, at_lo)
            };
            let mut kept: Option<bool> = None;
            for _ in 0..CLOSING_STEPS {
                let step = k_good.mul(bad.sub(good)).div(k_good.sub(k_bad))?;
                let mut next = good.add(step).sharpen();
                let between = |x: Scalar| {
                    let (a, b) = if good.definitely_less(bad) {
                        (good, bad)
                    } else {
                        (bad, good)
                    };
                    x.definitely_greater(a) && x.definitely_less(b)
                };
                if !between(next) {
                    next = good.add(bad).div(Scalar::TWO)?.sharpen();
                    if !between(next) {
                        break;
                    }
                }
                let k = turning(next)?;
                if k.could_be_equal(Scalar::ZERO) {
                    good = next;
                    break;
                }
                if positive(k) {
                    (good, k_good) = (next, k);
                    if kept == Some(false) {
                        k_bad = k_bad.div(Scalar::TWO)?;
                    }
                    kept = Some(false);
                } else {
                    (bad, k_bad) = (next, k);
                    if kept == Some(true) {
                        k_good = k_good.div(Scalar::TWO)?;
                    }
                    kept = Some(true);
                }
            }
            // The row kept is the last one known to run on: the offset's
            // row there is a point to the bracket's width, which the fit
            // encloses and checks.
            let side = Side {
                u_fixed: across_u,
                at_end,
            };
            if let Some((first, _)) = found {
                return Err(GeopError::new(format!(
                    "the offset by {distance:?} closes inside the surface along {first:?} and along {side:?}: not supported"
                )));
            }
            found = Some((side, good));
        }
        Ok(found)
    }

    /// Where the offset by `distance` of this surface, with an apex along
    /// `side`, closes: cut back short of the apex, or — the second value —
    /// carried on through it, the surface carried on and the apex's
    /// parameter. `S_t·S_t − d N·S_tt` vanishes at the apex too, once:
    /// divided by the distance from it, what is left runs nearly straight
    /// through the root, which secant steps find.
    fn closes_at_apex(
        &self,
        side: Side,
        distance: Scalar,
    ) -> GeopResult<(Scalar, Option<(Self, Scalar)>)> {
        let across_u = side.u_fixed;
        let ((lo, hi), (other_lo, other_hi), breaks) = if across_u {
            (self.domain_u(), self.domain_v(), self.breakpoints_u())
        } else {
            (self.domain_v(), self.domain_u(), self.breakpoints_v())
        };
        let (apex, far) = if side.at_end { (hi, lo) } else { (lo, hi) };
        // Room through the apex: as far as the span next to it is long, a
        // free choice.
        let n = breaks.len();
        let room = if side.at_end {
            breaks[n - 1].sub(breaks[n - 2])
        } else {
            breaks[1].sub(breaks[0])
        }
        .sharpen();
        let longer = self.extended(across_u, side.at_end, room)?;
        let middle = other_lo.add(other_hi).div(Scalar::TWO)?.sharpen();
        let closes = |x: Scalar| -> GeopResult<Scalar> {
            // Past the apex the carried-on patch faces the other way.
            let past = if side.at_end {
                x.definitely_greater(apex)
            } else {
                x.definitely_less(apex)
            };
            longer
                .row_turning(across_u, middle, x, distance, past)?
                .div(x.sub(apex))
        };
        let inward = if side.at_end { room.neg() } else { room };
        let mut a = apex.add(inward.div(Scalar::from_i64(4))?).sharpen();
        let mut b = apex.add(inward.div(Scalar::TWO)?).sharpen();
        let (mut ka, mut kb) = (closes(a)?, closes(b)?);
        let (reach_lo, reach_hi) = if side.at_end {
            (far, apex.add(room))
        } else {
            (apex.sub(room), far)
        };
        for _ in 0..CLOSING_STEPS {
            if kb.could_be_equal(Scalar::ZERO) {
                break;
            }
            let slope = kb.sub(ka).div(b.sub(a))?;
            let next = b.sub(kb.div(slope)?).sharpen();
            if next.could_be_equal(b) {
                break;
            }
            if !next.definitely_greater(reach_lo) || !next.definitely_less(reach_hi) {
                return Err(GeopError::new(format!(
                    "the offset by {distance:?} closes past the apex at {apex:?} farther than {room:?}, or not before the far side {far:?}: at {next:?}"
                )));
            }
            if next.could_be_equal(apex) {
                return Err(GeopError::new(format!(
                    "the offset by {distance:?} closes at the apex itself, {apex:?}"
                )));
            }
            (a, ka) = (b, kb);
            b = next;
            kb = closes(b)?;
        }
        // Cut back short of the apex, or carried on through it.
        let inside = if side.at_end {
            b.definitely_less(apex)
        } else {
            b.definitely_greater(apex)
        };
        Ok((b, (!inside).then_some((longer, apex))))
    }
}

#[cfg(test)]
mod tests {
    use crate::test_fixtures::knots;
    use geop_core_math::{
        scalars::Scalar,
        vector::{Vector3, Vector4},
    };

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

    /// Whether `offset` is `surface`'s offset by `distance` (see
    /// [`assert_offset`]) on `surface`'s own net, as narrow as the rounding
    /// of the polynomials that bound it (measured: up to `1.2e-13`).
    fn assert_exact(surface: &NurbSurface3D, offset: &NurbSurface3D, distance: f64) {
        assert_offset(surface, offset, distance);
        assert_eq!(
            (offset.degree_u, offset.degree_v, offset.num_u, offset.num_v),
            (
                surface.degree_u,
                surface.degree_v,
                surface.num_u,
                surface.num_v
            )
        );
        for cp in &offset.control_points {
            for k in 0..4 {
                assert!(cp[k].width().to_f64() < 2e-13, "{cp:?}");
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
        assert_exact(
            &flat,
            &flat.offset(Scalar::from_f64(0.3)).unwrap().unwrap(),
            0.3,
        );
        assert_exact(
            &flat,
            &flat.offset(Scalar::from_f64(-0.3)).unwrap().unwrap(),
            -0.3,
        );
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
            assert_exact(
                &cylinder,
                &cylinder.offset(Scalar::from_f64(d)).unwrap().unwrap(),
                d,
            );
        }
        assert!(cylinder.offset(Scalar::from_f64(-2.5)).unwrap().is_none());
        // Offset by exactly its radius, a cylinder is its axis: a factor
        // `1 − d κ` is zero all over, which no box tells from a fold, and
        // nowhere is the offset a surface either.
        assert!(cylinder.offset(Scalar::from_f64(-2.0)).unwrap().is_none());
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
        assert_exact(
            &cone,
            &cone.offset(Scalar::from_f64(-0.25)).unwrap().unwrap(),
            -0.25,
        );
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
        assert_exact(
            &sphere,
            &sphere.offset(Scalar::from_f64(0.5)).unwrap().unwrap(),
            0.5,
        );
        assert_exact(
            &sphere,
            &sphere.offset(Scalar::from_f64(-0.5)).unwrap().unwrap(),
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
        assert_exact(
            &torus,
            &torus.offset(Scalar::from_f64(0.25)).unwrap().unwrap(),
            0.25,
        );
        assert_exact(
            &torus,
            &torus.offset(Scalar::from_f64(-0.25)).unwrap().unwrap(),
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
            let offset = cone.offset(Scalar::from_f64(d)).unwrap().unwrap();
            let rim =
                |s: &NurbSurface3D, u: f64| s.evaluate(Scalar::from_f64(u), Scalar::ONE).unwrap();
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
                assert!(
                    offset.evaluate(u, v).unwrap().could_be_equal(&expected),
                    "{d}: {u:?}"
                );
            }
        }
    }

    /// A quarter of a cone cut off short of its tip, from radius 1 at
    /// z = 0 to radius 0.1 at z = 2 (`v` up it), offset 0.2 inward: the
    /// offset cone's apex, `0.2 / sin α` below the cone's at `z = 2 + 0.1 /
    /// 0.45`, lies inside the patch, and the offset is closed there, its
    /// `v = 1` side the apex; outward it is the plain offset.
    #[test]
    fn a_cone_cut_off_short_of_its_tip_closes_inside() {
        let ring = [(1., 0., 1.), (1., 1., R2), (0., 1., 1.)];
        let cps = ring
            .iter()
            .flat_map(|&(x, y, w)| [h(x, y, 0., w), h(0.1 * x, 0.1 * y, 2., w)])
            .collect();
        let cone = NurbSurface3D::try_new(
            2,
            1,
            cps,
            knots(&[0., 0., 0., 1., 1., 1.]),
            knots(&[0., 0., 1., 1.]),
        )
        .unwrap();
        let sin = 0.45 / (1f64 + 0.45 * 0.45).sqrt();
        let apex = Vector3::from_array([0., 0., 2. + 0.1 / 0.45 - 0.2 / sin].map(Scalar::from_f64));
        let mut closed = 0;
        for d in [0.2, -0.2] {
            let offset = cone.offset(Scalar::from_f64(d)).unwrap().unwrap();
            match offset.pole_at(Scalar::ZERO, Scalar::ONE) {
                Some(pole) => {
                    assert!(pole.could_be_equal(&apex), "{d}: {pole:?} vs {apex:?}");
                    closed += 1;
                }
                None => assert_exact(&cone, &offset, d),
            }
        }
        assert_eq!(closed, 1);
    }

    /// A quarter cylinder bent out of round: no revolution any more, so
    /// its offset is fitted, and encloses the true one.
    #[test]
    fn freeform_surfaces_are_fitted() {
        let mut bent = quarter_cylinder();
        bent.control_points[2] = h(2.5, 2., 0., R2);
        assert_offset(
            &bent,
            &bent.offset(Scalar::from_f64(0.1)).unwrap().unwrap(),
            0.1,
        );
    }
}
