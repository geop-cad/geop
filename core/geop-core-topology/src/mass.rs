//! Mass properties of solids, and areas of faces, integrated over the exact
//! trimmed NURBS faces: [`Model::mass_properties`], [`Model::face_area`].
//!
//! # How
//!
//! A volume integral is turned into a surface integral by the divergence
//! theorem — `∫ div F dV = ∮ F · n dA` — with `F` chosen per quantity:
//! `p / 3` for the volume, `(x²/2, 0, 0)` for `∫ x dV`, `(x³/3, 0, 0)` for
//! `∫ x² dV`, `(x² y / 2, 0, 0)` for `∫ x y dV`, and so on. Each face is a
//! surface `S(u, v)` over a trimmed region `D` of its parameter plane, with
//! `n dA = S_u × S_v du dv` pointing out of the solid. A double integral over
//! `D` is turned into a line integral around it by Green's theorem:
//!
//! ```text
//! ∬_D g du dv = ∮_∂D G dv,   G(u, v) = ∫_{u0}^{u} g(s, v) ds
//! ```
//!
//! with `∂D` the face's loops as the kernel orients them — the outer one
//! counter-clockwise, holes clockwise — and `u0` the start of the surface's
//! `u` domain, which the untrimmed surface covers. So every face is
//! integrated along its own pcurves, each piece of a pcurve by adaptive
//! Gauss–Kronrod, and every point of those by an inner Gauss–Kronrod along
//! `u`, split at the surface's knots (see [`geop_core_math::quadrature`]).
//!
//! # Why not a tessellation
//!
//! A triangle mesh of the faces gives exact integrals of the wrong shape:
//! its error is the chord error of the mesh, which nothing bounds without
//! curvature bounds the rasterizer does not compute, and it converges only
//! quadratically. Integrating the exact surfaces converges spectrally, and
//! what is uncertain about the result can be stated: the quadrature rule's
//! own value is enclosed in interval arithmetic, and its truncation error
//! is the rule's standard estimate (see [`geop_core_math::quadrature`] for
//! exactly what is proven and what is estimated). Every quantity here is
//! that enclosure, widened by that estimate.

use std::cell::Cell;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, symmetric_eigen3},
    primitives::Pose,
    quadrature::{Integral, Quadrature, integrate},
    scalars::Scalar,
    vector::Vector3,
};

use crate::{FaceId, Model, SolidId, boundary::BoundaryType};

/// How hard the outer integral, along a face's pcurves, tries.
const OUTER: Quadrature = Quadrature {
    relative_tolerance: 1e-10,
    max_panels: 64,
};

/// How hard each inner integral, along `u`, tries. Tighter than the outer
/// one: its error is integrated along the boundary.
const INNER: Quadrature = Quadrature {
    relative_tolerance: 1e-11,
    max_panels: 32,
};

/// The mass properties of a body: its volume, the area of its boundary,
/// its mass, where its centre of mass is and its inertia tensor about it.
///
/// Every value is an enclosure in the sense of [`geop_core_math::quadrature`]:
/// proven as far as interval arithmetic goes, widened by the quadrature's
/// estimate of its truncation error — `converged` says whether that
/// estimate came within the tolerance asked.
#[derive(Clone, Copy, Debug)]
pub struct MassProperties<S: Scalar> {
    pub volume: S,
    pub area: S,
    /// The volume times the density.
    pub mass: S,
    pub center: Vector3<S>,
    /// The inertia tensor about `center`, along the world's axes:
    /// `I_ij = ∫ ρ (|r|² δ_ij - r_i r_j) dV`, `r` measured from `center`.
    pub inertia: [[S; 3]; 3],
    pub converged: bool,
}

/// The principal moments of inertia, ascending, each enclosed, and the axis
/// each is about (see [`symmetric_eigen3`]: where moments coincide, any axis
/// of their plane is one).
#[derive(Clone, Copy, Debug)]
pub struct PrincipalAxes<S: Scalar> {
    pub moments: [S; 3],
    pub axes: [Vector3<S>; 3],
}

/// The components a solid's integrand has, in this order: area, volume, the
/// three first moments `∫ x`, `∫ y`, `∫ z`, the three `∫ x²`, `∫ y²`, `∫ z²`
/// and the three `∫ x y`, `∫ y z`, `∫ z x` — every coordinate measured from
/// a reference point near the solid, so that the integrands do not cancel.
const SOLID_COMPONENTS: usize = 11;

impl<S: Scalar> Model<S> {
    /// The area of the face `face`.
    pub fn face_area(&self, face: FaceId) -> GeopResult<(S, bool)> {
        let ctx = |e: GeopError| e.with_context(format!("Model::face_area(face={face})"));
        let integral = self
            .integrate_over_face(face, 1, &|_, n| Ok(vec![n.norm()]))
            .with_context(&ctx)?;
        Ok((integral.value[0], integral.converged))
    }

    /// The mass properties of `solid`, of uniform `density`.
    ///
    /// Fails for a solid whose faces do not enclose a volume definitely
    /// greater than zero — its faces pointing into the material, or its
    /// volume not resolved by the quadrature — naming it.
    pub fn mass_properties(&self, solid: SolidId, density: S) -> GeopResult<MassProperties<S>> {
        let ctx = |e: GeopError| e.with_context(format!("Model::mass_properties(solid={solid})"));
        let faces = self.solid_faces(solid).with_context(&ctx)?;
        let reference = self.reference_point(&faces).with_context(&ctx)?;
        let mut total = vec![S::ZERO; SOLID_COMPONENTS];
        let mut converged = true;
        for &face in &faces {
            let integral = self
                .integrate_over_face(face, SOLID_COMPONENTS, &|p, n| {
                    let r = p.sub(&reference);
                    let (x, y, z) = (r[0], r[1], r[2]);
                    let half = |a: S| a.div(S::TWO);
                    let third = |a: S| a.div(S::from_f64(3.0));
                    Ok(vec![
                        n.norm(),
                        third(r.prod_dot(n))?,
                        half(x.mul(x).mul(n[0]))?,
                        half(y.mul(y).mul(n[1]))?,
                        half(z.mul(z).mul(n[2]))?,
                        third(x.mul(x).mul(x).mul(n[0]))?,
                        third(y.mul(y).mul(y).mul(n[1]))?,
                        third(z.mul(z).mul(z).mul(n[2]))?,
                        half(x.mul(x).mul(y).mul(n[0]))?,
                        half(y.mul(y).mul(z).mul(n[1]))?,
                        half(z.mul(z).mul(x).mul(n[2]))?,
                    ])
                })
                .with_context(&ctx)?;
            converged &= integral.converged;
            for (t, v) in total.iter_mut().zip(integral.value) {
                *t = t.add(v);
            }
        }
        let [area, volume, hx, hy, hz, jxx, jyy, jzz, jxy, jyz, jzx] = total[..] else {
            unreachable!("{SOLID_COMPONENTS} components");
        };
        if !volume.definitely_greater(S::ZERO) {
            return Err(ctx(GeopError::new(format!(
                "its faces enclose a volume of {volume:?}, not one definitely greater than zero: \
                 they point into the material, or the solid is not closed"
            ))));
        }
        // About the centre: `J_c = J - h hᵀ / V`, then `I = tr(J_c) E - J_c`.
        let h = Vector3::from_array([hx, hy, hz]);
        let offset = h.prod_scalar(S::ONE.div(volume).with_context(&ctx)?);
        let j = [[jxx, jxy, jzx], [jxy, jyy, jyz], [jzx, jyz, jzz]];
        let mut jc = [[S::ZERO; 3]; 3];
        for a in 0..3 {
            for b in 0..3 {
                jc[a][b] = j[a][b].sub(h[a].mul(offset[b]));
            }
        }
        let trace = jc[0][0].add(jc[1][1]).add(jc[2][2]);
        let mut inertia = [[S::ZERO; 3]; 3];
        for a in 0..3 {
            for b in 0..3 {
                let delta = if a == b { trace } else { S::ZERO };
                inertia[a][b] = delta.sub(jc[a][b]).mul(density);
            }
        }
        Ok(MassProperties {
            volume,
            area,
            mass: volume.mul(density),
            center: reference.add(&offset),
            inertia,
            converged,
        })
    }

    /// A point near the faces, sharp — where the moments are measured from
    /// is a free choice — the mean of their surfaces' middles.
    fn reference_point(&self, faces: &[FaceId]) -> GeopResult<Vector3<S>> {
        let mut sum = Vector3::zero();
        for &face in faces {
            let surface = &self.get_face(face)?.surface;
            let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
            let middle = surface.evaluate(
                u0.add(u1).div(S::TWO)?.sharpen(),
                v0.add(v1).div(S::TWO)?.sharpen(),
            )?;
            sum = sum.add(&middle);
        }
        let count = S::from_i64(faces.len().max(1) as i64);
        Ok(sum.prod_scalar(S::ONE.div(count)?).sharpen())
    }

    /// `∬ g(S, S_u × S_v) du dv` over the trimmed region of `face`, for a
    /// `components`-valued `g` — by Green's theorem, along the face's
    /// pcurves (see the module docs).
    fn integrate_over_face(
        &self,
        face_id: FaceId,
        components: usize,
        g: &dyn Fn(&Vector3<S>, &Vector3<S>) -> GeopResult<Vec<S>>,
    ) -> GeopResult<Integral<S>> {
        let face = self.get_face(face_id)?;
        let surface = &face.surface;
        let u_breaks = surface.breakpoints_u();
        let u0 = u_breaks[0];
        let converged = Cell::new(true);
        // `G(u, v)`: from the start of the domain to `u`, split at every
        // knot passed on the way.
        let inner = |u: S, v: S| -> GeopResult<Vec<S>> {
            let mut breaks = vec![u0];
            breaks.extend(u_breaks[1..].iter().filter(|k| k.definitely_less(u)));
            breaks.push(u);
            let along = |s: S| -> GeopResult<Vec<S>> {
                let p = surface.evaluate(s, v)?;
                let (su, sv) = surface.derivatives(s, v)?;
                g(&p, &su.prod_cross(&sv))
            };
            let integral = integrate(along, &breaks, components, &INNER)?;
            if !integral.converged {
                converged.set(false);
            }
            Ok(integral.value)
        };
        let mut total = vec![S::ZERO; components];
        for boundary in face.boundaries() {
            let BoundaryType::Loop(anchor) = boundary else {
                // A bare vertex bounds nothing.
                continue;
            };
            for coedge_id in self.iterate_loop_coedges(anchor) {
                let pcurve = &self.get_coedge(coedge_id)?.pcurve;
                let around = |t: S| -> GeopResult<Vec<S>> {
                    let uv = pcurve.evaluate(t)?;
                    let dv = pcurve.tangent(t)?[1];
                    // Along an iso-`v` line nothing is added.
                    if dv.is_sharp() && dv.could_be_equal(S::ZERO) {
                        return Ok(vec![S::ZERO; components]);
                    }
                    Ok(inner(uv[0], uv[1])?
                        .into_iter()
                        .map(|value| value.mul(dv))
                        .collect())
                };
                let integral = integrate(around, &pcurve.breakpoints(), components, &OUTER)
                    .with_context(&|e: GeopError| {
                        e.with_context(format!("along coedge {coedge_id} of face {face_id}"))
                    })?;
                converged.set(converged.get() && integral.converged);
                for (t, v) in total.iter_mut().zip(integral.value) {
                    *t = t.add(v);
                }
            }
        }
        Ok(Integral {
            value: total,
            converged: converged.get(),
        })
    }
}

impl<S: Scalar> MassProperties<S> {
    /// The body moved by `pose`: its centre moved, its inertia turned with
    /// it (`R I Rᵀ`).
    pub fn placed(&self, pose: &Pose<S>) -> GeopResult<Self> {
        let motion = pose.motion();
        let axes = [0, 1, 2].map(|k| motion.rotate(&Vector3::axis(k)));
        // `R` has the turned axes as its columns.
        let r = Matrix::<S, 3, 3>::from_columns(axes);
        let i = Matrix::from_rows(self.inertia);
        let turned = r.mul_mat(&i).mul_mat(&r.transpose());
        let mut inertia = [[S::ZERO; 3]; 3];
        for (a, row) in inertia.iter_mut().enumerate() {
            for (b, value) in row.iter_mut().enumerate() {
                *value = turned[(a, b)];
            }
        }
        Ok(Self {
            center: motion.apply(&self.center),
            inertia,
            ..*self
        })
    }

    /// The bodies `parts` taken together: masses, volumes and areas added,
    /// the centre their mass-weighted mean, and each inertia moved to it by
    /// the parallel axis theorem. `None` for no parts.
    pub fn combine(parts: &[Self]) -> GeopResult<Option<Self>> {
        let Some(first) = parts.first() else {
            return Ok(None);
        };
        let sum = |f: &dyn Fn(&Self) -> S| parts.iter().skip(1).fold(f(first), |a, p| a.add(f(p)));
        let mass = sum(&|p| p.mass);
        let mut weighted = Vector3::zero();
        for p in parts {
            weighted = weighted.add(&p.center.prod_scalar(p.mass));
        }
        let center = weighted.prod_scalar(S::ONE.div(mass).map_err(|e| {
            e.with_context(format!(
                "MassProperties::combine: a total mass of {mass:?} is not definitely positive"
            ))
        })?);
        let mut inertia = [[S::ZERO; 3]; 3];
        for p in parts {
            let d = p.center.sub(&center);
            let d2 = d.norm_sq();
            for a in 0..3 {
                for b in 0..3 {
                    let delta = if a == b { d2 } else { S::ZERO };
                    let shift = delta.sub(d[a].mul(d[b])).mul(p.mass);
                    inertia[a][b] = inertia[a][b].add(p.inertia[a][b]).add(shift);
                }
            }
        }
        Ok(Some(Self {
            volume: sum(&|p| p.volume),
            area: sum(&|p| p.area),
            mass,
            center,
            inertia,
            converged: parts.iter().all(|p| p.converged),
        }))
    }

    /// The principal moments of inertia and their axes.
    pub fn principal(&self) -> GeopResult<PrincipalAxes<S>> {
        let (moments, axes) = symmetric_eigen3(&Matrix::from_rows(self.inertia))?;
        Ok(PrincipalAxes { moments, axes })
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{for_all_scalars, scalars::Scalar};

    use crate::{Model, test_fixtures::test_cube_solid};

    /// The unit cube: volume 1, area 6, centre `(0.5, 0.5, 0.5)`, inertia
    /// `1/6` about every axis through it, no products — each within its
    /// enclosure.
    fn check_unit_cube<S: Scalar>() {
        let mut model = Model::<S>::new();
        let solid = test_cube_solid(&mut model);
        let mass = model.mass_properties(solid, S::TWO).unwrap();
        assert!(mass.converged);
        assert!(mass.volume.could_be_equal(S::ONE), "{:?}", mass.volume);
        assert!(mass.area.could_be_equal(S::from_f64(6.0)), "{:?}", mass.area);
        assert!(mass.mass.could_be_equal(S::TWO));
        let half = S::from_f64(0.5);
        for k in 0..3 {
            assert!(mass.center[k].could_be_equal(half), "{:?}", mass.center);
            for j in 0..3 {
                let want = if j == k {
                    S::from_ratio(1, 3).unwrap()
                } else {
                    S::ZERO
                };
                assert!(mass.inertia[k][j].could_be_equal(want), "{:?}", mass.inertia);
            }
        }
        let (area, converged) = model
            .face_area(*model.faces.keys().next().unwrap())
            .unwrap();
        assert!(converged);
        assert!(area.could_be_equal(S::ONE));
    }
    #[test]
    fn unit_cube() {
        for_all_scalars!(check_unit_cube);
    }
}
