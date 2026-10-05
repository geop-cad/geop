//! Mass properties of solids, and areas of faces and solids, integrated over
//! the exact trimmed NURBS faces: [`Model::mass_properties`],
//! [`Model::face_area`], [`Model::solid_area`].
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
//! `u` domain, which the untrimmed surface covers. (Or the same with the
//! roles swapped: `∬_D g du dv = -∮_∂D H du`, `H` integrating along `v`.)
//! So every face is integrated along its own pcurves, each piece of a
//! pcurve by adaptive Gauss–Kronrod, and every point of those by an inner
//! integral split at the surface's knots (see [`geop_core_math::quadrature`]).
//!
//! # The inner integral is exact where it can be
//!
//! The inner integral is evaluated at every point of the outer one, so it
//! decides the cost. It runs along a direction in which the surface is a
//! polynomial where there is one: where the weights do not change along
//! it — any plane, an extrusion along its straight direction, a cylinder
//! along its axis. For a fixed other parameter the point is then a
//! polynomial spline of the surface's degree `p` there, `S_u × S_v` one of
//! degree `2p - 1`, and the moments, at most cubic in the point, of degree
//! `5p - 1`: the Gauss–Legendre rule of `⌈5p / 2⌉` points integrates every
//! span exactly (three points on a plane, against the fifteen of a
//! Gauss–Kronrod panel), and the inner integral has no truncation error at
//! all. The weights must be the same numbers, not merely overlapping
//! enclosures: the same interval along every row, which is what a sweep or
//! a plane is built with. Elsewhere — a sphere, a torus, an area's square
//! root — it is adaptive Gauss–Kronrod.
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

use geop_core_geometry::nurb_surface::NurbSurface3D;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    matrix::{Matrix, symmetric_eigen3},
    primitives::Pose,
    quadrature::{Integral, MAX_POLYNOMIAL_DEGREE, Quadrature, integrate, integrate_polynomial},
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

/// The mass properties of a body: its volume, its mass, where its centre of
/// mass is and its inertia tensor about it. (The area of its boundary is no
/// mass property, and costs more to integrate: see [`Model::solid_area`].)
///
/// Every value is an enclosure in the sense of [`geop_core_math::quadrature`]:
/// proven as far as interval arithmetic goes, widened by the quadrature's
/// estimate of its truncation error — `converged` says whether that
/// estimate came within the tolerance asked.
#[derive(Clone, Copy, Debug)]
pub struct MassProperties<S: Scalar> {
    pub volume: S,
    /// The volume times the density: the volume itself, until a density
    /// is given (see [`MassProperties::with_density`]).
    pub mass: S,
    pub center: Vector3<S>,
    /// The inertia tensor about `center`, along the world's axes:
    /// `I_ij = ∫ ρ (|r|² δ_ij - r_i r_j) dV`, `r` measured from `center` —
    /// with `ρ = 1` until a density is given.
    pub inertia: [[S; 3]; 3],
    pub converged: bool,
    /// How many surface points the integration took: the work it did.
    pub evaluations: usize,
}

/// The principal moments of inertia, ascending, each enclosed, and the axis
/// each is about (see [`symmetric_eigen3`]: where moments coincide, any axis
/// of their plane is one).
#[derive(Clone, Copy, Debug)]
pub struct PrincipalAxes<S: Scalar> {
    pub moments: [S; 3],
    pub axes: [Vector3<S>; 3],
}

/// The components a solid's integrand has, in this order: volume, the
/// three first moments `∫ x`, `∫ y`, `∫ z`, the three `∫ x²`, `∫ y²`, `∫ z²`
/// and the three `∫ x y`, `∫ y z`, `∫ z x` — every coordinate measured from
/// a reference point near the solid, so that the integrands do not cancel.
const SOLID_COMPONENTS: usize = 10;

/// A function of the point `S(u, v)` and of `S_u × S_v`, of several
/// components.
type PointAndNormal<'a, S> = dyn Fn(&Vector3<S>, &Vector3<S>) -> GeopResult<Vec<S>> + 'a;

/// What is integrated over a face: `g`, of `components` components.
#[derive(Clone, Copy)]
struct FaceIntegrand<'a, S: Scalar> {
    components: usize,
    g: &'a PointAndNormal<'a, S>,
    /// The degree of `g` along a direction in which the surface is a
    /// polynomial of degree `p`, as a function of `p` — `None` where it is
    /// no polynomial there (see the module docs).
    degree: Option<fn(usize) -> usize>,
}

impl<S: Scalar> Model<S> {
    /// The area of the face `face`, and whether its quadrature converged.
    pub fn face_area(&self, face: FaceId) -> GeopResult<(S, bool)> {
        let ctx = |e: GeopError| e.with_context(format!("Model::face_area(face={face})"));
        let area = FaceIntegrand {
            components: 1,
            g: &|_, n| Ok(vec![n.norm()]),
            degree: None,
        };
        let integral = self.integrate_over_face(face, area).with_context(&ctx)?;
        Ok((integral.value[0], integral.converged))
    }

    /// The area of the boundary of `solid`: the sum of its faces' areas,
    /// and whether every face's quadrature converged.
    pub fn solid_area(&self, solid: SolidId) -> GeopResult<(S, bool)> {
        let ctx = |e: GeopError| e.with_context(format!("Model::solid_area(solid={solid})"));
        let mut total = (S::ZERO, true);
        for face in self.solid_faces(solid).with_context(&ctx)? {
            let (area, converged) = self.face_area(face).with_context(&ctx)?;
            total = (total.0.add(area), total.1 && converged);
        }
        Ok(total)
    }

    /// The mass properties of `solid`, of density one: what its shape
    /// alone decides, and a material scales (see
    /// [`MassProperties::with_density`]) — so they can be kept for a
    /// shape, whatever it is made of.
    ///
    /// Fails for a solid whose faces do not enclose a volume definitely
    /// greater than zero — its faces pointing into the material, or its
    /// volume not resolved by the quadrature — naming it.
    pub fn mass_properties(&self, solid: SolidId) -> GeopResult<MassProperties<S>> {
        let ctx = |e: GeopError| e.with_context(format!("Model::mass_properties(solid={solid})"));
        let faces = self.solid_faces(solid).with_context(&ctx)?;
        let reference = self.reference_point(&faces).with_context(&ctx)?;
        let mut total = [S::ZERO; SOLID_COMPONENTS];
        let mut converged = true;
        let mut evaluations = 0;
        let g = |p: &Vector3<S>, n: &Vector3<S>| {
            let r = p.sub(&reference);
            let (x, y, z) = (r[0], r[1], r[2]);
            let half = |a: S| a.div(S::TWO);
            let third = |a: S| a.div(S::from_f64(3.0));
            Ok(vec![
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
        };
        let moments = FaceIntegrand {
            components: SOLID_COMPONENTS,
            g: &g,
            // At most cubic in the point, of degree `p`, times `S_u × S_v`,
            // of degree `2p - 1`.
            degree: Some(|p| 5 * p - 1),
        };
        for &face in &faces {
            let integral = self.integrate_over_face(face, moments).with_context(&ctx)?;
            converged &= integral.converged;
            evaluations += integral.evaluations;
            for (t, v) in total.iter_mut().zip(integral.value) {
                *t = t.add(v);
            }
        }
        let [volume, hx, hy, hz, jxx, jyy, jzz, jxy, jyz, jzx] = total[..] else {
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
                inertia[a][b] = delta.sub(jc[a][b]);
            }
        }
        Ok(MassProperties {
            volume,
            mass: volume,
            center: reference.add(&offset),
            inertia,
            converged,
            evaluations,
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

    /// `∬ g(S, S_u × S_v) du dv` over the trimmed region of `face` — by
    /// Green's theorem, along the face's pcurves, the inner integral along
    /// the direction it is exact in where there is one (see the module
    /// docs).
    fn integrate_over_face(
        &self,
        face_id: FaceId,
        integrand: FaceIntegrand<'_, S>,
    ) -> GeopResult<Integral<S>> {
        let FaceIntegrand {
            components,
            g,
            degree,
        } = integrand;
        let face = self.get_face(face_id)?;
        let surface = &face.surface;
        let (along_u, polynomial) = match (
            polynomial_along(surface, true),
            polynomial_along(surface, false),
        ) {
            (true, true) => (surface.degree_u <= surface.degree_v, true),
            (true, false) => (true, true),
            (false, true) => (false, true),
            (false, false) => (true, false),
        };
        let (breaks, p) = if along_u {
            (surface.breakpoints_u(), surface.degree_u)
        } else {
            (surface.breakpoints_v(), surface.degree_v)
        };
        let exact = degree
            .filter(|_| polynomial)
            .map(|degree| degree(p))
            .filter(|&degree| degree <= MAX_POLYNOMIAL_DEGREE);
        let start = breaks[0];
        let converged = Cell::new(true);
        let evaluations = Cell::new(0);
        // From the start of the domain to `a` along the inner direction, at
        // `b` along the other, split at every knot passed on the way.
        let inner = |a: S, b: S| -> GeopResult<Vec<S>> {
            // On the domain's own start — along a seam or a boundary of the
            // untrimmed surface there — the integral is over nothing.
            if a.is_sharp() && a.could_be_equal(start) {
                return Ok(vec![S::ZERO; components]);
            }
            let mut cuts = vec![start];
            cuts.extend(breaks[1..].iter().filter(|k| k.definitely_less(a)));
            cuts.push(a);
            let along = |s: S| -> GeopResult<Vec<S>> {
                let (u, v) = if along_u { (s, b) } else { (b, s) };
                let point = surface.evaluate(u, v)?;
                let (su, sv) = surface.derivatives(u, v)?;
                g(&point, &su.prod_cross(&sv))
            };
            let integral = match exact {
                Some(degree) => integrate_polynomial(along, &cuts, components, degree)?,
                None => integrate(along, &cuts, components, &INNER)?,
            };
            if !integral.converged {
                converged.set(false);
            }
            evaluations.set(evaluations.get() + integral.evaluations);
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
                    let tangent = pcurve.tangent(t)?;
                    // `∮ G dv`, or `-∮ H du`.
                    let (a, b, db) = if along_u {
                        (uv[0], uv[1], tangent[1])
                    } else {
                        (uv[1], uv[0], tangent[0].neg())
                    };
                    // Along an iso-line of the other direction nothing is
                    // added.
                    if db.is_sharp() && db.could_be_equal(S::ZERO) {
                        return Ok(vec![S::ZERO; components]);
                    }
                    Ok(inner(a, b)?
                        .into_iter()
                        .map(|value| value.mul(db))
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
            evaluations: evaluations.get(),
        })
    }
}

/// Whether `surface` is a polynomial along `u` (`along_u`) or `v`: whether
/// the weights of every row of control points along it are the same — the
/// same enclosure, not merely overlapping ones, so the same number — so
/// that for a fixed other parameter its denominator is constant.
fn polynomial_along<S: Scalar>(surface: &NurbSurface3D<S>, along_u: bool) -> bool {
    let (nu, nv) = (surface.num_u, surface.num_v);
    let weight = |i: usize, j: usize| surface.control_points[i * nv + j][3];
    let same = |a: S, b: S| a.is_subset_of(b) && b.is_subset_of(a);
    if along_u {
        (0..nv).all(|j| (1..nu).all(|i| same(weight(i, j), weight(0, j))))
    } else {
        (0..nu).all(|i| (1..nv).all(|j| same(weight(i, j), weight(i, 0))))
    }
}

impl<S: Scalar> MassProperties<S> {
    /// The body of density one (see [`Model::mass_properties`]) made of a
    /// uniform `density`: its mass and inertia that many times.
    pub fn with_density(&self, density: S) -> Self {
        Self {
            mass: self.volume.mul(density),
            inertia: self.inertia.map(|row| row.map(|i| i.mul(density))),
            ..*self
        }
    }

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
            mass,
            center,
            inertia,
            converged: parts.iter().all(|p| p.converged),
            evaluations: parts.iter().map(|p| p.evaluations).sum(),
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
        let mass = model.mass_properties(solid).unwrap().with_density(S::TWO);
        assert!(mass.converged);
        assert!(mass.volume.could_be_equal(S::ONE), "{:?}", mass.volume);
        let (area, converged) = model.solid_area(solid).unwrap();
        assert!(converged);
        assert!(area.could_be_equal(S::from_f64(6.0)), "{area:?}");
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
                assert!(
                    mass.inertia[k][j].could_be_equal(want),
                    "{:?}",
                    mass.inertia
                );
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
