//! Test-only fixtures the curve and surface tests share: control points,
//! knot vectors, and a few small curves and surfaces.

use geop_core_math::{
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use crate::{nurb_curve::NurbCurve3D, nurb_surface::NurbSurface3D};

/// The homogeneous control point `(x, y, z, w)`, its coordinates taken as
/// given (already multiplied by the weight).
pub fn hom(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
    Vector4::from_array([x, y, z, w].map(Scalar::from_f64))
}

/// The point `(x, y, z)` with weight `w`, homogeneous: `(w x, w y, w z, w)`.
pub fn weighted(x: f64, y: f64, z: f64, w: f64) -> Vector4 {
    hom(x * w, y * w, z * w, w)
}

/// A knot vector of the values `k`.
pub fn knots(k: &[f64]) -> Vec<Scalar> {
    k.iter().map(|&x| Scalar::from_f64(x)).collect()
}

/// The straight line from `a` to `b`, over `[0, 1]`.
pub fn line(a: [f64; 3], b: [f64; 3]) -> NurbCurve3D {
    let p = |c: [f64; 3]| Vector3::from_array(c.map(Scalar::from_f64));
    NurbCurve3D::line(p(a), p(b)).unwrap()
}

/// The unit square in the plane `z = 0`, `u` along `x` and `v` along `y`,
/// as a degree-1 patch over `[0, 1]²`.
pub fn bilinear() -> NurbSurface3D {
    NurbSurface3D::try_new(
        1,
        1,
        vec![
            hom(0., 0., 0., 1.),
            hom(0., 1., 0., 1.),
            hom(1., 0., 0., 1.),
            hom(1., 1., 0., 1.),
        ],
        knots(&[0., 0., 1., 1.]),
        knots(&[0., 0., 1., 1.]),
    )
    .unwrap()
}

/// The exact rational octant of the unit sphere with `x, y, z >= 0`; `v = 1`
/// is the pole `(0, 0, 1)`.
pub fn sphere_octant() -> NurbSurface3D {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let circle = [(1., 0., 1.), (1., 1., w), (0., 1., 1.)];
    let cps = circle
        .iter()
        .flat_map(|&(x, y, wu)| {
            circle
                .iter()
                .map(move |&(r, z, wv)| weighted(x * r, y * r, z, wu * wv))
        })
        .collect();
    let k = knots(&[0., 0., 0., 1., 1., 1.]);
    NurbSurface3D::try_new(2, 2, cps, k.clone(), k).unwrap()
}

/// Quarter of a revolved patch: `u` sweeps the unit quarter arc at height
/// `rim_z`, `v` runs linearly from the collapsed `v = 0` row at
/// `(0, 0, apex_z)` out to the arc. `apex_z = rim_z` is a flat disc, any
/// other a cone.
pub fn revolved(apex_z: f64, rim_z: f64) -> NurbSurface3D {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let rim = [(1., 0., 1.), (1., 1., w), (0., 1., 1.)];
    let mut cps = Vec::new();
    for (x, y, wi) in rim {
        cps.push(hom(0., 0., apex_z * wi, wi));
        cps.push(hom(x * wi, y * wi, rim_z * wi, wi));
    }
    NurbSurface3D::try_new(
        2,
        1,
        cps,
        knots(&[0., 0., 0., 1., 1., 1.]),
        knots(&[0., 0., 1., 1.]),
    )
    .unwrap()
}
