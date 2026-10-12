//! Test-only fixtures the curve and surface tests share: control points,
//! knot vectors, and a few small curves and surfaces.

use geop_core_math::{
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use crate::{nurb_curve::NurbCurve3D, nurb_surface::NurbSurface3D};

/// The homogeneous control point `(x, y, z, w)`, its coordinates taken as
/// given (already multiplied by the weight).
pub fn hom<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
    Vector4::from_array([x, y, z, w].map(S::from_f64))
}

/// The point `(x, y, z)` with weight `w`, homogeneous: `(w x, w y, w z, w)`.
pub fn weighted<S: Scalar>(x: f64, y: f64, z: f64, w: f64) -> Vector4<S> {
    hom(x * w, y * w, z * w, w)
}

/// A knot vector of the values `k`.
pub fn knots<S: Scalar>(k: &[f64]) -> Vec<S> {
    k.iter().map(|&x| S::from_f64(x)).collect()
}

/// The straight line from `a` to `b`, over `[0, 1]`.
pub fn line<S: Scalar>(a: [f64; 3], b: [f64; 3]) -> NurbCurve3D<S> {
    let p = |c: [f64; 3]| Vector3::from_array(c.map(S::from_f64));
    NurbCurve3D::line(p(a), p(b)).unwrap()
}

/// The unit square in the plane `z = 0`, `u` along `x` and `v` along `y`,
/// as a degree-1 patch over `[0, 1]²`.
pub fn bilinear<S: Scalar>() -> NurbSurface3D<S> {
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
pub fn sphere_octant<S: Scalar>() -> NurbSurface3D<S> {
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
