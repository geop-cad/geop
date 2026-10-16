//! Basic solids built directly from profiles and surface patches — cubes,
//! cylinders, spheres, a figure-eight prism — for tests to be written
//! against. Only built for tests: with `testing`, or in this crate's
//! own. Besides the solids, what tests describe them with: points, plain
//! profiles and planes, from plain numbers.

mod cube;
mod cylinder;
mod figure8_profile;
mod sphere;

pub use cube::cube_solid;
pub use cylinder::{Axis, extruded_cylinder, revolved_cylinder, revolved_cylinder_along_axis};
pub use figure8_profile::figure8_profile;
pub use sphere::sphere_solid;

use geop_core_geometry::nurb_curve::NurbCurve2D;
use geop_core_math::{
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_topology::SolidId;
use geop_ops_model::NamedModel;

use crate::common::polygon;

/// The point `(x, y)`.
pub fn v2<S: Scalar>(x: f64, y: f64) -> Vector2<S> {
    Vector2::from_array([S::from_f64(x), S::from_f64(y)])
}

/// The point `(x, y, z)`.
pub fn v3<S: Scalar>(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
}

/// The box spanning `[min, max]` in `model`, named `cube(name)` (see
/// [`cube_solid`]).
pub fn block<S: Scalar>(
    model: &mut NamedModel<S>,
    name: &str,
    min: [f64; 3],
    max: [f64; 3],
) -> SolidId {
    let [x0, y0, z0] = min;
    let [x1, y1, z1] = max;
    cube_solid(model, name, v3(x0, y0, z0), v3(x1, y1, z1)).unwrap()
}

/// A part of only the box spanning `[min, max]`, named `cube(name)` (see
/// [`block`]).
pub fn block_part<S: Scalar>(name: &str, min: [f64; 3], max: [f64; 3]) -> NamedModel<S> {
    let mut part = NamedModel::new();
    block(&mut part, name, min, max);
    part
}

/// The cylinder of `radius` and `height` in `model`, standing on
/// `base_center` along `z`, named `cylinder(name)` (see
/// [`revolved_cylinder`]).
pub fn cylinder<S: Scalar>(
    model: &mut NamedModel<S>,
    name: &str,
    base_center: [f64; 3],
    radius: f64,
    height: f64,
) -> SolidId {
    let [x, y, z] = base_center;
    let (radius, height) = (S::from_f64(radius), S::from_f64(height));
    revolved_cylinder(model, name, v3(x, y, z), radius, height).unwrap()
}

/// A circle of radius `r` around `(cx, cy)` as four quarter arcs,
/// counter-clockwise (reversed: clockwise, for a hole).
pub fn circle_at<S: Scalar>(cx: f64, cy: f64, r: f64, clockwise: bool) -> Vec<NurbCurve2D<S>> {
    let arcs = crate::common::circle_at(cx, cy, r).unwrap();
    match clockwise {
        true => arcs.iter().rev().map(|c| c.reverse()).collect(),
        false => arcs,
    }
}

/// A circle of radius `r` around the origin, as four quarter arcs,
/// counter-clockwise.
pub fn circle<S: Scalar>(r: f64) -> Vec<NurbCurve2D<S>> {
    circle_at(0.0, 0.0, r, false)
}

/// The square of half side `half` around the origin, counter-clockwise.
pub fn square<S: Scalar>(half: f64) -> Vec<NurbCurve2D<S>> {
    polygon(&[
        v2(-half, -half),
        v2(half, -half),
        v2(half, half),
        v2(-half, half),
    ])
    .unwrap()
}

/// The plane through `origin` spanned by `u` and `v`.
pub fn plane<S: Scalar>(origin: Vector3<S>, u: Vector3<S>, v: Vector3<S>) -> CoordinateSystem<S> {
    CoordinateSystem::try_new(origin, u, v, u.prod_cross(&v)).unwrap()
}
