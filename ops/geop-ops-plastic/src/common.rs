//! What the features here share: reading a solid described as a
//! [`geop_core_topology::build::BodySpec`]. Their search budgets are the
//! shell's, whose machinery they build on.

use geop_core_geometry::{contains::surface::surface_could_contain, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_topology::Curve3;
use geop_ops_shell::shell::{MAX_NODES, PROJECT_ITERATIONS, min_subdivision_size};

/// Halfway along `curve`: the point, and the direction it runs there.
pub fn halfway<S: Scalar>(curve: &Curve3<S>) -> GeopResult<(Vector3<S>, Vector3<S>)> {
    let (t0, t1) = curve.domain();
    let t = S::interpolate(t0, t1, S::from_f64(0.5));
    Ok((curve.evaluate(t)?, curve.tangent(t)?))
}

/// The normal of `surface` at `point`, which lies on it — pointing out of
/// the solid, for a face's surface.
pub fn normal_at<S: Scalar>(
    surface: &NurbSurface3D<S>,
    point: &Vector3<S>,
) -> GeopResult<Vector3<S>> {
    let (u, v) = surface_could_contain(surface, point, MAX_NODES, min_subdivision_size())?
        .ok_or_else(|| GeopError::new(format!("{point:?} is not on the surface")))?;
    let (u, v) = surface.project(*point, u.sharpen(), v.sharpen(), PROJECT_ITERATIONS)?;
    surface.normal(u, v)
}
