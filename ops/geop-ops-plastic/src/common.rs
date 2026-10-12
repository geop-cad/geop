//! What the features here share: reading a solid described as a
//! [`geop_core_topology::build::BodySpec`].

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};
use geop_core_topology::Curve3;

/// Halfway along `curve`: the point, and the direction it runs there.
pub fn halfway<S: Scalar>(curve: &Curve3<S>) -> GeopResult<(Vector3<S>, Vector3<S>)> {
    let (t0, t1) = curve.domain();
    let t = S::interpolate(t0, t1, S::from_f64(0.5));
    Ok((curve.evaluate(t)?, curve.tangent(t)?))
}
