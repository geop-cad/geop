//! What the features here share: reading a solid described as a
//! [`geop_core_topology::build::BodySpec`].

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector3};
use geop_core_topology::Curve3;

/// Halfway along `curve`: the point, and the direction it runs there.
pub fn halfway<S: Scalar>(curve: &Curve3<S>) -> GeopResult<(Vector3<S>, Vector3<S>)> {
    let t = curve.mid_parameter()?;
    Ok((curve.evaluate(t)?, curve.tangent(t)?))
}
