use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};

pub fn validate_curve_start_and_end<S: Scalar>(
    curve: &NurbCurve3D<S>,
    start: &Vector3<S>,
    end: &Vector3<S>,
) -> GeopResult<()> {
    let (t0, t1) = curve.domain();
    let curve_start = curve.evaluate(t0)?;
    if !curve_start.could_be_equal(start) {
        return Err(GeopError::new(format!(
            "curve {curve} start point {curve_start:?} does not match the given 3D point {start:?}"
        )));
    }
    let curve_end = curve.evaluate(t1)?;
    if !curve_end.could_be_equal(end) {
        return Err(GeopError::new(format!(
            "curve {curve} end point {curve_end:?} does not match the given 3D point {end:?}"
        )));
    }
    Ok(())
}
