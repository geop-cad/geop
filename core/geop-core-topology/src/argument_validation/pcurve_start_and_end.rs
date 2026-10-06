use geop_core_geometry::{nurb_curve::NurbCurve2D, nurb_surface::NurbSurface3D};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};

pub fn validate_pcurve_start_and_end<S: Scalar>(
    surface: &NurbSurface3D<S>,
    pcurve: &NurbCurve2D<S>,
    start: &Vector3<S>,
    end: &Vector3<S>,
) -> GeopResult<()> {
    // Validate pcurve start and end
    let (t0, t1) = pcurve.domain();
    let pcurve_start = pcurve.evaluate(t0)?;
    let pcurve_start_3d = surface.evaluate(pcurve_start[0], pcurve_start[1])?;
    if !pcurve_start_3d.could_be_equal(start) {
        return Err(GeopError::new(format!(
            "pcurve {pcurve} start point {pcurve_start_3d:?} does not match the given 3D point {start:?} (pcurve start uv {pcurve_start:?})"
        )));
    }
    let pcurve_end = pcurve.evaluate(t1)?;
    let pcurve_end_3d = surface.evaluate(pcurve_end[0], pcurve_end[1])?;
    if !pcurve_end_3d.could_be_equal(end) {
        return Err(GeopError::new(format!(
            "pcurve {pcurve} end point {pcurve_end_3d:?} does not match the given 3D point {end:?} (pcurve end uv {pcurve_end:?})"
        )));
    }
    Ok(())
}
