use geop_core_geometry::{
    contains::surface::surface_could_contain,
    nurb_curve::{NurbCurve2D, NurbCurve3D},
    nurb_surface::NurbSurface3D,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    vector::Vector3,
};

use crate::{Model, validation::ValidationParameters};

/// Checks that every edge's 3-D curve, and every coedge's pcurve, actually
/// land on the vertices they're supposed to: an edge's curve must start/end
/// at its `start_vertex`/`end_vertex`, and a coedge's pcurve — evaluated
/// through its face's surface — must start/end at the same two points its
/// own edge does (in whichever order its `sense` implies). Pushes every
/// violation it finds onto `errors` rather than stopping at the first.
pub fn check_curves_and_surfaces_vertices(
    params: &ValidationParameters,
    errors: &mut Vec<GeopError>,
    model: &Model,
) {
    for (&edge_id, edge) in &model.edges {
        let (Some(start), Some(end)) = (
            model.vertices.get(&edge.start_vertex),
            model.vertices.get(&edge.end_vertex),
        ) else {
            continue;
        };
        if let Err(e) = validate_curve_start_and_end(&edge.curve, &start.point, &end.point) {
            errors.push(e.with_context(format!("edge {}", edge_id.0)));
        }
    }

    for (&coedge_id, coedge) in &model.coedges {
        let (Ok(start_id), Ok(end_id)) = (
            model.coedge_start_vertex_id(coedge_id),
            model.coedge_end_vertex_id(coedge_id),
        ) else {
            continue;
        };
        let (Some(start), Some(end)) = (model.vertices.get(&start_id), model.vertices.get(&end_id))
        else {
            continue;
        };
        let Some(face) = model.faces.get(&coedge.face) else {
            continue;
        };
        if let Err(e) =
            validate_pcurve_start_and_end(&face.surface, &coedge.pcurve, &start.point, &end.point)
        {
            // Which face, which edge, and which vertices — a pcurve endpoint
            // that misses its vertex by far more than any search tolerance was
            // *assigned* rather than computed (see `fit_pcurve`'s endpoint
            // pinning), and telling that apart from a genuinely mislocated
            // vertex needs to know what else is attached at the same place.
            // The discriminating fact: does the vertex lie on this face's
            // surface *at all*? If it does, the pcurve simply names the wrong
            // `(u, v)` for it. If it does not, the edge was spliced onto a
            // face it does not belong to, and no pcurve could have been
            // right — a different bug entirely, and one no amount of
            // refitting fixes.
            let on_surface = |p| match surface_could_contain(
                &face.surface,
                p,
                params.max_nodes,
                params.min_subdivision_size,
            ) {
                Ok(Some(uv)) => format!("yes, at {uv:?}"),
                Ok(None) => "NO".to_string(),
                Err(e) => format!("<search failed: {e}>"),
            };
            errors.push(e.with_context(format!(
                "coedge {}, face {}, geometry {:?}, sense {:?}, start vertex {start_id} at {:?} (on this surface? {}), end vertex {end_id} at {:?} (on this surface? {})",
                coedge_id.0,
                coedge.face,
                coedge.geometry,
                coedge.sense,
                start.point,
                on_surface(&start.point),
                end.point,
                on_surface(&end.point),
            )));
        }
    }
}

/// Whether `curve` starts at `start` and ends at `end`.
fn validate_curve_start_and_end(
    curve: &NurbCurve3D,
    start: &Vector3,
    end: &Vector3,
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

/// Whether `pcurve`, evaluated through `surface`, starts at `start` and
/// ends at `end`.
fn validate_pcurve_start_and_end(
    surface: &NurbSurface3D,
    pcurve: &NurbCurve2D,
    start: &Vector3,
    end: &Vector3,
) -> GeopResult<()> {
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

#[cfg(test)]
mod tests {
    use super::check_curves_and_surfaces_vertices;
    use crate::Model;
    use crate::test_fixtures::{check_errors, test_cube_model, test_cube_solid};
    use geop_core_math::{scalars::Scalar, vector::Vector3};

    #[test]
    fn valid_cube_passes() {
        assert!(check_errors(check_curves_and_surfaces_vertices, &test_cube_model()).is_empty());
    }

    #[test]
    fn moved_vertex_breaks_edge_curve() {
        let mut model = Model::new();
        test_cube_solid(&mut model);
        let vertex_id = *model.vertices.keys().next().unwrap();
        model.vertices.get_mut(&vertex_id).unwrap().point =
            Vector3::from_array([Scalar::from_f64(42.0); 3]);
        assert!(!check_errors(check_curves_and_surfaces_vertices, &model).is_empty());
    }

    #[test]
    fn moving_two_vertices_reports_both() {
        let mut model = Model::new();
        test_cube_solid(&mut model);
        let mut ids = model.vertices.keys().copied();
        let v1 = ids.next().unwrap();
        let v2 = ids.next().unwrap();
        model.vertices.get_mut(&v1).unwrap().point =
            Vector3::from_array([Scalar::from_f64(42.0); 3]);
        model.vertices.get_mut(&v2).unwrap().point =
            Vector3::from_array([Scalar::from_f64(43.0); 3]);
        assert!(check_errors(check_curves_and_surfaces_vertices, &model).len() >= 2);
    }
}
