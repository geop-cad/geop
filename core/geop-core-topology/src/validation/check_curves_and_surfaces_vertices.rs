use geop_core_geometry::contains::surface::surface_could_contain;
use geop_core_math::{geop_error::GeopError, scalars::Scalar};

use crate::{
    Model,
    argument_validation::{validate_curve_start_and_end, validate_pcurve_start_and_end},
    validation::ValidationParameters,
};

/// Checks that every edge's 3-D curve, and every coedge's pcurve, actually
/// land on the vertices they're supposed to: an edge's curve must start/end
/// at its `start_vertex`/`end_vertex`, and a coedge's pcurve — evaluated
/// through its face's surface — must start/end at the same two points its
/// own edge does (in whichever order its `sense` implies). Pushes every
/// violation it finds onto `errors` rather than stopping at the first.
pub fn check_curves_and_surfaces_vertices<S: Scalar>(
    params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
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

#[cfg(test)]
mod tests {
    use super::check_curves_and_surfaces_vertices;
    use crate::test_fixtures::test_cube_solid;
    use crate::{Model, validation::ValidationParameters};
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector3};

    fn run<S: Scalar>(model: &Model<S>) -> Vec<geop_core_math::geop_error::GeopError> {
        let mut errors = Vec::new();
        check_curves_and_surfaces_vertices(&ValidationParameters::default(), &mut errors, model);
        errors
    }

    fn check_valid_cube_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        assert!(run(&model).is_empty());
    }
    #[test]
    fn valid_cube_passes() {
        for_all_scalars!(check_valid_cube_passes);
    }

    fn check_moved_vertex_breaks_edge_curve<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let vertex_id = *model.vertices.keys().next().unwrap();
        model.vertices.get_mut(&vertex_id).unwrap().point =
            Vector3::from_array([S::from_f64(42.0); 3]);
        assert!(!run(&model).is_empty());
    }
    #[test]
    fn moved_vertex_breaks_edge_curve() {
        for_all_scalars!(check_moved_vertex_breaks_edge_curve);
    }

    fn check_moving_two_vertices_reports_both<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let mut ids = model.vertices.keys().copied();
        let v1 = ids.next().unwrap();
        let v2 = ids.next().unwrap();
        model.vertices.get_mut(&v1).unwrap().point = Vector3::from_array([S::from_f64(42.0); 3]);
        model.vertices.get_mut(&v2).unwrap().point = Vector3::from_array([S::from_f64(43.0); 3]);
        assert!(run(&model).len() >= 2);
    }
    #[test]
    fn moving_two_vertices_reports_both() {
        for_all_scalars!(check_moving_two_vertices_reports_both);
    }
}
