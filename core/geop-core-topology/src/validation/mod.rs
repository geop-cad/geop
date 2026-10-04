//! Whole-model consistency checks — as opposed to
//! [`super::argument_validation`], which checks a single euler operator
//! call's own arguments. Each check lives in its own file, one check per
//! file. Every check function takes `(params, errors, model)` and pushes
//! its own violations onto `errors` rather than returning on the first one,
//! so [`validate`] always runs every check and reports everything wrong in
//! one pass.

mod check_curves_and_surfaces_vertices;
mod curve_and_surface_sampling_check;
mod disjointness_check;
mod face_face_numerical_intersection;
mod face_orientation;
mod faces_have_interior;
mod holes_inside_outer;
mod manifold_check;
pub mod numerical_accuracy;
mod parameters;
mod pcurve_loop_continuity;
mod pointer_check;
mod two_way_references;

pub use face_orientation::{Outward, normal_points_outward};
pub use manifold_check::validate_manifold;
pub use parameters::ValidationParameters;

use std::collections::{BTreeMap, HashMap};

use crate::{Body, CoedgeGeometry, EdgeId, FaceId, Model, VertexId};
use geop_core_math::{geop_error::GeopError, scalars::Scalar};

/// The vertices, edges and faces of one body (see [`body_groups`]).
#[derive(Default)]
pub(crate) struct BodyGroup {
    pub vertices: Vec<VertexId>,
    pub edges: Vec<EdgeId>,
    pub faces: Vec<FaceId>,
}

/// The model's entities, grouped by the body they belong to — what must not
/// overlap, or cross without sharing a vertex, is the entities of one body:
/// separate bodies may touch, or overlap, as they please (the two halves of
/// a split, a new body extruded into another). An entity of no body — of a
/// face whose shell is not there, as in a model still being put together —
/// is in a group of its own with all others like it.
pub(crate) fn body_groups<S: Scalar>(model: &Model<S>) -> Vec<BodyGroup> {
    let body_of_face = |face: FaceId| model.body_of_face(face).ok();
    let body_of_coedge = |c: &crate::Coedge<S>| body_of_face(c.face);
    let mut edge_body: HashMap<EdgeId, Option<Body>> = HashMap::new();
    let mut vertex_body: HashMap<VertexId, Option<Body>> = HashMap::new();
    for coedge in model.coedges.values() {
        let body = body_of_coedge(coedge);
        match coedge.geometry {
            CoedgeGeometry::Edge(e) => {
                edge_body.entry(e).or_insert(body);
            }
            CoedgeGeometry::Vertex(v) => {
                vertex_body.entry(v).or_insert(body);
            }
        }
    }
    for (&id, edge) in &model.edges {
        let body = edge_body.get(&id).copied().flatten();
        for v in [edge.start_vertex, edge.end_vertex] {
            vertex_body.entry(v).or_insert(body);
        }
    }
    let key = |body: Option<Body>| match body {
        None => (0, 0),
        Some(Body::Solid(s)) => (1, s.0),
        Some(Body::Sheet(s)) => (2, s.0),
    };
    let mut groups: BTreeMap<(u8, u64), BodyGroup> = BTreeMap::new();
    for &id in model.vertices.keys() {
        let body = vertex_body.get(&id).copied().flatten();
        groups.entry(key(body)).or_default().vertices.push(id);
    }
    for &id in model.edges.keys() {
        let body = edge_body.get(&id).copied().flatten();
        groups.entry(key(body)).or_default().edges.push(id);
    }
    for &id in model.faces.keys() {
        groups
            .entry(key(body_of_face(id)))
            .or_default()
            .faces
            .push(id);
    }
    groups.into_values().collect()
}

pub fn validate<S: Scalar>(
    params: &ValidationParameters<S>,
    model: &Model<S>,
) -> Result<(), Vec<GeopError>> {
    let mut errors = Vec::new();

    // First, because it explains the rest — see `check_numerical_accuracy`.
    numerical_accuracy::check_numerical_accuracy(params, &mut errors, model);
    pointer_check::check_pointers(params, &mut errors, model);
    two_way_references::check_two_way_references(params, &mut errors, model);
    check_curves_and_surfaces_vertices::check_curves_and_surfaces_vertices(
        params,
        &mut errors,
        model,
    );
    curve_and_surface_sampling_check::check_curve_and_surface_sampling(params, &mut errors, model);
    pcurve_loop_continuity::check_pcurve_loop_continuity(params, &mut errors, model);
    disjointness_check::check_vertices_disjoint(params, &mut errors, model);
    disjointness_check::check_edges_disjoint(params, &mut errors, model);
    disjointness_check::check_edges_and_faces_consistent(params, &mut errors, model);
    face_face_numerical_intersection::check_face_face_numerical_intersection(
        params,
        &mut errors,
        model,
    );
    holes_inside_outer::check_holes_inside_outer(params, &mut errors, model);
    face_orientation::check_loop_winding(params, &mut errors, model);
    face_orientation::check_normals_point_outward(params, &mut errors, model);

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

/// A cheap subset of [`validate`]: only the checks that are purely
/// structural/combinatorial (pointer validity, two-way `next`/`prev`/backref
/// consistency, pcurve loop continuity) or a single pass over already-stored
/// geometry (vertices vs. their edges'/faces' curves/surfaces) — none of the
/// `O(n^2)` pairwise numerical-intersection searches (`disjointness_check`,
/// `face_face_numerical_intersection`) or the sampling-based
/// `curve_and_surface_sampling_check`. Meant for call sites that want a
/// quick "is this model still well-formed" check after every mutating step
/// (e.g. a test sweeping many scenes) without paying for the searches that
/// dominate `validate`'s cost.
pub fn validate_fast<S: Scalar>(
    params: &ValidationParameters<S>,
    model: &Model<S>,
) -> Result<(), Vec<GeopError>> {
    let mut errors = Vec::new();

    // First, because it explains the rest: an entity carrying more
    // uncertainty than the searches assume fails somewhere else entirely.
    numerical_accuracy::check_numerical_accuracy(params, &mut errors, model);
    pointer_check::check_pointers(params, &mut errors, model);
    two_way_references::check_two_way_references(params, &mut errors, model);
    check_curves_and_surfaces_vertices::check_curves_and_surfaces_vertices(
        params,
        &mut errors,
        model,
    );
    pcurve_loop_continuity::check_pcurve_loop_continuity(params, &mut errors, model);
    faces_have_interior::check_faces_have_interior(params, &mut errors, model);

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

#[cfg(test)]
mod tests {
    use super::{ValidationParameters, validate};
    use crate::Model;
    use crate::test_fixtures::test_cube_solid;
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector3};

    fn check_cube_passes_full_validate<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        validate(&ValidationParameters::default(), &model).unwrap();
    }
    #[test]
    fn cube_passes_full_validate() {
        for_all_scalars!(check_cube_passes_full_validate);
    }

    fn check_independent_failures_are_all_reported<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);

        // Break two independent checks at once: a shell/face backref
        // mismatch (`check_two_way_references`) and a moved vertex that no
        // longer matches its edges' curves (`check_curves_and_surfaces_vertices`).
        let face_id = *model.faces.keys().next().unwrap();
        let shell_id = model.faces[&face_id].shell;
        model
            .shells
            .get_mut(&shell_id)
            .unwrap()
            .faces
            .retain(|&f| f != face_id);

        let vertex_id = *model.vertices.keys().next().unwrap();
        model.vertices.get_mut(&vertex_id).unwrap().point =
            Vector3::from_array([S::from_f64(42.0); 3]);

        // Both the shell backref break and the moved vertex (which affects
        // every edge/coedge touching it) show up; the exact count isn't the
        // point, only that more than one independent failure got reported.
        let errors = validate(&ValidationParameters::default(), &model).unwrap_err();
        assert!(errors.len() >= 2);
    }
    #[test]
    fn independent_failures_are_all_reported() {
        for_all_scalars!(check_independent_failures_are_all_reported);
    }
}
