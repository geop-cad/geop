use geop_core_math::{geop_error::GeopError, scalars::Scalar};

use crate::{CoedgeGeometry, Model, boundary::BoundaryType, validation::ValidationParameters};

/// Walks every id referenced anywhere in the model and checks it points to
/// an entry that actually exists in the corresponding arena — the most
/// basic sanity check. Pushes every violation it finds onto `errors` rather
/// than stopping at the first.
pub fn check_pointers<S: Scalar>(
    _params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    for edge in model.edges.values() {
        if !model.vertices.contains_key(&edge.start_vertex) {
            errors.push(GeopError::new(format!(
                "edge references start_vertex {} which does not exist",
                edge.start_vertex.0
            )));
        }
        if !model.vertices.contains_key(&edge.end_vertex) {
            errors.push(GeopError::new(format!(
                "edge references end_vertex {} which does not exist",
                edge.end_vertex.0
            )));
        }
    }

    for coedge in model.coedges.values() {
        match coedge.geometry {
            CoedgeGeometry::Edge(edge_id) => {
                if !model.edges.contains_key(&edge_id) {
                    errors.push(GeopError::new(format!(
                        "coedge references edge {} which does not exist",
                        edge_id.0
                    )));
                }
            }
            CoedgeGeometry::Vertex(vertex_id) => {
                if !model.vertices.contains_key(&vertex_id) {
                    errors.push(GeopError::new(format!(
                        "coedge references vertex {} which does not exist",
                        vertex_id.0
                    )));
                }
            }
        }
        if !model.coedges.contains_key(&coedge.next) {
            errors.push(GeopError::new(format!(
                "coedge references next coedge {} which does not exist",
                coedge.next.0
            )));
        }
        if !model.coedges.contains_key(&coedge.prev) {
            errors.push(GeopError::new(format!(
                "coedge references prev coedge {} which does not exist",
                coedge.prev.0
            )));
        }
        if !model.faces.contains_key(&coedge.face) {
            errors.push(GeopError::new(format!(
                "coedge references face {} which does not exist",
                coedge.face.0
            )));
        }
    }

    for face in model.faces.values() {
        for boundary in face.boundaries() {
            match boundary {
                BoundaryType::Vertex(vertex_id) => {
                    if !model.vertices.contains_key(&vertex_id) {
                        errors.push(GeopError::new(format!(
                            "face boundary references vertex {} which does not exist",
                            vertex_id.0
                        )));
                    }
                }
                BoundaryType::Loop(coedge_id) => {
                    if !model.coedges.contains_key(&coedge_id) {
                        errors.push(GeopError::new(format!(
                            "face boundary references coedge {} which does not exist",
                            coedge_id.0
                        )));
                    }
                }
            }
        }
        if !model.shells.contains_key(&face.shell) {
            errors.push(GeopError::new(format!(
                "face references shell {} which does not exist",
                face.shell.0
            )));
        }
    }

    for shell in model.shells.values() {
        for &face_id in &shell.faces {
            if !model.faces.contains_key(&face_id) {
                errors.push(GeopError::new(format!(
                    "shell references face {} which does not exist",
                    face_id.0
                )));
            }
        }
        if let Some(solid) = shell.solid
            && !model.solids.contains_key(&solid)
        {
            errors.push(GeopError::new(format!(
                "shell references solid {} which does not exist",
                solid.0
            )));
        }
    }

    for solid in model.solids.values() {
        for &shell_id in &solid.shells {
            if !model.shells.contains_key(&shell_id) {
                errors.push(GeopError::new(format!(
                    "solid references shell {} which does not exist",
                    shell_id.0
                )));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::check_pointers;
    use crate::test_fixtures::test_cube_solid;
    use crate::{Model, validation::ValidationParameters};
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector3};

    fn check_valid_cube_passes<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let mut errors = Vec::new();
        check_pointers(&ValidationParameters::default(), &mut errors, &model);
        assert!(errors.is_empty());
    }
    #[test]
    fn valid_cube_passes() {
        for_all_scalars!(check_valid_cube_passes);
    }

    fn check_dangling_vertex_reference_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        let (_v, _f, solid_id) = model.mvfs(Vector3::from_array([S::ZERO; 3]));
        let shell_id = model.get_solid(solid_id).unwrap().shells[0];
        let face_id = model.get_shell(shell_id).unwrap().faces[0];
        model.faces.get_mut(&face_id).unwrap().shell = crate::ShellId(9999);
        let mut errors = Vec::new();
        check_pointers(&ValidationParameters::default(), &mut errors, &model);
        assert!(!errors.is_empty());
    }
    #[test]
    fn dangling_vertex_reference_fails() {
        for_all_scalars!(check_dangling_vertex_reference_fails);
    }

    fn check_multiple_dangling_references_are_all_reported<S: Scalar>() {
        let mut model = Model::<S>::new();
        let (_v, _f, solid_id) = model.mvfs(Vector3::from_array([S::ZERO; 3]));
        let shell_id = model.get_solid(solid_id).unwrap().shells[0];
        let face_id = model.get_shell(shell_id).unwrap().faces[0];
        model.faces.get_mut(&face_id).unwrap().shell = crate::ShellId(9999);
        model.shells.get_mut(&shell_id).unwrap().solid = Some(crate::SolidId(9999));
        let mut errors = Vec::new();
        check_pointers(&ValidationParameters::default(), &mut errors, &model);
        assert_eq!(errors.len(), 2);
    }
    #[test]
    fn multiple_dangling_references_are_all_reported() {
        for_all_scalars!(check_multiple_dangling_references_are_all_reported);
    }
}
