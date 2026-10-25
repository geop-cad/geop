use geop_core_math::geop_error::GeopError;

use crate::{CoedgeGeometry, Model, boundary::BoundaryType, validation::ValidationParameters};

/// Walks every id referenced anywhere in the model and checks it points to
/// an entry that actually exists in the corresponding arena — the most
/// basic sanity check. Pushes every violation it finds onto `errors` rather
/// than stopping at the first.
pub fn check_pointers(_params: &ValidationParameters, errors: &mut Vec<GeopError>, model: &Model) {
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

    for (id, wire) in &model.wires {
        for &vertex_id in &wire.vertices {
            if !model.vertices.contains_key(&vertex_id) {
                errors.push(GeopError::new(format!(
                    "{id} references vertex {} which does not exist",
                    vertex_id.0
                )));
            }
        }
        for &edge_id in &wire.edges {
            if !model.edges.contains_key(&edge_id) {
                errors.push(GeopError::new(format!(
                    "{id} references edge {} which does not exist",
                    edge_id.0
                )));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::check_pointers;
    use crate::Model;
    use crate::test_fixtures::{check_errors, test_cube_model, test_cube_solid};

    #[test]
    fn valid_cube_passes() {
        assert!(check_errors(check_pointers, &test_cube_model()).is_empty());
    }

    #[test]
    fn dangling_vertex_reference_fails() {
        let mut model = Model::new();
        let solid_id = test_cube_solid(&mut model);
        let shell_id = model.get_solid(solid_id).unwrap().shells[0];
        let face_id = model.get_shell(shell_id).unwrap().faces[0];
        model.faces.get_mut(&face_id).unwrap().shell = crate::ShellId(9999);
        let errors = check_errors(check_pointers, &model);
        assert!(!errors.is_empty());
    }

    #[test]
    fn multiple_dangling_references_are_all_reported() {
        let mut model = Model::new();
        let solid_id = test_cube_solid(&mut model);
        let shell_id = model.get_solid(solid_id).unwrap().shells[0];
        let face_id = model.get_shell(shell_id).unwrap().faces[0];
        model.faces.get_mut(&face_id).unwrap().shell = crate::ShellId(9999);
        model.shells.get_mut(&shell_id).unwrap().solid = Some(crate::SolidId(9999));
        let errors = check_errors(check_pointers, &model);
        assert_eq!(errors.len(), 2);
    }
}
