use std::collections::HashMap;

use geop_core_math::{geop_error::GeopError, scalars::Scalar};

use crate::{
    CoedgeGeometry, CoedgeId, EdgeId, FaceId, Model, VertexId, WireId, boundary::BoundaryType,
    validation::ValidationParameters,
};

/// Checks every reference in the model that is meant to be mirrored by a
/// reference pointing back the other way: parent/child links (a face's
/// `shell` names a shell that in turn lists that face, and so on up to
/// solids), and the doubly-linked coedge topology (`next`/`prev` pointers,
/// and each coedge in a face's loop actually naming that face, and vice
/// versa — that face's boundaries actually reach it). Pushes every
/// violation it finds onto `errors` rather than stopping at the first;
/// single-hop lookups (e.g. `face.shell`) are done via `.get()` and skipped
/// if dangling (already reported by [`super::pointer_check::check_pointers`]),
/// but a face's boundary loop is still walked assuming its whole `next`
/// chain resolves — a chain that's corrupted deep inside is on
/// `check_pointers` to catch, not this.
pub fn check_two_way_references<S: Scalar>(
    _params: &ValidationParameters<S>,
    errors: &mut Vec<GeopError>,
    model: &Model<S>,
) {
    // coedge.next.prev == self, coedge.prev.next == self.
    for (&coedge_id, coedge) in &model.coedges {
        if let Some(next) = model.coedges.get(&coedge.next) {
            if next.prev != coedge_id {
                errors.push(GeopError::new(format!(
                    "coedge {} has next {} whose prev is {} (expected {})",
                    coedge_id.0, coedge.next.0, next.prev.0, coedge_id.0
                )));
            }
        }
        if let Some(prev) = model.coedges.get(&coedge.prev) {
            if prev.next != coedge_id {
                errors.push(GeopError::new(format!(
                    "coedge {} has prev {} whose next is {} (expected {})",
                    coedge_id.0, coedge.prev.0, prev.next.0, coedge_id.0
                )));
            }
        }
    }

    // Every coedge reachable from some face's boundary loop, mapped to the
    // face that reaches it — built up front so the coedge pass below can
    // check the reverse direction (`coedge.face` names a face that actually
    // lists it back) in one lookup instead of a walk per coedge.
    let mut claimed: HashMap<CoedgeId, FaceId> = HashMap::new();

    for (&face_id, face) in &model.faces {
        // A boundary no longer carries a back-pointer to its own face, so
        // there is nothing here to disagree with the face that owns it — the
        // check this replaces existed only to catch that field going stale.
        for boundary in face.boundaries() {
            let BoundaryType::Loop(anchor) = boundary else {
                continue;
            };
            if !model.coedges.contains_key(&anchor) {
                continue;
            }
            // Walked with a hard cap so a corrupted next-chain (not yet
            // ruled out by this same check) can't hang.
            let coedge_ids: Vec<CoedgeId> = model
                .iterate_loop_coedges(anchor)
                .take(model.coedges.len() + 1)
                .collect();
            if coedge_ids.len() > model.coedges.len() {
                errors.push(GeopError::new(format!(
                    "face {}'s boundary loop anchored at coedge {} never returns to its anchor",
                    face_id.0, anchor.0
                )));
            }
            for coedge_id in coedge_ids {
                let Some(coedge) = model.coedges.get(&coedge_id) else {
                    continue;
                };
                if coedge.face != face_id {
                    errors.push(GeopError::new(format!(
                        "coedge {} is on face {}'s boundary loop but names face {} instead",
                        coedge_id.0, face_id.0, coedge.face.0
                    )));
                }
                if let Some(&other_face) = claimed.get(&coedge_id) {
                    errors.push(GeopError::new(format!(
                        "coedge {} is reachable from both face {} and face {}'s boundaries",
                        coedge_id.0, other_face.0, face_id.0
                    )));
                }
                claimed.insert(coedge_id, face_id);
            }
        }

        // Every face's `shell` must list that face back.
        if let Some(shell) = model.shells.get(&face.shell) {
            if !shell.faces.contains(&face_id) {
                errors.push(GeopError::new(format!(
                    "face {} names shell {}, but that shell does not list it back",
                    face_id.0, face.shell.0
                )));
            }
        }
    }

    // Every coedge's `face` must actually list it back in some boundary.
    for (&coedge_id, coedge) in &model.coedges {
        if claimed.get(&coedge_id) != Some(&coedge.face) {
            errors.push(GeopError::new(format!(
                "coedge {} names face {}, but that face's boundaries never reach it",
                coedge_id.0, coedge.face.0
            )));
        }
    }

    for (&shell_id, shell) in &model.shells {
        // Every face a shell lists must name that shell back.
        for &face_id in &shell.faces {
            if let Some(face) = model.faces.get(&face_id) {
                if face.shell != shell_id {
                    errors.push(GeopError::new(format!(
                        "shell {} lists face {}, but that face names shell {} instead",
                        shell_id.0, face_id.0, face.shell.0
                    )));
                }
            }
        }

        // Every shell's `solid` must list that shell back.
        if let Some(solid_id) = shell.solid
            && let Some(solid) = model.solids.get(&solid_id)
            && !solid.shells.contains(&shell_id)
        {
            errors.push(GeopError::new(format!(
                "shell {} names solid {}, but that solid does not list it back",
                shell_id.0, solid_id.0
            )));
        }
    }

    for (&solid_id, solid) in &model.solids {
        // Every shell a solid lists must name that solid back.
        for &shell_id in &solid.shells {
            if let Some(shell) = model.shells.get(&shell_id) {
                if shell.solid != Some(solid_id) {
                    errors.push(GeopError::new(format!(
                        "solid {} lists shell {}, but that shell names solid {:?} instead",
                        solid_id.0, shell_id.0, shell.solid
                    )));
                }
            }
        }
    }

    check_wires(errors, model);
}

/// A wire owns its edges and vertices alone (see [`crate::Wire`]): no
/// coedge uses an edge of it or ends at a vertex of it, its edges end at
/// its own vertices, and nothing is in two wires.
fn check_wires<S: Scalar>(errors: &mut Vec<GeopError>, model: &Model<S>) {
    let mut vertex_wire: HashMap<VertexId, WireId> = HashMap::new();
    let mut edge_wire: HashMap<EdgeId, WireId> = HashMap::new();
    for (&wire_id, wire) in &model.wires {
        for &v in &wire.vertices {
            if let Some(other) = vertex_wire.insert(v, wire_id) {
                errors.push(GeopError::new(format!(
                    "{v} is in both {other} and {wire_id}"
                )));
            }
        }
        for &e in &wire.edges {
            if let Some(other) = edge_wire.insert(e, wire_id) {
                errors.push(GeopError::new(format!(
                    "{e} is in both {other} and {wire_id}"
                )));
            }
            let Some(edge) = model.edges.get(&e) else {
                continue;
            };
            for end in [edge.start_vertex, edge.end_vertex] {
                if !wire.vertices.contains(&end) {
                    errors.push(GeopError::new(format!(
                        "{wire_id} has {e}, but not {end}, where it ends"
                    )));
                }
            }
        }
    }
    for (&coedge_id, coedge) in &model.coedges {
        let (edge, vertices) = match coedge.geometry {
            CoedgeGeometry::Edge(e) => match model.edges.get(&e) {
                Some(edge) => (Some(e), vec![edge.start_vertex, edge.end_vertex]),
                None => (Some(e), Vec::new()),
            },
            CoedgeGeometry::Vertex(v) => (None, vec![v]),
        };
        let used = edge
            .and_then(|e| Some((e.to_string(), *edge_wire.get(&e)?)))
            .into_iter()
            .chain(
                vertices
                    .iter()
                    .filter_map(|v| Some((v.to_string(), *vertex_wire.get(v)?))),
            );
        for (id, wire) in used {
            errors.push(GeopError::new(format!(
                "coedge {} of face {} uses {id} of {wire}, which bounds no face",
                coedge_id.0, coedge.face.0
            )));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::check_two_way_references;
    use crate::test_fixtures::test_cube_solid;
    use crate::{Model, validation::ValidationParameters};
    use geop_core_math::{for_all_scalars, scalars::Scalar, vector::Vector3};

    fn run<S: Scalar>(model: &Model<S>) -> Vec<geop_core_math::geop_error::GeopError> {
        let mut errors = Vec::new();
        check_two_way_references(&ValidationParameters::default(), &mut errors, model);
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

    fn check_broken_next_prev_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);
        let coedge_id = *model.coedges.keys().next().unwrap();
        let real_next = model.coedges[&coedge_id].next;
        // Pick a target that isn't already `coedge_id`'s real `next`, or
        // reassigning it wouldn't actually change anything.
        let other_id = *model
            .coedges
            .keys()
            .find(|&&id| id != coedge_id && id != real_next)
            .unwrap();
        model.coedges.get_mut(&coedge_id).unwrap().next = other_id;
        assert!(!run(&model).is_empty());
    }
    #[test]
    fn broken_next_prev_fails() {
        for_all_scalars!(check_broken_next_prev_fails);
    }

    fn check_mismatched_shell_backref_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        let (_v, _f, solid_id) = model.mvfs(Vector3::from_array([S::ZERO; 3]));
        let shell_id = model.get_solid(solid_id).unwrap().shells[0];
        model.shells.get_mut(&shell_id).unwrap().faces = vec![];
        assert!(!run(&model).is_empty());
    }
    #[test]
    fn mismatched_shell_backref_fails() {
        for_all_scalars!(check_mismatched_shell_backref_fails);
    }

    fn check_mismatched_solid_backref_fails<S: Scalar>() {
        let mut model = Model::<S>::new();
        let (_v, _f, solid_id) = model.mvfs(Vector3::from_array([S::ZERO; 3]));
        // The shell still (validly) names `solid_id`, but the solid no
        // longer lists it back — a mismatch, not a dangling reference.
        model.solids.get_mut(&solid_id).unwrap().shells = vec![];
        assert!(!run(&model).is_empty());
    }
    #[test]
    fn mismatched_solid_backref_fails() {
        for_all_scalars!(check_mismatched_solid_backref_fails);
    }

    fn check_multiple_independent_violations_are_all_reported<S: Scalar>() {
        let mut model = Model::<S>::new();
        test_cube_solid(&mut model);

        // Break next/prev symmetry on one coedge...
        let coedge_id = *model.coedges.keys().next().unwrap();
        let real_next = model.coedges[&coedge_id].next;
        let other_id = *model
            .coedges
            .keys()
            .find(|&&id| id != coedge_id && id != real_next)
            .unwrap();
        model.coedges.get_mut(&coedge_id).unwrap().next = other_id;

        // ...and a shell/face backref, independently.
        let face_id = *model.faces.keys().next().unwrap();
        let shell_id = model.faces[&face_id].shell;
        model
            .shells
            .get_mut(&shell_id)
            .unwrap()
            .faces
            .retain(|&f| f != face_id);

        assert!(run(&model).len() >= 2);
    }
    #[test]
    fn multiple_independent_violations_are_all_reported() {
        for_all_scalars!(check_multiple_independent_violations_are_all_reported);
    }
}
