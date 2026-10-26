use std::collections::{HashMap, HashSet};

use imbl::OrdMap;

use crate::{
    Body, CoedgeGeometry, CoedgeId, EdgeId, FaceId, Model, Shell, ShellId, Solid, SolidId,
    VertexId, boundary::BoundaryType,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    union_find::UnionFind,
};

impl Model {
    /// Replace the bodies `consumed` by one new solid bounded by `keep`,
    /// which must be faces of those bodies: a shell for each connected set
    /// of them, faces sharing an edge being connected. A boolean that cuts
    /// a bar in two, or leaves a void inside a block, so keeps two shells,
    /// each a closed surface of its own (see [`Solid`]). Every other face
    /// the bodies owned is deleted, and so is everything no face reaches any
    /// more.
    ///
    /// What a boolean does last, once it has decided which faces survive.
    /// `Ok(None)` (and no solid) when `keep` is empty: the result is empty,
    /// which is an answer rather than a failure — and also how a body is
    /// deleted outright.
    ///
    /// Deletion is by reachability rather than by tracking what was split or
    /// re-homed on the way here, so no dangling id can be left behind — and
    /// it only ever deletes from what `consumed` owned: faces of any other
    /// body in the model are none of this operation's business.
    pub fn assemble_solid(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
    ) -> GeopResult<Option<SolidId>> {
        let shells = self.assemble_shells(consumed, &connected(self, keep))?;
        if shells.is_empty() {
            return Ok(None);
        }
        let solid_id = self.insert_solid(Solid {
            shells: shells.clone(),
        });
        for shell_id in shells {
            self.get_shell_mut(shell_id)?.solid = Some(solid_id);
        }
        Ok(Some(solid_id))
    }

    /// Like [`Model::assemble_solid`], but `keep` becomes a sheet — faces
    /// standing on their own, in one shell whether connected or not —
    /// rather than a solid: what is left of a sheet once it is trimmed.
    pub fn assemble_sheet(
        &mut self,
        consumed: &[Body],
        keep: &[FaceId],
    ) -> GeopResult<Option<ShellId>> {
        let groups = if keep.is_empty() {
            Vec::new()
        } else {
            vec![keep.to_vec()]
        };
        Ok(self.assemble_shells(consumed, &groups)?.first().copied())
    }

    /// [`Model::assemble_solid`]'s work, short of making the shells of
    /// `groups` (one shell each) a solid's.
    fn assemble_shells(
        &mut self,
        consumed: &[Body],
        groups: &[Vec<FaceId>],
    ) -> GeopResult<Vec<ShellId>> {
        let mut consumed_faces: HashSet<FaceId> = HashSet::new();
        for &body in consumed {
            consumed_faces.extend(self.body_faces(body)?);
        }
        if let Some(face) = groups
            .iter()
            .flatten()
            .find(|f| !consumed_faces.contains(f))
        {
            return Err(GeopError::new(format!(
                "Model::assemble_solid: face {face} is to be kept, but belongs to none of the consumed bodies {consumed:?}"
            )));
        }
        for &body in consumed {
            match body {
                Body::Solid(solid) => {
                    for shell in self.get_solid(solid)?.shells.clone() {
                        self.shells.remove(&shell);
                    }
                    self.solids.remove(&solid);
                }
                Body::Sheet(shell) => {
                    self.shells.remove(&shell);
                }
            }
        }

        let kept: HashSet<FaceId> = groups.iter().flatten().copied().collect();
        retain(&mut self.faces, |id| {
            !consumed_faces.contains(id) || kept.contains(id)
        });

        let mut shells = Vec::with_capacity(groups.len());
        for faces in groups {
            let shell_id = self.insert_shell(Shell {
                faces: faces.clone(),
                solid: None,
            });
            for &face_id in faces {
                self.get_face_mut(face_id)?.shell = shell_id;
            }
            shells.push(shell_id);
        }
        self.prune_unreachable();
        Ok(shells)
    }

    /// Move every shell of `from` into `into`, deleting `from`: one solid of
    /// both bodies. Only valid for bodies that don't touch — nothing is
    /// intersected, so two overlapping ones would make a solid whose shells
    /// cross; combining those is a boolean union's job.
    pub fn merge_solids(&mut self, into: SolidId, from: SolidId) -> GeopResult<()> {
        if into == from {
            return Err(GeopError::new(format!(
                "Model::merge_solids: refusing to merge solid {into} into itself"
            )));
        }
        self.get_solid(into)?;
        let shells = self.get_solid(from)?.shells.clone();
        for &shell in &shells {
            self.get_shell_mut(shell)?.solid = Some(into);
        }
        self.solids.remove(&from);
        self.get_solid_mut(into)?.shells.extend(shells);
        Ok(())
    }

    /// Drop everything no longer reachable from a face or a wire: the
    /// coedges no face owns, then edges and vertices nothing refers to any
    /// more.
    pub(crate) fn prune_unreachable(&mut self) {
        let live_faces: Vec<FaceId> = self.faces.keys().copied().collect();
        let live_coedges: HashSet<CoedgeId> = live_faces
            .iter()
            .flat_map(|&f| self.iterate_face_coedges(f).collect::<Vec<_>>())
            .collect();
        retain(&mut self.coedges, |id| live_coedges.contains(id));

        let live_edges: HashSet<EdgeId> = self
            .coedges
            .values()
            .filter_map(|c| c.edge().ok())
            .chain(self.wires.values().flat_map(|w| w.edges.iter().copied()))
            .collect();
        retain(&mut self.edges, |id| live_edges.contains(id));

        let mut live_vertices: HashSet<VertexId> = self
            .edges
            .values()
            .flat_map(|e| [e.start_vertex, e.end_vertex])
            .collect();
        for coedge in self.coedges.values() {
            if let CoedgeGeometry::Vertex(v) = coedge.geometry {
                live_vertices.insert(v);
            }
        }
        for face in self.faces.values() {
            for boundary in face.boundaries() {
                if let BoundaryType::Vertex(v) = boundary {
                    live_vertices.insert(v);
                }
            }
        }
        for wire in self.wires.values() {
            live_vertices.extend(wire.vertices.iter().copied());
        }
        retain(&mut self.vertices, |id| live_vertices.contains(id));
    }
}

/// `faces` in sets connected by the edges they share, each in the order
/// given, the sets ordered by their first face.
fn connected(model: &Model, faces: &[FaceId]) -> Vec<Vec<FaceId>> {
    let mut sets = UnionFind::new(faces.len());
    let mut first_user: HashMap<EdgeId, usize> = HashMap::new();
    for (i, &face) in faces.iter().enumerate() {
        for coedge in model.iterate_face_coedges(face) {
            let Some(Ok(edge)) = model.coedges.get(&coedge).map(|c| c.edge()) else {
                continue;
            };
            match first_user.get(&edge) {
                Some(&j) => sets.union(i, j),
                None => {
                    first_user.insert(edge, i);
                }
            }
        }
    }
    let mut groups = sets.groups();
    for group in &mut groups {
        group.sort();
    }
    groups.sort_by_key(|g| g[0]);
    groups
        .into_iter()
        .map(|g| g.into_iter().map(|i| faces[i]).collect())
        .collect()
}

/// Removes the entries of `map` whose ids `keep` rejects, touching nothing
/// else, so that a copy sharing the map shares what is kept.
fn retain<K: Ord + Clone, V: Clone>(map: &mut OrdMap<K, V>, keep: impl Fn(&K) -> bool) {
    let dropped: Vec<K> = map.keys().filter(|id| !keep(id)).cloned().collect();
    for id in dropped {
        map.remove(&id);
    }
}

#[cfg(test)]
mod tests {
    use crate::{Model, test_fixtures::test_cube_solid};

    /// A copy of a model shares its arenas, and pruning what is all reachable
    /// touches none of them: a version made by editing a model costs what the
    /// edit touches, not the model's size.
    #[test]
    fn pruning_a_copy_keeps_it_shared() {
        let mut model = Model::new();
        test_cube_solid(&mut model);
        let copy = model.clone();
        model.prune_unreachable();
        assert!(model.vertices.ptr_eq(&copy.vertices));
        assert!(model.edges.ptr_eq(&copy.edges));
        assert!(model.coedges.ptr_eq(&copy.coedges));
        assert!(model.faces.ptr_eq(&copy.faces));
    }
}
