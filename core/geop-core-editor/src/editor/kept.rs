//! [`Kept`]: what a viewer keeps of the scenes it was sent — the views of
//! the components its placed parts are drawn from, and the meshes of the
//! faces drawn — each by its key, sent once and forgotten once nothing
//! drawn has used it for longest (see [`super::SceneState`]).

use std::{collections::BTreeMap, rc::Rc};

use crate::view::{FaceMesh, PartView};

/// How many components no placed part uses a viewer keeps (see
/// [`super::SceneState::forgotten`]).
const KEPT_COMPONENTS: usize = 8;

/// How many triangles of meshes no face drawn uses a viewer keeps (see
/// [`super::SceneState::forgotten_meshes`]): a few large parts' worth, so
/// that showing a step before a large part, and the part again, does not
/// send its faces again.
const KEPT_TRIANGLES: usize = 250_000;

/// A mesh a viewer keeps.
struct KeptMesh {
    /// The scene its part last drew it in.
    used: u64,
    triangles: usize,
    /// How many faces of the components the viewer keeps are drawn with it:
    /// while any are, it is kept.
    in_components: usize,
}

/// What a viewer keeps, by key: see the module docs.
#[derive(Default)]
pub(super) struct Kept {
    /// How many scenes were sent.
    scenes: u64,
    /// The components, with the scene a placed part last used each in.
    components: BTreeMap<String, (u64, Rc<PartView>)>,
    meshes: BTreeMap<String, KeptMesh>,
}

/// What a scene sends of what the viewer keeps, and what it forgets.
pub(super) struct Sent {
    pub components: BTreeMap<String, Rc<PartView>>,
    pub forgotten: Vec<String>,
    pub meshes: BTreeMap<String, Rc<FaceMesh>>,
    pub forgotten_meshes: Vec<String>,
}

impl Kept {
    /// What a viewer that keeps nothing keeps: one that may have started
    /// afresh.
    pub fn clear(&mut self) {
        *self = Self::default();
    }

    /// The scene drawing `part`, and its placed parts from their
    /// components: what of it the viewer does not have, and what it no
    /// longer keeps.
    pub fn send(&mut self, part: &PartView) -> Sent {
        self.scenes += 1;
        let now = self.scenes;
        let mut meshes = BTreeMap::new();
        let mut components = BTreeMap::new();
        for instance in &part.instances {
            let view = &instance.view;
            let kept = (self.components).insert(instance.component.clone(), (now, view.clone()));
            if kept.is_none() {
                components.insert(instance.component.clone(), view.clone());
                for face in &view.faces {
                    let kept = self.keep(&face.mesh, &mut meshes);
                    kept.in_components += 1;
                }
            }
        }
        for face in &part.faces {
            self.keep(&face.mesh, &mut meshes).used = now;
        }
        // Of the components no placed part uses now, the viewer keeps the
        // last used.
        let mut unused: Vec<(u64, String)> = (self.components.iter())
            .filter(|(_, (used, _))| *used < now)
            .map(|(key, (used, _))| (*used, key.clone()))
            .collect();
        unused.sort();
        let forget = unused.len().saturating_sub(KEPT_COMPONENTS);
        let forgotten: Vec<String> = (unused.into_iter().take(forget))
            .map(|(_, key)| key)
            .collect();
        for key in &forgotten {
            let Some((_, view)) = self.components.remove(key) else {
                continue;
            };
            for face in &view.faces {
                if let Some(kept) = self.meshes.get_mut(&face.mesh.key) {
                    kept.in_components -= 1;
                }
            }
        }
        // Of the meshes nothing drawn uses, it keeps the last used, as many
        // triangles of them as it keeps.
        let mut unused: Vec<(u64, &String, usize)> = (self.meshes.iter())
            .filter(|(_, kept)| kept.used < now && kept.in_components == 0)
            .map(|(key, kept)| (kept.used, key, kept.triangles))
            .collect();
        unused.sort_by(|a, b| b.cmp(a));
        let mut triangles = 0;
        let forgotten_meshes: Vec<String> = (unused.into_iter())
            .skip_while(|(_, _, n)| {
                triangles += n;
                triangles <= KEPT_TRIANGLES
            })
            .map(|(_, key, _)| key.clone())
            .collect();
        for key in &forgotten_meshes {
            self.meshes.remove(key);
        }
        Sent {
            components,
            forgotten,
            meshes,
            forgotten_meshes,
        }
    }

    /// The mesh `mesh` as the viewer keeps it: put in `sending` if it does
    /// not have it yet.
    fn keep(
        &mut self,
        mesh: &Rc<FaceMesh>,
        sending: &mut BTreeMap<String, Rc<FaceMesh>>,
    ) -> &mut KeptMesh {
        self.meshes.entry(mesh.key.clone()).or_insert_with(|| {
            sending.insert(mesh.key.clone(), mesh.clone());
            KeptMesh {
                used: 0,
                triangles: mesh.triangles.len(),
                in_components: 0,
            }
        })
    }
}
