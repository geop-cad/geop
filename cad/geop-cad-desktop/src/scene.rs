//! The scene as the desktop app keeps it: what the editor sent of the part
//! drawn, made plain data that can cross from the kernel's thread to the
//! window's ([`SceneData`], [`PartData`]), and what is kept of the scenes
//! sent before ([`Kept`]): the placed parts by name, the views of their
//! components and the meshes of the faces by key, each sent once — as the
//! web app keeps them (`applyScene`).

use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
};

use geop_core_editor::{
    editor::SceneState,
    registry::StructureItem,
    view::{FaceMesh, PartView, ViewInstance, lit::Targets},
};
use geop_core_math::{
    primitives::{CoordinateSystem, DatumKind},
    vector::{Vector2, Vector3},
};

pub type V3 = [f64; 3];

pub fn v3(v: &Vector3) -> V3 {
    v.to_array().map(|x| x.to_f64())
}

fn v2(v: &Vector2) -> [f64; 2] {
    v.to_array().map(|x| x.to_f64())
}

/// A frame: `(x, y, z)` in it lies at `origin + x u + y v + z w`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Frame {
    pub origin: V3,
    pub u: V3,
    pub v: V3,
    pub w: V3,
}

impl Frame {
    pub fn of(frame: &CoordinateSystem) -> Self {
        Self {
            origin: v3(frame.origin()),
            u: v3(frame.u()),
            v: v3(frame.v()),
            w: v3(frame.w()),
        }
    }

    pub const WORLD: Frame = Frame {
        origin: [0.0; 3],
        u: [1.0, 0.0, 0.0],
        v: [0.0, 1.0, 0.0],
        w: [0.0, 0.0, 1.0],
    };

    /// The point `(x, y, z)` of the frame, in the world.
    pub fn point(&self, p: V3) -> V3 {
        std::array::from_fn(|k| {
            self.origin[k] + p[0] * self.u[k] + p[1] * self.v[k] + p[2] * self.w[k]
        })
    }

    /// The direction `d` of the frame, in the world.
    pub fn direction(&self, d: V3) -> V3 {
        std::array::from_fn(|k| d[0] * self.u[k] + d[1] * self.v[k] + d[2] * self.w[k])
    }

    /// The point `(x, y)` of its `u`/`v` plane.
    pub fn in_plane(&self, [x, y]: [f64; 2]) -> V3 {
        self.point([x, y, 0.0])
    }
}

/// A face's triangles, as the GPU takes them: three corners a triangle,
/// each with the surface's normal there.
#[derive(Debug, PartialEq)]
pub struct MeshData {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
}

impl MeshData {
    fn of(mesh: &FaceMesh) -> Self {
        let f = |p: &Vector3| v3(p).map(|x| x as f32);
        Self {
            positions: mesh.triangles.iter().flatten().map(f).collect(),
            normals: mesh.normals.iter().flatten().map(f).collect(),
        }
    }

    pub fn triangles(&self) -> usize {
        self.positions.len() / 3
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct FaceData {
    pub name: String,
    pub solid: Option<String>,
    pub feature: Option<String>,
    /// The key of its mesh (see [`Kept::meshes`]).
    pub mesh: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EdgeData {
    pub name: String,
    pub solid: Option<String>,
    pub faces: Vec<String>,
    pub polyline: Vec<V3>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VertexData {
    pub name: String,
    pub solid: Option<String>,
    pub faces: Vec<String>,
    pub at: V3,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CurveData {
    pub id: String,
    pub construction: bool,
    /// In the world.
    pub polyline: Vec<V3>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SketchData {
    pub name: String,
    pub target: String,
    pub curves: Vec<CurveData>,
    /// Its points, by id, in the world.
    pub points: Vec<(String, V3)>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct DatumData {
    pub name: String,
    pub kind: DatumKind,
    pub frame: Frame,
}

#[derive(Clone, Debug, PartialEq)]
pub struct AnnotationData {
    pub polyline: Vec<V3>,
}

/// A part as drawn (see [`PartView`]), its faces' meshes by key.
#[derive(Clone, Debug, PartialEq)]
pub struct PartData {
    pub targets: Targets,
    pub vertices: Vec<VertexData>,
    pub edges: Vec<EdgeData>,
    pub faces: Vec<FaceData>,
    pub sketches: Vec<SketchData>,
    pub datums: Vec<DatumData>,
    pub annotations: Vec<AnnotationData>,
    pub solids: Vec<String>,
    pub center: V3,
    pub size: f64,
    /// `#rrggbb`, if it has a colour of its own.
    pub color: Option<String>,
}

impl PartData {
    pub fn of(view: &PartView) -> Self {
        Self {
            targets: Targets {
                model: view.model.clone(),
                datum_target: view.datum_target.clone(),
                sketches: (view.sketches.iter())
                    .map(|s| (s.target.clone(), s.name.clone()))
                    .collect(),
            },
            vertices: (view.vertices.iter())
                .map(|v| VertexData {
                    name: v.name.clone(),
                    solid: v.solid.clone(),
                    faces: v.faces.clone(),
                    at: v3(&v.at),
                })
                .collect(),
            edges: (view.edges.iter())
                .map(|e| EdgeData {
                    name: e.name.clone(),
                    solid: e.solid.clone(),
                    faces: e.faces.clone(),
                    polyline: e.polyline.iter().map(v3).collect(),
                })
                .collect(),
            faces: (view.faces.iter())
                .map(|f| FaceData {
                    name: f.name.clone(),
                    solid: f.solid.clone(),
                    feature: f.feature.clone(),
                    mesh: f.mesh.key.clone(),
                })
                .collect(),
            sketches: (view.sketches.iter())
                .map(|s| {
                    let plane = Frame::of(&s.plane);
                    SketchData {
                        name: s.name.clone(),
                        target: s.target.clone(),
                        curves: (s.curves.iter())
                            .map(|c| CurveData {
                                id: c.id.clone(),
                                construction: c.construction,
                                polyline: c
                                    .polyline
                                    .iter()
                                    .map(|p| plane.in_plane(v2(p)))
                                    .collect(),
                            })
                            .collect(),
                        points: (s.points.iter())
                            .map(|p| (p.id.clone(), plane.in_plane(v2(&p.at))))
                            .collect(),
                    }
                })
                .collect(),
            datums: (view.datums.iter())
                .map(|d| DatumData {
                    name: d.name.clone(),
                    kind: d.kind,
                    frame: Frame::of(&d.frame),
                })
                .collect(),
            annotations: (view.annotations.iter())
                .map(|a| AnnotationData {
                    polyline: a.polyline.iter().map(v3).collect(),
                })
                .collect(),
            solids: view.solids.clone(),
            center: v3(&view.extent.center),
            size: view.extent.size.to_f64(),
            color: view.color.clone(),
        }
    }

    pub fn empty() -> Self {
        Self::of(&PartView::new())
    }

    pub fn triangles(&self, meshes: &HashMap<String, Arc<MeshData>>) -> usize {
        (self.faces.iter())
            .filter_map(|f| meshes.get(&f.mesh))
            .map(|m| m.triangles())
            .sum()
    }
}

/// A part placed in the part drawn.
#[derive(Clone, Debug, PartialEq)]
pub struct InstanceData {
    pub name: String,
    /// The key of the component it is drawn from.
    pub component: String,
    pub frame: Frame,
}

impl InstanceData {
    fn of(instance: &ViewInstance) -> Self {
        Self {
            name: instance.name.clone(),
            component: instance.component.clone(),
            frame: Frame::of(&instance.frame),
        }
    }
}

/// What the editor sent of a scene (see [`SceneState`]), as plain data.
#[derive(Clone, Debug)]
pub struct SceneData {
    pub part: Arc<PartData>,
    pub hidden: Vec<String>,
    pub instances: Vec<InstanceData>,
    pub all: bool,
    pub removed: Vec<String>,
    pub components: BTreeMap<String, Arc<PartData>>,
    pub forgotten: Vec<String>,
    pub meshes: BTreeMap<String, Arc<MeshData>>,
    pub forgotten_meshes: Vec<String>,
    pub structure: Vec<StructureItem>,
}

impl SceneData {
    pub fn of(scene: &SceneState) -> Self {
        Self {
            part: Arc::new(PartData::of(&scene.part)),
            hidden: scene.hidden.clone(),
            instances: scene.instances.iter().map(InstanceData::of).collect(),
            all: scene.all,
            removed: scene.removed.clone(),
            components: (scene.components.iter())
                .map(|(key, view)| (key.clone(), Arc::new(PartData::of(view))))
                .collect(),
            forgotten: scene.forgotten.clone(),
            meshes: (scene.meshes.iter())
                .map(|(key, mesh)| (key.clone(), Arc::new(MeshData::of(mesh))))
                .collect(),
            forgotten_meshes: scene.forgotten_meshes.clone(),
            structure: scene.structure.clone(),
        }
    }
}

/// What is kept of the scenes sent: the part drawn, its placed parts by
/// name, and the components and meshes by key until they are forgotten.
#[derive(Clone, Debug)]
pub struct Kept {
    pub part: Arc<PartData>,
    pub hidden: Vec<String>,
    pub instances: BTreeMap<String, InstanceData>,
    pub components: HashMap<String, Arc<PartData>>,
    pub meshes: HashMap<String, Arc<MeshData>>,
    pub structure: Vec<StructureItem>,
    /// Counts every scene taken: what a viewer redraws by.
    pub version: u64,
}

impl Default for Kept {
    fn default() -> Self {
        Self {
            part: Arc::new(PartData::empty()),
            hidden: Vec::new(),
            instances: BTreeMap::new(),
            components: HashMap::new(),
            meshes: HashMap::new(),
            structure: Vec::new(),
            version: 0,
        }
    }
}

impl Kept {
    /// What is kept once `scene` is taken in.
    pub fn take(&mut self, scene: SceneData) {
        if scene.all {
            self.instances.clear();
            self.components.clear();
            self.meshes.clear();
        }
        for name in &scene.removed {
            self.instances.remove(name);
        }
        for instance in scene.instances {
            self.instances.insert(instance.name.clone(), instance);
        }
        self.components.extend(scene.components);
        for key in &scene.forgotten {
            self.components.remove(key);
        }
        self.meshes.extend(scene.meshes);
        for key in &scene.forgotten_meshes {
            self.meshes.remove(key);
        }
        self.part = scene.part;
        self.hidden = scene.hidden;
        self.structure = scene.structure;
        self.version += 1;
    }

    /// How many solids and placed parts are drawn: what framing goes by.
    pub fn solids(&self) -> usize {
        self.part.solids.len() + self.instances.len()
    }

    /// How many triangles are drawn, the placed parts' included.
    pub fn triangles(&self) -> usize {
        let placed: usize = (self.instances.values())
            .filter_map(|i| self.components.get(&i.component))
            .map(|c| c.triangles(&self.meshes))
            .sum();
        self.part.triangles(&self.meshes) + placed
    }
}
