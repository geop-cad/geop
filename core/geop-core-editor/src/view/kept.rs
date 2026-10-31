//! How a [`PartView`] is kept between processes (see
//! [`geop_core_program::kept`]): what a drawer drew of a part, and a placed
//! part as drawn, read back instead of meshed again. Written exactly, so
//! that what is picked in a view read back is what is picked in the view
//! drawn.
//!
//! Each face's mesh is put in the pool by itself, so that two views of
//! revisions of one model — and the same part drawn alone and placed —
//! write a mesh they have alike once; so is the view of each part placed in
//! it, by its component.
//!
//! A view with sketches is not kept: a sketch is drawn from what it is,
//! cheaply, and is drawn again ([`write`] refuses it).

use std::{collections::HashMap, rc::Rc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{CoordinateSystem, DatumComponent, DatumKind, Pose},
    scalars::Scalar,
    vector::{Vector, Vector3},
};
use geop_core_program::{
    kept::Pool,
    workspace::{base64, from_base64},
};
use serde::{Deserialize, Serialize};

use super::{
    FaceMesh, PartView, ViewAnnotation, ViewDatum, ViewEdge, ViewFace, ViewInstance, ViewVertex,
};
use crate::editing::Extent;

/// A [`PartView`] as it is written.
#[derive(Serialize, Deserialize)]
struct Written {
    model: String,
    vertices: Vec<Vertex>,
    edges: Vec<Edge>,
    faces: Vec<Face>,
    datum_target: String,
    datums: Vec<Datum>,
    annotations: Vec<Annotation>,
    solids: Vec<String>,
    instances: Vec<Instance>,
    #[serde(with = "geop_core_math::exact")]
    center: Vector3,
    #[serde(with = "geop_core_math::exact")]
    size: Scalar,
    color: Option<String>,
}

#[derive(Serialize, Deserialize)]
struct Vertex {
    name: String,
    solid: Option<String>,
    faces: Vec<String>,
    #[serde(with = "geop_core_math::exact")]
    at: Vector3,
}

#[derive(Serialize, Deserialize)]
struct Edge {
    name: String,
    solid: Option<String>,
    faces: Vec<String>,
    #[serde(with = "geop_core_math::exact")]
    polyline: Vec<Vector3>,
    ends: [String; 2],
}

/// A face, its mesh by its key in the pool.
#[derive(Serialize, Deserialize)]
struct Face {
    name: String,
    solid: Option<String>,
    feature: Option<String>,
    mesh: String,
}

/// A [`FaceMesh`] as it is put in the pool: its key, each corner its
/// triangles share once — its point and its normal, six numbers, the bits
/// of each in base64 — and each triangle by its corners' indices, in
/// base64 too. A mesh's numbers are sharp ([`FaceMesh::new`]): the bits of
/// one are all of it.
#[derive(Serialize, Deserialize)]
struct Mesh {
    key: String,
    corners: String,
    triangles: String,
}

impl Mesh {
    fn of(mesh: &FaceMesh) -> GeopResult<Self> {
        let mut index: HashMap<[u64; 6], u32> = HashMap::new();
        let (mut corners, mut triangles) = (Vec::new(), Vec::new());
        for (points, normals) in mesh.triangles.iter().zip(&mesh.normals) {
            for (p, n) in points.iter().zip(normals) {
                let mut bits = [0; 6];
                let numbers = p.to_array().into_iter().chain(n.to_array());
                for (bit, x) in bits.iter_mut().zip(numbers) {
                    if x.lo.to_bits() != x.hi.to_bits() {
                        return Err(GeopError::new("a mesh with a corner not sharp is not kept"));
                    }
                    *bit = x.lo.to_bits();
                }
                let next = index.len() as u32;
                let at = *index.entry(bits).or_insert_with(|| {
                    corners.extend(bits.iter().flat_map(|b| b.to_le_bytes()));
                    next
                });
                triangles.extend(at.to_le_bytes());
            }
        }
        Ok(Self {
            key: mesh.key.clone(),
            corners: base64(&corners),
            triangles: base64(&triangles),
        })
    }

    fn mesh(self) -> GeopResult<FaceMesh> {
        let corner_bytes = from_base64(&self.corners)?;
        let corners: Vec<[Scalar; 6]> = (corner_bytes.as_chunks::<48>().0.iter())
            .map(|corner| {
                std::array::from_fn(|k| {
                    let bytes = corner.get(8 * k..8 * k + 8).unwrap_or_default();
                    let bits = u64::from_le_bytes(bytes.try_into().unwrap_or_default());
                    Scalar::from_f64(f64::from_bits(bits))
                })
            })
            .collect();
        let corner = |bytes: &[u8]| -> GeopResult<&[Scalar; 6]> {
            let at = u32::from_le_bytes(bytes.try_into().unwrap_or_default());
            (corners.get(at as usize)).ok_or_else(|| broken("a mesh's corners"))
        };
        let (mut triangles, mut normals) = (Vec::new(), Vec::new());
        let triangle_bytes = from_base64(&self.triangles)?;
        for triangle in triangle_bytes.as_chunks::<12>().0 {
            let [a, b, c] = [0, 4, 8].map(|k| corner(triangle.get(k..k + 4).unwrap_or_default()));
            let [a, b, c] = [a?, b?, c?];
            let point = |c: &[Scalar; 6]| Vector3::from_array([c[0], c[1], c[2]]);
            let normal = |c: &[Scalar; 6]| Vector3::from_array([c[3], c[4], c[5]]);
            triangles.push([point(a), point(b), point(c)]);
            normals.push([normal(a), normal(b), normal(c)]);
        }
        Ok(FaceMesh {
            key: self.key,
            triangles,
            normals,
        })
    }
}

#[derive(Serialize, Deserialize)]
struct Datum {
    name: String,
    kind: DatumKind,
    #[serde(with = "geop_core_math::exact")]
    frame: CoordinateSystem,
    components: Vec<(DatumComponent, String)>,
}

#[derive(Serialize, Deserialize)]
struct Annotation {
    name: String,
    label: String,
    #[serde(with = "geop_core_math::exact")]
    polyline: Vec<Vector3>,
}

/// A part placed, its part's view by its key in the pool.
#[derive(Serialize, Deserialize)]
struct Instance {
    name: String,
    component: String,
    #[serde(with = "geop_core_math::exact")]
    pose: Pose,
    view: String,
    state: Option<String>,
    fixed: bool,
}

/// `view` written, its meshes and the views of its placed parts put in
/// `pool`. A view with sketches is refused.
pub fn write(view: &PartView, pool: &mut Pool) -> GeopResult<String> {
    text(&written(view, pool, &mut HashMap::new())?)
}

/// The view `text` writes, its parts taken from `pool`.
pub fn read(text: &str, pool: &Pool) -> GeopResult<PartView> {
    let written: Written = serde_json::from_str(text)
        .map_err(|e| GeopError::new(format!("reading a kept view: {e}")))?;
    view_of(written, pool, &mut HashMap::new())
}

/// Puts `view` in `pool`, as [`write`] writes it, and gives what it is
/// taken by ([`get`]): a view kept as a part of another value.
pub fn put(view: &PartView, pool: &mut Pool) -> GeopResult<String> {
    let written = written(view, pool, &mut HashMap::new())?;
    pool.put(&written)
}

/// The view [`put`] put as `key` in `pool`: read once, however many
/// records read it — the drawing of a part placed a thousand times.
pub fn get(key: &str, pool: &Pool) -> GeopResult<Rc<PartView>> {
    pool.shared(key, || view_of(pool.get(key)?, pool, &mut HashMap::new()))
}

fn text<T: Serialize>(value: &T) -> GeopResult<String> {
    serde_json::to_string(value).map_err(|e| GeopError::new(format!("writing a kept view: {e}")))
}

/// `view` as it is written, its parts put in `pool` — each placed part's
/// view once, its key kept in `placed` by the view.
fn written(
    view: &PartView,
    pool: &mut Pool,
    placed: &mut HashMap<*const PartView, String>,
) -> GeopResult<Written> {
    if !view.sketches.is_empty() {
        return Err(GeopError::new(
            "a view with sketches is not kept: its sketches are drawn again",
        ));
    }
    let faces = (view.faces.iter())
        .map(|face| {
            Ok(Face {
                name: face.name.clone(),
                solid: face.solid.clone(),
                feature: face.feature.clone(),
                mesh: pool.put(&Mesh::of(&face.mesh)?)?,
            })
        })
        .collect::<GeopResult<_>>()?;
    let instances = (view.instances.iter())
        .map(|instance| {
            let key = match placed.get(&Rc::as_ptr(&instance.view)) {
                Some(key) => key.clone(),
                None => {
                    let inner = written(&instance.view, pool, placed)?;
                    let key = pool.put(&inner)?;
                    placed.insert(Rc::as_ptr(&instance.view), key.clone());
                    key
                }
            };
            Ok(Instance {
                name: instance.name.clone(),
                component: instance.component.clone(),
                pose: instance.pose,
                view: key,
                state: instance.state.clone(),
                fixed: instance.fixed,
            })
        })
        .collect::<GeopResult<_>>()?;
    Ok(Written {
        model: view.model.clone(),
        vertices: (view.vertices.iter())
            .map(|v| Vertex {
                name: v.name.clone(),
                solid: v.solid.clone(),
                faces: v.faces.clone(),
                at: v.at,
            })
            .collect(),
        edges: (view.edges.iter())
            .map(|e| Edge {
                name: e.name.clone(),
                solid: e.solid.clone(),
                faces: e.faces.clone(),
                polyline: e.polyline.clone(),
                ends: e.ends.clone(),
            })
            .collect(),
        faces,
        datum_target: view.datum_target.clone(),
        datums: (view.datums.iter())
            .map(|d| Datum {
                name: d.name.clone(),
                kind: d.kind,
                frame: d.frame.clone(),
                components: d.components.clone(),
            })
            .collect(),
        annotations: (view.annotations.iter())
            .map(|a| Annotation {
                name: a.name.clone(),
                label: a.label.clone(),
                polyline: a.polyline.clone(),
            })
            .collect(),
        solids: view.solids.clone(),
        instances,
        center: view.extent.center,
        size: view.extent.size,
        color: view.color.clone(),
    })
}

/// The view `written` writes, its parts taken from `pool` — each placed
/// part's view once, by its key in `placed`, shared by every instance of it.
fn view_of(
    written: Written,
    pool: &Pool,
    placed: &mut HashMap<String, Rc<PartView>>,
) -> GeopResult<PartView> {
    let faces = (written.faces.into_iter())
        .map(|face| {
            let mesh: Mesh = pool.get(&face.mesh)?;
            Ok(ViewFace {
                name: face.name,
                solid: face.solid,
                feature: face.feature,
                mesh: Rc::new(mesh.mesh()?),
            })
        })
        .collect::<GeopResult<_>>()?;
    let mut instances = Vec::new();
    for instance in written.instances {
        let view = match placed.get(&instance.view) {
            Some(view) => view.clone(),
            None => {
                let inner: Written = pool.get(&instance.view)?;
                let view = Rc::new(view_of(inner, pool, placed)?);
                placed.insert(instance.view.clone(), view.clone());
                view
            }
        };
        let origin = CoordinateSystem::world_at(Vector::zero());
        instances.push(ViewInstance {
            frame: instance.pose.motion().apply_frame(&origin)?,
            name: instance.name,
            component: instance.component,
            pose: instance.pose,
            view,
            state: instance.state,
            fixed: instance.fixed,
            color: None,
        });
    }
    Ok(PartView {
        model: written.model,
        vertices: (written.vertices.into_iter())
            .map(|v| ViewVertex {
                name: v.name,
                solid: v.solid,
                faces: v.faces,
                at: v.at,
            })
            .collect(),
        edges: (written.edges.into_iter())
            .map(|e| ViewEdge {
                name: e.name,
                solid: e.solid,
                faces: e.faces,
                polyline: e.polyline,
                ends: e.ends,
            })
            .collect(),
        faces,
        sketches: Vec::new(),
        datum_target: written.datum_target,
        datums: (written.datums.into_iter())
            .map(|d| ViewDatum {
                name: d.name,
                kind: d.kind,
                frame: d.frame,
                components: d.components,
            })
            .collect(),
        annotations: (written.annotations.into_iter())
            .map(|a| ViewAnnotation {
                name: a.name,
                label: a.label,
                polyline: a.polyline,
            })
            .collect(),
        solids: written.solids,
        instances,
        extent: Extent {
            center: written.center,
            size: written.size,
        },
        color: written.color,
        faces_tree: std::cell::RefCell::default(),
    })
}

fn broken(what: &str) -> GeopError {
    GeopError::new(format!("reading a kept view: {what} are not all there"))
}
