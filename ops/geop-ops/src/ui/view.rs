//! [`PartView`]: a part as the viewport draws it, and picking entities of it
//! with a [`Pointer`].
//!
//! A part is drawn from its rasterization — sampled edges, triangulated
//! faces, outlined sketches — and a pick tests against exactly that, never
//! re-deriving geometry of its own, so a pick cannot disagree with what is
//! on screen. The same goes for what the viewer draws besides the part: the
//! datums are laid out here, by the sizes the viewer draws them with, and
//! picked by the same rules.
//!
//! The parts placed in a part are drawn by reference: each placed part —
//! however deep — is listed once, with where it is ([`ViewInstance`]), and
//! drawn from its component's own view, built once and shared by every
//! instance of it. Moving a placed part changes where it is, not what is
//! drawn. A pick moves the pointer into each placed part's own frame and
//! names what it hits as the part it is placed in names it — behind the
//! instance's name (see [`crate::operation::INSTANCE_SEPARATOR`]) — so a
//! pick of one is a reference like any other.

use std::sync::Arc;

use geop_core_math::{
    geop_error::GeopResult,
    polygon::loops_contain,
    primitives::{CoordinateSystem, DatumComponent, DatumKind, FrameAxis, Motion, Pose, Ray},
    scalars::{Scalar, as_f64},
    vector::{Vector2, Vector3},
};
use geop_core_sketch::{CurveId, PointId, profile::curve_polyline};
use geop_core_topology::{EdgeId, FaceId, Model, SolidId};
use geop_ops_rasterize::rasterize;
use serde::Serialize;

use super::{Pointer, hit::nearer};
use crate::Design;
use crate::{
    Part,
    operation::{Aspects, EntityRef, INSTANCE_SEPARATOR, Role},
};

/// Samples per edge and per parametric direction of a face. A face's grid is
/// refined further where its own curvature asks for it (see
/// `geop_ops_rasterize::grid`), so this is the floor, not the ceiling.
const RESOLUTION: usize = 24;

/// How long a frame datum's axes are, in reaches: it is drawn at a constant
/// size on screen, with its origin ball, its axes and the squares of its
/// planes laid out in fractions of that, in [`PartView::pick`] as in the
/// viewer.
pub const FRAME: f64 = 10.0;

/// What a pick for a [`Role::Frame`] takes something of: anything that gives
/// one (see [`Aspects::frame_on`]) — the roles of the entities it is
/// picked among are what they can be, not what a frame is.
const MOUNTABLE: [Role; 6] = [
    Role::Frame,
    Role::Point,
    Role::Line,
    Role::Circle,
    Role::Round,
    Role::Plane,
];

/// How near an end of a straight edge a pick for a frame is taken to be at
/// it: that part of its length from it.
const END_OF_A_LINE: f64 = 0.2;

/// How near a corner of a face, in reaches of the pointer, a pick for a
/// frame on it is taken to be at it, and how near a side: the corner first.
const NEAR_CORNER: f64 = 4.0;
const NEAR_SIDE: f64 = 3.0;

/// An entity a pointer is over, where, and how far along its ray.
#[derive(Clone, Debug, PartialEq)]
pub struct PartHit<S: Scalar> {
    pub entity: EntityRef,
    pub point: Vector3<S>,
    pub t: S,
}

/// A vertex of the part, as drawn.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewVertex<S: Scalar> {
    pub name: String,
    /// The solid it is a corner of.
    pub solid: Option<String>,
    /// The faces it is a corner of. Of no solid, it is hidden with all of
    /// them; a face hit does not hide it (see [`PartView::pick`]).
    pub faces: Vec<String>,
    /// The 3-D sketch it is a point of — a vertex of the wire the sketch
    /// is built as (see [`Part::add_sketch3d`]).
    pub sketch: Option<String>,
    pub at: Vector3<S>,
}

/// The roles `entity` can fill in `part` (see [`Aspects`]): none for one
/// that does not resolve.
fn roles_of<S: Scalar>(entity: &EntityRef, part: &Part<S>) -> Vec<Role> {
    Aspects::of(entity, part)
        .map(|g| g.roles())
        .unwrap_or_default()
}

/// An edge of the part, as drawn: its curve sampled into a polyline.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewEdge<S: Scalar> {
    pub name: String,
    /// The solid it bounds.
    pub solid: Option<String>,
    /// The faces it bounds. Of no solid, it is hidden with all of them; a
    /// face hit does not hide it (see [`PartView::pick`]).
    pub faces: Vec<String>,
    /// The 3-D sketch it is a curve of — an edge of the wire the sketch is
    /// built as (see [`Part::add_sketch3d`]): picked as a curve, or for
    /// the whole sketch where a path is asked for.
    pub sketch: Option<String>,
    pub polyline: Vec<Vector3<S>>,
    /// The vertices it runs between, by name.
    #[serde(skip)]
    pub ends: [String; 2],
    /// What it can be picked as: a curve, an edge of a face, and a line or
    /// a circle if it is one.
    #[serde(skip)]
    pub roles: Vec<Role>,
}

/// A face of the part, as drawn: triangulated, with the surface's normal at
/// each corner.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewFace<S: Scalar> {
    pub name: String,
    /// The solid the face bounds.
    pub solid: Option<String>,
    /// The step whose feature made it, if one did (see
    /// [`crate::part::FeatureFaces::of`]): what picking it as a feature
    /// picks.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feature: Option<String>,
    pub triangles: Vec<[Vector3<S>; 3]>,
    pub normals: Vec<[Vector3<S>; 3]>,
    /// What it can be picked as: a plane if it is flat, round if it turns
    /// around an axis.
    #[serde(skip)]
    pub roles: Vec<Role>,
}

/// A curve of a sketch, as drawn, in the sketch's plane.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewCurve<S: Scalar> {
    pub id: CurveId,
    pub construction: bool,
    pub polyline: Vec<Vector2<S>>,
    /// What it can be picked as: a line, if it is one.
    #[serde(skip)]
    pub roles: Vec<Role>,
}

/// A point of a sketch, as drawn, in the sketch's plane.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewSketchPoint<S: Scalar> {
    pub id: PointId,
    pub at: Vector2<S>,
}

/// A sketch of the part, as drawn: its plane, its closed regions (each an
/// outer loop and its holes), its curves and its points, in the plane's
/// `u`/`v` coordinates.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewSketch<S: Scalar> {
    pub name: String,
    pub plane: CoordinateSystem<S>,
    #[serde(skip)]
    pub regions: Vec<Vec<Vec<Vector2<S>>>>,
    pub curves: Vec<ViewCurve<S>>,
    pub points: Vec<ViewSketchPoint<S>>,
}

/// A datum of the part.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewDatum<S: Scalar> {
    pub name: String,
    pub kind: DatumKind,
    pub frame: CoordinateSystem<S>,
}

/// Something an extension of the part draws (see
/// [`crate::part::Extension::annotations`]), and what it is called.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewAnnotation<S: Scalar> {
    pub name: String,
    pub label: String,
    pub polyline: Vec<Vector3<S>>,
}

/// Where the drawing is and how big: the center and diagonal (at least 1)
/// of the box around it — for an empty part, [`EMPTY_EXTENT`] around the
/// origin. Datum planes and axes, which are endless, are drawn this big
/// around the point of them nearest the center.
/// How big an empty part's drawing is: what a viewer frames, and its datum
/// planes' size, before anything is drawn. Lengths are millimetres, and
/// the parts of a robot are 10 to 500 mm.
pub const EMPTY_EXTENT: f64 = 100.0;

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct Extent<S: Scalar> {
    pub center: Vector3<S>,
    #[serde(with = "as_f64")]
    pub size: S,
}

/// A part placed in the part drawn — directly or in a part placed in it —
/// drawn as its part's own view ([`Part::view`]) moved to where it is.
/// Serialized, a viewer gets the part by its key ([`Instance::key`]) and
/// where to draw it.
#[derive(Clone, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct ViewInstance<S: Scalar> {
    /// Its name in the part drawn: `bolt`, or `asm/bolt` for one placed in
    /// a placed part.
    pub name: String,
    /// The key of its part.
    pub component: String,
    /// Where it is: its own frame in the part drawn.
    pub frame: CoordinateSystem<S>,
    #[serde(skip)]
    pose: Pose<S>,
    #[serde(skip)]
    source: Arc<Part<S>>,
    /// The parameter of the part drawn its pose is — `None` for a copy of
    /// a pattern, which goes where the pattern puts it.
    #[serde(skip)]
    parameter: Option<String>,
    /// Whether no mate moves it.
    #[serde(skip)]
    fixed: bool,
}

impl<S: Scalar> std::fmt::Debug for ViewInstance<S> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ViewInstance({}, {}, {:?})",
            self.name, self.component, self.pose
        )
    }
}

impl<S: Scalar> ViewInstance<S> {
    /// Its part.
    pub fn part(&self) -> &Arc<Part<S>> {
        &self.source
    }

    /// Where it is in the part drawn.
    pub fn pose(&self) -> &Pose<S> {
        &self.pose
    }

    /// The parameter of the part drawn its pose is, if it is one.
    pub fn parameter(&self) -> Option<&str> {
        self.parameter.as_deref()
    }
}

/// A part as the viewport draws it, every entity by name — and, serialized,
/// what a viewer draws.
#[derive(Clone, Debug, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct PartView<S: Scalar> {
    pub vertices: Vec<ViewVertex<S>>,
    pub edges: Vec<ViewEdge<S>>,
    pub faces: Vec<ViewFace<S>>,
    pub sketches: Vec<ViewSketch<S>>,
    pub datums: Vec<ViewDatum<S>>,
    /// What its extensions draw besides its topology: cosmetic threads.
    pub annotations: Vec<ViewAnnotation<S>>,
    /// The part's solids, oldest first.
    pub solids: Vec<String>,
    /// Every part placed in it, and in those, however deep. Not
    /// serialized: a viewer is sent them apart, as they change, by name
    /// (see `geop_cad_base::editor::SceneState`).
    #[serde(skip)]
    pub instances: Vec<ViewInstance<S>>,
    pub extent: Extent<S>,
    /// The part's colour, `#rrggbb` (see [`crate::parameters::COLOR`]);
    /// none for the viewer's own.
    pub color: Option<String>,
}

/// One part of what a pick tests: the part drawn itself, or a placed part,
/// with the pointer moved into its own frame.
struct Layer<'v, S: Scalar> {
    view: &'v PartView<S>,
    pointer: Pointer<S>,
    /// Whether the pointer comes near enough the box around its view
    /// ([`PartView::extent`]) to hit any of its vertices, edges or faces:
    /// only its datums can be hit otherwise — a frame is drawn at a size of
    /// its own, a plane or an axis as large as the whole drawing.
    near: bool,
    /// Where it is, and back; `None` for the part drawn itself.
    motion: Option<(Motion<S>, Motion<S>)>,
    /// The instance's name, for a placed part.
    instance: Option<&'v str>,
}

impl<S: Scalar> Layer<'_, S> {
    /// `entity` of this layer as the part drawn names it.
    fn name(&self, entity: EntityRef) -> EntityRef {
        match self.instance {
            Some(instance) => entity.in_instance(instance),
            None => entity,
        }
    }

    /// The point `p` of this layer where the part drawn has it.
    fn world(&self, p: Vector3<S>) -> Vector3<S> {
        match &self.motion {
            Some((there, _)) => there.apply(&p),
            None => p,
        }
    }

    /// The point `p` of the part drawn in this layer's own frame.
    fn local(&self, p: Vector3<S>) -> Vector3<S> {
        match &self.motion {
            Some((_, back)) => back.apply(&p),
            None => p,
        }
    }

    /// `hit`, of this layer, as a hit of the part drawn.
    fn hit(&self, hit: PartHit<S>) -> PartHit<S> {
        PartHit {
            entity: self.name(hit.entity),
            point: self.world(hit.point),
            t: hit.t,
        }
    }
}

/// The solid that owns `face`: a face is part of exactly one shell, and a
/// shell of at most one solid — none, for a sheet.
pub fn solid_of_face<S: Scalar>(model: &Model<S>, face: FaceId) -> Option<SolidId> {
    model
        .get_face(face)
        .ok()
        .and_then(|f| model.get_shell(f.shell).ok())
        .and_then(|s| s.solid)
}

/// The solid `edge` bounds: that of a face it runs along.
fn solid_of_edge<S: Scalar>(model: &Model<S>, edge: EdgeId) -> Option<SolidId> {
    let coedge = *model.coedges_of_edge(edge).first()?;
    solid_of_face(model, model.get_coedge(coedge).ok()?.face)
}

/// The name of the solid `entity` — a face, an edge or a vertex of `part` —
/// bounds, if any: what [`EntityRef::lies_in`] needs to know of the part.
pub(crate) fn solid_bounded_by<S: Scalar>(part: &Part<S>, entity: &EntityRef) -> Option<String> {
    let model = part.topology();
    let solid = match entity {
        EntityRef::Face { name } => solid_of_face(model, part.face_id(name).ok()?),
        EntityRef::Edge { name } => solid_of_edge(model, part.edge_id(name).ok()?),
        EntityRef::Vertex { name } => {
            let crate::RefId::Vertex(v) = part.id_of(name)? else {
                return None;
            };
            let (&edge, _) = model
                .edges
                .iter()
                .find(|(_, e)| e.start_vertex == v || e.end_vertex == v)?;
            solid_of_edge(model, edge)
        }
        _ => None,
    }?;
    part.name_of(solid).map(str::to_string)
}

impl<S: Scalar> PartView<S> {
    /// Nothing drawn, reaching as far as `extent`: what a viewer is sent
    /// while a step is edited on a sheet of its own (see
    /// [`crate::ui::Form::sheet`]), which it frames.
    pub fn blank(extent: Extent<S>) -> Self {
        Self {
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            sketches: Vec::new(),
            datums: Vec::new(),
            annotations: Vec::new(),
            solids: Vec::new(),
            instances: Vec::new(),
            extent,
            color: None,
        }
    }

    /// `part` as drawn.
    pub fn of(part: &Part<S>) -> GeopResult<Self> {
        let model = part.topology();
        let raster = rasterize(model, RESOLUTION)?;
        let name = |id: crate::RefId| part.name_of(id).unwrap_or_default().to_string();
        let uv = |polyline: Vec<Vector2<Design>>| -> Vec<Vector2<S>> {
            polyline.iter().map(|p| p.map(|c| c.cast())).collect()
        };

        let mut vertices: Vec<_> = raster.vertices.iter().collect();
        vertices.sort_by_key(|(id, _)| id.0);
        let mut edges: Vec<_> = raster.edges.iter().collect();
        edges.sort_by_key(|(id, _)| id.0);
        let mut faces: Vec<_> = raster.faces.iter().collect();
        faces.sort_by_key(|(id, _)| id.0);
        let mut solids: Vec<SolidId> = model.solids.keys().copied().collect();
        solids.sort_by_key(|s| s.0);

        let sketches = part
            .sketches()
            .map(|(id, placed)| {
                let sketch = name(id.into());
                let s = &placed.sketch;
                let positions = s.positions();
                // A sketch whose curves form no region is still a sketch,
                // hit on its curves.
                let regions = s
                    .regions()
                    .and_then(|regions| {
                        regions
                            .iter()
                            .map(|r| {
                                std::iter::once(&r.outer)
                                    .chain(&r.holes)
                                    .map(|l| Ok(uv(l.polyline(s)?)))
                                    .collect()
                            })
                            .collect()
                    })
                    .unwrap_or_default();
                Ok(ViewSketch {
                    name: sketch.clone(),
                    plane: placed.plane.clone(),
                    regions,
                    // What it was only given to draw against — its own
                    // axes, its origin, what it projected as construction
                    // geometry — is not drawn once it is done.
                    curves: s
                        .curves
                        .iter()
                        .filter(|(_, c)| !(c.fixed && c.construction))
                        .map(|(&id, c)| {
                            Ok(ViewCurve {
                                id,
                                construction: c.construction,
                                polyline: uv(curve_polyline(s, id)?),
                                roles: roles_of(
                                    &EntityRef::SketchCurve {
                                        sketch: sketch.clone(),
                                        curve: id,
                                    },
                                    part,
                                ),
                            })
                        })
                        .collect::<GeopResult<_>>()?,
                    points: positions
                        .iter()
                        .filter(|(id, _)| !s.points[id].fixed)
                        .map(|(&id, p)| ViewSketchPoint {
                            id,
                            at: p.map(|c| c.cast()),
                        })
                        .collect(),
                })
            })
            .collect::<GeopResult<_>>()?;

        // The 3-D sketch each edge and vertex of a wire is part of.
        let mut sketch_of = std::collections::HashMap::new();
        for (id, _) in part.sketches3d() {
            if let Some(wire) = part.sketch3d_wire(id)? {
                let wire = model.get_wire(wire)?;
                let sketch = name(id.into());
                for &v in &wire.vertices {
                    sketch_of.insert(crate::RefId::from(v), sketch.clone());
                }
                for &e in &wire.edges {
                    sketch_of.insert(crate::RefId::from(e), sketch.clone());
                }
            }
        }

        // A vertex is a corner of the solid an edge at it bounds, and of the
        // faces those edges bound.
        let mut corner_of = std::collections::HashMap::new();
        let mut faces_of_edge = std::collections::HashMap::new();
        let mut faces_at_corner: std::collections::HashMap<_, Vec<String>> = Default::default();
        for (&id, edge) in &model.edges {
            if let Some(solid) = solid_of_edge(model, id) {
                corner_of.insert(edge.start_vertex, solid);
                corner_of.insert(edge.end_vertex, solid);
            }
            let mut faces: Vec<String> = model
                .coedges_of_edge(id)
                .into_iter()
                .filter_map(|c| model.get_coedge(c).ok())
                .map(|c| name(c.face.into()))
                .collect();
            faces.sort();
            faces.dedup();
            for v in [edge.start_vertex, edge.end_vertex] {
                let at = faces_at_corner.entry(v).or_default();
                at.extend(faces.iter().cloned());
                at.sort();
                at.dedup();
            }
            faces_of_edge.insert(id, faces);
        }
        let features = part.feature_faces();
        let mut view = PartView {
            vertices: vertices
                .into_iter()
                .map(|(&id, p)| ViewVertex {
                    name: name(id.into()),
                    solid: corner_of.get(&id).map(|&s| name(s.into())),
                    faces: faces_at_corner.get(&id).cloned().unwrap_or_default(),
                    sketch: sketch_of.get(&id.into()).cloned(),
                    at: *p,
                })
                .collect(),
            edges: edges
                .into_iter()
                .map(|(&id, polyline)| {
                    let edge = name(id.into());
                    let ends = model
                        .get_edge(id)
                        .map(|e| [e.start_vertex, e.end_vertex].map(|v| name(v.into())))
                        .unwrap_or_default();
                    ViewEdge {
                        ends,
                        solid: solid_of_edge(model, id).map(|s| name(s.into())),
                        faces: faces_of_edge.get(&id).cloned().unwrap_or_default(),
                        sketch: sketch_of.get(&id.into()).cloned(),
                        roles: roles_of(&EntityRef::Edge { name: edge.clone() }, part),
                        name: edge,
                        polyline: polyline.clone(),
                    }
                })
                .collect(),
            faces: faces
                .into_iter()
                .map(|(&id, tris)| {
                    let face = name(id.into());
                    ViewFace {
                        roles: roles_of(&EntityRef::Face { name: face.clone() }, part),
                        feature: features.of(&face).map(str::to_string),
                        name: face,
                        solid: solid_of_face(model, id).map(|s| name(s.into())),
                        triangles: tris.iter().map(|t| [t.a, t.b, t.c]).collect(),
                        normals: tris
                            .iter()
                            .map(|t| t.vertex_normals.unwrap_or([t.normal; 3]))
                            .collect(),
                    }
                })
                .collect(),
            sketches,
            datums: part
                .datums()
                .map(|(id, datum)| ViewDatum {
                    name: name(id.into()),
                    kind: datum.kind,
                    frame: datum.frame.clone(),
                })
                .collect(),
            annotations: part
                .annotations()?
                .into_iter()
                .map(|annotation| ViewAnnotation {
                    name: annotation.name,
                    label: annotation.label,
                    polyline: annotation.polyline,
                })
                .collect(),
            solids: solids.into_iter().map(|s| name(s.into())).collect(),
            instances: Vec::new(),
            extent: Extent {
                center: Vector3::zero(),
                size: S::ONE,
            },
            color: part.color().map(str::to_string),
        };
        // The views of the parts placed, each drawn once: natively, the
        // distinct ones side by side — each is its own part, and
        // [`Part::view`] keeps the first drawn.
        #[cfg(not(target_arch = "wasm32"))]
        {
            use rayon::prelude::*;
            let mut distinct: Vec<&Arc<Part<S>>> = Vec::new();
            for (_, instance) in part.instances() {
                if !distinct.iter().any(|p| Arc::ptr_eq(p, &instance.part)) {
                    distinct.push(&instance.part);
                }
            }
            distinct.par_iter().for_each(|placed| {
                // A view that cannot be drawn fails again below, saying why.
                let _ = placed.view();
            });
        }
        for (id, instance) in part.instances() {
            let instance_name = name(id.into());
            view.add_instance(
                part,
                instance_name.clone(),
                instance.pose,
                instance.key(),
                &instance.part,
                instance.parameter.as_ref().map(|p| p.name.clone()),
            )?;
            // Those placed in it, where it puts it: its view has them all
            // already, however deep — their poses parameters of this part's
            // state.
            for nested in &instance.part.view()?.instances {
                view.add_instance(
                    part,
                    format!("{instance_name}{INSTANCE_SEPARATOR}{}", nested.name),
                    instance.pose.compose(&nested.pose),
                    nested.component.clone(),
                    &nested.source,
                    nested
                        .parameter
                        .as_ref()
                        .map(|p| format!("{instance_name}{INSTANCE_SEPARATOR}{p}")),
                )?;
            }
        }
        view.extent = view.measure()?;
        Ok(view)
    }

    /// Adds the part `source`, placed as `name` — a path in `part`, the part
    /// drawn — at `pose`.
    fn add_instance(
        &mut self,
        part: &Part<S>,
        name: String,
        pose: Pose<S>,
        key: String,
        source: &Arc<Part<S>>,
        parameter: Option<String>,
    ) -> GeopResult<()> {
        self.instances.push(ViewInstance {
            fixed: part.is_fixed(&name),
            name,
            component: key,
            frame: pose
                .motion()
                .apply_frame(&CoordinateSystem::world_at(Vector3::zero()))?,
            pose,
            source: source.clone(),
            parameter,
        });
        Ok(())
    }

    /// Whether `pointer` could hit any of its vertices, edges or faces —
    /// within one reach of them, as a pick asks: whether it passes within
    /// one reach of the ball around the box of everything drawn
    /// ([`Extent`]), the reach taken where the ball ends furthest along
    /// the ray. A pick tests every triangle of a part it comes near, so for
    /// a part placed hundreds of times, this is what it can skip.
    fn near(&self, pointer: &Pointer<S>) -> bool {
        let Ok(radius) = self.extent.size.div(S::TWO) else {
            return true;
        };
        let (dist, t) = pointer.ray.distance_to_point(&self.extent.center);
        !dist
            .sub(radius)
            .definitely_greater(pointer.reach_at(1.0, t.add(radius)))
    }

    /// The part drawn itself, then every placed part, each with `pointer`
    /// moved into its frame.
    fn layers(&self, pointer: &Pointer<S>) -> GeopResult<Vec<Layer<'_, S>>> {
        let mut layers = vec![Layer {
            view: self,
            pointer: *pointer,
            near: true,
            motion: None,
            instance: None,
        }];
        for instance in &self.instances {
            let back = instance.pose.inverse().motion();
            let ray = &pointer.ray;
            let Ok(ray) = Ray::try_new(back.apply(ray.origin()), back.rotate(ray.dir())) else {
                continue;
            };
            let view = instance.source.view()?;
            let pointer = Pointer {
                ray,
                reach: pointer.reach,
            };
            layers.push(Layer {
                near: view.near(&pointer),
                view,
                pointer,
                motion: Some((instance.pose.motion(), back)),
                instance: Some(&instance.name),
            });
        }
        Ok(layers)
    }

    /// The placed part a drag at `pointer` moves, and how far along the ray
    /// it is hit: the one whose face the ray enters first — or, for a part
    /// placed in one placed rigid, the innermost one around it whose pose is
    /// a parameter of the part drawn: what moves it. None for a fixed one:
    /// no drag moves it.
    pub fn part_to_drag(&self, pointer: &Pointer<S>) -> Option<(&ViewInstance<S>, S)> {
        let layers = self.layers(pointer).ok()?;
        // The part's own faces hide what is behind them too.
        let (t, hit) = layers
            .iter()
            .filter(|l| l.near)
            .filter_map(|l| Some((l.view.pick_face(&l.pointer)?.0, l.instance)))
            .min_by(|a, b| nearer(a.0, b.0))?;
        let hit = hit?;
        let moved = self
            .instances
            .iter()
            .filter(|i| i.parameter.is_some())
            .filter(|i| {
                i.name == hit || hit.starts_with(&format!("{}{INSTANCE_SEPARATOR}", i.name))
            })
            .max_by_key(|i| i.name.len())?;
        (!moved.fixed).then_some((moved, t))
    }

    /// How far along `pointer`'s ray it first enters the placed part
    /// `instance`, if it does.
    pub fn pick_instance(&self, instance: &str, pointer: &Pointer<S>) -> Option<S> {
        let layers = self.layers(pointer).ok()?;
        let layer = layers.iter().find(|l| l.instance == Some(instance))?;
        layer
            .near
            .then(|| layer.view.pick_face(&layer.pointer))?
            .map(|(t, _)| t)
    }

    /// The box around everything drawn: the union of every point of it,
    /// and of the corners of every placed part's own box, where it is.
    fn measure(&self) -> GeopResult<Extent<S>> {
        let sketch_points = self.sketches.iter().flat_map(|sketch| {
            sketch
                .curves
                .iter()
                .flat_map(|c| &c.polyline)
                .map(|p| sketch.plane.uv_to_xyz(p))
        });
        let hull = self
            .vertices
            .iter()
            .map(|v| v.at)
            .chain(self.edges.iter().flat_map(|e| e.polyline.iter().copied()))
            .chain(
                self.faces
                    .iter()
                    .flat_map(|f| f.triangles.iter().flatten().copied()),
            )
            .chain(sketch_points)
            .collect::<Vec<_>>();
        let mut corners = Vec::new();
        for instance in &self.instances {
            let view = instance.source.view()?;
            // An empty part has no box of its own to place.
            if view.vertices.is_empty() && view.faces.is_empty() && view.instances.is_empty() {
                continue;
            }
            let Extent { center, size } = view.extent;
            let half = size.div(S::TWO)?;
            let motion = instance.pose.motion();
            for i in 0..8 {
                let corner = Vector3::from_array([0, 1, 2].map(|k| match i >> k & 1 {
                    0 => center[k].sub(half),
                    _ => center[k].add(half),
                }));
                corners.push(motion.apply(&corner));
            }
        }
        let hull = hull.into_iter().chain(corners).reduce(|a, b| a.union(&b));
        let Some(hull) = hull else {
            return Ok(Extent {
                center: Vector3::zero(),
                size: S::from_f64(EMPTY_EXTENT),
            });
        };
        let size = Vector3::from_array([0, 1, 2].map(|k| hull[k].width())).norm();
        Ok(Extent {
            center: Vector3::from_array([0, 1, 2].map(|k| hull[k].midpoint())),
            size: if size.definitely_less(S::ONE) {
                S::ONE
            } else {
                size
            },
        })
    }

    /// Whatever the pointer is over that can fill one of `roles` — and, with
    /// a `scope`, is part of it: a line of one sketch. Frame datums are drawn
    /// on top of everything, so they are picked first. Otherwise a vertex or
    /// a sketch point, else an edge or a sketch curve, near the pointer — and
    /// not hidden behind a face — wins: the smallest entity under the pointer
    /// is the one meant. Else the nearest along the ray of the faces, solids,
    /// sketches and other datums hit. The parts placed in the part count
    /// alike, each where it is (see [`PartView::layers`]): a vertex of one is
    /// hidden behind a face of another.
    ///
    /// For a [`Role::Frame`], it is a frame that is picked, which what is
    /// under the pointer decides where it sits (see [`Aspects::frame_on`]): a
    /// face under the pointer gives its own, at the corner or the middle of
    /// the side the pointer is near — and in the middle of the face if it is
    /// near neither; a straight edge, at its middle, or at the end the
    /// pointer is near; a circular edge, a point or a datum, its own.
    pub fn pick(
        &self,
        pointer: &Pointer<S>,
        roles: &[Role],
        scope: Option<&EntityRef>,
    ) -> Option<PartHit<S>> {
        let mount = roles.contains(&Role::Frame);
        let hit = self.pick_entity(pointer, if mount { &MOUNTABLE } else { roles }, scope)?;
        // Whatever it is, a frame is picked: on it, if not already.
        Some(match hit.entity {
            EntityRef::Frame { .. } => hit,
            entity if mount => PartHit {
                entity: EntityRef::Frame {
                    on: Box::new(entity),
                    at: None,
                },
                ..hit
            },
            _ => hit,
        })
    }

    /// [`PartView::pick`], for what can be picked as one of `roles`: of a
    /// pick for a frame, the entities near it and the frames put at the
    /// corner or side of a face, or at the end of a straight edge.
    fn pick_entity(
        &self,
        pointer: &Pointer<S>,
        roles: &[Role],
        scope: Option<&EntityRef>,
    ) -> Option<PartHit<S>> {
        let layers = self.layers(pointer).ok()?;
        let mount = roles.contains(&Role::Frame);
        // What `layer` names `entity`, as the part drawn names it, can take
        // — `solid`, the name of the solid it bounds in `layer`, if any.
        let accept_of =
            |layer: &Layer<'_, S>, entity: &EntityRef, its: &[Role], solid: Option<&String>| {
                its.iter().any(|r| roles.contains(r))
                    && scope.is_none_or(|s| {
                        let solid = solid.and_then(|name| {
                            match layer.name(EntityRef::Solid { name: name.clone() }) {
                                EntityRef::Solid { name } => Some(name),
                                _ => None,
                            }
                        });
                        layer.name(entity.clone()).lies_in(s, solid.as_deref())
                    })
            };
        let accept = |layer: &Layer<'_, S>, entity: &EntityRef, its: &[Role]| {
            accept_of(layer, entity, its, None)
        };
        // What a vertex or an edge `entity` is picked as: itself, or —
        // where a path is asked for — the 3-D sketch `sketch` it is part of.
        let taken = |layer: &Layer<'_, S>,
                     entity: EntityRef,
                     its: &[Role],
                     solid: Option<&String>,
                     sketch: Option<&String>| {
            if accept_of(layer, &entity, its, solid) {
                return Some(entity);
            }
            let whole = EntityRef::Sketch3d {
                name: sketch?.clone(),
            };
            accept(layer, &whole, &[Role::Path]).then_some(whole)
        };
        let nearest = |hits: Vec<PartHit<S>>| hits.into_iter().min_by(|a, b| nearer(a.t, b.t));

        let frames = layers
            .iter()
            .filter_map(|l| {
                let hit = l.view.pick_frame(&l.pointer, &|e, r| accept(l, e, r))?;
                Some(l.hit(hit))
            })
            .collect();
        if let Some(hit) = nearest(frames) {
            return Some(hit);
        }

        // The nearest face of any layer: a rigid motion keeps distances
        // along the ray, so every layer's `t` is the part drawn's.
        let face = layers
            .iter()
            .filter(|l| l.near)
            .filter_map(|l| l.view.pick_face(&l.pointer).map(|(t, f)| (t, l, f)))
            .min_by(|a, b| nearer(a.0, b.0));
        // In front of the face hit, give or take the reach — or on its
        // boundary, of the same layer: `faces`, those a vertex or an edge
        // bounds, name it. That is asked of the topology, not the depth: at
        // a glancing look, an edge of the face hit lies further along the
        // ray than the face is hit by more than a reach, and its triangles
        // are chords, in front of a face that curves away.
        let visible = |t: S, faces: &[String], layer: &Layer<'_, S>| {
            face.as_ref().is_none_or(|(ft, l, f)| {
                (l.instance == layer.instance && faces.contains(&f.name))
                    || !t.definitely_greater(ft.add(pointer.reach_at(1.0, *ft)))
            })
        };

        let mut points = Vec::new();
        for l in layers.iter().filter(|l| l.near) {
            let ray = &l.pointer.ray;
            let near = |(dist, t): (S, S), faces: &[String]| {
                (l.pointer.within(dist, t, 1.0) && visible(t, faces, l)).then_some(t)
            };
            let vertices = l.view.vertices.iter().map(|v| {
                let entity = EntityRef::Vertex {
                    name: v.name.clone(),
                };
                let entity = taken(
                    l,
                    entity,
                    &[Role::Point],
                    v.solid.as_ref(),
                    v.sketch.as_ref(),
                );
                (entity, v.at, &v.faces[..])
            });
            let sketch_points = l.view.sketches.iter().flat_map(|sketch| {
                sketch.points.iter().map(|p| {
                    let entity = EntityRef::SketchPoint {
                        sketch: sketch.name.clone(),
                        point: p.id,
                    };
                    let entity = accept(l, &entity, &[Role::Point]).then_some(entity);
                    (entity, sketch.plane.uv_to_xyz(&p.at), &[][..])
                })
            });
            for (entity, at, faces) in vertices.chain(sketch_points) {
                if let Some(entity) = entity
                    && let Some(t) = near(ray.distance_to_point(&at), faces)
                {
                    // A corner of a face the pointer is over is a frame of
                    // that face, there.
                    let entity = match (&face, &entity) {
                        (Some((_, fl, f)), EntityRef::Vertex { .. })
                            if mount
                                && fl.instance == l.instance
                                && faces.contains(&f.name)
                                && f.roles.contains(&Role::Frame) =>
                        {
                            EntityRef::Frame {
                                on: Box::new(EntityRef::Face {
                                    name: f.name.clone(),
                                }),
                                at: Some(Box::new(entity)),
                            }
                        }
                        _ => entity,
                    };
                    points.push(l.hit(PartHit {
                        entity,
                        point: at,
                        t,
                    }));
                }
            }
        }
        if let Some(hit) = nearest(points) {
            return Some(hit);
        }

        let mut curves = Vec::new();
        for l in layers.iter().filter(|l| l.near) {
            let ray = &l.pointer.ray;
            let near = |(dist, t): (S, S), faces: &[String]| {
                (l.pointer.within(dist, t, 1.0) && visible(t, faces, l)).then_some(t)
            };
            // `entity`, hit by the polyline at its point `at`: as it is, or,
            // for a frame, where `at` puts it.
            let mut polyline_hit =
                |entity: EntityRef,
                 faces: &[String],
                 polyline: &mut dyn Iterator<Item = (Vector3<S>, Vector3<S>)>,
                 frame: &dyn Fn(EntityRef, Vector3<S>) -> EntityRef| {
                    let t = polyline
                        .filter_map(|(a, b)| near(ray.distance_to_segment(&a, &b), faces))
                        .min_by(|&a, &b| nearer(a, b));
                    if let Some(t) = t {
                        curves.push(l.hit(PartHit {
                            entity: frame(entity, ray.at(t)),
                            point: ray.at(t),
                            t,
                        }));
                    }
                };
            for e in &l.view.edges {
                let entity = EntityRef::Edge {
                    name: e.name.clone(),
                };
                if let Some(entity) =
                    taken(l, entity, &e.roles, e.solid.as_ref(), e.sketch.as_ref())
                {
                    let mut segments = e.polyline.windows(2).map(|w| (w[0], w[1]));
                    // Near one end of a straight edge, the frame is there.
                    let frame = |entity: EntityRef, at: Vector3<S>| -> EntityRef {
                        let (Some(first), Some(last)) = (e.polyline.first(), e.polyline.last())
                        else {
                            return entity;
                        };
                        if !mount
                            || !matches!(entity, EntityRef::Edge { .. })
                            || !e.roles.contains(&Role::Line)
                        {
                            return entity;
                        }
                        let reach = last.sub(first).norm().mul(S::from_f64(END_OF_A_LINE));
                        let end = if at.sub(first).norm().definitely_less(reach) {
                            0
                        } else if at.sub(last).norm().definitely_less(reach) {
                            1
                        } else {
                            return entity;
                        };
                        EntityRef::Frame {
                            on: Box::new(entity),
                            at: Some(Box::new(EntityRef::Vertex {
                                name: e.ends[end].clone(),
                            })),
                        }
                    };
                    polyline_hit(entity, &e.faces, &mut segments, &frame);
                }
            }
            for sketch in &l.view.sketches {
                for c in &sketch.curves {
                    let entity = EntityRef::SketchCurve {
                        sketch: sketch.name.clone(),
                        curve: c.id,
                    };
                    if accept(l, &entity, &c.roles) {
                        let world = |p: &Vector2<S>| sketch.plane.uv_to_xyz(p);
                        let mut segments =
                            c.polyline.windows(2).map(|w| (world(&w[0]), world(&w[1])));
                        polyline_hit(entity, &[], &mut segments, &|entity, _| entity);
                    }
                }
            }
        }
        if let Some(hit) = nearest(curves) {
            return Some(hit);
        }

        let mut hits: Vec<PartHit<S>> = Vec::new();
        if let Some((t, l, f)) = face {
            let face = EntityRef::Face {
                name: f.name.clone(),
            };
            let solid = f.solid.clone().map(|name| EntityRef::Solid { name });
            let feature = f.feature.clone().map(|name| EntityRef::Feature { name });
            let entity = if accept_of(l, &face, &f.roles, f.solid.as_ref()) {
                Some(face)
            } else {
                solid
                    .filter(|solid| accept(l, solid, &[Role::Solid]))
                    .or_else(|| feature.filter(|feature| accept(l, feature, &[Role::Feature])))
            };
            let entity = match entity {
                Some(EntityRef::Face { name }) if mount => {
                    let at = l.view.corner_or_side_near(&l.pointer, &name);
                    Some(EntityRef::Frame {
                        on: Box::new(EntityRef::Face { name }),
                        at: at.map(Box::new),
                    })
                }
                other => other,
            };
            hits.extend(entity.map(|entity| {
                l.hit(PartHit {
                    entity,
                    point: l.pointer.ray.at(t),
                    t,
                })
            }));
        }
        for l in &layers {
            let accept = |e: &EntityRef, r: &[Role]| accept(l, e, r);
            hits.extend(l.view.pick_sketch(&l.pointer, &accept).map(|h| l.hit(h)));
            let extent = Extent {
                center: l.local(self.extent.center),
                size: self.extent.size,
            };
            hits.extend(
                l.view
                    .pick_datum(&l.pointer, &accept, extent)
                    .map(|h| l.hit(h)),
            );
        }
        nearest(hits)
    }

    /// Whether `entity` — a sketch or a datum — or a part of it can fill one
    /// of `roles`: whether a pick for them could take something of it.
    pub fn can_fill(&self, entity: &EntityRef, roles: &[Role]) -> bool {
        if let Some((instance, inner)) = entity.split_instance() {
            return self
                .instances
                .iter()
                .find(|i| i.name == instance)
                .and_then(|i| i.source.view().ok())
                .is_some_and(|view| view.can_fill(&inner, roles));
        }
        let fills = |its: &[Role]| its.iter().any(|r| roles.contains(r));
        match entity {
            EntityRef::Sketch3d { name } => {
                let of = |sketch: &Option<String>| sketch.as_ref() == Some(name);
                fills(&[Role::Path])
                    && (self.edges.iter().any(|e| of(&e.sketch))
                        || self.vertices.iter().any(|v| of(&v.sketch)))
                    || self.edges.iter().any(|e| of(&e.sketch) && fills(&e.roles))
                    || self.vertices.iter().any(|v| of(&v.sketch)) && fills(&[Role::Point])
            }
            EntityRef::Sketch { name } => self.sketches.iter().any(|s| {
                s.name == *name
                    && (fills(&[Role::Sketch, Role::Path])
                        || (!s.points.is_empty() && fills(&[Role::Point]))
                        || s.curves.iter().any(|c| fills(&c.roles)))
            }),
            EntityRef::Datum { name, .. } => self.datums.iter().any(|d| {
                d.name == *name
                    && match d.kind {
                        DatumKind::Frame => fills(&[Role::Point, Role::Line, Role::Plane]),
                        kind => fills(&[datum_role(kind)]),
                    }
            }),
            _ => false,
        }
    }

    /// The corner of the face `face` the pointer is near, or else the side —
    /// the vertex or the edge — within [`NEAR_CORNER`] and [`NEAR_SIDE`]
    /// reaches of it, whichever is nearer its ray.
    fn corner_or_side_near(&self, pointer: &Pointer<S>, face: &str) -> Option<EntityRef> {
        let ray = &pointer.ray;
        let corner = self
            .vertices
            .iter()
            .filter(|v| v.faces.iter().any(|f| f == face))
            .map(|v| (v, ray.distance_to_point(&v.at)))
            .filter(|(_, (dist, t))| pointer.within(*dist, *t, NEAR_CORNER))
            .min_by(|a, b| nearer(a.1.0, b.1.0))
            .map(|(v, _)| EntityRef::Vertex {
                name: v.name.clone(),
            });
        corner.or_else(|| {
            self.edges
                .iter()
                .filter(|e| e.faces.iter().any(|f| f == face))
                .filter_map(|e| {
                    e.polyline
                        .windows(2)
                        .map(|w| ray.distance_to_segment(&w[0], &w[1]))
                        .filter(|(dist, t)| pointer.within(*dist, *t, NEAR_SIDE))
                        .min_by(|a, b| nearer(a.0, b.0))
                        .map(|(dist, _)| (e, dist))
                })
                .min_by(|a, b| nearer(a.1, b.1))
                .map(|(e, _)| EntityRef::Edge {
                    name: e.name.clone(),
                })
        })
    }

    /// The nearest face the ray enters.
    fn pick_face(&self, pointer: &Pointer<S>) -> Option<(S, &ViewFace<S>)> {
        self.faces
            .iter()
            .flat_map(|f| f.triangles.iter().map(move |tri| (f, tri)))
            .filter_map(|(f, [a, b, c])| pointer.ray.intersect_triangle(a, b, c).map(|t| (t, f)))
            .min_by(|a, b| nearer(a.0, b.0))
    }

    /// The nearest sketch hit inside one of its closed regions — the area a
    /// viewer shades — or near one of its curves.
    fn pick_sketch(
        &self,
        pointer: &Pointer<S>,
        accept: &impl Fn(&EntityRef, &[Role]) -> bool,
    ) -> Option<PartHit<S>> {
        self.sketches
            .iter()
            .filter(|sketch| {
                let entity = EntityRef::Sketch {
                    name: sketch.name.clone(),
                };
                accept(&entity, &[Role::Sketch, Role::Path])
            })
            .filter_map(|sketch| {
                let (t, p) = pointer.ray.intersect_uv_plane(&sketch.plane)?;
                let reach = pointer.reach_at(1.0, t);
                let in_region = sketch.regions.iter().any(|loops| loops_contain(loops, &p));
                let on_curve = || {
                    sketch.curves.iter().any(|c| {
                        c.polyline.windows(2).any(|w| {
                            !p.distance_to_segment(&w[0], &w[1])
                                .definitely_greater(reach)
                        })
                    })
                };
                (in_region || on_curve()).then(|| PartHit {
                    entity: EntityRef::Sketch {
                        name: sketch.name.clone(),
                    },
                    point: pointer.ray.at(t),
                    t,
                })
            })
            .min_by(|a, b| nearer(a.t, b.t))
    }

    /// The nearest datum other than a frame `accept`ed the ray hits, as the
    /// viewer draws it: a point at its origin, an axis as a line and a plane
    /// as a square, each [`Extent::size`] of `extent` — the whole drawing's —
    /// long around the point of it nearest the extent's center.
    fn pick_datum(
        &self,
        pointer: &Pointer<S>,
        accept: &impl Fn(&EntityRef, &[Role]) -> bool,
        extent: Extent<S>,
    ) -> Option<PartHit<S>> {
        let Extent { center, size } = extent;
        let ray = &pointer.ray;
        let half = size.div(S::TWO).ok()?;
        self.datums
            .iter()
            .filter(|d| {
                d.kind != DatumKind::Frame
                    && accept(&EntityRef::datum(d.name.clone()), &[datum_role(d.kind)])
            })
            .filter_map(|d| {
                let f = &d.frame;
                let (origin, w) = (f.origin(), f.w());
                let t = match d.kind {
                    DatumKind::Point => {
                        let (dist, t) = ray.distance_to_point(origin);
                        pointer.within(dist, t, 1.0).then_some(t)?
                    }
                    DatumKind::Axis => {
                        let mid = origin.add(&w.prod_scalar(center.sub(origin).prod_dot(w)));
                        let reach = w.prod_scalar(size);
                        let (dist, t) = ray.distance_to_segment(&mid.sub(&reach), &mid.add(&reach));
                        pointer.within(dist, t, 1.0).then_some(t)?
                    }
                    DatumKind::Plane => {
                        let (t, point) = ray.intersect_plane(origin, w)?;
                        let d = point.sub(&center);
                        let inside =
                            |axis: &Vector3<S>| !d.prod_dot(axis).abs().definitely_greater(half);
                        (inside(f.u()) && inside(f.v())).then_some(t)?
                    }
                    DatumKind::Frame => unreachable!("frames are filtered out above"),
                };
                Some(PartHit {
                    entity: EntityRef::datum(d.name.clone()),
                    point: ray.at(t),
                    t,
                })
            })
            .min_by(|a, b| nearer(a.t, b.t))
    }

    /// The part of a frame datum `accept`ed the ray hits, the frame
    /// [`FRAME`] reaches tall: its origin ball — the frame as a whole, a
    /// point — else an axis (from a tenth of its length to its tip), else a
    /// plane's square (off the origin, between the other two axes). The
    /// smallest part under the pointer, as for the part's entities, and of
    /// the nearest frame.
    fn pick_frame(
        &self,
        pointer: &Pointer<S>,
        accept: &impl Fn(&EntityRef, &[Role]) -> bool,
    ) -> Option<PartHit<S>> {
        let ray = &pointer.ray;
        let frac = |size: S, x: f64| size.mul(S::from_f64(x));
        let (mut balls, mut axes, mut planes) = (Vec::new(), Vec::new(), Vec::new());
        for d in self.datums.iter().filter(|d| d.kind == DatumKind::Frame) {
            let f = &d.frame;
            let origin = f.origin();
            let size = pointer.reach_at(FRAME, ray.closest_to_point(origin));
            if accept(&EntityRef::datum(d.name.clone()), &[Role::Point]) {
                let (dist, t) = ray.distance_to_point(origin);
                if !dist.definitely_greater(frac(size, 0.12)) {
                    balls.push(PartHit {
                        entity: EntityRef::datum(d.name.clone()),
                        point: *origin,
                        t,
                    });
                }
            }
            for axis in FrameAxis::ALL {
                let dir = axis.of(f);
                let component = |c| EntityRef::datum_component(d.name.clone(), c);
                if accept(&component(DatumComponent::Axis(axis)), &[Role::Line]) {
                    let (a, b) = (
                        origin.add(&dir.prod_scalar(frac(size, 0.1))),
                        origin.add(&dir.prod_scalar(size)),
                    );
                    let (dist, t) = ray.distance_to_segment(&a, &b);
                    if pointer.within(dist, t, 0.6) {
                        axes.push(PartHit {
                            entity: EntityRef::datum_component(
                                d.name.clone(),
                                DatumComponent::Axis(axis),
                            ),
                            point: b,
                            t,
                        });
                    }
                }
                if accept(&component(DatumComponent::Plane(axis)), &[Role::Plane])
                    && let Some((t, point)) = ray.intersect_plane(origin, &dir)
                {
                    let local = point.sub(origin);
                    let in_square =
                        FrameAxis::ALL
                            .iter()
                            .filter(|&&other| other != axis)
                            .all(|other| {
                                let x = local
                                    .prod_dot(&other.of(f))
                                    .div(size)
                                    .unwrap_or(S::INFINITY);
                                !x.sub(S::from_f64(0.45))
                                    .abs()
                                    .definitely_greater(S::from_f64(0.15))
                            });
                    if in_square {
                        planes.push(PartHit {
                            entity: EntityRef::datum_component(
                                d.name.clone(),
                                DatumComponent::Plane(axis),
                            ),
                            point,
                            t,
                        });
                    }
                }
            }
        }
        let nearest = |hits: Vec<PartHit<S>>| hits.into_iter().min_by(|a, b| nearer(a.t, b.t));
        nearest(balls)
            .or_else(|| nearest(axes))
            .or_else(|| nearest(planes))
    }
}

/// The role a datum of `kind` fills as a whole, other than a frame's.
fn datum_role(kind: DatumKind) -> Role {
    match kind {
        DatumKind::Point | DatumKind::Frame => Role::Point,
        DatumKind::Axis => Role::Line,
        DatumKind::Plane => Role::Plane,
    }
}
