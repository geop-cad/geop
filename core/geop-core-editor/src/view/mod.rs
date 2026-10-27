//! [`PartView`]: a part as the viewport draws it, and picking elements of
//! it with a [`Pointer`].
//!
//! A part is drawn from its rasterization — sampled edges, triangulated
//! faces, outlined sketches — and a pick tests against exactly that, never
//! re-deriving geometry of its own, so a pick cannot disagree with what is
//! on screen. The same goes for what the viewer draws besides the part: the
//! datums are laid out here, by the sizes the viewer draws them with, and
//! picked by the same rules.
//!
//! Each crate draws its own targets into a view: the model its vertices,
//! edges and faces, the sketches theirs, the datums theirs; an editor
//! composes them. A pick names what it hits as the crate that drew it named
//! it ([`PickedElement`]) and by the kind of thing drawn ([`drawn`]), which
//! the crate defining an element turns into one ([`crate::selectable`]).
//!
//! The parts placed in a part are drawn by reference: each placed part —
//! however deep — is listed once, with where it is ([`ViewInstance`]), and
//! drawn from its component's own view, built once and shared by every
//! instance of it. Moving a placed part changes where it is, not what is
//! drawn. A pick moves the pointer into each placed part's own frame and
//! names what it hits as that part names it, its target behind the
//! instance's path ([`Located`]): `/bolt:model`.

pub mod icons;
pub mod kept;
#[cfg(test)]
mod tests;
mod tree;

use std::{cell::RefCell, rc::Rc};

use crate::editing::{
    Extent, Pointer,
    hit::{ElementHit, Scene, nearer},
};
use geop_core_math::{
    geop_error::GeopResult,
    polygon::polygons_contain,
    primitives::{CoordinateSystem, DatumComponent, DatumKind, FrameAxis, Motion, Pose, Ray},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    placed::{INSTANCE_SEPARATOR, Located},
    reader::Reader,
    target::{Target, TargetReference},
};
use serde::Serialize;
use tree::FaceTree;
pub use tree::triangles_tested;

/// The kinds of what a view draws, as a pick names them
/// ([`PickedElement::kind`]).
pub mod drawn {
    /// A vertex of the model.
    pub const VERTEX: &str = "vertex";
    /// An edge of the model.
    pub const EDGE: &str = "edge";
    /// A face of the model.
    pub const FACE: &str = "face";
    /// A sketch, hit inside one of its regions or near one of its curves.
    pub const SKETCH: &str = "sketch";
    /// A point of a sketch.
    pub const SKETCH_POINT: &str = "sketch_point";
    /// A curve of a sketch.
    pub const SKETCH_CURVE: &str = "sketch_curve";
    /// A datum, or a component of a frame datum.
    pub const DATUM: &str = "datum";
    /// A frame put on what is picked — a face, an edge, a point, a datum —
    /// at the corner or side of a face, or the end of an edge, the pointer
    /// is near (see [`super::FramePick`]).
    pub const FRAME: &str = "frame";
}

/// How near an end of an edge a pick for a frame is taken to be at it: that
/// part of its length from it.
const END_OF_AN_EDGE: f64 = 0.2;

/// How near a corner of a face, in reaches of the pointer, a pick for a
/// frame on it is taken to be at it, and how near a side: the corner first.
const NEAR_CORNER: f64 = 4.0;
const NEAR_SIDE: f64 = 3.0;

/// A frame picked (see [`drawn::FRAME`]): what it is put on, and what of
/// that it sits at — a corner or a side of a face, an end of an edge —
/// each as a pick names it, of one target. Picked as one element of the
/// kind [`drawn::FRAME`], named `on_kind/on_name`, or
/// `on_kind/on_name|at_kind/at_name`: no name has a `/` or a `|`.
#[derive(Clone, Debug, PartialEq)]
pub struct FramePick {
    pub on: PickedElement,
    pub at: Option<PickedElement>,
}

impl FramePick {
    /// It as one element picked.
    pub fn picked(&self) -> PickedElement {
        let named = |p: &PickedElement| format!("{}/{}", p.kind, p.name);
        let name = match &self.at {
            None => named(&self.on),
            Some(at) => format!("{}|{}", named(&self.on), named(at)),
        };
        PickedElement {
            target: self.on.target.clone(),
            kind: drawn::FRAME.to_string(),
            name,
        }
    }

    /// The frame `picked` names, if it names one.
    pub fn of(picked: &PickedElement) -> Option<Self> {
        (picked.kind == drawn::FRAME).then_some(())?;
        let element = |text: &str| {
            let (kind, name) = text.split_once('/')?;
            Some(PickedElement {
                target: picked.target.clone(),
                kind: kind.to_string(),
                name: name.to_string(),
            })
        };
        let (on, at) = match picked.name.split_once('|') {
            Some((on, at)) => (on, Some(element(at)?)),
            None => (picked.name.as_str(), None),
        };
        Some(Self {
            on: element(on)?,
            at,
        })
    }
}

/// Samples per edge and per parametric direction of a face. A face's grid
/// is refined further where its own curvature asks for it, so this is the
/// floor, not the ceiling.
pub const RESOLUTION: usize = 24;

/// How long a frame datum's axes are, in reaches: it is drawn at a constant
/// size on screen, with its origin ball, its axes and the squares of its
/// planes laid out in fractions of that, in [`PartView::pick`] as in the
/// viewer.
pub const FRAME: f64 = 10.0;

/// How big an empty part's drawing is: what a viewer frames, and its datum
/// planes' size, before anything is drawn. Lengths are millimetres, and
/// the parts of a robot are 10 to 500 mm.
pub const EMPTY_EXTENT: f64 = 100.0;

/// A vertex of the part, as drawn.
#[derive(Clone, Debug, Serialize)]
pub struct ViewVertex {
    pub name: String,
    /// The solid it is a corner of.
    pub solid: Option<String>,
    /// The faces it is a corner of. Of no solid, it is hidden with all of
    /// them; a face hit does not hide it (see [`PartView::pick`]).
    pub faces: Vec<String>,
    pub at: Vector3,
}

/// An edge of the part, as drawn: its curve sampled into a polyline.
#[derive(Clone, Debug, Serialize)]
pub struct ViewEdge {
    pub name: String,
    /// The solid it bounds.
    pub solid: Option<String>,
    /// The faces it bounds. Of no solid, it is hidden with all of them; a
    /// face hit does not hide it (see [`PartView::pick`]).
    pub faces: Vec<String>,
    pub polyline: Vec<Vector3>,
    /// The vertices it runs between, by name.
    #[serde(skip)]
    pub ends: [String; 2],
}

/// A face of the part, as drawn: its name, and its triangles. Serialized,
/// the triangles are named by their key ([`FaceMesh::key`]): a viewer is
/// sent each mesh once ([`crate::editor::SceneState::meshes`]).
#[derive(Clone, Debug, Serialize)]
pub struct ViewFace {
    pub name: String,
    /// The solid the face bounds.
    pub solid: Option<String>,
    /// The step whose feature made it, if one did.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub feature: Option<String>,
    #[serde(serialize_with = "FaceMesh::serialize_key")]
    pub mesh: Rc<FaceMesh>,
}

/// A face triangulated, with the surface's normal at each corner, and what
/// it is known by: its key, a hash of what a viewer is sent of it — so a
/// face drawn alike, in this part or another, is one mesh to a viewer.
///
/// Its corners and normals are sharp: a mesh is a polygon drawn near the
/// face, and which point of a corner's enclosure it is drawn through is a
/// free choice — the middle — which a viewer is sent and a pick tests.
///
/// A viewer is sent it as a viewer draws it: each corner its triangles
/// share once, and each triangle by its corners' indices — in base64, the
/// corners' points and normals as 32-bit floats, `x y z` after one
/// another, the indices as 32-bit integers, three a triangle, all little
/// endian: `{"positions", "normals", "indices"}`. A tenth of a mesh's size
/// written as numbers.
#[derive(Debug)]
pub struct FaceMesh {
    pub key: String,
    pub triangles: Vec<[Vector3; 3]>,
    pub normals: Vec<[Vector3; 3]>,
}

impl Serialize for FaceMesh {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        use serde::ser::SerializeStruct;
        let mut index: std::collections::HashMap<[u32; 6], u32> = Default::default();
        let (mut positions, mut normals, mut indices) = (Vec::new(), Vec::new(), Vec::new());
        for (points, at) in self.triangles.iter().zip(&self.normals) {
            for (p, n) in points.iter().zip(at) {
                let p = p.to_array().map(|x| x.to_f64() as f32);
                let n = n.to_array().map(|x| x.to_f64() as f32);
                let bits = [p[0], p[1], p[2], n[0], n[1], n[2]].map(f32::to_bits);
                let next = index.len() as u32;
                let corner = *index.entry(bits).or_insert_with(|| {
                    positions.extend(p.iter().flat_map(|x| x.to_le_bytes()));
                    normals.extend(n.iter().flat_map(|x| x.to_le_bytes()));
                    next
                });
                indices.extend(corner.to_le_bytes());
            }
        }
        let base64 = geop_core_program::workspace::base64;
        let mut s = serializer.serialize_struct("FaceMesh", 3)?;
        s.serialize_field("positions", &base64(&positions))?;
        s.serialize_field("normals", &base64(&normals))?;
        s.serialize_field("indices", &base64(&indices))?;
        s.end()
    }
}

impl FaceMesh {
    /// The mesh of `triangles`, each with the normals `normals` at its
    /// corners, each taken at the middle of its enclosure.
    pub fn new(triangles: Vec<[Vector3; 3]>, normals: Vec<[Vector3; 3]>) -> Rc<Self> {
        use std::hash::{Hash, Hasher};
        FACES_DRAWN.set(FACES_DRAWN.get() + 1);
        let sharp = |corners: Vec<[Vector3; 3]>| -> Vec<[Vector3; 3]> {
            (corners.into_iter())
                .map(|corner| corner.map(|p| p.map(|x| Scalar::from_f64(x.to_f64()))))
                .collect()
        };
        let (triangles, normals) = (sharp(triangles), sharp(normals));
        let mut key = std::hash::DefaultHasher::new();
        for p in triangles.iter().chain(&normals).flatten() {
            for x in p.to_array() {
                x.to_f64().to_bits().hash(&mut key);
            }
        }
        triangles.len().hash(&mut key);
        Rc::new(Self {
            key: format!("{:016x}", key.finish()),
            triangles,
            normals,
        })
    }

    /// Serializes `mesh` as its key.
    fn serialize_key<S: serde::Serializer>(mesh: &Rc<Self>, s: S) -> Result<S::Ok, S::Error> {
        s.serialize_str(&mesh.key)
    }
}

/// A curve of a sketch, as drawn, in the sketch's plane, by the name its
/// sketch gives it.
#[derive(Clone, Debug, Serialize)]
pub struct ViewCurve {
    pub id: String,
    pub construction: bool,
    pub polyline: Vec<Vector2>,
}

/// A point of a sketch, as drawn, in the sketch's plane, by the name its
/// sketch gives it.
#[derive(Clone, Debug, Serialize)]
pub struct ViewSketchPoint {
    pub id: String,
    pub at: Vector2,
}

/// A sketch of the part, as drawn: its plane, its closed regions (each an
/// outer loop and its holes), its curves and its points, in the plane's
/// `u`/`v` coordinates.
#[derive(Clone, Debug, Serialize)]
pub struct ViewSketch {
    pub name: String,
    /// The target it is, which a pick of it or its elements names.
    pub target: String,
    pub plane: CoordinateSystem,
    #[serde(skip)]
    pub regions: Vec<Vec<Vec<Vector2>>>,
    pub curves: Vec<ViewCurve>,
    pub points: Vec<ViewSketchPoint>,
}

/// A datum of the part. A frame's axes and planes are picked on their own,
/// by the names its target gives them ([`ViewDatum::components`]).
#[derive(Clone, Debug, Serialize)]
pub struct ViewDatum {
    pub name: String,
    pub kind: DatumKind,
    pub frame: CoordinateSystem,
    /// The names of a frame's components, as its target names them.
    #[serde(skip)]
    pub components: Vec<(DatumComponent, String)>,
}

/// Something drawn on the part besides its topology — a cosmetic thread —
/// and what it is called.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ViewAnnotation {
    pub name: String,
    pub label: String,
    pub polyline: Vec<Vector3>,
}

/// What a step draws on the part besides its topology — cosmetic threads —
/// the value of its target [`annotations`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Annotations(pub Vec<ViewAnnotation>);

/// What the step `step` draws on the part: defined by a step that draws
/// something, and drawn for every step that does ([`annotations_of`]).
pub fn annotations(step: &str) -> TargetReference<Annotations> {
    TargetReference::new(format!("annotations:{step}"))
}

/// Everything the steps `reader` sees draw on the part, step by step.
pub fn annotations_of(reader: &Reader) -> Vec<ViewAnnotation> {
    (reader.targets_of::<Annotations>().into_iter())
        .filter_map(|target| {
            let whole = ElementReference::<Annotations, Annotations>::whole(target);
            reader
                .retrieve_element(&whole)
                .ok()
                .map(|drawn| drawn.0.clone())
        })
        .flatten()
        .collect()
}

/// Each step's own.
impl Target for Annotations {}

/// Read whole.
impl Kind for Annotations {
    const NAME: &'static str = "annotations";
}

impl HasElements<Annotations> for Annotations {
    type Element<'a> = &'a Annotations;

    fn element(&self, name: &str) -> GeopResult<&Annotations> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with("annotations:")
    }
}

/// An entry of the part's structure a step adds beside what every part has:
/// a mate, of kind `mate`.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Listed {
    /// What it is — the group a structure tree lists it in: `mate`.
    pub kind: String,
    pub name: String,
    /// What it refers to: the elements a mate is between, lit while it is
    /// pointed at.
    pub elements: Vec<PickedElement>,
}

/// The entries of the part's structure a step adds: the value of its target
/// [`listing`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Listing(pub Vec<Listed>);

/// The entries of the part's structure the step `step` adds.
pub fn listing(step: &str) -> TargetReference<Listing> {
    TargetReference::new(format!("listing:{step}"))
}

/// Every entry the steps `reader` sees add to the structure, step by step.
pub fn listed_by_steps(reader: &Reader) -> Vec<Listed> {
    (reader.targets_of::<Listing>().into_iter())
        .filter_map(|target| {
            let whole = ElementReference::<Listing, Listing>::whole(target);
            reader
                .retrieve_element(&whole)
                .ok()
                .map(|listing| listing.0.clone())
        })
        .flatten()
        .collect()
}

/// Each step's own.
impl Target for Listing {}

/// Read whole.
impl Kind for Listing {
    const NAME: &'static str = "listing";
}

impl HasElements<Listing> for Listing {
    type Element<'a> = &'a Listing;

    fn element(&self, name: &str) -> GeopResult<&Listing> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with("listing:")
    }
}

/// A part placed in the part drawn — directly or in a part placed in it —
/// drawn as its part's own view moved to where it is. Serialized, a viewer
/// gets the part by its key and where to draw it.
#[derive(Clone, Serialize)]
pub struct ViewInstance {
    /// Its name in the part drawn: `bolt`, or `asm/bolt` for one placed in
    /// a placed part.
    pub name: String,
    /// The key of its part.
    pub component: String,
    /// Where it is: its own frame in the part drawn.
    pub frame: CoordinateSystem,
    #[serde(skip)]
    pub pose: Pose,
    /// Its part as drawn.
    #[serde(skip)]
    pub view: Rc<PartView>,
    /// The state of the part drawn its pose is — `None` for a copy of a
    /// pattern, which goes where the pattern puts it.
    #[serde(skip)]
    pub state: Option<String>,
    /// Whether no mate moves it.
    #[serde(skip)]
    pub fixed: bool,
    /// The colour it is drawn in over its own, `#rrggbb`, if the user chose
    /// one for it (see [`crate::editor::Command::Appearance`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub color: Option<String>,
}

impl std::fmt::Debug for ViewInstance {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "ViewInstance({}, {}, {:?})",
            self.name, self.component, self.pose
        )
    }
}

/// A part as the viewport draws it, every element by name — and, serialized,
/// what a viewer draws. Each crate draws its own targets into it; then
/// [`PartView::finish`] measures it.
#[derive(Clone, Debug, Serialize)]
pub struct PartView {
    /// The target its vertices, edges and faces are of: what a viewer
    /// matches an element's target with to light it.
    pub model: String,
    pub vertices: Vec<ViewVertex>,
    pub edges: Vec<ViewEdge>,
    pub faces: Vec<ViewFace>,
    pub sketches: Vec<ViewSketch>,
    /// The target its datums are of.
    pub datum_target: String,
    pub datums: Vec<ViewDatum>,
    /// What its steps draw on it besides its topology: cosmetic threads.
    pub annotations: Vec<ViewAnnotation>,
    /// The part's solids, oldest first.
    pub solids: Vec<String>,
    /// Every part placed in it, and in those, however deep. Not
    /// serialized: a viewer is sent them apart, as they change, by name.
    #[serde(skip)]
    pub instances: Vec<ViewInstance>,
    /// Where the drawing is and how big: the center and diagonal, at least
    /// one, of the box around it — for an empty part, [`EMPTY_EXTENT`]
    /// around the origin. Datum planes and axes, which are endless, are
    /// drawn this big around the point of them nearest the center.
    pub extent: Extent,
    /// The part's colour, `#rrggbb`; none for the viewer's own.
    pub color: Option<String>,
    /// The tree of its faces' triangles a pick tests the ray against (see
    /// [`tree`]): built when it is first picked in, and again only for
    /// other faces.
    #[serde(skip)]
    faces_tree: RefCell<Option<Rc<FaceTree>>>,
}

/// One part of what a pick tests: the part drawn itself, or a placed part,
/// with the pointer moved into its own frame.
struct Layer<'v> {
    view: &'v PartView,
    pointer: Pointer,
    /// Whether the pointer comes near enough the box around its view
    /// ([`PartView::extent`]) to hit any of its vertices, edges or faces:
    /// only its datums can be hit otherwise — a frame is drawn at a size of
    /// its own, a plane or an axis as large as the whole drawing.
    near: bool,
    /// Where it is, and back; `None` for the part drawn itself.
    motion: Option<(Motion, Motion)>,
    /// The instance's name, for a placed part.
    instance: Option<&'v str>,
}

impl Layer<'_> {
    /// The element `kind` `name` of the target `target` of this layer, as
    /// the part drawn names it.
    fn element(&self, target: &str, kind: &str, name: &str) -> PickedElement {
        PickedElement {
            target: Located::in_instance(self.instance.unwrap_or_default(), target),
            kind: kind.to_string(),
            name: name.to_string(),
        }
    }

    /// The point `p` of this layer where the part drawn has it.
    fn world(&self, p: Vector3) -> Vector3 {
        match &self.motion {
            Some((there, _)) => there.apply(&p),
            None => p,
        }
    }

    /// The point `p` of the part drawn in this layer's own frame.
    fn local(&self, p: Vector3) -> Vector3 {
        match &self.motion {
            Some((_, back)) => back.apply(&p),
            None => p,
        }
    }

    /// The element `kind` `name` of `target`, hit at `point`, `t` along the
    /// ray, as a hit of the part drawn.
    fn hit(&self, target: &str, kind: &str, name: &str, point: Vector3, t: Scalar) -> ElementHit {
        ElementHit {
            element: self.element(target, kind, name),
            point: self.world(point),
            t,
        }
    }
}

impl PartView {
    /// Nothing drawn, reaching as far as `extent`: what a viewer is sent
    /// while a step is edited on a sheet of its own, which it frames.
    pub fn blank(extent: Extent) -> Self {
        Self {
            model: String::new(),
            vertices: Vec::new(),
            edges: Vec::new(),
            faces: Vec::new(),
            sketches: Vec::new(),
            datum_target: String::new(),
            datums: Vec::new(),
            annotations: Vec::new(),
            solids: Vec::new(),
            instances: Vec::new(),
            extent,
            color: None,
            faces_tree: RefCell::default(),
        }
    }

    /// Nothing drawn yet, to be drawn into: as big as an empty part.
    pub fn new() -> Self {
        Self::blank(Extent {
            center: Vector3::zero(),
            size: Scalar::from_f64(EMPTY_EXTENT),
        })
    }

    /// What `drawn` drew added: what one drawer drew of a part, into what
    /// the others drew of it (see [`crate::registry::Drawings`]). Drawers
    /// draw what is theirs alone, so nothing is drawn twice.
    pub fn add(&mut self, drawn: &PartView) {
        if !drawn.model.is_empty() {
            self.model.clone_from(&drawn.model);
        }
        if !drawn.datum_target.is_empty() {
            self.datum_target.clone_from(&drawn.datum_target);
        }
        if drawn.color.is_some() {
            self.color.clone_from(&drawn.color);
        }
        self.vertices.extend_from_slice(&drawn.vertices);
        self.edges.extend_from_slice(&drawn.edges);
        self.faces.extend_from_slice(&drawn.faces);
        self.sketches.extend_from_slice(&drawn.sketches);
        self.datums.extend_from_slice(&drawn.datums);
        self.annotations.extend_from_slice(&drawn.annotations);
        self.solids.extend_from_slice(&drawn.solids);
        self.instances.extend_from_slice(&drawn.instances);
    }

    /// Done drawing: measured.
    pub fn finish(mut self) -> Self {
        PARTS_DRAWN.set(PARTS_DRAWN.get() + 1);
        self.extent = self.measure();
        self
    }

    /// Whether `pointer` could hit any of its vertices, edges or faces —
    /// within one reach of them, as a pick asks: whether it passes within
    /// one reach of the ball around the box of everything drawn
    /// ([`Extent`]), the reach taken where the ball ends furthest along
    /// the ray. A pick tests every triangle of a part it comes near, so for
    /// a part placed hundreds of times, this is what it can skip.
    fn near(&self, pointer: &Pointer) -> bool {
        let Ok(radius) = self.extent.size.div(Scalar::TWO) else {
            return true;
        };
        let (dist, t) = pointer.ray.distance_to_point(&self.extent.center);
        !dist
            .sub(radius)
            .definitely_greater(pointer.reach_at(1.0, t.add(radius)))
    }

    /// The part drawn itself, then every placed part, each with `pointer`
    /// moved into its frame.
    fn layers(&self, pointer: &Pointer) -> Vec<Layer<'_>> {
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
            let view = &*instance.view;
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
        layers
    }

    /// The placed part a drag at `pointer` moves, and how far along the ray
    /// it is hit: the one whose face the ray enters first — or, for a part
    /// placed in one placed rigid, the innermost one around it whose pose is
    /// a state of the part drawn: what moves it. None for a fixed one: no
    /// drag moves it.
    pub fn part_to_drag(&self, pointer: &Pointer) -> Option<(&ViewInstance, Scalar)> {
        let layers = self.layers(pointer);
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
            .filter(|i| i.state.is_some())
            .filter(|i| {
                i.name == hit
                    || (hit.strip_prefix(&i.name))
                        .is_some_and(|r| r.starts_with(INSTANCE_SEPARATOR))
            })
            .max_by_key(|i| i.name.len())?;
        (!moved.fixed).then_some((moved, t))
    }

    /// How far along `pointer`'s ray it first enters the placed part
    /// `instance`, if it does.
    pub fn pick_instance(&self, instance: &str, pointer: &Pointer) -> Option<Scalar> {
        let layers = self.layers(pointer);
        let layer = layers.iter().find(|l| l.instance == Some(instance))?;
        layer
            .near
            .then(|| layer.view.pick_face(&layer.pointer))?
            .map(|(t, _)| t)
    }

    /// The box around everything drawn: the union of every point of it,
    /// and of the corners of every placed part's own box, where it is.
    fn measure(&self) -> Extent {
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
                    .flat_map(|f| f.mesh.triangles.iter().flatten().copied()),
            )
            .chain(sketch_points)
            .collect::<Vec<_>>();
        let mut corners = Vec::new();
        for instance in &self.instances {
            let view = &instance.view;
            // An empty part has no box of its own to place.
            if view.vertices.is_empty() && view.faces.is_empty() && view.instances.is_empty() {
                continue;
            }
            let Extent { center, size } = view.extent;
            let half = size.mul(Scalar::from_f64(0.5));
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
            return Extent {
                center: Vector3::zero(),
                size: Scalar::from_f64(EMPTY_EXTENT),
            };
        };
        let size = Vector3::from_array([0, 1, 2].map(|k| hull[k].width())).norm();
        Extent {
            center: Vector3::from_array([0, 1, 2].map(|k| hull[k].midpoint())),
            size: if size.definitely_less(Scalar::ONE) {
                Scalar::ONE
            } else {
                size
            },
        }
    }

    /// Whatever drawn of a kind in `accepts` the pointer is over. Frame
    /// datums are drawn on top of everything, so they are picked first.
    /// Otherwise a vertex or a sketch point, else an edge or a sketch curve,
    /// near the pointer — and not hidden behind a face — wins: the smallest
    /// element under the pointer is the one meant. Else the nearest along
    /// the ray of the faces, sketches and other datums hit. The parts placed
    /// in the part count alike, each where it is (see [`PartView::layers`]):
    /// a vertex of one is hidden behind a face of another.
    pub fn pick(&self, pointer: &Pointer, accepts: &[String]) -> Option<ElementHit> {
        let accepted = |kind: &str| accepts.iter().any(|a| a == kind);
        if accepted(drawn::FRAME) {
            return self.pick_frame_on(pointer);
        }
        let layers = self.layers(pointer);
        if accepted(drawn::DATUM) {
            let frames = layers.iter().filter_map(|l| l.view.pick_frame(l));
            if let Some(hit) = nearest(frames) {
                return Some(hit);
            }
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
        let visible = |t: Scalar, faces: &[String], layer: &Layer<'_>| {
            face.as_ref().is_none_or(|(ft, l, f)| {
                (l.instance == layer.instance && faces.contains(&f.name))
                    || !t.definitely_greater(ft.add(pointer.reach_at(1.0, *ft)))
            })
        };

        let mut points = Vec::new();
        for l in layers.iter().filter(|l| l.near) {
            let ray = &l.pointer.ray;
            let near = |at: &Vector3, faces: &[String]| {
                let (dist, t) = ray.distance_to_point(at);
                (l.pointer.within(dist, t, 1.0) && visible(t, faces, l)).then_some(t)
            };
            if accepted(drawn::VERTEX) {
                for v in &l.view.vertices {
                    if let Some(t) = near(&v.at, &v.faces) {
                        points.push(l.hit(&l.view.model, drawn::VERTEX, &v.name, v.at, t));
                    }
                }
            }
            if accepted(drawn::SKETCH_POINT) {
                for sketch in &l.view.sketches {
                    for p in &sketch.points {
                        let at = sketch.plane.uv_to_xyz(&p.at);
                        if let Some(t) = near(&at, &[]) {
                            points.push(l.hit(&sketch.target, drawn::SKETCH_POINT, &p.id, at, t));
                        }
                    }
                }
            }
        }
        if let Some(hit) = nearest(points) {
            return Some(hit);
        }

        let (mut edges, mut sketch_curves) = (Vec::new(), Vec::new());
        for l in layers.iter().filter(|l| l.near) {
            let ray = &l.pointer.ray;
            // How far along the ray `polyline` is hit, if it is near enough
            // and not hidden.
            let hit_along = |polyline: &mut dyn Iterator<Item = (Vector3, Vector3)>,
                             faces: &[String]| {
                polyline
                    .map(|(a, b)| ray.distance_to_segment(&a, &b))
                    .filter(|&(dist, t)| l.pointer.within(dist, t, 1.0) && visible(t, faces, l))
                    .map(|(_, t)| t)
                    .min_by(|&a, &b| nearer(a, b))
            };
            if accepted(drawn::EDGE) {
                for e in &l.view.edges {
                    let mut segments = e.polyline.array_windows().map(|&[a, b]| (a, b));
                    if let Some(t) = hit_along(&mut segments, &e.faces) {
                        edges.push(l.hit(&l.view.model, drawn::EDGE, &e.name, ray.at(t), t));
                    }
                }
            }
            if accepted(drawn::SKETCH_CURVE) {
                for sketch in &l.view.sketches {
                    for c in &sketch.curves {
                        let world = |p: &Vector2| sketch.plane.uv_to_xyz(p);
                        let mut segments = c
                            .polyline
                            .array_windows()
                            .map(|[a, b]| (world(a), world(b)));
                        if let Some(t) = hit_along(&mut segments, &[]) {
                            let at = ray.at(t);
                            let hit = l.hit(&sketch.target, drawn::SKETCH_CURVE, &c.id, at, t);
                            sketch_curves.push(hit);
                        }
                    }
                }
            }
        }
        // A sketch curve a feature was built from lies on the edge it made:
        // which of the two is nearer along the ray is rounding, so an edge
        // within a reach of the sketch curve hit is taken — the part's own.
        let curve = match (nearest(edges), nearest(sketch_curves)) {
            (Some(edge), Some(drawn))
                if edge
                    .t
                    .definitely_greater(drawn.t.add(pointer.reach_at(1.0, drawn.t))) =>
            {
                Some(drawn)
            }
            (edge, drawn) => edge.or(drawn),
        };
        if let Some(hit) = curve {
            return Some(hit);
        }

        let mut hits: Vec<ElementHit> = Vec::new();
        if accepted(drawn::FACE)
            && let Some((t, l, f)) = face
        {
            let point = l.pointer.ray.at(t);
            hits.push(l.hit(&l.view.model, drawn::FACE, &f.name, point, t));
        }
        for l in &layers {
            if accepted(drawn::SKETCH) {
                hits.extend(l.view.pick_sketch(l));
            }
            if accepted(drawn::DATUM) {
                let extent = Extent {
                    center: l.local(self.extent.center),
                    size: self.extent.size,
                };
                hits.extend(l.view.pick_datum(l, extent));
            }
        }
        nearest(hits)
    }

    /// A frame put on whatever the pointer is over (see [`FramePick`]): a
    /// face under it gives its own, at the corner or the side the pointer
    /// is near — and in the middle of the face if it is near neither; a
    /// vertex of the face under the pointer, the face's at that corner; an
    /// edge, its own, at the end the pointer is near; a point or a datum,
    /// its own.
    fn pick_frame_on(&self, pointer: &Pointer) -> Option<ElementHit> {
        let kinds = [
            drawn::DATUM,
            drawn::VERTEX,
            drawn::EDGE,
            drawn::FACE,
            drawn::SKETCH_POINT,
            drawn::SKETCH_CURVE,
        ];
        let accepts: Vec<String> = kinds.iter().map(|k| k.to_string()).collect();
        let hit = self.pick(pointer, &accepts)?;
        let layers = self.layers(pointer);
        let instance = Some(hit.element.target.path()).filter(|path| !path.is_empty());
        let layer = layers.iter().find(|l| l.instance == instance)?;
        let view = layer.view;
        let element = |kind: &str, name: &str| PickedElement {
            target: hit.element.target.clone(),
            kind: kind.to_string(),
            name: name.to_string(),
        };
        let at = match hit.element.kind.as_str() {
            drawn::VERTEX => {
                let vertex = view.vertices.iter().find(|v| v.name == hit.element.name)?;
                let under = view.pick_face(&layer.pointer);
                if let Some((_, face)) = under.filter(|(_, f)| vertex.faces.contains(&f.name)) {
                    let on = element(drawn::FACE, &face.name);
                    let frame = FramePick {
                        on,
                        at: Some(hit.element.clone()),
                    };
                    return Some(ElementHit {
                        element: frame.picked(),
                        ..hit
                    });
                }
                None
            }
            drawn::EDGE => {
                let edge = view.edges.iter().find(|e| e.name == hit.element.name)?;
                let (first, last) = (edge.polyline.first()?, edge.polyline.last()?);
                let local = layer.local(hit.point);
                let reach = last.sub(first).norm().mul(Scalar::from_f64(END_OF_AN_EDGE));
                if local.sub(first).norm().definitely_less(reach) {
                    Some(element(drawn::VERTEX, &edge.ends[0]))
                } else if local.sub(last).norm().definitely_less(reach) {
                    Some(element(drawn::VERTEX, &edge.ends[1]))
                } else {
                    None
                }
            }
            drawn::FACE => (view.corner_or_side_near(&layer.pointer, &hit.element.name))
                .map(|(kind, name)| element(kind, &name)),
            _ => None,
        };
        let frame = FramePick {
            on: hit.element.clone(),
            at,
        };
        Some(ElementHit {
            element: frame.picked(),
            ..hit
        })
    }

    /// The corner of the face `face` the pointer is near, or else the side —
    /// the vertex or the edge, by its kind and name — within [`NEAR_CORNER`]
    /// and [`NEAR_SIDE`] reaches of it, whichever is nearer its ray.
    fn corner_or_side_near(&self, pointer: &Pointer, face: &str) -> Option<(&'static str, String)> {
        let ray = &pointer.ray;
        let corner = (self.vertices.iter())
            .filter(|v| v.faces.iter().any(|f| f == face))
            .map(|v| (v, ray.distance_to_point(&v.at)))
            .filter(|(_, (dist, t))| pointer.within(*dist, *t, NEAR_CORNER))
            .min_by(|a, b| nearer(a.1.0, b.1.0))
            .map(|(v, _)| (drawn::VERTEX, v.name.clone()));
        corner.or_else(|| {
            (self.edges.iter())
                .filter(|e| e.faces.iter().any(|f| f == face))
                .filter_map(|e| {
                    (e.polyline.array_windows())
                        .map(|[a, b]| ray.distance_to_segment(a, b))
                        .filter(|(dist, t)| pointer.within(*dist, *t, NEAR_SIDE))
                        .min_by(|a, b| nearer(a.0, b.0))
                        .map(|(dist, _)| (e, dist))
                })
                .min_by(|a, b| nearer(a.1, b.1))
                .map(|(e, _)| (drawn::EDGE, e.name.clone()))
        })
    }

    /// The nearest face the ray enters, of the triangles its tree finds the
    /// ray could meet (see [`tree`]) — the first of the nearest, in the
    /// order of the faces and their triangles, as testing every one finds.
    fn pick_face(&self, pointer: &Pointer) -> Option<(Scalar, &ViewFace)> {
        let mut hits = Vec::new();
        self.faces_tree().near(&pointer.ray, |f, t| {
            if let Some(face) = self.faces.get(f)
                && let Some([a, b, c]) = face.mesh.triangles.get(t)
                && let Some(hit) = pointer.ray.intersect_triangle(a, b, c)
            {
                hits.push(((f, t), hit, face));
            }
        });
        hits.sort_by_key(|(at, ..)| *at);
        (hits.into_iter())
            .map(|(_, hit, face)| (hit, face))
            .min_by(|a, b| nearer(a.0, b.0))
    }

    /// The tree of its faces' triangles: built once, and again only once
    /// its faces are others.
    fn faces_tree(&self) -> Rc<FaceTree> {
        let mut kept = self.faces_tree.borrow_mut();
        if let Some(tree) = kept.as_ref().filter(|tree| tree.is_of(&self.faces)) {
            return tree.clone();
        }
        let tree = Rc::new(FaceTree::of(&self.faces));
        *kept = Some(tree.clone());
        tree
    }

    /// The nearest sketch of `layer` hit inside one of its closed regions —
    /// the area a viewer shades — or near one of its curves.
    fn pick_sketch(&self, layer: &Layer<'_>) -> Option<ElementHit> {
        let pointer = &layer.pointer;
        let hits = self.sketches.iter().filter_map(|sketch| {
            let (t, p) = pointer.ray.intersect_uv_plane(&sketch.plane)?;
            let reach = pointer.reach_at(1.0, t);
            let in_region = sketch
                .regions
                .iter()
                .any(|loops| polygons_contain(loops, &p));
            let on_curve = || {
                sketch.curves.iter().any(|c| {
                    c.polyline
                        .array_windows()
                        .any(|[a, b]| !p.distance_to_segment(a, b).definitely_greater(reach))
                })
            };
            (in_region || on_curve())
                .then(|| layer.hit(&sketch.target, drawn::SKETCH, "", pointer.ray.at(t), t))
        });
        nearest(hits)
    }

    /// The nearest datum of `layer` other than a frame the ray hits, as the
    /// viewer draws it: a point at its origin, an axis as a line and a plane
    /// as a square, each [`Extent::size`] of `extent` — the whole drawing's —
    /// long around the point of it nearest the extent's center.
    fn pick_datum(&self, layer: &Layer<'_>, extent: Extent) -> Option<ElementHit> {
        let pointer = &layer.pointer;
        let Extent { center, size } = extent;
        let ray = &pointer.ray;
        let half = size.div(Scalar::TWO).ok()?;
        let hits = self.datums.iter().filter_map(|d| {
            let f = &d.frame;
            let (origin, w) = (f.origin(), f.w());
            let t = match d.kind {
                DatumKind::Frame => None?,
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
                    let inside = |axis: &Vector3| !d.prod_dot(axis).abs().definitely_greater(half);
                    (inside(f.u()) && inside(f.v())).then_some(t)?
                }
            };
            Some(layer.hit(&self.datum_target, drawn::DATUM, &d.name, ray.at(t), t))
        });
        nearest(hits)
    }

    /// The part of a frame datum of `layer` the ray hits, the frame
    /// [`FRAME`] reaches tall: its origin ball — the frame as a whole —
    /// else an axis (from a tenth of its length to its tip), else a plane's
    /// square (off the origin, between the other two axes). The smallest
    /// part under the pointer, as for the part's elements, and of the
    /// nearest frame.
    fn pick_frame(&self, layer: &Layer<'_>) -> Option<ElementHit> {
        let pointer = &layer.pointer;
        let ray = &pointer.ray;
        let frac = |size: Scalar, x: f64| size.mul(Scalar::from_f64(x));
        let hit = |name: &str, point: Vector3, t: Scalar| {
            layer.hit(&self.datum_target, drawn::DATUM, name, point, t)
        };
        let (mut balls, mut axes, mut planes) = (Vec::new(), Vec::new(), Vec::new());
        for d in self.datums.iter().filter(|d| d.kind == DatumKind::Frame) {
            let f = &d.frame;
            let origin = f.origin();
            let size = pointer.reach_at(FRAME, ray.closest_to_point(origin));
            let (dist, t) = ray.distance_to_point(origin);
            if !dist.definitely_greater(frac(size, 0.12)) {
                balls.push(hit(&d.name, *origin, t));
            }
            for (component, name) in &d.components {
                match *component {
                    DatumComponent::Axis(axis) => {
                        let dir = axis.of(f);
                        let (a, b) = (
                            origin.add(&dir.prod_scalar(frac(size, 0.1))),
                            origin.add(&dir.prod_scalar(size)),
                        );
                        let (dist, t) = ray.distance_to_segment(&a, &b);
                        if pointer.within(dist, t, 0.6) {
                            axes.push(hit(name, b, t));
                        }
                    }
                    DatumComponent::Plane(normal) => {
                        let Some((t, point)) = ray.intersect_plane(origin, &normal.of(f)) else {
                            continue;
                        };
                        let local = point.sub(origin);
                        let in_square = (FrameAxis::ALL.iter())
                            .filter(|&&other| other != normal)
                            .all(|other| {
                                let x = (local.prod_dot(&other.of(f)).div(size))
                                    .unwrap_or(Scalar::INFINITY);
                                !x.sub(Scalar::from_f64(0.45))
                                    .abs()
                                    .definitely_greater(Scalar::from_f64(0.15))
                            });
                        if in_square {
                            planes.push(hit(name, point, t));
                        }
                    }
                }
            }
        }
        nearest(balls)
            .or_else(|| nearest(axes))
            .or_else(|| nearest(planes))
    }
}

impl Default for PartView {
    fn default() -> Self {
        Self::new()
    }
}

thread_local! {
    static PARTS_DRAWN: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
    static FACES_DRAWN: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// How many faces this thread has drawn ([`FaceMesh::new`]) ever: what
/// showing a part drawn before, with other sketches, must not do — so a
/// test bounds it by counting these, not by timing it.
pub fn faces_drawn() -> usize {
    FACES_DRAWN.get()
}

/// How many parts this thread has drawn ([`PartView::finish`]) ever.
/// Drawing a part is what a pointer event must not do, so a test can bound
/// the work an event does by counting these, rather than timing it.
pub fn parts_drawn() -> usize {
    PARTS_DRAWN.get()
}

/// A part as drawn: what a step editor picks in (see [`Scene`]) — the part
/// before the step, or what the step built.
impl Scene for PartView {
    fn pick(&self, pointer: &Pointer, accepts: &[String]) -> Option<ElementHit> {
        PartView::pick(self, pointer, accepts)
    }

    fn hit_instance(&self, name: &str, pointer: &Pointer) -> Option<Scalar> {
        self.pick_instance(name, pointer)
    }
}

/// The nearest of `hits` along the ray.
fn nearest(hits: impl IntoIterator<Item = ElementHit>) -> Option<ElementHit> {
    hits.into_iter().min_by(|a, b| nearer(a.t, b.t))
}
