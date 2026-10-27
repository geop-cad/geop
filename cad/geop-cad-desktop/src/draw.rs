//! What the 3-D view draws, as plain lists the GPU takes ([`Drawing`]):
//! made of what the controller holds — the scene, the step's visuals and
//! gizmo, the highlights, the datums, the grid of the plane worked in —
//! seen by the camera. As the web app's viewer (`SceneViewer.tsx`,
//! `partScene.ts`, `datums3d.ts`, `visuals3d.ts`, `gizmo3d.ts`,
//! `planeGrid.ts`) draws them, at the same sizes on screen — what the
//! kernel lays out in reaches is drawn [`REACH_PX`] pixels a reach — so
//! everything is hit where it is seen. Nothing here decides what the
//! pointer hits: the kernel does, from the pointer's ray.
//!
//! The part's faces, edges and sketches change only with the scene and
//! with what is lit, and are kept between frames ([`Statics`]); what is
//! sized to the screen — datums' frames, handles, the gizmo, the grid — is
//! drawn anew every frame.

use std::{
    collections::{BTreeSet, HashMap},
    hash::{Hash, Hasher},
    sync::Arc,
};

use geop_core_editor::{
    editing::{
        GizmoPart, GizmoView, Presentation, Shape, Style, TRIAD, Visual, gizmo::size as gizmo_size,
    },
    view::{
        RESOLUTION,
        lit::{Lighting, Lit},
    },
};
use geop_core_math::{primitives::DatumKind, scalars::Scalar, vector::Vector3};

use crate::{
    camera::{Camera, REACH_PX},
    math::{V3, add, cross, dot, length, neg, normalized, scale, sub},
    scene::{Frame, InstanceData, Kept, MeshData, PartData},
};

/// A colour, `0xrrggbb`, and how opaque.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rgba(pub u32, pub f32);

impl Rgba {
    pub const fn hex(hex: u32) -> Self {
        Self(hex, 1.0)
    }

    pub fn with_alpha(self, alpha: f32) -> Self {
        Self(self.0, alpha.clamp(0.0, 1.0))
    }

    /// As the GPU takes it: sRGB components, unpremultiplied.
    pub fn f32(self) -> [f32; 4] {
        let c = |shift: u32| ((self.0 >> shift) & 0xff) as f32 / 255.0;
        [c(16), c(8), c(0), self.1]
    }

    /// Blended most of the way to `to`.
    pub fn tint(self, to: Rgba, k: f32) -> Self {
        let [r, g, b, a] = self.f32();
        let [tr, tg, tb, _] = to.f32();
        let mix = |x: f32, y: f32| ((x + (y - x) * k) * 255.0).round() as u32;
        Self((mix(r, tr) << 16) | (mix(g, tg) << 8) | mix(b, tb), a)
    }
}

/// How the part's entities are coloured (`partScene.ts`).
pub const VERTEX: Rgba = Rgba::hex(0x3c3c3c);
pub const EDGE: Rgba = Rgba::hex(0x30343b);
pub const FACE: Rgba = Rgba::hex(0xb0b4b8);
pub const SKETCH: Rgba = Rgba::hex(0xffa040);
pub const CONSTRUCTION: Rgba = Rgba::hex(0x808080);
const ANNOTATION: Rgba = Rgba::hex(0x2a2a2a);
pub const HIGHLIGHT: Rgba = Rgba::hex(0xffc94a);
pub const CAP: Rgba = Rgba::hex(0xc86464);

/// How each style of a visual is drawn (`visuals3d.ts`).
pub fn style_color(style: Style) -> Rgba {
    Rgba::hex(match style {
        Style::Free => 0x6cb4ff,
        Style::Fixed => 0xe6e6e6,
        Style::Selected => 0xffd84a,
        Style::Hover => 0xffe0a0,
        Style::Failed => 0xff5d5d,
        Style::Construction => 0x8a8a8a,
        Style::Reference => 0xb48cf0,
        Style::Draft => 0xffa040,
        Style::Region => 0x6cb4ff,
        Style::Guide => 0x777777,
        Style::Handle => 0xffa040,
        Style::Snap => 0x58e07a,
        Style::Removed => 0xff4f9a,
        Style::Paper => 0xbbbbbb,
    })
}

/// A corner of a face's mesh.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MeshVertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

/// A face drawn: where its mesh is put (the rows of an affine map), and
/// its colour.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FaceInstance {
    pub rows: [[f32; 4]; 3],
    pub color: [f32; 4],
}

/// A corner of a flat-coloured triangle.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct FlatVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

/// A segment of a line, drawn `width` pixels wide — dashed if `dash`
/// pixels on and `gap` off are given, measured from `start` along the
/// line, in the world, `per_pixel` world a pixel.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LineInstance {
    pub a: [f32; 3],
    pub width: f32,
    pub b: [f32; 3],
    pub start: f32,
    pub color: [f32; 4],
    pub dash: f32,
    pub gap: f32,
    pub per_pixel: f32,
    pub _pad: f32,
}

/// A point, `size` pixels across: a dot, or a ring (`ring` 1).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PointInstance {
    pub at: [f32; 3],
    pub size: f32,
    pub color: [f32; 4],
    pub ring: f32,
    pub _pad: [f32; 3],
}

/// Triangles, lines and points drawn together, in that order.
#[derive(Clone, Debug, Default)]
pub struct Layer {
    pub triangles: Vec<FlatVertex>,
    pub lines: Vec<LineInstance>,
    pub points: Vec<PointInstance>,
}

impl Layer {
    fn line(&mut self, a: V3, b: V3, color: Rgba, width: f32) {
        self.lines.push(line(a, b, color, width));
    }

    fn polyline(&mut self, points: &[V3], color: Rgba, width: f32) {
        for w in points.windows(2) {
            self.line(w[0], w[1], color, width);
        }
    }

    /// A polyline dashed `dash` on and `gap` off, in pixels, `per_pixel`
    /// world a pixel.
    fn dashed(
        &mut self,
        points: &[V3],
        color: Rgba,
        width: f32,
        dash: f32,
        gap: f32,
        per_pixel: f64,
    ) {
        let mut start = 0.0;
        for w in points.windows(2) {
            let mut l = line(w[0], w[1], color, width);
            l.start = start as f32;
            l.dash = dash;
            l.gap = gap;
            l.per_pixel = per_pixel as f32;
            start += length(sub(w[1], w[0]));
            self.lines.push(l);
        }
    }

    fn point(&mut self, at: V3, color: Rgba, size: f32) {
        self.points.push(PointInstance {
            at: f(at),
            size,
            color: color.f32(),
            ..Default::default()
        });
    }

    fn ring(&mut self, at: V3, color: Rgba, size: f32) {
        self.points.push(PointInstance {
            at: f(at),
            size,
            color: color.f32(),
            ring: 1.0,
            ..Default::default()
        });
    }

    fn triangle(&mut self, corners: [V3; 3], color: Rgba) {
        let color = color.f32();
        for p in corners {
            self.triangles.push(FlatVertex {
                position: f(p),
                color,
            });
        }
    }

    fn is_empty(&self) -> bool {
        self.triangles.is_empty() && self.lines.is_empty() && self.points.is_empty()
    }
}

fn f(p: V3) -> [f32; 3] {
    p.map(|x| x as f32)
}

fn line(a: V3, b: V3, color: Rgba, width: f32) -> LineInstance {
    LineInstance {
        a: f(a),
        b: f(b),
        width,
        color: color.f32(),
        ..Default::default()
    }
}

/// Text drawn over the view, centred at a point of it.
#[derive(Clone, Debug, PartialEq)]
pub struct Label {
    pub at: V3,
    pub text: String,
    pub style: Style,
}

/// The faces, edges, sketches and vertices of the part and its placed
/// parts, as lit: kept while the scene and what is lit stay as they are.
#[derive(Debug, Default)]
pub struct Statics {
    /// What it was made of — the scene and what is lit: drawn again only
    /// when that changes.
    pub key: u64,
    /// Every face drawn, by its mesh's key: each mesh is drawn once with
    /// every place and colour it is drawn in.
    pub faces: Vec<(String, Arc<MeshData>, Vec<FaceInstance>)>,
    /// The scene's version the edges and sketches were drawn of: they do
    /// not change with what is lit, so a hover over a large assembly does
    /// not draw them again.
    pub base_key: u64,
    /// The edges and sketches, depth tested.
    pub base: Arc<Layer>,
    /// What is lit of the edges, vertices and sketches, over everything.
    pub lit: Layer,
}

/// A section view: what lies on the side `normal` points to is not drawn,
/// and solids cut open are capped with a square `size` across.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cut {
    pub origin: V3,
    pub normal: V3,
    pub size: f64,
}

/// Everything the view draws in one frame.
#[derive(Debug, Default)]
pub struct Drawing {
    pub statics: Arc<Statics>,
    /// The grid, under everything.
    pub grid: Layer,
    /// Datum planes and axes: seen through, behind what is in front.
    pub translucent: Layer,
    /// Drawn over everything, in order: datums' points and frames, the
    /// step's visuals, the gizmo.
    pub overlays: Vec<Layer>,
    pub labels: Vec<Label>,
    /// What the gizmo says of a drag under way, and where.
    pub readout: Option<(V3, String)>,
    pub cut: Option<Cut>,
}

/// A transform as the rows of an affine map, for [`FaceInstance::rows`].
fn rows(frame: &Frame) -> [[f32; 4]; 3] {
    std::array::from_fn(|r| {
        [
            frame.u[r] as f32,
            frame.v[r] as f32,
            frame.w[r] as f32,
            frame.origin[r] as f32,
        ]
    })
}

/// Whether what bounds `faces` of `solid` is shown with `hidden` hidden:
/// of no solid, it goes when its faces all do.
fn shown(hidden: &BTreeSet<&str>, solid: Option<&str>, faces: &[String]) -> bool {
    match solid {
        Some(solid) => !hidden.contains(solid),
        None => faces.is_empty() || faces.iter().any(|f| !hidden.contains(f.as_str())),
    }
}

/// What one part (or placed part) contributes to [`Statics`].
struct PartStatics<'a> {
    part: &'a PartData,
    frame: Frame,
    /// Its solids and loose faces hidden.
    hidden: BTreeSet<&'a str>,
    /// Its sketches hidden — unless lit.
    hidden_sketches: BTreeSet<&'a str>,
    lit: Lit,
}

impl PartStatics<'_> {
    /// Its faces, coloured as lit, and what is lit of the rest.
    fn draw_lit(
        &self,
        meshes: &HashMap<String, Arc<MeshData>>,
        faces: &mut HashMap<String, (Arc<MeshData>, Vec<FaceInstance>)>,
        lit: &mut Layer,
    ) {
        let part = self.part;
        let at = |p: V3| self.frame.point(p);
        let color = part
            .color
            .as_deref()
            .and_then(|c| c.strip_prefix('#'))
            .filter(|c| c.len() == 6)
            .and_then(|c| u32::from_str_radix(c, 16).ok())
            .map_or(FACE, Rgba::hex);
        let instance_rows = rows(&self.frame);
        for face in &part.faces {
            let hidden = match &face.solid {
                Some(solid) => self.hidden.contains(solid.as_str()),
                None => self.hidden.contains(face.name.as_str()),
            };
            if hidden {
                continue;
            }
            let Some(mesh) = meshes.get(&face.mesh) else {
                continue;
            };
            let is_lit =
                (self.lit).face(&face.name, face.solid.as_deref(), face.feature.as_deref());
            let color = if is_lit {
                color.tint(HIGHLIGHT, 0.7)
            } else {
                color
            };
            let entry =
                (faces.entry(face.mesh.clone())).or_insert_with(|| (mesh.clone(), Vec::new()));
            entry.1.push(FaceInstance {
                rows: instance_rows,
                color: color.f32(),
            });
        }
        if self.lit.edges.is_empty()
            && self.lit.vertices.is_empty()
            && self.lit.sketches.is_empty()
            && self.lit.curves.is_empty()
            && self.lit.sketch_points.is_empty()
        {
            return;
        }
        for edge in &part.edges {
            if self.lit.edges.contains(&edge.name)
                && shown(&self.hidden, edge.solid.as_deref(), &edge.faces)
            {
                let points: Vec<V3> = edge.polyline.iter().map(|&p| at(p)).collect();
                lit.polyline(&points, HIGHLIGHT, 2.5);
            }
        }
        for vertex in &part.vertices {
            if self.lit.vertices.contains(&vertex.name)
                && shown(&self.hidden, vertex.solid.as_deref(), &vertex.faces)
            {
                lit.point(at(vertex.at), HIGHLIGHT, 9.0);
            }
        }
        for sketch in &part.sketches {
            let whole = self.lit.sketches.contains(&sketch.name);
            for curve in &sketch.curves {
                let points = || curve.polyline.iter().map(|&p| at(p)).collect::<Vec<V3>>();
                if whole
                    || self
                        .lit
                        .curves
                        .contains(&(sketch.name.clone(), curve.id.clone()))
                {
                    lit.polyline(&points(), HIGHLIGHT, 2.5);
                }
            }
            for (id, p) in &sketch.points {
                if whole
                    || self
                        .lit
                        .sketch_points
                        .contains(&(sketch.name.clone(), id.clone()))
                {
                    lit.point(at(*p), HIGHLIGHT, 9.0);
                }
            }
        }
    }

    /// Its edges, vertices, annotations and sketches, as they are drawn
    /// whatever is lit.
    fn draw_base(&self, base: &mut Layer) {
        let part = self.part;
        let at = |p: V3| self.frame.point(p);
        for edge in &part.edges {
            if shown(&self.hidden, edge.solid.as_deref(), &edge.faces) {
                let points: Vec<V3> = edge.polyline.iter().map(|&p| at(p)).collect();
                base.polyline(&points, EDGE, 1.2);
            }
        }
        for vertex in &part.vertices {
            if shown(&self.hidden, vertex.solid.as_deref(), &vertex.faces) {
                base.point(at(vertex.at), VERTEX, 3.0);
            }
        }
        for annotation in &part.annotations {
            let points: Vec<V3> = annotation.polyline.iter().map(|&p| at(p)).collect();
            base.polyline(&points, ANNOTATION, 1.2);
        }
        for sketch in &part.sketches {
            if self.hidden_sketches.contains(sketch.name.as_str()) {
                continue;
            }
            for curve in &sketch.curves {
                let points: Vec<V3> = curve.polyline.iter().map(|&p| at(p)).collect();
                let color = if curve.construction {
                    CONSTRUCTION
                } else {
                    SKETCH
                };
                base.polyline(&points, color, 1.6);
            }
            for (_, p) in &sketch.points {
                base.point(at(*p), SKETCH, 5.0);
            }
        }
    }
}

/// What the view shows of the controller's state: everything but the
/// scene itself, which is `kept`.
pub struct Shown<'a> {
    pub kept: &'a Kept,
    pub presentation: Option<&'a Presentation>,
    /// What the editor says is lit.
    pub lit: &'a Lighting,
    pub plane: Option<Frame>,
    pub section: Option<([f64; 3], [f64; 3])>,
    pub dark: bool,
}

/// What `shown` draws, seen by `camera`; `statics` is what the last frame
/// kept of the part, used again if nothing it is made of changed.
pub fn drawing(shown: &Shown, camera: &Camera, statics: &Arc<Statics>) -> Drawing {
    let kept = shown.kept;
    let visuals: &[Visual] = shown.presentation.map_or(&[], |p| &p.visuals);
    let statics = {
        let key = statics_key(shown);
        if statics.key == key && key != 0 {
            statics.clone()
        } else {
            Arc::new(make_statics(shown, key, statics))
        }
    };
    let part = &kept.part;
    let mut drawing = Drawing {
        statics,
        ..Default::default()
    };
    if let Some(plane) = shown.plane
        && shown.presentation.is_none_or(|p| p.sheet.is_none())
    {
        drawing.grid = grid(&plane, camera, shown.dark);
    }
    let pickable: Vec<DatumKind> = match shown.presentation {
        Some(p) if p.pickable.iter().any(|k| k == "datum" || k == "frame") => {
            vec![
                DatumKind::Point,
                DatumKind::Axis,
                DatumKind::Plane,
                DatumKind::Frame,
            ]
        }
        _ => Vec::new(),
    };
    let mut on_top = Layer::default();
    // The part's own datums, and those of its placed parts, where they are.
    let mut datums: Vec<(String, DatumKind, Frame)> = Vec::new();
    let mut lit_datums: BTreeSet<String> = BTreeSet::new();
    if let Some(own) = shown.lit.get("") {
        lit_datums.extend(own.datums.iter().cloned());
    }
    for d in &part.datums {
        datums.push((d.name.clone(), d.kind, d.frame));
    }
    for instance in kept.instances.values() {
        let Some(view) = kept.components.get(&instance.component) else {
            continue;
        };
        if let Some(lit) = shown.lit.get(&instance.name) {
            lit_datums.extend(lit.datums.iter().map(|d| format!("/{}:{d}", instance.name)));
        }
        for d in &view.datums {
            let frame = Frame {
                origin: instance.frame.point(d.frame.origin),
                u: instance.frame.direction(d.frame.u),
                v: instance.frame.direction(d.frame.v),
                w: instance.frame.direction(d.frame.w),
            };
            datums.push((format!("/{}:{}", instance.name, d.name), d.kind, frame));
        }
    }
    let hidden: BTreeSet<&str> = kept.hidden.iter().map(String::as_str).collect();
    for (name, kind, frame) in &datums {
        let is_lit = |element: &str| lit_datums.contains(element);
        let any_lit = lit_datums
            .iter()
            .any(|l| l == name || l.starts_with(&format!("{name}:")));
        if hidden.contains(name.as_str()) && !any_lit {
            continue;
        }
        let fade = |kinds: &[DatumKind], element: &str| -> (bool, f32) {
            let lit = is_lit(element);
            let factor = if lit {
                2.5
            } else if pickable.is_empty() || kinds.iter().any(|k| pickable.contains(k)) {
                1.0
            } else {
                0.3
            };
            (lit, factor)
        };
        datum(
            &mut drawing.translucent,
            &mut on_top,
            name,
            *kind,
            frame,
            part.size,
            part.center,
            camera,
            &fade,
        );
    }
    drawing.overlays.push(on_top);

    let mut visual_layer = Layer::default();
    for visual in visuals {
        draw_visual(visual, camera, &mut visual_layer, &mut drawing.labels);
    }
    drawing.overlays.push(visual_layer);

    if let Some(gizmo) = shown.presentation.and_then(|p| p.gizmo.as_ref()) {
        let mut layer = Layer::default();
        drawing.readout = draw_gizmo(gizmo, camera, &mut layer);
        drawing.overlays.push(layer);
    }
    drawing.overlays.retain(|l| !l.is_empty());
    drawing.cut = shown.section.map(|(origin, normal)| Cut {
        origin,
        normal,
        size: part.size * 3.0,
    });
    drawing
}

/// What the faces and edges drawn are made of, hashed: the scene and what
/// is lit.
fn statics_key(shown: &Shown) -> u64 {
    let mut hasher = std::hash::DefaultHasher::new();
    shown.kept.version.hash(&mut hasher);
    shown.lit.hash(&mut hasher);
    hasher.finish().max(1)
}

fn make_statics(shown: &Shown, key: u64, before: &Statics) -> Statics {
    let kept = shown.kept;
    let hidden: BTreeSet<&str> = kept.hidden.iter().map(String::as_str).collect();
    let mut faces = HashMap::new();
    let base_key = kept.version.max(1);
    let fresh = before.base_key != base_key;
    let mut base = Layer::default();
    let mut lit = Layer::default();
    let own = PartStatics {
        part: &kept.part,
        frame: Frame::WORLD,
        hidden: hidden.clone(),
        hidden_sketches: hidden.clone(),
        lit: shown.lit.get("").cloned().unwrap_or_default(),
    };
    own.draw_lit(&kept.meshes, &mut faces, &mut lit);
    if fresh {
        own.draw_base(&mut base);
    }
    for instance in kept.instances.values() {
        if hidden.contains(instance.name.as_str()) {
            continue;
        }
        let Some(view) = kept.components.get(&instance.component) else {
            continue;
        };
        placed(
            instance,
            view,
            shown,
            &kept.meshes,
            &mut faces,
            fresh.then_some(&mut base),
            &mut lit,
        );
    }
    let base = if fresh {
        Arc::new(base)
    } else {
        before.base.clone()
    };
    let mut faces: Vec<_> = faces.into_iter().map(|(k, (m, i))| (k, m, i)).collect();
    faces.sort_by(|a, b| a.0.cmp(&b.0));
    Statics {
        key,
        faces,
        base_key,
        base,
        lit,
    }
}

#[allow(clippy::too_many_arguments)]
fn placed(
    instance: &InstanceData,
    view: &PartData,
    shown: &Shown,
    meshes: &HashMap<String, Arc<MeshData>>,
    faces: &mut HashMap<String, (Arc<MeshData>, Vec<FaceInstance>)>,
    base: Option<&mut Layer>,
    lit: &mut Layer,
) {
    let prefix = format!("/{}:", instance.name);
    let hidden_sketches = (shown.kept.hidden.iter())
        .filter_map(|h| h.strip_prefix(&prefix))
        .collect();
    let statics = PartStatics {
        part: view,
        frame: instance.frame,
        hidden: BTreeSet::new(),
        hidden_sketches,
        lit: shown.lit.get(&instance.name).cloned().unwrap_or_default(),
    };
    statics.draw_lit(meshes, faces, lit);
    if let Some(base) = base {
        statics.draw_base(base);
    }
}

// ── shapes, at unit size, put where they go ─────────────────────────────

/// Put at `origin`, its `x`, `y`, `z` along `axes`, scaled by `k`.
struct Place {
    origin: V3,
    axes: [V3; 3],
    k: f64,
}

impl Place {
    fn at(&self, p: V3) -> V3 {
        let [x, y, z] = self.axes;
        add(
            self.origin,
            scale(
                add(add(scale(x, p[0]), scale(y, p[1])), scale(z, p[2])),
                self.k,
            ),
        )
    }
}

/// Two unit vectors square to `d` and to each other, right-handed with it.
fn square_to(d: V3) -> (V3, V3) {
    let other = if d[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let a = normalized(cross(d, other));
    (a, cross(d, a))
}

/// A sphere of radius `r` about `c`, in the place.
fn sphere(layer: &mut Layer, place: &Place, c: V3, r: f64, color: Rgba) {
    const SEG: usize = 16;
    const RINGS: usize = 10;
    let p = |i: usize, j: usize| {
        let theta = std::f64::consts::PI * j as f64 / RINGS as f64;
        let phi = std::f64::consts::TAU * i as f64 / SEG as f64;
        place.at(add(
            c,
            scale(
                [
                    theta.sin() * phi.cos(),
                    theta.cos(),
                    theta.sin() * phi.sin(),
                ],
                r,
            ),
        ))
    };
    for j in 0..RINGS {
        for i in 0..SEG {
            let (a, b, cc, d) = (p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1));
            layer.triangle([a, b, cc], color);
            layer.triangle([a, cc, d], color);
        }
    }
}

/// A cone (`r1` 0) or cylinder along `dir` from `from`, `h` long, of radius
/// `r0` at its foot and `r1` at its top.
#[allow(clippy::too_many_arguments)]
fn frustum(
    layer: &mut Layer,
    place: &Place,
    from: V3,
    dir: V3,
    h: f64,
    r0: f64,
    r1: f64,
    color: Rgba,
) {
    const SEG: usize = 14;
    let (a, b) = square_to(dir);
    let ring = |i: usize, r: f64, t: f64| {
        let phi = std::f64::consts::TAU * i as f64 / SEG as f64;
        place.at(add(
            add(from, scale(dir, t)),
            add(scale(a, r * phi.cos()), scale(b, r * phi.sin())),
        ))
    };
    let foot = place.at(from);
    let top = place.at(add(from, scale(dir, h)));
    for i in 0..SEG {
        let (p0, p1) = (ring(i, r0, 0.0), ring(i + 1, r0, 0.0));
        let (q0, q1) = (ring(i, r1, h), ring(i + 1, r1, h));
        layer.triangle([p0, p1, q1], color);
        layer.triangle([p0, q1, q0], color);
        layer.triangle([foot, p1, p0], color);
        if r1 > 0.0 {
            layer.triangle([top, q0, q1], color);
        }
    }
}

/// A square `size` across about `c`, in the plane of `a` and `b`.
fn square(
    layer: &mut Layer,
    place: &Place,
    c: V3,
    a: V3,
    b: V3,
    size: f64,
    color: Rgba,
) -> [V3; 4] {
    let h = size / 2.0;
    let corner = |x: f64, y: f64| place.at(add(c, add(scale(a, x * h), scale(b, y * h))));
    let q = [
        corner(-1.0, -1.0),
        corner(1.0, -1.0),
        corner(1.0, 1.0),
        corner(-1.0, 1.0),
    ];
    layer.triangle([q[0], q[1], q[2]], color);
    layer.triangle([q[0], q[2], q[3]], color);
    q
}

fn cube(layer: &mut Layer, place: &Place, c: V3, h: f64, color: Rgba) {
    for axis in 0..3 {
        let n = std::array::from_fn(|k| if k == axis { 1.0 } else { 0.0 });
        let (a, b) = square_to(n);
        for sign in [1.0, -1.0] {
            square(
                layer,
                place,
                add(c, scale(n, sign * h)),
                a,
                b,
                2.0 * h,
                color,
            );
        }
    }
}

fn torus(layer: &mut Layer, place: &Place, axis: V3, radius: f64, tube: f64, color: Rgba) {
    const SEG: usize = 64;
    const SIDES: usize = 6;
    let (a, b) = square_to(axis);
    let p = |i: usize, j: usize| {
        let phi = std::f64::consts::TAU * i as f64 / SEG as f64;
        let theta = std::f64::consts::TAU * j as f64 / SIDES as f64;
        let out = add(scale(a, phi.cos()), scale(b, phi.sin()));
        let c = scale(out, radius);
        place.at(add(
            c,
            add(
                scale(out, tube * theta.cos()),
                scale(axis, tube * theta.sin()),
            ),
        ))
    };
    for i in 0..SEG {
        for j in 0..SIDES {
            let (q0, q1, q2, q3) = (p(i, j), p(i + 1, j), p(i + 1, j + 1), p(i, j + 1));
            layer.triangle([q0, q1, q2], color);
            layer.triangle([q0, q2, q3], color);
        }
    }
}

// ── datums (`datums3d.ts`) ──────────────────────────────────────────────

const DATUM: Rgba = Rgba::hex(0xb58cff);
const LIT: Rgba = Rgba::hex(0xffc94a);
const AXIS_TINTS: [Rgba; 3] = [
    Rgba::hex(0xff8080),
    Rgba::hex(0x80e080),
    Rgba::hex(0x80b0ff),
];
const FRAME_AXES: [Rgba; 3] = [
    Rgba::hex(0xff5555),
    Rgba::hex(0x55dd55),
    Rgba::hex(0x5599ff),
];
/// A point datum's axes, in pixels.
const TRIAD_PX: f64 = 32.0;

/// A datum: a plane as a translucent square `size` across about the point
/// of it nearest `center`, an axis as a dashed line that long, a point as
/// its own small triad, a frame as a ball, an arrow along each axis and a
/// square in each of its planes, each picked on its own and lit or faded
/// on its own (`fade`, by the kinds it picks as and its element's name).
#[allow(clippy::too_many_arguments)]
fn datum(
    translucent: &mut Layer,
    on_top: &mut Layer,
    name: &str,
    kind: DatumKind,
    frame: &Frame,
    size: f64,
    center: V3,
    camera: &Camera,
    fade: &Fade<'_>,
) {
    let paint = |base: Rgba, opacity: f32, (lit, factor): (bool, f32)| {
        let color = if lit { LIT } else { base };
        color.with_alpha(opacity * factor)
    };
    let offset = sub(center, frame.origin);
    match kind {
        DatumKind::Plane => {
            let origin = sub(
                add(frame.origin, offset),
                scale(frame.w, dot(offset, frame.w)),
            );
            let look = fade(&[DatumKind::Plane], name);
            let place = Place {
                origin: [0.0; 3],
                axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                k: 1.0,
            };
            let q = square(
                translucent,
                &place,
                origin,
                frame.u,
                frame.v,
                size,
                paint(DATUM, 0.12, look),
            );
            let edge = paint(DATUM, 0.8, look);
            for i in 0..4 {
                translucent.line(q[i], q[(i + 1) % 4], edge, 1.2);
            }
        }
        DatumKind::Axis => {
            let origin = add(frame.origin, scale(frame.w, dot(offset, frame.w)));
            let look = fade(&[DatumKind::Axis], name);
            let ends = [
                sub(origin, scale(frame.w, size)),
                add(origin, scale(frame.w, size)),
            ];
            // Dashes `size / 30` long, gaps half that, in the world.
            translucent.dashed(&ends, paint(DATUM, 0.9, look), 1.5, 2.0, 1.0, size / 60.0);
        }
        DatumKind::Point => {
            let k = camera.world_per_pixel(frame.origin) * TRIAD_PX;
            let place = Place {
                origin: frame.origin,
                axes: [frame.u, frame.v, frame.w],
                k,
            };
            let look = fade(&[DatumKind::Point], name);
            sphere(on_top, &place, [0.0; 3], 0.12, paint(DATUM, 1.0, look));
            for (axis, tint) in AXIS_TINTS.iter().enumerate() {
                let end = std::array::from_fn(|i| if i == axis { 1.0 } else { 0.0 });
                on_top.line(
                    place.at([0.0; 3]),
                    place.at(end),
                    paint(*tint, 1.0, look),
                    1.5,
                );
            }
        }
        DatumKind::Frame => {
            let k = camera.world_per_pixel(frame.origin) * 10.0 * REACH_PX;
            let place = Place {
                origin: frame.origin,
                axes: [frame.u, frame.v, frame.w],
                k,
            };
            let ball = fade(&[DatumKind::Point, DatumKind::Frame], name);
            sphere(
                on_top,
                &place,
                [0.0; 3],
                0.07,
                paint(Rgba::hex(0xffffff), 1.0, ball),
            );
            for (axis, (color, (axis_name, plane_name))) in FRAME_AXES
                .iter()
                .zip([("x", "yz"), ("y", "zx"), ("z", "xy")])
                .enumerate()
            {
                let dir: V3 = std::array::from_fn(|i| if i == axis { 1.0 } else { 0.0 });
                let look = fade(&[DatumKind::Axis], &format!("{name}:{axis_name}"));
                let c = paint(*color, 1.0, look);
                frustum(
                    on_top,
                    &place,
                    scale(dir, 0.075),
                    dir,
                    0.85,
                    0.015,
                    0.015,
                    c,
                );
                frustum(on_top, &place, scale(dir, 0.85), dir, 0.15, 0.05, 0.0, c);
                let look = fade(&[DatumKind::Plane], &format!("{name}:{plane_name}"));
                let [a, b] = [(axis + 1) % 3, (axis + 2) % 3]
                    .map(|o| -> V3 { std::array::from_fn(|i| if i == o { 1.0 } else { 0.0 }) });
                let mid = scale(add(a, b), 0.45);
                let q = square(on_top, &place, mid, a, b, 0.3, paint(*color, 0.3, look));
                let edge = paint(*color, 1.0, look);
                for i in 0..4 {
                    on_top.line(q[i], q[(i + 1) % 4], edge, 1.2);
                }
            }
        }
    }
}

// ── the step's visuals (`visuals3d.ts`) ─────────────────────────────────

/// Whether a part of a datum, picked as `kinds`, named `element`, is lit,
/// and by how much its opacity is multiplied.
type Fade<'a> = dyn Fn(&[DatumKind], &str) -> (bool, f32) + 'a;

/// How big a point is on screen, in pixels.
const POINT_PX: f32 = 7.0;

fn v(p: &Vector3) -> V3 {
    crate::scene::v3(p)
}

fn draw_visual(visual: &Visual, camera: &Camera, layer: &mut Layer, labels: &mut Vec<Label>) {
    let color = style_color(visual.style);
    let dashed = matches!(
        visual.style,
        Style::Construction | Style::Draft | Style::Guide | Style::Reference
    );
    match &visual.shape {
        Shape::Point { at } => {
            if visual.style == Style::Snap {
                layer.ring(v(at), color, 12.0);
            } else {
                let size = match visual.style {
                    Style::Selected | Style::Hover => POINT_PX + 3.0,
                    _ => POINT_PX,
                };
                layer.point(v(at), color, size);
            }
        }
        Shape::Polyline { points } => {
            let points: Vec<V3> = points.iter().map(v).collect();
            if dashed {
                let per_pixel = camera.world_per_pixel(camera.pose.target);
                let (dash, gap) = if visual.style == Style::Reference {
                    (2.0, 3.0)
                } else {
                    (6.0, 4.0)
                };
                layer.dashed(&points, color, 1.8, dash, gap, per_pixel);
            } else {
                let width = if matches!(visual.style, Style::Selected | Style::Hover) {
                    2.6
                } else {
                    1.8
                };
                layer.polyline(&points, color, width);
            }
        }
        Shape::Triangles { triangles } => {
            for t in triangles {
                layer.triangle(t.map(|p| v(&p)), color.with_alpha(0.15));
            }
        }
        Shape::Label { at, text, offset } => {
            let at = v(at);
            let reach = camera.reach_at(at);
            labels.push(Label {
                at: add(at, scale(v(offset), reach)),
                text: text.clone(),
                style: visual.style,
            });
        }
        Shape::Instance { .. } => {}
        Shape::Handle { at, direction } => {
            let at = v(at);
            let k = camera.world_per_pixel(at) * 0.8 * REACH_PX;
            let place = Place {
                origin: at,
                axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                k,
            };
            sphere(layer, &place, [0.0; 3], 1.0, color);
            let dir = normalized(v(direction));
            if length(v(direction)) > 0.0 {
                for sign in [1.0, -1.0] {
                    let d = scale(dir, sign);
                    frustum(layer, &place, scale(d, 1.4), d, 1.6, 0.8, 0.0, color);
                }
            }
        }
        Shape::Triad { at, axes } => {
            let at = v(at);
            let k = camera.world_per_pixel(at) * REACH_PX * TRIAD;
            let place = Place {
                origin: at,
                axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                k,
            };
            for (axis, color) in axes.iter().zip(FRAME_AXES) {
                let dir = normalized(v(axis));
                frustum(layer, &place, [0.0; 3], dir, 0.8, 0.02, 0.02, color);
                frustum(layer, &place, scale(dir, 0.8), dir, 0.2, 0.06, 0.0, color);
            }
        }
    }
}

// ── the gizmo (`gizmo3d.ts`) ────────────────────────────────────────────

const GIZMO_AXES: [Rgba; 3] = [
    Rgba::hex(0xe0584f),
    Rgba::hex(0x5cc25c),
    Rgba::hex(0x4f86f0),
];
const GIZMO_PLAIN: Rgba = Rgba::hex(0xdddddd);
const GIZMO_LIT: Rgba = Rgba::hex(0xffd84a);

/// The gizmo, at a constant size on screen — a reach a unit, as the kernel
/// lays it out and hit-tests it — showing the parts the kernel says are
/// seen from where the eye looks along: what the pointer is over or drags
/// lit, the rest faded while a drag is under way. What a drag has done,
/// and where to say it.
fn draw_gizmo(gizmo: &GizmoView, camera: &Camera, layer: &mut Layer) -> Option<(V3, String)> {
    let at = v(&gizmo.at);
    let reach = camera.reach_at(at);
    let axes = gizmo.axes.each_ref().map(v);
    let place = Place {
        origin: at,
        axes,
        k: reach,
    };
    let eye = match camera.projection {
        crate::camera::Projection::Orthographic => neg(camera.pose.back()),
        crate::camera::Projection::Perspective => normalized(sub(at, camera.pose.position)),
    };
    let dir = Vector3::from_array(eye.map(Scalar::from_f64));
    let unit = |i: usize| -> V3 { std::array::from_fn(|k| if k == i { 1.0 } else { 0.0 }) };
    for &part in &gizmo.parts {
        if !gizmo.shown(part, &dir) {
            continue;
        }
        let axis = match part {
            GizmoPart::Move(a)
            | GizmoPart::Plane(a)
            | GizmoPart::Turn(a)
            | GizmoPart::Stretch(a) => Some(a as usize),
            GizmoPart::Free | GizmoPart::Scale => None,
        };
        let lit =
            gizmo.active == Some(part) || (gizmo.active.is_none() && gizmo.hover == Some(part));
        let faded = gizmo.active.is_some() && !lit;
        let base = axis.map_or(GIZMO_PLAIN, |a| GIZMO_AXES[a]);
        let color = |opacity: f32| {
            let c = if lit { GIZMO_LIT } else { base };
            c.with_alpha(if faded { 0.15 } else { opacity })
        };
        match part {
            GizmoPart::Free => sphere(layer, &place, [0.0; 3], gizmo_size::FREE, color(0.55)),
            GizmoPart::Move(_) => {
                let d = unit(axis.unwrap_or(0));
                let (from, tip) = gizmo_size::ARROW;
                let head = 2.0;
                frustum(
                    layer,
                    &place,
                    scale(d, from),
                    d,
                    tip - head - from,
                    0.18,
                    0.18,
                    color(0.9),
                );
                frustum(
                    layer,
                    &place,
                    scale(d, tip - head),
                    d,
                    head,
                    0.65,
                    0.0,
                    color(0.9),
                );
            }
            GizmoPart::Plane(_) => {
                let i = axis.unwrap_or(0);
                let (lo, hi) = gizmo_size::PLANE;
                let (a, b) = (unit((i + 1) % 3), unit((i + 2) % 3));
                let mid = scale(add(a, b), (lo + hi) / 2.0);
                square(layer, &place, mid, a, b, hi - lo, color(0.45));
            }
            GizmoPart::Turn(_) => {
                torus(
                    layer,
                    &place,
                    unit(axis.unwrap_or(0)),
                    gizmo_size::RING,
                    0.16,
                    color(0.9),
                );
            }
            GizmoPart::Stretch(_) => {
                let c = scale(unit(axis.unwrap_or(0)), gizmo_size::CUBE_AT);
                cube(layer, &place, c, gizmo_size::CUBE, color(0.9));
            }
            GizmoPart::Scale => {
                let c = scale(normalized([1.0, 1.0, 1.0]), gizmo_size::CUBE_AT);
                cube(layer, &place, c, gizmo_size::CUBE, color(0.9));
            }
        }
    }
    gizmo.readout.clone().map(|text| (at, text))
}

// ── the grid (`planeGrid.ts`) ───────────────────────────────────────────

/// Lines on `plane` a power of ten apart, 20 to 200 pixels at the current
/// zoom, every tenth stronger, and the plane's own axes through its origin.
fn grid(plane: &Frame, camera: &Camera, dark: bool) -> Layer {
    let (minor, major) = if dark {
        (Rgba::hex(0x2c2c2c), Rgba::hex(0x3c3c3c))
    } else {
        (Rgba::hex(0xdedede), Rgba::hex(0xc4c4c4))
    };
    let target = camera.pose.target;
    let px = camera.world_per_pixel(target);
    let step = 10f64.powf((20.0 * px).log10().ceil());
    let local = sub(target, plane.origin);
    let (cu, cv) = (
        (dot(local, plane.u) / step).round(),
        (dot(local, plane.v) / step).round(),
    );
    let n = ((px * camera.size[1] * 1.2) / step).ceil() + 1.0;
    let at = |x: f64, y: f64| plane.in_plane([x, y]);
    let mut layer = Layer::default();
    let mut i = -n;
    while i <= n {
        let (x, y) = ((cu + i) * step, (cv + i) * step);
        let color = |k: f64| if (k % 10.0).abs() < 0.5 { major } else { minor };
        layer.line(
            at(x, (cv - n) * step),
            at(x, (cv + n) * step),
            color(cu + i),
            1.0,
        );
        layer.line(
            at((cu - n) * step, y),
            at((cu + n) * step, y),
            color(cv + i),
            1.0,
        );
        i += 1.0;
    }
    let span = (n + cu.abs().max(cv.abs())) * step;
    layer.line(at(-span, 0.0), at(span, 0.0), Rgba::hex(0x994444), 1.2);
    layer.line(at(0.0, -span), at(0.0, span), Rgba::hex(0x449944), 1.2);
    layer
}

/// How many segments the kernel draws a full circle with: the rings of the
/// gizmo are drawn finer, as the kernel tests them.
pub const _RESOLUTION: usize = RESOLUTION;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_colour_tints_most_of_the_way() {
        let tinted = FACE.tint(HIGHLIGHT, 0.7);
        let [r, g, b, a] = tinted.f32();
        let [hr, hg, hb, _] = HIGHLIGHT.f32();
        assert!((r - hr).abs() < 0.2 && (g - hg).abs() < 0.2 && (b - hb).abs() < 0.3);
        assert_eq!(a, 1.0);
    }

    #[test]
    fn an_edge_of_no_solid_goes_with_all_its_faces() {
        let hidden = BTreeSet::from(["a", "s"]);
        assert!(shown(&hidden, None, &["a".into(), "b".into()]));
        assert!(!shown(&hidden, None, &["a".into()]));
        assert!(!shown(&hidden, Some("s"), &[]));
        assert!(shown(&hidden, None, &[]));
    }
}
