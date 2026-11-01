//! [`picture`]: a scene drawn into a PNG as the viewer draws it, and the
//! [`Camera`] it is seen by, which also says where the pointer of a pixel
//! goes ([`Camera::pointer`]): what is clicked in a picture is what the
//! editor's hit tests pick there, as for a click in a viewer.
//!
//! A picture is drawn, not reasoned about: in plain numbers, by a
//! z-buffered rasterizer, supersampled. It draws what the web viewer draws
//! (`web/src/partScene.ts`, `placed3d.ts`, `datums3d.ts`, `visuals3d.ts`),
//! in its colours: faces shaded by a light that moves with the eye, edges
//! over them where they are seen, sketches and datums where they are not
//! hidden, what is lit over it all, then what the step being edited draws,
//! and in a corner the world's axes, `x` red, `y` green, `z` blue.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain::{self, P2, P3},
    primitives::{DatumComponent, DatumKind, FrameAxis, Motion, Ray},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::placed::{INSTANCE_SEPARATOR, Located};

use super::{FRAME, PartView, ViewDatum, lit::Lighting, lit::Lit};
use crate::{
    editing::{Pointer, Reach, Shape, Style, TRIAD, Visual},
    editor::SceneState,
};

/// How far a pointer reaches, in pixels. What a picture draws at a
/// constant size on screen — a frame datum, a handle, a triad — is laid
/// out in reaches of this many pixels, as a viewer lays it out in its own,
/// so it is hit where it is drawn.
pub const REACH_PX: f64 = 4.0;

/// Samples a pixel is drawn from, along each of its sides.
const SUPERSAMPLE: usize = 2;
/// The room a camera fitted to the scene leaves round it, of the
/// picture's height or width, on each side.
const MARGIN: f64 = 0.06;
/// How far behind the faces drawn a line still shows, in pixels' worth of
/// depth: an edge lies on its faces, which are drawn through the edge's
/// samples by triangles of their own, and at a glancing look those part by
/// several pixels' worth of depth within one pixel.
const DEPTH_BIAS_PX: f64 = 3.0;
/// Half an edge's width, in pixels.
const EDGE_HALF_WIDTH: f64 = 0.7;
/// Half a lit edge's width, in pixels.
const LIT_HALF_WIDTH: f64 = 1.2;
/// The light a face is never darker than, and what a face turned to the
/// light adds to it.
const AMBIENT: f32 = 0.4;
const DIFFUSE: f32 = 0.6;

/// The viewer's colours (see the module docs).
const BACKGROUND: u32 = 0x1b1f26;
const FACE: u32 = 0xb0b4b8;
const EDGE: u32 = 0x30343b;
const SKETCH: u32 = 0xffa040;
const CONSTRUCTION: u32 = 0x808080;
const ANNOTATION: u32 = 0x2a2a2a;
const HIGHLIGHT: u32 = 0xffc94a;
/// How far a lit face's colour goes towards [`HIGHLIGHT`], and how opaque
/// it is drawn again over whatever stands in front of it.
const LIT_TINT: f32 = 0.7;
const LIT_FACE_OPACITY: f32 = 0.35;
const DATUM: u32 = 0xb58cff;
/// A frame's axes, `x`, `y`, `z`, and a point datum's, tinted so.
const AXES: [u32; 3] = [0xff5555, 0x55dd55, 0x5599ff];
const AXIS_TINTS: [u32; 3] = [0xff8080, 0x80e080, 0x80b0ff];
/// How long a point datum's axes are, in reaches: 32 of a viewer's pixels.
const POINT_TRIAD: f64 = 32.0 / 9.0;

/// An orthographic camera: what a picture shows and where a pixel's
/// pointer goes.
#[derive(Clone, Debug, PartialEq)]
pub struct Camera {
    /// The direction the eye looks along: unit.
    pub look: Vector3,
    /// Up the picture: unit, square to `look`.
    pub up: Vector3,
    /// The world point at the picture's centre.
    pub center: Vector3,
    /// The world height the picture shows.
    pub span: f64,
    /// Its size in pixels.
    pub width: u32,
    pub height: u32,
    /// How far before `center`, along `look`, a pointer's ray starts: in
    /// front of everything drawn.
    pub near: f64,
}

/// A camera's frame in plain numbers: right, up and into the picture, its
/// centre, and the world length of a pixel.
#[derive(Clone, Copy, Debug)]
struct Frame {
    right: P3,
    up: P3,
    look: P3,
    center: P3,
    per_px: f64,
}

impl Frame {
    /// Where `p` is along right, up and into the picture, from its centre.
    fn coordinates(&self, p: P3) -> P3 {
        let d = plain::sub(p, self.center);
        [
            plain::dot(d, self.right),
            plain::dot(d, self.up),
            plain::dot(d, self.look),
        ]
    }

    /// The direction `d` on screen, right and down, a pixel a unit of it.
    fn on_screen(&self, d: P3) -> P2 {
        [plain::dot(d, self.right), -plain::dot(d, self.up)]
    }
}

/// The world's axis `k`.
fn unit(k: usize) -> P3 {
    std::array::from_fn(|i| f64::from(u8::from(i == k)))
}

/// `up` made square to the unit `look`: if it runs (nearly) along it, the
/// world's axis `look` runs least along, the first of those it runs least
/// along alike, made square to it instead.
fn square_up(look: P3, up: P3) -> P3 {
    let square = |up: P3| {
        let up = plain::normalize(up)?;
        let square = plain::sub(up, plain::scale(look, plain::dot(up, look)));
        (plain::norm(square) > 1e-3).then(|| plain::normalize(square))?
    };
    square(up).unwrap_or_else(|| {
        let least = (look.iter().enumerate()).fold((0, f64::INFINITY), |least, (k, x)| {
            match x.abs() < least.1 {
                true => (k, x.abs()),
                false => least,
            }
        });
        let axis = unit(least.0);
        square(axis).unwrap_or(axis)
    })
}

impl Camera {
    /// Looking along `look` with `up` up — made square to `look`, or, if
    /// it runs along `look`, the world's axis `look` runs least along —
    /// framed to everything the scene draws, the part and every part
    /// placed in it, with a margin; to the part's extent where it draws
    /// nothing.
    pub fn fitting(
        scene: &SceneState,
        look: Vector3,
        up: Vector3,
        width: u32,
        height: u32,
    ) -> GeopResult<Camera> {
        if width == 0 || height == 0 {
            return Err(GeopError::new(format!(
                "a picture of {width} by {height} pixels shows nothing"
            )));
        }
        let look = plain::normalize(look.to_f64_array()).ok_or_else(|| {
            GeopError::new(format!("a camera looking along {look:?} looks nowhere"))
        })?;
        let up = square_up(look, up.to_f64_array());
        let right = plain::cross(look, up);
        let frame = Frame {
            right,
            up,
            look,
            center: [0.0; 3],
            per_px: 1.0,
        };
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        let hidden = hidden_of(scene);
        for layer in &layers(scene, None, &hidden) {
            layer.points(|p| {
                let c = frame.coordinates(p);
                for ((low, high), c) in low.iter_mut().zip(&mut high).zip(c) {
                    *low = low.min(c);
                    *high = high.max(c);
                }
            });
        }
        let extent = &scene.part.extent;
        let size = extent.size.to_f64();
        let aspect = f64::from(height) / f64::from(width);
        let (middle, span) = match low[0] <= high[0] {
            true => {
                let middle = plain::scale(plain::add(low, high), 0.5);
                let span = (high[1] - low[1]).max((high[0] - low[0]) * aspect);
                (middle, span / (1.0 - 2.0 * MARGIN))
            }
            false => {
                let c = frame.coordinates(extent.center.to_f64_array());
                low[2] = c[2] - size / 2.0;
                (c, size)
            }
        };
        let span = match span > 0.0 && span.is_finite() {
            true => span,
            false => size,
        };
        let center = ([right, up, look].into_iter().zip(middle)).fold([0.0; 3], |c, (axis, x)| {
            plain::add(c, plain::scale(axis, x))
        });
        let v = |p: P3| Vector3::from_array(p.map(Scalar::from_f64));
        Ok(Camera {
            look: v(look),
            up: v(up),
            center: v(center),
            span,
            width,
            height,
            near: middle[2] - low[2] + span,
        })
    }

    fn frame(&self) -> Frame {
        let look = self.look.to_f64_array();
        let up = self.up.to_f64_array();
        Frame {
            right: plain::cross(look, up),
            up,
            look,
            center: self.center.to_f64_array(),
            per_px: self.span / f64::from(self.height),
        }
    }

    /// How far the pointer reaches: [`REACH_PX`] pixels.
    fn reach(&self) -> Reach {
        Reach::Tube {
            radius: Scalar::from_f64(REACH_PX * self.frame().per_px),
        }
    }

    /// The pointer of the point `(x, y)` of the picture, in pixels from
    /// its top left corner, `y` down — the middle of the pixel `(i, j)` is
    /// `(i + 0.5, j + 0.5)`: a ray along `look` from in front of
    /// everything drawn, reaching [`REACH_PX`] pixels, meaning the first
    /// of what lies under it. What the editor's hit tests take.
    pub fn pointer(&self, x: f64, y: f64) -> GeopResult<Pointer> {
        let f = self.frame();
        let across = (x - f64::from(self.width) / 2.0) * f.per_px;
        let down = (y - f64::from(self.height) / 2.0) * f.per_px;
        let at = plain::add(
            f.center,
            plain::sub(plain::scale(f.right, across), plain::scale(f.up, down)),
        );
        self.ray_through(at, &f)
    }

    /// The pointer through the world point `at`, along `look`, reaching as
    /// [`Camera::pointer`]'s does.
    pub fn through(&self, at: Vector3) -> GeopResult<Pointer> {
        self.ray_through(at.to_f64_array(), &self.frame())
    }

    fn ray_through(&self, at: P3, f: &Frame) -> GeopResult<Pointer> {
        let back = f.coordinates(at)[2] + self.near;
        let origin = plain::sub(at, plain::scale(f.look, back));
        let v = |p: P3| Vector3::from_array(p.map(Scalar::from_f64));
        Ok(Pointer {
            ray: Ray::try_new(v(origin), v(f.look))?,
            reach: self.reach(),
            cycle: 0,
        })
    }

    /// Where the world point `at` is in the picture, in pixels from its top
    /// left corner, `y` down, as [`Camera::pointer`] takes them.
    pub fn pixel(&self, at: &Vector3) -> [f64; 2] {
        let f = self.frame();
        let c = f.coordinates(at.to_f64_array());
        [
            f64::from(self.width) / 2.0 + c[0] / f.per_px,
            f64::from(self.height) / 2.0 - c[1] / f.per_px,
        ]
    }
}

/// The picture of `scene` seen by `camera`, as a PNG: its part, and every
/// part placed in it, but what `hidden` names — none, if the scene does
/// not say; what `lit` lights, lit; and `visuals`, what a step being
/// edited draws, over it all.
///
/// A placed part's colour is that of the scene's `instances`, where it is
/// one of them, else that of the part's own (see [`super::ViewInstance`]).
pub fn picture(
    scene: &SceneState,
    lit: Option<&Lighting>,
    visuals: &[Visual],
    camera: &Camera,
) -> GeopResult<Vec<u8>> {
    let frame = camera.frame();
    let (width, height) = (camera.width as usize, camera.height as usize);
    let mut raster = Raster::new(width * SUPERSAMPLE, height * SUPERSAMPLE, frame);
    let hidden = hidden_of(scene);
    let layers = layers(scene, lit, &hidden);
    let [own, ..] = layers.as_slice() else {
        return Err(GeopError::new("a scene draws its part".to_string()));
    };
    for layer in &layers {
        raster.faces(layer);
    }
    raster.datums(scene.part.as_ref(), own);
    for layer in &layers {
        raster.lines(layer);
    }
    for layer in &layers {
        raster.lit(layer);
    }
    raster.frames(scene.part.as_ref(), own);
    for visual in visuals {
        raster.visual(visual);
    }
    raster.compass();
    raster.png(width, height)
}

/// The part drawn or a part placed in it, where it is drawn, and how.
struct Layer<'s> {
    view: &'s PartView,
    /// The placed part's name: empty for the part drawn.
    instance: &'s str,
    place: Place,
    hidden: &'s BTreeSet<&'s str>,
    lit: Option<&'s Lit>,
    /// Its faces' colour.
    face: [f32; 3],
    /// Whether its instance is drawn in a colour of the user's: over the
    /// colours of its faces too.
    tinted: bool,
}

/// A rigid motion in plain numbers: where a placed part's origin and axes
/// go.
#[derive(Clone, Copy, Debug)]
struct Place {
    origin: P3,
    axes: [P3; 3],
}

impl Place {
    const IDENTITY: Place = Place {
        origin: [0.0; 3],
        axes: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };

    fn of(motion: &Motion) -> Self {
        Self {
            origin: motion.apply(&Vector3::zero()).to_f64_array(),
            axes: [0, 1, 2].map(|k| motion.rotate(&Vector3::axis(k)).to_f64_array()),
        }
    }

    fn rotate(&self, d: P3) -> P3 {
        (self.axes.iter().zip(d)).fold([0.0; 3], |r, (axis, x)| {
            plain::add(r, plain::scale(*axis, x))
        })
    }

    fn apply(&self, p: &Vector3) -> P3 {
        plain::add(self.origin, self.rotate(p.to_f64_array()))
    }
}

/// The part drawn, then every part placed in it not hidden, nor placed in
/// one hidden, each lit as `lit` says.
fn layers<'s>(
    scene: &'s SceneState,
    lit: Option<&'s Lighting>,
    hidden: &'s BTreeSet<&'s str>,
) -> Vec<Layer<'s>> {
    let part = scene.part.as_ref();
    let own = |view: &PartView| parsed(view.color.as_ref()).unwrap_or(rgb(FACE));
    let lit_of = |name: &str| lit.and_then(|l| l.get(name));
    let colors: BTreeMap<&str, Option<&String>> = (scene.instances.iter())
        .map(|i| (i.name.as_str(), i.color.as_ref()))
        .collect();
    let placed_hidden = |name: &str| {
        let mut path = name;
        loop {
            if hidden.contains(path) {
                return true;
            }
            match path.rsplit_once(INSTANCE_SEPARATOR) {
                Some((outer, _)) => path = outer,
                None => return false,
            }
        }
    };
    let mut layers = vec![Layer {
        view: part,
        instance: "",
        place: Place::IDENTITY,
        hidden,
        lit: lit_of(""),
        face: own(part),
        tinted: false,
    }];
    for instance in part.instances.iter().filter(|i| !placed_hidden(&i.name)) {
        let view = instance.view.as_ref();
        let color = colors.get(instance.name.as_str()).copied();
        let tint = parsed(color.unwrap_or(instance.color.as_ref()));
        let face = own(view);
        layers.push(Layer {
            view,
            instance: &instance.name,
            place: Place::of(&instance.pose.motion()),
            hidden,
            lit: lit_of(&instance.name),
            face: tint.map_or(face, |t| times(face, t)),
            tinted: tint.is_some(),
        });
    }
    layers
}

/// What `scene` says not to draw: nothing, where it does not say.
fn hidden_of(scene: &SceneState) -> BTreeSet<&str> {
    scene.hidden.iter().flatten().map(String::as_str).collect()
}

impl Layer<'_> {
    /// The colour of its face `name`: its own, if it has one, unless a
    /// colour its instance is drawn in is over them all.
    fn color_of(&self, name: &str) -> [f32; 3] {
        match self.tinted {
            true => self.face,
            false => parsed(self.view.face_colors.get(name)).unwrap_or(self.face),
        }
    }

    /// Whether what this layer names `name` is hidden — a solid, a face of
    /// none, a sketch, a datum — by its key in the part drawn, as a pick
    /// asks it.
    fn hides(&self, name: &str) -> bool {
        let key = Located::in_instance(self.instance, name).key();
        self.hidden.contains(key.as_str())
    }

    fn shows_face(&self, solid: &Option<String>, name: &str) -> bool {
        !self.hides(solid.as_deref().unwrap_or(name))
    }

    /// Whether a vertex or an edge of `solid`, bounding `faces`, is drawn:
    /// its solid is not hidden, or, of none, not every face it bounds is.
    fn shows_boundary(&self, solid: &Option<String>, faces: &[String]) -> bool {
        match solid {
            Some(solid) => !self.hides(solid),
            None => faces.is_empty() || faces.iter().any(|f| !self.hides(f)),
        }
    }

    fn shows_sketch(&self, name: &str) -> bool {
        self.lit.is_some_and(|l| l.sketches.contains(name)) || !self.hides(name)
    }

    /// Every point drawn of its faces, edges, sketches and annotations,
    /// where it is drawn: what a camera is fitted to.
    fn points(&self, mut at: impl FnMut(P3)) {
        let view = self.view;
        for face in view
            .faces
            .iter()
            .filter(|f| self.shows_face(&f.solid, &f.name))
        {
            for p in face.mesh.triangles.iter().flatten() {
                at(self.place.apply(p));
            }
        }
        for edge in &view.edges {
            if self.shows_boundary(&edge.solid, &edge.faces) {
                edge.polyline.iter().for_each(|p| at(self.place.apply(p)));
            }
        }
        for a in &view.annotations {
            a.polyline.iter().for_each(|p| at(self.place.apply(p)));
        }
        for sketch in view.sketches.iter().filter(|s| self.shows_sketch(&s.name)) {
            for p in sketch.curves.iter().flat_map(|c| &c.polyline) {
                at(self.place.apply(&sketch.plane.uv_to_xyz(p)));
            }
            for p in &sketch.points {
                at(self.place.apply(&sketch.plane.uv_to_xyz(&p.at)));
            }
        }
    }
}

/// The colour `hex`, `0xrrggbb`, as three numbers of one.
fn rgb(hex: u32) -> [f32; 3] {
    [16, 8, 0].map(|shift| ((hex >> shift) & 0xff) as f32 / 255.0)
}

/// The colour `#rrggbb`, if `color` is one.
fn parsed(color: Option<&String>) -> Option<[f32; 3]> {
    let hex = color?.strip_prefix('#')?;
    (hex.len() == 6)
        .then(|| u32::from_str_radix(hex, 16).ok())
        .flatten()
        .map(rgb)
}

/// `a` taken `t` of the way to `b`.
fn mix(a: [f32; 3], b: [f32; 3], t: f32) -> [f32; 3] {
    let mut mixed = a;
    for (a, b) in mixed.iter_mut().zip(b) {
        *a += (b - *a) * t;
    }
    mixed
}

/// The colour `a` seen through the colour `b`: a placed part's own, in the
/// colour it is drawn in.
fn times(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    let mut product = a;
    for (a, b) in product.iter_mut().zip(b) {
        *a *= b;
    }
    product
}

/// What is drawn at a sample: its colour, and how far into the picture its
/// nearest face is.
#[derive(Clone, Copy, Debug)]
struct Sample {
    color: [f32; 3],
    depth: f64,
}

/// How a line is drawn: unbroken, or in dashes this long and gaps this
/// wide, in pixels of the raster.
#[derive(Clone, Copy, Debug)]
enum Dashes {
    Solid,
    Dashed(f64, f64),
}

/// How a line or a disc is drawn: of what colour and opacity, how wide,
/// and whether what is in front of it hides it.
#[derive(Clone, Copy, Debug)]
struct Pen {
    color: [f32; 3],
    alpha: f32,
    /// Half its width, in pixels of the raster.
    half: f64,
    tested: bool,
    dashes: Dashes,
}

/// How far into the picture the triangle `t`, projected, is at the
/// barycentric coordinates `w`.
fn depth(t: [P3; 3], w: [f64; 3]) -> f64 {
    t.iter().zip(w).map(|(p, w)| p[2] * w).sum()
}

/// What is drawn, sample by sample, and how far into the picture each
/// sample's nearest face is.
struct Raster {
    width: usize,
    height: usize,
    samples: Vec<Sample>,
    frame: Frame,
    /// Where the light comes from: above and to the left of the eye, so
    /// that faces turned alike to the eye are not shaded alike.
    light: P3,
    /// How far behind the faces a line still shows (see
    /// [`DEPTH_BIAS_PX`]).
    bias: f64,
}

impl Raster {
    fn new(width: usize, height: usize, frame: Frame) -> Self {
        let mut frame = frame;
        frame.per_px /= SUPERSAMPLE as f64;
        let towards = plain::add(
            plain::scale(frame.look, -1.0),
            plain::add(plain::scale(frame.up, 0.6), plain::scale(frame.right, -0.4)),
        );
        Self {
            width,
            height,
            samples: vec![
                Sample {
                    color: rgb(BACKGROUND),
                    depth: f64::INFINITY,
                };
                width * height
            ],
            light: plain::normalize(towards).unwrap_or(frame.look),
            bias: DEPTH_BIAS_PX * SUPERSAMPLE as f64 * frame.per_px,
            frame,
        }
    }

    /// Pixels of the picture, in pixels of the raster.
    fn px(pixels: f64) -> f64 {
        pixels * SUPERSAMPLE as f64
    }

    /// Where `p` is in the raster: across, down, and into the picture.
    fn project(&self, p: P3) -> P3 {
        let c = self.frame.coordinates(p);
        [
            self.width as f64 / 2.0 + c[0] / self.frame.per_px,
            self.height as f64 / 2.0 - c[1] / self.frame.per_px,
            c[2],
        ]
    }

    /// The samples of the raster within the box `low` to `high`.
    fn samples(&self, low: P2, high: P2) -> impl Iterator<Item = (usize, usize)> + use<> {
        let clamp = |x: f64, n: usize| x.max(0.0).min(n as f64) as usize;
        let (x0, x1) = (
            clamp(low[0].floor(), self.width),
            clamp(high[0].ceil(), self.width),
        );
        let (y0, y1) = (
            clamp(low[1].floor(), self.height),
            clamp(high[1].ceil(), self.height),
        );
        (y0..y1).flat_map(move |y| (x0..x1).map(move |x| (x, y)))
    }

    /// Draws `color` at the sample `i`, `alpha` opaque, if it is no further
    /// into the picture than `depth` allows: than the face drawn there and
    /// [`Raster::bias`], or anywhere, for `None`.
    fn blend(&mut self, i: usize, color: [f32; 3], alpha: f32, depth: Option<f64>) {
        let bias = self.bias;
        if let Some(sample) = self.samples.get_mut(i)
            && depth.is_none_or(|z| z <= sample.depth + bias)
        {
            sample.color = mix(sample.color, color, alpha.clamp(0.0, 1.0));
        }
    }

    /// The triangle `t`, projected, sample by sample: its barycentric
    /// coordinates at each sample it covers.
    fn cover(&self, t: [P3; 3], mut at: impl FnMut(usize, [f64; 3])) {
        let edge =
            |a: P3, b: P3, p: P2| (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]);
        let area = edge(t[0], t[1], [t[2][0], t[2][1]]);
        if area == 0.0 || !area.is_finite() {
            return;
        }
        let [low, high] = t.iter().fold(
            [[f64::INFINITY; 2], [f64::NEG_INFINITY; 2]],
            |[low, high], p| {
                [
                    [low[0].min(p[0]), low[1].min(p[1])],
                    [high[0].max(p[0]), high[1].max(p[1])],
                ]
            },
        );
        for (x, y) in self.samples(low, high) {
            let p = [x as f64 + 0.5, y as f64 + 0.5];
            let w = [
                edge(t[1], t[2], p) / area,
                edge(t[2], t[0], p) / area,
                edge(t[0], t[1], p) / area,
            ];
            if w.iter().all(|&w| w >= 0.0) {
                at(y * self.width + x, w);
            }
        }
    }

    /// A face's triangle `t`, with the normals `n` at its corners, of the
    /// colour `color`, shaded and kept where it is nearer than what was.
    fn shaded(&mut self, t: [P3; 3], n: [P3; 3], color: [f32; 3]) {
        let t = t.map(|p| self.project(p));
        let mut drawn = Vec::new();
        self.cover(t, |i, w| drawn.push((i, w)));
        for (i, w) in drawn {
            let z = depth(t, w);
            let light = self.light;
            let look = self.frame.look;
            let Some(sample) = self.samples.get_mut(i).filter(|s| z < s.depth) else {
                continue;
            };
            sample.depth = z;
            let normal =
                (n.iter().zip(w)).fold([0.0; 3], |s, (n, w)| plain::add(s, plain::scale(*n, w)));
            let shade = match plain::normalize(normal) {
                Some(normal) => {
                    let facing = match plain::dot(normal, look) > 0.0 {
                        true => plain::scale(normal, -1.0),
                        false => normal,
                    };
                    AMBIENT + DIFFUSE * plain::dot(facing, light).max(0.0) as f32
                }
                None => AMBIENT + DIFFUSE / 2.0,
            };
            sample.color = color.map(|c| (c * shade).min(1.0));
        }
    }

    /// A filled triangle `t`, in the raster's own coordinates, over what is
    /// drawn — hidden by what is in front of it, if `tested`.
    fn filled(&mut self, t: [P3; 3], color: [f32; 3], alpha: f32, tested: bool) {
        let mut drawn = Vec::new();
        self.cover(t, |i, w| drawn.push((i, w)));
        for (i, w) in drawn {
            self.blend(i, color, alpha, tested.then(|| depth(t, w)));
        }
    }

    /// The segment from `a` to `b`, in the raster's own coordinates, drawn
    /// by `pen`, `run` along its dashes where it starts; how far along
    /// them it ends.
    fn segment(&mut self, a: P3, b: P3, pen: &Pen, run: f64) -> f64 {
        let d = [b[0] - a[0], b[1] - a[1]];
        let length_sq = d[0] * d[0] + d[1] * d[1];
        let reach = pen.half + 1.0;
        let low = [a[0].min(b[0]) - reach, a[1].min(b[1]) - reach];
        let high = [a[0].max(b[0]) + reach, a[1].max(b[1]) + reach];
        let length = length_sq.sqrt();
        if !(length.is_finite() && low[0].is_finite() && low[1].is_finite()) {
            return run;
        }
        for (x, y) in self.samples(low, high) {
            let p = [x as f64 + 0.5 - a[0], y as f64 + 0.5 - a[1]];
            let s = match length_sq > 0.0 {
                true => ((p[0] * d[0] + p[1] * d[1]) / length_sq).clamp(0.0, 1.0),
                false => 0.0,
            };
            if let Dashes::Dashed(dash, gap) = pen.dashes
                && (run + s * length) % (dash + gap) > dash
            {
                continue;
            }
            let off = [p[0] - s * d[0], p[1] - s * d[1]];
            let distance = (off[0] * off[0] + off[1] * off[1]).sqrt();
            let coverage = (pen.half + 0.5 - distance).clamp(0.0, 1.0) as f32;
            if coverage <= 0.0 {
                continue;
            }
            let i = y * self.width + x;
            let z = pen.tested.then(|| a[2] + s * (b[2] - a[2]));
            self.blend(i, pen.color, pen.alpha * coverage, z);
        }
        run + length
    }

    /// The polyline through the world points `points`, drawn by `pen`.
    fn polyline(&mut self, points: impl IntoIterator<Item = P3>, pen: &Pen) {
        let mut run = 0.0;
        let mut last: Option<P3> = None;
        for p in points {
            let p = self.project(p);
            if let Some(a) = last {
                run = self.segment(a, p, pen, run);
            }
            last = Some(p);
        }
    }

    /// A disc about `at`, in the raster's own coordinates, `pen.half`
    /// across, drawn by `pen`.
    fn disc(&mut self, at: P3, pen: &Pen) {
        self.segment(at, at, pen, 0.0);
    }

    /// A cone's outline from `base` to `tip`, in the raster's own
    /// coordinates, `radius` wide at its base, filled.
    fn arrowhead(&mut self, base: P3, tip: P3, radius: f64, color: [f32; 3]) {
        let d = [tip[0] - base[0], tip[1] - base[1]];
        let Some(d) = plain::normalize(d) else {
            return;
        };
        let side = [-d[1] * radius, d[0] * radius];
        let corner = |sign: f64| [base[0] + sign * side[0], base[1] + sign * side[1], base[2]];
        self.filled([corner(1.0), corner(-1.0), tip], color, 1.0, false);
    }

    /// The faces of `layer`, shaded, lit ones tinted towards the
    /// highlight.
    fn faces(&mut self, layer: &Layer<'_>) {
        let highlight = rgb(HIGHLIGHT);
        for face in &layer.view.faces {
            if !layer.shows_face(&face.solid, &face.name) {
                continue;
            }
            let lit = (layer.lit).is_some_and(|l| {
                l.face(&face.name, face.solid.as_deref(), face.feature.as_deref())
            });
            let own = layer.color_of(&face.name);
            let color = match lit {
                true => mix(own, highlight, LIT_TINT),
                false => own,
            };
            for (t, n) in face.mesh.triangles.iter().zip(&face.mesh.normals) {
                let t = t.map(|p| layer.place.apply(&p));
                let n = n.map(|n| layer.place.rotate(n.to_f64_array()));
                self.shaded(t, n, color);
            }
        }
    }

    /// The edges, threads and sketches of `layer`, where they are seen.
    fn lines(&mut self, layer: &Layer<'_>) {
        let pen = |color: u32| Pen {
            color: rgb(color),
            alpha: 1.0,
            half: Self::px(EDGE_HALF_WIDTH),
            tested: true,
            dashes: Dashes::Solid,
        };
        let place = &layer.place;
        for edge in &layer.view.edges {
            if !edge.within_a_surface && layer.shows_boundary(&edge.solid, &edge.faces) {
                self.polyline(edge.polyline.iter().map(|p| place.apply(p)), &pen(EDGE));
            }
        }
        for a in &layer.view.annotations {
            self.polyline(a.polyline.iter().map(|p| place.apply(p)), &pen(ANNOTATION));
        }
        for sketch in &layer.view.sketches {
            if !layer.shows_sketch(&sketch.name) {
                continue;
            }
            let lit = layer.lit.is_some_and(|l| l.sketches.contains(&sketch.name));
            let in_plane = |p| place.apply(&sketch.plane.uv_to_xyz(p));
            for curve in &sketch.curves {
                let pen = match (lit, curve.construction) {
                    (true, _) => Pen {
                        tested: false,
                        ..pen(HIGHLIGHT)
                    },
                    (false, true) => pen(CONSTRUCTION),
                    (false, false) => pen(SKETCH),
                };
                self.polyline(curve.polyline.iter().map(in_plane), &pen);
            }
            for point in &sketch.points {
                let at = self.project(in_plane(&point.at));
                let color = if lit { HIGHLIGHT } else { SKETCH };
                let pen = Pen {
                    half: Self::px(2.5),
                    tested: !lit,
                    ..pen(color)
                };
                self.disc(at, &pen);
            }
        }
    }

    /// What is lit of `layer` again over everything, so that it is seen
    /// wherever it is: its lit faces see-through — unless all of it is lit
    /// — and its lit edges, vertices, curves and sketch points whole.
    fn lit(&mut self, layer: &Layer<'_>) {
        let Some(lit) = layer.lit else {
            return;
        };
        let place = &layer.place;
        let highlight = rgb(HIGHLIGHT);
        let view = layer.view;
        if !lit.whole {
            for face in &view.faces {
                if !lit.face(&face.name, face.solid.as_deref(), face.feature.as_deref()) {
                    continue;
                }
                for t in &face.mesh.triangles {
                    let t = t.map(|p| self.project(place.apply(&p)));
                    self.filled(t, highlight, LIT_FACE_OPACITY, false);
                }
            }
        }
        let pen = Pen {
            color: highlight,
            alpha: 1.0,
            half: Self::px(LIT_HALF_WIDTH),
            tested: false,
            dashes: Dashes::Solid,
        };
        let dot = Pen {
            half: Self::px(4.5),
            ..pen
        };
        for edge in view.edges.iter().filter(|e| lit.edges.contains(&e.name)) {
            self.polyline(edge.polyline.iter().map(|p| place.apply(p)), &pen);
        }
        for vertex in view
            .vertices
            .iter()
            .filter(|v| lit.vertices.contains(&v.name))
        {
            let at = self.project(place.apply(&vertex.at));
            self.disc(at, &dot);
        }
        for sketch in &view.sketches {
            let in_plane = |p| place.apply(&sketch.plane.uv_to_xyz(p));
            let named = |id: &String| (sketch.name.clone(), id.clone());
            for curve in sketch
                .curves
                .iter()
                .filter(|c| lit.curves.contains(&named(&c.id)))
            {
                self.polyline(curve.polyline.iter().map(in_plane), &pen);
            }
            let points = sketch.points.iter();
            for point in points.filter(|p| lit.sketch_points.contains(&named(&p.id))) {
                let at = self.project(in_plane(&point.at));
                self.disc(at, &dot);
            }
        }
    }

    /// The colour of the datum `d`'s part named `name` — the datum's own,
    /// or a component's — as `layer` lights it.
    fn datum_color(layer: &Layer<'_>, d: &ViewDatum, name: &str, own: u32) -> [f32; 3] {
        let lit = layer
            .lit
            .is_some_and(|l| l.datums.contains(&d.name) || l.datums.contains(name));
        rgb(if lit { HIGHLIGHT } else { own })
    }

    /// Whether `layer` draws the datum `d`: it is not hidden, or it or a
    /// component of it is lit.
    fn shows_datum(layer: &Layer<'_>, d: &ViewDatum) -> bool {
        let lit = layer.lit.is_some_and(|l| {
            (l.datums.iter()).any(|name| {
                name == &d.name
                    || (name.strip_prefix(d.name.as_str()))
                        .is_some_and(|rest| rest.starts_with(':'))
            })
        });
        lit || !layer.hides(&d.name)
    }

    /// The datum planes and axes of `part`, drawn as large as the drawing
    /// around the point of them nearest its centre, part of the scene.
    fn datums(&mut self, part: &PartView, layer: &Layer<'_>) {
        let size = part.extent.size.to_f64();
        let center = part.extent.center.to_f64_array();
        for d in part.datums.iter().filter(|d| Self::shows_datum(layer, d)) {
            let f = &d.frame;
            let origin = f.origin().to_f64_array();
            let [u, v, w] = [f.u(), f.v(), f.w()].map(|a| a.to_f64_array());
            let offset = plain::sub(center, origin);
            let color = Self::datum_color(layer, d, &d.name, DATUM);
            let pen = Pen {
                color,
                alpha: 0.8,
                half: Self::px(EDGE_HALF_WIDTH),
                tested: true,
                dashes: Dashes::Solid,
            };
            match d.kind {
                DatumKind::Plane => {
                    let at = plain::sub(center, plain::scale(w, plain::dot(offset, w)));
                    let corner = |a: f64, b: f64| {
                        let along = plain::add(plain::scale(u, a), plain::scale(v, b));
                        plain::add(at, plain::scale(along, size / 2.0))
                    };
                    let corners = [
                        corner(-1.0, -1.0),
                        corner(1.0, -1.0),
                        corner(1.0, 1.0),
                        corner(-1.0, 1.0),
                    ];
                    let [a, b, c, e] = corners.map(|p| self.project(p));
                    self.filled([a, b, c], color, 0.12, true);
                    self.filled([a, c, e], color, 0.12, true);
                    self.polyline(corners.into_iter().chain([corners[0]]), &pen);
                }
                DatumKind::Axis => {
                    let at = plain::add(origin, plain::scale(w, plain::dot(offset, w)));
                    let ends = [-1.0, 1.0].map(|s| plain::add(at, plain::scale(w, s * size)));
                    let dash = size / 30.0 / self.frame.per_px;
                    let pen = Pen {
                        alpha: 0.9,
                        dashes: Dashes::Dashed(dash, dash / 2.0),
                        ..pen
                    };
                    self.polyline(ends, &pen);
                }
                DatumKind::Point | DatumKind::Frame => {}
            }
        }
    }

    /// The point and frame datums of `part`, drawn over everything at a
    /// size of their own on screen, in reaches, as a pick lays them out:
    /// a frame's ball, its axes and the squares of its planes; a point's
    /// ball and its three axes.
    fn frames(&mut self, part: &PartView, layer: &Layer<'_>) {
        let reach = Self::px(REACH_PX);
        for d in part.datums.iter().filter(|d| Self::shows_datum(layer, d)) {
            let f = &d.frame;
            let o = self.project(f.origin().to_f64_array());
            let axes = FrameAxis::ALL.map(|a| self.frame.on_screen(a.of(f).to_f64_array()));
            let at = |size: f64, x: [f64; 3]| {
                let along = (axes.iter().zip(x)).fold([0.0; 2], |s, (axis, x)| {
                    plain::add(s, plain::scale(*axis, x))
                });
                [o[0] + along[0] * size, o[1] + along[1] * size, o[2]]
            };
            let pen = |color: [f32; 3], half: f64| Pen {
                color,
                alpha: 1.0,
                half: half.max(Self::px(0.6)),
                tested: false,
                dashes: Dashes::Solid,
            };
            let unit = |k: usize, length: f64| plain::scale(unit(k), length);
            match d.kind {
                DatumKind::Frame => {
                    let size = FRAME * reach;
                    let component = |c: DatumComponent| {
                        (d.components.iter())
                            .find(|(of, _)| *of == c)
                            .map(|(_, name)| name.as_str())
                    };
                    let axes = FrameAxis::ALL.into_iter().zip(AXES).enumerate();
                    for (k, (axis, own)) in axes.clone() {
                        let name = component(DatumComponent::Plane(axis)).unwrap_or_default();
                        let color = Self::datum_color(layer, d, name, own);
                        let mut others = (0..3).filter(|&j| j != k).map(|j| unit(j, 1.0));
                        let (Some(a), Some(b)) = (others.next(), others.next()) else {
                            continue;
                        };
                        let corner = |s: f64, t: f64| {
                            at(size, plain::add(plain::scale(a, s), plain::scale(b, t)))
                        };
                        let square = [
                            corner(0.3, 0.3),
                            corner(0.6, 0.3),
                            corner(0.6, 0.6),
                            corner(0.3, 0.6),
                        ];
                        let [p, q, r, s] = square;
                        self.filled([p, q, r], color, 0.3, false);
                        self.filled([p, r, s], color, 0.3, false);
                        for (a, b) in [(p, q), (q, r), (r, s), (s, p)] {
                            self.segment(a, b, &pen(color, Self::px(0.5)), 0.0);
                        }
                    }
                    for (k, (axis, own)) in axes {
                        let name = component(DatumComponent::Axis(axis)).unwrap_or_default();
                        let color = Self::datum_color(layer, d, name, own);
                        let (base, tip) = (at(size, unit(k, 0.85)), at(size, unit(k, 1.0)));
                        self.segment(at(size, unit(k, 0.1)), base, &pen(color, 0.015 * size), 0.0);
                        self.arrowhead(base, tip, 0.05 * size, color);
                    }
                    let ball = Self::datum_color(layer, d, &d.name, 0xffffff);
                    self.disc(o, &pen(ball, 0.07 * size));
                }
                DatumKind::Point => {
                    let size = POINT_TRIAD * reach;
                    for (k, tint) in AXIS_TINTS.into_iter().enumerate() {
                        let color = Self::datum_color(layer, d, &d.name, tint);
                        self.segment(o, at(size, unit(k, 1.0)), &pen(color, Self::px(0.6)), 0.0);
                    }
                    let ball = Self::datum_color(layer, d, &d.name, DATUM);
                    self.disc(o, &pen(ball, 0.12 * size));
                }
                DatumKind::Plane | DatumKind::Axis => {}
            }
        }
    }

    /// What a step being edited draws, over everything, in its style's
    /// colour. A label is drawn as a dot where its text would stand: a
    /// picture has no type.
    fn visual(&mut self, visual: &Visual) {
        let color = rgb(match visual.style {
            Style::Free | Style::Region => 0x6cb4ff,
            Style::Fixed => 0xe6e6e6,
            Style::Selected => 0xffd84a,
            Style::Hover => 0xffe0a0,
            Style::Failed => 0xff5d5d,
            Style::Construction => 0x8a8a8a,
            Style::Reference => 0xb48cf0,
            Style::Draft | Style::Handle => 0xffa040,
            Style::Guide => 0x777777,
            Style::Snap => 0x58e07a,
            Style::Removed => 0xff4f9a,
            Style::Paper => 0xbbbbbb,
        });
        let reach = Self::px(REACH_PX);
        let pen = Pen {
            color,
            alpha: 1.0,
            half: Self::px(EDGE_HALF_WIDTH),
            tested: false,
            dashes: Dashes::Solid,
        };
        let disc = |radius: f64| Pen {
            half: Self::px(radius),
            ..pen
        };
        match &visual.shape {
            Shape::Point { at } => {
                let at = self.project(at.to_f64_array());
                match visual.style {
                    Style::Snap => {
                        let ring = Self::px(6.0);
                        let around = (0..=24).map(|i| {
                            let angle = f64::from(i) * std::f64::consts::TAU / 24.0;
                            [
                                at[0] + ring * angle.cos(),
                                at[1] + ring * angle.sin(),
                                at[2],
                            ]
                        });
                        let mut last = None;
                        for p in around {
                            if let Some(a) = last {
                                self.segment(a, p, &pen, 0.0);
                            }
                            last = Some(p);
                        }
                    }
                    Style::Selected | Style::Hover => self.disc(at, &disc(5.0)),
                    _ => self.disc(at, &disc(3.5)),
                }
            }
            Shape::Polyline { points } => {
                let dashes = match visual.style {
                    Style::Reference => Dashes::Dashed(Self::px(1.5), Self::px(3.0)),
                    Style::Construction | Style::Draft | Style::Guide => {
                        Dashes::Dashed(Self::px(6.0), Self::px(4.0))
                    }
                    _ => Dashes::Solid,
                };
                let pen = Pen { dashes, ..pen };
                self.polyline(points.iter().map(|p| p.to_f64_array()), &pen);
            }
            Shape::Triangles { triangles } => {
                for t in triangles {
                    let t = t.map(|p| self.project(p.to_f64_array()));
                    self.filled(t, color, 0.15, false);
                }
            }
            Shape::Label { at, offset, .. } => {
                let shift = self.frame.on_screen(offset.to_f64_array());
                let at = self.project(at.to_f64_array());
                let at = [at[0] + shift[0] * reach, at[1] + shift[1] * reach, at[2]];
                self.disc(at, &disc(2.0));
            }
            Shape::Handle { at, direction } => {
                let unit = 0.8 * reach;
                let at = self.project(at.to_f64_array());
                self.disc(at, &Pen { half: unit, ..pen });
                if let Some(d) = plain::normalize(self.frame.on_screen(direction.to_f64_array())) {
                    for sign in [1.0, -1.0] {
                        let along = |x: f64| {
                            [
                                at[0] + sign * d[0] * x * unit,
                                at[1] + sign * d[1] * x * unit,
                                at[2],
                            ]
                        };
                        self.arrowhead(along(1.4), along(3.0), unit, color);
                    }
                }
            }
            Shape::Instance { .. } => {}
            Shape::Triad { at, axes } => {
                let size = TRIAD * reach;
                let at = self.project(at.to_f64_array());
                for (axis, color) in axes.iter().zip(AXES.map(rgb)) {
                    let Some(d) = plain::normalize(axis.to_f64_array()) else {
                        continue;
                    };
                    let d = self.frame.on_screen(d);
                    let along = |x: f64| [at[0] + d[0] * x * size, at[1] + d[1] * x * size, at[2]];
                    let pen = Pen {
                        color,
                        half: (0.02 * size).max(Self::px(0.6)),
                        ..pen
                    };
                    self.segment(along(0.0), along(0.8), &pen, 0.0);
                    self.arrowhead(along(0.8), along(1.0), 0.06 * size, color);
                }
            }
        }
    }

    /// The world's axes in the bottom left corner, `x` red, `y` green, `z`
    /// blue, each lettered at its tip: those further into the picture
    /// first, so that the nearer cross them.
    fn compass(&mut self) {
        let origin = [Self::px(34.0), self.height as f64 - Self::px(34.0), 0.0];
        let length = Self::px(24.0);
        let mut order: Vec<(usize, u32)> = AXES.into_iter().enumerate().collect();
        let into = |k: usize| plain::dot(unit(k), self.frame.look);
        order.sort_by(|a, b| into(b.0).total_cmp(&into(a.0)));
        for (k, color) in order {
            let d = self.frame.on_screen(unit(k));
            let tip = [origin[0] + d[0] * length, origin[1] + d[1] * length, 0.0];
            let pen = Pen {
                color: rgb(color),
                alpha: 1.0,
                half: Self::px(1.0),
                tested: false,
                dashes: Dashes::Solid,
            };
            self.segment(origin, tip, &pen, 0.0);
            let towards = plain::normalize(d).unwrap_or([0.0, 0.0]);
            let letter = [
                tip[0] + towards[0] * Self::px(9.0),
                tip[1] + towards[1] * Self::px(9.0),
            ];
            self.letter(k, letter, rgb(color));
        }
    }

    /// The letter of the axis `k`, `x`, `y` or `z`, centred on `at`.
    fn letter(&mut self, k: usize, at: P2, color: [f32; 3]) {
        const GLYPHS: [[&str; 5]; 3] = [
            ["#.#", "#.#", ".#.", "#.#", "#.#"],
            ["#.#", "#.#", ".#.", ".#.", ".#."],
            ["###", "..#", ".#.", "#..", "###"],
        ];
        let cell = Self::px(2.0);
        for (row, line) in GLYPHS.get(k).into_iter().flatten().enumerate() {
            for (column, c) in line.chars().enumerate() {
                if c != '#' {
                    continue;
                }
                let x = at[0] + (column as f64 - 1.5) * cell;
                let y = at[1] + (row as f64 - 2.5) * cell;
                for (sx, sy) in self.samples([x, y], [x + cell, y + cell]) {
                    self.blend(sy * self.width + sx, color, 1.0, None);
                }
            }
        }
    }

    /// The raster, each pixel of `width` by `height` the mean of its
    /// samples, as a PNG.
    fn png(&self, width: usize, height: usize) -> GeopResult<Vec<u8>> {
        let mut sums = vec![[0.0f32; 3]; width * height];
        for (i, sample) in self.samples.iter().enumerate() {
            let (x, y) = (i % self.width / SUPERSAMPLE, i / self.width / SUPERSAMPLE);
            if let Some(sum) = sums.get_mut(y * width + x) {
                for (s, c) in sum.iter_mut().zip(sample.color) {
                    *s += c;
                }
            }
        }
        let samples = (SUPERSAMPLE * SUPERSAMPLE) as f32;
        let data: Vec<u8> = (sums.iter().flatten())
            .map(|s| (s / samples * 255.0).round().clamp(0.0, 255.0) as u8)
            .collect();
        let failed =
            |e: png::EncodingError| GeopError::new(format!("writing a picture as a PNG: {e}"));
        let mut out = Vec::new();
        let mut encoder = png::Encoder::new(&mut out, width as u32, height as u32);
        encoder.set_color(png::ColorType::Rgb);
        encoder.set_depth(png::BitDepth::Eight);
        let mut writer = encoder.write_header().map_err(failed)?;
        writer.write_image_data(&data).map_err(failed)?;
        writer.finish().map_err(failed)?;
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use std::{collections::BTreeMap, rc::Rc};

    use geop_core_math::primitives::{CoordinateSystem, Pose};

    use super::*;
    use crate::view::{FaceMesh, ViewEdge, ViewFace, ViewInstance, drawn};

    fn v3(p: P3) -> Vector3 {
        Vector3::from_array(p.map(Scalar::from_f64))
    }

    /// The unit cube from `low`: a solid `block` of six faces, `x-` to
    /// `z+`, and twelve edges, with the origin's frame beside it.
    fn block(low: P3) -> PartView {
        let mut view = PartView::new();
        view.model = "model".into();
        view.solids = vec!["block".into()];
        let corner = |c: [usize; 3]| v3(std::array::from_fn(|k| low[k] + c[k] as f64));
        for k in 0..3 {
            let (a, b) = ((k + 1) % 3, (k + 2) % 3);
            for side in 0..2 {
                let at = |i: usize, j: usize| {
                    let mut c = [0; 3];
                    (c[k], c[a], c[b]) = (side, i, j);
                    corner(c)
                };
                let quad = [at(0, 0), at(1, 0), at(1, 1), at(0, 1)];
                let mut normal = [0.0; 3];
                normal[k] = if side == 0 { -1.0 } else { 1.0 };
                view.faces.push(ViewFace {
                    name: format!("{}{}", ["x", "y", "z"][k], ["-", "+"][side]),
                    solid: Some("block".into()),
                    feature: None,
                    mesh: FaceMesh::new(
                        vec![[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]],
                        vec![[v3(normal); 3]; 2],
                    ),
                });
                for i in 0..2 {
                    let mut c = [0; 3];
                    (c[k], c[a]) = (side, i);
                    let mut d = c;
                    d[b] = 1;
                    view.edges.push(ViewEdge {
                        name: format!("e{}", view.edges.len()),
                        solid: Some("block".into()),
                        faces: vec![],
                        polyline: vec![corner(c), corner(d)],
                        within_a_surface: false,
                        ends: [String::new(), String::new()],
                    });
                }
            }
        }
        view.datum_target = "datums".into();
        view.datums.push(ViewDatum {
            name: "origin".into(),
            kind: DatumKind::Frame,
            frame: CoordinateSystem::world_at(Vector3::zero()),
            components: FrameAxis::ALL
                .into_iter()
                .flat_map(|a| [DatumComponent::Axis(a), DatumComponent::Plane(a)])
                .map(|c| (c, format!("origin:{c}")))
                .collect(),
        });
        view
    }

    fn scene(part: PartView) -> SceneState {
        SceneState {
            part: Rc::new(part.finish()),
            hidden: Some(vec![]),
            instances: vec![],
            all: true,
            removed: vec![],
            components: BTreeMap::new(),
            forgotten: vec![],
            meshes: BTreeMap::new(),
            forgotten_meshes: vec![],
            structure: None,
        }
    }

    fn isometric(scene: &SceneState) -> Camera {
        let (look, up) = (v3([-1.0, -1.0, -1.0]), v3([0.0, 0.0, 1.0]));
        Camera::fitting(scene, look, up, 400, 300).unwrap()
    }

    /// The picture's pixels, `[r, g, b]` by row, and its size.
    fn decoded(png: &[u8]) -> (Vec<[u8; 3]>, u32, u32) {
        let decoder = png::Decoder::new(png);
        let mut reader = decoder.read_info().unwrap();
        let mut data = vec![0; reader.output_buffer_size()];
        let info = reader.next_frame(&mut data).unwrap();
        assert_eq!(info.color_type, png::ColorType::Rgb);
        let pixels = (data[..info.buffer_size()].chunks(3))
            .map(|c| [c[0], c[1], c[2]])
            .collect();
        (pixels, info.width, info.height)
    }

    fn at(pixels: &(Vec<[u8; 3]>, u32, u32), p: [f64; 2]) -> [u8; 3] {
        pixels.0[p[1] as usize * pixels.1 as usize + p[0] as usize]
    }

    fn grey(c: [u8; 3]) -> bool {
        let (low, high) = (c.iter().min().unwrap(), c.iter().max().unwrap());
        *low > 60 && high - low < 16
    }

    #[test]
    fn a_block_is_drawn_in_the_middle_of_its_picture() {
        let scene = scene(block([2.0, 2.0, 2.0]));
        let camera = isometric(&scene);
        let png = picture(&scene, None, &[], &camera).unwrap();
        let pixels = decoded(&png);
        assert_eq!((pixels.1, pixels.2), (400, 300));
        let middle = camera.pixel(&v3([2.5, 2.5, 2.5]));
        assert!((middle[0] - 200.0).abs() < 1.0 && (middle[1] - 150.0).abs() < 1.0);
        assert!(grey(at(&pixels, middle)), "{:?}", at(&pixels, middle));
        assert_eq!(at(&pixels, [399.0, 0.0]), [0x1b, 0x1f, 0x26]);
        // The three faces seen are shaded apart, so the block reads as one.
        let faces = [[2.5, 2.5, 3.0], [3.0, 2.5, 2.5], [2.5, 3.0, 2.5]];
        let shades = faces.map(|f| at(&pixels, camera.pixel(&v3(f)))[0]);
        assert!(shades[0] != shades[1] && shades[1] != shades[2] && shades[0] != shades[2]);
        // And drawn alike every time.
        assert_eq!(png, picture(&scene, None, &[], &camera).unwrap());
    }

    #[test]
    fn a_pixel_points_where_it_is_drawn() {
        let scene = scene(block([2.0, 2.0, 2.0]));
        let camera = isometric(&scene);
        let pointer = camera.pointer(200.0, 150.0).unwrap();
        let accepts = [drawn::FACE.to_string()];
        let hit = scene.part.pick(&pointer, &accepts, &[]).unwrap();
        // Looking along (-1, -1, -1), the faces seen are x+, y+ and z+.
        assert!(hit.element.name.ends_with('+'), "{:?}", hit.element);
        let p = v3([2.3, 2.9, 2.1]);
        let [x, y] = camera.pixel(&p);
        let through = camera.through(p).unwrap();
        let back = camera.pixel(through.ray.origin());
        assert!((back[0] - x).abs() < 1e-9 && (back[1] - y).abs() < 1e-9);
        let (distance, t) = camera.pointer(x, y).unwrap().ray.distance_to_point(&p);
        assert!(distance.to_f64() < 1e-9 && t.to_f64() > 0.0);
        let Reach::Tube { radius } = pointer.reach else {
            panic!("an orthographic pointer reaches as a tube");
        };
        assert!((radius.to_f64() - REACH_PX * camera.span / 300.0).abs() < 1e-12);
    }

    #[test]
    fn a_placed_part_is_framed_and_drawn_in_its_colour_and_what_is_lit_lit() {
        let component = Rc::new(block([0.0, 0.0, 0.0]).finish());
        let at_x = v3([10.0, 0.0, 0.0]);
        let mut part = block([0.0, 0.0, 0.0]);
        part.instances.push(ViewInstance {
            name: "bolt".into(),
            component: "block".into(),
            frame: CoordinateSystem::world_at(at_x),
            pose: Pose::identity().with_position(at_x),
            view: component,
            state: None,
            fixed: true,
            color: None,
        });
        let mut scene = scene(part);
        scene.instances = vec![ViewInstance {
            color: Some("#ff0000".into()),
            ..scene.part.instances[0].clone()
        }];
        let (look, up) = (v3([0.0, 0.0, -1.0]), v3([0.0, 1.0, 0.0]));
        let camera = Camera::fitting(&scene, look, up, 400, 300).unwrap();
        let lit = Lighting::from([(
            String::new(),
            Lit {
                faces: BTreeSet::from(["z+".to_string()]),
                ..Lit::default()
            },
        )]);
        let pixels = decoded(&picture(&scene, Some(&lit), &[], &camera).unwrap());
        let [r, g, b] = at(&pixels, camera.pixel(&v3([10.5, 0.5, 1.0])));
        assert!(r > 60 && g < 20 && b < 20, "{:?}", [r, g, b]);
        // Clear of the origin frame, drawn over the corner of the block.
        let [r, g, b] = at(&pixels, camera.pixel(&v3([0.8, 0.25, 1.0])));
        assert!(r > b + 60 && g > b + 30, "{:?}", [r, g, b]);
        // Both blocks are in the picture, with room round them.
        for x in [0.0, 11.0] {
            let [px, _] = camera.pixel(&v3([x, 0.5, 1.0]));
            assert!(px > 10.0 && px < 390.0, "{px}");
        }
    }

    /// A face in a colour of its own is drawn in it, over the part's; a
    /// colour its instance is drawn in is over them all, as over the part's:
    /// the same view placed twice, once in a colour of the user's.
    #[test]
    fn a_face_is_drawn_in_its_colour_and_a_placed_parts_colour_is_over_it() {
        let mut own = block([0.0, 0.0, 0.0]);
        own.color = Some("#0000ff".into());
        own.face_colors = BTreeMap::from([("z+".to_string(), "#00ff00".to_string())]);
        let component = Rc::new(own.finish());
        let placed = |name: &str, x: f64, color: Option<&str>| {
            let at = v3([x, 0.0, 0.0]);
            ViewInstance {
                name: name.into(),
                component: "block".into(),
                frame: CoordinateSystem::world_at(at),
                pose: Pose::identity().with_position(at),
                view: component.clone(),
                state: None,
                fixed: true,
                color: color.map(str::to_string),
            }
        };
        let mut part = block([0.0, 0.0, 0.0]);
        part.face_colors = BTreeMap::from([("z+".to_string(), "#00ff00".to_string())]);
        part.instances = vec![
            placed("own", 3.0, None),
            placed("over", 6.0, Some("#ff0000")),
        ];
        let mut scene = scene(part);
        scene.instances = scene.part.instances.clone();
        let (look, up) = (v3([0.0, 0.0, -1.0]), v3([0.0, 1.0, 0.0]));
        let camera = Camera::fitting(&scene, look, up, 400, 300).unwrap();
        let pixels = decoded(&picture(&scene, None, &[], &camera).unwrap());
        let seen = |x: f64| at(&pixels, camera.pixel(&v3([x + 0.8, 0.25, 1.0])));
        for x in [0.0, 3.0] {
            let [r, g, b] = seen(x);
            assert!(g > r + 60 && g > b + 60, "{:?}", [r, g, b]);
        }
        // The part's blue times the placed part's red, as without a colour
        // of the face: no green of the face's left.
        let [_, g, _] = seen(6.0);
        assert!(g < 20, "{g}");
    }

    #[test]
    fn a_look_along_up_takes_another_up() {
        let scene = scene(block([0.0, 0.0, 0.0]));
        let (look, up) = (v3([0.0, 0.0, -1.0]), v3([0.0, 0.0, 1.0]));
        let camera = Camera::fitting(&scene, look, up, 80, 60).unwrap();
        assert_eq!(camera.up.to_f64_array(), [1.0, 0.0, 0.0]);
        assert!(Camera::fitting(&scene, Vector3::zero(), up, 80, 60).is_err());
    }
}
