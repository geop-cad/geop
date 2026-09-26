use std::collections::HashMap;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::{
    geop_error::{DebugContext, GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};

use super::{Color10, Line, TriangleFace};

/// Trait for objects that can be uniformly sampled into 3D points for rendering.
pub trait RasterizableCurve<S: Scalar> {
    /// Evaluate the curve at parameter `t` and return a 3D point.
    fn eval_at(&self, t: S) -> GeopResult<Vector3<S>>;
}

/// Trait for objects that can be uniformly sampled into 3D points for rendering.
pub trait RasterizableSurface<S: Scalar> {
    /// Evaluate the surface at `(u, v)` and return a 3D point.
    fn eval_at(&self, u: S, v: S) -> GeopResult<Vector3<S>>;
}

// ── unique id counter ─────────────────────────────────────────────────────────

static SCENE_ID: AtomicU64 = AtomicU64::new(0);
fn next_id() -> u64 {
    SCENE_ID.fetch_add(1, Ordering::Relaxed)
}

fn sample_surface_grid<S: Scalar>(
    surface: &dyn RasterizableSurface<S>,
    u_min: S,
    u_max: S,
    v_min: S,
    v_max: S,
    n: usize,
) -> GeopResult<Vec<Vector3<S>>> {
    let mut grid = Vec::with_capacity(n * n);
    for j in 0..n {
        for i in 0..n {
            let u = if i == 0 {
                u_min
            } else if i == n - 1 {
                u_max
            } else {
                let f = S::from_ratio(i as i64, (n - 1) as i64)?;
                u_min.add(u_max.sub(u_min).mul(f))
            };
            let v = if j == 0 {
                v_min
            } else if j == n - 1 {
                v_max
            } else {
                let f = S::from_ratio(j as i64, (n - 1) as i64)?;
                v_min.add(v_max.sub(v_min).mul(f))
            };
            grid.push(surface.eval_at(u, v)?);
        }
    }
    Ok(grid)
}

// ── PrimitiveScene ────────────────────────────────────────────────────────────

pub struct PrimitiveScene<S: Scalar> {
    pub points: Vec<(Vector3<S>, Color10)>,
    pub lines: Vec<(Line<S>, Color10)>,
    /// Like `lines`, but rendered with depth-testing disabled (and after
    /// everything else), so they stay visible on top of solid triangles
    /// instead of being occluded — useful for highlighting intersection
    /// curves on a solid (non-wireframe) render.
    pub highlight_lines: Vec<(Line<S>, Color10)>,
    pub triangles: Vec<(TriangleFace<S>, Color10)>,
    /// Triangles with an independent color per vertex (`a`, `b`, `c`),
    /// interpolated across the face — useful for telling the three corners
    /// of a debug triangle apart at a glance.
    pub triangles_rgb: Vec<(TriangleFace<S>, Color10, Color10, Color10)>,
    /// Triangles rendered with alpha blending (`color`, `opacity` in
    /// `[0, 1]`) — e.g. so a face's fill doesn't hide the trim curves,
    /// vertices or labels drawn on top of/behind it.
    pub triangles_transparent: Vec<(TriangleFace<S>, Color10, f64)>,
    /// Text labels anchored at a 3-D world position (e.g. an entity's id),
    /// rendered as always-facing-camera HTML overlays via `CSS2DRenderer`.
    pub labels: Vec<(Vector3<S>, String, Color10)>,
    /// Thick tubes (`start`, `end`, `radius`, `color`) — e.g. coordinate
    /// system axes, rendered as solid geometry instead of a `Line`, since a
    /// `THREE.Line`'s width can't be controlled portably across GPUs.
    pub cylinders: Vec<(Vector3<S>, Vector3<S>, f64, Color10)>,
    pub debug_text: String,
    rendered_path: OnceLock<String>,
}

impl<S: Scalar> core::fmt::Debug for PrimitiveScene<S> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "PrimitiveScene({} pts, {} lines, {} tris)",
            self.points.len(),
            self.lines.len(),
            self.triangles.len()
        )
    }
}

impl<S: Scalar> PrimitiveScene<S> {
    pub fn new() -> Self {
        Self {
            points: Vec::new(),
            lines: Vec::new(),
            highlight_lines: Vec::new(),
            triangles: Vec::new(),
            triangles_rgb: Vec::new(),
            triangles_transparent: Vec::new(),
            labels: Vec::new(),
            cylinders: Vec::new(),
            debug_text: String::new(),
            rendered_path: OnceLock::new(),
        }
    }

    pub fn add_point(&mut self, p: Vector3<S>, c: Color10) {
        self.points.push((p, c));
    }

    pub fn add_line(&mut self, l: Line<S>, c: Color10) {
        self.lines.push((l, c));
    }

    /// Draw an already-sampled polyline as `n - 1` line segments, skipping
    /// any consecutive pair of (near-)coincident points.
    pub fn add_polyline(&mut self, points: &[Vector3<S>], color: Color10) {
        for w in points.windows(2) {
            if let Ok(seg) = Line::try_new(w[0], w[1]) {
                self.add_line(seg, color);
            }
        }
    }

    /// Like `add_line`, but drawn with depth-testing disabled — see
    /// `highlight_lines`'s doc comment.
    pub fn add_highlight_line(&mut self, l: Line<S>, c: Color10) {
        self.highlight_lines.push((l, c));
    }

    /// Like `add_polyline`, but drawn with depth-testing disabled — see
    /// `highlight_lines`'s doc comment.
    pub fn add_highlight_polyline(&mut self, points: &[Vector3<S>], color: Color10) {
        for w in points.windows(2) {
            if let Ok(seg) = Line::try_new(w[0], w[1]) {
                self.add_highlight_line(seg, color);
            }
        }
    }

    pub fn add_triangle(&mut self, t: TriangleFace<S>, c: Color10) {
        self.triangles.push((t, c));
    }

    /// Add a triangle with an independent color per vertex (`t.a` → `ca`,
    /// `t.b` → `cb`, `t.c` → `cc`), interpolated across the face.
    pub fn add_triangle_rgb(&mut self, t: TriangleFace<S>, ca: Color10, cb: Color10, cc: Color10) {
        self.triangles_rgb.push((t, ca, cb, cc));
    }

    /// Add a triangle rendered with alpha blending — `opacity` in `[0, 1]`
    /// (`0` invisible, `1` opaque).
    pub fn add_triangle_transparent(&mut self, t: TriangleFace<S>, c: Color10, opacity: f64) {
        self.triangles_transparent.push((t, c, opacity));
    }

    /// Add a text label anchored at `pos` in world space — rendered as an
    /// always-facing-camera HTML overlay, colored `c`.
    pub fn add_label(&mut self, pos: Vector3<S>, text: impl Into<String>, c: Color10) {
        self.labels.push((pos, text.into(), c));
    }

    /// Add a solid cylindrical tube from `start` to `end`, `radius` wide —
    /// e.g. for drawing thick axes/arrows that stay visibly wide regardless
    /// of GPU line-width support.
    pub fn add_cylinder(&mut self, start: Vector3<S>, end: Vector3<S>, radius: f64, c: Color10) {
        self.cylinders.push((start, end, radius, c));
    }

    pub fn add_scene(&mut self, other: PrimitiveScene<S>) {
        self.points.extend(other.points);
        self.lines.extend(other.lines);
        self.highlight_lines.extend(other.highlight_lines);
        self.triangles.extend(other.triangles);
        self.triangles_rgb.extend(other.triangles_rgb);
        self.triangles_transparent
            .extend(other.triangles_transparent);
        self.labels.extend(other.labels);
        self.cylinders.extend(other.cylinders);
        if !other.debug_text.is_empty() {
            if !self.debug_text.is_empty() {
                self.debug_text.push('\n');
            }
            self.debug_text.push_str(&other.debug_text);
        }
    }

    pub fn set_debug_text(&mut self, text: String) {
        self.debug_text = text;
    }
    pub fn add_debug_text(&mut self, text: String) {
        if !self.debug_text.is_empty() {
            self.debug_text.push('\n');
        }
        self.debug_text.push_str(&text);
    }

    /// Rasterize a curve: n uniform samples → n-1 line segments.
    pub fn add_curve(
        &mut self,
        curve: &dyn RasterizableCurve<S>,
        t_min: S,
        t_max: S,
        color: Color10,
        n: usize,
    ) -> GeopResult<()> {
        if n < 2 {
            return Err(GeopError::new("add_curve: n must be >= 2"));
        }
        let mut pts = Vec::with_capacity(n);
        for i in 0..n {
            let t = if i == 0 {
                t_min
            } else if i == n - 1 {
                t_max
            } else {
                let frac = S::from_ratio(i as i64, (n - 1) as i64)?;
                t_min.add(t_max.sub(t_min).mul(frac))
            };
            pts.push(curve.eval_at(t)?);
        }
        for i in 0..n - 1 {
            if let Ok(seg) = Line::try_new(pts[i].clone(), pts[i + 1].clone()) {
                self.add_line(seg, color);
            }
        }
        Ok(())
    }

    /// Rasterize a surface: n×n quad grid → 2n² triangles.
    #[allow(clippy::too_many_arguments)]
    pub fn add_surface(
        &mut self,
        surface: &dyn RasterizableSurface<S>,
        color: Color10,
        u_min: S,
        u_max: S,
        v_min: S,
        v_max: S,
        n: usize,
    ) -> GeopResult<()> {
        if n < 2 {
            return Err(GeopError::new("add_surface: n must be >= 2"));
        }
        let grid = sample_surface_grid(surface, u_min, u_max, v_min, v_max, n)?;
        for j in 0..n - 1 {
            for i in 0..n - 1 {
                let p00 = grid[j * n + i].clone();
                let p10 = grid[j * n + i + 1].clone();
                let p01 = grid[(j + 1) * n + i].clone();
                let p11 = grid[(j + 1) * n + i + 1].clone();
                if let Ok(t) = TriangleFace::try_new(p00, p10.clone(), p01.clone()) {
                    self.add_triangle(t, color);
                }
                if let Ok(t) = TriangleFace::try_new(p10, p11, p01) {
                    self.add_triangle(t, color);
                }
            }
        }
        Ok(())
    }

    /// Wireframe: n×n grid of quads emitted as line segments.
    #[allow(clippy::too_many_arguments)]
    pub fn add_surface_wireframe(
        &mut self,
        surface: &dyn RasterizableSurface<S>,
        color: Color10,
        u_min: S,
        u_max: S,
        v_min: S,
        v_max: S,
        n: usize,
    ) -> GeopResult<()> {
        if n < 2 {
            return Err(GeopError::new("add_surface_wireframe: n must be >= 2"));
        }
        let grid = sample_surface_grid(surface, u_min, u_max, v_min, v_max, n)?;
        for j in 0..n {
            for i in 0..n - 1 {
                if let Ok(l) = Line::try_new(grid[j * n + i].clone(), grid[j * n + i + 1].clone()) {
                    self.add_line(l, color);
                }
            }
        }
        for i in 0..n {
            for j in 0..n - 1 {
                if let Ok(l) = Line::try_new(grid[j * n + i].clone(), grid[(j + 1) * n + i].clone())
                {
                    self.add_line(l, color);
                }
            }
        }
        Ok(())
    }

    /// Check that `self.triangles` forms a watertight (closed, 2-manifold)
    /// mesh: every triangle edge, quantized to a grid of size `eps` and
    /// identified regardless of direction, must be shared by exactly 2
    /// triangles.
    pub fn is_watertight(&self, eps: f64) -> Result<(), String> {
        let quantize = |p: &Vector3<S>| -> (i64, i64, i64) {
            (
                (p[0].to_f64() / eps).round() as i64,
                (p[1].to_f64() / eps).round() as i64,
                (p[2].to_f64() / eps).round() as i64,
            )
        };

        let mut edge_uses: HashMap<((i64, i64, i64), (i64, i64, i64)), Vec<usize>> = HashMap::new();
        for (idx, (t, _)) in self.triangles.iter().enumerate() {
            let qa = quantize(&t.a);
            let qb = quantize(&t.b);
            let qc = quantize(&t.c);
            for (p, q) in [(qa, qb), (qb, qc), (qc, qa)] {
                let key = if p <= q { (p, q) } else { (q, p) };
                edge_uses.entry(key).or_default().push(idx);
            }
        }

        let bad: Vec<_> = edge_uses
            .iter()
            .filter(|(_, tris)| tris.len() != 2)
            .collect();
        if bad.is_empty() {
            return Ok(());
        }
        let mut msg = format!(
            "mesh is not watertight: {} edge(s) not shared by exactly 2 triangles:",
            bad.len()
        );
        for (edge, tris) in bad.iter().take(10) {
            msg.push_str(&format!(
                "\n  edge {edge:?} used by {} triangle(s): {tris:?}",
                tris.len()
            ));
        }
        Err(msg)
    }

    /// Write a self-contained HTML file with an embedded three.js scene.
    pub fn save_to_file(&self, filename: &str) -> GeopResult<()> {
        let html = self.render_html();
        std::fs::write(filename, html)
            .map_err(|e| GeopError::new(format!("PrimitiveScene::save_to_file: {e}")))?;
        Ok(())
    }

    fn render_html(&self) -> String {
        let points_js = self
            .points
            .iter()
            .map(|(p, c)| {
                format!(
                    "[{},{},{},{}]",
                    p[0].to_f64(),
                    p[1].to_f64(),
                    p[2].to_f64(),
                    c.to_hex()
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let lines_js = self
            .lines
            .iter()
            .map(|(l, c)| {
                let s = l.start();
                let e = l.end();
                format!(
                    "[{},{},{},{},{},{},{}]",
                    s[0].to_f64(),
                    s[1].to_f64(),
                    s[2].to_f64(),
                    e[0].to_f64(),
                    e[1].to_f64(),
                    e[2].to_f64(),
                    c.to_hex()
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let highlight_lines_js = self
            .highlight_lines
            .iter()
            .map(|(l, c)| {
                let s = l.start();
                let e = l.end();
                format!(
                    "[{},{},{},{},{},{},{}]",
                    s[0].to_f64(),
                    s[1].to_f64(),
                    s[2].to_f64(),
                    e[0].to_f64(),
                    e[1].to_f64(),
                    e[2].to_f64(),
                    c.to_hex()
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let tris_js = self
            .triangles
            .iter()
            .map(|(t, c)| {
                format!(
                    "[{},{},{},{},{},{},{},{},{},{}]",
                    t.a[0].to_f64(),
                    t.a[1].to_f64(),
                    t.a[2].to_f64(),
                    t.b[0].to_f64(),
                    t.b[1].to_f64(),
                    t.b[2].to_f64(),
                    t.c[0].to_f64(),
                    t.c[1].to_f64(),
                    t.c[2].to_f64(),
                    c.to_hex()
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let tris_rgb_js = self
            .triangles_rgb
            .iter()
            .map(|(t, ca, cb, cc)| {
                format!(
                    "[{},{},{},{},{},{},{},{},{},{},{},{}]",
                    t.a[0].to_f64(),
                    t.a[1].to_f64(),
                    t.a[2].to_f64(),
                    t.b[0].to_f64(),
                    t.b[1].to_f64(),
                    t.b[2].to_f64(),
                    t.c[0].to_f64(),
                    t.c[1].to_f64(),
                    t.c[2].to_f64(),
                    ca.to_hex(),
                    cb.to_hex(),
                    cc.to_hex()
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let tris_transparent_js = self
            .triangles_transparent
            .iter()
            .map(|(t, c, opacity)| {
                format!(
                    "[{},{},{},{},{},{},{},{},{},{},{}]",
                    t.a[0].to_f64(),
                    t.a[1].to_f64(),
                    t.a[2].to_f64(),
                    t.b[0].to_f64(),
                    t.b[1].to_f64(),
                    t.b[2].to_f64(),
                    t.c[0].to_f64(),
                    t.c[1].to_f64(),
                    t.c[2].to_f64(),
                    c.to_hex(),
                    opacity
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let labels_js = self
            .labels
            .iter()
            .map(|(p, text, c)| {
                let escaped = text.replace('\\', "\\\\").replace('`', "'");
                format!(
                    "[{},{},{},`{}`,{}]",
                    p[0].to_f64(),
                    p[1].to_f64(),
                    p[2].to_f64(),
                    escaped,
                    c.to_hex()
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let cylinders_js = self
            .cylinders
            .iter()
            .map(|(s, e, r, c)| {
                format!(
                    "[{},{},{},{},{},{},{},{}]",
                    s[0].to_f64(),
                    s[1].to_f64(),
                    s[2].to_f64(),
                    e[0].to_f64(),
                    e[1].to_f64(),
                    e[2].to_f64(),
                    r,
                    c.to_hex()
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let text_js = self.debug_text.replace('`', "'").replace('\\', "\\\\");

        format!(
            r#"<!DOCTYPE html>
<html><head><meta charset="utf-8">
<title>Geop Debug Scene</title>
<style>body{{margin:0;overflow:hidden;background:#1a1a2e}}#info{{position:absolute;top:8px;left:8px;color:#ccc;font:13px monospace;white-space:pre;pointer-events:none}}.geop-label{{font:11px monospace;padding:0 2px;background:rgba(0,0,0,0.55);border-radius:2px;white-space:nowrap;pointer-events:none}}</style>
<script type="importmap">{{"imports":{{"three":"https://cdn.jsdelivr.net/npm/three@0.169.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.169.0/examples/jsm/"}}}}</script>
</head><body>
<div id="info"></div>
<script type="module">
import * as THREE from 'three';
import {{OrbitControls}} from 'three/addons/controls/OrbitControls.js';
import {{CSS2DRenderer, CSS2DObject}} from 'three/addons/renderers/CSS2DRenderer.js';

const renderer=new THREE.WebGLRenderer({{antialias:true}});
renderer.setSize(window.innerWidth,window.innerHeight);
renderer.setPixelRatio(devicePixelRatio);
document.body.appendChild(renderer.domElement);

const labelRenderer=new CSS2DRenderer();
labelRenderer.setSize(window.innerWidth,window.innerHeight);
labelRenderer.domElement.style.position='absolute';
labelRenderer.domElement.style.top='0';
labelRenderer.domElement.style.left='0';
labelRenderer.domElement.style.pointerEvents='none';
document.body.appendChild(labelRenderer.domElement);

const scene=new THREE.Scene();
scene.background=new THREE.Color(0x1a1a2e);
scene.add(new THREE.AmbientLight(0xffffff,0.6));
const dLight=new THREE.DirectionalLight(0xffffff,0.8);
dLight.position.set(5,10,7);
scene.add(dLight);

const camera=new THREE.PerspectiveCamera(60,innerWidth/innerHeight,0.001,10000);
const controls=new OrbitControls(camera,renderer.domElement);
controls.enableDamping=true;

const POINTS=[{points_js}];
const LINES=[{lines_js}];
const HIGHLIGHT_LINES=[{highlight_lines_js}];
const TRIS=[{tris_js}];
const TRIS_RGB=[{tris_rgb_js}];
const TRIS_TRANSPARENT=[{tris_transparent_js}];
const LABELS=[{labels_js}];
const CYLINDERS=[{cylinders_js}];
const TEXT=`{text_js}`;

document.getElementById('info').textContent=TEXT;

// Points
const ptGeo=new THREE.BufferGeometry();
if(POINTS.length){{
  const pos=new Float32Array(POINTS.length*3);
  const col=new Float32Array(POINTS.length*3);
  POINTS.forEach(([x,y,z,hex],i)=>{{
    pos[i*3]=x;pos[i*3+1]=y;pos[i*3+2]=z;
    const c=new THREE.Color(hex);col[i*3]=c.r;col[i*3+1]=c.g;col[i*3+2]=c.b;
  }});
  ptGeo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  ptGeo.setAttribute('color',new THREE.BufferAttribute(col,3));
  scene.add(new THREE.Points(ptGeo,new THREE.PointsMaterial({{size:0.05,vertexColors:true}})));
}}

// Lines
if(LINES.length){{
  const pos=new Float32Array(LINES.length*6);
  const col=new Float32Array(LINES.length*6);
  LINES.forEach(([x1,y1,z1,x2,y2,z2,hex],i)=>{{
    pos[i*6]=x1;pos[i*6+1]=y1;pos[i*6+2]=z1;
    pos[i*6+3]=x2;pos[i*6+4]=y2;pos[i*6+5]=z2;
    const c=new THREE.Color(hex);
    col[i*6]=c.r;col[i*6+1]=c.g;col[i*6+2]=c.b;
    col[i*6+3]=c.r;col[i*6+4]=c.g;col[i*6+5]=c.b;
  }});
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  geo.setAttribute('color',new THREE.BufferAttribute(col,3));
  scene.add(new THREE.LineSegments(geo,new THREE.LineBasicMaterial({{vertexColors:true}})));
}}

// Highlight lines (depth-test disabled, drawn last, so they stay visible on
// top of solid triangles instead of being occluded)
if(HIGHLIGHT_LINES.length){{
  const pos=new Float32Array(HIGHLIGHT_LINES.length*6);
  const col=new Float32Array(HIGHLIGHT_LINES.length*6);
  HIGHLIGHT_LINES.forEach(([x1,y1,z1,x2,y2,z2,hex],i)=>{{
    pos[i*6]=x1;pos[i*6+1]=y1;pos[i*6+2]=z1;
    pos[i*6+3]=x2;pos[i*6+4]=y2;pos[i*6+5]=z2;
    const c=new THREE.Color(hex);
    col[i*6]=c.r;col[i*6+1]=c.g;col[i*6+2]=c.b;
    col[i*6+3]=c.r;col[i*6+4]=c.g;col[i*6+5]=c.b;
  }});
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  geo.setAttribute('color',new THREE.BufferAttribute(col,3));
  const seg=new THREE.LineSegments(geo,new THREE.LineBasicMaterial({{vertexColors:true,depthTest:false,depthWrite:false}}));
  seg.renderOrder=999;
  scene.add(seg);
}}

// Triangles
if(TRIS.length){{
  const pos=new Float32Array(TRIS.length*9);
  const col=new Float32Array(TRIS.length*9);
  TRIS.forEach(([ax,ay,az,bx,by,bz,cx,cy,cz,hex],i)=>{{
    pos[i*9+0]=ax;pos[i*9+1]=ay;pos[i*9+2]=az;
    pos[i*9+3]=bx;pos[i*9+4]=by;pos[i*9+5]=bz;
    pos[i*9+6]=cx;pos[i*9+7]=cy;pos[i*9+8]=cz;
    const c=new THREE.Color(hex);
    for(let k=0;k<3;k++){{col[i*9+k*3]=c.r;col[i*9+k*3+1]=c.g;col[i*9+k*3+2]=c.b;}}
  }});
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  geo.setAttribute('color',new THREE.BufferAttribute(col,3));
  geo.computeVertexNormals();
  scene.add(new THREE.Mesh(geo,new THREE.MeshLambertMaterial({{vertexColors:true,side:THREE.DoubleSide}})));
}}

// Vertex-colored triangles
if(TRIS_RGB.length){{
  const pos=new Float32Array(TRIS_RGB.length*9);
  const col=new Float32Array(TRIS_RGB.length*9);
  TRIS_RGB.forEach(([ax,ay,az,bx,by,bz,cx,cy,cz,ha,hb,hc],i)=>{{
    pos[i*9+0]=ax;pos[i*9+1]=ay;pos[i*9+2]=az;
    pos[i*9+3]=bx;pos[i*9+4]=by;pos[i*9+5]=bz;
    pos[i*9+6]=cx;pos[i*9+7]=cy;pos[i*9+8]=cz;
    const ca=new THREE.Color(ha),cb=new THREE.Color(hb),cc=new THREE.Color(hc);
    col[i*9+0]=ca.r;col[i*9+1]=ca.g;col[i*9+2]=ca.b;
    col[i*9+3]=cb.r;col[i*9+4]=cb.g;col[i*9+5]=cb.b;
    col[i*9+6]=cc.r;col[i*9+7]=cc.g;col[i*9+8]=cc.b;
  }});
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  geo.setAttribute('color',new THREE.BufferAttribute(col,3));
  geo.computeVertexNormals();
  scene.add(new THREE.Mesh(geo,new THREE.MeshLambertMaterial({{vertexColors:true,side:THREE.DoubleSide}})));
}}

// Transparent triangles: one mesh per distinct (color, opacity) pair — not
// one per triangle (opacity is a per-material, not per-vertex, property in
// three.js, so triangles can't just be vertex-colored into a single mesh
// like TRIS/TRIS_RGB above) — but a curved face rasterized at any real
// resolution has thousands of same-colored triangles, and a separate
// Mesh/BufferGeometry/Material per one of those tanks frame rate; grouping
// keeps draw calls down to the number of distinct (color, opacity) pairs
// actually used, typically a handful.
{{
  const groups=new Map();
  TRIS_TRANSPARENT.forEach(([ax,ay,az,bx,by,bz,cx,cy,cz,hex,opacity])=>{{
    const key=hex+'|'+opacity;
    if(!groups.has(key))groups.set(key,{{hex,opacity,verts:[]}});
    groups.get(key).verts.push(ax,ay,az,bx,by,bz,cx,cy,cz);
  }});
  groups.forEach(({{hex,opacity,verts}})=>{{
    const geo=new THREE.BufferGeometry();
    geo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(verts),3));
    geo.computeVertexNormals();
    const mat=new THREE.MeshLambertMaterial({{color:hex,transparent:true,opacity,side:THREE.DoubleSide,depthWrite:false}});
    scene.add(new THREE.Mesh(geo,mat));
  }});
}}

// Cylinders (solid tubes, e.g. thick axes)
CYLINDERS.forEach(([x1,y1,z1,x2,y2,z2,radius,hex])=>{{
  const start=new THREE.Vector3(x1,y1,z1),end=new THREE.Vector3(x2,y2,z2);
  const dir=new THREE.Vector3().subVectors(end,start);
  const height=dir.length();
  if(height<=0)return;
  const geo=new THREE.CylinderGeometry(radius,radius,height,12);
  const mat=new THREE.MeshLambertMaterial({{color:hex}});
  const mesh=new THREE.Mesh(geo,mat);
  mesh.position.copy(start).addScaledVector(dir,0.5);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),dir.clone().normalize());
  scene.add(mesh);
}});

// Labels (CSS2D overlays, always facing the camera). Labels anchored at the
// same (or nearly the same) world position — e.g. a vertex and the midpoint
// label of a very short edge touching it — would otherwise land on the same
// screen pixels and hide each other, which reads as "nothing is there"
// instead of "there are two things here that need a closer look". Bucketing
// by a rounded position key and stacking each bucket's labels vertically
// (via a per-label CSS offset on an inner span, so it stays correct every
// frame without needing to redo the stacking on every camera move) keeps
// every label visible.
const labelBuckets=new Map();
LABELS.forEach(([x,y,z,text,hex])=>{{
  const key=x.toFixed(3)+','+y.toFixed(3)+','+z.toFixed(3);
  const bucket=labelBuckets.get(key)||[];
  bucket.push([x,y,z,text,hex]);
  labelBuckets.set(key,bucket);
}});
labelBuckets.forEach(bucket=>{{
  bucket.forEach(([x,y,z,text,hex],i)=>{{
    const outer=document.createElement('div');
    const inner=document.createElement('div');
    inner.className='geop-label';
    inner.textContent=text;
    inner.style.color='#'+hex.toString(16).padStart(6,'0');
    if(bucket.length>1){{
      inner.style.transform=`translateY(${{i*14}}px)`;
      inner.style.outline='1px solid #'+hex.toString(16).padStart(6,'0');
    }}
    outer.appendChild(inner);
    const obj=new CSS2DObject(outer);
    obj.position.set(x,y,z);
    scene.add(obj);
  }});
}});

// Fit camera to content
const box=new THREE.Box3().setFromObject(scene);
if(!box.isEmpty()){{
  const center=box.getCenter(new THREE.Vector3());
  const size=box.getSize(new THREE.Vector3()).length();
  camera.position.copy(center).addScaledVector(new THREE.Vector3(0.5,0.5,1).normalize(),size*1.5);
  controls.target.copy(center);
  camera.near=size/1000;camera.far=size*100;camera.updateProjectionMatrix();
}}

window.addEventListener('resize',()=>{{
  camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();
  renderer.setSize(innerWidth,innerHeight);
  labelRenderer.setSize(innerWidth,innerHeight);
}});

(function animate(){{requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);labelRenderer.render(scene,camera);}})();
</script></body></html>"#,
            points_js = points_js,
            lines_js = lines_js,
            tris_js = tris_js,
            tris_rgb_js = tris_rgb_js,
            tris_transparent_js = tris_transparent_js,
            labels_js = labels_js,
            text_js = text_js,
        )
    }
}

impl<S: Scalar> Default for PrimitiveScene<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Scalar> DebugContext for PrimitiveScene<S> {
    fn label(&self) -> &str {
        self.rendered_path.get_or_init(|| {
            let path = format!("/tmp/geop_scene_{}.html", next_id());
            if let Err(e) = self.save_to_file(&path) {
                return format!("PrimitiveScene (render failed: {e})");
            }
            path
        })
    }
}

// ── PrimitiveSceneRecorder ────────────────────────────────────────────────────

pub struct PrimitiveSceneRecorder<S: Scalar> {
    pub scenes: Vec<PrimitiveScene<S>>,
    rendered_path: OnceLock<String>,
}

impl<S: Scalar> core::fmt::Debug for PrimitiveSceneRecorder<S> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "PrimitiveSceneRecorder({} scenes)", self.scenes.len())
    }
}

impl<S: Scalar> PrimitiveSceneRecorder<S> {
    pub fn new() -> Self {
        Self {
            scenes: Vec::new(),
            rendered_path: OnceLock::new(),
        }
    }

    pub fn add_scene(&mut self, scene: PrimitiveScene<S>) {
        self.scenes.push(scene);
    }

    /// Write `scene_0.html`, `scene_1.html`, … into `folder_path`.
    pub fn save_to_folder(&self, folder_path: &str) -> GeopResult<()> {
        std::fs::create_dir_all(folder_path)
            .map_err(|e| GeopError::new(format!("PrimitiveSceneRecorder: mkdir {e}")))?;
        for (i, scene) in self.scenes.iter().enumerate() {
            let path = format!("{folder_path}/scene_{i}.html");
            scene.save_to_file(&path)?;
        }
        Ok(())
    }
}

impl<S: Scalar> Default for PrimitiveSceneRecorder<S> {
    fn default() -> Self {
        Self::new()
    }
}

impl<S: Scalar> DebugContext for PrimitiveSceneRecorder<S> {
    fn label(&self) -> &str {
        self.rendered_path.get_or_init(|| {
            let folder = format!("/tmp/geop_rec_{}", next_id());
            if let Err(e) = self.save_to_folder(&folder) {
                return format!("PrimitiveSceneRecorder (render failed: {e})");
            }
            folder
        })
    }
}
