// The 3-D view: faces lit as a studio lights them, lines and points of a
// constant width on screen, flat-coloured triangles, the stencil counters
// that cap a section, and the blit of what was drawn into egui's pass.
//
// Colours come in as sRGB and leave as sRGB: what is lit is lit in linear
// light, then encoded again.

struct View {
    view_proj: mat4x4<f32>,
    // The eye, and `w` 1 for an orthographic view.
    eye: vec4<f32>,
    // Towards the eye, for an orthographic view.
    back: vec4<f32>,
    // The viewport's width and height in pixels.
    viewport: vec4<f32>,
    // A section: its normal, and the plane's offset along it; `w` of
    // `cut_on` 1 while one is on.
    cut: vec4<f32>,
    cut_on: vec4<f32>,
};

@group(0) @binding(0) var<uniform> view: View;

fn to_linear(c: vec3<f32>) -> vec3<f32> {
    return pow(c, vec3<f32>(2.2));
}

fn to_srgb(c: vec3<f32>) -> vec3<f32> {
    return pow(clamp(c, vec3<f32>(0.0), vec3<f32>(1.0)), vec3<f32>(1.0 / 2.2));
}

fn cut_away(world: vec3<f32>) -> bool {
    return view.cut_on.w > 0.5 && dot(world, view.cut.xyz) > view.cut.w;
}

// ── faces ───────────────────────────────────────────────────────────────

struct FaceOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) world: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) color: vec4<f32>,
};

@vertex
fn vs_face(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) r0: vec4<f32>,
    @location(3) r1: vec4<f32>,
    @location(4) r2: vec4<f32>,
    @location(5) color: vec4<f32>,
) -> FaceOut {
    let p = vec4<f32>(position, 1.0);
    let world = vec3<f32>(dot(r0, p), dot(r1, p), dot(r2, p));
    let n = vec3<f32>(dot(r0.xyz, normal), dot(r1.xyz, normal), dot(r2.xyz, normal));
    var out: FaceOut;
    out.clip = view.view_proj * vec4<f32>(world, 1.0);
    out.world = world;
    out.normal = n;
    out.color = color;
    return out;
}

@fragment
fn fs_face(in: FaceOut) -> @location(0) vec4<f32> {
    if cut_away(in.world) {
        discard;
    }
    var n = normalize(in.normal);
    // Seen from behind, a face is lit as its other side.
    var to_eye = view.back.xyz;
    if view.eye.w < 0.5 {
        to_eye = normalize(view.eye.xyz - in.world);
    }
    if dot(n, to_eye) < 0.0 {
        n = -n;
    }
    let albedo = to_linear(in.color.rgb);
    let sky = to_linear(vec3<f32>(0.949, 0.961, 1.0));
    let ground = to_linear(vec3<f32>(0.188, 0.204, 0.235));
    let hemi = mix(ground, sky, 0.5 * n.y + 0.5) * 0.9;
    let key_dir = normalize(vec3<f32>(5.0, 8.0, 6.0));
    let fill_dir = normalize(vec3<f32>(-6.0, -2.0, -5.0));
    let key = max(dot(n, key_dir), 0.0) * 1.1;
    let fill = max(dot(n, fill_dir), 0.0) * 0.45 * to_linear(vec3<f32>(0.875, 0.91, 1.0));
    let half_dir = normalize(key_dir + to_eye);
    let spec = pow(max(dot(n, half_dir), 0.0), 24.0) * 0.12;
    let lit = albedo * (hemi * 0.55 + key * 0.75 + fill * 0.75) + vec3<f32>(spec);
    return vec4<f32>(to_srgb(lit), in.color.a);
}

// What a stencil counter needs of a face: where it is.
@fragment
fn fs_count(in: FaceOut) -> @location(0) vec4<f32> {
    if cut_away(in.world) {
        discard;
    }
    return vec4<f32>(0.0);
}

// ── flat triangles ──────────────────────────────────────────────────────

struct FlatOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
};

@vertex
fn vs_flat(@location(0) position: vec3<f32>, @location(1) color: vec4<f32>) -> FlatOut {
    var out: FlatOut;
    out.clip = view.view_proj * vec4<f32>(position, 1.0);
    out.color = color;
    return out;
}

@fragment
fn fs_flat(in: FlatOut) -> @location(0) vec4<f32> {
    return in.color;
}

// ── lines ───────────────────────────────────────────────────────────────

struct LineOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world: vec3<f32>,
    // Along the line from where its dashes are measured, in pixels; and
    // its dashes, on and off.
    @location(2) along: f32,
    @location(3) dash: vec2<f32>,
};

// A point moved a hair towards the eye: what lies on a face is drawn over
// it, not through it.
fn towards_eye(p: vec3<f32>) -> vec3<f32> {
    if view.eye.w > 0.5 {
        return p + view.back.xyz * (0.0015 * length(view.eye.xyz - p));
    }
    return p + (view.eye.xyz - p) * 0.0015;
}

// `a` moved along to `b` until it is in front of the eye.
fn in_front(a: vec4<f32>, b: vec4<f32>) -> vec4<f32> {
    let near = 1e-5;
    if a.w >= near {
        return a;
    }
    let t = (near - a.w) / (b.w - a.w);
    return mix(a, b, t);
}

@vertex
fn vs_line(
    @builtin(vertex_index) corner: u32,
    @location(0) a: vec3<f32>,
    @location(1) width: f32,
    @location(2) b: vec3<f32>,
    @location(3) start: f32,
    @location(4) color: vec4<f32>,
    @location(5) dash: f32,
    @location(6) gap: f32,
    @location(7) per_pixel: f32,
) -> LineOut {
    // Two triangles: `t` along the line, `side` across it.
    var ts = array<f32, 6>(0.0, 1.0, 1.0, 0.0, 1.0, 0.0);
    var sides = array<f32, 6>(-1.0, -1.0, 1.0, -1.0, 1.0, 1.0);
    let t = ts[corner];
    let side = sides[corner];
    let ca0 = view.view_proj * vec4<f32>(towards_eye(a), 1.0);
    let cb0 = view.view_proj * vec4<f32>(towards_eye(b), 1.0);
    let ca = in_front(ca0, cb0);
    let cb = in_front(cb0, ca0);
    let half_px = view.viewport.xy * 0.5;
    let pa = ca.xy / ca.w * half_px;
    let pb = cb.xy / cb.w * half_px;
    var dir = pb - pa;
    let len = length(dir);
    if len < 1e-6 {
        dir = vec2<f32>(1.0, 0.0);
    } else {
        dir = dir / len;
    }
    let across = vec2<f32>(-dir.y, dir.x);
    // Out by half the width across, and past each end along: round-ish
    // joints between segments.
    let along_px = (t * 2.0 - 1.0) * width * 0.5;
    let offset = across * side * width * 0.5 + dir * along_px;
    var clip = mix(ca, cb, t);
    clip = vec4<f32>(clip.xy + offset / half_px * clip.w, clip.zw);
    var out: LineOut;
    out.clip = clip;
    out.color = color;
    out.world = mix(a, b, t);
    var pixels = 0.0;
    if per_pixel > 0.0 {
        pixels = (start + t * length(b - a)) / per_pixel;
    }
    out.along = pixels;
    out.dash = vec2<f32>(dash, gap);
    return out;
}

fn dashed_away(in: LineOut) -> bool {
    if in.dash.x <= 0.0 {
        return false;
    }
    let period = in.dash.x + in.dash.y;
    return in.along - floor(in.along / period) * period > in.dash.x;
}

@fragment
fn fs_line(in: LineOut) -> @location(0) vec4<f32> {
    if dashed_away(in) {
        discard;
    }
    return in.color;
}

@fragment
fn fs_line_cut(in: LineOut) -> @location(0) vec4<f32> {
    if cut_away(in.world) || dashed_away(in) {
        discard;
    }
    return in.color;
}

// ── points ──────────────────────────────────────────────────────────────

struct PointOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) offset: vec2<f32>,
    @location(2) ring: f32,
    @location(3) world: vec3<f32>,
    @location(4) size: f32,
};

@vertex
fn vs_point(
    @builtin(vertex_index) corner: u32,
    @location(0) at: vec3<f32>,
    @location(1) size: f32,
    @location(2) color: vec4<f32>,
    @location(3) ring: f32,
) -> PointOut {
    var xs = array<f32, 6>(-1.0, 1.0, 1.0, -1.0, 1.0, -1.0);
    var ys = array<f32, 6>(-1.0, -1.0, 1.0, -1.0, 1.0, 1.0);
    let o = vec2<f32>(xs[corner], ys[corner]);
    var clip = view.view_proj * vec4<f32>(towards_eye(at), 1.0);
    let half_px = view.viewport.xy * 0.5;
    let r = size * 0.5 + 1.0;
    clip = vec4<f32>(clip.xy + o * r / half_px * clip.w, clip.zw);
    var out: PointOut;
    out.clip = clip;
    out.color = color;
    out.offset = o * r;
    out.ring = ring;
    out.world = at;
    out.size = size;
    return out;
}

fn point_color(in: PointOut) -> vec4<f32> {
    let r = length(in.offset);
    let outer = in.size * 0.5;
    var alpha = clamp(outer + 0.5 - r, 0.0, 1.0);
    if in.ring > 0.5 {
        alpha = alpha * clamp(r - (outer - 2.0) + 0.5, 0.0, 1.0);
    }
    if alpha <= 0.0 {
        discard;
    }
    return vec4<f32>(in.color.rgb, in.color.a * alpha);
}

@fragment
fn fs_point(in: PointOut) -> @location(0) vec4<f32> {
    return point_color(in);
}

@fragment
fn fs_point_cut(in: PointOut) -> @location(0) vec4<f32> {
    if cut_away(in.world) {
        discard;
    }
    return point_color(in);
}

// ── the blit into egui's pass ───────────────────────────────────────────

struct Blit {
    // `x` 1 where egui's target takes linear light.
    linear: vec4<f32>,
};

@group(0) @binding(0) var drawn: texture_2d<f32>;
@group(0) @binding(1) var drawn_sampler: sampler;
@group(0) @binding(2) var<uniform> blit: Blit;

struct BlitOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_blit(@builtin(vertex_index) i: u32) -> BlitOut {
    var xs = array<f32, 3>(-1.0, 3.0, -1.0);
    var ys = array<f32, 3>(-1.0, -1.0, 3.0);
    var out: BlitOut;
    out.clip = vec4<f32>(xs[i], ys[i], 0.0, 1.0);
    out.uv = vec2<f32>((xs[i] + 1.0) * 0.5, (1.0 - ys[i]) * 0.5);
    return out;
}

@fragment
fn fs_blit(in: BlitOut) -> @location(0) vec4<f32> {
    let c = textureSample(drawn, drawn_sampler, in.uv);
    if blit.linear.x > 0.5 {
        return vec4<f32>(to_linear(c.rgb), c.a);
    }
    return c;
}
