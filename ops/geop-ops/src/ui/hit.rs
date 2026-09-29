//! Hit tests: which [`Visual`] a [`Pointer`] is over, and the ray geometry
//! behind it, shared by every operation so a click means the same thing
//! whatever is being edited.
//!
//! Tolerances are in screen pixels, turned into world units where along the
//! ray the thing is (see [`super::PixelScale`]), so "near" means near on
//! screen however far away or zoomed in the view is. This is UI geometry in
//! plain `f64`, like the visuals themselves: how close a click has to land
//! is a question about the screen, not one the kernel's interval arithmetic
//! answers.

use super::{Pointer, Shape, Visual};

/// How near a point or a curve a click has to land, in pixels.
pub const HIT_PX: f64 = 9.0;
/// A handle's radius on screen, in pixels; its arrows reach three times as
/// far along its direction.
pub const HANDLE_PX: f64 = 7.0;
/// Half the width and height of a label's box on screen, in pixels.
pub const LABEL_PX: [f64; 2] = [12.0, 9.0];

pub(crate) fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(crate) fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(crate) fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub(crate) fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(crate) fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub(crate) fn norm(a: [f64; 3]) -> f64 {
    dot(a, a).sqrt()
}

impl Pointer {
    /// The point `t` along the ray.
    pub fn at(&self, t: f64) -> [f64; 3] {
        add(self.origin, scale(self.dir, t))
    }

    /// `pixels` screen pixels, in world units, at distance `t` along the ray.
    pub fn pixels(&self, pixels: f64, t: f64) -> f64 {
        pixels * self.pixel.at(t)
    }
}

/// How far the ray passes from `p`, and the distance along it (never
/// behind its origin) where it comes closest.
pub fn ray_point(pointer: &Pointer, p: [f64; 3]) -> (f64, f64) {
    let t = dot(sub(p, pointer.origin), pointer.dir).max(0.0);
    (norm(sub(pointer.at(t), p)), t)
}

/// How far the ray passes from the segment `a..b`, and the distance along
/// the ray where it comes closest.
pub fn ray_segment(pointer: &Pointer, a: [f64; 3], b: [f64; 3]) -> (f64, f64) {
    let d = sub(b, a);
    let r = sub(pointer.origin, a);
    let e = dot(d, d);
    if e == 0.0 {
        return ray_point(pointer, a);
    }
    let b_ = dot(pointer.dir, d);
    let c = dot(pointer.dir, r);
    let f = dot(d, r);
    // `pointer.dir` is unit length, so `dot(dir, dir) = 1`.
    let denom = e - b_ * b_;
    let mut t = if denom > 0.0 {
        ((b_ * f - c * e) / denom).max(0.0)
    } else {
        0.0
    };
    let mut u = (b_ * t + f) / e;
    if !(0.0..=1.0).contains(&u) {
        u = u.clamp(0.0, 1.0);
        t = dot(sub(add(a, scale(d, u)), pointer.origin), pointer.dir).max(0.0);
    }
    (norm(sub(pointer.at(t), add(a, scale(d, u)))), t)
}

/// Where the ray enters the triangle `a, b, c`, if it does
/// (Möller–Trumbore).
pub fn ray_triangle(pointer: &Pointer, a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> Option<f64> {
    let e1 = sub(b, a);
    let e2 = sub(c, a);
    let h = cross(pointer.dir, e2);
    let det = dot(e1, h);
    if det == 0.0 {
        return None;
    }
    let s = sub(pointer.origin, a);
    let u = dot(s, h) / det;
    let q = cross(s, e1);
    let v = dot(pointer.dir, q) / det;
    let t = dot(e2, q) / det;
    (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t >= 0.0).then_some(t)
}

/// Where the ray meets the plane through `origin` normal to `normal`, and
/// how far along it: `None` if it runs along the plane or away from it.
pub fn ray_plane(pointer: &Pointer, origin: [f64; 3], normal: [f64; 3]) -> Option<(f64, [f64; 3])> {
    let denom = dot(pointer.dir, normal);
    if denom == 0.0 {
        return None;
    }
    let t = dot(sub(origin, pointer.origin), normal) / denom;
    (t >= 0.0).then(|| (t, pointer.at(t)))
}

/// The parameter `s` of the point `at + s direction` of a line nearest the
/// ray — where along its track a handle is grabbed. `None` when looking
/// straight along the line, where no point of it is nearer than another.
pub fn line_parameter(pointer: &Pointer, at: [f64; 3], direction: [f64; 3]) -> Option<f64> {
    let w0 = sub(at, pointer.origin);
    let b = dot(direction, pointer.dir);
    let dd = dot(direction, direction);
    let denom = dd - b * b;
    if denom <= 1e-12 * dd {
        return None;
    }
    Some((b * dot(pointer.dir, w0) - dot(direction, w0)) / denom)
}

/// A visual the pointer is over.
#[derive(Clone, Copy, Debug)]
pub struct VisualHit<'a> {
    pub visual: &'a Visual,
    /// How far along the ray.
    pub t: f64,
}

/// How near the pointer is to `visual`, in pixels, and where along the ray
/// — `None` if not near enough to count. `rank` orders kinds of shapes:
/// what is drawn small and on top wins over what is drawn large.
fn distance(pointer: &Pointer, visual: &Visual) -> Option<(u8, f64, f64)> {
    let within = |dist: f64, t: f64, px: f64| {
        let pixels = dist / pointer.pixel.at(t);
        (pixels <= px).then_some((pixels, t))
    };
    match &visual.shape {
        Shape::Handle { at, direction } => {
            let (_, t) = ray_point(pointer, *at);
            let radius = pointer.pixels(HANDLE_PX, t);
            let (dist, t) = match direction {
                Some(d) => {
                    let reach = scale(*d, 3.0 * radius / norm(*d));
                    ray_segment(pointer, sub(*at, reach), add(*at, reach))
                }
                None => ray_point(pointer, *at),
            };
            within(dist, t, HANDLE_PX).map(|(p, t)| (0, p, t))
        }
        Shape::Label { at, offset, .. } => {
            let (_, t) = ray_point(pointer, *at);
            let px = pointer.pixel.at(t);
            let center = add(
                *at,
                add(
                    scale(pointer.right, offset[0] * px),
                    scale(pointer.up, offset[1] * px),
                ),
            );
            let (_, t) = ray_point(pointer, center);
            let d = sub(pointer.at(t), center);
            let (dx, dy) = (dot(d, pointer.right) / px, dot(d, pointer.up) / px);
            (dx.abs() <= LABEL_PX[0] && dy.abs() <= LABEL_PX[1]).then_some((0, dx.hypot(dy), t))
        }
        Shape::Point { at } => {
            let (dist, t) = ray_point(pointer, *at);
            within(dist, t, HIT_PX).map(|(p, t)| (1, p, t))
        }
        Shape::Polyline { points } => points
            .windows(2)
            .filter_map(|w| {
                let (dist, t) = ray_segment(pointer, w[0], w[1]);
                within(dist, t, HIT_PX)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(p, t)| (2, p, t)),
        Shape::Triangles { triangles } => triangles
            .iter()
            .filter_map(|[a, b, c]| ray_triangle(pointer, *a, *b, *c))
            .min_by(f64::total_cmp)
            .map(|t| (3, 0.0, t)),
    }
}

/// The visual among those `accept` takes that the pointer is over: handles
/// and labels first, then points, curves and areas — within each, the one
/// nearest the pointer on screen.
pub fn hit_visuals<'a>(
    visuals: &'a [Visual],
    pointer: &Pointer,
    accept: impl Fn(&Visual) -> bool,
) -> Option<VisualHit<'a>> {
    visuals
        .iter()
        .filter(|v| accept(v))
        .filter_map(|v| distance(pointer, v).map(|(rank, pixels, t)| (rank, pixels, t, v)))
        .min_by(|a, b| a.0.cmp(&b.0).then(a.1.total_cmp(&b.1)))
        .map(|(_, _, t, visual)| VisualHit { visual, t })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ui::{PixelScale, Style};

    /// Looking straight down `-z` from `(x, y, 10)`, one pixel a hundredth
    /// of a unit wherever it looks.
    fn down(x: f64, y: f64) -> Pointer {
        Pointer {
            origin: [x, y, 10.0],
            dir: [0.0, 0.0, -1.0],
            right: [1.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            pixel: PixelScale {
                at_origin: 0.01,
                per_distance: 0.0,
            },
        }
    }

    fn point(key: &str, at: [f64; 3]) -> Visual {
        Visual::new(key, Shape::Point { at }, Style::Free)
    }

    /// A point wins over the curve it lies on, within a few pixels of it;
    /// further along, the curve is hit.
    #[test]
    fn points_win_over_curves() {
        let visuals = [
            Visual::new(
                "line",
                Shape::Polyline {
                    points: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
                },
                Style::Free,
            ),
            point("end", [1.0, 0.0, 0.0]),
        ];
        let key = |x: f64, y: f64| {
            hit_visuals(&visuals, &down(x, y), |_| true).map(|h| h.visual.key.clone())
        };
        assert_eq!(key(0.95, 0.0).as_deref(), Some("end"));
        assert_eq!(key(0.5, 0.05).as_deref(), Some("line"));
        assert_eq!(key(0.5, 0.2), None);
    }

    /// A label is hit in its box, which is moved on screen by its offset.
    #[test]
    fn labels_are_hit_where_they_are_drawn() {
        let visuals = [Visual::new(
            "k1",
            Shape::Label {
                at: [0.0, 0.0, 0.0],
                text: "H".into(),
                offset: [20.0, 10.0],
            },
            Style::Free,
        )];
        assert!(hit_visuals(&visuals, &down(0.2, 0.1), |_| true).is_some());
        assert!(hit_visuals(&visuals, &down(0.0, 0.0), |_| true).is_none());
    }

    /// Dragging along a handle's direction: the grab parameter follows the
    /// pointer's projection onto its line.
    #[test]
    fn line_parameters_follow_the_pointer() {
        // Seen from the side: the line runs along z through the origin.
        let side = |z: f64| Pointer {
            origin: [5.0, 0.0, z],
            dir: [-1.0, 0.0, 0.0],
            right: [0.0, 1.0, 0.0],
            up: [0.0, 0.0, 1.0],
            pixel: PixelScale {
                at_origin: 0.01,
                per_distance: 0.0,
            },
        };
        let s = line_parameter(&side(0.7), [0.0, 0.0, 0.0], [0.0, 0.0, 2.0]).unwrap();
        assert!((s - 0.35).abs() < 1e-12);
        assert!(line_parameter(&down(0.0, 0.0), [0.0; 3], [0.0, 0.0, 1.0]).is_none());
    }
}
