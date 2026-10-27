//! The 3-D camera: where it is, how the view projects, the pointer it
//! makes of a pixel ([`Camera::pointer`], reaching [`REACH_PX`] pixels, as
//! the web app's does), and how it moves — tumbled as a trackball, panned,
//! zoomed towards the pointer, with momentum that dies away, and gliding
//! eased to a pose it is sent to ([`Camera::glide_to`]): to face a plane
//! being worked in head on ([`head_on`]), or to frame the drawing
//! ([`fit`]).

use geop_core_editor::editing::{Pointer, Reach};
use geop_core_math::{primitives::Ray, scalars::Scalar, vector::Vector3};

use crate::{
    math::{
        Quat, V3, add, cross, distance, dot, length, lerp, mul, neg, normalized, normalized_or,
        rotate, scale, sub,
    },
    scene::Frame,
};

/// The vertical field of view, in degrees.
pub const FOV: f64 = 50.0;

/// How far the pointer reaches, in pixels: what counts as under it (see
/// [`Reach`]). Whatever the kernel lays out at a constant size on screen is
/// laid out in reaches, and drawn this many pixels each.
pub const REACH_PX: f64 = 9.0;

/// How long a glide takes, in seconds.
pub const GLIDE: f64 = 0.55;

/// How much of its momentum a tumble, a pan or a zoom keeps each frame of
/// a sixtieth of a second, once let go.
const MOMENTUM: f64 = 0.85;

/// How the view projects. Both show the same at the camera's target: an
/// orthographic view is sized from the camera's distance to it, as a
/// perspective one is, so switching keeps the view.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Projection {
    #[default]
    Perspective,
    Orthographic,
}

/// Where the camera is, what it looks at, and which way is up for it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub position: V3,
    pub target: V3,
    pub up: V3,
}

/// The view a session starts from: an empty part's drawing framed.
pub const DEFAULT_POSE: Pose = Pose {
    position: [66.0, 44.0, 88.0],
    target: [0.0, 0.0, 0.0],
    up: [0.0, 1.0, 0.0],
};

impl Pose {
    /// Looking from the target to the eye: the camera's `z`.
    pub fn back(&self) -> V3 {
        normalized(sub(self.position, self.target))
    }

    /// The camera's right, up and back, orthonormal.
    pub fn basis(&self) -> (V3, V3, V3) {
        let back = self.back();
        let right = normalized_or(cross(self.up, back), any_square_to(back));
        let up = cross(back, right);
        (right, up, back)
    }

    pub fn distance(&self) -> f64 {
        distance(self.position, self.target)
    }

    fn orientation(&self) -> Quat {
        let (right, up, back) = self.basis();
        Quat::from_basis(right, up, back)
    }
}

/// A unit vector square to `d`.
fn any_square_to(d: V3) -> V3 {
    let other = if d[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    normalized(cross(d, other))
}

/// Smoothstep: starting and ending at rest.
fn ease(t: f64) -> f64 {
    t * t * (3.0 - 2.0 * t)
}

/// What a glide goes from and to, and how far it is.
#[derive(Clone, Copy, Debug)]
struct Glide {
    from: Pose,
    to: Pose,
    from_q: Quat,
    to_q: Quat,
    elapsed: f64,
}

/// What a drag of the camera does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Drag {
    Tumble,
    Pan,
}

/// The camera, and what moves it (see the module docs).
#[derive(Clone, Debug)]
pub struct Camera {
    pub pose: Pose,
    pub projection: Projection,
    /// The viewport's size, in points.
    pub size: [f64; 2],
    glide: Option<Glide>,
    /// What is left of the last tumble — an axis and an angle a frame —
    /// and of the last pan, in the world a frame.
    spin: Option<(V3, f64)>,
    drift: V3,
    /// Whether the camera moved since the view was last said to settle.
    moved: bool,
}

impl Camera {
    pub fn new() -> Self {
        Self {
            pose: DEFAULT_POSE,
            projection: Projection::Perspective,
            size: [800.0, 600.0],
            glide: None,
            spin: None,
            drift: [0.0; 3],
            moved: false,
        }
    }

    fn height(&self) -> f64 {
        self.size[1].max(1.0)
    }

    fn aspect(&self) -> f64 {
        self.size[0].max(1.0) / self.height()
    }

    fn tan_half(&self) -> f64 {
        (FOV.to_radians() / 2.0).tan()
    }

    /// Half the height the view shows at its target, in the world.
    fn half_height(&self) -> f64 {
        self.pose.distance() * self.tan_half()
    }

    /// What one pixel measures, in the world, at `at`.
    pub fn world_per_pixel(&self, at: V3) -> f64 {
        let along = match self.projection {
            Projection::Orthographic => self.pose.distance(),
            Projection::Perspective => dot(sub(at, self.pose.position), neg(self.pose.back())).max(1e-9),
        };
        2.0 * along * self.tan_half() / self.height()
    }

    /// How large a reach is, in the world, at `at`.
    pub fn reach_at(&self, at: V3) -> f64 {
        self.world_per_pixel(at) * REACH_PX
    }

    /// The ray through the point `(x, y)` of the viewport, in points from
    /// its top left.
    pub fn ray(&self, x: f64, y: f64) -> (V3, V3) {
        let (right, up, back) = self.pose.basis();
        let nx = 2.0 * x / self.size[0].max(1.0) - 1.0;
        let ny = 1.0 - 2.0 * y / self.height();
        match self.projection {
            Projection::Perspective => {
                let t = self.tan_half();
                let dir = add(
                    neg(back),
                    add(scale(right, nx * t * self.aspect()), scale(up, ny * t)),
                );
                (self.pose.position, normalized(dir))
            }
            Projection::Orthographic => {
                let half = self.half_height();
                let at = add(
                    self.pose.position,
                    add(scale(right, nx * half * self.aspect()), scale(up, ny * half)),
                );
                (at, neg(back))
            }
        }
    }

    /// The pointer at `(x, y)`: the ray through it, reaching [`REACH_PX`]
    /// pixels — a cone from the eye in perspective, a tube in an
    /// orthographic view.
    pub fn pointer(&self, x: f64, y: f64) -> Option<Pointer> {
        let (origin, dir) = self.ray(x, y);
        let v = |p: V3| Vector3::from_array(p.map(Scalar::from_f64));
        let ray = Ray::try_new(v(origin), v(dir)).ok()?;
        let reach = match self.projection {
            Projection::Orthographic => Reach::Tube {
                radius: Scalar::from_f64(REACH_PX * 2.0 * self.half_height() / self.height()),
            },
            Projection::Perspective => Reach::Cone {
                slope: Scalar::from_f64(REACH_PX * 2.0 * self.tan_half() / self.height()),
            },
        };
        Some(Pointer { ray, reach })
    }

    /// The world's direction that points up the screen: what a step names
    /// what it shows by ([`geop_core_editor::editing::StepEditEvent::View`]).
    pub fn up(&self) -> V3 {
        self.pose.basis().1
    }

    /// The view and projection, as the GPU takes them, near and far
    /// following the distance to the target and the drawing's `size`, so
    /// that neither a large assembly nor a close look is clipped.
    pub fn view_projection(&self, size: f64) -> [[f64; 4]; 4] {
        let (r, u, b) = self.pose.basis();
        let e = self.pose.position;
        let view = [
            [r[0], u[0], b[0], 0.0],
            [r[1], u[1], b[1], 0.0],
            [r[2], u[2], b[2], 0.0],
            [-dot(r, e), -dot(u, e), -dot(b, e), 1.0],
        ];
        let reach = self.pose.distance() + size.max(0.0);
        let projection = match self.projection {
            Projection::Perspective => {
                let (near, far) = (reach / 10000.0, reach * 10.0);
                let f = 1.0 / self.tan_half();
                // Depth 0 at near, 1 at far, as wgpu takes it.
                [
                    [f / self.aspect(), 0.0, 0.0, 0.0],
                    [0.0, f, 0.0, 0.0],
                    [0.0, 0.0, far / (near - far), -1.0],
                    [0.0, 0.0, near * far / (near - far), 0.0],
                ]
            }
            Projection::Orthographic => {
                let (near, far) = (-reach * 10.0, reach * 10.0);
                let top = self.half_height();
                let right = top * self.aspect();
                [
                    [1.0 / right, 0.0, 0.0, 0.0],
                    [0.0, 1.0 / top, 0.0, 0.0],
                    [0.0, 0.0, 1.0 / (near - far), 0.0],
                    [0.0, 0.0, near / (near - far), 1.0],
                ]
            }
        };
        mul(&projection, &view)
    }

    /// Where `p` is in the viewport, in points from its top left, and
    /// whether it is in front of the eye.
    pub fn project(&self, p: V3) -> Option<[f64; 2]> {
        let (right, up, back) = self.pose.basis();
        let d = sub(p, self.pose.position);
        let (x, y, z) = (dot(d, right), dot(d, up), -dot(d, back));
        let (nx, ny) = match self.projection {
            Projection::Perspective => {
                if z <= 1e-12 {
                    return None;
                }
                let t = self.tan_half();
                (x / (z * t * self.aspect()), y / (z * t))
            }
            Projection::Orthographic => {
                let half = self.half_height();
                (x / (half * self.aspect()), y / half)
            }
        };
        Some([
            (nx + 1.0) / 2.0 * self.size[0],
            (1.0 - ny) / 2.0 * self.height(),
        ])
    }

    /// Whether the camera is gliding to a pose.
    pub fn gliding(&self) -> bool {
        self.glide.is_some()
    }

    /// Glide, eased, from where the camera is to `to`.
    pub fn glide_to(&mut self, to: Pose) {
        self.spin = None;
        self.drift = [0.0; 3];
        self.glide = Some(Glide {
            from: self.pose,
            to,
            from_q: self.pose.orientation(),
            to_q: to.orientation(),
            elapsed: 0.0,
        });
    }

    /// Frame `extent` — `center` and `size` — from where the camera looks
    /// now, or from `from`'s direction and with its up.
    pub fn fit(&mut self, center: V3, size: f64, from: Option<Pose>) {
        let pose = fit(center, size, &from.unwrap_or(self.pose), self.aspect());
        self.glide_to(pose);
    }

    /// Tumble as a trackball by a drag of `(dx, dy)` points: about the axis
    /// square to the drag on screen, through the target.
    pub fn tumble(&mut self, dx: f64, dy: f64) {
        if self.glide.is_some() {
            return;
        }
        let (right, up, back) = self.pose.basis();
        let moved = add(scale(right, dx), scale(up, -dy));
        let l = length(moved);
        if l == 0.0 {
            return;
        }
        let angle = 3.0 * l / self.height();
        let axis = normalized(cross(moved, back));
        // The drag turns what is seen with it: the camera turns the other way.
        self.turn(axis, -angle);
        self.spin = Some((axis, -angle));
    }

    fn turn(&mut self, axis: V3, angle: f64) {
        let eye = sub(self.pose.position, self.pose.target);
        self.pose.position = add(self.pose.target, rotate(eye, axis, angle));
        self.pose.up = normalized(rotate(self.pose.up, axis, angle));
        self.moved = true;
    }

    /// Pan by a drag of `(dx, dy)` points: what is under the pointer stays
    /// under it.
    pub fn pan(&mut self, dx: f64, dy: f64) {
        if self.glide.is_some() {
            return;
        }
        let (right, up, _) = self.pose.basis();
        let k = self.world_per_pixel(self.pose.target);
        let by = add(scale(right, -dx * k), scale(up, dy * k));
        self.shift(by);
        self.drift = by;
    }

    fn shift(&mut self, by: V3) {
        self.pose.position = add(self.pose.position, by);
        self.pose.target = add(self.pose.target, by);
        self.moved = true;
    }

    /// Zoom by `steps` (positive in) towards the point of the viewport at
    /// `(x, y)`: what is under the pointer stays under it.
    pub fn zoom(&mut self, steps: f64, x: f64, y: f64) {
        if self.glide.is_some() || steps == 0.0 {
            return;
        }
        let factor = 0.85f64.powf(steps);
        let back = self.pose.back();
        // The point under the pointer in the plane of the target, facing
        // the eye.
        let (origin, dir) = self.ray(x, y);
        let denom = dot(dir, back);
        let under = if denom.abs() > 1e-12 {
            let t = dot(sub(self.pose.target, origin), back) / denom;
            add(origin, scale(dir, t))
        } else {
            self.pose.target
        };
        let d = self.pose.distance() * factor;
        let target = lerp(under, self.pose.target, factor);
        self.pose.target = target;
        self.pose.position = add(target, scale(back, d.max(1e-6)));
        self.moved = true;
    }

    /// Let go of a drag: what it was doing goes on, dying away.
    pub fn release(&mut self, drag: Drag) {
        match drag {
            Drag::Tumble => self.drift = [0.0; 3],
            Drag::Pan => self.spin = None,
        }
    }

    /// Stop any momentum: a press holds the view where it is.
    pub fn hold(&mut self) {
        self.spin = None;
        self.drift = [0.0; 3];
    }

    /// Advance by `dt` seconds: a glide, or momentum dying away. Whether
    /// it is still moving — the window draws again while it is — and
    /// whether it came to rest just now, for the view to be told
    /// ([`Camera::settled`]).
    pub fn advance(&mut self, dt: f64) -> bool {
        if let Some(glide) = &mut self.glide {
            glide.elapsed += dt;
            let t = (glide.elapsed / GLIDE).min(1.0);
            let k = ease(t);
            let q = glide.from_q.slerp(&glide.to_q, k);
            let target = lerp(glide.from.target, glide.to.target, k);
            let d = glide.from.distance() + (glide.to.distance() - glide.from.distance()) * k;
            let back = q.apply([0.0, 0.0, 1.0]);
            self.pose = Pose {
                position: add(target, scale(back, d)),
                target,
                up: q.apply([0.0, 1.0, 0.0]),
            };
            if t >= 1.0 {
                self.pose = glide.to;
                self.glide = None;
            }
            self.moved = true;
            return true;
        }
        let keep = MOMENTUM.powf(dt * 60.0);
        let mut moving = false;
        if let Some((axis, angle)) = self.spin {
            let angle = angle * keep;
            if angle.abs() < 1e-5 {
                self.spin = None;
            } else {
                self.turn(axis, angle * dt * 60.0);
                self.spin = Some((axis, angle));
                moving = true;
            }
        }
        if length(self.drift) > 0.0 {
            let drift = scale(self.drift, keep);
            if length(drift) < 1e-4 * self.world_per_pixel(self.pose.target) {
                self.drift = [0.0; 3];
            } else {
                self.shift(scale(drift, dt * 60.0));
                self.drift = drift;
                moving = true;
            }
        }
        moving
    }

    /// Whether the view came to rest since it was last asked: a glide
    /// landed, momentum died away, or a drag was let go with none.
    pub fn settled(&mut self, held: bool) -> bool {
        let at_rest = !held
            && self.glide.is_none()
            && self.spin.is_none()
            && length(self.drift) == 0.0;
        if at_rest && self.moved {
            self.moved = false;
            return true;
        }
        false
    }
}

impl Default for Camera {
    fn default() -> Self {
        Self::new()
    }
}

/// The view framing the ball around `center` of diameter `size`, in a
/// viewport of `aspect`: looking at its centre from the direction, and
/// with the up, `pose` has, from just far enough that the ball fits both
/// ways.
pub fn fit(center: V3, size: f64, pose: &Pose, aspect: f64) -> Pose {
    let direction = normalized_or(sub(pose.position, pose.target), [0.0, 0.0, 1.0]);
    let half = FOV.to_radians() / 2.0;
    let narrowest = half.min((half.tan() * aspect).atan());
    let d = size / 2.0 / narrowest.sin();
    Pose {
        position: add(center, scale(direction, d)),
        target: center,
        up: pose.up,
    }
}

/// The same view as `pose`, turned as little as it can to face `frame`'s
/// `u`/`v` plane head on: the same look-at point (projected onto the
/// plane) and distance, from the side of the plane the camera is on, and
/// with whichever of `±u`, `±v` is nearest the camera's up as up. Working
/// in a plane rotates the view into it, rather than cutting elsewhere.
pub fn head_on(frame: &Frame, pose: &Pose) -> Pose {
    let d = sub(pose.target, frame.origin);
    let center = [dot(d, frame.u), dot(d, frame.v)];
    let eye = sub(pose.position, pose.target);
    let distance = length(eye);
    let side = if dot(eye, frame.w) < 0.0 { -1.0 } else { 1.0 };
    let ups = [frame.v, frame.u, neg(frame.u), neg(frame.v)];
    let up = ups
        .into_iter()
        .fold(ups[0], |best, a| if dot(a, pose.up) > dot(best, pose.up) { a } else { best });
    Pose {
        position: frame.point([center[0], center[1], side * distance]),
        target: frame.in_plane(center),
        up,
    }
}

/// What a drawing is, for whether to frame it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Drawn {
    pub size: f64,
    pub solids: usize,
}

/// Whether the drawing, `before` and now `after`, is to be framed from a
/// camera at `pose`: when its first solid or placed part appears, or when
/// it grew by half again and is now twice as big as the view shows. Any
/// other edit leaves the camera where the user put it.
pub fn wants_framing(before: Drawn, after: Drawn, pose: &Pose) -> bool {
    if before.solids == 0 && after.solids > 0 {
        return true;
    }
    let shown = 2.0 * pose.distance() * (FOV.to_radians() / 2.0).tan();
    after.size > 1.5 * before.size && after.size > 2.0 * shown
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: V3, b: V3) -> bool {
        distance(a, b) < 1e-6
    }

    #[test]
    fn the_middle_of_the_view_is_the_ray_to_the_target_and_projects_back() {
        let mut camera = Camera::new();
        camera.size = [800.0, 600.0];
        for projection in [Projection::Perspective, Projection::Orthographic] {
            camera.projection = projection;
            let (_, dir) = camera.ray(400.0, 300.0);
            assert!(near(dir, neg(camera.pose.back())));
            let p = camera.project([0.0, 0.0, 0.0]).unwrap();
            assert!((p[0] - 400.0).abs() < 1e-6 && (p[1] - 300.0).abs() < 1e-6, "{p:?}");
            // A point a few pixels right of the middle lies along the ray there.
            let (o, d) = camera.ray(500.0, 200.0);
            let p = camera.project(add(o, scale(d, 50.0))).unwrap();
            assert!((p[0] - 500.0).abs() < 1e-6 && (p[1] - 200.0).abs() < 1e-6, "{p:?}");
        }
    }

    #[test]
    fn a_reach_is_nine_pixels_where_the_camera_looks() {
        let mut camera = Camera::new();
        camera.size = [800.0, 600.0];
        let pointer = camera.pointer(400.0, 300.0).unwrap();
        let t = Scalar::from_f64(camera.pose.distance());
        let reach = pointer.reach.at(t).to_f64();
        assert!((reach - camera.reach_at(camera.pose.target)).abs() < 1e-9);
    }

    #[test]
    fn zooming_keeps_what_is_under_the_pointer_under_it() {
        let mut camera = Camera::new();
        camera.size = [800.0, 600.0];
        let (o, d) = camera.ray(600.0, 100.0);
        let back = camera.pose.back();
        let t = dot(sub(camera.pose.target, o), back) / dot(d, back);
        let under = add(o, scale(d, t));
        camera.zoom(3.0, 600.0, 100.0);
        let p = camera.project(under).unwrap();
        assert!((p[0] - 600.0).abs() < 1e-6 && (p[1] - 100.0).abs() < 1e-6, "{p:?}");
    }

    #[test]
    fn a_glide_lands_on_its_pose_and_settles_once() {
        let mut camera = Camera::new();
        let to = head_on(&Frame::WORLD, &camera.pose);
        camera.glide_to(to);
        let mut frames = 0;
        while camera.gliding() {
            assert!(camera.advance(1.0 / 60.0));
            frames += 1;
            if camera.gliding() {
                assert!(!camera.settled(false));
            }
        }
        assert!(!camera.advance(1.0 / 60.0));
        assert!(frames > 10);
        assert_eq!(camera.pose, to);
        assert!(camera.settled(false));
        assert!(!camera.settled(false));
        // Facing the xy plane from the side the camera was on, its up the
        // plane's direction nearest the old up.
        assert!(near(camera.pose.back(), [0.0, 0.0, 1.0]));
        assert!(near(camera.pose.up, [0.0, 1.0, 0.0]));
    }

    #[test]
    fn a_tumble_spins_on_and_dies_away() {
        let mut camera = Camera::new();
        camera.tumble(30.0, 0.0);
        camera.release(Drag::Tumble);
        let mut frames = 0;
        while camera.advance(1.0 / 60.0) {
            frames += 1;
            assert!(frames < 1000, "the spin dies away");
        }
        assert!(frames > 5);
        assert!((camera.pose.distance() - DEFAULT_POSE.position.map(|x| x * x).iter().sum::<f64>().sqrt()).abs() < 1e-6);
    }

    #[test]
    fn a_drawing_is_framed_when_its_first_solid_appears_or_it_outgrows_the_view() {
        let pose = DEFAULT_POSE;
        let empty = Drawn { size: 100.0, solids: 0 };
        let one = Drawn { size: 10.0, solids: 1 };
        assert!(wants_framing(empty, one, &pose));
        assert!(!wants_framing(one, Drawn { size: 12.0, solids: 1 }, &pose));
        assert!(wants_framing(one, Drawn { size: 1000.0, solids: 1 }, &pose));
    }
}
