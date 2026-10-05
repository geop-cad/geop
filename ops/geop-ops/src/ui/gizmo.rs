//! [`Gizmo`]: moving, turning and scaling something in the viewport by
//! dragging a manipulator drawn at it — the one way every operation offers
//! a drag in three dimensions.
//!
//! An operation asks for a gizmo at a point, with the modes it allows and,
//! if what it stands on has an orientation of its own, its axes (see
//! [`super::Form::gizmo`]). The editor draws it, tests the pointer against
//! it, follows a drag of one of its parts and snaps it, and sends the
//! operation what the drag did as a [`GizmoDrag`]: a translation, a turn
//! about one of its axes, or a scale along them — always measured from
//! where the drag started, and applied by the operation to its arguments
//! as they were then (see [`super::StepEditor`]).
//!
//! It is laid out in the pointer's reaches (see [`super::Reach`]), so it
//! keeps its size on screen however far away or zoomed in the view is: a
//! ball at its centre to drag freely in the plane facing the eye, an arrow
//! along each axis and a square in each plane of two of them, a ring about
//! each axis, a cube beyond each arrow to stretch along it and one on the
//! diagonal to scale evenly. The viewer draws it by the same sizes, from
//! [`GizmoView`].

use geop_core_math::{primitives::Pose, scalars::Scalar, vector::Vector3};
use serde::Serialize;

use super::{Pointer, hit::nearer};

/// How the gizmo is laid out, in reaches of the pointer at its centre.
pub mod size {
    /// The radius of the ball at the centre.
    pub const FREE: f64 = 1.4;
    /// Where an arrow starts, out from the centre, and where its tip is.
    pub const ARROW: (f64, f64) = (2.5, 10.0);
    /// Where a plane's square starts and ends, along both its axes.
    pub const PLANE: (f64, f64) = (2.5, 4.0);
    /// The radius of the rings.
    pub const RING: f64 = 7.0;
    /// How far out a cube to scale by sits: along its axis, or the
    /// diagonal.
    pub const CUBE_AT: f64 = 12.5;
    /// Half the size of such a cube.
    pub const CUBE: f64 = 0.9;
    /// How near the pointer has to pass to an arrow, a ring or a cube.
    pub const NEAR: f64 = 1.0;
    /// How far from looking along it an arrow, or a cube, is still shown —
    /// and how far from looking along its plane a square is: the sine of
    /// the angle. Nearer, it is foreshortened to a dot or a line on
    /// screen, and a drag of it could not be followed.
    pub const FACING: f64 = 0.2;
}

/// The finest spacing of the grid the viewer draws on a plane worked in,
/// in reaches: what a translation snaps to is the power of ten at least
/// this far on screen.
pub const GRID: f64 = 20.0 / 9.0;
/// What a turn snaps to, in degrees.
pub const ANGLE_SNAP: f64 = 15.0;
/// What a scale factor snaps to.
pub const SCALE_SNAP: f64 = 0.1;

/// What a gizmo lets the user do.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct Modes {
    /// Move along an axis, in a plane of two, or freely.
    pub translate: bool,
    /// Turn about an axis.
    pub rotate: bool,
    /// Scale along an axis, or evenly.
    pub scale: bool,
}

/// A gizmo an operation asks for (see [`super::Form::gizmo`]): at `at`,
/// doing what `modes` allow — oriented along the world's axes, or `axes`,
/// what it stands on's own, orthonormal and right-handed, if it has an
/// orientation of its own: then the user chooses which.
#[derive(Clone, Debug, PartialEq)]
pub struct Gizmo<S: Scalar> {
    pub at: Vector3<S>,
    pub axes: Option<[Vector3<S>; 3]>,
    pub modes: Modes,
}

impl<S: Scalar> Gizmo<S> {
    /// A gizmo at `at` that does nothing yet: see [`Gizmo::translate`],
    /// [`Gizmo::rotate`] and [`Gizmo::scale`].
    pub fn new(at: Vector3<S>) -> Self {
        Self {
            at,
            axes: None,
            modes: Modes::default(),
        }
    }

    pub fn translate(mut self) -> Self {
        self.modes.translate = true;
        self
    }

    pub fn rotate(mut self) -> Self {
        self.modes.rotate = true;
        self
    }

    pub fn scale(mut self) -> Self {
        self.modes.scale = true;
        self
    }

    /// Oriented along `axes` too, if the user chooses: orthonormal and
    /// right-handed.
    pub fn local(mut self, axes: [Vector3<S>; 3]) -> Self {
        self.axes = Some(axes);
        self
    }
}

/// Which way a gizmo is turned: along the world's axes, or its own.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Orientation {
    World,
    /// Along what it stands on's own axes, where it has them.
    #[default]
    Local,
}

/// A part of a gizmo, to hover and drag: by the index of its axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(tag = "part", content = "axis", rename_all = "snake_case")]
pub enum GizmoPart {
    /// The ball at the centre: moving freely, in the plane facing the eye.
    Free,
    /// An arrow: moving along its axis.
    Move(usize),
    /// The square in the plane normal to an axis: moving in that plane.
    Plane(usize),
    /// A ring: turning about its axis.
    Turn(usize),
    /// A cube beyond an arrow: scaling along its axis.
    Stretch(usize),
    /// The cube on the diagonal: scaling evenly.
    Scale,
}

/// What a drag of a gizmo did, from where it started.
#[derive(Clone, Debug, PartialEq)]
pub enum Change<S: Scalar> {
    /// Moved by this much, in world coordinates.
    Translate(Vector3<S>),
    /// Turned by `degrees` about the line through the centre along the
    /// unit vector `axis` — counter-clockwise, looking against it.
    Rotate { axis: Vector3<S>, degrees: f64 },
    /// Scaled about the centre by `factors[i]` along the unit vector
    /// `axes[i]`.
    Scale {
        axes: [Vector3<S>; 3],
        factors: [f64; 3],
    },
}

/// What a drag of a gizmo did, from where it started: a [`Change`] about
/// `centre`, where the gizmo stood when the drag started.
#[derive(Clone, Debug, PartialEq)]
pub struct GizmoDrag<S: Scalar> {
    pub centre: Vector3<S>,
    pub change: Change<S>,
}

impl<S: Scalar> GizmoDrag<S> {
    /// Where the change takes the point `p`.
    pub fn apply(&self, p: &Vector3<S>) -> Vector3<S> {
        let c = &self.centre;
        match &self.change {
            Change::Translate(by) => p.add(by),
            Change::Rotate { .. } => match self.turn() {
                Some(pose) => pose.apply(p),
                None => *p,
            },
            Change::Scale { axes, factors } => {
                let d = p.sub(c);
                axes.iter().zip(factors).fold(*c, |at, (axis, &factor)| {
                    at.add(&axis.prod_scalar(d.prod_dot(axis).mul(S::from_f64(factor))))
                })
            }
        }
    }

    /// Where the change takes the point `p`, given as plain coordinates.
    pub fn apply_f64(&self, p: [f64; 3]) -> [f64; 3] {
        let moved = self.apply(&Vector3::from_array(p.map(S::from_f64)));
        [0, 1, 2].map(|k| moved[k].to_f64())
    }

    /// The change as a rigid motion: a pose taking where something was to
    /// where it goes — `None` for a scale, which is none.
    pub fn turn(&self) -> Option<Pose<S>> {
        match &self.change {
            Change::Translate(by) => Some(Pose::identity().with_position(*by)),
            Change::Rotate { axis, degrees } => {
                let radians = S::from_f64(degrees.to_radians());
                Pose::rotation_about(&self.centre, axis, radians).ok()
            }
            Change::Scale { .. } => None,
        }
    }
}

/// A gizmo as the viewer draws it: where, along which axes, what it lets
/// the user do, which part the pointer is over and which is dragged, and
/// what a drag under way has done so far, in words.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(bound = "S: Scalar")]
pub struct GizmoView<S: Scalar> {
    pub at: Vector3<S>,
    /// Unit and orthogonal: the world's, or what it stands on's own.
    pub axes: [Vector3<S>; 3],
    pub modes: Modes,
    pub hover: Option<GizmoPart>,
    pub active: Option<GizmoPart>,
    pub readout: Option<String>,
}

/// The world's axes.
fn world<S: Scalar>() -> [Vector3<S>; 3] {
    [0, 1, 2].map(Vector3::axis)
}

impl<S: Scalar> GizmoView<S> {
    /// `gizmo` turned as `orientation` says.
    pub fn of(gizmo: &Gizmo<S>, orientation: Orientation) -> Self {
        let axes = match (orientation, gizmo.axes) {
            (Orientation::Local, Some(axes)) => axes,
            _ => world(),
        };
        Self {
            at: gizmo.at,
            axes,
            modes: gizmo.modes,
            hover: None,
            active: None,
            readout: None,
        }
    }

    /// The parts it shows.
    fn parts(&self) -> Vec<GizmoPart> {
        let mut parts = Vec::new();
        let m = self.modes;
        if m.translate {
            parts.push(GizmoPart::Free);
            parts.extend((0..3).map(GizmoPart::Move));
            parts.extend((0..3).map(GizmoPart::Plane));
        }
        if m.rotate {
            parts.extend((0..3).map(GizmoPart::Turn));
        }
        if m.scale {
            parts.extend((0..3).map(GizmoPart::Stretch));
            parts.push(GizmoPart::Scale);
        }
        parts
    }

    /// The diagonal the even scale's cube sits on.
    fn diagonal(&self) -> Option<Vector3<S>> {
        let [a, b, c] = &self.axes;
        a.add(b).add(c).normalize().ok()
    }

    /// Whether `part` is shown looking along `dir` (see [`size::FACING`]).
    pub fn shown(&self, part: GizmoPart, dir: &Vector3<S>) -> bool {
        let across = |a: &Vector3<S>| a.prod_cross(dir).norm().to_f64() >= size::FACING;
        match part {
            GizmoPart::Free | GizmoPart::Turn(_) => true,
            GizmoPart::Move(i) | GizmoPart::Stretch(i) => across(&self.axes[i]),
            GizmoPart::Plane(i) => self.axes[i].prod_dot(dir).abs().to_f64() >= size::FACING,
            GizmoPart::Scale => self.diagonal().is_some_and(|d| across(&d)),
        }
    }

    /// How large one reach is at its centre, for `pointer`.
    pub fn reach(&self, pointer: &Pointer<S>) -> S {
        pointer.reach_at(1.0, pointer.ray.closest_to_point(&self.at))
    }

    /// The part `pointer` is over: of the cubes and the ball, then the
    /// arrows and squares, then the rings, the one nearest the pointer on
    /// screen.
    pub fn hit(&self, pointer: &Pointer<S>) -> Option<GizmoPart> {
        let ray = &pointer.ray;
        let r = self.reach(pointer);
        let out =
            |d: &Vector3<S>, reaches: f64| self.at.add(&d.prod_scalar(r.mul(S::from_f64(reaches))));
        let near = |dist: S, reaches: f64| {
            (!dist.definitely_greater(r.mul(S::from_f64(reaches))))
                .then(|| dist.div(r).unwrap_or(S::ZERO))
        };
        self.parts()
            .into_iter()
            .filter(|&part| self.shown(part, ray.dir()))
            .filter_map(|part| {
                let (rank, dist) = match part {
                    GizmoPart::Free => (0, near(ray.distance_to_point(&self.at).0, size::FREE)?),
                    GizmoPart::Stretch(i) => {
                        let at = out(&self.axes[i], size::CUBE_AT);
                        (
                            0,
                            near(ray.distance_to_point(&at).0, size::CUBE + size::NEAR)?,
                        )
                    }
                    GizmoPart::Scale => {
                        let at = out(&self.diagonal()?, size::CUBE_AT);
                        (
                            0,
                            near(ray.distance_to_point(&at).0, size::CUBE + size::NEAR)?,
                        )
                    }
                    GizmoPart::Move(i) => {
                        let (a, b) = size::ARROW;
                        let d = &self.axes[i];
                        let dist = ray.distance_to_segment(&out(d, a), &out(d, b)).0;
                        (1, near(dist, size::NEAR)?)
                    }
                    GizmoPart::Plane(i) => {
                        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
                        let (lo, hi) = size::PLANE;
                        let corner = |x: f64, y: f64| {
                            out(&self.axes[j], x)
                                .add(&self.axes[k].prod_scalar(r.mul(S::from_f64(y))))
                        };
                        let [p, q, s, t] =
                            [(lo, lo), (hi, lo), (hi, hi), (lo, hi)].map(|(x, y)| corner(x, y));
                        let inside = ray.intersect_triangle(&p, &q, &s).is_some()
                            || ray.intersect_triangle(&p, &s, &t).is_some();
                        (1, inside.then_some(S::ZERO)?)
                    }
                    GizmoPart::Turn(i) => {
                        let ring = self.ring(i, r);
                        let dist = ring
                            .windows(2)
                            .map(|w| ray.distance_to_segment(&w[0], &w[1]).0)
                            .min_by(|&a, &b| nearer(a, b))?;
                        (2, near(dist, size::NEAR)?)
                    }
                };
                Some((rank, dist, part))
            })
            .min_by(|a, b| a.0.cmp(&b.0).then(nearer(a.1, b.1)))
            .map(|(_, _, part)| part)
    }

    /// The ring about axis `i`, as a closed polyline, a reach being `r`:
    /// as the viewer draws it, so it is hit where it is seen.
    fn ring(&self, i: usize, r: S) -> Vec<Vector3<S>> {
        const SEGMENTS: usize = 64;
        let (u, v) = (&self.axes[(i + 1) % 3], &self.axes[(i + 2) % 3]);
        let radius = r.mul(S::from_f64(size::RING));
        (0..=SEGMENTS)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / SEGMENTS as f64;
                let (s, c) = a.sin_cos();
                self.at
                    .add(&u.prod_scalar(radius.mul(S::from_f64(c))))
                    .add(&v.prod_scalar(radius.mul(S::from_f64(s))))
            })
            .collect()
    }
}

/// `value` snapped to `step` — a free choice of where the pointer is,
/// made for the user.
fn snap(value: f64, step: f64) -> f64 {
    (value / step).round() * step
}

/// The grid a translation snaps to, a reach being `reach`: a power of
/// ten, at least [`GRID`] reaches.
pub fn grid_step(reach: f64) -> f64 {
    10f64.powf((GRID * reach).log10().ceil())
}

/// `value` written to as many decimals as `step` has.
fn write(value: f64, step: f64) -> String {
    let decimals = (-step.log10()).ceil().max(0.0) as usize;
    let value = if value == 0.0 { 0.0 } else { value };
    format!("{value:.decimals$}")
}

const AXIS_NAMES: [&str; 3] = ["x", "y", "z"];

/// A drag of a gizmo under way: the gizmo as it was when the part was
/// pressed — what every event of the drag is measured against — the part,
/// how large a reach was there, and how far a turn has gone, to follow it
/// past half a turn.
#[derive(Clone, Debug)]
pub struct GizmoGrab<S: Scalar> {
    pub view: GizmoView<S>,
    pub part: GizmoPart,
    reach: S,
    turned: f64,
}

impl<S: Scalar> GizmoGrab<S> {
    /// Grabbing `part` of `view` with `pointer`.
    pub fn new(view: GizmoView<S>, part: GizmoPart, pointer: &Pointer<S>) -> Self {
        let reach = view.reach(pointer);
        Self {
            view,
            part,
            reach,
            turned: 0.0,
        }
    }

    /// What the drag from `from` to `to` did — snapped unless `free` — and
    /// what to say of it. `None` where the pointer cannot be followed:
    /// looking along an arrow, or along a ring's or a square's plane.
    pub fn drag(
        &mut self,
        from: &Pointer<S>,
        to: &Pointer<S>,
        free: bool,
    ) -> Option<(GizmoDrag<S>, String)> {
        let view = &self.view;
        let c = view.at;
        let axes = view.axes;
        let step = grid_step(self.reach.to_f64());
        let snapped = |v: f64, step: f64| if free { v } else { snap(v, step) };
        let in_plane = |pointer: &Pointer<S>, normal: &Vector3<S>| {
            pointer.ray.intersect_plane(&c, normal).map(|(_, p)| p)
        };
        // A move by `amounts` along the axes `which`.
        let translate = |which: &[usize], amounts: Vec<f64>| {
            let by = which
                .iter()
                .zip(&amounts)
                .fold(Vector3::zero(), |by, (&k, &a)| {
                    by.add(&axes[k].prod_scalar(S::from_f64(a)))
                });
            let text = which
                .iter()
                .zip(&amounts)
                .map(|(&k, &a)| format!("{} {}", AXIS_NAMES[k], write(a, step)))
                .collect::<Vec<_>>()
                .join("  ");
            (Change::Translate(by), text)
        };
        let (change, text) = match self.part {
            GizmoPart::Move(i) => {
                let s = |p: &Pointer<S>| p.ray.line_parameter(&c, &axes[i]);
                let moved = s(to)?.sub(s(from)?).to_f64();
                translate(&[i], vec![snapped(moved, step)])
            }
            GizmoPart::Plane(i) => {
                let d = in_plane(to, &axes[i])?.sub(&in_plane(from, &axes[i])?);
                let which = [(i + 1) % 3, (i + 2) % 3];
                let amounts = which
                    .iter()
                    .map(|&k| snapped(d.prod_dot(&axes[k]).to_f64(), step))
                    .collect();
                translate(&which, amounts)
            }
            GizmoPart::Free => {
                let d = in_plane(to, from.ray.dir())?.sub(&in_plane(from, from.ray.dir())?);
                let amounts = (0..3)
                    .map(|k| snapped(d.prod_dot(&axes[k]).to_f64(), step))
                    .collect();
                translate(&[0, 1, 2], amounts)
            }
            GizmoPart::Turn(i) => {
                let a = &axes[i];
                let v0 = in_plane(from, a)?.sub(&c);
                let v1 = in_plane(to, a)?.sub(&c);
                let sin = v0.prod_cross(&v1).prod_dot(a).to_f64();
                let cos = v0.prod_dot(&v1).to_f64();
                if sin == 0.0 && cos == 0.0 {
                    return None;
                }
                // Followed past half a turn: the angle nearest the last.
                let angle = sin.atan2(cos).to_degrees();
                let turns = ((self.turned - angle) / 360.0).round();
                self.turned = angle + 360.0 * turns;
                let degrees = snapped(self.turned, ANGLE_SNAP);
                let text = format!(
                    "{} {}°",
                    AXIS_NAMES[i],
                    write(degrees, if free { 0.1 } else { 1.0 })
                );
                (Change::Rotate { axis: *a, degrees }, text)
            }
            GizmoPart::Stretch(i) => {
                let factor = self.factor(from, to, &axes[i], free)?;
                let mut factors = [1.0; 3];
                factors[i] = factor;
                let text = format!("{} ×{}", AXIS_NAMES[i], write(factor, SCALE_SNAP / 10.0));
                (Change::Scale { axes, factors }, text)
            }
            GizmoPart::Scale => {
                let factor = self.factor(from, to, &view.diagonal()?, free)?;
                let text = format!("×{}", write(factor, SCALE_SNAP / 10.0));
                (
                    Change::Scale {
                        axes,
                        factors: [factor; 3],
                    },
                    text,
                )
            }
        };
        Some((GizmoDrag { centre: c, change }, text))
    }

    /// How far a cube along `direction` was pulled out, as a factor: how
    /// far out it is now over how far it was — snapped unless `free`, and
    /// never zero or less, which would flatten or turn inside out.
    fn factor(
        &self,
        from: &Pointer<S>,
        to: &Pointer<S>,
        direction: &Vector3<S>,
        free: bool,
    ) -> Option<f64> {
        let c = &self.view.at;
        let s0 = from.ray.line_parameter(c, direction)?.to_f64();
        let s1 = to.ray.line_parameter(c, direction)?.to_f64();
        let factor = s1 / s0;
        if !factor.is_finite() {
            return None;
        }
        let factor = if free {
            factor
        } else {
            snap(factor, SCALE_SNAP).max(SCALE_SNAP)
        };
        (factor > 0.0).then_some(factor)
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{primitives::Ray, scalars::ScalInF64, vector::Vector3};

    use super::*;
    use crate::ui::Reach;

    type S = ScalInF64;

    fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    /// From `origin` along `dir`, a reach being a tenth of a unit.
    fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
        Pointer {
            ray: Ray::try_new(
                v(origin[0], origin[1], origin[2]),
                v(dir[0], dir[1], dir[2]),
            )
            .unwrap(),
            reach: Reach::Tube {
                radius: S::from_f64(0.1),
            },
        }
    }

    /// Looking straight down `-z` at `(x, y)`.
    fn down(x: f64, y: f64) -> Pointer<S> {
        pointer([x, y, 10.0], [0.0, 0.0, -1.0])
    }

    fn every() -> GizmoView<S> {
        let gizmo = Gizmo::new(v(0.0, 0.0, 0.0)).translate().rotate().scale();
        GizmoView::of(&gizmo, Orientation::World)
    }

    /// Seen from above, each part is hit where it is laid out: the ball,
    /// the arrows, the squares, the rings, the cubes.
    #[test]
    fn parts_are_hit_where_they_are_laid_out() {
        let gizmo = every();
        let hit = |x, y| gizmo.hit(&down(x, y));
        assert_eq!(hit(0.0, 0.0), Some(GizmoPart::Free));
        assert_eq!(hit(0.6, 0.0), Some(GizmoPart::Move(0)));
        assert_eq!(hit(0.0, 0.9), Some(GizmoPart::Move(1)));
        // The xy plane's square is the one normal to z.
        assert_eq!(hit(0.32, 0.32), Some(GizmoPart::Plane(2)));
        // The ring about z, between the arrows.
        let r = 0.7 / 2f64.sqrt();
        assert_eq!(hit(r, r), Some(GizmoPart::Turn(2)));
        assert_eq!(hit(1.25, 0.0), Some(GizmoPart::Stretch(0)));
        assert_eq!(hit(3.0, 3.0), None);
        // Translation alone shows no ring and no cube.
        let moving = GizmoView::of(
            &Gizmo::new(v(0.0, 0.0, 0.0)).translate(),
            Orientation::World,
        );
        assert_eq!(moving.hit(&down(r, r)), None);
        assert_eq!(moving.hit(&down(1.25, 0.0)), None);
    }

    /// An arrow dragged moves along its axis only, snapped to the grid —
    /// a power of ten, here a whole unit — unless shift is held.
    #[test]
    fn arrows_move_along_their_axis() {
        let side = |z: f64| pointer([0.0, -10.0, z], [0.0, 1.0, 0.0]);
        let gizmo = every();
        let part = gizmo.hit(&side(0.6)).unwrap();
        assert_eq!(part, GizmoPart::Move(2));
        let mut grab = GizmoGrab::new(gizmo, part, &side(0.6));
        let (drag, text) = grab.drag(&side(0.6), &side(2.0), false).unwrap();
        let Change::Translate(by) = drag.change else {
            panic!("{drag:?}")
        };
        assert_eq!([0, 1, 2].map(|k| by[k].to_f64()), [0.0, 0.0, 1.0]);
        assert_eq!(text, "z 1");
        let (drag, _) = grab.drag(&side(0.6), &side(2.0), true).unwrap();
        let Change::Translate(by) = drag.change else {
            panic!("{drag:?}")
        };
        assert!((by[2].to_f64() - 1.4).abs() < 1e-12);
    }

    /// A ring dragged a quarter round turns by 90 degrees, followed past
    /// half a turn, and snapped to 15 degrees.
    #[test]
    fn rings_turn_about_their_axis() {
        let gizmo = every();
        let at = |a: f64| {
            let a = a.to_radians();
            down(0.7 * a.cos(), 0.7 * a.sin())
        };
        let mut grab = GizmoGrab::new(gizmo, GizmoPart::Turn(2), &at(0.0));
        let degrees = |grab: &mut GizmoGrab<S>, a: f64| match grab.drag(&at(0.0), &at(a), false) {
            Some((
                GizmoDrag {
                    change: Change::Rotate { degrees, .. },
                    ..
                },
                _,
            )) => degrees,
            other => panic!("{other:?}"),
        };
        assert_eq!(degrees(&mut grab, 92.0), 90.0);
        assert_eq!(degrees(&mut grab, 170.0), 165.0);
        // Past half a turn: on, not back.
        assert_eq!(degrees(&mut grab, 200.0), 195.0);
        let (drag, text) = grab.drag(&at(0.0), &at(90.0), false).unwrap();
        assert_eq!(text, "z 90°");
        let turned = drag.apply_f64([1.0, 0.0, 0.0]);
        assert!(
            turned[0].abs() < 1e-12 && (turned[1] - 1.0).abs() < 1e-12,
            "{turned:?}"
        );
    }

    /// A cube pulled out to twice as far scales by two along its axis;
    /// the diagonal's, evenly.
    #[test]
    fn cubes_scale() {
        let gizmo = every();
        let mut grab = GizmoGrab::new(gizmo.clone(), GizmoPart::Stretch(0), &down(1.25, 0.0));
        let (drag, text) = grab.drag(&down(1.25, 0.0), &down(2.5, 0.0), false).unwrap();
        assert_eq!(text, "x ×2.00");
        let stretched = drag.apply_f64([1.0, 1.0, 1.0]);
        let want = [2.0, 1.0, 1.0];
        assert!(
            (0..3).all(|k| (stretched[k] - want[k]).abs() < 1e-12),
            "{stretched:?}"
        );
        let diagonal = |s: f64| {
            let d = s / 3f64.sqrt();
            pointer([d - 10.0, d + 10.0, d], [1.0, -1.0, 0.0])
        };
        let part = gizmo.hit(&diagonal(1.25)).unwrap();
        assert_eq!(part, GizmoPart::Scale);
        let mut grab = GizmoGrab::new(gizmo, part, &diagonal(1.25));
        let (drag, _) = grab.drag(&diagonal(1.25), &diagonal(0.625), false).unwrap();
        let half = drag.apply_f64([1.0, 1.0, 1.0]);
        assert!(half.iter().all(|c| (c - 0.5).abs() < 1e-12), "{half:?}");
    }

    /// Turned to its own axes, the gizmo moves along them.
    #[test]
    fn local_axes_are_followed() {
        let s = 0.5f64.sqrt();
        let gizmo = Gizmo::new(v(0.0, 0.0, 0.0)).translate().local([
            v(s, s, 0.0),
            v(-s, s, 0.0),
            v(0.0, 0.0, 1.0),
        ]);
        let local = GizmoView::of(&gizmo, Orientation::Local);
        assert_eq!(local.hit(&down(0.5 * s, 0.5 * s)), Some(GizmoPart::Move(0)));
        let world = GizmoView::of(&gizmo, Orientation::World);
        assert_eq!(world.hit(&down(0.5, 0.0)), Some(GizmoPart::Move(0)));
        let mut grab = GizmoGrab::new(local, GizmoPart::Plane(2), &down(0.4 * s, 0.0));
        let (drag, _) = grab
            .drag(&down(0.0, 0.0), &down(2.0 * s, 2.0 * s), false)
            .unwrap();
        let moved = drag.apply_f64([0.0, 0.0, 0.0]);
        assert!((moved[0] - 2.0 * s).abs() < 1e-12 && (moved[1] - 2.0 * s).abs() < 1e-12);
    }
}
