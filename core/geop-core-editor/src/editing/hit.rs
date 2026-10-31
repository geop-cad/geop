//! Hit tests: which [`Visual`] a [`Pointer`] is over, shared by every
//! operation so a click means the same thing whatever is being edited.
//!
//! What counts as over something is measured in the pointer's reach (see
//! [`super::Reach`]), so "near" means near on screen however far away or
//! zoomed in the view is. The geometry itself — how near a ray passes to a
//! point, a segment, a triangle — is [`geop_core_math::primitives::Ray`]'s.

use std::cmp::Ordering;

use geop_core_math::{
    plain::{add, cross, dot, normalize, scale, sub},
    scalars::Scalar,
    vector::Vector3,
};

use serde::Serialize;

use super::{Pointer, Reach, Shape, Style, Visual};
use geop_core_program::element::PickedElement;

/// A handle's radius, in reaches; its arrows reach three times as far along
/// its direction.
pub const HANDLE: f64 = 0.8;
/// How near a label's center a click has to land, in reaches.
pub const LABEL: f64 = 1.2;

/// Orders two distances for picking the nearest. Which of two overlapping
/// enclosures is "nearer" is no geometric claim, only which of two things
/// under the pointer to prefer, so their midpoints decide.
pub fn nearer(a: Scalar, b: Scalar) -> Ordering {
    a.to_f64().total_cmp(&b.to_f64())
}

/// The part as an editor draws it, which picks are made in: what an editor
/// needs to know of it beyond a step's own visuals, and what an operation
/// is given to find what the pointer is over (see
/// [`crate::editable::Context::scene`]).
pub trait Scene {
    /// Every element of the kinds `accepts` under `pointer`, each once: the
    /// one a click picks first — the smallest, in front, nearest the eye
    /// (see [`crate::view::PartView::under`]) — then the rest front to back
    /// along the ray, those hidden behind the first included. Its
    /// [`Pointer::cycle`] is not looked at.
    fn under(&self, pointer: &Pointer, accepts: &[String]) -> Vec<ElementHit>;

    /// What a click with `pointer` picks: of what lies [`Scene::under`]
    /// it, the one its [`Pointer::cycle`] names, and where it is hit.
    fn pick(&self, pointer: &Pointer, accepts: &[String]) -> Option<ElementHit> {
        cycled(self.under(pointer, accepts), pointer.cycle)
    }

    /// How far along `pointer`'s ray the placed part `name` is hit, if it
    /// is (see [`Shape::Instance`]).
    fn hit_instance(&self, name: &str, pointer: &Pointer) -> Option<Scalar>;
}

/// The one of `candidates` a pointer's [`Pointer::cycle`] `cycle` names:
/// counted from the first, wrapping round. None of none.
pub fn cycled<T>(candidates: Vec<T>, cycle: usize) -> Option<T> {
    let count = candidates.len();
    (count > 0)
        .then(|| candidates.into_iter().nth(cycle % count))
        .flatten()
}

/// Which of the candidates under the pointer is lit: the `at`-th of `of`,
/// front to back, counted from 0 — what a front end says while its user
/// steps through them (see [`Pointer::cycle`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Cycle {
    pub at: usize,
    pub of: usize,
}

impl Cycle {
    /// Of `of` candidates, the one `pointer` names: none of none.
    pub fn of(of: usize, pointer: &Pointer) -> Option<Self> {
        (of > 0).then(|| Cycle {
            at: pointer.cycle % of,
            of,
        })
    }
}

/// An element of the part a pointer is over, as drawn: where, and how far
/// along its ray (see [`Scene::pick`]).
#[derive(Clone, Debug, PartialEq)]
pub struct ElementHit {
    pub element: PickedElement,
    pub point: Vector3,
    pub t: Scalar,
}

/// A visual the pointer is over.
#[derive(Clone, Copy, Debug)]
pub struct VisualHit<'a> {
    pub visual: &'a Visual,
    /// How far along the ray.
    pub t: Scalar,
}

/// How near the pointer is to `visual`, in reaches, and where along the
/// ray — `None` if not near enough to count. `rank` orders kinds of shapes:
/// what is drawn small and on top wins over what is drawn large, and a
/// curve that is part of the result over construction or reference geometry
/// it lies on — a line drawn along a sketch's axis is that line. A placed part is
/// hit where `instance` says, by its name: how far along the ray, if at all.
fn distance(
    pointer: &Pointer,
    visual: &Visual,
    instance: &impl Fn(&str, &Pointer) -> Option<Scalar>,
) -> Option<(u8, Scalar, Scalar)> {
    let ray = &pointer.ray;
    let within = |dist: Scalar, t: Scalar, reaches: f64| {
        pointer
            .within(dist, t, reaches)
            .then(|| (dist.div(pointer.reach.at(t)).unwrap_or(Scalar::ZERO), t))
    };
    match &visual.shape {
        Shape::Handle { at, direction } => {
            let radius = pointer.reach_at(HANDLE, ray.closest_to_point(at));
            let (dist, t) = match direction.normalize() {
                Ok(d) => {
                    let reach = d.prod_scalar(radius.mul(Scalar::from_f64(3.0)));
                    ray.distance_to_segment(&at.sub(&reach), &at.add(&reach))
                }
                Err(_) => ray.distance_to_point(at),
            };
            within(dist, t, HANDLE).map(|(p, t)| (0, p, t))
        }
        Shape::Label { at, offset, .. } => {
            let center = at.add(&offset.prod_scalar(pointer.reach.at(ray.closest_to_point(at))));
            let (dist, t) = ray.distance_to_point(&center);
            within(dist, t, LABEL).map(|(p, t)| (0, p, t))
        }
        Shape::Point { at } => {
            let (dist, t) = ray.distance_to_point(at);
            within(dist, t, 1.0).map(|(p, t)| (1, p, t))
        }
        Shape::Polyline { points } => points
            .windows(2)
            .filter_map(|w| match w {
                [a, b] => {
                    let (dist, t) = ray.distance_to_segment(a, b);
                    within(dist, t, 1.0)
                }
                _ => None,
            })
            .min_by(|a, b| nearer(a.0, b.0))
            .map(|(p, t)| {
                let rank = if matches!(visual.style, Style::Construction | Style::Reference) {
                    3
                } else {
                    2
                };
                (rank, p, t)
            }),
        Shape::Triangles { triangles } => triangles
            .iter()
            .filter_map(|[a, b, c]| ray.intersect_triangle(a, b, c))
            .min_by(|&a, &b| nearer(a, b))
            .map(|t| (4, Scalar::ZERO, t)),
        Shape::Instance { name } => instance(name, pointer).map(|t| (4, Scalar::ZERO, t)),
        Shape::Triad { .. } => None,
    }
}

/// The visuals among those `accept` takes that the pointer is over: first
/// the one a click takes — handles and labels first, then points, curves
/// and areas; within each, the one nearest the pointer on screen, and of
/// those as near, the one nearest the eye: of two areas the pointer is
/// over, the one in front — then the rest front to back along the ray.
/// Placed parts are hit where `instance` says (see [`no_instances`]).
pub fn visuals_under<'a>(
    visuals: &'a [Visual],
    pointer: &Pointer,
    instance: impl Fn(&str, &Pointer) -> Option<Scalar>,
    accept: impl Fn(&Visual) -> bool,
) -> Vec<VisualHit<'a>> {
    let mut hits: Vec<_> = (visuals.iter())
        .filter(|v| accept(v))
        .filter_map(|v| {
            distance(pointer, v, &instance).map(|(rank, reaches, t)| (rank, reaches, t, v))
        })
        .collect();
    let first = (0..hits.len()).min_by(|&a, &b| {
        let (a, b) = (&hits[a], &hits[b]);
        a.0.cmp(&b.0).then(nearer(a.1, b.1)).then(nearer(a.2, b.2))
    });
    let first = first.map(|at| hits.remove(at));
    hits.sort_by(|a, b| nearer(a.2, b.2));
    (first.into_iter().chain(hits))
        .map(|(_, _, t, visual)| VisualHit { visual, t })
        .collect()
}

/// The visual among those `accept` takes that the pointer is over: of
/// those [`visuals_under`] it, the one its [`Pointer::cycle`] names.
pub fn hit_visuals<'a>(
    visuals: &'a [Visual],
    pointer: &Pointer,
    instance: impl Fn(&str, &Pointer) -> Option<Scalar>,
    accept: impl Fn(&Visual) -> bool,
) -> Option<VisualHit<'a>> {
    cycled(
        visuals_under(visuals, pointer, instance, accept),
        pointer.cycle,
    )
}

/// For [`hit_visuals`] where no placed part is drawn: none is hit.
pub fn no_instances(_: &str, _: &Pointer) -> Option<Scalar> {
    None
}

/// The window a drag from one pointer to another sweeps over the view: the
/// rectangle between the two on the plane facing the eye, for selecting
/// everything within it. Its sides run along the world's axes as the view
/// shows them — the two least along the line of sight — which are the
/// screen's own in a view along an axis. Seen in perspective it is the
/// frustum through that rectangle.
#[derive(Clone, Debug)]
pub struct Window {
    /// The eye, or where the first pointer's ray starts.
    origin: [f64; 3],
    /// The line of sight, unit.
    sight: [f64; 3],
    /// The window's two sides, unit and square to the line of sight.
    sides: [[f64; 3]; 2],
    /// Whether the view is in perspective: rays from one eye.
    perspective: bool,
    /// The window's corners in its sides' coordinates, low and high.
    low: [f64; 2],
    high: [f64; 2],
}

impl Window {
    /// The window swept from `from` to `to`.
    pub fn new(from: &Pointer, to: &Pointer) -> Option<Self> {
        let sight = normalize(from.ray.dir().to_f64_array())?;
        // The axis least along the line of sight, made square to it.
        let axis = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
            .into_iter()
            .min_by(|a, b| dot(*a, sight).abs().total_cmp(&dot(*b, sight).abs()))?;
        let v = normalize(sub(axis, scale(sight, dot(axis, sight))))?;
        let u = cross(v, sight);
        let perspective = matches!(from.reach, Reach::Cone { .. });
        let mut window = Self {
            origin: from.ray.origin().to_f64_array(),
            sight,
            sides: [u, v],
            perspective,
            low: [0.0; 2],
            high: [0.0; 2],
        };
        let [x, y] = match perspective {
            true => window.screen(&to.ray.at(Scalar::ONE).to_f64_array())?,
            false => window.screen(&to.ray.origin().to_f64_array())?,
        };
        window.low = [x.min(0.0), y.min(0.0)];
        window.high = [x.max(0.0), y.max(0.0)];
        Some(window)
    }

    /// Where `p` is on the screen, in the window's sides: `None` behind the
    /// eye.
    fn screen(&self, p: &[f64; 3]) -> Option<[f64; 2]> {
        let r = sub(*p, self.origin);
        let [u, v] = self.sides;
        let (x, y) = (dot(r, u), dot(r, v));
        if !self.perspective {
            return Some([x, y]);
        }
        let depth = dot(r, self.sight);
        (depth > 0.0).then(|| [x / depth, y / depth])
    }

    /// Whether `p` is seen within the window.
    pub fn contains(&self, p: &Vector3) -> bool {
        let [low_x, low_y] = self.low;
        let [high_x, high_y] = self.high;
        self.screen(&p.to_f64_array())
            .is_some_and(|[x, y]| low_x <= x && x <= high_x && low_y <= y && y <= high_y)
    }

    /// How far `p` is along the line of sight.
    pub fn depth(&self, p: &Vector3) -> f64 {
        dot(sub(p.to_f64_array(), self.origin), self.sight)
    }

    /// The window's corners, around, `depth` along the line of sight: to
    /// draw it there.
    pub fn corners(&self, depth: f64) -> [Vector3; 4] {
        let [u, v] = self.sides;
        let size = if self.perspective { depth } else { 1.0 };
        let centre = add(self.origin, scale(self.sight, depth));
        let at = |x: f64, y: f64| {
            let across = add(scale(u, x), scale(v, y));
            Vector3::from_array(add(centre, scale(across, size)).map(Scalar::from_f64))
        };
        let [low_x, low_y] = self.low;
        let [high_x, high_y] = self.high;
        [
            at(low_x, low_y),
            at(high_x, low_y),
            at(high_x, high_y),
            at(low_x, high_y),
        ]
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{primitives::Ray, scalars::Scalar, vector::Vector3};

    use super::*;
    use crate::editing::Reach;

    fn v(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    /// Looking straight down `-z` from `(x, y, 10)`, reaching a tenth of a
    /// unit wherever it looks.
    fn down(x: f64, y: f64) -> Pointer {
        Pointer {
            cycle: 0,
            ray: Ray::try_new(v(x, y, 10.0), v(0.0, 0.0, -1.0)).unwrap(),
            reach: Reach::Tube {
                radius: Scalar::from_f64(0.1),
            },
        }
    }

    fn point(key: &str, at: Vector3) -> Visual {
        Visual::new(key, Shape::Point { at }, Style::Free)
    }

    /// A point wins over the curve it lies on, within reach of it; further
    /// along, the curve is hit.
    #[test]
    fn points_win_over_curves() {
        let visuals = [
            Visual::new(
                "line",
                Shape::Polyline {
                    points: vec![v(0.0, 0.0, 0.0), v(1.0, 0.0, 0.0)],
                },
                Style::Free,
            ),
            point("end", v(1.0, 0.0, 0.0)),
        ];
        let key = |x: f64, y: f64| {
            hit_visuals(&visuals, &down(x, y), no_instances, |_| true).map(|h| h.visual.key.clone())
        };
        assert_eq!(key(0.95, 0.0).as_deref(), Some("end"));
        assert_eq!(key(0.5, 0.05).as_deref(), Some("line"));
        assert_eq!(key(0.5, 0.2), None);
    }

    /// Of two areas one behind the other, the one in front is hit, whichever
    /// is listed first.
    #[test]
    fn the_area_in_front_is_hit() {
        let square = |key: &str, z: f64| {
            Visual::new(
                key,
                Shape::Triangles {
                    triangles: vec![
                        [v(-1.0, -1.0, z), v(1.0, -1.0, z), v(1.0, 1.0, z)],
                        [v(-1.0, -1.0, z), v(1.0, 1.0, z), v(-1.0, 1.0, z)],
                    ],
                },
                Style::Region,
            )
        };
        let visuals = [square("below", -1.0), square("above", 1.0)];
        let hit = hit_visuals(&visuals, &down(0.2, 0.3), no_instances, |_| true);
        assert_eq!(hit.map(|h| h.visual.key.as_str()), Some("above"));
    }

    /// A label is hit where it is drawn: moved from its point by its
    /// offset, in reaches.
    #[test]
    fn labels_are_hit_where_they_are_drawn() {
        let visuals = [Visual::new(
            "k1",
            Shape::Label {
                at: v(0.0, 0.0, 0.0),
                text: "H".into(),
                offset: v(2.0, 1.0, 0.0),
            },
            Style::Free,
        )];
        assert!(hit_visuals(&visuals, &down(0.2, 0.1), no_instances, |_| true).is_some());
        assert!(hit_visuals(&visuals, &down(0.0, 0.0), no_instances, |_| true).is_none());
    }

    /// A window dragged looking down from `(0, 0)` to `(2, 1)` takes a
    /// point at `(1, 0.5)` at any depth and not one at `(3, 0.5)`; seen in
    /// perspective from `z = 10` towards `(1, 1, 0)`, the window widens
    /// with depth: a point at `(0.5, 0.5, 0)` is within it, one at
    /// `(0.5, 0.5, 8)`, nearer the eye, is not.
    #[test]
    fn windows_take_what_is_seen_within_them() {
        let window = Window::new(&down(0.0, 0.0), &down(2.0, 1.0)).unwrap();
        assert!(window.contains(&v(1.0, 0.5, -3.0)));
        assert!(window.contains(&v(1.0, 0.5, 7.0)));
        assert!(!window.contains(&v(3.0, 0.5, 0.0)));
        assert!(!window.contains(&v(1.0, -0.5, 0.0)));
        let corners = window.corners(10.0);
        assert!(corners[2].could_be_equal(&v(2.0, 1.0, 0.0)), "{corners:?}");

        let eye = |x: f64, y: f64| Pointer {
            cycle: 0,
            ray: Ray::try_new(v(0.0, 0.0, 10.0), v(x, y, -10.0)).unwrap(),
            reach: Reach::Cone {
                slope: Scalar::from_f64(0.01),
            },
        };
        let window = Window::new(&eye(0.0, 0.0), &eye(1.0, 1.0)).unwrap();
        assert!(window.contains(&v(0.5, 0.5, 0.0)));
        assert!(!window.contains(&v(0.5, 0.5, 8.0)));
    }

    /// In perspective, what is further away is hit from further off.
    #[test]
    fn cones_widen_with_distance() {
        let visuals = [point("p", v(0.3, 0.0, 0.0))];
        let from = |z: f64| Pointer {
            cycle: 0,
            ray: Ray::try_new(v(0.0, 0.0, z), v(0.0, 0.0, -1.0)).unwrap(),
            reach: Reach::Cone {
                slope: Scalar::from_f64(0.01),
            },
        };
        assert!(hit_visuals(&visuals, &from(10.0), no_instances, |_| true).is_none());
        assert!(hit_visuals(&visuals, &from(40.0), no_instances, |_| true).is_some());
    }
}
