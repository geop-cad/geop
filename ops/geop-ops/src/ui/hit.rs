//! Hit tests: which [`Visual`] a [`Pointer`] is over, shared by every
//! operation so a click means the same thing whatever is being edited.
//!
//! What counts as over something is measured in the pointer's reach (see
//! [`super::Reach`]), so "near" means near on screen however far away or
//! zoomed in the view is. The geometry itself — how near a ray passes to a
//! point, a segment, a triangle — is [`geop_core_math::primitives::Ray`]'s.

use std::cmp::Ordering;

use geop_core_math::scalars::Scalar;

use super::{Pointer, Shape, Visual};

/// A handle's radius, in reaches; its arrows reach three times as far along
/// its direction.
pub const HANDLE: f64 = 0.8;
/// How near a label's center a click has to land, in reaches.
pub const LABEL: f64 = 1.2;

/// Orders two distances for picking the nearest. Which of two overlapping
/// enclosures is "nearer" is no geometric claim, only which of two things
/// under the pointer to prefer, so their midpoints decide.
pub fn nearer<S: Scalar>(a: S, b: S) -> Ordering {
    a.to_f64().total_cmp(&b.to_f64())
}

/// A visual the pointer is over.
#[derive(Clone, Copy, Debug)]
pub struct VisualHit<'a, S: Scalar> {
    pub visual: &'a Visual<S>,
    /// How far along the ray.
    pub t: S,
}

/// How near the pointer is to `visual`, in reaches, and where along the
/// ray — `None` if not near enough to count. `rank` orders kinds of shapes:
/// what is drawn small and on top wins over what is drawn large.
fn distance<S: Scalar>(pointer: &Pointer<S>, visual: &Visual<S>) -> Option<(u8, S, S)> {
    let ray = &pointer.ray;
    let within = |dist: S, t: S, reaches: f64| {
        pointer
            .within(dist, t, reaches)
            .then(|| (dist.div(pointer.reach.at(t)).unwrap_or(S::ZERO), t))
    };
    match &visual.shape {
        Shape::Handle { at, direction } => {
            let radius = pointer.reach_at(HANDLE, ray.closest_to_point(at));
            let (dist, t) = match direction.and_then(|d| d.normalize().ok()) {
                Some(d) => {
                    let reach = d.prod_scalar(radius.mul(S::from_f64(3.0)));
                    ray.distance_to_segment(&at.sub(&reach), &at.add(&reach))
                }
                None => ray.distance_to_point(at),
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
            .filter_map(|w| {
                let (dist, t) = ray.distance_to_segment(&w[0], &w[1]);
                within(dist, t, 1.0)
            })
            .min_by(|a, b| nearer(a.0, b.0))
            .map(|(p, t)| (2, p, t)),
        Shape::Triangles { triangles } => triangles
            .iter()
            .filter_map(|[a, b, c]| ray.intersect_triangle(a, b, c))
            .min_by(|&a, &b| nearer(a, b))
            .map(|t| (3, S::ZERO, t)),
    }
}

/// The visual among those `accept` takes that the pointer is over: handles
/// and labels first, then points, curves and areas — within each, the one
/// nearest the pointer on screen.
pub fn hit_visuals<'a, S: Scalar>(
    visuals: &'a [Visual<S>],
    pointer: &Pointer<S>,
    accept: impl Fn(&Visual<S>) -> bool,
) -> Option<VisualHit<'a, S>> {
    visuals
        .iter()
        .filter(|v| accept(v))
        .filter_map(|v| distance(pointer, v).map(|(rank, reaches, t)| (rank, reaches, t, v)))
        .min_by(|a, b| a.0.cmp(&b.0).then(nearer(a.1, b.1)))
        .map(|(_, _, t, visual)| VisualHit { visual, t })
}

#[cfg(test)]
mod tests {
    use geop_core_math::{primitives::Ray, scalars::ScalInF64, vector::Vector3};

    use super::*;
    use crate::ui::{Reach, Style};

    type S = ScalInF64;

    fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
        Vector3::from_array([x, y, z].map(S::from_f64))
    }

    /// Looking straight down `-z` from `(x, y, 10)`, reaching a tenth of a
    /// unit wherever it looks.
    fn down(x: f64, y: f64) -> Pointer<S> {
        Pointer {
            ray: Ray::try_new(v(x, y, 10.0), v(0.0, 0.0, -1.0)).unwrap(),
            reach: Reach::Tube {
                radius: S::from_f64(0.1),
            },
        }
    }

    fn point(key: &str, at: Vector3<S>) -> Visual<S> {
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
            hit_visuals(&visuals, &down(x, y), |_| true).map(|h| h.visual.key.clone())
        };
        assert_eq!(key(0.95, 0.0).as_deref(), Some("end"));
        assert_eq!(key(0.5, 0.05).as_deref(), Some("line"));
        assert_eq!(key(0.5, 0.2), None);
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
        assert!(hit_visuals(&visuals, &down(0.2, 0.1), |_| true).is_some());
        assert!(hit_visuals(&visuals, &down(0.0, 0.0), |_| true).is_none());
    }

    /// In perspective, what is further away is hit from further off.
    #[test]
    fn cones_widen_with_distance() {
        let visuals = [point("p", v(0.3, 0.0, 0.0))];
        let from = |z: f64| Pointer {
            ray: Ray::try_new(v(0.0, 0.0, z), v(0.0, 0.0, -1.0)).unwrap(),
            reach: Reach::Cone {
                slope: S::from_f64(0.01),
            },
        };
        assert!(hit_visuals(&visuals, &from(10.0), |_| true).is_none());
        assert!(hit_visuals(&visuals, &from(40.0), |_| true).is_some());
    }
}
