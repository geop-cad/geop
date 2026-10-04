//! The directions a part is drawn from, and projecting onto the paper.
//!
//! Parts are modelled with `z` up (a sketch on the `xy` plane extrudes
//! upwards), so the views are the usual ones of a `z`-up modeller: the front
//! view looks along `+y`, the top view down `-z`, the right view along `-x`,
//! and the isometric view from the front, right and top at once.

use geop_core_geometry::nurb_curve::{NurbCurve2D, NurbCurve3D};
use geop_core_math::{
    geop_error::GeopResult,
    scalars::Scalar,
    vector::{Vector2, Vector3},
};
use serde::{Deserialize, Serialize};

/// Which way a view looks at the part.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ViewKind {
    Front,
    Top,
    Right,
    Left,
    Bottom,
    Back,
    /// Isometric, from the front, right and top.
    Iso,
}

impl ViewKind {
    pub const ALL: [ViewKind; 7] = [
        ViewKind::Front,
        ViewKind::Top,
        ViewKind::Right,
        ViewKind::Left,
        ViewKind::Bottom,
        ViewKind::Back,
        ViewKind::Iso,
    ];

    /// Its name, as written in a program and on the command line.
    pub fn name(self) -> &'static str {
        match self {
            ViewKind::Front => "front",
            ViewKind::Top => "top",
            ViewKind::Right => "right",
            ViewKind::Left => "left",
            ViewKind::Bottom => "bottom",
            ViewKind::Back => "back",
            ViewKind::Iso => "iso",
        }
    }

    /// The view named `name`, if there is one.
    pub fn from_name(name: &str) -> Option<ViewKind> {
        ViewKind::ALL.into_iter().find(|k| k.name() == name)
    }

    /// The frame it looks along.
    pub fn frame<S: Scalar>(self) -> GeopResult<ViewFrame<S>> {
        let unit = |index: usize, negative: bool| ViewAxis::Unit { index, negative };
        let (direction, right, up) = match self {
            ViewKind::Front => (unit(1, false), unit(0, false), unit(2, false)),
            ViewKind::Back => (unit(1, true), unit(0, true), unit(2, false)),
            ViewKind::Top => (unit(2, true), unit(0, false), unit(1, false)),
            ViewKind::Bottom => (unit(2, false), unit(0, false), unit(1, true)),
            ViewKind::Right => (unit(0, true), unit(1, false), unit(2, false)),
            ViewKind::Left => (unit(0, false), unit(1, true), unit(2, false)),
            ViewKind::Iso => {
                let v = |x: f64, y: f64, z: f64| {
                    Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
                };
                (
                    ViewAxis::General(v(-1.0, 1.0, -1.0).normalize()?),
                    ViewAxis::General(v(1.0, 1.0, 0.0).normalize()?),
                    ViewAxis::General(v(-1.0, 1.0, 2.0).normalize()?),
                )
            }
        };
        Ok(ViewFrame {
            direction,
            right,
            up,
        })
    }
}

/// A unit vector of a view's frame. A coordinate axis is kept as one, so
/// projecting along it picks a coordinate rather than multiplying by 1 and
/// 0, which would widen every interval by rounding (see `AGENTS.md`,
/// "Construction code is sensitive at the last bit").
#[derive(Clone, Copy, Debug)]
pub enum ViewAxis<S: Scalar> {
    /// The coordinate axis `index`, negated if `negative`.
    Unit { index: usize, negative: bool },
    General(Vector3<S>),
}

impl<S: Scalar> ViewAxis<S> {
    /// `v`'s component along this axis.
    pub fn dot(&self, v: &Vector3<S>) -> S {
        match *self {
            ViewAxis::Unit { index, negative } => {
                if negative {
                    v[index].neg()
                } else {
                    v[index]
                }
            }
            ViewAxis::General(axis) => axis.prod_dot(v),
        }
    }

    pub fn vector(&self) -> Vector3<S> {
        match *self {
            ViewAxis::Unit { index, negative } => {
                let mut v = Vector3::zero();
                v[index] = if negative { S::ONE.neg() } else { S::ONE };
                v
            }
            ViewAxis::General(axis) => axis,
        }
    }
}

/// How a view looks: `direction` from the eye into the part, and the
/// paper's `right` and `up` — right-handed, `right x up = -direction`.
#[derive(Clone, Copy, Debug)]
pub struct ViewFrame<S: Scalar> {
    pub direction: ViewAxis<S>,
    pub right: ViewAxis<S>,
    pub up: ViewAxis<S>,
}

impl<S: Scalar> ViewFrame<S> {
    /// The frame looking along `direction`, with `right` and `up` on the
    /// paper; `direction` is normalized, and `right` and `up` made
    /// perpendicular to it.
    pub fn looking(direction: Vector3<S>, up: Vector3<S>) -> GeopResult<Self> {
        let d = direction.normalize()?;
        let right = d.prod_cross(&up).normalize()?;
        let up = right.prod_cross(&d).normalize()?;
        Ok(ViewFrame {
            direction: ViewAxis::General(d),
            right: ViewAxis::General(right),
            up: ViewAxis::General(up),
        })
    }

    /// Where `p` lands on the paper.
    pub fn project_point(&self, p: &Vector3<S>) -> Vector2<S> {
        Vector2::from_array([self.right.dot(p), self.up.dot(p)])
    }

    /// `curve` as drawn on the paper: the projection is linear, so it maps
    /// homogeneous control points to homogeneous control points, keeping
    /// the knots and weights — the same parametrization, exactly.
    pub fn project_curve(&self, curve: &NurbCurve3D<S>) -> GeopResult<NurbCurve2D<S>> {
        let points = curve
            .control_points
            .iter()
            .map(|q| {
                let h = q.head::<3>();
                Vector3::from_array([self.right.dot(&h), self.up.dot(&h), q[3]])
            })
            .collect();
        NurbCurve2D::try_new(curve.degree, points, curve.knot_vector.clone())
    }

    /// The unit vector from the part towards the eye.
    pub fn toward_eye(&self) -> Vector3<S> {
        self.direction.vector().neg()
    }
}
