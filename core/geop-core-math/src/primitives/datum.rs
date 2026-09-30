//! [`Datum`]: reference geometry — a point, an axis, a plane or a whole
//! coordinate frame — that geometry is built *with*, not *of*.

use serde::{Deserialize, Serialize};

use super::CoordinateSystem;
use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

/// What a datum stands for: which part of its frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatumKind {
    /// The frame's origin.
    Point,
    /// The line through the frame's origin along its `w`.
    Axis,
    /// The plane through the frame's origin normal to its `w`; `u`/`v` are
    /// a sketch's `x`/`y` on it.
    Plane,
    /// The frame itself: a coordinate system — its origin, its three axes
    /// and the three planes between them, each usable on its own (see
    /// [`Datum::component`]).
    Frame,
}

/// One of a frame's own axes: `x` is its `u`, `y` its `v`, `z` its `w`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FrameAxis {
    X,
    Y,
    Z,
}

impl FrameAxis {
    pub const ALL: [FrameAxis; 3] = [FrameAxis::X, FrameAxis::Y, FrameAxis::Z];

    /// `x`, `y` or `z`.
    pub fn letter(self) -> &'static str {
        match self {
            FrameAxis::X => "x",
            FrameAxis::Y => "y",
            FrameAxis::Z => "z",
        }
    }

    /// The frame's axis this is.
    pub fn of<S: Scalar>(self, frame: &CoordinateSystem<S>) -> crate::vector::Vector3<S> {
        match self {
            FrameAxis::X => *frame.u(),
            FrameAxis::Y => *frame.v(),
            FrameAxis::Z => *frame.w(),
        }
    }
}

/// A part of a frame datum that can be used on its own. Its origin is not
/// one: the frame as a whole is used as its origin already.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatumComponent {
    /// One of its axes.
    Axis(FrameAxis),
    /// The plane normal to one of its axes: `Plane(Z)` is its `xy` plane.
    Plane(FrameAxis),
}

impl std::fmt::Display for DatumComponent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            DatumComponent::Axis(axis) => write!(f, "{} axis", axis.letter()),
            DatumComponent::Plane(normal) => {
                let [a, b] = match normal {
                    FrameAxis::X => [FrameAxis::Y, FrameAxis::Z],
                    FrameAxis::Y => [FrameAxis::Z, FrameAxis::X],
                    FrameAxis::Z => [FrameAxis::X, FrameAxis::Y],
                };
                write!(f, "{}{} plane", a.letter(), b.letter())
            }
        }
    }
}

/// One datum: a right-handed orthonormal frame, and which part of it the
/// datum stands for. Every datum has a whole frame, whatever its kind, so
/// anything built on it — a sketch on a plane, a datum offset from a point
/// — has axes to be built along.
#[derive(Clone, Debug)]
pub struct Datum<S: Scalar> {
    pub kind: DatumKind,
    pub frame: CoordinateSystem<S>,
}

impl<S: Scalar> Datum<S> {
    /// The datum `component` of this frame stands for, as a datum of its
    /// own, with the frame turned so its `w` is that axis — or that plane's
    /// normal — and its other two axes keep their order: a sketch's `x`/`y`
    /// on the plane normal to `z` run along `x`/`y`, on the one normal to
    /// `x` along `y`/`z`, on the one normal to `y` along `x`/`-z`. Fails if
    /// this datum is not a frame.
    pub fn component(&self, component: DatumComponent) -> GeopResult<Datum<S>> {
        if self.kind != DatumKind::Frame {
            return Err(GeopError::new(format!(
                "Datum::component: a {:?} datum has no {component}, only a frame has",
                self.kind
            )));
        }
        let f = &self.frame;
        let along = |axis: FrameAxis| {
            let (u, v, w) = (*f.u(), *f.v(), *f.w());
            let (u, v, w) = match axis {
                FrameAxis::X => (v, w, u),
                FrameAxis::Y => (u, w.neg(), v),
                FrameAxis::Z => (u, v, w),
            };
            CoordinateSystem::try_new(*f.origin(), u, v, w)
        };
        Ok(match component {
            DatumComponent::Axis(axis) => Datum {
                kind: DatumKind::Axis,
                frame: along(axis)?,
            },
            DatumComponent::Plane(normal) => Datum {
                kind: DatumKind::Plane,
                frame: along(normal)?,
            },
        })
    }
}
