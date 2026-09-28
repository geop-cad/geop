//! [`Datum`]: reference geometry — a point, an axis, a plane or a whole
//! coordinate frame — that geometry is built *with*, not *of*.

use serde::{Deserialize, Serialize};

use super::CoordinateSystem;
use crate::scalars::Scalar;

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
    /// The frame itself: a coordinate system, its origin and all three of
    /// its axes.
    Frame,
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
