//! A [`Part`]'s datums: reference geometry — points, axes and planes the
//! part is built *with*, not *of*. A sketch is placed on a datum plane, a
//! datum axis is picked as a direction, a datum is built from another one.
//! Named like any other entity.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
};
use serde::{Deserialize, Serialize};

use crate::ids::DatumId;
use crate::part::Part;

/// What a datum stands for.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DatumKind {
    /// The frame's origin — with the frame itself, a coordinate system.
    Point,
    /// The line through the frame's origin along its `w`.
    Axis,
    /// The plane through the frame's origin normal to its `w`; `u`/`v` are
    /// a sketch's `x`/`y` on it.
    Plane,
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

impl<S: Scalar> Part<S> {
    /// Adds `datum` to the part under `name`. Fails, leaving the part
    /// unchanged, if `name` is already taken.
    pub fn add_datum(&mut self, datum: Datum<S>, name: impl Into<String>) -> GeopResult<DatumId> {
        let id = DatumId(self.fresh_id());
        self.names.insert(id, name)?;
        self.datums.insert(id, datum);
        Ok(id)
    }

    pub fn datum(&self, id: DatumId) -> GeopResult<&Datum<S>> {
        self.datums
            .get(&id)
            .ok_or_else(|| GeopError::new(format!("Part has no datum {id}")))
    }

    /// Every datum, in the order they were added.
    pub fn datums(&self) -> impl Iterator<Item = (DatumId, &Datum<S>)> {
        self.datums.iter().map(|(&id, d)| (id, d))
    }
}
