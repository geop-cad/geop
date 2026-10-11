//! [`PlacedSketch`]: a sketch and the plane it was placed on — the value of
//! a sketch's target ([`sketch`]) — and a 3-D sketch, the value of its
//! target ([`sketch3d`]).

use geop_core_geometry::nurb_curve::{NurbCurve, NurbCurve3D};
use geop_core_math::{geop_error::GeopResult, primitives::CoordinateSystem, scalars::Scalar};
use geop_core_sketch::{CurveId, Enclosure, Sketch, profile::curve_nurbs, space::Sketch3d};

use geop_core_program::target::TargetReference;

use crate::Design;

/// A sketch together with the plane it lies on: `plane.u`/`plane.v` are the
/// sketch's `x`/`y` (unit, orthogonal), `plane.w = u x v` its unit normal.
///
/// The plane is resolved once, when the sketch is added — a sketch drawn on a
/// face stays where that face *was*, even after a later operation reshapes or
/// consumes the face.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedSketch<S: Scalar> {
    pub plane: CoordinateSystem<S>,
    pub sketch: Sketch<Design>,
}

impl<S: Scalar> PlacedSketch<S> {
    /// The curve `curve` of the sketch where its plane puts it: one NURBS
    /// from its start to its end — a circle round from its seam — of the
    /// sketch solved as `geometry` says (see [`Sketch::enclose`]).
    pub fn curve_in_space(
        &self,
        curve: CurveId,
        geometry: &Enclosure<S>,
    ) -> GeopResult<NurbCurve3D<S>> {
        let pieces = curve_nurbs(&self.sketch, geometry, curve)?;
        let plane = &self.plane;
        NurbCurve::join(&pieces)?.embed(plane.origin(), plane.u(), plane.v())
    }
}

/// What the target of a sketch is named, before the sketch's name.
const PREFIX: &str = "sketch:";

/// The sketch `name`: what the step of that id draws.
pub fn sketch(name: &str) -> TargetReference<PlacedSketch<Design>> {
    TargetReference::new(format!("{PREFIX}{name}"))
}

/// The name of the sketch whose target is named `target`, if it is one.
pub fn sketch_name(target: &str) -> Option<&str> {
    target.strip_prefix(PREFIX)
}

/// A 3-D sketch as a part holds it: the sketch, what it is given of the
/// part brought up to date and solved, and the names of the edges of the
/// wire it is built as — what a pick of one of them takes the whole sketch
/// for, where a path is asked for.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedSketch3d {
    pub sketch: Sketch3d<Design>,
    pub edges: Vec<String>,
}

/// What the target of a 3-D sketch is named, before the sketch's name.
const PREFIX_3D: &str = "sketch3d:";

/// The 3-D sketch `name`: what the step of that id draws.
pub fn sketch3d(name: &str) -> TargetReference<PlacedSketch3d> {
    TargetReference::new(format!("{PREFIX_3D}{name}"))
}

/// The name of the 3-D sketch whose target is named `target`, if it is one.
pub fn sketch3d_name(target: &str) -> Option<&str> {
    target.strip_prefix(PREFIX_3D)
}
