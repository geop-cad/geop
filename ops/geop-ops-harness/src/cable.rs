//! A part's cables, kept as its target [`cables`]: what a routed harness is
//! cut from — its wires, each with the length to cut it to — recorded by
//! the step that routed it, for its editor to show. What a bill of
//! materials orders of it is noted on its bundle (see
//! [`geop_ops::solids::solid_notes`]).

use geop_core_math::scalars::Scalar;
use geop_core_program::target::TargetReference;
use geop_ops::{Design, model::Records, solids::CutWire};

/// A bundle of wires laid along one route.
#[derive(Clone, Debug, PartialEq)]
pub struct Cable<S: Scalar> {
    /// How long the route is, end to end.
    pub length: S,
    /// The bundle's diameter, which its solid is swept with.
    pub diameter: f64,
    /// The smallest radius the route may bend with.
    pub min_bend_radius: f64,
    /// The smallest radius it does bend with — `None` for a straight one.
    pub tightest_bend: Option<S>,
    pub wires: Vec<CutWire<S>>,
}

/// A part's cables, by the name of the solid each is swept into: the value
/// of its target [`cables`], which every step that routes one revises.
pub type Cables<S> = Records<Cable<S>>;

/// The part's cables.
pub fn cables() -> TargetReference<Cables<Design>> {
    TargetReference::new("cables")
}
