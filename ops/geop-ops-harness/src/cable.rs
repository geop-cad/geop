//! A part's cables, kept as its target [`cables`]: what a routed harness is
//! cut from — its wires, each with the length to cut it to — recorded by
//! the step that routed it, for its editor to show. What a bill of
//! materials orders of it is noted on its bundle (see
//! [`geop_ops_model::solid_notes`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    part::Reader,
    target::TargetReference,
    values::Design,
};
use geop_ops_model::{CutWire, Records};

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

/// A cable, by the name of the solid it is swept into: an element of
/// [`cables`].
impl Kind for Cable<Design> {
    const NAME: &'static str = "cable";
}

impl HasElements<Cable<Design>> for Cables<Design> {
    type Element<'a> = &'a Cable<Design>;
    const SINGLETON: Option<&'static str> = Some("cables");

    fn element(&self, name: &str) -> GeopResult<&Cable<Design>> {
        self.get(name)
            .ok_or_else(|| GeopError::new(format!("the part has no cable named {name:?}")))
    }
}

/// The cable `name` of the part `reader` reads, if it has one: none before
/// a step routed one.
pub fn cable<'a>(reader: &Reader<'a>, name: &str) -> Option<&'a Cable<Design>> {
    reader.defines(&cables()).then_some(())?;
    reader
        .retrieve_element(&ElementReference::new(cables(), name))
        .ok()
}
