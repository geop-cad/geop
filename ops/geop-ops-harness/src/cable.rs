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
    reader::Reader,
    target::TargetReference,
};
use geop_ops_model::{CutWire, Records};

/// A bundle of wires laid along one route.
#[derive(Clone, Debug, PartialEq)]
pub struct Cable {
    /// How long the route is, end to end.
    pub length: Scalar,
    /// The bundle's diameter, which its solid is swept with.
    pub diameter: f64,
    /// The smallest radius the route may bend with.
    pub min_bend_radius: f64,
    /// The smallest radius it does bend with — `None` for a straight one.
    pub tightest_bend: Option<Scalar>,
    pub wires: Vec<CutWire>,
}

/// A part's cables, by the name of the solid each is swept into: the value
/// of its target [`cables`], which every step that routes one revises.
pub type Cables = Records<Cable>;

/// The part's cables.
pub fn cables() -> TargetReference<Cables> {
    TargetReference::new("cables")
}

/// A cable, by the name of the solid it is swept into: an element of
/// [`cables`].
impl Kind for Cable {
    const NAME: &'static str = "cable";
}

impl HasElements<Cable> for Cables {
    type Element<'a> = &'a Cable;
    const SINGLETON: Option<&'static str> = Some("cables");

    fn element(&self, name: &str) -> GeopResult<&Cable> {
        self.get(name)
            .ok_or_else(|| GeopError::new(format!("the part has no cable named {name:?}")))
    }
}

/// The cable `name` of the part `reader` reads, if it has one: none before
/// a step routed one.
pub fn cable<'a>(reader: &Reader<'a>, name: &str) -> Option<&'a Cable> {
    reader.defines(&cables()).then_some(())?;
    reader
        .retrieve_element(&ElementReference::new(cables(), name))
        .ok()
}
