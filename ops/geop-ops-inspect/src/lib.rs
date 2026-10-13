//! Inspecting a part: what an engineer asks of a design without changing
//! it — none of it is an operation, and none of it adds a step. Each is an
//! [`Inspector`] an editor offers (see [`geop_ops_output`]):
//!
//! - [`Measure`]: distances, angles, lengths, areas and radii of picked
//!   entities ([`measure()`]).
//! - [`MassProperties`]: the mass properties of every solid of a part and
//!   of the parts placed in it, each of its own part's material, and of
//!   all of them together ([`mass_report`]).
//! - [`Interference`]: which solids overlap, by how much, and which only
//!   touch ([`interference_report`]).
//!
//! Every number is computed from the exact geometry and carries how well it
//! is known ([`Bounded`]): an interval enclosure, widened — for the
//! integrals — by the quadrature's estimate of its truncation error (see
//! [`geop_core_math::quadrature`]). Lengths are millimetres, masses
//! kilograms (see [`geop_ops_parameters::Material`]).

pub mod interference;
pub mod mass;
pub mod measure;

pub use interference::{InterferenceReport, interference_report};
pub use mass::{MassReport, mass_report};
pub use measure::{Measurement, measure};

use geop_core_math::geop_error::GeopResult;
use geop_core_editor::editing::Visual;
use geop_core_program::element::PickedElement;
use geop_ops_model::Design;
use geop_ops_output::{Answer, Inspector, InspectorInfo, Output, Picks};

/// Measures picked entities: one alone, or the two last picked together.
pub struct Measure;

impl Inspector for Measure {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "measure",
        label: "Measure",
        doc: "Measure picked entities: a point, a length, an area, a radius; the distance and angle between two.",
        icon: Some(crate::icons::MEASURE),
        picks: Some(Picks {
            accepts: measure::PICKS,
            most: 2,
        }),
    };
    type Options = ();
    type Report = Measurement;

    fn inspect(
        output: &mut Output,
        picked: &[PickedElement],
        (): (),
    ) -> GeopResult<(Measurement, Vec<Visual<Design>>)> {
        let measurement = measure(&output.part(), picked);
        let visuals = measurement.visuals();
        Ok((measurement, visuals))
    }

    fn show(measurement: &Measurement) -> Answer {
        measurement.shown()
    }
}

/// Weighs the part: every solid, each of its own part's material.
pub struct MassProperties;

impl Inspector for MassProperties {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "mass_properties",
        label: "Mass",
        doc: "The volume, area, mass, centre of mass and inertia of every solid, and of all of them.",
        icon: Some(crate::icons::MASS),
        picks: None,
    };
    type Options = ();
    type Report = MassReport;

    fn inspect(
        output: &mut Output,
        _: &[PickedElement],
        (): (),
    ) -> GeopResult<(MassReport, Vec<Visual<Design>>)> {
        Ok((mass_report(&output.part())?, Vec::new()))
    }

    fn show(report: &MassReport) -> Answer {
        report.shown()
    }
}

/// Checks the part's solids for interference.
pub struct Interference;

impl Inspector for Interference {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "interference",
        label: "Interference",
        doc: "Which solids overlap, and by how much, and which only touch.",
        icon: Some(crate::icons::INTERFERENCE),
        picks: None,
    };
    type Options = ();
    type Report = InterferenceReport;

    fn inspect(
        output: &mut Output,
        _: &[PickedElement],
        (): (),
    ) -> GeopResult<(InterferenceReport, Vec<Visual<Design>>)> {
        Ok((interference_report(&output.part())?, Vec::new()))
    }

    fn show(report: &InterferenceReport) -> Answer {
        report.shown()
    }
}

mod icons;
#[cfg(test)]
mod tests;
