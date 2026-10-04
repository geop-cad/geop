//! A [`Part`]'s cables: what a routed harness is cut from — its wires, each
//! with the length to cut it to — recorded by the step that routed it, for
//! its editor to show and a bill of materials to read.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};

use super::Part;

/// One wire of a [`Cable`], and the length to cut it to.
#[derive(Clone, Debug, PartialEq)]
pub struct CutWire<S: Scalar> {
    pub name: String,
    /// Its colour, `#rrggbb`.
    pub colour: String,
    /// Its outer diameter, insulation included.
    pub diameter: f64,
    /// Its American wire gauge, if it was given by one.
    pub gauge: Option<i32>,
    /// The length to cut it to: the route's length and a service loop at
    /// each end. An enclosure, as every measured length is.
    pub cut_length: S,
}

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

impl<S: Scalar> Part<S> {
    /// Records `cable` under `name` — the name of the solid it is swept
    /// into.
    pub fn add_cable(&mut self, name: impl Into<String>, cable: Cable<S>) -> GeopResult<()> {
        let name = name.into();
        if self.cables.contains_key(&name) {
            return Err(GeopError::new(format!(
                "Part::add_cable: there is a cable {name:?} already"
            )));
        }
        self.cables.insert(name, cable);
        Ok(())
    }

    /// The cable `name`.
    pub fn cable(&self, name: &str) -> GeopResult<&Cable<S>> {
        self.cables
            .get(name)
            .ok_or_else(|| GeopError::new(format!("there is no cable {name:?}")))
    }

    /// Every cable of the part itself — not of the parts placed in it — by
    /// name.
    pub fn cables(&self) -> &BTreeMap<String, Cable<S>> {
        &self.cables
    }
}
