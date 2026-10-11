//! A part's cables, kept as its target [`cables`]: what a routed harness is
//! cut from — its wires, each with the length to cut it to — recorded by
//! the step that routed it, for its editor to show and a bill of materials
//! to read.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_program::target::{Revisable, TargetReference};
use geop_ops::Design;

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

/// A part's cables, by the name of the solid each is swept into: the value
/// of its target [`cables`], which every step that routes one revises.
#[derive(Clone, Debug, PartialEq)]
pub struct Cables<S: Scalar>(BTreeMap<String, Cable<S>>);

/// The part's cables.
pub fn cables() -> TargetReference<Cables<Design>> {
    TargetReference::new("cables")
}

impl<S: Scalar> Default for Cables<S> {
    fn default() -> Self {
        Self(BTreeMap::new())
    }
}

/// A part has few cables: a revision copies them.
impl<S: Scalar + PartialEq> Revisable for Cables<S> {
    fn to_revise(&self) -> Self {
        self.clone()
    }
}

impl<S: Scalar> Cables<S> {
    /// Records `cable` under `name` — the name of the solid it is swept
    /// into. Fails, leaving the cables as they are, if there is a cable of
    /// that name already.
    pub fn add(&mut self, name: impl Into<String>, cable: Cable<S>) -> GeopResult<()> {
        let name = name.into();
        if self.0.contains_key(&name) {
            return Err(GeopError::new(format!(
                "the part has a cable named {name:?} already"
            )));
        }
        self.0.insert(name, cable);
        Ok(())
    }

    /// The cable `name`.
    pub fn get(&self, name: &str) -> GeopResult<&Cable<S>> {
        (self.0.get(name))
            .ok_or_else(|| GeopError::new(format!("the part has no cable named {name:?}")))
    }

    /// Every cable, by name.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Cable<S>)> {
        self.0.iter().map(|(name, cable)| (name.as_str(), cable))
    }
}
