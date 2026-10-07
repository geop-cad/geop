//! A part's cables, kept in it as the extension [`Cables`] and read and
//! written through [`PartCables`]: what a routed harness is cut from — its wires, each
//! with the length to cut it to — recorded by the step that routed it, for
//! its editor to show and a bill of materials to read.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_ops::{Extension, Part};

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

/// A part's cables, by the name of the solid each is swept into.
#[derive(Clone, Debug)]
pub struct Cables<S: Scalar>(BTreeMap<String, Cable<S>>);

impl<S: Scalar> Default for Cables<S> {
    fn default() -> Self {
        Self(BTreeMap::new())
    }
}

impl<S: Scalar> Extension<S> for Cables<S> {
    const NAME: &'static str = "cables";
}

/// A part's cables (see the module).
pub trait PartCables<S: Scalar> {
    /// Records `cable` under `name` — the name of the solid it is swept
    /// into.
    fn add_cable(&mut self, name: impl Into<String>, cable: Cable<S>) -> GeopResult<()>;

    /// The cable `name`.
    fn cable(&self, name: &str) -> GeopResult<&Cable<S>>;

    /// Every cable of the part itself — not of the parts placed in it — by
    /// name.
    fn cables(&self) -> impl Iterator<Item = (&str, &Cable<S>)>;
}

impl<S: Scalar> PartCables<S> for Part<S> {
    fn add_cable(&mut self, name: impl Into<String>, cable: Cable<S>) -> GeopResult<()> {
        let name = name.into();
        if self
            .ext::<Cables<S>>()
            .is_some_and(|c| c.0.contains_key(&name))
        {
            return Err(GeopError::new(format!(
                "Part::add_cable: there is a cable {name:?} already"
            )));
        }
        self.ext_mut::<Cables<S>>().0.insert(name, cable);
        Ok(())
    }

    fn cable(&self, name: &str) -> GeopResult<&Cable<S>> {
        self.ext::<Cables<S>>()
            .and_then(|cables| cables.0.get(name))
            .ok_or_else(|| GeopError::new(format!("there is no cable {name:?}")))
    }

    fn cables(&self) -> impl Iterator<Item = (&str, &Cable<S>)> {
        self.ext::<Cables<S>>()
            .into_iter()
            .flat_map(|cables| cables.0.iter().map(|(name, c)| (name.as_str(), c)))
    }
}
