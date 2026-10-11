//! A part's datums: reference geometry it is built *with*, not *of* — a
//! point, an axis, a plane or a frame, each the value of a target of its
//! own ([`datum`]), defined by the step that adds it. Every part also has
//! [`crate::ORIGIN`], which no step adds.

use geop_core_math::primitives::Datum;
use geop_core_program::target::TargetReference;

use crate::Design;

/// What the target of a datum is named, before the datum's name.
const PREFIX: &str = "datum:";

/// The datum `name`: what the step of that id adds.
pub fn datum(name: &str) -> TargetReference<Datum<Design>> {
    TargetReference::new(format!("{PREFIX}{name}"))
}

/// The name of the datum whose target is named `target`, if it is one.
pub fn datum_name(target: &str) -> Option<&str> {
    target.strip_prefix(PREFIX)
}
