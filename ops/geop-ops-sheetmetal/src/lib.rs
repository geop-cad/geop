//! Sheet metal: bodies of one thickness, flat faces joined by bends —
//! brackets and enclosures — and the flat blanks they are cut from.
//!
//! - [`BaseFlange`] starts a body from a sketch: a plate from its area, or
//!   a bent strip from a chain of lines and arcs, with the body's
//!   [`SheetMetalRules`] (thickness, bend radius, K-factor, reliefs).
//! - [`EdgeFlange`] bends a flange up from a straight edge of the body,
//!   cutting reliefs beside it and keeping a gap at corners.
//! - [`FlatPattern`] unfolds the body: each bend laid flat as long as its
//!   developed length, with its bend lines.
//!
//! Each operation records the body's [`Sheet`] on the solid it builds (see
//! [`geop_ops::Part::body_data`]): which faces are flat and which are
//! bends, and how they join, so that the next flange and the flat pattern
//! work from what the body is made of rather than guessing it from its
//! geometry. The solid itself is built from the sheet in one go (see
//! [`thicken`]).

pub mod base_flange;
pub mod cut;
pub mod edge_flange;
pub mod flat_pattern;
mod fold;
pub mod layout;
pub mod sheet;
pub mod thicken;

#[cfg(test)]
mod tests;

pub use base_flange::{BaseFlange, BaseFlangeArgs};
pub use cut::{SheetCut, SheetCutArgs};
pub use edge_flange::{EdgeFlange, EdgeFlangeArgs, FlangePosition, LengthReference};
pub use flat_pattern::{BendLine, FlatPattern, FlatPatternArgs, FlatPatternData};
pub use sheet::{Relief, Sheet, SheetMetalRules};
