//! Sheet metal: bodies of one thickness, flat faces joined by bends —
//! brackets and enclosures — and the flat blanks they are cut from.
//!
//! - [`BaseFlange`] starts a body from a sketch: a plate from its area, or
//!   a bent strip from a chain of lines and arcs, with the body's
//!   [`SheetMetalRules`] (thickness, bend radius, K-factor, reliefs).
//! - [`EdgeFlange`] bends a flange up from a straight edge of the body,
//!   cutting reliefs beside it and keeping a gap at corners; [`Hem`] folds
//!   an edge right back on itself.
//! - [`SheetCut`] cuts holes and notches through the body, across its bends
//!   unrolled.
//! - [`FlatPattern`] unfolds the body: each bend laid flat as long as its
//!   developed length, with its bend lines; [`FlatPatternDxf`] writes it
//!   for laser cutting.
//!
//! Each operation records the body's [`Sheet`] for the solid it builds, in
//! the part's target [`sheets`]: which faces are flat and which are bends,
//! and how they join, so that the next flange and the flat pattern work
//! from what the body is made of rather than guessing it from its
//! geometry. The solid itself is built from the sheet in one go (see
//! [`thicken`]).

pub mod base_flange;
pub mod body;
pub mod cut;
pub mod cutting;
pub mod edge_flange;
pub mod flat_pattern;
mod fold;
pub mod hem;
pub mod layout;
pub mod sheet;
pub mod thicken;

mod icons;
#[cfg(test)]
mod program_tests;

pub use base_flange::{BaseFlange, BaseFlangeArgs};
pub use body::{SheetBody, Sheets, sheet_body, sheets};
pub use cut::{FlatFaceRef, SheetCut, SheetCutArgs};
pub use cutting::{FlatPatternDxf, FlatPatternOptions, cutting_sheet};
pub use edge_flange::{
    Corner, EdgeFlange, EdgeFlangeArgs, EdgeRef, FlangePosition, LengthReference, edge,
};
pub use flat_pattern::{BendLine, FlatPattern, FlatPatternArgs, FlatPatternData, SolidRef};
pub use hem::{Hem, HemArgs, HemKind};
pub use sheet::{Relief, Sheet, SheetMetalRules};
