//! The layers a drawing is drawn on (see [`Layer`]), in the order a sheet
//! writes them ([`ALL`]).

use geop_ops::output::sheet::{Layer, LineType};

/// The part's edges and silhouettes seen.
pub const VISIBLE: Layer = Layer {
    name: "VISIBLE",
    width: 0.5,
    line_type: LineType::Continuous,
    color: 7,
};

/// The part's edges and silhouettes behind its faces.
pub const HIDDEN: Layer = Layer {
    name: "HIDDEN",
    width: 0.3,
    line_type: LineType::Hidden,
    color: 8,
};

/// Centre lines and marks: axes of holes and cylinders.
pub const CENTER: Layer = Layer {
    name: "CENTER",
    width: 0.25,
    line_type: LineType::Center,
    color: 1,
};

/// Dimensions, balloons and the parts list.
pub const DIMENSION: Layer = Layer {
    name: "DIMENSIONS",
    width: 0.25,
    line_type: LineType::Continuous,
    color: 3,
};

/// The hatching of a section's cut faces.
pub const HATCH: Layer = Layer {
    name: "HATCH",
    width: 0.18,
    line_type: LineType::Continuous,
    color: 9,
};

/// The sheet's frame and title block.
pub const BORDER: Layer = Layer {
    name: "BORDER",
    width: 0.5,
    line_type: LineType::Continuous,
    color: 7,
};

/// Thin continuous lines: a cosmetic thread seen.
pub const THREAD: Layer = Layer {
    name: "THREAD",
    width: 0.25,
    line_type: LineType::Continuous,
    color: 4,
};

/// Every layer of a drawing, as its sheet lists them.
pub const ALL: [Layer; 7] = [VISIBLE, HIDDEN, CENTER, DIMENSION, HATCH, BORDER, THREAD];
