//! Handles: how a step can be edited by dragging in a 3-D view.
//!
//! An operation describes, for a step, where its adjustable values sit in
//! space and how dragging them maps back onto its arguments: a handle has a
//! position, a way it moves (along a line, or in a plane), and the path of
//! the argument(s) in the step it rewrites. An editor draws the handles it
//! wants to offer, and turns a drag into new argument values at those paths
//! — so dragging edits the program exactly like typing a value in a form
//! does, and the editor needs to know nothing about the operation.
//!
//! Every step provides all of its handles; which to show is the editor's
//! choice (see [`HandleGroup`]).

use serde::Serialize;

/// Where a handle writes in its step: field names into the arguments as
/// they serialize — `["distance"]`, or `["sketch", "points", "7", "x"]`.
pub type ArgPath = Vec<String>;

/// A path from `&str` segments.
pub fn arg_path(segments: &[&str]) -> ArgPath {
    segments.iter().map(|s| s.to_string()).collect()
}

/// What a handle belongs to, so an editor can offer only what fits what the
/// user is doing: a feature's parameters while looking at the part, a
/// sketch's points while sketching.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum HandleGroup {
    Feature,
    Sketch,
}

/// How a handle moves, and how that maps onto its step's arguments.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "motion", rename_all = "snake_case")]
pub enum HandleMotion {
    /// Slides along the unit vector `direction`: moving it by `d` world
    /// units changes the number at `arg`, now `value`, by `d / scale`.
    Linear {
        direction: [f64; 3],
        arg: ArgPath,
        value: f64,
        scale: f64,
    },
    /// Slides in the plane of the unit vectors `u` and `v`: moving it by
    /// `a u + b v` changes the numbers at `x` and `y`, now `value`, by `a`
    /// and `b`.
    Planar {
        u: [f64; 3],
        v: [f64; 3],
        x: ArgPath,
        y: ArgPath,
        value: [f64; 2],
    },
}

/// One draggable value of a step.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Handle {
    /// What it adjusts, e.g. `distance`.
    pub label: String,
    pub group: HandleGroup,
    /// Where it is drawn.
    pub position: [f64; 3],
    #[serde(flatten)]
    pub motion: HandleMotion,
}
