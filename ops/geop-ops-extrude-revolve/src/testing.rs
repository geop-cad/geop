//! What the tests of programs share that start from sketches: the
//! operations they are written in, the steps extruding sketches, and the
//! bodies a sketch and an extrusion or two build (a block with a post
//! sketched on it, a drilled box, and the [`bodies`] the stress sweeps run
//! over). Only built for tests: with `testing`, or in this crate's own.
//!
//! The sketches themselves are drawn with `geop_ops_sketch::figures`, a
//! program's steps are read and run with `geop_core_program::testing`, and
//! the model it builds with `geop_ops_model::testing`.

// Helpers of tests fail as tests do: by panicking, naming what is wrong.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{
    operation_registry::OperationRegistry,
    program::{Program, ProgramStep},
};
use geop_ops_model::Combine;
use geop_ops_model::parameters::{DefineParameters, LookupTable, PartProperties};
use geop_ops_sketch::{
    AddSketch, AddSketchArgs, Sketch, SketchPlane,
    figures::{disc, on_face, on_origin, polygon, rectangle_outline},
    whole_sketch,
};

pub mod bodies;

use crate::{Extent, Extents, Extrude, ExtrudeArgs, Loft, Output, Revolve, Sweep};

/// Sketching, and the solids and sheets made from sketches: the operations
/// a test's programs are written in, besides those it tests.
pub fn registry() -> GeopResult<OperationRegistry> {
    (OperationRegistry::default()
        .with(DefineParameters)?
        .with(LookupTable)?
        .with(PartProperties)?
        .with(AddSketch)?
        .with(Extrude)?
        .with(Revolve)?)
    .with(Sweep)?
    .with(Loft)
}

/// One side only, ending as `side1` says.
pub fn one_way(side1: Extent) -> Extents {
    Extents {
        side1,
        symmetric: false,
        side2: None,
        reversed: false,
    }
}

/// Both ways alike, `length` in all.
pub fn both_ways(length: f64) -> Extents {
    Extents {
        symmetric: true,
        ..Extents::blind(length)
    }
}

/// `first` one way and `second` the other.
pub fn two_ways(first: f64, second: f64) -> Extents {
    Extents {
        side2: Some(Extent::blind(second)),
        ..Extents::blind(first)
    }
}

/// The step extruding the sketch `sketch` as far as `extent` says into a
/// solid, combined as `combine` says.
pub fn extrusion(sketch: &str, extent: Extents, combine: Combine) -> ExtrudeArgs {
    ExtrudeArgs {
        sketch: whole_sketch(sketch),
        extent,
        output: Output::Solid(combine),
    }
}

/// The step extruding the sketch `sketch` by `distance` into a new body:
/// one way, or both ways alike if `symmetric`.
pub fn extrude(sketch: &str, distance: f64, symmetric: bool) -> ExtrudeArgs {
    extrusion(
        sketch,
        Extents {
            side1: Extent::blind(distance),
            symmetric,
            side2: None,
            reversed: false,
        },
        Combine::NewBody,
    )
}

/// A block, from `z0` to `z1` over `[0, 2]^2`, sketched on the Z plane
/// (`base`) and extruded both ways from it (`block`), the square
/// `post_sketch` drawn on the Z plane under its middle, and `steps` after
/// them.
pub fn block_and_post(z0: f64, z1: f64, steps: Vec<ProgramStep>) -> Program {
    let mut base = Sketch::new();
    polygon(&mut base, &[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]);
    let mut post = Sketch::new();
    polygon(&mut post, &[[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5]]);
    let mut program = Program::new();
    program.push("base", AddSketchArgs::on(on_origin(FrameAxis::Z), base));
    program.push(
        "block",
        ExtrudeArgs {
            extent: two_ways(z1, -z0),
            ..extrude("base", 1.0, false)
        },
    );
    program.push(
        "post_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), post),
    );
    program.steps.extend(steps);
    program
}

/// Pushes `drawn` on `on` as `{name}_sketch`, and its extrude as far as
/// `extent` says as `name`, combined as `combine` says.
pub fn extrude_sketch(
    program: &mut Program,
    name: &str,
    on: SketchPlane,
    drawn: Sketch,
    extent: Extents,
    combine: Combine,
) {
    let sketch_name = format!("{name}_sketch");
    program.push(&sketch_name, AddSketchArgs::on(on, drawn));
    program.push(name, extrusion(&sketch_name, extent, combine));
}

/// A 2 x 2 x 1 box with a blind hole drilled into its top, in the steps the
/// example `box_with_drill_hole` has: a square sketched on the Z plane
/// (`outline`) and extruded up (`box`), and a circle of radius 0.4 around
/// its middle sketched on the box's end cap (`hole_sketch`) and cut half
/// way back into it (`hole`) — the drilled box is `extrude(hole)`.
pub fn drilled_box() -> Program {
    let mut program = Program::new();
    let outline = AddSketchArgs::on(
        on_origin(FrameAxis::Z),
        rectangle_outline(0.0, 0.0, 2.0, 2.0),
    );
    program.push("outline", outline);
    let body = extrusion("outline", Extents::blind(1.0), Combine::NewBody);
    program.push("box", body);
    let hole = AddSketchArgs::on(on_face("extrude(box,end)"), disc([1.0, 1.0], 0.4));
    program.push("hole_sketch", hole);
    let cut = extrusion(
        "hole_sketch",
        Extents::blind(-0.5),
        Combine::cut("extrude(box)"),
    );
    program.push("hole", cut);
    program
}
