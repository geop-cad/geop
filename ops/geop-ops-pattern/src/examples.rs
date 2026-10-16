//! The examples of patterns an editor offers: a plate drilled with a row
//! of holes, one hole patterned; and two drawn in the editor, a turbine
//! and a flange, each patterned round its axis.

use geop_core_editor::registry::ExampleSet;
use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_ops_datums::{origin_axis, origin_plane};
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, Output};
use geop_ops_model::Combine;
use geop_ops_sketch::{
    AddSketchArgs, Sketch, SketchPlane,
    figures::{circle, rectangle, solved},
    whole_sketch,
};

use crate::{Direction, LinearPatternArgs, PatternDirection, Seeds, Spacing, feature};

/// The examples of this crate, by name.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![
            ("patterned_plate", patterned_plate),
            ("turbine_blade", turbine_blade),
            ("cool_flange", cool_flange),
        ],
        workspaces: Vec::new(),
    }
}

/// An 8 x 3 x 0.5 plate with a row of four holes: a circle sketched on the
/// Z plane and cut through the plate (`hole`), and that feature done again
/// three more times 2 apart along `x` (`holes`) — the drilled plate is
/// `linear_pattern(holes)`, and the holes follow the hole when it changes.
pub fn patterned_plate() -> Program {
    let mut program = Program::new();
    let z_plane = || SketchPlane::Datum(origin_plane(FrameAxis::Z));
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 8.0, 3.0);
    program.push("outline", AddSketchArgs::on(z_plane(), solved(outline)));
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(0.5),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let mut hole = Sketch::new();
    circle(&mut hole, [1.0, 1.5], 0.3);
    program.push("hole_sketch", AddSketchArgs::on(z_plane(), solved(hole)));
    let mut through = Extents::blind(2.0);
    through.symmetric = true;
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: through,
            output: Output::Solid(Combine::cut("extrude(plate)")),
        },
    );
    program.push(
        "holes",
        LinearPatternArgs {
            seeds: Seeds::features(vec![feature("hole")]),
            first: Direction {
                along: Some(PatternDirection::Datum(origin_axis(FrameAxis::X))),
                reversed: false,
                count: 4.0.into(),
                spacing: Spacing::step(2.0),
            },
            second: None,
        },
    );
    program
}

/// A turbine: a blade lofted through sections on three offset datum
/// planes, patterned six times round a revolved hub and joined to it, as
/// drawn in the editor.
///
/// An example drawn in the editor and kept as the program it saved (in
/// `src/drawn/`): its sketches as they were drawn, which no code says
/// better. Its tests build it like every other example, so a change to the
/// format that breaks it is found there: save it again from the editor.
pub fn turbine_blade() -> Program {
    Program::from_json(include_str!("drawn/turbine_blade.geop"))
        .expect("a drawn example is a program")
}

/// A revolved flange with a slot cut through it, the cut patterned sixteen
/// times round its axis, as drawn in the editor (see [`turbine_blade`]).
pub fn cool_flange() -> Program {
    Program::from_json(include_str!("drawn/cool_flange.geop"))
        .expect("a drawn example is a program")
}
