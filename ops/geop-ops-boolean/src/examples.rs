//! The examples of combining and splitting solids, which an editor offers
//! to load ([`set`]).

use geop_core_editor::registry::ExampleSet;
use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_ops_model::datums::origin_plane;
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output};
use geop_ops_model::{Combine, element, solid};
use geop_ops_sketch::{
    AddSketchArgs, Constraint, Sketch, SketchPlane,
    figures::{circle, n, rectangle, solved},
    whole_sketch,
};

use crate::SplitArgs;

/// The examples of this crate, as an editor offers them.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("split_plate", split_plate)],
        workspaces: Vec::new(),
    }
}

/// The plane of the origin normal to `normal`, as a plane to sketch on.
fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
}

/// A plate with a round hole, cut in two: the plate (`plate`) extruded both
/// ways from its sketch, a line across it sketched on the plane through its
/// middle and extruded into a face standing on its own (`cut`), and the
/// plate split along that face (`halves`) into `split(halves,0)` and
/// `split(halves,1)`, one of them with the hole.
pub fn split_plate() -> Program {
    let mut program = Program::new();
    let mut plate = Sketch::new();
    rectangle(&mut plate, [0.0, 0.0], 3.0, 1.0);
    circle(&mut plate, [0.75, 0.5], 0.3);
    program.push(
        "plate_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(plate)),
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("plate_sketch"),
            extent: Extents {
                side1: Extent::blind(0.5),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            output: Output::Solid(Combine::NewBody),
        },
    );

    // Sketch x runs along world x, sketch y along world -z: a line across
    // the plate's thickness at x = 2.
    let mut cut = Sketch::new();
    let (a, b) = (
        cut.add_point(n(2.0), n(-1.0)),
        cut.add_point(n(2.0), n(1.0)),
    );
    let line = cut.add_line(a, b);
    for (point, y) in [(a, -1.0), (b, 1.0)] {
        cut.constrain(Constraint::Fix {
            point,
            x: n(2.0),
            y: n(y),
        });
    }
    program.push(
        "cut_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Y), solved(cut)),
    );
    program.push(
        "cut",
        ExtrudeArgs {
            sketch: whole_sketch("cut_sketch"),
            extent: Extents {
                side1: Extent::blind(3.0),
                symmetric: true,
                side2: None,
                reversed: false,
            },
            output: Output::Faces { up_to: None },
        },
    );
    program.push(
        "halves",
        SplitArgs {
            solid: solid("extrude(plate)"),
            face: element(format!("extrude(cut,cut_sketch,{line})")),
        },
    );
    program
}
