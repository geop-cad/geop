//! The examples of combining and splitting solids, which an editor offers
//! to load ([`set`]).

use geop_core_editor::registry::ExampleSet;
use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output};
use geop_ops_model::datums::origin_plane;
use geop_ops_model::sketch::{
    Constraint, Sketch, SketchPlane,
    figures::{circle, n, rectangle, solved},
    whole_sketch,
};
use geop_ops_model::{Combine, element, solid};
use geop_ops_sketch::AddSketchArgs;

use crate::{Along, SplitArgs, SplitFaceArgs};

/// The examples of this crate, as an editor offers them.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("split_plate", split_plate), ("badge", badge)],
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
            draft: 0.0,
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
            draft: 0.0,
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

/// A plate with a disc on its top, ready to colour or to pick on its own: the
/// plate extruded from its sketch (`plate`), a circle sketched on the plane
/// under it (`mark`), and the top divided along it (`disc`) into the disc,
/// `split_face(disc,extrude(plate,end),...)`, and the rest of the top, which
/// keeps its name. The solid is as it was.
pub fn badge() -> Program {
    let mut program = Program::new();
    let mut plate = Sketch::new();
    rectangle(&mut plate, [0.0, 0.0], 4.0, 2.0);
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
                symmetric: false,
                side2: None,
                reversed: false,
            },
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );
    let mut mark = Sketch::new();
    circle(&mut mark, [2.0, 1.0], 0.6);
    program.push(
        "mark",
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(mark)),
    );
    program.push(
        "disc",
        SplitFaceArgs {
            face: Some(element("extrude(plate,end)")),
            sketch: whole_sketch("mark"),
            along: Along::SketchNormal,
        },
    );
    program
}
