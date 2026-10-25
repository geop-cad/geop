//! The examples of rounding and bevelling edges, which an editor offers
//! to load ([`set`]).

use geop_core_editor::registry::ExampleSet;
use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, Output};
use geop_ops_model::Combine;
use geop_ops_model::datums::origin_plane;
use geop_ops_model::sketch::{
    Sketch, SketchPlane,
    figures::{rectangle, solved},
    whole_sketch,
};
use geop_ops_sketch::AddSketchArgs;

use crate::FilletArgs;

/// The examples of this crate, as an editor offers them.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("rounded_box", rounded_box)],
        workspaces: Vec::new(),
    }
}

/// The plane of the origin normal to `normal`, as a plane to sketch on.
fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
}

/// A 2 x 1.5 x 1 box with every edge rounded by 0.2, at once: where three
/// fillets meet, at each corner, the ball touching its three faces rounds
/// it — the solid `fillet(round)`.
pub fn rounded_box() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    let lines = rectangle(&mut outline, [0.0, 0.0], 2.0, 1.5);
    program.push(
        "outline",
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(outline)),
    );
    program.push(
        "box",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(1.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let mut edges = Vec::new();
    for line in &lines {
        for cap in ["start", "end"] {
            edges.push(format!("extrude(box,outline,{line},{cap})"));
        }
    }
    for corner in 0..4 {
        edges.push(format!("extrude(box,outline,p{corner})"));
    }
    program.push("round", FilletArgs::constant(edges, 0.2));
    program
}
