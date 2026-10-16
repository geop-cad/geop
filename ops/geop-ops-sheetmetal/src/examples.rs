//! The examples of sheet metal an editor offers: a bracket flanged and cut.

use geop_core_editor::registry::ExampleSet;
use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_core_sketch::CurveId;
use geop_ops_datums::origin_plane;
use geop_ops_sketch::{
    AddSketchArgs, Sketch, SketchPlane,
    figures::{circle, rectangle, solved},
    whole_sketch,
};

use crate::{
    BaseFlangeArgs, Corner, EdgeFlangeArgs, FlangePosition, LengthReference, SheetCutArgs,
    SheetMetalRules, edge,
};

/// The examples of this crate, by name.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("sheet_metal_bracket", sheet_metal_bracket)],
        workspaces: Vec::new(),
    }
}

/// A sheet-metal mounting bracket: a 2 x 1.2 plate 0.08 thick (`plate`),
/// a flange bent up along part of its front edge with reliefs beside it
/// (`front`), one along all of its back edge (`back`), and two mounting
/// holes cut through the plate after flanging (`mount`, drawn in `holes`)
/// — the body is `sheet_cut(mount)`.
pub fn sheet_metal_bracket() -> Program {
    let on_z = || Some(SketchPlane::Datum(origin_plane(FrameAxis::Z)));
    let mut program = Program::new();
    let mut outline = Sketch::new();
    let lines = rectangle(&mut outline, [0.0, 0.0], 2.0, 1.2);
    program.push(
        "outline",
        AddSketchArgs {
            plane: on_z(),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "plate",
        BaseFlangeArgs {
            sketch: whole_sketch("outline"),
            rules: SheetMetalRules {
                thickness: 0.08,
                bend_radius: 0.08,
                ..SheetMetalRules::default()
            },
            depth: 1.0.into(),
            flip: false,
        },
    );
    let flange = |line: CurveId, length: f64, offset: f64| EdgeFlangeArgs {
        edge: Some(edge(format!("base_flange(plate,outline,{line},b)"))),
        angle: 90.0.into(),
        length: length.into(),
        reference: LengthReference::OuterSharp,
        position: FlangePosition::MaterialInside,
        radius: None,
        offset_start: offset,
        offset_end: offset,
        corner: Corner::Open,
    };
    program.push("front", flange(lines[0], 0.6, 0.3));
    program.push("back", flange(lines[2], 0.4, 0.0));
    let mut holes = Sketch::new();
    circle(&mut holes, [0.5, 0.6], 0.15);
    circle(&mut holes, [1.5, 0.6], 0.15);
    program.push(
        "holes",
        AddSketchArgs {
            plane: on_z(),
            sketch: solved(holes),
            ..Default::default()
        },
    );
    program.push(
        "mount",
        SheetCutArgs {
            sketch: whole_sketch("holes"),
            face: None,
        },
    );
    program
}
