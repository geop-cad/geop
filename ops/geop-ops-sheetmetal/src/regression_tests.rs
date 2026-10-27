//! Programs from tutorials and bug reports, rebuilt as the smallest
//! programs found that still fail.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_ops_model::{
    sketch::{
        Sketch,
        figures::{on_origin, outline},
        whole_sketch,
    },
    testing::{Built, assert_valid},
};
use geop_ops_sketch::{AddSketch, AddSketchArgs};

use crate::{
    BaseFlange, BaseFlangeArgs, Corner, EdgeFlange, EdgeFlangeArgs, FlangePosition,
    LengthReference, Relief, SheetMetalRules, edge,
};

/// The operations these programs are written in: a sketch, and the sheet
/// metal made of it.
fn registry() -> GeopResult<OperationRegistry> {
    (OperationRegistry::default().with(AddSketch)?)
        .with(BaseFlange)?
        .with(EdgeFlange)
}

/// A flange 30 high, bent up from the whole of the edge `edge`.
fn flange(edge_name: &str) -> EdgeFlangeArgs {
    EdgeFlangeArgs {
        edge: Some(edge(edge_name)),
        angle: 90.0.into(),
        length: 30.0.into(),
        reference: LengthReference::OuterSharp,
        position: FlangePosition::MaterialInside,
        radius: None,
        offset_start: 0.0,
        offset_end: 0.0,
        corner: Corner::Open,
    }
}

/// A tray's first two walls (every sheet-metal tutorial: a plate, a flange
/// up from each of two edges that meet at a corner): a 100 x 60 plate 1
/// thick, bend radius 1, with the corner gap between the flanges 0.5 —
/// less than the sheet's thickness, as SolidWorks' default of 0.1 is — and
/// a flange 30 high on its edge along `x` and then on its edge along `y`.
///
/// The first flange builds. The second is refused, whichever relief the
/// rules ask for, each refusal sending the user to the other:
///
/// > rectangular: the relief at the start of edge base_flange(plate,k,c5)
/// > does not fit beside it: offset the flange further, or use a tear
///
/// > tear: a flange on part of edge base_flange(plate,k,c5), set back into
/// > the sheet, needs a relief beside it: with a tear, the bend would touch
/// > the sheet beside it — use a rectangular relief, run the flange along
/// > the whole edge, or bend it outside
///
/// The flange does run along the whole edge, and beside it, past the gap,
/// is the first flange's bend, not sheet. With a gap as wide as the sheet
/// is thick (1) or wider, both flanges build; the same plate scaled to 1 x
/// 1, 0.1 thick, fails alike with a gap of 0.05.
#[test]
#[ignore = "bug: a second edge flange next to the first at a plate's corner is refused when the corner gap is less than the thickness — rectangular relief \"does not fit beside it ... use a tear\", tear \"needs a relief beside it ... use a rectangular relief\"; a gap >= the thickness builds"]
fn tray_walls_meeting_at_a_corner_with_a_narrow_gap() {
    for relief in [Relief::Rectangular, Relief::Tear] {
        let mut program = Program::new();
        let plate: Sketch = outline(&[[0.0, 0.0], [100.0, 0.0], [100.0, 60.0], [0.0, 60.0]]);
        program.push("k", AddSketchArgs::on(on_origin(FrameAxis::Z), plate));
        program.push(
            "plate",
            BaseFlangeArgs {
                sketch: whole_sketch("k"),
                rules: SheetMetalRules {
                    thickness: 1.0,
                    bend_radius: 1.0,
                    corner_gap: 0.5,
                    relief,
                    ..SheetMetalRules::default()
                },
                depth: 1.0.into(),
                flip: false,
            },
        );
        program.push("f1", flange("base_flange(plate,k,c4,b)"));
        program.push("f2", flange("base_flange(plate,k,c5,b)"));
        let part = program
            .build_model(registry)
            .unwrap_or_else(|e| panic!("{relief:?}: {e}"));
        assert_valid(&part);
    }
}
