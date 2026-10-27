//! Programs from tutorials and bug reports, rebuilt as the smallest
//! programs found that still fail.

use geop_core_math::{geop_error::GeopResult, primitives::FrameAxis};
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_ops_model::{
    sketch::{
        Sketch,
        figures::{disc, on_origin, outline},
        whole_sketch,
    },
    testing::{Built, assert_valid},
};
use geop_ops_sketch::{AddSketch, AddSketchArgs};

use crate::{
    BaseFlange, BaseFlangeArgs, Corner, EdgeFlange, EdgeFlangeArgs, FlangePosition, HemArgs,
    HemKind, LengthReference, Relief, SheetMetalRules, edge,
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
/// thick, bend radius 1, with the corner gap between the flanges 0.5 and
/// a flange 30 high on its edge along `x` and then on its edge along `y`.
///
/// The second flange starts the gap short of the first one's bend, and the
/// strip of sheet left in the gap lies beside its own bend, set back into
/// the sheet: only a relief there keeps the two apart, and a bend's line
/// cannot be cut back to make room for one. With a relief 0.5 wide (half
/// the thickness) no relief fits in the gap. The second flange was refused
/// either way, each refusal sending the user to the other relief type
/// ("does not fit beside it ... or use a tear", "needs a relief beside it
/// ... use a rectangular relief"). Both now name the corner and what
/// builds, a rectangular relief narrower than the gap, which does.
#[test]
fn tray_walls_meeting_at_a_corner_with_a_narrow_gap() {
    let tray = |relief: Relief, relief_ratio: f64| {
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
                    relief_ratio,
                    ..SheetMetalRules::default()
                },
                depth: 1.0.into(),
                flip: false,
            },
        );
        program.push("f1", flange("base_flange(plate,k,c4,b)"));
        program.push("f2", flange("base_flange(plate,k,c5,b)"));
        program.build_model(registry)
    };
    for relief in [Relief::Rectangular, Relief::Tear] {
        let Err(err) = tray(relief, 0.5) else {
            panic!("{relief:?}: built");
        };
        let err = err.to_string();
        assert!(
            err.contains("meets the bent edge") && err.contains("corner gap"),
            "{relief:?}: {err}"
        );
        assert!(!err.contains("use a tear"), "{relief:?}: {err}");
    }
    let part = tray(Relief::Rectangular, 0.4).unwrap();
    assert_valid(&part);
}

/// A plate with a closed hem along one edge and a flange up from the next
/// edge round the corner (a hemmed tray): a 100 x 60 plate 1 thick, bend
/// radius 1, gap 1 — wide enough for two flanges at a corner — its edge
/// along `x` hemmed, folded back 10 over the plate with a gap of 1, then a
/// flange 30 high up the edge at `x = 100`.
///
/// Both steps build, the body invalid (44 errors): the flange runs the
/// whole edge, up through the hem's fold lying over the plate there.
///
/// > edge hem(h,flange,side1,a) and edge edge_flange(f2,flange,side0,a)
/// > intersect at a point that is not an existing vertex
///
/// > edge hem(h,flange,end,a) crosses face edge_flange(f2,flange,b)'s
/// > trimmed region at (99, 10, 4)
///
/// The flange keeps the corner gap from the hem's bend, but not from its
/// folded-back flat; it should stop short of it, or be refused by name.
#[test]
#[ignore = "bug: a flange up the edge next to a hemmed edge passes through the hem's folded flat: the sheet body is invalid (hem(h,flange,..) and edge_flange(f2,flange,..) intersect), not refused"]
fn flange_beside_a_hem() {
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
                corner_gap: 1.0,
                ..SheetMetalRules::default()
            },
            depth: 1.0.into(),
            flip: false,
        },
    );
    program.push(
        "h",
        HemArgs {
            edge: Some(edge("base_flange(plate,k,c4,b)")),
            kind: HemKind::Closed,
            length: 10.0,
            gap: 1.0,
            offset_start: 0.0,
            offset_end: 0.0,
        },
    );
    program.push("f2", flange("base_flange(plate,k,c5,b)"));
    let part = program.build_model(hem_registry).unwrap();
    assert_valid(&part);
}

/// The operations of [`registry`], and the hem.
fn hem_registry() -> GeopResult<OperationRegistry> {
    registry()?.with(crate::Hem)
}

/// A round hole centred on a bend (a vent across a fold): a 100 x 60
/// plate 1 thick, bend radius 1, a flange 30 high up its edge along `x`,
/// then a sheet cut of a circle of radius 5 centred at `(50, 2)` — on the
/// bend's line, where the plate meets the bend.
///
/// A circle is drawn as four quarter arcs, their ends at its left, right,
/// top and bottom; centred on the line, two of those joints lie on it.
/// The layout looked for crossings only between the ends of the cut's
/// curves, and refused one at a joint ("crosses, at an end of one of
/// them"). A joint on an edge now splits it, where the loop arrives from
/// one side of the edge and leaves to the other (`joint_on`).
#[test]
fn round_hole_centred_on_a_bend_line() {
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
                corner_gap: 1.0,
                ..SheetMetalRules::default()
            },
            depth: 1.0.into(),
            flip: false,
        },
    );
    program.push("f1", flange("base_flange(plate,k,c4,b)"));
    program.push(
        "c",
        AddSketchArgs::on(on_origin(FrameAxis::Z), disc([50.0, 2.0], 5.0)),
    );
    program.push(
        "sc",
        crate::SheetCutArgs {
            sketch: whole_sketch("c"),
            face: None,
        },
    );
    let part = program
        .build_model(|| registry()?.with(crate::SheetCut))
        .unwrap();
    assert_valid(&part);
}
