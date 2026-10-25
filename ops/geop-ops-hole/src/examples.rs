//! The examples of holes an editor offers: a mounting plate drilled with
//! counterbored and tapped holes.

use geop_core_editor::registry::ExampleSet;
use geop_core_math::primitives::FrameAxis;
use geop_core_program::program::Program;
use geop_core_sketch::PointId;
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output};
use geop_ops_model::datums::origin_plane;
use geop_ops_model::sketch::{
    Constraint, Sketch, SketchPlane,
    figures::{P2, n, rectangle, solved},
    whole_sketch,
};
use geop_ops_model::{Combine, element};
use geop_ops_sketch::AddSketchArgs;

use crate::{HoleArgs, HoleCentre, HoleKind, Standard, iso::Fit};

/// The examples of this crate, by name.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("hole_plate", hole_plate)],
        workspaces: Vec::new(),
    }
}

/// A 60 x 40 x 10 mm mounting plate: four counterbored M5 holes through
/// its corners, at the points of one sketch on its top (`bolts`), and a
/// tapped M6 hole 8 deep in its middle (`tapped`), which carries the
/// cosmetic thread M6x1 — the result is `hole(tapped)`.
pub fn hole_plate() -> Program {
    let mut program = Program::new();
    let mut outline = Sketch::new();
    rectangle(&mut outline, [0.0, 0.0], 60.0, 40.0);
    program.push(
        "outline",
        AddSketchArgs::on(
            SketchPlane::Datum(origin_plane(FrameAxis::Z)),
            solved(outline),
        ),
    );
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(10.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    let points = |at: &[P2]| {
        let mut sketch = Sketch::new();
        let ids: Vec<PointId> = at
            .iter()
            .map(|p| sketch.add_point(n(p[0]), n(p[1])))
            .collect();
        for (id, p) in ids.into_iter().zip(at) {
            sketch.constrain(Constraint::Fix {
                point: id,
                x: n(p[0]),
                y: n(p[1]),
            });
        }
        solved(sketch)
    };
    let top = || SketchPlane::Face(element("extrude(plate,end)"));
    program.push(
        "corners",
        AddSketchArgs::on(
            top(),
            points(&[[8.0, 8.0], [52.0, 8.0], [52.0, 32.0], [8.0, 32.0]]),
        ),
    );
    program.push(
        "bolts",
        HoleArgs {
            face: Some(element("extrude(plate,end)")),
            points: vec![HoleCentre::Sketch(whole_sketch("corners"))],
            kind: HoleKind::Counterbore,
            standard: Standard::Iso {
                size: "M5".into(),
                fit: Fit::Normal,
            },
            end: Extent::ThroughAll,
            drill_point: false,
            thread_length: None,
        },
    );
    program.push("middle", AddSketchArgs::on(top(), points(&[[30.0, 20.0]])));
    program.push(
        "tapped",
        HoleArgs {
            face: Some(element("extrude(plate,end)")),
            points: vec![HoleCentre::Sketch(whole_sketch("middle"))],
            kind: HoleKind::Tapped,
            standard: Standard::Iso {
                size: "M6".into(),
                fit: Fit::Normal,
            },
            end: Extent::blind(8.0),
            drill_point: true,
            thread_length: None,
        },
    );
    program
}
