//! The examples of surfaces an editor offers: a doubly curved panel
//! through a network of curves, a block split along it.

use geop_core_editor::registry::ExampleSet;
use geop_core_math::{primitives::FrameAxis, vector::Vector3};
use geop_core_program::program::Program;
use geop_ops_boolean::SplitArgs;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs, Output};
use geop_ops_model::datums::origin_plane;
use geop_ops_model::sketch::{
    Sketch, SketchPlane,
    figures::{n, rectangle, solved},
    whole_sketch,
};
use geop_ops_model::sketch3d::{CurveSource, Sketch3d};
use geop_ops_model::{Combine, element, solid};
use geop_ops_sketch::AddSketchArgs;
use geop_ops_sketch3d::AddSketch3dArgs;

use crate::NetworkSurfaceArgs;

/// The examples of this crate, by name.
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("curved_panel", curved_panel)],
        workspaces: Vec::new(),
    }
}

/// A doubly curved panel, a saddle: a UV surface (`panel`) through three
/// ribs along `x` and three spines along `z`, each a spline of a 3-D
/// sketch through points of the saddle — the ribs' and the spines' meeting
/// where they cross; and a block under it (`block`, up along `y`) split
/// along it (`shaped`) into a piece with the saddle for its top and one
/// with it for its bottom. Drawn in tens of millimetres: the panel is 40 by
/// 30.
pub fn curved_panel() -> Program {
    let height = |x: f64, z: f64| 0.6 + 0.08 * (x - 2.0).powi(2) - 0.12 * (z - 1.5).powi(2);
    let mut program = Program::new();
    let mut splines = |name: &str, lines: Vec<Vec<[f64; 2]>>| -> Vec<CurveSource> {
        let mut sketch = Sketch3d::new();
        let mut curves = Vec::new();
        for line in lines {
            let points = line
                .iter()
                .map(|&[x, z]| {
                    let at = [x, height(x, z), z].map(|c| n(10.0 * c));
                    sketch.add_point(Vector3::from_array(at))
                })
                .collect();
            let curve = sketch.add_spline(points);
            curves.push(CurveSource::Edge(element(format!(
                "sketch3d({name},{curve})"
            ))));
        }
        program.push(
            name,
            AddSketch3dArgs {
                sketch,
                references: Vec::new(),
            },
        );
        curves
    };
    let ribs = splines(
        "ribs",
        [0.0, 1.5, 3.0]
            .iter()
            .map(|&y| [0.0, 1.0, 2.0, 3.0, 4.0].map(|x| [x, y]).to_vec())
            .collect(),
    );
    let spines = splines(
        "spines",
        [0.0, 2.0, 4.0]
            .iter()
            .map(|&x| [0.0, 0.75, 1.5, 2.25, 3.0].map(|y| [x, y]).to_vec())
            .collect(),
    );
    program.push(
        "panel",
        NetworkSurfaceArgs {
            u_curves: ribs,
            v_curves: spines,
        },
    );
    let mut outline = Sketch::new();
    // Sketch `y` runs along world `-z`.
    rectangle(&mut outline, [5.0, -25.0], 30.0, 20.0);
    program.push(
        "block_sketch",
        AddSketchArgs {
            plane: Some(SketchPlane::Datum(origin_plane(FrameAxis::Y))),
            sketch: solved(outline),
            ..Default::default()
        },
    );
    program.push(
        "block",
        ExtrudeArgs {
            sketch: whole_sketch("block_sketch"),
            extent: Extents::blind(15.0),
            output: Output::Solid(Combine::NewBody),
            draft: 0.0,
        },
    );
    program.push(
        "shaped",
        SplitArgs {
            solid: solid("extrude(block)"),
            face: element("network(panel)"),
        },
    );
    program
}
