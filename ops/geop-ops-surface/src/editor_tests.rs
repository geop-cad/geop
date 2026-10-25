//! Surfaces made as the front end makes them: the curves they span —
//! edges of the drilled box, lines of planar and 3-D sketches — clicked,
//! and the sheet they make thickened.

use geop_core_editor::{
    editing::{Button, StepEditEvent, Value},
    editor::Command,
    registry::Registry,
    testing::{built, click_at, dialog, editor_with, from_above, pointer},
    view::drawn,
};
use geop_core_math::{
    primitives::FrameAxis,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{program::Program, testing::args};
use geop_ops_extrude_revolve::examples::box_with_drill_hole;
use geop_ops_model::{element, testing::model_of};
use geop_ops_sketch3d::testing::route;

/// What these tests edit: sketches, planar and in space, the drilled box's
/// extrusions, and surfaces.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_sketch3d::register)
        .and_then(geop_ops_extrude_revolve::register)
        .and_then(crate::register)
        .expect("the operations each have a name of their own")
}

/// A boundary surface picks the box's four top edges, clicked from above
/// and outside, and fills them; a new thicken then starts from that sheet
/// and makes it a slab.
#[test]
fn new_boundary_surface_picks_edges_then_thickens() {
    let mut editor = editor_with(registry(), box_with_drill_hole());
    let update = editor.handle(Command::New {
        kind: "boundary_surface".into(),
    });
    assert_eq!(
        update.step.unwrap().presentation.pickable,
        [drawn::EDGE, drawn::SKETCH_CURVE]
    );
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    // Each down onto the box's top rim at 45 degrees, from outside it.
    for (origin, dir) in [
        ([0.5, -5.0, 6.0], [0.0, 1.0, -1.0]),
        ([7.0, 0.5, 6.0], [-1.0, 0.0, -1.0]),
        ([1.5, 7.0, 6.0], [0.0, -1.0, -1.0]),
        ([-5.0, 1.5, 6.0], [1.0, 0.0, -1.0]),
    ] {
        let update = editor.handle(click(pointer(origin, dir)));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = editor.program().steps.last().unwrap().clone();
    let boundary: crate::BoundarySurfaceArgs = args(&step);
    assert_eq!(boundary.edges.len(), 4, "{boundary:?}");
    let sheet = format!("boundary({})", step.id);
    let scene = update.scene.expect("the scene is sent anew");
    let faces = |scene: &geop_core_editor::editor::SceneState| -> Vec<String> {
        scene.part.faces.iter().map(|f| f.name.clone()).collect()
    };
    assert!(faces(&scene).contains(&sheet), "{:?}", faces(&scene));

    let update = editor.handle(Command::New {
        kind: "thicken".into(),
    });
    let step = update.step.expect("the thicken is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    editor.handle(dialog("thickness", Value::Number(0.2)));
    let update = editor.handle(dialog("side", Value::Choice("both".into())));
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = editor.program().steps.last().unwrap().clone();
    let thicken: crate::ThickenArgs = args(&step);
    assert_eq!(thicken.face.name(), sheet);
    assert_eq!(thicken.thickness, 0.2);
    let scene = update.scene.expect("the scene is sent anew");
    let slab = format!("thicken({})", step.id);
    assert!(scene.part.solids.contains(&slab), "{:?}", scene.part.solids);
    assert!(!faces(&scene).contains(&sheet));
}

/// A boundary surface between a planar sketch's line and a 3-D sketch's
/// line, both clicked: the ruled face between them, a valid sheet.
#[test]
fn a_boundary_surface_spans_sketch_lines() {
    let mut base = geop_ops_model::sketch::Sketch::new();
    let (a, b) = (
        base.add_point(
            geop_ops_model::sketch::figures::n(0.0),
            geop_ops_model::sketch::figures::n(0.0),
        ),
        base.add_point(
            geop_ops_model::sketch::figures::n(2.0),
            geop_ops_model::sketch::figures::n(0.0),
        ),
    );
    let line = base.add_line(a, b);
    let mut program = Program::new();
    program.push(
        "base",
        geop_ops_sketch::AddSketchArgs {
            plane: Some(geop_ops_model::sketch::SketchPlane::Datum(
                geop_ops_model::datums::origin_plane(FrameAxis::Z),
            )),
            sketch: base,
            ..Default::default()
        },
    );
    program.push("route", route());
    let mut editor = editor_with(registry(), program);
    editor.handle(Command::New {
        kind: "boundary_surface".into(),
    });
    editor.handle(click_at(from_above(1.0, 0.004)));
    editor.handle(click_at(from_above(2.0, 1.004)));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: crate::BoundarySurfaceArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.edges,
        [
            geop_ops_sketch3d::CurveSource::Sketch(geop_ops_model::sketch::sketch_element(
                "base", line
            )),
            geop_ops_sketch3d::CurveSource::Edge(element("sketch3d(route,c2)"))
        ]
    );
    let part = model_of(&built(&editor).unwrap());
    assert!(part.face_id("boundary(boundary_surface1)").is_ok());
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// Two lines of a 3-D sketch, `(x, y, z) = (s, 0.5, 0)` and `(s, 1.5, 1)`
/// for `s` from -0.5 to 2.5 — or, with `across`, lines
/// `(0, s, s - 0.5)` and `(2, s, s - 0.5)` crossing them, run downwards.
fn network_lines(across: bool) -> geop_ops_sketch3d::AddSketch3dArgs {
    let mut s = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| Vector3::from_array(p.map(geop_ops_model::sketch::figures::n));
    let lines: [[[f64; 3]; 2]; 2] = if across {
        [
            [[0.0, 2.5, 2.0], [0.0, -0.5, -1.0]],
            [[2.0, 2.5, 2.0], [2.0, -0.5, -1.0]],
        ]
    } else {
        [
            [[-0.5, 0.5, 0.0], [2.5, 0.5, 0.0]],
            [[-0.5, 1.5, 1.0], [2.5, 1.5, 1.0]],
        ]
    };
    for [a, b] in lines {
        let (a, b) = (s.add_point(at(a)), s.add_point(at(b)));
        s.add_line(a, b);
    }
    geop_ops_sketch3d::AddSketch3dArgs {
        sketch: s,
        references: Vec::new(),
    }
}

/// A UV surface through two 3-D sketches' lines, crossing each other: the
/// u curves clicked first, then the v field pressed and the v curves
/// clicked. The sheet spans the grid between them, a valid face.
#[test]
fn a_uv_surface_picks_its_curves_by_clicks() {
    let mut program = Program::new();
    program.push("along", network_lines(false));
    program.push("across", network_lines(true));
    let mut editor = editor_with(registry(), program);
    let update = editor.handle(Command::New {
        kind: "network_surface".into(),
    });
    assert_eq!(
        update.step.unwrap().presentation.pickable,
        [drawn::EDGE, drawn::SKETCH_CURVE]
    );
    for (x, y) in [(1.0, 1.504), (1.0, 0.504)] {
        let update = editor.handle(click_at(from_above(x, y)));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    editor.handle(dialog("v_curves", Value::Press));
    for (x, y) in [(2.004, 1.0), (0.004, 1.0)] {
        let update = editor.handle(click_at(from_above(x, y)));
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: crate::NetworkSurfaceArgs = args(editor.program().steps.last().unwrap());
    let edge = |name: &str| geop_ops_sketch3d::CurveSource::Edge(element(name));
    assert_eq!(
        args.u_curves,
        [edge("sketch3d(along,c5)"), edge("sketch3d(along,c2)")]
    );
    assert_eq!(
        args.v_curves,
        [edge("sketch3d(across,c5)"), edge("sketch3d(across,c2)")]
    );
    let scene = update.scene.expect("the scene is sent anew");
    let faces: Vec<&str> = scene.part.faces.iter().map(|f| f.name.as_str()).collect();
    assert!(faces.contains(&"network(uv_surface1)"), "{faces:?}");
    let part = model_of(&built(&editor).unwrap());
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
    // Its corner where the first u curve picked meets the first v curve.
    let corner = part
        .vertex_id("network(uv_surface1,sketch3d(along,c5),sketch3d(across,c5))")
        .unwrap();
    let p = part.topology().get_vertex(corner).unwrap().point;
    assert!(p.could_be_equal(&Vector3::from_array([2.0, 1.5, 1.0].map(S::from_f64))));
}
