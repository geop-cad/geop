//! A SubD cage shaped in the viewport as the front end shapes it: its
//! faces clicked, extruded from the dialog, and moved, turned and scaled
//! by their handles and the gizmo.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{Button, Control, GizmoPart, Shape, StepEditEvent, Style, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{dialog, drag_gizmo, event, from_above, pointer},
};
use geop_core_math::scalars::Scalar;
use geop_core_program::testing::args;

/// What these tests edit: SubD bodies, beside the origin.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(crate::register)
        .and_then(geop_ops_parameters::register)
        .expect("the operations each have a name of their own")
}

/// A new SubD step starts from a box cage, its vertices, edges and faces
/// drawn to click. Clicked from above, the cage's top face is selected —
/// not the bottom one behind it — and its centre's coordinates shown;
/// Extrude pulls it out; dragged by its gizmo's `z` arrow and then by the
/// face itself, it moves; and the limit body the editor shows is the solid the
/// step builds.
#[test]
fn subd_cage_is_shaped_in_the_viewport() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = editor.handle(Command::New {
        kind: "subd".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the subd is edited");
    for key in ["f9", "e4-5", "v6"] {
        assert!(
            step.presentation.visuals.iter().any(|v| v.key == key),
            "{key} is not drawn"
        );
    }
    let click = |pointer| Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let number = |step: &geop_core_editor::editor::StepState, key: &str| match step
        .presentation
        .dialog
        .get(key)
    {
        Some(Control::Number(n)) => n.value,
        other => panic!("{key}: {other:?}"),
    };
    let style = |step: &geop_core_editor::editor::StepState, key: &str| {
        step.presentation
            .visuals
            .iter()
            .find(|v| v.key == key)
            .map(|v| v.style)
    };
    let update = editor.handle(click(from_above(0.3, 0.2)));
    let step = update.step.expect("the subd is edited");
    assert_eq!(style(&step, "f9"), Some(Style::Selected));
    assert_eq!(style(&step, "f8"), Some(Style::Region));
    assert_eq!(number(&step, "z"), 1.0);

    let update = editor.handle(dialog("edit", Value::Choice("extrude".into())));
    let step = update.step.expect("the subd is edited");
    assert_eq!(step.error, None);
    assert_eq!(number(&step, "z"), 1.5);

    // The gizmo's `z` arrow, seen from the side, pulled up by a quarter —
    // shift held, so not snapped.
    let side = |dz: f64| pointer([0.0, -10.0, 1.55 + dz], [0.0, 1.0, 0.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: side(0.0),
            shift: false,
        },
    });
    assert!(update.step.unwrap().presentation.grab);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: side(0.0),
            to: side(0.25),
            done: true,
            shift: true,
        },
    });
    let step = update.step.expect("the subd is edited");
    assert!((number(&step, "z") - 1.75).abs() < 1e-9);

    // The face itself, grabbed looking down at 45 degrees and dragged in
    // the plane facing the eye: moving the eye up by `d` moves it by `d / 2`
    // up and `d / 2` back.
    let slanted = |dz: f64| pointer([0.3, -10.0, 11.75 + dz], [0.0, 1.0, -1.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: slanted(0.0),
            shift: false,
        },
    });
    assert!(
        update.step.unwrap().presentation.grab,
        "the face offers a grab"
    );
    for (dz, done) in [(0.2, false), (0.5, true)] {
        editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: slanted(0.0),
                to: slanted(dz),
                done,
                shift: false,
            },
        });
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let last = program.steps.last().unwrap();
    let args: crate::SubdArgs = args(last);
    let top = args.cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = args.cage.vertex(v).unwrap().at;
        assert!((at[2] - 2.0).abs() < 1e-9, "{at:?}");
    }
    assert_eq!(args.cage.faces.len(), 10);
    let scene = update.scene.expect("the part changed");
    assert_eq!(scene.part.solids, [format!("subd({})", last.id)]);
}

/// A drag of the subd cage's top face whose release never arrives, and
/// then another drag of it, started elsewhere: the second is a new drag,
/// and moves the face on from where the first left it — it neither carries
/// on the first, nor starts from the cage as it was before the first.
#[test]
fn subd_drag_without_a_release_is_not_undone_by_the_next() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = editor.handle(Command::New {
        kind: "subd".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    // Looking down at 45 degrees, at `x` across: moving the eye up by `d`
    // moves the face by `d / 2` up and `d / 2` back.
    let slanted = |x: f64, dz: f64| pointer([x, -10.0, 11.75 + dz], [0.0, 1.0, -1.0]);
    for (x, done) in [(0.3, false), (-0.3, true)] {
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Hover {
                pointer: slanted(x, 0.0),
                shift: false,
            },
        });
        assert!(
            update.step.unwrap().presentation.grab,
            "the face offers a grab"
        );
        let update = editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: slanted(x, 0.0),
                to: slanted(x, 0.2),
                done,
                shift: false,
            },
        });
        assert!(update.error.is_none(), "{:?}", update.error);
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let args: crate::SubdArgs = args(program.steps.last().unwrap());
    let top = args.cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = args.cage.vertex(v).unwrap().at;
        assert!((at[2] - 1.2).abs() < 1e-9, "{at:?}");
    }
}

/// The subd cage's top face, selected, moved up by its gizmo's `z` arrow,
/// turned by its ring about `z` and stretched along `x` by the cube beyond
/// the `x` arrow — about the face's centre, each snapped: to the grid, to
/// 15 degrees, to a tenth.
#[test]
fn subd_faces_are_moved_turned_and_scaled_by_the_gizmo() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    editor.handle(Command::New {
        kind: "subd".into(),
    });
    let step = event(
        &mut editor,
        StepEditEvent::Click {
            pointer: from_above(0.3, 0.2),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    );
    let gizmo = step.presentation.gizmo.expect("the selection has a gizmo");
    assert!(gizmo.modes.translate && gizmo.modes.rotate && gizmo.modes.scale);
    assert_eq!(
        [0, 1, 2].map(|k| gizmo.at[k].to_f64()),
        [0.0, 0.0, 1.0],
        "at the face's centre"
    );
    // A face has an orientation of its own; the world's is chosen.
    assert!(
        step.presentation
            .dialog
            .get(geop_core_editor::step_editor::GIZMO_ORIENTATION)
            .is_some()
    );
    editor.handle(dialog(
        geop_core_editor::step_editor::GIZMO_ORIENTATION,
        Value::Choice("world".into()),
    ));
    // No handles any more: the gizmo is what drags.
    assert!(
        !step
            .presentation
            .visuals
            .iter()
            .any(|v| matches!(v.shape, Shape::Handle { .. }))
    );

    // Seen from the side, the `z` arrow pulled up by 0.31: a reach of
    // 0.009 snaps to fiftieths.
    let side = |z: f64| pointer([0.0, -10.0, z], [0.0, 1.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(geop_core_editor::editing::Axis::Z),
        side(1.05),
        side(1.355),
        side(1.2),
        false,
    );
    // From above, the ring about `z` turned from 45 to 90 degrees round.
    let reach = 0.009;
    let round = |degrees: f64| {
        let (s, c) = degrees.to_radians().sin_cos();
        let r = 7.0 * reach;
        from_above(r * c, r * s)
    };
    drag_gizmo(
        &mut editor,
        GizmoPart::Turn(geop_core_editor::editing::Axis::Z),
        round(45.0),
        round(92.0),
        round(60.0),
        false,
    );
    // The cube beyond the `x` arrow pulled out to twice as far.
    let along = |x: f64| from_above(x, 0.0);
    let step = drag_gizmo(
        &mut editor,
        GizmoPart::Stretch(geop_core_editor::editing::Axis::X),
        along(12.5 * reach),
        along(25.0 * reach),
        along(20.0 * reach),
        false,
    );
    assert_eq!(step.error, None);

    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let last = editor.program().steps.last().unwrap().clone();
    let args: crate::SubdArgs = args(&last);
    // The top's corners `(±1, ±1)`, turned by 45 degrees, lie on the axes
    // √2 out; stretched along `x`, twice that along it.
    let r = 2f64.sqrt();
    let mut corners: Vec<[f64; 3]> = args
        .cage
        .face(9)
        .unwrap()
        .vertices
        .iter()
        .map(|&v| args.cage.vertex(v).unwrap().at)
        .collect();
    corners.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    let want = [
        [-2.0 * r, 0.0, 1.3],
        [0.0, -r, 1.3],
        [0.0, r, 1.3],
        [2.0 * r, 0.0, 1.3],
    ];
    for (got, want) in corners.iter().zip(want) {
        assert!(
            (0..3).all(|k| (got[k] - want[k]).abs() < 1e-9),
            "{corners:?}"
        );
    }
}
