//! A SubD cage shaped in the viewport as the front end shapes it: its
//! faces clicked, extruded from the dialog, and moved, turned and scaled
//! by their handles and the gizmo.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{Control, GizmoPart, Shape, StepEditEvent, Style, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{dialog, drag_gizmo, event, from_above, pointer},
};
use geop_core_program::testing::args;

/// What these tests edit: SubD bodies, beside the origin.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(crate::register)
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
    let program = editor.program().clone();
    let last = program.steps.last().unwrap();
    let args: crate::SubdArgs = args(last);
    let top = args.cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = args.cage.vertex(v).unwrap().at;
        assert!((at[2] - 2.0).abs() < 1e-9, "{at:?}");
    }
    assert_eq!(args.cage.faces.len(), 10);
    let scene = editor
        .handle(Command::Show)
        .scene
        .expect("the part changed");
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

/// A click at `pointer`, double or not, shift held or not.
fn click(pointer: geop_core_editor::editing::Pointer, double: bool, shift: bool) -> StepEditEvent {
    StepEditEvent::Click {
        pointer,
        double,
        shift,
    }
}

/// The arguments of the step `editor` edits, as its edits left them.
fn editing_args(editor: &Editor) -> crate::SubdArgs {
    let step = editor.editing().expect("a step is edited");
    serde_json::from_value(step.args.clone()).expect("subd arguments")
}

/// The keys of the visuals `step` shows selected, sorted.
fn selection(step: &geop_core_editor::editor::StepState) -> Vec<String> {
    let mut keys: Vec<String> = (step.presentation.visuals.iter())
        .filter(|v| v.style == Style::Selected)
        .map(|v| v.key.clone())
        .collect();
    keys.sort();
    keys
}

/// Edges are selected by their loops and rings: a double click on the box's
/// vertical edge `e0-4` with shift held selects its ring — all four vertical
/// edges — and a loop cut across them, pressed once, cuts the ring once:
/// four new vertices at `z = 0`, ten faces. A double click on one of the
/// cut's edges selects the whole loop of four. The cut built is a valid
/// solid.
#[test]
fn subd_loops_and_rings_are_selected_by_double_click() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    editor.handle(Command::New {
        kind: "subd".into(),
    });
    // From the front, at the box's left front vertical edge.
    let side = pointer([-1.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    let step = event(&mut editor, click(side, false, false));
    assert_eq!(selection(&step), ["e0-4"]);
    let step = event(&mut editor, click(side, true, true));
    assert_eq!(selection(&step), ["e0-4", "e1-5", "e2-6", "e3-7"]);

    let step = event(
        &mut editor,
        StepEditEvent::Dialog {
            key: "edit".into(),
            value: Value::Choice("loop_cut".into()),
        },
    );
    assert_eq!(step.error, None);
    let args = editing_args(&editor);
    assert_eq!(args.cage.faces.len(), 10);
    let cut: Vec<[u32; 2]> = (args.cage.edges().into_iter())
        .filter(|&[a, b]| a >= 14 && b >= 14)
        .collect();
    assert_eq!(cut.len(), 4);
    // The edge's middle is on a side of the box, one unit out: look at it
    // from ten units further out.
    let [a, b] = cut[0];
    let (p, q) = (
        args.cage.vertex(a).unwrap().at,
        args.cage.vertex(b).unwrap().at,
    );
    let mid = [0, 1, 2].map(|c| (p[c] + q[c]) / 2.0);
    let at = pointer(mid.map(|c| 10.0 * c), mid.map(|c| -c));
    event(&mut editor, click(at, false, false));
    let step = event(&mut editor, click(at, true, false));
    assert_eq!(selection(&step).len(), 4, "{:?}", selection(&step));
    assert!(selection(&step).iter().all(|k| k.starts_with("e1")));

    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let part = geop_core_editor::testing::built(&editor).unwrap();
    geop_ops_model::testing::assert_valid(&geop_ops_model::read_model(&part.current()).unwrap());
}

/// Box select: `b` arms it — a press anywhere grabs — and a drag from above
/// draws its window while it goes and, released, selects what lies within:
/// the front of the box, with `y <= 0`, is its four front vertices, its
/// four front edges and its front face. Shift adds the back face's window
/// to it.
#[test]
fn subd_box_select_takes_what_is_within_the_window() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    editor.handle(Command::New {
        kind: "subd".into(),
    });
    let step = event(&mut editor, StepEditEvent::Key { key: "b".into() });
    assert!(step.presentation.grab, "box select grabs anywhere");
    let step = event(
        &mut editor,
        StepEditEvent::Drag {
            from: from_above(-1.5, -1.5),
            to: from_above(1.5, 0.0),
            done: false,
            shift: false,
        },
    );
    assert!(
        (step.presentation.visuals.iter()).any(|v| v.key == "window"),
        "the window is drawn while dragged"
    );
    let step = event(
        &mut editor,
        StepEditEvent::Drag {
            from: from_above(-1.5, -1.5),
            to: from_above(1.5, 0.0),
            done: true,
            shift: false,
        },
    );
    assert_eq!(
        selection(&step),
        [
            "e0-1", "e0-4", "e1-5", "e4-5", "f10", "v0", "v1", "v4", "v5"
        ]
    );
    assert!(
        !step.presentation.grab,
        "box select is put down after one window"
    );

    event(&mut editor, StepEditEvent::Key { key: "b".into() });
    let step = event(
        &mut editor,
        StepEditEvent::Drag {
            from: from_above(-1.5, 0.5),
            to: from_above(1.5, 1.5),
            done: true,
            shift: true,
        },
    );
    assert_eq!(selection(&step).len(), 18);
}

/// The edits from the dialog, with their amounts typed: the top face inset
/// by 0.3 and pulled up by `e`; the top's rim creased soft at a half, then
/// one rim edge bevelled 0.2 wide; the whole committed as one valid solid
/// whose crease is saved with its sharpness. The dialog names what cannot
/// be done: bridging one face.
#[test]
fn subd_edits_are_made_from_the_dialog_with_typed_amounts() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    editor.handle(Command::New {
        kind: "subd".into(),
    });
    let step = event(&mut editor, click(from_above(0.3, 0.2), false, false));
    assert_eq!(selection(&step), ["f9"]);
    let step = event(
        &mut editor,
        StepEditEvent::Dialog {
            key: "edit".into(),
            value: Value::Choice("bridge".into()),
        },
    );
    match step.presentation.dialog.get("problem") {
        Some(Control::Text { text, .. }) => assert!(text.contains("two faces"), "{text}"),
        other => panic!("{other:?}"),
    }
    editor.handle(dialog("thickness", Value::Number(0.3)));
    editor.handle(dialog("edit", Value::Choice("inset".into())));
    editor.handle(dialog("distance", Value::Number(0.4)));
    let step = event(&mut editor, StepEditEvent::Key { key: "e".into() });
    assert_eq!(step.error, None);
    let args = editing_args(&editor);
    assert_eq!(args.cage.faces.len(), 14);
    let top = args.cage.face(9).unwrap();
    for &v in &top.vertices {
        let at = args.cage.vertex(v).unwrap().at;
        assert!(
            (at[0].abs() - 0.7).abs() < 1e-9 && (at[2] - 1.4).abs() < 1e-9,
            "{at:?}"
        );
    }

    // The top's rim: the four edges at `z = 1` on the box's outline, seen
    // from the front near a corner, clear of the inset edge behind.
    event(
        &mut editor,
        StepEditEvent::Key {
            key: "Escape".into(),
        },
    );
    let rim = pointer([0.9, -10.0, 1.0], [0.0, 1.0, 0.0]);
    let step = event(&mut editor, click(rim, false, false));
    assert_eq!(selection(&step), ["e4-5"]);
    let step = event(&mut editor, click(rim, true, false));
    assert_eq!(selection(&step).len(), 4);
    editor.handle(dialog("sharpness", Value::Number(0.5)));
    let args = editing_args(&editor);
    assert_eq!(args.cage.creases.len(), 4);
    assert_eq!(args.cage.sharpness(4, 5), 0.5);

    event(
        &mut editor,
        StepEditEvent::Key {
            key: "Escape".into(),
        },
    );
    event(&mut editor, click(rim, false, false));
    editor.handle(dialog("width", Value::Number(0.2)));
    let step = event(
        &mut editor,
        StepEditEvent::Dialog {
            key: "edit".into(),
            value: Value::Choice("bevel".into()),
        },
    );
    assert_eq!(step.presentation.dialog.get("problem"), None);
    let args = editing_args(&editor);
    assert_eq!(args.cage.faces.len(), 14 + 3);
    assert_eq!(args.cage.creases.len(), 3, "{:?}", args.cage.creases);

    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let part = geop_core_editor::testing::built(&editor).unwrap();
    let model = geop_ops_model::read_model(&part.current()).unwrap();
    geop_ops_model::testing::assert_valid(&model);
    assert_eq!(model.solid_names().len(), 1);
}

/// "Cage only" hides the limit surface: the scene's hidden names take in
/// the step's solid, and the cage stays drawn; off again, it is shown.
#[test]
fn subd_cage_only_hides_the_limit_surface() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = editor.handle(Command::New {
        kind: "subd".into(),
    });
    let id = editor.editing().expect("a step is edited").id.clone();
    let solid = format!("subd({id})");
    let scene = update.scene.expect("the new step is drawn");
    assert!(!scene.hidden.contains(&solid));
    let update = editor.handle(dialog("cage_only", Value::Bool(true)));
    let scene = update.scene.expect("what is hidden changed");
    assert!(scene.hidden.contains(&solid), "{:?}", scene.hidden);
    let step = update.step.expect("the subd is edited");
    assert!((step.presentation.visuals.iter()).any(|v| v.key == "f9"));
    let update = editor.handle(dialog("cage_only", Value::Bool(false)));
    let scene = update.scene.expect("what is hidden changed");
    assert!(!scene.hidden.contains(&solid));
}
