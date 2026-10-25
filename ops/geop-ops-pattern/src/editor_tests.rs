//! Patterns and moved bodies set up as the front end sets them up: what
//! is copied clicked, the spacing dragged by its handle and the body by
//! its gizmo.

use geop_core_editor::{
    editing::{Axis, Control, GizmoPart, Shape, StepEditEvent, Value},
    editor::Command,
    registry::Registry,
    testing::{click_at, dialog, drag_gizmo, editor_with, from_above, handled, pointer},
    view::drawn,
};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_program::{element::PickedElement, testing::args};
use geop_ops_extrude_revolve::examples::box_with_drill_hole;
use geop_ops_hole::testing::plate_with_hole;

/// What these tests edit: the drilled box and the drilled plate — their
/// sketches, extrusions and holes — and patterns.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_extrude_revolve::register)
        .and_then(geop_ops_hole::register)
        .and_then(crate::register)
        .and_then(geop_ops_parameters::register)
        .expect("the operations each have a name of their own")
}

/// A new linear pattern picks nothing by itself: it waits for the body or
/// the feature to copy, here the drilled box, clicked. Along `x`, its
/// spacing's handle, grabbed from above and pulled one further along `x`,
/// spreads the copies two apart; a fourth is asked for in the dialog, and
/// committed, the scene lists the drilled box and its three copies, the
/// copies named after the step.
#[test]
fn new_linear_pattern_dragged_by_its_spacing_handle() {
    let mut editor = editor_with(registry(), box_with_drill_hole());
    let update = editor.handle(Command::New {
        kind: "linear_pattern".into(),
    });
    let step = update.step.expect("the pattern is edited");
    assert_eq!(step.missing, ["bodies"], "nothing is picked by itself");
    // The box's top, beside its hole.
    let update = handled(&mut editor, click_at(from_above(0.3, 0.3)));
    let step = update.step.expect("the pattern is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let (at, direction) = step
        .presentation
        .visuals
        .iter()
        .find_map(|v| match v.shape {
            Shape::Handle { at, direction } if v.key == "spacing" => Some((at, direction)),
            _ => None,
        })
        .expect("the spacing has a handle");
    assert!(direction[0].could_be_equal(S::ONE), "{direction:?}");
    let [x, y, z] = [0, 1, 2].map(|k| at[k].to_f64());
    let above = |dx: f64| pointer([x + dx, y, z + 10.0], [0.0, 0.0, -1.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: above(0.0),
            shift: false,
        },
    });
    assert!(update.step.unwrap().presentation.grab, "no grab");
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: above(0.0),
            to: above(1.0),
            done: true,
            shift: false,
        },
    });
    handled(&mut editor, dialog("count", Value::Number(4.0)));
    let update = handled(&mut editor, Command::Commit);
    let args = args::<crate::LinearPatternArgs>(editor.program().steps.last().unwrap());
    assert_eq!(args.first.count, 4.0.into());
    match &args.first.spacing {
        crate::Spacing::Step(step) => {
            let step = step.plain().expect("a plain spacing");
            assert!((step - 2.0).abs() < 1e-9, "{step}")
        }
        other => panic!("{other:?}"),
    }
    let solids: Vec<String> = update
        .scene
        .expect("the scene changed")
        .structure
        .into_iter()
        .filter(|item| item.kind == "solid")
        .map(|item| item.name)
        .collect();
    assert_eq!(solids.len(), 4, "{solids:?}");
    let id = editor.program().steps.last().unwrap().id.clone();
    assert!(
        solids.contains(&format!("linear_pattern({id},3,extrude(hole))")),
        "{solids:?}"
    );
}

/// A new linear pattern picks the hole cut through a plate as a feature by
/// a click on the hole's wall, through the hole's mouth from above: the
/// dialog lists the feature and no body any more, hovering lights the
/// hole, and committed with four instances two apart, the scene holds the
/// plate alone, drilled four times.
#[test]
fn new_linear_pattern_picks_a_feature_by_its_wall() {
    let mut editor = editor_with(registry(), plate_with_hole());
    editor.handle(Command::New {
        kind: "linear_pattern".into(),
    });
    editor.handle(dialog("features", Value::Press));
    // From above the hole's middle down onto its wall at 45°, 0.1 above
    // the plate's bottom — clear of the rim and of the wall's seams.
    let r = 0.3 * std::f64::consts::FRAC_1_SQRT_2;
    let wall = pointer([1.0, 1.0, 3.0], [r, r, 0.1 - 3.0]);
    let hole = crate::feature("hole").picked();
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: wall,
            shift: false,
        },
    });
    let step = update.step.expect("the pattern is edited");
    // What is hovered is the wall's face, as the pointer hits it.
    let walls: Vec<PickedElement> = (step.presentation.highlights.iter())
        .filter(|e| e.kind == drawn::FACE)
        .cloned()
        .collect();
    assert_eq!(walls.len(), 1, "{:?}", step.presentation.highlights);
    let update = handled(&mut editor, click_at(wall));
    let step = update.step.expect("the pattern is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    // The hole picked is lit as its faces, the wall among them.
    assert!(
        step.presentation.highlights.contains(&walls[0]),
        "{:?}",
        step.presentation.highlights
    );
    let picked = |key: &str| match step.presentation.dialog.get(key) {
        Some(Control::Reference(r)) => r.elements().cloned().collect::<Vec<_>>(),
        other => panic!("{key}: {other:?}"),
    };
    assert_eq!(picked("features"), std::slice::from_ref(&hole));
    assert!(picked("bodies").is_empty());
    // A feature combines as it did: there is no combining to choose.
    assert!(step.presentation.dialog.get("combine").is_none());

    editor.handle(dialog("count", Value::Number(4.0)));
    editor.handle(dialog("spacing", Value::Number(2.0)));
    handled(&mut editor, Command::Commit);
    let args = args::<crate::LinearPatternArgs>(editor.program().steps.last().unwrap());
    assert_eq!(args.seeds.features, [crate::feature("hole")]);
    assert!(args.seeds.bodies.is_empty());
    let scene = editor
        .handle(Command::Seek { marker: None })
        .scene
        .expect("the scene is sent");
    let id = editor.program().steps.last().unwrap().id.clone();
    assert_eq!(scene.part.solids, [format!("linear_pattern({id})")]);
    let copied = scene
        .part
        .faces
        .iter()
        .filter(|f| f.feature.as_deref() == Some(id.as_str()))
        .count();
    assert!(copied >= 3, "the copies' faces are the pattern's: {copied}");
}

/// Move body shifts the body by a gizmo where its middle goes: its `y`
/// arrow dragged, then the square of the `x`–`z` plane.
#[test]
fn a_body_is_moved_by_its_gizmo() {
    let mut editor = editor_with(registry(), box_with_drill_hole());
    let update = editor.handle(Command::New {
        kind: "move_body".into(),
    });
    let step = update.step.expect("move body is edited");
    let gizmo = step.presentation.gizmo.expect("a gizmo");
    assert!(gizmo.modes.translate && !gizmo.modes.rotate);
    let at = [0, 1, 2].map(|k| gizmo.at[k].to_f64());
    // Looking down `x`, the `y` arrow dragged 0.3 along.
    let side = |y: f64, z: f64| pointer([at[0] - 10.0, y, z], [1.0, 0.0, 0.0]);
    drag_gizmo(
        &mut editor,
        GizmoPart::Move(Axis::Y),
        side(at[1] + 0.05, at[2]),
        side(at[1] + 0.35, at[2]),
        side(at[1] + 0.2, at[2]),
        false,
    );
    // Looking down `y` at the moved gizmo, its `x`–`z` square — the one
    // normal to `y` — dragged by (0.2, -0.4).
    let at = [at[0], at[1] + 0.3, at[2]];
    let front = |x: f64, z: f64| pointer([x, at[1] - 10.0, z], [0.0, 1.0, 0.0]);
    let corner = 0.03;
    drag_gizmo(
        &mut editor,
        GizmoPart::Plane(Axis::Y),
        front(at[0] + corner, at[2] + corner),
        front(at[0] + corner + 0.2, at[2] + corner - 0.4),
        front(at[0] + corner + 0.1, at[2] + corner),
        false,
    );
    handled(&mut editor, Command::Commit);
    let args: crate::MoveBodyArgs = args(editor.program().steps.last().unwrap());
    let want = [0.2, 0.3, -0.4];
    assert!(
        (0..3).all(|k| (args.translation[k] - want[k]).abs() < 1e-9),
        "{:?}",
        args.translation
    );
}
