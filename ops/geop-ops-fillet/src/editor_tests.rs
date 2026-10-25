//! Fillets and chamfers set up as the front end sets them up: edges and
//! vertices picked by clicks on the drilled box, radii set in the dialog,
//! and the program seeked back before a fillet and forward again.

use geop_core_editor::{
    editing::{Control, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{built, click_at, dialog, editor_with, handled, pointer},
    view::drawn,
};
use geop_core_program::testing::args;
use geop_lib_fixtures::drilled_box;
use geop_ops_model::testing::{check_valid, model_of};

/// What these tests edit: the drilled box's given solids, and fillets.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_lib_fixtures::register)
        .and_then(crate::register)
        .expect("the operations each have a name of their own")
}

/// An editor of [`registry`] on the drilled box.
fn editor() -> Editor {
    editor_with(registry(), drilled_box())
}

/// Seeking back before a fillet and forward again runs the steps up to the
/// marker each time, the fillet's part shown only once it runs.
#[test]
fn seeking_back_before_a_fillet() {
    let mut editor = editor();
    let before = model_of(&built(&editor).unwrap());
    let rim = before
        .topology()
        .edges
        .iter()
        .find(|(_, e)| e.curve.as_arc().unwrap().is_some())
        .map(|(&id, _)| before.name_of(id).unwrap().to_string())
        .unwrap();
    let mut program = editor.program().clone();
    program.push("round", crate::FilletArgs::constant(vec![rim], 0.1));
    let steps = program.steps.len();
    handled(
        &mut editor,
        Command::Load {
            program,
            path: None,
        },
    );
    for marker in [Some(steps - 1), Some(1), None, Some(steps - 1)] {
        let update = editor.handle(Command::Seek { marker });
        assert!(update.error.is_none(), "{marker:?}: {:?}", update.error);
        let state = update.program.expect("the program is sent");
        assert_eq!(state.marker, marker.unwrap_or(steps), "{marker:?}");
        for (i, step) in state.steps.iter().enumerate() {
            assert_eq!(step.runs, i < state.marker, "{marker:?}: step {i}");
            assert!(
                step.error.is_none(),
                "{marker:?}: step {i}: {:?}",
                step.error
            );
        }
        let scene = update.scene.expect("the scene is sent anew");
        let rounded = scene.part.solids.iter().any(|s| s.starts_with("fillet("));
        assert_eq!(
            rounded,
            state.marker == steps,
            "{marker:?}: {:?}",
            scene.part.solids
        );
    }
}

/// A fillet whose radius varies, set up as the front end does: the box's
/// upright edge at the origin picked, "variable radius" ticked — which
/// brings up the end radius, starting at the radius — the end radius set,
/// then the corner on top picked as a vertex to give a radius of its own,
/// which brings up a number for it. The step builds, rounded.
#[test]
fn variable_fillet_set_up_in_the_dialog() {
    let mut editor = editor();
    editor.handle(Command::New {
        kind: "fillet".into(),
    });
    // Diagonally onto the upright edge at the origin, halfway up.
    handled(
        &mut editor,
        click_at(pointer([-5.0, -5.0, 0.5], [1.0, 1.0, 0.0])),
    );
    let update = editor.handle(dialog("variable", Value::Bool(true)));
    let step = update.step.expect("the fillet is edited");
    let number = |step: &geop_core_editor::editor::StepState, key: &str| match step
        .presentation
        .dialog
        .get(key)
    {
        Some(Control::Number(n)) => n.value,
        other => panic!("{key}: {other:?}"),
    };
    assert_eq!(number(&step, "end_radius"), number(&step, "radius"));
    editor.handle(dialog("end_radius", Value::Number(0.2)));
    // The corner on top of that edge, picked from above and outside.
    editor.handle(dialog("radius_vertices", Value::Press));
    let update = handled(
        &mut editor,
        click_at(pointer([-5.0, -5.0, 6.0], [1.0, 1.0, -1.0])),
    );
    let step = update.step.expect("the fillet is edited");
    assert_eq!(number(&step, "vertex_radius_0"), number(&step, "radius"));
    handled(&mut editor, dialog("vertex_radius_0", Value::Number(0.15)));
    handled(&mut editor, Command::Commit);
    let program = editor.program();
    let last = program.steps.last().unwrap();
    let args = args::<crate::FilletArgs>(last);
    assert_eq!(args.edges.len(), 1, "{args:?}");
    assert_eq!(args.end_radius, Some(0.2.into()), "{args:?}");
    assert_eq!(args.vertex_radii.len(), 1, "{args:?}");
    assert_eq!(args.vertex_radii[0].radius, 0.15, "{args:?}");
    let part = model_of(&built(&editor).unwrap());
    assert!(
        part.solid_names().iter().any(|s| s.starts_with("fillet(")),
        "{:?}",
        part.solid_names()
    );
}

/// The three edges at the box's corner `(0, 0, 1)` picked one after the
/// other in a new fillet, as the front end does: the edges field holds all
/// three, and the step builds the box with that corner rounded by a ball's
/// piece, named after the corner's vertex.
#[test]
fn fillet_rounds_a_corner_picked_edge_by_edge() {
    let mut editor = editor();
    editor.handle(Command::New {
        kind: "fillet".into(),
    });
    // The upright edge, diagonally halfway up; the top's edges along x and
    // y, from above and outside.
    for (origin, dir) in [
        ([-5.0, -5.0, 0.5], [1.0, 1.0, 0.0]),
        ([0.5, -5.0, 6.0], [0.0, 1.0, -1.0]),
        ([-5.0, 0.5, 6.0], [1.0, 0.0, -1.0]),
    ] {
        handled(&mut editor, click_at(pointer(origin, dir)));
    }
    handled(&mut editor, dialog("radius", Value::Number(0.2)));
    handled(&mut editor, Command::Commit);
    let program = editor.program();
    let args: crate::FilletArgs = args(program.steps.last().unwrap());
    assert_eq!(args.edges.len(), 3, "{args:?}");
    let part = model_of(&built(&editor).unwrap());
    let corners: Vec<&str> = part
        .topology()
        .faces
        .keys()
        .filter_map(|&f| part.name_of(f))
        .filter(|name| name.ends_with(",corner)"))
        .collect();
    assert_eq!(corners.len(), 1, "{corners:?}");
    assert!(check_valid(&part).is_ok());
}

/// A new fillet waits for its edges to be picked, as edges.
#[test]
fn new_fillet_picks_edges() {
    let mut editor = editor();
    for kind in ["fillet", "chamfer"] {
        let update = editor.handle(Command::New { kind: kind.into() });
        let step = update.step.expect("a step is edited");
        assert_eq!(step.missing, ["edges"]);
        assert_eq!(step.presentation.pickable, [drawn::EDGE]);
        editor.handle(Command::Cancel);
    }
}
