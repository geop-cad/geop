//! Extrusions, revolutions and sweeps edited as the front end edits them:
//! every gesture a command to the [`geop_core_editor::editor::Editor`] —
//! handles dragged, axes and rails picked, formulas typed into dialog
//! fields — and the parametric plate's parameters set, renamed and undone
//! through the parameters panel, the part built anew with them (only where
//! they are read); and what the editor offers around them: the steps it
//! runs while one is edited, and inspectors.

use std::{borrow::Cow, collections::BTreeMap};

use geop_core_editor::{
    editing::{Button, Control, Pointer, Reach, Shape, StepEditEvent, Style, Tone, Value, Visual},
    editor::{Command, Editor, ProgramState, Update},
    output::{Inspector, InspectorInfo, Output as Made, Picks},
    registry::{Panel, Registry},
    testing::{built, click_at, dialog, editor_with, from_above, handled, pointer},
    view::drawn,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{FrameAxis, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{
    element::PickedElement, operation::Operation, reader::Reader, values::Setting,
};
use geop_core_sketch::PointId;
use geop_ops_datums::origin_axis;
use geop_ops_model::{
    BooleanOp, Combine, Face, Point, element, read_model,
    testing::{model_shown, solid_volume},
};
use geop_ops_parameters::{
    DefineParameters, Parameter, ParameterKind, Parameters, ParametersCommand, ParametersPanel,
    ParametersPanelState, parameters::Row, parameters_of,
};
use geop_ops_sketch::{AddSketchArgs, Constraint, sketch_element, whole_sketch};

use crate::{
    ExtrudeArgs, Output,
    examples::parametric_plate,
    testing::{Built, args, model_of, registry as operations},
};

/// What these tests edit: sketches — planar and in space — on the origin's
/// datums, swept into solids, and the parameters their dialogs read: the
/// model's, the sketches', the datums', the extrusions' and the
/// parameters' operations, drawers and panels.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_datums::register)
        .and_then(geop_ops_sketch3d::register)
        .and_then(crate::register)
        .and_then(geop_ops_parameters::register)
        .expect("the operations each have a name of their own")
}

/// The command setting the program's parameters to `parameters`, as the
/// parameters panel sends it.
fn set_parameters(parameters: Parameters) -> Command {
    Command::panel::<ParametersPanel>(&ParametersCommand::SetParameters { parameters })
        .expect("parameters are plain data")
}

/// What the parameters panel shows of `program`.
fn parameters_panel(program: &ProgramState) -> ParametersPanelState {
    serde_json::from_value(program.panels[ParametersPanel::KEY].clone())
        .expect("the parameters panel shows its state")
}

/// The command renaming the parameter `from` `to`, as the parameters panel
/// sends it.
fn rename_parameter(from: &str, to: &str) -> Command {
    let (from, to) = (from.into(), to.into());
    Command::panel::<ParametersPanel>(&ParametersCommand::RenameParameter { from, to })
        .expect("names are plain data")
}

/// The point `p2` of the sketch `outline` of the part `view`, in space.
fn outline_corner(view: &Reader) -> [f64; 3] {
    let corner = view.retrieve_element(&sketch_element::<Point>("outline", PointId(2)));
    let p = corner.unwrap();
    [0, 1, 2].map(|k| p[k].to_f64())
}

/// The program's parameters are edited as a whole: the part is built
/// anew with them, what they resolve to — and why one does not — is
/// shown, invalid ones are refused, and the edit is undone like any other.
#[test]
fn parameters_are_edited_and_undone() {
    let program = parametric_plate();
    let mut editor = editor_with(registry(), program.clone());
    let mut parameters = parameters_of(&program);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[1].kind else {
        panic!("depth is a number")
    };
    *expression = "width / 4 + nothing".into();
    let update = editor.handle(set_parameters(parameters.clone()));
    let state = update.program.unwrap();
    assert!(
        parameters_panel(&state).parameters.errors["depth"].contains("nothing"),
        "{state:?}"
    );
    let ParameterKind::Number { expression, .. } = &mut parameters.values[1].kind else {
        unreachable!()
    };
    *expression = "width / 4".into();
    let state = (editor.handle(set_parameters(parameters.clone())).program).unwrap();
    assert_eq!(
        (parameters_panel(&state).parameters.values)
            .get("depth")
            .cloned(),
        Some(Setting::Number(1.0))
    );
    assert!(
        state.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        state.steps
    );
    parameters.values[1].name = "width".into();
    assert!(editor.handle(set_parameters(parameters)).error.is_some());
    editor.handle(Command::Undo);
    editor.handle(Command::Undo);
    assert_eq!(parameters_of(editor.program()), parameters_of(&program));
}

/// A parameter changed rebuilds the part — with no step edited, and with
/// one edited: the sketch open follows its formulas at once, solved, and
/// what it builds follows once it is committed. A table added on the fly
/// is read by its columns in a formula typed into a dimension.
#[test]
fn parameters_changed_rebuild_what_reads_them() {
    let program = parametric_plate();
    let mut editor = editor_with(registry(), program.clone());
    let corner_x = |view: Reader| outline_corner(&view)[0];
    assert!((corner_x(editor.part()) - 4.0).abs() < 1e-9);
    // Its state is where its parts are — it places none.
    assert!(
        editor.program().state.is_empty(),
        "{:?}",
        editor.program().state
    );
    let mut parameters = parameters_of(&program);
    let set_width = |parameters: &mut Parameters, w: &str| {
        let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
            panic!("width is a number")
        };
        *expression = w.into();
    };
    set_width(&mut parameters, "5");
    let update = handled(&mut editor, set_parameters(parameters.clone()));
    let widest_vertex = update
        .scene
        .as_ref()
        .expect("the part changed")
        .part
        .vertices
        .iter()
        .map(|v| v.at[0].to_f64())
        .fold(f64::MIN, f64::max);
    assert!((widest_vertex - 5.0).abs() < 1e-9, "drawn: {widest_vertex}");
    let state = update.program.unwrap();
    assert!(
        state.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        state.steps
    );
    let built = editor.program().built(operations).unwrap();
    let corner_built = corner_x(built.current());
    assert!(
        (corner_built - 5.0).abs() < 1e-9,
        "the program itself: {corner_built}"
    );
    let sketch_of = |view: Reader| {
        let placed = view.retrieve_element(&whole_sketch("outline"));
        placed.unwrap().sketch.clone()
    };
    assert!(
        (corner_x(editor.part()) - 5.0).abs() < 1e-9,
        "{}: {:?} vs built {:?}",
        corner_x(editor.part()),
        sketch_of(editor.part()).points.get(&PointId(2)),
        sketch_of(built.current()).points.get(&PointId(2)),
    );

    // Editing the outline, the width changes: the drawing follows, solved.
    editor.handle(Command::Open {
        id: "outline".into(),
    });
    set_width(&mut parameters, "7");
    let update = editor.handle(set_parameters(parameters.clone()));
    let visuals = update.step.unwrap().presentation.visuals;
    let widest = visuals
        .iter()
        .filter_map(|v| match &v.shape {
            Shape::Point { at } if v.key.starts_with('p') => Some(at[0].to_f64()),
            _ => None,
        })
        .fold(f64::MIN, f64::max);
    assert!((widest - 7.0).abs() < 1e-6, "{widest}");
    editor.handle(Command::Commit);
    assert!(
        (corner_x(editor.part()) - 7.0).abs() < 1e-9,
        "{}",
        corner_x(editor.part())
    );

    // A table, added while the hole's sketch is edited, read in a formula.
    editor.handle(Command::Open {
        id: "hole_sketch".into(),
    });
    parameters.values.push(Parameter {
        name: "size".into(),
        kind: ParameterKind::Table {
            columns: vec!["diameter".into()],
            rows: vec![
                Row {
                    name: "small".into(),
                    values: vec![0.3],
                },
                Row {
                    name: "large".into(),
                    values: vec![0.5],
                },
            ],
            selected: "large".into(),
        },
    });
    handled(&mut editor, set_parameters(parameters.clone()));
    let sketch: AddSketchArgs = args(editor.editing().unwrap());
    let (&diameter, _) = sketch
        .sketch
        .constraints
        .iter()
        .find(|(_, c)| matches!(c, Constraint::Diameter { .. }))
        .unwrap();
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Dialog {
            key: format!("constraint:{}", diameter.0),
            value: Value::Text("size.diameter".into()),
        },
    });
    let step = update.step.unwrap();
    let hint = step.presentation.dialog.get("hint");
    assert!(
        !matches!(
            hint,
            Some(Control::Text {
                tone: Tone::Error,
                ..
            })
        ),
        "{hint:?}"
    );
    let sketch: AddSketchArgs = args(editor.editing().unwrap());
    assert_eq!(sketch.formulas[&diameter], "size.diameter".into());
    handled(&mut editor, Command::Commit);
}

/// Picking a colour answers at once — the part is drawn again in it, but
/// nothing is built again.
#[test]
fn colours_are_picked_without_rebuilding() {
    let program = parametric_plate();
    let mut editor = editor_with(registry(), program.clone());
    // What the program built: kept, not built again, it is the very same.
    let model = |editor: &Editor| match read_model(&editor.part()) {
        Ok(Cow::Borrowed(built)) => built as *const _ as usize,
        other => panic!("no model built: {:?}", other.err()),
    };
    let before = model(&editor);
    let mut parameters = parameters_of(&program);
    parameters.color = Some("#123456".into());
    let update = editor.handle(set_parameters(parameters));
    assert_eq!(
        update
            .scene
            .expect("drawn in the new colour")
            .part
            .color
            .as_deref(),
        Some("#123456")
    );
    // Not timed: drawn again, but no step built again.
    assert_eq!(model(&editor), before, "picking a colour rebuilt steps");
}

/// An editor of [`registry`] on the drilled box, and what loading it sent.
fn editor() -> (Editor, Update) {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = handled(
        &mut editor,
        Command::Load {
            program: crate::examples::box_with_drill_hole(),
            path: None,
        },
    );
    (editor, update)
}

fn distance(editor: &Editor, id: &str) -> f64 {
    let program = editor.program();
    let args: ExtrudeArgs = args(&program.steps[program.index_of(id).unwrap()]);
    match &args.extent.side1 {
        crate::Extent::Blind(d) => d.plain().expect("a plain distance"),
        other => panic!("{other:?} is no distance"),
    }
}

/// A handle dragged along its track sets its field, snapped; the combine
/// mode follows the distance's sign.
#[test]
fn handles_are_dragged() {
    let (mut editor, _) = editor();
    editor.handle(Command::Open { id: "hole".into() });
    // Seen from the side: the hole's end cap is half a unit into the box,
    // at (1, 1, 0.5); dragged up past its sketch plane, it builds up.
    let side = |z: f64| pointer([1.0, -10.0, z], [0.0, 1.0, 0.0]);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: side(0.5),
            shift: false,
        },
    });
    assert!(update.step.unwrap().presentation.grab);
    editor.handle(Command::Event {
        event: StepEditEvent::Drag {
            from: side(0.5),
            to: side(1.253),
            done: true,
            shift: false,
        },
    });
    handled(&mut editor, Command::Commit);
    assert!((distance(&editor, "hole") - 0.25).abs() < 1e-9);
    let program = editor.program();
    let hole: ExtrudeArgs = args(&program.steps[program.index_of("hole").unwrap()]);
    assert!(matches!(
        hole.output,
        Output::Solid(Combine::With {
            op: BooleanOp::Union,
            ..
        })
    ));
}

/// A new extrude's distance handle, dragged as a viewer drags it: straight
/// away — the newest sketch it starts with needs no click — seen in
/// perspective, the pointer moving in several steps before it is released.
#[test]
fn new_steps_handles_are_dragged_in_steps() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "extrude".into(),
    });
    // From the side, in perspective: the hole sketch's extrude ends a unit
    // above the box's top, at (1, 1, 2).
    let side = |z: f64| {
        let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
        Pointer {
            ray: Ray::try_new(v([1.0, -10.0, 2.0]), v([0.0, 11.0, z - 2.0])).unwrap(),
            reach: Reach::Cone {
                slope: S::from_f64(0.002),
            },
        }
    };
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: side(2.0),
            shift: false,
        },
    });
    assert!(
        update.step.unwrap().presentation.grab,
        "the handle offers a grab"
    );
    for (z, done) in [(2.1, false), (2.2, false), (2.3, true)] {
        editor.handle(Command::Event {
            event: StepEditEvent::Drag {
                from: side(2.0),
                to: side(z),
                done,
                shift: false,
            },
        });
    }
    handled(&mut editor, Command::Commit);
    let program = editor.program();
    let extrude: ExtrudeArgs = args(program.steps.last().unwrap());
    assert_eq!(extrude.extent.side1, crate::Extent::blind(1.3));
}

/// A new step whose first input is still empty starts by picking it — a
/// new sketch waits for its plane, nothing preselected, and says so rather
/// than failing. One that already
/// holds its first input — the newest sketch, for an extrude — waits for no
/// click; pressing the field picks another, and while it does, the sketches
/// the program used — hidden otherwise — are shown to pick from.
#[test]
fn new_steps_pick_their_first_input() {
    let (mut editor, update) = editor();
    let hidden = update.scene.unwrap().hidden;
    assert!(hidden.contains(&"outline".to_string()));
    assert!(hidden.contains(&"hole_sketch".to_string()));

    let update = editor.handle(Command::New {
        kind: "add_sketch".into(),
    });
    let step = update.step.unwrap();
    // Waiting for its plane is no error; it cannot be committed yet.
    assert_eq!(step.missing, ["plane"]);
    assert_eq!(step.error, None);
    let presentation = step.presentation;
    assert_eq!(
        presentation.pickable,
        [drawn::FACE, drawn::DATUM, drawn::SKETCH]
    );
    let Some(Control::Reference(plane)) = presentation.dialog.get("plane") else {
        panic!("the plane is picked");
    };
    assert!(plane.armed && plane.value.is_empty(), "{plane:?}");
    let refused = editor.handle(Command::Commit);
    let refused = refused.error.expect("the commit is refused");
    assert!(refused.contains("pick the plane first"), "{refused}");
    editor.handle(Command::Cancel);

    let update = editor.handle(Command::New {
        kind: "extrude".into(),
    });
    assert!(update.step.unwrap().presentation.pickable.is_empty());
    let update = editor.handle(dialog("sketch", Value::Press));
    assert_eq!(update.step.unwrap().presentation.pickable, [drawn::SKETCH]);
    assert!(update.scene.unwrap().hidden.is_empty());
}

/// Choosing to cut sticks while the distance changes on the same side of
/// the sketch plane; crossing it turns a cut into a join, and back.
#[test]
fn a_chosen_cut_stays_a_cut() {
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "extrude".into(),
    });
    let combine = |editor: &mut Editor| {
        editor.handle(Command::Commit);
        let program = editor.program();
        let extrude: ExtrudeArgs = args(program.steps.last().unwrap());
        let Output::Solid(combine) = extrude.output else {
            panic!("a solid");
        };
        editor.handle(Command::Undo);
        combine
    };
    editor.handle(dialog("combine", Value::Choice("difference".into())));
    editor.handle(dialog("distance", Value::Number(2.0)));
    assert!(matches!(
        combine(&mut editor),
        Combine::With {
            op: BooleanOp::Difference,
            ..
        }
    ));
}

/// A revolve picks its axis in the viewport, like anything else it builds
/// on: any line in the sketch's plane — here the origin's y axis.
#[test]
fn revolve_axes_are_picked() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::New {
        kind: "revolve".into(),
    });
    let presentation = update.step.unwrap().presentation;
    let Some(Control::Reference(axis)) = presentation.dialog.get("axis") else {
        panic!("the axis is picked");
    };
    assert!(
        axis.value.is_empty(),
        "the newest sketch, a circle, has no line to default to"
    );
    // Left apart from the box: joining the two is a boolean of its own.
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    // From below: the outline's square, under the box.
    let below = |x: f64, y: f64| pointer([x, y, -10.0], [0.0, 0.0, 1.0]);
    editor.handle(click_at(below(1.0, 0.5)));
    editor.handle(dialog("axis", Value::Press));
    // The origin's y axis, ten reaches of 0.009 long, from below.
    let update = editor.handle(click_at(below(0.0, 0.06)));
    let presentation = update.step.unwrap().presentation;
    let Some(Control::Reference(axis)) = presentation.dialog.get("axis") else {
        panic!("the axis is picked");
    };
    assert_eq!(
        axis.elements().cloned().collect::<Vec<_>>(),
        [origin_axis(FrameAxis::Y).picked()]
    );
    assert_eq!(axis.value[0].tone, Tone::Normal);
    assert!(!axis.armed, "one axis is picked, and done");
}

/// Seeking runs only the steps before the marker, and new steps go there.
#[test]
fn new_steps_go_where_the_program_runs_to() {
    let (mut editor, _) = editor();
    let update = editor.handle(Command::Seek { marker: Some(2) });
    let state = update.program.unwrap();
    assert_eq!(state.marker, 2);
    assert!(state.steps[1].runs && !state.steps[2].runs);
    editor.handle(Command::New {
        kind: "extrude".into(),
    });
    editor.handle(Command::Commit);
    assert_eq!(editor.program().steps[2].id, "extrude1");
    let update = editor.handle(Command::Show);
    assert_eq!(update.program.unwrap().marker, 3);
}

/// Editing the first sketch of a program builds only up to it, however
/// many steps follow: a point of the handle's outline dragged rebuilds the
/// sketch, not the sweep, hole and boolean after it, on every move of the
/// pointer. Put away, the whole program is built again.
#[test]
fn editing_an_early_sketch_builds_only_up_to_it() {
    let mut editor = editor_with(registry(), crate::examples::handle_with_hole());
    let steps = editor.program().steps.len();
    assert!(steps >= 3, "{steps}");
    handled(
        &mut editor,
        Command::Open {
            id: "outline".into(),
        },
    );
    let args: geop_ops_sketch::AddSketchArgs = args(editor.editing().unwrap());
    let corner = args
        .sketch
        .points
        .values()
        .find(|p| !p.fixed)
        .expect("a point of the outline")
        .xy();
    let corner = [corner[0].to_f64(), corner[1].to_f64()];
    let down = |[x, y]: [f64; 2]| from_above(x, y);
    editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: down(corner),
            shift: false,
        },
    });
    for (k, d) in [0.05, 0.1, 0.15].into_iter().enumerate() {
        let update = handled(
            &mut editor,
            Command::Event {
                event: StepEditEvent::Drag {
                    from: down(corner),
                    to: down([corner[0] + d, corner[1] + d]),
                    done: k == 2,
                    shift: true,
                },
            },
        );
        // What follows the sketch does not run while it is edited.
        if let Some(program) = update.program {
            assert!(program.steps.iter().all(|s| !s.runs), "{:?}", program.steps);
        }
    }
    let dragged: geop_ops_sketch::AddSketchArgs = crate::testing::args(editor.editing().unwrap());
    assert_ne!(dragged.sketch, args.sketch, "the drag changed nothing");
    let update = handled(&mut editor, Command::Commit);
    let program = update.program.expect("the program changed");
    assert_eq!(program.steps.len(), steps);
    assert!(
        program.steps.iter().all(|s| s.runs && s.error.is_none()),
        "{:?}",
        program.steps
    );
}

/// The plate's thickness, typed into its extrude's dialog as a formula of
/// the parameters, as one types it in the parameters panel: the field
/// shows the formula and what it comes to, and drops its slider and its
/// handle — a value that follows a formula is not dragged. A formula that
/// does not evaluate is kept as typed, the field and the step saying why;
/// a number typed makes it plain, and draggable, again.
#[test]
fn formulas_are_typed_into_dialog_fields() {
    let mut editor = editor_with(registry(), crate::examples::parametric_plate());
    let field =
        |step: &geop_core_editor::editor::StepState| match step.presentation.dialog.get("distance")
        {
            Some(Control::Number(n)) => n.clone(),
            other => panic!("{other:?} is no number field"),
        };
    let handle = |step: &geop_core_editor::editor::StepState| {
        step.presentation
            .visuals
            .iter()
            .any(|v| v.key == "distance" && matches!(v.shape, Shape::Handle { .. }))
    };
    let thickness = |editor: &Editor| {
        let program = editor.program();
        let args: ExtrudeArgs = args(&program.steps[program.index_of("plate").unwrap()]);
        args.extent.side1.clone()
    };
    let top = |editor: &Editor| {
        model_shown(editor)
            .topology()
            .vertices
            .values()
            .map(|v| v.point[2].to_f64())
            .fold(f64::NEG_INFINITY, f64::max)
    };

    let update = editor.handle(Command::Open { id: "plate".into() });
    let step = update.step.expect("the plate is edited");
    // The example's own formula.
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("thickness"));
    assert_eq!((n.value, n.range, n.error), (0.5, None, None));
    assert!(!handle(&step), "a formula's value has a handle");

    let update = handled(
        &mut editor,
        dialog("distance", Value::Text("3 * thickness".into())),
    );
    let step = update.step.expect("the plate is edited");
    assert_eq!(step.error, None);
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("3 * thickness"));
    assert!((n.value - 1.5).abs() < 1e-12, "{}", n.value);
    assert!(n.range.is_none() && !handle(&step));

    let update = editor.handle(dialog("distance", Value::Text("3 * thicknes".into())));
    let step = update.step.expect("the plate is edited");
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("3 * thicknes"));
    assert_eq!(
        n.error.as_deref(),
        Some(r#"there is no parameter "thicknes""#)
    );
    let error = step.error.expect("the step fails");
    assert!(error.contains(r#""3 * thicknes""#), "{error}");

    editor.handle(dialog("distance", Value::Text("3 * thickness".into())));
    handled(&mut editor, Command::Commit);
    assert_eq!(thickness(&editor), crate::Extent::blind("3 * thickness"));
    assert!((top(&editor) - 1.5).abs() < 1e-9, "{}", top(&editor));

    // A number typed is plain again: slid and dragged.
    editor.handle(Command::Open { id: "plate".into() });
    let update = editor.handle(dialog("distance", Value::Text(" 0.8 ".into())));
    let step = update.step.expect("the plate is edited");
    let n = field(&step);
    assert_eq!(n.text.as_deref(), Some("0.8"));
    assert!(n.range.is_some() && handle(&step));
    editor.handle(Command::Commit);
    assert_eq!(thickness(&editor), crate::Extent::blind(0.8));
}

/// A sweep made in the editor: the horn's sketches picked, its rail
/// dropped for a twist and taper, the profile kept facing one way —
/// the dialog showing the twist and scale only once there is no rail.
///
/// It is made a new body first: joined to the horn, every change in the
/// dialog would run a boolean of the sweep with it, which is not what is
/// tested here.
#[test]
fn sweep_with_rails_twist_and_orientation() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = handled(
        &mut editor,
        Command::Load {
            program: crate::examples::horn(),
            path: None,
        },
    );
    let solids = |update: &Update| {
        let scene = update.scene.as_ref().expect("a scene");
        scene.structure.iter().filter(|i| i.kind == "solid").count()
    };
    assert_eq!(solids(&update), 1);

    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    let sketch = |name: &str| Value::Elements(vec![geop_ops_sketch::whole_sketch(name).picked()]);
    editor.handle(dialog("profile", sketch("mouth")));
    editor.handle(dialog("path", sketch("axis")));
    let update = editor.handle(dialog("rails", Value::Press));
    assert_eq!(
        update.step.unwrap().presentation.pickable,
        [drawn::SKETCH, drawn::EDGE]
    );
    let update = editor.handle(dialog("rails", sketch("flare")));
    let dialog_shown = update.step.unwrap().presentation.dialog;
    assert!(
        dialog_shown.get("twist").is_none(),
        "the rail decides the twist"
    );

    editor.handle(dialog("rails", Value::Elements(Vec::new())));
    editor.handle(dialog("orientation", Value::Choice("fixed_normal".into())));
    editor.handle(dialog("twist", Value::Number(90.0)));
    let update = editor.handle(dialog("end_scale", Value::Number(0.5)));
    let step = update.step.unwrap();
    assert_eq!(step.error, None);
    let Some(Control::Number(twist)) = step.presentation.dialog.get("twist") else {
        panic!("the twist is shown without rails");
    };
    assert_eq!(twist.value, 90.0);
    let update = handled(&mut editor, Command::Commit);
    assert_eq!(solids(&update), 2);
    let program = editor.program();
    let args: crate::SweepArgs = args(program.steps.last().unwrap());
    assert_eq!(
        (
            args.twist,
            args.end_scale,
            args.orientation,
            args.rails.len()
        ),
        (90.0, 0.5, geop_lib_sweep::Orientation::FixedNormal, 0)
    );
}

/// A 3-D sketch drawn the way the front end draws it: a new step, clicks
/// that place points — at the origin, fixed there, and then in space —
/// chained by lines, Enter to end; committed, a sweep along it pipes the
/// section drawn before into a valid solid.
#[test]
fn a_3d_sketch_is_drawn_and_swept_along() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let mut program = crate::examples::pipe();
    program.steps.truncate(1);
    let update = editor.handle(Command::Load {
        program,
        path: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "add_sketch3d".into(),
    });
    assert_eq!(update.step.unwrap().label, "3-D sketch");
    let click = |editor: &mut Editor, origin: [f64; 3], dir: [f64; 3]| {
        editor.handle(Command::Event {
            event: StepEditEvent::Click {
                pointer: pointer(origin, dir),
                button: Button::Primary,
                double: false,
                shift: false,
            },
        })
    };
    // On the origin's ball: a point fixed at the origin.
    click(&mut editor, [0.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    // Then in the plane through the last point, facing the eye.
    click(&mut editor, [2.0, -10.0, 0.0], [0.0, 1.0, 0.0]);
    click(&mut editor, [2.0, -10.0, 2.0], [0.0, 1.0, 0.0]);
    let update = click(&mut editor, [-10.0, 2.0, 2.0], [1.0, 0.0, 0.0]);
    let presentation = update.step.unwrap().presentation;
    let points = presentation
        .visuals
        .iter()
        .filter(|v| matches!(v.shape, Shape::Point { .. }))
        .count();
    assert!(points >= 4, "{:?}", presentation.visuals);
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Key {
            key: "Enter".into(),
        },
    });
    let step = update.step.unwrap();
    assert!(step.error.is_none(), "{:?}", step.error);
    match step.presentation.dialog.get("status") {
        Some(Control::Text { text, .. }) => assert!(text.contains("1 chain"), "{text}"),
        other => panic!("{other:?}"),
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let route = editor.program().steps.last().unwrap().clone();
    let drawn: geop_ops_sketch3d::AddSketch3dArgs = args(&route);
    assert_eq!(drawn.sketch.curves.len(), 3);
    assert_eq!(drawn.references.len(), 1, "the origin, as a fixed point");

    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let sweep: crate::SweepArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        (sweep.profile, sweep.path),
        (
            geop_ops_sketch::whole_sketch("section"),
            Some(crate::PathRef::Sketch3d(geop_ops_sketch3d::sketch3d_path(
                &route.id
            )))
        )
    );
    let part = model_of(&built(&editor).unwrap());
    assert_eq!(part.solid_names().len(), 1);
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// The horn swept with a 3-D sketch as its rail, clicked where it runs:
/// the rail is the whole sketch, and the horn builds as from the planar
/// one.
#[test]
fn a_sweep_takes_a_3d_sketch_as_its_rail() {
    let mut program = crate::examples::horn();
    program.steps.truncate(2);
    let mut flare = geop_ops_sketch3d::Sketch3d::new();
    let points = [
        [0.0, 0.5, 0.0],
        [1.5, 0.4, 0.0],
        [3.0, 0.9, 0.0],
        [4.0, 1.6, 0.0],
    ]
    .map(|p| flare.add_point(Vector3::from_array(p.map(crate::testing::n))));
    flare.add_spline(points.to_vec());
    program.push(
        "flare",
        geop_ops_sketch3d::AddSketch3dArgs {
            sketch: flare,
            references: Vec::new(),
        },
    );
    let mut editor = editor_with(registry(), program);
    editor.handle(Command::New {
        kind: "sweep".into(),
    });
    let sketch = |name: &str| Value::Elements(vec![geop_ops_sketch::whole_sketch(name).picked()]);
    editor.handle(dialog("combine", Value::Choice("new_body".into())));
    editor.handle(dialog("profile", sketch("mouth")));
    editor.handle(dialog("path", sketch("axis")));
    editor.handle(dialog("rails", Value::Press));
    editor.handle(click_at(from_above(1.5, 0.4)));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let args: crate::SweepArgs = args(editor.program().steps.last().unwrap());
    assert_eq!(
        args.rails,
        [crate::PathRef::Sketch3d(geop_ops_sketch3d::sketch3d_path(
            "flare"
        ))]
    );
    let part = model_of(&built(&editor).unwrap());
    assert_eq!(part.solid_names().len(), 1);
    let params = geop_core_topology::validation::ValidationParameters::default();
    geop_core_topology::validation::validate(&params, part.topology()).unwrap();
}

/// An inspector of the tests that picks faces, two at most: what it says
/// is which are picked, and it draws a point where the first one is hit.
struct Picked;

impl Inspector for Picked {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "picked",
        label: "Picked",
        doc: "Which faces are picked.",
        icon: None,
        picks: Some(Picks {
            accepts: &[drawn::FACE],
            most: 2,
        }),
    };
    type Options = ();
    type Report = Vec<String>;

    fn inspect(
        _: &mut Made,
        picked: &[PickedElement],
        _: (),
    ) -> GeopResult<(Vec<String>, Vec<Visual<S>>)> {
        let names = picked.iter().map(|p| format!("{p:?}")).collect();
        let at = Vector3::from_array([0.5, 0.5, 1.0].map(S::from_f64));
        let visuals = (picked.first())
            .map(|_| Visual::new("first", Shape::Point { at }, Style::Selected))
            .into_iter()
            .collect();
        Ok((names, visuals))
    }
}

/// An inspector of the tests asked of the whole part: how many solids it
/// has, and the label it is told to give that.
struct Solids;

#[derive(Default, serde::Deserialize)]
struct SolidsOptions {
    label: String,
}

impl Inspector for Solids {
    const INFO: InspectorInfo = InspectorInfo {
        kind: "solids",
        label: "Solids",
        doc: "How many solids the part has.",
        icon: None,
        picks: None,
    };
    type Options = SolidsOptions;
    type Report = (String, usize);

    fn inspect(
        output: &mut Made,
        _: &[PickedElement],
        options: SolidsOptions,
    ) -> GeopResult<((String, usize), Vec<Visual<S>>)> {
        let model = read_model(&output.part())?;
        Ok(((options.label, model.solid_names().len()), Vec::new()))
    }
}

/// Inspecting as the inspect panel does, with whatever inspectors the
/// editor offers: listed in the program state; one that picks taken in
/// hand, the box's top hovered and clicked, then its bottom clicked from
/// below — each pick asked about again, the answer's point drawn; one of
/// the whole part asked once, told its options. None of it changes the
/// program, and the tool put down shows nothing.
#[test]
fn inspectors_are_offered_picked_with_and_asked() {
    // An editor of what the box is built of, offering only these
    // inspectors.
    let registry = (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_datums::register)
        .and_then(crate::register)
        .and_then(|r| r.inspector(Picked))
        .and_then(|r| r.inspector(Solids))
        .unwrap();
    let mut editor = editor_with(registry, crate::examples::box_with_drill_hole());
    let before = editor.program().clone();
    let update = handled(&mut editor, Command::Show);
    let program = update.program.unwrap();
    let kinds: Vec<&str> = program.inspectors.iter().map(|i| i.kind).collect();
    assert_eq!(kinds, ["picked", "solids"]);
    assert_eq!(program.tool, None);

    let update = handled(
        &mut editor,
        Command::Tool {
            key: Some("pick:picked".into()),
        },
    );
    assert_eq!(update.program.unwrap().tool.as_deref(), Some("pick:picked"));
    let top = element::<Face>("extrude(box,end)").picked();
    let bottom = element::<Face>("extrude(box,start)").picked();
    let report = |update: &Update| {
        let inspection = update.inspection.as_ref().expect("an answer");
        assert_eq!(inspection.kind, "picked");
        serde_json::from_value::<Vec<String>>(inspection.report.clone()).unwrap()
    };
    let update = handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Hover {
                pointer: from_above(0.5, 0.5),
                shift: false,
            },
        },
    );
    assert_eq!(update.tool.as_ref().unwrap().highlights, vec![top.clone()]);
    assert!(report(&update).is_empty());

    let update = handled(&mut editor, click_at(from_above(0.5, 0.5)));
    assert!(update.program.is_none(), "inspecting changes no program");
    assert_eq!(report(&update), [format!("{top:?}")]);
    assert!(
        update
            .tool
            .as_ref()
            .unwrap()
            .visuals
            .iter()
            .any(|v| v.key == "first")
    );
    let update = handled(
        &mut editor,
        click_at(pointer([0.5, 0.5, -10.0], [0.0, 0.0, 1.0])),
    );
    assert_eq!(report(&update), [format!("{top:?}"), format!("{bottom:?}")]);

    let json = editor
        .handle_json(
            r#"{"command": "inspect", "inspector": "solids", "options": {"label": "bodies"}}"#,
        )
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{update}");
    assert_eq!(update["inspection"]["kind"], "solids");
    assert_eq!(
        update["inspection"]["report"],
        serde_json::json!(["bodies", 1])
    );

    let update = editor.handle(Command::Tool {
        key: Some("pick:solids".into()),
    });
    assert!(update.error.expect("refused").contains("picks nothing"));
    let update = editor.handle(Command::Inspect {
        inspector: "mass".into(),
        options: serde_json::Value::Null,
    });
    assert!(
        update
            .error
            .expect("refused")
            .contains("there is no inspector `mass`")
    );

    let update = handled(&mut editor, Command::Tool { key: None });
    assert!(update.tool.is_none() && update.inspection.is_none());
    assert_eq!(*editor.program(), before);
}

/// Parameters given to a program without any, while a step is edited, put
/// their step first: the step edited, and the marker, stay at the steps they
/// were at, and the commit keeps both the parameters and the step's edit.
#[test]
fn the_first_parameter_given_while_a_step_is_edited() {
    let (mut editor, _) = editor();
    assert!(parameters_of(editor.program()).is_empty());
    editor.handle(Command::Seek { marker: Some(2) });
    handled(
        &mut editor,
        Command::New {
            kind: "extrude".into(),
        },
    );
    editor.handle(dialog("distance", Value::Number(2.0)));
    let parameters = Parameters {
        values: vec![Parameter::number("depth", "3", 0.0, 100.0)],
        ..Parameters::default()
    };
    handled(&mut editor, set_parameters(parameters));
    handled(&mut editor, Command::Commit);
    let steps = &editor.program().steps;
    assert_eq!(steps[0].operation, DefineParameters::KIND);
    assert_eq!(steps[3].id, "extrude1");
    assert_eq!(distance(&editor, "extrude1"), 2.0);
    let update = editor.handle(Command::Show);
    assert_eq!(update.program.unwrap().marker, 4);
}

/// What reads each parameter is sent with the program, for the parameters
/// panel to warn before one is removed; and renaming one, as its dialog's
/// name field does, renames it in every formula — the other parameters',
/// a sketch's dimensions, an extrude's length, a table's columns — so
/// nothing fails and the part is the same. A name taken, and a rename
/// while a step is edited, are refused; undo takes a rename back.
#[test]
fn renaming_a_parameter_renames_what_reads_it() {
    let mut editor = Editor::new(registry(), BTreeMap::new());
    let update = handled(
        &mut editor,
        Command::Load {
            program: parametric_plate(),
            path: None,
        },
    );
    let uses = parameters_panel(&update.program.expect("the program is sent")).parameter_uses;
    let readers = |name: &str| uses.get(name).cloned().unwrap_or_default();
    assert_eq!(readers("width"), ["depth", "outline"]);
    assert_eq!(readers("depth"), ["outline"]);
    assert_eq!(readers("thickness"), ["plate"]);
    assert_eq!(readers("screw"), ["hole_sketch"]);
    let volume = |editor: &Editor| {
        let part = model_shown(editor);
        let [name] = part.solid_names().try_into().expect("one solid");
        solid_volume(&part, &name)
    };
    let before = volume(&editor);

    for (from, to) in [
        ("width", "plate_width"),
        ("screw", "bolt"),
        ("thickness", "t"),
    ] {
        let update = editor.handle(rename_parameter(from, to));
        assert!(update.error.is_none(), "{from} -> {to}: {:?}", update.error);
        let program = update.program.expect("the program is sent");
        assert!(
            program.steps.iter().all(|s| s.error.is_none()),
            "{:?}",
            program.steps
        );
        let panel = parameters_panel(&program);
        assert!(
            panel.parameters.errors.is_empty(),
            "{:?}",
            panel.parameters.errors
        );
        assert!(
            !panel.parameter_uses.contains_key(from),
            "{from} is still read"
        );
    }
    let program = editor.program();
    let json = program.to_json().unwrap();
    for (old, new) in [
        ("\"width / 2\"", "\"plate_width / 2\""),
        ("\"screw.clearance\"", "\"bolt.clearance\""),
        (
            "{\"blind\":{\"formula\":\"thickness\"}}",
            "{\"blind\":{\"formula\":\"t\"}}",
        ),
    ] {
        let compact: String = json.split_whitespace().collect();
        assert!(
            !compact.contains(&old.replace(' ', "")),
            "{old} is left in {json}"
        );
        assert!(
            compact.contains(&new.replace(' ', "")),
            "{new} is not in {json}"
        );
    }
    assert_eq!(volume(&editor), before);

    let taken = editor.handle(rename_parameter("depth", "t"));
    let error = taken.error.expect("a name taken is refused");
    assert!(
        error.contains(r#"there is a parameter "t" already"#),
        "{error}"
    );
    editor.handle(Command::Open { id: "plate".into() });
    let editing = editor.handle(rename_parameter("depth", "plate_depth"));
    assert!(editing.error.is_some(), "renamed while a step is edited");
    editor.handle(Command::Cancel);

    editor.handle(Command::Undo);
    assert!(parameters_of(editor.program()).get("thickness").is_some());
    assert!(parameters_of(editor.program()).get("t").is_none());
}
