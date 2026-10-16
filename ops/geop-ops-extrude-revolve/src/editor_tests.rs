//! The parametric plate edited as a front end edits it: its parameters
//! set through the parameters panel, the part built anew with them (only
//! where they are read), and undone — driven through the
//! [`geop_core_editor::editor::Editor`].

use std::borrow::Cow;

use geop_core_editor::{
    editing::{Control, Shape, StepEditEvent, Tone, Value},
    editor::{Command, Editor, ProgramState},
    registry::{Panel, Registry},
    testing::{editor_with, handled},
};
use geop_core_math::scalars::Scalar;
use geop_core_program::{reader::Reader, values::Setting};
use geop_core_sketch::PointId;
use geop_ops_model::{Point, read_model};
use geop_ops_parameters::{
    Parameter, ParameterKind, Parameters, ParametersCommand, ParametersPanel, ParametersPanelState,
    parameters::Row, parameters_of,
};
use geop_ops_sketch::{AddSketchArgs, Constraint, sketch_element, whole_sketch};

use crate::{
    examples::parametric_plate,
    testing::{Built, args, registry as operations},
};

/// What the plate is edited among: the model's, the sketches', the
/// datums', the extrusions' and the parameters' operations, drawers and
/// panels.
fn registry() -> Registry {
    (geop_ops_model::register(Registry::default()))
        .and_then(geop_ops_sketch::register)
        .and_then(geop_ops_datums::register)
        .and_then(crate::register)
        .and_then(geop_ops_parameters::register)
        .unwrap()
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
