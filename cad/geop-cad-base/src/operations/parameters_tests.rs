//! Parts designed by their parameters: the parametric plate's dimensions,
//! hole and colour following them, built on its own, placed with other
//! values, and edited in the editor.

use geop_core_program::{
    numeric::{ParameterValue, read_parameter_values},
    part::Part,
    program::Program,
    reader::Reader,
    values::{Design, Setting},
};
use geop_ops_parameters::{
    COLOR, ParameterKind, parameter_numbers, parameters_of, read_properties, set_parameters,
    steps::{LOOKUP, PARAMETERS, PROPERTIES},
};
use std::{borrow::Cow, collections::BTreeMap};

use geop_core_math::{
    primitives::FrameAxis,
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_sketch::PointId;
use geop_ops_assembly::placed::{Placed, locate, placed_parts};
use geop_ops_model::{Point, element, read_model};
use geop_ops_sketch::{
    AddSketchArgs, CurveKind, PlacedSketch,
    references::{Projected, Reference, Source},
    sketch_element, whole_sketch,
};

use geop_ops_assembly::examples;

use super::program_tests::Running;
use crate::registry::testing::{Built, args, on_origin};

/// The sketch `name` of `part`.
fn placed_sketch(part: &Part, name: &str) -> PlacedSketch<Design> {
    let reader = part.current();
    reader
        .retrieve_element(&whole_sketch(name))
        .unwrap()
        .clone()
}

/// The value of the parameter `name` the part `view` shows was built with:
/// a table's row, a number, a table's column (`table.column`) — or, for
/// [`COLOR`], its colour.
fn input(view: &Reader, name: &str) -> Option<Setting> {
    if name == COLOR {
        return read_properties(view).ok()?.color.clone().map(Setting::Text);
    }
    let values = read_parameter_values(view).ok()?;
    let (_, value) = values.0.iter().find(|(n, _)| n == name)?;
    match value {
        ParameterValue::Number(n) => Some(Setting::Number(*n)),
        ParameterValue::Text(row) => Some(Setting::Text(row.clone())),
    }
}

/// The point `p2` of the sketch `outline` of the part `view`, in space.
fn outline_corner(view: &Reader) -> [f64; 3] {
    let corner = view.retrieve_element(&sketch_element::<Point>("outline", PointId(2)));
    let p = corner.unwrap();
    [0, 1, 2].map(|k| p[k].to_f64())
}

/// A projection follows what it projects: change the parameter the box's
/// width is a formula of, and the top's edges projected into a sketch
/// built after it move with it — the sketch's ids for them unchanged.
#[test]
fn projections_follow_the_part() {
    let mut program = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    let mut on_floor = AddSketchArgs::new(Some(on_origin(FrameAxis::Z)));
    // What it projects is put into the sketch when it is built.
    on_floor.references.push(Reference::new(Source::Projection {
        entity: Projected::Face(element("extrude(plate,end)")),
    }));
    // Right after the plate: the hole renames the faces it cuts.
    let mut floor = Program::new();
    floor.push("floor", on_floor);
    let after_plate = program.index_of("plate").unwrap() + 1;
    program.steps.insert(after_plate, floor.steps.remove(0));

    let widest = |program: &Program| {
        let placed = placed_sketch(&program.built().unwrap(), "floor");
        let ids: Vec<_> = placed.sketch.points.keys().copied().collect();
        let x = placed
            .sketch
            .points
            .values()
            .map(|p| p.x.to_f64())
            .fold(f64::MIN, f64::max);
        (x, ids)
    };
    let (before, ids) = widest(&program);
    assert!((before - 4.0).abs() < 1e-9, "{before}");
    let mut parameters = parameters_of(&program);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
        panic!("width is a number")
    };
    *expression = "5.5".into();
    set_parameters(&mut program, parameters);
    let (after, ids_after) = widest(&program);
    assert!((after - 5.5).abs() < 1e-9, "{after}");
    assert_eq!(ids, ids_after);
}

/// The plate's dimensions follow its parameters — the depth a formula of
/// the width, the hole sized from the screw table — and its colour is one
/// of them too.
#[test]
fn dimensions_follow_parameters() {
    let program = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    let part = program.built().unwrap();
    let view = part.current();
    assert_eq!(input(&view, COLOR), Some(Setting::Text("#d0893e".into())));
    let p = outline_corner(&view);
    assert!((p[0] - 4.0).abs() < 1e-9 && (p[1] - 2.0).abs() < 1e-9);
    // Read back from JSON, the same.
    let json = program.to_json().unwrap();
    assert!(json.contains("screw.clearance"), "{json}");
    let back = Program::from_json(&json).unwrap();
    assert_eq!(back, program);
}

/// Placed by another program, the plate is built with the values given
/// there: wider, with a larger hole, in another colour — and as it is
/// where nothing is given.
#[test]
fn placed_parts_take_the_parameters_given() {
    let workspace = crate::workspace(BTreeMap::from([(
        "plate.geop".to_string(),
        crate::registry()
            .unwrap()
            .example("parametric_plate")
            .unwrap()
            .to_json()
            .unwrap(),
    )]));
    let program = examples::plates_assembly();
    let part = crate::build(&program, "plates.geop", workspace.clone())
        .unwrap()
        .part;
    let view = part.current();
    // An element of a placed part, where it is placed.
    let corner = |instance: &str| {
        let located = locate(&view, instance).unwrap();
        let component = located.part.clone();
        let reference = sketch_element::<Point>("outline", PointId(2));
        let p = located
            .world
            .apply(&component.retrieve_element(&reference).unwrap());
        [p[0].to_f64(), p[1].to_f64()]
    };
    let small = corner("small");
    let large = corner("large");
    assert!(
        (small[0] - 4.0).abs() < 1e-9 && (small[1] - 2.0).abs() < 1e-9,
        "{small:?}"
    );
    // 5 wide, and so 2.5 deep; placed 3 along y.
    assert!(
        (large[0] - 5.0).abs() < 1e-9 && (large[1] - 5.5).abs() < 1e-9,
        "{large:?}"
    );
    let placed = placed_parts(&view).unwrap();
    let color = |part: &Placed| input(&part.located.part.clone(), COLOR);
    let colors: Vec<Option<Setting>> = placed.iter().map(color).collect();
    let text = |c: &str| Some(Setting::Text(c.into()));
    assert!(colors.contains(&text("#3e7bd0")), "{colors:?}");
    assert!(colors.contains(&text("#d0893e")), "{colors:?}");
    // The table's row and the number given are what the large plate was
    // built with.
    let large = placed.iter().find(|p| color(p) == text("#3e7bd0")).unwrap();
    let large = large.located.part.clone();
    assert_eq!(input(&large, "screw"), text("M6"));
    let clearance = match input(&large, "screw.clearance") {
        Some(Setting::Number(v)) => Some(v),
        _ => None,
    };
    assert_eq!(clearance, Some(0.66));
}

/// The parametric plate is a plate — 4 x 2 x 0.5 — with a hole in it:
/// its vertices span the plate, and the hole's sketch is built where the
/// plate's top is.
#[test]
fn the_parametric_plate_is_a_plate_with_a_hole() {
    let built = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap()
        .built()
        .unwrap();
    let part = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap()
        .build_model()
        .unwrap();
    let points: Vec<[f64; 3]> = part
        .topology()
        .vertices
        .values()
        .map(|v| [0, 1, 2].map(|k| v.point[k].to_f64()))
        .collect();
    let span = |k: usize| {
        let lo = points.iter().map(|p| p[k]).fold(f64::MAX, f64::min);
        let hi = points.iter().map(|p| p[k]).fold(f64::MIN, f64::max);
        [lo, hi]
    };
    assert!(
        (span(0)[0]).abs() < 1e-9 && (span(0)[1] - 4.0).abs() < 1e-9,
        "{:?} {points:?}",
        span(0)
    );
    assert!((span(1)[1] - 2.0).abs() < 1e-9, "{:?}", span(1));
    assert!((span(2)[1] - 0.5).abs() < 1e-9, "{:?}", span(2));
    let hole = placed_sketch(&built, "hole_sketch");
    let geometry = hole.sketch.enclose::<S>().unwrap();
    let centers: Vec<_> = hole
        .sketch
        .curves
        .values()
        .filter_map(|c| match c.kind {
            CurveKind::Circle { center, .. } => Some(geometry.points[&center]),
            _ => None,
        })
        .collect();
    let c = &centers[0];
    assert!(
        (c[0].to_f64() - 2.0).abs() < 1e-9 && (c[1].to_f64() - 1.0).abs() < 1e-9,
        "{c:?}"
    );
}

/// The parametric plate made 6 wide with an M6 hole. Its cut once left the
/// result open around the hole's rim: the disc cut from the top was taken
/// for outside the hole, because the point classifying it, beside the rim,
/// was found inside the disc by a ray grazing the rim — cast from a box
/// around the point, it was a strip, and touching the rim counted as one
/// crossing. Reduced to a plate and a cylinder in the booleans' tests.
#[test]
fn wide_plate_with_an_m6_hole() {
    let mut plate = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    let mut parameters = parameters_of(&plate);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
        panic!("width is a number")
    };
    *expression = "6".into();
    let screw = parameters.values.iter_mut().find(|p| p.name == "screw");
    let Some(ParameterKind::Table { selected, .. }) = screw.map(|p| &mut p.kind) else {
        panic!("screw is a table")
    };
    *selected = "M6".into();
    set_parameters(&mut plate, parameters);
    plate.build_model().unwrap();
}

/// Placing the plate, its parameters are offered as what they are: its
/// colour to pick, its numbers on sliders over their ranges, its screw
/// from a table to search — showing what the plate is built with here,
/// the depth following the width given — and what is set there is what
/// the plate is placed with.
#[test]
fn placing_a_part_offers_its_parameters() {
    use crate::Command;
    use crate::registry::testing::handled;
    use geop_core_editor::editing::{Control, StepEditEvent, Value};
    let mut editor = crate::editor();
    let files = BTreeMap::from([(
        "plate.geop".to_string(),
        Some(
            crate::registry()
                .unwrap()
                .example("parametric_plate")
                .unwrap()
                .to_json()
                .unwrap(),
        ),
    )]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    handled(
        &mut editor,
        Command::Load {
            program: examples::plates_assembly(),
            path: Some("plates.geop".into()),
        },
    );
    let update = editor.handle(Command::Open { id: "large".into() });
    let dialog = update.step.unwrap().presentation.dialog;
    assert!(matches!(
        dialog.get("parameter:color"),
        Some(Control::Color { value, .. }) if value == "#3e7bd0"
    ));
    let Some(Control::Number(width)) = dialog.get("parameter:width") else {
        panic!("width on a slider")
    };
    assert_eq!((width.value, width.range), (5.0, Some([1.0, 10.0])));
    let Some(Control::Number(depth)) = dialog.get("parameter:depth") else {
        panic!("depth on a slider")
    };
    assert!((depth.value - 2.5).abs() < 1e-9, "{}", depth.value);
    let Some(Control::Select {
        options,
        searchable,
        value,
        ..
    }) = dialog.get("parameter:screw")
    else {
        panic!("the screw from its table")
    };
    assert!(*searchable && value == "M6" && options.len() == 4);

    handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Dialog {
                key: "parameter:screw".into(),
                value: Value::Choice("M3".into()),
            },
        },
    );
    handled(&mut editor, Command::Commit);
    let args: geop_ops_assembly::AddPartArgs = args(&editor.program().steps[1]);
    assert_eq!(args.parameters["screw"], Setting::Text("M3".into()));
}

/// The program's parameters are edited as a whole: the part is built
/// anew with them, what they resolve to — and why one does not — is
/// shown, invalid ones are refused, and the edit is undone like any other.
#[test]
fn parameters_are_edited_and_undone() {
    use crate::Command;
    let mut editor = crate::editor();
    let program = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    editor.handle(Command::Load {
        program: program.clone(),
        path: None,
    });
    let mut parameters = parameters_of(&program);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[1].kind else {
        panic!("depth is a number")
    };
    *expression = "width / 4 + nothing".into();
    let update = editor.handle(crate::registry::testing::set_parameters(parameters.clone()));
    let state = update.program.unwrap();
    assert!(
        crate::registry::testing::parameters_panel(&state)
            .parameters
            .errors["depth"]
            .contains("nothing"),
        "{state:?}"
    );
    let ParameterKind::Number { expression, .. } = &mut parameters.values[1].kind else {
        unreachable!()
    };
    *expression = "width / 4".into();
    let state = editor
        .handle(crate::registry::testing::set_parameters(parameters.clone()))
        .program
        .unwrap();
    assert_eq!(
        crate::registry::testing::parameters_panel(&state)
            .parameters
            .values
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
    assert!(
        editor
            .handle(crate::registry::testing::set_parameters(parameters))
            .error
            .is_some()
    );
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
    use crate::Command;
    use crate::registry::testing::handled;
    use geop_core_editor::editing::{Control, Shape, StepEditEvent, Tone, Value};
    use geop_ops_parameters::{Parameter, Parameters, parameters::Row};
    let mut editor = crate::editor();
    let program = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    editor.handle(Command::Load {
        program: program.clone(),
        path: None,
    });
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
    let update = handled(
        &mut editor,
        crate::registry::testing::set_parameters(parameters.clone()),
    );
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
    let built = editor.program().built().unwrap();
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
    let update = editor.handle(crate::registry::testing::set_parameters(parameters.clone()));
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
    handled(
        &mut editor,
        crate::registry::testing::set_parameters(parameters.clone()),
    );
    let sketch: AddSketchArgs = args(editor.editing().unwrap());
    let (&diameter, _) = sketch
        .sketch
        .constraints
        .iter()
        .find(|(_, c)| matches!(c, geop_ops_sketch::Constraint::Diameter { .. }))
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

/// A runner rebuilds from scratch what a parameter changed reaches.
#[test]
fn runners_rebuild_when_parameters_change() {
    let mut program = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    let mut running = Running::new();
    let corner = |running: &Running| outline_corner(&running.part.current())[0];
    assert_eq!(running.run(&program, None), []);
    assert!((corner(&running) - 4.0).abs() < 1e-9);
    let mut parameters = parameters_of(&program);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
        panic!("width is a number")
    };
    *expression = "5".into();
    set_parameters(&mut program, parameters);
    let numbers = parameter_numbers(&program).unwrap();
    assert_eq!(numbers.get("width").copied(), Some(5.0));
    assert_eq!(running.run(&program, None), []);
    assert!(
        (corner(&running) - 5.0).abs() < 1e-9,
        "{}",
        corner(&running)
    );
}

/// A parameter no step reads — the part's colour — changes the part
/// without building anything again: its colour is new, and every step's
/// part is the one already built.
#[test]
fn parameters_no_step_reads_rebuild_nothing() {
    let mut program = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    let mut running = Running::new();
    assert_eq!(running.run(&program, None), []);
    // Every step but those of its parameters and properties, which build
    // neither a sketch nor a model.
    let building = (program.steps.iter())
        .filter(|s| ![PARAMETERS, PROPERTIES, LOOKUP].contains(&&*s.operation))
        .count();
    assert_eq!(running.built_anew(&program), building);
    let built = running.model();
    let mut parameters = parameters_of(&program);
    parameters.color = Some("#123456".into());
    set_parameters(&mut program, parameters);
    assert_eq!(running.run(&program, None), []);
    assert_eq!(
        input(&running.part.current(), COLOR),
        Some(Setting::Text("#123456".into()))
    );
    // Counted, not timed: no step read the colour, so none is built again.
    assert_eq!(
        running.built_anew(&program),
        0,
        "a colour change rebuilt steps"
    );
    assert_eq!(
        geop_ops_model::PartDescription::of(&running.model()).unwrap(),
        geop_ops_model::PartDescription::of(&built).unwrap()
    );
}

/// The same in the editor: picking a colour answers at once — the part is
/// drawn again in it, but nothing is built again.
#[test]
fn colours_are_picked_without_rebuilding() {
    use crate::{Command, Editor};
    let mut editor = crate::editor();
    let program = crate::registry()
        .unwrap()
        .example("parametric_plate")
        .unwrap();
    editor.handle(Command::Load {
        program: program.clone(),
        path: None,
    });
    // What the program built: kept, not built again, it is the very same.
    let model = |editor: &Editor| match read_model(&editor.part()) {
        Ok(Cow::Borrowed(built)) => built as *const _ as usize,
        other => panic!("no model built: {:?}", other.err()),
    };
    let before = model(&editor);
    let mut parameters = parameters_of(&program);
    parameters.color = Some("#123456".into());
    let update = editor.handle(crate::registry::testing::set_parameters(parameters));
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
