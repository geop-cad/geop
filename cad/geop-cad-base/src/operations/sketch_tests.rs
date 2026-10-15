//! Sketches built on the part: what they project of it, and the
//! parameters their dimensions follow.

use geop_core_program::{
    numeric::{ParameterValue, read_parameter_values},
    part::Part,
    program::Program,
    reader::Reader,
    values::{Design, Setting},
};
use geop_ops_parameters::{
    COLOR, ParameterKind, parameter_numbers, parameters_of, read_properties, set_parameters,
    steps::{PARAMETERS, PROPERTIES},
};
use std::{borrow::Cow, collections::BTreeMap};

use geop_core_editor::selectable::Selectable;
use geop_core_math::{
    primitives::FrameAxis,
    scalars::{ScalInF64 as S, Scalar},
};
use geop_core_sketch::PointId;
use geop_ops_assembly::placed::{Placed, locate, placed_parts};
use geop_ops_model::{Point, element, read_model};
use geop_ops_sketch::{
    AddSketchArgs, CurveKind, PlacedSketch, Sketch, SketchPlane,
    references::{Projected, Reference, Source},
    sketch_element, whole_sketch,
};

use super::program_tests::Running;
use crate::{
    examples,
    operations::testing::{Built, args, from_above, on_face, on_origin, sketch},
};

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
    let (parameter, column) = match name.split_once('.') {
        Some((table, column)) => (table, Some(column)),
        None => (name, None),
    };
    let (_, value) = values.0.iter().find(|(n, _)| n == parameter)?;
    match (value, column) {
        (ParameterValue::Number(n), None) => Some(Setting::Number(*n)),
        (ParameterValue::Row { name, .. }, None) => Some(Setting::Text(name.clone())),
        (ParameterValue::Row { columns, .. }, Some(column)) => {
            columns.get(column).map(|v| Setting::Number(*v))
        }
        (ParameterValue::Number(_), Some(_)) => None,
    }
}

/// The point `p2` of the sketch `outline` of the part `view`, in space.
fn outline_corner(view: &Reader) -> [f64; 3] {
    let corner = view.retrieve_element(&sketch_element::<Point>("outline", PointId(2)));
    let p = corner.unwrap();
    [0, 1, 2].map(|k| p[k].to_f64())
}

/// The top face of the box of [`examples::box_with_drill_hole`].
fn box_top() -> Projected {
    Projected::Face(element("extrude(box,end)"))
}

/// `entity` of `part` projected into a new sketch on `plane`: the sketch,
/// and the reference that gave it.
fn projected(part: &Part, plane: &SketchPlane, entity: Projected) -> (Sketch, Reference) {
    let reader = part.current();
    let frame = plane.resolve(&reader).unwrap();
    let mut sketch = Sketch::new();
    let mut reference = Reference::new(Source::Projection { entity });
    reference.update(&mut sketch, &reader, &frame).unwrap();
    sketch.validate().unwrap();
    (sketch, reference)
}

/// What kinds of curve a sketch holds, sorted.
fn kinds(sketch: &Sketch) -> Vec<&'static str> {
    let mut kinds: Vec<_> = sketch
        .curves
        .values()
        .map(|c| match c.kind {
            CurveKind::Line { .. } => "line",
            CurveKind::Arc { .. } => "arc",
            CurveKind::Circle { .. } => "circle",
            CurveKind::Spline { .. } => "spline",
        })
        .collect();
    kinds.sort();
    kinds
}

/// Projecting reads what an edge is from its NURBS: the box's top, seen
/// from above, is four lines around the hole's circle — as lines and a
/// circle, to constrain against, every one of them fixed. Seen from the
/// front, the hole's edge is no circle any more but the spline it
/// projects to, and the top's edges along the view collapse to points.
#[test]
fn projections_read_lines_arcs_and_splines() {
    let part = examples::box_with_drill_hole().built().unwrap();
    let top = box_top();
    let (sketch, reference) = projected(&part, &on_origin(FrameAxis::Z), top.clone());
    let round = kinds(&sketch)
        .iter()
        .filter(|k| **k == "arc" || **k == "circle")
        .count();
    assert_eq!(kinds(&sketch).iter().filter(|k| **k == "line").count(), 4);
    assert!(round >= 1, "{:?}", kinds(&sketch));
    assert!(!kinds(&sketch).contains(&"spline"));
    assert!(sketch.points.values().all(|p| p.fixed));
    // Reference geometry: fixed, and construction — never a profile.
    assert!(sketch.curves.values().all(|c| c.fixed && c.construction));
    assert!(sketch.regions().map_or(true, |r| r.is_empty()));
    // The hole's circle: around (1, 1), radius 0.4.
    for c in sketch.curves.values() {
        if let CurveKind::Circle { center, radius } = c.kind {
            let at = sketch.points[&center].xy();
            assert!((at[0].to_f64() - 1.0).abs() < 1e-9 && (at[1].to_f64() - 1.0).abs() < 1e-9);
            assert!((radius.to_f64() - 0.4).abs() < 1e-9);
        }
    }
    // Every curve is keyed by the name of the edge it comes from.
    assert!(reference.curves.keys().all(|k| k.starts_with("extrude(")));

    // Seen from the front the top is edge-on: its edges along the view are
    // points, the others lines, and the hole's circle the line it collapses
    // to — no spline lying flat along one.
    let (front, _) = projected(&part, &on_origin(FrameAxis::Y), top);
    assert!(front.validate().is_ok());
    let kinds = kinds(&front);
    assert!(
        !kinds.is_empty() && kinds.iter().all(|k| *k == "line"),
        "{kinds:?}"
    );
}

/// A projection follows what it projects: change the parameter the box's
/// width is a formula of, and the top's edges projected into a sketch
/// built after it move with it — the sketch's ids for them unchanged.
#[test]
fn projections_follow_the_part() {
    let mut program = examples::parametric_plate();
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
    let program = examples::parametric_plate();
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
    let workspace = super::workspace(BTreeMap::from([(
        "plate.geop".to_string(),
        examples::parametric_plate().to_json().unwrap(),
    )]));
    let program = examples::plates_assembly();
    let part = super::build(&program, "plates.geop", workspace.clone())
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
    let built = examples::parametric_plate().built().unwrap();
    let part = examples::parametric_plate().build_model().unwrap();
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
    let mut plate = examples::parametric_plate();
    let mut parameters = parameters_of(&plate);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
        panic!("width is a number")
    };
    *expression = "6".into();
    let ParameterKind::Table { selected, .. } = &mut parameters.values[2].kind else {
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
    use crate::operations::testing::handled;
    use crate::{Command, Editor};
    use geop_core_editor::editing::{Control, StepEditEvent, Value};
    let mut editor = Editor::new();
    let files = BTreeMap::from([(
        "plate.geop".to_string(),
        Some(examples::parametric_plate().to_json().unwrap()),
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

/// The project field of a sketch takes edges, vertices and faces of the
/// part: each picked is projected into the sketch, fixed, and taken out
/// again with its geometry when it is taken out of the field. Projecting
/// is optional: a sketch builds without.
#[test]
fn the_project_field_projects_what_is_picked() {
    use crate::operations::testing::handled;
    use crate::{Command, Editor};
    use geop_core_editor::editing::{Control, StepEditEvent, Value};
    let mut editor = Editor::new();
    let mut program = examples::parametric_plate();
    let after_plate = program.index_of("plate").unwrap() + 1;
    program.steps.truncate(after_plate);
    assert!(
        editor
            .handle(Command::Load {
                program,
                path: None
            })
            .error
            .is_none()
    );
    editor.handle(Command::New {
        kind: "add_sketch".into(),
    });
    let top = on_face("extrude(plate,end)");
    let dialog = |key: &str, value: Value| Command::Event {
        event: StepEditEvent::Dialog {
            key: key.into(),
            value,
        },
    };
    editor.handle(dialog("plane", Value::Elements(vec![top.picked()])));
    let update = editor.handle(Command::Show);
    let step = update.step.unwrap();
    assert!(
        step.missing.is_empty(),
        "projecting is optional: {:?}",
        step.missing
    );
    assert!(matches!(
        step.presentation.dialog.get("project"),
        Some(Control::Reference(r)) if !r.armed && !r.required
    ));
    let edge = Projected::Edge(element("extrude(plate,outline,c4,end)"));
    handled(
        &mut editor,
        dialog("project", Value::Elements(vec![edge.picked()])),
    );
    let projected = |editor: &Editor| {
        let args: AddSketchArgs = args(editor.editing().unwrap());
        let curves: Vec<_> = args
            .references
            .iter()
            .filter(|r| matches!(r.source, Source::Projection { .. }))
            .flat_map(|r| r.curves.values())
            .collect();
        assert!(curves.iter().all(|c| args.sketch.curves[c].construction));
        curves.len()
    };
    assert_eq!(projected(&editor), 1);
    editor.handle(dialog("project", Value::Elements(vec![])));
    assert_eq!(projected(&editor), 0);
}

/// The program's parameters are edited as a whole: the part is built
/// anew with them, what they resolve to — and why one does not — is
/// shown, invalid ones are refused, and the edit is undone like any other.
#[test]
fn parameters_are_edited_and_undone() {
    use crate::{Command, Editor};
    let mut editor = Editor::new();
    let program = examples::parametric_plate();
    editor.handle(Command::Load {
        program: program.clone(),
        path: None,
    });
    let mut parameters = parameters_of(&program);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[1].kind else {
        panic!("depth is a number")
    };
    *expression = "width / 4 + nothing".into();
    let update = editor.handle(Command::Parameters {
        parameters: parameters.clone(),
    });
    let state = update.program.unwrap();
    assert!(
        state.parameters.errors["depth"].contains("nothing"),
        "{state:?}"
    );
    let ParameterKind::Number { expression, .. } = &mut parameters.values[1].kind else {
        unreachable!()
    };
    *expression = "width / 4".into();
    let state = editor
        .handle(Command::Parameters {
            parameters: parameters.clone(),
        })
        .program
        .unwrap();
    assert_eq!(
        state.parameters.values.get("depth"),
        Some(&Setting::Number(1.0))
    );
    assert!(
        state.steps.iter().all(|s| s.error.is_none()),
        "{:?}",
        state.steps
    );
    parameters.values[1].name = "width".into();
    assert!(
        editor
            .handle(Command::Parameters { parameters })
            .error
            .is_some()
    );
    editor.handle(Command::Undo);
    editor.handle(Command::Undo);
    assert_eq!(parameters_of(editor.program()), parameters_of(&program));
}

/// The part drawn is listed beyond its faces — its datums, sketches,
/// solids — each shown or hidden as the editor would, until the user says
/// otherwise; and a placed part's mates are listed too, never drawn.
#[test]
fn the_structure_is_listed_and_shown_as_chosen() {
    use crate::operations::testing::handled;
    use crate::{Command, Editor};
    let mut editor = Editor::new();
    let update = editor.handle(Command::Load {
        program: examples::box_with_drill_hole(),
        path: None,
    });
    let scene = update.scene.unwrap();
    let listed = |kind: &str| -> Vec<(String, Option<bool>)> {
        scene
            .structure
            .iter()
            .filter(|i| i.kind == kind)
            .map(|i| (i.name.clone(), i.visible))
            .collect()
    };
    assert!(listed("datum").contains(&("origin".into(), Some(true))));
    // The sketches were extruded: hidden by themselves.
    assert_eq!(
        listed("sketch"),
        [
            ("outline".into(), Some(false)),
            ("hole_sketch".into(), Some(false))
        ]
    );
    let solid = listed("solid")[0].0.clone();
    assert_eq!(listed("solid"), [(solid.clone(), Some(true))]);

    let shown = |editor: &mut Editor, name: &str, visible: bool| {
        let update = handled(
            editor,
            Command::Visibility {
                name: name.into(),
                visible,
            },
        );
        let scene = update.scene.expect("what is hidden changed");
        assert!(
            scene
                .structure
                .iter()
                .any(|i| i.name == name && i.visible == Some(visible))
        );
        scene.hidden
    };
    let hidden = shown(&mut editor, "outline", true);
    assert!(!hidden.contains(&"outline".to_string()));
    let hidden = shown(&mut editor, &solid, false);
    assert!(hidden.contains(&solid));
    let hidden = shown(&mut editor, "origin", false);
    assert!(hidden.contains(&"origin".to_string()));
}

/// A parameter changed rebuilds the part — with no step edited, and with
/// one edited: the sketch open follows its formulas at once, solved, and
/// what it builds follows once it is committed. A table added on the fly
/// is read by its columns in a formula typed into a dimension.
#[test]
fn parameters_changed_rebuild_what_reads_them() {
    use crate::operations::testing::handled;
    use crate::{Command, Editor};
    use geop_core_editor::editing::{Control, Shape, StepEditEvent, Tone, Value};
    use geop_ops_parameters::{Parameter, Parameters, parameters::Row};
    let mut editor = Editor::new();
    let program = examples::parametric_plate();
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
        Command::Parameters {
            parameters: parameters.clone(),
        },
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
    let update = editor.handle(Command::Parameters {
        parameters: parameters.clone(),
    });
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
        Command::Parameters {
            parameters: parameters.clone(),
        },
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
    let mut program = examples::parametric_plate();
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

/// Undo and redo work while a sketch is edited, edit by edit — a drag one
/// edit however many events it takes — and the program's own undo comes
/// back once the sketch is put away.
#[test]
fn sketches_undo_their_own_edits() {
    use crate::operations::testing::{handled, loaded};
    use crate::{Command, Editor};
    use geop_core_editor::editing::{Button, StepEditEvent};
    let mut editor = loaded(Program::new());
    editor.handle(Command::New {
        kind: "add_sketch".into(),
    });
    let event = |editor: &mut Editor, event| editor.handle(Command::Event { event });
    let down = from_above;
    let click = |editor: &mut Editor, x, y| {
        event(
            editor,
            StepEditEvent::Click {
                pointer: down(x, y),
                button: Button::Primary,
                double: false,
                shift: false,
            },
        )
    };
    // The origin's xy plane's square, picked from above.
    click(&mut editor, 0.04, 0.04);
    event(&mut editor, StepEditEvent::Key { key: "l".into() });
    click(&mut editor, 0.5, 0.5);
    click(&mut editor, 1.5, 0.9);
    click(&mut editor, 1.2, 1.8);
    event(
        &mut editor,
        StepEditEvent::Key {
            key: "Escape".into(),
        },
    );
    event(
        &mut editor,
        StepEditEvent::Key {
            key: "Escape".into(),
        },
    );
    let drawn = |editor: &Editor| {
        let args: AddSketchArgs = args(editor.editing().unwrap());
        args.sketch.curves.values().filter(|c| !c.fixed).count()
    };
    let end = |editor: &Editor| {
        let args: AddSketchArgs = args(editor.editing().unwrap());
        let p = args
            .sketch
            .points
            .values()
            .rfind(|p| !p.fixed)
            .unwrap()
            .xy();
        [p[0].to_f64(), p[1].to_f64()]
    };
    assert_eq!(drawn(&editor), 2);
    // A drag, of three events: one edit.
    let before = end(&editor);
    event(
        &mut editor,
        StepEditEvent::Hover {
            pointer: down(before[0], before[1]),
            shift: false,
        },
    );
    for (to, done) in [([1.3, 1.9], false), ([1.4, 2.0], false), ([1.5, 2.1], true)] {
        event(
            &mut editor,
            StepEditEvent::Drag {
                from: down(before[0], before[1]),
                to: down(to[0], to[1]),
                done,
                shift: true,
            },
        );
    }
    assert!((end(&editor)[0] - 1.5).abs() < 1e-6, "{:?}", end(&editor));
    handled(&mut editor, Command::Undo);
    assert!(
        (end(&editor)[0] - before[0]).abs() < 1e-9,
        "{:?} vs {before:?}",
        end(&editor)
    );
    assert_eq!(drawn(&editor), 2);
    editor.handle(Command::Undo);
    assert_eq!(drawn(&editor), 1);
    let update = editor.handle(Command::Redo);
    let step = update.step.unwrap();
    assert!(step.can_undo && step.can_redo);
    assert_eq!(drawn(&editor), 2);
    editor.handle(Command::Redo);
    assert!((end(&editor)[0] - 1.5).abs() < 1e-6);
    // Put away, the program's own undo is back: the sketch added.
    let update = editor.handle(Command::Commit);
    assert!(update.program.unwrap().can_undo);
    editor.handle(Command::Undo);
    assert!(editor.program().steps.is_empty());
}

/// The box's top, with its hole, projected onto a plane seen from the side
/// — its hole's circle edge-on — and a rectangle drawn on its projected
/// corners: the sketch's regions are found — "could not tell which way it
/// winds" was a projected circle left as a spline lying flat along a line
/// — and only what was drawn bounds one.
#[test]
fn drawing_on_an_edge_on_projection() {
    let part = examples::box_with_drill_hole().built().unwrap();
    let (mut sketch, reference) = projected(&part, &on_origin(FrameAxis::Y), box_top());
    // Down from the top's two projected corners, and across.
    let corners: Vec<_> = reference.points.values().copied().collect();
    let xs: Vec<f64> = corners
        .iter()
        .map(|p| sketch.points[p].x.to_f64())
        .collect();
    let (lo, hi) = (
        corners[xs
            .iter()
            .position(|&x| x == xs.iter().cloned().fold(f64::MAX, f64::min))
            .unwrap()],
        corners[xs
            .iter()
            .position(|&x| x == xs.iter().cloned().fold(f64::MIN, f64::max))
            .unwrap()],
    );
    let below = |p: geop_core_sketch::PointId, s: &mut Sketch| {
        let q = s.points[&p];
        s.add_point(q.x, S::from_f64(q.y.to_f64() - 1.0))
    };
    let (lo2, hi2) = (below(lo, &mut sketch), below(hi, &mut sketch));
    for (a, b) in [(lo, hi), (hi, hi2), (hi2, lo2), (lo2, lo)] {
        sketch.add_line(a, b);
    }
    sketch.validate().unwrap();
    let regions = sketch.regions().unwrap();
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].outer.edges.len(), 4);
}

/// A parameter no step reads — the part's colour — changes the part
/// without building anything again: its colour is new, and every step's
/// part is the one already built.
#[test]
fn parameters_no_step_reads_rebuild_nothing() {
    let mut program = examples::parametric_plate();
    let mut running = Running::new();
    assert_eq!(running.run(&program, None), []);
    // Every step but those of its parameters and properties, which build
    // neither a sketch nor a model.
    let building = (program.steps.iter())
        .filter(|s| ![PARAMETERS, PROPERTIES].contains(&&*s.operation))
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
    let mut editor = Editor::new();
    let program = examples::parametric_plate();
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
    let update = editor.handle(Command::Parameters { parameters });
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

/// Sketches made with the sketch tools — patterns, mirror, offset — extrude
/// into valid solids.
mod tools {
    use std::f64::consts::PI;

    use geop_core_sketch::{copies::Step, offset::Corners};
    use geop_ops_booleans::Combine;
    use geop_ops_extrude_revolve::Extents;
    use geop_ops_sketch::Constraint;

    use super::*;
    use crate::examples::{circle, n, rectangle, solved};
    use crate::operations::testing::{check_valid, extrusion};

    /// `sketch`, solved, on the origin's `xy` plane, extruded up by
    /// `height`: the part, checked to be valid.
    fn extruded(sketch: Sketch, height: f64) -> geop_ops_model::NamedModel<S> {
        let mut program = Program::new();
        program.push(
            "outline",
            crate::operations::testing::sketch(on_origin(FrameAxis::Z), solved(sketch)),
        );
        program.push(
            "body",
            extrusion("outline", Extents::blind(height), Combine::NewBody),
        );
        let part = program.build_model().unwrap();
        if let Err(e) = check_valid(&part) {
            panic!("{e}");
        }
        assert_eq!(part.solid_names().len(), 1);
        part
    }

    /// A 4 x 3 plate with a bolt circle of six holes round its middle and a
    /// row of three along its bottom: each a pattern of one hole.
    #[test]
    fn a_plate_with_patterned_holes_extrudes() {
        let mut sketch = Sketch::new();
        let sides = rectangle(&mut sketch, [0.0, 0.0], 4.0, 3.0);
        let middle = sketch.add_point(n(2.0), n(1.7));
        sketch.constrain(Constraint::Fix {
            point: middle,
            x: n(2.0),
            y: n(1.7),
        });
        let bolt = circle(&mut sketch, [2.8, 1.7], 0.15);
        sketch
            .pattern(
                &[bolt],
                &Step::Round {
                    center: middle,
                    angle: n(PI / 3.0),
                },
                6,
            )
            .unwrap();
        let first = circle(&mut sketch, [0.8, 0.4], 0.1);
        sketch
            .pattern(
                &[first],
                &Step::Along {
                    along: sides[0],
                    backwards: false,
                    spacing: n(1.2),
                },
                3,
            )
            .unwrap();
        let part = extruded(sketch, 0.4);
        // The plate's six faces and a wall for each of the nine holes, at
        // least: none of them merged or lost.
        assert!(
            part.topology().faces.len() >= 15,
            "{}",
            part.topology().faces.len()
        );
    }

    /// Half a slot right of the `y` axis, its ends on the axis, mirrored
    /// into the whole slot, and offset outwards into a ring round it.
    #[test]
    fn a_mirrored_slot_offset_into_a_ring_extrudes() {
        let mut sketch = Sketch::new();
        let o = sketch.add_fixed_point(n(0.0), n(0.0));
        let up = sketch.add_fixed_point(n(0.0), n(1.0));
        let axis = sketch.add_line(o, up);
        sketch.set_construction(axis, true);
        let p = [[0.02, 0.0], [1.0, 0.03], [1.0, 1.0], [0.0, 1.02]]
            .map(|q| sketch.add_point(n(q[0]), n(q[1])));
        let bottom = sketch.add_line(p[0], p[1]);
        let round = sketch.add_arc(p[1], p[2], n(PI));
        let top = sketch.add_line(p[2], p[3]);
        for (a, b) in [(bottom, round), (round, top)] {
            sketch.constrain(Constraint::Tangent { a, b });
        }
        for point in [p[0], p[3]] {
            sketch.constrain(Constraint::PointOnCurve { point, curve: axis });
        }
        sketch.constrain(Constraint::Fix {
            point: p[0],
            x: n(0.0),
            y: n(0.0),
        });
        sketch.constrain(Constraint::Horizontal { line: bottom });
        sketch.constrain(Constraint::Horizontal { line: top });
        sketch.constrain(Constraint::Length {
            curve: bottom,
            value: n(1.0),
        });
        sketch.constrain(Constraint::Radius {
            curve: round,
            value: n(0.5),
        });
        let mirrored = sketch.mirror(&[bottom, round, top], axis).unwrap();
        let mut slot = vec![bottom, round, top];
        slot.extend(mirrored);
        let report = sketch.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        let chain = sketch.chain(&slot).unwrap();
        assert!(chain.closed);
        // Outwards: the slot's bottom runs to the right, so to its right.
        let ring = sketch
            .offset(&slot, Design::from_f64(-0.2), Corners::Round)
            .unwrap();
        assert_eq!(ring.curves.len(), 6, "a curve round each of the slot's");
        let part = extruded(sketch, 0.3);
        let z = |k: usize| {
            part.topology()
                .vertices
                .values()
                .map(|v| v.point[k].to_f64())
                .fold(f64::MIN, f64::max)
        };
        assert!(
            (z(0) - 1.7).abs() < 1e-9 && (z(1) - 1.2).abs() < 1e-9,
            "{} {}",
            z(0),
            z(1)
        );
    }

    /// The box's top projected into a sketch on it, and its outline offset
    /// 0.1 in: the offset is the sketch's one region — the projection is
    /// construction — fully constrained by the part, and it follows the
    /// part when the program is built, a pad on the box.
    #[test]
    fn projected_edges_offset_into_a_profile() {
        let mut program = examples::box_with_drill_hole();
        let built = program.built().unwrap();
        let plane = on_face("extrude(box,end)");
        let (mut sketch, reference) = projected(&built, &plane, box_top());
        let line = *sketch
            .curves
            .iter()
            .find(|(_, c)| matches!(c.kind, CurveKind::Line { .. }))
            .unwrap()
            .0;
        let outline = sketch.chain_through(line).unwrap();
        assert_eq!(outline.len(), 4);
        let chain = sketch.chain(&outline).unwrap();
        // Inwards: towards the outline's middle, wherever the face's own
        // frame puts it.
        let corners: Vec<[f64; 2]> = outline
            .iter()
            .flat_map(|c| sketch.curves[c].points())
            .map(|p| geop_core_sketch::plain::xy(&sketch, p))
            .collect();
        let middle =
            [0, 1].map(|k| corners.iter().map(|q| q[k]).sum::<f64>() / corners.len() as f64);
        let inside = sketch.chain_side(&chain, middle).unwrap();
        let toward = Design::from_f64(0.1 * inside.signum());
        sketch.offset(&outline, toward, Corners::Round).unwrap();
        let report = sketch.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert_eq!(sketch.regions().unwrap().len(), 1);
        program.push(
            "pad_sketch",
            AddSketchArgs {
                plane: Some(plane),
                sketch,
                references: vec![reference],
                ..Default::default()
            },
        );
        program.push(
            "pad",
            extrusion("pad_sketch", Extents::blind(0.2), Combine::NewBody),
        );
        let part = program.build_model().unwrap();
        if let Err(e) = check_valid(&part) {
            panic!("{e}");
        }
        let tallest = part
            .topology()
            .vertices
            .values()
            .map(|v| v.point[2].to_f64())
            .fold(f64::MIN, f64::max);
        assert!((tallest - 1.2).abs() < 1e-9, "{tallest}");
    }
}

/// A sketch projects another sketch's curve as it projects an edge: a
/// circle on the floor seen from above is that circle, keyed by the sketch
/// and the curve.
#[test]
fn sketch_curves_are_projected() {
    let mut program = Program::new();
    let mut floor = Sketch::new();
    let c = floor.add_point(examples::n(1.0), examples::n(1.0));
    let circle = floor.add_circle(c, examples::n(0.5));
    program.push("floor", sketch(on_origin(FrameAxis::Z), floor));
    let part = program.built().unwrap();
    let curve = Projected::SketchCurve(sketch_element("floor", circle));
    let (sketch, reference) = projected(&part, &on_origin(FrameAxis::Z), curve);
    assert_eq!(kinds(&sketch), ["circle"]);
    assert!(
        reference
            .curves
            .keys()
            .all(|k| k == &format!("floor,{circle}"))
    );
}

/// A sketch projects a 3-D sketch's curve as it projects an edge — it is
/// one, of the wire the 3-D sketch is built as: a line rising across the
/// view is the line beneath it, keyed by its edge's name.
#[test]
fn sketch3d_curves_are_projected() {
    let mut program = Program::new();
    let mut route = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| geop_core_math::vector::Vector3::from_array(p.map(examples::n));
    let (a, b) = (
        route.add_point(at([3.0, 0.0, 1.0])),
        route.add_point(at([4.0, 2.0, 3.0])),
    );
    route.add_line(a, b);
    program.push(
        "route",
        geop_ops_sketch3d::AddSketch3dArgs {
            sketch: route,
            references: Vec::new(),
        },
    );
    let part = program.built().unwrap();
    let edge = Projected::Edge(element("sketch3d(route,c2)"));
    let (sketch, reference) = projected(&part, &on_origin(FrameAxis::Z), edge);
    assert_eq!(kinds(&sketch), ["line"]);
    assert_eq!(
        reference.curves.keys().collect::<Vec<_>>(),
        ["sketch3d(route,c2)"]
    );
    let mut ends: Vec<[f64; 2]> = sketch
        .points
        .values()
        .map(|p| [p.x.to_f64(), p.y.to_f64()])
        .collect();
    ends.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let near = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9;
    assert!(
        near(ends[0], [3.0, 0.0]) && near(ends[1], [4.0, 2.0]),
        "{ends:?}"
    );
}
