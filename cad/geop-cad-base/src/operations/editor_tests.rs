//! The editor of everything, as the front end drives it: commands that
//! edit the program, and only what changed sent back; the operations
//! offered and the examples loaded by name.

use geop_core_editor::{
    editing::{Button, Control, Shape, StepEditEvent, Value},
    output::Bounded,
};
use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_program::{
    element::PickedElement,
    program::{Program, ProgramStep},
};
use geop_ops_extrude_revolve::ExtrudeArgs;
use geop_ops_model::{Face, element};

use crate::{
    Command, Editor, Update,
    registry::testing::{args, click_at, dialog, from_above, handled, pointer},
};

/// The face `name` of the model, as a pick names it.
fn face(name: &str) -> PickedElement {
    element::<Face>(name).picked()
}

fn editor() -> (Editor, Update) {
    let mut editor = crate::editor();
    let update = handled(
        &mut editor,
        Command::LoadExample {
            name: "box_with_drill_hole".into(),
        },
    );
    (editor, update)
}

fn distance(editor: &Editor, id: &str) -> f64 {
    let program = editor.program();
    let args: ExtrudeArgs = args(&program.steps[program.index_of(id).unwrap()]);
    match &args.extent.side1 {
        geop_ops_extrude_revolve::Extent::Blind(d) => d.plain().expect("a plain distance"),
        other => panic!("{other:?} is no distance"),
    }
}

/// A new step goes in where the program runs to, once it builds; an
/// existing one is edited in place, moved and removed by id — and every
/// change can be undone and redone.
#[test]
fn commands_edit_the_program() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();

    let update = editor.handle(Command::New {
        kind: "extrude".into(),
    });
    let step = update.step.expect("a step is edited");
    assert_eq!(step.label, "Extrude");
    assert_eq!(step.id, None);
    let update = handled(&mut editor, Command::Commit);
    assert!(update.step.is_none());
    let steps = update.program.expect("the program changed").steps;
    assert_eq!(steps.last().unwrap().id, "extrude1");

    editor.handle(Command::Open {
        id: "extrude1".into(),
    });
    editor.handle(dialog("distance", Value::Number(2.0)));
    // Not in the program until committed.
    assert_eq!(distance(&editor, "extrude1"), 1.0);
    editor.handle(Command::Commit);
    assert_eq!(distance(&editor, "extrude1"), 2.0);

    editor.handle(Command::Move {
        id: "extrude1".into(),
        index: 0,
    });
    assert_eq!(editor.program().steps[0].id, "extrude1");
    editor.handle(Command::Remove {
        id: "extrude1".into(),
    });
    assert_eq!(*editor.program(), before);

    for _ in 0..3 {
        editor.handle(Command::Undo);
    }
    assert_eq!(distance(&editor, "extrude1"), 1.0);
    let update = editor.handle(Command::Redo);
    assert!(update.program.unwrap().can_redo);
    assert_eq!(distance(&editor, "extrude1"), 2.0);
}

/// A command that would break the program is refused, saying why, and
/// changes nothing.
#[test]
fn bad_commands_are_refused() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();
    let mut twice = before.clone();
    twice.steps.push(twice.steps[0].clone());
    for bad in [
        Command::Remove { id: "nope".into() },
        Command::Move {
            id: "box".into(),
            index: before.steps.len(),
        },
        Command::Load {
            program: twice,
            path: None,
        },
        Command::Commit,
        Command::New {
            kind: "no_such_operation".into(),
        },
    ] {
        let update = editor.handle(bad);
        assert!(update.error.is_some());
        assert_eq!(*editor.program(), before);
    }

    // A step that does not build cannot go in: without the box, the hole
    // has no face to be sketched on.
    let update = editor.handle(Command::Remove { id: "box".into() });
    assert!(update.error.is_none());
    let update = editor.handle(Command::Open {
        id: "hole_sketch".into(),
    });
    assert!(update.step.unwrap().error.is_some());
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_some());
}

/// The part is sent again only when what is drawn changed, the program only
/// when it or its run did — a hover sends neither.
#[test]
fn only_what_changed_is_sent() {
    let (mut editor, update) = editor();
    assert!(update.program.is_some() && update.scene.is_some());
    let update = editor.handle(Command::Open { id: "hole".into() });
    assert!(update.program.is_some(), "the step is marked as edited");
    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: pointer([5.0, 5.0, 5.0], [0.0, 0.0, -1.0]),
            shift: false,
        },
    });
    assert!(update.program.is_none() && update.scene.is_none());
    assert!(update.step.is_some());
    let update = editor.handle(Command::Preview { preview: false });
    assert!(update.scene.is_some(), "the part before the step is drawn");
    let update = editor.handle(Command::Show);
    assert!(update.program.is_some() && update.scene.is_some());
}

/// Every example loads by name, and builds.
#[test]
fn examples_load_by_name() {
    let mut editor = crate::editor();
    let names = editor.handle(Command::Show).program.unwrap().examples;
    assert_eq!(
        names.len(),
        crate::registry().unwrap().example_programs().len()
    );
    let update = editor.handle(Command::LoadExample {
        name: names[0].into(),
    });
    let steps = update.program.unwrap().steps;
    assert!(steps.iter().all(|s| s.error.is_none()), "{steps:?}");
    assert_ne!(*editor.program(), Program::new());
}

/// The toolbar's sections are the editor's: it sends the operations group
/// by group, in the groups' order, every operation in one, and within a
/// group the few used most (shown big) first, then the common ones (small),
/// then those only in the group's menu.
#[test]
fn operations_are_offered_by_group() {
    use geop_core_editor::editable::OperationTier;
    let (_, update) = editor();
    let operations = update.program.expect("the program is sent").operations;
    assert_eq!(operations.len(), crate::registry().unwrap().infos().len());
    let order: Vec<_> = operations
        .iter()
        .map(|o| (o.info.group, o.info.tier))
        .collect();
    assert!(
        order.is_sorted(),
        "not group by group, the most used first: {order:?}"
    );
    let big: Vec<&str> = operations
        .iter()
        .filter(|o| o.info.tier == OperationTier::Big)
        .map(|o| o.kind)
        .collect();
    assert_eq!(
        big,
        [
            "add_sketch",
            "extrude",
            "revolve",
            "hole",
            "fillet",
            "boolean",
            "linear_pattern",
            "subd",
            "base_flange",
            "add_part",
            "drawing",
        ]
    );
    // The part's own parameters and properties come first, in its menu.
    let json = serde_json::to_value(&operations[0]).unwrap();
    assert_eq!(json["group"], "Part");
    assert_eq!(json["tier"], "Menu");
    let sketch = operations.iter().find(|o| o.kind == "add_sketch").unwrap();
    let json = serde_json::to_value(sketch).unwrap();
    assert_eq!(json["group"], "Sketch");
    assert_eq!(json["tier"], "Big");
    let mirror = operations.iter().find(|o| o.kind == "mirror").unwrap();
    assert_eq!(
        serde_json::to_value(mirror).unwrap()["group"],
        "Pattern & bodies"
    );
}

/// A program the editor is given is checked against its operations before
/// it is taken: one loaded or pasted with a step of an operation there is
/// none of, or with arguments its operation does not read, is refused,
/// naming the step, and the program edited stays as it was.
#[test]
fn a_program_of_unknown_steps_is_refused() {
    let mut editor = crate::editor();
    let mut unknown = Program::new();
    unknown.steps.push(ProgramStep {
        id: "mystery".into(),
        operation: "no_such_operation".into(),
        args: serde_json::json!({}),
    });
    let refused = editor.handle(Command::Load {
        program: unknown.clone(),
        path: None,
    });
    let error = refused.error.unwrap();
    assert!(
        error.contains("step `mystery`") && error.contains("no_such_operation"),
        "{error}"
    );
    assert!(editor.program().steps.is_empty());

    let refused = editor.handle(Command::Paste {
        text: unknown.to_json().unwrap(),
    });
    assert!(refused.error.unwrap().contains("no_such_operation"));
    assert!(editor.program().steps.is_empty());

    let mut unreadable = geop_ops_extrude_revolve::examples::box_with_drill_hole();
    unreadable.steps[0].args = serde_json::json!("not arguments");
    let refused = editor.handle(Command::Load {
        program: unreadable,
        path: None,
    });
    let error = refused.error.unwrap();
    assert!(error.contains("arguments of operation"), "{error}");
    assert!(editor.program().steps.is_empty());
}

/// The measure tool, used as the viewer uses it: taken in hand, the box's
/// top hovered and clicked — its area, its plane — then its bottom clicked
/// from below: one unit apart, parallel, the least distance drawn between
/// them. Then the mass properties and the interference of the part asked
/// for. None of it changes the program, and the tool put down shows nothing.
#[test]
fn measure_tool_measures_what_is_clicked() {
    let (mut editor, _) = editor();
    let before = editor.program().clone();
    let update = editor.handle(Command::Tool {
        key: Some("pick:measure".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(
        update.program.unwrap().tool.as_deref(),
        Some("pick:measure")
    );
    let top = face("extrude(box,end)");
    let bottom = face("extrude(box,start)");
    let from_below = pointer([0.5, 0.5, -10.0], [0.0, 0.0, 1.0]);
    let measured = |update: &Update| match &update.inspection {
        Some(inspection) if inspection.kind == "measure" => inspection.report.clone(),
        other => panic!("no measurement: {other:?}"),
    };
    let value = |m: &serde_json::Value, label: &str| {
        let found = (m["values"].as_array().unwrap().iter())
            .find(|v| v["label"] == label)
            .unwrap_or_else(|| panic!("no {label}: {m}"));
        Bounded {
            value: found["value"].as_f64().unwrap(),
            error: found["error"].as_f64().unwrap(),
        }
    };

    let update = editor.handle(Command::Event {
        event: StepEditEvent::Hover {
            pointer: from_above(0.5, 0.5),
            shift: false,
        },
    });
    assert_eq!(update.tool.unwrap().highlights, vec![top.clone()]);

    let update = editor.handle(click_at(from_above(0.5, 0.5)));
    assert!(update.program.is_none(), "measuring changes no program");
    let m = measured(&update);
    assert_eq!(m["entities"], serde_json::json!([top]));
    let hole = std::f64::consts::PI * 0.4 * 0.4;
    assert!(value(&m, "Area").contains(4.0 - hole), "{m}");
    assert!(!m["plane"].is_null());

    let update = editor.handle(click_at(from_below));
    let m = measured(&update);
    assert_eq!(m["entities"], serde_json::json!([top, bottom]));
    assert!(value(&m, "Distance").contains(1.0), "{m}");
    assert!(value(&m, "Angle").contains(0.0), "{m}");
    let tool = update.tool.unwrap();
    assert!(tool.visuals.iter().any(|v| v.key == "distance"));

    let update = editor.handle(Command::Inspect {
        inspector: "mass_properties".into(),
        options: serde_json::Value::Null,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let report = update.inspection.unwrap().report;
    assert_eq!(report["bodies"].as_array().unwrap().len(), 1);
    let volume = &report["total"]["volume"];
    let volume = Bounded {
        value: volume["value"].as_f64().unwrap(),
        error: volume["error"].as_f64().unwrap(),
    };
    assert!(volume.contains(4.0 - hole * 0.5), "{volume:?}");

    let update = editor.handle(Command::Inspect {
        inspector: "interference".into(),
        options: serde_json::Value::Null,
    });
    let report = update.inspection.unwrap().report;
    assert_eq!(report["solids"], 1);
    assert_eq!(report["found"], serde_json::json!([]));

    let update = editor.handle(Command::Tool { key: None });
    assert!(update.tool.is_none() && update.inspection.is_none());
    assert_eq!(*editor.program(), before);
}

/// The arguments of the drawing being edited.
fn drawing_args(editor: &Editor) -> geop_ops_drawing::DrawingArgs {
    args(editor.editing().expect("a drawing is edited"))
}

/// The drawing `editor` shows, downloaded as the dialog's buttons do, in
/// `format`, dated `date`.
fn drawing_file(editor: &mut Editor, format: &str, date: &str) -> Update {
    editor.handle(Command::Export {
        exporter: "drawing".into(),
        format: Some(format.into()),
        options: serde_json::Value::Null,
        date: date.into(),
    })
}

/// Where the vertices of the drawing being edited are on its sheet, in the
/// view `view`, as the front end shows them: by name.
fn sheet_vertices(editor: &Editor, view: geop_ops_drawing::DrawnView) -> Vec<(String, [f64; 2])> {
    let args = drawing_args(editor);
    let layout = geop_ops_drawing::layout(&editor.part(), &args, "", &[]).unwrap();
    layout
        .candidates
        .into_iter()
        .filter(|c| c.view == view)
        .filter_map(|c| match c.what {
            geop_ops_drawing::Pickable::Point {
                target: geop_ops_drawing::Target::Vertex { name },
                at,
            } => Some((name, at)),
            _ => None,
        })
        .collect()
}

/// The drilled hole's diameter dimensioned on the drawing's sheet by
/// clicking its rim from above, and a centre mark put on it; the diameter,
/// selected in the list, removed with Delete.
#[test]
fn a_hole_is_dimensioned_on_a_drawing_and_a_dimension_removed() {
    use geop_ops_drawing::{Annotation, DrawnView, EdgeShape, Pickable, ViewKind};
    let (mut editor, _) = editor();
    editor.handle(Command::New {
        kind: "drawing".into(),
    });
    let args = drawing_args(&editor);
    let layout = geop_ops_drawing::layout(&editor.part(), &args, "", &[]).unwrap();
    let rim = layout
        .candidates
        .iter()
        .filter(|c| c.view == DrawnView::View(ViewKind::Top))
        .find_map(|c| match &c.what {
            Pickable::Edge {
                points,
                shape: EdgeShape::Circle { .. },
                ..
            } => Some(points[points.len() / 2]),
            _ => None,
        })
        .expect("the hole's rim, seen round from above");
    let click = |p: [f64; 2]| Command::Event {
        event: StepEditEvent::Click {
            pointer: pointer([p[0], p[1], 10.0], [0.0, 0.0, -1.0]),
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let key = |key: &str| Command::Event {
        event: StepEditEvent::Key { key: key.into() },
    };
    editor.handle(dialog("tools", Value::Choice("diameter".into())));
    editor.handle(click(rim));
    let placed = editor.handle(click([rim[0] + 15.0, rim[1] + 15.0]));
    let step = placed.step.unwrap();
    let Some(Control::List { items, .. }) = step.presentation.dialog.get("annotations") else {
        panic!("the annotations are listed");
    };
    let detail = items[0].detail.clone().unwrap_or_default();
    assert!(
        detail.starts_with('⌀') && detail.ends_with("top"),
        "{items:?}"
    );
    editor.handle(dialog("tools", Value::Choice("center_mark".into())));
    editor.handle(click(rim));
    let args = drawing_args(&editor);
    assert!(matches!(
        args.annotations.as_slice(),
        [
            Annotation::Radius { diameter: true, .. },
            Annotation::CenterMark { .. }
        ]
    ));

    editor.handle(key("Escape"));
    editor.handle(dialog("annotation:0", Value::Press));
    let update = editor.handle(key("Delete"));
    assert!(update.error.is_none(), "{:?}", update.error);
    let args = drawing_args(&editor);
    assert!(
        matches!(args.annotations.as_slice(), [Annotation::CenterMark { .. }]),
        "{:?}",
        args.annotations
    );
}

/// A drawing step, the way the front end makes one: started — its sheet
/// shown head on, no part drawn — its views chosen and the title block's
/// name typed; a diagonal of the box's top dimensioned by clicking two of
/// its corners in the top view and then where the value goes, the value
/// dragged further out, a note added pointing at a corner; then
/// downloaded from the dialog as SVG and DXF, and committed.
#[test]
fn a_drawing_is_annotated_on_its_sheet_and_downloaded() {
    use geop_ops_drawing::{Annotation, DrawnView, ViewKind};
    let (mut editor, _) = editor();
    let started = editor.handle(Command::New {
        kind: "drawing".into(),
    });
    assert!(started.error.is_none(), "{:?}", started.error);
    let step = started.step.unwrap();
    // The sheet, in the world's xy plane, and nothing of the part.
    let sheet = step.presentation.sheet.expect("the sheet is shown");
    assert!(step.presentation.focus.is_some());
    assert!(sheet.size.to_f64() > 400.0, "an A3 sheet");
    let scene = started.scene.expect("a scene");
    assert!(scene.part.faces.is_empty() && scene.part.datums.is_empty());
    assert!(matches!(
        step.presentation.dialog.get("download"),
        Some(Control::Download { formats, .. }) if formats.len() == 2
    ));
    editor.handle(dialog("view:iso", Value::Bool(false)));
    editor.handle(dialog("title:name", Value::Text("Drilled box".into())));

    // Two opposite corners of the box's top, seen from above.
    let top = DrawnView::View(ViewKind::Top);
    let corners = sheet_vertices(&editor, top);
    let (low, high) = {
        let by = |f: fn(f64, f64) -> bool| {
            corners
                .iter()
                .cloned()
                .reduce(|a, b| {
                    if f(b.1[0] + b.1[1], a.1[0] + a.1[1]) {
                        b
                    } else {
                        a
                    }
                })
                .unwrap()
        };
        (by(|a, b| a < b), by(|a, b| a > b))
    };
    let at = |p: [f64; 2]| pointer([p[0], p[1], 10.0], [0.0, 0.0, -1.0]);
    let event = |event| Command::Event { event };
    let click = |p: [f64; 2]| {
        event(StepEditEvent::Click {
            pointer: at(p),
            button: Button::Primary,
            double: false,
            shift: false,
        })
    };
    let hover = |p: [f64; 2]| {
        event(StepEditEvent::Hover {
            pointer: at(p),
            shift: false,
        })
    };
    editor.handle(dialog("tools", Value::Choice("distance".into())));
    let hovered = editor.handle(hover(low.1)).step.unwrap();
    assert!(
        hovered
            .presentation
            .visuals
            .iter()
            .any(|v| v.key == "hover"),
        "the corner under the pointer is lit"
    );
    editor.handle(click(low.1));
    editor.handle(click(high.1));
    // The dimension follows the pointer, and goes down where clicked.
    let middle = [(low.1[0] + high.1[0]) / 2.0, (low.1[1] + high.1[1]) / 2.0];
    let out = [middle[0] - 10.0, middle[1] + 10.0];
    let placing = editor.handle(hover(out)).step.unwrap();
    assert!(
        placing
            .presentation
            .visuals
            .iter()
            .any(|v| v.key == "placing"),
        "the dimension being placed is shown"
    );
    editor.handle(click(out));
    let args = drawing_args(&editor);
    let [
        Annotation::Distance {
            from, to, label, ..
        },
    ] = args.annotations.as_slice()
    else {
        panic!("one distance: {:?}", args.annotations);
    };
    assert_eq!(
        (from.to_string(), to.to_string()),
        (low.0.clone(), high.0.clone())
    );
    assert!(
        (label[0] + 10.0).abs() < 1e-9 && (label[1] - 10.0).abs() < 1e-9,
        "{label:?}"
    );

    // Put down, the tool lets its value be dragged.
    editor.handle(event(StepEditEvent::Key {
        key: "Escape".into(),
    }));
    let update = editor.handle(event(StepEditEvent::Key {
        key: "Escape".into(),
    }));
    let step = update.step.unwrap();
    let Some(Control::List { items, .. }) = step.presentation.dialog.get("annotations") else {
        panic!("the annotations are listed");
    };
    assert_eq!(items[0].detail.as_deref(), Some("2.83 · top"), "{items:?}");
    editor.handle(hover(out));
    let dragged = editor.handle(event(StepEditEvent::Drag {
        from: at(out),
        to: at([out[0] - 5.0, out[1] + 5.0]),
        done: true,
        shift: false,
    }));
    assert!(dragged.error.is_none(), "{:?}", dragged.error);
    let args = drawing_args(&editor);
    let label = match &args.annotations[0] {
        Annotation::Distance { label, .. } => *label,
        other => panic!("{other:?}"),
    };
    assert!(
        (label[0] + 15.0).abs() < 1e-9 && (label[1] - 15.0).abs() < 1e-9,
        "{label:?}"
    );

    // A note, pointing at a corner.
    editor.handle(dialog("tools", Value::Choice("note".into())));
    editor.handle(dialog("note_text", Value::Text("DEBURR".into())));
    editor.handle(click(low.1));
    editor.handle(click([low.1[0] - 20.0, low.1[1] - 20.0]));
    let args = drawing_args(&editor);
    assert!(
        matches!(&args.annotations[1], Annotation::Note { text, leader: Some(_), .. } if text == "DEBURR"),
        "{:?}",
        args.annotations
    );

    // Downloaded from the dialog, as it now is.
    let exported = drawing_file(&mut editor, "svg", "2026-10-04");
    assert!(exported.error.is_none(), "{:?}", exported.error);
    let file = exported.export.unwrap();
    assert_eq!(file.name, "part.svg");
    let svg = file.text().unwrap();
    assert!(svg.starts_with("<svg"));
    for text in ["Drilled box", ">2.83<", ">DEBURR<"] {
        assert!(svg.contains(text), "{text} is not on the sheet");
    }
    // The blind hole, seen from the front, is hidden.
    let hidden = &svg[svg.find(r#"<g class="HIDDEN""#).unwrap()..];
    assert!(hidden[..hidden.find("</g>").unwrap()].contains("<line"));
    let dxf = drawing_file(&mut editor, "dxf", "");
    let dxf = dxf.export.unwrap();
    assert!(dxf.text().unwrap().ends_with("EOF\n") && dxf.text().unwrap().contains("DEBURR"));

    let committed = editor.handle(Command::Commit);
    assert!(committed.error.is_none(), "{:?}", committed.error);
    // Back to the part: the scene draws its faces again.
    assert!(!committed.scene.expect("a scene").part.faces.is_empty());
    let step = editor.program().steps.last().unwrap();
    assert_eq!(step.operation, "drawing");
    let args: geop_ops_drawing::DrawingArgs = crate::registry::testing::args(step);
    assert_eq!((args.views.len(), args.annotations.len()), (3, 2));
    assert_eq!(args.name, "Drilled box");
}

/// The bolted plate's drawing with its bill of materials, as the front end
/// makes it: a new drawing step, "Bill of materials" ticked, committed and
/// exported. The sheet draws the parts placed, where they are placed, and
/// lists them: the plate, and the standard screw and nut by the titles and
/// designations their own programs carry — which name their products in
/// a STEP file too (see [`an_assembly_step_file_names_its_parts`]) — each
/// line ballooned once with its item number.
#[test]
fn an_assembly_drawing_lists_its_parts() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "bolted_plate".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "drawing".into(),
    });
    let step = update.step.expect("a drawing is edited");
    assert!(matches!(
        step.presentation.dialog.get("bom"),
        Some(Control::Checkbox { value: false, .. })
    ));
    let ticked = editor.handle(dialog("bom", Value::Bool(true)));
    // The sheet shown is the one downloaded: its parts list, given by the
    // editor, and the balloons pointing at the parts.
    let shown: Vec<String> = ticked
        .step
        .expect("a drawing is edited")
        .presentation
        .visuals
        .into_iter()
        .filter_map(|v| match v.shape {
            Shape::Label { text, .. } => Some(text),
            _ => None,
        })
        .collect();
    for text in ["ITEM", "ISO 4762 M4x12", "ISO 4032 M4"] {
        assert!(
            shown.iter().any(|t| t == text),
            "{text} is not shown: {shown:?}"
        );
    }
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);

    let exported = drawing_file(&mut editor, "svg", "2026-10-04");
    assert!(exported.error.is_none(), "{:?}", exported.error);
    let svg = exported.export.unwrap().text().unwrap().to_string();
    for text in [
        ">ITEM<",
        ">DESIGNATION<",
        ">ISO 4762 M4x12<",
        ">ISO 4032 M4<",
        ">plate<",
        ">Steel<",
    ] {
        assert!(svg.contains(text), "{text} is not on the sheet");
    }
    // The four default views draw the parts, with their hidden lines; three
    // balloons, each with its item number, stand on the dimension layer.
    let layer = |name: &str| {
        let start = svg.find(&format!("<g class=\"{name}\"")).unwrap();
        let end = start + svg[start..].find("</g>").unwrap();
        svg[start..end].to_string()
    };
    assert!(layer("VISIBLE").contains("<line"), "no view line");
    assert!(layer("HIDDEN").contains("<line"), "no hidden line");
    let dimensions = layer("DIMENSIONS");
    assert_eq!(dimensions.matches("r=\"4\"/>").count(), 3, "{dimensions}");
    for item in ["1", "2", "3"] {
        assert!(
            dimensions.contains(&format!(">{item}</text>")),
            "balloon {item}"
        );
    }
}

/// The arm, exported as a URDF robot the way the front end asks — the
/// command and the update as JSON: a ZIP archive named after the file, its
/// bytes in base64, holding `robot.urdf` with the arm's joints. Refused for
/// the four-bar, whose bars only mates hold: the error names them.
#[test]
fn an_assembly_is_exported_as_urdf() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let json = editor
        .handle_json(r#"{"command": "export", "exporter": "urdf"}"#)
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    let export = &update["export"];
    assert_eq!(export["name"], "arm.zip");
    assert!(export["text"].is_null());
    // "PK\x03\x04", a ZIP archive's first local header, in base64.
    let bytes = export["bytes"].as_str().unwrap();
    assert!(bytes.starts_with("UEsDBA"), "{}", &bytes[..16]);
    assert_eq!(bytes.len() % 4, 0);

    let urdf = |editor: &mut Editor| {
        editor.handle(Command::Export {
            exporter: "urdf".into(),
            format: None,
            options: serde_json::Value::Null,
            date: String::new(),
        })
    };
    let zip = urdf(&mut editor).export.unwrap();
    let text = String::from_utf8_lossy(zip.bytes());
    assert!(text.contains("robot.urdf") && text.contains("meshes/link.stl"));
    assert!(text.contains(r#"<joint name="add_part(fore,m1)" type="revolute">"#));
    assert!(text.contains(r#"<joint name="add_part(hand,m1)" type="revolute">"#));

    editor.handle(Command::LoadWorkspaceExample {
        name: "four_bar".into(),
        folder: None,
    });
    let refused = urdf(&mut editor);
    assert!(refused.export.is_none());
    let error = refused.error.unwrap();
    assert!(
        error.contains("free to move") && error.contains("coupler"),
        "{error}"
    );
}

/// The part shown, downloaded as STEP the way the front end asks for it.
fn step_file(editor: &mut Editor) -> String {
    let update = editor.handle(Command::Export {
        exporter: "step".into(),
        format: None,
        options: serde_json::Value::Null,
        date: String::new(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let export = update.export.expect("a file is written");
    assert!(export.name.ends_with(".step"), "{}", export.name);
    let text = export.text().expect("STEP is text").to_string();
    assert!(text.contains("MANIFOLD_SOLID_BREP"), "no solid in {text}");
    text
}

/// The part shown written as STEP, stored next to a new program in a
/// folder — as the front end stores a file the user chooses in the file
/// field — is offered by a new "Import STEP" step by its path relative to
/// the program, and brings its solid back, named after the step.
#[test]
fn a_part_exported_as_step_is_imported_back() {
    let (mut editor, _) = editor();
    let files = std::collections::BTreeMap::from([(
        "parts/box.step".to_string(),
        Some(step_file(&mut editor)),
    )]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("parts/imported.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let update = editor.handle(Command::New {
        kind: "import_step".into(),
    });
    let step = update.step.expect("a step is edited");
    assert_eq!(step.label, "Import STEP");
    let Some(Control::File {
        options, accept, ..
    }) = step.presentation.dialog.get("file")
    else {
        panic!("the file is a file field");
    };
    assert!(options.iter().any(|o| o.value == "box.step"), "{options:?}");
    assert_eq!(accept, &["step", "stp"]);
    let update = editor.handle(dialog("file", Value::Choice("box.step".into())));
    assert!(update.step.unwrap().error.is_none());
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let id = editor.program().steps.last().unwrap().id.clone();
    let solids: Vec<String> = update
        .scene
        .expect("the scene changed")
        .structure
        .into_iter()
        .filter(|item| item.kind == "solid")
        .map(|item| item.name)
        .collect();
    assert_eq!(solids, vec![format!("import({id},s0)")]);
}

/// A STEP file whose corner lies 3e-4 above where its three faces meet —
/// further than the kernel can carry as one point — imports with the corner
/// put back where they meet, and the dialog says what was rebuilt, and how
/// far it moved.
#[test]
fn an_import_that_heals_the_file_says_what_it_rebuilt() {
    let (mut editor, _) = editor();
    let text = step_file(&mut editor);
    // The first vertex's point, raised.
    let at = text.find("=VERTEX_POINT(").expect("a vertex");
    let id = &text[at..];
    let id = &id[id.find("',#").unwrap() + 3..id.find(");").unwrap()];
    let line = format!("#{id}=CARTESIAN_POINT('',(");
    let start = text.find(&line).expect("its point") + line.len();
    let end = start + text[start..].find("))").unwrap();
    let mut xyz: Vec<f64> = text[start..end]
        .split(',')
        .map(|c| c.parse().unwrap())
        .collect();
    xyz[2] += 3e-4;
    let raised = format!(
        "{}{:?},{:?},{:?}{}",
        &text[..start],
        xyz[0],
        xyz[1],
        xyz[2],
        &text[end..]
    );

    let files = std::collections::BTreeMap::from([("box.step".to_string(), Some(raised))]);
    assert!(editor.handle(Command::Files { files }).error.is_none());
    let update = editor.handle(Command::Load {
        program: Program::new(),
        path: Some("imported.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "import_step".into(),
    });
    let update = editor.handle(dialog("file", Value::Choice("box.step".into())));
    let step = update.step.expect("a step is edited");
    assert!(step.error.is_none(), "{:?}", step.error);
    let Some(Control::Text { text, .. }) = step.presentation.dialog.get("healed_0") else {
        panic!(
            "the dialog does not say what was rebuilt: {:?}",
            step.presentation.dialog
        );
    };
    assert!(
        text.contains("by up to 3.0e-4 mm") && text.contains("1 vertices and"),
        "{text}"
    );
    assert!(editor.handle(Command::Commit).error.is_none());
}

/// The arm, exported as STEP the way the front end asks: one product for
/// the arm and one for each of the three links it places, each placement an
/// occurrence named after its step — and read back, the three links where
/// the arm has them.
#[test]
fn an_assembly_is_exported_as_step_products() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "arm".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let text = step_file(&mut editor);
    // TODO: one product of the link for all three, once the parts of no
    // state are shared between instances (see
    // `geop_core_program::part::Part`).
    assert_eq!(
        text.matches("=PRODUCT('").count(),
        4,
        "the arm and each link"
    );
    assert!(text.contains("PRODUCT('link','link'"));
    for occurrence in ["upper", "fore", "hand"] {
        assert!(
            text.contains(&format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('{occurrence}'")),
            "{occurrence} is placed"
        );
    }
    let bodies = geop_ops_step::read_step::<S>(&text).unwrap();
    assert_eq!(bodies.len(), 3, "the link, once where each step places it");
}

/// The bolted plate written as STEP: the standard screw and nut are named
/// by the designations their own programs carry, one product per size.
#[test]
fn an_assembly_step_file_names_its_parts() {
    let mut editor = crate::editor();
    handled(
        &mut editor,
        Command::LoadWorkspaceExample {
            name: "bolted_plate".into(),
            folder: None,
        },
    );
    let text = step_file(&mut editor);
    assert!(
        text.contains("PRODUCT('ISO 4762 M4x12'"),
        "the screw's product"
    );
    assert!(text.contains("PRODUCT('plate'"), "the plate's product");
}

/// The bill of materials, asked for the way the inspect panel asks — the
/// command and the update as JSON: the bolted plate's three parts, the
/// screw and nut designated by their norms. Of the part shown: seeking
/// back before the nut drops it. Exported as CSV, indented, named after
/// the file.
#[test]
fn the_bill_of_materials_is_asked_for_and_exported() {
    let mut editor = crate::editor();
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: "bolted_plate".into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let ask = r#"{"command": "inspect", "inspector": "bom", "options": {"structure": "flat"}}"#;
    let json = editor.handle_json(ask).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["inspection"]["kind"], "bom");
    let bom = &update["inspection"]["report"];
    assert_eq!(bom["structure"], "flat");
    let designations: Vec<&str> = (bom["lines"].as_array().unwrap().iter())
        .map(|l| l["designation"].as_str().unwrap_or("-"))
        .collect();
    assert_eq!(designations, ["-", "ISO 4762 M4x12", "ISO 4032 M4"]);
    let screw = &bom["lines"][1];
    assert_eq!(
        (&screw["kind"], &screw["quantity"]),
        (&"part".into(), &1.into())
    );
    assert_eq!(screw["material"], "Steel");
    assert!(screw["unit_mass"]["value"].as_f64().unwrap() > 0.0);
    assert!(bom["total_mass"]["value"].as_f64().unwrap() > 0.0);

    // Up to the screw: no nut yet.
    editor.handle(Command::Seek { marker: Some(2) });
    let json = editor.handle_json(ask).unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    let lines = update["inspection"]["report"]["lines"].as_array().unwrap();
    assert_eq!(lines.len(), 2);
    editor.handle(Command::Seek { marker: None });

    let json = editor
        .handle_json(
            r#"{"command": "export", "exporter": "bom", "options": {"structure": "indented"}}"#,
        )
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["export"]["name"], "bolted_plate_bom.csv");
    let csv = update["export"]["text"].as_str().unwrap();
    let rows: Vec<&str> = csv.lines().collect();
    assert_eq!(rows.len(), 1 + 3 + 1, "{csv}");
    assert!(
        rows[2].starts_with("2,1,1,ISO 4762 socket head cap screw,ISO 4762 M4x12,"),
        "{csv}"
    );
    assert!(rows[4].contains("Total"), "{csv}");
}
