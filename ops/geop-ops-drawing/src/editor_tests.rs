//! Drawings made as the front end makes them: a drawing step started, its
//! sheet shown head on, its views and title chosen in the dialog, the
//! part dimensioned and annotated by clicks on the sheet, dimensions
//! dragged and removed, and the sheet downloaded; an assembly's with its
//! parts list.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{Button, Control, Shape, StepEditEvent, Value},
    editor::{Command, Editor, Update},
    registry::Registry,
    testing::{dialog, loaded, pointer},
};
use geop_core_math::scalars::Scalar;
use geop_core_program::{program::Program, testing::args};
use geop_lib_stdlib::WithStandardParts;
use geop_ops_extrude_revolve::examples::box_with_drill_hole;

use crate::{Annotation, DrawnView, EdgeShape, Pickable, ViewKind};

/// What these tests edit: the parts and assemblies drawn, and drawings.
fn registry() -> Registry {
    crate::register(crate::program_tests::registry())
        .expect("the operations each have a name of their own")
}

/// An editor of [`registry`], the standard parts beside the files it is
/// sent.
fn editor() -> Editor {
    Editor::new(registry(), WithStandardParts(BTreeMap::new()))
}

/// An editor of [`registry`] on `program`, which loads without an error.
fn editor_on(program: Program) -> Editor {
    loaded(editor(), program)
}

/// The arguments of the drawing being edited.
fn drawing_args(editor: &Editor) -> crate::DrawingArgs {
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
fn sheet_vertices(editor: &Editor, view: crate::DrawnView) -> Vec<(String, [f64; 2])> {
    let args = drawing_args(editor);
    let layout = crate::layout(&editor.part(), &args, "", &[]).unwrap();
    layout
        .candidates
        .into_iter()
        .filter(|c| c.view == view)
        .filter_map(|c| match c.what {
            crate::Pickable::Point {
                target: crate::Target::Vertex { name },
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
    let mut editor = editor_on(box_with_drill_hole());
    editor.handle(Command::New {
        kind: "drawing".into(),
    });
    let args = drawing_args(&editor);
    let layout = crate::layout(&editor.part(), &args, "", &[]).unwrap();
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
    let mut editor = editor_on(box_with_drill_hole());
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
    let args: crate::DrawingArgs = geop_core_program::testing::args(step);
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
    let mut editor = editor();
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

/// A program with no drawing of its own, downloaded as a drawing, as
/// `geop compile bracket.geop -o bracket.dxf` writes it: the whole part
/// drawn, named after its file in the title block, in the format asked for
/// — DXF or SVG, and no other.
#[test]
fn a_program_is_drawn_whole() {
    let mut editor = editor();
    let update = editor.handle(Command::Load {
        program: geop_ops_extrude_revolve::examples::bracket(),
        path: Some("bracket.geop".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    let text = |update: Update| -> String {
        assert!(update.error.is_none(), "{:?}", update.error);
        let export = update.export.expect("a file is written");
        export.text().expect("a drawing is text").to_string()
    };
    let dxf = text(drawing_file(&mut editor, "dxf", ""));
    assert!(dxf.contains("\nAC1009\n") && dxf.ends_with("EOF\n"));
    for name in ["bracket", "THIRD ANGLE"] {
        assert!(dxf.contains(&format!("\n{name}\n")), "{name}");
    }
    let svg = text(drawing_file(&mut editor, "svg", ""));
    assert!(svg.starts_with("<svg"));
    let refused = drawing_file(&mut editor, "png", "");
    let error = refused.error.expect("no drawing is written as PNG");
    assert!(error.contains("svg, dxf"), "{error}");
}
