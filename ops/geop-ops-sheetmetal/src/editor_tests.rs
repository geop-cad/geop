//! Sheet metal in the editor, the way the front end does it: the bracket
//! example flanged, cut, hemmed and unfolded by steps picked with clicks
//! and set in their dialogs, and its flat pattern exported for cutting.

use geop_core_editor::{
    editing::{Button, StepEditEvent, Value},
    editor::{Command, Editor},
    registry::Registry,
    testing::{click_at, dialog, editor_with, from_above, handled, pointer},
};
use geop_core_math::geop_error::GeopResult;
use geop_ops_extrude_revolve::testing::{args, disc};
use geop_ops_model::{element, solid};
use geop_ops_sketch::{AddSketchArgs, SketchPlane, whole_sketch};

use crate::{EdgeFlangeArgs, FlatPatternArgs, HemArgs, HemKind, SheetCutArgs, edge};

/// What these tests edit: sketches on the origin's planes, and sheet
/// metal, with the examples.
fn registry() -> GeopResult<Registry> {
    let r = Registry::default();
    let r = geop_ops_model::register(r)?;
    let r = geop_ops_sketch::register(r)?;
    let r = geop_ops_datums::register(r)?;
    let r = crate::register(r)?;
    geop_ops_parameters::register(r)
}

/// An editor of [`registry`] on the bracket example.
fn bracket_editor() -> Editor {
    editor_with(registry().unwrap(), crate::examples::sheet_metal_bracket())
}

/// A new edge flange picks its edge by a click onto the bracket's right
/// edge from above — the plate's top edge there — builds as the dialog
/// sets it, and a new flat pattern starts from the flanged body.
#[test]
fn new_edge_flange_picks_its_edge_and_unfolds() {
    let mut editor = bracket_editor();
    handled(
        &mut editor,
        Command::New {
            kind: "edge_flange".into(),
        },
    );
    let update = handled(&mut editor, click_at(from_above(2.0, 0.4)));
    let step = update.step.expect("the flange is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    handled(&mut editor, dialog("length", Value::Number(0.3)));
    handled(
        &mut editor,
        dialog("position", Value::Choice("bend_outside".into())),
    );
    handled(&mut editor, Command::Commit);
    let step = editor.program().steps.last().unwrap().clone();
    assert_eq!(step.operation, "edge_flange");
    let flange: EdgeFlangeArgs = args(&step);
    assert_eq!(
        flange.edge,
        Some(edge("base_flange(plate,outline,c5,b)")),
        "{flange:?}"
    );
    assert_eq!(flange.length, 0.3.into());
    let flanged = format!("edge_flange({})", step.id);

    handled(
        &mut editor,
        Command::New {
            kind: "flat_pattern".into(),
        },
    );
    let update = handled(&mut editor, Command::Commit);
    let step = editor.program().steps.last().unwrap();
    assert_eq!(step.operation, "flat_pattern");
    assert_eq!(args::<FlatPatternArgs>(step).solid, solid(flanged));
    let scene = update.scene.expect("the scene is sent anew");
    assert!(
        scene
            .part
            .solids
            .iter()
            .any(|s| s.starts_with("flat_pattern(")),
        "{:?}",
        scene.part.solids
    );
}

/// Sheet metal the way the front end does it, on the bracket: a sketch on
/// the back flange cut through it by a new sheet-metal cut that takes the
/// newest sketch and picks the flange by a click; a new hem picked on the
/// plate's right edge and opened in the dialog; and the flat pattern
/// exported for laser cutting, as the File menu asks for it.
#[test]
fn sheet_metal_cut_hem_and_cutting_export() {
    let mut editor = bracket_editor();
    // A hole drawn on the back flange's outside, 0.3 up: the face's sketch
    // `x` runs along the world's, its `y` down.
    let mut program = editor.program().clone();
    let hole = disc([1.0, -0.3], 0.05);
    program.push(
        "flange_hole",
        AddSketchArgs {
            plane: Some(SketchPlane::Face(element("edge_flange(back,flange,a)"))),
            sketch: hole,
            ..Default::default()
        },
    );
    handled(
        &mut editor,
        Command::Load {
            program,
            path: None,
        },
    );

    handled(
        &mut editor,
        Command::New {
            kind: "sheet_cut".into(),
        },
    );
    // The face is the one the sketch lies on unless picked: pressed, it
    // takes the click.
    handled(&mut editor, dialog("face", Value::Press));
    handled(
        &mut editor,
        Command::Event {
            event: StepEditEvent::Click {
                pointer: pointer([1.0, 5.0, 0.3], [0.0, -1.0, 0.0]),
                button: Button::Primary,
                double: false,
                shift: false,
            },
        },
    );
    let update = handled(&mut editor, Command::Commit);
    let step = editor.program().steps.last().unwrap().clone();
    assert_eq!(step.operation, "sheet_cut");
    let cut: SheetCutArgs = args(&step);
    assert_eq!(cut.sketch, whole_sketch("flange_hole"));
    assert_eq!(cut.face, Some(element("edge_flange(back,flange,a)")));
    let scene = update.scene.expect("the scene is sent anew");
    assert_eq!(scene.part.solids, [format!("sheet_cut({})", step.id)]);

    handled(&mut editor, Command::New { kind: "hem".into() });
    handled(&mut editor, click_at(from_above(2.0, 0.4)));
    handled(&mut editor, dialog("kind", Value::Choice("open".into())));
    handled(&mut editor, dialog("gap", Value::Number(0.06)));
    let update = handled(&mut editor, Command::Commit);
    let step = editor.program().steps.last().unwrap().clone();
    assert_eq!(step.operation, "hem");
    let hem: HemArgs = args(&step);
    assert_eq!(hem.edge, Some(edge("base_flange(plate,outline,c5,b)")));
    assert_eq!((hem.kind, hem.gap), (HemKind::Open, 0.06));
    let scene = update.scene.expect("the scene is sent anew");
    assert_eq!(scene.part.solids, [format!("hem({})", step.id)]);

    let json = editor
        .handle_json(r#"{"command": "export", "exporter": "flat_pattern"}"#)
        .unwrap();
    let update: serde_json::Value = serde_json::from_str(&json).unwrap();
    assert!(update["error"].is_null(), "{}", update["error"]);
    assert_eq!(update["export"]["name"], "part_flat.dxf");
    let dxf = update["export"]["text"].as_str().unwrap();
    assert_eq!(dxf.matches("\nUP 90%%d R0.08\n").count(), 2, "{dxf}");
    assert_eq!(dxf.matches("\nUP 180%%d R0.03\n").count(), 1, "{dxf}");
    // Three holes, each one circle.
    assert_eq!(dxf.matches("\nCIRCLE\n8\nCUT\n").count(), 3, "{dxf}");
}

/// The bracket's flat pattern for laser cutting, downloaded by the exporter
/// the editor offers for it, as `geop compile` writes it: named after the
/// program's file, its outline and holes on the CUT layer, its two bends on
/// the BEND layer — and in DXF only.
#[test]
fn the_flat_pattern_is_exported_for_cutting() {
    let mut editor = Editor::new(registry().unwrap(), std::collections::BTreeMap::new());
    handled(
        &mut editor,
        Command::Load {
            program: crate::examples::sheet_metal_bracket(),
            path: Some("sheet_metal_bracket.geop".into()),
        },
    );
    let flat_pattern = |format: Option<&str>| Command::Export {
        exporter: "flat_pattern".into(),
        format: format.map(str::to_string),
        options: serde_json::Value::Null,
        date: String::new(),
    };
    let update = handled(&mut editor, flat_pattern(None));
    let export = update.export.expect("a file is written");
    assert_eq!(export.name, "sheet_metal_bracket_flat.dxf");
    let dxf = export.text().expect("DXF is text");
    assert_eq!(dxf.matches("\nCIRCLE\n8\nCUT\n").count(), 2, "{dxf}");
    assert_eq!(dxf.matches("\nLINE\n8\nBEND\n").count(), 2, "{dxf}");
    let refused = editor.handle(flat_pattern(Some("svg")));
    let error = refused.error.expect("no flat pattern is written as SVG");
    assert!(error.contains("dxf"), "{error}");
}
