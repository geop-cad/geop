//! Sheet metal in programs and in the editor: the bracket example built,
//! flanged, cut, hemmed and unfolded by steps, the way the front end does
//! it, and its flat pattern exported for cutting.

use geop_core_editor::editing::{Button, StepEditEvent, Value};
use geop_ops_model::solid;
use geop_ops_model::{element, notes_of};
use geop_ops_sheetmetal::{
    EdgeFlangeArgs, FlatPatternArgs, HemArgs, HemKind, SheetBody, SheetCutArgs, edge, sheet_body,
};
use geop_ops_sketch::{AddSketchArgs, SketchPlane, whole_sketch};

use geop_ops_extrude_revolve::testing::model_of;

use crate::registry::testing::{
    Built, args, assert_valid, click_at, dialog, disc, example_editor, from_above, handled, pointer,
};
use crate::{Command, examples};

/// The bracket example builds a valid sheet-metal body of three flats, two
/// bends and the holes cut through it after flanging, survives a trip
/// through JSON, and its flat pattern — a step like any other — is a valid
/// flat body with its bend lines.
#[test]
fn bracket_example_builds_and_unfolds() {
    let mut program = examples::sheet_metal_bracket();
    let json = serde_json::to_string(&program).unwrap();
    assert_eq!(
        serde_json::from_str::<geop_core_program::program::Program>(&json).unwrap(),
        program
    );
    let part = program.built().unwrap();
    let view = part.current();
    assert_valid(&model_of(&part));
    let sheet = sheet_body(&view, "sheet_cut(mount)")
        .and_then(|b| b.bent())
        .unwrap();
    assert_eq!((sheet.flats.len(), sheet.bends.len()), (3, 2));
    assert_eq!(sheet.cuts.len(), 1);
    assert_eq!(sheet.cuts[0].loops.len(), 2);

    program.push(
        "flat",
        FlatPatternArgs {
            solid: solid("sheet_cut(mount)"),
            keep: false,
        },
    );
    let part = program.built().unwrap();
    let view = part.current();
    let built = model_of(&part);
    assert_valid(&built);
    assert_eq!(built.solid_names(), ["flat_pattern(flat)"]);
    let Some(SheetBody::Flat(data)) = sheet_body(&view, "flat_pattern(flat)") else {
        panic!("no flat pattern recorded");
    };
    assert_eq!(data.bends.len(), 2);
    assert!(
        (view.retrieve_element(&whole_sketch("flat_pattern(flat,bend_lines)"))).is_ok(),
        "no bend lines"
    );
    let notes = notes_of(&view).unwrap();
    assert_eq!(
        notes.get("flat_pattern(flat)").unwrap().thickness,
        Some(0.08)
    );
}

/// A new edge flange picks its edge by a click onto the bracket's right
/// edge from above — the plate's top edge there — builds as the dialog
/// sets it, and a new flat pattern starts from the flanged body.
#[test]
fn new_edge_flange_picks_its_edge_and_unfolds() {
    let mut editor = example_editor("sheet_metal_bracket");
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
    let mut editor = example_editor("sheet_metal_bracket");
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
