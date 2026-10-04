//! Sheet metal in programs and in the editor: the bracket example built,
//! flanged and unfolded by steps, the way the front end does it.

use geop_core_math::{
    primitives::Ray,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    NoFiles, Part,
    ui::{Button, Pointer, Reach, StepEditEvent, Value},
};
use geop_ops_sheetmetal::{FlatPatternArgs, FlatPatternData, Sheet};

use crate::{Command, Editor, PartOperation, examples};

/// `validate_manifold` runs `validate` first: once is enough.
fn assert_valid(part: &Part<S>) {
    if let Err(report) = super::regression_tests::check_valid(part) {
        panic!("{report}");
    }
}

/// The bracket example builds a valid sheet-metal body of three flats and
/// two bends, survives a trip through JSON, and its flat pattern — a step
/// like any other — is a valid flat body with its bend lines.
#[test]
fn bracket_example_builds_and_unfolds() {
    let mut program = examples::sheet_metal_bracket();
    let json = serde_json::to_string(&program).unwrap();
    assert_eq!(
        serde_json::from_str::<crate::Program>(&json).unwrap(),
        program
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    let sheet = part.body_data::<Sheet<S>>("edge_flange(back)").unwrap();
    assert_eq!((sheet.flats.len(), sheet.bends.len()), (3, 2));
    assert_eq!(sheet.flats[0].holes.len(), 2);

    program.push(
        "flat",
        FlatPatternArgs {
            solid: "edge_flange(back)".into(),
            keep: false,
        },
    );
    let part = program.build::<S>(&NoFiles).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["flat_pattern(flat)"]);
    let data = part
        .body_data::<FlatPatternData>("flat_pattern(flat)")
        .unwrap();
    assert_eq!(data.bends.len(), 2);
    part.sketch_id("flat_pattern(flat,bend_lines)").unwrap();
}

/// A new edge flange picks its edge by a click onto the bracket's right
/// edge from above — the plate's top edge there — builds as the dialog
/// sets it, and a new flat pattern starts from the flanged body.
#[test]
fn new_edge_flange_picks_its_edge_and_unfolds() {
    let mut editor = Editor::<S>::new();
    let update = editor.handle(Command::LoadExample {
        name: "sheet_metal_bracket".into(),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor.handle(Command::New {
        kind: "edge_flange".into(),
    });
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    let click = Command::Event {
        event: StepEditEvent::Click {
            pointer: Pointer {
                ray: Ray::try_new(v([2.0, 0.4, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
                reach: Reach::Tube {
                    radius: S::from_f64(0.009),
                },
            },
            button: Button::Primary,
            double: false,
            shift: false,
        },
    };
    let update = editor.handle(click);
    assert!(update.error.is_none(), "{:?}", update.error);
    let step = update.step.expect("the flange is edited");
    assert!(step.missing.is_empty(), "{:?}", step.missing);
    let dialog = |key: &str, value| Command::Event {
        event: StepEditEvent::Dialog {
            key: key.into(),
            value,
        },
    };
    editor.handle(dialog("length", Value::Number(0.3)));
    editor.handle(dialog("position", Value::Choice("bend_outside".into())));
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    let program = editor.program();
    let step = program.steps.last().unwrap();
    match &step.operation {
        PartOperation::EdgeFlange(args) => {
            assert_eq!(args.edge, "base_flange(plate,outline,c5,b)", "{args:?}");
            assert_eq!(args.length, 0.3);
        }
        other => panic!("{other:?}"),
    }
    let id = step.id.clone();
    let flanged = format!("edge_flange({id})");

    editor.handle(Command::New {
        kind: "flat_pattern".into(),
    });
    let update = editor.handle(Command::Commit);
    assert!(update.error.is_none(), "{:?}", update.error);
    match &editor.program().steps.last().unwrap().operation {
        PartOperation::FlatPattern(args) => assert_eq!(args.solid, flanged),
        other => panic!("{other:?}"),
    }
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
