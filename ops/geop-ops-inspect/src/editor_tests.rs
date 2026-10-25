//! The inspect tools asked through the editor, as the inspect panel asks:
//! the mass properties of the example assemblies, each placed part of its
//! own material where it is placed, and the interference between placed
//! parts; and the measure tool used as the viewer uses it, on a drilled
//! box given by its coordinates.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::StepEditEvent,
    editor::{Command, Editor, Update},
    output::Bounded,
    registry::Registry,
    testing::{click_at, editor_with, from_above, pointer},
};
use geop_core_math::geop_error::GeopResult;
use geop_lib_fixtures::drilled_box;
use geop_lib_stdlib::WithStandardParts;
use geop_ops_model::{Face, element};

/// The operations the example assemblies' parts are written in (sketched
/// and extruded), with everything assemblies offer an editor — their
/// examples among it — the given solids the measured box is built from,
/// and the inspectors: each crate's `register`, as the editor's registry
/// has them.
fn registry() -> Registry {
    let registry = (|| -> GeopResult<Registry> {
        let r = Registry::default();
        let r = geop_ops_model::register(r)?;
        let r = geop_ops_sketch::register(r)?;
        let r = geop_ops_extrude_revolve::register(r)?;
        let r = geop_ops_assembly::register(r)?;
        let r = geop_lib_fixtures::register(r)?;
        let r = crate::register(r)?;
        Ok(r)
    })();
    registry.expect("the operations each have a name of their own")
}

/// The editor with the workspace example `name` loaded, as the browser
/// loads one.
fn workspace(name: &str) -> Editor {
    let mut editor = Editor::new(registry(), WithStandardParts(BTreeMap::new()));
    let update = editor.handle(Command::LoadWorkspaceExample {
        name: name.into(),
        folder: None,
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    editor
}

/// The answer of the inspector `kind`, asked of the part as drawn the way
/// the inspect panel asks: as the front end reads it.
fn ask(editor: &mut Editor, kind: &str) -> serde_json::Value {
    let update = editor.handle(Command::Inspect {
        inspector: kind.into(),
        options: serde_json::Value::Null,
    });
    assert!(update.error.is_none(), "{kind}: {:?}", update.error);
    let inspection = update.inspection.expect("an answer");
    assert_eq!(inspection.kind, kind);
    inspection.report
}

/// `json` as a number shown with its bounds.
fn bounded(json: &serde_json::Value) -> Bounded {
    Bounded {
        value: json["value"].as_f64().expect("a value"),
        error: json["error"].as_f64().expect("an error"),
    }
}

/// Two plates placed in an assembly, both aluminium as their file says:
/// each weighed where it is placed, the total their sum, its centre
/// between theirs. Asked through the editor, as the inspect panel asks.
#[test]
fn an_assembly_weighs_its_placed_parts() {
    let mut editor = workspace("parametric_plates");
    let report = ask(&mut editor, "mass_properties");
    let bodies = report["bodies"].as_array().expect("bodies");
    assert_eq!(bodies.len(), 2, "{report}");
    let mut sum = 0.0;
    let mut moment = 0.0;
    for body in bodies {
        let name = body["name"].as_str().unwrap();
        assert!(name.starts_with('/'), "{name} is of a placed part");
        assert_eq!(body["material"], "Aluminium 6061");
        assert_eq!(body["assumed"], false);
        let properties = &body["properties"];
        let (volume, mass) = (bounded(&properties["volume"]), bounded(&properties["mass"]));
        // Aluminium: 2.7e-6 kg per mm³.
        assert!(
            Bounded {
                value: volume.value * 2.7e-6,
                error: volume.error * 2.7e-6 + 1e-15,
            }
            .contains(mass.value)
        );
        sum += mass.value;
        moment += mass.value * bounded(&properties["center"][0]).value;
    }
    let total = &report["total"];
    let (mass, x) = (bounded(&total["mass"]), bounded(&total["center"][0]));
    assert!((mass.value - sum).abs() <= mass.error + 1e-12);
    assert!((x.value - moment / sum).abs() <= x.error + 1e-9);

    // Placed apart, they do not interfere.
    let report = ask(&mut editor, "interference");
    assert_eq!(report["solids"], 2);
    assert_eq!(report["unchecked"], serde_json::json!([]), "{report}");
    let found = report["found"].as_array().unwrap();
    assert!(found.iter().all(|f| f["contact"] != "overlap"), "{report}");
}

/// The linkages' bars, stacked each on the one it is pinned to, touch but
/// never overlap; neither do the pin and the plate it is placed in.
#[test]
#[ignore = "slow: interference of every pair of the assembly examples' parts — run with `cargo test -- --ignored`"]
fn assembly_examples_do_not_overlap() {
    for name in ["chain", "four_bar", "pin_in_plate"] {
        let report = ask(&mut workspace(name), "interference");
        assert_eq!(
            report["unchecked"],
            serde_json::json!([]),
            "{name}: {report:#}"
        );
        let found = report["found"].as_array().unwrap();
        let overlaps: Vec<_> = found.iter().filter(|f| f["contact"] == "overlap").collect();
        assert!(overlaps.is_empty(), "{name}: {overlaps:#?}");
    }
}

/// The measure tool, used as the viewer uses it: taken in hand, the box's
/// top hovered and clicked — its area, its plane — then its bottom clicked
/// from below: one unit apart, parallel, the least distance drawn between
/// them. Then the mass properties and the interference of the part asked
/// for. None of it changes the program, and the tool put down shows nothing.
#[test]
fn measure_tool_measures_what_is_clicked() {
    let mut editor = editor_with(registry(), drilled_box());
    let before = editor.program().clone();
    let update = editor.handle(Command::Tool {
        key: Some("pick:measure".into()),
    });
    assert!(update.error.is_none(), "{:?}", update.error);
    assert_eq!(
        update.program.unwrap().tool.as_deref(),
        Some("pick:measure")
    );
    let top = element::<Face>("block(box,end)").picked();
    let bottom = element::<Face>("block(box,start)").picked();
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
