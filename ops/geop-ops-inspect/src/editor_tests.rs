//! The inspect tools asked through the editor, as the inspect panel asks:
//! the mass properties of the example assemblies, each placed part of its
//! own material where it is placed, and the interference between placed
//! parts.

use std::collections::BTreeMap;

use geop_core_editor::{
    editor::{Command, Editor},
    output::Bounded,
    registry::Registry,
};
use geop_core_math::geop_error::GeopResult;
use geop_lib_stdlib::WithStandardParts;

/// The operations the example assemblies are written in, with everything
/// assemblies offer an editor — their examples among it — and the
/// inspectors: each crate's `register`, as the editor's registry has them.
fn registry() -> Registry {
    let registry = (|| -> GeopResult<Registry> {
        let r = Registry::default();
        let r = geop_ops_model::register(r)?;
        let r = geop_ops_sketch::register(r)?;
        let r = geop_ops_datums::register(r)?;
        let r = geop_ops_reference_geometry::register(r)?;
        let r = geop_ops_extrude_revolve::register(r)?;
        let r = geop_ops_pattern::register(r)?;
        let r = geop_ops_assembly::register(r)?;
        let r = crate::register(r)?;
        geop_ops_parameters::register(r)
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
        assert!(name.contains('/'), "{name} is of a placed part");
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
