//! STEP: example parts written as STEP files and read back the same, and
//! the import step reading a file next to its program.

use std::collections::BTreeMap;

use geop_core_math::{plain, scalars::ScalInF64 as S};
use geop_core_topology::{
    Model,
    validation::{ValidationParameters, validate},
};
use geop_ops_extrude_revolve::testing::model_of;
use geop_ops_model::{Design, NamedModel};
use geop_ops_step::{ImportStepArgs, write_step};

use crate::operations::{build, testing::Built, workspace};
use crate::{Program, examples};

/// `(faces, edges, vertices)`.
fn counts(model: &Model<S>) -> (usize, usize, usize) {
    (model.faces.len(), model.edges.len(), model.vertices.len())
}

/// The box around a model's vertices.
fn bounds(model: &Model<S>) -> ([f64; 3], [f64; 3]) {
    plain::bounds(model.vertices.values().map(|v| v.point.to_f64_array())).unwrap()
}

/// The model a program of one import step builds, reading `file` — named
/// `part.step` next to the program — from `text`.
fn imported(text: String) -> NamedModel<Design> {
    let mut program = Program::new();
    program.push(
        "imp",
        ImportStepArgs {
            file: "part.step".into(),
        },
    );
    let files = BTreeMap::from([("part.step".to_string(), text)]);
    let built = build(&program, "main.geop", workspace(files))
        .unwrap_or_else(|e| panic!("importing failed: {e}"));
    model_of(&built.part)
}

/// Example parts — extruded, revolved, drilled, filleted, booleaned — come
/// back from STEP with as many faces, edges and vertices, in the same
/// place, and valid.
#[test]
fn examples_round_trip_through_step() {
    let chosen = [
        "box_with_drill_hole",
        "bracket",
        "cross_drilled_shaft",
        "revolved_cone_on_box",
        "pin",
    ];
    let mut found = 0;
    for (name, program) in examples::all() {
        if !chosen.contains(&name) {
            continue;
        }
        found += 1;
        let part = program().build_model().unwrap();
        let text = write_step(&part, name).unwrap();
        let back = imported(text);
        assert_eq!(
            counts(back.topology()),
            counts(part.topology()),
            "{name}: counts changed"
        );
        let (a, b) = (bounds(part.topology()), bounds(back.topology()));
        for c in 0..3 {
            assert!(
                (a.0[c] - b.0[c]).abs() < 1e-9 && (a.1[c] - b.1[c]).abs() < 1e-9,
                "{name}: bounds {a:?} became {b:?}"
            );
        }
        if let Err(errors) = validate(&ValidationParameters::default(), back.topology()) {
            panic!("{name}: invalid after import: {:?}", errors);
        }
        assert_eq!(back.solid_names().len(), part.solid_names().len(), "{name}");
    }
    assert_eq!(found, chosen.len(), "every example chosen is one");
}
