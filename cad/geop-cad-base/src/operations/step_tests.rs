//! STEP: example parts written as STEP files and read back the same, and
//! the import step reading a file next to its program.

use std::collections::BTreeMap;

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_core_topology::{
    Model,
    validation::{ValidationParameters, validate},
};
use geop_ops::{NoFiles, Part};
use geop_ops_step::{ImportStepArgs, write_step};

use crate::{PartOperation, Program, Workspace, examples};

/// `(faces, edges, vertices)`.
fn counts(model: &Model<S>) -> (usize, usize, usize) {
    (model.faces.len(), model.edges.len(), model.vertices.len())
}

/// The box around a model's vertices.
fn bounds(model: &Model<S>) -> ([f64; 3], [f64; 3]) {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in model.vertices.values() {
        for c in 0..3 {
            lo[c] = lo[c].min(v.point[c].to_f64());
            hi[c] = hi[c].max(v.point[c].to_f64());
        }
    }
    (lo, hi)
}

/// The part a program of one import step builds, reading `file` — named
/// `part.step` next to the program — from `text`.
fn imported(text: String) -> Part<S> {
    let mut program = Program::default();
    program.push(
        "imp",
        PartOperation::ImportStep(ImportStepArgs {
            file: "part.step".into(),
        }),
    );
    let files = BTreeMap::from([("part.step".to_string(), text)]);
    let workspace = Workspace::<S>::new(crate::stdlib::WithStandardParts(files));
    program
        .build(&workspace.scope("main.geop"))
        .unwrap_or_else(|e| panic!("importing failed: {e}"))
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
    for (name, program) in examples::all() {
        if !chosen.contains(&name) {
            continue;
        }
        let part = program.build(&NoFiles).unwrap();
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
}
