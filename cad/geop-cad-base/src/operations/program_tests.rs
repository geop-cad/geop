//! The examples of every operation, run incrementally: edited one after
//! another on one part, each builds what it builds from scratch.

use geop_core_program::{
    program::Program,
    testing::{assert_incremental, edit_sequence},
    workspace::Workspace,
};
use geop_ops_parameters::{ParameterKind, parameters_of, set_parameters};
use std::{collections::BTreeMap, rc::Rc};

/// A runner that takes steps from what it built before must build what
/// building from scratch builds: after a step is dropped from the middle of
/// a program and put back, and after a number a step reads is changed. A
/// step skipped for reading what did not change would otherwise show here
/// as a target that differs, or an error that is missing (see
/// [`assert_incremental`]).
#[test]
#[ignore = "slow: 10 s — run with `cargo test -- --ignored`"]
fn a_runner_builds_what_building_from_scratch_builds() {
    let examples = [
        "box_with_drill_hole",
        "bracket",
        "cross_drilled_shaft",
        "split_plate",
        "parametric_plate",
        "pin",
        "patterned_plate",
        "rounded_box",
        "hole_plate",
    ];
    for (name, make) in crate::registry().unwrap().example_programs() {
        if !examples.contains(&name) {
            continue;
        }
        let original = make();
        let steps = original.steps.len();
        let mut edits = Vec::new();
        for index in [0, steps / 2, steps - 1] {
            let mut dropped = original.clone();
            dropped.steps.remove(index);
            edits.push((format!("drop step {index}"), dropped));
            edits.push(("restore".to_string(), original.clone()));
        }
        let parameters = parameters_of(&original);
        for (k, parameter) in parameters.values.iter().enumerate() {
            if let ParameterKind::Number { expression, .. } = &parameter.kind
                && let Ok(value) = expression.parse::<f64>()
            {
                let mut scaled = original.clone();
                let mut changed = parameters.clone();
                if let ParameterKind::Number { expression, .. } = &mut changed.values[k].kind {
                    *expression = (value * 1.25).to_string();
                }
                set_parameters(&mut scaled, changed);
                edits.push((format!("scale {}", parameter.name), scaled));
                edits.push(("restore".to_string(), original.clone()));
            }
        }

        let programs: Vec<Program> = std::iter::once(original)
            .chain(edits.into_iter().map(|(_, program)| program))
            .collect();
        let workspace: Rc<dyn Workspace> = crate::registry::workspace(BTreeMap::new());
        let registry = Rc::new(crate::registry().unwrap().operations());
        assert_incremental(registry, Some((workspace, name)), &programs);
    }
}

/// Every example program named in `only` — all, for none — and every file
/// of every example made of several, run through [`edit_sequence`],
/// incrementally builds what it builds from scratch (see
/// [`assert_incremental`]).
fn assert_examples_incremental(only: Option<&[&str]>) {
    let offered = crate::registry().unwrap();
    let registry = Rc::new(offered.operations());
    let wanted = |name: &str| only.is_none_or(|only| only.contains(&name));
    for (name, make) in offered.example_programs() {
        if !wanted(name) {
            continue;
        }
        let workspace: Rc<dyn Workspace> = crate::registry::workspace(BTreeMap::new());
        let programs = edit_sequence(&make());
        assert_incremental(registry.clone(), Some((workspace, name)), &programs);
    }
    for name in offered
        .workspace_example_names()
        .filter(|name| wanted(name))
    {
        let files = offered.workspace_example(name).unwrap();
        let texts: BTreeMap<String, String> = (files.iter())
            .map(|(file, program)| (file.to_string(), program.to_json().unwrap()))
            .collect();
        let workspace: Rc<dyn Workspace> = crate::registry::workspace(texts);
        for (file, program) in &files {
            assert_incremental(
                registry.clone(),
                Some((workspace.clone(), file)),
                &edit_sequence(program),
            );
        }
    }
}

/// [`assert_examples_incremental`] on a few cheap examples: a part with a
/// table, an assembly placing parts.
#[test]
fn examples_build_incrementally_what_they_build_from_scratch() {
    assert_examples_incremental(Some(&[
        "box_with_drill_hole",
        "parametric_plate",
        "pin_in_plate",
    ]));
}

/// [`assert_examples_incremental`] on every example.
#[test]
#[ignore = "slow: every example built several times over — run with `cargo test -- --ignored`"]
fn every_example_builds_incrementally_what_it_builds_from_scratch() {
    assert_examples_incremental(None);
}
