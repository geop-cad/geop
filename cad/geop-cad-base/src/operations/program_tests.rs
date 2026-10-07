//! Programs of the editor's operations, run.

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_ops::{
    NoFiles, Part, PartDescription,
    parameters::{Parameter, ParameterKind, Parameters},
};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};

use crate::{PartOperation, Program, ProgramRunner, examples::box_with_drill_hole};

fn extrude(sketch: &str, distance: f64) -> PartOperation {
    ExtrudeArgs {
        sketch: sketch.into(),
        extent: Extents::blind(distance),
        face: false,
        combine: Combine::NewBody,
    }
    .into()
}

/// A runner builds what [`Program::build`] builds; stopping early builds
/// just the steps before the stop; and a run after an edit starts from
/// the last unchanged step.
#[test]
fn runner_stops_early_and_reuses_the_unchanged_prefix() {
    let program = box_with_drill_hole();
    let describe = |part: &Part<S>| PartDescription::of(part).unwrap();
    let mut runner = ProgramRunner::<S>::new();

    runner.run(&program, None, &NoFiles);
    assert!(runner.results().iter().all(|r| r.error.is_none()));
    assert_eq!(
        describe(runner.part()),
        describe(&program.build(&NoFiles).unwrap())
    );

    // Back in time: only the box.
    runner.run(&program, Some(2), &NoFiles);
    assert_eq!(runner.results().len(), 2);
    let description = describe(runner.part());
    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["extrude(box)"]
    );

    // An edit to the hole keeps the box's part: the first two steps are
    // served from the cache, which has to hold exactly what they built.
    let mut edited = program.clone();
    let hole = edited.index_of("hole").unwrap();
    edited.steps[hole].operation = extrude("hole_sketch", -0.25);
    runner.run(&edited, None, &NoFiles);
    assert!(runner.results().iter().all(|r| r.error.is_none()));
    assert_eq!(
        describe(runner.part()),
        describe(&edited.build(&NoFiles).unwrap())
    );
}

/// A step that fails ends the run there, reported by id; the part is
/// what the steps before it built.
#[test]
fn runner_reports_the_failing_step() {
    let mut program = box_with_drill_hole();
    let hole = program.index_of("hole").unwrap();
    program.steps[hole].operation = ExtrudeArgs {
        sketch: "hole_sketch".into(),
        extent: Extents::blind(-0.5),
        face: false,
        combine: Combine::Difference {
            target: "extrude(nothing)".into(),
        },
    }
    .into();
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&program, None, &NoFiles);
    let last = runner.results().last().unwrap();
    assert_eq!(last.id, "hole");
    assert!(last.error.as_deref().unwrap().contains("extrude(nothing)"));
    assert!(runner.part().solid_id("extrude(box)").is_ok());
}

/// The box of [`box_with_drill_hole`], `height` high, drilled `depth`
/// deep — both formulas of its parameters — with a parameter `unrelated`
/// no step reads.
fn box_by_formulas() -> Program {
    let mut program = box_with_drill_hole();
    let number = |name: &str, expression: &str| Parameter {
        name: name.into(),
        kind: ParameterKind::Number {
            expression: expression.into(),
            min: None,
            max: None,
        },
    };
    program.parameters = Parameters {
        values: vec![
            number("height", "1"),
            number("depth", "height / 2"),
            number("unrelated", "7"),
        ],
        ..Parameters::default()
    };
    let with = |sketch: &str, distance: &str, combine: Combine| -> PartOperation {
        ExtrudeArgs {
            sketch: sketch.into(),
            extent: Extents::blind(distance),
            face: false,
            combine,
        }
        .into()
    };
    let box_ = program.index_of("box").unwrap();
    program.steps[box_].operation = with("outline", "height", Combine::NewBody);
    let hole = program.index_of("hole").unwrap();
    program.steps[hole].operation = with(
        "hole_sketch",
        "-depth",
        Combine::Difference {
            target: "extrude(box)".into(),
        },
    );
    program
}

/// Sets the number parameter `name` of `program` to `expression`.
fn set(program: &mut Program, name: &str, expression: &str) {
    let parameter = program
        .parameters
        .values
        .iter_mut()
        .find(|p| p.name == name);
    let Some(Parameter {
        kind: ParameterKind::Number { expression: e, .. },
        ..
    }) = parameter
    else {
        panic!("no number parameter {name}")
    };
    *e = expression.into();
}

/// Checks the part's vertices lie at the heights `want`, and each of
/// those has some.
fn assert_heights(part: &Part<S>, want: &[f64]) {
    let z: Vec<f64> = part
        .topology()
        .vertices
        .values()
        .map(|v| v.point[2].to_f64())
        .collect();
    let near = |a: f64, b: f64| (a - b).abs() < 1e-9;
    assert!(
        z.iter().all(|&z| want.iter().any(|&w| near(z, w)))
            && want.iter().all(|&w| z.iter().any(|&z| near(z, w))),
        "the vertices are at {z:?}, not {want:?}"
    );
}

/// An extrude driven by a formula is built again when a parameter it
/// reads changes — and with it every step after it — and not when one it
/// does not read does: changing `depth` builds the hole again, but not
/// the box; changing `height` the box and all after it; changing
/// `unrelated` nothing. A plain number saves as a number, a formula as its
/// text, and both read back as they were.
#[test]
fn extrudes_follow_the_parameters_they_read() {
    let mut program = box_by_formulas();
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&program, None, &NoFiles);
    assert!(
        runner.results().iter().all(|r| r.error.is_none()),
        "{:?}",
        runner.results()
    );
    assert_eq!(runner.built_anew(), 4);
    // The bottom, the hole's floor and the top.
    assert_heights(runner.part(), &[0.0, 0.5, 1.0]);
    let read: Vec<&String> = runner.part().declared().keys().collect();
    assert_eq!(read, ["depth", "height"]);

    set(&mut program, "depth", "0.25");
    runner.run(&program, None, &NoFiles);
    assert_eq!(runner.built_anew(), 1, "only the hole reads depth");
    assert_heights(runner.part(), &[0.0, 0.75, 1.0]);

    set(&mut program, "height", "2");
    runner.run(&program, None, &NoFiles);
    assert_eq!(runner.built_anew(), 3, "the box reads height");
    assert_heights(runner.part(), &[0.0, 1.75, 2.0]);

    set(&mut program, "unrelated", "8");
    runner.run(&program, None, &NoFiles);
    assert_eq!(runner.built_anew(), 0, "no step reads unrelated");

    let json = program.to_json().unwrap();
    assert!(json.contains(r#""blind": "height""#), "{json}");
    let plain = box_with_drill_hole().to_json().unwrap();
    assert!(plain.contains(r#""blind": 1.0"#), "{plain}");
    assert_eq!(Program::from_json(&json).unwrap(), program);
}

/// A formula that does not evaluate fails its step, the error naming the
/// step and the formula, and why.
#[test]
fn formula_errors_name_the_step_and_the_formula() {
    let mut program = box_by_formulas();
    let hole = program.index_of("hole").unwrap();
    program.steps[hole].operation = ExtrudeArgs {
        sketch: "hole_sketch".into(),
        extent: Extents::blind("-(dpeth + 1)"),
        face: false,
        combine: Combine::Difference {
            target: "extrude(box)".into(),
        },
    }
    .into();
    let mut runner = ProgramRunner::<S>::new();
    runner.run(&program, None, &NoFiles);
    let last = runner.results().last().unwrap();
    assert_eq!(last.id, "hole");
    let error = last.error.as_deref().expect("the hole fails");
    for said in [
        r#""hole""#,
        r#""-(dpeth + 1)""#,
        r#"there is no parameter "dpeth""#,
    ] {
        assert!(error.contains(said), "{said} is not said in {error}");
    }
}

/// A step copied takes the state stored under its id along (where a placed
/// part is), and pasted under a fresh id stores it under that one; state
/// merely named like it stays behind.
#[test]
fn a_pasted_step_takes_its_state_along() {
    use geop_ops::part::{ParamValue, pose_parameter};
    let mut program = box_with_drill_hole();
    let id = program.steps[0].id.clone();
    let pose = ParamValue::Pose(crate::examples::pose([1.0, 2.0, 3.0], [0.0; 3]));
    program.state.insert(pose_parameter(&id), pose.clone());
    program.state.insert(
        format!("{id}_width"),
        ParamValue::Number(geop_ops::Design::from_f64(2.0)),
    );

    let copied = program.excerpt(&id).unwrap();
    assert_eq!(copied.state.len(), 1);
    let at = program.steps.len();
    let ids = program.paste(copied, at).unwrap();
    assert_eq!(ids.len(), 1);
    assert_ne!(ids[0], id);
    assert_eq!(program.steps[at].id, ids[0]);
    assert_eq!(program.state.get(&pose_parameter(&ids[0])), Some(&pose));
    assert!(!program.state.contains_key(&format!("{}_width", ids[0])));
    program.validate().unwrap();
}

/// A runner that takes steps from what it built before must build what
/// building from scratch builds: after a step is dropped from the middle of
/// a program and put back, and after a number a step reads is changed. A
/// step skipped for reading what did not change would otherwise show here
/// as a part that differs, or an error that is missing.
#[test]
fn a_runner_builds_what_building_from_scratch_builds() {
    use crate::examples;
    use geop_ops::parameters::ParameterKind;

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
    let describe = |part: &Part<S>| PartDescription::of(part).unwrap();
    for (name, make) in examples::all() {
        if !examples.contains(&name) {
            continue;
        }
        let original = make();
        let mut edits = Vec::new();
        for index in [0, original.steps.len() / 2, original.steps.len() - 1] {
            let mut dropped = original.clone();
            dropped.steps.remove(index);
            edits.push((format!("drop step {index}"), dropped));
            edits.push(("restore".to_string(), original.clone()));
        }
        for (k, parameter) in original.parameters.values.iter().enumerate() {
            if let ParameterKind::Number { expression, .. } = &parameter.kind
                && let Ok(value) = expression.parse::<f64>()
            {
                let mut scaled = original.clone();
                if let ParameterKind::Number { expression, .. } =
                    &mut scaled.parameters.values[k].kind
                {
                    *expression = (value * 1.25).to_string();
                }
                edits.push((format!("scale {}", parameter.name), scaled));
                edits.push(("restore".to_string(), original.clone()));
            }
        }

        let mut runner = ProgramRunner::<S>::new();
        runner.run(&original, None, &NoFiles);
        for (edit, program) in edits {
            let context = format!("{name}, {edit}");
            runner.run(&program, None, &NoFiles);
            let failed = runner.results().iter().any(|r| r.error.is_some());
            match program.build::<S>(&NoFiles) {
                Ok(scratch) => {
                    assert!(!failed, "{context}: {:?}", runner.results());
                    assert_eq!(describe(runner.part()), describe(&scratch), "{context}");
                }
                Err(_) => assert!(failed, "{context}: building from scratch fails"),
            }
        }
    }
}
