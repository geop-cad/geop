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
    let read: Vec<&String> = runner.part().state().keys().collect();
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
