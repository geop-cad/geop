//! Programs of the editor's operations, run.

use crate::operations::testing::{cut, extrusion};
use geop_ops_extrude_revolve::Output;
use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::scalars::{ScalInF64 as S, Scalar};
use geop_ops::{
    Design, NamedModel, PartDescription,
    model::{model, sketch},
    parameters::{Parameter, ParameterKind, Parameters, Values},
    placed::Library,
};
use geop_ops_booleans::Combine;
use geop_ops_extrude_revolve::{Extents, ExtrudeArgs};

use crate::{
    Program,
    examples::box_with_drill_hole,
    operations::{
        Runner,
        testing::{Built, edit_args, step},
    },
    registry,
};

fn extrude(sketch: &str, distance: f64) -> ExtrudeArgs {
    extrusion(sketch, Extents::blind(distance), Combine::NewBody)
}

/// A runner of programs, with the one library it builds them with: a
/// library given anew is a change, which builds everything again.
pub(super) struct Running {
    pub(super) runner: Runner,
    library: Rc<dyn Library>,
    /// What each step built when last seen, by step: to tell what a run
    /// built anew from what it took from the run before.
    seen: BTreeMap<String, usize>,
}

impl Running {
    pub(super) fn new() -> Self {
        let workspace = super::workspace(BTreeMap::new());
        Self {
            runner: super::runner(),
            library: workspace.scope("part.geop"),
            seen: BTreeMap::new(),
        }
    }

    /// Runs the first `end` steps of `program` — all, if `None` — and says
    /// which failed, and why.
    pub(super) fn run(&mut self, program: &Program, end: Option<usize>) -> Vec<(String, String)> {
        let steps = &program.program;
        let end = end.unwrap_or(steps.steps.len());
        let inputs = program.inputs(&Values::new(), self.library.clone());
        let output = self.runner.run_until(steps, end, &inputs, &program.state);
        (output.step_errors.into_iter())
            .map(|(step, error)| (step, error.to_string()))
            .collect()
    }

    /// The model the last run built.
    pub(super) fn model(&self) -> NamedModel<Design> {
        let built = self.runner.part.current().retrieve_target(&model());
        built.cloned().unwrap_or_default()
    }

    /// How many steps of `program` the last run built anew: whose sketch,
    /// or whose version of the model, is not the very one it was when last
    /// seen. A version reused is kept where it is; one built again is made
    /// while the old one still is, so elsewhere.
    pub(super) fn built_anew(&mut self, program: &Program) -> usize {
        let mut anew = 0;
        for step in &program.program.steps {
            let Some(view) = self.runner.part.view_after(&step.id) else {
                continue;
            };
            let address = match step.operation.as_str() {
                "add_sketch" => view
                    .retrieve_target(&sketch(&step.id))
                    .map(|s| s as *const _ as usize),
                _ => view
                    .retrieve_target(&model())
                    .map(|m| m as *const _ as usize),
            };
            let Ok(address) = address else {
                continue;
            };
            if self.seen.insert(step.id.clone(), address) != Some(address) {
                anew += 1;
            }
        }
        anew
    }
}

fn describe(part: &NamedModel<S>) -> PartDescription {
    PartDescription::of(part).unwrap()
}

/// A runner builds what building from scratch builds; stopping early builds
/// just the steps before the stop; and a run after an edit starts from
/// the last unchanged step.
#[test]
fn runner_stops_early_and_reuses_the_unchanged_prefix() {
    let program = box_with_drill_hole();
    let mut running = Running::new();

    assert_eq!(running.run(&program, None), []);
    assert_eq!(
        describe(&running.model()),
        describe(&program.build_model().unwrap())
    );

    // Back in time: only the box.
    assert_eq!(running.run(&program, Some(2)), []);
    let description = describe(&running.model());
    assert_eq!(
        description.solids.keys().collect::<Vec<_>>(),
        ["extrude(box)"]
    );

    // An edit to the hole keeps the box's part: the first two steps are
    // served from the cache, which has to hold exactly what they built.
    let mut edited = program.clone();
    edit_args(step(&mut edited, "hole"), |args: &mut ExtrudeArgs| {
        *args = extrude("hole_sketch", -0.25)
    });
    assert_eq!(running.run(&edited, None), []);
    assert_eq!(
        describe(&running.model()),
        describe(&edited.build_model().unwrap())
    );
}

/// A step that fails is reported by id, and does not stop the run; the
/// part before it is what the steps before it built.
#[test]
fn runner_reports_the_failing_step() {
    let mut program = box_with_drill_hole();
    edit_args(step(&mut program, "hole"), |args: &mut ExtrudeArgs| {
        args.output = Output::Solid(cut("extrude(nothing)"))
    });
    let mut running = Running::new();
    let errors = running.run(&program, None);
    let [(id, error)] = errors.as_slice() else {
        panic!("one step fails: {errors:?}");
    };
    assert_eq!(id, "hole");
    assert!(error.contains("extrude(nothing)"), "{error}");
    // The model the steps before it built is there for it to build on.
    let before = running.runner.part.view_before("hole").unwrap();
    let built = before.retrieve_target(&model()).unwrap();
    assert!(built.solid_id("extrude(box)").is_ok());
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
    edit_args(step(&mut program, "box"), |args: &mut ExtrudeArgs| {
        args.extent = Extents::blind("height")
    });
    edit_args(step(&mut program, "hole"), |args: &mut ExtrudeArgs| {
        args.extent = Extents::blind("-depth")
    });
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
fn assert_heights(part: &NamedModel<S>, want: &[f64]) {
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
    let mut running = Running::new();
    assert_eq!(running.run(&program, None), []);
    assert_eq!(running.built_anew(&program), 4);
    // The bottom, the hole's floor and the top.
    assert_heights(&running.model(), &[0.0, 0.5, 1.0]);
    let uses = program.parameter_uses(&registry().unwrap());
    let read: Vec<&String> = uses.keys().collect();
    assert_eq!(read, ["depth", "height"]);

    set(&mut program, "depth", "0.25");
    running.run(&program, None);
    assert_eq!(running.built_anew(&program), 1, "only the hole reads depth");
    assert_heights(&running.model(), &[0.0, 0.75, 1.0]);

    set(&mut program, "height", "2");
    running.run(&program, None);
    assert_eq!(running.built_anew(&program), 3, "the box reads height");
    assert_heights(&running.model(), &[0.0, 1.75, 2.0]);

    set(&mut program, "unrelated", "8");
    running.run(&program, None);
    assert_eq!(running.built_anew(&program), 0, "no step reads unrelated");

    let json = program.to_json().unwrap();
    assert!(json.contains(r#""blind": "height""#), "{json}");
    let plain = box_with_drill_hole().to_json().unwrap();
    assert!(plain.contains(r#""blind": 1.0"#), "{plain}");
    assert_eq!(Program::from_json(&json).unwrap(), program);
}

/// A formula that does not evaluate fails its step, the error — reported
/// by the step's id — naming the formula, and why.
#[test]
fn formula_errors_name_the_step_and_the_formula() {
    let mut program = box_by_formulas();
    edit_args(step(&mut program, "hole"), |args: &mut ExtrudeArgs| {
        args.extent = Extents::blind("-(dpeth + 1)")
    });
    let mut running = Running::new();
    let errors = running.run(&program, None);
    let [(id, error)] = errors.as_slice() else {
        panic!("the hole fails: {errors:?}");
    };
    assert_eq!(id, "hole");
    for said in [r#""-(dpeth + 1)""#, r#"there is no parameter "dpeth""#] {
        assert!(error.contains(said), "{said} is not said in {error}");
    }
}

/// A step copied takes the state stored under its id along (where a placed
/// part is), and pasted under a fresh id stores it under that one; state
/// merely named like it stays behind.
#[test]
fn a_pasted_step_takes_its_state_along() {
    use geop_ops::{ParamValue, model::pose_parameter};
    let mut program = box_with_drill_hole();
    let id = program.program.steps[0].id.clone();
    let pose = ParamValue::Pose(crate::examples::pose([1.0, 2.0, 3.0], [0.0; 3]));
    program.state.insert(pose_parameter(&id), pose.clone());
    program.state.insert(
        format!("{id}_width"),
        ParamValue::Number(geop_ops::Design::from_f64(2.0)),
    );

    let copied = program.excerpt(&id).unwrap();
    assert_eq!(copied.state.len(), 1);
    let at = program.program.steps.len();
    let ids = program.paste(copied, at, |s| s.operation.clone()).unwrap();
    assert_eq!(ids.len(), 1);
    assert_ne!(ids[0], id);
    assert_eq!(program.program.steps[at].id, ids[0]);
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
#[ignore = "slow: 10 s — run with `cargo test -- --ignored`"]
fn a_runner_builds_what_building_from_scratch_builds() {
    use crate::examples;

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
    for (name, make) in examples::all() {
        if !examples.contains(&name) {
            continue;
        }
        let original = make();
        let steps = original.program.steps.len();
        let mut edits = Vec::new();
        for index in [0, steps / 2, steps - 1] {
            let mut dropped = original.clone();
            dropped.program.steps.remove(index);
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

        let mut running = Running::new();
        running.run(&original, None);
        for (edit, program) in edits {
            let context = format!("{name}, {edit}");
            let errors = running.run(&program, None);
            match program.build_model() {
                Ok(scratch) => {
                    assert_eq!(errors, [], "{context}");
                    assert_eq!(describe(&running.model()), describe(&scratch), "{context}");
                }
                Err(_) => assert!(!errors.is_empty(), "{context}: building from scratch fails"),
            }
        }
    }
}
