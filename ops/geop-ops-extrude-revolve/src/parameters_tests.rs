//! Extrusions driven by the program's parameters, run: their lengths
//! formulas of the parameters, built again when a parameter they read
//! changes and only then, and failing by name where a formula does not
//! evaluate — and the parametric plate, its dimensions, hole and colour
//! following its parameters.

use std::borrow::Cow;
use std::collections::BTreeMap;

use geop_core_program::{
    numeric::{ParameterValue, read_parameter_values},
    part::Part,
    program::Program,
    reader::Reader,
    runner::Runner,
    testing::{edit_args, step},
    values::Setting,
};
use geop_core_sketch::PointId;
use geop_ops_model::parameters::{
    COLOR, Parameter, ParameterKind, Parameters, parameter_numbers, parameter_uses, parameters_of,
    read_properties, set_parameters,
    steps::{LOOKUP, PARAMETERS, PROPERTIES},
};
use geop_ops_model::sketch::{CurveKind, PlacedSketch, sketch_element, whole_sketch};
use geop_ops_model::{
    NamedModel, PartDescription, Point, read_model,
    testing::{Built, runner_of},
};

use crate::{
    Extents, ExtrudeArgs,
    examples::{box_with_drill_hole, parametric_plate},
    testing::registry,
};

/// A runner of programs, as the file `part.geop` of a workspace of no
/// other files. It runs them on one part, which keeps what the last run
/// built.
struct Running {
    runner: Runner,
    part: Part,
    /// What each step built when last seen, by step: to tell what a run
    /// built anew from what it took from the run before.
    seen: BTreeMap<String, usize>,
}

impl Running {
    fn new() -> Self {
        Self {
            runner: runner_of(registry().unwrap()),
            part: Part::default(),
            seen: BTreeMap::new(),
        }
    }

    /// Runs `program`, and says which steps failed, and why.
    fn run(&mut self, program: &Program) -> Vec<(String, String)> {
        let output = (self.runner).run(&mut self.part, program, None);
        (output.step_errors.into_iter())
            .map(|(step, error)| (step, error.to_string()))
            .collect()
    }

    /// The model the last run built.
    fn model(&self) -> NamedModel {
        let built = read_model(&self.part.current());
        built.map(|model| model.into_owned()).unwrap_or_default()
    }

    /// How many steps of `program` the last run built anew: whose sketch,
    /// or whose version of the model, is not the very one it was when last
    /// seen. A version reused is kept where it is; one built again is made
    /// while the old one still is, so elsewhere.
    fn built_anew(&mut self, program: &Program) -> usize {
        let mut anew = 0;
        for step in &program.steps {
            let view = self.part.after(&step.id);
            let address = match step.operation.as_str() {
                "add_sketch" => view
                    .retrieve_element(&whole_sketch(&step.id))
                    .map(|s| s as *const _ as usize)
                    .ok(),
                _ => match read_model(&view) {
                    Ok(Cow::Borrowed(m)) => Some(m as *const _ as usize),
                    _ => None,
                },
            };
            let Some(address) = address else {
                continue;
            };
            if self.seen.insert(step.id.clone(), address) != Some(address) {
                anew += 1;
            }
        }
        anew
    }
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
    set_parameters(
        &mut program,
        Parameters {
            values: vec![
                number("height", "1"),
                number("depth", "height / 2"),
                number("unrelated", "7"),
            ],
            ..Parameters::default()
        },
    );
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
    let mut parameters = parameters_of(program);
    let parameter = parameters.values.iter_mut().find(|p| p.name == name);
    let Some(Parameter {
        kind: ParameterKind::Number { expression: e, .. },
        ..
    }) = parameter
    else {
        panic!("no number parameter {name}")
    };
    *e = expression.into();
    set_parameters(program, parameters);
}

/// Checks the part's vertices lie at the heights `want`, and each of
/// those has some.
fn assert_heights(part: &NamedModel, want: &[f64]) {
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
/// text (`{"formula": ...}`), and both read back as they were.
#[test]
fn extrudes_follow_the_parameters_they_read() {
    let mut program = box_by_formulas();
    let mut running = Running::new();
    assert_eq!(running.run(&program), []);
    assert_eq!(running.built_anew(&program), 4);
    // The bottom, the hole's floor and the top.
    assert_heights(&running.model(), &[0.0, 0.5, 1.0]);
    let uses = parameter_uses(&program);
    let read: Vec<&String> = uses.keys().collect();
    assert_eq!(read, ["depth", "height"]);

    set(&mut program, "depth", "0.25");
    running.run(&program);
    assert_eq!(running.built_anew(&program), 1, "only the hole reads depth");
    assert_heights(&running.model(), &[0.0, 0.75, 1.0]);

    set(&mut program, "height", "2");
    running.run(&program);
    assert_eq!(running.built_anew(&program), 3, "the box reads height");
    assert_heights(&running.model(), &[0.0, 1.75, 2.0]);

    set(&mut program, "unrelated", "8");
    running.run(&program);
    assert_eq!(running.built_anew(&program), 0, "no step reads unrelated");

    let json = program.to_json().unwrap();
    let compact: String = json.split_whitespace().collect();
    assert!(
        compact.contains(r#""blind":{"formula":"height"}"#),
        "{json}"
    );
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
    let errors = running.run(&program);
    let [(id, error)] = errors.as_slice() else {
        panic!("the hole fails: {errors:?}");
    };
    assert_eq!(id, "hole");
    for said in [r#""-(dpeth + 1)""#, r#"there is no parameter "dpeth""#] {
        assert!(error.contains(said), "{said} is not said in {error}");
    }
}

/// The sketch `name` of `part`.
fn placed_sketch(part: &Part, name: &str) -> PlacedSketch {
    let reader = part.current();
    reader
        .retrieve_element(&whole_sketch(name))
        .unwrap()
        .clone()
}

/// The value of the parameter `name` the part `view` shows was built with:
/// a table's row, a number, a table's column (`table.column`) — or, for
/// [`COLOR`], its colour.
fn input(view: &Reader, name: &str) -> Option<Setting> {
    if name == COLOR {
        return read_properties(view).ok()?.color.clone().map(Setting::Text);
    }
    let values = read_parameter_values(view).ok()?;
    let (_, value) = values.0.iter().find(|(n, _)| n == name)?;
    match value {
        ParameterValue::Number(n) => Some(Setting::Number(*n)),
        ParameterValue::Text(row) => Some(Setting::Text(row.clone())),
    }
}

/// The point `p2` of the sketch `outline` of the part `view`, in space.
fn outline_corner(view: &Reader) -> [f64; 3] {
    let corner = view.retrieve_element(&sketch_element::<Point>("outline", PointId(2)));
    let p = corner.unwrap();
    [0, 1, 2].map(|k| p[k].to_f64())
}

/// The plate's dimensions follow its parameters — the depth a formula of
/// the width, the hole sized from the screw table — and its colour is one
/// of them too.
#[test]
fn dimensions_follow_parameters() {
    let program = parametric_plate();
    let part = program.built(registry).unwrap();
    let view = part.current();
    assert_eq!(input(&view, COLOR), Some(Setting::Text("#d0893e".into())));
    let p = outline_corner(&view);
    assert!((p[0] - 4.0).abs() < 1e-9 && (p[1] - 2.0).abs() < 1e-9);
    // Read back from JSON, the same.
    let json = program.to_json().unwrap();
    assert!(json.contains("screw.clearance"), "{json}");
    let back = Program::from_json(&json).unwrap();
    assert_eq!(back, program);
}

/// The parametric plate is a plate — 4 x 2 x 0.5 — with a hole in it:
/// its vertices span the plate, and the hole's sketch is built where the
/// plate's top is.
#[test]
fn the_parametric_plate_is_a_plate_with_a_hole() {
    let built = parametric_plate().built(registry).unwrap();
    let part = parametric_plate().build_model(registry).unwrap();
    let points: Vec<[f64; 3]> = part
        .topology()
        .vertices
        .values()
        .map(|v| [0, 1, 2].map(|k| v.point[k].to_f64()))
        .collect();
    let span = |k: usize| {
        let lo = points.iter().map(|p| p[k]).fold(f64::MAX, f64::min);
        let hi = points.iter().map(|p| p[k]).fold(f64::MIN, f64::max);
        [lo, hi]
    };
    assert!(
        (span(0)[0]).abs() < 1e-9 && (span(0)[1] - 4.0).abs() < 1e-9,
        "{:?} {points:?}",
        span(0)
    );
    assert!((span(1)[1] - 2.0).abs() < 1e-9, "{:?}", span(1));
    assert!((span(2)[1] - 0.5).abs() < 1e-9, "{:?}", span(2));
    let hole = placed_sketch(&built, "hole_sketch");
    let geometry = hole.sketch.enclose().unwrap();
    let centers: Vec<_> = hole
        .sketch
        .curves
        .values()
        .filter_map(|c| match c.kind {
            CurveKind::Circle { center, .. } => Some(geometry.points[&center]),
            _ => None,
        })
        .collect();
    let c = &centers[0];
    assert!(
        (c[0].to_f64() - 2.0).abs() < 1e-9 && (c[1].to_f64() - 1.0).abs() < 1e-9,
        "{c:?}"
    );
}

/// A runner rebuilds from scratch what a parameter changed reaches.
#[test]
fn runners_rebuild_when_parameters_change() {
    let mut program = parametric_plate();
    let mut running = Running::new();
    let corner = |running: &Running| outline_corner(&running.part.current())[0];
    assert_eq!(running.run(&program), []);
    assert!((corner(&running) - 4.0).abs() < 1e-9);
    let mut parameters = parameters_of(&program);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
        panic!("width is a number")
    };
    *expression = "5".into();
    set_parameters(&mut program, parameters);
    let numbers = parameter_numbers(&program).unwrap();
    assert_eq!(numbers.get("width").copied(), Some(5.0));
    assert_eq!(running.run(&program), []);
    assert!(
        (corner(&running) - 5.0).abs() < 1e-9,
        "{}",
        corner(&running)
    );
}

/// A parameter no step reads — the part's colour — changes the part
/// without building anything again: its colour is new, and every step's
/// part is the one already built.
#[test]
fn parameters_no_step_reads_rebuild_nothing() {
    let mut program = parametric_plate();
    let mut running = Running::new();
    assert_eq!(running.run(&program), []);
    // Every step but those of its parameters and properties, which build
    // neither a sketch nor a model.
    let building = (program.steps.iter())
        .filter(|s| ![PARAMETERS, PROPERTIES, LOOKUP].contains(&&*s.operation))
        .count();
    assert_eq!(running.built_anew(&program), building);
    let built = running.model();
    let mut parameters = parameters_of(&program);
    parameters.color = Some("#123456".into());
    set_parameters(&mut program, parameters);
    assert_eq!(running.run(&program), []);
    assert_eq!(
        input(&running.part.current(), COLOR),
        Some(Setting::Text("#123456".into()))
    );
    // Counted, not timed: no step read the colour, so none is built again.
    assert_eq!(
        running.built_anew(&program),
        0,
        "a colour change rebuilt steps"
    );
    assert_eq!(
        PartDescription::of(&running.model()).unwrap(),
        PartDescription::of(&built).unwrap()
    );
}
