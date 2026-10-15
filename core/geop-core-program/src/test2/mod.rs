//! The tests of [`crate::test`] again, with elements instead of targets of
//! their own: a part has one target of datums ([`Datums`]), which its steps
//! revise one after another — adding two datums, adding the one halfway
//! between two others — and a step builds on a datum by an
//! [`ElementReference`] into it.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use crate::{
    element::{ElementReference, HasElements, Kind},
    operation::Operation,
    operation_registry::OperationRegistry,
    part::{Part, StepContext},
    program::{Program, ProgramStep},
    runner::Runner,
    target::{Target, TargetReference},
    test::{Frame, Running, assert_encloses, inputs, label, record_built, take_built, vector},
    values::Design,
};

/// The datums of a part, by name: the one target of its kind a part has,
/// revised by every step that adds one.
#[derive(Clone, Debug, Default, PartialEq)]
struct Datums(BTreeMap<String, Frame>);

impl Target for Datums {
    const CHAIN: bool = true;
}

/// The part's datums.
fn datums() -> TargetReference<Datums> {
    TargetReference::new("datums")
}

/// A datum, as an element of [`Datums`].
struct Datum;

impl Kind for Datum {
    const NAME: &'static str = "datum";
}

impl HasElements<Datum> for Datums {
    type Element<'a> = &'a Frame;
    const SINGLETON: Option<&'static str> = Some("datums");

    fn element(&self, name: &str) -> GeopResult<&Frame> {
        (self.0.get(name)).ok_or_else(|| GeopError::new(format!("there is no datum `{name}`")))
    }
}

/// A reference to the datum `name`.
fn datum(name: &str) -> ElementReference<Datums, Datum> {
    ElementReference::in_singleton(name).unwrap()
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct AddTwoDatumsArgs {
    data1: Vector3<Design>,
    data2: Vector3<Design>,
}

/// Adds the datums `<step>_1` and `<step>_2` at the two points.
struct AddTwoDatums;

impl Operation for AddTwoDatums {
    const KIND: &'static str = "add_two_datums";
    type Args = AddTwoDatumsArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let args = (label("datums", step), step.to_string(), args.clone());
        ctx.produce(&datums(), &args, |reader, (label, step, args)| {
            record_built(label);
            let mut datums: Datums = reader.previous()?;
            for (suffix, data) in [("_1", args.data1), ("_2", args.data2)] {
                datums.0.insert(format!("{step}{suffix}"), Frame { data });
            }
            Ok(datums)
        })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
struct MiddleDatumArgs {
    a: ElementReference<Datums, Datum>,
    b: ElementReference<Datums, Datum>,
}

/// Adds the datum `<step>` halfway between the datums `a` and `b`.
struct MiddleDatum;

impl Operation for MiddleDatum {
    const KIND: &'static str = "middle_datum";
    type Args = MiddleDatumArgs;

    fn run(ctx: &mut StepContext, args: &Self::Args) -> GeopResult<()> {
        let step = ctx.step();
        let args = (label("datums", step), step.to_string(), args.clone());
        ctx.produce(&datums(), &args, |reader, (label, step, args)| {
            record_built(label);
            let mut datums: Datums = reader.previous()?;
            // Read through the reader, as of the steps before: the datums
            // this step revises.
            let a = reader.retrieve_element(&args.a)?.data;
            let b = reader.retrieve_element(&args.b)?.data;
            let half = Design::from_ratio(1, 2)?;
            let data = a.add(&b).prod_scalar(half);
            datums.0.insert(step.clone(), Frame { data });
            Ok(datums)
        })
    }
}

fn runner() -> Running {
    let registry = (OperationRegistry::default().with(AddTwoDatums))
        .and_then(|r| r.with(MiddleDatum))
        .unwrap();
    Running {
        runner: Runner::new(registry),
        part: Part::default(),
    }
}

fn add_two_datums(id: &str, data1: [f64; 3], data2: [f64; 3]) -> ProgramStep {
    let args = AddTwoDatumsArgs {
        data1: vector(data1),
        data2: vector(data2),
    };
    ProgramStep::new::<AddTwoDatums>(id, &args).unwrap()
}

fn middle_datum(id: &str, a: &str, b: &str) -> ProgramStep {
    let args = MiddleDatumArgs {
        a: datum(a),
        b: datum(b),
    };
    ProgramStep::new::<MiddleDatum>(id, &args).unwrap()
}

/// The datum `name`, as the part is now.
fn at(runner: &Running, name: &str) -> Vector3<Design> {
    let current = runner.part.current();
    current.retrieve_element(&datum(name)).unwrap().data
}

/// The failed steps and what went wrong at the root.
fn errors(runner: &mut Running, program: &Program) -> Vec<(String, String)> {
    let output = runner.run(program, &inputs(&[]), &BTreeMap::new());
    output
        .step_errors
        .iter()
        .map(|(step, e)| (step.clone(), e.root_message().to_string()))
        .collect()
}

/// Every step revises the one target of datums; a step reading datums of
/// an earlier one is rebuilt when that one changes, and nothing is rebuilt
/// when nothing did.
#[test]
fn datums_are_revised_by_each_step_and_rebuilt_only_when_changed() {
    let mut runner = runner();
    let program = |first: [f64; 3]| {
        Program::from_steps(vec![
            add_two_datums("s1", first, [0.0, 0.0, 1.0]),
            middle_datum("s2", "s1_1", "s1_2"),
        ])
    };

    assert_eq!(errors(&mut runner, &program([0.0; 3])), vec![]);
    assert_eq!(take_built(), ["datums@s1", "datums@s2"]);
    assert_encloses(at(&runner, "s1_1"), [0.0; 3]);
    assert_encloses(at(&runner, "s1_2"), [0.0, 0.0, 1.0]);
    assert_encloses(at(&runner, "s2"), [0.0, 0.0, 0.5]);

    assert_eq!(errors(&mut runner, &program([0.0; 3])), vec![]);
    assert_eq!(take_built(), Vec::<String>::new());

    assert_eq!(errors(&mut runner, &program([1.0, 0.0, 0.0])), vec![]);
    assert_eq!(take_built(), ["datums@s1", "datums@s2"]);
    assert_encloses(at(&runner, "s2"), [0.5, 0.0, 0.5]);
}

/// The datums a step sees are those of the steps before it: a later
/// step's datum does not exist for it. And since every step revises the
/// one target of datums, a step after the one that failed fails with it,
/// though it reads none of its datums: its revision starts from a version
/// that failed.
#[test]
fn a_step_sees_only_the_datums_added_before_it() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        middle_datum("m", "s1_1", "s1_2"),
        add_two_datums("s1", [0.0; 3], [1.0, 0.0, 0.0]),
    ]);
    let errors = errors(&mut runner, &program);
    assert_eq!(
        errors,
        [
            (
                "m".to_string(),
                "no step before here produces target `datums`".to_string()
            ),
            (
                "s1".to_string(),
                "target `datums` failed in step `m`: no step before here produces target `datums`"
                    .to_string()
            ),
        ]
    );
}

/// A datum that is not there is named, with the reference that asked for
/// it.
#[test]
fn a_missing_datum_is_named() {
    let mut runner = runner();
    let program = Program::from_steps(vec![
        add_two_datums("s1", [0.0; 3], [1.0, 0.0, 0.0]),
        middle_datum("m", "s1_1", "s9_2"),
    ]);
    assert_eq!(
        errors(&mut runner, &program),
        [("m".to_string(), "there is no datum `s9_2`".to_string())]
    );
}

/// In the one target of datums a part has, a step's arguments name a datum
/// by its name alone.
#[test]
fn a_program_names_datums_by_their_names() {
    let step = middle_datum("m", "s1_1", "s1_2");
    assert_eq!(
        serde_json::to_value(&step.args).unwrap(),
        serde_json::json!({"a": "s1_1", "b": "s1_2"})
    );
}
