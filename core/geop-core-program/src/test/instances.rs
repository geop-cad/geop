//! Instances: a step places the part a program of its own builds, and the
//! part reads its elements behind the instance's name.

use std::{any::Any, collections::BTreeMap, rc::Rc};

use geop_core_math::geop_error::GeopResult;
use serde::{Deserialize, Serialize};

use crate::{
    instance::{Factor, Instance, Placement, Subpart},
    operation::Operation,
    part::{Part, Reader},
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
    test::{
        Pose, Sc, StateFrameArgs, StateFrameOperation, assert_encloses, inputs, label, ok,
        record_built, registry, take_built, whole_frame,
    },
};

/// A part a program built, as placed.
struct Built(Part<Sc, Pose>);

impl Subpart<Sc, Pose> for Built {
    fn part(&self) -> &Part<Sc, Pose> {
        &self.0
    }

    fn into_any(self: Rc<Self>) -> Rc<dyn Any> {
        self
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PlaceArgs {
    pub program: Program,
}

/// Places the part `program` builds as the instance named by the step,
/// where the state `<step>.pose` puts it.
pub struct PlaceOperation;

impl Operation for PlaceOperation {
    const KIND: &'static str = "place";
    type Args = PlaceArgs;
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        let args = (label(step, step), step.to_string(), args.program.clone());
        part.define_instance(step, &args, build)
    }
}

/// The instance the step `step` places: `program` run with `state`.
fn build(
    _: &Reader<Sc, Pose>,
    (label, step, program): &(String, String, Program),
    state: &BTreeMap<String, Pose>,
) -> GeopResult<Instance<Sc, Pose>> {
    record_built(label);
    let mut runner = ProgramRunner::new(registry());
    ok(runner.run(program, &inputs(&[]), state));
    Ok(Instance {
        component: Rc::new(Built(runner.part)),
        placement: Placement::State(format!("{step}.pose")),
    })
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CopyArgs {
    pub of: String,
    pub by: [f64; 3],
}

/// Places a copy of the instance `of`, `by` after it.
pub struct CopyOperation;

impl Operation for CopyOperation {
    const KIND: &'static str = "copy";
    type Args = CopyArgs;
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        part.define_instance(step, args, |reader, args, _| {
            Ok(Instance {
                component: reader.instance(&args.of)?.component.clone(),
                placement: Placement::After {
                    of: args.of.clone(),
                    motion: Pose::at(args.by),
                },
            })
        })
    }
}

pub(crate) fn place(id: &str, steps: Vec<ProgramStep>) -> ProgramStep {
    let args = PlaceArgs {
        program: Program::new(steps),
    };
    ProgramStep::new::<PlaceOperation>(id, &args).unwrap()
}

fn copy(id: &str, of: &str, by: [f64; 3]) -> ProgramStep {
    let args = CopyArgs {
        of: of.to_string(),
        by,
    };
    ProgramStep::new::<CopyOperation>(id, &args).unwrap()
}

/// A frame at the state `state`.
fn state_frame(id: &str, state: &str) -> ProgramStep {
    let args = StateFrameArgs {
        state: state.to_string(),
    };
    ProgramStep::new::<StateFrameOperation>(id, &args).unwrap()
}

fn state(values: &[(&str, [f64; 3])]) -> BTreeMap<String, Pose> {
    (values.iter())
        .map(|(name, at)| (name.to_string(), Pose::at(*at)))
        .collect()
}

/// An element of an instance is read behind its name, as its own part has
/// it; the instance is run with the part's state under its name, and built
/// again only when that changes.
#[test]
fn an_instance_is_run_with_the_state_under_its_name() {
    let mut runner = ProgramRunner::new(registry());
    let program = Program::new(vec![place("p", vec![state_frame("f", "s")])]);

    let at = |x| state(&[("p/s", [x, 0.0, 0.0]), ("p.pose", [5.0, 0.0, 0.0])]);
    ok(runner.run(&program, &inputs(&[]), &at(1.0)));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("p/f")).unwrap();
    assert_eq!(placed.instance, "p");
    assert_encloses(placed.element.data, [1.0, 0.0, 0.0]);
    assert_eq!(
        reader.placement("p").unwrap(),
        [Factor::State {
            state: "p.pose".into(),
            value: Pose::at([5.0, 0.0, 0.0])
        }]
    );
    assert_eq!(take_built(), ["p@p", "f@f"]);

    // Moving it builds nothing: where it is is read where it is asked.
    let moved = state(&[("p/s", [1.0, 0.0, 0.0]), ("p.pose", [6.0, 0.0, 0.0])]);
    ok(runner.run(&program, &inputs(&[]), &moved));
    assert_eq!(take_built(), Vec::<String>::new());

    ok(runner.run(&program, &inputs(&[]), &at(2.0)));
    assert_eq!(take_built(), ["p@p", "f@f"]);
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("p/f")).unwrap();
    assert_encloses(placed.element.data, [2.0, 0.0, 0.0]);
}

/// An element of an instance is no element of the part itself.
#[test]
fn an_element_of_an_instance_is_not_the_parts_own() {
    let mut runner = ProgramRunner::new(registry());
    let program = Program::new(vec![place("p", vec![state_frame("f", "s")])]);
    ok(runner.run(&program, &inputs(&[]), &state(&[])));
    let error = (runner.part.current())
        .retrieve_element(&whole_frame("p/f"))
        .unwrap_err()
        .to_string();
    assert!(error.contains("placed part `p`"), "{error}");
}

/// A part placed in a placed part is where both put it, each by the state
/// the part names it by; in a copy, by what nothing moves.
#[test]
fn nested_and_copied_instances_compose() {
    let mut runner = ProgramRunner::new(registry());
    let inner = vec![place("q", vec![state_frame("f", "s")])];
    let program = Program::new(vec![place("p", inner), copy("c", "p", [0.0, 3.0, 0.0])]);
    let given = state(&[
        ("p.pose", [1.0, 0.0, 0.0]),
        ("p/q.pose", [0.0, 2.0, 0.0]),
        ("p/q/s", [0.0, 0.0, 4.0]),
    ]);
    ok(runner.run(&program, &inputs(&[]), &given));
    let reader = runner.part.current();
    let placed = reader.retrieve_placed(&whole_frame("p/q/f")).unwrap();
    assert_eq!(placed.instance, "p/q");
    assert_encloses(placed.element.data, [0.0, 0.0, 4.0]);
    let moved = |state: &str, at| Factor::State {
        state: state.into(),
        value: Pose::at(at),
    };
    assert_eq!(
        reader.placement("p/q").unwrap(),
        [moved("p.pose", [1.0, 0.0, 0.0]), moved("p/q.pose", [0.0, 2.0, 0.0])]
    );
    assert_eq!(
        reader.placement("c/q").unwrap(),
        [
            Factor::Fixed(Pose::at([0.0, 3.0, 0.0])),
            moved("p.pose", [1.0, 0.0, 0.0]),
            Factor::Fixed(Pose::at([0.0, 2.0, 0.0])),
        ]
    );
    let names: Vec<&str> = reader.instances().iter().map(|(name, _)| *name).collect();
    assert_eq!(names, ["p", "c"]);
}
