//! Fixtures — a state of positions, frames defined from inputs, an
//! operation pinning a position to a frame — and the tests of constraints
//! and unknowns as targets of a part.

mod solve;

use std::{cell::RefCell, collections::BTreeMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{Ring, ScalInF64, Scalar},
    solvers::{
        least_squares::Rows,
        system::{Mobility, Variable},
    },
    vector::{Vector, Vector3},
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind},
    operation::Operation,
    operation_registry::OperationRegistry,
    part::{Part, Reader},
    program::{Program, ProgramStep},
    program_runner::{ProgramRunner, RunOutput},
    target::TargetReference,
};
use serde::{Deserialize, Serialize};

use crate::{
    constraint::{Bound, Constraint, Constraints, define_constraints},
    unknown::{Unknown, Unknowns, declare_unknowns},
};

/// The tests' numbers.
type Sc = ScalInF64;

/// Where a body is: what the tests' state holds — its position alone.
#[derive(Debug, Default, Clone, PartialEq)]
struct Pose {
    position: Vector3<Sc>,
}

impl Pose {
    fn at(position: [f64; 3]) -> Self {
        Self {
            position: vector(position),
        }
    }
}

/// A position moves by a step added to it.
impl Variable for Pose {
    type Scalar = Sc;

    fn dimension(&self) -> usize {
        3
    }

    fn retract(&self, step: &[Sc]) -> GeopResult<Self> {
        let by = three(step)?;
        Ok(Self {
            position: self.position.add(&by).map(Scalar::sharpen),
        })
    }

    fn retract_over(&self, step: &[Sc]) -> GeopResult<Self> {
        let by = three(step)?;
        Ok(Self {
            position: self.position.add(&by),
        })
    }

    fn differential(&self, step: &[Sc]) -> GeopResult<Option<Vec<Vec<Sc>>>> {
        three(step)?;
        Ok(None)
    }

    fn stay(&self, at: &Self, _: Sc) -> GeopResult<Rows<Sc>> {
        let mut rows = Rows::default();
        for (k, row) in identity(3).into_iter().enumerate() {
            rows.push(self.position[k].sub(at.position[k]), row);
        }
        Ok(rows)
    }

    fn same(&self, other: &Self) -> bool {
        self == other
    }
}

/// A step of a position: three numbers.
fn three(step: &[Sc]) -> GeopResult<Vector3<Sc>> {
    let [x, y, z] = step else {
        return Err(GeopError::new(format!(
            "a step of a position is three numbers, not {}",
            step.len()
        )));
    };
    Ok(Vector3::from_array([*x, *y, *z]))
}

/// The rows of the identity of size `n`: a derivative by a step that is
/// the change itself.
fn identity(n: usize) -> Vec<Vec<Sc>> {
    (0..n)
        .map(|i| {
            (0..n)
                .map(|j| if i == j { Sc::ONE } else { Sc::ZERO })
                .collect()
        })
        .collect()
}

/// A vector of sharp scalars, from plain numbers.
fn vector<const N: usize>(xs: [f64; N]) -> Vector<Sc, N> {
    Vector::from_array(xs.map(Sc::from_f64))
}

/// Asserts that `actual` encloses the point `expected`: computed values are
/// enclosures, rounded outward, so equal to no literal.
#[track_caller]
fn assert_encloses<const N: usize>(actual: Vector<Sc, N>, expected: [f64; N]) {
    assert!(
        actual.could_be_equal(&vector(expected)),
        "{actual:?} does not enclose {expected:?}"
    );
}

/// A point a step defines, which a pin holds a body to.
#[derive(Debug, Clone, Default, PartialEq)]
struct Frame {
    data: Vector3<Sc>,
}

/// A frame is read whole: a kind of its own.
impl Kind for Frame {
    const NAME: &'static str = "frame";
}

impl HasElements<Frame> for Frame {
    type Element<'a> = &'a Frame;

    fn element(&self, name: &str) -> GeopResult<&Frame> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!("a frame has no element `{name}`"))),
        }
    }
}

/// The frame target `name`, whole.
fn whole_frame(name: &str) -> ElementReference<Frame, Frame> {
    ElementReference::whole(TargetReference::new(name))
}

thread_local! {
    /// The steps whose generators ran in this test, in order.
    static BUILT: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
}

fn record_built(step: &str) {
    BUILT.with(|built| built.borrow_mut().push(step.to_string()));
}

fn take_built() -> Vec<String> {
    BUILT.with(|built| built.take())
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct InputFrameArgs {
    target: String,
    input: String,
}

/// Defines the target `target` as the frame at `(input, 0, 0)`.
struct InputFrameOperation;

impl Operation for InputFrameOperation {
    const KIND: &'static str = "input_frame";
    type Args = InputFrameArgs;
    type Input = Sc;
    type State = Pose;

    fn run(_: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        let reference = TargetReference::<Frame>::new(&args.target);
        part.define_target(&reference, args, |reader, args| {
            let x = (reader.retrieve_input(&args.input))
                .ok_or_else(|| GeopError::new(format!("input `{}` is not given", args.input)))?;
            Ok(Frame {
                data: Vector3::from_array([x, Sc::ZERO, Sc::ZERO]),
            })
        })
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct PinFrameArgs {
    frame: String,
    state: String,
}

/// Constrains the position `state` to the frame `frame`: a constraint
/// named after its step.
struct PinFrameOperation;

impl Operation for PinFrameOperation {
    const KIND: &'static str = "pin_frame";
    type Args = PinFrameArgs;
    type Input = Sc;
    type State = Pose;

    fn run(step: &str, part: &mut Part<Sc, Pose>, args: &Self::Args) -> GeopResult<()> {
        let args = (step.to_string(), args.clone());
        define_constraints(part, step, &args, |reader, (step, args)| {
            record_built(step);
            let frame = reader.retrieve_element(&whole_frame(&args.frame))?;
            let pose = reader.retrieve_state(&args.state);
            let pin = Constraint {
                residuals: pose.position.sub(&frame.data).to_array().to_vec(),
                jacobian: BTreeMap::from([(args.state.clone(), identity(3))]),
                bound: Bound::Zero,
            };
            Ok(BTreeMap::from([(step.clone(), pin)]))
        })
    }
}

fn input_frame(id: &str, target: &str, input: &str) -> ProgramStep {
    let args = InputFrameArgs {
        target: target.to_string(),
        input: input.to_string(),
    };
    ProgramStep::new::<InputFrameOperation>(id, &args).unwrap()
}

fn pin(id: &str, frame: &str, state: &str) -> ProgramStep {
    let args = PinFrameArgs {
        frame: frame.to_string(),
        state: state.to_string(),
    };
    ProgramStep::new::<PinFrameOperation>(id, &args).unwrap()
}

fn inputs(values: &[(&str, f64)]) -> BTreeMap<String, Sc> {
    (values.iter())
        .map(|(name, value)| (name.to_string(), Sc::from_f64(*value)))
        .collect()
}

fn pose_at(x: f64) -> BTreeMap<String, Pose> {
    BTreeMap::from([("body".to_string(), Pose::at([x, 0.0, 0.0]))])
}

/// The failed steps and what went wrong at the root.
fn errors(output: &RunOutput) -> Vec<(String, String)> {
    (output.step_errors.iter())
        .map(|(step, e)| (step.clone(), e.root_message().to_string()))
        .collect()
}

/// The operations of the fixtures.
fn registry() -> OperationRegistry<Sc, Pose> {
    (OperationRegistry::default().with(InputFrameOperation))
        .and_then(|o| o.with(PinFrameOperation))
        .unwrap()
}

fn runner() -> ProgramRunner<Sc, Pose> {
    ProgramRunner::new(registry())
}

/// The constraints of the part as it is now.
fn constraints_now(part: &Part<Sc, Pose>) -> GeopResult<Constraints<Pose>> {
    Constraints::of(&part.current())
}

/// An operation constrains the state, and the run reports the constraint
/// with its derivatives. It is made again when the state it read moves, as
/// a solver moves it, and only then. A part no step constrains has no
/// constraints.
#[test]
fn constraints_on_the_state_are_reported() {
    let mut runner = runner();
    let x = inputs(&[("x", 1.0)]);
    let frame_only = Program::new(vec![input_frame("a", "a", "x")]);
    assert_eq!(errors(&runner.run(&frame_only, &x, &pose_at(3.0))), []);
    assert!(constraints_now(&runner.part).unwrap().is_empty());

    let program = Program::new(vec![input_frame("a", "a", "x"), pin("pin", "a", "body")]);
    let residuals = |runner: &ProgramRunner<_, Pose>| {
        let constraints = constraints_now(&runner.part).unwrap();
        let residuals = &constraints.get("pin").unwrap().residuals;
        Vector3::from_array([residuals[0], residuals[1], residuals[2]])
    };

    assert_eq!(errors(&runner.run(&program, &x, &pose_at(3.0))), []);
    let constraints = constraints_now(&runner.part).unwrap();
    assert_eq!(
        constraints.iter().map(|(name, _)| name).collect::<Vec<_>>(),
        ["pin"]
    );
    assert_eq!(
        constraints.get("pin").unwrap().jacobian,
        BTreeMap::from([("body".to_string(), identity(3))])
    );
    assert_encloses(residuals(&runner), [2.0, 0.0, 0.0]);
    assert_eq!(take_built(), ["pin"]);

    runner.run(&program, &x, &pose_at(3.0));
    assert_eq!(take_built(), Vec::<String>::new());
    assert_encloses(residuals(&runner), [2.0, 0.0, 0.0]);

    runner.run(&program, &x, &pose_at(1.0));
    assert_eq!(take_built(), ["pin"]);
    assert_encloses(residuals(&runner), [0.0, 0.0, 0.0]);
}

/// A constraint of `residuals` residuals and one row of derivatives by
/// `body`.
fn by_body(residuals: usize) -> Constraint<Pose> {
    Constraint {
        residuals: vec![Sc::ZERO; residuals],
        jacobian: BTreeMap::from([("body".to_string(), vec![vec![Sc::ZERO; 3]])]),
        bound: Bound::Zero,
    }
}

/// A constraint's derivatives must fit it: by states it read, a row per
/// residual. A step that defines one that does not fails, and so do the
/// part's constraints.
#[test]
fn inconsistent_constraint_is_an_error() {
    let mut part = Part::<Sc, Pose>::default();
    part.begin_run(&BTreeMap::new(), &pose_at(0.0));
    part.begin_step();
    let unread = define_constraints(&mut part, "s1", &(), |_, _| {
        Ok(BTreeMap::from([("unread".to_string(), by_body(1))]))
    });
    assert_eq!(
        unread.unwrap_err().root_message(),
        "constraint `unread` gives a derivative by state `body`, which it does not read"
    );

    part.begin_step();
    let rows = define_constraints(&mut part, "s2", &(), |reader, _| {
        reader.retrieve_state("body");
        Ok(BTreeMap::from([("rows".to_string(), by_body(2))]))
    });
    assert_eq!(
        rows.unwrap_err().root_message(),
        "constraint `rows` has 2 residuals but 1 rows of derivatives by state `body`"
    );
    assert!(constraints_now(&part).is_err());
}

/// Each step's constraints are a target of its own: one that fails leaves
/// the later steps' alone, and a solve reading them all finds the failure,
/// not a part with fewer constraints.
#[test]
fn a_failed_constraint_step_fails_the_constraints() {
    let mut runner = runner();
    let program = Program::new(vec![
        input_frame("a", "a", "x"),
        pin("broken", "missing", "body"),
        pin("pin", "a", "body"),
    ]);
    let output = runner.run(&program, &inputs(&[("x", 1.0)]), &pose_at(0.0));
    let errors = errors(&output);
    let [(broken, why_broken)] = &errors[..] else {
        panic!("{errors:?}");
    };
    assert_eq!(broken, "broken");
    assert!(
        why_broken.contains("target `missing` is not defined"),
        "{why_broken}"
    );
    let error = constraints_now(&runner.part).unwrap_err();
    assert!(error.root_message().contains("`constraints:broken` failed"), "{error}");
}

/// A constraint's name is the part's to resolve by: a second step defining
/// it makes the part's constraints an error naming both.
#[test]
fn constraint_defined_twice_is_an_error() {
    fn define(_: &Reader<Sc, Pose>, _: &()) -> GeopResult<BTreeMap<String, Constraint<Pose>>> {
        let free = Constraint {
            residuals: vec![Sc::ZERO],
            jacobian: BTreeMap::new(),
            bound: Bound::Zero,
        };
        Ok(BTreeMap::from([("pin".to_string(), free)]))
    }
    let mut part = Part::<Sc, Pose>::default();
    part.begin_run(&BTreeMap::new(), &BTreeMap::new());
    part.begin_step();
    define_constraints(&mut part, "one", &(), define).unwrap();
    part.begin_step();
    define_constraints(&mut part, "two", &(), define).unwrap();
    part.end_run(0);
    assert_eq!(
        constraints_now(&part).unwrap_err().root_message(),
        "constraint `pin` is defined twice: by step `one` and by step `two`"
    );
}

/// The unknowns are one target by the state they are: of two declarations
/// of a state, one that holds it fixed wins, and otherwise the first.
#[test]
fn unknowns_merge_by_state() {
    fn declare(
        _: &Reader<Sc, Pose>,
        declared: &[(String, Mobility); 2],
    ) -> GeopResult<Vec<Unknown<Pose>>> {
        Ok((declared.iter())
            .map(|(state, mobility)| Unknown {
                state: state.clone(),
                mobility: *mobility,
                seed: None,
                unless: None,
            })
            .collect())
    }
    let declared = |state: &str, mobility| (state.to_string(), mobility);
    let mobility = |part: &Part<Sc, Pose>, state: &str| {
        let unknowns = Unknowns::of(&part.current()).unwrap();
        unknowns.get(state).map(|unknown| unknown.mobility)
    };

    let mut part = Part::<Sc, Pose>::default();
    part.begin_run(&BTreeMap::new(), &BTreeMap::new());
    assert!(
        Unknowns::of(&part.current())
            .unwrap()
            .iter()
            .next()
            .is_none()
    );
    part.begin_step();
    let first = [
        declared("body", Mobility::Held),
        declared("loose", Mobility::Held),
    ];
    declare_unknowns(&mut part, &first, declare).unwrap();
    part.begin_step();
    let second = [
        declared("body", Mobility::Fixed),
        declared("loose", Mobility::Follows),
    ];
    declare_unknowns(&mut part, &second, declare).unwrap();
    part.end_run(0);

    assert_eq!(mobility(&part, "body"), Some(Mobility::Fixed));
    assert_eq!(mobility(&part, "loose"), Some(Mobility::Held));
    assert_eq!(mobility(&part, "other"), None);
}
