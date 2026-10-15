//! [`Editors`] giving each step its arguments, form and events against the
//! part as it was before it, and as it built it.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::{ScalInF64, Scalar},
    vector::Vector3,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    operation::Operation,
    part::{Input, Part, Reader, State},
    program::{Program, ProgramStep},
    program_runner::ProgramRunner,
    target::TargetReference,
};
use serde::{Deserialize, Serialize};

use crate::{
    edit_form::{EditForm, FormState},
    editable::{Context, Editable, Editors, OperationGroup, OperationInfo, OperationTier},
    editing::{Control, Dialog, Event, Form, Unit, Value},
    numeric_input::{NumericInput, ParameterValue, parameter},
    part_operation,
    part_operation::PartOperation,
    selectable::{Selectable, SelectableElement, selected},
};

/// The tests' numbers: what a part is given, its state and its forms.
type Sc = ScalInF64;

/// A part of the tests, given numbers and a number as its state.
type TestPart = Part<Sc, Sc>;
type TestRunner = ProgramRunner<Sc, Sc>;

/// What a test operation is to an editor: only its name matters.
const INFO: OperationInfo = OperationInfo::new("", "", OperationGroup::Solid, OperationTier::Menu);

fn v(x: f64, y: f64, z: f64) -> Vector3<Sc> {
    Vector3::from_array([x, y, z].map(Sc::from_f64))
}

/// A point, read whole: a kind of its own.
#[derive(Clone, Debug, Default, PartialEq)]
struct Point(Vector3<Sc>);

impl Kind for Point {
    const NAME: &'static str = "point";
}

impl HasElements<Point> for Point {
    type Element<'a> = &'a Point;

    fn element(&self, name: &str) -> GeopResult<&Point> {
        match name.is_empty() {
            true => Ok(self),
            false => Err(GeopError::new(format!("a point has no element `{name}`"))),
        }
    }
}

/// Defines the point target `<step>` at `at`.
struct GivenPoint;

impl Operation for GivenPoint {
    const KIND: &'static str = "given_point";
    type Args = Vector3<Sc>;
    type Input = Sc;
    type State = Sc;

    fn run(step: &str, _: &TestRunner, part: &mut TestPart, at: &Vector3<Sc>) -> GeopResult<()> {
        part.define_target(&TargetReference::new(step), at, |_, at| Ok(Point(*at)))
    }
}

impl Editable for GivenPoint {
    const INFO: OperationInfo = INFO;
    type Scalar = Sc;
    type EditingState = ();

    fn new_args(_: &Context<Sc, Sc, Sc>) -> Vector3<Sc> {
        Vector3::zero()
    }

    fn presentation(_: &Context<Sc, Sc, Sc>, _: &Vector3<Sc>, _: &()) -> Form<Sc> {
        Form::default()
    }

    fn process_user_event(
        _: &Context<Sc, Sc, Sc>,
        at: &Vector3<Sc>,
        _: &(),
        _: Event<Sc>,
    ) -> (Vector3<Sc>, ()) {
        (*at, ())
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct ShiftArgs {
    point: String,
    by: Vector3<Sc>,
}

/// Moves the point target `point` by `by`.
struct Shift;

impl Operation for Shift {
    const KIND: &'static str = "shift";
    type Args = ShiftArgs;
    type Input = Sc;
    type State = Sc;

    fn run(_: &str, _: &TestRunner, part: &mut TestPart, args: &ShiftArgs) -> GeopResult<()> {
        let point = TargetReference::<Point>::new(&args.point);
        part.revise_target(&point, &args.by, |reader, by| {
            Ok(Point(reader.previous::<Point>()?.0.add(by)))
        })
    }
}

impl Editable for Shift {
    const INFO: OperationInfo = INFO;
    type Scalar = Sc;
    type EditingState = ();

    fn new_args(_: &Context<Sc, Sc, Sc>) -> ShiftArgs {
        ShiftArgs::default()
    }

    fn presentation(_: &Context<Sc, Sc, Sc>, _: &ShiftArgs, _: &()) -> Form<Sc> {
        Form::default()
    }

    /// Pressing the field `origin` moves the point to the origin, as the
    /// step sees it; pressing `built` shifts it by where the step left it.
    fn process_user_event(
        context: &Context<Sc, Sc, Sc>,
        args: &ShiftArgs,
        _: &(),
        event: Event<Sc>,
    ) -> (ShiftArgs, ()) {
        let point = ElementReference::<Point, Point>::whole(TargetReference::new(&args.point));
        let mut args = args.clone();
        let Event::Field { key, .. } = event else {
            return (args, ());
        };
        let view = match key.as_str() {
            "origin" => Some(&context.before),
            "built" => context.built.as_ref(),
            _ => None,
        };
        if let Some(point) = view.and_then(|view| view.retrieve_element(&point).ok()) {
            args.by = match key.as_str() {
                "origin" => point.0.neg(),
                _ => point.0,
            };
        }
        (args, ())
    }
}

fn editors() -> Editors<Sc, Sc, Sc> {
    (Editors::default().with(GivenPoint))
        .and_then(|e| e.with(Shift))
        .unwrap()
}

fn given(id: &str, x: f64) -> ProgramStep {
    ProgramStep::new::<GivenPoint>(id, &v(x, 0.0, 0.0)).unwrap()
}

fn shift(id: &str, point: &str, x: f64) -> ProgramStep {
    let args = ShiftArgs {
        point: point.into(),
        by: v(x, 0.0, 0.0),
    };
    ProgramStep::new::<Shift>(id, &args).unwrap()
}

/// What a press of `origin` makes the shift `step` of `program`, as last
/// run on `part`.
fn pressed(
    editors: &Editors<Sc, Sc, Sc>,
    runner: &TestRunner,
    part: &TestPart,
    program: &Program,
    step: &str,
) -> GeopResult<f64> {
    press(editors, runner, part, program, step, "origin")
}

/// What a press of the field `key` makes the shift `step` of `program`, as
/// last run on `part`.
fn press(
    editors: &Editors<Sc, Sc, Sc>,
    runner: &TestRunner,
    part: &TestPart,
    program: &Program,
    step: &str,
    key: &str,
) -> GeopResult<f64> {
    let state = editors.initial_editing(program, step)?;
    let press = Event::Field {
        key: key.into(),
        value: Value::Press,
    };
    let edited = editors.process_user_event(runner, part, program, step, &*state, press, None)?;
    let args: ShiftArgs = serde_json::from_value(edited.args).unwrap();
    Ok(args.by[0].to_f64())
}

/// A step is edited against the part as it was before it, whichever step
/// the editor looked at last.
#[test]
fn a_step_is_edited_against_the_part_before_it() {
    let editors = editors();
    let runner = ProgramRunner::new(editors.operations().unwrap());
    let mut part = TestPart::default();
    let program = Program::new(vec![
        given("a", 1.0),
        shift("s1", "a", 1.0),
        shift("s2", "a", 5.0),
    ]);
    runner.run(&mut part, &program, &Default::default(), &Default::default());
    for (step, by) in [("s2", -2.0), ("s1", -1.0), ("s2", -2.0)] {
        assert_eq!(pressed(&editors, &runner, &part, &program, step).unwrap(), by);
    }
}

/// A new step starts from its operation's defaults, inserted before a step
/// of the last run or after its last; its form is the operation's.
#[test]
fn a_new_step_starts_from_its_operation() {
    let editors = editors();
    let runner = ProgramRunner::new(editors.operations().unwrap());
    let mut part = TestPart::default();
    let program = Program::new(vec![given("a", 1.0)]);
    runner.run(&mut part, &program, &Default::default(), &Default::default());

    let step = editors
        .new_step(&runner, &part, &program, "shift", "s", None)
        .unwrap();
    assert_eq!(step.operation, "shift");
    let args: ShiftArgs = serde_json::from_value(step.args.clone()).unwrap();
    assert_eq!(args, ShiftArgs::default());
    assert!(
        editors
            .new_step(&runner, &part, &program, "shift", "s", Some("a"))
            .is_ok()
    );
    let unknown = editors.new_step(&runner, &part, &program, "nope", "s", None);
    assert_eq!(
        unknown.unwrap_err().root_message(),
        "unknown operation `nope`"
    );

    let editing = editors.initial_editing(&program, "a").unwrap();
    let form = editors
        .presentation(&runner, &part, &program, "a", &*editing)
        .unwrap();
    assert_eq!(form, Form::default());
}

/// A step sees the part as it built it: before the next step that ran — a
/// step inserted after it since does not count, having not run — or, for
/// the last, the part as the run left it.
#[test]
fn a_step_sees_what_it_built() {
    let editors = editors();
    let runner = ProgramRunner::new(editors.operations().unwrap());
    let mut part = TestPart::default();
    let program = Program::new(vec![
        given("a", 1.0),
        shift("s1", "a", 1.0),
        shift("s2", "a", 5.0),
    ]);
    runner.run(&mut part, &program, &Default::default(), &Default::default());
    assert_eq!(
        press(&editors, &runner, &part, &program, "s1", "built").unwrap(),
        2.0
    );
    assert_eq!(
        press(&editors, &runner, &part, &program, "s2", "built").unwrap(),
        7.0
    );

    let edited = Program::new(vec![
        given("a", 1.0),
        shift("s1", "a", 1.0),
        shift("new", "a", 3.0),
        shift("s2", "a", 5.0),
    ]);
    assert_eq!(
        press(&editors, &runner, &part, &edited, "s1", "built").unwrap(),
        2.0
    );
}

/// Edges in chains: a target whose edges are drawn, and whose chains are
/// picked by clicking an edge of one.
#[derive(Clone, Debug, Default, PartialEq)]
struct Chains(Vec<(String, Vec<String>)>);

struct Edge;

impl Kind for Edge {
    const NAME: &'static str = "edge";
}

struct Chain;

impl Kind for Chain {
    const NAME: &'static str = "chain";
}

impl HasElements<Edge> for Chains {
    type Element<'a> = &'a str;

    fn element(&self, name: &str) -> GeopResult<&str> {
        (self.0.iter())
            .flat_map(|(_, edges)| edges)
            .find(|edge| *edge == name)
            .map(String::as_str)
            .ok_or_else(|| GeopError::new(format!("there is no edge `{name}`")))
    }
}

impl HasElements<Chain> for Chains {
    type Element<'a> = &'a [String];

    fn element(&self, name: &str) -> GeopResult<&[String]> {
        (self.0.iter())
            .find(|(chain, _)| chain == name)
            .map(|(_, edges)| edges.as_slice())
            .ok_or_else(|| GeopError::new(format!("there is no chain `{name}`")))
    }
}

impl HasElements<Chains> for Chains {
    type Element<'a> = &'a Chains;

    fn element(&self, name: &str) -> GeopResult<&Chains> {
        whole(self, name)
    }
}

impl Kind for Chains {
    const NAME: &'static str = "chains";
}

impl SelectableElement<Chains> for Chain {
    const PICKED_FROM: &'static [&'static str] = &["edge"];

    fn select<I: Input, S: State>(
        reader: &Reader<I, S>,
        hit: &PickedElement,
    ) -> Option<ElementReference<Chains, Chain>> {
        let target = TargetReference::<Chains>::new(&hit.target);
        let chains = reader
            .retrieve_element(&ElementReference::<Chains, Chains>::whole(target.clone()))
            .ok()?;
        let (chain, _) = (chains.0.iter()).find(|(_, edges)| edges.contains(&hit.name))?;
        Some(ElementReference::new(target, chain))
    }

    fn drawn<I: Input, S: State>(
        reader: &Reader<I, S>,
        chain: &ElementReference<Chains, Chain>,
    ) -> Vec<PickedElement> {
        let edges = reader.retrieve_element(chain).unwrap_or_default();
        (edges.iter())
            .map(|edge| {
                ElementReference::<Chains, Edge>::new(chain.target().clone(), edge).picked()
            })
            .collect()
    }
}

/// A part of one target of chains, `c1` of the edges `e1` and `e2`, `c2`
/// of `e3`.
fn chains() -> TestPart {
    let mut part = TestPart::default();
    part.begin_run(&Default::default(), &Default::default());
    part.begin_step();
    let chains = Chains(vec![
        ("c1".into(), vec!["e1".into(), "e2".into()]),
        ("c2".into(), vec!["e3".into()]),
    ]);
    (part.define_target(&TargetReference::new("chains"), &chains, |_, c| {
        Ok(c.clone())
    }))
    .unwrap();
    part.end_run(0);
    part
}

fn edge(name: &str) -> PickedElement {
    ElementReference::<Chains, Edge>::new(TargetReference::new("chains"), name).picked()
}

fn chain(name: &str) -> ElementReference<Chains, Chain> {
    ElementReference::new(TargetReference::new("chains"), name)
}

/// A field of chains is picked by clicking edges, and lights every edge of
/// the chains it holds.
#[test]
fn a_field_picks_what_its_elements_are_picked_from() {
    let part = chains();
    let mut dialog = Dialog::<Sc>::new();
    dialog.pick("chains", "chains", &part.current(), &[chain("c1")], true);
    let Some(Control::Reference(reference)) = dialog.get("chains") else {
        panic!("no reference field");
    };
    assert_eq!(reference.accepts, ["edge"]);
    assert_eq!(
        dialog.picked().cloned().collect::<Vec<_>>(),
        [edge("e1"), edge("e2")]
    );
}

/// A click on an edge selects its chain; a click on an edge of a chain
/// held takes the chain out; a click that selects nothing is dropped.
#[test]
fn a_click_selects_the_element_it_is_on() {
    let part = chains();
    let reader = part.current();
    let held = chain("c1").picked();
    let picks = [held.clone(), edge("e3")];
    let picked: Vec<ElementReference<Chains, Chain>> = selected(&reader, &picks);
    assert_eq!(picked, [chain("c1"), chain("c2")]);
    let picks = [held, edge("e2")];
    assert_eq!(selected::<_, _, ElementReference<Chains, Chain>>(&reader, &picks), []);
    let picks = [edge("e9")];
    assert_eq!(selected::<_, _, ElementReference<Chains, Chain>>(&reader, &picks), []);
}

/// Picked whole, from a list of what the part has.
impl SelectableElement<Chains> for Chains {
    const PICKED_FROM: &'static [&'static str] = &["chains"];

    fn select<I: Input, S: State>(
        _: &Reader<I, S>,
        hit: &PickedElement,
    ) -> Option<ElementReference<Chains, Chains>> {
        ElementReference::from_picked(hit)
    }
}

/// Names of edges.
#[derive(Debug, PartialEq)]
struct Names(Vec<String>);

impl From<&[String]> for Names {
    fn from(names: &[String]) -> Self {
        Names(names.to_vec())
    }
}

impl From<&Chains> for Names {
    fn from(chains: &Chains) -> Self {
        Names((chains.0.iter()).flat_map(|(_, edges)| edges.clone()).collect())
    }
}

crate::one_of! {
    /// A chain, or all of them, as names of edges.
    enum Edges -> Names {
        Chain(ElementReference<Chains, Chain>),
        All(ElementReference<Chains, Chains>),
    }
}

/// A one-of field picks from what any of its variants is picked from, the
/// first variant that selects a click taking it; it saves by its variant's
/// name, and resolves through whichever it holds.
#[test]
fn one_of_several_sources_is_picked_by_the_first_that_selects_it() {
    let part = chains();
    let reader = part.current();
    assert_eq!(Edges::picked_from(), ["edge", "chains"]);
    let all = Edges::All(ElementReference::whole(TargetReference::new("chains")));
    let picked: Vec<Edges> = selected(&reader, &[edge("e3"), all.picked()]);
    assert_eq!(picked, [Edges::Chain(chain("c2")), all.clone()]);
    assert_eq!(
        serde_json::to_value(&picked).unwrap(),
        serde_json::json!([{"chain": {"target": "chains", "name": "c2"}}, {"all": {"target": "chains"}}])
    );
    assert_eq!(picked[0].resolve(&reader).unwrap().0, ["e3"]);
    assert_eq!(all.resolve(&reader).unwrap().0, ["e1", "e2", "e3"]);
    assert_eq!(picked[0].drawn(&reader), [edge("e3")]);
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct ScaleArgs {
    chains: Vec<ElementReference<Chains, Chain>>,
    by: NumericInput,
}

/// Scales chains by a number, typed plain or as a formula: a
/// [`PartOperation`] of a pick field and a numeric field.
struct Scale;

impl PartOperation for Scale {
    const KIND: &'static str = "scale";
    const INFO: OperationInfo = INFO;
    type Args = ScaleArgs;
    type Input = Sc;
    type State = Sc;
    type Scalar = Sc;
    type Session = ();

    fn new_args(_: &Context<Sc, Sc, Sc>) -> ScaleArgs {
        ScaleArgs::default()
    }

    fn run(step: &str, _: &TestRunner, part: &mut TestPart, args: &ScaleArgs) -> GeopResult<()> {
        let scaled = TargetReference::<f64>::new(step);
        part.define_target(&scaled, &args.by, |reader, by| by.resolve(reader))
    }

    fn form<'a>(
        context: &'a Context<'a, Sc, Sc, Sc>,
        args: &ScaleArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Sc, ScaleArgs> {
        let mut f = EditForm::<Sc, ScaleArgs>::default();
        let reader = &context.before;
        f.pick(
            "chains",
            "chains",
            reader,
            &args.chains,
            true,
            |edit, chains| edit.args.chains = chains,
        );
        let by = args.by.number("by", reader, Unit::Length);
        f.numeric("by", by, |edit, by| edit.args.by = by);
        f
    }
}

part_operation!(Scale(ScaleArgs));

/// A part operation is edited through its form: a click on an edge picks
/// its chain, text typed into a numeric field sets a formula, and the
/// field shows what the formula comes to with the part's inputs.
#[test]
fn a_part_operation_is_edited_through_its_form() {
    let mut part = chains();
    part.begin_run(&Default::default(), &Default::default());
    part.begin_step();
    let chains = Chains(vec![("c1".into(), vec!["e1".into(), "e2".into()])]);
    (part.define_target(&TargetReference::new("chains"), &chains, |_, c| {
        Ok(c.clone())
    }))
    .unwrap();
    (part.define_target(&parameter("width"), &6.0, |_, &width| {
        Ok(ParameterValue::Number(width))
    }))
    .unwrap();
    part.end_run(0);
    let runner = TestRunner::new(Default::default());
    let context = Context {
        step: "s",
        runner: &runner,
        before: part.current(),
        built: None,
        scene: None,
    };
    let editing = FormState::default();
    let send = |args: &ScaleArgs, key: &str, value: Value| {
        let event = Event::Field {
            key: key.into(),
            value,
        };
        <Scale as Editable>::process_user_event(&context, args, &editing, event).0
    };
    let args = send(
        &ScaleArgs::default(),
        "chains",
        Value::Elements(vec![edge("e2")]),
    );
    assert_eq!(args.chains, [chain("c1")]);
    let args = send(&args, "by", Value::Text("width / 2".into()));
    assert_eq!(args.by, NumericInput::from("width / 2"));
    let form = <Scale as Editable>::presentation(&context, &args, &editing);
    let Some(Control::Number(by)) = form.dialog.get("by") else {
        panic!("no number field");
    };
    assert_eq!((by.value, by.text.as_deref()), (3.0, Some("width / 2")));
}
