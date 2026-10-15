//! [`Editors`] giving each step its arguments, form and events against the
//! part as it was before it, and as it built it.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, PickedElement, whole},
    numeric::{NumericInput, ParameterValue, parameter},
    operation::Operation,
    operation_registry::OperationRegistry,
    part::{Part, StepContext},
    program::{Program, ProgramStep},
    reader::Reader,
    runner::Runner,
    target::{Target, TargetReference},
    values::Design,
};
use serde::{Deserialize, Serialize};

use crate::{
    edit_form::{EditForm, FormState},
    editable::{Context, Editable, Editors, OperationGroup, OperationInfo, OperationTier},
    editing::{Control, Dialog, Event, Form, Number, Unit, Value, is_formula},
    part_operation,
    part_operation::PartOperation,
    selectable::{Selectable, SelectableElement, selected},
};

/// What a test operation is to an editor: only its name matters.
const INFO: OperationInfo = OperationInfo::new("", "", OperationGroup::Solid, OperationTier::Menu);

fn v(x: f64, y: f64, z: f64) -> Vector3<Design> {
    Vector3::from_array([x, y, z].map(Design::from_f64))
}

/// A point, read whole: a kind of its own.
#[derive(Clone, Debug, Default, PartialEq)]
struct Point(Vector3<Design>);

/// Given by one step, shifted by the ones after it.
impl Target for Point {
    const CHAIN: bool = true;
}

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
    type Args = Vector3<Design>;

    fn run(ctx: &mut StepContext, at: &Vector3<Design>) -> GeopResult<()> {
        let step = ctx.step();
        ctx.produce(&TargetReference::new(step), at, |_, at| Ok(Point(*at)))
    }
}

impl Editable for GivenPoint {
    const INFO: OperationInfo = INFO;
    type EditingState = ();

    fn new_args(_: &Context) -> Vector3<Design> {
        Vector3::zero()
    }

    fn presentation(_: &Context, _: &Vector3<Design>, _: &()) -> Form<Design> {
        Form::default()
    }

    fn process_user_event(
        _: &Context,
        at: &Vector3<Design>,
        _: &(),
        _: Event<Design>,
    ) -> (Vector3<Design>, ()) {
        (*at, ())
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
struct ShiftArgs {
    point: String,
    by: Vector3<Design>,
}

/// Moves the point target `point` by `by`.
struct Shift;

impl Operation for Shift {
    const KIND: &'static str = "shift";
    type Args = ShiftArgs;

    fn run(ctx: &mut StepContext, args: &ShiftArgs) -> GeopResult<()> {
        let point = TargetReference::<Point>::new(&args.point);
        ctx.produce(&point, &args.by, |reader, by| {
            Ok(Point(reader.previous::<Point>()?.0.add(by)))
        })
    }
}

impl Editable for Shift {
    const INFO: OperationInfo = INFO;
    type EditingState = ();

    fn new_args(_: &Context) -> ShiftArgs {
        ShiftArgs::default()
    }

    fn presentation(_: &Context, _: &ShiftArgs, _: &()) -> Form<Design> {
        Form::default()
    }

    /// Pressing the field `origin` moves the point to the origin, as the
    /// step sees it; pressing `built` shifts it by where the step left it.
    fn process_user_event(
        context: &Context,
        args: &ShiftArgs,
        _: &(),
        event: Event<Design>,
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

fn editors() -> Editors {
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
    editors: &Editors,
    runner: &Runner,
    part: &Part,
    program: &Program,
    step: &str,
) -> GeopResult<f64> {
    press(editors, runner, part, program, step, "origin")
}

/// What a press of the field `key` makes the shift `step` of `program`, as
/// last run on `part`.
fn press(
    editors: &Editors,
    runner: &Runner,
    part: &Part,
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
    let runner = Runner::new(editors.operations().unwrap());
    let mut part = Part::default();
    let program = Program::from_steps(vec![
        given("a", 1.0),
        shift("s1", "a", 1.0),
        shift("s2", "a", 5.0),
    ]);
    runner.run(&mut part, &program, None);
    for (step, by) in [("s2", -2.0), ("s1", -1.0), ("s2", -2.0)] {
        assert_eq!(
            pressed(&editors, &runner, &part, &program, step).unwrap(),
            by
        );
    }
}

/// A new step starts from its operation's defaults, inserted before a step
/// of the last run or after its last; its form is the operation's.
#[test]
fn a_new_step_starts_from_its_operation() {
    let editors = editors();
    let runner = Runner::new(editors.operations().unwrap());
    let mut part = Part::default();
    let program = Program::from_steps(vec![given("a", 1.0)]);
    runner.run(&mut part, &program, None);

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
    let runner = Runner::new(editors.operations().unwrap());
    let mut part = Part::default();
    let program = Program::from_steps(vec![
        given("a", 1.0),
        shift("s1", "a", 1.0),
        shift("s2", "a", 5.0),
    ]);
    runner.run(&mut part, &program, None);
    assert_eq!(
        press(&editors, &runner, &part, &program, "s1", "built").unwrap(),
        2.0
    );
    assert_eq!(
        press(&editors, &runner, &part, &program, "s2", "built").unwrap(),
        7.0
    );

    let edited = Program::from_steps(vec![
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

impl Target for Chains {}

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

    fn select(reader: &Reader, hit: &PickedElement) -> Option<ElementReference<Chains, Chain>> {
        let target = TargetReference::<Chains>::new(&hit.target);
        let chains = reader
            .retrieve_element(&ElementReference::<Chains, Chains>::whole(target.clone()))
            .ok()?;
        let (chain, _) = (chains.0.iter()).find(|(_, edges)| edges.contains(&hit.name))?;
        Some(ElementReference::new(target, chain))
    }

    fn drawn(reader: &Reader, chain: &ElementReference<Chains, Chain>) -> Vec<PickedElement> {
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
fn chains() -> Part {
    let mut part = Part::default();
    let chains = Chains(vec![
        ("c1".into(), vec!["e1".into(), "e2".into()]),
        ("c2".into(), vec!["e3".into()]),
    ]);
    let runner = Runner::new(OperationRegistry::default());
    (runner.run_step(&mut part, "given", |ctx| {
        ctx.produce(&TargetReference::new("chains"), &chains, |_, c| {
            Ok(c.clone())
        })
    }))
    .unwrap();
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
    let mut dialog = Dialog::<Design>::new();
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
    assert_eq!(
        selected::<ElementReference<Chains, Chain>>(&reader, &picks),
        []
    );
    let picks = [edge("e9")];
    assert_eq!(
        selected::<ElementReference<Chains, Chain>>(&reader, &picks),
        []
    );
}

/// Picked whole, from a list of what the part has.
impl SelectableElement<Chains> for Chains {
    const PICKED_FROM: &'static [&'static str] = &["chains"];

    fn select(_: &Reader, hit: &PickedElement) -> Option<ElementReference<Chains, Chains>> {
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
        Names(
            (chains.0.iter())
                .flat_map(|(_, edges)| edges.clone())
                .collect(),
        )
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
    type Session = ();

    fn new_args(_: &Context) -> ScaleArgs {
        ScaleArgs::default()
    }

    fn run(ctx: &mut StepContext, args: &ScaleArgs) -> GeopResult<()> {
        let step = ctx.step();
        let scaled = TargetReference::<f64>::new(step);
        ctx.produce(&scaled, &args.by, |reader, by| by.resolve(reader))
    }

    fn form<'a>(
        context: &'a Context<'a>,
        args: &ScaleArgs,
        _: &FormState<()>,
    ) -> EditForm<'a, Design, ScaleArgs> {
        let mut f = EditForm::<Design, ScaleArgs>::default();
        let reader = &context.before;
        f.pick(
            "chains",
            "chains",
            reader,
            &args.chains,
            true,
            |edit, chains| edit.args.chains = chains,
        );
        let by = Number::numeric(&args.by, "by", reader, Unit::Length);
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
    let mut part = Part::default();
    let runner = Runner::new(OperationRegistry::default());
    let chains = Chains(vec![("c1".into(), vec!["e1".into(), "e2".into()])]);
    (runner.run_step(&mut part, "given", |ctx| {
        ctx.produce(&TargetReference::new("chains"), &chains, |_, c| {
            Ok(c.clone())
        })?;
        ctx.produce(&parameter("width"), &6.0, |_, &width| {
            Ok(ParameterValue::Number(width))
        })
    }))
    .unwrap();
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

/// What a number field holds is a formula unless it reads as a number.
#[test]
fn a_number_field_holds_a_formula_unless_it_reads_as_a_number() {
    assert!(!is_formula(" 12.5 ") && is_formula("width"));
}
