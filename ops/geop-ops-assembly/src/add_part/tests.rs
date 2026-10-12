use std::{
    collections::{BTreeMap, BTreeSet},
    rc::Rc,
};

use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Pose, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{
    operation::Operation,
    operation_registry::OperationRegistry,
    program::{Program, ProgramStep},
    program_runner::{ProgramRunner, RunOutput},
    step_editor::{Editing, StepEditor},
};
use geop_ops::{
    Design, EntityRef, NamedModel, ParamValue,
    model::model,
    operation::{Aspects, BuildView, Input, Role, Seen},
    parameters::{Parameters, Values},
    placed::{Component, LIBRARY, Library, locate, placed_parts},
    ui::{
        CanvasEvent, Control, PartScene, PartView, Pointer, Pull, Reach, StepEditEvent, Tone, Value,
    },
};
use geop_ops_extrude_revolve::shapes::cube_solid;

use super::*;
use crate::{Kind, MateKind, editor::PART};
use geop_ops::solving::{self, check as check_mates, solve, solved_parameters};

/// Programs' parts by file name, built beforehand.
struct Shelf(BTreeMap<String, Rc<Component>>);

impl Library for Shelf {
    fn file(&self) -> String {
        "assembly.geop".into()
    }

    fn component(&self, file: &str, _: &Values) -> GeopResult<Rc<Component>> {
        (self.0.get(file).cloned()).ok_or_else(|| GeopError::new(format!("no file {file:?}")))
    }

    fn files(&self) -> Vec<String> {
        self.0.keys().cloned().collect()
    }

    fn read(&self, file: &str) -> GeopResult<(String, String)> {
        Err(GeopError::new(format!("no file {file:?} to read")))
    }
}

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([x, y, z].map(S::from_f64))
}

/// The model `model`, as the generator of a program's one target.
fn given(
    _: &mut geop_ops::operation::BuildReader,
    model: &NamedModel<Design>,
) -> GeopResult<NamedModel<Design>> {
    Ok(model.clone())
}

/// A shelf with `cube.geop`: the unit cube from the origin to `(1, 1, 1)`.
fn shelf() -> Rc<Shelf> {
    let mut cube = NamedModel::new();
    cube_solid(&mut cube, "c", v(0.0, 0.0, 0.0), v(1.0, 1.0, 1.0)).unwrap();
    let mut part = geop_ops::operation::Build::default();
    part.begin_run(&BTreeMap::new(), &BTreeMap::new());
    part.begin_step("cube").unwrap();
    part.define_target(&model(), &cube, given).unwrap();
    part.end_run(Default::default());
    let files = BTreeSet::from(["cube.geop".to_string()]);
    let component = Component::new(
        "cube.geop".into(),
        files,
        "cube.geop#1".into(),
        Parameters::default(),
        part,
    );
    Rc::new(Shelf(BTreeMap::from([(
        "cube.geop".into(),
        Rc::new(component),
    )])))
}

fn args(fixed: bool, mates: &[(&str, MateKind, [EntityRef; 2])]) -> AddPartArgs {
    let args = AddPartArgs {
        file: "cube.geop".into(),
        mates: (mates.iter())
            .map(|(id, kind, entities)| {
                let mate = Mate {
                    kind: *kind,
                    entities: entities.to_vec(),
                    joints: Vec::new(),
                };
                (id.to_string(), mate)
            })
            .collect(),
        ..Default::default()
    };
    if fixed { args.fixed() } else { args }
}

fn n(x: f64) -> Design {
    Design::from_f64(x)
}

fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<Design> {
    Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n)).unwrap()
}

/// Where `pose` puts the point `p`.
fn apply(pose: &Pose<Design>, p: [f64; 3]) -> [f64; 3] {
    let q = pose.apply(&Vector3::from_array(p.map(n)));
    [0, 1, 2].map(|k| q[k].to_f64())
}

fn at(position: [f64; 3]) -> Pose<Design> {
    pose(position, [0.0; 3])
}

/// The frame of the part placed as `instance`, or one of its axes or planes.
fn origin(instance: &str, component: Option<DatumComponent>) -> EntityRef {
    EntityRef::Datum {
        name: format!("{instance}/origin"),
        component,
    }
}

fn axis(instance: &str) -> EntityRef {
    origin(instance, Some(DatumComponent::Axis(FrameAxis::Z)))
}

fn base(instance: &str) -> EntityRef {
    origin(instance, Some(DatumComponent::Plane(FrameAxis::Z)))
}

/// Asserts `a` is within `tol` of `b` in every coordinate.
#[track_caller]
fn assert_close(a: [f64; 3], b: [f64; 3], tol: f64) {
    assert!(
        (0..3).all(|k| (a[k] - b[k]).abs() < tol),
        "{a:?} is not within {tol} of {b:?}"
    );
}

/// How near the solver brings mates to holding: a billionth of the
/// assembly's size (see `geop_core_math::solvers::system`), and the cube is
/// 1 across.
const SOLVED: f64 = 1e-8;

/// A pointer looking straight down at `at`.
fn down_at(at: Vector3<S>) -> Pointer<S> {
    Pointer {
        ray: Ray::try_new(at.add(&v(0.0, 0.0, 10.0)), v(0.0, 0.0, -1.0)).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.01),
        },
    }
}

/// The pose `moved` gives the part placed as `instance`.
fn moved_to(moved: &Values, instance: &str) -> Pose<Design> {
    match &moved[&pose_parameter(instance)] {
        ParamValue::Pose(pose) => *pose,
        other => panic!("a pose, not {other:?}"),
    }
}

/// A program of steps placing the cube, run with a state.
struct Assembly {
    runner: ProgramRunner<Input, ParamValue>,
    program: Program,
    inputs: BTreeMap<String, Input>,
    state: Values,
}

impl Assembly {
    /// The program `steps`, placing parts from `library`, run with the
    /// state `state`.
    fn with(library: Rc<dyn Library>, steps: Vec<ProgramStep>, state: Values) -> Self {
        let registry = (OperationRegistry::default().with(AddPart))
            .and_then(|r| r.with(crate::PartPattern))
            .unwrap();
        let mut assembly = Assembly {
            runner: ProgramRunner::new(registry),
            program: Program::new(steps),
            inputs: BTreeMap::from([(LIBRARY.to_string(), Input::Library(library))]),
            state,
        };
        assembly.run();
        assembly
    }

    /// The steps `steps`, each placing a part of the shelf's as its id,
    /// run with the state `state`.
    fn new(steps: &[(&str, AddPartArgs)], state: Values) -> Self {
        let steps = (steps.iter())
            .map(|(id, args)| ProgramStep::new::<AddPart>(*id, args).unwrap())
            .collect();
        Self::with(shelf(), steps, state)
    }

    /// The cube placed as `a`, fixed at the origin, then `steps`.
    fn on_a(steps: &[(&str, AddPartArgs)], state: Values) -> Self {
        let mut all = vec![("a", args(true, &[]))];
        all.extend(steps.iter().cloned());
        Self::new(&all, state)
    }

    fn run(&mut self) -> RunOutput {
        (self.runner).run(&self.program, &self.inputs, &self.state)
    }

    fn view(&self) -> BuildView<'_> {
        self.runner.part.current()
    }

    /// Solves the mates with `pulls`, and runs the program with the state
    /// the solve gives. A solve whose mates hold encloses a true solution
    /// near where it ended.
    fn solve(&mut self, pulls: &[Pull<Design>]) -> solving::Report {
        let (moved, report) = solve(&mut self.runner.part, &self.state, pulls, None).unwrap();
        if report.converged {
            assert!(report.unproven.is_empty(), "{:?}", report.unproven);
        }
        self.state.extend(moved);
        self.run();
        report
    }

    /// Where the entity `entity` of the part placed as `instance` is, as
    /// the run placed it.
    fn point(&self, instance: &str, entity: &EntityRef) -> [f64; 3] {
        let located = locate(&mut self.view(), instance).unwrap();
        let (_, local) = entity.split_instance().unwrap();
        let mut seen = Seen::new(located.instance.component.view());
        let aspects = Aspects::of(&local, &mut seen)
            .unwrap()
            .placed(&located.world)
            .unwrap();
        let p = aspects.point.unwrap();
        [0, 1, 2].map(|k| p[k].to_f64())
    }
}

/// A placed part is where its pose puts it: its entities resolve there, and
/// it is drawn there, named behind its instance's name.
#[test]
fn a_placed_part_is_where_its_pose_puts_it() {
    let pose = pose([2.0, 0.0, 0.0], [0.0, 0.0, 90.0]);
    let state = Values::from([(pose_parameter("p"), ParamValue::Pose(pose))]);
    let assembly = Assembly::new(&[("p", args(false, &[]))], state);
    assert_eq!(
        solved_parameters(assembly.view()).unwrap(),
        ["p.pose"],
        "solving moves where the step puts the part"
    );
    assert_close(
        assembly.point("p", &origin("p", None)),
        [2.0, 0.0, 0.0],
        1e-12,
    );

    // Drawn by reference: its component's view, where it is.
    let view = PartView::of(&Seen::new(assembly.view())).unwrap();
    assert!(view.faces.is_empty());
    let [placed] = &view.instances[..] else {
        panic!("one part is placed: {:?}", view.instances);
    };
    assert_eq!(placed.name, "p");
    let component = placed.view();
    assert!(!component.faces.is_empty());
    // Turned a quarter about z, the cube spans x in [1, 2] and y in [0, 1].
    for p in (component.faces.iter()).flat_map(|f| f.triangles.iter().flatten()) {
        let p = placed.frame.to_xyz(p);
        let [x, y] = [p[0].to_f64(), p[1].to_f64()];
        assert!((1.0 - 1e-9..=2.0 + 1e-9).contains(&x), "{x}");
        assert!((-1e-9..=1.0 + 1e-9).contains(&y), "{y}");
    }
    // And picked where it is, named behind its instance's name.
    let hit = view
        .pick(&down_at(v(1.5, 0.5, 0.0)), &[Role::Plane], None)
        .unwrap();
    assert!(
        matches!(&hit.entity, EntityRef::Face { name } if name.starts_with("p/")),
        "{hit:?}"
    );
    assert!((hit.point[2].to_f64() - 1.0).abs() < 1e-9, "{hit:?}");
}

/// The steps place, they do not solve: solving their mates gives where the
/// part placed has to be — on the axis of the part before, at a distance
/// from its base, on the side it was put — and the part before, fixed,
/// stays.
#[test]
fn solving_the_mates_moves_a_part_onto_the_part_before() {
    let mates = args(
        false,
        &[
            (
                "m1",
                MateKind::Constraint(Kind::Concentric),
                [axis("b"), axis("a")],
            ),
            (
                "m2",
                MateKind::Constraint(Kind::Distance { value: n(1.5) }),
                [base("b"), base("a")],
            ),
        ],
    );
    let state = Values::from([(pose_parameter("b"), ParamValue::Pose(at([5.0, 4.0, 3.0])))]);
    let mut assembly = Assembly::on_a(&[("b", mates)], state);
    assert!(!check_mates(assembly.view()).unwrap().converged);
    let (moved, _) = solve(&mut assembly.runner.part, &assembly.state, &[], None).unwrap();
    assert_eq!(moved.keys().collect::<Vec<_>>(), ["b.pose"], "a is fixed");
    let report = assembly.solve(&[]);
    assert!(report.converged, "{report:?}");
    assert!(check_mates(assembly.view()).unwrap().converged);
    assert_close(
        assembly.point("b", &origin("b", None)),
        [0.0, 0.0, 1.5],
        SOLVED,
    );
    // Each mate's constraint on the state, as the run left it, holds.
    for id in ["m1", "m2"] {
        let name = format!("add_part(b,{id})");
        assert!(
            crate::mechanism::mate_holds(assembly.view(), &name),
            "{name}"
        );
    }
}

/// A step placing the cube as `b`, edited as an editor does: every event
/// handled, the mates solved with what the step pulls, and the program run
/// again.
struct Editor {
    assembly: Assembly,
    editor: StepEditor<S>,
}

impl Editor {
    fn new(assembly: Assembly) -> Self {
        let editor = StepEditor::new(&assembly.runner, &assembly.program, "b", false).unwrap();
        Self { assembly, editor }
    }

    /// The part as the step built it, which picks are made in.
    fn send(&mut self, event: StepEditEvent<S>) {
        let built = self.assembly.runner.part.view_after("b").unwrap();
        let view = PartView::of(&Seen::new(built)).unwrap();
        let scene = PartScene {
            view: &view,
            seen: Seen::new(built),
        };
        let editing = Editing {
            runner: &self.assembly.runner,
            program: &mut self.assembly.program,
            scene: &scene,
        };
        self.editor.handle(editing, &event).unwrap();
        self.assembly.run();
        let pulls = self.form().pulls;
        self.assembly.solve(&pulls);
    }

    fn dialog(&mut self, key: &str, value: Value) {
        self.send(StepEditEvent::Dialog {
            key: key.into(),
            value,
        });
    }

    fn form(&self) -> geop_ops::ui::Form<S> {
        (self.editor)
            .form(&self.assembly.runner, &self.assembly.program)
            .unwrap()
    }

    fn presentation(&self) -> geop_ops::ui::Presentation<S> {
        let built = self.assembly.runner.part.view_after("b").unwrap();
        let view = PartView::of(&Seen::new(built)).unwrap();
        let scene = PartScene {
            view: &view,
            seen: Seen::new(built),
        };
        (self.editor)
            .presentation(&self.assembly.runner, &self.assembly.program, &scene)
            .unwrap()
    }

    fn args(&self) -> AddPartArgs {
        let step = &self.assembly.program.steps[1];
        serde_json::from_value(step.args.clone()).unwrap()
    }
}

/// Adding a mate in the editor arms its entities to be picked; once it has
/// both, picking is done.
#[test]
fn adding_a_mate_arms_its_entities() {
    let mut editor = Editor::new(Assembly::on_a(&[("b", args(false, &[]))], Values::new()));
    editor.dialog("add_mate", Value::Choice("coincident".into()));
    let presentation = editor.presentation();
    let Some(Control::Reference(entities)) = presentation.dialog.get("mate:m1:entities") else {
        panic!("the new mate's entities are a reference field");
    };
    assert!(entities.armed, "a new mate waits for its entities");

    let pair = vec![origin("b", None), origin("a", None)];
    editor.dialog("mate:m1:entities", Value::Entities(pair.clone()));
    assert!(
        editor
            .presentation()
            .dialog
            .get("mate:m1:entities")
            .is_none(),
        "done picking"
    );
    assert_eq!(editor.args().mates["m1"].entities, pair);
}

/// Typing where the part goes pulls it there, turned as typed.
#[test]
fn the_placement_fields_pull_the_part_there() {
    let mut editor = Editor::new(Assembly::on_a(&[("b", args(false, &[]))], Values::new()));
    editor.dialog("x", Value::Number(2.0));
    editor.dialog("rotation_z", Value::Number(90.0));
    let pose = moved_to(&editor.assembly.state, "b");
    assert_close(apply(&pose, [0.0; 3]), [2.0, 0.0, 0.0], 1e-6);
    assert_close(apply(&pose, [1.0, 0.0, 0.0]), [2.0, 1.0, 0.0], 1e-6);
}

/// Dragging the part asks for the point grabbed to be pulled where the
/// pointer is — the editor solves it — and solving with that pull turns a
/// part on an axis about the axis.
#[test]
fn a_drag_pulls_the_point_grabbed() {
    let mates = args(
        false,
        &[
            (
                "m1",
                MateKind::Constraint(Kind::Concentric),
                [axis("b"), axis("a")],
            ),
            (
                "m2",
                MateKind::Constraint(Kind::Coincident),
                [base("b"), base("a")],
            ),
        ],
    );
    let mut assembly = Assembly::on_a(&[("b", mates.clone())], Values::new());
    let context = geop_core_program::operation::Context {
        step: "b",
        before: assembly.runner.part.view_before("b").unwrap(),
        built: assembly.runner.part.view_after("b"),
        scene: None,
    };
    let drag = |editing: &PartEditing, done: bool| {
        let event = CanvasEvent::Move {
            key: PART.into(),
            from: v(1.0, 0.0, 0.0),
            to: v(0.0, 1.0, 0.0),
            pointer: down_at(v(0.0, 1.0, 0.0)),
            done,
            shift: false,
        };
        AddPart::process_user_event(
            &context,
            &mates,
            editing,
            geop_ops::ui::Event::Canvas(event),
        )
        .1
    };
    let editing = drag(&PartEditing::default(), false);
    let pulls = AddPart::presentation(&context, &mates, &editing).pulls;
    let [
        Pull::Point {
            parameter,
            local,
            target,
        },
    ] = pulls.as_slice()
    else {
        panic!("one pull: {pulls:?}");
    };
    assert_eq!(parameter, "b.pose");
    let plain = |p: &Vector3<S>| [0, 1, 2].map(|k| p[k].to_f64());
    assert_close(plain(local), [1.0, 0.0, 0.0], 1e-12);
    assert_close(plain(target), [0.0, 1.0, 0.0], 1e-12);
    // Released, it pulls once more, and is gone at the next event.
    let released = drag(&editing, true);
    assert_eq!(
        AddPart::presentation(&context, &mates, &released).pulls,
        pulls
    );
    let leave = geop_ops::ui::Event::Canvas(CanvasEvent::Leave);
    let left = AddPart::process_user_event(&context, &mates, &released, leave).1;
    let gone = AddPart::presentation(&context, &mates, &left).pulls;
    assert!(gone.is_empty(), "released: {gone:?}");
    let (moved, report) = solve(&mut assembly.runner.part, &assembly.state, &pulls, None).unwrap();
    assert!(report.converged);
    let corner = apply(&moved_to(&moved, "b"), [1.0, 0.0, 0.0]);
    assert_close(corner, [0.0, 1.0, 0.0], 1e-2);
}

/// A fixed part goes where it is dragged: no mate moves it, so the solve
/// moves it alone.
#[test]
fn a_fixed_part_goes_where_it_is_dragged() {
    let mut editor = Editor::new(Assembly::on_a(&[("b", args(true, &[]))], Values::new()));
    editor.send(StepEditEvent::Hover {
        pointer: down_at(v(0.5, 0.5, 1.0)),
        shift: false,
    });
    editor.send(StepEditEvent::Drag {
        from: down_at(v(0.5, 0.5, 1.0)),
        to: down_at(v(2.5, 0.0, 1.0)),
        done: true,
        shift: false,
    });
    let pose = moved_to(&editor.assembly.state, "b");
    assert_close(apply(&pose, [0.0; 3]), [2.0, -0.5, 0.0], 1e-6);
}

/// Mates that cannot all hold are listed as such.
#[test]
fn mates_that_cannot_hold_are_said_so() {
    let mates = args(
        false,
        &[
            (
                "m1",
                MateKind::Constraint(Kind::Distance { value: n(1.0) }),
                [base("b"), base("a")],
            ),
            (
                "m2",
                MateKind::Constraint(Kind::Distance { value: n(2.0) }),
                [base("b"), base("a")],
            ),
        ],
    );
    let mut editor = Editor::new(Assembly::on_a(&[("b", mates)], Values::new()));
    editor.assembly.solve(&[]);
    let form = editor.form();
    let Some(Control::List { items, .. }) = form.dialog.get("mates") else {
        panic!("the mates are a list");
    };
    assert!(items.iter().any(|i| i.tone == Tone::Error), "{items:?}");
}

/// Without a file there is nothing to place.
#[test]
fn a_part_needs_a_file() {
    let mut args = args(true, &[]);
    args.file.clear();
    let mut assembly = Assembly::new(&[("a", args)], Values::new());
    let output = assembly.run();
    assert_eq!(output.step_errors.len(), 1, "{:?}", output.step_errors);
    assert!(placed_parts(assembly.view()).unwrap().is_empty());
}

/// Copies of a placed part go where the pattern puts them after it, and
/// move with it.
#[test]
fn a_pattern_places_copies_after_the_part() {
    let pattern = crate::PartPatternArgs {
        part: "a".into(),
        axis: Some(EntityRef::datum_component(
            geop_ops::ORIGIN,
            DatumComponent::Axis(FrameAxis::X),
        )),
        reversed: false,
        layout: crate::Layout::Linear {
            spacing: geop_ops::operation::pattern::Spacing::step(2.0),
        },
        count: 3.0.into(),
    };
    let steps = vec![
        ProgramStep::new::<AddPart>("a", &args(false, &[])).unwrap(),
        ProgramStep::new::<crate::PartPattern>("row", &pattern).unwrap(),
    ];
    let state = Values::from([(pose_parameter("a"), ParamValue::Pose(at([0.0, 5.0, 0.0])))]);
    let assembly = Assembly::with(shelf(), steps, state);
    let placed = placed_parts(assembly.view()).unwrap();
    let names: Vec<&str> = placed.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["a", "part_pattern(row,1)", "part_pattern(row,2)"]);
    for (k, part) in placed.iter().enumerate() {
        assert_close(
            apply(&part.located.world, [0.0; 3]),
            [2.0 * k as f64, 5.0, 0.0],
            1e-12,
        );
    }
    assert_eq!(placed[1].located.parameter, None, "no mate moves a copy");
}

/// A part placed with mates of its own between the parts placed in it
/// brings them along, behind its name: solving the part that places it
/// holds them there too.
#[test]
fn a_placed_part_brings_its_mates() {
    let mates = args(
        false,
        &[
            (
                "m1",
                MateKind::Constraint(Kind::Concentric),
                [axis("b"), axis("a")],
            ),
            (
                "m2",
                MateKind::Constraint(Kind::Distance { value: n(1.5) }),
                [base("b"), base("a")],
            ),
        ],
    );
    let mut pair = Assembly::on_a(&[("b", mates)], Values::new());
    pair.solve(&[]);
    let mut shelf = Rc::try_unwrap(shelf()).ok().unwrap();
    let part = std::mem::take(&mut pair.runner.part);
    let files = BTreeSet::from(["pair.geop".to_string(), "cube.geop".to_string()]);
    let component = Component::new(
        "pair.geop".into(),
        files,
        "pair.geop#1".into(),
        Parameters::default(),
        part,
    );
    shelf.0.insert("pair.geop".into(), Rc::new(component));
    let placing = AddPartArgs {
        file: "pair.geop".into(),
        ..Default::default()
    };
    let steps = vec![ProgramStep::new::<AddPart>("p", &placing.fixed()).unwrap()];
    // The pair's second cube pulled off its axis, as the outer program
    // may move it: its own state does not say otherwise.
    let off = Values::from([(pose_parameter("p/b"), ParamValue::Pose(at([5.0, 4.0, 3.0])))]);
    let mut assembly = Assembly::with(Rc::new(shelf), steps, off);
    let names: Vec<&str> = (crate::mechanism::mates_of(assembly.view()).into_iter())
        .map(|(name, _)| name)
        .collect();
    // Its own, and the pair's — but not the pair's fixed one: where a
    // placed part is fixed is for the part placing it to say.
    assert_eq!(
        names,
        ["add_part(p,fixed)", "p/add_part(b,m1)", "p/add_part(b,m2)"]
    );
    assert!(!check_mates(assembly.view()).unwrap().converged);
    let report = assembly.solve(&[]);
    assert!(report.converged, "{report:?}");
    assert!(crate::mechanism::mate_holds(
        assembly.view(),
        "p/add_part(b,m2)"
    ));
    // The pair's first cube is free here — its fixed mate stayed behind —
    // so the solve may move either; the second stands on the first's axis.
    let a = locate(&mut assembly.view(), "p/a").unwrap().world;
    let b = locate(&mut assembly.view(), "p/b").unwrap().world;
    assert_close(
        apply(&a.inverse().compose(&b), [0.0; 3]),
        [0.0, 0.0, 1.5],
        SOLVED,
    );
}

/// A joint's constraint on the state reads the poses of the parts it joins
/// and its coordinate, and gives its derivatives by each; setting the
/// coordinate turns the joint there.
#[test]
fn a_joint_turns_to_the_coordinate_set() {
    let joint = MateKind::Joint(crate::JointKind::Revolute {
        min: None,
        max: None,
    });
    let mates = args(
        false,
        &[("m1", joint, [origin("a", None), origin("b", None)])],
    );
    // A pose the state gives — a state it does not give puts the part at
    // the origin, but is nothing a constraint is differentiated by.
    let state = Values::from([(pose_parameter("b"), ParamValue::Pose(at([0.0; 3])))]);
    let mut assembly = Assembly::on_a(&[("b", mates)], state);
    assembly.solve(&[]);
    let name = "add_part(b,m1)";
    let angle = format!("{name}.angle");
    let reference = geop_core_program::target::TargetReference::<
        geop_core_program::constraint::Constraint<ParamValue>,
    >::new(name);
    let constraint = assembly.view().retrieve_target(&reference).unwrap();
    let states: Vec<&str> = constraint.jacobian.keys().map(String::as_str).collect();
    // `a` is fixed, so nothing solves for its pose: the state gives none,
    // the run puts it at the origin, and there is no step of it to
    // differentiate by.
    assert_eq!(states, [angle.as_str(), "b.pose"]);
    for rows in constraint.jacobian.values() {
        assert_eq!(rows.len(), constraint.residuals.len());
    }
    assert!(crate::mechanism::mate_holds(assembly.view(), name));

    let set = BTreeMap::from([(angle.clone(), n(90.0))]);
    let (moved, report) = solving::set(&mut assembly.runner.part, &assembly.state, &set).unwrap();
    assert!(report.converged, "{report:?}");
    assembly.state.extend(moved);
    assembly.run();
    let b = locate(&mut assembly.view(), "b").unwrap();
    assert_close(apply(&b.world, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], 1e-6);
    assert!(crate::mechanism::mate_holds(assembly.view(), name));
}

/// A joint of `kind` between the frames of `a` and `b`, as the mate `id`.
fn jointed(id: &str, kind: crate::JointKind, a: &str, b: &str) -> (String, Mate) {
    let mate = Mate::joint(kind, vec![origin(a, None), origin(b, None)]);
    (id.to_string(), mate)
}

/// A part placed from the cube with `mates` and nothing fixed.
fn mated(mates: Vec<(String, Mate)>) -> AddPartArgs {
    AddPartArgs {
        file: "cube.geop".into(),
        mates: mates.into_iter().collect(),
        ..Default::default()
    }
}

/// The number the state `moved`, or else `state`, gives `parameter`.
fn number(moved: &Values, parameter: &str) -> f64 {
    match &moved[parameter] {
        ParamValue::Number(v) => v.to_f64(),
        other => panic!("a number, not {other:?}"),
    }
}

/// Asserts every constraint's derivatives, as the run gives them, are the
/// true ones: central differences of its residuals over a step of each
/// state it is differentiated by, the part brought up to date for each.
#[track_caller]
fn assert_derivatives_are_true(assembly: &mut Assembly) {
    use geop_core_math::solvers::system::Variable;
    let state = assembly.state.clone();
    let names: Vec<String> = assembly
        .runner
        .part
        .constraints()
        .into_keys()
        .map(String::from)
        .collect();
    assert!(!names.is_empty());
    let h = 1e-6;
    for name in &names {
        let constraint = assembly.runner.part.constraints()[name.as_str()].clone();
        assert!(
            !constraint.jacobian.is_empty(),
            "{name} is differentiated by nothing"
        );
        for (parameter, rows) in &constraint.jacobian {
            let value = &state[parameter];
            for k in 0..value.dimension() {
                let mut residuals_at = |t: f64| {
                    let mut step = vec![Design::ZERO; value.dimension()];
                    step[k] = n(t);
                    let mut moved = state.clone();
                    moved.insert(parameter.clone(), value.retract(&step).unwrap());
                    assembly.runner.part.update(&moved, &[name]);
                    let made = assembly.runner.part.constraints()[name.as_str()]
                        .residuals
                        .clone();
                    made.iter().map(|r| r.to_f64()).collect::<Vec<_>>()
                };
                let (plus, minus) = (residuals_at(h), residuals_at(-h));
                for (r, row) in rows.iter().enumerate() {
                    let difference = (plus[r] - minus[r]) / (2.0 * h);
                    let derivative = row[k].to_f64();
                    assert!(
                        (difference - derivative).abs() < 1e-5 * (1.0 + difference.abs()),
                        "{name}, residual {r}, by {parameter}[{k}]: {derivative} is not {difference}"
                    );
                }
            }
        }
    }
    assembly.runner.part.update(&state, &[]);
}

/// The derivatives a mate's constraint gives — through the exponential map
/// of each part's pose, turned and away from the origin, and by a joint's
/// coordinate — are the true ones.
#[test]
fn mate_derivatives_are_true() {
    let revolute = crate::JointKind::Revolute {
        min: None,
        max: None,
    };
    let distance = Mate::constraint(
        Kind::Distance { value: n(1.5) },
        vec![origin("b", None), origin("a", None)],
    );
    let concentric = Mate::constraint(Kind::Concentric, vec![axis("c"), axis("b")]);
    let state = Values::from([
        (
            pose_parameter("b"),
            ParamValue::Pose(pose([3.0, -2.0, 1.0], [20.0, -35.0, 70.0])),
        ),
        (
            pose_parameter("c"),
            ParamValue::Pose(pose([-1.0, 4.0, 2.0], [-50.0, 10.0, 15.0])),
        ),
        ("add_part(c,m2).angle".into(), ParamValue::Number(n(30.0))),
    ]);
    let mut assembly = Assembly::on_a(
        &[
            ("b", mated(vec![("m1".into(), distance)])),
            (
                "c",
                mated(vec![
                    ("m1".into(), concentric),
                    jointed("m2", revolute, "b", "c"),
                ]),
            ),
        ],
        state,
    );
    assert_derivatives_are_true(&mut assembly);
}

/// Through a part placed in a placed part, a mate is differentiated by the
/// poses of both, and truly.
#[test]
fn nested_mate_derivatives_are_true() {
    let mates = args(
        false,
        &[(
            "m1",
            MateKind::Constraint(Kind::Distance { value: n(1.5) }),
            [origin("b", None), origin("a", None)],
        )],
    );
    let mut pair = Assembly::on_a(&[("b", mates)], Values::new());
    pair.solve(&[]);
    let mut shelf = Rc::try_unwrap(shelf()).ok().unwrap();
    let part = std::mem::take(&mut pair.runner.part);
    let files = BTreeSet::from(["pair.geop".to_string(), "cube.geop".to_string()]);
    let component = Component::new(
        "pair.geop".into(),
        files,
        "pair.geop#1".into(),
        Parameters::default(),
        part,
    );
    shelf.0.insert("pair.geop".into(), Rc::new(component));
    let placing = AddPartArgs {
        file: "pair.geop".into(),
        mates: BTreeMap::from([(
            "m1".to_string(),
            Mate::constraint(
                Kind::Coincident,
                vec![origin("q/b", None), origin("p/b", None)],
            ),
        )]),
        ..Default::default()
    };
    let steps = vec![
        ProgramStep::new::<AddPart>(
            "p",
            &AddPartArgs {
                file: "pair.geop".into(),
                ..Default::default()
            },
        )
        .unwrap(),
        ProgramStep::new::<AddPart>("q", &placing).unwrap(),
    ];
    let state = Values::from([
        (
            pose_parameter("p"),
            ParamValue::Pose(pose([1.0, 2.0, 3.0], [10.0, 20.0, 30.0])),
        ),
        (
            pose_parameter("q"),
            ParamValue::Pose(pose([-2.0, 1.0, 0.5], [-40.0, 5.0, 60.0])),
        ),
        (
            pose_parameter("p/b"),
            ParamValue::Pose(pose([0.5, 0.0, 2.0], [0.0, 45.0, 0.0])),
        ),
        (
            pose_parameter("q/b"),
            ParamValue::Pose(pose([0.0, 1.0, 1.0], [30.0, 0.0, -20.0])),
        ),
    ]);
    let mut assembly = Assembly::with(Rc::new(shelf), steps, state);
    let reference = geop_core_program::target::TargetReference::<
        geop_core_program::constraint::Constraint<ParamValue>,
    >::new("add_part(q,m1)");
    let constraint = assembly.view().retrieve_target(&reference).unwrap();
    let states: Vec<&str> = constraint.jacobian.keys().map(String::as_str).collect();
    assert_eq!(states, ["p.pose", "p/b.pose", "q.pose", "q/b.pose"]);
    assert_derivatives_are_true(&mut assembly);
}

/// A slider moves its part only along its axis, and stops at its limits:
/// dragged far up, it goes as far as its upper limit, and the report says
/// it is there.
#[test]
fn a_slider_stops_at_its_limits() {
    let slider = crate::JointKind::Slider {
        min: Some(n(-1.0)),
        max: Some(n(2.0)),
    };
    let mut assembly = Assembly::on_a(
        &[("b", mated(vec![jointed("m1", slider, "a", "b")]))],
        Values::new(),
    );
    assert!(assembly.solve(&[]).converged);
    let pull = Pull::Point {
        parameter: pose_parameter("b"),
        local: v(0.0, 0.0, 0.0),
        target: v(0.5, 0.0, 5.0),
    };
    let report = assembly.solve(&[pull]);
    assert!(report.converged, "{report:?}");
    assert_eq!(report.at_limit, ["add_part(b,m1).distance"]);
    assert_close(
        apply(&moved_to(&assembly.state, "b"), [0.0; 3]),
        [0.0, 0.0, 2.0],
        SOLVED,
    );
    assert!((number(&assembly.state, "add_part(b,m1).distance") - 2.0).abs() < SOLVED);
}

/// A screw — one cylindrical joint, its turn coupled to its slide — moves
/// its lead per turn: set to a turn and a half, driven there in steps, it
/// has moved one and a half leads along its axis.
#[test]
fn a_screw_advances_its_lead_per_turn() {
    let screw = Mate::coupling(
        crate::CouplingKind {
            transmission: crate::Transmission::Screw,
            value: n(0.5),
            reverse: false,
        },
        vec!["add_part(b,m1)".into(), "add_part(b,m1)".into()],
    );
    let joint = jointed("m1", crate::JointKind::Cylindrical, "a", "b");
    let mut assembly = Assembly::on_a(
        &[("b", mated(vec![joint, ("m2".into(), screw)]))],
        Values::new(),
    );
    assert!(assembly.solve(&[]).converged);
    let set = BTreeMap::from([("add_part(b,m1).angle".to_string(), n(540.0))]);
    let (moved, report) = solving::set(&mut assembly.runner.part, &assembly.state, &set).unwrap();
    assert!(report.converged, "{report:?}");
    assembly.state.extend(moved);
    assembly.run();
    assert_eq!(number(&assembly.state, "add_part(b,m1).angle"), 540.0);
    assert!((number(&assembly.state, "add_part(b,m1).distance") - 0.75).abs() < SOLVED);
    // Half a turn past whole ones: its x axis points back.
    let b = locate(&mut assembly.view(), "b").unwrap().world;
    assert_close(apply(&b, [1.0, 0.0, 0.0]), [-1.0, 0.0, 0.75], 1e-6);
}

/// A rack and pinion: the pinion, of pitch radius 0.5, turned a quarter
/// turn moves the rack a quarter of the pitch circle along its axis.
#[test]
fn a_rack_follows_its_pinion() {
    let revolute = crate::JointKind::Revolute {
        min: None,
        max: None,
    };
    let slider = crate::JointKind::Slider {
        min: None,
        max: None,
    };
    let rack = Mate::coupling(
        crate::CouplingKind {
            transmission: crate::Transmission::RackPinion,
            value: n(0.5),
            reverse: false,
        },
        vec!["add_part(b,m1)".into(), "add_part(c,m1)".into()],
    );
    let mut assembly = Assembly::on_a(
        &[
            ("b", mated(vec![jointed("m1", revolute, "a", "b")])),
            (
                "c",
                mated(vec![jointed("m1", slider, "a", "c"), ("m2".into(), rack)]),
            ),
        ],
        Values::new(),
    );
    assert!(assembly.solve(&[]).converged);
    let set = BTreeMap::from([("add_part(b,m1).angle".to_string(), n(90.0))]);
    let (moved, report) = solving::set(&mut assembly.runner.part, &assembly.state, &set).unwrap();
    assert!(report.converged, "{report:?}");
    let quarter = 0.5 * std::f64::consts::FRAC_PI_2;
    assert!((number(&moved, "add_part(c,m1).distance") - quarter).abs() < SOLVED);
    assert_close(
        apply(&moved_to(&moved, "c"), [0.0; 3]),
        [0.0, 0.0, quarter],
        SOLVED,
    );
}

/// Mates that conflict are named: two distances between the same planes
/// cannot both hold — those two, and not the mate of another part that has
/// nothing to do with them.
#[test]
fn conflicting_mates_are_named() {
    let distance = |value| MateKind::Constraint(Kind::Distance { value: n(value) });
    let b = args(
        false,
        &[
            ("m1", distance(1.0), [base("b"), base("a")]),
            ("m2", distance(2.0), [base("b"), base("a")]),
        ],
    );
    let c = args(false, &[("m1", distance(3.0), [base("c"), base("a")])]);
    let mut assembly = Assembly::on_a(&[("b", b), ("c", c)], Values::new());
    assert!(!assembly.solve(&[]).converged);
    let conflicting = solving::conflicting(&mut assembly.runner.part, &assembly.state).unwrap();
    assert_eq!(conflicting, ["add_part(b,m1)", "add_part(b,m2)"]);
}

/// A gear coupling of a joint that does not turn is refused, saying why.
#[test]
fn a_coupling_of_a_joint_that_does_not_turn_is_refused() {
    let slider = crate::JointKind::Slider {
        min: None,
        max: None,
    };
    let revolute = crate::JointKind::Revolute {
        min: None,
        max: None,
    };
    let gear = Mate::coupling(
        crate::CouplingKind {
            transmission: crate::Transmission::Gear,
            value: n(2.0),
            reverse: false,
        },
        vec!["add_part(b,m1)".into(), "add_part(c,m1)".into()],
    );
    let mut assembly = Assembly::on_a(
        &[
            ("b", mated(vec![jointed("m1", slider, "a", "b")])),
            (
                "c",
                mated(vec![jointed("m1", revolute, "a", "c"), ("m2".into(), gear)]),
            ),
        ],
        Values::new(),
    );
    let output = assembly.run();
    let errors: Vec<String> = output
        .step_errors
        .iter()
        .map(|(_, e)| e.to_string())
        .collect();
    assert!(
        errors
            .iter()
            .any(|e| e.contains("needs its first joint to turn")),
        "{errors:?}"
    );
}
