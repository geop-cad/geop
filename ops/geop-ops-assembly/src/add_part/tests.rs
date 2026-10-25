use geop_core_editor::{
    edit_form::FormState,
    editable::{Context, Editable},
    editing::{CanvasEvent, Event, Pointer, Pull, Reach},
    editor::Rerun,
};
use geop_core_program::{
    element::ElementReference,
    operation_registry::OperationRegistry,
    part::Part,
    placed::key_behind,
    program::{Program, ProgramStep},
    reader::Reader,
    runner::{RunOutput, Runner},
    values::{Design, StateValue, StateValues, pose_state},
    workspace::Location,
};
use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::view::{PartView, drawn};
use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Pose, Ray},
    scalars::{ScalInF64 as S, Scalar},
    solvers::system::Variable,
    vector::Vector3,
};
use geop_lib_sweep::shapes::{block_part, v3};
use geop_lib_constraints::Constraints;
use geop_ops_datums::{ORIGIN, component_name, datums, origin_axis, read_datums};
use geop_ops_model::{read_model, testing::assert_close};
use geop_ops_pattern::spacing::Spacing;
use geop_ops_reference_geometry::Selected;

use super::*;
use crate::{
    Kind, MateKind, PatternAxis,
    editor::PART,
    placed::{locate, placed_parts},
    solving::{self, check as check_mates, solve, solved_states},
    testing::{Shelf, given_model},
};

/// The part `reader` reads, drawn: its model, its datums and the parts
/// placed in it — what a component is drawn as where it is placed.
fn draw(reader: &Reader) -> GeopResult<PartView> {
    let mut view = PartView::new();
    read_model(reader)?.draw("model", &mut view)?;
    read_datums(reader)?.draw(&mut view)?;
    crate::placed::draw_instances(reader, &mut view)?;
    Ok(view.finish())
}

/// The operations the shelf's programs are written in: those of
/// [`crate::testing`], placing parts and patterns of them.
fn registry() -> OperationRegistry {
    (crate::testing::registry().with(AddPart))
        .and_then(|r| r.with(crate::PartPattern))
        .unwrap()
}

/// A shelf of `cube.geop` alone: the unit cube from the origin to
/// `(1, 1, 1)`, with the origin's datums.
fn shelf() -> Shelf {
    let mut cube = Program::new();
    let model = block_part("c", [0.0; 3], [1.0; 3]);
    cube.steps.push(given_model("model", model, None));
    Shelf::default().with("cube.geop", cube)
}

fn args(fixed: bool, mates: &[(&str, MateKind, [MateEntity; 2])]) -> AddPartArgs {
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

/// The datum `name` of the part placed as `instance`.
fn datum(instance: &str, name: String) -> MateEntity {
    MateEntity::of(
        instance,
        Selected::Datum(ElementReference::new(datums(), name)),
    )
}

/// The frame of the part placed as `instance`.
fn origin(instance: &str) -> MateEntity {
    datum(instance, ORIGIN.to_string())
}

/// The `z` axis of the frame of the part placed as `instance`.
fn axis(instance: &str) -> MateEntity {
    datum(
        instance,
        component_name(ORIGIN, DatumComponent::Axis(FrameAxis::Z)),
    )
}

/// The `xy` plane of the frame of the part placed as `instance`.
fn base(instance: &str) -> MateEntity {
    datum(
        instance,
        component_name(ORIGIN, DatumComponent::Plane(FrameAxis::Z)),
    )
}

/// How near the solver brings mates to holding: a billionth of the
/// assembly's size (see `geop_core_math::solvers::system`), and the cube is
/// 1 across.
const SOLVED: f64 = 1e-8;

/// A pointer looking straight down at `at`.
fn down_at(at: Vector3<S>) -> Pointer<S> {
    Pointer {
        ray: Ray::try_new(at.add(&v3(0.0, 0.0, 10.0)), v3(0.0, 0.0, -1.0)).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.01),
        },
    }
}

/// The pose `moved` gives the part placed as `instance`.
fn moved_to(moved: &StateValues, instance: &str) -> Pose<Design> {
    match &moved[&pose_state(instance)] {
        StateValue::Pose(pose) => *pose,
        other => panic!("a pose, not {other:?}"),
    }
}

/// A program of steps placing the cube, run with a state.
struct Assembly {
    runner: Runner,
    part: Part,
    program: Program,
    inputs: Settings,
    state: StateValues,
}

impl Assembly {
    /// The program `steps` of the file `assembly.geop`, placing parts
    /// from `shelf`, run with the state `state`.
    fn with(shelf: Shelf, steps: Vec<ProgramStep>, state: StateValues) -> Self {
        let mut runner = Runner::new(registry());
        runner.workspace = Some(Rc::new(shelf));
        runner.location = Location {
            file: "assembly.geop".into(),
            within: Vec::new(),
        };
        let mut assembly = Assembly {
            runner,
            part: Part::default(),
            program: Program::from_steps(steps),
            inputs: BTreeMap::new(),
            state,
        };
        assembly.run();
        assembly
    }

    /// The steps `steps`, each placing a part of the shelf's as its id,
    /// run with the state `state`.
    fn new(steps: &[(&str, AddPartArgs)], state: StateValues) -> Self {
        let steps = (steps.iter())
            .map(|(id, args)| ProgramStep::of(*id, args).unwrap())
            .collect();
        Self::with(shelf(), steps, state)
    }

    /// The cube placed as `a`, fixed at the origin, then `steps`.
    fn on_a(steps: &[(&str, AddPartArgs)], state: StateValues) -> Self {
        let mut all = vec![("a", args(true, &[]))];
        all.extend(steps.iter().cloned());
        Self::new(&all, state)
    }

    fn run(&mut self) -> RunOutput {
        self.program.state = self.state.clone();
        (self.runner).run_with(&mut self.part, &self.program, &self.inputs, None)
    }

    fn reader(&self) -> Reader<'_> {
        self.part.current()
    }

    /// The program as a solve runs it again.
    fn rerun(&mut self) -> Rerun<'_> {
        Rerun {
            runner: &self.runner,
            part: &mut self.part,
            program: &self.program,
        }
    }

    /// Solves the mates with `pulls`, and runs the program with the state
    /// the solve gives. A solve whose mates hold encloses a true solution
    /// near where it ended.
    fn solve(&mut self, pulls: &[Pull<Design>]) -> solving::Report {
        let state = self.state.clone();
        let (moved, report) = solve(&mut self.rerun(), &state, pulls, None).unwrap();
        if report.converged {
            assert!(report.unproven.is_empty(), "{:?}", report.unproven);
        }
        self.state.extend(moved);
        self.run();
        report
    }

    /// Solves with the values `set` held there, and runs the program with
    /// the state the solve gives.
    fn set(&mut self, set: &BTreeMap<String, Design>) -> (StateValues, solving::Report) {
        let state = self.state.clone();
        let (moved, report) = solving::set(&mut self.rerun(), &state, set).unwrap();
        self.state.extend(moved.clone());
        self.run();
        (moved, report)
    }

    /// Where the entity `entity` of a placed part is, as the run placed it.
    fn point(&self, entity: &MateEntity) -> [f64; 3] {
        let p = entity.shape(&self.reader()).unwrap().point.unwrap();
        let p = match entity.part().is_empty() {
            true => p,
            false => self.world(entity.part()).apply(&p),
        };
        [0, 1, 2].map(|k| p[k].to_f64())
    }

    /// Where the part placed at `path` is in the world, as the run placed
    /// it.
    fn world(&self, path: &str) -> Pose<Design> {
        locate(&self.reader(), path).unwrap().world
    }
}

/// A placed part is where its pose puts it: its entities resolve there, and
/// it is drawn there, named behind its instance's name.
#[test]
fn a_placed_part_is_where_its_pose_puts_it() {
    let pose = pose([2.0, 0.0, 0.0], [0.0, 0.0, 90.0]);
    let state = StateValues::from([(pose_state("p"), StateValue::Pose(pose))]);
    let assembly = Assembly::new(&[("p", args(false, &[]))], state);
    assert_eq!(
        solved_states(&assembly.reader()).unwrap(),
        ["p.pose"],
        "solving moves where the step puts the part"
    );
    assert_close(assembly.point(&origin("p")), [2.0, 0.0, 0.0], 1e-12);

    // Drawn by reference: its component's view, where it is.
    let view = draw(&assembly.reader()).unwrap();
    assert!(view.faces.is_empty());
    let [placed] = &view.instances[..] else {
        panic!("one part is placed: {:?}", view.instances.len());
    };
    assert_eq!(placed.name, "p");
    let component = &placed.view;
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
        .pick(&down_at(v3(1.5, 0.5, 0.0)), &[drawn::FACE.to_string()])
        .unwrap();
    assert_eq!(hit.element.target.to_string(), "/p:model", "{hit:?}");
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
    let state = StateValues::from([(pose_state("b"), StateValue::Pose(at([5.0, 4.0, 3.0])))]);
    let mut assembly = Assembly::on_a(&[("b", mates)], state);
    assert!(!check_mates(&assembly.reader()).unwrap().converged);
    let state = assembly.state.clone();
    let (moved, _) = solve(&mut assembly.rerun(), &state, &[], None).unwrap();
    assert_eq!(moved.keys().collect::<Vec<_>>(), ["b.pose"], "a is fixed");
    assembly.run();
    let report = assembly.solve(&[]);
    assert!(report.converged, "{report:?}");
    assert!(check_mates(&assembly.reader()).unwrap().converged);
    assert_close(assembly.point(&origin("b")), [0.0, 0.0, 1.5], SOLVED);
    // Each mate's constraint on the state, as the run left it, holds.
    for id in ["m1", "m2"] {
        let name = format!("add_part(b,{id})");
        assert!(
            crate::mechanism::mate_holds(&assembly.reader(), &name),
            "{name}"
        );
    }
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
    let mut assembly = Assembly::on_a(&[("b", mates.clone())], StateValues::new());
    let pulls = {
        let context: Context = Context {
            step: "b",
            runner: &assembly.runner,
            before: assembly.part.before("b"),
            built: Some(assembly.part.after("b")),
            scene: None,
        };
        let drag = |editing: &PartEditing, done: bool| {
            let event = CanvasEvent::Move {
                key: PART.into(),
                from: v3(1.0, 0.0, 0.0),
                to: v3(0.0, 1.0, 0.0),
                pointer: down_at(v3(0.0, 1.0, 0.0)),
                done,
                shift: false,
            };
            AddPart::process_user_event(&context, &mates, editing, Event::Canvas(event)).1
        };
        let editing = drag(&FormState::default(), false);
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
        let leave = Event::Canvas(CanvasEvent::Leave);
        let left = AddPart::process_user_event(&context, &mates, &released, leave).1;
        let gone = AddPart::presentation(&context, &mates, &left).pulls;
        assert!(gone.is_empty(), "released: {gone:?}");
        pulls
    };
    let state = assembly.state.clone();
    let (moved, report) = solve(&mut assembly.rerun(), &state, &pulls, None).unwrap();
    assert!(report.converged);
    let corner = apply(&moved_to(&moved, "b"), [1.0, 0.0, 0.0]);
    assert_close(corner, [0.0, 1.0, 0.0], 1e-2);
}

/// Without a file there is nothing to place.
#[test]
fn a_part_needs_a_file() {
    let mut args = args(true, &[]);
    args.file.clear();
    let mut assembly = Assembly::new(&[("a", args)], StateValues::new());
    let output = assembly.run();
    assert_eq!(output.step_errors.len(), 1, "{:?}", output.step_errors);
    assert!(placed_parts(&assembly.reader()).unwrap().is_empty());
}

/// Copies of a placed part go where the pattern puts them after it, and
/// move with it.
#[test]
fn a_pattern_places_copies_after_the_part() {
    let pattern = crate::PartPatternArgs {
        part: "a".into(),
        axis: Some(PatternAxis::Datum(origin_axis(FrameAxis::X))),
        reversed: false,
        layout: crate::Layout::Linear {
            spacing: Spacing::step(2.0),
        },
        count: 3.0.into(),
    };
    let steps = vec![
        ProgramStep::of("a", &args(false, &[])).unwrap(),
        ProgramStep::of("row", &pattern).unwrap(),
    ];
    let state = StateValues::from([(pose_state("a"), StateValue::Pose(at([0.0, 5.0, 0.0])))]);
    let assembly = Assembly::with(shelf(), steps, state);
    let reader = assembly.reader();
    let placed = placed_parts(&reader).unwrap();
    let names: Vec<&str> = placed.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(names, ["a", "part_pattern(row,1)", "part_pattern(row,2)"]);
    for (k, part) in placed.iter().enumerate() {
        assert_close(
            apply(&part.located.world, [0.0; 3]),
            [2.0 * k as f64, 5.0, 0.0],
            1e-12,
        );
    }
    assert_eq!(placed[1].located.state, None, "no mate moves a copy");
}

/// The shelf, with `pair.geop` besides: the program of `pair`, which places
/// cubes, as it was solved.
fn with_pair(pair: Assembly) -> Shelf {
    let pair = Program {
        steps: pair.program.steps,
        state: pair.state,
    };
    shelf().with("pair.geop", pair)
}

/// A part placed with mates of its own between the parts placed in it
/// brings them along, behind its name, with its constraints: solving the
/// part that places it holds them there too. What it holds fixed stays
/// fixed in it, which moves as a whole.
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
    let mut pair = Assembly::on_a(&[("b", mates)], StateValues::new());
    pair.solve(&[]);
    let placing = AddPartArgs {
        file: "pair.geop".into(),
        ..Default::default()
    };
    let steps = vec![ProgramStep::of("p", &placing.fixed()).unwrap()];
    // The pair's second cube pulled off its axis, as the outer program
    // may move it: its own state does not say otherwise.
    let off = StateValues::from([(
        key_behind("p", &pose_state("b")),
        StateValue::Pose(at([5.0, 4.0, 3.0])),
    )]);
    let mut assembly = Assembly::with(with_pair(pair), steps, off);
    let reader = assembly.reader();
    let constraints = Constraints::of(&reader).unwrap();
    let names: Vec<&str> = constraints.iter().map(|(name, _)| name).collect();
    // The pair's, behind its name; its own fixed mate is no constraint.
    assert_eq!(names, ["/p:add_part(b,m1)", "/p:add_part(b,m2)"]);
    assert!(!check_mates(&reader).unwrap().converged);
    let report = assembly.solve(&[]);
    assert!(report.converged, "{report:?}");
    assert!(crate::mechanism::mate_holds(
        &assembly.reader(),
        "/p:add_part(b,m2)"
    ));
    // The pair's first cube stays where the pair has it; the second stands
    // on its axis.
    let (a, b) = (assembly.world("p/a"), assembly.world("p/b"));
    assert_close(
        apply(&a.inverse().compose(&b), [0.0; 3]),
        [0.0, 0.0, 1.5],
        SOLVED,
    );
}

/// The constraint `name` of the part `reader` reads.
fn constraint(reader: &Reader, name: &str) -> geop_lib_constraints::Constraint {
    let constraints = Constraints::of(reader).unwrap();
    (constraints.get(name).cloned()).unwrap_or_else(|| panic!("no constraint {name}"))
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
    let mates = args(false, &[("m1", joint, [origin("a"), origin("b")])]);
    // A pose the state gives — a state it does not give puts the part at
    // the origin, but is nothing a constraint is differentiated by.
    let state = StateValues::from([(pose_state("b"), StateValue::Pose(at([0.0; 3])))]);
    let mut assembly = Assembly::on_a(&[("b", mates)], state);
    assembly.solve(&[]);
    let name = "add_part(b,m1)";
    let angle = format!("{name}.angle");
    let constraint = constraint(&assembly.reader(), name);
    let states: Vec<&str> = constraint.jacobian.keys().map(String::as_str).collect();
    // `a` is fixed, so nothing solves for its pose: the state gives none,
    // the run puts it at the origin, and there is no step of it to
    // differentiate by.
    assert_eq!(states, [angle.as_str(), "b.pose"]);
    for rows in constraint.jacobian.values() {
        assert_eq!(rows.len(), constraint.residuals.len());
    }
    assert!(crate::mechanism::mate_holds(&assembly.reader(), name));

    let (_, report) = assembly.set(&BTreeMap::from([(angle.clone(), n(90.0))]));
    assert!(report.converged, "{report:?}");
    assert_close(
        apply(&assembly.world("b"), [1.0, 0.0, 0.0]),
        [0.0, 1.0, 0.0],
        1e-6,
    );
    assert!(crate::mechanism::mate_holds(&assembly.reader(), name));
}

/// A joint of `kind` between the frames of `a` and `b`, as the mate `id`.
fn jointed(id: &str, kind: crate::JointKind, a: &str, b: &str) -> (String, Mate) {
    let mate = Mate::joint(kind, vec![origin(a), origin(b)]);
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

/// The number the state `moved` gives `state`.
fn number(moved: &StateValues, state: &str) -> f64 {
    match &moved[state] {
        StateValue::Number(v) => v.to_f64(),
        other => panic!("a number, not {other:?}"),
    }
}

/// Asserts every constraint's derivatives, as the run gives them, are the
/// true ones: central differences of its residuals over a step of each
/// state it is differentiated by, the program run again for each.
#[track_caller]
fn assert_derivatives_are_true(assembly: &mut Assembly) {
    let state = assembly.state.clone();
    let all = Constraints::of(&assembly.reader()).unwrap();
    assert!(!all.is_empty());
    let h = 1e-6;
    for (name, constraint) in all.iter() {
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
                    let Assembly {
                        runner,
                        part,
                        program,
                        inputs,
                        ..
                    } = &mut *assembly;
                    program.state = moved;
                    runner.run_with(part, program, inputs, None);
                    let made = self::constraint(&assembly.reader(), name).residuals;
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
    assembly.run();
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
        vec![origin("b"), origin("a")],
    );
    let concentric = Mate::constraint(Kind::Concentric, vec![axis("c"), axis("b")]);
    let state = StateValues::from([
        (
            pose_state("b"),
            StateValue::Pose(pose([3.0, -2.0, 1.0], [20.0, -35.0, 70.0])),
        ),
        (
            pose_state("c"),
            StateValue::Pose(pose([-1.0, 4.0, 2.0], [-50.0, 10.0, 15.0])),
        ),
        ("add_part(c,m2).angle".into(), StateValue::Number(n(30.0))),
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
            [origin("b"), origin("a")],
        )],
    );
    let mut pair = Assembly::on_a(&[("b", mates)], StateValues::new());
    pair.solve(&[]);
    let placing = AddPartArgs {
        file: "pair.geop".into(),
        mates: BTreeMap::from([(
            "m1".to_string(),
            Mate::constraint(Kind::Coincident, vec![origin("q/b"), origin("p/b")]),
        )]),
        ..Default::default()
    };
    let steps = vec![
        ProgramStep::of(
            "p",
            &AddPartArgs {
                file: "pair.geop".into(),
                ..Default::default()
            },
        )
        .unwrap(),
        ProgramStep::of("q", &placing).unwrap(),
    ];
    let state = StateValues::from([
        (
            pose_state("p"),
            StateValue::Pose(pose([1.0, 2.0, 3.0], [10.0, 20.0, 30.0])),
        ),
        (
            pose_state("q"),
            StateValue::Pose(pose([-2.0, 1.0, 0.5], [-40.0, 5.0, 60.0])),
        ),
        (
            key_behind("p", &pose_state("b")),
            StateValue::Pose(pose([0.5, 0.0, 2.0], [0.0, 45.0, 0.0])),
        ),
        (
            key_behind("q", &pose_state("b")),
            StateValue::Pose(pose([0.0, 1.0, 1.0], [30.0, 0.0, -20.0])),
        ),
    ]);
    let mut assembly = Assembly::with(with_pair(pair), steps, state);
    let constraint = constraint(&assembly.reader(), "add_part(q,m1)");
    let states: Vec<&str> = constraint.jacobian.keys().map(String::as_str).collect();
    assert_eq!(states, ["/p:b.pose", "/q:b.pose", "p.pose", "q.pose"]);
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
        StateValues::new(),
    );
    assert!(assembly.solve(&[]).converged);
    let pull = Pull::Point {
        parameter: pose_state("b"),
        local: v3(0.0, 0.0, 0.0),
        target: v3(0.5, 0.0, 5.0),
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
        StateValues::new(),
    );
    assert!(assembly.solve(&[]).converged);
    let set = BTreeMap::from([("add_part(b,m1).angle".to_string(), n(540.0))]);
    let (_, report) = assembly.set(&set);
    assert!(report.converged, "{report:?}");
    assert_eq!(number(&assembly.state, "add_part(b,m1).angle"), 540.0);
    assert!((number(&assembly.state, "add_part(b,m1).distance") - 0.75).abs() < SOLVED);
    // Half a turn past whole ones: its x axis points back.
    assert_close(
        apply(&assembly.world("b"), [1.0, 0.0, 0.0]),
        [-1.0, 0.0, 0.75],
        1e-6,
    );
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
        StateValues::new(),
    );
    assert!(assembly.solve(&[]).converged);
    let set = BTreeMap::from([("add_part(b,m1).angle".to_string(), n(90.0))]);
    let (moved, report) = assembly.set(&set);
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
    let mut assembly = Assembly::on_a(&[("b", b), ("c", c)], StateValues::new());
    assert!(!assembly.solve(&[]).converged);
    let state = assembly.state.clone();
    let conflicting = solving::conflicting(&mut assembly.rerun(), &state).unwrap();
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
        StateValues::new(),
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
