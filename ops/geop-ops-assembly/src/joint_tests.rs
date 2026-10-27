//! The joints of the table ([`crate::Joint`]), the couplings between their
//! coordinates, and the mates that keep a point on a curve or a face of
//! another part ([`Kind::Tangent`], [`Kind::Path`]): each placed between
//! parts made in Rust — cubes, a drum and a roller — solved, set, dragged,
//! and differentiated truly.

use geop_core_editor::{editing::Pull, editor::Rerun};
use geop_core_math::{
    primitives::{DatumComponent, FrameAxis, Pose},
    scalars::Scalar,
    solvers::system::Variable,
    vector::Vector3,
};
use geop_core_program::{
    element::ElementReference,
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    reader::Reader,
    runner::Runner,
    values::{StateValue, StateValues, pose_state},
    workspace::Location,
};
use geop_lib_constraints::Constraints;
use geop_lib_sweep::shapes::{block_part, cylinder, v3};
use geop_ops_model::{
    NamedModel, PartDescription,
    datums::{ORIGIN, component_name, datums},
    element,
    mate_entity::MateEntity,
    placed::locate,
    selection::Selected,
    testing::assert_close,
};
use std::{collections::BTreeMap, rc::Rc};

use crate::{
    AddPart, AddPartArgs, CouplingKind, Joint, JointKind, Kind, Limits, Mate, Motion, Transmission,
    foot_parameter, joint_parameter,
    solving::{self, Report},
    testing::{Shelf, given_model},
};

/// How near the solver brings mates to holding: a billionth of the
/// assembly's size, and its parts are a few across.
const SOLVED: f64 = 1e-8;

fn n(x: f64) -> Scalar {
    Scalar::from_f64(x)
}

/// The drum: a cylinder of radius 2 and height 1 standing on the origin.
fn drum() -> NamedModel {
    let mut model = NamedModel::new();
    cylinder(&mut model, "drum", [0.0; 3], 2.0, 1.0);
    model
}

/// The roller: a cylinder of radius 0.5 and height 1 standing on the
/// origin.
fn roller() -> NamedModel {
    let mut model = NamedModel::new();
    cylinder(&mut model, "roller", [0.0; 3], 0.5, 1.0);
    model
}

/// The parts the tests place: the unit cube (`cube.geop`), the drum and
/// the roller, each with the origin's datums.
fn shelf() -> Shelf {
    let part = |model| {
        let mut program = Program::new();
        program.steps.push(given_model("model", model, None));
        program
    };
    Shelf::default()
        .with("cube.geop", part(block_part("c", [0.0; 3], [1.0; 3])))
        .with("drum.geop", part(drum()))
        .with("roller.geop", part(roller()))
}

/// A program of parts placed from the shelf, run with a state, to solve.
struct Mechanism {
    runner: Runner,
    part: Part,
    program: Program,
}

impl Mechanism {
    /// The steps `steps`, each placing a part of the shelf's as its id,
    /// run with the state `state`.
    fn new(steps: &[(&str, AddPartArgs)], state: StateValues) -> Self {
        let registry = (crate::testing::registry().with(AddPart))
            .and_then(|r: OperationRegistry| r.with(crate::PartPattern))
            .unwrap();
        let mut runner = Runner::new(registry);
        runner.workspace = Some(Rc::new(shelf()));
        runner.location = Location {
            file: "assembly.geop".into(),
            within: Vec::new(),
        };
        let steps = (steps.iter())
            .map(|(id, args)| ProgramStep::of(*id, args).unwrap())
            .collect();
        let mut program = Program::from_steps(steps);
        program.state = state;
        let mut mechanism = Mechanism {
            runner,
            part: Part::default(),
            program,
        };
        mechanism.run();
        mechanism
    }

    fn run(&mut self) {
        let inputs = BTreeMap::new();
        (self.runner).run_with(&mut self.part, &self.program, &inputs, None);
    }

    fn reader(&self) -> Reader<'_> {
        self.part.current()
    }

    fn rerun(&mut self) -> Rerun<'_> {
        Rerun {
            runner: &self.runner,
            part: &mut self.part,
            program: &self.program,
        }
    }

    /// Moves the state to `moved` and runs the program there.
    fn moved(&mut self, moved: StateValues) {
        self.program.state.extend(moved);
        self.run();
    }

    /// Solves the mates with `pulls`, and runs the program with the state
    /// the solve gives. A solve whose mates hold encloses a true solution
    /// near where it ended.
    fn solve(&mut self, pulls: &[Pull]) -> Report {
        let state = self.program.state.clone();
        let (moved, report) = solving::solve(&mut self.rerun(), &state, pulls, None).unwrap();
        if report.converged {
            assert!(report.unproven.is_empty(), "{:?}", report.unproven);
        }
        self.moved(moved);
        report
    }

    /// Solves with the values `set` held there.
    fn set(&mut self, set: &[(&str, f64)]) -> Report {
        let set = (set.iter()).map(|(p, v)| (p.to_string(), n(*v))).collect();
        let state = self.program.state.clone();
        let (moved, report) = solving::set(&mut self.rerun(), &state, &set).unwrap();
        self.moved(moved);
        report
    }

    /// How many ways the part placed as `name` can still move.
    fn freedom(&mut self, name: &str) -> usize {
        let state = self.program.state.clone();
        let freedom = solving::freedom(&mut self.rerun(), &state, &Default::default()).unwrap();
        freedom.parts[name]
    }

    /// The number the state gives `state`.
    fn number(&self, state: &str) -> f64 {
        match &self.program.state[state] {
            StateValue::Number(v) => v.to_f64(),
            other => panic!("{state} is a number, not {other:?}"),
        }
    }

    /// Where the part placed as `path` puts its point `p`, in the world.
    fn at(&self, path: &str, p: [f64; 3]) -> [f64; 3] {
        let world: Pose = locate(&self.reader(), path).unwrap().world;
        let q = world.apply(&Vector3::from_array(p.map(n)));
        [0, 1, 2].map(|k| q[k].to_f64())
    }

    /// Where the part placed as `path` turns the direction `d`.
    fn turned(&self, path: &str, d: [f64; 3]) -> [f64; 3] {
        let world: Pose = locate(&self.reader(), path).unwrap().world;
        let q = world.motion().rotate(&Vector3::from_array(d.map(n)));
        [0, 1, 2].map(|k| q[k].to_f64())
    }
}

/// The frame of the part placed as `instance`: its origin datum.
fn origin(instance: &str) -> MateEntity {
    MateEntity::of(
        instance,
        Selected::Datum(ElementReference::new(datums(), ORIGIN.to_string())),
    )
}

/// The datum plane of the part placed as `instance` normal to `axis`.
fn origin_plane(instance: &str, axis: FrameAxis) -> MateEntity {
    let name = component_name(ORIGIN, DatumComponent::Plane(axis));
    MateEntity::of(
        instance,
        Selected::Datum(ElementReference::new(datums(), name)),
    )
}

/// The cube placed fixed as `a`, then `steps`.
fn on_a(steps: Vec<(&str, AddPartArgs)>, state: StateValues) -> Mechanism {
    let fixed = AddPartArgs {
        file: "cube.geop".into(),
        ..Default::default()
    }
    .fixed();
    let mut all = vec![("a", fixed)];
    all.extend(steps);
    Mechanism::new(&all, state)
}

/// A part of `file` held by `mates`, `m1`, `m2`, ... in order.
fn placed(file: &str, mates: Vec<Mate>) -> AddPartArgs {
    AddPartArgs {
        file: file.into(),
        mates: (mates.into_iter().enumerate())
            .map(|(k, mate)| (format!("m{}", k + 1), mate))
            .collect(),
        ..Default::default()
    }
}

/// The cube placed as `b`, joined to `a` by a joint of `kind` between their
/// origins, at `pose`.
fn joined(kind: JointKind, pose: Pose) -> Mechanism {
    let joint = Mate::joint(kind, vec![origin("a"), origin("b")]);
    let state = StateValues::from([(pose_state("b"), StateValue::Pose(pose))]);
    on_a(vec![("b", placed("cube.geop", vec![joint]))], state)
}

fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose {
    Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n)).unwrap()
}

/// Every joint of the table frees as many ways of moving as its row says:
/// solved from a pose a little off, its part is as free as that, and the
/// coordinates it frees are what the joint's settings list.
#[test]
fn every_joint_frees_what_its_row_says() {
    for (joint, free) in [
        (Joint::Revolute, 1),
        (Joint::Slider, 1),
        (Joint::Cylindrical, 2),
        (Joint::Planar, 3),
        (Joint::PinSlot, 2),
        (Joint::Ball, 3),
        (Joint::Universal, 2),
        (Joint::Fastened, 0),
    ] {
        let mut mechanism = joined(
            JointKind::new(joint),
            pose([0.1, -0.2, 0.05], [3.0, -2.0, 10.0]),
        );
        let report = mechanism.solve(&[]);
        assert!(report.converged, "{joint:?}: {report:?}");
        assert_eq!(mechanism.freedom("b"), free, "{joint:?}");
        let settings = solving::settings(&mechanism.reader());
        let listed: Vec<String> = (settings.iter().flat_map(|s| &s.values))
            .map(|v| v.state.clone())
            .collect();
        let frees: Vec<String> = (JointKind::new(joint).motions().into_iter())
            .map(|m| joint_parameter("add_part(b,m1)", m))
            .collect();
        assert_eq!(listed, frees, "{joint:?}");
        // The origins are one, whatever turns: but for a slide.
        if !matches!(
            joint,
            Joint::Slider | Joint::Cylindrical | Joint::Planar | Joint::PinSlot
        ) {
            assert_close(mechanism.at("b", [0.0; 3]), [0.0; 3], SOLVED);
        }
    }
}

/// A planar joint goes where its coordinates say: along its first frame's
/// `x` and `y`, turned about its `z`.
#[test]
fn a_planar_joint_goes_to_its_coordinates() {
    let mut mechanism = joined(JointKind::new(Joint::Planar), pose([0.0; 3], [0.0; 3]));
    let joint = "add_part(b,m1)";
    let report = mechanism.set(&[
        (&joint_parameter(joint, Motion::SlideX), 0.5),
        (&joint_parameter(joint, Motion::SlideY), -0.25),
        (&joint_parameter(joint, Motion::Turn), 90.0),
    ]);
    assert!(report.converged, "{report:?}");
    assert_close(mechanism.at("b", [0.0; 3]), [0.5, -0.25, 0.0], SOLVED);
    assert_close(
        mechanism.turned("b", [1.0, 0.0, 0.0]),
        [0.0, 1.0, 0.0],
        1e-7,
    );
    assert_eq!(
        mechanism.number(&joint_parameter(joint, Motion::SlideX)),
        0.5
    );
}

/// A pin in a straight slot slides along its first frame's `x` and turns
/// about its `z`, and stops at the limits of its slide: dragged far along
/// the slot and off it, it goes to the slot's end, and no further off.
#[test]
fn a_pin_in_a_slot_stops_at_the_slots_end() {
    let kind =
        JointKind::new(Joint::PinSlot).limited(Motion::SlideX, Limits::between(n(-1.0), n(2.0)));
    let mut mechanism = joined(kind, pose([0.0; 3], [0.0; 3]));
    assert!(mechanism.solve(&[]).converged);
    let pull = Pull::Point {
        parameter: pose_state("b"),
        local: v3(0.0, 0.0, 0.0),
        target: v3(6.0, 1.0, 3.0),
    };
    let report = mechanism.solve(&[pull]);
    assert!(report.converged, "{report:?}");
    assert_eq!(report.at_limit, ["add_part(b,m1).x"]);
    assert_close(mechanism.at("b", [0.0; 3]), [2.0, 0.0, 0.0], SOLVED);
}

/// A ball joint turns its part any way about the common origin: a corner
/// pulled anywhere at its distance from the origin gets there, the origins
/// staying one.
#[test]
fn a_ball_joint_turns_any_way_about_its_center() {
    let mut mechanism = joined(JointKind::new(Joint::Ball), pose([0.0; 3], [0.0; 3]));
    let corner = 3.0_f64.sqrt();
    let pull = Pull::Point {
        parameter: pose_state("b"),
        local: v3(1.0, 1.0, 1.0),
        target: v3(0.0, 0.0, -corner),
    };
    let report = mechanism.solve(&[pull]);
    assert!(report.converged, "{report:?}");
    assert_close(mechanism.at("b", [0.0; 3]), [0.0; 3], SOLVED);
    // Near: a part pulled by a point resists turning a little.
    assert_close(mechanism.at("b", [1.0; 3]), [0.0, 0.0, -corner], 1e-3);
}

/// A universal joint keeps its two frames' `z` axes square, meeting at the
/// origins: a part started with them parallel is turned until they are
/// square, and is then free to turn about either.
#[test]
fn a_universal_joint_keeps_its_cross_square() {
    let mut mechanism = joined(
        JointKind::new(Joint::Universal),
        pose([0.0; 3], [20.0, 0.0, 0.0]),
    );
    let report = mechanism.solve(&[]);
    assert!(report.converged, "{report:?}");
    let z = mechanism.turned("b", [0.0, 0.0, 1.0]);
    assert!(z[2].abs() < SOLVED, "{z:?}");
    assert_close(mechanism.at("b", [0.0; 3]), [0.0; 3], SOLVED);
}

/// A joint's kind reads back as it was written, each coordinate's limits by
/// its name, and a limit on a coordinate it does not free is refused.
#[test]
fn joint_kinds_are_written_by_name_and_limits() {
    let kind = JointKind::new(Joint::Planar)
        .limited(
            Motion::SlideX,
            Limits {
                min: Some(n(-1.0)),
                max: None,
            },
        )
        .limited(Motion::Turn, Limits::between(n(-30.0), n(30.0)));
    let json = serde_json::to_value(kind).unwrap();
    assert_eq!(
        json,
        serde_json::json!({
            "type": "planar",
            "limits": {"angle": {"min": -30.0, "max": 30.0}, "x": {"min": -1.0}},
        })
    );
    assert_eq!(serde_json::from_value::<JointKind>(json).unwrap(), kind);
    for joint in Joint::ALL {
        let plain = JointKind::new(joint);
        let json = serde_json::to_value(plain).unwrap();
        assert_eq!(json, serde_json::json!({"type": joint.name()}));
        assert_eq!(serde_json::from_value::<JointKind>(json).unwrap(), plain);
    }
    let wrong =
        JointKind::new(Joint::Revolute).limited(Motion::Slide, Limits::between(n(0.0), n(1.0)));
    let err = wrong.validate().unwrap_err().to_string();
    assert!(
        err.contains("no distance to limit") && err.contains("frees angle"),
        "{err}"
    );
}

/// Two revolute joints on the cube `a`, the parts `b` and `c` each turning
/// on it, tied by the coupling `kind`.
fn coupled(kinds: [JointKind; 2], coupling: CouplingKind) -> Mechanism {
    let joint = |kind, on: &str| Mate::joint(kind, vec![origin("a"), origin(on)]);
    let tie = Mate::coupling(
        coupling,
        vec!["add_part(b,m1)".into(), "add_part(c,m1)".into()],
    );
    on_a(
        vec![
            ("b", placed("cube.geop", vec![joint(kinds[0], "b")])),
            ("c", placed("cube.geop", vec![joint(kinds[1], "c"), tie])),
        ],
        StateValues::new(),
    )
}

/// A belt turns its pulleys the same way: the driven one half as far, at a
/// ratio of 2.
#[test]
fn a_belt_turns_its_pulleys_the_same_way() {
    let revolute = JointKind::new(Joint::Revolute);
    let belt = CouplingKind::new(Transmission::Belt, n(2.0), false);
    let mut mechanism = coupled([revolute, revolute], belt);
    assert!(mechanism.solve(&[]).converged);
    let report = mechanism.set(&[("add_part(b,m1).angle", 60.0)]);
    assert!(report.converged, "{report:?}");
    let driven = mechanism.number("add_part(c,m1).angle");
    assert!((driven - 30.0).abs() < 1e-9, "{driven}");
}

/// A linear coupling moves one slide by its ratio times the other, plus
/// its offset.
#[test]
fn a_linear_coupling_moves_its_slides_by_ratio_and_offset() {
    let slider = JointKind::new(Joint::Slider);
    let linear = CouplingKind {
        offset: Some(n(0.5)),
        ..CouplingKind::new(Transmission::Linear, n(-2.0), false)
    };
    let mut mechanism = coupled([slider, slider], linear);
    assert!(mechanism.solve(&[]).converged);
    let report = mechanism.set(&[("add_part(b,m1).distance", 1.25)]);
    assert!(report.converged, "{report:?}");
    let driven = mechanism.number("add_part(c,m1).distance");
    assert!((driven + 2.0).abs() < 1e-9, "{driven}");
    assert_close(mechanism.at("c", [0.0; 3]), [0.0, 0.0, -2.0], SOLVED);
}

/// The name of the face of the part `model` that is not flat, nearest
/// `near`, and of its circular edge whose center is nearest `center`.
fn round_face(model: &NamedModel, near: [f64; 3]) -> String {
    let near = Vector3::from_array(near.map(n));
    let description = PartDescription::of(model).unwrap();
    let distance = |name: &String| {
        let face = model.face_id(name).unwrap();
        let surface = &model.topology().get_face(face).unwrap().surface;
        let curved = surface.as_plane().unwrap().is_none();
        let (_, _, p) = surface.closest_point(&near).unwrap();
        curved.then(|| p.sub(&near).norm().to_f64())
    };
    (description.faces.keys())
        .filter_map(|name| Some((distance(name)?, name.clone())))
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .unwrap()
        .1
}

/// The name of a circular edge of `model` whose center is `center`.
fn rim(model: &NamedModel, center: [f64; 3]) -> String {
    let center = Vector3::from_array(center.map(n));
    let description = PartDescription::of(model).unwrap();
    (description.edges.keys())
        .find(|name| {
            let edge = model.edge_id(name).unwrap();
            let curve = &model.topology().get_edge(edge).unwrap().curve;
            curve
                .as_arc()
                .unwrap()
                .is_some_and(|arc| arc.circle.center.sub(&center).norm().to_f64() < 1e-9)
        })
        .unwrap()
        .clone()
}

/// The roller placed on the drum, fixed as `a`: its bottom rim tangent to
/// the drum's side — `m1` — and its axis kept parallel to the drum's
/// (`m2`), started at `x`.
fn roller_on_drum(x: f64) -> Mechanism {
    let drum_side = MateEntity::of(
        "a",
        Selected::Face(element(round_face(&drum(), [3.0, 0.0, 0.5]))),
    );
    let roller_rim = MateEntity::of("b", Selected::Edge(element(rim(&roller(), [0.0; 3]))));
    let tangent = Mate::constraint(Kind::Tangent, vec![drum_side, roller_rim]);
    let upright = Mate::constraint(
        Kind::Parallel,
        vec![
            origin_plane("a", FrameAxis::Z),
            origin_plane("b", FrameAxis::Z),
        ],
    );
    let fixed = AddPartArgs {
        file: "drum.geop".into(),
        ..Default::default()
    }
    .fixed();
    let state = StateValues::from([(
        pose_state("b"),
        StateValue::Pose(pose([x, 0.3, 0.4], [0.0; 3])),
    )]);
    Mechanism::new(
        &[
            ("a", fixed),
            ("b", placed("roller.geop", vec![tangent, upright])),
        ],
        state,
    )
}

/// A roller's rim tangent to a drum's side keeps its center the roller's
/// radius off the drum, wherever it is pushed round it: its foot point on
/// the drum's face is solved with it.
#[test]
fn a_roller_keeps_touching_the_drum() {
    let mut mechanism = roller_on_drum(3.5);
    let report = mechanism.solve(&[]);
    assert!(report.converged, "{report:?}");
    let off = |m: &Mechanism| {
        let [x, y, _] = m.at("b", [0.0; 3]);
        x.hypot(y)
    };
    assert!(
        (off(&mechanism) - 2.5).abs() < SOLVED,
        "{}",
        off(&mechanism)
    );
    for foot in ["u", "v"] {
        let state = foot_parameter("add_part(b,m1)", foot);
        assert!(
            mechanism.program.state.contains_key(&state),
            "{state} solved"
        );
    }
    // Pushed round the drum and down into it, it rolls on round it.
    let pull = Pull::Point {
        parameter: pose_state("b"),
        local: v3(0.0, 0.0, 0.0),
        target: v3(1.5, 1.0, 0.5),
    };
    let report = mechanism.solve(&[pull]);
    assert!(report.converged, "{report:?}");
    assert!(
        (off(&mechanism) - 2.5).abs() < SOLVED,
        "{}",
        off(&mechanism)
    );
    let [x, y, _] = mechanism.at("b", [0.0; 3]);
    assert!(y > 0.3 && x > 0.0, "it went round: {x}, {y}");
}

/// A path keeps a point on a curve: the cube's corner on the drum's top
/// rim, a circle of radius 2 at height 1, wherever it is dragged.
#[test]
fn a_corner_runs_along_the_rim() {
    let top = MateEntity::of(
        "a",
        Selected::Edge(element(rim_near(&drum(), [2.0, 0.3, 1.0]))),
    );
    let path = Mate::constraint(Kind::Path, vec![top, origin("b")]);
    let fixed = AddPartArgs {
        file: "drum.geop".into(),
        ..Default::default()
    }
    .fixed();
    let state = StateValues::from([(
        pose_state("b"),
        StateValue::Pose(pose([2.2, 0.5, 1.1], [0.0; 3])),
    )]);
    let mut mechanism = Mechanism::new(
        &[("a", fixed), ("b", placed("cube.geop", vec![path]))],
        state,
    );
    let report = mechanism.solve(&[]);
    assert!(report.converged, "{report:?}");
    let [x, y, z] = mechanism.at("b", [0.0; 3]);
    assert!(
        (x.hypot(y) - 2.0).abs() < SOLVED && (z - 1.0).abs() < SOLVED,
        "{x} {y} {z}"
    );
    assert_eq!(
        mechanism.freedom("b"),
        4,
        "along the rim, and turned any way"
    );
}

/// The name of the edge of `model` nearest `near`.
fn rim_near(model: &NamedModel, near: [f64; 3]) -> String {
    let near = Vector3::from_array(near.map(n));
    let description = PartDescription::of(model).unwrap();
    let distance = |name: &String| {
        let edge = model.edge_id(name).unwrap();
        let curve = &model.topology().get_edge(edge).unwrap().curve;
        let (_, p) = curve.closest_point(&near).unwrap();
        p.sub(&near).norm().to_f64()
    };
    (description.edges.keys())
        .min_by(|a, b| distance(a).total_cmp(&distance(b)))
        .unwrap()
        .clone()
}

/// A tangent mate between two flat faces is refused, saying what it takes.
#[test]
fn a_tangent_mate_names_what_it_needs() {
    let tangent = Mate::constraint(Kind::Tangent, vec![origin("a"), origin("b")]);
    let mechanism = on_a(
        vec![("b", placed("cube.geop", vec![tangent]))],
        StateValues::new(),
    );
    let errors: Vec<String> = (mechanism.reader().failed_targets_of::<crate::Resolved>())
        .iter()
        .map(|e| e.to_string())
        .collect();
    assert!(
        errors.iter().any(|e| e.contains("is not a face")),
        "{errors:?}"
    );
}

/// Every constraint's derivatives, as the run gives them, are the true
/// ones: central differences of its residuals over a step of each state
/// it is differentiated by, the program run again for each.
#[track_caller]
fn assert_derivatives_are_true(mechanism: &mut Mechanism) {
    let state = mechanism.program.state.clone();
    let all = Constraints::of(&mechanism.reader()).unwrap();
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
                    let mut step = vec![Scalar::ZERO; value.dimension()];
                    step[k] = n(t);
                    let mut moved = state.clone();
                    moved.insert(parameter.clone(), value.retract(&step).unwrap());
                    mechanism.program.state = moved;
                    mechanism.run();
                    let made = Constraints::of(&mechanism.reader()).unwrap();
                    let residuals = &made.get(name).unwrap().residuals;
                    residuals.iter().map(|r| r.to_f64()).collect::<Vec<_>>()
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
    mechanism.program.state = state;
    mechanism.run();
}

/// The derivatives of every joint of the table — by the poses of its parts,
/// turned and away from the origin, and by its coordinates — and of a
/// tangent's and a path's — by their foot points too — are the true ones.
#[test]
fn joint_and_contact_derivatives_are_true() {
    for joint in Joint::ALL {
        let mut mechanism = joined(
            JointKind::new(joint),
            pose([0.3, -0.2, 0.4], [20.0, -35.0, 70.0]),
        );
        let joint_name = "add_part(b,m1)";
        for motion in JointKind::new(joint).motions() {
            let value = StateValue::Number(n(0.3 + motion.index() as f64));
            mechanism
                .program
                .state
                .insert(joint_parameter(joint_name, motion), value);
        }
        mechanism.run();
        assert_derivatives_are_true(&mut mechanism);
    }
    // Solved, so that the foot points are where a solve seeds them, then
    // moved off, so that nothing holds.
    let mut mechanism = roller_on_drum(3.5);
    assert!(mechanism.solve(&[]).converged);
    let off = StateValue::Pose(pose([2.8, 0.9, 0.3], [10.0, -5.0, 30.0]));
    mechanism.moved(StateValues::from([(pose_state("b"), off)]));
    assert_derivatives_are_true(&mut mechanism);
}
