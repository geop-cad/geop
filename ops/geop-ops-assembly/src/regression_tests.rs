//! Users' assemblies that went wrong, rebuilt as programs.

use std::collections::BTreeMap;

use geop_core_editor::{
    editing::{GizmoPart, Pointer, Reach, StepEditEvent},
    editor::Command,
    testing::{handled, pointer},
};
use geop_core_math::scalars::Scalar;
use geop_core_program::{
    program::Program,
    values::{StateValue, pose_state},
};
use geop_lib_fixtures::{PrimitiveArgs, drilled_box};

use geop_lib_constraints::Constraints;

use crate::{
    AddPartArgs,
    program_tests::{editor, workspace},
    scale_tests::arm_editor,
    testing::robot_arm::RobotArm,
};

/// A robot's base held fixed beside a servo held fixed too, its pose saved
/// with a quaternion of norm 1.09 — a quaternion stands for its rotation
/// whatever its norm. Its gizmo's ball grabbed and let go where it was
/// grabbed leaves the base where it was. The pose's position was turned by
/// the quaternion without dividing by its norm squared, so each drag event
/// moved the base 1.19 times as far from the origin as the solve before
/// had left it, and a slight drag threw it out of sight, then out of
/// memory.
#[test]
fn a_part_saved_with_a_rotation_off_unit_stays_where_it_is_grabbed() {
    let mut cube = Program::new();
    cube.push("c", PrimitiveArgs::block([0.0; 3], [1.0; 3]));
    let files = BTreeMap::from([
        ("base.geop".to_string(), Some(cube.to_json().unwrap())),
        (
            "servo.geop".to_string(),
            Some(drilled_box().to_json().unwrap()),
        ),
    ]);
    let mut program = Program::new();
    let placed = |file: &str| AddPartArgs {
        file: file.into(),
        ..Default::default()
    };
    program.push("part2", placed("base.geop").fixed());
    program.push("part1", placed("servo.geop").fixed());
    let saved = serde_json::json!({
        "position": [0.0, -7.281465399433621, 0.0],
        "rotation": [1.0901040959577553, 0.0, 6.788445791068112e-16, 0.0],
    });
    let pose: StateValue = serde_json::from_value(saved).unwrap();
    program.state.insert(pose_state("part2"), pose);

    let mut editor = editor();
    handled(&mut editor, Command::Files { files });
    let path = Some("bot.geop".into());
    handled(&mut editor, Command::Load { program, path });
    handled(
        &mut editor,
        Command::Open {
            id: "part2".into(),
            placed: None,
        },
    );
    // Looked at in perspective from up and to the side, the ball grabbed
    // and moved a little at a time, in the plane facing the eye.
    let base = [0.0, -7.281465399433621, 0.0];
    let eye = [base[0] + 90.0, base[1] - 110.0, base[2] + 80.0];
    let towards = |p: [f64; 3]| {
        let mut pointer = pointer(eye, [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]]);
        pointer.reach = Reach::Cone {
            slope: Scalar::from_f64(0.004),
        };
        pointer
    };
    let from = towards(base);
    let hovered = handled(&mut editor, hover(from));
    let gizmo = hovered.step.and_then(|s| s.presentation.gizmo);
    assert_eq!(gizmo.and_then(|g| g.hover), Some(GizmoPart::Free));
    // Pressed, not moved yet: the base stays where it is.
    let event = StepEditEvent::Drag {
        from,
        to: from,
        done: false,
        shift: false,
    };
    let pressed = handled(&mut editor, Command::Event { event });
    let drawn = (pressed.scene.into_iter().flat_map(|s| s.instances))
        .find(|i| i.name == "part2")
        .map(|i| i.frame.origin().to_f64_array());
    if let Some(drawn) = drawn {
        let off = (drawn.iter().zip(base)).map(|(a, b)| (a - b).abs());
        assert!(
            off.fold(0.0, f64::max) < 1e-6,
            "pressed, the base moved to {drawn:?}"
        );
    }
    for k in 1..=4 {
        let k = f64::from(k);
        let to = towards([base[0] + 0.5 * k, base[1], base[2] + 0.2 * k]);
        let event = StepEditEvent::Drag {
            from,
            to,
            done: k == 4.0,
            shift: false,
        };
        handled(&mut editor, Command::Event { event });
    }
    handled(&mut editor, Command::Commit);
    let Some(StateValue::Pose(pose)) = editor.program().state.get(&pose_state("part2")) else {
        panic!("the base has a pose");
    };
    let position = pose.position().to_f64_array();
    let off = (position.iter().zip(base))
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max);
    assert!(
        off < 3.0,
        "a drag of two units moved the base to {position:?}"
    );
}

/// The pointer hovering `pointer`.
fn hover(pointer: Pointer) -> Command {
    let event = StepEditEvent::Hover {
        pointer,
        shift: false,
    };
    Command::Event { event }
}

/// A gripper — a sub-assembly of a palm and two fingers on sliders —
/// fastened to an arm's last link by a frame of its own, not of one of its
/// parts. The mate was refused: what it turns a turn into a length by, the
/// gripper's size, was read from the gripper as drawn, which is drawn
/// where its fingers are, so the constraint read a target that reads the
/// state. A part made of placed parts is as large as the largest of them.
#[test]
fn a_gripper_fastened_by_a_frame_of_its_own_holds_where_it_is() {
    let arm = RobotArm {
        axes: 1,
        brackets_per_link: 0,
        screws_per_bracket: 0,
    };
    // The editor loads it, every step built and every mate holding.
    let editor = arm_editor(&arm);
    let constraints = Constraints::of(&editor.part()).unwrap();
    assert!(
        constraints.get("add_part(gripper,m1)").is_some(),
        "the gripper is fastened"
    );
}

/// A cube on a fixed cube, its side `block(c,c1)` at an angle to the fixed
/// cube's: at 179° it is turned nearly half way round, and at 180° all the
/// way — its side facing opposite the other's — not left facing the same
/// way, as at 0°.
#[test]
fn an_angle_mate_of_half_a_turn_turns_the_part_half_way_round() {
    use crate::{Kind, Mate};
    use geop_ops_model::{element, mate_entity::MateEntity, selection::Selected};
    let face = |instance: &str, name: &str| MateEntity::of(instance, Selected::Face(element(name)));
    let mut cube = Program::new();
    cube.push("c", PrimitiveArgs::block([0.0; 3], [1.0; 3]));
    let turned = |degrees: f64| {
        let mates = BTreeMap::from([
            (
                "m1".to_string(),
                Mate::constraint(
                    Kind::Coincident,
                    vec![face("b", "block(c,start)"), face("a", "block(c,end)")],
                ),
            ),
            (
                "m2".to_string(),
                Mate::constraint(
                    Kind::Angle {
                        value: Scalar::from_f64(degrees),
                    },
                    vec![face("b", "block(c,c1)"), face("a", "block(c,c1)")],
                ),
            ),
        ]);
        let mut program = Program::new();
        let placed = AddPartArgs {
            file: "cube.geop".into(),
            ..Default::default()
        };
        program.push("a", placed.clone().fixed());
        program.push("b", AddPartArgs { mates, ..placed });
        let start = crate::examples::pose([2.0, 0.3, 0.4], [0.0, 0.0, 20.0]);
        program
            .state
            .insert(pose_state("b"), StateValue::Pose(start));
        let mut editor = editor();
        let files = BTreeMap::from([("cube.geop".to_string(), Some(cube.to_json().unwrap()))]);
        handled(&mut editor, Command::Files { files });
        let path = Some("cubes.geop".into());
        handled(&mut editor, Command::Load { program, path });
        assert!(crate::solving::check(&editor.part()).unwrap().converged);
        match editor.program().state.get(&pose_state("b")) {
            Some(StateValue::Pose(pose)) => pose.euler_degrees()[2].abs(),
            other => panic!("the cube has no pose: {other:?}"),
        }
    };
    let nearly = turned(179.0);
    assert!((nearly - 179.0).abs() < 1e-6, "turned {nearly}° at 179°");
    let half = turned(180.0);
    assert!((half - 180.0).abs() < 1e-6, "turned {half}° at 180°");
}

/// A unit cube left some 700 away from a fixed one — dragged off, or
/// drawn far from where it goes — and then mated onto it by three faces:
/// on top, flush at the front, half a unit beside it. Every mate is a
/// plane against a plane and the cube is not turned, so the mates are
/// linear in where it is: the solve puts it there, however far it starts —
/// turned a little, too, from ten thousand away.
#[test]
fn a_part_mated_from_far_away_is_brought_onto_its_mates() {
    use crate::{Kind, Mate};
    use geop_ops_model::{element, mate_entity::MateEntity, selection::Selected};
    let face = |instance: &str, name: &str| MateEntity::of(instance, Selected::Face(element(name)));
    let mut cube = Program::new();
    cube.push("c", PrimitiveArgs::block([0.0; 3], [1.0; 3]));
    let mates = [
        (
            Kind::Coincident,
            [face("b", "block(c,start)"), face("a", "block(c,end)")],
        ),
        (
            Kind::Coincident,
            [face("b", "block(c,c0)"), face("a", "block(c,c0)")],
        ),
        (
            Kind::Distance {
                value: Scalar::from_f64(0.5),
            },
            [face("b", "block(c,c3)"), face("a", "block(c,c1)")],
        ),
    ];
    let mates = (mates.into_iter().enumerate())
        .map(|(k, (kind, entities))| {
            (
                format!("m{}", k + 1),
                Mate::constraint(kind, entities.to_vec()),
            )
        })
        .collect();
    let placed = AddPartArgs {
        file: "cube.geop".into(),
        ..Default::default()
    };
    let mut placing = Program::new();
    placing.push("a", placed.clone().fixed());
    placing.push("b", AddPartArgs { mates, ..placed });
    let texts = BTreeMap::from([("cube.geop".to_string(), cube.to_json().unwrap())]);
    let starts = [
        ([600.0, -300.0, 200.0], [0.0; 3]),
        ([1e4, -5e3, 2e3], [0.0, 0.0, 20.0]),
    ];
    for (at, turn) in starts {
        let mut program = placing.clone();
        let far = crate::examples::pose(at, turn);
        program.state.insert(pose_state("b"), StateValue::Pose(far));
        // The solve itself, which the settle asks and says nothing of when
        // it fails.
        let crate::program_tests::BuiltPart { runner, mut part } =
            crate::program_tests::build_file(&program, "cubes.geop", workspace(texts.clone()))
                .unwrap();
        let mut rerun = geop_core_editor::editor::Rerun {
            runner: &runner,
            part: &mut part,
            program: &program,
        };
        let (_, report) =
            crate::solving::solve(&mut rerun, &program.state, &[], None, true, None).unwrap();
        assert!(report.converged, "from {at:?}: {report:?}");
        let mut editor = editor();
        let files = BTreeMap::from([("cube.geop".to_string(), Some(cube.to_json().unwrap()))]);
        handled(&mut editor, Command::Files { files });
        let path = Some("cubes.geop".into());
        handled(&mut editor, Command::Load { program, path });
        let report = crate::solving::check(&editor.part()).unwrap();
        assert!(report.converged, "from {at:?}: {report:?}");
        // Turned, it may as well settle turned half way round, on the
        // other side of the line it is held half a unit off.
        if turn != [0.0; 3] {
            continue;
        }
        let Some(StateValue::Pose(pose)) = editor.program().state.get(&pose_state("b")) else {
            panic!("the cube has a pose");
        };
        let to = pose.position().to_f64_array();
        let off = (to.iter().zip([1.5, 0.0, 1.0])).map(|(a, b)| (a - b).abs());
        assert!(
            off.fold(0.0, f64::max) < 1e-6,
            "from {at:?}, the cube is at {to:?}"
        );
    }
}
