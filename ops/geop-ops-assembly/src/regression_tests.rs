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
    AddPartArgs, program_tests::editor, scale_tests::arm_editor, testing::robot_arm::RobotArm,
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
