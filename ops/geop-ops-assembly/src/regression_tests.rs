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

use crate::{AddPartArgs, program_tests::editor};

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

/// A cube on a fixed cube, its side `block(c,c1)` at an angle to the fixed
/// cube's: at 179° it is turned nearly half way round, and at 180° all the
/// way — its side facing opposite the other's — not left facing the same
/// way, as at 0°.
#[test]
#[ignore = "bug: residuals::Kind::Angle holds 180° as it holds 0°, as a parallel (either way), so \
            the cube placed as `b` on the fixed cube `a` with an angle mate of 180° between their \
            faces `block(c,c1)` stays turned 0° (where it starts at 20°), while 179° turns it \
            179°: the mate's geometry jumps half a turn between 179.9° and 180°"]
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
        program.state.insert(pose_state("b"), StateValue::Pose(start));
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
/// linear in where it is: the solve puts it there, however far it starts.
#[test]
#[ignore = "bug: the settle leaves a part that starts far away unsolved: the cube placed as `b` \
            at (600, -300, 200), not turned, mated to the fixed cube `a` by three plane-on-plane \
            mates, is reported with add_part(b,m1), (b,m2) and (b,m3) all failing (a report of \
            0 iterations); started at (60, -30, 20) it solves"]
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
        .map(|(k, (kind, entities))| (format!("m{}", k + 1), Mate::constraint(kind, entities.to_vec())))
        .collect();
    let placed = AddPartArgs {
        file: "cube.geop".into(),
        ..Default::default()
    };
    let mut program = Program::new();
    program.push("a", placed.clone().fixed());
    program.push("b", AddPartArgs { mates, ..placed });
    let far = crate::examples::pose([600.0, -300.0, 200.0], [0.0; 3]);
    program.state.insert(pose_state("b"), StateValue::Pose(far));
    let mut editor = editor();
    let files = BTreeMap::from([("cube.geop".to_string(), Some(cube.to_json().unwrap()))]);
    handled(&mut editor, Command::Files { files });
    let path = Some("cubes.geop".into());
    handled(&mut editor, Command::Load { program, path });
    let report = crate::solving::check(&editor.part()).unwrap();
    assert!(report.converged, "{report:?}");
    let Some(StateValue::Pose(pose)) = editor.program().state.get(&pose_state("b")) else {
        panic!("the cube has a pose");
    };
    let at = pose.position().to_f64_array();
    let off = (at.iter().zip([1.5, 0.0, 1.0])).map(|(a, b)| (a - b).abs());
    assert!(off.fold(0.0, f64::max) < 1e-6, "the cube is at {at:?}");
}
