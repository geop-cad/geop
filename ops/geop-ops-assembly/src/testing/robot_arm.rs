//! The `robot_arm` benchmark: an assembly shaped like a real robot, not
//! like a plate of screws. A fixed base and a chain of links, each turning
//! on the one before by a revolute joint; on each link brackets fastened to
//! it and screws mated to the brackets — not to anything fixed; per axis a
//! NEMA 17 motor fastened to the link before, a 16-tooth pulley turning on
//! its shaft, belted (a gear coupling, not reversed) to a 36-tooth pulley
//! fastened to the link it drives; and a gripper, a sub-assembly of a palm
//! and two fingers on sliders, fastened to the last link.
//!
//! Every mate ties a part to a part that moves, so nothing splits the arm
//! into the small groups of [`crate::solving`] that the screws on a plate
//! fall into: what makes a large assembly hard for a solver. With the
//! defaults of [`RobotArm::default`] it places about 1000 parts; 80 screws
//! per bracket make about 2000.
//!
//! The state puts every part where its mates hold it, so loading it solves
//! nothing.

use std::collections::BTreeMap;

use geop_core_math::{
    primitives::{FrameAxis, Pose, Quaternion},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    program::Program,
    values::{StateValue, StateValues, pose_state},
};
use geop_lib_fixtures::{GivenDatumArgs, PrimitiveArgs};
use geop_ops_model::{datums::datum, element, mate_entity::MateEntity, selection::Selected};

use crate::{
    AddPartArgs, CouplingKind, Joint, JointKind, Kind, Limits, Mate, Motion, Transmission,
    joint_parameter,
};

/// The sizes of a [`robot_arm`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RobotArm {
    /// Links in the chain, each turning on the one before.
    pub axes: usize,
    /// Brackets fastened to each link.
    pub brackets_per_link: usize,
    /// Screws mated to each bracket: at most 81.
    pub screws_per_bracket: usize,
}

impl Default for RobotArm {
    /// Six axes, four brackets per link, 40 screws per bracket: about 1000
    /// parts.
    fn default() -> Self {
        RobotArm {
            axes: 6,
            brackets_per_link: 4,
            screws_per_bracket: 40,
        }
    }
}

/// The file of the top program.
pub const ARM: &str = "arm.geop";

/// How long a link is between its joints, and how thick.
const LINK: f64 = 200.0;
const THICK: f64 = 20.0;
/// Where a link's motor is fastened under it, along it.
const MOTOR_AT: f64 = 60.0;
/// Screw holes of a bracket: a 9 x 9 grid, 4 apart, on its top.
const GRID: usize = 9;
const PITCH: f64 = 4.0;
const BRACKET_TOP: f64 = 10.0;
/// Teeth of the motor's pulley and of the joint's: the belt's ratio.
const DRIVER_TEETH: f64 = 16.0;
const DRIVEN_TEETH: f64 = 36.0;
/// How far each finger of the gripper is slid open.
const OPEN: f64 = 5.0;

impl RobotArm {
    /// How many parts it places, however deep: the base; per axis a link,
    /// a motor, two pulleys, the brackets and their screws; the gripper,
    /// its palm and two fingers.
    pub fn parts(&self) -> usize {
        let per_axis = 4 + self.brackets_per_link * (1 + self.screws_per_bracket);
        1 + self.axes * per_axis + 4
    }

    /// Where along a link the bracket `k` stands.
    fn slot(&self, k: usize) -> f64 {
        let span = LINK - 50.0;
        match self.brackets_per_link {
            0 | 1 => 25.0,
            n => 25.0 + span * k as f64 / (n - 1) as f64,
        }
    }

    /// The joint angle of axis `i` (from one), in degrees: the arm bent
    /// one way and the other, nowhere straight.
    pub fn angle(i: usize) -> f64 {
        const ANGLES: [f64; 6] = [30.0, -40.0, 35.0, -30.0, 40.0, -35.0];
        ANGLES[(i - 1) % ANGLES.len()]
    }

    /// The step placing link `i` (from one), and the joint that turns it.
    pub fn link(i: usize) -> String {
        format!("link{i}")
    }

    /// The name of the revolute joint of axis `i`.
    pub fn joint(i: usize) -> String {
        format!("add_part({},m1)", Self::link(i))
    }

    /// The steps placing the brackets and screws of link `i`.
    pub fn bracket(i: usize, k: usize) -> String {
        format!("bracket{i}_{k}")
    }

    pub fn screw(i: usize, k: usize, j: usize) -> String {
        format!("screw{i}_{k}_{j}")
    }
}

/// A sharp vector.
fn v(p: [f64; 3]) -> Vector3 {
    Vector3::from_array(p.map(Scalar::from_f64))
}

/// The frame datum at `origin`, its `x` along `x` and `z` along `z`.
fn frame(origin: [f64; 3], x: [f64; 3], z: [f64; 3]) -> GivenDatumArgs {
    GivenDatumArgs::frame(origin, x, z)
}

/// The pose of a frame at `origin`, its `x` along `x`, its `z` along `z`
/// (unit, square to each other).
fn frame_pose(origin: [f64; 3], x: [f64; 3], z: [f64; 3]) -> Pose {
    let (x, z) = (v(x), v(z));
    let y = z.prod_cross(&x);
    let rotation = Quaternion::from_rotation_columns([x, y, z]).expect("a rotation");
    Pose::new(v(origin), rotation).expect("a pose")
}

/// Turned by `degrees` about `z`.
fn turned(degrees: f64) -> Pose {
    Pose::from_euler(Vector3::zero(), [0.0, 0.0, degrees].map(Scalar::from_f64)).expect("a turn")
}

/// Moved by `offset`.
fn moved(offset: [f64; 3]) -> Pose {
    Pose::identity().with_position(v(offset))
}

/// The pose as a file holds it.
fn saved(pose: Pose) -> StateValue {
    StateValue::Pose(pose.saved().expect("a pose saves"))
}

/// The datum `name` of the part placed as `instance`.
fn datum_of(instance: &str, name: &str) -> MateEntity {
    MateEntity::of(instance, Selected::Datum(datum(name)))
}

/// The face `name` of the part placed as `instance`.
fn face_of(instance: &str, name: &str) -> MateEntity {
    MateEntity::of(instance, Selected::Face(element(name)))
}

/// A part placed from `file`, held by `mates`, named `m1`, `m2`, ….
fn placed(file: &str, mates: Vec<Mate>) -> AddPartArgs {
    AddPartArgs {
        file: file.into(),
        mates: (mates.into_iter().enumerate())
            .map(|(k, mate)| (format!("m{}", k + 1), mate))
            .collect(),
        ..Default::default()
    }
}

/// A joint of `kind` from `a` to `b`.
fn joint(kind: JointKind, a: MateEntity, b: MateEntity) -> Mate {
    Mate::joint(kind, vec![a, b])
}

/// The base: a block under the origin, the first joint's frame `far` on
/// its top at the origin, the first motor's `motor` under it.
fn base() -> Program {
    let mut program = Program::new();
    program.push(
        "base",
        PrimitiveArgs::block([-100.0, -100.0, -THICK], [100.0, 100.0, 0.0]),
    );
    program.push("far", frame([0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    program.push(
        "motor",
        frame([MOTOR_AT, 0.0, -THICK], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
    );
    program
}

/// The frame datums of a link: `near`, the joint it turns on, at its
/// origin under it; `far`, the next joint, on its top at its far end;
/// `motor`, under it, where the next axis' motor is fastened; `slot<k>`
/// on its top where bracket `k` is.
fn link_frames(arm: &RobotArm) -> Vec<(String, Pose, GivenDatumArgs)> {
    let up = [0.0, 0.0, 1.0];
    let x = [1.0, 0.0, 0.0];
    let mut frames = vec![
        ("near".to_string(), [0.0; 3], x, up),
        ("far".to_string(), [LINK, 0.0, THICK], x, up),
        (
            "motor".to_string(),
            [MOTOR_AT, 0.0, 0.0],
            x,
            [0.0, 0.0, -1.0],
        ),
    ];
    for k in 0..arm.brackets_per_link {
        frames.push((format!("slot{k}"), [arm.slot(k), 0.0, THICK], x, up));
    }
    (frames.into_iter())
        .map(|(name, origin, x, z)| (name, frame_pose(origin, x, z), frame(origin, x, z)))
        .collect()
}

/// A link: a bar from the origin along `x`, with its [`link_frames`].
fn link(arm: &RobotArm) -> Program {
    let mut program = Program::new();
    program.push(
        "link",
        PrimitiveArgs::block([-20.0, -20.0, 0.0], [LINK + 20.0, 20.0, THICK]),
    );
    for (name, _, datum) in link_frames(arm) {
        program.push(name, datum);
    }
    program
}

/// Where the screw hole `j` of a bracket is, on its top.
fn hole(j: usize) -> [f64; 3] {
    let half = (GRID - 1) as f64 / 2.0;
    let (gx, gy) = ((j % GRID) as f64 - half, (j / GRID) as f64 - half);
    [gx * PITCH, gy * PITCH, BRACKET_TOP]
}

/// A bracket: a block standing on its frame `foot`, its top
/// `block(bracket,end)` with the axes `hole<j>` of `screws` holes.
fn bracket(screws: usize) -> Program {
    let mut program = Program::new();
    program.push(
        "bracket",
        PrimitiveArgs::block([-20.0, -20.0, 0.0], [20.0, 20.0, BRACKET_TOP]),
    );
    program.push("foot", frame([0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    for j in 0..screws {
        program.push(
            format!("hole{j}"),
            GivenDatumArgs::axis(hole(j), [0.0, 0.0, 1.0]),
        );
    }
    program
}

/// A screw: a pin standing on the origin, its bottom
/// `cylinder(screw,start)`, its axis `axis`.
fn screw() -> Program {
    let mut program = Program::new();
    program.push("screw", PrimitiveArgs::cylinder_z([0.0; 3], 1.2, 8.0));
    program.push("axis", GivenDatumArgs::axis([0.0; 3], [0.0, 0.0, 1.0]));
    program
}

/// The palm of the gripper: a block on its frame `mount`, with the rails
/// `rail0` and `rail1` its fingers slide along, along `x`.
fn palm() -> Program {
    let mut program = Program::new();
    program.push(
        "palm",
        PrimitiveArgs::block([-15.0, -15.0, 0.0], [15.0, 15.0, 10.0]),
    );
    program.push("mount", frame([0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    for (k, y) in [-10.0, 10.0].into_iter().enumerate() {
        program.push(
            format!("rail{k}"),
            frame([0.0, y, 10.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]),
        );
    }
    program
}

/// A finger: a block on its frame `base`, which slides along `x`.
fn finger() -> Program {
    let mut program = Program::new();
    program.push(
        "finger",
        PrimitiveArgs::cylinder([0.0; 3], FrameAxis::Z, 2.0, 15.0),
    );
    program.push("base", frame([0.0; 3], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]));
    program
}

/// The gripper: the palm, fixed, and two fingers on sliders along its
/// rails, opened [`OPEN`]; its frame `mount` where the palm's is, what it
/// is fastened by.
fn gripper() -> Program {
    let mut program = Program::new();
    program.push("palm", placed("palm.geop", Vec::new()).fixed());
    program.push("mount", frame([0.0; 3], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]));
    let mut state = StateValues::from([(pose_state("palm"), saved(Pose::identity()))]);
    for k in 0..2 {
        let id = format!("finger{k}");
        let slider = JointKind::new(Joint::Slider).limited(
            Motion::Slide,
            Limits {
                min: Some(Scalar::ZERO),
                max: Some(Scalar::from_f64(10.0)),
            },
        );
        let rail = datum_of("palm", &format!("rail{k}"));
        program.push(
            &id,
            placed(
                "finger.geop",
                vec![joint(slider, rail, datum_of(&id, "base"))],
            ),
        );
        let y = if k == 0 { -10.0 } else { 10.0 };
        state.insert(pose_state(&id), saved(moved([OPEN, y, 10.0])));
        state.insert(
            joint_parameter(&format!("add_part({id},m1)"), Motion::Slide),
            StateValue::Number(Scalar::from_f64(OPEN)),
        );
    }
    program.state = state;
    program
}

/// The files of the arm `arm`, by path, its top program [`ARM`] among
/// them; the motors and pulleys are standard parts.
pub fn robot_arm(arm: &RobotArm) -> BTreeMap<String, String> {
    let revolute = || {
        JointKind::new(Joint::Revolute).limited(
            Motion::Turn,
            Limits {
                min: Some(Scalar::from_f64(-170.0)),
                max: Some(Scalar::from_f64(170.0)),
            },
        )
    };
    let fastened = || JointKind::new(Joint::Fastened);
    let frames: BTreeMap<String, Pose> = (link_frames(arm).into_iter())
        .map(|(name, pose, _)| (name, pose))
        .collect();
    let mut program = Program::new();
    let mut state = StateValues::new();
    program.push("base", placed("base.geop", Vec::new()).fixed());
    state.insert(pose_state("base"), saved(Pose::identity()));
    // Where the link before is, and its frames.
    let base_far = Pose::identity();
    let base_motor = frame_pose([MOTOR_AT, 0.0, -THICK], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]);
    let (mut before, mut far, mut motor_frame) = ("base".to_string(), base_far, base_motor);
    let mut at = Pose::identity();
    for i in 1..=arm.axes {
        let id = RobotArm::link(i);
        let angle = RobotArm::angle(i);
        // The link, turned on the one before.
        let world = at.compose(&far).compose(&turned(angle));
        program.push(
            &id,
            placed(
                "link.geop",
                vec![joint(
                    revolute(),
                    datum_of(&before, "far"),
                    datum_of(&id, "near"),
                )],
            ),
        );
        state.insert(pose_state(&id), saved(world));
        state.insert(
            joint_parameter(&RobotArm::joint(i), Motion::Turn),
            StateValue::Number(Scalar::from_f64(angle)),
        );
        // Its motor, fastened under the link before, and the pulley on
        // the motor's shaft, belted to the one on the joint.
        let motor = format!("motor{i}");
        let motor_world = at.compose(&motor_frame);
        program.push(
            &motor,
            placed(
                "std:nema17_stepper.geop",
                vec![joint(
                    fastened(),
                    datum_of(&before, "motor"),
                    datum_of(&motor, "axis"),
                )],
            ),
        );
        state.insert(pose_state(&motor), saved(motor_world));
        let driver = format!("pulley{i}");
        let ratio = DRIVEN_TEETH / DRIVER_TEETH;
        let belt = Mate::coupling(
            CouplingKind {
                transmission: Transmission::Gear,
                value: Scalar::from_f64(ratio),
                reverse: false,
                offset: None,
            },
            vec![format!("add_part({driver},m1)"), RobotArm::joint(i)],
        );
        program.push(
            &driver,
            AddPartArgs {
                file: "std:gt2_pulley_16t.geop".into(),
                mates: BTreeMap::from([
                    (
                        "m1".to_string(),
                        joint(
                            JointKind::new(Joint::Revolute),
                            datum_of(&motor, "axis"),
                            datum_of(&driver, "axis"),
                        ),
                    ),
                    ("m2".to_string(), belt),
                ]),
                ..Default::default()
            },
        );
        state.insert(
            pose_state(&driver),
            saved(motor_world.compose(&turned(ratio * angle))),
        );
        state.insert(
            joint_parameter(&format!("add_part({driver},m1)"), Motion::Turn),
            StateValue::Number(Scalar::from_f64(ratio * angle)),
        );
        let driven = format!("hub{i}");
        program.push(
            &driven,
            placed(
                "std:gt2_pulley_36t.geop",
                vec![joint(
                    fastened(),
                    datum_of(&id, "near"),
                    datum_of(&driven, "axis"),
                )],
            ),
        );
        state.insert(pose_state(&driven), saved(world));
        // The brackets fastened to the link, and their screws.
        for k in 0..arm.brackets_per_link {
            let bracket = RobotArm::bracket(i, k);
            let bracket_world = world.compose(&frames[&format!("slot{k}")]);
            program.push(
                &bracket,
                placed(
                    "bracket.geop",
                    vec![joint(
                        fastened(),
                        datum_of(&id, &format!("slot{k}")),
                        datum_of(&bracket, "foot"),
                    )],
                ),
            );
            state.insert(pose_state(&bracket), saved(bracket_world));
            for j in 0..arm.screws_per_bracket {
                let screw = RobotArm::screw(i, k, j);
                program.push(
                    &screw,
                    placed(
                        "screw.geop",
                        vec![
                            Mate::constraint(
                                Kind::Coincident,
                                vec![
                                    face_of(&screw, "cylinder(screw,start)"),
                                    face_of(&bracket, "block(bracket,end)"),
                                ],
                            ),
                            Mate::constraint(
                                Kind::Concentric,
                                vec![
                                    datum_of(&bracket, &format!("hole{j}")),
                                    datum_of(&screw, "axis"),
                                ],
                            ),
                        ],
                    ),
                );
                state.insert(
                    pose_state(&screw),
                    saved(bracket_world.compose(&moved(hole(j)))),
                );
            }
        }
        (before, far, motor_frame, at) = (id, frames["far"], frames["motor"], world);
    }
    // The gripper, fastened by its own frame on the last link's far end.
    program.push(
        "gripper",
        placed(
            "gripper.geop",
            vec![joint(
                fastened(),
                datum_of(&before, "far"),
                datum_of("gripper", "mount"),
            )],
        ),
    );
    state.insert(pose_state("gripper"), saved(at.compose(&far)));
    program.state = state;
    [
        (ARM, program),
        ("base.geop", base()),
        ("link.geop", link(arm)),
        ("bracket.geop", bracket(arm.screws_per_bracket)),
        ("screw.geop", screw()),
        ("palm.geop", palm()),
        ("finger.geop", finger()),
        ("gripper.geop", gripper()),
    ]
    .into_iter()
    .map(|(path, program)| (path.to_string(), program.to_json().expect("plain data")))
    .collect()
}
