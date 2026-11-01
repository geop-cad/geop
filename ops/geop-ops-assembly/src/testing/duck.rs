//! The `duck` benchmark: an assembly shaped like a user's toy duck, not
//! like a plate of screws. Few parts, but nested: the duck's own body
//! places a neck turning on it, the neck a head turning on it, the head a
//! mouth turning on it — a part placed in a part placed in a part — and two
//! legs, each a sub-assembly of a hip, a knee, an ankle and a foot, each
//! turning on the one before, the hip turning on the body by a joint the
//! duck makes into the leg (`/left_leg/hip`).
//!
//! What it measures is what nesting costs: the mouth dragged moves a part
//! three levels down, and every level above it is built again with the
//! state it is placed with.
//!
//! The state puts every part where its joints hold it, each turned by
//! nothing, so loading it solves nothing.

use std::collections::BTreeMap;

use geop_core_math::primitives::Pose;
use geop_core_program::{
    program::Program,
    values::{StateValues, pose_state},
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::{datums::datum, mate_entity::MateEntity, selection::Selected};

use super::robot_arm::{datum_of, frame, joint, moved, placed, saved};
use crate::{Joint, JointKind, Mate};

/// The file of the top program.
pub const DUCK: &str = "duck.geop";

/// The path of the mouth, as the duck places it: in the head, in the neck.
pub const MOUTH: &str = "neck/head/mouth";

/// Where the mouth's top is, in its own frame: what a drag grabs it by.
pub const MOUTH_TOP: [f64; 3] = [10.0, 0.0, 2.0];

/// Up: the axis every joint turns about.
const UP: [f64; 3] = [0.0, 0.0, 1.0];
const X: [f64; 3] = [1.0, 0.0, 0.0];

/// A revolute joint from `a`, which holds, to `b`, which turns.
fn revolute(a: MateEntity, b: MateEntity) -> Mate {
    joint(JointKind::new(Joint::Revolute), a, b)
}

/// The datum `name` of the part itself.
fn own(name: &str) -> MateEntity {
    MateEntity::on(Selected::Datum(datum(name)))
}

/// A part of the solid `solid`, its frame `near` at its origin — what it
/// turns on — and the frames `frames` it holds others by.
fn part(id: &str, solid: PrimitiveArgs, frames: &[(&str, [f64; 3])]) -> Program {
    let mut program = Program::new();
    program.push(id, solid);
    program.push("near", frame([0.0; 3], X, UP));
    for (name, origin) in frames {
        program.push(*name, frame(*origin, X, UP));
    }
    program
}

/// The leg's parts, from the hip down: each turns on the frame `far` of
/// the one before, which is that far below its origin.
const LEG: [(&str, f64); 4] = [("hip", 10.0), ("knee", 15.0), ("ankle", 8.0), ("foot", 3.0)];

/// A leg: the hip placed free, or `fixed` in the leg as the user's right
/// leg has it — the duck turns it on its body either way — the knee, the
/// ankle and the foot each turning on the one before.
fn leg(fixed: bool) -> Program {
    let mut program = Program::new();
    let mut state = StateValues::new();
    let mut below = 0.0;
    for (k, (id, length)) in LEG.into_iter().enumerate() {
        let file = format!("{id}.geop");
        let mates = match k {
            0 if fixed => vec![Mate::fixed_here()],
            0 => Vec::new(),
            _ => vec![revolute(
                datum_of(LEG[k - 1].0, "far"),
                datum_of(id, "near"),
            )],
        };
        program.push(id, placed(&file, mates));
        state.insert(pose_state(id), saved(moved([0.0, 0.0, -below])));
        below += length;
    }
    program.state = state;
    program
}

/// The files of the duck, by path, its top program [`DUCK`] among them.
pub fn duck() -> BTreeMap<String, String> {
    let mut files = BTreeMap::new();
    let mut add = |path: &str, program: Program| {
        files.insert(
            path.to_string(),
            program.to_json().expect("a program saves"),
        );
    };

    // The body, the duck's own: the neck turns on its frame `neck`, the
    // legs on `left_hip` and `right_hip`.
    let hips = [("left", -20.0), ("right", 20.0)];
    let mut duck = Program::new();
    duck.push(
        "body",
        PrimitiveArgs::block([-30.0, -20.0, 0.0], [30.0, 20.0, 30.0]),
    );
    duck.push("neck_frame", frame([20.0, 0.0, 30.0], X, UP));
    for (side, y) in hips {
        duck.push(format!("{side}_hip"), frame([-10.0, y, 10.0], X, UP));
    }
    let mut state = StateValues::new();
    duck.push(
        "neck",
        placed(
            "neck.geop",
            vec![revolute(own("neck_frame"), datum_of("neck", "near"))],
        ),
    );
    state.insert(pose_state("neck"), saved(moved([20.0, 0.0, 30.0])));
    // Where the neck has the head, and the head the mouth: the duck's,
    // behind their paths, as their own files have them.
    let head = moved([0.0, 0.0, 30.0]);
    let mouth = moved([15.0, 0.0, 15.0]);
    state.insert(format!("/neck:{}", pose_state("head")), saved(head));
    state.insert(format!("/neck/head:{}", pose_state("mouth")), saved(mouth));
    for (side, y) in hips {
        let id = format!("{side}_leg");
        let hip = format!("{id}/hip");
        duck.push(
            &id,
            placed(
                &format!("{id}.geop"),
                vec![revolute(
                    own(&format!("{side}_hip")),
                    datum_of(&hip, "near"),
                )],
            ),
        );
        state.insert(pose_state(&id), saved(Pose::identity()));
        let mut below = 0.0;
        for (part, length) in LEG {
            let at = moved([-10.0, y, 10.0 - below]);
            state.insert(format!("/{id}:{}", pose_state(part)), saved(at));
            below += length;
        }
        add(&format!("{id}.geop"), leg(side == "right"));
    }
    duck.state = state;
    add(DUCK, duck);

    let mut neck = part(
        "neck",
        PrimitiveArgs::block([-5.0, -5.0, 0.0], [5.0, 5.0, 30.0]),
        &[("far", [0.0, 0.0, 30.0])],
    );
    neck.push(
        "head",
        placed(
            "head.geop",
            vec![revolute(own("far"), datum_of("head", "near"))],
        ),
    );
    neck.state = StateValues::from([(pose_state("head"), saved(head))]);
    add("neck.geop", neck);

    let mut head = part(
        "head",
        PrimitiveArgs::sphere([0.0, 0.0, 15.0], 15.0),
        &[("mouth_frame", [15.0, 0.0, 15.0])],
    );
    head.push(
        "mouth",
        placed(
            "mouth.geop",
            vec![revolute(own("mouth_frame"), datum_of("mouth", "near"))],
        ),
    );
    head.state = StateValues::from([(pose_state("mouth"), saved(mouth))]);
    add("head.geop", head);

    add(
        "mouth.geop",
        part(
            "mouth",
            PrimitiveArgs::block([0.0, -5.0, -2.0], [15.0, 5.0, 2.0]),
            &[],
        ),
    );
    for (id, length) in LEG {
        let solid = match id {
            "foot" => PrimitiveArgs::block([-3.0, -3.0, -length], [12.0, 3.0, 0.0]),
            _ => PrimitiveArgs::cylinder_z([0.0, 0.0, -length], 3.0, length),
        };
        add(
            &format!("{id}.geop"),
            part(id, solid, &[("far", [0.0, 0.0, -length])]),
        );
    }
    files
}
