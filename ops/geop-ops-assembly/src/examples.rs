//! The examples of assemblies ([`set`]): programs placing the parts other
//! files build, written in Rust against the operation types — and the
//! parts they place, the files of each example made of several
//! ([`workspaces`]).
//!
//! Each refers to what earlier steps built only by name, and so reads as
//! the recipe it is: `extrude(pin,start)` is the bottom of the pin.

// An example is fixed data: one that does not build is a bug in it, said
// where it is made.
#![allow(
    clippy::expect_used,
    reason = "an example that does not build is a bug in it"
)]

use std::collections::BTreeMap;

use geop_core_editor::registry::{Example, ExampleFiles, ExampleSet};
use geop_core_math::{
    primitives::{FrameAxis, Pose},
    vector::Vector3,
};
use geop_core_program::{
    operation_registry::OperationRegistry,
    part::Part,
    program::Program,
    runner::Runner,
    values::{Design, Setting, Settings, StateValue, StateValues, pose_state},
};
use geop_core_sketch::CurveId;
use geop_lib_stdlib::{Drawing, outline_plane};
use geop_ops_extrude_revolve::{
    Extent, Extents, Extrude, ExtrudeArgs, Output,
    examples::{box_with_drill_hole, parametric_plate},
};
use geop_ops_model::datums::{datum, origin_plane};
use geop_ops_model::sketch::{
    Sketch, SketchPlane,
    figures::{circle, n, rectangle, solved},
    whole_sketch,
};
use geop_ops_model::{Combine, PartDescription, element, read_model};
use geop_ops_reference_geometry::Selected;
use geop_ops_sketch::{AddSketch, AddSketchArgs};

use crate::{AddPartArgs, JointKind, Kind, Mate, MateEntity, Motion, joint_parameter};

/// The plane of the origin normal to `normal`, as a plane to sketch on.
fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
}

/// A pose of an example: at `position`, turned by `degrees` (see
/// [`Pose::from_euler`]) — as a file holds it ([`Pose::saved`]): where an
/// example puts a part to start from is a free choice, and one its file
/// can say saves and reads back as it is.
pub fn pose(position: [f64; 3], degrees: [f64; 3]) -> Pose<Design> {
    (Pose::from_euler(Vector3::from_array(position.map(n)), degrees.map(n)))
        .and_then(|pose| pose.saved())
        .expect("an example's rotation is one")
}

/// The part `program` builds on its own, written in sketches and
/// extrusions: what an example reads of a part it places.
fn built(program: &Program) -> Part {
    let registry = (OperationRegistry::default().with(AddSketch))
        .and_then(|r| r.with(Extrude))
        .expect("the operations each have a name of their own");
    let mut part = Part::default();
    (program.run(&Runner::new(registry), &mut part)).expect("the part builds");
    part
}

/// A pin: a circle of radius 0.4 around the origin, extruded 2 up — made to
/// fit the hole of [`box_with_drill_hole`].
pub fn pin() -> Program {
    let mut program = Program::new();
    let mut sketch = Sketch::new();
    circle(&mut sketch, [0.0, 0.0], 0.4);
    program.push(
        "pin_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), solved(sketch)),
    );
    program.push(
        "pin",
        ExtrudeArgs {
            sketch: whole_sketch("pin_sketch"),
            extent: Extents::blind(2.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// The assembly of `pin_in_plate`: the plate (`plate.geop`, the box of
/// [`box_with_drill_hole`]) placed fixed, and the pin (`pin.geop`, [`pin`])
/// mated into its hole — its side on the hole's axis, its bottom on the
/// hole's. Its state puts both where those mates hold them.
pub fn pin_in_plate_assembly() -> Program {
    let mut program = Program::new();
    program.push(
        "plate",
        AddPartArgs {
            file: "plate.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    let mate = |kind, [a, b]: [&str; 2]| {
        Mate::constraint(kind, vec![placed_face("pin", a), placed_face("plate", b)])
    };
    program.push(
        "pin",
        AddPartArgs {
            file: "pin.geop".into(),
            mates: BTreeMap::from([
                (
                    "m1".into(),
                    mate(
                        Kind::Concentric,
                        ["extrude(pin,pin_sketch,c1)", "extrude(hole,hole_sketch,c1)"],
                    ),
                ),
                (
                    "m2".into(),
                    mate(
                        Kind::Coincident,
                        ["extrude(pin,start)", "extrude(hole,end)"],
                    ),
                ),
            ]),
            ..Default::default()
        },
    );
    let pose = |position| StateValue::Pose(pose(position, [0.0; 3]));
    program.state = StateValues::from([
        (pose_state("plate"), pose([0.0; 3])),
        (pose_state("pin"), pose([1.0, 1.0, 0.5])),
    ]);
    program
}

/// A link's sketch: a 4 x 1 bar around the origin's `x` axis, from -0.5
/// to 3.5, with a hole of radius 0.3 at each end, 3 apart — and the holes.
/// A slotted bar `length` long between the centers of the holes at its
/// ends — the near one at the origin, the far one along `x` — and its
/// holes, near one first. Built alike for every length, so its entities are
/// named alike.
fn link_sketch(length: f64) -> (Sketch, [CurveId; 2]) {
    let mut sketch = Sketch::new();
    rectangle(&mut sketch, [-0.5, -0.5], length + 1.0, 1.0);
    let near = circle(&mut sketch, [0.0, 0.0], 0.3);
    let far = circle(&mut sketch, [length, 0.0], 0.3);
    (solved(sketch), [near, far])
}

/// A bar: [`link_sketch`] `length` long, extruded 0.2 up, to be linked by
/// its holes.
pub fn bar(length: f64) -> Program {
    let mut program = Program::new();
    program.push(
        "link_sketch",
        AddSketchArgs::on(on_origin(FrameAxis::Z), link_sketch(length).0),
    );
    program.push(
        "link",
        ExtrudeArgs {
            sketch: whole_sketch("link_sketch"),
            extent: Extents::blind(0.2),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program
}

/// The link of the chain: a [`bar`] 3 long.
pub fn link() -> Program {
    bar(3.0)
}

/// The face `name` of the part placed as `instance`, as a mate holds it.
fn placed_face(instance: &str, name: impl Into<String>) -> MateEntity {
    MateEntity::of(instance, Selected::Face(element(name)))
}

/// The faces of the holes of a [`bar`] placed as `instance`, near one first.
pub fn link_holes(instance: &str) -> [MateEntity; 2] {
    (link_sketch(3.0).1)
        .map(|hole| placed_face(instance, format!("extrude(link,link_sketch,{hole})")))
}

/// The [`bar`] placed as `above` lies on the one placed as `below`: its
/// bottom face on the other's top — so bars linked in a stack never cut
/// into each other.
fn on_top(above: &str, below: &str) -> Mate {
    Mate::constraint(
        Kind::Coincident,
        vec![
            placed_face(above, "extrude(link,start)"),
            placed_face(below, "extrude(link,end)"),
        ],
    )
}

/// The hole `hole` of the bar placed as `link` on the axis of the hole `pin`
/// of the one placed as `on`.
fn pinned(link: &str, hole: usize, on: &str, pin: usize) -> Mate {
    Mate::constraint(
        Kind::Concentric,
        vec![link_holes(on)[pin].clone(), link_holes(link)[hole].clone()],
    )
}

/// A part placed from `file`, held by `mates`.
fn placed(file: &str, fixed: bool, mates: Vec<Mate>) -> AddPartArgs {
    let args = AddPartArgs {
        file: file.into(),
        mates: mates
            .into_iter()
            .enumerate()
            .map(|(i, mate)| (format!("m{}", i + 1), mate))
            .collect(),
        ..Default::default()
    };
    if fixed { args.fixed() } else { args }
}

/// The assembly of `chain`: three links (`link.geop`, [`link`]), the first
/// fixed, each next one's near hole on the axis of the one before's far
/// hole, lying on it — and nothing else: each turns about its pin.
pub fn chain_assembly() -> Program {
    let mut program = Program::new();
    program.push("link1", placed("link.geop", true, Vec::new()));
    for (before, link) in [("link1", "link2"), ("link2", "link3")] {
        program.push(
            link,
            placed(
                "link.geop",
                false,
                vec![pinned(link, 0, before, 1), on_top(link, before)],
            ),
        );
    }
    let pose = |x, z| StateValue::Pose(pose([x, 0.0, z], [0.0; 3]));
    program.state = StateValues::from([
        (pose_state("link1"), pose(0.0, 0.0)),
        (pose_state("link2"), pose(3.0, 0.2)),
        (pose_state("link3"), pose(6.0, 0.4)),
    ]);
    program
}

/// The circular edge of the hole `hole` (0 near, 1 far) of the [`link`]
/// placed as `instance`, on its top (`end`) or bottom (`start`) face — and
/// the frame it gives: at its center, its `z` along its axis, what a click
/// on the rim of a hole picks.
pub fn link_rim(instance: &str, hole: usize, cap: &str) -> MateEntity {
    let name = format!(
        "extrude(link,link_sketch,{},{cap})",
        link_sketch(3.0).1[hole]
    );
    MateEntity::of(instance, Selected::Edge(element(name)))
}

/// The [`link`] placed as `link` turns on the one placed as `on`: a
/// revolute joint from the frame on the rim of `on`'s far hole on its top
/// to the frame on the rim of `link`'s near hole on its bottom — so it lies
/// on it, pinned there — turning from `min` to `max` degrees.
fn hinged(link: &str, on: &str, min: f64, max: f64) -> Mate {
    Mate::joint(
        JointKind::Revolute {
            min: Some(n(min)),
            max: Some(n(max)),
        },
        vec![link_rim(on, 1, "end"), link_rim(link, 0, "start")],
    )
}

/// The assembly of `arm`: three links (`link.geop`, [`link`]) — the first
/// fixed, each next one hinged on the far end of the one before by a
/// revolute joint (see [`hinged`]), the elbow turning at most 150° either
/// way, the wrist 120°. Its state sets the joints — 30° and -45° — and puts
/// the links where those joints have them.
pub fn arm_assembly() -> Program {
    let mut program = Program::new();
    program.push("upper", placed("link.geop", true, Vec::new()));
    program.push(
        "fore",
        placed(
            "link.geop",
            false,
            vec![hinged("fore", "upper", -150.0, 150.0)],
        ),
    );
    program.push(
        "hand",
        placed(
            "link.geop",
            false,
            vec![hinged("hand", "fore", -120.0, 120.0)],
        ),
    );
    let (elbow, wrist) = (30.0_f64, -45.0_f64);
    let turned = |angle: f64| {
        [
            3.0 * angle.to_radians().cos(),
            3.0 * angle.to_radians().sin(),
        ]
    };
    let fore = [3.0, 0.0];
    let [dx, dy] = turned(elbow);
    let hand = [fore[0] + dx, fore[1] + dy];
    let at = |[x, y]: [f64; 2], z, turn| StateValue::Pose(pose([x, y, z], [0.0, 0.0, turn]));
    program.state = StateValues::from([
        (pose_state("upper"), at([0.0, 0.0], 0.0, 0.0)),
        (pose_state("fore"), at(fore, 0.2, elbow)),
        (pose_state("hand"), at(hand, 0.4, elbow + wrist)),
        (
            joint_parameter("add_part(fore,m1)", Motion::Turn),
            StateValue::Number(n(elbow)),
        ),
        (
            joint_parameter("add_part(hand,m1)", Motion::Turn),
            StateValue::Number(n(wrist)),
        ),
    ]);
    program
}

/// The assembly of `four_bar`: a four-bar linkage of [`bar`]s — the ground
/// (4 long, fixed), the crank (1.5) and the rocker (3) each pinned to one end
/// of it and lying on it, and the coupler (4) pinned to their far ends,
/// lying on the crank. The crank is the shortest bar and
/// `1.5 + 4 ≤ 4 + 3`, so it turns all the way round while the rocker rocks.
pub fn four_bar_assembly() -> Program {
    let mut program = Program::new();
    program.push("ground", placed("ground.geop", true, Vec::new()));
    program.push(
        "crank",
        placed(
            "crank.geop",
            false,
            vec![pinned("crank", 0, "ground", 0), on_top("crank", "ground")],
        ),
    );
    program.push(
        "rocker",
        placed(
            "rocker.geop",
            false,
            vec![pinned("rocker", 0, "ground", 1), on_top("rocker", "ground")],
        ),
    );
    program.push(
        "coupler",
        placed(
            "coupler.geop",
            false,
            vec![
                pinned("coupler", 0, "crank", 1),
                pinned("coupler", 1, "rocker", 1),
                on_top("coupler", "crank"),
            ],
        ),
    );
    // The crank at 60°, and where that puts the others (worked out from the
    // bars' lengths; the mates hold them there exactly).
    let pose = |position, turn| StateValue::Pose(pose(position, [0.0, 0.0, turn]));
    program.state = StateValues::from([
        (pose_state("ground"), pose([0.0, 0.0, 0.0], 0.0)),
        (pose_state("crank"), pose([0.0, 0.0, 0.2], 60.0)),
        (
            pose_state("rocker"),
            pose([4.0, 0.0, 0.2], 82.690_722_887_668_12),
        ),
        (
            pose_state("coupler"),
            pose([0.75, 1.299_038_105_676_658, 0.4], 24.780_674_143_948_414),
        ),
    ]);
    program
}

/// Two of [`parametric_plate`], placed with other values: one as it is,
/// one 5 wide with an M6 hole, in blue.
pub fn plates_assembly() -> Program {
    let mut program = Program::new();
    program.push(
        "small",
        AddPartArgs {
            file: "plate.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    program.push(
        "large",
        AddPartArgs {
            file: "plate.geop".into(),
            parameters: Settings::from([
                ("width".to_string(), Setting::Number(5.0)),
                ("screw".to_string(), Setting::Text("M6".into())),
                ("color".to_string(), Setting::Text("#3e7bd0".into())),
            ]),
            ..Default::default()
        }
        .fixed(),
    );
    program.state = StateValues::from([
        (
            pose_state("small"),
            StateValue::Pose(pose([0.0; 3], [0.0; 3])),
        ),
        (
            pose_state("large"),
            StateValue::Pose(pose([0.0, 3.0, 0.0], [0.0; 3])),
        ),
    ]);
    program
}

/// A plate 40 x 40 x 5 with a 4.5 hole through it at `(20, 20)` — in
/// millimetres, as the standard parts are: clearance for an M4 screw.
pub fn metric_plate() -> Program {
    let none = Program::new();
    let corner = |x: &str, y: &str| [x.to_string(), y.to_string()];
    let mut outline = Drawing::new(&none).expect("no parameters to resolve");
    let mut hole = Drawing::new(&none).expect("no parameters to resolve");
    outline
        .polygon(&[
            corner("0", "0"),
            corner("40", "0"),
            corner("40", "40"),
            corner("0", "40"),
        ])
        .expect("the corners are numbers");
    let centre = hole.point("20", "20").expect("the centre is numbers");
    hole.circle(centre, "4.5")
        .expect("the diameter is a number");
    let mut program = Program::new();
    program.push("outline", outline.on(outline_plane()));
    program.push(
        "plate",
        ExtrudeArgs {
            sketch: whole_sketch("outline"),
            extent: Extents::blind(5.0),
            output: Output::Solid(Combine::NewBody),
        },
    );
    program.push("hole_sketch", hole.on(outline_plane()));
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: Extents {
                side1: Extent::ThroughAll,
                ..Extents::blind(0.0)
            },
            output: Output::Solid(Combine::cut("extrude(plate)")),
        },
    );
    program
}

/// The name of the face of [`metric_plate`]'s hole.
pub fn metric_plate_hole() -> String {
    let plate = built(&metric_plate());
    let part = read_model(&plate.current()).expect("the plate has a model");
    PartDescription::of(&part)
        .expect("the plate describes itself")
        .faces
        .into_keys()
        .find(|name| name.starts_with("extrude(hole,hole_sketch,"))
        .expect("the hole has a face")
}

/// [`metric_plate`] (`plate.geop`) bolted together from standard parts: an
/// ISO 4762 M4 x 12 cap screw down its hole, and an ISO 4032 M4 nut under
/// it — each mated by its datums `axis` and `seat` or `base`.
pub fn bolted_plate() -> Program {
    let mut program = Program::new();
    program.push(
        "plate",
        AddPartArgs {
            file: "plate.geop".into(),
            ..Default::default()
        }
        .fixed(),
    );
    let mate = |kind, a: MateEntity, b: MateEntity| Mate::constraint(kind, vec![a, b]);
    let datum_of = |part: &str, name: &str| MateEntity::of(part, Selected::Datum(datum(name)));
    let size = |row: &str| Settings::from([("size".to_string(), Setting::Text(row.into()))]);
    program.push(
        "screw",
        AddPartArgs {
            file: "std:iso4762_socket_head_cap_screw.geop".into(),
            parameters: size("M4x12"),
            mates: BTreeMap::from([
                (
                    "m1".into(),
                    mate(
                        Kind::Concentric,
                        datum_of("screw", "axis"),
                        placed_face("plate", metric_plate_hole()),
                    ),
                ),
                (
                    "m2".into(),
                    mate(
                        Kind::Coincident,
                        datum_of("screw", "seat"),
                        placed_face("plate", "extrude(plate,end)"),
                    ),
                ),
            ]),
        },
    );
    program.push(
        "nut",
        AddPartArgs {
            file: "std:iso4032_hex_nut.geop".into(),
            parameters: size("M4"),
            mates: BTreeMap::from([
                (
                    "m1".into(),
                    mate(
                        Kind::Concentric,
                        datum_of("nut", "axis"),
                        datum_of("screw", "axis"),
                    ),
                ),
                (
                    "m2".into(),
                    mate(
                        Kind::Coincident,
                        datum_of("nut", "base"),
                        placed_face("plate", "extrude(plate,start)"),
                    ),
                ),
            ]),
        },
    );
    program.state = StateValues::from([
        (
            pose_state("plate"),
            StateValue::Pose(pose([0.0; 3], [0.0; 3])),
        ),
        (
            pose_state("screw"),
            StateValue::Pose(pose([20.0, 20.0, 5.0], [0.0; 3])),
        ),
        (
            pose_state("nut"),
            StateValue::Pose(pose([20.0, 20.0, 0.0], [180.0, 0.0, 0.0])),
        ),
    ]);
    program
}

/// An example drawn in the editor, kept as the program it saved (in
/// `src/drawn/`): its sketches as they were drawn, which no code here says
/// better. Its tests build it like every other example, so a change to the
/// format that breaks it is found there: save it again from the editor.
fn drawn(text: &str) -> Program {
    Program::from_json(text).expect("a drawn example is a program")
}

/// The examples of assemblies, as the editor offers them: the parts they
/// place that are examples of their own ([`pin`], [`link`]), and the
/// examples made of several files ([`workspaces`]).
pub fn set() -> ExampleSet {
    ExampleSet {
        programs: vec![("pin", pin), ("link", link)],
        workspaces: workspaces(),
    }
}

/// Every example made of several files, by name, each with what makes its
/// files: listing them builds nothing.
pub fn workspaces() -> Vec<Example<ExampleFiles>> {
    vec![
        ("press", || {
            vec![
                ("asm.geop", drawn(include_str!("drawn/press/asm.geop"))),
                ("base.geop", drawn(include_str!("drawn/press/base.geop"))),
                (
                    "handle.geop",
                    drawn(include_str!("drawn/press/handle.geop")),
                ),
                (
                    "slider.geop",
                    drawn(include_str!("drawn/press/slider.geop")),
                ),
                ("link.geop", drawn(include_str!("drawn/press/link.geop"))),
            ]
        }),
        ("pin_in_plate", || {
            vec![
                ("assembly.geop", pin_in_plate_assembly()),
                ("plate.geop", box_with_drill_hole()),
                ("pin.geop", pin()),
            ]
        }),
        ("chain", || {
            vec![("chain.geop", chain_assembly()), ("link.geop", link())]
        }),
        ("parametric_plates", || {
            vec![
                ("plates.geop", plates_assembly()),
                ("plate.geop", parametric_plate()),
            ]
        }),
        ("four_bar", || {
            vec![
                ("four_bar.geop", four_bar_assembly()),
                ("ground.geop", bar(4.0)),
                ("crank.geop", bar(1.5)),
                ("rocker.geop", bar(3.0)),
                ("coupler.geop", bar(4.0)),
            ]
        }),
        ("arm", || {
            vec![("arm.geop", arm_assembly()), ("link.geop", link())]
        }),
        ("bolted_plate", || {
            vec![
                ("bolted_plate.geop", bolted_plate()),
                ("plate.geop", metric_plate()),
            ]
        }),
    ]
}
