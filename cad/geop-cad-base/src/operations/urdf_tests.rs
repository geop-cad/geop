//! URDF export: assemblies built from the examples' parts, exported as
//! robots, and read back — as a tree of XML elements, and by placing every
//! link where its joints put it.

use std::collections::{BTreeMap, HashMap};

use geop_core_math::{
    primitives::Pose,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_ops::{
    Part,
    assembly::{CouplingKind, JointKind, Mate},
    part::{ParamValue, pose_parameter},
};
use geop_ops_assembly::AddPartArgs;
use geop_ops_urdf::{JointType, Origin, Robot, URDF_FILE, UrdfExport, export};

use crate::examples::{self, n, pose};
use crate::{Program, Workspace, stdlib::WithStandardParts};

/// The files of the workspace example `name`, by path, and its first
/// program.
fn workspace_example(name: &str) -> (BTreeMap<String, String>, Program) {
    let files = examples::workspaces()
        .into_iter()
        .find(|(n, _)| *n == name)
        .unwrap()
        .1();
    let program = files[0].1.clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    (files, program)
}

/// `program`, saved as `path` among `files`, built.
fn build(files: BTreeMap<String, String>, path: &str, program: &Program) -> Part<S> {
    let workspace = Workspace::<S>::new(WithStandardParts(files));
    program.build(&workspace.scope(path)).unwrap()
}

/// The arm, exported.
fn arm() -> (Part<S>, UrdfExport) {
    let (files, program) = workspace_example("arm");
    let part = build(files, "arm.geop", &program);
    let robot = export(&part, "arm", 8).unwrap();
    (part, robot)
}

// --- A small XML reader, enough to read a URDF file back. ---

#[derive(Debug)]
struct Element {
    name: String,
    attributes: HashMap<String, String>,
    children: Vec<Element>,
}

impl Element {
    fn attr(&self, name: &str) -> &str {
        self.attributes
            .get(name)
            .unwrap_or_else(|| panic!("<{}> has no {name}: {self:?}", self.name))
    }

    fn all(&self, name: &str) -> Vec<&Element> {
        self.children.iter().filter(|c| c.name == name).collect()
    }

    fn child(&self, name: &str) -> &Element {
        self.all(name)
            .into_iter()
            .next()
            .unwrap_or_else(|| panic!("<{}> has no <{name}>", self.name))
    }
}

fn unescape(text: &str) -> String {
    text.replace("&lt;", "<")
        .replace("&gt;", ">")
        .replace("&quot;", "\"")
        .replace("&apos;", "'")
        .replace("&amp;", "&")
}

/// The document element of `xml`: declarations, comments and text skipped,
/// every element closed by its own name.
fn parse_xml(xml: &str) -> Element {
    let mut stack = vec![Element {
        name: String::new(),
        attributes: HashMap::new(),
        children: Vec::new(),
    }];
    let mut rest = xml;
    while let Some(start) = rest.find('<') {
        rest = &rest[start..];
        if let Some(after) = rest.strip_prefix("<!--") {
            rest = &after[after.find("-->").expect("a comment ends") + 3..];
            continue;
        }
        let end = rest.find('>').expect("a tag ends");
        let tag = &rest[1..end];
        rest = &rest[end + 1..];
        if tag.starts_with('?') {
            continue;
        }
        if let Some(name) = tag.strip_prefix('/') {
            let done = stack.pop().unwrap();
            assert_eq!(done.name, name.trim(), "closed by its own name");
            stack.last_mut().unwrap().children.push(done);
            continue;
        }
        let closed = tag.ends_with('/');
        let tag = tag.trim_end_matches('/');
        let (name, mut attrs) = tag.split_once(char::is_whitespace).unwrap_or((tag, ""));
        let mut attributes = HashMap::new();
        while let Some(eq) = attrs.find('=') {
            let key = attrs[..eq].trim().to_string();
            let open = attrs[eq..].find('"').unwrap() + eq + 1;
            let close = attrs[open..].find('"').unwrap() + open;
            attributes.insert(key, unescape(&attrs[open..close]));
            attrs = &attrs[close + 1..];
        }
        let element = Element {
            name: name.to_string(),
            attributes,
            children: Vec::new(),
        };
        if closed {
            stack.last_mut().unwrap().children.push(element);
        } else {
            stack.push(element);
        }
    }
    assert_eq!(stack.len(), 1, "every element is closed");
    let mut document = stack.pop().unwrap();
    assert_eq!(document.children.len(), 1, "one document element");
    document.children.pop().unwrap()
}

// --- Placing the links where their joints put them. ---

/// A URDF origin as a pose, in millimetres.
fn pose_of_origin(origin: &Origin) -> Pose<S> {
    Pose::from_euler(
        Vector3::from_array(origin.xyz.map(|x| S::from_f64(x * 1000.0))),
        origin.rpy.map(|r| S::from_f64(r.to_degrees())),
    )
    .unwrap()
}

/// Every link's frame in the world, its joints at `coordinates` — radians
/// and metres, by joint name; zero where not given.
fn link_frames(robot: &Robot, coordinates: &HashMap<String, f64>) -> HashMap<String, Pose<S>> {
    let mut frames = HashMap::from([(robot.links[0].name.clone(), Pose::identity())]);
    for joint in &robot.joints {
        let q = coordinates.get(&joint.name).copied().unwrap_or(0.0);
        let axis = Vector3::from_array(joint.axis.map(S::from_f64));
        let motion = match joint.kind {
            JointType::Prismatic { .. } => {
                Pose::identity().with_position(axis.prod_scalar(S::from_f64(q * 1000.0)))
            }
            _ => Pose::rotation_about(&Vector3::zero(), &axis, S::from_f64(q)).unwrap(),
        };
        let parent = frames[&joint.parent];
        frames.insert(
            joint.child.clone(),
            parent
                .compose(&pose_of_origin(&joint.origin))
                .compose(&motion),
        );
    }
    frames
}

#[track_caller]
fn assert_same_pose(a: &Pose<S>, b: &Pose<S>, what: &str) {
    for p in [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ] {
        let p = Vector3::from_array(p.map(S::from_f64));
        let (pa, pb) = (a.apply(&p), b.apply(&p));
        for k in 0..3 {
            let (x, y) = (pa[k].to_f64(), pb[k].to_f64());
            assert!((x - y).abs() < 1e-9, "{what}: {pa:?} is not {pb:?}");
        }
    }
}

/// Placed by its joints at the coordinates the assembly is at, every part
/// of `robot` is where `part` has it.
fn assert_links_where_drawn(part: &Part<S>, robot: &Robot) {
    let mechanism = part.mechanism().unwrap();
    let mut coordinates = HashMap::new();
    for (j, joint) in mechanism.assembly.joints.iter().enumerate() {
        let q = match joint.kind {
            JointKind::Slider { .. } => joint.distance.value.to_f64() / 1000.0,
            _ => joint.angle.value.to_f64().to_radians(),
        };
        coordinates.insert(mechanism.joint_name(j).to_string(), q);
    }
    let frames = link_frames(robot, &coordinates);
    for link in &robot.links {
        for visual in &link.visuals {
            let Some(body) = mechanism.bodies.iter().find(|b| b.name == visual.name) else {
                continue;
            };
            let drawn = frames[&link.name].compose(&pose_of_origin(&visual.origin));
            assert_same_pose(&drawn, &body.world, &visual.name);
        }
    }
}

/// The arm exports as a robot of three links hanging from the fixed upper
/// arm: its two revolute joints with their limits, each link the mass the
/// inspector weighs its part at, and — at the angles the arm is at, 30°
/// and -45° — every link where it is drawn.
#[test]
fn the_arm_exports_its_joints_limits_and_masses() {
    let (part, exported) = arm();
    let robot = &exported.robot;
    let names: Vec<&str> = robot.links.iter().map(|l| l.name.as_str()).collect();
    assert_eq!(names, ["upper", "fore", "hand"]);
    let joints: Vec<(&str, &str, &str, JointType)> = robot
        .joints
        .iter()
        .map(|j| (j.name.as_str(), j.parent.as_str(), j.child.as_str(), j.kind))
        .collect();
    let limits = |degrees: f64| JointType::Revolute {
        lower: (-degrees).to_radians(),
        upper: degrees.to_radians(),
    };
    assert_eq!(
        joints,
        [
            ("add_part(fore,m1)", "upper", "fore", limits(150.0)),
            ("add_part(hand,m1)", "fore", "hand", limits(120.0)),
        ]
    );
    for joint in &robot.joints {
        assert_eq!(joint.axis, [0.0, 0.0, 1.0]);
        assert!(joint.mimic.is_none());
    }

    let report = geop_ops_inspect::mass_report(&part).unwrap();
    for link in &robot.links {
        let prefix = format!("{}/", link.name);
        let weighed: f64 = report
            .bodies
            .iter()
            .filter(|b| b.name.starts_with(&prefix))
            .map(|b| b.properties.as_ref().unwrap().mass.value)
            .sum();
        let inertial = link.inertial.unwrap();
        assert!(weighed > 0.0);
        assert!(
            ((inertial.mass - weighed) / weighed).abs() < 1e-12,
            "{}: {} kg, weighed {weighed} kg",
            link.name,
            inertial.mass
        );
        // A bar's inertia, about its centre along its own axes: its
        // principal moments, the largest about its normal, `z`.
        let i = inertial.inertia;
        assert!(i[2][2] > i[0][0] && i[2][2] > i[1][1], "{i:?}");
    }
    // The root's frame is the assembly's: its centre of mass is where the
    // inspector puts the upper arm's, in metres.
    let upper = report.bodies.iter().find(|b| b.name.starts_with("upper/"));
    let center = upper.unwrap().properties.as_ref().unwrap().center;
    let root = robot.links[0].inertial.unwrap().center;
    for k in 0..3 {
        assert!(
            (root[k] - center[k].value / 1000.0).abs() < 1e-12,
            "{root:?}"
        );
    }

    // One mesh, of the link the three parts are placed from.
    let meshes: Vec<&str> = exported.meshes.iter().map(|(p, _)| p.as_str()).collect();
    assert_eq!(meshes, ["meshes/link.stl"]);
    assert!(robot.links.iter().all(|l| l.visuals.len() == 1));
    assert_links_where_drawn(&part, robot);
}

/// The arm's URDF, read back as XML: a robot whose links form one tree from
/// the upper arm, each joint naming links there are, with an origin, an
/// axis and its limits; every link with its inertia and its mesh — a file
/// of the export.
#[test]
fn the_arm_urdf_reads_back_as_a_tree() {
    let (_, exported) = arm();
    let files = exported.files();
    assert_eq!(files[0].0, URDF_FILE);
    let xml = String::from_utf8(files[0].1.clone()).unwrap();
    let robot = parse_xml(&xml);
    assert_eq!(robot.name, "robot");
    assert_eq!(robot.attr("name"), "arm");
    let links: Vec<&str> = robot
        .all("link")
        .into_iter()
        .map(|l| l.attr("name"))
        .collect();
    assert_eq!(links, ["upper", "fore", "hand"]);
    let mut parent_of = HashMap::new();
    for joint in robot.all("joint") {
        assert_eq!(joint.attr("type"), "revolute");
        let parent = joint.child("parent").attr("link");
        let child = joint.child("child").attr("link");
        assert!(links.contains(&parent) && links.contains(&child));
        assert!(parent_of.insert(child, parent).is_none(), "one parent each");
        joint.child("origin").attr("xyz");
        assert_eq!(joint.child("axis").attr("xyz"), "0 0 1");
        let limit = joint.child("limit");
        let lower: f64 = limit.attr("lower").parse().unwrap();
        let upper: f64 = limit.attr("upper").parse().unwrap();
        assert!(lower < 0.0 && upper > 0.0 && (lower + upper).abs() < 1e-12);
        limit.attr("effort");
        limit.attr("velocity");
    }
    // One root, and every link reaches it.
    let roots: Vec<&&str> = links
        .iter()
        .filter(|l| !parent_of.contains_key(*l))
        .collect();
    assert_eq!(roots, [&"upper"]);
    for link in &links {
        let mut at = *link;
        for _ in 0..links.len() {
            at = parent_of.get(at).copied().unwrap_or(at);
        }
        assert_eq!(at, "upper");
    }
    for link in robot.all("link") {
        let inertial = link.child("inertial");
        let mass: f64 = inertial.child("mass").attr("value").parse().unwrap();
        assert!(mass > 0.0);
        let inertia = inertial.child("inertia");
        for k in ["ixx", "ixy", "ixz", "iyy", "iyz", "izz"] {
            inertia.attr(k).parse::<f64>().unwrap();
        }
        for element in ["visual", "collision"] {
            let mesh = link.child(element).child("geometry").child("mesh");
            assert_eq!(mesh.attr("scale"), "0.001 0.001 0.001");
            let file = mesh.attr("filename");
            let (_, bytes) = files.iter().find(|(p, _)| p == file).unwrap();
            // A binary STL: a header, a count, 50 bytes per triangle.
            let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
            assert!(count > 0);
            assert_eq!(bytes.len(), 84 + 50 * count);
        }
    }
}

/// The joint of `link`'s near hole, on its bottom, on the hole `hole` of
/// the bar placed as `on`, on its top: turning all the way round.
fn pin_joint(on: &str, hole: usize, link: &str, link_hole: usize) -> Mate {
    Mate::joint(
        JointKind::Revolute {
            min: None,
            max: None,
        },
        vec![
            examples::link_rim(on, hole, "end"),
            examples::link_rim(link, link_hole, "start"),
        ],
    )
}

fn placed(file: &str, fixed: bool, mates: Vec<Mate>) -> AddPartArgs {
    AddPartArgs {
        file: file.into(),
        fixed,
        mates: mates
            .into_iter()
            .enumerate()
            .map(|(i, mate)| (format!("m{}", i + 1), mate))
            .collect(),
        ..Default::default()
    }
}

/// The four-bar of the examples, its bars pinned by revolute joints rather
/// than by mates: a closed loop, refused, naming its four joints.
#[test]
fn a_four_bar_is_refused_naming_its_loop() {
    let (files, example) = workspace_example("four_bar");
    let mut program = Program::new();
    program.push("ground", placed("ground.geop", true, Vec::new()));
    program.push(
        "crank",
        placed(
            "crank.geop",
            false,
            vec![pin_joint("ground", 0, "crank", 0)],
        ),
    );
    program.push(
        "rocker",
        placed(
            "rocker.geop",
            false,
            vec![pin_joint("ground", 1, "rocker", 0)],
        ),
    );
    program.push(
        "coupler",
        placed(
            "coupler.geop",
            false,
            vec![
                pin_joint("crank", 1, "coupler", 0),
                pin_joint("rocker", 1, "coupler", 1),
            ],
        ),
    );
    program.state = example
        .state
        .clone()
        .into_iter()
        .filter(|(k, _)| k.ends_with(".pose"))
        .collect();
    let part = build(files.clone(), "four_bar.geop", &program);
    assert!(part.check_mates(|_| true).unwrap().converged);
    let err = export(&part, "four_bar", 8).unwrap_err().to_string();
    for joint in [
        "add_part(crank,m1)",
        "add_part(rocker,m1)",
        "add_part(coupler,m1)",
        "add_part(coupler,m2)",
    ] {
        assert!(err.contains(joint), "{joint}: {err}");
    }
    assert!(err.contains("close a loop through"), "{err}");
    assert!(err.contains("ground") && err.contains("coupler"), "{err}");

    // As the example has it, pinned by mates, it is no robot either: the
    // mates leave the bars free to move, which only a joint can say.
    let part = build(files, "four_bar.geop", &example);
    let err = export(&part, "four_bar", 8).unwrap_err().to_string();
    assert!(err.contains("free to move"), "{err}");
    assert!(
        err.contains("crank") && err.contains("add_part(crank,m1)"),
        "{err}"
    );
}

/// Two links on joints at the ends of a fixed one, coupled as gears 2:1
/// turning opposite ways: continuous joints, the driven one mimicking the
/// driver at -1/2.
#[test]
fn a_gear_coupling_exports_as_a_mimic_joint() {
    let (files, _) = workspace_example("arm");
    let mut program = Program::new();
    program.push("base", placed("link.geop", true, Vec::new()));
    program.push(
        "driver",
        placed("link.geop", false, vec![pin_joint("base", 0, "driver", 0)]),
    );
    let gear = Mate::coupling(
        CouplingKind::Gear {
            ratio: n(2.0),
            reverse: true,
        },
        vec!["add_part(driver,m1)".into(), "add_part(driven,m1)".into()],
    );
    program.push(
        "driven",
        placed(
            "link.geop",
            false,
            vec![pin_joint("base", 1, "driven", 0), gear],
        ),
    );
    let at = |x: f64, turn: f64| ParamValue::Pose(pose([x, 0.0, 0.2], [0.0, 0.0, turn]));
    program.state = BTreeMap::from([
        (
            pose_parameter("base"),
            ParamValue::Pose(pose([0.0; 3], [0.0; 3])),
        ),
        (pose_parameter("driver"), at(0.0, 90.0)),
        (pose_parameter("driven"), at(3.0, -45.0)),
        (
            "add_part(driver,m1).angle".into(),
            ParamValue::Number(n(90.0)),
        ),
        (
            "add_part(driven,m1).angle".into(),
            ParamValue::Number(n(-45.0)),
        ),
    ]);
    let part = build(files, "gears.geop", &program);
    let robot = export(&part, "gears", 8).unwrap().robot;
    let driven = robot.joint("add_part(driven,m1)").unwrap();
    assert_eq!(driven.kind, JointType::Continuous);
    let mimic = driven.mimic.as_ref().unwrap();
    assert_eq!(mimic.joint, "add_part(driver,m1)");
    assert!((mimic.multiplier + 0.5).abs() < 1e-15, "{mimic:?}");
    assert!(robot.joint("add_part(driver,m1)").unwrap().mimic.is_none());
    assert!(
        robot
            .to_urdf()
            .contains(r#"<mimic joint="add_part(driver,m1)" multiplier="-0.5""#)
    );
    assert_links_where_drawn(&part, &robot);
}

/// The arm's export, loaded by an independent URDF reader — Python's
/// `yourdfpy` — which validates it, finds its meshes, and puts the hand
/// where the assembly draws it at the arm's angles.
#[test]
#[ignore = "external: needs Python's yourdfpy (`pip install --user yourdfpy`) — run with `cargo test -- --ignored`"]
fn the_arm_loads_in_yourdfpy() {
    let (part, exported) = arm();
    let dir = std::env::temp_dir().join(format!("geop-urdf-{}", std::process::id()));
    for (file, bytes) in exported.files() {
        let path = dir.join(file);
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, bytes).unwrap();
    }
    let script = r#"
import sys, math, yourdfpy
robot = yourdfpy.URDF.load(sys.argv[1], build_scene_graph=True, load_meshes=True)
assert robot.validate(), "invalid"
robot.update_cfg([math.radians(30), math.radians(-45)])
print(" ".join(str(x) for x in robot.get_transform("hand")[:3, 3]))
"#;
    let output = std::process::Command::new("python3")
        .arg("-c")
        .arg(script)
        .arg(dir.join(URDF_FILE))
        .output()
        .expect("python3 runs");
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let text = String::from_utf8(output.stdout).unwrap();
    let hand: Vec<f64> = text
        .split_whitespace()
        .map(|x| x.parse().unwrap())
        .collect();
    // Where our own reading of the file puts the hand's frame — which
    // the fast tests check against the assembly.
    let joint = exported.robot.joint("add_part(hand,m1)").unwrap();
    let frames = link_frames(
        &exported.robot,
        &HashMap::from([
            ("add_part(fore,m1)".to_string(), 30f64.to_radians()),
            (joint.name.clone(), (-45f64).to_radians()),
        ]),
    );
    let ours = frames["hand"].position();
    for k in 0..3 {
        assert!(
            (hand[k] - ours[k].to_f64() / 1000.0).abs() < 1e-12,
            "{hand:?}"
        );
    }
    assert_links_where_drawn(&part, &exported.robot);
}

/// A slider lifting a link off a fixed base, a tool fastened on the
/// link's far end, and a link hinged on the base's far end by a joint
/// whose first end is on the link — the joint runs against the tree: a
/// prismatic joint with its travel in metres, the tool merged into the
/// slider's link, and the hinge turning the other way round its axis, so
/// that every part is still where it is drawn.
#[test]
fn sliders_fastened_parts_and_reversed_joints() {
    let (files, _) = workspace_example("arm");
    let mut program = Program::new();
    program.push("base", placed("link.geop", true, Vec::new()));
    let slider = Mate::joint(
        JointKind::Slider {
            min: Some(n(0.0)),
            max: Some(n(2.0)),
        },
        vec![
            examples::link_rim("base", 0, "end"),
            examples::link_rim("lift", 0, "start"),
        ],
    );
    program.push("lift", placed("link.geop", false, vec![slider]));
    let fastened = Mate::joint(
        JointKind::Fastened,
        vec![
            examples::link_rim("lift", 1, "end"),
            examples::link_rim("tool", 0, "start"),
        ],
    );
    program.push("tool", placed("link.geop", false, vec![fastened]));
    let hinge = Mate::joint(
        JointKind::Revolute {
            min: Some(n(-90.0)),
            max: Some(n(90.0)),
        },
        vec![
            examples::link_rim("swing", 0, "start"),
            examples::link_rim("base", 1, "end"),
        ],
    );
    program.push("swing", placed("link.geop", false, vec![hinge]));
    let at = |p: [f64; 3], turn: f64| ParamValue::Pose(pose(p, [0.0, 0.0, turn]));
    program.state = BTreeMap::from([
        (pose_parameter("base"), at([0.0; 3], 0.0)),
        (pose_parameter("lift"), at([0.0, 0.0, 0.7], 0.0)),
        (pose_parameter("tool"), at([3.0, 0.0, 0.9], 0.0)),
        (pose_parameter("swing"), at([3.0, 0.0, 0.2], 40.0)),
    ]);
    let part = build(files, "machine.geop", &program);
    assert!(part.check_mates(|_| true).unwrap().converged);
    let robot = export(&part, "machine", 8).unwrap().robot;

    let links: Vec<(&str, Vec<&str>)> = robot
        .links
        .iter()
        .map(|l| {
            (
                l.name.as_str(),
                l.parts.iter().map(|p| p.as_str()).collect(),
            )
        })
        .collect();
    assert_eq!(
        links,
        [
            ("base", vec!["base"]),
            ("lift", vec!["lift", "tool"]),
            ("swing", vec!["swing"]),
        ]
    );
    let lift = robot.joint("add_part(lift,m1)").unwrap();
    assert_eq!(
        lift.kind,
        JointType::Prismatic {
            lower: 0.0,
            upper: 0.002
        }
    );
    assert_eq!((lift.parent.as_str(), lift.axis), ("base", [0.0, 0.0, 1.0]));
    let swing = robot.joint("add_part(swing,m1)").unwrap();
    assert_eq!(
        (swing.parent.as_str(), swing.axis),
        ("base", [0.0, 0.0, -1.0])
    );
    // The tool weighs as much as the slider: both are links.
    let link = robot.link("lift").unwrap();
    let swing_mass = robot.link("swing").unwrap().inertial.unwrap().mass;
    let ratio = link.inertial.unwrap().mass / swing_mass;
    assert!((ratio - 2.0).abs() < 1e-12, "{ratio}");
    assert_eq!(link.visuals.len(), 2);
    assert_links_where_drawn(&part, &robot);
}

/// What a URDF joint cannot be is refused, naming the joint: a cylindrical
/// joint, a revolute joint limited one way, a slider without its travel;
/// and a part nothing joins to the fixed one, or no fixed part at all.
#[test]
fn what_urdf_cannot_express_is_refused_by_name() {
    let (files, _) = workspace_example("arm");
    let refusal = |kind: Option<JointKind<geop_ops::Design>>, fixed: bool| {
        let mut program = Program::new();
        program.push("base", placed("link.geop", fixed, Vec::new()));
        let mates = kind
            .map(|kind| {
                Mate::joint(
                    kind,
                    vec![
                        examples::link_rim("base", 1, "end"),
                        examples::link_rim("arm", 0, "start"),
                    ],
                )
            })
            .into_iter()
            .collect();
        program.push("arm", placed("link.geop", false, mates));
        let at = |x: f64, z: f64| ParamValue::Pose(pose([x, 0.0, z], [0.0; 3]));
        program.state = BTreeMap::from([
            (pose_parameter("base"), at(0.0, 0.0)),
            (pose_parameter("arm"), at(3.0, 0.2)),
        ]);
        let part = build(files.clone(), "bad.geop", &program);
        export(&part, "bad", 8).unwrap_err().to_string()
    };
    let joint = "\"add_part(arm,m1)\"";
    let err = refusal(Some(JointKind::Cylindrical), true);
    assert!(err.contains(joint) && err.contains("cylindrical"), "{err}");
    let one_way = JointKind::Revolute {
        min: None,
        max: Some(n(90.0)),
    };
    let err = refusal(Some(one_way), true);
    assert!(err.contains(joint) && err.contains("one way only"), "{err}");
    let endless = JointKind::Slider {
        min: Some(n(0.0)),
        max: None,
    };
    let err = refusal(Some(endless), true);
    assert!(err.contains(joint) && err.contains("its travel"), "{err}");
    let err = refusal(None, true);
    assert!(
        err.contains("arm is joined to the fixed part base by neither"),
        "{err}"
    );
    let err = refusal(None, false);
    assert!(err.contains("no part is fixed"), "{err}");
}
