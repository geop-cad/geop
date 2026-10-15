//! URDF export: an assembly as a robot description, for simulators — ROS,
//! Gazebo, MuJoCo, PyBullet, Isaac — to load ([`export`]), written by the
//! exporter [`Urdf`] as one ZIP archive of the robot and its meshes.
//!
//! A robot is a tree of **links** joined by **joints**. Its links are the
//! placed parts, those that mates hold rigidly together merged into one,
//! the root the fixed part (see [`tree`]); its joints are the assembly's
//! revolute joints (`revolute` with limits, `continuous` without) and
//! sliders (`prismatic`). A coupling between two of them — gears, a rack
//! and pinion, a screw — makes the second **mimic** the first.
//!
//! **Frames.** The root link's frame is the assembly's. Every other link's
//! frame is its joint's: origin on the axis, `z` along it, `x` the
//! direction turns are measured from — so every joint's axis is `z`, and a
//! joint's coordinate in the URDF is the assembly's own: at the angle the
//! assembly measures, the link is where it is drawn, and the limits are the
//! joint's limits. Joint origins come from the connectors where the
//! assembly is now; the exported pose is that of zero coordinates.
//!
//! **Units.** URDF is SI: lengths in metres, angles in radians, masses in
//! kilograms, inertia in kg·m². geop's lengths are millimetres, so meshes
//! are written in millimetres and scaled by `0.001`.
//!
//! **Inertia.** Each link's mass, centre of mass and inertia tensor come
//! from the exact B-rep of its solids, each of its own part's material
//! (see [`geop_ops_assembly::solids`]), the tensor about the centre of mass
//! along the link frame's axes.
//!
//! **Meshes.** Each placed component is meshed once, in its own frame, as
//! binary STL (`meshes/<file>.stl`), and is each link's `<visual>` and
//! `<collision>` where its parts are. geop models no actuators: `effort`
//! and `velocity`, which URDF requires of a limited joint, are written as
//! 0, as CAD exporters do — set them for the drives.
//!
//! What URDF cannot express is refused, by name: closed loops (a four-bar),
//! parts that mates leave free to move but no joint names, cylindrical
//! joints, limits one way only.

pub mod tree;
mod xml;
pub mod zip;

use geop_core_program::{
    placed::instance_path,
    reader::Reader,
    values::{Design, StateValue},
};
use std::collections::{BTreeMap, BTreeSet, HashMap};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, Quaternion},
    scalars::Scalar,
    solvers::walk::{Walked, walk},
    vector::Vector3,
};
use geop_core_topology::mass::MassProperties;
use geop_ops_assembly::{
    CouplingKind, JointKind, Motion, Resolved, STL_QUALITY, drawn, joint_parameter,
    joints::{Connector, measure},
    mechanism::mates_of,
    placed_parts,
    residuals::D,
    solids::{Shapes, own_solids_at},
    solving::{self, Rerun, check},
    stl_bytes,
};
use geop_ops_model::read_model;
use geop_ops_output::{Content, Export, Exporter, ExporterInfo, Output};
use geop_ops_parameters::stem;
use geop_ops_rasterize::stl::StlFormat;
use serde::Deserialize;

/// The URDF file's path in an export.
pub const URDF_FILE: &str = "robot.urdf";

/// A frame relative to another, as URDF writes it: moved by `xyz` (metres)
/// after turning by `rpy` (radians) — about `x`, then `y`, then `z`, the
/// fixed axes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Origin {
    pub xyz: [f64; 3],
    pub rpy: [f64; 3],
}

impl Origin {
    /// `pose`, a geop pose in millimetres.
    fn of<S: Scalar>(pose: &Pose<S>) -> Self {
        let p = pose.position();
        Origin {
            xyz: [0, 1, 2].map(|k| p[k].to_f64() / 1000.0),
            rpy: pose.euler_degrees().map(f64::to_radians),
        }
    }
}

/// A link's mass (kg), centre of mass in its frame (m), and inertia tensor
/// about the centre along its frame's axes (kg·m²).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inertial {
    pub mass: f64,
    pub center: [f64; 3],
    pub inertia: [[f64; 3]; 3],
}

/// One placed part of a link, drawn: its mesh, a path in the export, where
/// it is in the link's frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Visual {
    /// The placed part, by name — or `base` for the assembly's own solids.
    pub name: String,
    pub origin: Origin,
    pub mesh: String,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Link {
    pub name: String,
    /// The placed parts it is made of, by name.
    pub parts: Vec<String>,
    /// None for a link without solids.
    pub inertial: Option<Inertial>,
    pub visuals: Vec<Visual>,
}

/// What a joint lets its child do, and within what limits: radians for a
/// turn, metres for a slide.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointType {
    Revolute { lower: f64, upper: f64 },
    Continuous,
    Prismatic { lower: f64, upper: f64 },
}

impl JointType {
    pub fn name(&self) -> &'static str {
        match self {
            JointType::Revolute { .. } => "revolute",
            JointType::Continuous => "continuous",
            JointType::Prismatic { .. } => "prismatic",
        }
    }
}

/// A joint's coordinate following another's: `multiplier` times it.
#[derive(Clone, Debug, PartialEq)]
pub struct Mimic {
    pub joint: String,
    pub multiplier: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Joint {
    /// The joint mate's name in the assembly.
    pub name: String,
    pub kind: JointType,
    pub parent: String,
    pub child: String,
    /// The child link's frame, at a coordinate of zero, in the parent's.
    pub origin: Origin,
    /// Along the child's `z`, either way.
    pub axis: [f64; 3],
    pub mimic: Option<Mimic>,
}

/// A robot: its links, the root first, and its joints, each after the
/// joint carrying its parent.
#[derive(Clone, Debug, PartialEq)]
pub struct Robot {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}

impl Robot {
    pub fn link(&self, name: &str) -> Option<&Link> {
        self.links.iter().find(|l| l.name == name)
    }

    pub fn joint(&self, name: &str) -> Option<&Joint> {
        self.joints.iter().find(|j| j.name == name)
    }
}

/// A robot and the files it is written as.
#[derive(Clone, Debug)]
pub struct UrdfExport {
    pub robot: Robot,
    /// The meshes, by path: `meshes/link.stl`.
    pub meshes: Vec<(String, Vec<u8>)>,
}

impl UrdfExport {
    /// Every file: the URDF ([`URDF_FILE`]) first, then the meshes, by
    /// paths relative to it.
    pub fn files(&self) -> Vec<(String, Vec<u8>)> {
        let mut files = vec![(URDF_FILE.to_string(), self.robot.to_urdf().into_bytes())];
        files.extend(self.meshes.iter().cloned());
        files
    }

    /// Every file in one ZIP archive.
    pub fn zip(&self) -> GeopResult<Vec<u8>> {
        zip::zip(&self.files())
    }
}

/// Writes an assembly as a URDF robot: one ZIP archive of `robot.urdf` and
/// its meshes (see [`export`]).
pub struct Urdf;

/// How a URDF robot is written.
#[derive(Clone, Debug, PartialEq, Deserialize)]
#[serde(default)]
pub struct UrdfOptions {
    /// How finely curved faces are meshed: higher is smoother, and bigger.
    pub quality: usize,
}

impl Default for UrdfOptions {
    fn default() -> Self {
        Self {
            quality: STL_QUALITY,
        }
    }
}

impl Exporter for Urdf {
    const INFO: ExporterInfo = ExporterInfo {
        kind: "urdf",
        label: "URDF",
        doc: "robot, for simulators",
        formats: &["zip"],
    };
    type Options = UrdfOptions;

    fn export(output: &mut Output, _: &str, options: UrdfOptions) -> GeopResult<Export> {
        let name = output.stem()?;
        let robot = export(output, &name, options.quality)?;
        Ok(Export {
            name: format!("{name}.zip"),
            media_type: "application/zip".into(),
            content: Content::Bytes(robot.zip()?),
        })
    }
}

/// `name` as a file name: letters, digits, `-` and `_`, every other
/// character `_`.
fn file_name(name: &str) -> String {
    let name: String = name
        .chars()
        .map(|c| match c {
            'a'..='z' | 'A'..='Z' | '0'..='9' | '-' | '_' => c,
            _ => '_',
        })
        .collect();
    if name.is_empty() { "part".into() } else { name }
}

/// A rigid body of the mechanism: a part placed in the assembly, however
/// deep.
pub struct Body<'a> {
    /// Its name in the assembly: `hinge/pin`.
    pub name: String,
    /// Whether the mates may move it: it has a pose of its own to solve
    /// for, and no fixed mate holds it. A copy of a pattern goes where the
    /// pattern puts it, and is held to the ground like a fixed part.
    pub free: bool,
    /// Where it is in the assembly.
    pub world: Pose<Design>,
    /// Its part, as placed.
    pub part: Reader<'a>,
    /// The file its part is placed from.
    pub file: String,
}

/// An end of a joint: the body it is on — none for the assembly itself —
/// and its connector, in that body's frame.
pub struct End {
    pub body: Option<usize>,
    pub connector: Connector<Design>,
}

/// A joint of the mechanism, at the coordinates the state has: the angle in
/// degrees, and the distance.
pub struct MechanismJoint {
    pub name: String,
    pub kind: JointKind,
    pub a: End,
    pub b: End,
    pub angle: Design,
    pub distance: Design,
}

/// A coupling of the mechanism: its second joint's coordinate following
/// its first's, by index in [`Mechanism::joints`].
pub struct Coupling {
    pub name: String,
    pub kind: CouplingKind,
    pub a: usize,
    pub b: usize,
}

/// The rigid bodies of an assembly and its mates between them, as the
/// part a reader reads has them: the bodies in the order they were placed,
/// the constraints — by name, and the bodies of their two entities — the
/// joints and the couplings each in the order the steps made them.
pub struct Mechanism<'a> {
    pub bodies: Vec<Body<'a>>,
    pub constraints: Vec<(String, [Option<usize>; 2])>,
    pub joints: Vec<MechanismJoint>,
    pub couplings: Vec<Coupling>,
}

impl<'a> Mechanism<'a> {
    /// The mechanism of the part `reader` reads, its joints at the
    /// coordinates the state has them — or, where it has none, where their
    /// parts are.
    pub fn of(reader: &Reader<'a>) -> GeopResult<Self> {
        let placed = placed_parts(reader)?;
        let resolved = mates_of(reader);
        let fixed: BTreeSet<&str> = (resolved.iter())
            .filter_map(|(_, r)| match r {
                Resolved::Fixed { body } => Some(body.as_str()),
                _ => None,
            })
            .collect();
        let bodies: Vec<Body> = (placed.into_iter())
            .map(|p| Body {
                free: p.located.state.is_some() && !fixed.contains(p.name.as_str()),
                world: p.located.world,
                file: p.located.instance.file.clone(),
                part: p.located.part,
                name: p.name,
            })
            .collect();
        let index: HashMap<&str, usize> = (bodies.iter().enumerate())
            .map(|(i, b)| (b.name.as_str(), i))
            .collect();
        let body = |name: &Option<String>| -> GeopResult<Option<usize>> {
            name.as_ref()
                .map(|n| {
                    (index.get(n.as_str()).copied())
                        .ok_or_else(|| GeopError::new(format!("no part is placed as {n:?}")))
                })
                .transpose()
        };
        let mut constraints = Vec::new();
        let mut joints = Vec::new();
        let mut coupled = Vec::new();
        for (name, mate) in &resolved {
            let ctx = |e: GeopError| e.with_context(format!("the mate {name:?}"));
            match mate {
                Resolved::Constraint { a, b, .. } => constraints.push((
                    name.to_string(),
                    [
                        body(&a.body).with_context(&ctx)?,
                        body(&b.body).with_context(&ctx)?,
                    ],
                )),
                Resolved::Joint { kind, a, b } => {
                    let (a, b) = (
                        End {
                            body: body(&a.body).with_context(&ctx)?,
                            connector: a.what,
                        },
                        End {
                            body: body(&b.body).with_context(&ctx)?,
                            connector: b.what,
                        },
                    );
                    let world = |end: &End| end.body.map(|b| bodies[b].world);
                    let measured = measure(
                        &end_world(&a.connector, world(&a).as_ref()),
                        &end_world(&b.connector, world(&b).as_ref()),
                    );
                    let mut coordinates = [Design::ZERO; 2];
                    for (k, motion) in Motion::ALL.into_iter().enumerate() {
                        if kind.moves(motion) {
                            coordinates[k] =
                                match reader.retrieve_state(&joint_parameter(name, motion)) {
                                    StateValue::Number(value) => value,
                                    // Where its parts are: what a solve
                                    // would start it from.
                                    _ => Design::from_f64(measured[k]),
                                };
                        }
                    }
                    joints.push(MechanismJoint {
                        name: name.to_string(),
                        kind: *kind,
                        a,
                        b,
                        angle: coordinates[0],
                        distance: coordinates[1],
                    });
                }
                Resolved::Coupling { kind, joints } => {
                    coupled.push((name.to_string(), *kind, joints.clone()));
                }
                Resolved::Fixed { .. } => {}
            }
        }
        let couplings = (coupled.into_iter())
            .map(|(name, kind, [a, b])| {
                let find = |joint: &str| {
                    (joints.iter().position(|j: &MechanismJoint| j.name == joint)).ok_or_else(
                        || {
                            GeopError::new(format!(
                                "the coupling {name:?} ties the joint {joint:?}, which there is none of"
                            ))
                        },
                    )
                };
                Ok(Coupling {
                    a: find(&a)?,
                    b: find(&b)?,
                    name,
                    kind,
                })
            })
            .collect::<GeopResult<_>>()?;
        Ok(Mechanism {
            bodies,
            constraints,
            joints,
            couplings,
        })
    }
}

/// `connector` where `pose` puts it — or, with none, where it is — in
/// dual numbers, as [`measure`] takes it.
fn end_world(connector: &Connector<Design>, pose: Option<&Pose<Design>>) -> Connector<D> {
    let pose = pose.map(|p| p.map(D::cst));
    connector.placed(pose.as_ref())
}

/// The frame of a joint's end `end`, where its body is: origin on the
/// axis, `z` along it, `x` the reference turns are measured from.
fn frame(end: &End, bodies: &[Body]) -> GeopResult<Pose<Design>> {
    let c = &end.connector;
    let y = c.axis.prod_cross(&c.reference);
    let local = Pose::new(
        c.origin,
        Quaternion::from_rotation_columns([c.reference, y, c.axis])?,
    )?;
    Ok(match end.body {
        Some(b) => bodies[b].world.compose(&local),
        None => local,
    })
}

/// A turn by `degrees` about `z`.
fn turn(degrees: Design) -> GeopResult<Pose<Design>> {
    let radians = degrees.degrees_to_radians()?;
    Pose::rotation_about(&Vector3::zero(), &Vector3::axis(2), radians)
}

/// A slide by `distance` along `z`.
fn slide(distance: Design) -> Pose<Design> {
    Pose::identity().with_position(Vector3::from_array([Design::ZERO, Design::ZERO, distance]))
}

/// The meshes of an export, each written once: per placed part, by its
/// part's id — none for the assembly's own solids — the mesh's path, if it
/// has any triangles.
struct Meshes {
    paths: HashMap<Option<String>, Option<String>>,
    files: Vec<(String, Vec<u8>)>,
    quality: usize,
}

impl Meshes {
    /// The path of the mesh of the solids of the part `reader` reads itself —
    /// not of the parts placed in it, which are bodies of their own — in its
    /// own frame, called `stem`, written now if it was not yet; `None` if it
    /// has no solids. `key` names the part.
    fn of(
        &mut self,
        key: Option<String>,
        stem: &str,
        reader: &Reader,
    ) -> GeopResult<Option<String>> {
        if let Some(path) = self.paths.get(&key) {
            return Ok(path.clone());
        }
        let meshes = read_model(reader)?.solid_meshes(self.quality)?;
        let triangles: Vec<_> = meshes.iter().flat_map(|(_, triangles)| triangles).collect();
        let path = if triangles.is_empty() {
            None
        } else {
            let stem = file_name(stem);
            // The first name no file has: of one more names than there
            // are files, one is free.
            let path = (1..=self.files.len() + 1)
                .map(|n| match n {
                    1 => format!("meshes/{stem}.stl"),
                    n => format!("meshes/{stem}_{n}.stl"),
                })
                .find(|path| self.files.iter().all(|(p, _)| p != path))
                .expect("one of more names than files is free");
            let bytes = stl_bytes(triangles, &stem, StlFormat::Binary)?;
            self.files.push((path.clone(), bytes));
            Some(path)
        };
        self.paths.insert(key, path.clone());
        Ok(path)
    }
}

/// The mass properties of the solids of the part `reader` reads itself,
/// named behind `prefix`, where `pose` puts them, each of its part's
/// material (see [`own_solids_at`]).
fn masses(
    reader: &Reader,
    prefix: &str,
    pose: Option<Pose<Design>>,
    shapes: &mut Shapes,
    out: &mut Vec<MassProperties<Design>>,
) -> GeopResult<()> {
    for solid in own_solids_at(reader, prefix, pose)? {
        out.push(solid.mass_properties(shapes)?);
    }
    Ok(())
}

/// The assembly `output` built — where its parts are now — as a URDF robot
/// called `name`, meshed `quality` fine (see
/// [`geop_ops_rasterize::rasterize`]). Refuses an assembly whose mates do
/// not hold where its parts are — it would describe a robot that is not the
/// one drawn — and one URDF cannot express (see [`tree`]).
pub fn export(output: &mut Output, name: &str, quality: usize) -> GeopResult<UrdfExport> {
    let ctx = |e: GeopError| e.with_context(format!("exporting {name:?} as URDF"));
    let report = check(&output.part()).with_context(&ctx)?;
    if !report.converged {
        return Err(ctx(GeopError::new(format!(
            "the mates {} do not hold where the parts are: solve the assembly before exporting it",
            report.failed.join(", ")
        ))));
    }
    // The tree, and every joint's coordinates: what is held to ask what
    // mates alone leave free to move.
    let (tree, held) = {
        let mechanism = Mechanism::of(&output.part()).with_context(&ctx)?;
        let held: BTreeSet<String> = (mechanism.joints.iter())
            .flat_map(|j| (j.kind.motions().into_iter()).map(|m| joint_parameter(&j.name, m)))
            .collect();
        (tree::tree(&mechanism).with_context(&ctx)?, held)
    };
    let mut rerun = Rerun {
        runner: output.runner,
        part: &mut *output.part,
        program: output.file,
    };
    let freedom = solving::freedom(&mut rerun, &output.file.state, &held).with_context(&ctx)?;
    let reader = output.part();
    let mechanism = Mechanism::of(&reader).with_context(&ctx)?;
    tree::check_rigid(&mechanism, &tree, &freedom.parts).with_context(&ctx)?;
    let bodies = &mechanism.bodies;

    // Each link's frame in the assembly, and each joint, in tree order.
    let mut frames = vec![Pose::identity(); tree.links.len()];
    let mut joints = Vec::new();
    for tj in &tree.joints {
        let joint = &mechanism.joints[tj.joint];
        let jctx = |e: GeopError| e.with_context(format!("the joint {:?}", joint.name));
        let start = frame(&joint.a, bodies).with_context(&jctx)?;
        // What of the joint's motion stays — a revolute joint's offset
        // along its axis, a slider's turn about it — and what moves.
        let (fixed, moving, motion) = match joint.kind {
            JointKind::Revolute { .. } => (slide(joint.distance), turn(joint.angle)?, Motion::Turn),
            JointKind::Slider { .. } => (turn(joint.angle)?, slide(joint.distance), Motion::Slide),
            _ => unreachable!("tree refuses every other moving joint"),
        };
        let at_zero = start.compose(&fixed);
        // The child is carried by the joint from its frame at zero: on the
        // second end's body if the joint runs forward, else on the first's,
        // turning the other way.
        let (origin, child, axis) = if tj.forward {
            (at_zero, at_zero.compose(&moving), [0.0, 0.0, 1.0])
        } else {
            (at_zero.compose(&moving), at_zero, [0.0, 0.0, -1.0])
        };
        frames[tj.child] = child;
        let [min, max] = joint.kind.limits(motion).map(|l| l.map(|v| v.to_f64()));
        let kind = match (motion, min, max) {
            (Motion::Turn, Some(lower), Some(upper)) => JointType::Revolute {
                lower: lower.to_radians(),
                upper: upper.to_radians(),
            },
            (Motion::Turn, _, _) => JointType::Continuous,
            (Motion::Slide, Some(lower), Some(upper)) => JointType::Prismatic {
                lower: lower / 1000.0,
                upper: upper / 1000.0,
            },
            (Motion::Slide, _, _) => unreachable!("tree refuses a slider without limits"),
        };
        joints.push(Joint {
            name: joint.name.clone(),
            kind,
            parent: tree.links[tj.parent].name.clone(),
            child: tree.links[tj.child].name.clone(),
            origin: Origin::of(&frames[tj.parent].inverse().compose(&origin)),
            axis,
            mimic: None,
        });
    }
    mimics(&mechanism, &tree, &mut joints).with_context(&ctx)?;

    let mut meshes = Meshes {
        paths: HashMap::new(),
        files: Vec::new(),
        quality,
    };
    let mut shapes = Shapes::default();
    let mut links = Vec::new();
    for (index, link) in tree.links.iter().enumerate() {
        let back = frames[index].inverse();
        let mut masses_of = Vec::new();
        let mut visuals = Vec::new();
        // The assembly's own solids are the ground's: the root's.
        if index == 0 {
            masses(&reader, "", None, &mut shapes, &mut masses_of).with_context(&ctx)?;
            if let Some(mesh) = meshes.of(None, "base", &reader)? {
                visuals.push(Visual {
                    name: "base".into(),
                    origin: Origin::of(&back),
                    mesh,
                });
            }
        }
        for &b in &link.bodies {
            let body = &bodies[b];
            // A part placed is a body of its own solids: the parts placed
            // in it are bodies of their own.
            let prefix = instance_path(&body.name, "");
            let placed = body.part.clone();
            masses(
                &placed,
                &prefix,
                Some(body.world),
                &mut shapes,
                &mut masses_of,
            )
            .with_context(&ctx)?;
            let key = Some(drawn(&reader, &body.name).with_context(&ctx)?.key.clone());
            let stem = stem(&body.file).to_string();
            if let Some(mesh) = meshes.of(key, &stem, &placed)? {
                visuals.push(Visual {
                    name: body.name.clone(),
                    origin: Origin::of(&back.compose(&body.world)),
                    mesh,
                });
            }
        }
        let inertial = match MassProperties::combine(&masses_of)? {
            Some(total) => {
                let local = total.placed(&back)?;
                Some(Inertial {
                    mass: local.mass.to_f64(),
                    center: [0, 1, 2].map(|k| local.center[k].to_f64() / 1000.0),
                    inertia: local.inertia.map(|row| row.map(|v| v.to_f64() * 1e-6)),
                })
            }
            None => None,
        };
        links.push(Link {
            name: link.name.clone(),
            parts: (link.bodies.iter())
                .map(|&b| bodies[b].name.clone())
                .collect(),
            inertial,
            visuals,
        });
    }
    Ok(UrdfExport {
        robot: Robot {
            name: name.to_string(),
            links,
            joints,
        },
        meshes: meshes.files,
    })
}

/// Makes every joint a coupling drives mimic its driver: the coupling's
/// factor, in URDF's units — radians and metres. A joint driven through a
/// chain of couplings mimics the first driver, the factors multiplied, as
/// URDF readers expect. Refuses a joint two couplings drive, and couplings
/// driving each other round in a circle.
fn mimics(mechanism: &Mechanism, tree: &tree::Tree, joints: &mut [Joint]) -> GeopResult<()> {
    let couplings = &mechanism.couplings;
    // Per driven joint (by index in the mechanism), its driver, the
    // factor, and the coupling — by index, so that a circle of couplings is
    // reported from the same joint on every run.
    let mut drives: BTreeMap<usize, (usize, f64, usize)> = BTreeMap::new();
    for (i, coupling) in couplings.iter().enumerate() {
        let [ma, mb] = coupling.kind.transmission.motions();
        let unit = |m: Motion| match m {
            Motion::Turn => 1.0,
            Motion::Slide => 1e-3,
        };
        let multiplier = coupling.kind.factor()?.to_f64() * unit(mb) / unit(ma);
        if let Some((_, _, other)) = drives.insert(coupling.b, (coupling.a, multiplier, i)) {
            return Err(GeopError::new(format!(
                "the joint {:?} is driven by two couplings, {} and {}, and a URDF joint mimics one other only",
                mechanism.joints[coupling.b].name, couplings[other].name, coupling.name
            )));
        }
    }
    let index: HashMap<usize, usize> = tree
        .joints
        .iter()
        .enumerate()
        .map(|(k, tj)| (tj.joint, k))
        .collect();
    for (&driven, &(_, factor, _)) in &drives {
        // From the joint driven to each joint driving it, to the one
        // nothing drives.
        let through = walk(driven, |j| {
            Ok::<_, GeopError>(drives.get(j).map(|&(next, _, _)| next))
        })?;
        if through.end != Walked::Open {
            let names: Vec<&str> = (through.nodes.iter())
                .map(|&j| mechanism.joints[j].name.as_str())
                .collect();
            return Err(GeopError::new(format!(
                "the couplings of the joints {} drive each other round in a circle, which URDF's mimic joints cannot express",
                names.join(", ")
            )));
        }
        let (&driver, between) = through.nodes[1..]
            .split_last()
            .expect("driven by one joint at least");
        let multiplier = (between.iter()).fold(factor, |m, j| m * drives[j].1);
        joints[index[&driven]].mimic = Some(Mimic {
            joint: mechanism.joints[driver].name.clone(),
            multiplier,
        });
    }
    Ok(())
}
