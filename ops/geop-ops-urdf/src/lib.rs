//! URDF export: an assembly as a robot description, for simulators — ROS,
//! Gazebo, MuJoCo, PyBullet, Isaac — to load ([`export`]).
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
//! (see [`geop_ops_inspect::mass`]), the tensor about the centre of mass
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

use std::{
    collections::{BTreeMap, HashMap},
    sync::Arc,
};

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{Pose, Quaternion, TriangleFace},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_solve::mates::{JointEnd, JointKind, Motion};
use geop_core_topology::mass::MassProperties;
use geop_ops::{
    Instance, Part,
    assembly::{Mechanism, PlacedBody},
};
use geop_ops_inspect::bodies::placed_solids;
use geop_ops_rasterize::{
    rasterize,
    stl::{StlFormat, outward, write_stl},
};

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

/// The frame of a joint's end `end`, where its body is: origin on the
/// axis, `z` along it, `x` the reference turns are measured from.
fn frame<S: Scalar>(end: &JointEnd<S>, world: &[Pose<S>]) -> GeopResult<Pose<S>> {
    let c = &end.connector;
    let y = c.axis.prod_cross(&c.reference);
    let local = Pose::new(
        c.origin,
        Quaternion::from_rotation_columns([c.reference, y, c.axis])?,
    )?;
    Ok(match end.body {
        Some(b) => world[b].compose(&local),
        None => local,
    })
}

/// A turn by `degrees` about `z`.
fn turn<S: Scalar>(degrees: S) -> GeopResult<Pose<S>> {
    let radians = degrees.mul(S::PI).div(S::from_i64(180))?;
    Pose::rotation_about(&Vector3::zero(), &Vector3::axis(2), radians)
}

/// A slide by `distance` along `z`.
fn slide<S: Scalar>(distance: S) -> Pose<S> {
    Pose::identity().with_position(Vector3::from_array([S::ZERO, S::ZERO, distance]))
}

/// The triangles of every solid of `part` — not of the parts placed in it,
/// which are bodies of their own — in its own frame, moved by `pose`.
fn triangles<S: Scalar>(
    part: &Part<S>,
    pose: &Pose<S>,
    quality: usize,
    out: &mut Vec<TriangleFace<S>>,
) -> GeopResult<()> {
    let model = part.topology();
    let raster = rasterize(model, quality)?;
    let motion = pose.motion();
    let mut solids: Vec<_> = model.solids.keys().copied().collect();
    solids.sort_by_key(|s| s.0);
    for solid in solids {
        let mut faces = model.solid_faces(solid)?;
        faces.sort_by_key(|f| f.0);
        for t in faces.iter().filter_map(|f| raster.faces.get(f)).flatten() {
            out.push(TriangleFace {
                a: motion.apply(&t.a),
                b: motion.apply(&t.b),
                c: motion.apply(&t.c),
                normal: motion.rotate(&t.normal),
                vertex_normals: t.vertex_normals.map(|ns| ns.map(|n| motion.rotate(&n))),
            });
        }
    }
    Ok(())
}

/// The meshes of an export, each written once: per component — and
/// whether its placed parts are in it — the mesh's path, if it has any
/// triangles.
struct Meshes {
    paths: HashMap<usize, Option<String>>,
    files: Vec<(String, Vec<u8>)>,
    quality: usize,
}

impl Meshes {
    /// The path of the mesh of `part`, called `stem`, written now if it was
    /// not yet; `None` if it has no solids. `key` names the part.
    fn of<S: Scalar>(
        &mut self,
        key: usize,
        stem: &str,
        part: &Part<S>,
    ) -> GeopResult<Option<String>> {
        if let Some(path) = self.paths.get(&key) {
            return Ok(path.clone());
        }
        let mut faces = Vec::new();
        triangles(part, &Pose::identity(), self.quality, &mut faces)?;
        let path = if faces.is_empty() {
            None
        } else {
            let stem = file_name(stem);
            let mut path = format!("meshes/{stem}.stl");
            let mut n = 1;
            while self.files.iter().any(|(p, _)| *p == path) {
                n += 1;
                path = format!("meshes/{stem}_{n}.stl");
            }
            let stl: Vec<_> = faces.iter().map(outward).collect();
            let mut bytes = Vec::new();
            write_stl(&stl, stem.as_str(), StlFormat::Binary, &mut bytes)
                .map_err(|e| GeopError::new(format!("writing {path}: {e}")))?;
            self.files.push((path.clone(), bytes));
            Some(path)
        };
        self.paths.insert(key, path.clone());
        Ok(path)
    }
}

/// The name of an instance's file without folders or `.geop`: `link`.
fn stem_of<S: Scalar>(instance: &Instance<S>) -> String {
    let file = instance.file.rsplit(['/', ':']).next().unwrap_or_default();
    file.trim_end_matches(".geop").to_string()
}

/// `part` — an assembly, where its parts are now — as a URDF robot called
/// `name`, meshed `quality` fine (see [`geop_ops_rasterize::rasterize`]).
/// Refuses an assembly whose mates do not hold where its parts are — it
/// would describe a robot that is not the one drawn — and one URDF cannot
/// express (see [`tree`]).
pub fn export<S: Scalar>(part: &Part<S>, name: &str, quality: usize) -> GeopResult<UrdfExport> {
    let ctx = |e: GeopError| e.with_context(format!("exporting {name:?} as URDF"));
    let report = part.check_mates(|_| true).with_context(&ctx)?;
    if !report.converged {
        return Err(ctx(GeopError::new(format!(
            "the mates {} do not hold where the parts are: solve the assembly before exporting it",
            report.failed.join(", ")
        ))));
    }
    let mechanism = part.mechanism().with_context(&ctx)?;
    let tree = tree::tree(&mechanism).with_context(&ctx)?;
    let assembly = &mechanism.assembly;
    let world: Vec<Pose<S>> = assembly.bodies.iter().map(|b| b.pose).collect();

    // Each link's frame in the assembly, and each joint, in tree order.
    let mut frames = vec![Pose::identity(); tree.links.len()];
    let mut joints = Vec::new();
    for tj in &tree.joints {
        let joint = &assembly.joints[tj.joint];
        let joint_name = mechanism.joint_name(tj.joint);
        let jctx = |e: GeopError| e.with_context(format!("the joint {joint_name:?}"));
        let start = frame(&joint.a, &world).with_context(&jctx)?;
        // What of the joint's motion stays — a revolute joint's offset
        // along its axis, a slider's turn about it — and what moves.
        let (fixed, moving, motion) = match joint.kind {
            JointKind::Revolute { .. } => (
                slide(joint.distance.value),
                turn(joint.angle.value)?,
                Motion::Turn,
            ),
            JointKind::Slider { .. } => (
                turn(joint.angle.value)?,
                slide(joint.distance.value),
                Motion::Slide,
            ),
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
            name: joint_name.to_string(),
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
    let mut links = Vec::new();
    for (index, link) in tree.links.iter().enumerate() {
        let back = frames[index].inverse();
        let mut masses = Vec::new();
        let mut visuals = Vec::new();
        // The assembly's own solids are the ground's: the root's.
        if index == 0 {
            for solid in placed_solids(part)?.iter().filter(|s| s.pose.is_none()) {
                masses.push(solid.mass_properties().with_context(&|e: GeopError| {
                    e.with_context(format!("the mass of {}", solid.name))
                })?);
            }
            if let Some(mesh) = meshes.of(0, "base", part)? {
                visuals.push(Visual {
                    name: "base".into(),
                    origin: Origin::of(&back),
                    mesh,
                });
            }
        }
        for &b in &link.bodies {
            let body: &PlacedBody<S> = &mechanism.bodies[b];
            let instance = body.instance;
            // A part placed is a body of its own solids: the parts placed
            // in it are bodies of their own.
            for solid in placed_solids(&instance.part)?
                .iter()
                .filter(|s| s.pose.is_none())
            {
                let own = solid.mass_properties().with_context(&|e: GeopError| {
                    e.with_context(format!("the mass of {}/{}", body.name, solid.name))
                })?;
                masses.push(own.placed(&world[b])?);
            }
            let key = Arc::as_ptr(&instance.part) as usize;
            if let Some(mesh) = meshes.of(key, &stem_of(instance), &instance.part)? {
                visuals.push(Visual {
                    name: body.name.clone(),
                    origin: Origin::of(&back.compose(&world[b])),
                    mesh,
                });
            }
        }
        let inertial = match MassProperties::combine(&masses)? {
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
            parts: link
                .bodies
                .iter()
                .map(|&b| mechanism.bodies[b].name.clone())
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
fn mimics<S: Scalar>(
    mechanism: &Mechanism<'_, S>,
    tree: &tree::Tree,
    joints: &mut [Joint],
) -> GeopResult<()> {
    let couplings = &mechanism.assembly.couplings;
    // Per driven joint (by index in the mechanism), its driver, the
    // factor, and the coupling — by index, so that a circle of couplings is
    // reported from the same joint on every run.
    let mut drives: BTreeMap<usize, (usize, f64, usize)> = BTreeMap::new();
    for (i, coupling) in couplings.iter().enumerate() {
        let [ma, mb] = coupling.kind.motions();
        let unit = |m: Motion| match m {
            Motion::Turn => 1.0,
            Motion::Slide => 1e-3,
        };
        let multiplier = coupling.kind.factor()?.to_f64() * unit(mb) / unit(ma);
        if let Some((_, _, other)) = drives.insert(coupling.b, (coupling.a, multiplier, i)) {
            return Err(GeopError::new(format!(
                "the joint {:?} is driven by two couplings, {} and {}, and a URDF joint mimics one other only",
                mechanism.joint_name(coupling.b),
                mechanism.coupling_name(other),
                mechanism.coupling_name(i)
            )));
        }
    }
    let index: HashMap<usize, usize> = tree
        .joints
        .iter()
        .enumerate()
        .map(|(k, tj)| (tj.joint, k))
        .collect();
    for (&driven, &(first, factor, _)) in &drives {
        let (mut driver, mut multiplier) = (first, factor);
        let mut through = vec![driven];
        while let Some(&(next, factor, _)) = drives.get(&driver) {
            if through.contains(&driver) {
                let names: Vec<&str> = through.iter().map(|&j| mechanism.joint_name(j)).collect();
                return Err(GeopError::new(format!(
                    "the couplings of the joints {} drive each other round in a circle, which URDF's mimic joints cannot express",
                    names.join(", ")
                )));
            }
            through.push(driver);
            multiplier *= factor;
            driver = next;
        }
        joints[index[&driven]].mimic = Some(Mimic {
            joint: mechanism.joint_name(driver).to_string(),
            multiplier,
        });
    }
    Ok(())
}
