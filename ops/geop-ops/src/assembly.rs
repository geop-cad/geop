//! Mates: what holds the parts placed in a part together — two entities,
//! each of a placed part or of the part itself, and what they must be to
//! each other (see [`MateKind`]) — and solving them, by moving the placed
//! parts ([`Part::solve_mates`]).
//!
//! The solving is `geop_core_solve::mates`'s. What is added here is how an
//! entity, named like any other ([`EntityRef`]), becomes a point, a line, a
//! plane or a joint's connector attached to a rigid body: an entity of a
//! placed part — named behind the instance's name — moves with that
//! instance; an entity of the part itself is ground, and never moves.
//!
//! A joint's coordinates — how far it is turned and slid — are parameters of
//! the part's state, like the poses of its placed parts: named after the
//! mate, `add_part(arm2,m1).angle` (see [`joint_parameter`]). Solving sets
//! them, and a program sets them to place a mechanism at a pose. A joint
//! whose coordinate the state does not give starts where its parts are.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_solve::mates::{
    Assembly, Body, Connector, Constraint, Coordinate, Coupling, Feature, Geometry, Joint,
    JointEnd, Pull as BodyPull,
};
pub use geop_core_solve::mates::{CouplingKind, JointKind, Kind, Motion};
use serde::{Deserialize, Serialize};

use crate::{
    Design, Instance, Part,
    operation::{Aspects, EntityRef, INSTANCE_SEPARATOR},
    part::{ParamValue, State},
};

/// What a mate asks: that two entities touch, line up or keep a distance
/// (a constraint); that one part may only turn or slide against another
/// about an axis (a joint); or that two joints move together (a coupling).
/// Serializes flat, by the kind's own `type`: `{"type": "revolute", "max":
/// 90.0}`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MateKind {
    Constraint(Kind<Design>),
    Joint(JointKind<Design>),
    Coupling(CouplingKind<Design>),
}

impl MateKind {
    pub fn label(&self) -> &'static str {
        match self {
            MateKind::Constraint(kind) => kind.label(),
            MateKind::Joint(kind) => kind.label(),
            MateKind::Coupling(kind) => kind.label(),
        }
    }
}

/// The name of the parameter that is the coordinate of `motion` of the
/// joint named `mate`: `add_part(arm2,m1).angle` — in degrees — or
/// `.distance`.
pub fn joint_parameter(mate: &str, motion: Motion) -> String {
    format!("{mate}.{}", motion.name())
}

/// A mate: what two entities must be to each other — or, for a coupling,
/// two joints. Serializes flat: `{"type": "distance", "value": 1.0,
/// "entities": [...]}`.
///
/// One being edited may hold fewer than two entities or joints yet; only a
/// complete one ([`Mate::is_complete`]) holds anything.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mate {
    #[serde(flatten)]
    pub kind: MateKind,
    #[serde(default)]
    pub entities: Vec<EntityRef>,
    /// The joints a coupling ties, by their names in the part:
    /// `add_part(gear2,m1)`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<String>,
}

impl Mate {
    /// A constraint of `kind` between `entities`.
    pub fn constraint(kind: Kind<Design>, entities: Vec<EntityRef>) -> Self {
        Mate {
            kind: MateKind::Constraint(kind),
            entities,
            joints: Vec::new(),
        }
    }

    /// A joint of `kind` between `entities`: the first part's connector,
    /// then the second's.
    pub fn joint(kind: JointKind<Design>, entities: Vec<EntityRef>) -> Self {
        Mate {
            kind: MateKind::Joint(kind),
            entities,
            joints: Vec::new(),
        }
    }

    /// A coupling of `kind` between the joints named `joints`.
    pub fn coupling(kind: CouplingKind<Design>, joints: Vec<String>) -> Self {
        Mate {
            kind: MateKind::Coupling(kind),
            entities: Vec::new(),
            joints,
        }
    }

    /// Its two entities, if it has exactly two.
    pub fn pair(&self) -> Option<[&EntityRef; 2]> {
        match self.entities.as_slice() {
            [a, b] => Some([a, b]),
            _ => None,
        }
    }

    /// Whether it has what it holds together: two entities, or for a
    /// coupling two joints.
    pub fn is_complete(&self) -> bool {
        match self.kind {
            MateKind::Coupling(_) => self.joints.len() == 2,
            _ => self.pair().is_some(),
        }
    }

    /// What an entity has to be for this kind of mate, in words — for a
    /// coupling, what its joints have to be.
    pub fn needs(&self) -> &'static str {
        match self.kind {
            MateKind::Constraint(Kind::Coincident | Kind::Distance { .. }) => {
                "a point, a line or a plane"
            }
            MateKind::Constraint(Kind::Concentric) => "something round, or a line",
            MateKind::Constraint(Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. }) => {
                "a line or a plane"
            }
            MateKind::Joint(_) => "a circular edge, a sketch circle or a datum",
            MateKind::Coupling(CouplingKind::Gear { .. }) => "two joints that turn",
            MateKind::Coupling(_) => "a joint that turns, then one that slides",
        }
    }

    /// The geometry `aspects` contributes to a constraint of `kind`: for one
    /// that touches or keeps a distance, a point before a line before a
    /// plane — the center of a circle is a point; for a concentric one, an
    /// axis; for one that turns, a direction.
    fn geometry<S: Scalar>(kind: &Kind<Design>, aspects: &Aspects<S>) -> Option<Geometry<S>> {
        let point = || {
            aspects
                .point
                .as_ref()
                .or(aspects.arc.as_ref().map(|a| &a.circle.center))
                .map(|p| Geometry::Point { at: *p })
        };
        let line = |axes: [&Option<geop_core_geometry::shape::Axis<S>>; 2]| {
            axes.into_iter().flatten().next().map(|a| Geometry::Line {
                point: a.point,
                direction: a.direction,
            })
        };
        let plane = || {
            aspects.plane.as_ref().map(|p| Geometry::Plane {
                point: *p.origin(),
                normal: *p.w(),
            })
        };
        match kind {
            Kind::Coincident | Kind::Distance { .. } => point()
                .or_else(|| line([&aspects.line, &aspects.round]))
                .or_else(plane),
            Kind::Concentric => line([&aspects.round, &aspects.line]),
            Kind::Parallel | Kind::Perpendicular | Kind::Angle { .. } => {
                line([&aspects.line, &aspects.round]).or_else(plane)
            }
        }
    }

    /// The connector `aspects` gives a joint — an axis through a point it
    /// defines: a circular edge's center and normal, a sketch circle's, or a
    /// datum's origin and `z` axis, turns measured from its `x` axis. A
    /// round face or a straight edge has an axis, but no point of it the
    /// joint could sit at.
    fn connector<S: Scalar>(aspects: &Aspects<S>) -> Option<GeopResult<Connector<S>>> {
        if let Some(arc) = &aspects.arc {
            return Some(Connector::new(arc.circle.center, arc.circle.normal, None));
        }
        if let Some(frame) = &aspects.frame {
            return Some(Connector::new(
                *frame.origin(),
                *frame.w(),
                Some(*frame.u()),
            ));
        }
        if let (Some(round), false) = (&aspects.round, aspects.face) {
            return Some(Connector::new(round.point, round.direction, None));
        }
        None
    }
}

/// A drag of a placed part: its point `local` — in its own frame — pulled
/// towards the point `target`, before the mates are solved alone (see
/// `geop_core_solve::mates`). The part is named by the parameter its pose
/// is.
#[derive(Clone, Debug, PartialEq)]
pub struct Drag<S: Scalar> {
    pub parameter: String,
    pub local: Vector3<S>,
    pub target: Vector3<S>,
}

/// How a solve went.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MateReport {
    pub converged: bool,
    /// The mates that do not hold, by name.
    pub failed: Vec<String>,
    /// The joints' coordinates the solve stopped at a limit of, by the
    /// parameters they are.
    pub at_limit: Vec<String>,
    /// How many steps the solve took, and each of its minimizations — none
    /// for a check: why it stopped, after how many steps, and the largest
    /// residual it left.
    pub iterations: usize,
    pub phases: Vec<(geop_core_math::least_squares::Stop, usize, f64)>,
}

/// A joint's coordinate, as a dialog shows it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct JointValue {
    /// The parameter it is (see [`joint_parameter`]).
    pub parameter: String,
    pub motion: &'static str,
    /// Where it is: the state's value, or — not given one — measured from
    /// where its parts are.
    pub value: f64,
    pub min: Option<f64>,
    pub max: Option<f64>,
}

/// A joint of a part, as a dialog or the program's panel lists it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct JointInfo {
    /// The mate it is, by name.
    pub name: String,
    pub kind: &'static str,
    pub values: Vec<JointValue>,
}

/// How free the placed parts are (see `geop_core_solve::mates::Freedom`),
/// and which mates conflict.
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct MateFreedom {
    /// Per placed part, by name, how many independent ways it can still
    /// move: 0 for one held fast or fixed, 6 for one nothing holds.
    pub parts: std::collections::BTreeMap<String, usize>,
    /// How many independent ways the assembly can move as a whole.
    pub total: usize,
    /// A smallest set of mates that cannot all hold together, by name —
    /// empty when they all hold.
    pub conflicting: Vec<String>,
}

/// The box around every vertex of `part` and of the parts placed in it, as
/// placed — what a body turns about and how large a solve is are free
/// choices made from it, so its corners are sharp.
fn bounds<S: Scalar>(part: &Part<S>) -> Option<[Vector3<S>; 2]> {
    let own = part.topology().vertices.values().map(|v| v.point.sharpen());
    let placed = part.instances().flat_map(|(_, instance)| {
        let corners = bounds(instance.part()).map(|[lo, hi]| {
            (0..8).map(move |i| {
                let corner = Vector3::from_array(
                    [0, 1, 2].map(|k| if i >> k & 1 == 0 { lo[k] } else { hi[k] }),
                );
                instance.pose.apply(&corner).sharpen()
            })
        });
        corners.into_iter().flatten().collect::<Vec<_>>()
    });
    own.chain(placed).fold(None, |acc, p| {
        let [lo, hi] = acc.unwrap_or([p, p]);
        Some([
            Vector3::from_array([0, 1, 2].map(|k| lo[k].min(p[k]))),
            Vector3::from_array([0, 1, 2].map(|k| hi[k].max(p[k]))),
        ])
    })
}

/// A rigid body of a solve: a part placed in the part — or, inside one
/// placed flexibly, a part placed in that, however deep.
struct Placed<'p, S: Scalar> {
    /// Its name in the part: `hinge/pin`.
    name: String,
    /// The parameter of the part its pose is — relative to `parent`'s
    /// frame: `hinge/pin.pose` — if it is one: a copy of a pattern goes
    /// where the pattern puts it.
    parameter: Option<String>,
    instance: &'p Instance<S>,
    /// The body it is placed in, if it is placed in a flexible one.
    parent: Option<usize>,
    /// Where it is in the part.
    world: Pose<S>,
}

/// The rigid bodies of a part and the mates between them: the assembly,
/// the bodies as placed, the names of its mates — constraints, then
/// joints, then couplings, as [`Assembly::solve`] reports them — and the
/// coordinates its joints' parameters are, per joint and motion.
struct Solvable<'p, S: Scalar> {
    assembly: Assembly<S>,
    bodies: Vec<Placed<'p, S>>,
    names: Vec<String>,
}

impl<S: Scalar> Solvable<'_, S> {
    /// The name of joint `joint`.
    fn joint_name(&self, joint: usize) -> &str {
        &self.names[self.assembly.constraints.len() + joint]
    }
}

/// The bodies of a solve in `part`: its instances, and inside every
/// flexible one, the instances of the part it places — named, and their
/// parameters named, behind `prefix`, and placed in the body `parent` at
/// `frame`.
fn placed<'p, S: Scalar>(
    part: &'p Part<S>,
    prefix: &str,
    parent: Option<usize>,
    frame: &Pose<S>,
    out: &mut Vec<Placed<'p, S>>,
) {
    for (id, instance) in part.instances() {
        let name = format!("{prefix}{}", part.name_of(id).unwrap_or_default());
        let world = frame.compose(&instance.pose);
        out.push(Placed {
            name: name.clone(),
            parameter: instance.parameter.as_ref().map(|p| format!("{prefix}{p}")),
            instance,
            parent,
            world,
        });
        if instance.flexible {
            let body = Some(out.len() - 1);
            placed(
                instance.part(),
                &format!("{name}{INSTANCE_SEPARATOR}"),
                body,
                &world,
                out,
            );
        }
    }
}

impl<S: Scalar> Part<S> {
    /// The mates of the part, and of every part placed flexibly in it, as
    /// the part names them: those of a part placed flexibly hold its parts
    /// as they held them in it, named — and the joints a coupling of it
    /// ties named — behind its name.
    fn all_mates(&self, bodies: &[Placed<'_, S>]) -> Vec<(String, Mate)> {
        let mut mates: Vec<(String, Mate)> = self
            .mates()
            .map(|(name, mate)| (name.to_string(), mate.clone()))
            .collect();
        for body in bodies.iter().filter(|b| b.instance.flexible) {
            let behind = |name: &str| format!("{}{INSTANCE_SEPARATOR}{name}", body.name);
            for (name, mate) in body.instance.part().mates() {
                mates.push((
                    behind(name),
                    Mate {
                        kind: mate.kind,
                        entities: mate
                            .entities
                            .iter()
                            .map(|e| e.in_instance(&body.name))
                            .collect(),
                        joints: mate.joints.iter().map(|j| behind(j)).collect(),
                    },
                ));
            }
        }
        mates
    }

    /// The mates as constraints, joints and couplings between rigid bodies
    /// — its instances, and the instances of every part placed flexibly in
    /// it, however deep (see [`placed`]) — each free unless fixed, a copy
    /// of a pattern, or, with `only`, other than the one whose pose is that
    /// parameter. A joint's coordinates are the state's, held if `held`
    /// names them, or measured where the state has none. Mates still
    /// missing an entity hold nothing, and are left out.
    fn assembly(&self, only: Option<&str>, held: &[String]) -> GeopResult<Solvable<'_, S>> {
        let mut bodies_of = Vec::new();
        placed(self, "", None, &Pose::identity(), &mut bodies_of);
        let mut scale = S::ONE;
        let mut bodies = Vec::new();
        for body in &bodies_of {
            let center = match bounds(body.instance.part()) {
                Some([lo, hi]) => {
                    let diagonal = hi.sub(&lo).norm().sharpen();
                    if diagonal.definitely_greater(scale) {
                        scale = diagonal;
                    }
                    lo.add(&hi).prod_scalar(S::ONE.div(S::TWO)?).sharpen()
                }
                None => Vector3::zero(),
            };
            bodies.push(Body {
                pose: body.world,
                free: !body.instance.fixed
                    && body
                        .parameter
                        .as_deref()
                        .is_some_and(|p| only.is_none_or(|o| o == p)),
                center,
            });
        }
        let mates = self.all_mates(&bodies_of);
        let mut assembly = Assembly::new(bodies, Vec::new(), scale);
        let (mut constraint_names, mut joint_names, mut coupling_names) =
            (Vec::new(), Vec::new(), Vec::new());
        let mut couplings = Vec::new();
        for (name, mate) in &mates {
            if !mate.is_complete() {
                continue;
            }
            let ctx = with_context!("mate {name:?}");
            // The body an entity moves with — the innermost one it is of —
            // and what it is there.
            let resolve = |entity: &EntityRef| -> GeopResult<(Option<usize>, Aspects<S>)> {
                let mut body: Option<usize> = None;
                let mut rest = entity.clone();
                while let Some((instance, inner)) = rest.split_instance() {
                    let name = match body {
                        Some(b) => format!("{}{INSTANCE_SEPARATOR}{instance}", bodies_of[b].name),
                        None => instance,
                    };
                    let Some(found) = bodies_of.iter().position(|p| p.name == name) else {
                        break;
                    };
                    body = Some(found);
                    rest = inner;
                }
                let aspects = match body {
                    Some(b) => Aspects::of(&rest, bodies_of[b].instance.part())?,
                    None => Aspects::of(entity, self)?,
                };
                Ok((body, aspects))
            };
            match mate.kind {
                MateKind::Constraint(kind) => {
                    let feature = |entity: &EntityRef| -> GeopResult<Feature<S>> {
                        let (body, aspects) = resolve(entity)?;
                        let geometry = Mate::geometry(&kind, &aspects).ok_or_else(|| {
                            GeopError::new(format!("{entity} is not {}", mate.needs()))
                        })?;
                        Ok(Feature { body, geometry })
                    };
                    let [a, b] = mate.pair().expect("complete");
                    let constraint = Constraint {
                        kind: kind.cast(),
                        a: feature(a).with_context(ctx)?,
                        b: feature(b).with_context(ctx)?,
                    };
                    constraint.validate().with_context(ctx)?;
                    assembly.constraints.push(constraint);
                    constraint_names.push(name.clone());
                }
                MateKind::Joint(kind) => {
                    let end = |entity: &EntityRef| -> GeopResult<JointEnd<S>> {
                        let (body, aspects) = resolve(entity)?;
                        let connector = Mate::connector(&aspects).ok_or_else(|| {
                            GeopError::new(format!(
                                "{entity} is not {}: a joint needs an axis through a point it defines",
                                mate.needs()
                            ))
                        })??;
                        Ok(JointEnd { body, connector })
                    };
                    let [a, b] = mate.pair().expect("complete");
                    let mut joint = Joint {
                        kind: kind.cast(),
                        a: end(a).with_context(ctx)?,
                        b: end(b).with_context(ctx)?,
                        angle: Coordinate::free(S::ZERO),
                        distance: Coordinate::free(S::ZERO),
                    };
                    joint.validate().with_context(ctx)?;
                    let measured =
                        joint.measure(|b| b.map_or(Pose::identity(), |b| assembly.bodies[b].pose));
                    for (k, motion) in Motion::ALL.into_iter().enumerate() {
                        if !kind.moves(motion) {
                            continue;
                        }
                        let parameter = joint_parameter(name, motion);
                        let value = match self.inputs.get(&parameter) {
                            Some(ParamValue::Number(v)) => v.cast(),
                            Some(other) => {
                                return Err(GeopError::new(format!(
                                    "the joint coordinate {parameter:?} is a number, but the program gives it {other:?}"
                                )));
                            }
                            // Where its parts are: a seed, a free choice.
                            None => S::from_f64(measured[k]),
                        };
                        *joint.coordinate_mut(motion) = Coordinate {
                            value,
                            held: held.contains(&parameter),
                        };
                    }
                    assembly.joints.push(joint);
                    joint_names.push(name.clone());
                }
                MateKind::Coupling(kind) => {
                    couplings.push((name.clone(), kind, mate.joints.clone()));
                }
            }
        }
        for (name, kind, joints) in couplings {
            let ctx = with_context!("mate {name:?}");
            let index = |joint: &String| {
                joint_names
                    .iter()
                    .position(|n| n == joint)
                    .ok_or_else(|| GeopError::new(format!("there is no joint {joint:?}")))
                    .with_context(ctx)
            };
            let coupling = Coupling {
                kind: kind.cast(),
                a: index(&joints[0])?,
                b: index(&joints[1])?,
            };
            coupling
                .validate(&assembly.joints)
                .map_err(|e| e.with_context(format!("joints {:?} and {:?}", joints[0], joints[1])))
                .with_context(ctx)?;
            assembly.couplings.push(coupling);
            coupling_names.push(name);
        }
        let mut names = constraint_names;
        names.extend(joint_names);
        names.extend(coupling_names);
        Ok(Solvable {
            assembly,
            bodies: bodies_of,
            names,
        })
    }

    /// Where the instances — and the parts placed in those placed flexibly
    /// — would have to be for every mate to hold, as little away from where
    /// they are as the mates allow, after `drags` pulled them — only the one
    /// whose pose is the parameter `only`, if given, and never a fixed one —
    /// and where their joints are, every coordinate within its limits and
    /// those `held` names kept where the state has them: the new values of
    /// the parameters their poses are, each relative to the part it is
    /// placed in, and of the joints' coordinates, and which mates hold
    /// there. Fails for a mate that cannot hold between its entities at
    /// all.
    pub fn solve_mates(
        &self,
        only: Option<&str>,
        held: &[String],
        drags: &[Drag<S>],
    ) -> GeopResult<(State, MateReport)> {
        let mut solvable = self.assembly(only, held)?;
        let pulls = drags
            .iter()
            .map(|drag| {
                let body = solvable
                    .bodies
                    .iter()
                    .position(|b| b.parameter.as_ref() == Some(&drag.parameter))
                    .ok_or_else(|| {
                        GeopError::new(format!("no placed part's pose is {:?}", drag.parameter))
                    })?;
                Ok(BodyPull::Point {
                    body,
                    local: drag.local,
                    target: drag.target,
                })
            })
            .collect::<GeopResult<Vec<_>>>()?;
        let report = solvable.assembly.solve(&pulls)?;
        let assembly = &solvable.assembly;
        let mut moved = State::new();
        for (placed, body) in solvable.bodies.iter().zip(&assembly.bodies) {
            if let (true, Some(parameter)) = (body.free, &placed.parameter) {
                let pose = match placed.parent {
                    Some(parent) => assembly.bodies[parent].pose.inverse().compose(&body.pose),
                    None => body.pose,
                };
                moved.insert(parameter.clone(), ParamValue::Pose(pose.cast()));
            }
        }
        for (i, joint) in assembly.joints.iter().enumerate() {
            for motion in joint.kind.motions() {
                moved.insert(
                    joint_parameter(solvable.joint_name(i), motion),
                    ParamValue::Number(joint.coordinate(motion).value.cast()),
                );
            }
        }
        Ok((
            moved,
            MateReport {
                converged: report.converged,
                failed: report
                    .failed
                    .iter()
                    .map(|&i| solvable.names[i].clone())
                    .collect(),
                at_limit: report
                    .at_limit
                    .iter()
                    .map(|&(j, m)| joint_parameter(solvable.joint_name(j), m))
                    .collect(),
                iterations: report.iterations,
                phases: report
                    .phases
                    .iter()
                    .map(|p| (p.stop, p.iterations, p.max_residual.to_f64()))
                    .collect(),
            },
        ))
    }

    /// Solves the mates with the joint coordinates `set` held where the
    /// state has them: the parts move to them as a mechanism's joints would
    /// move them — every other joint keeping its coordinates, if the mates
    /// let it, as they do along a serial chain; where they do not, round a
    /// closed loop, the other joints give way.
    pub fn solve_joints(&self, set: &[String]) -> GeopResult<(State, MateReport)> {
        let mut bodies = Vec::new();
        placed(self, "", None, &Pose::identity(), &mut bodies);
        let mut every: Vec<String> = Vec::new();
        for (name, mate) in self.all_mates(&bodies) {
            if let MateKind::Joint(kind) = mate.kind {
                every.extend(
                    kind.motions()
                        .into_iter()
                        .map(|m| joint_parameter(&name, m)),
                );
            }
        }
        let (moved, report) = self.solve_mates(None, &every, &[])?;
        if report.converged {
            return Ok((moved, report));
        }
        self.solve_mates(None, set, &[])
    }

    /// Which mates hold where the instances are now.
    pub fn check_mates(&self) -> GeopResult<MateReport> {
        let solvable = self.assembly(None, &[])?;
        let report = solvable.assembly.report()?;
        Ok(MateReport {
            converged: report.converged,
            failed: report
                .failed
                .iter()
                .map(|&i| solvable.names[i].clone())
                .collect(),
            at_limit: Vec::new(),
            iterations: 0,
            phases: Vec::new(),
        })
    }

    /// Every joint of the part and of the parts placed flexibly in it, with
    /// where its coordinates are and their limits.
    pub fn joints(&self) -> GeopResult<Vec<JointInfo>> {
        let solvable = self.assembly(None, &[])?;
        Ok(solvable
            .assembly
            .joints
            .iter()
            .enumerate()
            .map(|(i, joint)| {
                let name = solvable.joint_name(i).to_string();
                let values = joint
                    .kind
                    .motions()
                    .into_iter()
                    .map(|motion| {
                        let [min, max] = joint.kind.limits(motion).map(|l| l.map(|v| v.to_f64()));
                        JointValue {
                            parameter: joint_parameter(&name, motion),
                            motion: motion.name(),
                            value: joint.coordinate(motion).value.to_f64(),
                            min,
                            max,
                        }
                    })
                    .collect();
                JointInfo {
                    name,
                    kind: joint.kind.label(),
                    values,
                }
            })
            .collect())
    }

    /// How free every placed part is where it is now, and — if the mates do
    /// not all hold — a smallest set of them that conflict (see
    /// `geop_core_solve::mates::Assembly::conflicting`): as many solves as
    /// there are mates, so for a dialog to show, not for every drag.
    pub fn mate_freedom(&self) -> GeopResult<MateFreedom> {
        let solvable = self.assembly(None, &[])?;
        let freedom = solvable.assembly.freedom()?;
        let conflicting = if solvable.assembly.report()?.converged {
            Vec::new()
        } else {
            solvable
                .assembly
                .conflicting()?
                .into_iter()
                .map(|i| solvable.names[i].clone())
                .collect()
        };
        Ok(MateFreedom {
            parts: solvable
                .bodies
                .iter()
                .zip(freedom.bodies)
                .map(|(b, dof)| (b.name.clone(), dof))
                .collect(),
            total: freedom.total,
            conflicting,
        })
    }

    /// The parameters solving the mates sets: the poses of the placed parts
    /// — and of those in the parts placed flexibly — and the coordinates of
    /// the joints. What a program's file keeps of its state.
    pub fn solved_parameters(&self) -> Vec<String> {
        let mut bodies = Vec::new();
        placed(self, "", None, &Pose::identity(), &mut bodies);
        let mut names: Vec<String> = bodies.iter().filter_map(|b| b.parameter.clone()).collect();
        for (name, mate) in self.all_mates(&bodies) {
            if let MateKind::Joint(kind) = mate.kind {
                names.extend(
                    kind.motions()
                        .into_iter()
                        .map(|m| joint_parameter(&name, m)),
                );
            }
        }
        names
    }
}
