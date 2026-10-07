//! The mechanism of a part: its mates as constraints, joints and couplings
//! between the rigid bodies placed in it (see [`Mechanism`]), and solving,
//! checking and measuring them — the methods of [`PartMates`] — where
//! [`geop_core_solve::mates`] does the solving.

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_solve::mates::{
    Assembly, Body, Connector, Constraint, Coordinate, Coupling, Feature, Joint, JointEnd, Motion,
    Pull as BodyPull,
};
use geop_ops::{
    Instance, Part,
    operation::{Aspects, EntityRef, INSTANCE_SEPARATOR},
    part::{ParamValue, State},
    ui::Drag,
};

use crate::mates::{
    JointInfo, JointValue, Mate, MateFreedom, MateKind, MateReport, Mates, joint_parameter,
};

/// A rigid body of a solve: a part placed in the part — or a part placed
/// in one of those, however deep.
pub struct PlacedBody<'p, S: Scalar> {
    /// Its name in the part: `hinge/pin`.
    pub name: String,
    /// The parameter of the part its pose is — relative to `parent`'s
    /// frame: `hinge/pin.pose` — if it is one: a copy of a pattern goes
    /// where the pattern puts it.
    pub parameter: Option<String>,
    pub instance: &'p Instance<S>,
    /// The body it is placed in, if it is placed in a part placed.
    pub parent: Option<usize>,
    /// Where it is in the part.
    pub world: Pose<S>,
}

/// An entity of a mate, resolved: the body it moves with, what it is there,
/// itself as that body's part names it, and the part.
type Resolved<'p, S> = (Option<usize>, Aspects<S>, EntityRef, &'p Part<S>);

/// The rigid bodies of a part and the mates between them: the assembly,
/// the bodies as placed — in the order of [`Assembly::bodies`] — and the
/// names of its mates: constraints, then joints, then couplings, as
/// [`Assembly::solve`] reports them.
pub struct Mechanism<'p, S: Scalar> {
    pub assembly: Assembly<S>,
    pub bodies: Vec<PlacedBody<'p, S>>,
    pub names: Vec<String>,
}

impl<S: Scalar> Mechanism<'_, S> {
    /// The name of the constraint `constraint`.
    pub fn constraint_name(&self, constraint: usize) -> &str {
        &self.names[constraint]
    }

    /// The name of joint `joint`.
    pub fn joint_name(&self, joint: usize) -> &str {
        &self.names[self.assembly.constraints.len() + joint]
    }

    /// The name of coupling `coupling`.
    pub fn coupling_name(&self, coupling: usize) -> &str {
        &self.names[self.assembly.constraints.len() + self.assembly.joints.len() + coupling]
    }
}

thread_local! {
    static MATES_RESOLVED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// How many mates this thread has resolved — their entities found and
/// measured on the parts placed — to solve or check them, ever. What any
/// question about the mates costs grows with this, so a test can bound the
/// work an edit does by the mates it touches, rather than time it: asking
/// the whole assembly something once per step is quadratic, and shows here
/// at any size.
pub fn mates_resolved() -> usize {
    MATES_RESOLVED.get()
}

/// The bodies of a solve in `part`: its instances, and inside every one, the
/// instances of the part it places — named, and their parameters named,
/// behind `prefix`, and placed in the body `parent` at `frame`. Those
/// placed in a copy of a pattern, which goes where the pattern puts it, go
/// with it: they have no parameter either — unless `parametrized`.
fn placed<'p, S: Scalar>(
    part: &'p Part<S>,
    prefix: &str,
    parent: Option<usize>,
    frame: &Pose<S>,
    parametrized: bool,
    out: &mut Vec<PlacedBody<'p, S>>,
) {
    for (id, instance) in part.instances() {
        let name = format!("{prefix}{}", part.name_of(id).unwrap_or_default());
        let world = frame.compose(&instance.pose);
        out.push(PlacedBody {
            name: name.clone(),
            parameter: instance
                .parameter
                .as_ref()
                .filter(|_| parametrized)
                .map(|p| format!("{prefix}{}", p.name)),
            instance,
            parent,
            world,
        });
        let body = Some(out.len() - 1);
        placed(
            &instance.part,
            &format!("{name}{INSTANCE_SEPARATOR}"),
            body,
            &world,
            parametrized && instance.parameter.is_some(),
            out,
        );
    }
}

/// The mates of the part, and of every part placed in it, as the part
/// names them: those of a part placed hold its parts as they held them
/// in it, named — and the joints a coupling of it ties named — behind
/// its name. Not those that fix a part: what is fixed in a part placed
/// is where it is placed, which the part it is placed in decides.
fn all_mates<S: Scalar>(part: &Part<S>, bodies: &[PlacedBody<'_, S>]) -> Vec<(String, Mate)> {
    let mut mates: Vec<(String, Mate)> = part
        .mates()
        .map(|(name, mate)| (name.to_string(), mate.clone()))
        .collect();
    for body in bodies {
        let behind = |name: &str| format!("{}{INSTANCE_SEPARATOR}{name}", body.name);
        for (name, mate) in body.instance.part.mates().filter(|(_, m)| !m.is_fixed()) {
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
/// — its instances, and the instances of every part placed in it,
/// however deep (see [`placed`]) — each free unless a fixed mate holds
/// it, a copy of a pattern, or, with `only`, other than the one whose
/// pose is that parameter. A joint's coordinates are the state's, held if `held`
/// names them, or measured where the state has none. Mates still
/// missing an entity hold nothing, and are left out, and so are those
/// `named` does not pick — with a coupling of a joint it does not.
fn assembly<'p, S: Scalar>(
    part: &'p Part<S>,
    only: Option<&str>,
    held: &[String],
    named: &dyn Fn(&str) -> bool,
) -> GeopResult<Mechanism<'p, S>> {
    let mut bodies_of = Vec::new();
    placed(part, "", None, &Pose::identity(), true, &mut bodies_of);
    let body_named: std::collections::HashMap<&str, usize> = bodies_of
        .iter()
        .enumerate()
        .map(|(i, b)| (b.name.as_str(), i))
        .collect();
    // The body an entity moves with — the innermost one it is of — and
    // the entity as that body's part names it.
    let body_of = |entity: &EntityRef| -> (Option<usize>, EntityRef) {
        let mut body: Option<usize> = None;
        let mut rest = entity.clone();
        while let Some((instance, inner)) = rest.split_instance() {
            let name = match body {
                Some(b) => format!("{}{INSTANCE_SEPARATOR}{instance}", bodies_of[b].name),
                None => instance,
            };
            let Some(&found) = body_named.get(name.as_str()) else {
                break;
            };
            body = Some(found);
            rest = inner;
        }
        (body, rest)
    };
    let mates = all_mates(part, &bodies_of);
    let mut held_fast = vec![false; bodies_of.len()];
    for (name, mate) in mates
        .iter()
        .filter(|(_, m)| m.is_fixed() && m.is_complete())
    {
        let entity = &mate.entities[0];
        let Some(body) = body_of(entity).0 else {
            return Err(GeopError::new(format!(
                    "{entity} is of this part itself, which never moves: a fixed mate holds a part placed in it"
                ))
                .with_context(format!("mate {name:?}")));
        };
        held_fast[body] = true;
    }
    let mut scale = S::ONE;
    let mut bodies = Vec::new();
    for (b, body) in bodies_of.iter().enumerate() {
        let center = match body.instance.part.bounds() {
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
            free: !held_fast[b]
                && body
                    .parameter
                    .as_deref()
                    .is_some_and(|p| only.is_none_or(|o| o == p)),
            center,
        });
    }
    let mut assembly = Assembly::new(bodies, Vec::new(), scale);
    let (mut constraint_names, mut joint_names, mut coupling_names) =
        (Vec::new(), Vec::new(), Vec::new());
    let mut couplings = Vec::new();
    for (name, mate) in &mates {
        if !mate.is_complete() || !named(name) {
            continue;
        }
        MATES_RESOLVED.set(MATES_RESOLVED.get() + 1);
        let ctx = with_context!("mate {name:?}");
        // The body an entity moves with, and what it is there.
        let resolve = |entity: &EntityRef| -> GeopResult<Resolved<'_, S>> {
            let (body, rest) = body_of(entity);
            let (part, local) = match body {
                Some(b) => (&*bodies_of[b].instance.part, rest),
                None => (part, entity.clone()),
            };
            // What an entity of a placed part is, is the same for every
            // instance of it.
            let aspects = part.memo(&format!("{local:?}"), || Aspects::of(&local, part))?;
            Ok((body, aspects, local, part))
        };
        match mate.kind {
            MateKind::Constraint(kind) => {
                let feature = |entity: &EntityRef| -> GeopResult<Feature<S>> {
                    let (body, aspects, ..) = resolve(entity)?;
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
                    let (body, aspects, local, part) = resolve(entity)?;
                    let connector = match Mate::connector(&aspects) {
                        Some(connector) => connector?,
                        // Anything else is used by the frame it gives.
                        None => {
                            let frame = Aspects::frame_on(&local, None, part)?;
                            Connector::new(*frame.origin(), *frame.w(), Some(*frame.u()))?
                        }
                    };
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
                    let value = match part.input(&parameter) {
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
            // Held already, above.
            MateKind::Anchor(_) => {}
        }
    }
    for (name, kind, joints) in couplings {
        if !joints.iter().all(|j| named(j)) {
            continue;
        }
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
    Ok(Mechanism {
        assembly,
        bodies: bodies_of,
        names,
    })
}
/// The mates of a part, and solving them (see the module docs of [`crate::mates`]).
pub trait PartMates<S: Scalar> {
    /// Adds `mate` under `name`: a mate is no entity — nothing is built on
    /// it — but its name keeps it apart from the other mates. Fails if the
    /// name is taken.
    fn add_mate(&mut self, mate: Mate, name: impl Into<String>) -> GeopResult<()>;

    /// Every mate of the part itself, by name.
    fn mates(&self) -> impl Iterator<Item = (&str, &Mate)>;

    /// The part's mechanism where its parts are now: every mate, each
    /// joint's coordinates where the state has them — or measured where
    /// their parts are — and every free body free (see
    /// [`Part::solve_mates`]).
    fn mechanism(&self) -> GeopResult<Mechanism<'_, S>>;

    /// Where the instances — and the parts placed in those placed flexibly
    /// — would have to be for every mate to hold, as little away from where
    /// they are as the mates allow, after `drags` pulled them — only the one
    /// whose pose is the parameter `only`, if given, and never a fixed one —
    /// and where their joints are, every coordinate within its limits and
    /// those `held` names kept where the state has them: the new values of
    /// the parameters the poses of those it moved are, each relative to the part it is
    /// placed in, and of the joints' coordinates, and which mates hold
    /// there. Fails for a mate that cannot hold between its entities at
    /// all.
    fn solve_mates(
        &self,
        only: Option<&str>,
        held: &[String],
        drags: &[Drag<S>],
    ) -> GeopResult<(State, MateReport)>;

    /// Solves the mates with the joint coordinates `set` held where the
    /// state has them: the parts move to them as a mechanism's joints would
    /// move them — every other joint keeping its coordinates, if the mates
    /// let it, as they do along a serial chain; where they do not, round a
    /// closed loop, the other joints give way.
    fn solve_joints(&self, set: &[String]) -> GeopResult<(State, MateReport)>;

    /// Which of the mates `named` picks — by name — hold where the
    /// instances are now: `|_| true` for all of them. Checking only some
    /// costs only theirs.
    fn check_mates(&self, named: impl Fn(&str) -> bool) -> GeopResult<MateReport>;

    /// Every joint of the part and of the parts placed flexibly in it, with
    /// where its coordinates are and their limits.
    fn joints(&self) -> GeopResult<Vec<JointInfo>>;

    /// How free every placed part is where it is now, and — if the mates do
    /// not all hold — a smallest set of them that conflict (see
    /// `geop_core_solve::mates::Assembly::conflicting`): as many solves as
    /// there are mates, so for a dialog to show, not for every drag.
    fn mate_freedom(&self) -> GeopResult<MateFreedom>;

    /// The parameters solving the mates sets: the poses of the placed parts
    /// — and of those in the parts placed flexibly — and the coordinates of
    /// the joints. What a program's file keeps of its state.
    fn solved_parameters(&self) -> Vec<String>;
}

impl<S: Scalar> PartMates<S> for Part<S> {
    fn add_mate(&mut self, mate: Mate, name: impl Into<String>) -> GeopResult<()> {
        let name = name.into();
        if self.entry::<Mates>(&name).is_some() {
            return Err(GeopError::new(format!("Part already has a mate {name:?}")));
        }
        self.insert_entry::<Mates>(name, mate);
        Ok(())
    }

    fn mates(&self) -> impl Iterator<Item = (&str, &Mate)> {
        self.entries::<Mates>()
    }

    fn mechanism(&self) -> GeopResult<Mechanism<'_, S>> {
        assembly(self, None, &[], &|_| true)
    }

    fn solve_mates(
        &self,
        only: Option<&str>,
        held: &[String],
        drags: &[Drag<S>],
    ) -> GeopResult<(State, MateReport)> {
        let mut mechanism = assembly(self, only, held, &|_| true)?;
        let pulls = drags
            .iter()
            .map(|drag| {
                let body = mechanism
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
        let report = mechanism.assembly.solve(&pulls)?;
        let assembly = &mechanism.assembly;
        // Only those the solve may have moved: the others are where the
        // state has them already, and written back they would only pick
        // up the rounding of composing their poses.
        let mut moved = State::new();
        for (b, (placed, body)) in mechanism.bodies.iter().zip(&assembly.bodies).enumerate() {
            if let (true, Some(parameter)) = (
                body.free && report.moved.binary_search(&b).is_ok(),
                &placed.parameter,
            ) {
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
                    joint_parameter(mechanism.joint_name(i), motion),
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
                    .map(|&i| mechanism.names[i].clone())
                    .collect(),
                at_limit: report
                    .at_limit
                    .iter()
                    .map(|&(j, m)| joint_parameter(mechanism.joint_name(j), m))
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

    fn solve_joints(&self, set: &[String]) -> GeopResult<(State, MateReport)> {
        let mut bodies = Vec::new();
        placed(self, "", None, &Pose::identity(), true, &mut bodies);
        let mut every: Vec<String> = Vec::new();
        for (name, mate) in all_mates(self, &bodies) {
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

    fn check_mates(&self, named: impl Fn(&str) -> bool) -> GeopResult<MateReport> {
        let mechanism = assembly(self, None, &[], &named)?;
        let report = mechanism.assembly.report()?;
        Ok(MateReport {
            converged: report.converged,
            failed: report
                .failed
                .iter()
                .map(|&i| mechanism.names[i].clone())
                .collect(),
            at_limit: Vec::new(),
            iterations: 0,
            phases: Vec::new(),
        })
    }

    fn joints(&self) -> GeopResult<Vec<JointInfo>> {
        let mechanism = assembly(self, None, &[], &|_| true)?;
        let mates = all_mates(self, &mechanism.bodies);
        Ok(mechanism
            .assembly
            .joints
            .iter()
            .enumerate()
            .map(|(i, joint)| {
                let name = mechanism.joint_name(i).to_string();
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
                let between = mates
                    .iter()
                    .find(|(mate, _)| *mate == name)
                    .map(|(_, mate)| mate.entities.iter().map(EntityRef::label).collect())
                    .unwrap_or_default();
                JointInfo {
                    name,
                    kind: joint.kind.label(),
                    between,
                    values,
                }
            })
            .collect())
    }

    fn mate_freedom(&self) -> GeopResult<MateFreedom> {
        let mechanism = assembly(self, None, &[], &|_| true)?;
        let freedom = mechanism.assembly.freedom()?;
        let conflicting = if mechanism.assembly.report()?.converged {
            Vec::new()
        } else {
            mechanism
                .assembly
                .conflicting()?
                .into_iter()
                .map(|i| mechanism.names[i].clone())
                .collect()
        };
        Ok(MateFreedom {
            parts: mechanism
                .bodies
                .iter()
                .zip(freedom.bodies)
                .map(|(b, dof)| (b.name.clone(), dof))
                .collect(),
            total: freedom.total,
            conflicting,
        })
    }

    fn solved_parameters(&self) -> Vec<String> {
        let mut bodies = Vec::new();
        placed(self, "", None, &Pose::identity(), true, &mut bodies);
        let mut names: Vec<String> = bodies.iter().filter_map(|b| b.parameter.clone()).collect();
        for (name, mate) in all_mates(self, &bodies) {
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
