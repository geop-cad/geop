//! The mechanism of a part: its mates as constraints on the program's state
//! — where its placed parts are, how its joints are turned — and what they
//! declare for solving it: the parts a fixed mate holds, each joint's
//! coordinates as values a solve moves and a user sets. Solving, checking
//! and measuring is generic ([`geop_ops::solving`]).
//!
//! A mate is worked out in two steps, each a target of the run. Its
//! entities are resolved once, in the frames of the parts they are of
//! ([`resolve`], the target [`crate::mate`]), which reads no state. Its
//! constraint on the state ([`constrain`]) places those where the state
//! has the parts, and is all that is made again when a part moves. A
//! joint's limits are constraints of their own ([`limit`]), which hold
//! anywhere short of them.
//!
//! A constraint's derivatives are by the state's own steps (see
//! [`geop_core_program::part::State`]): a pose's by a twist in its own frame, a
//! coordinate's by what is added to it. They are exact, through every pose
//! a placement reads — a part placed in a placed part moves with both —
//! computed in dual numbers, one state at a time.
//!
//! The editor solves ([`geop_ops::solving::solve`]): the solve moves the
//! state until the constraints the run reported hold, bringing only them up
//! to date for every state it tries, and the editor runs the program again
//! with the state found.

use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::{Ring, Scalar},
    solvers::system::tolerance,
    vector::Vector3,
    with_context,
};
use geop_core_program::{
    constraint::{Bound, Constraint},
    part::Retrieve,
    solve::Mobility,
    target::TargetReference,
};
use geop_ops::{
    Design, ParamValue,
    model::model_or_empty,
    operation::{Aspects, Build, BuildReader, BuildView, EntityRef, Input, Lookup, Scope, Seen},
    placed::{locate, locate_in, moved_by, within},
    solving::{Setting, Settings, Unknown, largest, scale_of, settings_of, unknown},
};

use crate::{
    joints::{Connector, JointKind, Motion, joint_residuals, measure},
    mates::{Mate, MateKind, On, Resolved, joint_parameter, mate, mate_name},
    residuals::{D, Geometry, Kind, residuals},
};

thread_local! {
    static MATES_RESOLVED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// How many mates this thread has resolved — their entities found and
/// measured on the parts placed — ever. What any question about the mates
/// costs grows with this, so a test can bound the work an edit does by the
/// mates it touches, rather than time it.
pub fn mates_resolved() -> usize {
    MATES_RESOLVED.get()
}

/// The size of the placed parts `bodies` — at least one: what a mate
/// between them turns a turn into a length by.
fn size(reader: &mut BuildReader, bodies: &[&Option<String>]) -> GeopResult<Design> {
    largest((bodies.iter().copied().flatten()).map(|body| Ok(locate(reader, body)?.instance)))
}

/// The geometry the entity `entity` of `mate`, a constraint of `kind`, is
/// to it, in `lookup`.
fn geometry_in(
    kind: &Kind,
    mate: &Mate,
    entity: &EntityRef,
    lookup: &mut (impl Lookup<Design> + ?Sized),
) -> GeopResult<Geometry> {
    let aspects = Aspects::of(entity, lookup)?;
    kind.geometry(&aspects)
        .ok_or_else(|| GeopError::new(format!("{entity} is not {}", mate.needs())))
}

/// The connector the entity `entity` is to a joint, in `lookup`: what it
/// makes one of itself, or the frame it gives.
fn connector_in(
    entity: &EntityRef,
    lookup: &mut (impl Lookup<Design> + ?Sized),
) -> GeopResult<Connector<Design>> {
    let aspects = Aspects::of(entity, lookup)?;
    match Connector::of(&aspects) {
        Some(connector) => connector,
        None => {
            let frame = Aspects::frame_on(entity, None, lookup)?;
            Connector::new(*frame.origin(), *frame.w(), Some(*frame.u()))
        }
    }
}

/// What `what` makes of the entity `entity` of the part read, on the body
/// it moves with: in the frame of the placed part it is of, or of the part
/// itself.
fn on<T>(
    reader: &mut BuildReader,
    entity: &EntityRef,
    what: impl Fn(&EntityRef, &mut dyn Lookup<Design>) -> GeopResult<T>,
) -> GeopResult<On<T>> {
    let (body, local) = match within(reader, entity) {
        Some((path, placed, local)) => (Some((path, placed.component.view())), local),
        None => (None, entity.clone()),
    };
    let what = match &body {
        Some((_, view)) => what(&local, &mut Seen::new(*view))?,
        None => {
            // Before the first step that builds any topology, an entity of
            // the part itself can only be one that needs none: its origin.
            let built = model_or_empty(reader);
            what(
                &local,
                &mut Scope {
                    model: &built,
                    reader,
                },
            )?
        }
    };
    Ok(On {
        body: body.map(|(name, _)| name),
        what,
        entity: entity.clone(),
    })
}

/// The mate `name`, `mate`, resolved in the part read: the generator of its
/// target [`crate::mate`]. A fixed mate fixes the part its entity is of.
pub(crate) fn resolve(
    reader: &mut BuildReader,
    (name, mate): &(String, Mate),
) -> GeopResult<Resolved> {
    MATES_RESOLVED.set(MATES_RESOLVED.get() + 1);
    let ctx = with_context!("mate {name:?}");
    let resolved = match mate.kind {
        MateKind::Constraint(kind) => {
            let [a, b] = mate
                .pair()
                .ok_or_else(|| GeopError::new("it needs two entities"))
                .with_context(ctx)?;
            let geometry =
                |e: &EntityRef, l: &mut dyn Lookup<Design>| geometry_in(&kind, mate, e, l);
            let (a, b) = (on(reader, a, geometry)?, on(reader, b, geometry)?);
            // Whether the kind can hold between the two is a question of
            // their geometry alone, whatever bodies they are of.
            kind.fits(&a.what, &b.what).with_context(ctx)?;
            Resolved::Constraint { kind, a, b }
        }
        MateKind::Joint(kind) => {
            let [a, b] = mate
                .pair()
                .ok_or_else(|| GeopError::new("it needs two frames"))
                .with_context(ctx)?;
            kind.validate().with_context(ctx)?;
            Resolved::Joint {
                kind,
                a: on(reader, a, |e, l| connector_in(e, l))?,
                b: on(reader, b, |e, l| connector_in(e, l))?,
            }
        }
        MateKind::Coupling(kind) => match mate.joints.as_slice() {
            [a, b] => Resolved::Coupling {
                kind,
                joints: [a.clone(), b.clone()],
            },
            _ => return Err(GeopError::new("it needs two joints")).with_context(ctx),
        },
        MateKind::Anchor(_) => {
            let [entity] = mate.entities.as_slice() else {
                return Err(GeopError::new("it needs one entity")).with_context(ctx);
            };
            match within(reader, entity) {
                Some((body, _, _)) => Resolved::Fixed { body },
                None => {
                    return Err(GeopError::new(format!(
                        "{entity} is of this part itself, which never moves: a fixed mate holds a part placed in it"
                    )))
                    .with_context(ctx);
                }
            }
        }
    };
    Ok(resolved)
}

/// The step of a pose by its six variables: a turn, then a move, in its
/// own frame (see [`State::retract`]).
fn twist() -> GeopResult<Pose<D>> {
    let var = |k: usize| D::var(Design::ZERO, k);
    Pose::exp(
        &Vector3::from_array([var(0), var(1), var(2)]),
        &Vector3::from_array([var(3), var(4), var(5)]),
    )
}

/// What a state is to a mate that reads it.
#[derive(Clone, Copy, PartialEq)]
enum Read {
    /// A pose a placement reads.
    Pose,
    /// A joint's coordinate.
    Coordinate,
}

/// The states a mate reads, as it is evaluated: each with what it is, and
/// the one it is differentiated by, if any.
struct Reading<'a> {
    read: BTreeMap<String, Read>,
    by: Option<&'a str>,
    twist: Pose<D>,
}

impl Reading<'_> {
    /// Where the placed part `body` is, as the state has it — none for the
    /// part itself, which never moves.
    fn pose(
        &mut self,
        reader: &mut BuildReader,
        body: &Option<String>,
    ) -> GeopResult<Option<Pose<D>>> {
        let Some(body) = body else {
            return Ok(None);
        };
        let Reading { read, by, twist } = self;
        let located = locate_in(reader, body, &mut |state, pose| {
            read.insert(state.to_string(), Read::Pose);
            let pose = pose.map(D::cst);
            match *by == Some(state) {
                true => pose.compose(twist),
                false => pose,
            }
        })?;
        Ok(Some(located.world))
    }

    /// The coordinate `parameter`, as the state has it.
    fn coordinate(&mut self, reader: &mut BuildReader, parameter: &str) -> GeopResult<D> {
        self.read.insert(parameter.to_string(), Read::Coordinate);
        let ParamValue::Number(value) = reader.retrieve_state(parameter) else {
            return Err(GeopError::new(format!(
                "the joint coordinate {parameter:?} holds no number"
            )));
        };
        Ok(match self.by == Some(parameter) {
            true => D::var(value, 0),
            false => D::cst(value),
        })
    }

    /// The coordinates `[angle, distance]` of the joint `name` of `kind`:
    /// zero for a motion it does not free.
    fn coordinates(
        &mut self,
        reader: &mut BuildReader,
        name: &str,
        kind: &JointKind,
    ) -> GeopResult<[D; 2]> {
        let mut out = [D::cst(Design::ZERO); 2];
        for (k, motion) in Motion::ALL.into_iter().enumerate() {
            if kind.moves(motion) {
                out[k] = self.coordinate(reader, &joint_parameter(name, motion))?;
            }
        }
        Ok(out)
    }

    /// The size of the bodies of `a` and `b` — what a mate between them
    /// turns a turn into a length by (see [`size`]) — and where the state
    /// has them.
    fn ends<T>(
        &mut self,
        reader: &mut BuildReader,
        a: &On<T>,
        b: &On<T>,
    ) -> GeopResult<(D, [Option<Pose<D>>; 2])> {
        let scale = D::cst(size(reader, &[&a.body, &b.body])?);
        let poses = [self.pose(reader, &a.body)?, self.pose(reader, &b.body)?];
        Ok((scale, poses))
    }
}

/// A joint resolved: its kind, and its two ends.
type Joint<'a> = (
    &'a JointKind,
    &'a On<Connector<Design>>,
    &'a On<Connector<Design>>,
);

/// The joint `name` of the part read, resolved.
fn joint<'a>(reader: &mut BuildReader<'a>, name: &str) -> GeopResult<Joint<'a>> {
    match reader.retrieve_target(&mate(name))? {
        Resolved::Joint { kind, a, b } => Ok((kind, a, b)),
        _ => Err(GeopError::new(format!("the mate {name:?} is no joint"))),
    }
}

/// The residuals of the mate `name`, `resolved`, where the state has its
/// parts and joints (see [`constrain`]).
fn residuals_of(
    reader: &mut BuildReader,
    name: &str,
    resolved: &Resolved,
    reading: &mut Reading,
) -> GeopResult<Vec<D>> {
    match resolved {
        Resolved::Constraint { kind, a, b } => {
            let (scale, [pa, pb]) = reading.ends(reader, a, b)?;
            let (wa, wb) = (a.what.placed(pa.as_ref())?, b.what.placed(pb.as_ref())?);
            residuals(kind, &wa, &wb, scale)
        }
        Resolved::Joint { kind, a, b } => {
            let (scale, [pa, pb]) = reading.ends(reader, a, b)?;
            let [angle, distance] = reading.coordinates(reader, name, kind)?;
            let (ca, cb) = (a.what.placed(pa.as_ref()), b.what.placed(pb.as_ref()));
            joint_residuals(&ca, &cb, angle, distance, scale)
        }
        Resolved::Coupling { kind, joints } => {
            let [ja, jb] = [&joints[0], &joints[1]].map(|j| joint(reader, j));
            let ((ka, a1, a2), (kb, b1, b2)) = (ja?, jb?);
            let [ma, mb] = kind.transmission.motions();
            kind.validate(joints[0] == joints[1], [ka.moves(ma), kb.moves(mb)])
                .map_err(|e| {
                    e.with_context(format!("joints {:?} and {:?}", joints[0], joints[1]))
                })?;
            let scale = D::cst(size(reader, &[&a1.body, &a2.body, &b1.body, &b2.body])?);
            let a = reading.coordinate(reader, &joint_parameter(&joints[0], ma))?;
            let b = reading.coordinate(reader, &joint_parameter(&joints[1], mb))?;
            Ok(vec![kind.residual(a, b, scale)?])
        }
        Resolved::Fixed { .. } => Ok(Vec::new()),
    }
}

/// A constraint of the residuals `evaluate` gives, where the state is, and
/// their derivatives by every state they read that holds what it is read
/// as — a pose not solved for yet puts its part at the origin, and is not
/// one to move.
fn differentiated(
    reader: &mut BuildReader,
    bound: Bound,
    evaluate: impl Fn(&mut BuildReader, &mut Reading) -> GeopResult<Vec<D>>,
) -> GeopResult<Constraint<ParamValue>> {
    let twist = twist()?;
    let mut plain = Reading {
        read: BTreeMap::new(),
        by: None,
        twist,
    };
    let values = evaluate(reader, &mut plain)?;
    let mut jacobian = BTreeMap::new();
    for (state, read) in plain.read {
        let dimension = match (reader.retrieve_state(&state), read) {
            (ParamValue::Pose(_), Read::Pose) => 6,
            (ParamValue::Number(_), Read::Coordinate) => 1,
            _ => continue,
        };
        let mut by = Reading {
            read: BTreeMap::new(),
            by: Some(&state),
            twist,
        };
        let rows = (evaluate(reader, &mut by)?.into_iter())
            .map(|r| r.d.into_iter().take(dimension).collect())
            .collect();
        jacobian.insert(state, rows);
    }
    Ok(Constraint {
        residuals: values.into_iter().map(|r| r.v).collect(),
        jacobian,
        bound,
    })
}

/// The constraint the mate `name` puts on the state: the generator of its
/// constraint, named as the mate is. Its residuals are lengths — a turn's
/// the distance it moves a point as far from its axis as its parts are
/// large — that vanish where it holds.
#[expect(
    clippy::ptr_arg,
    reason = "a generator is handed its arguments as they are kept"
)]
pub(crate) fn constrain(
    reader: &mut BuildReader,
    name: &String,
) -> GeopResult<Constraint<ParamValue>> {
    let resolved = reader.retrieve_target(&mate(name))?;
    differentiated(reader, Bound::Zero, |reader, reading| {
        residuals_of(reader, name, resolved, reading)
    })
}

/// The name of the constraint that keeps the coordinate `parameter` of a
/// joint within its upper limit, or its lower: `….angle.max`.
fn limit_name(parameter: &str, upper: bool) -> String {
    format!("{parameter}.{}", if upper { "max" } else { "min" })
}

/// The constraint that keeps the coordinate of `motion` of the joint
/// `name` within its upper limit, or its lower: a limit (see
/// [`Bound::AtMost`]), as a length as [`constrain`]'s are.
pub(crate) fn limit(
    reader: &mut BuildReader,
    (name, motion, upper): &(String, Motion, bool),
) -> GeopResult<Constraint<ParamValue>> {
    let (kind, a, b) = joint(reader, name)?;
    let [min, max] = kind.limits(*motion);
    let bound = (if *upper { max } else { min })
        .ok_or_else(|| GeopError::new(format!("the joint {name:?} has no such limit")))?;
    // An angle is turned into a length by the parts' size; a distance is
    // one, and is not multiplied by an exact one.
    let scale = match motion {
        Motion::Turn => Some(size(reader, &[&a.body, &b.body])?),
        Motion::Slide => None,
    };
    let parameter = joint_parameter(name, *motion);
    differentiated(reader, Bound::AtMost, |reader, reading| {
        let value = reading.coordinate(reader, &parameter)?;
        let past = match upper {
            true => value.sub(D::cst(bound)),
            false => D::cst(bound).sub(value),
        };
        let past = motion.natural(past)?;
        Ok(vec![match scale {
            Some(scale) => past.mul(D::cst(scale)),
            None => past,
        }])
    })
}

/// Defines what the mate `name`, resolved already in the part, puts on the
/// state: its constraint, and its limits'. A fixed mate puts none: it
/// declares the part it holds fixed. A joint declares its coordinates as
/// values a solve moves, and a user sets.
pub(crate) fn define(part: &mut Build, name: &str) -> GeopResult<()> {
    let resolved = part.current().retrieve_target(&mate(name))?.clone();
    if let Resolved::Fixed { .. } = resolved {
        return part.define_target(&unknown(name), &name.to_string(), fixed);
    }
    part.define_constraint(name, &name.to_string(), constrain)?;
    if let Resolved::Joint { kind, .. } = resolved {
        part.define_target(&settings_of(name), &name.to_string(), settings)?;
        for motion in kind.motions() {
            let parameter = joint_parameter(name, motion);
            part.define_target(
                &unknown(&parameter),
                &(name.to_string(), motion),
                coordinate,
            )?;
            for (upper, given) in [false, true].into_iter().zip(kind.limits(motion)) {
                if given.is_some() {
                    let args = (name.to_string(), motion, upper);
                    part.define_constraint(&limit_name(&parameter, upper), &args, limit)?;
                }
            }
        }
    }
    Ok(())
}

/// The mates of the part `view` shows, resolved, by name, in the order the
/// steps made them.
pub(crate) fn mates_of<'a>(view: BuildView<'a>) -> Vec<(&'a str, &'a Resolved)> {
    (view.targets_of::<Resolved>().into_iter())
        .filter_map(|(target, resolved)| Some((mate_name(target)?, resolved)))
        .collect()
}

/// Where the connectors of the joint `a`–`b` are, as the part read has
/// them: in the world.
fn connectors_at<'a>(
    r: &mut impl Retrieve<'a, Input, ParamValue>,
    a: &On<Connector<Design>>,
    b: &On<Connector<Design>>,
) -> GeopResult<[Connector<D>; 2]> {
    let mut world = |on: &On<Connector<Design>>| -> GeopResult<Option<Pose<D>>> {
        on.body
            .as_ref()
            .map(|body| Ok(locate(&mut *r, body)?.world.map(D::cst)))
            .transpose()
    };
    let (wa, wb) = (world(a)?, world(b)?);
    Ok([a.what.placed(wa.as_ref()), b.what.placed(wb.as_ref())])
}

/// The coordinates `[angle, distance]` of a joint, measured where its parts
/// are — an angle at the turn nearest `near`.
fn measured<'a>(
    r: &mut impl Retrieve<'a, Input, ParamValue>,
    a: &On<Connector<Design>>,
    b: &On<Connector<Design>>,
    near: f64,
) -> GeopResult<[f64; 2]> {
    let [ca, cb] = connectors_at(r, a, b)?;
    let [angle, distance] = measure(&ca, &cb);
    Ok([angle + 360.0 * ((near - angle) / 360.0).round(), distance])
}

/// What the fixed mate `name` declares: the state of the part it holds,
/// fixed — the generator of its unknown. A part no state moves is fixed
/// already, and so is a state none has.
fn fixed(reader: &mut BuildReader, name: &String) -> GeopResult<Unknown<ParamValue>> {
    let Resolved::Fixed { body } = reader.retrieve_target(&mate(name))? else {
        return Err(GeopError::new(format!("the mate {name:?} fixes nothing")));
    };
    Ok(Unknown {
        state: moved_by(reader, body)?.unwrap_or_else(|| body.clone()),
        mobility: Mobility::Fixed,
        seed: None,
        unless: None,
    })
}

/// The coordinate of `motion` of the joint `name`, as a value a solve
/// moves: following its parts, and — where the joint does not hold —
/// started where they are, measured from them at the turn nearest where it
/// is: a seed, so a free choice, which the solve makes exact. The
/// generator of its unknown.
fn coordinate(
    reader: &mut BuildReader,
    (name, motion): &(String, Motion),
) -> GeopResult<Unknown<ParamValue>> {
    let parameter = joint_parameter(name, *motion);
    let near = match reader.retrieve_state(&parameter) {
        ParamValue::Number(v) => v.to_f64(),
        _ => 0.0,
    };
    let (_, a, b) = joint(reader, name)?;
    // Where the parts cannot be measured, there is nothing to start from.
    let seed = measured(reader, a, b, near).ok().map(|[angle, distance]| {
        ParamValue::Number(Design::from_f64(match motion {
            Motion::Turn => angle,
            Motion::Slide => distance,
        }))
    });
    Ok(Unknown {
        state: parameter,
        mobility: Mobility::Follows,
        seed,
        unless: Some(name.clone()),
    })
}

/// The coordinates of the joint `name` as values a user sets, with their
/// limits: the generator of its settings.
#[expect(
    clippy::ptr_arg,
    reason = "a generator is handed its arguments as they are kept"
)]
fn settings(reader: &mut BuildReader, name: &String) -> GeopResult<Settings> {
    let (kind, a, b) = joint(reader, name)?;
    Ok(Settings {
        name: name.clone(),
        kind: kind.label(),
        between: vec![a.entity.label(), b.entity.label()],
        values: (kind.motions().into_iter())
            .map(|motion| {
                let [min, max] = kind.limits(motion);
                Setting {
                    parameter: joint_parameter(name, motion),
                    measure: motion.measure(),
                    min,
                    max,
                }
            })
            .collect(),
    })
}

/// Whether the constraint of the mate `name`, as `view` has it, holds.
pub fn mate_holds(view: BuildView, name: &str) -> bool {
    let reference = TargetReference::<Constraint<ParamValue>>::new(name);
    match (
        view.retrieve_target(&reference),
        scale_of(view).and_then(tolerance),
    ) {
        (Ok(constraint), Ok(tolerance)) => constraint.holds(tolerance),
        _ => false,
    }
}
