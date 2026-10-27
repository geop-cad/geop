//! The mechanism of a part: its mates as constraints on the program's state
//! — where its placed parts are, how its joints are turned — and what they
//! declare for solving it: the parts a fixed mate holds, each joint's
//! coordinates as values a solve moves and a user sets. Solving, checking
//! and measuring is generic ([`crate::solving`]).
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
//! [`geop_core_program::values::StateValue`]): a pose's by a twist in its own frame, a
//! coordinate's by what is added to it. They are exact, through every pose
//! a placement reads — a part placed in a placed part moves with both —
//! computed in dual numbers, one state at a time.
//!
//! The editor solves ([`crate::solving::solve`]): the solve moves the state
//! until the constraints hold, making only the constraints it asks again
//! at every state it tries, of the part as built (see
//! [`geop_lib_constraints::constraint::make_again`]) — which is why a
//! constraint reads only the state and what reads none — and the editor
//! runs the program once more with the state found.
//!
//! The constraints of one step's mates are defined together, and so are
//! their unknowns (see [`define`]).

use geop_core_program::{
    element::ElementReference, part::StepContext, reader::Reader, values::StateValue,
};
use std::collections::BTreeMap;

use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Pose,
    scalars::{Number, Scalar},
    solvers::system::{Mobility, tolerance},
    vector::Vector3,
    with_context,
};
use geop_lib_constraints::{
    Bound, Constraint, Constraints,
    constraint::define_constraints,
    unknown::{Unknown, declare_unknowns},
};
use geop_ops_model::parameters::{compose, placement};
use geop_ops_model::selection::Selected;

use geop_ops_model::placed::{drawn, locate, moved_by};

use crate::{
    joints::{Connector, JointKind, Motion, joint_residuals, measure},
    mates::{Mate, MateKind, On, Resolved, joint_parameter, mate, mate_name},
    residuals::{D, Geometry, Kind, residuals},
    solving::{Setting, Settings, largest, scale_of, settings_of},
};
use geop_ops_model::mate_entity::MateEntity;

thread_local! {
    static MATES_RESOLVED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// How many mates this thread has resolved — their elements found and
/// measured on the parts placed — ever. What any question about the mates
/// costs grows with this, so a test can bound the work an edit does by the
/// mates it touches, rather than time it.
pub fn mates_resolved() -> usize {
    MATES_RESOLVED.get()
}

/// The size of the placed parts `bodies` — at least one: what a mate
/// between them turns a turn into a length by.
fn size(reader: &Reader, bodies: &[&Option<String>]) -> GeopResult<Scalar> {
    largest((bodies.iter().copied().flatten()).map(|body| drawn(reader, body)))
}

/// The geometry `entity` of `mate`, a constraint of `kind`, is to it, in its
/// own part.
fn geometry_in(
    kind: &Kind,
    mate: &Mate,
    entity: &MateEntity,
    reader: &Reader,
) -> GeopResult<Geometry> {
    let shape = entity.shape(reader)?;
    kind.geometry(&shape)
        .ok_or_else(|| GeopError::new(format!("{} is not {}", entity.label(), mate.needs())))
}

/// The connector `entity` is to a joint, in its own part: what it makes one
/// of itself, or the frame it gives.
fn connector_in(entity: &MateEntity, reader: &Reader) -> GeopResult<Connector> {
    let shape = entity.shape(reader)?;
    let face = matches!(entity.on, Selected::Face(_)) && entity.at.is_none();
    match Connector::of(&shape, face) {
        Some(connector) => connector,
        None => {
            let frame = entity.frame(reader)?;
            Connector::new(*frame.origin(), *frame.w(), Some(*frame.u()))
        }
    }
}

/// What `what` makes of `entity`, on the body it moves with: in the frame
/// of the placed part it is of, or of the part itself.
fn on<T>(
    reader: &Reader,
    entity: &MateEntity,
    what: impl Fn(&MateEntity, &Reader) -> GeopResult<T>,
) -> GeopResult<On<T>> {
    Ok(On {
        body: (!entity.part().is_empty()).then(|| entity.part().to_string()),
        what: what(entity, reader)?,
        entity: entity.clone(),
    })
}

/// The mate `name`, `mate`, resolved in the part read: the generator of its
/// target [`crate::mate`]. A fixed mate fixes the part its element is of.
pub(crate) fn resolve(reader: &Reader, (name, mate): &(String, Mate)) -> GeopResult<Resolved> {
    MATES_RESOLVED.set(MATES_RESOLVED.get() + 1);
    let ctx = with_context!("mate {name:?}");
    let resolved = match mate.kind {
        MateKind::Constraint(kind) => {
            let [a, b] = mate
                .pair()
                .ok_or_else(|| GeopError::new("it needs two elements"))
                .with_context(ctx)?;
            let geometry = |e: &MateEntity, r: &Reader| geometry_in(&kind, mate, e, r);
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
                a: on(reader, a, connector_in)?,
                b: on(reader, b, connector_in)?,
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
                return Err(GeopError::new("it needs one element")).with_context(ctx);
            };
            if entity.part().is_empty() {
                return Err(GeopError::new(format!(
                    "{} is of this part itself, which never moves: a fixed mate holds a part placed in it",
                    entity.label()
                )))
                .with_context(ctx);
            }
            Resolved::Fixed {
                body: entity.part().to_string(),
            }
        }
    };
    Ok(resolved)
}

/// The mate `name` of the part `reader` reads, resolved.
fn resolved<'a>(reader: &Reader<'a>, name: &str) -> GeopResult<&'a Resolved> {
    reader.retrieve_element(&ElementReference::whole(mate(name)))
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
    fn pose(&mut self, reader: &Reader, body: &Option<String>) -> GeopResult<Option<Pose<D>>> {
        let Some(body) = body else {
            return Ok(None);
        };
        let Reading { read, by, twist } = self;
        let placement = placement(reader, body)?;
        Ok(Some(compose(&placement, &mut |state, pose| {
            read.insert(state.to_string(), Read::Pose);
            let pose = pose.map(D::cst);
            match *by == Some(state) {
                true => pose.compose(twist),
                false => pose,
            }
        })))
    }

    /// The coordinate `state`, as the state has it.
    fn coordinate(&mut self, reader: &Reader, state: &str) -> GeopResult<D> {
        self.read.insert(state.to_string(), Read::Coordinate);
        let StateValue::Number(value) = reader.retrieve_state(state) else {
            return Err(GeopError::new(format!(
                "the joint coordinate {state:?} holds no number"
            )));
        };
        Ok(match self.by == Some(state) {
            true => D::var(value, 0),
            false => D::cst(value),
        })
    }

    /// The coordinates `[angle, distance]` of the joint `name` of `kind`:
    /// zero for a motion it does not free.
    fn coordinates(&mut self, reader: &Reader, name: &str, kind: &JointKind) -> GeopResult<[D; 2]> {
        let mut out = [D::cst(Scalar::ZERO); 2];
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
        reader: &Reader,
        a: &On<T>,
        b: &On<T>,
    ) -> GeopResult<(D, [Option<Pose<D>>; 2])> {
        let scale = D::cst(size(reader, &[&a.body, &b.body])?);
        let poses = [self.pose(reader, &a.body)?, self.pose(reader, &b.body)?];
        Ok((scale, poses))
    }
}

/// A joint resolved: its kind, and its two ends.
type Joint<'a> = (&'a JointKind, &'a On<Connector>, &'a On<Connector>);

/// The joint `name` of the part read, resolved.
fn joint<'a>(reader: &Reader<'a>, name: &str) -> GeopResult<Joint<'a>> {
    match resolved(reader, name)? {
        Resolved::Joint { kind, a, b } => Ok((kind, a, b)),
        _ => Err(GeopError::new(format!("the mate {name:?} is no joint"))),
    }
}

/// The residuals of the mate `name`, `resolved`, where the state has its
/// parts and joints (see [`constrain`]).
fn residuals_of(
    reader: &Reader,
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
    reader: &Reader,
    bound: Bound,
    evaluate: impl Fn(&Reader, &mut Reading) -> GeopResult<Vec<D>>,
) -> GeopResult<Constraint> {
    // The step of a pose by its six variables, in its own frame.
    let twist = Pose::exp_with_derivatives(&Vector3::zero(), &Vector3::zero())?;
    let mut plain = Reading {
        read: BTreeMap::new(),
        by: None,
        twist,
    };
    let values = evaluate(reader, &mut plain)?;
    let mut jacobian = BTreeMap::new();
    for (state, read) in plain.read {
        let dimension = match (reader.retrieve_state(&state), read) {
            (StateValue::Pose(_), Read::Pose) => 6,
            (StateValue::Number(_), Read::Coordinate) => 1,
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

/// The constraint the mate `name` puts on the state. Its residuals are
/// lengths — a turn's the distance it moves a point as far from its axis as
/// its parts are large — that vanish where it holds.
fn constrain(reader: &Reader, name: &str) -> GeopResult<Constraint> {
    let resolved = resolved(reader, name)?;
    differentiated(reader, Bound::Zero, |reader, reading| {
        residuals_of(reader, name, resolved, reading)
    })
}

/// The name of the constraint that keeps the coordinate `state` of a joint
/// within its upper limit, or its lower: `….angle.max`.
fn limit_name(state: &str, upper: bool) -> String {
    format!("{state}.{}", if upper { "max" } else { "min" })
}

/// The constraint that keeps the coordinate of `motion` of the joint
/// `name` within its upper limit, or its lower: a limit (see
/// [`Bound::AtMost`]), as a length as [`constrain`]'s are.
fn limit(reader: &Reader, name: &str, motion: Motion, upper: bool) -> GeopResult<Constraint> {
    let (kind, a, b) = joint(reader, name)?;
    let [min, max] = kind.limits(motion);
    let bound = (if upper { max } else { min })
        .ok_or_else(|| GeopError::new(format!("the joint {name:?} has no such limit")))?;
    // An angle is turned into a length by the parts' size; a distance is
    // one, and is not multiplied by an exact one.
    let scale = match motion {
        Motion::Turn => Some(size(reader, &[&a.body, &b.body])?),
        Motion::Slide => None,
    };
    let state = joint_parameter(name, motion);
    differentiated(reader, Bound::AtMost, |reader, reading| {
        let value = reading.coordinate(reader, &state)?;
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

/// The constraints the mates `names`, resolved already in the part, put on
/// the state: each mate's own, named as the mate is, and its limits'. A
/// fixed mate puts none: it declares the part it holds fixed. The generator
/// of a step's constraints.
fn constraints_of_mates(
    reader: &Reader,
    names: &Vec<String>,
) -> GeopResult<BTreeMap<String, Constraint>> {
    let mut out = BTreeMap::new();
    for name in names {
        let resolved = resolved(reader, name)?;
        if let Resolved::Fixed { .. } = resolved {
            continue;
        }
        out.insert(name.clone(), constrain(reader, name)?);
        if let Resolved::Joint { kind, .. } = resolved {
            for motion in kind.motions() {
                let state = joint_parameter(name, motion);
                for (upper, given) in [false, true].into_iter().zip(kind.limits(motion)) {
                    if given.is_some() {
                        let constraint = limit(reader, name, motion, upper)?;
                        out.insert(limit_name(&state, upper), constraint);
                    }
                }
            }
        }
    }
    Ok(out)
}

/// The unknowns a step declares: the pose state `placed` of the part it
/// places, if it places one, held where it is as far as the mates let it;
/// and those its mates `names` declare: the part a fixed mate holds, and
/// each joint's coordinates. The generator of a step's unknowns.
fn unknowns_of_mates(
    reader: &Reader,
    (placed, names): &(Option<String>, Vec<String>),
) -> GeopResult<Vec<Unknown>> {
    let mut out = Vec::new();
    if let Some(placed) = placed {
        out.push(Unknown {
            state: placed.clone(),
            mobility: Mobility::Held,
            // Where the run has a part whose pose the state does not give.
            seed: Some(StateValue::Pose(Pose::identity())),
            unless: None,
        });
    }
    for name in names {
        match resolved(reader, name)? {
            Resolved::Fixed { body } => out.push(fixed(reader, body)?),
            Resolved::Joint { kind, .. } => {
                for motion in kind.motions() {
                    out.push(coordinate(reader, name, motion)?);
                }
            }
            Resolved::Constraint { .. } | Resolved::Coupling { .. } => {}
        }
    }
    Ok(out)
}

/// Defines what the mates `mates` of the step being run, each resolved
/// already in the part and called as `labels` says, put on the state: their
/// constraints and limits, the parts the fixed ones hold and the joints'
/// coordinates as values a solve moves — and, as values a user sets, each
/// joint's coordinates. The pose state `placed` of the part the step
/// places, if it places one, is a value a solve moves too.
pub(crate) fn define(
    ctx: &mut StepContext,
    placed: Option<&str>,
    mates: &[(String, String)],
) -> GeopResult<()> {
    let names: Vec<String> = mates.iter().map(|(name, _)| name.clone()).collect();
    let constrained = define_constraints(ctx, &names, constraints_of_mates);
    let unknowns = (placed.map(str::to_string), names);
    let declared = declare_unknowns(ctx, &unknowns, unknowns_of_mates);
    let mut set = Ok(());
    for (name, label) in mates {
        let is_joint = matches!(resolved(&ctx.reader(), name), Ok(Resolved::Joint { .. }));
        if is_joint {
            let named = (name.clone(), label.clone());
            set = set.and(ctx.produce(&settings_of(name), &named, settings));
        }
    }
    constrained.and(declared).and(set)
}

/// The mates of the part `reader` reads, resolved, by name, in the order
/// the steps made them.
pub fn mates_of<'a>(reader: &Reader<'a>) -> Vec<(String, &'a Resolved)> {
    (reader.targets_of::<Resolved>().into_iter())
        .filter_map(|target| {
            let name = mate_name(target.name())?.to_string();
            let resolved = reader
                .retrieve_element(&ElementReference::whole(target))
                .ok()?;
            Some((name, resolved))
        })
        .collect()
}

/// Where the connectors of the joint `a`–`b` are, as the part read has
/// them: in the world.
fn connectors_at(
    reader: &Reader,
    a: &On<Connector>,
    b: &On<Connector>,
) -> GeopResult<[Connector<D>; 2]> {
    let world = |on: &On<Connector>| -> GeopResult<Option<Pose<D>>> {
        (on.body.as_ref())
            .map(|body| Ok(locate(reader, body)?.world.map(D::cst)))
            .transpose()
    };
    let (wa, wb) = (world(a)?, world(b)?);
    Ok([a.what.placed(wa.as_ref()), b.what.placed(wb.as_ref())])
}

/// The coordinates `[angle, distance]` of a joint, measured where its parts
/// are — an angle at the turn nearest `near`.
fn measured(
    reader: &Reader,
    a: &On<Connector>,
    b: &On<Connector>,
    near: f64,
) -> GeopResult<[f64; 2]> {
    let [ca, cb] = connectors_at(reader, a, b)?;
    let [angle, distance] = measure(&ca, &cb);
    Ok([angle + 360.0 * ((near - angle) / 360.0).round(), distance])
}

/// What a fixed mate declares of the placed part `body` it holds: its
/// state, fixed. A part no state moves is fixed already, and so is a state
/// none has.
fn fixed(reader: &Reader, body: &str) -> GeopResult<Unknown> {
    Ok(Unknown {
        state: moved_by(reader, body)?.unwrap_or_else(|| body.to_string()),
        mobility: Mobility::Fixed,
        seed: None,
        unless: None,
    })
}

/// The coordinate of `motion` of the joint `name`, as a value a solve
/// moves: following its parts, and — where the joint does not hold —
/// started where they are, measured from them at the turn nearest where it
/// is: a seed, so a free choice, which the solve makes exact.
fn coordinate(reader: &Reader, name: &str, motion: Motion) -> GeopResult<Unknown> {
    let state = joint_parameter(name, motion);
    let near = match reader.retrieve_state(&state) {
        StateValue::Number(v) => v.to_f64(),
        StateValue::Pose(_) => 0.0,
    };
    let (_, a, b) = joint(reader, name)?;
    // Where the parts cannot be measured, there is nothing to start from.
    let seed = measured(reader, a, b, near).ok().map(|[angle, distance]| {
        StateValue::Number(Scalar::from_f64(match motion {
            Motion::Turn => angle,
            Motion::Slide => distance,
        }))
    });
    Ok(Unknown {
        state,
        mobility: Mobility::Follows,
        seed,
        unless: Some(name.to_string()),
    })
}

/// The coordinates of the joint `name`, called `label`, as values a user
/// sets, with their limits: the generator of its settings.
fn settings(reader: &Reader, (name, label): &(String, String)) -> GeopResult<Settings> {
    let (kind, a, b) = joint(reader, name)?;
    Ok(Settings {
        name: name.clone(),
        label: label.clone(),
        kind: kind.label(),
        between: vec![a.entity.label(), b.entity.label()],
        values: (kind.motions().into_iter())
            .map(|motion| {
                let [min, max] = kind.limits(motion);
                Setting {
                    state: joint_parameter(name, motion),
                    measure: motion.measure(),
                    min,
                    max,
                }
            })
            .collect(),
    })
}

/// Whether the constraint of the mate `name`, as `reader` reads the part,
/// holds.
pub(crate) fn mate_holds(reader: &Reader, name: &str) -> bool {
    let (Ok(constraints), Ok(tolerance)) = (
        Constraints::of(reader),
        scale_of(reader).and_then(tolerance),
    ) else {
        return false;
    };
    (constraints.get(name)).is_some_and(|constraint| constraint.holds(tolerance))
}
