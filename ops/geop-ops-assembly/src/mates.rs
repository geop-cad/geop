//! Mates: what holds the parts placed in a part together — two entities,
//! each of a placed part or of the part itself, and what they must be to
//! each other (see [`MateKind`]) — and solving them, by moving the placed
//! parts (see [`crate::mechanism`]).
//!
//! What a mate asks is said by [`crate::residuals`] and [`crate::joints`].
//! What is added here is how an entity, named like any other
//! ([`EntityRef`]), becomes a point, a line, a plane or a joint's connector
//! of a placed part: an entity of a placed part — named behind the
//! instance's name — moves with that part; an entity of the part itself
//! never moves.
//!
//! A joint is made between two frames, one on each part, which a click puts
//! on whatever is picked — a face, an edge, a point or a datum (see
//! [`EntityRef::Frame`] and [`Aspects::frame_on`]): it turns about, or slides
//! along, their common `z` axis, the second part's frame moving on the
//! first's.
//!
//! A joint's coordinates — how far it is turned and slid — are parameters of
//! the part's state, like the poses of its placed parts: named after the
//! mate, `add_part(arm2,m1).angle` (see [`joint_parameter`]). Solving sets
//! them, and a program sets them to place a mechanism at a pose. A joint
//! whose coordinate the state does not give starts where its parts are.

pub use crate::joints::{Connector, CouplingKind, JointKind, Motion, Transmission};
pub use crate::residuals::{Geometry, Kind};
use geop_core_program::target::TargetReference;
use geop_ops::{
    Design,
    operation::{EntityRef, INSTANCE_SEPARATOR, Role, describe_roles},
};
use serde::{Deserialize, Serialize};

/// What a mate asks: that two entities touch, line up or keep a distance
/// (a constraint); that one part may only turn or slide against another
/// about an axis (a joint); or that two joints move together (a coupling).
/// Serializes flat, by the kind's own `type`: `{"type": "revolute", "max":
/// 90.0}`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum MateKind {
    Constraint(Kind),
    Joint(JointKind),
    Coupling(CouplingKind),
    Anchor(Anchor),
}

/// What holds a part where it is, whatever the other mates ask: the part
/// one entity of the mate is of never moves in a solve.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Anchor {
    Fixed,
}

impl MateKind {
    pub fn label(&self) -> &'static str {
        match self {
            MateKind::Constraint(kind) => kind.label(),
            MateKind::Joint(kind) => kind.label(),
            MateKind::Coupling(kind) => kind.transmission.label(),
            MateKind::Anchor(Anchor::Fixed) => "Fixed",
        }
    }

    /// What an entity of a mate of this kind can be: of a constraint, what
    /// its geometry is taken from (see [`Kind::roles`]); of a joint, where a
    /// frame is put. A coupling ties joints, and a fixed mate is given its
    /// entity by the step: neither is picked.
    pub fn roles(&self) -> &'static [Role] {
        match self {
            MateKind::Constraint(kind) => kind.roles(),
            MateKind::Joint(_) => &[Role::Frame],
            MateKind::Coupling(_) | MateKind::Anchor(_) => &[],
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
    pub fn constraint(kind: Kind, entities: Vec<EntityRef>) -> Self {
        Mate {
            kind: MateKind::Constraint(kind),
            entities,
            joints: Vec::new(),
        }
    }

    /// A joint of `kind` between `entities`: the first part's connector,
    /// then the second's.
    pub fn joint(kind: JointKind, entities: Vec<EntityRef>) -> Self {
        Mate {
            kind: MateKind::Joint(kind),
            entities,
            joints: Vec::new(),
        }
    }

    /// A fixed mate of no entity yet: in a step that places a part, it
    /// holds that part (see `AddPart`).
    pub fn fixed_here() -> Self {
        Mate {
            kind: MateKind::Anchor(Anchor::Fixed),
            entities: Vec::new(),
            joints: Vec::new(),
        }
    }

    /// Whether it holds a part fixed.
    pub fn is_fixed(&self) -> bool {
        matches!(self.kind, MateKind::Anchor(Anchor::Fixed))
    }

    /// A coupling of `kind` between the joints named `joints`.
    pub fn coupling(kind: CouplingKind, joints: Vec<String>) -> Self {
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
            MateKind::Anchor(_) => self.entities.len() == 1,
            _ => self.pair().is_some(),
        }
    }

    /// What an entity has to be for this kind of mate, in words — for a
    /// coupling, what its joints have to be.
    pub fn needs(&self) -> String {
        match self.kind {
            MateKind::Constraint(_) => describe_roles(self.kind.roles()),
            MateKind::Joint(_) => {
                format!(
                    "{}: where it is picked, a frame is put",
                    Role::Frame.describe()
                )
            }
            MateKind::Anchor(_) => "an entity of the part to hold".to_string(),
            MateKind::Coupling(kind) => match kind.transmission.motions() {
                [Motion::Turn, Motion::Turn] => "two joints that turn".to_string(),
                [a, b] => format!("a joint that {}s, then one that {}s", a.verb(), b.verb()),
            },
        }
    }
}

/// What an entity of a mate is to a solve: `what` it is, in the frame of
/// the body it moves with — the placed part it is of, however deep, by
/// name; none for the part itself, which never moves — and the entity, as
/// the part names it, to say what the mate joins.
#[derive(Clone, Debug, PartialEq)]
pub struct On<T> {
    pub body: Option<String>,
    pub what: T,
    pub entity: EntityRef,
}

/// A mate as a solve takes it, its entities resolved where their parts
/// are: the value of its target ([`mate`]). It reads no state, so a part
/// moved does not resolve it again.
#[derive(Clone, Debug, PartialEq)]
pub enum Resolved {
    Constraint {
        kind: Kind,
        a: On<Geometry>,
        b: On<Geometry>,
    },
    Joint {
        kind: JointKind,
        a: On<Connector<Design>>,
        b: On<Connector<Design>>,
    },
    /// The joints it ties, by their names.
    Coupling {
        kind: CouplingKind,
        joints: [String; 2],
    },
    /// The placed part `body` held where it is.
    Fixed { body: String },
}

impl<T: Clone> On<T> {
    /// It as the part that places the part it is of — as `instance` — has
    /// it: its body named behind the instance's name, or, of the part
    /// itself, which held still there, now the instance.
    fn behind(&self, instance: &str) -> Self {
        On {
            body: Some(match &self.body {
                Some(body) => format!("{instance}{INSTANCE_SEPARATOR}{body}"),
                None => instance.to_string(),
            }),
            what: self.what.clone(),
            entity: self.entity.in_instance(instance),
        }
    }
}

impl Resolved {
    /// The mate as the part that places the part it is a mate of — as
    /// `instance` — has it: its bodies and joints named behind the
    /// instance's name, and the part itself, which held still there, now
    /// the instance. None for a fixed one: where a placed part is fixed is
    /// for the part placing it to say.
    pub fn behind(&self, instance: &str) -> Option<Resolved> {
        Some(match self {
            Resolved::Constraint { kind, a, b } => Resolved::Constraint {
                kind: *kind,
                a: a.behind(instance),
                b: b.behind(instance),
            },
            Resolved::Joint { kind, a, b } => Resolved::Joint {
                kind: *kind,
                a: a.behind(instance),
                b: b.behind(instance),
            },
            Resolved::Coupling { kind, joints } => Resolved::Coupling {
                kind: *kind,
                joints: joints
                    .clone()
                    .map(|joint| format!("{instance}{INSTANCE_SEPARATOR}{joint}")),
            },
            Resolved::Fixed { .. } => return None,
        })
    }

    /// The entities it is between: none for a coupling, which is between
    /// joints, or for a part held where it is.
    pub fn entities(&self) -> Vec<EntityRef> {
        match self {
            Resolved::Constraint { a, b, .. } => vec![a.entity.clone(), b.entity.clone()],
            Resolved::Joint { a, b, .. } => vec![a.entity.clone(), b.entity.clone()],
            Resolved::Coupling { .. } | Resolved::Fixed { .. } => Vec::new(),
        }
    }
}

/// What the target of a mate is named, before the mate's name.
const PREFIX: &str = "mate:";

/// The mate `name`, resolved.
pub fn mate(name: &str) -> TargetReference<Resolved> {
    TargetReference::new(format!("{PREFIX}{name}"))
}

/// The name of the mate whose target is named `target`, if it is one.
pub fn mate_name(target: &str) -> Option<&str> {
    target.strip_prefix(PREFIX)
}
