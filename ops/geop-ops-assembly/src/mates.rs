//! Mates: what holds the parts placed in a part together — two entities,
//! each of a placed part or of the part itself, and what they must be to
//! each other (see [`MateKind`]) — and solving them, by moving the placed
//! parts (see [`crate::mechanism`]).
//!
//! What a mate asks is said by [`crate::residuals`] and [`crate::joints`].
//! What is added here is how an element ([`MateEntity`]) becomes a point, a
//! line, a plane or a joint's connector of a placed part: an element of a
//! placed part moves with that part; an element of the part itself never
//! moves.
//!
//! A joint is made between two frames, one on each part, which a click puts
//! on whatever is picked — a face, an edge, a point or a datum (see
//! [`MateEntity::frame`]): it turns about, or slides along, their common `z`
//! axis, the second part's frame moving on the first's.
//!
//! A joint's coordinates — how far it is turned and slid — are parameters of
//! the part's state, like the poses of its placed parts: named after the
//! mate, `add_part(arm2,m1).angle` (see [`joint_parameter`]). Solving sets
//! them, and a program sets them to place a mechanism at a pose. A joint
//! whose coordinate the state does not give starts where its parts are.

pub use crate::{
    joints::{Connector, CouplingKind, Joint, JointKind, Limits, MOTIONS, Motion, Transmission},
    residuals::{Geometry, Kind},
};
pub use geop_ops_model::mate_entity::MateEntity;

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::{HasElements, Kind as ElementKind, whole},
    target::{Target, TargetReference},
};
use geop_ops_model::selection::Role;
use serde::{Deserialize, Serialize};

/// `roles` in words: `a point, a line or a plane`.
fn describe_roles(roles: &[Role]) -> String {
    match roles {
        [] => String::new(),
        [only] => only.describe().to_string(),
        [rest @ .., last] => {
            let rest: Vec<&str> = rest.iter().map(|r| r.describe()).collect();
            format!("{} or {}", rest.join(", "), last.describe())
        }
    }
}

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

    /// What an element of a mate of this kind can be, for a constraint:
    /// what its geometry is taken from (see [`Kind::roles`]). A joint puts
    /// a frame on any element; a coupling ties joints, and a fixed mate is
    /// given its element by the step: neither is picked by role.
    pub fn roles(&self) -> &'static [Role] {
        match self {
            MateKind::Constraint(kind) => kind.roles(),
            MateKind::Joint(_) | MateKind::Coupling(_) | MateKind::Anchor(_) => &[],
        }
    }
}

/// The name of the parameter that is the coordinate of `motion` of the
/// joint named `mate`: `add_part(arm2,m1).angle` — in degrees — or
/// `.distance`.
pub fn joint_parameter(mate: &str, motion: Motion) -> String {
    format!("{mate}.{}", motion.name())
}

/// The name of the parameter `foot` of the foot point a constraint named
/// `mate` keeps its follower at (see [`Kind::feet`]): `add_part(cam,m1).t`.
pub fn foot_parameter(mate: &str, foot: &str) -> String {
    format!("{mate}.{foot}")
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
    pub entities: Vec<MateEntity>,
    /// The joints a coupling ties, by their names in the part:
    /// `add_part(gear2,m1)`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub joints: Vec<String>,
}

impl Mate {
    /// A constraint of `kind` between `entities`.
    pub fn constraint(kind: Kind, entities: Vec<MateEntity>) -> Self {
        Mate {
            kind: MateKind::Constraint(kind),
            entities,
            joints: Vec::new(),
        }
    }

    /// A joint of `kind` between `entities`: the first part's connector,
    /// then the second's.
    pub fn joint(kind: JointKind, entities: Vec<MateEntity>) -> Self {
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
    pub fn pair(&self) -> Option<[&MateEntity; 2]> {
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
            MateKind::Constraint(kind) => match (kind.needs(0), kind.needs(1)) {
                (Some(first), Some(second)) => format!("first {first}, then {second}"),
                _ => describe_roles(self.kind.roles()),
            },
            MateKind::Joint(_) => {
                "a face, an edge, a point or a datum: where it is picked, a frame is put"
                    .to_string()
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
    pub entity: MateEntity,
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
        a: On<Connector>,
        b: On<Connector>,
    },
    /// The joints it ties, by their names.
    Coupling {
        kind: CouplingKind,
        joints: [String; 2],
    },
    /// The placed part `body` held where it is.
    Fixed { body: String },
}

impl Resolved {
    /// The entities it is between: none for a coupling, which is between
    /// joints, or for a part held where it is.
    pub fn entities(&self) -> Vec<MateEntity> {
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

/// Each mate's own.
impl Target for Resolved {}

/// Read whole.
impl ElementKind for Resolved {
    const NAME: &'static str = "mate";
}

impl HasElements<Resolved> for Resolved {
    type Element<'a> = &'a Resolved;

    fn element(&self, name: &str) -> GeopResult<&Resolved> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}
