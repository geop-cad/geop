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

use geop_core_math::{geop_error::GeopResult, scalars::Scalar};
use geop_core_solve::mates::{Connector, Geometry};
pub use geop_core_solve::mates::{CouplingKind, JointKind, Kind, Motion};
use geop_ops::{
    Design,
    operation::{Aspects, EntityRef},
    part::EntryKind,
};
use serde::{Deserialize, Serialize};

/// The kind of entry mates are in a part: by name, each a cell of its own.
pub struct Mates;

impl<S: Scalar> EntryKind<S> for Mates {
    const NAME: &'static str = "mates";
    type Value = Mate;

    fn holds(mates: &std::collections::BTreeMap<String, Mate>, instance: &str) -> bool {
        mates.values().any(|mate| {
            mate.is_fixed()
                && mate
                    .entities
                    .iter()
                    .any(|e| e.instance_path().as_deref() == Some(instance))
        })
    }

    fn describe(
        mates: &std::collections::BTreeMap<String, Mate>,
    ) -> std::collections::BTreeMap<String, serde_json::Value> {
        mates
            .iter()
            .map(|(name, mate)| {
                (
                    name.clone(),
                    serde_json::json!({ "kind": mate.kind.label() }),
                )
            })
            .collect()
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
    Constraint(Kind<Design>),
    Joint(JointKind<Design>),
    Coupling(CouplingKind<Design>),
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
            MateKind::Coupling(kind) => kind.label(),
            MateKind::Anchor(Anchor::Fixed) => "Fixed",
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

    /// The part `entity` is of, held fixed: any entity of it will do — the
    /// part's frame [`crate::ORIGIN`], say.
    pub fn fixed(entity: EntityRef) -> Self {
        Mate {
            kind: MateKind::Anchor(Anchor::Fixed),
            entities: vec![entity],
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
            MateKind::Anchor(_) => self.entities.len() == 1,
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
            MateKind::Joint(_) => {
                "a face, an edge, a point or a datum: where it is picked, a frame is put"
            }
            MateKind::Anchor(_) => "an entity of the part to hold",
            MateKind::Coupling(CouplingKind::Gear { .. }) => "two joints that turn",
            MateKind::Coupling(_) => "a joint that turns, then one that slides",
        }
    }

    /// The geometry `aspects` contributes to a constraint of `kind`: for one
    /// that touches or keeps a distance, a point before a line before a
    /// plane — the center of a circle is a point; for a concentric one, an
    /// axis; for one that turns, a direction.
    pub(crate) fn geometry<S: Scalar>(
        kind: &Kind<Design>,
        aspects: &Aspects<S>,
    ) -> Option<Geometry<S>> {
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

    /// The connector `aspects` gives a joint where it is made of what the
    /// entity is — an axis through a point it defines: a circular edge's
    /// center and normal, a sketch circle's, or the origin and `z` axis of
    /// a frame, turns measured from its `x` axis. Of anything else, there is
    /// none: it is used by the frame it gives (see [`Aspects::frame_on`]).
    pub(crate) fn connector<S: Scalar>(aspects: &Aspects<S>) -> Option<GeopResult<Connector<S>>> {
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
    /// What it joins, as the entities its frames are on read: the part it
    /// holds, then the part it moves.
    pub between: Vec<String>,
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
