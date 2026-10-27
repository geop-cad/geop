//! Joints and couplings: mates that say how one part may move against
//! another, rather than only what touches what.
//!
//! A joint holds a [`Connector`] of one part on a connector of another — an
//! origin, an axis through it and a direction square to the axis that
//! turns are measured from — and lets the second part turn about the first
//! one's axis, slide along it, or both, as its [`JointKind`] says. How far
//! it is turned and slid are the joint's **coordinates**: values of the
//! program's state, solved with the poses, so that a joint can be set to a
//! value, held there, kept within limits, and coupled to another joint's.
//!
//! A coupling ([`CouplingKind`]) ties a coordinate of one joint to one of
//! another — or of the same one — linearly: a gear pair, a rack and its
//! pinion, a screw.
//!
//! Like [`crate::residuals`], every residual is computed in dual numbers.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    plain::{cross, dot, sub},
    primitives::Pose,
    scalars::{Number, Scalar, as_f64},
    vector::Vector3,
};
use geop_ops_model::selection::Shape;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

use crate::residuals::{D, Placing, parallel};

/// A joint's end on a body, in that body's frame — or, placed, where its
/// body puts it: where its axis passes, the axis, and the direction its
/// turns are measured from — unit vectors, the reference square to the
/// axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Connector<T: Number = Scalar> {
    pub origin: Vector3<T>,
    pub axis: Vector3<T>,
    pub reference: Vector3<T>,
}

impl Connector {
    /// The connector at `origin`, along `axis` (any non-zero length), its
    /// turns measured from `reference` made square to the axis — or, with
    /// none given, from whichever of the body's own axes `x`, `y`, `z` runs
    /// least along it, made square to it: a free choice, and the
    /// well-conditioned one. So two bodies drawn the same way round, mated
    /// on the same axis of each, are at a turn of zero.
    pub fn new(origin: Vector3, axis: Vector3, reference: Option<Vector3>) -> GeopResult<Self> {
        let axis = axis
            .normalize()
            .map_err(|e| e.with_context(format!("the axis {axis:?} of a joint")))?;
        let reference = match reference {
            None => axis.across()?[0],
            Some(reference) => {
                let square = reference.sub(&axis.prod_scalar(axis.prod_dot(&reference)));
                square.normalize().map_err(|e| {
                    e.with_context(format!(
                        "the reference {reference:?} of a joint runs along its axis {axis:?}"
                    ))
                })?
            }
        };
        Ok(Connector {
            origin,
            axis,
            reference,
        })
    }

    /// The connector `aspects` gives a joint where it is made of what the
    /// entity is — an axis through a point it defines: a circular edge's
    /// center and normal, a sketch circle's, or the origin and `z` axis of
    /// a frame, turns measured from its `x` axis. Of anything else, there is
    /// none: it is used by the frame it gives (see [`geop_ops_model::mate_entity::MateEntity::frame`]).
    pub fn of(aspects: &Shape, face: bool) -> Option<GeopResult<Self>> {
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
        if let (Some(round), false) = (&aspects.round, face) {
            return Some(Connector::new(round.point, round.direction, None));
        }
        None
    }

    /// Where `pose` puts it — or, with none, where it is: of a part that
    /// never moves.
    pub fn placed(&self, pose: Option<&Pose<D>>) -> Connector<D> {
        let placing = Placing::new(pose);
        let turn = |d: &Vector3| placing.turn(&d.map(D::cst));
        Connector {
            origin: placing.point(&self.origin),
            axis: turn(&self.axis),
            reference: turn(&self.reference),
        }
    }
}

/// One of the ways a joint may let its second frame move on its first:
/// turning about the first frame's `z` axis, or sliding along its `z`, `x`
/// or `y` axis. Each it frees is a **coordinate** of the joint, called as
/// it serializes ([`Motion::name`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub enum Motion {
    #[serde(rename = "angle")]
    Turn,
    #[serde(rename = "distance")]
    Slide,
    #[serde(rename = "x")]
    SlideX,
    #[serde(rename = "y")]
    SlideY,
}

/// How many motions there are: what a joint's coordinates are indexed by.
pub const MOTIONS: usize = 4;

impl Motion {
    pub const ALL: [Motion; MOTIONS] =
        [Motion::Turn, Motion::Slide, Motion::SlideX, Motion::SlideY];

    /// Its place in [`Motion::ALL`]: what a joint's coordinates and limits
    /// are indexed by.
    pub fn index(self) -> usize {
        self as usize
    }

    /// What its coordinate is called: `angle` (in degrees), `distance` —
    /// along the axis — `x`, `y`.
    pub fn name(self) -> &'static str {
        match self {
            Motion::Turn => "angle",
            Motion::Slide => "distance",
            Motion::SlideX => "x",
            Motion::SlideY => "y",
        }
    }

    /// What its coordinate measures, as a user sets it.
    pub fn measure(self) -> crate::solving::Measure {
        match self {
            Motion::Turn => crate::solving::Measure::Angle,
            Motion::Slide | Motion::SlideX | Motion::SlideY => crate::solving::Measure::Distance,
        }
    }

    /// What a joint that frees it does: `turn`, `slide`.
    pub fn verb(self) -> &'static str {
        match self {
            Motion::Turn => "turn",
            Motion::Slide | Motion::SlideX | Motion::SlideY => "slide",
        }
    }

    /// A coordinate of it in its natural units — radians for an angle,
    /// given in degrees; a length for a distance.
    pub fn natural(self, value: D) -> GeopResult<D> {
        match self {
            Motion::Turn => value.degrees_to_radians(),
            Motion::Slide | Motion::SlideX | Motion::SlideY => Ok(value),
        }
    }
}

/// How a joint lets its second frame turn against its first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Turning {
    /// Not at all: the frames' axes keep where they are to each other.
    Not,
    /// About the first frame's `z` axis, by the joint's angle.
    AboutZ,
    /// About the first frame's `z` axis and about the second's, which are
    /// kept square to each other: the cross of a universal joint. How far
    /// it is turned is no coordinate: two angles of a cross go singular
    /// where it folds over, as Euler angles do (see AGENTS.md, "Degenerate
    /// parametrizations").
    Cross,
    /// Any way about the second frame's origin. No coordinate either, for
    /// the same reason.
    Free,
}

/// The joints there are, each a row of one table — how it lets the second
/// frame turn, and which motions it frees ([`Joint::frees`]): every joint
/// is the second frame held on the first within those, so that one
/// formulation ([`joint_residuals`]) holds them all.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Joint {
    /// Turns about the axis, nothing else: a hinge, a robot's joint.
    Revolute,
    /// Slides along the axis, nothing else.
    Slider,
    /// Turns about the axis and slides along it: a shaft in a bearing.
    Cylindrical,
    /// Slides in the first frame's `xy` plane and turns about its `z`: a
    /// part sliding on a table.
    Planar,
    /// Slides along the first frame's `x` axis and turns about its `z`: a
    /// pin in a straight slot.
    PinSlot,
    /// Turns any way about the frames' common origin: a ball in its socket.
    Ball,
    /// Turns about either frame's `z` axis, the two kept square and meeting
    /// at the origins: a universal joint's cross.
    Universal,
    /// Neither: the frames coincide, and the two bodies move as one.
    Fastened,
}

impl Joint {
    /// Every joint, in the order they are offered.
    pub const ALL: [Joint; 8] = [
        Joint::Revolute,
        Joint::Slider,
        Joint::Cylindrical,
        Joint::Planar,
        Joint::PinSlot,
        Joint::Ball,
        Joint::Universal,
        Joint::Fastened,
    ];

    /// The table: how it lets the second frame turn, and the motions it
    /// frees as coordinates — the turn about `z` among them where it turns
    /// about `z`.
    pub fn frees(self) -> (Turning, &'static [Motion]) {
        use Motion::*;
        match self {
            Joint::Revolute => (Turning::AboutZ, &[Turn]),
            Joint::Slider => (Turning::Not, &[Slide]),
            Joint::Cylindrical => (Turning::AboutZ, &[Turn, Slide]),
            Joint::Planar => (Turning::AboutZ, &[Turn, SlideX, SlideY]),
            Joint::PinSlot => (Turning::AboutZ, &[Turn, SlideX]),
            Joint::Ball => (Turning::Free, &[]),
            Joint::Universal => (Turning::Cross, &[]),
            Joint::Fastened => (Turning::Not, &[]),
        }
    }

    /// Its name, as it serializes: `pin_slot`.
    pub fn name(self) -> &'static str {
        match self {
            Joint::Revolute => "revolute",
            Joint::Slider => "slider",
            Joint::Cylindrical => "cylindrical",
            Joint::Planar => "planar",
            Joint::PinSlot => "pin_slot",
            Joint::Ball => "ball",
            Joint::Universal => "universal",
            Joint::Fastened => "fastened",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Joint::Revolute => "Revolute",
            Joint::Slider => "Slider",
            Joint::Cylindrical => "Cylindrical",
            Joint::Planar => "Planar",
            Joint::PinSlot => "Pin in slot",
            Joint::Ball => "Ball",
            Joint::Universal => "Universal",
            Joint::Fastened => "Fastened",
        }
    }

    /// What it does, in words, as the editor offers it.
    pub fn doc(self) -> &'static str {
        match self {
            Joint::Revolute => {
                "The second part turns about the first one's frame's z axis, and nothing else — a hinge: pick a frame on each part."
            }
            Joint::Slider => {
                "The second part slides along the first one's frame's z axis, and nothing else: pick a frame on each part."
            }
            Joint::Cylindrical => {
                "The second part turns about the first one's frame's z axis and slides along it — a shaft in a bearing."
            }
            Joint::Planar => {
                "The second part slides in the plane of the first one's frame — along its x and y — and turns about its z: a part on a table."
            }
            Joint::PinSlot => {
                "The second part slides along the first one's frame's x axis, the red arrow, and turns about its z: a pin in a straight slot."
            }
            Joint::Ball => {
                "The second part turns any way about the first one's frame's origin, the two origins one: a ball in its socket."
            }
            Joint::Universal => {
                "The second part turns about the first one's frame's z axis and about its own, the two kept square and meeting: a universal joint's cross."
            }
            Joint::Fastened => {
                "The two parts are held together, their frames one: pick a frame on each part."
            }
        }
    }
}

/// The limits a joint's coordinate is kept within, either missing — an
/// angle's in degrees.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    #[serde(
        default,
        with = "as_f64::option",
        skip_serializing_if = "Option::is_none"
    )]
    pub min: Option<Scalar>,
    #[serde(
        default,
        with = "as_f64::option",
        skip_serializing_if = "Option::is_none"
    )]
    pub max: Option<Scalar>,
}

impl Limits {
    /// No limit either way.
    pub const NONE: Limits = Limits {
        min: None,
        max: None,
    };

    /// Within `min` and `max`.
    pub fn between(min: Scalar, max: Scalar) -> Self {
        Limits {
            min: Some(min),
            max: Some(max),
        }
    }

    pub fn is_none(&self) -> bool {
        self.min.is_none() && self.max.is_none()
    }
}

/// How a joint lets its second body move against its first: which joint
/// of the table it is ([`Joint`]), and the limits of its coordinates.
/// Serializes flat, the limits of only those limited, by name: `{"type":
/// "revolute", "limits": {"angle": {"min": -90.0, "max": 90.0}}}`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(from = "JointKindFile", into = "JointKindFile")]
pub struct JointKind {
    pub joint: Joint,
    /// The limits of each coordinate, by [`Motion::index`].
    pub limits: [Limits; MOTIONS],
}

/// [`JointKind`] as a file has it.
#[derive(Serialize, Deserialize)]
struct JointKindFile {
    #[serde(rename = "type")]
    joint: Joint,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    limits: BTreeMap<Motion, Limits>,
}

impl From<JointKindFile> for JointKind {
    fn from(file: JointKindFile) -> Self {
        let mut kind = JointKind::new(file.joint);
        for (motion, limits) in file.limits {
            kind.limits[motion.index()] = limits;
        }
        kind
    }
}

impl From<JointKind> for JointKindFile {
    fn from(kind: JointKind) -> Self {
        JointKindFile {
            joint: kind.joint,
            limits: (Motion::ALL.into_iter())
                .map(|m| (m, kind.limits[m.index()]))
                .filter(|(_, limits)| !limits.is_none())
                .collect(),
        }
    }
}

impl JointKind {
    /// The joint `joint`, unlimited.
    pub const fn new(joint: Joint) -> Self {
        JointKind {
            joint,
            limits: [Limits::NONE; MOTIONS],
        }
    }

    /// It with the coordinate of `motion` kept within `limits`.
    pub fn limited(mut self, motion: Motion, limits: Limits) -> Self {
        self.limits[motion.index()] = limits;
        self
    }

    /// How it lets the second frame turn (see [`Joint::frees`]).
    pub fn turning(&self) -> Turning {
        self.joint.frees().0
    }

    /// Whether it frees `motion`, as a coordinate.
    pub fn moves(&self, motion: Motion) -> bool {
        self.joint.frees().1.contains(&motion)
    }

    /// The motions it frees as coordinates.
    pub fn motions(&self) -> Vec<Motion> {
        self.joint.frees().1.to_vec()
    }

    /// The limits of `motion`'s coordinate, `[min, max]`, either missing.
    pub fn limits(&self, motion: Motion) -> [Option<Scalar>; 2] {
        let Limits { min, max } = self.limits[motion.index()];
        [min, max]
    }

    pub fn label(&self) -> &'static str {
        self.joint.label()
    }

    /// Checks it limits only coordinates it frees, and each the right way
    /// round.
    pub fn validate(&self) -> GeopResult<()> {
        let label = self.label().to_lowercase();
        for motion in Motion::ALL {
            let [min, max] = self.limits(motion);
            if !self.moves(motion) && (min.is_some() || max.is_some()) {
                return Err(GeopError::new(format!(
                    "a {label} joint has no {} to limit: it frees {}",
                    motion.name(),
                    describe_motions(&self.motions()),
                )));
            }
            if let (Some(min), Some(max)) = (min, max)
                && min.definitely_greater(max)
            {
                return Err(GeopError::new(format!(
                    "the {} limits of a {label} joint are the wrong way round: {min:?} > {max:?}",
                    motion.name(),
                )));
            }
        }
        Ok(())
    }
}

/// `motions` in words: `angle and distance`, or `no coordinate`.
fn describe_motions(motions: &[Motion]) -> String {
    match motions {
        [] => "no coordinate".to_string(),
        [one] => one.name().to_string(),
        [rest @ .., last] => {
            let rest: Vec<&str> = rest.iter().map(|m| m.name()).collect();
            format!("{} and {}", rest.join(", "), last.name())
        }
    }
}

/// A joint's coordinates where they are, by [`Motion::index`]: zero for a
/// motion it does not free.
pub type Coordinates = [D; MOTIONS];

/// The residuals of a joint of `kind` between `a` — the first part's
/// connector — and `b`, at the coordinates `at` (an angle in degrees),
/// where they are: each a length, a direction's scaled by the part's size
/// `scale`.
///
/// `b`'s origin is `a`'s moved along `a`'s axis by the distance, and along
/// its reference and the direction square to both by `x` and `y`; and as
/// the joint turns, `b`'s axis is parallel to `a`'s — facing either way, as
/// a pin in a hole may — with `b`'s reference `a`'s turned about `a`'s axis
/// by the angle (zero for a joint that does not turn), or `b`'s axis square
/// to `a`'s for a cross, or turned any way for a ball. Fails — infeasible —
/// where `b` is turned half a turn from what the angle says.
pub fn joint_residuals(
    kind: &JointKind,
    a: &Connector<D>,
    b: &Connector<D>,
    at: &Coordinates,
    scale: D,
) -> GeopResult<Vec<D>> {
    let (za, ra) = (&a.axis, &a.reference);
    let l = scale;
    let ya = za.prod_cross(ra);
    let mut origin = a.origin.add(&za.prod_scalar(at[Motion::Slide.index()]));
    // Only a joint that slides across its axis adds those slides: an exact
    // zero added rounds a hair outward.
    for (motion, along) in [(Motion::SlideX, ra), (Motion::SlideY, &ya)] {
        if kind.moves(motion) {
            origin = origin.add(&along.prod_scalar(at[motion.index()]));
        }
    }
    let mut out: Vec<D> = b.origin.sub(&origin).to_array().to_vec();
    match kind.turning() {
        Turning::Not | Turning::AboutZ => {
            // The axes parallel: `b`'s axis has no part along `a`'s
            // reference or the direction square to both.
            out.extend(parallel(&[*ra, ya], &b.axis, l));
            // How far `b`'s reference is turned past the angle, `φ - θ`, as
            // `2 tan((φ - θ) / 2)` from its sine and cosine: zero only at
            // `φ = θ` — a sine alone is zero half a turn away too — and one
            // row, of slope one there. Measured as `rb - u`, `b`'s reference
            // less `a`'s turned by the angle, the part of it along `u` would
            // vanish to second order only: a row that, at a solution the
            // minimizer reached to its tolerance, has a slope as small as
            // that tolerance and still counts as a constraint, pinning the
            // joint.
            let angle = Motion::Turn.natural(at[Motion::Turn.index()])?;
            let (cos, sin) = (angle.cos(), angle.sin());
            let u = ra.prod_scalar(cos).add(&ya.prod_scalar(sin));
            let t = ya.prod_scalar(cos).sub(&ra.prod_scalar(sin));
            let past = b
                .reference
                .prod_dot(&t)
                .div(D::ONE.add(b.reference.prod_dot(&u)))?;
            out.push(past.mul(D::TWO).mul(l));
        }
        // One row, of slope one where the axes are square.
        Turning::Cross => out.push(za.prod_dot(&b.axis).mul(l)),
        Turning::Free => {}
    }
    Ok(out)
}

/// Where the coordinates of a joint between `a` and `b` are, measured from
/// where its ends are, by [`Motion::index`]: the angle `b`'s reference is
/// turned from `a`'s about `a`'s axis, in degrees in `(-180, 180]`, and how
/// far `b`'s origin is along `a`'s axis, its reference and the direction
/// square to both. Only a seed for a solve, which finds the exact values:
/// read off midpoints.
pub fn measure(a: &Connector<D>, b: &Connector<D>) -> [f64; MOTIONS] {
    let f = |v: &Vector3<D>| v.to_array().map(|x| x.to_f64());
    let (oa, za, ra, ob, rb) = (
        f(&a.origin),
        f(&a.axis),
        f(&a.reference),
        f(&b.origin),
        f(&b.reference),
    );
    let ya = cross(za, ra);
    let off = sub(ob, oa);
    let angle = dot(rb, ya).atan2(dot(rb, ra)).to_degrees();
    [angle, dot(off, za), dot(off, ra), dot(off, ya)]
}

/// What a coupling transmits between its two joints: the motions of each
/// it ties ([`Transmission::motions`]) and what its value is
/// ([`CouplingKind::factor`]) — presets of one linear relation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Transmission {
    /// Two turning joints: the first turns `value` times — the ratio — for
    /// every turn of the second. A pinion of 10 teeth driving a wheel of 20
    /// is a ratio of 2; a pair of external gears turns opposite ways.
    Gear,
    /// Two turning joints, as for a gear, by a belt or a chain round two
    /// pulleys: they turn the same way.
    Belt,
    /// A turning joint, the pinion, and a sliding one, the rack: the rack
    /// moves the length of arc the pinion's pitch circle, of radius
    /// `value`, rolls off.
    RackPinion,
    /// A turning joint and a sliding one — the turn and the slide of one
    /// cylindrical joint, usually: every turn moves it by `value`, the
    /// lead.
    Screw,
    /// Two sliding joints: the second slides `value` times as far as the
    /// first — a lever's two ends, a cable round a pulley.
    Linear,
}

impl Transmission {
    /// Every transmission, as offered.
    pub const ALL: [Transmission; 5] = [
        Transmission::Gear,
        Transmission::Belt,
        Transmission::RackPinion,
        Transmission::Screw,
        Transmission::Linear,
    ];

    /// The motions of the two joints it ties.
    pub fn motions(self) -> [Motion; 2] {
        match self {
            Transmission::Gear | Transmission::Belt => [Motion::Turn, Motion::Turn],
            Transmission::RackPinion | Transmission::Screw => [Motion::Turn, Motion::Slide],
            Transmission::Linear => [Motion::Slide, Motion::Slide],
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Transmission::Gear => "Gear",
            Transmission::Belt => "Belt or chain",
            Transmission::RackPinion => "Rack and pinion",
            Transmission::Screw => "Screw",
            Transmission::Linear => "Linear",
        }
    }
}

/// How a coupling ties its two joints' coordinates: the second's is the
/// first's times a factor — given by `value` as the transmission says,
/// negated if `reverse` — plus `offset`, in the second's units, degrees for
/// an angle. Serializes flat: `{"type": "gear", "value": 2.0, "reverse":
/// true}`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CouplingKind {
    #[serde(rename = "type")]
    pub transmission: Transmission,
    #[serde(with = "as_f64")]
    pub value: Scalar,
    #[serde(default)]
    pub reverse: bool,
    #[serde(
        default,
        with = "as_f64::option",
        skip_serializing_if = "Option::is_none"
    )]
    pub offset: Option<Scalar>,
}

impl CouplingKind {
    /// A coupling of `transmission` by `value`, turning the way that
    /// transmission does unless `reverse`, at no offset.
    pub fn new(transmission: Transmission, value: Scalar, reverse: bool) -> Self {
        CouplingKind {
            transmission,
            value,
            reverse,
            offset: None,
        }
    }

    /// The second coordinate per radian or unit of length of the first,
    /// both in natural units — radians and lengths.
    pub fn factor(&self) -> GeopResult<Scalar> {
        let factor = match self.transmission {
            Transmission::Gear | Transmission::Belt => Scalar::ONE.div(self.value)?,
            Transmission::RackPinion | Transmission::Linear => self.value,
            Transmission::Screw => self.value.div(Scalar::TWO.mul(Scalar::PI))?,
        };
        Ok(if self.reverse { factor.neg() } else { factor })
    }

    /// Checks it can tie the motions of one joint and another — `same`
    /// whether they are the one joint — that the joints free: `frees`
    /// saying whether a joint does.
    pub fn validate(&self, same: bool, frees: [bool; 2]) -> GeopResult<()> {
        let label = self.transmission.label().to_lowercase();
        let [ma, mb] = self.transmission.motions();
        if same && ma == mb {
            return Err(GeopError::new(format!(
                "a {label} coupling ties a joint's {} to itself",
                ma.name()
            )));
        }
        for (k, motion) in [ma, mb].into_iter().enumerate() {
            if !frees[k] {
                return Err(GeopError::new(format!(
                    "a {label} coupling needs its {} joint to {}",
                    ["first", "second"][k],
                    motion.verb()
                )));
            }
        }
        let ratio = matches!(self.transmission, Transmission::Gear | Transmission::Belt);
        if ratio && self.value.could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "a {label} coupling's ratio must not be zero: {:?}",
                self.value
            )));
        }
        Ok(())
    }

    /// The residual between the coordinates `a` and `b` of its two joints —
    /// in their own units, degrees for an angle: the second less the first
    /// times the factor and the offset, in natural units, as a length — an
    /// angle's turned into one by the part's size `scale`.
    pub fn residual(&self, a: D, b: D, scale: D) -> GeopResult<D> {
        let [ma, mb] = self.transmission.motions();
        let mut off = mb
            .natural(b)?
            .sub(ma.natural(a)?.mul(D::cst(self.factor()?)));
        // Only one given: an exact zero taken away rounds a hair outward.
        if let Some(offset) = self.offset {
            off = off.sub(mb.natural(D::cst(offset))?);
        }
        Ok(match mb {
            Motion::Turn => off.mul(scale),
            Motion::Slide | Motion::SlideX | Motion::SlideY => off,
        })
    }
}
