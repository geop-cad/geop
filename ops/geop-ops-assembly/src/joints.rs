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
    scalars::{Field, Ring, Scalar, as_f64},
    vector::Vector3,
};
use geop_ops::{Design, operation::Aspects};
use serde::{Deserialize, Serialize};

use crate::residuals::{D, Placing, parallel};

/// A joint's end on a body, in that body's frame — or, placed, where its
/// body puts it: where its axis passes, the axis, and the direction its
/// turns are measured from — unit vectors, the reference square to the
/// axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Connector<S: Scalar> {
    pub origin: Vector3<S>,
    pub axis: Vector3<S>,
    pub reference: Vector3<S>,
}

impl Connector<Design> {
    /// The connector at `origin`, along `axis` (any non-zero length), its
    /// turns measured from `reference` made square to the axis — or, with
    /// none given, from whichever of the body's own axes `x`, `y`, `z` runs
    /// least along it, made square to it: a free choice, and the
    /// well-conditioned one. So two bodies drawn the same way round, mated
    /// on the same axis of each, are at a turn of zero.
    pub fn new(
        origin: Vector3<Design>,
        axis: Vector3<Design>,
        reference: Option<Vector3<Design>>,
    ) -> GeopResult<Self> {
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
    /// none: it is used by the frame it gives (see [`Aspects::frame_on`]).
    pub fn of(aspects: &Aspects<Design>) -> Option<GeopResult<Self>> {
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

    /// Where `pose` puts it — or, with none, where it is: of a part that
    /// never moves.
    pub fn placed(&self, pose: Option<&Pose<D>>) -> Connector<D> {
        let placing = Placing::new(pose);
        let turn = |d: &Vector3<Design>| placing.turn(&d.map(D::cst));
        Connector {
            origin: placing.point(&self.origin),
            axis: turn(&self.axis),
            reference: turn(&self.reference),
        }
    }
}

/// One of a joint's two motions: turning about the first connector's axis,
/// or sliding along it. Serializes as what its coordinate is called (see
/// [`Motion::name`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
pub enum Motion {
    #[serde(rename = "angle")]
    Turn,
    #[serde(rename = "distance")]
    Slide,
}

impl Motion {
    pub const ALL: [Motion; 2] = [Motion::Turn, Motion::Slide];

    /// What its coordinate is called: `angle` (in degrees), `distance`.
    pub fn name(self) -> &'static str {
        match self {
            Motion::Turn => "angle",
            Motion::Slide => "distance",
        }
    }

    /// What its coordinate measures, as a user sets it.
    pub fn measure(self) -> geop_ops::solving::Measure {
        match self {
            Motion::Turn => geop_ops::solving::Measure::Angle,
            Motion::Slide => geop_ops::solving::Measure::Distance,
        }
    }

    /// What a joint that frees it does: `turn`, `slide`.
    pub fn verb(self) -> &'static str {
        match self {
            Motion::Turn => "turn",
            Motion::Slide => "slide",
        }
    }

    /// A coordinate of it in its natural units — radians for an angle,
    /// given in degrees; a length for a distance.
    pub fn natural(self, value: D) -> GeopResult<D> {
        match self {
            Motion::Turn => value.degrees_to_radians(),
            Motion::Slide => Ok(value),
        }
    }
}

/// How a joint lets its second body move against its first: which of the
/// motions it frees, and within what limits — an angle's in degrees.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum JointKind {
    /// Turns about the axis, nothing else: a hinge, a robot's joint.
    Revolute {
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        min: Option<Design>,
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        max: Option<Design>,
    },
    /// Slides along the axis, nothing else.
    Slider {
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        min: Option<Design>,
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        max: Option<Design>,
    },
    /// Turns about the axis and slides along it: a shaft in a bearing.
    Cylindrical,
    /// Neither: the connectors coincide, and the two bodies move as one.
    Fastened,
}

impl JointKind {
    /// Every kind, unlimited.
    pub const ALL: [JointKind; 4] = [
        JointKind::Revolute {
            min: None,
            max: None,
        },
        JointKind::Slider {
            min: None,
            max: None,
        },
        JointKind::Cylindrical,
        JointKind::Fastened,
    ];

    /// Whether it frees `motion`.
    pub fn moves(&self, motion: Motion) -> bool {
        matches!(
            (self, motion),
            (JointKind::Revolute { .. }, Motion::Turn)
                | (JointKind::Slider { .. }, Motion::Slide)
                | (JointKind::Cylindrical, _)
        )
    }

    /// The motions it frees.
    pub fn motions(&self) -> Vec<Motion> {
        Motion::ALL.into_iter().filter(|&m| self.moves(m)).collect()
    }

    /// The limits of `motion`'s coordinate, `[min, max]`, either missing.
    pub fn limits(&self, motion: Motion) -> [Option<Design>; 2] {
        match (*self, motion) {
            (JointKind::Revolute { min, max }, Motion::Turn)
            | (JointKind::Slider { min, max }, Motion::Slide) => [min, max],
            _ => [None, None],
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            JointKind::Revolute { .. } => "Revolute",
            JointKind::Slider { .. } => "Slider",
            JointKind::Cylindrical => "Cylindrical",
            JointKind::Fastened => "Fastened",
        }
    }

    /// Checks its limits are the right way round.
    pub fn validate(&self) -> GeopResult<()> {
        for motion in self.motions() {
            if let [Some(min), Some(max)] = self.limits(motion)
                && min.definitely_greater(max)
            {
                return Err(GeopError::new(format!(
                    "the {} limits of a {} joint are the wrong way round: {min:?} > {max:?}",
                    motion.name(),
                    self.label().to_lowercase()
                )));
            }
        }
        Ok(())
    }
}

/// The residuals of a joint between `a` — the first part's connector — and
/// `b`, at the coordinates `angle` (degrees) and `distance`, where they are:
/// each a length, a direction's scaled by the part's size `scale`.
///
/// `b`'s origin at `a`'s moved along `a`'s axis by the distance, the axes
/// parallel — facing either way, as a pin in a hole may — and `b`'s
/// reference `a`'s turned about `a`'s axis by the angle. Fails —
/// infeasible — where `b` is turned half a turn from that.
pub fn joint_residuals(
    a: &Connector<D>,
    b: &Connector<D>,
    angle: D,
    distance: D,
    scale: D,
) -> GeopResult<Vec<D>> {
    let (za, ra) = (&a.axis, &a.reference);
    let l = scale;
    let mut out: Vec<D> = b
        .origin
        .sub(&a.origin.add(&za.prod_scalar(distance)))
        .to_array()
        .to_vec();
    // The axes parallel: `b`'s axis has no part along `a`'s reference or
    // the direction square to both.
    let ya = za.prod_cross(ra);
    out.extend(parallel(&[*ra, ya], &b.axis, l));
    // How far `b`'s reference is turned past the angle, `φ - θ`, as
    // `2 tan((φ - θ) / 2)` from its sine and cosine: zero only at `φ = θ` —
    // a sine alone is zero half a turn away too — and one row, of slope one
    // there. Measured as `rb - u`, `b`'s reference less `a`'s turned by the
    // angle, the part of it along `u` would vanish to second order only: a
    // row that, at a solution the minimizer reached to its tolerance, has a
    // slope as small as that tolerance and still counts as a constraint,
    // pinning the joint.
    let angle = Motion::Turn.natural(angle)?;
    let (cos, sin) = (angle.cos(), angle.sin());
    let u = ra.prod_scalar(cos).add(&ya.prod_scalar(sin));
    let t = ya.prod_scalar(cos).sub(&ra.prod_scalar(sin));
    let past = b
        .reference
        .prod_dot(&t)
        .div(D::ONE.add(b.reference.prod_dot(&u)))?;
    out.push(past.mul(D::TWO).mul(l));
    Ok(out)
}

/// Where the coordinates of a joint between `a` and `b` are, measured from
/// where its ends are: the angle `b`'s reference is turned from `a`'s about
/// `a`'s axis, in degrees in `(-180, 180]`, and how far `b`'s origin is
/// along it. Only a seed for a solve, which finds the exact values: read
/// off midpoints.
pub fn measure(a: &Connector<D>, b: &Connector<D>) -> [f64; 2] {
    let f = |v: &Vector3<D>| v.to_array().map(|x| x.to_f64());
    let (oa, za, ra, ob, rb) = (
        f(&a.origin),
        f(&a.axis),
        f(&a.reference),
        f(&b.origin),
        f(&b.reference),
    );
    let ya = cross(za, ra);
    let angle = dot(rb, ya).atan2(dot(rb, ra)).to_degrees();
    let distance = dot(sub(ob, oa), za);
    [angle, distance]
}

/// What a coupling transmits between its two joints.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Transmission {
    /// Two turning joints: the first turns `value` times — the ratio — for
    /// every turn of the second. A pinion of 10 teeth driving a wheel of 20
    /// is a ratio of 2; a pair of external gears turns opposite ways.
    Gear,
    /// A turning joint, the pinion, and a sliding one, the rack: the rack
    /// moves the length of arc the pinion's pitch circle, of radius
    /// `value`, rolls off.
    RackPinion,
    /// A turning joint and a sliding one — the turn and the slide of one
    /// cylindrical joint, usually: every turn moves it by `value`, the
    /// lead.
    Screw,
}

impl Transmission {
    /// The motions of the two joints it ties.
    pub fn motions(self) -> [Motion; 2] {
        match self {
            Transmission::Gear => [Motion::Turn, Motion::Turn],
            Transmission::RackPinion | Transmission::Screw => [Motion::Turn, Motion::Slide],
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Transmission::Gear => "Gear",
            Transmission::RackPinion => "Rack and pinion",
            Transmission::Screw => "Screw",
        }
    }
}

/// How a coupling ties its two joints' coordinates: the second's is the
/// first's times a factor — given by `value` as the transmission says,
/// negated if `reverse`. Serializes flat: `{"type": "gear", "value": 2.0,
/// "reverse": true}`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CouplingKind {
    #[serde(rename = "type")]
    pub transmission: Transmission,
    #[serde(with = "as_f64")]
    pub value: Design,
    #[serde(default)]
    pub reverse: bool,
}

impl CouplingKind {
    /// The second coordinate per radian or unit of length of the first,
    /// both in natural units — radians and lengths.
    pub fn factor(&self) -> GeopResult<Design> {
        let factor = match self.transmission {
            Transmission::Gear => Design::ONE.div(self.value)?,
            Transmission::RackPinion => self.value,
            Transmission::Screw => self.value.div(Design::TWO.mul(Design::PI))?,
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
        if self.transmission == Transmission::Gear && self.value.could_be_equal(Design::ZERO) {
            return Err(GeopError::new(format!(
                "a gear coupling's ratio must not be zero: {:?}",
                self.value
            )));
        }
        Ok(())
    }

    /// The residual between the coordinates `a` and `b` of its two joints —
    /// in their own units, degrees for an angle: the second less the first
    /// times the factor, in natural units, as a length — an angle's turned
    /// into one by the part's size `scale`.
    pub fn residual(&self, a: D, b: D, scale: D) -> GeopResult<D> {
        let [ma, mb] = self.transmission.motions();
        let off = mb
            .natural(b)?
            .sub(ma.natural(a)?.mul(D::cst(self.factor()?)));
        Ok(match mb {
            Motion::Turn => off.mul(scale),
            Motion::Slide => off,
        })
    }
}
