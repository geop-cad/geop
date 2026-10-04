//! Joints and couplings: mates that say how one body may move against
//! another, rather than only what touches what.
//!
//! A [`Joint`] holds a [`Connector`] of one body on a connector of another
//! — an origin, an axis through it and a direction square to the axis that
//! turns are measured from — and lets the second body turn about the first
//! one's axis, slide along it, or both, as its [`JointKind`] says. How far
//! it is turned and slid are the joint's **coordinates**: numbers of the
//! system, solved with the poses, so that a joint can be set to a value,
//! held there, kept within limits, and coupled to another joint's.
//!
//! A [`Coupling`] ties a coordinate of one joint to one of another — or of
//! the same one — linearly: a gear pair, a rack and its pinion, a screw.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Field, Ring, Scalar, as_f64},
    vector::Vector3,
};
use serde::{Deserialize, Serialize};

use super::{Dual, V, cst};
use crate::{Placed, Residual, Value};

/// A joint's end on a body, in that body's frame: where its axis passes, the
/// axis, and the direction its turns are measured from — unit vectors, the
/// reference square to the axis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Connector<S: Scalar> {
    pub origin: Vector3<S>,
    pub axis: Vector3<S>,
    pub reference: Vector3<S>,
}

impl<S: Scalar> Connector<S> {
    /// The connector at `origin`, along `axis` (any non-zero length), its
    /// turns measured from `reference` made square to the axis — or, with
    /// none given, from whichever of the body's own axes `x`, `y`, `z` runs
    /// least along it, made square to it: a free choice, and the
    /// well-conditioned one. So two bodies drawn the same way round, mated
    /// on the same axis of each, are at a turn of zero.
    pub fn new(
        origin: Vector3<S>,
        axis: Vector3<S>,
        reference: Option<Vector3<S>>,
    ) -> GeopResult<Self> {
        let axis = axis
            .normalize()
            .map_err(|e| e.with_context(format!("the axis {axis:?} of a joint")))?;
        let reference = reference.unwrap_or_else(|| {
            let size = |k: usize| axis[k].abs().sharpen();
            let least = (1..3).fold(0, |best, k| {
                if size(k).definitely_less(size(best)) {
                    k
                } else {
                    best
                }
            });
            Vector3::axis(least)
        });
        let square = reference.sub(&axis.prod_scalar(axis.prod_dot(&reference)));
        let reference = square.normalize().map_err(|e| {
            e.with_context(format!(
                "the reference {reference:?} of a joint runs along its axis {axis:?}"
            ))
        })?;
        Ok(Connector {
            origin,
            axis,
            reference,
        })
    }
}

/// A connector attached to a body — by its index in
/// [`super::Assembly::bodies`] — or to the ground (`None`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointEnd<S: Scalar> {
    pub body: Option<usize>,
    pub connector: Connector<S>,
}

/// One of a joint's two motions: turning about the first connector's axis,
/// or sliding along it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Motion {
    Turn,
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
}

/// How a joint lets its second body move against its first: which of the
/// motions it frees, and within what limits — an angle's in degrees.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", bound = "S: Scalar")]
pub enum JointKind<S: Scalar> {
    /// Turns about the axis, nothing else: a hinge, a robot's joint.
    Revolute {
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        min: Option<S>,
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        max: Option<S>,
    },
    /// Slides along the axis, nothing else.
    Slider {
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        min: Option<S>,
        #[serde(
            default,
            with = "as_f64::option",
            skip_serializing_if = "Option::is_none"
        )]
        max: Option<S>,
    },
    /// Turns about the axis and slides along it: a shaft in a bearing.
    Cylindrical,
    /// Neither: the connectors coincide, and the two bodies move as one.
    Fastened,
}

impl<S: Scalar> JointKind<S> {
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
        Motion::ALL
            .into_iter()
            .filter(|&m| self.moves(m))
            .collect()
    }

    /// The limits of `motion`'s coordinate, `[min, max]`, either missing.
    pub fn limits(&self, motion: Motion) -> [Option<S>; 2] {
        match (*self, motion) {
            (JointKind::Revolute { min, max }, Motion::Turn)
            | (JointKind::Slider { min, max }, Motion::Slide) => [min, max],
            _ => [None, None],
        }
    }

    /// The same kind in the scalar type `T` (see [`Scalar::cast`]).
    pub fn cast<T: Scalar>(&self) -> JointKind<T> {
        let cast = |v: Option<S>| v.map(|v| v.cast());
        match *self {
            JointKind::Revolute { min, max } => JointKind::Revolute {
                min: cast(min),
                max: cast(max),
            },
            JointKind::Slider { min, max } => JointKind::Slider {
                min: cast(min),
                max: cast(max),
            },
            JointKind::Cylindrical => JointKind::Cylindrical,
            JointKind::Fastened => JointKind::Fastened,
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
}

/// A coordinate of a joint: its value — an angle in degrees, a distance —
/// and whether a solve must keep it there.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coordinate<S: Scalar> {
    pub value: S,
    pub held: bool,
}

impl<S: Scalar> Coordinate<S> {
    pub fn free(value: S) -> Self {
        Coordinate { value, held: false }
    }
}

/// A joint: body `b`'s connector held on body `a`'s, `b` turned about `a`'s
/// axis by the angle and moved along it by the distance. The coordinates of
/// the motions its kind frees are solved, unless held; the others stay at
/// their values — zero, usually: a revolute joint's distance is how far
/// along the axis it sits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Joint<S: Scalar> {
    pub kind: JointKind<S>,
    pub a: JointEnd<S>,
    pub b: JointEnd<S>,
    pub angle: Coordinate<S>,
    pub distance: Coordinate<S>,
}

impl<S: Scalar> Joint<S> {
    pub fn coordinate(&self, motion: Motion) -> &Coordinate<S> {
        match motion {
            Motion::Turn => &self.angle,
            Motion::Slide => &self.distance,
        }
    }

    pub fn coordinate_mut(&mut self, motion: Motion) -> &mut Coordinate<S> {
        match motion {
            Motion::Turn => &mut self.angle,
            Motion::Slide => &mut self.distance,
        }
    }

    /// Where its coordinates are, measured from where its ends are — each
    /// at the pose `pose` gives its body: the angle `b`'s reference is
    /// turned from `a`'s about `a`'s axis, in degrees in `(-180, 180]`, and
    /// how far `b`'s origin is along it. Only a seed for a solve, which
    /// finds the exact values: read off midpoints.
    pub fn measure(&self, pose: impl Fn(Option<usize>) -> Pose<S>) -> [f64; 2] {
        let world = |end: &JointEnd<S>| {
            let motion = pose(end.body).motion();
            let c = &end.connector;
            let f = |v: Vector3<S>| v.to_array().map(|x| x.to_f64());
            (
                f(motion.apply(&c.origin)),
                f(motion.rotate(&c.axis)),
                f(motion.rotate(&c.reference)),
            )
        };
        let ((oa, za, ra), (ob, _, rb)) = (world(&self.a), world(&self.b));
        let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|k| a[k] * b[k]).sum::<f64>();
        let ya = [
            za[1] * ra[2] - za[2] * ra[1],
            za[2] * ra[0] - za[0] * ra[2],
            za[0] * ra[1] - za[1] * ra[0],
        ];
        let angle = dot(rb, ya).atan2(dot(rb, ra)).to_degrees();
        let distance = dot([0, 1, 2].map(|k| ob[k] - oa[k]), za);
        [angle, distance]
    }

    /// Checks that it joins two different bodies.
    pub fn validate(&self) -> GeopResult<()> {
        if self.a.body == self.b.body {
            return Err(GeopError::new(match self.a.body {
                Some(body) => format!(
                    "both ends of a {} joint are on body {body}: it holds nothing together",
                    self.kind.label().to_lowercase()
                ),
                None => format!(
                    "both ends of a {} joint are on the ground: it holds nothing together",
                    self.kind.label().to_lowercase()
                ),
            }));
        }
        for motion in self.kind.motions() {
            if let [Some(min), Some(max)] = self.kind.limits(motion)
                && min.definitely_greater(max)
            {
                return Err(GeopError::new(format!(
                    "the {} limits of a {} joint are the wrong way round: {min:?} > {max:?}",
                    motion.name(),
                    self.kind.label().to_lowercase()
                )));
            }
        }
        Ok(())
    }
}

/// How a coupling ties its two joints' coordinates: the second's is the
/// first's times a factor — negated if `reverse`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", bound = "S: Scalar")]
pub enum CouplingKind<S: Scalar> {
    /// Two turning joints: the first turns `ratio` times for every turn of
    /// the second — a pinion of 10 teeth driving a wheel of 20 is a ratio
    /// of 2. A pair of external gears turns opposite ways: `reverse`.
    Gear {
        #[serde(with = "as_f64")]
        ratio: S,
        #[serde(default)]
        reverse: bool,
    },
    /// A turning joint, the pinion, and a sliding one, the rack: the rack
    /// moves the length of arc the pinion's pitch circle, of `radius`,
    /// rolls off.
    RackPinion {
        #[serde(with = "as_f64")]
        radius: S,
        #[serde(default)]
        reverse: bool,
    },
    /// A turning joint and a sliding one — the turn and the slide of one
    /// cylindrical joint, usually: every turn moves it by `lead`.
    Screw {
        #[serde(with = "as_f64")]
        lead: S,
        #[serde(default)]
        reverse: bool,
    },
}

impl<S: Scalar> CouplingKind<S> {
    /// The motions of the two joints it ties.
    pub fn motions(&self) -> [Motion; 2] {
        match self {
            CouplingKind::Gear { .. } => [Motion::Turn, Motion::Turn],
            CouplingKind::RackPinion { .. } | CouplingKind::Screw { .. } => {
                [Motion::Turn, Motion::Slide]
            }
        }
    }

    /// The second coordinate per radian or unit of length of the first,
    /// both in natural units — radians and lengths.
    fn factor(&self) -> GeopResult<S> {
        let (factor, reverse) = match *self {
            CouplingKind::Gear { ratio, reverse } => (S::ONE.div(ratio)?, reverse),
            CouplingKind::RackPinion { radius, reverse } => (radius, reverse),
            CouplingKind::Screw { lead, reverse } => (lead.div(S::TWO.mul(S::PI))?, reverse),
        };
        Ok(if reverse { factor.neg() } else { factor })
    }

    /// The same kind in the scalar type `T` (see [`Scalar::cast`]).
    pub fn cast<T: Scalar>(&self) -> CouplingKind<T> {
        match *self {
            CouplingKind::Gear { ratio, reverse } => CouplingKind::Gear {
                ratio: ratio.cast(),
                reverse,
            },
            CouplingKind::RackPinion { radius, reverse } => CouplingKind::RackPinion {
                radius: radius.cast(),
                reverse,
            },
            CouplingKind::Screw { lead, reverse } => CouplingKind::Screw {
                lead: lead.cast(),
                reverse,
            },
        }
    }

    pub fn label(&self) -> &'static str {
        match self {
            CouplingKind::Gear { .. } => "Gear",
            CouplingKind::RackPinion { .. } => "Rack and pinion",
            CouplingKind::Screw { .. } => "Screw",
        }
    }
}

/// A coupling of the joints `a` and `b` — by index in
/// [`super::Assembly::joints`], possibly the same joint twice.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Coupling<S: Scalar> {
    pub kind: CouplingKind<S>,
    pub a: usize,
    pub b: usize,
}

impl<S: Scalar> Coupling<S> {
    /// Checks that its joints are there and free the motions it ties, and
    /// that it does not tie a coordinate to itself.
    pub fn validate(&self, joints: &[Joint<S>]) -> GeopResult<()> {
        let [ma, mb] = self.kind.motions();
        if self.a == self.b && ma == mb {
            return Err(GeopError::new(format!(
                "a {} coupling ties joint {}'s {} to itself",
                self.kind.label().to_lowercase(),
                self.a,
                ma.name()
            )));
        }
        for (joint, motion) in [(self.a, ma), (self.b, mb)] {
            let Some(j) = joints.get(joint) else {
                return Err(GeopError::new(format!(
                    "a coupling refers to joint {joint}, but there are {} joints",
                    joints.len()
                )));
            };
            if !j.kind.moves(motion) {
                return Err(GeopError::new(format!(
                    "a {} coupling needs joint {joint} to {}, but a {} joint does not",
                    self.kind.label().to_lowercase(),
                    match motion {
                        Motion::Turn => "turn",
                        Motion::Slide => "slide",
                    },
                    j.kind.label().to_lowercase()
                )));
            }
        }
        if let CouplingKind::Gear { ratio, .. } = self.kind
            && ratio.could_be_equal(S::ZERO)
        {
            return Err(GeopError::new(format!(
                "a gear coupling's ratio must not be zero: {ratio:?}"
            )));
        }
        Ok(())
    }
}

/// A coordinate's value as a variable of the system: a length — an angle
/// as the arc it turns a point the system's size from the axis — so that a
/// step turns as far as it moves, as the turns of a pose do.
pub(crate) fn to_variable<S: Scalar>(motion: Motion, value: S, scale: S) -> GeopResult<S> {
    Ok(match motion {
        Motion::Turn => value.mul(S::PI.div(S::from_i64(180))?).mul(scale),
        Motion::Slide => value,
    })
}

/// The coordinate a variable is (see [`to_variable`]).
pub(crate) fn from_variable<S: Scalar>(motion: Motion, variable: S, scale: S) -> GeopResult<S> {
    Ok(match motion {
        Motion::Turn => variable.div(scale)?.mul(S::from_i64(180).div(S::PI)?),
        Motion::Slide => variable,
    })
}

/// A coordinate during a solve: a parameter of the system, by index, or a
/// value that stays.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Held<S: Scalar> {
    Param(usize),
    Value(S),
}

/// A joint as a residual of its bodies' poses and its coordinates.
pub(crate) struct JointResidual<S: Scalar> {
    joint: Joint<S>,
    /// Its parameters: its bodies' poses, then its coordinates that are
    /// parameters.
    params: Vec<usize>,
    /// Per motion, where its coordinate comes from: in natural units for a
    /// value — radians, lengths.
    coordinates: [Held<S>; 2],
    scale: S,
}

/// The value at `param` among `values`, the values of `params`.
fn value_of<'v, T: Scalar>(
    params: &[usize],
    values: &'v [Value<T>],
    param: usize,
) -> &'v Value<T> {
    let slot = params
        .iter()
        .position(|&p| p == param)
        .expect("its own parameters");
    &values[slot]
}

impl<S: Scalar> JointResidual<S> {
    /// The residual of `joint`, its coordinates' parameters as `coordinates`
    /// gives them — per motion, the index of the parameter, if it is one.
    pub fn new(joint: Joint<S>, coordinates: [Option<usize>; 2], scale: S) -> GeopResult<Self> {
        let mut params: Vec<usize> = [joint.a.body, joint.b.body]
            .into_iter()
            .flatten()
            .collect();
        params.extend(coordinates.iter().flatten());
        let mut held = [Held::Value(S::ZERO); 2];
        for (k, motion) in Motion::ALL.into_iter().enumerate() {
            held[k] = match coordinates[k] {
                Some(p) => Held::Param(p),
                None => Held::Value(to_variable(
                    motion,
                    joint.coordinate(motion).value,
                    S::ONE,
                )?),
            };
        }
        Ok(JointResidual {
            joint,
            params,
            coordinates: held,
            scale,
        })
    }

    fn end(
        &self,
        values: &[Value<Dual<S>>],
        end: &JointEnd<S>,
    ) -> GeopResult<(V<S>, V<S>, V<S>)> {
        let c = &end.connector;
        Ok(match end.body {
            Some(body) => {
                let placed: &Placed<Dual<S>> = value_of(&self.params, values, body).pose()?;
                (
                    placed.point(&cst(&c.origin)),
                    placed.direction(&cst(&c.axis)),
                    placed.direction(&cst(&c.reference)),
                )
            }
            None => (cst(&c.origin), cst(&c.axis), cst(&c.reference)),
        })
    }

    /// Its coordinate of `motion` at `values`, in natural units.
    fn coordinate(&self, values: &[Value<Dual<S>>], k: usize) -> GeopResult<Dual<S>> {
        Ok(match self.coordinates[k] {
            Held::Param(p) => {
                let v = value_of(&self.params, values, p).scalar()?;
                match Motion::ALL[k] {
                    Motion::Turn => v.div(Dual::cst(self.scale))?,
                    Motion::Slide => v,
                }
            }
            Held::Value(v) => Dual::cst(v),
        })
    }
}

impl<S: Scalar> Residual<S, { super::MATE_VARS }> for JointResidual<S> {
    fn params(&self) -> &[usize] {
        &self.params
    }

    /// `b`'s origin at `a`'s moved along `a`'s axis by the distance, the
    /// axes parallel — facing either way, as a pin in a hole may — and
    /// `b`'s reference `a`'s turned about `a`'s axis by the angle. Fails —
    /// infeasible — where `b` is turned half a turn from that.
    fn eval(&self, values: &[Value<Dual<S>>], out: &mut Vec<Dual<S>>) -> GeopResult<()> {
        let (oa, za, ra) = self.end(values, &self.joint.a)?;
        let (ob, zb, rb) = self.end(values, &self.joint.b)?;
        let angle = self.coordinate(values, 0)?;
        let distance = self.coordinate(values, 1)?;
        let l = Dual::cst(self.scale);
        out.extend(ob.sub(&oa.add(&za.prod_scalar(distance))).to_array());
        out.extend(zb.prod_cross(&za).prod_scalar(l).to_array());
        // How far `b`'s reference is turned past the angle, `φ - θ`, as
        // `2 tan((φ - θ) / 2)` from its sine and cosine: zero only at
        // `φ = θ` — a sine alone is zero half a turn away too — and one
        // row, of slope one there. Measured as `rb - u`, `b`'s reference
        // less `a`'s turned by the angle, the part of it along `u` would
        // vanish to second order only: a row that, at a solution the
        // minimizer reached to its tolerance, has a slope as small as that
        // tolerance and still counts as a constraint, pinning the joint.
        let ya = za.prod_cross(&ra);
        let (cos, sin) = (angle.cos(), angle.sin());
        let u = ra.prod_scalar(cos).add(&ya.prod_scalar(sin));
        let t = ya.prod_scalar(cos).sub(&ra.prod_scalar(sin));
        let past = rb.prod_dot(&t).div(Dual::ONE.add(rb.prod_dot(&u)))?;
        out.push(past.mul(Dual::TWO).mul(l));
        Ok(())
    }
}

/// A coupling as a residual of its two coordinates.
pub(crate) struct CouplingResidual<S: Scalar> {
    /// The coordinates, as they enter: variables or values, in the units
    /// of the system's variables (see [`to_variable`]).
    coordinates: [Held<S>; 2],
    params: Vec<usize>,
    /// Per unit of the first variable, the second, in the system's units.
    factor: S,
}

impl<S: Scalar> CouplingResidual<S> {
    /// The residual of `coupling` between `joints`, each coordinate a
    /// parameter where `params` gives one — per joint and motion.
    pub fn new(
        coupling: &Coupling<S>,
        joints: &[Joint<S>],
        params: &dyn Fn(usize, Motion) -> Option<usize>,
        scale: S,
    ) -> GeopResult<Self> {
        let [ma, mb] = coupling.kind.motions();
        let held = |joint: usize, motion: Motion| -> GeopResult<Held<S>> {
            Ok(match params(joint, motion) {
                Some(p) => Held::Param(p),
                None => Held::Value(to_variable(
                    motion,
                    joints[joint].coordinate(motion).value,
                    scale,
                )?),
            })
        };
        let coordinates = [held(coupling.a, ma)?, held(coupling.b, mb)?];
        // Natural units to the system's: an angle's variable is the angle
        // times the size.
        let unit = |m: Motion| match m {
            Motion::Turn => scale,
            Motion::Slide => S::ONE,
        };
        let factor = coupling.kind.factor()?.mul(unit(mb)).div(unit(ma))?;
        let mut ps: Vec<usize> = coordinates
            .iter()
            .filter_map(|c| match c {
                Held::Param(p) => Some(*p),
                Held::Value(_) => None,
            })
            .collect();
        ps.dedup();
        Ok(CouplingResidual {
            coordinates,
            params: ps,
            factor,
        })
    }
}

impl<S: Scalar> Residual<S, { super::MATE_VARS }> for CouplingResidual<S> {
    fn params(&self) -> &[usize] {
        &self.params
    }

    /// The second coordinate less the first times the factor: a length.
    fn eval(&self, values: &[Value<Dual<S>>], out: &mut Vec<Dual<S>>) -> GeopResult<()> {
        let at = |c: &Held<S>| -> GeopResult<Dual<S>> {
            Ok(match *c {
                Held::Param(p) => value_of(&self.params, values, p).scalar()?,
                Held::Value(v) => Dual::cst(v),
            })
        };
        let [a, b] = &self.coordinates;
        out.push(at(b)?.sub(at(a)?.mul(Dual::cst(self.factor))));
        Ok(())
    }
}
