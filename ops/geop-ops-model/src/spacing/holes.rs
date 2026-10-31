//! Where a part's holes are and what fits them: each hole a step drilled
//! recorded as a [`HoleSite`] — its axis, the screw size it is for, how a
//! head seats in it and how deep it runs — in the part's target
//! [`hole_sites`], which every step that drills holes revises, and every
//! step that repeats features copies with them. What reads them: a pattern
//! laid out at holes ([`super::Frames::Holes`]), and fasteners placed in
//! them, sized by them.

use std::collections::BTreeMap;

use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::{CoordinateSystem, Motion, Pose, Quaternion},
    scalars::Scalar,
    with_context,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    reader::Reader,
    target::{Target, TargetReference},
};
use serde::{Deserialize, Serialize};

use crate::{FeatureRef, frame_along, operation_of, parameters::located};

/// How a hole takes a screw.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HoleFit {
    /// The screw passes through: a clearance hole.
    #[default]
    Clearance,
    /// The screw passes through, its head sunk in a counterbore.
    Counterbore,
    /// The screw passes through, its head sunk flush in a countersink.
    Countersink,
    /// The screw is screwed in: a tapped hole.
    Tapped,
}

/// One hole, as the step that drilled it recorded it — kept as geometry,
/// so that it says the same whatever later steps do to its faces.
#[derive(Clone, Debug, PartialEq)]
pub struct HoleSite {
    /// Where it enters its face, and the way it runs into the material
    /// from there (a unit vector).
    pub axis: Axis,
    /// The ISO metric size of the screw it is for: `M6`. None for a hole
    /// sized by hand, or tapped for a pipe thread.
    pub size: Option<String>,
    pub fit: HoleFit,
    /// How far below its face a screw's head seats: a counterbore's depth;
    /// none for any other.
    pub seat: f64,
    /// How far it runs from its face: a blind hole's depth, or, through
    /// the material, as far as its wall goes.
    pub depth: f64,
    /// It comes out of the material at its far end.
    pub through: bool,
}

impl HoleSite {
    /// The frame a screw's head seats on: on the axis, as far below the
    /// face as a head seats, its `w` pointing out of the hole — the way a
    /// standard screw's datum `seat` faces, its shank running down `-w`
    /// into the hole.
    pub fn seat_frame(&self) -> GeopResult<CoordinateSystem> {
        let at = self.along(self.seat);
        frame_along(at, &self.axis.direction.neg())
    }

    /// The frame where it comes out of the material, its `w` pointing out
    /// of the hole there — where a nut on a screw through it seats. None for
    /// a blind hole.
    pub fn exit_frame(&self) -> GeopResult<Option<CoordinateSystem>> {
        if !self.through {
            return Ok(None);
        }
        Ok(Some(frame_along(
            self.along(self.depth),
            &self.axis.direction,
        )?))
    }

    /// How long the hole is below where a head seats: the grip of a screw
    /// through it, or how far a screw can go into it.
    pub fn grip(&self) -> f64 {
        self.depth - self.seat
    }

    /// The point `distance` along its axis from where it enters its face.
    fn along(&self, distance: f64) -> geop_core_math::vector::Vector3 {
        (self.axis.point).add(&self.axis.direction.prod_scalar(Scalar::from_f64(distance)))
    }

    /// The same hole, moved by `motion`.
    pub fn moved(&self, motion: &Motion) -> Self {
        Self {
            axis: Axis {
                point: motion.apply(&self.axis.point),
                direction: motion.rotate(&self.axis.direction),
            },
            ..self.clone()
        }
    }
}

/// The pose of the frame `frame`: what takes the world's origin and axes
/// to it.
pub fn pose_of(frame: &CoordinateSystem) -> GeopResult<Pose> {
    let rotation = Quaternion::from_rotation_columns([*frame.u(), *frame.v(), *frame.w()])?;
    Pose::new(*frame.origin(), rotation)
}

/// A part's holes, by name: the value of its target [`hole_sites`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct HoleSites(BTreeMap<String, HoleSite>);

/// The part's holes.
pub fn hole_sites() -> TargetReference<HoleSites> {
    TargetReference::new("holes")
}

/// What a part reads as its holes before a step drilled one: none.
static NO_HOLES: HoleSites = HoleSites(BTreeMap::new());

impl HoleSites {
    /// Records `site` under `name`. Fails, leaving the holes as they are,
    /// if there is a hole of that name already.
    pub fn add(&mut self, name: impl Into<String>, site: HoleSite) -> GeopResult<()> {
        let name = name.into();
        if self.0.contains_key(&name) {
            return Err(GeopError::new(format!(
                "the part has a hole named {name:?} already"
            )));
        }
        self.0.insert(name, site);
        Ok(())
    }

    /// Every hole, by name.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &HoleSite)> {
        self.0.iter().map(|(name, site)| (name.as_str(), site))
    }

    /// The holes the step `step` drilled — those named after it — by name,
    /// in order.
    pub fn of_step(&self, step: &str) -> Vec<(&str, &HoleSite)> {
        (self.iter())
            .filter(|(name, _)| operation_of(name) == Some(step))
            .collect()
    }
}

/// Every step that drills holes, or repeats them, adds to them.
impl Target for HoleSites {
    const CHAIN: bool = true;
}

/// Read whole.
impl Kind for HoleSites {
    const NAME: &'static str = "holes";
}

impl HasElements<HoleSites> for HoleSites {
    type Element<'a> = &'a HoleSites;
    const SINGLETON: Option<&'static str> = Some("holes");

    fn element(&self, name: &str) -> GeopResult<&HoleSites> {
        whole(self, name)
    }

    fn undefined() -> Option<&'static Self> {
        Some(&NO_HOLES)
    }
}

/// The part's holes as `reader` reads them: none, before the first step
/// that drills one.
pub fn read_hole_sites<'a>(reader: &Reader<'a>) -> GeopResult<&'a HoleSites> {
    reader.retrieve_element(&ElementReference::whole(hole_sites()))
}

/// The holes of the features `features` — of the part `reader` reads, or
/// of parts placed in it — each by its name, behind its instance's path,
/// and where the part read has it. A feature that drilled no hole is
/// refused by name.
pub fn holes_of(reader: &Reader, features: &[FeatureRef]) -> GeopResult<Vec<(String, HoleSite)>> {
    let mut found = Vec::new();
    for feature in features {
        let ctx = with_context!("the holes of {feature:?}");
        let instance = feature.instance();
        let all = ElementReference::<HoleSites, HoleSites>::whole(hole_sites());
        let all = match instance.is_empty() {
            true => all,
            false => all.behind(instance),
        };
        let sites = reader.retrieve_placed(&all).with_context(ctx)?;
        let motion = located(reader, instance).with_context(ctx)?.motion();
        let of = sites.element.of_step(feature.name());
        if of.is_empty() {
            return Err(GeopError::new(format!(
                "the feature {:?} drilled no holes: pick the wall of a hole a hole step drilled, or a pattern of one",
                feature.name()
            )))
            .with_context(ctx);
        }
        for (name, site) in of {
            let name = match instance.is_empty() {
                true => name.to_string(),
                false => geop_core_program::placed::Located::in_instance(instance, name).key(),
            };
            found.push((name, site.moved(&motion)));
        }
    }
    Ok(found)
}
