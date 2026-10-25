//! Cosmetic threads: threads recorded on a round face rather than modelled
//! — what a drawing, a bill of materials or a fastener check reads, and
//! what the viewer draws as a helix on the face. Held as the part's target
//! [`threads`], and drawn as the [`annotations`] of the step that recorded
//! them ([`draw_threads`]).
//!
//! They are the framework's, not the hole's that taps one: a step that
//! repeats features — a pattern, a mirror — copies the threads they
//! carry and draws its copies, knowing no step that records one.

use std::collections::BTreeMap;

use crate::{frame_along, operation_of};
use geop_core_editor::view::{Annotations, ViewAnnotation, annotations};
use geop_core_geometry::{
    nurb_curve::{Handedness, NurbCurve3D},
    shape::Axis,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    part::StepContext,
    reader::Reader,
    target::{Target, TargetReference},
};

/// A thread recorded on a cylindrical face, by its specification and where
/// it lies — kept as geometry, so that it says the same whatever later
/// steps do to the face it was put on.
#[derive(Clone, Debug, PartialEq)]
pub struct CosmeticThread {
    /// How it is called: `M6x1`.
    pub designation: String,
    /// The face it was put on, by its name then.
    pub face: String,
    /// Where it starts, on the axis of its face, and the way it runs from
    /// there (a unit vector).
    pub axis: Axis,
    /// The radius of the face it lies on: where it is drawn.
    pub radius: Scalar,
    /// The thread's nominal (major) diameter, its minor diameter and its
    /// pitch, as its standard gives them.
    pub major_diameter: f64,
    pub minor_diameter: f64,
    pub pitch: f64,
    /// How far along the axis it runs from where it starts.
    pub length: f64,
    /// In a hole (a nut's thread) rather than on a shaft (a bolt's).
    pub internal: bool,
    pub handedness: Handedness,
}

impl CosmeticThread {
    /// The helix it is drawn as: on its face, from where it starts, one
    /// turn per pitch, as far as it runs.
    pub fn helix(&self) -> GeopResult<NurbCurve3D> {
        let frame: CoordinateSystem = frame_along(self.axis.point, &self.axis.direction)?;
        NurbCurve3D::helix(
            &frame,
            self.radius,
            Scalar::from_f64(self.pitch),
            self.length / self.pitch,
            self.handedness,
        )
    }

    /// The helix as a polyline, `SEGMENTS_PER_SPAN` segments to a quarter turn.
    pub fn polyline(&self) -> GeopResult<Vec<Vector3>> {
        let helix = self.helix()?;
        // A span between every pair of the double interior knots.
        let spans = (helix.control_points.len() - 1) / 2;
        let n = (spans * SEGMENTS_PER_SPAN) as i64;
        (0..=n)
            .map(|i| helix.evaluate(Scalar::from_ratio(i, n)?))
            .collect()
    }
}

/// Segments a cosmetic thread's helix is drawn with per span of a quarter
/// turn.
const SEGMENTS_PER_SPAN: usize = 8;

/// A part's cosmetic threads, by name: the value of its target [`threads`],
/// which every step that records one revises.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Threads(BTreeMap<String, CosmeticThread>);

/// The part's cosmetic threads.
pub fn threads() -> TargetReference<Threads> {
    TargetReference::new("threads")
}

impl Threads {
    /// Records `thread` under `name`. Fails, leaving the threads as they
    /// are, if there is a thread of that name already.
    pub fn add(&mut self, name: impl Into<String>, thread: CosmeticThread) -> GeopResult<()> {
        let name = name.into();
        if self.0.contains_key(&name) {
            return Err(GeopError::new(format!(
                "the part has a thread named {name:?} already"
            )));
        }
        self.0.insert(name, thread);
        Ok(())
    }

    /// Every cosmetic thread, by name.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &CosmeticThread)> {
        self.0.iter().map(|(name, thread)| (name.as_str(), thread))
    }

    /// The cosmetic thread `name`.
    pub fn get(&self, name: &str) -> GeopResult<&CosmeticThread> {
        (self.0.get(name))
            .ok_or_else(|| GeopError::new(format!("the part has no thread named {name:?}")))
    }

    /// The cosmetic threads the step `step` recorded — those named after
    /// it — by name.
    pub fn of_step(&self, step: &str) -> Vec<String> {
        (self.0.keys())
            .filter(|name| operation_of(name) == Some(step))
            .cloned()
            .collect()
    }
}

impl Threads {
    /// What a viewer draws of each thread the step `step` recorded: its
    /// name, its designation and its helix as a polyline.
    pub fn drawn(&self, step: &str) -> GeopResult<Vec<ViewAnnotation>> {
        (self.0.iter())
            .filter(|(name, _)| operation_of(name) == Some(step))
            .map(|(name, thread)| {
                Ok(ViewAnnotation {
                    name: name.clone(),
                    label: thread.designation.clone(),
                    polyline: thread.polyline()?,
                })
            })
            .collect()
    }
}

/// Every step that records threads adds to them.
impl Target for Threads {
    const CHAIN: bool = true;
}

/// Read whole.
impl Kind for Threads {
    const NAME: &'static str = "threads";
}

impl HasElements<Threads> for Threads {
    type Element<'a> = &'a Threads;
    const SINGLETON: Option<&'static str> = Some("threads");

    fn element(&self, name: &str) -> GeopResult<&Threads> {
        whole(self, name)
    }
}

/// The part's cosmetic threads as `reader` reads them: none, before the
/// first step that records one.
pub fn read_threads<'a>(reader: &Reader<'a>) -> GeopResult<std::borrow::Cow<'a, Threads>> {
    if !reader.defines(&threads()) {
        return Ok(std::borrow::Cow::Owned(Threads::default()));
    }
    let whole = ElementReference::<Threads, Threads>::whole(threads());
    Ok(std::borrow::Cow::Borrowed(reader.retrieve_element(&whole)?))
}

/// Draws the threads the step recorded on the part, as its
/// [`annotations`]: what a step that records threads does after.
pub fn draw_threads(ctx: &mut StepContext) -> GeopResult<()> {
    let step = ctx.step();
    ctx.produce(&annotations(step), &step.to_string(), draw)
}

/// The make of [`draw_threads`]: its arguments, the step, are what a
/// node is made from, so owned.
#[allow(clippy::ptr_arg)]
fn draw(reader: &Reader, step: &String) -> GeopResult<Annotations> {
    Ok(Annotations(read_threads(reader)?.drawn(step)?))
}
