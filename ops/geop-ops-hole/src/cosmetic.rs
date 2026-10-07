//! Cosmetic threads: threads recorded on a round face rather than modelled
//! — what a drawing, a bill of materials or a fastener check reads, and
//! what the viewer draws as a helix on the face. Kept in a part as the
//! extension [`Threads`], read and written through [`PartThreads`].

use std::collections::BTreeMap;

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
use geop_ops::{Annotation, Extension, Part, operation::frame_along, part::operation_of};

/// A thread recorded on a cylindrical face, by its specification and where
/// it lies — kept as geometry, so that it says the same whatever later
/// steps do to the face it was put on.
#[derive(Clone, Debug)]
pub struct CosmeticThread<S: Scalar> {
    /// How it is called: `M6x1`.
    pub designation: String,
    /// The face it was put on, by its name then.
    pub face: String,
    /// Where it starts, on the axis of its face, and the way it runs from
    /// there (a unit vector).
    pub axis: Axis<S>,
    /// The radius of the face it lies on: where it is drawn.
    pub radius: S,
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

impl<S: Scalar> CosmeticThread<S> {
    /// The helix it is drawn as: on its face, from where it starts, one
    /// turn per pitch, as far as it runs.
    pub fn helix(&self) -> GeopResult<NurbCurve3D<S>> {
        let frame: CoordinateSystem<S> = frame_along(self.axis.point, &self.axis.direction)?;
        NurbCurve3D::helix(
            &frame,
            self.radius,
            S::from_f64(self.pitch),
            self.length / self.pitch,
            self.handedness,
        )
    }

    /// The helix as a polyline, [`SEGMENTS_PER_SPAN`] to a quarter turn.
    fn polyline(&self) -> GeopResult<Vec<Vector3<S>>> {
        let helix = self.helix()?;
        // A span between every pair of the double interior knots.
        let spans = (helix.control_points.len() - 1) / 2;
        let n = (spans * SEGMENTS_PER_SPAN) as i64;
        (0..=n)
            .map(|i| helix.evaluate(S::from_ratio(i, n)?))
            .collect()
    }
}

/// Segments a cosmetic thread's helix is drawn with per span of a quarter
/// turn.
const SEGMENTS_PER_SPAN: usize = 8;

/// A part's cosmetic threads, by name.
#[derive(Clone, Debug)]
pub struct Threads<S: Scalar>(BTreeMap<String, CosmeticThread<S>>);

impl<S: Scalar> Default for Threads<S> {
    fn default() -> Self {
        Self(BTreeMap::new())
    }
}

impl<S: Scalar> Extension<S> for Threads<S> {
    const NAME: &'static str = "threads";

    fn annotations(&self) -> GeopResult<Vec<Annotation<S>>> {
        self.0
            .iter()
            .map(|(name, thread)| {
                Ok(Annotation {
                    name: name.clone(),
                    label: thread.designation.clone(),
                    polyline: thread.polyline()?,
                })
            })
            .collect()
    }

    fn describe(&self) -> BTreeMap<String, serde_json::Value> {
        self.0
            .iter()
            .map(|(name, thread)| {
                let description = serde_json::json!({
                    "designation": thread.designation,
                    "face": thread.face,
                    "length": thread.length,
                    "internal": thread.internal,
                });
                (name.clone(), description)
            })
            .collect()
    }
}

/// A part's cosmetic threads (see the module).
pub trait PartThreads<S: Scalar> {
    /// Records `thread` under `name`. Fails, leaving the part unchanged, if
    /// the part has a thread of that name already.
    fn add_thread(&mut self, name: impl Into<String>, thread: CosmeticThread<S>) -> GeopResult<()>;

    /// Every cosmetic thread, by name.
    fn threads(&self) -> impl Iterator<Item = (&str, &CosmeticThread<S>)>;

    /// The cosmetic thread `name`.
    fn thread(&self, name: &str) -> GeopResult<&CosmeticThread<S>>;

    /// The cosmetic threads the step `step` recorded — those named after
    /// it — by name.
    fn threads_of(&self, step: &str) -> Vec<String>;
}

impl<S: Scalar> PartThreads<S> for Part<S> {
    fn add_thread(&mut self, name: impl Into<String>, thread: CosmeticThread<S>) -> GeopResult<()> {
        let name = name.into();
        if self
            .ext::<Threads<S>>()
            .is_some_and(|t| t.0.contains_key(&name))
        {
            return Err(GeopError::new(format!(
                "the part has a thread named {name:?} already"
            )));
        }
        self.ext_mut::<Threads<S>>().0.insert(name, thread);
        Ok(())
    }

    fn threads(&self) -> impl Iterator<Item = (&str, &CosmeticThread<S>)> {
        self.ext::<Threads<S>>()
            .into_iter()
            .flat_map(|threads| threads.0.iter().map(|(name, t)| (name.as_str(), t)))
    }

    fn thread(&self, name: &str) -> GeopResult<&CosmeticThread<S>> {
        self.ext::<Threads<S>>()
            .and_then(|threads| threads.0.get(name))
            .ok_or_else(|| GeopError::new(format!("the part has no thread named {name:?}")))
    }

    fn threads_of(&self, step: &str) -> Vec<String> {
        self.threads()
            .map(|(name, _)| name)
            .filter(|name| operation_of(name) == Some(step))
            .map(str::to_string)
            .collect()
    }
}
