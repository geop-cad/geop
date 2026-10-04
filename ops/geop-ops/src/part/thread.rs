//! A [`Part`]'s cosmetic threads: threads recorded on a round face rather
//! than modelled — what a drawing, a bill of materials or a fastener check
//! reads, and what the viewer draws as a helix on the face.

use geop_core_geometry::{
    nurb_curve::{Handedness, NurbCurve3D},
    shape::Axis,
};
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
};

use super::Part;
use crate::operation::frame_along;

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
}

impl<S: Scalar> Part<S> {
    /// Records `thread` under `name`. Fails, leaving the part unchanged, if
    /// the part has a thread of that name already.
    pub fn add_thread(
        &mut self,
        name: impl Into<String>,
        thread: CosmeticThread<S>,
    ) -> GeopResult<()> {
        let name = name.into();
        if self.threads.contains_key(&name) {
            return Err(GeopError::new(format!(
                "the part has a thread named {name:?} already"
            )));
        }
        self.threads.insert(name, thread);
        Ok(())
    }

    /// Every cosmetic thread, by name.
    pub fn threads(&self) -> impl Iterator<Item = (&str, &CosmeticThread<S>)> {
        self.threads.iter().map(|(name, t)| (name.as_str(), t))
    }

    /// The cosmetic thread `name`.
    pub fn thread(&self, name: &str) -> GeopResult<&CosmeticThread<S>> {
        self.threads
            .get(name)
            .ok_or_else(|| GeopError::new(format!("the part has no thread named {name:?}")))
    }
}
