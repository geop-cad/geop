//! Motion studies: joint coordinates driven by a time table
//! ([`MotionStudy`]), each a step's ([`motion_study`]), read in the order
//! the steps made them ([`motion_studies`]) — by the editor, which plays
//! one by setting the coordinates to their values at a time and solving,
//! and by whatever plays, checks or exports a motion.

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_program::{
    element::{ElementReference, HasElements, Kind, whole},
    reader::Reader,
    target::{Target, TargetReference},
};
use serde::{Deserialize, Serialize};

/// Joint coordinates driven by a time table: their values at keyframes,
/// and between two keyframes the straight line between them. A motion runs
/// from its first keyframe's time to its last's, and is played, checked
/// and exported at `frames` times evenly apart over that.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MotionStudy {
    /// The coordinates driven, by the names of their states:
    /// `add_part(fore,m1).angle`.
    pub drivers: Vec<String>,
    /// The keyframes, each its time — in seconds — and then a value per
    /// driver, in its own units: degrees for an angle.
    pub keys: Vec<Vec<f64>>,
    /// How many frames it is played at, checked and exported at: at least
    /// two, its first keyframe and its last.
    pub frames: usize,
}

impl MotionStudy {
    /// Checks it is one: keyframes a time and a value for every driver
    /// each, later each than the one before, and frames enough to start
    /// and end.
    pub fn validate(&self) -> GeopResult<()> {
        if self.keys.is_empty() {
            return Err(GeopError::new("a motion needs a keyframe: add a row"));
        }
        for (k, key) in self.keys.iter().enumerate() {
            if key.len() != self.drivers.len() + 1 {
                return Err(GeopError::new(format!(
                    "keyframe {} has {} values for {} drivers and its time",
                    k + 1,
                    key.len(),
                    self.drivers.len()
                )));
            }
            if key.iter().any(|v| !v.is_finite()) {
                return Err(GeopError::new(format!(
                    "keyframe {} holds a value that is no number: {key:?}",
                    k + 1
                )));
            }
        }
        for (k, pair) in self.keys.windows(2).enumerate() {
            if pair[1][0] <= pair[0][0] {
                return Err(GeopError::new(format!(
                    "keyframe {} is at {} s, no later than the one before it at {} s",
                    k + 2,
                    pair[1][0],
                    pair[0][0]
                )));
            }
        }
        if self.frames < 2 {
            return Err(GeopError::new(format!(
                "a motion is played at its start and its end at least: 2 frames, not {}",
                self.frames
            )));
        }
        Ok(())
    }

    /// When it starts and ends, in seconds.
    pub fn span(&self) -> (f64, f64) {
        let time = |key: Option<&Vec<f64>>| key.and_then(|k| k.first().copied());
        let start = time(self.keys.first()).unwrap_or(0.0);
        (start, time(self.keys.last()).unwrap_or(start))
    }

    /// The value of each driver at the time `t`, by driver: between two
    /// keyframes on the line between them, before the first its first, and
    /// after the last its last.
    pub fn at(&self, t: f64) -> Vec<(String, f64)> {
        let after = self.keys.iter().position(|key| key[0] > t);
        let value = |d: usize| match after {
            Some(0) => self.keys[0][d + 1],
            None => self.keys.last().map_or(0.0, |key| key[d + 1]),
            Some(k) => {
                let (a, b) = (&self.keys[k - 1], &self.keys[k]);
                let s = (t - a[0]) / (b[0] - a[0]);
                a[d + 1] + s * (b[d + 1] - a[d + 1])
            }
        };
        (self.drivers.iter().enumerate())
            .map(|(d, driver)| (driver.clone(), value(d)))
            .collect()
    }

    /// The times of its frames: evenly apart from its start to its end.
    pub fn times(&self) -> Vec<f64> {
        let (start, end) = self.span();
        let frames = self.frames.max(2);
        (0..frames)
            .map(|k| start + (end - start) * k as f64 / (frames - 1) as f64)
            .collect()
    }
}

/// Each motion step's own.
impl Target for MotionStudy {}

/// Read whole.
impl Kind for MotionStudy {
    const NAME: &'static str = "motion";
}

/// What the target of a motion study is named, before the step's id.
const PREFIX: &str = "motion:";

impl HasElements<MotionStudy> for MotionStudy {
    type Element<'a> = &'a MotionStudy;

    fn element(&self, name: &str) -> GeopResult<&MotionStudy> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(PREFIX)
    }
}

/// The motion study the step `step` makes.
pub fn motion_study(step: &str) -> TargetReference<MotionStudy> {
    TargetReference::new(format!("{PREFIX}{step}"))
}

/// Every motion study of the part `reader` reads, by the id of the step
/// that made it, in the order the steps made them.
pub fn motion_studies<'a>(reader: &Reader<'a>) -> Vec<(String, &'a MotionStudy)> {
    (reader.targets_of::<MotionStudy>().into_iter())
        .filter_map(|target| {
            let step = target.name().strip_prefix(PREFIX)?.to_string();
            let study = reader
                .retrieve_element(&ElementReference::whole(target))
                .ok()?;
            Some((step, study))
        })
        .collect()
}
