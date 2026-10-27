//! American National Standard taper pipe threads, NPT (ASME B1.20.1), from
//! 1/16 to 2 inch, in millimetres: a 60° thread on a cone tapering 1 in 16
//! on the diameter, so that its radius changes by [`TAPER`] per unit of
//! length.
//!
//! Each size is given by its threads per inch, the pipe's outside diameter,
//! the pitch diameter `E0` at the small end of the external thread (the
//! pipe's end), the hand-tight engagement `L1` and the effective thread
//! length `L2`. An internal thread is cut so that a pipe screwed in by hand
//! reaches `L1` in: its pitch diameter at the face it is tapped from is
//! `E1 = E0 + L1 / 16`. The thread is `h = 0.8 P` high, truncated equally at
//! its crest and its root from the sharp V of height `H = 0.866 P`, so its
//! major and minor diameters are `h` either side of the pitch diameter.
//!
//! A tapped hole for it is reamed to the taper, its diameter at the face the
//! minor diameter there, `E1 - h`.

use geop_core_math::geop_error::{GeopError, GeopResult};

use crate::iso::TRIANGLE_HEIGHT;

/// How much an NPT thread's radius changes per unit of length: half its
/// taper of 1 in 16 on the diameter.
pub const TAPER: f64 = 1.0 / 32.0;

/// Millimetres per inch.
const INCH: f64 = 25.4;

/// One size of NPT.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PipeSize {
    /// `1/2`: the nominal pipe size, in inches.
    pub name: &'static str,
    pub threads_per_inch: f64,
    /// The pipe's outside diameter.
    pub outside: f64,
    /// The pitch diameter at the small end of the external thread.
    pub e0: f64,
    /// The hand-tight engagement.
    pub l1: f64,
    /// The effective thread length of the external thread.
    pub l2: f64,
}

impl PipeSize {
    pub fn pitch(&self) -> f64 {
        INCH / self.threads_per_inch
    }

    /// The thread's height, from its root to its crest: `0.8 P`.
    pub fn height(&self) -> f64 {
        0.8 * self.pitch()
    }

    /// The pitch diameter at the large end of the internal thread, where a
    /// hole is tapped from: `E1 = E0 + L1 / 16`.
    pub fn e1(&self) -> f64 {
        self.e0 + 2.0 * TAPER * self.l1
    }

    /// The major and minor diameters where a thread starts: an internal
    /// one at its large end, the face a hole is tapped from (`E1`); an
    /// external one at its small end, the pipe's end (`E0`).
    pub fn diameters(&self, internal: bool) -> (f64, f64) {
        let pitch_diameter = if internal { self.e1() } else { self.e0 };
        (
            pitch_diameter + self.height(),
            pitch_diameter - self.height(),
        )
    }

    /// How far the sharp V's crest lies beyond the major diameter, in
    /// radius: half of what the truncation takes off.
    pub fn crest(&self) -> f64 {
        (TRIANGLE_HEIGHT * self.pitch() - self.height()) / 2.0
    }

    /// How the thread is called: `1/2-14 NPT`.
    pub fn designation(&self) -> String {
        format!("{}-{} NPT", self.name, self.threads_per_inch)
    }
}

const fn size(
    name: &'static str,
    threads_per_inch: f64,
    outside: f64,
    e0: f64,
    l1: f64,
    l2: f64,
) -> PipeSize {
    PipeSize {
        name,
        threads_per_inch,
        outside: outside * INCH,
        e0: e0 * INCH,
        l1: l1 * INCH,
        l2: l2 * INCH,
    }
}

/// The sizes of ASME B1.20.1 Table 2 from 1/16 to 2, smallest first.
pub const SIZES: [PipeSize; 10] = [
    size("1/16", 27.0, 0.3125, 0.27118, 0.160, 0.2611),
    size("1/8", 27.0, 0.405, 0.36351, 0.1615, 0.2639),
    size("1/4", 18.0, 0.540, 0.47739, 0.2278, 0.4018),
    size("3/8", 18.0, 0.675, 0.61201, 0.240, 0.4078),
    size("1/2", 14.0, 0.840, 0.75843, 0.320, 0.5337),
    size("3/4", 14.0, 1.050, 0.96768, 0.339, 0.5457),
    size("1", 11.5, 1.315, 1.21363, 0.400, 0.6828),
    size("1-1/4", 11.5, 1.660, 1.55713, 0.420, 0.7068),
    size("1-1/2", 11.5, 1.900, 1.79609, 0.420, 0.7235),
    size("2", 11.5, 2.375, 2.26902, 0.436, 0.7565),
];

/// The size called `name`: `1/2`.
pub fn npt(name: &str) -> GeopResult<&'static PipeSize> {
    SIZES.iter().find(|s| s.name == name).ok_or_else(|| {
        let known: Vec<&str> = SIZES.iter().map(|s| s.name).collect();
        GeopError::new(format!(
            "{name:?} is no NPT pipe size; there are {}",
            known.join(", ")
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The table holds together: the thread's major diameter at the pipe's
    /// end lies below the pipe's outside diameter, and the hand-tight plane
    /// within the effective length; sizes grow.
    #[test]
    fn the_table_is_consistent() {
        for s in &SIZES {
            let (major, minor) = s.diameters(false);
            assert!(minor < s.e0 && s.e0 < major, "{s:?}");
            assert!(major < s.outside, "{s:?}");
            assert!(s.l1 < s.l2, "{s:?}");
            assert!(s.diameters(true).0 > major, "{s:?}");
            assert!(s.crest() > 0.0, "{s:?}");
        }
        for w in SIZES.windows(2) {
            assert!(w[0].outside < w[1].outside, "{w:?}");
        }
        let half = npt("1/2").unwrap();
        assert_eq!(half.designation(), "1/2-14 NPT");
        // E1 of 1/2 NPT is 0.77843 in.
        assert!((half.e1() / INCH - 0.77843).abs() < 1e-9);
        assert!(npt("5/8").is_err());
    }
}
