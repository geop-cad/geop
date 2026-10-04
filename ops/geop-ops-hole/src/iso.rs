//! ISO metric screw threads and the holes made for them, in millimetres:
//! the coarse series of ISO 261 from M1.6 to M24, with
//!
//! - the pitch (ISO 261) and the basic minor diameter `D1 = D - 1.0825 P`
//!   (ISO 724);
//! - the tap drill for a 6H internal thread (ISO 2306);
//! - clearance holes, close (fine), normal (medium) and loose (coarse) fit
//!   (ISO 273);
//! - the counterbore for a socket head cap screw (ISO 4762): its diameter as
//!   DIN 974-1 gives it, its depth the screw's head height `k = D`, so the
//!   head sits flush;
//! - the 90° countersink for a countersunk socket screw (ISO 10642): its
//!   diameter as ISO 15065 gives it, which stops at M20.

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

/// One size of the coarse series.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MetricSize {
    /// `M6`.
    pub name: &'static str,
    /// The nominal (major) diameter.
    pub diameter: f64,
    pub pitch: f64,
    pub tap_drill: f64,
    /// Close, normal and loose clearance holes.
    pub clearance: [f64; 3],
    /// The counterbore's diameter; its depth is the head height, `diameter`.
    pub counterbore: f64,
    /// The countersink's diameter, if ISO 15065 gives one.
    pub countersink: Option<f64>,
}

/// `H`, the height of the fundamental triangle of the ISO metric profile,
/// per unit of pitch: `sqrt(3) / 2`.
pub const TRIANGLE_HEIGHT: f64 = 0.866_025_403_784_438_6;

impl MetricSize {
    /// The basic minor diameter, ISO 724: `D - 5/4 H`, twice over.
    pub fn minor_diameter(&self) -> f64 {
        self.diameter - 1.25 * TRIANGLE_HEIGHT * self.pitch
    }

    /// How the thread is called: `M6x1`.
    pub fn designation(&self) -> String {
        format!("{}x{}", self.name, self.pitch)
    }

    /// The clearance hole for `fit`.
    pub fn clearance(&self, fit: Fit) -> f64 {
        self.clearance[fit as usize]
    }
}

/// How much play a clearance hole leaves around its screw (ISO 273).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Fit {
    /// Fine.
    Close = 0,
    /// Medium.
    #[default]
    Normal = 1,
    /// Coarse.
    Loose = 2,
}

impl Fit {
    pub const ALL: [Fit; 3] = [Fit::Close, Fit::Normal, Fit::Loose];

    /// As it serializes: `close`.
    pub fn value(self) -> &'static str {
        match self {
            Fit::Close => "close",
            Fit::Normal => "normal",
            Fit::Loose => "loose",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Fit::Close => "Close",
            Fit::Normal => "Normal",
            Fit::Loose => "Loose",
        }
    }

    pub fn from_value(value: &str) -> Option<Fit> {
        Fit::ALL.into_iter().find(|f| f.value() == value)
    }
}

const fn size(
    name: &'static str,
    diameter: f64,
    pitch: f64,
    tap_drill: f64,
    clearance: [f64; 3],
    counterbore: f64,
    countersink: Option<f64>,
) -> MetricSize {
    MetricSize {
        name,
        diameter,
        pitch,
        tap_drill,
        clearance,
        counterbore,
        countersink,
    }
}

/// The coarse series, smallest first.
pub const SIZES: [MetricSize; 16] = [
    size("M1.6", 1.6, 0.35, 1.25, [1.7, 1.8, 2.0], 3.5, Some(3.6)),
    size("M2", 2.0, 0.4, 1.6, [2.2, 2.4, 2.6], 4.4, Some(4.4)),
    size("M2.5", 2.5, 0.45, 2.05, [2.7, 2.9, 3.1], 5.5, Some(5.5)),
    size("M3", 3.0, 0.5, 2.5, [3.2, 3.4, 3.6], 6.5, Some(6.3)),
    size("M4", 4.0, 0.7, 3.3, [4.3, 4.5, 4.8], 8.0, Some(9.4)),
    size("M5", 5.0, 0.8, 4.2, [5.3, 5.5, 5.8], 10.0, Some(10.4)),
    size("M6", 6.0, 1.0, 5.0, [6.4, 6.6, 7.0], 11.0, Some(12.6)),
    size("M8", 8.0, 1.25, 6.8, [8.4, 9.0, 10.0], 15.0, Some(17.3)),
    size("M10", 10.0, 1.5, 8.5, [10.5, 11.0, 12.0], 18.0, Some(20.0)),
    size(
        "M12",
        12.0,
        1.75,
        10.2,
        [13.0, 13.5, 14.5],
        20.0,
        Some(24.0),
    ),
    size("M14", 14.0, 2.0, 12.0, [15.0, 15.5, 16.5], 24.0, Some(28.0)),
    size("M16", 16.0, 2.0, 14.0, [17.0, 17.5, 18.5], 26.0, Some(32.0)),
    size("M18", 18.0, 2.5, 15.5, [19.0, 20.0, 21.0], 30.0, Some(36.0)),
    size("M20", 20.0, 2.5, 17.5, [21.0, 22.0, 24.0], 33.0, Some(40.0)),
    size("M22", 22.0, 2.5, 19.5, [23.0, 24.0, 26.0], 36.0, None),
    size("M24", 24.0, 3.0, 21.0, [25.0, 26.0, 28.0], 40.0, None),
];

/// The size called `name`: `M6`.
pub fn metric(name: &str) -> GeopResult<&'static MetricSize> {
    SIZES.iter().find(|s| s.name == name).ok_or_else(|| {
        let known: Vec<&str> = SIZES.iter().map(|s| s.name).collect();
        GeopError::new(format!(
            "{name:?} is no ISO metric coarse thread size; there are {}",
            known.join(", ")
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The tables hold together: each hole is larger than the one it
    /// clears — tap drill below the nominal diameter, between minor and
    /// nominal; clearance holes above it, close to loose; heads wider still.
    #[test]
    fn the_tables_are_consistent() {
        for s in &SIZES {
            assert!(
                s.minor_diameter() < s.tap_drill && s.tap_drill < s.diameter,
                "{s:?}"
            );
            assert!(s.diameter < s.clearance[0], "{s:?}");
            assert!(
                s.clearance[0] < s.clearance[1] && s.clearance[1] < s.clearance[2],
                "{s:?}"
            );
            assert!(s.clearance[2] < s.counterbore, "{s:?}");
            if let Some(cs) = s.countersink {
                assert!(s.clearance[2] < cs, "{s:?}");
            }
        }
        assert_eq!(metric("M6").unwrap().designation(), "M6x1");
        assert_eq!(metric("M8").unwrap().designation(), "M8x1.25");
        assert!(metric("M7").is_err());
    }
}
