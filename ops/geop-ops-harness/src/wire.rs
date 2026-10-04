//! [`Wire`]s, and the bundle they make: how thick each is, by its gauge or
//! its diameter, and how thick they are together.

use geop_core_math::geop_error::{GeopError, GeopResult};
use serde::{Deserialize, Serialize};

/// How thick a wire is.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum WireSize {
    /// By its American wire gauge — `0` to `40`, the conductor's — and the
    /// wall of insulation round it.
    Awg { gauge: i32, insulation: f64 },
    /// By its outer diameter, insulation included.
    Diameter(f64),
}

/// The gauges [`WireSize::Awg`] takes.
pub const GAUGES: std::ops::RangeInclusive<i32> = 0..=40;

impl WireSize {
    /// The outer diameter, insulation included. The conductor of gauge `n`
    /// is `0.127 mm × 92^((36 − n) / 39)` across, which is what defines the
    /// gauges.
    pub fn diameter(&self) -> GeopResult<f64> {
        let d = match *self {
            WireSize::Awg { gauge, insulation } => {
                if !GAUGES.contains(&gauge) {
                    return Err(GeopError::new(format!(
                        "AWG {gauge} is no gauge: gauges run from {} to {}",
                        GAUGES.start(),
                        GAUGES.end()
                    )));
                }
                if !(insulation >= 0.0 && insulation.is_finite()) {
                    return Err(GeopError::new(format!(
                        "an insulation {insulation} thick: it is a length, zero or more"
                    )));
                }
                0.127 * 92f64.powf((36 - gauge) as f64 / 39.0) + 2.0 * insulation
            }
            WireSize::Diameter(d) => d,
        };
        if d > 0.0 && d.is_finite() {
            Ok(d)
        } else {
            Err(GeopError::new(format!(
                "a wire {d} across: a diameter is more than zero"
            )))
        }
    }

    /// Its gauge, if it is given by one.
    pub fn gauge(&self) -> Option<i32> {
        match *self {
            WireSize::Awg { gauge, .. } => Some(gauge),
            WireSize::Diameter(_) => None,
        }
    }
}

/// A wire of a route: what it is called, how thick it is, and its colour.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Wire {
    pub name: String,
    pub size: WireSize,
    /// `#rrggbb`.
    pub colour: String,
}

impl Wire {
    /// A wire of gauge 22 with a wall of 0.3 — common hookup wire, about
    /// 1.2 mm across — black.
    pub fn new(name: impl Into<String>) -> Self {
        Wire {
            name: name.into(),
            size: WireSize::Awg {
                gauge: 22,
                insulation: 0.3,
            },
            colour: "#000000".into(),
        }
    }
}

/// How thick `wires` are, bundled: the diameter of a circle whose area is
/// theirs over `fill`, the fraction of a bundle's section its wires fill —
/// round wires never fill it all — `sqrt(sum d² / fill)`.
pub fn bundle_diameter(wires: &[Wire], fill: f64) -> GeopResult<f64> {
    if wires.is_empty() {
        return Err(GeopError::new("the route has no wires: add one"));
    }
    if !(fill > 0.0 && fill <= 1.0) {
        return Err(GeopError::new(format!(
            "a fill factor of {fill}: it is the fraction of the bundle the wires fill, more than 0 and at most 1"
        )));
    }
    let mut area = 0.0;
    for wire in wires {
        let d = wire
            .size
            .diameter()
            .map_err(|e| e.with_context(format!("wire {:?}", wire.name)))?;
        area += d * d;
    }
    Ok((area / fill).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Gauges are what they are defined as: 36 is 0.127 across, every six
    /// gauges down doubles it, roughly; 22 is 0.644.
    #[test]
    fn gauges_give_diameters() {
        let bare = |gauge| {
            WireSize::Awg {
                gauge,
                insulation: 0.0,
            }
            .diameter()
            .unwrap()
        };
        assert_eq!(bare(36), 0.127);
        assert!((bare(22) - 0.6438).abs() < 1e-4, "{}", bare(22));
        assert!((bare(16) / bare(22) - 2.0).abs() < 0.01);
        let insulated = WireSize::Awg {
            gauge: 22,
            insulation: 0.3,
        };
        assert!((insulated.diameter().unwrap() - bare(22) - 0.6).abs() < 1e-12);
        assert!(
            WireSize::Awg {
                gauge: 41,
                insulation: 0.3
            }
            .diameter()
            .is_err()
        );
        assert!(WireSize::Diameter(0.0).diameter().is_err());
    }

    /// Four wires of one diameter, filling half the bundle, make a bundle
    /// `sqrt(4 / 0.5) = 2 sqrt 2` of them across.
    #[test]
    fn bundles_are_as_thick_as_their_wires_and_fill() {
        let wires: Vec<Wire> = (0..4)
            .map(|i| Wire {
                size: WireSize::Diameter(1.5),
                ..Wire::new(format!("w{i}"))
            })
            .collect();
        let d = bundle_diameter(&wires, 0.5).unwrap();
        assert!((d - 1.5 * 8f64.sqrt()).abs() < 1e-12, "{d}");
        assert!(bundle_diameter(&[], 0.5).is_err());
        assert!(bundle_diameter(&wires, 0.0).is_err());
        assert!(bundle_diameter(&wires, 1.5).is_err());
    }
}
