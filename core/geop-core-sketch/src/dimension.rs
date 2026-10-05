//! How a dimension is drawn in a plane, in plain numbers ([`Measure`]): the
//! lines that show what it measures with its value at a label point, and
//! where their arrowheads go. A sketch's dimensions and a drawing's are
//! both drawn by it, so the two look alike and are placed alike: a value
//! dragged anywhere, its lines following.
//!
//! Like [`crate::plain`], this is only ever what is shown — where a label
//! goes is the designer's free choice — never a result the kernel reasons
//! about.

use crate::plain::{P2, add, dist, dot, perp, rotate, scale, sub, unit};

/// What a dimension measures, where it is in the plane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Measure {
    /// From `a` to `b`, along the direction `along`: an aligned distance
    /// along `b - a`, a horizontal or a vertical one along an axis.
    Linear { a: P2, b: P2, along: P2 },
    /// The radius of a circle — or, `diameter`, its diameter.
    Radial {
        center: P2,
        radius: f64,
        diameter: bool,
    },
    /// The angle at `vertex` between its sides along the directions `from`
    /// and `to`, measured turning `sweep` radians counter-clockwise from
    /// `from`.
    Angular {
        vertex: P2,
        from: P2,
        to: P2,
        sweep: f64,
    },
}

/// How many pieces an angle's arc is drawn in.
const ARC_PIECES: usize = 32;

impl Measure {
    /// The lines it is drawn with when its value is shown at `label`: for a
    /// linear one, extension lines from what it measures out to a
    /// dimension line through the label, along what it measures — and on
    /// to the label if it is beside them; for a radial one, a leader from
    /// the centre — a diameter's from the far rim — across the circle, out
    /// to the label if it is beyond the rim; for an angle, an arc about its
    /// vertex through the label, and its sides out to the arc. None for
    /// what has no direction to be drawn along.
    pub fn lines(&self, label: P2) -> Vec<Vec<P2>> {
        match *self {
            Measure::Linear { a, b, along } => {
                let Some((a2, b2, u)) = Self::feet(a, b, along, label) else {
                    return Vec::new();
                };
                // The dimension line runs between the extension lines, and
                // on to the label if it is beside them.
                let t = |q: P2| dot(sub(q, a2), u);
                let (lo, hi) = (t(b2).min(0.0).min(t(label)), t(b2).max(0.0).max(t(label)));
                vec![
                    vec![a, a2],
                    vec![b, b2],
                    vec![add(a2, scale(u, lo)), add(a2, scale(u, hi))],
                ]
            }
            Measure::Radial {
                center,
                radius,
                diameter,
            } => {
                let Some(d) = unit(sub(label, center)) else {
                    return Vec::new();
                };
                let rim = add(center, scale(d, radius));
                let from = if diameter {
                    add(center, scale(d, -radius))
                } else {
                    center
                };
                // Out to the label, if it is beyond the rim.
                let to = if dot(sub(label, center), d) > radius {
                    label
                } else {
                    rim
                };
                vec![vec![from, to]]
            }
            Measure::Angular {
                vertex, from, to, ..
            } => {
                let r = dist(label, vertex);
                let side = |d: P2| vec![vertex, add(vertex, scale(unit(d).unwrap_or(d), r))];
                vec![self.arc(r), side(from), side(to)]
            }
        }
    }

    /// Its arrowheads, as drafting draws them, with its value at `label`:
    /// each tip, and the unit direction it points in. A linear dimension's
    /// point out from between its extension lines, a radius' out at the
    /// rim — a diameter's at both rims — and an angle's along its arc, out
    /// at either end.
    pub fn arrows(&self, label: P2) -> Vec<(P2, P2)> {
        match *self {
            Measure::Linear { a, b, along, .. } => {
                let Some((a2, b2, _)) = Self::feet(a, b, along, label) else {
                    return Vec::new();
                };
                match unit(sub(b2, a2)) {
                    Some(d) => vec![(a2, scale(d, -1.0)), (b2, d)],
                    None => Vec::new(),
                }
            }
            Measure::Radial {
                center,
                radius,
                diameter,
            } => {
                let Some(d) = unit(sub(label, center)) else {
                    return Vec::new();
                };
                let mut arrows = vec![(add(center, scale(d, radius)), d)];
                if diameter {
                    arrows.push((add(center, scale(d, -radius)), scale(d, -1.0)));
                }
                arrows
            }
            Measure::Angular {
                vertex,
                from,
                sweep,
                ..
            } => {
                let r = dist(label, vertex);
                let (Some(from), true) = (unit(from), r > 0.0 && sweep != 0.0) else {
                    return Vec::new();
                };
                let arc = self.arc(r);
                let turn = sweep.signum();
                let end = rotate(from, sweep);
                vec![
                    (arc[0], scale(perp(from), -turn)),
                    (arc[arc.len() - 1], scale(perp(end), turn)),
                ]
            }
        }
    }

    /// Where a linear dimension's extension lines meet its dimension line
    /// through `label`, and the unit direction it runs in.
    fn feet(a: P2, b: P2, along: P2, label: P2) -> Option<(P2, P2, P2)> {
        let u = unit(along)?;
        let n = perp(u);
        let lift = |q: P2| add(q, scale(n, dot(sub(label, q), n)));
        Some((lift(a), lift(b), u))
    }

    /// An angle's arc, `r` from its vertex.
    fn arc(&self, r: f64) -> Vec<P2> {
        let Measure::Angular {
            vertex,
            from,
            sweep,
            ..
        } = *self
        else {
            return Vec::new();
        };
        let start = from[1].atan2(from[0]);
        (0..=ARC_PIECES)
            .map(|i| {
                let t = start + sweep * i as f64 / ARC_PIECES as f64;
                add(vertex, [r * t.cos(), r * t.sin()])
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: P2, b: P2) -> bool {
        dist(a, b) < 1e-12
    }

    /// A horizontal distance drawn above what it measures: extension lines
    /// up to the label's height, the dimension line between them and on to
    /// a label beside them, arrows pointing out at its ends.
    #[test]
    fn a_linear_dimension_reaches_its_label() {
        let m = Measure::Linear {
            a: [0.0, 0.0],
            b: [4.0, 1.0],
            along: [1.0, 0.0],
        };
        let lines = m.lines([6.0, 3.0]);
        assert_eq!(lines[0], vec![[0.0, 0.0], [0.0, 3.0]]);
        assert_eq!(lines[1], vec![[4.0, 1.0], [4.0, 3.0]]);
        assert_eq!(lines[2], vec![[0.0, 3.0], [6.0, 3.0]]);
        let arrows = m.arrows([6.0, 3.0]);
        assert_eq!(
            arrows,
            vec![([0.0, 3.0], [-1.0, 0.0]), ([4.0, 3.0], [1.0, 0.0])]
        );
    }

    /// A diameter's leader crosses the circle from rim to rim, an arrow at
    /// each; a radius' runs from the centre, one arrow at the rim.
    #[test]
    fn radial_dimensions_cross_their_circle() {
        let diameter = Measure::Radial {
            center: [1.0, 1.0],
            radius: 2.0,
            diameter: true,
        };
        assert_eq!(
            diameter.lines([1.0, 5.0]),
            vec![vec![[1.0, -1.0], [1.0, 5.0]]]
        );
        assert_eq!(diameter.arrows([1.0, 5.0]).len(), 2);
        let radius = Measure::Radial {
            center: [1.0, 1.0],
            radius: 2.0,
            diameter: false,
        };
        assert_eq!(radius.lines([1.0, 2.0]), vec![vec![[1.0, 1.0], [1.0, 3.0]]]);
        assert_eq!(radius.arrows([1.0, 2.0]), vec![([1.0, 3.0], [0.0, 1.0])]);
    }

    /// A right angle's arc runs through its label, from one side to the
    /// other, its arrows pointing out along it.
    #[test]
    fn an_angle_arcs_through_its_label() {
        let m = Measure::Angular {
            vertex: [0.0, 0.0],
            from: [1.0, 0.0],
            to: [0.0, 1.0],
            sweep: std::f64::consts::FRAC_PI_2,
        };
        let label = [2.0_f64.sqrt(), 2.0_f64.sqrt()];
        let lines = m.lines(label);
        assert!(near(lines[0][0], [2.0, 0.0]) && near(*lines[0].last().unwrap(), [0.0, 2.0]));
        assert!(near(lines[2][1], [0.0, 2.0]));
        let arrows = m.arrows(label);
        assert!(near(arrows[0].1, [0.0, -1.0]) && near(arrows[1].1, [-1.0, 0.0]));
    }
}
