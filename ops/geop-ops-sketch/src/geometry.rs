//! The sketch editor's own plane geometry, in plain numbers: where the
//! pointer is, the arc a tool draws through it, where a curve's label goes.
//! This is the editor's side of the browser boundary — pointer positions,
//! previews and UI tolerances, never results the kernel reasons about.
//! What the sketch *is* is its design data, in [`Design`] scalars, read here
//! with [`xy`] and written back as sharp values; what it means comes from
//! its solver.

use std::f64::consts::TAU;

use geop_core_math::{geop_error::GeopResult, scalars::Scalar, vector::Vector2};
use geop_core_sketch::{
    CurveId, CurveKind, PointId, ProfileLoop, Sketch,
    geometry::{Arc, V},
    profile::curve_polyline,
};
use geop_ops::Design;

/// `[x, y]`, as the pointer and the screen have it.
pub type P2 = [f64; 2];

pub fn add(a: P2, b: P2) -> P2 {
    [a[0] + b[0], a[1] + b[1]]
}

pub fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

pub fn scale(a: P2, s: f64) -> P2 {
    [a[0] * s, a[1] * s]
}

pub fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub fn dist(a: P2, b: P2) -> f64 {
    let d = sub(a, b);
    d[0].hypot(d[1])
}

/// Where the sketch point `p` is drawn.
pub fn xy(sketch: &Sketch<Design>, p: geop_core_sketch::PointId) -> P2 {
    let q = sketch.points[&p].xy();
    [q[0].to_f64(), q[1].to_f64()]
}

/// Points as drawn.
fn plain(points: GeopResult<Vec<Vector2<Design>>>) -> Vec<P2> {
    points
        .map(|points| {
            points
                .iter()
                .map(|q| [q[0].to_f64(), q[1].to_f64()])
                .collect()
        })
        .unwrap_or_default()
}

/// The polyline a curve is drawn as (empty where it cannot be drawn).
pub fn polyline(sketch: &Sketch<Design>, curve: geop_core_sketch::CurveId) -> Vec<P2> {
    plain(curve_polyline(sketch, curve))
}

/// The polyline a loop is drawn as (empty where it cannot be drawn).
pub fn loop_polyline(sketch: &Sketch<Design>, lp: &ProfileLoop) -> Vec<P2> {
    plain(lp.polyline(sketch))
}

/// Where the segments from `a` to `b` and from `c` to `d` cross, if they
/// do: where a stroke of the pointer crosses a curve as drawn.
pub fn segments_cross(a: P2, b: P2, c: P2, d: P2) -> Option<P2> {
    let (u, v) = (sub(b, a), sub(d, c));
    let denom = cross(u, v);
    if denom == 0.0 {
        return None;
    }
    let w = sub(c, a);
    let (s, t) = (cross(w, v) / denom, cross(w, u) / denom);
    ((0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t)).then(|| add(a, scale(u, s)))
}

/// A curve as the pointer meets it, in plain numbers: what intersections
/// are found on, and where along it a point is. Only a suggestion of where
/// to place a point, or which piece of a curve is meant — the constraints
/// placing a point there are what holds it.
pub enum Plain {
    /// From `a` to `b`, or — `infinite` — the whole line through them.
    Line { a: P2, b: P2, infinite: bool },
    /// About `center`; an arc's part of it from the angle `start`,
    /// counter-clockwise by `sweep` (clockwise if negative).
    Round {
        center: P2,
        radius: f64,
        span: Option<(f64, f64)>,
    },
}

impl Plain {
    /// `curve` of `sketch`, `infinite` if a line; none for a spline.
    pub fn of(sketch: &Sketch<Design>, curve: CurveId, infinite: bool) -> Option<Plain> {
        let p = |q: PointId| xy(sketch, q);
        Some(match sketch.curves.get(&curve)?.kind {
            CurveKind::Line { start, end } => Plain::Line {
                a: p(start),
                b: p(end),
                infinite,
            },
            CurveKind::Circle { center, radius } => Plain::Round {
                center: p(center),
                radius: radius.to_f64(),
                span: None,
            },
            CurveKind::Arc { start, end, sweep } => {
                let arc = Arc {
                    s: V::of(&sketch.points[&start].xy()),
                    e: V::of(&sketch.points[&end].xy()),
                    half: Design::from_f64(sweep.to_f64() / 2.0),
                };
                let c = arc.center().ok()?;
                let center = [c.x.to_f64(), c.y.to_f64()];
                let from = sub(p(start), center);
                Plain::Round {
                    center,
                    radius: arc.radius().ok()?.to_f64(),
                    span: Some((from[1].atan2(from[0]), sweep.to_f64())),
                }
            }
            CurveKind::Spline { .. } => return None,
        })
    }

    /// Whether it is closed: a circle.
    pub fn closed(&self) -> bool {
        matches!(self, Plain::Round { span: None, .. })
    }

    /// Where `q`, a point of its full line or circle, is along it: from 0
    /// at its start to 1 at its end — a circle's start the angle 0, turning
    /// counter-clockwise. Beyond `[0, 1]` for a point of the line or circle
    /// outside the line's or arc's own part of it.
    pub fn param(&self, q: P2) -> f64 {
        match *self {
            Plain::Line { a, b, .. } => {
                let d = sub(b, a);
                dot(sub(q, a), d) / dot(d, d)
            }
            Plain::Round {
                center, span: None, ..
            } => {
                let r = sub(q, center);
                r[1].atan2(r[0]).rem_euclid(TAU) / TAU
            }
            Plain::Round {
                center,
                span: Some((start, sweep)),
                ..
            } => {
                let r = sub(q, center);
                ((r[1].atan2(r[0]) - start) * sweep.signum()).rem_euclid(TAU) / sweep.abs()
            }
        }
    }

    /// Where along it — in `[0, 1]`, as [`Plain::param`] — it comes
    /// nearest `q`.
    pub fn nearest(&self, q: P2) -> f64 {
        let t = self.param(q);
        match *self {
            Plain::Line { .. } => t.clamp(0.0, 1.0),
            Plain::Round { span: None, .. } => t,
            Plain::Round {
                span: Some((_, sweep)),
                ..
            } => {
                // Past the end, by `turned - sweep`, or still before the
                // start, by a full turn less `turned`.
                let turned = t * sweep.abs();
                if turned <= sweep.abs() {
                    t
                } else if turned - sweep.abs() < TAU - turned {
                    1.0
                } else {
                    0.0
                }
            }
        }
    }

    /// The point `t` along it (see [`Plain::param`]).
    pub fn at(&self, t: f64) -> P2 {
        match *self {
            Plain::Line { a, b, .. } => add(a, scale(sub(b, a), t)),
            Plain::Round {
                center,
                radius,
                span,
            } => {
                let angle = match span {
                    None => t * TAU,
                    Some((start, sweep)) => start + t * sweep,
                };
                add(center, [radius * angle.cos(), radius * angle.sin()])
            }
        }
    }

    /// The polyline from `t0` to `t1` along it (see [`Plain::param`]).
    pub fn polyline(&self, t0: f64, t1: f64) -> Vec<P2> {
        let steps = match self {
            Plain::Line { .. } => 1,
            Plain::Round { span, .. } => {
                let turn = span.map_or(1.0, |(_, sweep)| sweep.abs() / TAU);
                ((64.0 * turn * (t1 - t0).abs()).ceil() as usize).max(1)
            }
        };
        (0..=steps)
            .map(|i| self.at(t0 + (t1 - t0) * i as f64 / steps as f64))
            .collect()
    }

    /// Whether `q`, a point of its full line or circle, lies on it.
    pub fn holds(&self, q: P2) -> bool {
        match *self {
            Plain::Line { infinite: true, .. } | Plain::Round { span: None, .. } => true,
            _ => (0.0..=1.0).contains(&self.param(q)),
        }
    }

    /// Where the full lines and circles of `self` and `other` cross, kept
    /// where both actually run. Curves `touching` — tangent by constraint —
    /// meet at one point, where the two crossings close to coincide are
    /// taken as one.
    pub fn crossings(&self, other: &Plain, touching: bool) -> Vec<P2> {
        let found = match (self, other) {
            (Plain::Line { a, b, .. }, Plain::Line { a: c, b: d, .. }) => {
                let (u, v) = (sub(*b, *a), sub(*d, *c));
                let denom = cross(u, v);
                if denom == 0.0 {
                    Vec::new()
                } else {
                    vec![add(*a, scale(u, cross(sub(*c, *a), v) / denom))]
                }
            }
            (Plain::Line { a, b, .. }, Plain::Round { center, radius, .. })
            | (Plain::Round { center, radius, .. }, Plain::Line { a, b, .. }) => {
                // `a + t u` at `radius` from `center`.
                let u = sub(*b, *a);
                let f = sub(*a, *center);
                let (qa, qb, qc) = (dot(u, u), 2.0 * dot(f, u), dot(f, f) - radius * radius);
                let disc = qb * qb - 4.0 * qa * qc;
                if qa == 0.0 {
                    Vec::new()
                } else if touching {
                    vec![add(*a, scale(u, -qb / (2.0 * qa)))]
                } else if disc < 0.0 {
                    Vec::new()
                } else {
                    let root = disc.sqrt();
                    [-1.0, 1.0]
                        .map(|sign| add(*a, scale(u, (-qb + sign * root) / (2.0 * qa))))
                        .to_vec()
                }
            }
            (
                Plain::Round {
                    center: c0,
                    radius: r0,
                    ..
                },
                Plain::Round {
                    center: c1,
                    radius: r1,
                    ..
                },
            ) => {
                let d = sub(*c1, *c0);
                let dist = d[0].hypot(d[1]);
                if dist == 0.0 || (!touching && (dist > r0 + r1 || dist < (r0 - r1).abs())) {
                    Vec::new()
                } else {
                    let along = (r0 * r0 - r1 * r1 + dist * dist) / (2.0 * dist);
                    let base = add(*c0, scale(d, along / dist));
                    if touching {
                        vec![base]
                    } else {
                        let off = (r0 * r0 - along * along).max(0.0).sqrt();
                        let n = [-d[1] / dist, d[0] / dist];
                        [-1.0, 1.0]
                            .map(|sign| add(base, scale(n, sign * off)))
                            .to_vec()
                    }
                }
            }
        };
        found
            .into_iter()
            .filter(|&q| self.holds(q) && other.holds(q))
            .collect()
    }
}

/// The signed sweep of the arc from `s` to `e` through `p`, by the
/// inscribed angle theorem: counter-clockwise (positive) when `p` lies right
/// of the chord. Not finite when `p` is on the chord's line.
pub fn sweep_through(s: P2, e: P2, p: P2) -> f64 {
    let (a, b) = (sub(s, p), sub(e, p));
    let phi = cross(a, b).atan2(dot(a, b)).abs();
    let sign = if cross(sub(e, s), sub(p, s)) < 0.0 {
        1.0
    } else {
        -1.0
    };
    sign * (std::f64::consts::TAU - 2.0 * phi)
}

/// The point halfway along a polyline, by length: where a curve's label
/// goes.
pub fn polyline_mid(poly: &[P2]) -> P2 {
    let total: f64 = poly.windows(2).map(|w| dist(w[0], w[1])).sum();
    let mut along = 0.0;
    for w in poly.windows(2) {
        let d = dist(w[0], w[1]);
        if along + d >= total / 2.0 && d > 0.0 {
            return add(w[0], scale(sub(w[1], w[0]), (total / 2.0 - along) / d));
        }
        along += d;
    }
    poly.first().copied().unwrap_or([0.0, 0.0])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A half circle through the point above the chord's middle turns
    /// clockwise, one below it counter-clockwise.
    #[test]
    fn sweeps_through_a_point() {
        let s = sweep_through([-1.0, 0.0], [1.0, 0.0], [0.0, 1.0]);
        assert!((s + std::f64::consts::PI).abs() < 1e-12, "{s}");
        let s = sweep_through([-1.0, 0.0], [1.0, 0.0], [0.0, -1.0]);
        assert!((s - std::f64::consts::PI).abs() < 1e-12, "{s}");
    }

    #[test]
    fn polyline_middles() {
        assert_eq!(
            polyline_mid(&[[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]),
            [2.0, 0.0]
        );
    }
}
