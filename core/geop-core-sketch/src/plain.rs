//! Plane geometry in plain numbers: where a tool places what it builds, and
//! where the pointer meets the sketch. Only ever a free choice — a position
//! the solver then makes exact, a suggestion of which piece of a curve is
//! meant — never a result the kernel reasons about: what the sketch *is* is
//! its design data, and what that means comes from its solver.

use std::f64::consts::{PI, TAU};

use geop_core_math::scalars::Scalar;

use crate::sketch::{CurveId, CurveKind, PointId, Sketch};

/// `[x, y]`.
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

/// `v` of length one; none for no direction at all.
pub fn unit(v: P2) -> Option<P2> {
    let n = v[0].hypot(v[1]);
    (n > 0.0).then(|| scale(v, 1.0 / n))
}

/// `v` turned a quarter counter-clockwise.
pub fn perp(v: P2) -> P2 {
    [-v[1], v[0]]
}

/// `v` turned counter-clockwise by `angle`.
pub fn rotate(v: P2, angle: f64) -> P2 {
    let (s, c) = angle.sin_cos();
    [v[0] * c - v[1] * s, v[0] * s + v[1] * c]
}

/// `angle` brought into `(-π, π]`.
pub fn wrap(angle: f64) -> f64 {
    let a = (angle + PI).rem_euclid(TAU) - PI;
    if a == -PI { PI } else { a }
}

/// The counter-clockwise angle from `a` to `b`, in `(-π, π]`.
pub fn angle_between(a: P2, b: P2) -> f64 {
    cross(a, b).atan2(dot(a, b))
}

/// `p` mirrored across the line through `a` and `b`.
pub fn reflect(p: P2, a: P2, b: P2) -> P2 {
    let Some(u) = unit(sub(b, a)) else {
        return p;
    };
    let q = sub(p, a);
    let along = scale(u, dot(q, u));
    add(a, sub(scale(along, 2.0), q))
}

/// Where the sketch point `p` is drawn.
pub fn xy<S: Scalar>(sketch: &Sketch<S>, p: PointId) -> P2 {
    let q = sketch.points[&p].xy();
    [q[0].to_f64(), q[1].to_f64()]
}

/// Where the segments from `a` to `b` and from `c` to `d` cross, if they
/// do.
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
    sign * (TAU - 2.0 * phi)
}

/// A line, circle or arc in plain numbers: what intersections are found
/// on, and where along it a point is.
#[derive(Clone, Copy, Debug, PartialEq)]
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
    /// `curve` of `sketch` as drawn, `infinite` if a line; none for a
    /// spline, or an arc too straight to have a center.
    pub fn of<S: Scalar>(sketch: &Sketch<S>, curve: CurveId, infinite: bool) -> Option<Plain> {
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
            CurveKind::Arc { start, end, sweep } => Plain::arc(p(start), p(end), sweep.to_f64())?,
            CurveKind::Spline { .. } => return None,
        })
    }

    /// The arc from `s` to `e` turning by `sweep`; none if too straight to
    /// have a center.
    pub fn arc(s: P2, e: P2, sweep: f64) -> Option<Plain> {
        // As `crate::geometry::Arc::center`: the chord's middle, moved to
        // its left by `(L / 2) cot(half)`.
        let chord = sub(e, s);
        let length = chord[0].hypot(chord[1]);
        let half = sweep / 2.0;
        let left = perp(unit(chord)?);
        let center = add(
            scale(add(s, e), 0.5),
            scale(left, length / 2.0 * half.cos() / half.sin()),
        );
        let radius = length / (2.0 * half.sin().abs());
        let from = sub(s, center);
        (radius.is_finite() && center.iter().all(|v| v.is_finite())).then_some(Plain::Round {
            center,
            radius,
            span: Some((from[1].atan2(from[0]), sweep)),
        })
    }

    /// The whole line or circle it lies on.
    pub fn full(self) -> Plain {
        match self {
            Plain::Line { a, b, .. } => Plain::Line {
                a,
                b,
                infinite: true,
            },
            Plain::Round { center, radius, .. } => Plain::Round {
                center,
                radius,
                span: None,
            },
        }
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

    /// Its unit direction of travel at `t` along it (see [`Plain::param`]).
    pub fn tangent(&self, t: f64) -> P2 {
        match *self {
            Plain::Line { a, b, .. } => unit(sub(b, a)).unwrap_or([1.0, 0.0]),
            Plain::Round { span, .. } => {
                let (angle, turn) = match span {
                    None => (t * TAU, 1.0),
                    Some((start, sweep)) => (start + t * sweep, sweep.signum()),
                };
                scale([-angle.sin(), angle.cos()], turn)
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A half circle through the point above the chord's middle turns
    /// clockwise, one below it counter-clockwise.
    #[test]
    fn sweeps_through_a_point() {
        let s = sweep_through([-1.0, 0.0], [1.0, 0.0], [0.0, 1.0]);
        assert!((s + PI).abs() < 1e-12, "{s}");
        let s = sweep_through([-1.0, 0.0], [1.0, 0.0], [0.0, -1.0]);
        assert!((s - PI).abs() < 1e-12, "{s}");
    }

    #[test]
    fn reflections_and_tangents() {
        let r = reflect([1.0, 2.0], [0.0, 0.0], [1.0, 1.0]);
        assert!(dist(r, [2.0, 1.0]) < 1e-12, "{r:?}");
        // A quarter circle counter-clockwise from (1, 0): leaving upwards,
        // arriving leftwards.
        let q = Plain::arc([1.0, 0.0], [0.0, 1.0], PI / 2.0).unwrap();
        assert!(dist(q.tangent(0.0), [0.0, 1.0]) < 1e-12);
        assert!(dist(q.tangent(1.0), [-1.0, 0.0]) < 1e-12);
    }
}
