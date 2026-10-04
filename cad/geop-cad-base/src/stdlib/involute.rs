//! The gap between two teeth of an involute spur gear, of module 1: its
//! sides Bézier curves that follow the involute of the base circle to
//! within a bound computed with them, [`Gap::deviation`].
//!
//! The involute of a circle of radius `rb`, unwound from the point
//! `(rb, 0)`, is `rb · R(t) · (1, -t)` for `t ≥ 0`, with `R(t)` the turn
//! by `t` — its point at `t` lies `rb · sqrt(1 + t²)` from the centre, and
//! its tangent there is `R(t) · (1, 0)`. No polynomial is that curve, so a
//! flank is the Taylor polynomial of degree [`DEGREE`] of it around where
//! it starts, written as a Bézier curve, its two ends put back onto the
//! involute exactly: the deviation is bounded by the series' tail and by
//! how far the ends moved — a bound, not a sample.
//!
//! The gear is standard (no profile shift), [`PRESSURE_ANGLE`]: of `z`
//! teeth, pitch radius `z / 2`, base radius `z / 2 · cos 20°`, tip radius
//! `z / 2 + 1` and root radius `z / 2 - 1.25`, its teeth `π / 2` thick on
//! the pitch circle, one of them centred on `+x`.
//!
//! A flank is the involute wherever a standard gear or rack meshing with
//! it can touch it: from `z / 2 - 1` — the tip of the mate's tooth reaches
//! no deeper — or from a little above the base circle, if that is higher,
//! where the involute's parametrization turns singular. Below that a real
//! flank turns into the root fillet its cutter leaves; here it runs on
//! straight down along its tangent to the root circle. The flank goes on
//! past the tip circle, and an arc there closes the gap: cut from a disc
//! of the tip diameter, it leaves the teeth, the disc's rim crossing the
//! flanks rather than meeting a corner of the gap.

use std::f64::consts::PI;

use geop_core_math::geop_error::{GeopError, GeopResult};

/// The pressure angle, in degrees.
pub const PRESSURE_ANGLE: f64 = 20.0;

/// The degree of a flank's Bézier curve: [`DEGREE`] + 1 control points.
pub const DEGREE: usize = 9;

/// The fewest teeth a gear has: with fewer, the tip of a tooth gets
/// narrow, and its flanks undercut where a mating rack's would.
pub const FEWEST_TEETH: usize = 12;

/// How far past the tip circle a gap's flanks go on, in modules.
const PAST_TIP: f64 = 0.5;

/// The gap between two teeth of a gear of module 1, centred on `+x`: its
/// lower side, where `y < 0` — the upper one is its mirror image in the
/// `x` axis.
#[derive(Clone, Debug)]
pub struct Gap {
    /// Where the side starts, on the root circle.
    pub foot: [f64; 2],
    /// The control points of the flank's Bézier curve, from where the
    /// straight line up from the foot meets it to [`PAST_TIP`] past the
    /// tip circle — where the arc closing the gap joins it to its mirror
    /// image.
    pub flank: [[f64; 2]; DEGREE + 1],
    /// How far, at most, the flank is from the involute: the tail of the
    /// Taylor series and how far its ends moved, in modules.
    pub deviation: f64,
}

/// The involute function `tan a - a`: how far round from where it was
/// unwound the involute is where its pressure angle is `a`.
fn inv(a: f64) -> f64 {
    a.tan() - a
}

fn turn(a: f64, [x, y]: [f64; 2]) -> [f64; 2] {
    let (s, c) = a.sin_cos();
    [c * x - s * y, s * x + c * y]
}

fn binomial(n: usize, k: usize) -> f64 {
    (0..k).fold(1.0, |b, i| b * (n - i) as f64 / (i + 1) as f64)
}

fn factorial(k: usize) -> f64 {
    (1..=k).fold(1.0, |f, i| f * i as f64)
}

/// The gap of a gear of `z` teeth.
pub fn gap(z: usize) -> GeopResult<Gap> {
    if z < FEWEST_TEETH {
        return Err(GeopError::new(format!(
            "a spur gear of {z} teeth is not supported: it needs at least {FEWEST_TEETH}"
        )));
    }
    let alpha = PRESSURE_ANGLE.to_radians();
    let zf = z as f64;
    let pitch = zf / 2.0;
    let rb = pitch * alpha.cos();
    let (tip, root) = (pitch + 1.0, pitch - 1.25);
    // The involute's parameter at the tip, where the flank ends past it,
    // and where it starts (see the module doc): a tenth of the way up to
    // the tip keeps clear of its cusp on the base circle, a free choice.
    let at = |r: f64| ((r / rb).powi(2) - 1.0).max(0.0).sqrt();
    let t_tip = at(tip);
    let t_end = at(tip + PAST_TIP);
    let t0 = at(pitch - 1.0).max(t_tip / 10.0);
    let span = t_end - t0;
    // Built as the lower flank of the tooth centred on `+x`: the involute
    // turned so that it crosses the pitch circle at `-π / (2z)`.
    let phi = -(PI / (2.0 * zf) + inv(alpha));
    if phi + t_tip - t_tip.atan() >= 0.0 {
        return Err(GeopError::new(format!(
            "the teeth of a spur gear of {z} teeth come to a point below the tip circle"
        )));
    }
    let involute = |t: f64| turn(phi + t, [rb, -rb * t]);
    // Around `t0`, with `t = t0 + τ`: `R(t0) · R(τ) · (1, -t0 - τ)`, whose
    // Taylor coefficients in `τ` are those of `cos τ + (t0 + τ) sin τ` and
    // `sin τ - (t0 + τ) cos τ`.
    let cos = |k: usize| {
        (match k % 4 {
            0 => 1.0,
            2 => -1.0,
            _ => 0.0,
        }) / factorial(k)
    };
    let sin = |k: usize| {
        (match k % 4 {
            1 => 1.0,
            3 => -1.0,
            _ => 0.0,
        }) / factorial(k)
    };
    let below = |f: &dyn Fn(usize) -> f64, k: usize| if k == 0 { 0.0 } else { f(k - 1) };
    let coefficient = |k: usize| {
        [
            cos(k) + t0 * sin(k) + below(&sin, k),
            sin(k) - t0 * cos(k) - below(&cos, k),
        ]
    };
    // In `s = τ / span` on `[0, 1]`, then in Bernstein's basis.
    let power: Vec<[f64; 2]> = (0..=DEGREE)
        .map(|k| coefficient(k).map(|c| c * span.powi(k as i32)))
        .collect();
    let mut flank = [[0.0; 2]; DEGREE + 1];
    for (j, point) in flank.iter_mut().enumerate() {
        let mut b = [0.0; 2];
        for (k, a) in power.iter().enumerate().take(j + 1) {
            let w = binomial(j, k) / binomial(DEGREE, k);
            b[0] += w * a[0];
            b[1] += w * a[1];
        }
        *point = turn(phi + t0, b).map(|c| rb * c);
    }
    // The ends exactly on the involute: each moves the curve by at most as
    // far as it moves, a Bernstein weight being at most 1.
    let distance = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]);
    let (start, top) = (involute(t0), involute(t_end));
    let moved = distance(flank[0], start).max(distance(flank[DEGREE], top));
    flank[0] = start;
    flank[DEGREE] = top;
    // The tail: the coefficient of `τ^k` is at most `(1 + t0 + k) / k!`
    // in each coordinate, the terms shrinking from the first one left out.
    let tail: f64 = (DEGREE + 1..DEGREE + 40)
        .map(|k| (1.0 + t0 + k as f64) * span.powi(k as i32) / factorial(k))
        .sum();
    // And the rounding of a Bernstein sum: a few units in the last place
    // of the tip radius per term.
    let rounding = 4.0 * (DEGREE + 1) as f64 * f64::EPSILON * tip;
    let deviation = rb * 2f64.sqrt() * tail + moved + rounding;

    // Down the tangent at `start` to the root circle: `start - s · d` at
    // the distance `root` from the centre, the nearer of the two.
    let along = |t: f64| turn(phi + t, [1.0, 0.0]);
    let d = along(t0);
    let reach = start[0] * d[0] + start[1] * d[1];
    let s = reach - (reach * reach - (start[0].powi(2) + start[1].powi(2) - root * root)).sqrt();
    let foot = [start[0] - s * d[0], start[1] - s * d[1]];

    // That was a tooth's lower flank; its mirror image is the upper flank,
    // turned back half a tooth pitch the lower side of the gap on `+x`.
    let side = |[x, y]: [f64; 2]| turn(-PI / zf, [x, -y]);
    Ok(Gap {
        foot: side(foot),
        flank: flank.map(side),
        deviation,
    })
}

/// The point of the Bézier curve of `control` at `s` in `[0, 1]`.
#[cfg(test)]
fn bezier(control: &[[f64; 2]], s: f64) -> [f64; 2] {
    let mut points = control.to_vec();
    for level in 1..points.len() {
        for i in 0..points.len() - level {
            for k in 0..2 {
                points[i][k] = (1.0 - s) * points[i][k] + s * points[i + 1][k];
            }
        }
    }
    points[0]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every flank keeps within its stated deviation of the involute, and
    /// the deviation is far below anything a gear is made to; the gap
    /// starts on the root circle, ends past the tip circle, and stays
    /// on its side of `x`.
    #[test]
    fn flanks_follow_the_involute_within_their_deviation() {
        let alpha = PRESSURE_ANGLE.to_radians();
        for z in [12, 13, 17, 20, 33, 34, 41, 42, 60, 120, 200] {
            let gap = gap(z).unwrap();
            let zf = z as f64;
            let rb = zf / 2.0 * alpha.cos();
            assert!(gap.deviation < 1e-5, "z {z}: {}", gap.deviation);
            for i in 0..=100 {
                let p = bezier(&gap.flank, i as f64 / 100.0);
                // Where the involute is at the same distance from the
                // centre: the upper flank of the tooth below `x`.
                let r = p[0].hypot(p[1]);
                let pressure = (rb / r).acos();
                let want = -PI / (2.0 * zf) + inv(alpha) - inv(pressure);
                // Along the circle — across the flank, up to its slope.
                let off = (p[1].atan2(p[0]) - want).abs() * r * pressure.cos();
                assert!(
                    // The check itself rounds, by a few units in the last
                    // place of the radius.
                    off <= gap.deviation + 8.0 * f64::EPSILON * r,
                    "z {z} at {i}: {off} > {}",
                    gap.deviation
                );
                assert!(p[1] < 0.0, "z {z} at {i}: {p:?} crosses x");
            }
            let radius = |p: [f64; 2]| p[0].hypot(p[1]);
            assert!((radius(gap.foot) - (zf / 2.0 - 1.25)).abs() < 1e-12);
            assert!((radius(gap.flank[DEGREE]) - (zf / 2.0 + 1.0 + PAST_TIP)).abs() < 1e-12);
            assert!(gap.flank[DEGREE][1] < 0.0 && gap.foot[1] < 0.0, "z {z}");
            // The line from the foot rises to where the involute starts:
            // at the deepest a mate's tooth reaches, or higher.
            assert!(radius(gap.flank[0]) >= zf / 2.0 - 1.0 - 1e-12, "z {z}");
        }
    }

    #[test]
    fn too_few_teeth_are_refused() {
        let e = gap(8).unwrap_err().to_string();
        assert!(e.contains("at least 12"), "{e}");
    }
}
