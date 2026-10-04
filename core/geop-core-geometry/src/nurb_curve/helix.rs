//! Helices: [`NurbCurve3D::helix`], and the control rows it is built from
//! ([`helix_rows`]), which a screw sweep carries a whole profile along.
//!
//! A helix is not a rational curve: its height grows linearly with the angle
//! it has turned, and no rational parametrization of a circle turns at a
//! constant rate. So it is approximated, the same way every circle of the
//! kernel is built: one rational quadratic per span of at most a quarter
//! turn, its middle control point where the tangents of the circle meet,
//! weighted `cos(step / 2)`. That part is exact, so **the curve lies exactly
//! on its cylinder**: seen down the axis it is the kernel's own circle. Only
//! the height is approximate. It rises from span end to span end exactly as
//! a helix does, the middle control row sits at the height halfway between
//! (where the curve, by symmetry, passes at the span's middle angle, exactly
//! on the helix), and in between the height runs ahead of or behind the true
//! helix by at most **0.53 % of the pitch** for quarter-turn spans (`5.3e-3
//! * pitch`, measured over the span; see the test). Any other middle height
//! is worse: the error is odd about the middle of the span, and moving the
//! middle row adds an even term.
//!
//! For an ISO metric thread that is 5 µm on an M6 (pitch 1 mm), well inside
//! the tolerance of the thread itself.

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::CoordinateSystem,
    scalars::Scalar,
    vector::{Vector3, Vector4},
};

use super::{NurbCurve, NurbCurve3D};

/// Which way a helix turns as it rises: right-handed, like an ordinary
/// screw — counter-clockwise seen from where it rises to — or left-handed.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Handedness {
    #[default]
    Right,
    Left,
}

/// One control row of a helix around `frame.w()` through `frame.origin()`:
/// a point at distance `r` from the axis, raised by `z` above the row, lies
/// at `origin + r radial + (rise + z) w`, weighted `weight` — none for a row
/// at a span's end, whose weight is one.
#[derive(Clone, Debug)]
pub struct HelixRow<S: Scalar> {
    /// Unit length at a span's end; at a span's middle, as far out as the
    /// tangents of the unit circle at the span's ends meet.
    pub radial: Vector3<S>,
    pub rise: S,
    pub weight: Option<S>,
}

/// `(cos, sin)` of `degrees` — exact at every quarter turn, so that a
/// revolve through a right angle lands exactly on the plane it should, and
/// a helix's spans of a quarter turn sit on exactly the circle a revolve's
/// do.
pub fn cos_sin(degrees: f64) -> (f64, f64) {
    match degrees.rem_euclid(360.0) {
        0.0 => (1.0, 0.0),
        90.0 => (0.0, 1.0),
        180.0 => (-1.0, 0.0),
        270.0 => (0.0, -1.0),
        d => (d.to_radians().cos(), d.to_radians().sin()),
    }
}

/// The control rows of a helix around `frame.w()`, from `frame.u()`, rising
/// `pitch` along `w` per turn through `turns` turns, turning as
/// `handedness` says: `2 n + 1` rows for `n` equal spans of at most a
/// quarter turn — a span's ends at even indices, its middle in between.
pub fn helix_rows<S: Scalar>(
    frame: &CoordinateSystem<S>,
    pitch: S,
    turns: f64,
    handedness: Handedness,
) -> GeopResult<Vec<HelixRow<S>>> {
    if !(turns.is_finite() && turns > 0.0) {
        return Err(GeopError::new(format!(
            "helix: cannot turn {turns} times: it must be more than none"
        )));
    }
    if !pitch.definitely_greater(S::ZERO) {
        return Err(GeopError::new(format!(
            "helix: the pitch {pitch:?} must be definitely positive"
        )));
    }
    let spans = (turns * 4.0).ceil() as usize;
    let sign = match handedness {
        Handedness::Right => 1.0,
        Handedness::Left => -1.0,
    };
    let step = sign * 360.0 * turns / spans as f64;
    let (u, v) = (frame.u(), frame.v());
    let direction = |degrees: f64| {
        let (c, s) = cos_sin(degrees);
        u.prod_scalar(S::from_f64(c))
            .add(&v.prod_scalar(S::from_f64(s)))
    };
    // As a revolve weighs and places the middle of its arcs (see
    // `revolution` in geop-ops-extrude-revolve), so a quarter span lies on
    // the very circle a revolved quarter does.
    let (cos_step, _) = cos_sin(step);
    let weight = if step.abs() == 90.0 {
        S::from_f64(std::f64::consts::SQRT_2 / 2.0)
    } else {
        S::from_f64((step / 2.0).to_radians().cos())
    };
    let outwards = S::from_f64(1.0 / (1.0 + cos_step));
    let total = pitch.mul(S::from_f64(turns));
    // The height at the end of span `k`: none at the start and all of it at
    // the end, taken as they are rather than multiplied by 0 or 1.
    let rise = |k: usize| -> GeopResult<S> {
        Ok(match k {
            0 => S::ZERO,
            k if k == spans => total,
            k => total.mul(S::from_ratio(k as i64, spans as i64)?),
        })
    };
    let mut rows = Vec::with_capacity(2 * spans + 1);
    let mut start = HelixRow {
        radial: direction(0.0),
        rise: S::ZERO,
        weight: None,
    };
    for k in 0..spans {
        let end = HelixRow {
            radial: direction((k + 1) as f64 * step),
            rise: rise(k + 1)?,
            weight: None,
        };
        let middle = HelixRow {
            radial: start.radial.add(&end.radial).prod_scalar(outwards),
            rise: start.rise.add(end.rise).div(S::TWO)?,
            weight: Some(weight),
        };
        rows.push(start);
        rows.push(middle);
        start = end;
    }
    rows.push(start);
    Ok(rows)
}

impl<S: Scalar> NurbCurve3D<S> {
    /// The helix of `radius` around `frame.w()` through `frame.origin()`:
    /// starting at `origin + radius u`, rising `pitch` along `w` per turn,
    /// through `turns` turns — right-handed turning from `u` towards `v`,
    /// left-handed the other way. One rational quadratic per span of at
    /// most a quarter turn, exactly on the cylinder, its height within
    /// 0.53 % of the pitch of the true helix (see the module docs). The
    /// parameter runs over `[0, 1]`, a span's end at every `k / n`.
    pub fn helix(
        frame: &CoordinateSystem<S>,
        radius: S,
        pitch: S,
        turns: f64,
        handedness: Handedness,
    ) -> GeopResult<Self> {
        if !radius.definitely_greater(S::ZERO) {
            return Err(GeopError::new(format!(
                "helix: the radius {radius:?} must be definitely positive"
            )));
        }
        let rows = helix_rows(frame, pitch, turns, handedness)?;
        let spans = rows.len() / 2;
        let (origin, w) = (frame.origin(), frame.w());
        let control_points = rows
            .iter()
            .map(|row| {
                let p = origin
                    .add(&row.radial.prod_scalar(radius))
                    .add(&w.prod_scalar(row.rise));
                match row.weight {
                    None => Vector4::from_array([p[0], p[1], p[2], S::ONE]),
                    Some(h) => Vector4::from_array([p[0].mul(h), p[1].mul(h), p[2].mul(h), h]),
                }
            })
            .collect();
        let mut knots = vec![S::ZERO; 3];
        for k in 1..spans {
            let knot = S::from_ratio(k as i64, spans as i64)?;
            knots.extend([knot, knot]);
        }
        knots.extend([S::ONE; 3]);
        NurbCurve::try_new(2, control_points, knots)
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        for_all_scalars, primitives::CoordinateSystem, scalars::Scalar, vector::Vector3,
    };

    use super::*;

    fn frame<S: Scalar>() -> CoordinateSystem<S> {
        CoordinateSystem::world_at(Vector3::from_array([1.0, 2.0, 3.0].map(S::from_f64)))
    }

    /// Every point of the helix lies on its cylinder, it ends where it
    /// should, and its height stays within 0.53 % of the pitch of the true
    /// helix's at the same angle — turning the right way round.
    fn check_helix_is_on_its_cylinder_and_close_to_the_true_helix<S: Scalar>() {
        let (radius, pitch, turns) = (2.5, 0.8, 2.3);
        for handedness in [Handedness::Right, Handedness::Left] {
            let curve = NurbCurve3D::helix(
                &frame::<S>(),
                S::from_f64(radius),
                S::from_f64(pitch),
                turns,
                handedness,
            )
            .unwrap();
            let sign = if handedness == Handedness::Right {
                1.0
            } else {
                -1.0
            };
            let samples = 997;
            let mut unwrapped = 0.0f64;
            let mut last = 0.0f64;
            let mut worst = 0.0f64;
            for i in 0..=samples {
                let t = S::from_ratio(i, samples).unwrap();
                let p = curve.evaluate(t).unwrap();
                let centre = Vector3::from_array([1.0, 2.0, p[2].to_f64()].map(S::from_f64));
                let off = p.sub(&centre);
                let r2 = off[0].mul(off[0]).add(off[1].mul(off[1]));
                assert!(
                    r2.could_be_equal(S::from_f64(radius * radius)),
                    "{p:?} is off the cylinder"
                );
                let (x, y, z) = (
                    p[0].to_f64() - 1.0,
                    p[1].to_f64() - 2.0,
                    p[2].to_f64() - 3.0,
                );
                let angle = sign * y.atan2(x);
                let mut d = angle - last;
                if d < -std::f64::consts::PI {
                    d += 2.0 * std::f64::consts::PI;
                }
                if d > std::f64::consts::PI {
                    d -= 2.0 * std::f64::consts::PI;
                }
                // It never turns back.
                assert!(d >= -1e-12, "{i}: {d}");
                unwrapped += d;
                last = angle;
                let expected = pitch * unwrapped / (2.0 * std::f64::consts::PI);
                worst = worst.max((z - expected).abs());
            }
            assert!(
                (unwrapped - turns * 2.0 * std::f64::consts::PI).abs() < 1e-9,
                "{unwrapped}"
            );
            assert!(worst <= 5.3e-3 * pitch, "{worst}");
            // Not merely a circle lifted: it does rise with the angle.
            assert!(worst > 1e-4 * pitch, "{worst}");
            let end = curve.evaluate(S::ONE).unwrap();
            assert!(
                end[2].could_be_equal(
                    S::from_f64(3.0).add(S::from_f64(pitch).mul(S::from_f64(turns)))
                ),
                "{end:?}"
            );
        }
    }
    #[test]
    fn helix_is_on_its_cylinder_and_close_to_the_true_helix() {
        for_all_scalars!(check_helix_is_on_its_cylinder_and_close_to_the_true_helix);
    }

    /// Nothing to turn, no pitch, no radius: refused.
    #[test]
    fn degenerate_helices_are_refused() {
        use geop_core_math::scalars::ScalInF64 as S;
        let f = frame::<S>();
        let one = S::from_f64(1.0);
        assert!(NurbCurve3D::helix(&f, one, one, 0.0, Handedness::Right).is_err());
        assert!(NurbCurve3D::helix(&f, one, S::ZERO, 1.0, Handedness::Right).is_err());
        assert!(NurbCurve3D::helix(&f, S::ZERO, one, 1.0, Handedness::Right).is_err());
    }
}
