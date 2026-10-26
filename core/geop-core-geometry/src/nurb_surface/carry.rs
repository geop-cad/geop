//! One surface carried on over another: whether a patch lies on another
//! patch's surface, and where ([`NurbSurface3D::carrying`]); a patch
//! carried on until it reaches given points ([`NurbSurface3D::covering`]).

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::{Vector2, Vector3},
};

use super::NurbSurface3D;
use crate::{nurb_curve::NurbCurve2D, search::PROJECT_ITERATIONS};

/// How often [`NurbSurface3D::covering`] carries a surface on, at most: each
/// time by as much as its domain is long, so it grows past any reach in a
/// few.
const MOST_COVERINGS: usize = 8;

/// An affine map of `(u, v)` that takes each parameter to one parameter:
/// `x' = m x + shift`, `m` diagonal, or — the parameters swapped —
/// anti-diagonal.
#[derive(Clone, Copy, Debug)]
pub struct UvMap {
    pub m: [[Scalar; 2]; 2],
    pub shift: Vector2,
}

impl UvMap {
    /// Where `uv` goes.
    pub fn apply(&self, uv: Vector2) -> Vector2 {
        Vector2::from_array(std::array::from_fn(|k| {
            self.m[k][0]
                .mul(uv[0])
                .add(self.m[k][1].mul(uv[1]))
                .add(self.shift[k])
        }))
    }

    /// Which parameter the `k`-th maps to.
    fn image(&self, k: usize) -> usize {
        if self.m[0][k].could_be_equal(Scalar::ZERO) {
            1
        } else {
            0
        }
    }

    /// The pcurve `curve` mapped: each homogeneous control point `(w x, w)`
    /// to `(m (w x) + w shift, w)`, linear in it, so the mapped curve is
    /// exactly the curve mapped, rational or not.
    pub fn apply_to(&self, curve: &NurbCurve2D) -> GeopResult<NurbCurve2D> {
        let points = (curve.control_points.iter())
            .map(|cp| {
                let w = cp[2];
                let mut mapped = *cp;
                for k in 0..2 {
                    mapped[k] = self.m[k][0]
                        .mul(cp[0])
                        .add(self.m[k][1].mul(cp[1]))
                        .add(self.shift[k].mul(w));
                }
                mapped
            })
            .collect();
        NurbCurve2D::try_new(curve.degree, points, curve.knot_vector.clone())
    }
}

/// Where `point` is on `surface`, from the middle of its domain: its foot
/// point, if it lies on the surface there.
fn on(surface: &NurbSurface3D, point: Vector3) -> GeopResult<Option<Vector2>> {
    let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
    let half = Scalar::from_f64(0.5);
    let (u, v) = surface.project_in_domain(
        point,
        Scalar::interpolate(u0, u1, half).sharpen(),
        Scalar::interpolate(v0, v1, half).sharpen(),
        PROJECT_ITERATIONS,
    )?;
    Ok((surface.evaluate(u, v)?.could_be_equal(&point)).then(|| Vector2::from_array([u, v])))
}

impl NurbSurface3D {
    /// This surface carried on past its sides ([`NurbSurface3D::extended`])
    /// until the foot point of each of `points` on it lies inside its
    /// domain, not on a side: a surface something is to lie on has to reach
    /// it. Carried on past a side each time by as much as the domain is
    /// long that way — how much is a free choice — and refused, naming the
    /// points, where it cannot be carried on far enough.
    pub fn covering(&self, points: &[Vector3]) -> GeopResult<Self> {
        let mut surface = self.clone();
        for _ in 0..MOST_COVERINGS {
            let mut sides = [[false; 2]; 2];
            for point in points {
                let ((u0, u1), (v0, v1)) = (surface.domain_u(), surface.domain_v());
                let half = Scalar::from_f64(0.5);
                let (u, v) = surface.project_in_domain(
                    *point,
                    Scalar::interpolate(u0, u1, half).sharpen(),
                    Scalar::interpolate(v0, v1, half).sharpen(),
                    PROJECT_ITERATIONS,
                )?;
                for (k, (t, lo, hi)) in [(u, u0, u1), (v, v0, v1)].into_iter().enumerate() {
                    sides[k][0] |= t.could_be_equal(lo);
                    sides[k][1] |= t.could_be_equal(hi);
                }
            }
            if sides.iter().flatten().all(|&side| !side) {
                return Ok(surface);
            }
            for (k, ends) in sides.iter().enumerate() {
                for (at_end, &short) in ends.iter().enumerate() {
                    if !short {
                        continue;
                    }
                    let (lo, hi) = if k == 0 {
                        surface.domain_u()
                    } else {
                        surface.domain_v()
                    };
                    surface = surface.extended(k == 0, at_end == 1, hi.sub(lo).sharpen())?;
                }
            }
        }
        Err(GeopError::new(format!(
            "the surface, carried on {MOST_COVERINGS} times, does not reach the points {points:?}"
        )))
    }

    /// Where the patch `other` lies on this surface, if it does: this
    /// surface carried on until it covers `other` ([`Self::covering`]), and
    /// the map from `other`'s `(u, v)` to its own — the very same surface,
    /// each parameter of `other` one of its own shifted and scaled, the way
    /// patches of one surface cut apart, mirrored or built again are, its
    /// normal the same way. `None` where `other` is another surface, faces
    /// the other way, or is parametrized otherwise.
    ///
    /// The map comes from where `other`'s corners are on this surface. That
    /// it holds everywhere is a polynomial identity: on each cell between
    /// the two patches' knots, both are rational of the same degree `p` in
    /// each parameter, and two such that agree at `2p + 1` parameters in
    /// each direction agree everywhere — their difference, cleared of
    /// denominators, is a polynomial of degree `2p` with more roots than
    /// its degree. So they are compared there, and only there; within the
    /// enclosures' widths, as everything is.
    pub fn carrying(&self, other: &Self) -> GeopResult<Option<(Self, UvMap)>> {
        let ((u0, u1), (v0, v1)) = (other.domain_u(), other.domain_v());
        let corners = [
            other.evaluate(u0, v0)?,
            other.evaluate(u1, v0)?,
            other.evaluate(u0, v1)?,
            other.evaluate(u1, v1)?,
        ];
        let Ok(carried) = self.covering(&corners) else {
            return Ok(None);
        };
        let mut at = Vec::with_capacity(3);
        for &corner in &corners[..3] {
            match on(&carried, corner)? {
                Some(uv) => at.push(uv),
                None => return Ok(None),
            }
        }
        let (p00, p10, p01) = (at[0], at[1], at[2]);
        let (du, dv) = (p10.sub(&p00), p01.sub(&p00));
        let zero = |s: Scalar| s.could_be_equal(Scalar::ZERO);
        let swapped = if zero(du[1]) && zero(dv[0]) {
            false
        } else if zero(du[0]) && zero(dv[1]) {
            true
        } else {
            return Ok(None);
        };
        let (su, sv) = (
            du.prod_scalar(Scalar::ONE.div(u1.sub(u0))?),
            dv.prod_scalar(Scalar::ONE.div(v1.sub(v0))?),
        );
        let m = if swapped {
            [[Scalar::ZERO, sv[0]], [su[1], Scalar::ZERO]]
        } else {
            [[su[0], Scalar::ZERO], [Scalar::ZERO, sv[1]]]
        };
        // The same way round: the normal of this surface along the map is
        // its own times the map's determinant.
        let det = m[0][0].mul(m[1][1]).sub(m[0][1].mul(m[1][0]));
        if !det.definitely_greater(Scalar::ZERO) {
            return Ok(None);
        }
        let degrees = if swapped {
            (self.degree_v, self.degree_u)
        } else {
            (self.degree_u, self.degree_v)
        };
        if degrees != (other.degree_u, other.degree_v) {
            return Ok(None);
        }
        let shift = Vector2::from_array(std::array::from_fn(|k| {
            p00[k].sub(m[k][0].mul(u0)).sub(m[k][1].mul(v0))
        }));
        let map = UvMap { m, shift };
        // The cells: between `other`'s knots, and the carried surface's
        // taken back into `other`'s parameters.
        let cells = |k: usize| -> GeopResult<Vec<Scalar>> {
            let own = if k == 0 {
                other.breakpoints_u()
            } else {
                other.breakpoints_v()
            };
            let image = map.image(k);
            let theirs = if image == 0 {
                carried.breakpoints_u()
            } else {
                carried.breakpoints_v()
            };
            let (lo, hi) = (own[0], *own.last().expect("a domain"));
            let mut breaks = own;
            for b in theirs {
                let back = b.sub(map.shift[image]).div(map.m[image][k])?;
                if back.definitely_greater(lo) && back.definitely_less(hi) {
                    breaks.push(back);
                }
            }
            breaks.sort_by(|a, b| a.to_f64().total_cmp(&b.to_f64()));
            Ok(breaks)
        };
        let samples = |breaks: &[Scalar], degree: usize| -> GeopResult<Vec<Scalar>> {
            let n = 2 * degree + 1;
            let mut at = Vec::new();
            for pair in breaks.windows(2) {
                for i in 0..=n {
                    let f = Scalar::from_ratio(i as i64, n as i64)?;
                    at.push(Scalar::interpolate(pair[0], pair[1], f));
                }
            }
            Ok(at)
        };
        let (us, vs) = (cells(0)?, cells(1)?);
        for &u in &samples(&us, other.degree_u)? {
            for &v in &samples(&vs, other.degree_v)? {
                let mapped = map.apply(Vector2::from_array([u, v]));
                let theirs = other.evaluate(u, v)?;
                let ours = carried.evaluate(mapped[0], mapped[1])?;
                if !theirs.could_be_equal(&ours) {
                    return Ok(None);
                }
            }
        }
        Ok(Some((carried, map)))
    }
}

#[cfg(test)]
mod tests {
    use geop_core_math::{
        scalars::Scalar,
        vector::{Vector2, Vector3},
    };

    use crate::nurb_surface::NurbSurface3D;

    fn p(x: f64, y: f64, z: f64) -> Vector3 {
        Vector3::from_array([x, y, z].map(Scalar::from_f64))
    }

    fn square(x0: f64, x1: f64, z: f64) -> NurbSurface3D {
        NurbSurface3D::bilinear(p(x0, 0., z), p(x1, 0., z), p(x1, 1., z), p(x0, 1., z)).unwrap()
    }

    /// Two squares side by side in one plane, parametrized alike: the
    /// second lies on the first carried on, one further along `u`.
    #[test]
    fn a_square_beside_another_lies_on_it() {
        let (_, map) = square(0., 1., 1.)
            .carrying(&square(1., 3., 1.))
            .unwrap()
            .expect("one plane");
        let corner = map.apply(Vector2::from_array([Scalar::ONE, Scalar::ONE]));
        assert!(
            corner[0].could_be_equal(Scalar::from_f64(3.0)),
            "{corner:?}"
        );
    }

    /// Parametrized the other way round in both directions, as a mirrored
    /// copy may be, it still lies on it; facing the other way, it does not.
    #[test]
    fn a_square_turned_round_lies_on_it_facing_alike() {
        let turned =
            NurbSurface3D::bilinear(p(3., 1., 1.), p(1., 1., 1.), p(1., 0., 1.), p(3., 0., 1.))
                .unwrap();
        assert!(square(0., 1., 1.).carrying(&turned).unwrap().is_some());
        let facing_away = turned.reverse_u();
        assert!(square(0., 1., 1.).carrying(&facing_away).unwrap().is_none());
    }

    /// A square a hair above the plane of another is another surface.
    #[test]
    fn a_square_above_another_does_not() {
        assert!(
            square(0., 1., 1.)
                .carrying(&square(1., 3., 1.001))
                .unwrap()
                .is_none()
        );
    }
}
