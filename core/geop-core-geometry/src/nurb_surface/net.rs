//! Polynomials over one span of a surface, in Bernstein form: what a
//! surface's normal field is made of, as an explicit polynomial, so that a
//! pole's factor can be divided out of it exactly.
//!
//! A rational patch `S = A / w` has `S_u × S_v = M / w³` with
//! `M = w (A_u × A_v) − w_u (A × A_v) − w_v (A_u × A)`, a polynomial patch
//! whose direction is the normal's. Where a row of control points collapses
//! to a pole, `M` vanishes along that side, as `S_u × S_v` does, and the
//! normal there is the limit `0 / 0`. In Bernstein form the vanishing side
//! is a row of zero coefficients and the factor `t` (or `1 − t`) divides
//! out exactly ([`Net::divided`]), which leaves a polynomial with the
//! normal's direction and no zero at the pole.

use geop_core_math::{
    combinatorics::binomial,
    geop_error::GeopResult,
    scalars::Scalar,
    vector::{Vector, Vector3},
};

use super::NurbSurface3D;

/// A side of a patch: the row at the start or the end of `u` or of `v`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Side {
    /// Whether `u` is fixed along it (the side is a row along `v`).
    pub u_fixed: bool,
    pub at_end: bool,
}

impl Side {
    pub const ALL: [Side; 4] = [
        Side {
            u_fixed: true,
            at_end: false,
        },
        Side {
            u_fixed: true,
            at_end: true,
        },
        Side {
            u_fixed: false,
            at_end: false,
        },
        Side {
            u_fixed: false,
            at_end: true,
        },
    ];
}

/// A tensor-product polynomial in Bernstein form over the box `u` × `v`:
/// `c[i][j]` the coefficient of `B_i(u) B_j(v)`, of degree one less than
/// the counts.
#[derive(Clone, Debug)]
pub(crate) struct Net<const D: usize> {
    pub c: Vec<Vec<Vector<D>>>,
    pub u: (Scalar, Scalar),
    pub v: (Scalar, Scalar),
}

impl<const D: usize> Net<D> {
    fn degrees(&self) -> (usize, usize) {
        (self.c.len() - 1, self.c[0].len() - 1)
    }

    /// The derivative along `u` (`along_u`) or `v`: the scaled differences
    /// of the coefficients, a degree less.
    pub fn derivative(&self, along_u: bool) -> GeopResult<Self> {
        let (p, q) = self.degrees();
        let (lo, hi) = if along_u { self.u } else { self.v };
        let n = if along_u { p } else { q };
        if n == 0 {
            let zero = vec![vec![Vector::zero(); q + 1]; p + 1];
            return Ok(Net {
                c: zero,
                ..self.clone()
            });
        }
        let scale = Scalar::from_i64(n as i64).div(hi.sub(lo))?;
        let c = if along_u {
            (0..p)
                .map(|i| {
                    (0..=q)
                        .map(|j| self.c[i + 1][j].sub(&self.c[i][j]).prod_scalar(scale))
                        .collect()
                })
                .collect()
        } else {
            (0..=p)
                .map(|i| {
                    (0..q)
                        .map(|j| self.c[i][j + 1].sub(&self.c[i][j]).prod_scalar(scale))
                        .collect()
                })
                .collect()
        };
        Ok(Net { c, ..self.clone() })
    }

    /// The product of `self` and `other`, coefficient by coefficient
    /// through `f`, which has to be bilinear (a product, a cross product):
    /// `B_i^m B_k^n = C(m,i) C(n,k) / C(m+n,i+k) B_{i+k}^{m+n}` in each
    /// direction.
    pub fn product<const E: usize, const F: usize>(
        &self,
        other: &Net<E>,
        f: impl Fn(&Vector<D>, &Vector<E>) -> Vector<F>,
    ) -> GeopResult<Net<F>> {
        let ((p, q), (r, s)) = (self.degrees(), other.degrees());
        let mut c = vec![vec![Vector::<F>::zero(); q + s + 1]; p + r + 1];
        let weight = |m: usize, i: usize, n: usize, k: usize| -> GeopResult<Scalar> {
            Scalar::from_i64(binomial(m, i) * binomial(n, k))
                .div(Scalar::from_i64(binomial(m + n, i + k)))
        };
        for i in 0..=p {
            for k in 0..=r {
                let wu = weight(p, i, r, k)?;
                for j in 0..=q {
                    for l in 0..=s {
                        let term = f(&self.c[i][j], &other.c[k][l])
                            .prod_scalar(wu.mul(weight(q, j, s, l)?));
                        c[i + k][j + l] = c[i + k][j + l].add(&term);
                    }
                }
            }
        }
        Ok(Net {
            c,
            u: self.u,
            v: self.v,
        })
    }

    /// `self − other`, of the same degrees.
    pub fn minus(&self, other: &Self) -> Self {
        let c = self
            .c
            .iter()
            .zip(&other.c)
            .map(|(a, b)| a.iter().zip(b).map(|(x, y)| x.sub(y)).collect())
            .collect();
        Net { c, ..self.clone() }
    }

    /// The coefficients along `side`.
    fn side(&self, side: Side) -> Vec<Vector<D>> {
        let (p, q) = self.degrees();
        match (side.u_fixed, side.at_end) {
            (true, false) => self.c[0].clone(),
            (true, true) => self.c[p].clone(),
            (false, false) => self.c.iter().map(|row| row[0]).collect(),
            (false, true) => self.c.iter().map(|row| row[q]).collect(),
        }
    }

    /// The polynomial divided by the factor that vanishes along `side` —
    /// `t` at the start, `1 − t` at the end of the parameter fixed there,
    /// over the box — where every coefficient on that side could be zero:
    /// `Σ c_i B_i^n = (1 − t) Σ_{i<n} c_i n / (n − i) B_i^{n−1}` when
    /// `c_n = 0`, and likewise at the start. `None` where a coefficient
    /// there is definitely not zero, or the degree there is zero.
    ///
    /// Only for a polynomial the side truly is a zero of — a pole's: an
    /// enclosure of zero divided out is exact only if the truth is zero.
    pub fn divided(&self, side: Side) -> GeopResult<Option<Self>> {
        let (p, q) = self.degrees();
        let n = if side.u_fixed { p } else { q };
        if n == 0
            || !self
                .side(side)
                .iter()
                .all(|c| (0..D).all(|k| c[k].could_be_equal(Scalar::ZERO)))
        {
            return Ok(None);
        }
        // Coefficient `i` of the quotient is `c_i n / (n − i)` at the end,
        // `c_{i+1} n / (i + 1)` at the start.
        let factor = |i: usize| -> GeopResult<Scalar> {
            let rest = if side.at_end { n - i } else { i + 1 };
            Scalar::from_i64(n as i64).div(Scalar::from_i64(rest as i64))
        };
        let kept = |i: usize| if side.at_end { i } else { i + 1 };
        let c = if side.u_fixed {
            (0..n)
                .map(|i| {
                    let f = factor(i)?;
                    Ok(self.c[kept(i)].iter().map(|x| x.prod_scalar(f)).collect())
                })
                .collect::<GeopResult<Vec<Vec<Vector<D>>>>>()?
        } else {
            self.c
                .iter()
                .map(|row| {
                    (0..n)
                        .map(|j| Ok(row[kept(j)].prod_scalar(factor(j)?)))
                        .collect::<GeopResult<Vec<_>>>()
                })
                .collect::<GeopResult<Vec<_>>>()?
        };
        Ok(Some(Net { c, ..self.clone() }))
    }
}

impl Net<4> {
    /// The homogeneous control net of `patch`, a single Bézier patch over
    /// its domain, moved to a control point of it as the origin (see
    /// `spline::centered`): the polynomials made of it are of the patch's
    /// size, not of its distance from the origin.
    pub fn of(patch: &NurbSurface3D) -> GeopResult<Self> {
        let first = patch.control_points[0];
        let origin: Vec<Scalar> = (0..3)
            .map(|k| Ok(first[k].div(first[3])?.sharpen()))
            .collect::<GeopResult<_>>()?;
        let c = (0..patch.num_u)
            .map(|i| {
                (0..patch.num_v)
                    .map(|j| {
                        let mut p = patch.control_points[i * patch.num_v + j];
                        for k in 0..3 {
                            p[k] = p[k].sub(p[3].mul(origin[k]));
                        }
                        p
                    })
                    .collect()
            })
            .collect();
        Ok(Net {
            c,
            u: patch.domain_u(),
            v: patch.domain_v(),
        })
    }

    /// The point part `A` and the weight `w`.
    pub fn split(&self) -> (Net<3>, Net<1>) {
        let a = self
            .c
            .iter()
            .map(|row| {
                row.iter()
                    .map(|p| Vector::from_array([p[0], p[1], p[2]]))
                    .collect()
            })
            .collect();
        let w = self
            .c
            .iter()
            .map(|row| row.iter().map(|p| Vector::from_array([p[3]])).collect())
            .collect();
        (
            Net {
                c: a,
                u: self.u,
                v: self.v,
            },
            Net {
                c: w,
                u: self.u,
                v: self.v,
            },
        )
    }
}

impl Net<3> {
    /// The polynomial as a surface, weights one, to evaluate and
    /// differentiate (see [`NurbSurface3D::partials`]).
    pub fn surface(&self) -> GeopResult<NurbSurface3D> {
        let (p, q) = self.degrees();
        let knots = |(lo, hi): (Scalar, Scalar), n: usize| {
            let mut k = vec![lo; n + 1];
            k.extend(vec![hi; n + 1]);
            k
        };
        let control_points = self
            .c
            .iter()
            .flat_map(|row| {
                row.iter()
                    .map(|x| Vector::from_array([x[0], x[1], x[2], Scalar::ONE]))
            })
            .collect();
        NurbSurface3D::try_new(p, q, control_points, knots(self.u, p), knots(self.v, q))
    }
}

/// Bilinear products the normal field is made of.
pub(crate) fn scale(w: &Vector<1>, a: &Vector3) -> Vector3 {
    a.prod_scalar(w[0])
}

pub(crate) fn cross(a: &Vector3, b: &Vector3) -> Vector3 {
    a.prod_cross(b)
}

/// The polynomial `M = w (A_u × A_v) − w_u (A × A_v) − w_v (A_u × A)` of
/// the Bézier patch `patch`, `M / w³ = S_u × S_v` (see the module docs).
pub(crate) fn normal_field(patch: &NurbSurface3D) -> GeopResult<Net<3>> {
    let (a, w) = Net::of(patch)?.split();
    let (a_u, a_v) = (a.derivative(true)?, a.derivative(false)?);
    let (w_u, w_v) = (w.derivative(true)?, w.derivative(false)?);
    let first = w.product(&a_u.product(&a_v, cross)?, scale)?;
    let second = w_u.product(&a.product(&a_v, cross)?, scale)?;
    let third = w_v.product(&a_u.product(&a, cross)?, scale)?;
    Ok(first.minus(&second).minus(&third))
}

/// The sides of the Bézier patch `patch` that collapse to a point: a pole's.
pub(crate) fn poles(patch: &NurbSurface3D) -> GeopResult<Vec<Side>> {
    let mut out = Vec::new();
    for side in Side::ALL {
        let row: Vec<usize> = if side.u_fixed {
            let i = if side.at_end { patch.num_u - 1 } else { 0 };
            (0..patch.num_v).map(|j| i * patch.num_v + j).collect()
        } else {
            let j = if side.at_end { patch.num_v - 1 } else { 0 };
            (0..patch.num_u).map(|i| i * patch.num_v + j).collect()
        };
        let first = patch.control_points[row[0]].dehomogenized()?;
        let mut collapsed = true;
        for &k in &row[1..] {
            collapsed &= patch.control_points[k]
                .dehomogenized()?
                .could_be_equal(&first);
        }
        if collapsed {
            out.push(side);
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_fixtures::{revolved, sphere_octant};

    /// `M / w³` is `S_u × S_v`, at a point inside the patch.
    #[test]
    fn the_normal_field_is_the_cross_product_of_the_partials() {
        for patch in [sphere_octant(), revolved(1., 0.)] {
            let m = normal_field(&patch).unwrap().surface().unwrap();
            let (u, v) = (Scalar::from_f64(0.3), Scalar::from_f64(0.6));
            let (su, sv) = patch.derivatives(u, v).unwrap();
            let w = patch.weight(u, v).unwrap();
            let expected = su.prod_cross(&sv).prod_scalar(w.mul(w).mul(w));
            let got = m.evaluate(u, v).unwrap();
            assert!(got.could_be_equal(&expected), "{got:?} vs {expected:?}");
        }
    }

    /// At a pole the field vanishes along the collapsed side; divided by
    /// its factor, it is the normal's direction there, and nowhere zero.
    #[test]
    fn a_poles_factor_divides_out() {
        let patch = sphere_octant();
        let sides = poles(&patch).unwrap();
        assert_eq!(sides.len(), 1, "{sides:?}");
        let mut m = normal_field(&patch).unwrap();
        m = m
            .divided(sides[0])
            .unwrap()
            .expect("the pole's side is zero");
        let surface = m.surface().unwrap();
        let (u, v) = if sides[0].u_fixed {
            (
                if sides[0].at_end {
                    Scalar::ONE
                } else {
                    Scalar::ZERO
                },
                Scalar::from_f64(0.4),
            )
        } else {
            (
                Scalar::from_f64(0.4),
                if sides[0].at_end {
                    Scalar::ONE
                } else {
                    Scalar::ZERO
                },
            )
        };
        let at = surface.evaluate(u, v).unwrap();
        let pole = patch.evaluate(u, v).unwrap();
        // On the unit sphere about the origin the normal is the point.
        let direction = at.normalize().unwrap();
        let expected = pole.normalize().unwrap();
        assert!(
            direction.could_be_equal(&expected) || direction.could_be_equal(&expected.neg()),
            "{direction:?} vs {expected:?}"
        );
    }
}
