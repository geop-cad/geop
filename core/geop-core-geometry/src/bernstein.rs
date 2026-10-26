//! Polynomials in Bernstein form over a rectangle: one coordinate of a
//! Bézier patch, or — of degree zero along `v` — of a Bézier curve,
//! multiplied and added as polynomials, with no evaluation and no
//! sampling. What composing a patch with a polynomial map of space takes
//! (see [`crate::deform`]).
//!
//! The product rests on `B_i^m B_k^n = C(m, i) C(n, k) / C(m + n, i + k)
//! B_{i+k}^{m+n}`, along each direction.

use geop_core_math::{combinatorics::binomial, geop_error::GeopResult, scalars::Scalar};

/// A polynomial of degree `degree[0]` in `u` and `degree[1]` in `v`, by its
/// Bernstein coefficients, `u`-index major, the coefficient of `(i, j)` at
/// `i * (degree[1] + 1) + j`, the way a
/// [`crate::nurb_surface::NurbSurface`] stores its control points.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Bernstein {
    degree: [usize; 2],
    coefficients: Vec<Scalar>,
}

/// Adds `value` into `slot`, the first term taken as it is: adding it to
/// an exact zero would only widen it by rounding.
fn accumulate(slot: &mut Option<Scalar>, value: Scalar) {
    *slot = Some(match *slot {
        None => value,
        Some(sum) => sum.add(value),
    });
}

/// `C(m, i) C(n, k) / C(m + n, i + k)` for every `i ≤ m`, `k ≤ n`, as the
/// numerator and denominator of the ratio: what the term `i` of a
/// polynomial of degree `m` times the term `k` of one of degree `n`
/// contributes to the term `i + k` of their product.
fn product_weights(m: usize, n: usize) -> Vec<Vec<(i64, i64)>> {
    (0..=m)
        .map(|i| {
            (0..=n)
                .map(|k| (binomial(m, i) * binomial(n, k), binomial(m + n, i + k)))
                .collect()
        })
        .collect()
}

/// The exact ratio `num / den`: one where it is one, which multiplies
/// nothing (see `accumulate`).
fn ratio(num: i64, den: i64) -> GeopResult<Option<Scalar>> {
    Ok(if num == den {
        None
    } else {
        Some(Scalar::from_ratio(num, den)?)
    })
}

impl Bernstein {
    pub(crate) fn new(degree: [usize; 2], coefficients: Vec<Scalar>) -> Self {
        assert_eq!(
            coefficients.len(),
            (degree[0] + 1) * (degree[1] + 1),
            "Bernstein::new: coefficients for degree {degree:?}"
        );
        Bernstein {
            degree,
            coefficients,
        }
    }

    /// The constant `value`, of degree zero.
    pub(crate) fn constant(value: Scalar) -> Self {
        Bernstein::new([0, 0], vec![value])
    }

    pub(crate) fn degree(&self) -> [usize; 2] {
        self.degree
    }

    pub(crate) fn coefficients(&self) -> &[Scalar] {
        &self.coefficients
    }

    fn at(&self, i: usize, j: usize) -> Scalar {
        self.coefficients[i * (self.degree[1] + 1) + j]
    }

    /// The product, of degree the sum of theirs.
    pub(crate) fn product(&self, other: &Self) -> GeopResult<Self> {
        let [m1, m2] = self.degree;
        let [n1, n2] = other.degree;
        let degree = [m1 + n1, m2 + n2];
        let (wu, wv) = (product_weights(m1, n1), product_weights(m2, n2));
        let mut sums = vec![None; (degree[0] + 1) * (degree[1] + 1)];
        for i in 0..=m1 {
            for j in 0..=m2 {
                let a = self.at(i, j);
                for k in 0..=n1 {
                    for l in 0..=n2 {
                        let (nu, du) = wu[i][k];
                        let (nv, dv) = wv[j][l];
                        let b = other.at(k, l);
                        // A factor of exactly one multiplies nothing.
                        let mut term = if a.is_exactly(Scalar::ONE) {
                            b
                        } else if b.is_exactly(Scalar::ONE) {
                            a
                        } else {
                            a.mul(b)
                        };
                        if let Some(weight) = ratio(nu * nv, du * dv)? {
                            term = term.mul(weight);
                        }
                        accumulate(&mut sums[(i + k) * (degree[1] + 1) + j + l], term);
                    }
                }
            }
        }
        let coefficients = sums.into_iter().map(|s| s.expect("every term met"));
        Ok(Bernstein::new(degree, coefficients.collect()))
    }

    /// `self` to the power `n`; one, of degree zero, for `n = 0`.
    pub(crate) fn power(&self, n: usize) -> GeopResult<Self> {
        let mut out = Bernstein::constant(Scalar::ONE);
        for _ in 0..n {
            out = out.product(self)?;
        }
        Ok(out)
    }

    /// The same polynomial, raised to the degree `to` (at least its own):
    /// along each direction `c'_k = Σ_i C(m, i) C(r, k - i) / C(m + r, k)
    /// c_i`, the end coefficients kept as they are.
    pub(crate) fn elevated(&self, to: [usize; 2]) -> GeopResult<Self> {
        let raise = |coefficients: &[Scalar], m: usize, r: usize| -> GeopResult<Vec<Scalar>> {
            (0..=m + r)
                .map(|k| {
                    let mut sum = None;
                    for (i, &c) in coefficients.iter().enumerate() {
                        if k < i || k - i > r {
                            continue;
                        }
                        let num = binomial(m, i) * binomial(r, k - i);
                        let term = match ratio(num, binomial(m + r, k))? {
                            None => c,
                            Some(weight) => c.mul(weight),
                        };
                        accumulate(&mut sum, term);
                    }
                    Ok(sum.expect("every term met"))
                })
                .collect()
        };
        let [m1, m2] = self.degree;
        let (r1, r2) = (to[0] - m1, to[1] - m2);
        // Along `v`, row by row; then along `u`, column by column.
        let mut rows = Vec::with_capacity((m1 + 1) * (to[1] + 1));
        for i in 0..=m1 {
            let row = &self.coefficients[i * (m2 + 1)..(i + 1) * (m2 + 1)];
            rows.extend(if r2 == 0 {
                row.to_vec()
            } else {
                raise(row, m2, r2)?
            });
        }
        if r1 == 0 {
            return Ok(Bernstein::new([m1, to[1]], rows));
        }
        let mut coefficients = vec![Scalar::ZERO; (to[0] + 1) * (to[1] + 1)];
        for j in 0..=to[1] {
            let column: Vec<Scalar> = (0..=m1).map(|i| rows[i * (to[1] + 1) + j]).collect();
            for (k, c) in raise(&column, m1, r1)?.into_iter().enumerate() {
                coefficients[k * (to[1] + 1) + j] = c;
            }
        }
        Ok(Bernstein::new(to, coefficients))
    }

    /// The sum `Σ factor · term` over `terms`, each raised to the highest
    /// degree among them; a factor of `None` is one and multiplies nothing.
    /// `None` for no terms.
    pub(crate) fn combination<'a>(
        terms: impl IntoIterator<Item = (Option<Scalar>, &'a Bernstein)>,
    ) -> GeopResult<Option<Self>> {
        let terms: Vec<_> = terms.into_iter().collect();
        let Some(degree) = terms
            .iter()
            .map(|(_, b)| b.degree)
            .reduce(|a, b| [a[0].max(b[0]), a[1].max(b[1])])
        else {
            return Ok(None);
        };
        let mut sums = vec![None; (degree[0] + 1) * (degree[1] + 1)];
        for (factor, term) in terms {
            let raised;
            let term = if term.degree == degree {
                term
            } else {
                raised = term.elevated(degree)?;
                &raised
            };
            for (sum, &c) in sums.iter_mut().zip(&term.coefficients) {
                accumulate(sum, factor.map_or(c, |f| f.mul(c)));
            }
        }
        let coefficients = sums.into_iter().map(|s| s.expect("every term met"));
        Ok(Some(Bernstein::new(degree, coefficients.collect())))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The value at `(u, v)` by de Casteljau along each direction.
    fn value(b: &Bernstein, u: f64, v: f64) -> f64 {
        let reduce = |c: Vec<f64>, t: f64| {
            let mut c = c;
            while c.len() > 1 {
                c = c.windows(2).map(|w| w[0] * (1.0 - t) + w[1] * t).collect();
            }
            c[0]
        };
        let [m1, m2] = b.degree();
        let rows: Vec<f64> = (0..=m1)
            .map(|i| reduce((0..=m2).map(|j| b.at(i, j).to_f64()).collect(), v))
            .collect();
        reduce(rows, u)
    }

    fn poly(degree: [usize; 2], seed: f64) -> Bernstein {
        let n = (degree[0] + 1) * (degree[1] + 1);
        let c = (0..n).map(|k| Scalar::from_f64((k as f64 * seed).sin()));
        Bernstein::new(degree, c.collect())
    }

    /// A product and an elevation are the polynomials they should be, at a
    /// few points.
    #[test]
    fn product_and_elevation_are_the_same_polynomials() {
        let (a, b) = (poly([2, 1], 0.7), poly([1, 3], 1.3));
        let ab = a.product(&b).unwrap();
        let raised = a.elevated([4, 3]).unwrap();
        let sum = Bernstein::combination([(None, &a), (Some(Scalar::TWO), &b)])
            .unwrap()
            .unwrap();
        for (u, v) in [(0.0, 0.0), (0.3, 0.8), (1.0, 0.5), (0.6, 1.0)] {
            let (va, vb) = (value(&a, u, v), value(&b, u, v));
            assert!((value(&ab, u, v) - va * vb).abs() < 1e-12);
            assert!((value(&raised, u, v) - va).abs() < 1e-12);
            assert!((value(&sum, u, v) - (va + 2.0 * vb)).abs() < 1e-12);
        }
        assert_eq!(a.power(3).unwrap().degree(), [6, 3]);
    }
}
