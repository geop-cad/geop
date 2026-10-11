//! A dense, fixed-size, row-major matrix — [`Vector`]'s 2-D counterpart.
//!
//! Exists so callers building a normal-equations system (`JᵀJ`, `Jᵀf`, …)
//! work with actual matrix/vector values and named operations
//! (`transpose`, `mul_vec`, `mul_mat`) instead of raw `[[S; N]; M]` arrays
//! and hand-rolled index loops at every call site — see
//! [`crate::solvers::linear::solve_linear_system`].

use std::ops::{Index, IndexMut};

use crate::{geop_error::GeopResult, scalars::Scalar, vector::Vector};

/// Dense `R×C` matrix, row-major (`self[(row, col)]`).
#[derive(Debug, Clone, Copy)]
pub struct Matrix<S, const R: usize, const C: usize> {
    data: [[S; C]; R],
}

impl<S: Scalar, const R: usize, const C: usize> Matrix<S, R, C> {
    pub fn from_rows(data: [[S; C]; R]) -> Self {
        Self { data }
    }

    pub fn zero() -> Self {
        Self {
            data: [[S::ZERO; C]; R],
        }
    }

    /// Build from `C` column vectors, each an `R`-vector.
    pub fn from_columns(cols: [Vector<S, R>; C]) -> Self {
        let mut out = Self::zero();
        for (c, col) in cols.iter().enumerate() {
            for r in 0..R {
                out.data[r][c] = col[r];
            }
        }
        out
    }

    pub fn row(&self, r: usize) -> Vector<S, C> {
        Vector::from_array(self.data[r])
    }

    pub fn col(&self, c: usize) -> Vector<S, R> {
        let mut out = Vector::<S, R>::zero();
        for r in 0..R {
            out[r] = self.data[r][c];
        }
        out
    }

    pub fn transpose(&self) -> Matrix<S, C, R> {
        let mut out = Matrix::<S, C, R>::zero();
        for r in 0..R {
            for c in 0..C {
                out[(c, r)] = self[(r, c)];
            }
        }
        out
    }

    /// `self · v`.
    pub fn mul_vec(&self, v: &Vector<S, C>) -> Vector<S, R> {
        let mut out = Vector::<S, R>::zero();
        for r in 0..R {
            out[r] = self.row(r).prod_dot(v);
        }
        out
    }

    /// `self · other`.
    pub fn mul_mat<const K: usize>(&self, other: &Matrix<S, C, K>) -> Matrix<S, R, K> {
        let mut out = Matrix::<S, R, K>::zero();
        for r in 0..R {
            for k in 0..K {
                out[(r, k)] = self.row(r).prod_dot(&other.col(k));
            }
        }
        out
    }

    pub(crate) fn swap_rows(&mut self, i: usize, j: usize) {
        self.data.swap(i, j);
    }
}

impl<S, const R: usize, const C: usize> Index<(usize, usize)> for Matrix<S, R, C> {
    type Output = S;
    fn index(&self, (r, c): (usize, usize)) -> &S {
        &self.data[r][c]
    }
}
impl<S, const R: usize, const C: usize> IndexMut<(usize, usize)> for Matrix<S, R, C> {
    fn index_mut(&mut self, (r, c): (usize, usize)) -> &mut S {
        &mut self.data[r][c]
    }
}

/// The eigenvalues of the symmetric `a`, ascending — each enclosed — and a
/// unit eigenvector of each.
///
/// Cyclic Jacobi rotations diagonalize `a`'s midpoint in `f64`; which
/// rotations to take is a free choice, so that is not where the enclosure
/// comes from. It comes from two bounds every computed pair `(ρ, v)`
/// satisfies: a symmetric matrix has an eigenvalue within `|M v - ρ v|` of
/// the Rayleigh quotient `ρ` of a unit `v` (evaluated here in `S`), and
/// moving `M` to any matrix `a` holds moves each eigenvalue, in order, by
/// at most the Frobenius norm of the difference (Weyl) — bounded by `a`'s
/// half-widths. Where two enclosures overlap their eigenvalues cannot be
/// told apart, so each gets their hull. The vectors are the midpoint's:
/// where eigenvalues coincide, as for a cylinder's two transverse moments,
/// any vector of their plane is one, and these are a free choice of it.
#[allow(clippy::needless_range_loop)]
pub fn symmetric_eigen3<S: Scalar>(a: &Matrix<S, 3, 3>) -> GeopResult<([S; 3], [Vector<S, 3>; 3])> {
    let mut m = [[0.0f64; 3]; 3];
    for r in 0..3 {
        for c in 0..3 {
            // Symmetric by construction: the mean of the two midpoints.
            m[r][c] = 0.5 * (a[(r, c)].to_f64() + a[(c, r)].to_f64());
        }
    }
    let midpoint = m;
    let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0f64]];
    for _ in 0..64 {
        let off = m[0][1].abs() + m[0][2].abs() + m[1][2].abs();
        if off == 0.0 {
            break;
        }
        for (p, q) in [(0, 1), (0, 2), (1, 2)] {
            if m[p][q] == 0.0 {
                continue;
            }
            let theta = (m[q][q] - m[p][p]) / (2.0 * m[p][q]);
            let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
            let c = 1.0 / (t * t + 1.0).sqrt();
            let s = t * c;
            for k in 0..3 {
                let (mkp, mkq) = (m[k][p], m[k][q]);
                m[k][p] = c * mkp - s * mkq;
                m[k][q] = s * mkp + c * mkq;
            }
            for k in 0..3 {
                let (mpk, mqk) = (m[p][k], m[q][k]);
                m[p][k] = c * mpk - s * mqk;
                m[q][k] = s * mpk + c * mqk;
            }
            for row in &mut v {
                let (vp, vq) = (row[p], row[q]);
                row[p] = c * vp - s * vq;
                row[q] = s * vp + c * vq;
            }
        }
    }
    // Weyl: how far `a` may be from its midpoint.
    let mut spread = S::ZERO;
    for r in 0..3 {
        for c in 0..3 {
            let half = a[(r, c)].width().div(S::TWO)?;
            spread = spread.add(half.mul(half));
        }
    }
    let spread = spread.sqrt()?.upper();
    let sharp = Matrix::<S, 3, 3>::from_rows(midpoint.map(|row| row.map(S::from_f64)));
    let mut pairs = Vec::with_capacity(3);
    for k in 0..3 {
        let column = Vector::from_array([v[0][k], v[1][k], v[2][k]].map(S::from_f64));
        let unit = column.normalize()?;
        let image = sharp.mul_vec(&unit);
        let rayleigh = unit.prod_dot(&image);
        let residual = image.sub(&unit.prod_scalar(rayleigh)).norm().upper();
        let reach = residual.add(spread);
        let enclosure = rayleigh
            .sub(reach)
            .lower()
            .union(rayleigh.add(reach).upper());
        pairs.push((enclosure, unit));
    }
    pairs.sort_by(|a, b| a.0.to_f64().total_cmp(&b.0.to_f64()));
    // Overlapping enclosures share their hull.
    let mut values: Vec<S> = pairs.iter().map(|(e, _)| *e).collect();
    for _ in 0..2 {
        for i in 0..2 {
            if values[i].could_be_equal(values[i + 1]) {
                let hull = values[i].union(values[i + 1]);
                values[i] = hull;
                values[i + 1] = hull;
            }
        }
    }
    Ok((
        [values[0], values[1], values[2]],
        [pairs[0].1, pairs[1].1, pairs[2].1],
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::for_all_scalars;

    /// A rotated diagonal matrix: its eigenvalues are enclosed, a repeated
    /// pair shares one enclosure, and each vector is one of its value.
    fn check_eigen_of_a_rotated_diagonal<S: Scalar>() {
        let (c, s) = (0.6, 0.8);
        // R diag(2, 5, 5) R^T, R a rotation about z.
        let r = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];
        let d = [2.0, 5.0, 5.0];
        let mut rows = [[S::ZERO; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let x: f64 = (0..3).map(|k| r[i][k] * d[k] * r[j][k]).sum();
                rows[i][j] = S::from_f64(x);
            }
        }
        let a = Matrix::from_rows(rows);
        let (values, vectors) = symmetric_eigen3(&a).unwrap();
        for (value, want) in values.iter().zip(d) {
            assert!(value.could_be_equal(S::from_f64(want)), "{values:?}");
            // `ScalInFPA64`'s square root of a residual at its resolution,
            // 2^-32, is 2^-16: the widest of the two scalars.
            assert!(value.width().to_f64() < 1e-3, "{values:?}");
        }
        let image = a.mul_vec(&vectors[0]);
        assert!(image.could_be_equal(&vectors[0].prod_scalar(values[0])));
    }
    #[test]
    fn eigen_of_a_rotated_diagonal() {
        for_all_scalars!(check_eigen_of_a_rotated_diagonal);
    }

    fn check_transpose_and_mul<S: Scalar>() {
        let m = Matrix::<S, 2, 3>::from_rows([
            [S::from_f64(1.0), S::from_f64(2.0), S::from_f64(3.0)],
            [S::from_f64(4.0), S::from_f64(5.0), S::from_f64(6.0)],
        ]);
        let mt = m.transpose();
        assert!(mt[(0, 1)].could_be_equal(S::from_f64(4.0)));
        assert!(mt[(2, 0)].could_be_equal(S::from_f64(3.0)));

        let v = Vector::<S, 3>::from_array([S::ONE, S::ONE, S::ONE]);
        let mv = m.mul_vec(&v);
        assert!(mv[0].could_be_equal(S::from_f64(6.0)));
        assert!(mv[1].could_be_equal(S::from_f64(15.0)));

        let mtm = mt.mul_mat(&m); // 3x3
        assert!(mtm[(0, 0)].could_be_equal(S::from_f64(1.0 + 16.0)));
    }
    #[test]
    fn transpose_and_mul() {
        for_all_scalars!(check_transpose_and_mul);
    }
}
