//! A dense, fixed-size, row-major matrix — [`Vector`]'s 2-D counterpart.
//!
//! Exists so callers building a normal-equations system (`JᵀJ`, `Jᵀf`, the
//! Krawczyk operator's `Y·J(X)`, …) work with actual matrix/vector values
//! and named operations (`transpose`, `mul_vec`, `mul_mat`) instead of raw
//! `[[S; N]; M]` arrays and hand-rolled index loops at every call site —
//! see [`crate::interval_newton`] and [`solve_linear_system`] for the two
//! main consumers.

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

    pub fn add(&self, other: &Self) -> Self {
        let mut out = Self::zero();
        for r in 0..R {
            for c in 0..C {
                out[(r, c)] = self[(r, c)].add(other[(r, c)]);
            }
        }
        out
    }

    pub fn sub(&self, other: &Self) -> Self {
        let mut out = Self::zero();
        for r in 0..R {
            for c in 0..C {
                out[(r, c)] = self[(r, c)].sub(other[(r, c)]);
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

    fn swap_rows(&mut self, i: usize, j: usize) {
        self.data.swap(i, j);
    }
}

impl<S: Scalar, const N: usize> Matrix<S, N, N> {
    pub fn identity() -> Self {
        let mut out = Self::zero();
        for i in 0..N {
            out[(i, i)] = S::ONE;
        }
        out
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

/// Solve the dense `N x N` system `a * x = b` by Gaussian elimination with
/// partial pivoting, returning `x`.
///
/// Errors if the system is singular — which, with interval scalars, means
/// the pivot *could* be zero, not merely that it is: a pivot straddling
/// zero carries no usable information about the solution, and dividing by
/// it would manufacture an arbitrarily wide answer rather than report that
/// there isn't one. Callers that can proceed without a solution (a
/// rank-deficient configuration they have a fallback for) should handle the
/// error rather than pre-screen the matrix.
///
/// Pivot selection compares the *sharpened* magnitudes of the candidates.
/// That is a conditioning choice, not a correctness claim: any row with a
/// nonzero pivot yields the same solution set, and picking the largest one
/// only keeps the elimination numerically well-behaved — so resolving the
/// comparison to a single value (rather than leaving it three-valued) costs
/// nothing, exactly as in `NurbSurface::project`'s per-iteration sharpening.
pub fn solve_linear_system<S: Scalar, const N: usize>(
    a: &Matrix<S, N, N>,
    b: &Vector<S, N>,
) -> GeopResult<Vector<S, N>> {
    let mut a = *a;
    let mut b = *b;

    for col in 0..N {
        let mut pivot = col;
        for row in (col + 1)..N {
            if a[(row, col)]
                .abs()
                .sharpen()
                .definitely_greater(a[(pivot, col)].abs().sharpen())
            {
                pivot = row;
            }
        }
        a.swap_rows(col, pivot);
        let tmp = b[col];
        b[col] = b[pivot];
        b[pivot] = tmp;

        for row in (col + 1)..N {
            let factor = a[(row, col)].div(a[(col, col)]).map_err(|e| {
                e.with_context(format!(
                    "solve_linear_system: singular at column {col}, pivot={:?}",
                    a[(col, col)]
                ))
            })?;
            for k in col..N {
                a[(row, k)] = a[(row, k)].sub(factor.mul(a[(col, k)]));
            }
            b[row] = b[row].sub(factor.mul(b[col]));
        }
    }

    let mut x = Vector::<S, N>::zero();
    for row in (0..N).rev() {
        let mut sum = b[row];
        for k in (row + 1)..N {
            sum = sum.sub(a[(row, k)].mul(x[k]));
        }
        x[row] = sum.div(a[(row, row)]).map_err(|e| {
            e.with_context(format!(
                "solve_linear_system: singular at row {row}, pivot={:?}",
                a[(row, row)]
            ))
        })?;
    }
    Ok(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::for_all_scalars;

    fn check_solves_identity<S: Scalar>() {
        let a = Matrix::<S, 2, 2>::identity();
        let b = Vector::<S, 2>::from_array([S::from_f64(3.0), S::from_f64(4.0)]);
        let x = solve_linear_system(&a, &b).unwrap();
        assert!(x[0].could_be_equal(S::from_f64(3.0)));
        assert!(x[1].could_be_equal(S::from_f64(4.0)));
    }
    #[test]
    fn solves_identity() {
        for_all_scalars!(check_solves_identity);
    }

    fn check_solves_general_2x2<S: Scalar>() {
        // [2 1; 1 3] x = [5; 10] -> x = [1, 3]
        let a = Matrix::from_rows([
            [S::from_f64(2.0), S::from_f64(1.0)],
            [S::from_f64(1.0), S::from_f64(3.0)],
        ]);
        let b = Vector::from_array([S::from_f64(5.0), S::from_f64(10.0)]);
        let x = solve_linear_system(&a, &b).unwrap();
        assert!(x[0].could_be_equal(S::ONE));
        assert!(x[1].could_be_equal(S::from_f64(3.0)));
    }
    #[test]
    fn solves_general_2x2() {
        for_all_scalars!(check_solves_general_2x2);
    }

    fn check_singular_errs<S: Scalar>() {
        let a = Matrix::from_rows([[S::ONE, S::ONE], [S::ONE, S::ONE]]);
        let b = Vector::from_array([S::ONE, S::TWO]);
        assert!(solve_linear_system(&a, &b).is_err());
    }
    #[test]
    fn singular_errs() {
        for_all_scalars!(check_singular_errs);
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
