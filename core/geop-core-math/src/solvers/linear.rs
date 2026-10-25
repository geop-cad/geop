//! Dense linear algebra, in honest enclosures: a pivot counts as zero
//! exactly when it could be zero. One implementation per algorithm, for
//! every solver of the kernel:
//!
//! - [`solve_linear_system`]: a small fixed-size system, by Gaussian
//!   elimination with partial pivoting on sharpened magnitudes.
//! - [`solve_pivoted`]: a square system with several right-hand sides, by
//!   Gaussian elimination with partial pivoting on midpoints.
//! - [`solve_banded`]: a banded system that needs no pivoting.
//! - [`solve_symmetric2`]: a symmetric 2x2 system — a Newton step on a
//!   surface's two parameters — by Cramer's rule.
//! - [`cholesky`] and [`solve_cholesky`]: a symmetric positive
//!   semi-definite one, rows that say nothing new left out.
//! - [`shortest_solution`]: the shortest step meeting a few linear
//!   conditions in space — least squares where they are too many.
//! - [`eliminate`], [`null_space`], [`rank`], [`inverse`]: Gauss-Jordan
//!   elimination with complete pivoting over a Jacobian — which rows are
//!   independent, what they leave free, and inverses.
//!
//! [`solve_linear_system`] and [`solve_pivoted`] are the same algorithm in
//! two shapes, but not the same arithmetic: they choose their pivots by
//! different comparisons, and the latter back-substitutes by multiplying
//! with the pivot's reciprocal where the former divides. Their callers
//! build curves and surfaces, which are sensitive to the last bit, so the
//! two are kept as they are rather than merged into one that would round
//! differently for one of them.

use crate::{
    geop_error::{GeopError, GeopResult},
    matrix::Matrix,
    scalars::Scalar,
    vector::{Vector, Vector3},
};

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
pub fn solve_linear_system<const N: usize>(
    a: &Matrix<N, N>,
    b: &Vector<N>,
) -> GeopResult<Vector<N>> {
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

    let mut x = Vector::<N>::zero();
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

/// The solution `(x, y)` of the symmetric 2x2 system
/// `[[a11, a12], [a12, a22]] (x, y) = (b1, b2)`, by Cramer's rule: an error
/// where the determinant could be zero.
pub fn solve_symmetric2(
    (a11, a12, a22): (Scalar, Scalar, Scalar),
    (b1, b2): (Scalar, Scalar),
) -> GeopResult<(Scalar, Scalar)> {
    let det = a11.mul(a22).sub(a12.mul(a12));
    Ok((
        b1.mul(a22).sub(b2.mul(a12)).div(det)?,
        a11.mul(b2).sub(a12.mul(b1)).div(det)?,
    ))
}

/// Solves `a x = rhs` for the `n x n` matrix `a` and `C` right-hand sides
/// by Gaussian elimination, pivoting on the entry of largest magnitude in
/// each column. Which row to pivot on is a free choice — every pivot that
/// cannot be zero gives the same enclosed solution — and the largest is the
/// well-conditioned one. Fails if no entry of a column definitely differs
/// from zero.
#[allow(
    clippy::needless_range_loop,
    reason = "rows of one matrix, indexed alike"
)]
pub fn solve_pivoted<const C: usize>(
    mut a: Vec<Vec<Scalar>>,
    mut rhs: Vec<Vector<C>>,
) -> GeopResult<Vec<Vector<C>>> {
    let n = a.len();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&i, &j| {
                a[i][col]
                    .abs()
                    .to_f64()
                    .total_cmp(&a[j][col].abs().to_f64())
            })
            .expect("a column has rows");
        if a[pivot][col].could_be_equal(Scalar::ZERO) {
            return Err(GeopError::new(format!(
                "solve_pivoted: column {col} has no entry that is definitely not zero"
            )));
        }
        a.swap(col, pivot);
        rhs.swap(col, pivot);
        for row in (col + 1)..n {
            let factor = a[row][col].div(a[col][col])?;
            for c in col..n {
                a[row][c] = a[row][c].sub(factor.mul(a[col][c]));
            }
            rhs[row] = rhs[row].sub(&rhs[col].prod_scalar(factor));
        }
    }
    let mut x = vec![Vector::<C>::zero(); n];
    for row in (0..n).rev() {
        let mut sum = rhs[row];
        for col in (row + 1)..n {
            sum = sum.sub(&x[col].prod_scalar(a[row][col]));
        }
        x[row] = sum.prod_scalar(Scalar::ONE.div(a[row][row])?);
    }
    Ok(x)
}

/// Solves `a x = rhs` for the banded `m x m` matrix `a` and `C`
/// right-hand sides, by Gaussian elimination without pivoting: row `k` is
/// nonzero only in the columns `first[k] ..= last[k]`, and the bands never
/// move left down the rows. Safe without pivoting only where the matrix
/// says so — a B-spline collocation matrix is totally positive (Piegl &
/// Tiller §9.2.1), the reason the curve interpolation that builds it does
/// not pivot either.
///
/// Elimination without pivoting keeps the band, so it runs within it:
/// below a pivot only the rows whose band reaches its column, and in them
/// only the columns the pivot row's band reaches. Outside the band every
/// factor and every entry is an exact zero, and subtracting zero times
/// zero changes nothing but the enclosure — outward rounding widened each
/// entry by an ulp per pivot above it, and the dense elimination took
/// `m³ / 3` steps for what the band holds in `m (degree + 1)²`.
#[allow(
    clippy::needless_range_loop,
    reason = "rows of one matrix, indexed alike"
)]
pub fn solve_banded<const C: usize>(
    mut a: Vec<Vec<Scalar>>,
    first: &[usize],
    last: &[usize],
    mut rhs: Vec<Vector<C>>,
) -> GeopResult<Vec<Vector<C>>> {
    let m = rhs.len();
    for col in 0..m {
        let pivot = a[col][col];
        for row in (col + 1)..m {
            if first[row] > col {
                break;
            }
            let factor = a[row][col].div(pivot)?;
            for c in col..=last[col] {
                a[row][c] = a[row][c].sub(factor.mul(a[col][c]));
            }
            for c in 0..C {
                rhs[row][c] = rhs[row][c].sub(factor.mul(rhs[col][c]));
            }
        }
    }

    let mut x = vec![Vector::<C>::zero(); m];
    for row in (0..m).rev() {
        let mut sum = rhs[row];
        for col in (row + 1)..=last[row] {
            let coef = a[row][col];
            for c in 0..C {
                sum[c] = sum[c].sub(coef.mul(x[col][c]));
            }
        }
        for c in 0..C {
            x[row][c] = sum[c].div(a[row][row])?;
        }
    }
    Ok(x)
}

/// The Cholesky factor `L` of the symmetric `a`, `a = L Lᵀ` — leaving out
/// every row whose pivot could be zero or less, or is too small to divide
/// by: one that, in the light of
/// the rows before it, says nothing new (or nothing positive). Which rows
/// were left out is `true` in the second.
pub fn cholesky(a: &[Vec<Scalar>]) -> GeopResult<(Vec<Vec<Scalar>>, Vec<bool>)> {
    let n = a.len();
    let mut l = vec![vec![Scalar::ZERO; n]; n];
    let mut left_out = vec![false; n];
    for i in 0..n {
        for j in 0..=i {
            if left_out[j] {
                continue;
            }
            let sum = (0..j).fold(a[i][j], |sum, k| sum.sub(l[i][k].mul(l[j][k])));
            if i == j {
                // A pivot positive by so little that dividing by it gives no
                // finite number says no more than one that could be zero:
                // the solve through it overflowed, `0 × ∞` in the next row.
                let root = if sum.definitely_greater(Scalar::ZERO) {
                    Some(sum.sqrt()?).filter(|r| Scalar::ONE.div(*r).is_ok_and(|v| v.is_finite()))
                } else {
                    None
                };
                if let Some(root) = root {
                    l[i][i] = root;
                } else {
                    left_out[i] = true;
                    l[i].iter_mut().for_each(|v| *v = Scalar::ZERO);
                }
            } else {
                l[i][j] = sum.div(l[j][j])?;
            }
        }
    }
    Ok((l, left_out))
}

/// The solution of `L Lᵀ x = b`, zero at the rows `left_out`.
pub fn solve_cholesky(
    l: &[Vec<Scalar>],
    left_out: &[bool],
    b: &[Scalar],
) -> GeopResult<Vec<Scalar>> {
    let n = b.len();
    let mut y = vec![Scalar::ZERO; n];
    for i in (0..n).filter(|&i| !left_out[i]) {
        let sum = (0..i).fold(b[i], |sum, k| sum.sub(l[i][k].mul(y[k])));
        y[i] = sum.div(l[i][i])?;
    }
    let mut x = vec![Scalar::ZERO; n];
    for i in (0..n).rev().filter(|&i| !left_out[i]) {
        let sum = (i + 1..n).fold(y[i], |sum, k| sum.sub(l[k][i].mul(x[k])));
        x[i] = sum.div(l[i][i])?;
    }
    Ok(x)
}

/// The outcome of Gauss-Jordan elimination (see [`eliminate`]).
pub struct Elimination {
    /// The rows, reduced: row `k` has a one at `pivots[k].0` and zeros at
    /// every other pivot column.
    pub rows: Vec<Vec<Scalar>>,
    /// Per pivot: its column, and the index of the row it was among the
    /// rows eliminated.
    pub pivots: Vec<(usize, usize)>,
    /// Per reduced row: the index of the row it was among the rows
    /// eliminated — for the rows past the pivots' too.
    pub origin: Vec<usize>,
}

/// Gauss-Jordan elimination of `rows` (each over `n` variables) with
/// complete pivoting: each pivot the entry of largest magnitude among the
/// rows and the columns not used yet — a free choice among those that
/// cannot be zero; elimination stops when every candidate could be zero.
/// Which constraints are independent and which variables they determine is
/// decided from the rows' own enclosures; where it matters for correctness,
/// [`crate::solvers::system`]'s enclosure verifies what it is used for.
///
/// Complete, not column by column: going column by column takes the first
/// column with any entry that cannot be zero, and rounding leaves entries
/// of `1e-16` that cannot. Such a pivot determines a variable the
/// constraints barely touch, in place of one they do — a half circle
/// tangent to two parallel sides was left with its sweep free and two
/// tangencies over the same coordinate, and could not be enclosed.
pub fn eliminate(mut rows: Vec<Vec<Scalar>>, n: usize) -> Elimination {
    // Per row, the columns it has an entry in other than an exact zero, in
    // order — of the whole row, which may be longer than `n`: columns past
    // the pivots' go along, as in `inverse`'s: the only ones that can be a
    // pivot (among the first `n`), and the only ones a pivot row changes in
    // another row. The Jacobian of an assembly has a dozen
    // in a row of hundreds, and a pivot row, still, a few dozen.
    let mut support: Vec<Vec<usize>> = rows
        .iter()
        .map(|row| (0..row.len()).filter(|&c| !row[c].is_zero()).collect())
        .collect();
    let mut origin: Vec<usize> = (0..rows.len()).collect();
    let mut pivots: Vec<(usize, usize)> = Vec::new();
    let mut used = vec![false; n];
    let mut r = 0;
    loop {
        let size = |v: Scalar| v.abs().lower();
        let mut best: Option<(usize, usize)> = None;
        for i in r..rows.len() {
            for &col in support[i].iter().filter(|&&c| c < n && !used[c]) {
                let v = rows[i][col];
                if v.definitely_not_equal(Scalar::ZERO)
                    && best.is_none_or(|(bi, bc)| size(v).definitely_greater(size(rows[bi][bc])))
                {
                    best = Some((i, col));
                }
            }
        }
        let Some((best, col)) = best else {
            break;
        };
        rows.swap(r, best);
        support.swap(r, best);
        origin.swap(r, best);
        let pivot = rows[r][col];
        for &c in &support[r] {
            rows[r][c] = rows[r][c]
                .div(pivot)
                .expect("a pivot is definitely not zero");
        }
        let pivot_support = std::mem::take(&mut support[r]);
        for i in (0..rows.len()).filter(|&i| i != r) {
            let factor = rows[i][col];
            if factor.is_zero() {
                continue;
            }
            for &c in &pivot_support {
                rows[i][c] = rows[i][c].sub(factor.mul(rows[r][c]));
            }
            support[i] = union(&support[i], &pivot_support);
        }
        support[r] = pivot_support;
        used[col] = true;
        pivots.push((col, origin[r]));
        r += 1;
    }
    Elimination {
        rows,
        pivots,
        origin,
    }
}

/// The columns of `a` and of `b`, in order.
fn union(a: &[usize], b: &[usize]) -> Vec<usize> {
    let mut out = Vec::with_capacity(a.len() + b.len());
    let (mut i, mut j) = (0, 0);
    while i < a.len() || j < b.len() {
        match (a.get(i), b.get(j)) {
            (Some(&x), Some(&y)) if x == y => {
                out.push(x);
                i += 1;
                j += 1;
            }
            (Some(&x), Some(&y)) if x < y => {
                out.push(x);
                i += 1;
            }
            (Some(&x), None) => {
                out.push(x);
                i += 1;
            }
            (_, Some(&y)) => {
                out.push(y);
                j += 1;
            }
            (None, None) => unreachable!("the loop ends when both are exhausted"),
        }
    }
    out
}

/// A basis of the null space of `rows` (each over `n` variables): the
/// directions in which every row stays put, to first order. One vector per
/// variable no pivot determines, which is one there and zero at every other
/// such variable.
pub fn null_space(rows: Vec<Vec<Scalar>>, n: usize) -> Vec<Vec<Scalar>> {
    let Elimination { rows, pivots, .. } = eliminate(rows, n);
    let is_pivot: Vec<bool> = (0..n).map(|c| pivots.iter().any(|p| p.0 == c)).collect();
    (0..n)
        .filter(|&f| !is_pivot[f])
        .map(|f| {
            let mut v = vec![Scalar::ZERO; n];
            v[f] = Scalar::ONE;
            for (i, &(pc, _)) in pivots.iter().enumerate() {
                v[pc] = rows[i][f].neg();
            }
            v
        })
        .collect()
}

/// How many of `rows` (each over `n` variables) are independent: those an
/// elimination finds a pivot that cannot be zero for.
pub fn rank(rows: Vec<Vec<Scalar>>, n: usize) -> usize {
    eliminate(rows, n).pivots.len()
}

/// `rows` each with the row of the identity beside it, `[row | I]`: an
/// elimination of them reduces the identity alongside, which then says what
/// combination of the rows each reduced row is.
pub fn augmented(rows: impl ExactSizeIterator<Item = Vec<Scalar>>) -> Vec<Vec<Scalar>> {
    let m = rows.len();
    rows.enumerate()
        .map(|(i, mut row)| {
            row.extend((0..m).map(|j| if i == j { Scalar::ONE } else { Scalar::ZERO }));
            row
        })
        .collect()
}

/// An enclosure of the inverse of the square matrix `a`, by Gauss-Jordan
/// elimination: each entry encloses that of the inverse of every matrix `a`
/// encloses. `None` if `a` could be singular.
pub fn inverse(a: &[Vec<Scalar>]) -> Option<Vec<Vec<Scalar>>> {
    let m = a.len();
    let Elimination { rows, pivots, .. } = eliminate(augmented(a.iter().cloned()), m);
    if pivots.len() < m {
        return None;
    }
    let mut inverse = vec![Vec::new(); m];
    for (row, &(col, _)) in rows.into_iter().zip(&pivots) {
        inverse[col] = row[m..].to_vec();
    }
    Some(inverse)
}

/// The shortest `d` with `n_i . d = r_i` for every `(n_i, r_i)` — for more
/// than three conditions, of which three can be met at most, the `d` that
/// misses them least in the least-squares sense.
///
/// Three independent conditions leave only one `d`, solved for directly:
/// the Gram matrix the fewer need squares the system's condition number.
pub fn shortest_solution(rows: &[(Vector3, Scalar)]) -> GeopResult<Vector3> {
    // `d = sum_i lambda_i n_i`, with the Gram matrix `G_ij = n_i . n_j`
    // solving `G lambda = r`.
    fn combine<const N: usize>(rows: &[(Vector3, Scalar)]) -> GeopResult<Vector3> {
        let mut gram = Matrix::<N, N>::zero();
        let mut r = Vector::<N>::zero();
        for i in 0..N {
            r[i] = rows[i].1;
            for j in 0..N {
                gram[(i, j)] = rows[i].0.prod_dot(&rows[j].0);
            }
        }
        let lambda = solve_linear_system(&gram, &r)?;
        Ok((0..N).fold(Vector3::zero(), |d, i| {
            d.add(&rows[i].0.prod_scalar(lambda[i]))
        }))
    }
    match rows.len() {
        0 => Err(GeopError::new("no condition to meet")),
        1 => combine::<1>(rows),
        2 => combine::<2>(rows),
        3 => solve_linear_system(
            &Matrix::from_rows([
                rows[0].0.to_array(),
                rows[1].0.to_array(),
                rows[2].0.to_array(),
            ]),
            &Vector3::from_array([rows[0].1, rows[1].1, rows[2].1]),
        ),
        _ => {
            let mut normal = Matrix::<3, 3>::zero();
            let mut b = Vector::<3>::zero();
            for (n, r) in rows {
                for i in 0..3 {
                    b[i] = b[i].add(n[i].mul(*r));
                    for j in 0..3 {
                        normal[(i, j)] = normal[(i, j)].add(n[i].mul(n[j]));
                    }
                }
            }
            solve_linear_system(&normal, &b)
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    /// A chain of `m` constraints, each between two neighbouring blocks of
    /// `block` variables, with entries that are neither zero nor alike, and
    /// each row's own variable the one that counts: the rows are independent.
    fn chain(m: usize, block: usize) -> (Vec<Vec<Scalar>>, usize) {
        let n = (m + 1) * block;
        let mut rows = Vec::new();
        for k in 0..m {
            for e in 0..block {
                let mut row = vec![Scalar::ZERO; n];
                for b in 0..block {
                    let own = if b == e { 10.0 } else { 0.0 };
                    row[k * block + b] = Scalar::from_f64(own + ((k + e + b) % 5) as f64 * 0.1);
                    row[(k + 1) * block + b] =
                        Scalar::from_f64(((k * 3 + e * 5 + b) % 7) as f64 * 0.1 - 0.3);
                }
                rows.push(row);
            }
        }
        (rows, n)
    }

    /// Keeping to the columns a row has something in finds every
    /// independent row, and a null space that is one — every row vanishes
    /// on each vector of it.
    #[test]
    fn a_sparse_elimination_finds_the_null_space() {
        let (rows, n) = chain(12, 6);
        assert_eq!(eliminate(rows.clone(), n).pivots.len(), rows.len());
        let basis = null_space(rows.clone(), n);
        assert_eq!(basis.len(), n - rows.len());
        for v in &basis {
            for row in &rows {
                let dot: f64 = row
                    .iter()
                    .zip(v)
                    .map(|(a, b)| a.to_f64() * b.to_f64())
                    .sum();
                assert!(dot.abs() < 1e-9, "a row is {dot} on a null vector");
            }
        }
    }

    /// A long chain of independent rows is all found.
    #[test]
    fn a_chain_has_its_rank() {
        let (rows, n) = chain(150, 6);
        let m = rows.len();
        assert_eq!(eliminate(rows, n).pivots.len(), m);
    }

    /// The inverse of a matrix with zeros in it: eliminated with an identity
    /// beside it, in rows longer than the variables they are pivoted on.
    #[test]
    fn the_inverse_of_a_sparse_matrix_is_one() {
        let a: Vec<Vec<Scalar>> = [[2.0, 0.0, 1.0], [0.0, 3.0, 0.0], [1.0, 0.0, 4.0]]
            .map(|row| row.map(Scalar::from_f64).to_vec())
            .to_vec();
        let inverse = inverse(&a).expect("the matrix is not singular");
        for (i, row) in a.iter().enumerate() {
            for j in 0..3 {
                let dot: f64 = (row.iter().zip(&inverse))
                    .map(|(a, inverse)| a.to_f64() * inverse[j].to_f64())
                    .sum();
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((dot - want).abs() < 1e-12, "({i}, {j}): {dot}");
            }
        }
    }
}

#[cfg(test)]
mod system_tests {
    use super::*;

    #[test]
    fn solves_identity() {
        let a = Matrix::from_rows([[Scalar::ONE, Scalar::ZERO], [Scalar::ZERO, Scalar::ONE]]);
        let b = Vector::<2>::from_array([Scalar::from_f64(3.0), Scalar::from_f64(4.0)]);
        let x = solve_linear_system(&a, &b).unwrap();
        assert!(x[0].could_be_equal(Scalar::from_f64(3.0)));
        assert!(x[1].could_be_equal(Scalar::from_f64(4.0)));
    }

    #[test]
    fn solves_general_2x2() {
        // [2 1; 1 3] x = [5; 10] -> x = [1, 3]
        let a = Matrix::from_rows([
            [Scalar::from_f64(2.0), Scalar::from_f64(1.0)],
            [Scalar::from_f64(1.0), Scalar::from_f64(3.0)],
        ]);
        let b = Vector::from_array([Scalar::from_f64(5.0), Scalar::from_f64(10.0)]);
        let x = solve_linear_system(&a, &b).unwrap();
        assert!(x[0].could_be_equal(Scalar::ONE));
        assert!(x[1].could_be_equal(Scalar::from_f64(3.0)));
    }

    #[test]
    fn singular_errs() {
        let a = Matrix::from_rows([[Scalar::ONE, Scalar::ONE], [Scalar::ONE, Scalar::ONE]]);
        let b = Vector::from_array([Scalar::ONE, Scalar::TWO]);
        assert!(solve_linear_system(&a, &b).is_err());
    }
}
