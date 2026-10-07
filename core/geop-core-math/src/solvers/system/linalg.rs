//! Dense linear algebra on Jacobians: which residuals are independent,
//! which variables they leave free, and inverses — in honest enclosures, so
//! that a pivot counts as zero exactly when it could be zero.

use crate::scalars::Scalar;

/// The outcome of Gauss-Jordan elimination (see [`eliminate`]).
pub(super) struct Elimination<S: Scalar> {
    /// The rows, reduced: row `k` has a one at `pivots[k].0` and zeros at
    /// every other pivot column.
    pub rows: Vec<Vec<S>>,
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
/// [`enclose`] verifies what it is used for.
///
/// Complete, not column by column: going column by column takes the first
/// column with any entry that cannot be zero, and rounding leaves entries
/// of `1e-16` that cannot. Such a pivot determines a variable the
/// constraints barely touch, in place of one they do — a half circle
/// tangent to two parallel sides was left with its sweep free and two
/// tangencies over the same coordinate, and could not be enclosed.
pub(super) fn eliminate<S: Scalar>(mut rows: Vec<Vec<S>>, n: usize) -> Elimination<S> {
    // Per row, the columns it has an entry in other than an exact zero, in
    // order — of the whole row, which may be longer than `n`: columns past
    // the pivots' go along, as in `inverse`'s: the only ones that can be a
    // pivot (among the first `n`), and the only ones a pivot row changes in
    // another row. The Jacobian of an assembly has a dozen
    // in a row of hundreds, and a pivot row, still, a few dozen.
    let mut support: Vec<Vec<usize>> = rows
        .iter()
        .map(|row| (0..row.len()).filter(|&c| !is_zero(row[c])).collect())
        .collect();
    let mut origin: Vec<usize> = (0..rows.len()).collect();
    let mut pivots: Vec<(usize, usize)> = Vec::new();
    let mut used = vec![false; n];
    let mut r = 0;
    loop {
        let size = |v: S| v.abs().lower();
        let mut best: Option<(usize, usize)> = None;
        for i in r..rows.len() {
            for &col in support[i].iter().filter(|&&c| c < n && !used[c]) {
                let v = rows[i][col];
                if v.definitely_not_equal(S::ZERO)
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
            if is_zero(factor) {
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

/// Whether `v` is definitely zero — no value it could be is less or more
/// than zero: the entry of a column a row has nothing in.
pub(super) fn is_zero<S: Scalar>(v: S) -> bool {
    !v.could_be_less(S::ZERO) && !v.could_be_greater(S::ZERO)
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
pub(super) fn null_space<S: Scalar>(rows: Vec<Vec<S>>, n: usize) -> Vec<Vec<S>> {
    let Elimination { rows, pivots, .. } = eliminate(rows, n);
    let is_pivot: Vec<bool> = (0..n).map(|c| pivots.iter().any(|p| p.0 == c)).collect();
    (0..n)
        .filter(|&f| !is_pivot[f])
        .map(|f| {
            let mut v = vec![S::ZERO; n];
            v[f] = S::ONE;
            for (i, &(pc, _)) in pivots.iter().enumerate() {
                v[pc] = rows[i][f].neg();
            }
            v
        })
        .collect()
}

/// How many of `rows` (each over `n` variables) are independent: those an
/// elimination finds a pivot that cannot be zero for.
pub fn rank<S: Scalar>(rows: Vec<Vec<S>>, n: usize) -> usize {
    eliminate(rows, n).pivots.len()
}

/// Which of `n` variables can move to first order without changing any
/// residual (those with a component in the Jacobian's null space that is
/// definitely not zero), and the null space's dimension. This only
/// classifies entities for display — the solve itself does not depend on
/// it.
pub(super) fn free_variables<S: Scalar>(rows: Vec<Vec<S>>, n: usize) -> (Vec<bool>, usize) {
    let basis = null_space(rows, n);
    let free = (0..n)
        .map(|k| basis.iter().any(|v| v[k].definitely_not_equal(S::ZERO)))
        .collect();
    (free, basis.len())
}

/// An enclosure of the inverse of the square matrix `a`, by Gauss-Jordan
/// elimination: each entry encloses that of the inverse of every matrix `a`
/// encloses. `None` if `a` could be singular.
pub(super) fn inverse<S: Scalar>(a: &[Vec<S>]) -> Option<Vec<Vec<S>>> {
    let m = a.len();
    let rows: Vec<Vec<S>> = a
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut row = row.clone();
            row.extend((0..m).map(|j| if i == j { S::ONE } else { S::ZERO }));
            row
        })
        .collect();
    let Elimination { rows, pivots, .. } = eliminate(rows, m);
    if pivots.len() < m {
        return None;
    }
    let mut inverse = vec![Vec::new(); m];
    for (row, &(col, _)) in rows.into_iter().zip(&pivots) {
        inverse[col] = row[m..].to_vec();
    }
    Some(inverse)
}

#[cfg(test)]
mod tests {
    use crate::scalars::{Field, Ring, ScalInF64 as S, Scalar};

    use super::*;

    /// The elimination as it was before it kept to the columns a row has
    /// something in: every column of every row, every time.
    fn dense(mut rows: Vec<Vec<S>>, n: usize) -> Elimination<S> {
        let mut origin: Vec<usize> = (0..rows.len()).collect();
        let mut pivots: Vec<(usize, usize)> = Vec::new();
        let mut used = vec![false; n];
        let mut r = 0;
        loop {
            let size = |v: S| v.abs().lower();
            let mut best: Option<(usize, usize)> = None;
            for i in r..rows.len() {
                for col in (0..n).filter(|&c| !used[c]) {
                    let v = rows[i][col];
                    if v.definitely_not_equal(S::ZERO)
                        && best
                            .is_none_or(|(bi, bc)| size(v).definitely_greater(size(rows[bi][bc])))
                    {
                        best = Some((i, col));
                    }
                }
            }
            let Some((best, col)) = best else { break };
            rows.swap(r, best);
            origin.swap(r, best);
            let pivot = rows[r][col];
            for v in &mut rows[r] {
                *v = (*v).div(pivot).unwrap();
            }
            for i in (0..rows.len()).filter(|&i| i != r) {
                let factor = rows[i][col];
                if is_zero(factor) {
                    continue;
                }
                for c in 0..rows[i].len() {
                    rows[i][c] = rows[i][c].sub(factor.mul(rows[r][c]));
                }
            }
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

    /// A chain of `m` constraints, each between two neighbouring blocks of
    /// `block` variables, with entries that are neither zero nor alike, and
    /// each row's own variable the one that counts: the rows are independent.
    fn chain(m: usize, block: usize) -> (Vec<Vec<S>>, usize) {
        let n = (m + 1) * block;
        let mut rows = Vec::new();
        for k in 0..m {
            for e in 0..block {
                let mut row = vec![S::ZERO; n];
                for b in 0..block {
                    let own = if b == e { 10.0 } else { 0.0 };
                    row[k * block + b] = S::from_f64(own + ((k + e + b) % 5) as f64 * 0.1);
                    row[(k + 1) * block + b] =
                        S::from_f64(((k * 3 + e * 5 + b) % 7) as f64 * 0.1 - 0.3);
                }
                rows.push(row);
            }
        }
        (rows, n)
    }

    /// Keeping to the columns a row has something in changes nothing but
    /// the work: as many independent rows as the dense elimination finds,
    /// and a null space that is one — every row vanishes on each vector of
    /// it. (Which of two pivots that are nearly as large is taken may
    /// differ: the dense one widens the entries it adds a zero to.)
    #[test]
    fn a_sparse_elimination_finds_what_the_dense_one_does() {
        let (rows, n) = chain(12, 6);
        let sparse = eliminate(rows.clone(), n);
        let dense = dense(rows.clone(), n);
        assert_eq!(sparse.pivots.len(), dense.pivots.len());
        assert_eq!(sparse.pivots.len(), rows.len());
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
        let a: Vec<Vec<S>> = [[2.0, 0.0, 1.0], [0.0, 3.0, 0.0], [1.0, 0.0, 4.0]]
            .map(|row| row.map(S::from_f64).to_vec())
            .to_vec();
        let inverse = inverse(&a).expect("the matrix is not singular");
        for i in 0..3 {
            for j in 0..3 {
                let dot: f64 = (0..3)
                    .map(|k| a[i][k].to_f64() * inverse[k][j].to_f64())
                    .sum();
                let want = if i == j { 1.0 } else { 0.0 };
                assert!((dot - want).abs() < 1e-12, "({i}, {j}): {dot}");
            }
        }
    }
}
