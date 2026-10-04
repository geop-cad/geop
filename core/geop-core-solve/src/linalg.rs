//! Dense linear algebra on Jacobians: which residuals are independent,
//! which variables they leave free, and inverses — in honest enclosures, so
//! that a pivot counts as zero exactly when it could be zero.

use geop_core_math::scalars::Scalar;

/// The outcome of Gauss-Jordan elimination (see [`eliminate`]).
pub(crate) struct Elimination<S: Scalar> {
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
/// [`crate::System::enclose`] verifies what it is used for.
///
/// Complete, not column by column: going column by column takes the first
/// column with any entry that cannot be zero, and rounding leaves entries
/// of `1e-16` that cannot. Such a pivot determines a variable the
/// constraints barely touch, in place of one they do — a half circle
/// tangent to two parallel sides was left with its sweep free and two
/// tangencies over the same coordinate, and could not be enclosed.
pub(crate) fn eliminate<S: Scalar>(mut rows: Vec<Vec<S>>, n: usize) -> Elimination<S> {
    let mut origin: Vec<usize> = (0..rows.len()).collect();
    let mut pivots: Vec<(usize, usize)> = Vec::new();
    let mut used = vec![false; n];
    let mut r = 0;
    loop {
        let size = |v: S| v.abs().sharpen();
        let mut best: Option<(usize, usize)> = None;
        for i in r..rows.len() {
            for col in (0..n).filter(|&c| !used[c]) {
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
        origin.swap(r, best);
        let pivot = rows[r][col];
        for v in &mut rows[r] {
            *v = v.div(pivot).expect("a pivot is definitely not zero");
        }
        for i in (0..rows.len()).filter(|&i| i != r) {
            let factor = rows[i][col];
            if factor.is_sharp() && factor.could_be_equal(S::ZERO) {
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

/// A basis of the null space of `rows` (each over `n` variables): the
/// directions in which every row stays put, to first order. One vector per
/// variable no pivot determines, which is one there and zero at every other
/// such variable.
pub(crate) fn null_space<S: Scalar>(rows: Vec<Vec<S>>, n: usize) -> Vec<Vec<S>> {
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
pub(crate) fn rank<S: Scalar>(rows: Vec<Vec<S>>, n: usize) -> usize {
    eliminate(rows, n).pivots.len()
}

/// Which of `n` variables can move to first order without changing any
/// residual (those with a component in the Jacobian's null space that is
/// definitely not zero), and the null space's dimension. This only
/// classifies entities for display — the solve itself does not depend on
/// it.
pub(crate) fn free_variables<S: Scalar>(rows: Vec<Vec<S>>, n: usize) -> (Vec<bool>, usize) {
    let basis = null_space(rows, n);
    let free = (0..n)
        .map(|k| basis.iter().any(|v| v[k].definitely_not_equal(S::ZERO)))
        .collect();
    (free, basis.len())
}

/// An approximate inverse of the square matrix `a`, by Gauss-Jordan
/// elimination: any matrix serves where it is only a preconditioner, as in
/// the Krawczyk test, so its entries are sharp. `None` if `a` could be
/// singular.
pub(crate) fn inverse<S: Scalar>(a: &[Vec<S>]) -> Option<Vec<Vec<S>>> {
    let m = a.len();
    let rows: Vec<Vec<S>> = a
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut row: Vec<S> = row.iter().map(|v| v.sharpen()).collect();
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
        inverse[col] = row[m..].iter().map(|v| v.sharpen()).collect();
    }
    Some(inverse)
}
