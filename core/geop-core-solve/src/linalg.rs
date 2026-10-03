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
}

/// Gauss-Jordan elimination of `rows` (each over `n` variables), each
/// column's pivot the entry of largest magnitude below the rows already
/// used — a free choice among those that cannot be zero; a column whose
/// every candidate could be zero has none. Which constraints are
/// independent and which variables they determine is decided from the
/// rows' own enclosures; where it matters for correctness,
/// [`crate::System::enclose`] verifies what it is used for.
pub(crate) fn eliminate<S: Scalar>(mut rows: Vec<Vec<S>>, n: usize) -> Elimination<S> {
    let mut origin: Vec<usize> = (0..rows.len()).collect();
    let mut pivots = Vec::new();
    let mut r = 0;
    for col in 0..n {
        let Some(best) = (r..rows.len())
            .filter(|&i| rows[i][col].definitely_not_equal(S::ZERO))
            .reduce(|a, b| {
                let size = |i: usize| rows[i][col].abs().sharpen();
                if size(b).definitely_greater(size(a)) {
                    b
                } else {
                    a
                }
            })
        else {
            continue;
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
        pivots.push((col, origin[r]));
        r += 1;
    }
    Elimination { rows, pivots }
}

/// Which of `n` variables can move to first order without changing any
/// residual (those with a component in the Jacobian's null space that is
/// definitely not zero), and the null space's dimension. This only
/// classifies entities for display — the solve itself does not depend on
/// it.
pub(crate) fn free_variables<S: Scalar>(rows: Vec<Vec<S>>, n: usize) -> (Vec<bool>, usize) {
    let Elimination { rows, pivots } = eliminate(rows, n);
    // Null space basis: one vector per non-pivot column `f`, with `v_f = 1`
    // and `v_{pivots[i].0} = -rows[i][f]`.
    let mut free = vec![false; n];
    let is_pivot: Vec<bool> = (0..n).map(|c| pivots.iter().any(|p| p.0 == c)).collect();
    for f in (0..n).filter(|&c| !is_pivot[c]) {
        free[f] = true;
        for (i, &(pc, _)) in pivots.iter().enumerate() {
            if rows[i][f].definitely_not_equal(S::ZERO) {
                free[pc] = true;
            }
        }
    }
    (free, n - pivots.len())
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
    let Elimination { rows, pivots } = eliminate(rows, m);
    if pivots.len() < m {
        return None;
    }
    let mut inverse = vec![Vec::new(); m];
    for (row, &(col, _)) in rows.into_iter().zip(&pivots) {
        inverse[col] = row[m..].iter().map(|v| v.sharpen()).collect();
    }
    Some(inverse)
}
