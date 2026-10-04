//! Levenberg–Marquardt, constrained: minimizing a sum of squared residuals
//! `Σ r(x)²` among the points where other residuals `c(x)` vanish, from
//! their values and Jacobians.
//!
//! Each step solves the problem linearized at the current point — minimize
//! `|r + J_r δ|² + δᵀ(B + λD)δ` subject to `c + J_c δ = 0`, or as near to it
//! as first order gets — by the null-space method. `D` is
//! Marquardt's scaling, the diagonal of the Jacobians' normal matrix, and
//! `λ` shrinks after every step taken and grows after every one turned down:
//! Gauss–Newton near a solution, gradient descent far from one. `B` is what
//! Gauss–Newton leaves out — the curvature of the constraints, weighted by
//! how hard the sum pulls against them — estimated from the steps taken (a
//! BFGS update, along the constraints only): without it, wherever the sum
//! cannot reach zero against curved constraints, steps along them overshoot
//! and the minimizer zig-zags. Where a step it proposed is turned down, the
//! step is tried again without it, and it is learned afresh if that does
//! better — a configuration passing through a singular one changes it
//! abruptly.
//!
//! Keeping `c` exact, rather than adding it to the sum with a large weight,
//! is what makes the sum a pure preference: it can never buy itself a
//! little lower with a little violation of `c` — which, wherever a long
//! lever multiplies that violation, is a lot of motion.
//!
//! Nor are the two weighed against each other to judge a step. A step is
//! taken where it definitely lowers either the sum or the constraints'
//! violation `|c|` — against the point it starts from and against every
//! point taken before (a filter), so the two cannot trade back and forth
//! forever. A merit `Σ r² + ρ |c|` would need `ρ` above the multipliers the
//! sum pulls against the constraints with, and those grow without bound at
//! a singular configuration — two links in line, say — where the merit's
//! own width then hides every step that does reduce the violation.
//!
//! A variable no residual depends on is never moved, and a step never has a
//! component along a direction no residual depends on — unlike a
//! quasi-Newton method on the sum alone, whose curvature estimate grows
//! without bound along such a direction until rounding noise in the
//! gradient throws it arbitrarily far.
//!
//! Every value is an honest enclosure. The steps themselves are free
//! choices — any step is a valid seed for the next — so they are sharpened;
//! whether a step is taken is not, and it is taken only where it
//! *definitely* improves. So the minimizer needs no tolerance: it stops where
//! no step can definitely improve on where it is, which is as near the
//! minimum as the residuals' own uncertainty can tell.

use crate::{geop_error::GeopResult, scalars::Scalar};

/// How long [`minimize`] may try, and how far a step may go.
#[derive(Clone, Copy, Debug)]
pub struct Options<S: Scalar> {
    /// Steps taken at most: how hard it tries, never what an answer means.
    pub max_iterations: usize,
    /// The longest step: where the variables have a natural size, no step
    /// longer than it is sensible.
    pub max_step: S,
}

#[derive(Clone, Debug)]
pub struct Outcome<S: Scalar> {
    pub x: Vec<S>,
    pub iterations: usize,
    pub stop: Stop,
    /// How damped the last step was (`λ`, relative to each variable's
    /// curvature).
    pub damping: S,
}

/// Why [`minimize`] stopped where it did.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Stop {
    /// The residuals cannot be evaluated at the start.
    Infeasible,
    /// The next step changes no variable at all — after the steps before it
    /// were turned down, the last one for `rejected`.
    NoChange { rejected: Option<Rejection> },
    /// Neither the sum's change nor the violation's can be told from their
    /// width, and the step no longer shrinks: as near the minimum as can be
    /// told.
    Undecided,
    /// The linearized model expects the next step to lower neither the sum
    /// nor the violation definitely — with `independent` of the
    /// `constraints` rows telling it something new.
    Flat {
        independent: usize,
        constraints: usize,
    },
    /// No step, however damped, is taken; the last one tried turned down
    /// for `rejected`.
    Damped { rejected: Option<Rejection> },
    /// [`Options::max_iterations`] steps were taken.
    Budget,
}

/// Why a step was turned down.
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Rejection {
    /// The damped model's curvature is not positive.
    NotPositive,
    /// The residuals cannot be evaluated where it leads.
    Infeasible,
    /// Cut so short that the linearized model cannot tell it from no step.
    Unresolved,
    /// Where it leads, neither the sum nor the violation is definitely
    /// lower than at every point taken before.
    NotLower,
}

/// Residuals, and their Jacobian row by row.
#[derive(Clone, Debug)]
pub struct Rows<S: Scalar> {
    pub values: Vec<S>,
    pub jacobian: Vec<Vec<S>>,
}

impl<S: Scalar> Default for Rows<S> {
    fn default() -> Self {
        Rows {
            values: Vec::new(),
            jacobian: Vec::new(),
        }
    }
}

impl<S: Scalar> Rows<S> {
    pub fn push(&mut self, value: S, row: Vec<S>) {
        self.values.push(value);
        self.jacobian.push(row);
    }

    /// `|values + jacobian δ|²`: at `δ = 0`, the sum of squares.
    fn squared(&self, delta: &[S]) -> S {
        self.values
            .iter()
            .zip(&self.jacobian)
            .fold(S::ZERO, |sum, (&v, row)| {
                let r = v.add(dot(row, delta));
                sum.add(r.mul(r))
            })
    }

    /// `jacobianᵀ w`.
    fn transposed(&self, w: &[S], n: usize) -> Vec<S> {
        let mut g = vec![S::ZERO; n];
        for (&wi, row) in w.iter().zip(&self.jacobian) {
            for (gi, &ri) in g.iter_mut().zip(row) {
                *gi = gi.add(ri.mul(wi));
            }
        }
        g
    }
}

/// The residuals at a point: those that must vanish, and those whose
/// squares are minimized.
#[derive(Clone, Debug)]
pub struct Evaluation<S: Scalar> {
    pub constraints: Rows<S>,
    pub sum: Rows<S>,
}

impl<S: Scalar> Default for Evaluation<S> {
    fn default() -> Self {
        Evaluation {
            constraints: Rows::default(),
            sum: Rows::default(),
        }
    }
}

/// By how much `λ` shrinks after a step taken, and grows after one not.
const SHRINK: i64 = 3;
const GROW: i64 = 4;

fn dot<S: Scalar>(a: &[S], b: &[S]) -> S {
    a.iter()
        .zip(b)
        .fold(S::ZERO, |sum, (&x, &y)| sum.add(x.mul(y)))
}

fn norm<S: Scalar>(v: &[S]) -> GeopResult<S> {
    dot(v, v).sqrt()
}

/// A value of the model a step is computed from — a slope, a residual, a
/// curvature, the step itself — chosen from its enclosure: any value in it
/// serves (see the module docs), so zero where zero is among them, else
/// the middle. A slope that could be zero is rounding around an exact
/// zero, and turned into a small number it would couple what does not
/// depend on each other — a coupling a weakly held variable then follows
/// arbitrarily far.
fn choose<S: Scalar>(v: S) -> S {
    if v.could_be_equal(S::ZERO) {
        S::ZERO
    } else {
        v.sharpen()
    }
}

fn sharp<S: Scalar>(v: &[S]) -> Vec<S> {
    v.iter().map(|&x| choose(x)).collect()
}

/// The Cholesky factor `L` of the symmetric `a`, `a = L Lᵀ` — leaving out
/// every row whose pivot could be zero or less: one that, in the light of
/// the rows before it, says nothing new (or nothing positive). Which rows
/// were left out is `true` in the second.
fn cholesky<S: Scalar>(a: &[Vec<S>]) -> GeopResult<(Vec<Vec<S>>, Vec<bool>)> {
    let n = a.len();
    let mut l = vec![vec![S::ZERO; n]; n];
    let mut left_out = vec![false; n];
    for i in 0..n {
        for j in 0..=i {
            if left_out[j] {
                continue;
            }
            let sum = (0..j).fold(a[i][j], |sum, k| sum.sub(l[i][k].mul(l[j][k])));
            if i == j {
                if sum.definitely_greater(S::ZERO) {
                    l[i][i] = sum.sqrt()?;
                } else {
                    left_out[i] = true;
                    l[i].iter_mut().for_each(|v| *v = S::ZERO);
                }
            } else {
                l[i][j] = sum.div(l[j][j])?;
            }
        }
    }
    Ok((l, left_out))
}

/// The solution of `L Lᵀ x = b`, zero at the rows `left_out`.
fn solve_cholesky<S: Scalar>(l: &[Vec<S>], left_out: &[bool], b: &[S]) -> GeopResult<Vec<S>> {
    let n = b.len();
    let mut y = vec![S::ZERO; n];
    for i in (0..n).filter(|&i| !left_out[i]) {
        let sum = (0..i).fold(b[i], |sum, k| sum.sub(l[i][k].mul(y[k])));
        y[i] = sum.div(l[i][i])?;
    }
    let mut x = vec![S::ZERO; n];
    for i in (0..n).rev().filter(|&i| !left_out[i]) {
        let sum = (i + 1..n).fold(y[i], |sum, k| sum.sub(l[k][i].mul(x[k])));
        x[i] = sum.div(l[i][i])?;
    }
    Ok(x)
}

/// A step of the linearized problem (see [`step`]).
struct Step<S: Scalar> {
    delta: Vec<S>,
    /// The constraints' multipliers.
    lambda: Vec<S>,
    /// How many of the constraints' rows say something the others do not.
    independent: usize,
}

/// `v - a w`, chosen (see [`choose`]).
fn minus_scaled<S: Scalar>(v: &[S], a: S, w: &[S]) -> Vec<S> {
    v.iter()
        .zip(w)
        .map(|(&x, &y)| choose(x.sub(a.mul(y))))
        .collect()
}

/// The step of the problem linearized at `e`, with curvature `b` and
/// damping `mu`, sharpened — and the constraints' multipliers. `None` if
/// the damped curvature is not positive along the directions the
/// constraints leave free, among the variables `moving`.
///
/// By the null-space method: an orthonormal basis `Q` of the constraints'
/// rows, and one `Z` of the directions they leave free. The step is the
/// shortest one meeting the linearized constraints, `δ_c = Q y` with
/// `R y = -c` for the rows' coefficients `R` in `Q` — no curvature in it at
/// all — plus the one along `Z` that minimizes the model there, from the
/// curvature reduced to those directions, `Zᵀ H Z`. Unlike a Schur
/// complement `J_c H⁻¹ J_cᵀ`, nothing inverts `H` along a direction it
/// barely curves in — a variable only the damping holds, say, which the
/// constraints pin — where the inverse is huge and the complement loses as
/// many digits as it is large.
///
/// A row says nothing the rows before it do not where what is left of it,
/// after taking out its component along their basis, could be zero — asked
/// of the row's honest enclosure: a question about the constraints, not a
/// choice. Everything else is the model's own choice, so sharp.
fn step<S: Scalar>(
    e: &Evaluation<S>,
    b: &[Vec<S>],
    moving: &[bool],
    mu: S,
) -> GeopResult<Option<Step<S>>> {
    let n = moving.len();
    let keep: Vec<usize> = (0..n).filter(|&i| moving[i]).collect();
    let k = keep.len();
    let squeeze = |v: &[S]| -> Vec<S> { keep.iter().map(|&i| v[i]).collect() };
    let jr: Vec<Vec<S>> = e.sum.jacobian.iter().map(|r| sharp(&squeeze(r))).collect();
    let jc_wide: Vec<Vec<S>> = e.constraints.jacobian.iter().map(|r| squeeze(r)).collect();
    let jc: Vec<Vec<S>> = jc_wide.iter().map(|r| sharp(r)).collect();
    let c = sharp(&e.constraints.values);
    let m = jc.len();

    // H = J_rᵀJ_r + B + λ D, over the variables that move.
    let mut h: Vec<Vec<S>> = keep.iter().map(|&i| squeeze(&b[i])).collect();
    for row in &jr {
        for i in 0..k {
            for j in 0..k {
                h[i][j] = h[i][j].add(row[i].mul(row[j]));
            }
        }
    }
    for (i, row) in h.iter_mut().enumerate() {
        let d = jr
            .iter()
            .chain(&jc)
            .fold(S::ZERO, |d, r| d.add(r[i].mul(r[i])));
        row[i] = row[i].add(mu.mul(d));
        row.iter_mut().for_each(|v| *v = choose(*v));
    }

    // Q, and each independent row's coefficients in it (modified
    // Gram–Schmidt): row `i` is `Σ_j R[i][j] q_j`, `R[i]` ending in its own
    // `q`'s.
    // Whether a row is independent is asked honestly: what is left of its
    // enclosure outside the span of the rows before — their sharp basis, not
    // exactly orthonormal, so projected onto through its Gram matrix `QᵀQ`
    // (enclosed, and close to the identity, so the projection stays tight).
    // What a sharp basis leaves of a row that depends on the others is
    // rounding, not zero; what this leaves of it could be zero.
    let mut q: Vec<Vec<S>> = Vec::new();
    // The Cholesky factor of the Gram matrix, row by row.
    let mut gram: Vec<Vec<S>> = Vec::new();
    let mut rows: Vec<(usize, Vec<S>)> = Vec::new();
    let mut dependent: Vec<(usize, Vec<S>)> = Vec::new();
    for i in 0..m {
        let r = q.len();
        let b: Vec<S> = q.iter().map(|qj| dot(&jc_wide[i], qj)).collect();
        let mut a = vec![S::ZERO; r];
        for j in 0..r {
            a[j] = (0..j)
                .fold(b[j], |sum, k| sum.sub(gram[j][k].mul(a[k])))
                .div(gram[j][j])?;
        }
        for j in (0..r).rev() {
            a[j] = (j + 1..r)
                .fold(a[j], |sum, k| sum.sub(gram[k][j].mul(a[k])))
                .div(gram[j][j])?;
        }
        let outside: Vec<S> = (0..k)
            .map(|p| (0..r).fold(jc_wide[i][p], |sum, j| sum.sub(a[j].mul(q[j][p]))))
            .collect();
        if !norm(&outside)?.definitely_greater(S::ZERO) {
            // Its coefficients in the basis, for the least-squares
            // correction below.
            dependent.push((i, a.iter().map(|&x| choose(x)).collect()));
            continue;
        }
        let mut v = jc[i].clone();
        let mut coefficients = Vec::new();
        for qj in &q {
            let a = choose(dot(&v, qj));
            v = minus_scaled(&v, a, qj);
            coefficients.push(a);
        }
        let length = norm(&v)?.sharpen();
        coefficients.push(length);
        let q_new: Vec<S> = v
            .iter()
            .map(|&x| choose(x.div(length).unwrap_or(S::ZERO)))
            .collect();
        let mut row = vec![S::ZERO; r + 1];
        for j in 0..r {
            row[j] = (0..j)
                .fold(dot(&q_new, &q[j]), |sum, k| sum.sub(row[k].mul(gram[j][k])))
                .div(gram[j][j])?;
        }
        row[r] = (0..r)
            .fold(dot(&q_new, &q_new), |sum, k| sum.sub(row[k].mul(row[k])))
            .sqrt()?;
        gram.push(row);
        q.push(q_new);
        rows.push((i, coefficients));
    }
    let r = q.len();

    // δ_c = Q y, the least-squares correction of every row — `|c + J_c Q
    // y|` least — by the normal equations of the rows' coefficients in Q:
    // exactly `R y = -c` where the dependent rows agree with the others, and
    // as near as first order gets where they do not.
    let coefficients_of = |coefficients: &[S]| -> Vec<S> {
        (0..r)
            .map(|j| coefficients.get(j).copied().unwrap_or(S::ZERO))
            .collect()
    };
    let all: Vec<(usize, Vec<S>)> = rows
        .iter()
        .chain(&dependent)
        .map(|(i, coefficients)| (*i, coefficients_of(coefficients)))
        .collect();
    let normal: Vec<Vec<S>> = (0..r)
        .map(|a| {
            (0..r)
                .map(|b| {
                    choose(
                        all.iter()
                            .fold(S::ZERO, |sum, (_, row)| sum.add(row[a].mul(row[b]))),
                    )
                })
                .collect()
        })
        .collect();
    let rhs_c: Vec<S> = (0..r)
        .map(|a| {
            choose(
                all.iter()
                    .fold(S::ZERO, |sum, (i, row)| sum.sub(row[a].mul(c[*i]))),
            )
        })
        .collect();
    let (ln, singular) = cholesky(&normal)?;
    let y = sharp(&solve_cholesky(&ln, &singular, &rhs_c)?);
    let mut delta_c = vec![S::ZERO; k];
    for (t, qt) in q.iter().enumerate() {
        delta_c = minus_scaled(&delta_c, y[t].neg(), qt);
    }

    // Z: the unit directions with what is left of them after Q, taken one
    // at a time by the largest remainder — orthonormalized against Q and
    // each other.
    let mut left: Vec<Vec<S>> = (0..k)
        .map(|p| {
            let mut v = vec![S::ZERO; k];
            v[p] = S::ONE;
            for qt in &q {
                v = minus_scaled(&v, choose(dot(&v, qt)), qt);
            }
            v
        })
        .collect();
    let mut z: Vec<Vec<S>> = Vec::new();
    while z.len() < k - r {
        let lengths: Vec<S> = left.iter().map(|v| dot(v, v)).collect();
        let Some(best) = (0..k).reduce(|a, b| {
            if lengths[b].definitely_greater(lengths[a]) {
                b
            } else {
                a
            }
        }) else {
            break;
        };
        let length = lengths[best].sqrt()?.sharpen();
        if !length.definitely_greater(S::ZERO) {
            break;
        }
        let zt: Vec<S> = left[best]
            .iter()
            .map(|&x| choose(x.div(length).unwrap_or(S::ZERO)))
            .collect();
        for v in &mut left {
            *v = minus_scaled(v, choose(dot(v, &zt)), &zt);
        }
        z.push(zt);
    }

    // Along Z: (Zᵀ H Z) w = -Zᵀ (g + H δ_c).
    let g = sharp(&squeeze(&e.sum.transposed(&sharp(&e.sum.values), n)));
    let h_times = |v: &[S]| -> Vec<S> { h.iter().map(|row| choose(dot(row, v))).collect() };
    let force: Vec<S> = g
        .iter()
        .zip(h_times(&delta_c))
        .map(|(&a, b)| choose(a.add(b)))
        .collect();
    let hz: Vec<Vec<S>> = z.iter().map(|zt| h_times(zt)).collect();
    let reduced: Vec<Vec<S>> = z
        .iter()
        .map(|zi| hz.iter().map(|hzj| choose(dot(zi, hzj))).collect())
        .collect();
    let (l, not_positive) = cholesky(&reduced)?;
    if not_positive.contains(&true) {
        return Ok(None);
    }
    let rhs: Vec<S> = z.iter().map(|zt| choose(dot(zt, &force).neg())).collect();
    let w = sharp(&solve_cholesky(&l, &not_positive, &rhs)?);
    let mut kept = delta_c;
    for (t, zt) in z.iter().enumerate() {
        kept = minus_scaled(&kept, w[t].neg(), zt);
    }

    // The multipliers: J_cᵀ λ = -(g + H δ), R ᵀ λ = Qᵀ (-(g + H δ)).
    let pull: Vec<S> = g
        .iter()
        .zip(h_times(&kept))
        .map(|(&a, b)| choose(a.add(b).neg()))
        .collect();
    // A curvature positive by so little that the step it solves for, or
    // the force along it, is no finite number is as good as not positive.
    if !kept.iter().chain(&pull).all(|v| v.is_finite()) {
        return Ok(None);
    }
    let mut lambda_rows = vec![S::ZERO; r];
    for t in (0..r).rev() {
        let known = (t + 1..r).fold(dot(&q[t], &pull), |sum, s| {
            sum.sub(rows[s].1[t].mul(lambda_rows[s]))
        });
        lambda_rows[t] = choose(known.div(rows[t].1[t])?);
    }
    let mut lambda = vec![S::ZERO; m];
    for (t, (i, _)) in rows.iter().enumerate() {
        lambda[*i] = lambda_rows[t];
    }
    let mut delta = vec![S::ZERO; n];
    for (p, &i) in keep.iter().enumerate() {
        delta[i] = kept[p];
    }
    Ok(Some(Step {
        delta,
        lambda,
        independent: r,
    }))
}

/// Minimizes `Σ r(x)²` from `x0` among the points where `c(x) = 0`, with
/// `evaluate` giving both and their Jacobians; `None` where they cannot be
/// evaluated, which makes the point infeasible. Where the constraints
/// cannot all hold, it ends where they come closest. Always returns the
/// best point found; whether it is good enough is the caller's question.
pub fn minimize<S: Scalar>(
    evaluate: impl Fn(&[S]) -> Option<Evaluation<S>>,
    x0: Vec<S>,
    options: Options<S>,
) -> GeopResult<Outcome<S>> {
    let n = x0.len();
    let mut x = x0;
    let Some(mut e) = evaluate(&x) else {
        return Ok(Outcome {
            x,
            iterations: 0,
            stop: Stop::Infeasible,
            damping: S::ZERO,
        });
    };
    // A variable moves only if some residual definitely depends on it: a
    // slope that could be zero — rounding around an exact zero — says
    // nothing about which way to move it, only how far a step along it
    // could go unchecked.
    let moving: Vec<bool> = (0..n)
        .map(|i| {
            e.sum
                .jacobian
                .iter()
                .chain(&e.constraints.jacobian)
                .any(|row| row[i].definitely_not_equal(S::ZERO))
        })
        .collect();
    // The sum and the violation at every point taken: a step must
    // definitely improve on each in one of the two.
    let mut filter: Vec<(S, S)> = Vec::new();
    let mut mu = S::from_ratio(1, 1000)?;
    let mut b = vec![vec![S::ZERO; n]; n];
    // The length of the last step taken.
    let mut last: Option<S> = None;
    // How far a step may go: what bounds a step along the constraints, which
    // `λ` does not damp — their linearization holds only so far.
    let mut radius = options.max_step;
    for iteration in 0..options.max_iterations {
        let (f, theta) = (e.sum.squared(&[]), norm(&e.constraints.values)?);
        let mut first = true;
        let mut rejected = None;
        // Whether this iteration's steps leave out the curvature learned
        // (see below).
        let mut plain = false;
        let no_curvature = vec![vec![S::ZERO; n]; n];
        let accepted = loop {
            let curvature = if plain { &no_curvature } else { &b };
            if let Some(Step {
                mut delta,
                lambda,
                independent,
            }) = step(&e, curvature, &moving, mu)?
            {
                // What the linearized model expects of a step: where its own
                // step expects no definite drop in either, no smaller step
                // makes one — as near the minimum as can be told, or, damped
                // already, no step helps — and no point is evaluated to
                // learn it.
                let expects = |delta: &[S]| -> GeopResult<bool> {
                    Ok(f.sub(e.sum.squared(delta)).definitely_greater(S::ZERO)
                        || theta
                            .sub(e.constraints.squared(delta).sqrt()?)
                            .definitely_greater(S::ZERO))
                };
                if !expects(&delta)? {
                    return Ok(Outcome {
                        x,
                        iterations: iteration,
                        stop: if first {
                            Stop::Flat {
                                independent,
                                constraints: e.constraints.values.len(),
                            }
                        } else {
                            Stop::Damped { rejected }
                        },
                        damping: mu,
                    });
                }
                let length = norm(&delta)?;
                if length.definitely_greater(radius) {
                    let shrink = radius.div(length)?;
                    delta.iter_mut().for_each(|d| *d = choose(d.mul(shrink)));
                }
                // Cut so short that not even the model can tell it from no
                // step at all, the cut is below what the arithmetic resolves:
                // it is lifted again.
                if !expects(&delta)? {
                    radius = options.max_step;
                    rejected = Some(Rejection::Unresolved);
                } else {
                    let next: Vec<S> = x
                        .iter()
                        .zip(&delta)
                        .map(|(a, d)| a.add(*d).sharpen())
                        .collect();
                    // Where not even this step changes anything, no smaller one
                    // will: nothing improves on here.
                    if next.iter().zip(&x).all(|(a, b)| a.could_be_equal(*b)) {
                        return Ok(Outcome {
                            x,
                            iterations: iteration,
                            stop: Stop::NoChange { rejected },
                            damping: mu,
                        });
                    }
                    let s: Vec<S> = next
                        .iter()
                        .zip(&x)
                        .map(|(a, b)| a.sub(*b).sharpen())
                        .collect();
                    let length = norm(&s)?;
                    if let Some(e_next) = evaluate(&next) {
                        // `Σ r² - Σ r'²` summed as `Σ (r - r')(r + r')`: near
                        // the minimum of a sum that stays large, the difference
                        // of the two sums is lost to their width.
                        let lower_sum = e
                            .sum
                            .values
                            .iter()
                            .zip(&e_next.sum.values)
                            .fold(S::ZERO, |sum, (&r, &r_next)| {
                                sum.add(r.sub(r_next).mul(r.add(r_next)))
                            });
                        let theta_next = norm(&e_next.constraints.values)?;
                        let lower_violation = theta.sub(theta_next);
                        let f_next = e_next.sum.squared(&[]);
                        // Far from the minimum, a step is taken where it
                        // definitely improves on here, and is not worse in both
                        // than any point taken before. Near it, both changes
                        // drown in their own width, and what still tells
                        // progress is Newton's: a step at most half the last one
                        // taken contracts towards a solution — until rounding
                        // stops it shrinking, which is where it ends. Shrinking
                        // any less is no contraction: steps that only barely
                        // shrink add up to any distance at all.
                        let improves = lower_sum.definitely_greater(S::ZERO)
                            || lower_violation.definitely_greater(S::ZERO);
                        let lower = improves
                            && filter.iter().all(|&(f_taken, theta_taken)| {
                                f_next.definitely_less(f_taken)
                                    || theta_next.definitely_less(theta_taken)
                            });
                        let undecided = [lower_sum, lower_violation].iter().all(|change| {
                            !change.definitely_greater(S::ZERO) && !change.definitely_less(S::ZERO)
                        });
                        let contracting =
                            last.is_some_and(|last| S::TWO.mul(length).definitely_less(last));
                        if first && undecided && last.is_some() && !contracting {
                            return Ok(Outcome {
                                x,
                                iterations: iteration,
                                stop: Stop::Undecided,
                                damping: mu,
                            });
                        }
                        if lower || (first && undecided && contracting) {
                            mu = mu.div(S::from_i64(SHRINK))?.sharpen();
                            let reach = S::TWO.mul(length);
                            if reach.definitely_greater(radius) {
                                radius = if reach.definitely_less(options.max_step) {
                                    reach.upper()
                                } else {
                                    options.max_step
                                };
                            }
                            // Without the curvature learned, where that did better:
                            // it is learned afresh from here.
                            if plain {
                                b.iter_mut().flatten().for_each(|v| *v = S::ZERO);
                            }
                            update_curvature(&mut b, &s, &e, &e_next, &lambda)?;
                            filter.push((f, theta));
                            last = Some(length);
                            break Some((next, e_next));
                        }
                        rejected = Some(Rejection::NotLower);
                        // The curvature learned from earlier steps proposed this
                        // one. Where it fails, that curvature may no longer
                        // hold — a configuration passing through a singular
                        // one changes it abruptly — so the step is first tried
                        // again without it, as it stands, before damping it.
                        if !plain
                            && b.iter()
                                .flatten()
                                .any(|v| !v.is_sharp() || v.definitely_not_equal(S::ZERO))
                        {
                            plain = true;
                            continue;
                        }
                    } else {
                        rejected = Some(Rejection::Infeasible);
                    }
                    radius = length.div(S::from_i64(GROW))?.sharpen();
                }
            } else {
                rejected = Some(Rejection::NotPositive);
            }
            first = false;
            mu = mu.mul(S::from_i64(GROW)).sharpen();
            if !mu.is_finite() {
                break None;
            }
        };
        let Some((next, e_next)) = accepted else {
            return Ok(Outcome {
                x,
                iterations: iteration,
                stop: Stop::Damped { rejected },
                damping: mu,
            });
        };
        (x, e) = (next, e_next);
    }
    Ok(Outcome {
        x,
        iterations: options.max_iterations,
        stop: Stop::Budget,
        damping: mu,
    })
}

/// The part of `v` along the constraints — orthogonal to the rows of their
/// Jacobian, `v - J_cᵀ (J_c J_cᵀ)⁻¹ J_c v` — chosen (see [`choose`]); rows
/// that say nothing the others do not left out.
fn along<S: Scalar>(constraints: &Rows<S>, v: &[S]) -> GeopResult<Vec<S>> {
    let jc: Vec<Vec<S>> = constraints.jacobian.iter().map(|r| sharp(r)).collect();
    let gram: Vec<Vec<S>> = jc
        .iter()
        .map(|a| jc.iter().map(|b| dot(a, b)).collect())
        .collect();
    let (l, dependent) = cholesky(&gram)?;
    let rhs: Vec<S> = jc.iter().map(|row| dot(row, v)).collect();
    let k = solve_cholesky(&l, &dependent, &rhs)?;
    Ok((0..v.len())
        .map(|i| choose((0..jc.len()).fold(v[i], |sum, r| sum.sub(jc[r][i].mul(k[r])))))
        .collect())
}

/// Updates `b`, the curvature Gauss–Newton leaves out, from the step `s`
/// from `e` to `e_next`: by how much the gradient of the Lagrangian
/// `½ Σ r² + λᵀc` turned along it, beyond what `J_rᵀJ_r` explains (a
/// structured BFGS update). Only where it curves up along `s` — a model
/// that curves down would step towards a maximum.
fn update_curvature<S: Scalar>(
    b: &mut [Vec<S>],
    s: &[S],
    e: &Evaluation<S>,
    e_next: &Evaluation<S>,
    lambda: &[S],
) -> GeopResult<()> {
    let n = s.len();
    let r_next = sharp(&e_next.sum.values);
    let constraints_next = e_next.constraints.transposed(lambda, n);
    let constraints_before = e.constraints.transposed(lambda, n);
    let sum_next = e_next.sum.transposed(&r_next, n);
    let sum_before = e.sum.transposed(&r_next, n);
    let y: Vec<S> = (0..n)
        .map(|i| {
            constraints_next[i]
                .sub(constraints_before[i])
                .add(sum_next[i])
                .sub(sum_before[i])
        })
        .collect();
    // Only along the constraints: off them, the constraints decide where a
    // step goes, so a curvature there means nothing — and a large one, along
    // a variable they hold outright, would only skew their solve.
    let (s, y) = (along(&e.constraints, s)?, along(&e.constraints, &y)?);
    let s = &s[..];
    let sy = dot(s, &y);
    if !sy.definitely_greater(S::ZERO) {
        return Ok(());
    }
    let bs: Vec<S> = (0..n).map(|i| dot(&b[i], s)).collect();
    let sbs = dot(s, &bs);
    for i in 0..n {
        for j in 0..n {
            let mut v = b[i][j].add(y[i].mul(y[j]).div(sy)?);
            if sbs.definitely_greater(S::ZERO) {
                v = v.sub(bs[i].mul(bs[j]).div(sbs)?);
            }
            b[i][j] = choose(v);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scalars::{Field, Ring, ScalInF64};

    type S = ScalInF64;

    fn s(v: f64) -> S {
        S::from_f64(v)
    }

    fn options() -> Options<S> {
        Options {
            max_iterations: 500,
            max_step: s(10.0),
        }
    }

    fn rows(values: Vec<S>, jacobian: Vec<Vec<S>>) -> Rows<S> {
        Rows { values, jacobian }
    }

    fn at(outcome: &Outcome<S>) -> Vec<f64> {
        outcome.x.iter().map(|v| v.to_f64()).collect()
    }

    /// Rosenbrock, as residuals `(1 - a, 10 (b - a²))`.
    #[test]
    fn minimizes_rosenbrock() {
        let outcome = minimize(
            |x: &[S]| {
                let (a, b) = (x[0], x[1]);
                Some(Evaluation {
                    constraints: Rows::default(),
                    sum: rows(
                        vec![S::ONE.sub(a), s(10.0).mul(b.sub(a.mul(a)))],
                        vec![vec![s(-1.0), S::ZERO], vec![s(-20.0).mul(a), s(10.0)]],
                    ),
                })
            },
            vec![s(-1.2), s(1.0)],
            options(),
        )
        .unwrap();
        let x = at(&outcome);
        assert!(
            (x[0] - 1.0).abs() < 1e-12 && (x[1] - 1.0).abs() < 1e-12,
            "{outcome:?}"
        );
    }

    /// A variable no residual depends on is not moved at all.
    #[test]
    fn free_variables_stay_put() {
        let outcome = minimize(
            |x: &[S]| {
                Some(Evaluation {
                    constraints: rows(vec![x[0].sub(s(2.0))], vec![vec![S::ONE, S::ZERO]]),
                    sum: Rows::default(),
                })
            },
            vec![s(0.0), s(0.7)],
            options(),
        )
        .unwrap();
        let x = at(&outcome);
        assert!((x[0] - 2.0).abs() < 1e-15, "{outcome:?}");
        assert_eq!(x[1], 0.7);
    }

    /// The point of the unit circle nearest `(3, 4)` — the sum pulling, the
    /// circle holding exactly, however hard the sum pulls — found quickly:
    /// the constraint's curvature is learned, not stumbled over.
    #[test]
    fn minimizes_on_a_circle() {
        let outcome = minimize(
            |x: &[S]| {
                let (a, b) = (x[0], x[1]);
                Some(Evaluation {
                    constraints: rows(
                        vec![a.mul(a).add(b.mul(b)).sub(S::ONE)],
                        vec![vec![S::TWO.mul(a), S::TWO.mul(b)]],
                    ),
                    sum: rows(
                        vec![a.sub(s(3.0)), b.sub(s(4.0))],
                        vec![vec![S::ONE, S::ZERO], vec![S::ZERO, S::ONE]],
                    ),
                })
            },
            vec![s(1.0), s(0.0)],
            options(),
        )
        .unwrap();
        let x = at(&outcome);
        assert!(
            (x[0] - 0.6).abs() < 1e-12 && (x[1] - 0.8).abs() < 1e-12,
            "{outcome:?}"
        );
        assert!(outcome.iterations < 30, "{outcome:?}");
    }

    /// Two constraints that say the same are met as one.
    #[test]
    fn repeated_constraints_are_met() {
        let outcome = minimize(
            |x: &[S]| {
                let c = x[0].add(x[1]).sub(S::ONE);
                Some(Evaluation {
                    constraints: rows(vec![c, c], vec![vec![S::ONE, S::ONE]; 2]),
                    sum: rows(vec![x[0]], vec![vec![S::ONE, S::ZERO]]),
                })
            },
            vec![s(0.3), s(0.3)],
            options(),
        )
        .unwrap();
        let x = at(&outcome);
        assert!(
            x[0].abs() < 1e-15 && (x[1] - 1.0).abs() < 1e-15,
            "{outcome:?}"
        );
    }

    /// A variable no residual depends on, between two that are minimized,
    /// stays put while they go where the sum wants them.
    #[test]
    fn a_free_variable_between_others_stays_put() {
        let outcome = minimize(
            |x: &[S]| {
                Some(Evaluation {
                    constraints: Rows::default(),
                    sum: rows(
                        vec![x[0].sub(S::ONE), x[2].sub(S::TWO)],
                        vec![
                            vec![S::ONE, S::ZERO, S::ZERO],
                            vec![S::ZERO, S::ZERO, S::ONE],
                        ],
                    ),
                })
            },
            vec![s(0.0), s(0.5), s(0.0)],
            options(),
        )
        .unwrap();
        assert_eq!(at(&outcome), [1.0, 0.5, 2.0], "{outcome:?}");
    }

    /// A point held a unit from a fixed one, pulled towards `(0, 3)` from
    /// `(1, 0)`: it turns about the fixed point until it is straight below
    /// the pull — a pull it can never meet, against a constraint that curves.
    /// The residuals of [`a_held_point_follows_a_pull_round`] at `x`.
    fn held_point(x: &[S]) -> Option<Evaluation<S>> {
        {
            {
                let (ax, ay, bx, by) = (x[0], x[1], x[2], x[3]);
                let (dx, dy) = (bx.sub(ax), by.sub(ay));
                let l = dx.mul(dx).add(dy.mul(dy)).sqrt().ok()?;
                let (ux, uy) = (dx.div(l).ok()?, dy.div(l).ok()?);
                let z = S::ZERO;
                let w = s(1e-4);
                Some(Evaluation {
                    constraints: rows(
                        vec![ax, ay, l.sub(S::ONE)],
                        vec![
                            vec![S::ONE, z, z, z],
                            vec![z, S::ONE, z, z],
                            vec![ux.neg(), uy.neg(), ux, uy],
                        ],
                    ),
                    sum: rows(
                        vec![bx, by.sub(s(3.0)), w.mul(ax), w.mul(ay)],
                        vec![
                            vec![z, z, S::ONE, z],
                            vec![z, z, z, S::ONE],
                            vec![w, z, z, z],
                            vec![z, w, z, z],
                        ],
                    ),
                })
            }
        }
    }

    #[test]
    fn a_held_point_follows_a_pull_round() {
        let outcome = minimize(
            held_point,
            vec![s(0.0), s(0.0), s(1.0), s(0.0)],
            Options {
                max_step: S::ONE,
                ..options()
            },
        )
        .unwrap();
        let x = at(&outcome);
        assert!(
            x[2].abs() < 1e-9 && (x[3] - 1.0).abs() < 1e-9,
            "{outcome:?}"
        );
    }

    /// Off a constraint, a step — however damped — corrects it: what holds
    /// the constraints is their linearization, which the damping does not
    /// weaken. The point held a unit from a fixed one (see
    /// [`a_held_point_follows_a_pull_round`]), where it once stalled.
    #[test]
    fn a_step_corrects_a_violated_constraint() {
        let (bx, by) = (s(0.0004572033706418954), s(1.0003621608700888));
        let l = bx.mul(bx).add(by.mul(by)).sqrt().unwrap();
        let (ux, uy) = (bx.div(l).unwrap(), by.div(l).unwrap());
        let (z, w) = (S::ZERO, s(1e-4));
        let e = Evaluation {
            constraints: rows(
                vec![z, z, l.sub(S::ONE)],
                vec![
                    vec![S::ONE, z, z, z],
                    vec![z, S::ONE, z, z],
                    vec![ux.neg(), uy.neg(), ux, uy],
                ],
            ),
            sum: rows(
                vec![bx, by.sub(s(3.0)), z, z],
                vec![
                    vec![z, z, S::ONE, z],
                    vec![z, z, z, S::ONE],
                    vec![w, z, z, z],
                    vec![z, w, z, z],
                ],
            ),
        };
        let b = vec![vec![S::ZERO; 4]; 4];
        for mu in [
            1e-30,
            1e-12,
            4.115226337448558e-6,
            1e-3,
            1.0,
            1e6,
            1e12,
            1e30,
        ] {
            let Step { delta, lambda, .. } = step(&e, &b, &[true; 4], s(mu)).unwrap().unwrap();
            let after = e.constraints.squared(&delta).sqrt().unwrap();
            assert!(
                after.definitely_less(l.sub(S::ONE)),
                "damped {mu}: {delta:?}, {lambda:?}, violation after {after:?}"
            );
        }
    }

    /// A step meets the linearized constraints, at every damping — here at
    /// the held point (see [`a_held_point_follows_a_pull_round`]) nearly on
    /// its circle, where one did not.
    #[test]
    fn a_step_meets_the_linearized_constraints() {
        let x = [
            s(0.0),
            s(0.0),
            s(-4.223620663585878e-8),
            s(1.0000000001445193),
        ];
        let e = held_point(&x).unwrap();
        let b = vec![vec![S::ZERO; 4]; 4];
        for mu in [4.572473708276175e-7, 1e-3, 1.0] {
            let Step { delta, lambda, .. } = step(&e, &b, &[true; 4], s(mu)).unwrap().unwrap();
            let after: Vec<S> = e
                .constraints
                .values
                .iter()
                .zip(&e.constraints.jacobian)
                .map(|(c, row)| c.add(dot(row, &delta)))
                .collect();
            assert!(
                after.iter().all(|a| a.abs().to_f64() < 1e-14),
                "damped {mu}: {delta:?}, {lambda:?}, linearized after {after:?}"
            );
        }
    }

    /// Two constraints that say the same: the step meets both, from the one
    /// that counts.
    #[test]
    fn a_step_meets_repeated_constraints() {
        let c = s(0.3).add(s(0.3)).sub(S::ONE);
        let e = Evaluation {
            constraints: rows(vec![c, c], vec![vec![S::ONE, S::ONE]; 2]),
            sum: rows(vec![s(0.3)], vec![vec![S::ONE, S::ZERO]]),
        };
        let Step { delta, lambda, .. } = step(&e, &vec![vec![S::ZERO; 2]; 2], &[true; 2], s(1e-3))
            .unwrap()
            .unwrap();
        let after: Vec<f64> = e
            .constraints
            .values
            .iter()
            .zip(&e.constraints.jacobian)
            .map(|(c, row)| c.add(dot(row, &delta)).to_f64())
            .collect();
        assert!(
            after.iter().all(|a| a.abs() < 1e-14),
            "{delta:?} {lambda:?} {after:?}"
        );
    }
}
