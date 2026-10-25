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
//! symmetric rank-one update, along the constraints only): without it,
//! wherever the sum cannot reach zero against curved constraints, steps
//! along them overshoot and the minimizer zig-zags. That curvature is as
//! often negative as positive — a mate pulled outwards curves the sum down
//! along it — so `B` is not kept positive, as a BFGS estimate would be:
//! one that cannot learn a negative curvature left an arm's drags creeping
//! towards the preferences' minimum until they ran out of steps. Only
//! `J_rᵀJ_r + B + λD`, over the directions the constraints leave free, must
//! be positive, and `λ` grows until it is. Where a step it proposed is
//! turned down, the step is tried again without it, and it is learned
//! afresh if that does better — a configuration passing through a singular
//! one changes it abruptly.
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

use crate::{
    geop_error::GeopResult,
    scalars::Scalar,
    solvers::linear::{cholesky, solve_cholesky},
};

/// How long [`minimize`] may try, and how far a step may go.
#[derive(Clone, Copy, Debug)]
pub struct Options {
    /// Steps taken at most: how hard it tries, never what an answer means.
    pub max_iterations: usize,
    /// The longest step: where the variables have a natural size, no step
    /// longer than it is sensible.
    pub max_step: Scalar,
}

/// Where [`minimize`] ended, and how it got there.
#[derive(Clone, Debug)]
pub struct Outcome<P> {
    pub x: P,
    pub iterations: usize,
    pub stop: Stop,
    /// Steps proposed and turned down — or proposed again without the
    /// curvature learned — on the way: what the iterations do not count.
    pub turned_down: usize,
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

/// Where [`minimize`]'s points are, and how a step moves one: the variables
/// themselves, a step added to them ([`Euclidean`]) — or a manifold, where
/// a step is taken from a point along its tangent space (a pose by the
/// exponential map of a twist, say), and the Jacobians evaluated at a
/// point are by steps from that point.
pub trait Space {
    type Point: Clone;

    /// How many numbers a step from `at` takes.
    fn dimension(&self, at: &Self::Point) -> usize;

    /// Where `step` takes `at`; `None` where it cannot. The point a step
    /// leads to is a free choice: sharp.
    fn step(&self, at: &Self::Point, step: &[Scalar]) -> Option<Self::Point>;

    /// Whether `b`, where a step from `a` led, is no different from `a`:
    /// the step changed nothing.
    fn same(&self, a: &Self::Point, b: &Self::Point) -> bool;
}

/// The variables themselves, as points: a step is added to them.
pub struct Euclidean;

impl Space for Euclidean {
    type Point = Vec<Scalar>;

    fn dimension(&self, at: &Vec<Scalar>) -> usize {
        at.len()
    }

    fn step(&self, at: &Vec<Scalar>, step: &[Scalar]) -> Option<Vec<Scalar>> {
        Some(
            at.iter()
                .zip(step)
                .map(|(a, d)| a.add(*d).sharpen())
                .collect(),
        )
    }

    fn same(&self, a: &Vec<Scalar>, b: &Vec<Scalar>) -> bool {
        a.iter().zip(b).all(|(a, b)| a.could_be_equal(*b))
    }
}

/// Residuals, and their Jacobian row by row.
#[derive(Clone, Debug, Default)]
pub struct Rows {
    pub values: Vec<Scalar>,
    pub jacobian: Vec<Vec<Scalar>>,
}

impl Rows {
    pub fn push(&mut self, value: Scalar, row: Vec<Scalar>) {
        self.values.push(value);
        self.jacobian.push(row);
    }

    /// `other`'s rows, after its own.
    pub fn append(&mut self, other: Rows) {
        self.values.extend(other.values);
        self.jacobian.extend(other.jacobian);
    }

    /// `values + jacobian δ`: the residuals a step `δ` leads to, to first
    /// order.
    fn linearized(&self, delta: &[Scalar]) -> Vec<Scalar> {
        (self.values.iter().zip(&self.jacobian))
            .map(|(&v, row)| v.add(dot(row, delta)))
            .collect()
    }

    /// `|values + jacobian δ|²`: at `δ = 0`, the sum of squares.
    fn squared(&self, delta: &[Scalar]) -> Scalar {
        let r = self.linearized(delta);
        dot(&r, &r)
    }

    /// `jacobianᵀ w`.
    fn transposed(&self, w: &[Scalar], n: usize) -> Vec<Scalar> {
        let mut g = vec![Scalar::ZERO; n];
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
#[derive(Clone, Debug, Default)]
pub struct Evaluation {
    pub constraints: Rows,
    pub sum: Rows,
}

/// By how much `λ` shrinks after a step taken, and grows after one not.
const SHRINK: i64 = 3;
const GROW: i64 = 4;

fn dot(a: &[Scalar], b: &[Scalar]) -> Scalar {
    a.iter()
        .zip(b)
        .fold(Scalar::ZERO, |sum, (&x, &y)| sum.add(x.mul(y)))
}

/// `|v|`, scaled by the largest size among `v` — a free choice, sharp —
/// before it is squared: the square of a number well below one keeps half
/// its digits, and in fixed point none at all. Squared as they were, the
/// residuals of a mate off by `1e-6` had a norm of `[0, 4.6e-5]` in fixed
/// point, which no step could definitely lower: a body pinned a half turn
/// away stopped `1e-6` off, ten times the tolerance.
fn norm(v: &[Scalar]) -> GeopResult<Scalar> {
    let largest = v.iter().fold(Scalar::ZERO, |m, x| m.max(x.abs().upper()));
    if !largest.definitely_greater(Scalar::ZERO) {
        return dot(v, v).sqrt();
    }
    let scaled = (v.iter().map(|x| x.div(largest))).collect::<GeopResult<Vec<Scalar>>>()?;
    Ok(largest.mul(dot(&scaled, &scaled).sqrt()?))
}

/// A value of the model a step is computed from — a slope, a residual, a
/// curvature, the step itself — chosen from its enclosure: any value in it
/// serves (see the module docs), so zero where zero is among them, else
/// the middle. A slope that could be zero is rounding around an exact
/// zero, and turned into a small number it would couple what does not
/// depend on each other — a coupling a weakly held variable then follows
/// arbitrarily far.
fn choose(v: Scalar) -> Scalar {
    if v.could_be_equal(Scalar::ZERO) {
        Scalar::ZERO
    } else {
        v.sharpen()
    }
}

fn sharp(v: &[Scalar]) -> Vec<Scalar> {
    v.iter().map(|&x| choose(x)).collect()
}

/// A step of the linearized problem (see [`step`]).
struct Step {
    delta: Vec<Scalar>,
    /// The constraints' multipliers.
    lambda: Vec<Scalar>,
    /// How many of the constraints' rows say something the others do not.
    independent: usize,
}

/// `v - a w`, chosen (see [`choose`]).
fn minus_scaled(v: &[Scalar], a: Scalar, w: &[Scalar]) -> Vec<Scalar> {
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
fn step(
    e: &Evaluation,
    b: &[Vec<Scalar>],
    moving: &[bool],
    mu: Scalar,
) -> GeopResult<Option<Step>> {
    let n = moving.len();
    let keep: Vec<usize> = (0..n).filter(|&i| moving[i]).collect();
    let k = keep.len();
    let squeeze = |v: &[Scalar]| -> Vec<Scalar> { keep.iter().map(|&i| v[i]).collect() };
    let jr: Vec<Vec<Scalar>> = e.sum.jacobian.iter().map(|r| sharp(&squeeze(r))).collect();
    let jc_wide: Vec<Vec<Scalar>> = e.constraints.jacobian.iter().map(|r| squeeze(r)).collect();
    let jc: Vec<Vec<Scalar>> = jc_wide.iter().map(|r| sharp(r)).collect();
    let c = sharp(&e.constraints.values);
    let m = jc.len();

    // H = J_rᵀJ_r + B + λ D, over the variables that move.
    let mut h: Vec<Vec<Scalar>> = keep.iter().map(|&i| squeeze(&b[i])).collect();
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
            .fold(Scalar::ZERO, |d, r| d.add(r[i].mul(r[i])));
        row[i] = row[i].add(mu.mul(d));
        row.iter_mut().for_each(|v| *v = choose(*v));
    }
    // Damped so hard that the curvature is no finite number: no step.
    if !h.iter().flatten().all(|v| v.is_finite()) {
        return Ok(None);
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
    let mut q: Vec<Vec<Scalar>> = Vec::new();
    // The Cholesky factor of the Gram matrix, row by row.
    let mut gram: Vec<Vec<Scalar>> = Vec::new();
    let mut rows: Vec<(usize, Vec<Scalar>)> = Vec::new();
    let mut dependent: Vec<(usize, Vec<Scalar>)> = Vec::new();
    for i in 0..m {
        let r = q.len();
        let b: Vec<Scalar> = q.iter().map(|qj| dot(&jc_wide[i], qj)).collect();
        let a = solve_cholesky(&gram, &vec![false; r], &b)?;
        let outside: Vec<Scalar> = (0..k)
            .map(|p| (0..r).fold(jc_wide[i][p], |sum, j| sum.sub(a[j].mul(q[j][p]))))
            .collect();
        if !norm(&outside)?.definitely_greater(Scalar::ZERO) {
            // Its coefficients in the basis, for the least-squares
            // correction below.
            dependent.push((i, sharp(&a)));
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
        let q_new: Vec<Scalar> = v
            .iter()
            .map(|&x| choose(x.div(length).unwrap_or(Scalar::ZERO)))
            .collect();
        let mut row = vec![Scalar::ZERO; r + 1];
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
    let coefficients_of = |coefficients: &[Scalar]| -> Vec<Scalar> {
        (0..r)
            .map(|j| coefficients.get(j).copied().unwrap_or(Scalar::ZERO))
            .collect()
    };
    let all: Vec<(usize, Vec<Scalar>)> = rows
        .iter()
        .chain(&dependent)
        .map(|(i, coefficients)| (*i, coefficients_of(coefficients)))
        .collect();
    let normal: Vec<Vec<Scalar>> = (0..r)
        .map(|a| {
            (0..r)
                .map(|b| {
                    choose(
                        all.iter()
                            .fold(Scalar::ZERO, |sum, (_, row)| sum.add(row[a].mul(row[b]))),
                    )
                })
                .collect()
        })
        .collect();
    let rhs_c: Vec<Scalar> = (0..r)
        .map(|a| {
            choose(
                all.iter()
                    .fold(Scalar::ZERO, |sum, (i, row)| sum.sub(row[a].mul(c[*i]))),
            )
        })
        .collect();
    let (ln, singular) = cholesky(&normal)?;
    let y = sharp(&solve_cholesky(&ln, &singular, &rhs_c)?);
    let mut delta_c = vec![Scalar::ZERO; k];
    for (t, qt) in q.iter().enumerate() {
        delta_c = minus_scaled(&delta_c, y[t].neg(), qt);
    }

    // Z: the unit directions with what is left of them after Q, taken one
    // at a time by the largest remainder — orthonormalized against Q and
    // each other.
    let mut left: Vec<Vec<Scalar>> = (0..k)
        .map(|p| {
            let mut v = vec![Scalar::ZERO; k];
            v[p] = Scalar::ONE;
            for qt in &q {
                v = minus_scaled(&v, choose(dot(&v, qt)), qt);
            }
            v
        })
        .collect();
    let mut z: Vec<Vec<Scalar>> = Vec::new();
    while z.len() < k - r {
        let lengths: Vec<Scalar> = left.iter().map(|v| dot(v, v)).collect();
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
        if !length.definitely_greater(Scalar::ZERO) {
            break;
        }
        let zt: Vec<Scalar> = left[best]
            .iter()
            .map(|&x| choose(x.div(length).unwrap_or(Scalar::ZERO)))
            .collect();
        for v in &mut left {
            *v = minus_scaled(v, choose(dot(v, &zt)), &zt);
        }
        z.push(zt);
    }

    // Along Z: (Zᵀ H Z) w = -Zᵀ (g + H δ_c).
    let g = sharp(&squeeze(&e.sum.transposed(&sharp(&e.sum.values), n)));
    let h_times =
        |v: &[Scalar]| -> Vec<Scalar> { h.iter().map(|row| choose(dot(row, v))).collect() };
    let force: Vec<Scalar> = g
        .iter()
        .zip(h_times(&delta_c))
        .map(|(&a, b)| choose(a.add(b)))
        .collect();
    let hz: Vec<Vec<Scalar>> = z.iter().map(|zt| h_times(zt)).collect();
    let reduced: Vec<Vec<Scalar>> = z
        .iter()
        .map(|zi| hz.iter().map(|hzj| choose(dot(zi, hzj))).collect())
        .collect();
    let (l, not_positive) = cholesky(&reduced)?;
    if not_positive.contains(&true) {
        return Ok(None);
    }
    let rhs: Vec<Scalar> = z.iter().map(|zt| choose(dot(zt, &force).neg())).collect();
    let w = sharp(&solve_cholesky(&l, &not_positive, &rhs)?);
    let mut kept = delta_c;
    for (t, zt) in z.iter().enumerate() {
        kept = minus_scaled(&kept, w[t].neg(), zt);
    }

    // The multipliers: J_cᵀ λ = -(g + H δ), R ᵀ λ = Qᵀ (-(g + H δ)).
    let pull: Vec<Scalar> = g
        .iter()
        .zip(h_times(&kept))
        .map(|(&a, b)| choose(a.add(b).neg()))
        .collect();
    // A curvature positive by so little that the step it solves for, or
    // the force along it, is no finite number is as good as not positive.
    if !kept.iter().chain(&pull).all(|v| v.is_finite()) {
        return Ok(None);
    }
    let mut lambda_rows = vec![Scalar::ZERO; r];
    for t in (0..r).rev() {
        let known = (t + 1..r).fold(dot(&q[t], &pull), |sum, s| {
            sum.sub(rows[s].1[t].mul(lambda_rows[s]))
        });
        lambda_rows[t] = choose(known.div(rows[t].1[t])?);
    }
    let mut lambda = vec![Scalar::ZERO; m];
    for (t, (i, _)) in rows.iter().enumerate() {
        lambda[*i] = lambda_rows[t];
    }
    let mut delta = vec![Scalar::ZERO; n];
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
/// `evaluate` giving both and their Jacobians — by steps from the point,
/// in `space` — and `None` where they cannot be evaluated, which makes the
/// point infeasible. Where the constraints cannot all hold, it ends where
/// they come closest. Always returns the best point found; whether it is
/// good enough is the caller's question.
///
/// `evaluate` is awaited: a caller running many minimizations side by side
/// answers all their evaluations at once (see
/// [`crate::solvers::system`]'s groups).
pub async fn minimize<P: Space>(
    space: &P,
    evaluate: impl AsyncFn(&P::Point) -> Option<Evaluation>,
    x0: P::Point,
    options: Options,
) -> GeopResult<Outcome<P::Point>> {
    let n = space.dimension(&x0);
    let mut x = x0;
    let Some(mut e) = evaluate(&x).await else {
        return Ok(Outcome {
            x,
            iterations: 0,
            stop: Stop::Infeasible,
            turned_down: 0,
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
                .any(|row| row[i].definitely_not_equal(Scalar::ZERO))
        })
        .collect();
    // The sum and the violation at every point taken: a step must
    // definitely improve on each in one of the two.
    let mut filter: Vec<(Scalar, Scalar)> = Vec::new();
    let mut mu = Scalar::from_ratio(1, 1000)?;
    let mut turned_down = 0;
    let mut b = vec![vec![Scalar::ZERO; n]; n];
    // The length of the last step taken.
    let mut last: Option<Scalar> = None;
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
        let no_curvature = vec![vec![Scalar::ZERO; n]; n];
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
                // learn it. The model of the sum is the one the step was
                // computed from: its rows' linearization, and the curvature
                // `B` it was computed with — what the constraints' curvature
                // costs or gains a step along them. Without `B`, a step that
                // `B` lengthened where the constraints curve the sum down
                // looked to the rows alone like no drop at all: a gear pair
                // dragged a quarter turn stopped, flat, at 36°.
                let expects = |delta: &[Scalar]| -> GeopResult<bool> {
                    let bent = (0..n).fold(Scalar::ZERO, |sum, i| {
                        sum.add(delta[i].mul(dot(&curvature[i], delta)))
                    });
                    Ok(f.sub(e.sum.squared(delta))
                        .sub(bent)
                        .definitely_greater(Scalar::ZERO)
                        || theta
                            .sub(norm(&e.constraints.linearized(delta))?)
                            .definitely_greater(Scalar::ZERO))
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
                        turned_down,
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
                    let delta = sharp(&delta);
                    let next = space.step(&x, &delta);
                    // Where not even this step changes anything, no smaller one
                    // will: nothing improves on here.
                    if next.as_ref().is_some_and(|next| space.same(next, &x)) {
                        return Ok(Outcome {
                            x,
                            iterations: iteration,
                            stop: Stop::NoChange { rejected },
                            turned_down,
                        });
                    }
                    // How far the step went, before any correction: what
                    // the radius shrinks from where it is turned down.
                    let tried = norm(&delta)?;
                    let landed = match next {
                        Some(next) => match evaluate(&next).await {
                            Some(e_next) => Some(
                                corrected(space, &evaluate, &moving, theta, next, e_next, delta)
                                    .await?,
                            ),
                            None => None,
                        },
                        None => None,
                    };
                    if let Some(landed) = landed {
                        let judged = judge(&e, &landed.1, theta, &filter, last, landed.2)?;
                        if first && judged.undecided && last.is_some() && !judged.contracting {
                            return Ok(Outcome {
                                x,
                                iterations: iteration,
                                stop: Stop::Undecided,
                                turned_down,
                            });
                        }
                        let taken = |j: &Judged| j.lower || (first && j.undecided && j.contracting);
                        if taken(&judged) {
                            let (next, e_next, _) = landed;
                            let length = judged.length;
                            // Never to zero, which the scalar's resolution
                            // rounds a small one to in fixed point: no
                            // growth brings it back, and a step turned down
                            // is then tried again as it was, forever.
                            let shrunk = mu.div(Scalar::from_i64(SHRINK))?.sharpen();
                            if shrunk.definitely_greater(Scalar::ZERO) {
                                mu = shrunk;
                            }
                            let reach = Scalar::TWO.mul(length);
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
                                b.iter_mut().flatten().for_each(|v| *v = Scalar::ZERO);
                            }
                            update_curvature(&mut b, &judged.s, &e, &e_next, &lambda)?;
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
                        if !plain && learned(&b) {
                            plain = true;
                            turned_down += 1;
                            continue;
                        }
                    } else {
                        rejected = Some(Rejection::Infeasible);
                    }
                    radius = tried.div(Scalar::from_i64(GROW))?.sharpen();
                }
            } else {
                rejected = Some(Rejection::NotPositive);
            }
            first = false;
            turned_down += 1;
            mu = mu.mul(Scalar::from_i64(GROW)).sharpen();
            if !mu.is_finite() {
                break None;
            }
        };
        let Some((next, e_next)) = accepted else {
            return Ok(Outcome {
                x,
                iterations: iteration,
                stop: Stop::Damped { rejected },
                turned_down,
            });
        };
        (x, e) = (next, e_next);
    }
    Ok(Outcome {
        x,
        iterations: options.max_iterations,
        stop: Stop::Budget,
        turned_down,
    })
}

/// How the step `s` from a point (evaluated as `e`) to the next (as
/// `e_next`) fares — see [`minimize`], where it is taken or turned down on
/// this.
struct Judged {
    /// The step, and its length.
    s: Vec<Scalar>,
    length: Scalar,
    /// It definitely improves on `x`, and is not worse in both than any
    /// point taken before.
    lower: bool,
    /// Neither the sum's change nor the violation's can be told from their
    /// width.
    undecided: bool,
    /// It is at most half the last step taken.
    contracting: bool,
}

fn judge(
    e: &Evaluation,
    e_next: &Evaluation,
    theta: Scalar,
    filter: &[(Scalar, Scalar)],
    last: Option<Scalar>,
    s: Vec<Scalar>,
) -> GeopResult<Judged> {
    let length = norm(&s)?;
    // `Σ r² - Σ r'²` summed as `Σ (r - r')(r + r')`: near the minimum of a
    // sum that stays large, the difference of the two sums is lost to their
    // width.
    let lower_sum = e
        .sum
        .values
        .iter()
        .zip(&e_next.sum.values)
        .fold(Scalar::ZERO, |sum, (&r, &r_next)| {
            sum.add(r.sub(r_next).mul(r.add(r_next)))
        });
    let theta_next = norm(&e_next.constraints.values)?;
    let lower_violation = theta.sub(theta_next);
    let f_next = e_next.sum.squared(&[]);
    // Far from the minimum, a step is taken where it definitely improves on
    // here, and is not worse in both than any point taken before. Near it,
    // both changes drown in their own width, and what still tells progress
    // is Newton's: a step at most half the last one taken contracts towards
    // a solution — until rounding stops it shrinking, which is where it
    // ends. Shrinking any less is no contraction: steps that only barely
    // shrink add up to any distance at all.
    let improves = lower_sum.definitely_greater(Scalar::ZERO)
        || lower_violation.definitely_greater(Scalar::ZERO);
    let lower = improves
        && filter.iter().all(|&(f_taken, theta_taken)| {
            f_next.definitely_less(f_taken) || theta_next.definitely_less(theta_taken)
        });
    let undecided = [lower_sum, lower_violation].iter().all(|change| {
        !change.definitely_greater(Scalar::ZERO) && !change.definitely_less(Scalar::ZERO)
    });
    let contracting = last.is_some_and(|last| Scalar::TWO.mul(length).definitely_less(last));
    Ok(Judged {
        s,
        length,
        lower,
        undecided,
        contracting,
    })
}

/// Corrections of a step back onto the constraints, at most: each a Newton
/// step on them alone, which squares how far off they are — how hard a
/// step is corrected, never whether it is taken.
const CORRECTIONS: usize = 4;

/// The point `x` (with its evaluation `e`), where the step `taken` from a
/// point off the constraints by `before` landed, moved back onto them if
/// the step left them further off — the shortest moves among the variables
/// `moving` that meet their linearization there, as long as each definitely
/// lowers their violation (a second-order correction) — and the step, with
/// the corrections, all told.
///
/// A step meets the constraints' linearization where it starts, so it
/// leaves them by as much as they curve over its length: a body turned by a
/// large step along the directions they leave free is off them by that
/// much. Judged there, the filter took steps that lowered the sum while the
/// violation grew, and the minimizer zig-zagged along the constraints — a
/// mated arm wandered off its mates by whole lengths and ran out of steps.
/// Corrected, such a step is judged where it actually leads along them.
/// A step that brought the constraints nearer is left as it is: the
/// shortest moves are not the ones the sum prefers, and correcting every
/// step drifted a sketch's line off where it was.
async fn corrected<P: Space>(
    space: &P,
    evaluate: &impl AsyncFn(&P::Point) -> Option<Evaluation>,
    moving: &[bool],
    before: Scalar,
    mut x: P::Point,
    mut e: Evaluation,
    mut taken: Vec<Scalar>,
) -> GeopResult<(P::Point, Evaluation, Vec<Scalar>)> {
    if !norm(&e.constraints.values)?.definitely_greater(before) {
        return Ok((x, e, taken));
    }
    for _ in 0..CORRECTIONS {
        let theta = norm(&e.constraints.values)?;
        // δ = -J_cᵀ k with J_c J_cᵀ k = c: the shortest step meeting the
        // linearized constraints, rows that say nothing new left out.
        let jc: Vec<Vec<Scalar>> = e
            .constraints
            .jacobian
            .iter()
            .map(|row| {
                row.iter()
                    .zip(moving)
                    .map(|(&v, &m)| if m { choose(v) } else { Scalar::ZERO })
                    .collect()
            })
            .collect();
        let zero = vec![Scalar::ZERO; taken.len()];
        let d: Vec<Scalar> = (minus_solved(&jc, &sharp(&e.constraints.values), &zero)?.into_iter())
            .map(Scalar::sharpen)
            .collect();
        if !d.iter().all(|v| v.is_finite()) {
            break;
        }
        let Some(next) = space.step(&x, &d) else {
            break;
        };
        if space.same(&next, &x) {
            break;
        }
        let Some(e_next) = evaluate(&next).await else {
            break;
        };
        if !norm(&e_next.constraints.values)?.definitely_less(theta) {
            break;
        }
        // The steps add up as they would among the variables themselves:
        // exactly so there, and to first order on a manifold.
        taken = taken
            .iter()
            .zip(&d)
            .map(|(a, b)| a.add(*b).sharpen())
            .collect();
        (x, e) = (next, e_next);
    }
    Ok((x, e, taken))
}

/// Whether any curvature has been learned into `b`.
fn learned(b: &[Vec<Scalar>]) -> bool {
    b.iter()
        .flatten()
        .any(|v| !v.is_sharp() || v.definitely_not_equal(Scalar::ZERO))
}

/// The part of `v` along the constraints — orthogonal to the rows of their
/// Jacobian, `v - J_cᵀ (J_c J_cᵀ)⁻¹ J_c v` — chosen (see [`choose`]); rows
/// that say nothing the others do not left out.
fn along(constraints: &Rows, v: &[Scalar]) -> GeopResult<Vec<Scalar>> {
    let jc: Vec<Vec<Scalar>> = constraints.jacobian.iter().map(|r| sharp(r)).collect();
    let rhs: Vec<Scalar> = jc.iter().map(|row| dot(row, v)).collect();
    minus_solved(&jc, &rhs, v)
}

/// `from - J_cᵀ k` with `J_c J_cᵀ k = rhs`, chosen (see [`choose`]); rows
/// of `jc` that say nothing the others do not left out.
fn minus_solved(jc: &[Vec<Scalar>], rhs: &[Scalar], from: &[Scalar]) -> GeopResult<Vec<Scalar>> {
    let gram: Vec<Vec<Scalar>> = jc
        .iter()
        .map(|a| jc.iter().map(|b| dot(a, b)).collect())
        .collect();
    let (l, dependent) = cholesky(&gram)?;
    let k = solve_cholesky(&l, &dependent, rhs)?;
    Ok((0..from.len())
        .map(|i| choose((0..jc.len()).fold(from[i], |sum, r| sum.sub(jc[r][i].mul(k[r])))))
        .collect())
}

/// Updates `b`, the curvature Gauss–Newton leaves out, from the step `s`
/// from `e` to `e_next`: by how much the gradient of the Lagrangian
/// `½ Σ r² + λᵀc` turned along it, beyond what `J_rᵀJ_r` explains (a
/// structured symmetric rank-one update): `b` then turns the gradient along
/// `s` exactly as it did, curving up or down. Skipped where the correction
/// could be square to `s`, which leaves its size undetermined.
fn update_curvature(
    b: &mut [Vec<Scalar>],
    s: &[Scalar],
    e: &Evaluation,
    e_next: &Evaluation,
    lambda: &[Scalar],
) -> GeopResult<()> {
    let n = s.len();
    let r_next = sharp(&e_next.sum.values);
    let constraints_next = e_next.constraints.transposed(lambda, n);
    let constraints_before = e.constraints.transposed(lambda, n);
    let sum_next = e_next.sum.transposed(&r_next, n);
    let sum_before = e.sum.transposed(&r_next, n);
    let y: Vec<Scalar> = (0..n)
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
    let s = along(&e.constraints, s)?;
    let bs: Vec<Scalar> = (0..n).map(|i| dot(&b[i], &s)).collect();
    // `(y - B s)·s`, asked of `y`'s honest enclosure — the same along the
    // constraints as off them, `s` being along them. Where it could be
    // zero, the correction is square to the step as far as the gradients
    // tell, and dividing by it would make the curvature whatever their
    // rounding says.
    if !dot(&y, &s)
        .sub(dot(&bs, &s))
        .definitely_not_equal(Scalar::ZERO)
    {
        return Ok(());
    }
    let y = along(&e.constraints, &y)?;
    let v: Vec<Scalar> = (0..n).map(|i| choose(y[i].sub(bs[i]))).collect();
    let vs = choose(dot(&v, &s));
    if !vs.definitely_not_equal(Scalar::ZERO) {
        return Ok(());
    }
    for i in 0..n {
        for j in 0..n {
            b[i][j] = choose(b[i][j].add(v[i].mul(v[j]).div(vs)?));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// [`super::minimize`], every evaluation answered as it is asked.
    fn minimize<P: Space>(
        space: &P,
        evaluate: impl Fn(&P::Point) -> Option<Evaluation>,
        x0: P::Point,
        options: Options,
    ) -> GeopResult<Outcome<P::Point>> {
        let work = std::pin::pin!(super::minimize(
            space,
            async |x: &P::Point| evaluate(x),
            x0,
            options
        ));
        let mut context = std::task::Context::from_waker(std::task::Waker::noop());
        match std::future::Future::poll(work, &mut context) {
            std::task::Poll::Ready(outcome) => outcome,
            std::task::Poll::Pending => panic!("nothing awaited is pending"),
        }
    }

    fn s(v: f64) -> Scalar {
        Scalar::from_f64(v)
    }

    fn options() -> Options {
        Options {
            max_iterations: 500,
            max_step: s(10.0),
        }
    }

    fn rows(values: Vec<Scalar>, jacobian: Vec<Vec<Scalar>>) -> Rows {
        Rows { values, jacobian }
    }

    fn at(outcome: &Outcome<Vec<Scalar>>) -> Vec<f64> {
        outcome.x.iter().map(|v| v.to_f64()).collect()
    }

    /// Rosenbrock, as residuals `(1 - a, 10 (b - a²))`.
    #[test]
    fn minimizes_rosenbrock() {
        let outcome = minimize(
            &Euclidean,
            |x: &Vec<Scalar>| {
                let (a, b) = (x[0], x[1]);
                Some(Evaluation {
                    constraints: Rows::default(),
                    sum: rows(
                        vec![Scalar::ONE.sub(a), s(10.0).mul(b.sub(a.mul(a)))],
                        vec![vec![s(-1.0), Scalar::ZERO], vec![s(-20.0).mul(a), s(10.0)]],
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
            &Euclidean,
            |x: &Vec<Scalar>| {
                Some(Evaluation {
                    constraints: rows(
                        vec![x[0].sub(s(2.0))],
                        vec![vec![Scalar::ONE, Scalar::ZERO]],
                    ),
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
    /// the constraint's curvature is learned, not stumbled over. Found as
    /// near as the sum tells: `ε` along the circle from the nearest point
    /// lowers the sum, about 16, by `5 ε²`, lost in its width below
    /// `ε` of a few `1e-8` — where the minimizer stops is then a matter of
    /// where its last step happened to land.
    #[test]
    fn minimizes_on_a_circle() {
        let outcome = minimize(
            &Euclidean,
            |x: &Vec<Scalar>| {
                let (a, b) = (x[0], x[1]);
                Some(Evaluation {
                    constraints: rows(
                        vec![a.mul(a).add(b.mul(b)).sub(Scalar::ONE)],
                        vec![vec![Scalar::TWO.mul(a), Scalar::TWO.mul(b)]],
                    ),
                    sum: rows(
                        vec![a.sub(s(3.0)), b.sub(s(4.0))],
                        vec![
                            vec![Scalar::ONE, Scalar::ZERO],
                            vec![Scalar::ZERO, Scalar::ONE],
                        ],
                    ),
                })
            },
            vec![s(1.0), s(0.0)],
            options(),
        )
        .unwrap();
        let x = at(&outcome);
        assert!(
            (x[0] - 0.6).abs() < 1e-9 && (x[1] - 0.8).abs() < 1e-9,
            "{outcome:?}"
        );
        assert!(outcome.iterations < 30, "{outcome:?}");
    }

    /// Two constraints that say the same are met as one.
    #[test]
    fn repeated_constraints_are_met() {
        let outcome = minimize(
            &Euclidean,
            |x: &Vec<Scalar>| {
                let c = x[0].add(x[1]).sub(Scalar::ONE);
                Some(Evaluation {
                    constraints: rows(vec![c, c], vec![vec![Scalar::ONE, Scalar::ONE]; 2]),
                    sum: rows(vec![x[0]], vec![vec![Scalar::ONE, Scalar::ZERO]]),
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
            &Euclidean,
            |x: &Vec<Scalar>| {
                Some(Evaluation {
                    constraints: Rows::default(),
                    sum: rows(
                        vec![x[0].sub(Scalar::ONE), x[2].sub(Scalar::TWO)],
                        vec![
                            vec![Scalar::ONE, Scalar::ZERO, Scalar::ZERO],
                            vec![Scalar::ZERO, Scalar::ZERO, Scalar::ONE],
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
    fn held_point(x: &[Scalar]) -> Option<Evaluation> {
        {
            {
                let (ax, ay, bx, by) = (x[0], x[1], x[2], x[3]);
                let (dx, dy) = (bx.sub(ax), by.sub(ay));
                let l = dx.mul(dx).add(dy.mul(dy)).sqrt().ok()?;
                let (ux, uy) = (dx.div(l).ok()?, dy.div(l).ok()?);
                let z = Scalar::ZERO;
                let w = s(1e-4);
                Some(Evaluation {
                    constraints: rows(
                        vec![ax, ay, l.sub(Scalar::ONE)],
                        vec![
                            vec![Scalar::ONE, z, z, z],
                            vec![z, Scalar::ONE, z, z],
                            vec![ux.neg(), uy.neg(), ux, uy],
                        ],
                    ),
                    sum: rows(
                        vec![bx, by.sub(s(3.0)), w.mul(ax), w.mul(ay)],
                        vec![
                            vec![z, z, Scalar::ONE, z],
                            vec![z, z, z, Scalar::ONE],
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
            &Euclidean,
            |x: &Vec<Scalar>| held_point(x),
            vec![s(0.0), s(0.0), s(1.0), s(0.0)],
            Options {
                max_step: Scalar::ONE,
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
        let (z, w) = (Scalar::ZERO, s(1e-4));
        let e = Evaluation {
            constraints: rows(
                vec![z, z, l.sub(Scalar::ONE)],
                vec![
                    vec![Scalar::ONE, z, z, z],
                    vec![z, Scalar::ONE, z, z],
                    vec![ux.neg(), uy.neg(), ux, uy],
                ],
            ),
            sum: rows(
                vec![bx, by.sub(s(3.0)), z, z],
                vec![
                    vec![z, z, Scalar::ONE, z],
                    vec![z, z, z, Scalar::ONE],
                    vec![w, z, z, z],
                    vec![z, w, z, z],
                ],
            ),
        };
        let b = vec![vec![Scalar::ZERO; 4]; 4];
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
                after.definitely_less(l.sub(Scalar::ONE)),
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
        let b = vec![vec![Scalar::ZERO; 4]; 4];
        for mu in [4.572473708276175e-7, 1e-3, 1.0] {
            let Step { delta, lambda, .. } = step(&e, &b, &[true; 4], s(mu)).unwrap().unwrap();
            let after: Vec<Scalar> = e
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
        let c = s(0.3).add(s(0.3)).sub(Scalar::ONE);
        let e = Evaluation {
            constraints: rows(vec![c, c], vec![vec![Scalar::ONE, Scalar::ONE]; 2]),
            sum: rows(vec![s(0.3)], vec![vec![Scalar::ONE, Scalar::ZERO]]),
        };
        let Step { delta, lambda, .. } =
            step(&e, &vec![vec![Scalar::ZERO; 2]; 2], &[true; 2], s(1e-3))
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
