//! [`System::enclose`]: a rigorous enclosure of a true solution near where
//! a solve ended, by Krawczyk's test, in each block's step coordinates.
//!
//! The constraints determine some variables and leave the rest free. The
//! free ones are the designer's choice, and stay exactly as they are. The
//! determined ones are first polished by Newton's method on the
//! independent rows, then enclosed: for a box `Δ` of steps around the
//! polished point `x̃` — a value `x = retract(x̃, δ)` for every `δ` in it —
//! with `Y` any approximate inverse of the Jacobian at `x̃`,
//!
//! ```text
//! K(Δ) = -Y f(x̃) + (I - Y J(Δ)) Δ
//! ```
//!
//! and `K(Δ) ⊆ Δ` together with `‖I - Y J(Δ)‖∞ < 1` prove that `Δ` holds
//! exactly one solution of the independent rows — in interval arithmetic,
//! so the proof is rigorous. `K(Δ) ⊆ Δ` gives a zero of `Y f` in `Δ`
//! (Brouwer); the norm bound makes `Y` and every matrix of `J(Δ)` regular,
//! so it is a zero of `f`, the independent rows stay independent over the
//! whole box, and no second zero exists. `J(Δ)` is the Jacobian by the
//! steps at every `x` the box reaches, times the differential of the
//! retraction there ([`Variable::differential`]). The box is widened until
//! the first test passes; its width is then how precisely the constraints
//! pin the solution down, and `retract(x̃, K(Δ) ∩ Δ)` is returned.
//!
//! Nothing is sharpened but the Newton iterates, each only a seed for the
//! next: `Y`, `f(x̃)` and `J(Δ)` are enclosures, and so is the box.
//!
//! **Which rows are independent** is decided over a box, not at a point. A
//! redundant constraint — a rectangle's fourth right angle, an arc
//! concentric with a center it already has, a mate a joint implies — says
//! what the others do, often only on the solution. Where the blocks are
//! now, a solution only to the solver's tolerance, it looks independent by
//! a pivot about as small as that tolerance; taken as independent, it makes
//! the solution a curve rather than a point, which nothing proves. So the
//! independent rows are chosen by eliminating the Jacobian over a box of
//! steps around where the blocks are — first the point itself, then a box
//! as wide as the solver's tolerance, then twice as wide, and so on: a
//! pivot that could be zero somewhere in the box is no pivot, and once the
//! box reaches the solution a redundant row's pivot straddles zero. Each
//! new choice is put to the Krawczyk test.
//!
//! Then, over the proven box, the rows left out must vanish and add nothing
//! to the rank. A constraint the others imply leaves the rank as it is
//! wherever they all hold, so over a box holding such a point too. One left
//! out only because the configuration is singular — a point on a circle and
//! on its tangent, the solve a hair off the touching point — is independent
//! there: its value too small to tell from zero, its derivative not. Without
//! this the box held the free variable exactly where the solve left it,
//! where no solution is. How free the blocks are is decided over the same
//! box.
//!
//! A row left out that does not vanish is a conflict only where one is
//! proven ([`EncloseError::Conflict`], naming it and the constraints it was
//! reduced by): the solution the box holds was found with the free
//! variables held, so it says nothing of other values of them. A
//! combination of the rows vanishes wherever they all do; definitely not
//! zero over a box in every variable around where the solve ended, it
//! proves that no point of that box is a solution. Interval arithmetic can
//! prove a conflict, never its absence: a row enclosing zero agrees with
//! the others to every digit the box resolves. A limit is enclosed as a
//! constraint that vanishes where it could be at its bound, and must be
//! proven to hold over the box elsewhere.
//!
//! The solution proven is the one Newton reaches from where the solve
//! ended: for a solve that converged, the one it ended near.

use crate::{
    geop_error::{GeopError, GeopResult},
    scalars::{Ring, Scalar},
    solvers::linear::{augmented, eliminate, inverse, null_space, rank},
};

use super::{Batch, Bound, Equations, Group, ScalarOf, Solving, System, Term, Variable, tolerance};

/// Newton steps polishing a solution before it is enclosed: each squares the
/// error, so a handful take [`super::RELATIVE_TOLERANCE`]
/// to rounding.
const POLISH_STEPS: usize = 6;

/// How many candidate boxes are tried, each twice as wide as the last one
/// needed: how hard it tries, never what a verified box means.
const ENCLOSE_ATTEMPTS: usize = 24;

/// An enclosure of a true solution, group by group, and how free the
/// blocks are (see [`System::enclose`]).
#[derive(Debug)]
pub struct Enclosure<V: Variable> {
    /// Per block, every value it could have at the true solution its group
    /// was proven to hold — `None` for a block whose group was not
    /// enclosed, or not proven.
    pub boxes: Vec<Option<V>>,
    /// Per block, how many independent ways it can still move: decided
    /// over the proven box, or — not proven — at the point.
    pub freedom: Vec<usize>,
    /// How many independent ways the blocks can move together.
    pub total: usize,
    /// Why the groups not proven were not, each with its blocks.
    pub failures: Vec<(Vec<usize>, EncloseError<V::Scalar>)>,
    /// How many times the equations were made to enclose it — every
    /// group's at once: the work it took, which a test can bound.
    pub evaluations: usize,
}

impl<V: Variable> Enclosure<V> {
    /// Nothing enclosed yet, of `blocks` blocks.
    pub(super) fn empty(blocks: usize) -> Self {
        Self {
            boxes: vec![None; blocks],
            freedom: vec![0; blocks],
            total: 0,
            failures: Vec::new(),
            evaluations: 0,
        }
    }
}

/// Why no enclosure of a solution is proven.
#[derive(Debug)]
pub enum EncloseError<S: Scalar> {
    /// Constraints that cannot all hold anywhere within `within` — in
    /// each block's steps — of where the solve ended: a combination of the
    /// rows, which vanishes wherever they all do, is definitely not zero
    /// over that box. By the constraint's index, each with what the
    /// combination reducing it is there; `with` are the constraints it was
    /// reduced by, by index: what it conflicts with.
    Conflict {
        conflicting: Vec<(usize, S)>,
        with: Vec<usize>,
        within: S,
    },
    /// No solution was proven (see [`System::enclose`]).
    Unproven(GeopError),
}

impl<S: Scalar> From<GeopError> for EncloseError<S> {
    fn from(e: GeopError) -> Self {
        EncloseError::Unproven(e)
    }
}

impl<S: Scalar> EncloseError<S> {
    /// As an error, each constraint named by `name`.
    pub fn named(self, name: impl Fn(usize) -> String) -> GeopError {
        match self {
            EncloseError::Conflict {
                conflicting,
                with,
                within,
            } => {
                let mut constraints: Vec<usize> = conflicting.iter().map(|&(c, _)| c).collect();
                constraints.dedup();
                let names =
                    |cs: &[usize]| cs.iter().map(|&c| name(c)).collect::<Vec<_>>().join(", ");
                let others = if with.is_empty() {
                    "the others".to_string()
                } else {
                    names(&with)
                };
                let off: Vec<S> = conflicting.iter().map(|&(_, v)| v).collect();
                GeopError::new(format!(
                    "the constraints conflict: {} cannot hold where {others} do within {within:?} of here (off by {off:?} there)",
                    names(&constraints),
                ))
            }
            EncloseError::Unproven(e) => e,
        }
    }
}

/// Why a proof of the rows chosen is not taken: rows that do not vanish
/// where the chosen ones do, by index among the rows — which may be a
/// conflict ([`Solving::conflict`]) — or why it failed.
enum Rejected<S: Scalar> {
    Off(Vec<usize>),
    Failed(EncloseError<S>),
}

use Rejected::{Failed, Off};

impl<S: Scalar> From<GeopError> for Rejected<S> {
    fn from(e: GeopError) -> Self {
        Failed(EncloseError::Unproven(e))
    }
}

/// One row of a constraint over a box (see [`Solving::rows_over`]).
#[derive(Clone)]
struct Row<S: Scalar> {
    /// The constraint it is one of, by index.
    constraint: usize,
    value: S,
    /// Its derivative by every variable of the group.
    slope: Vec<S>,
}

/// A group enclosed: the box of steps around `center` its true solution
/// is in — or why there is none — and how free its blocks are.
pub(super) struct Enclosed<V: Variable> {
    pub boxes: Result<Vec<V>, EncloseError<V::Scalar>>,
    /// Per block of the group.
    pub freedom: Vec<usize>,
    pub total: usize,
}

impl<E: Equations> System<'_, E> {
    /// [`System::enclose`] of the groups `groups`.
    pub(super) fn enclose_groups(
        &self,
        at: &[E::Value],
        groups: &[Group],
    ) -> GeopResult<super::Enclosure<E::Value>> {
        let batch = Batch::new(self, at);
        let enclosed = batch.run(groups.iter().map(|group| {
            let batch = &batch;
            async move { self.enclose_group(at, group, batch).await }
        }))?;
        let mut enclosure = Enclosure::empty(self.mobility.len());
        enclosure.evaluations = batch.evaluations();
        for (group, enclosed) in groups.iter().zip(enclosed) {
            for (k, &block) in group.blocks.iter().enumerate() {
                enclosure.freedom[block] = enclosed.freedom[k];
            }
            enclosure.total += enclosed.total;
            match enclosed.boxes {
                Ok(boxes) => {
                    for (&block, value) in group.blocks.iter().zip(boxes) {
                        enclosure.boxes[block] = Some(value);
                    }
                }
                Err(e) => enclosure.failures.push((group.blocks.clone(), e)),
            }
        }
        Ok(enclosure)
    }

    /// The enclosure of the group `group` of the blocks at `at`, its terms
    /// asked of `batch`.
    pub(super) async fn enclose_group(
        &self,
        at: &[E::Value],
        group: &Group,
        batch: &Batch<'_, '_, E>,
    ) -> GeopResult<Enclosed<E::Value>> {
        let solving = Solving::new(self, at, group, batch);
        let x = solving.start();
        let n = solving.n;
        let tolerance = tolerance(self.scale)?;
        // The limits that could be at their bound are enclosed as
        // constraints that vanish; the others must hold over the box.
        let limits: Vec<usize> = (group.constraints.iter())
            .filter(|&&c| self.equations.bound(c) == Bound::AtMost)
            .copied()
            .collect();
        let made = match limits.is_empty() {
            true => Some(Vec::new()),
            false => solving.constraints_at(&x, &limits).await,
        };
        let (mut equalities, mut inactive) = (Vec::new(), Vec::new());
        for &c in &group.constraints {
            let k = limits.iter().position(|&l| l == c);
            let short = match (k, &made) {
                (None, _) => false,
                (Some(k), Some(made)) => {
                    (made[k].values.iter()).all(|r| r.definitely_less(tolerance.neg()))
                }
                (Some(_), None) => false,
            };
            match short {
                true => inactive.push(c),
                false => equalities.push(c),
            }
        }
        let zero = vec![ScalarOf::<E>::ZERO; n];
        let mut radius = ScalarOf::<E>::ZERO;
        let mut tried: Vec<Vec<(usize, usize)>> = Vec::new();
        let mut failure: Option<EncloseError<ScalarOf<E>>> = None;
        // A limit leaves free the motion away from it.
        let vanishing = |rows: &[Row<ScalarOf<E>>]| -> Vec<Vec<ScalarOf<E>>> {
            (rows.iter())
                .filter(|r| self.equations.bound(r.constraint) == Bound::Zero)
                .map(|r| r.slope.clone())
                .collect()
        };
        // How free the blocks are: at the point, until a proof decides it
        // over its box.
        let mut freedom = None;
        for attempt in 0..ENCLOSE_ATTEMPTS {
            if attempt == 1 {
                radius = tolerance;
            } else if attempt > 1 {
                radius = ScalarOf::<E>::TWO.mul(radius);
            }
            let around: Vec<ScalarOf<E>> = match attempt {
                0 => zero.clone(),
                _ => vec![radius.neg().union(radius); n],
            };
            let rows = match solving.rows_over(&x, &around, &equalities).await {
                Ok(rows) => rows,
                // Undecidable somewhere in this box: no wider one helps.
                Err(e) => {
                    failure.get_or_insert(e.into());
                    break;
                }
            };
            let slopes: Vec<Vec<ScalarOf<E>>> = rows.iter().map(|r| r.slope.clone()).collect();
            if freedom.is_none() {
                freedom = Some(solving.freedom(vanishing(&rows)));
            }
            let mut pivots = eliminate(slopes, n).pivots;
            pivots.sort_unstable();
            if tried.contains(&pivots) {
                continue;
            }
            tried.push(pivots.clone());
            // At the point, the rows chosen over it are those Newton starts
            // from.
            let at = (attempt == 0).then(|| rows.clone());
            let proven = match solving.prove(x.clone(), at, &equalities, &pivots).await {
                Ok((center, steps)) => {
                    match (solving.verify(&center, &steps, &equalities, &inactive, &pivots)).await {
                        Ok(over) => Ok((center, steps, over)),
                        Err(e) => Err(e),
                    }
                }
                Err(e) => Err(e),
            };
            let proven = match proven {
                Ok(proven) => Ok(proven),
                Err(Off(off)) => {
                    Err((solving.conflict(&x, radius, &equalities, &rows, &off)).await)
                }
                Err(Failed(e)) => Err(e),
            };
            match proven {
                Ok((center, steps, over)) => {
                    let boxes = (group.blocks.iter().enumerate())
                        .map(|(k, _)| {
                            let column = solving.columns[k];
                            let dimension = center[k].dimension();
                            center[k].retract_over(&steps[column..column + dimension])
                        })
                        .collect::<GeopResult<Vec<_>>>()?;
                    // How free the blocks are, over the box proven.
                    let (freedom, total) = solving.freedom(vanishing(&over));
                    return Ok(Enclosed {
                        boxes: Ok(boxes),
                        freedom,
                        total,
                    });
                }
                Err(e) => {
                    // A conflict is proven: no wider box makes it hold.
                    let conflict = matches!(e, EncloseError::Conflict { .. });
                    failure = Some(e);
                    if conflict {
                        break;
                    }
                }
            }
        }
        let independent: Vec<usize> = tried.iter().map(Vec::len).collect();
        let failure = match failure {
            Some(EncloseError::Unproven(e)) => EncloseError::Unproven(e.with_context(format!(
                "enclose: no choice of independent rows was proven — tried {independent:?} rows, over {n} variables"
            ))),
            Some(conflict) => conflict,
            None => EncloseError::Unproven(GeopError::new(format!(
                "enclose: no choice of independent rows was proven — tried {independent:?} rows, over {n} variables"
            ))),
        };
        let (freedom, total) = freedom.unwrap_or_else(|| {
            let free: Vec<usize> = group.blocks.iter().map(|&b| at[b].dimension()).collect();
            (free, n)
        });
        Ok(Enclosed {
            boxes: Err(failure),
            freedom,
            total,
        })
    }
}

impl<E: Equations> Solving<'_, '_, E> {
    /// Every row of the constraints `which` over the box of steps `steps`
    /// from `x` — the group's values — and its derivative by every step:
    /// an enclosure of both, for every point of the box. Fails where a
    /// constraint cannot be made somewhere in it.
    async fn rows_over(
        &self,
        x: &[E::Value],
        steps: &[ScalarOf<E>],
        which: &[usize],
    ) -> GeopResult<Vec<Row<ScalarOf<E>>>> {
        let mut values = Vec::with_capacity(x.len());
        let mut differentials = Vec::with_capacity(x.len());
        for (k, value) in x.iter().enumerate() {
            let column = self.columns[k];
            let own = &steps[column..column + value.dimension()];
            values.push(value.retract_over(own)?);
            differentials.push(value.differential(own)?);
        }
        let terms: Vec<Term> = which.iter().map(|&c| Term::Constraint(c)).collect();
        let made = self.made(&values, &terms).await;
        let differential = |block: usize| {
            let k = self.group.blocks.binary_search(&block).ok()?;
            differentials.get(k)?.as_ref()
        };
        let mut rows = Vec::new();
        for (&c, made) in which.iter().zip(made) {
            let made = made.ok_or_else(|| {
                GeopError::new(format!(
                    "constraint {c} cannot be made over the box of steps {steps:?}"
                ))
            })?;
            let linear = self.rows(&made, &differential);
            for (value, slope) in linear.values.into_iter().zip(linear.jacobian) {
                rows.push(Row {
                    constraint: c,
                    value,
                    slope,
                });
            }
        }
        Ok(rows)
    }

    /// How free the group's blocks are, by the rows `slopes`: per block,
    /// the rank of their null space's part along it, and its dimension.
    fn freedom(&self, slopes: Vec<Vec<ScalarOf<E>>>) -> (Vec<usize>, usize) {
        let basis = null_space(slopes, self.n);
        let per_block = (self.group.blocks.iter().enumerate())
            .map(|(k, &block)| {
                let column = self.columns[k];
                let dimension = self.base[block].dimension();
                let along: Vec<Vec<ScalarOf<E>>> = (basis.iter())
                    .map(|v| v[column..column + dimension].to_vec())
                    .collect();
                rank(along, dimension)
            })
            .collect();
        (per_block, basis.len())
    }

    /// The Krawczyk proof for the independent rows `pivots` — `(variable,
    /// row)` pairs — from `x`, where the rows of `which` are `at` if given:
    /// the
    /// polished center, and the box of steps
    /// from it holding their solution, every variable no pivot determines
    /// at an exact zero step.
    #[allow(clippy::type_complexity)]
    async fn prove(
        &self,
        x: Vec<E::Value>,
        at: Option<Vec<Row<ScalarOf<E>>>>,
        which: &[usize],
        pivots: &[(usize, usize)],
    ) -> Result<(Vec<E::Value>, Vec<ScalarOf<E>>), Rejected<ScalarOf<E>>> {
        let n = self.n;
        let zero = vec![ScalarOf::<E>::ZERO; n];
        if pivots.is_empty() {
            return Ok((x, zero));
        }
        let (cols, picked): (Vec<usize>, Vec<usize>) = pivots.iter().copied().unzip();
        let m = cols.len();
        let square = |rows: &[Row<ScalarOf<E>>]| -> Vec<Vec<ScalarOf<E>>> {
            picked
                .iter()
                .map(|&r| cols.iter().map(|&c| rows[r].slope[c]).collect())
                .collect()
        };
        let singular = |rows: &[Row<ScalarOf<E>>]| {
            let rows: Vec<(ScalarOf<E>, &Vec<ScalarOf<E>>)> =
                rows.iter().map(|r| (r.value, &r.slope)).collect();
            GeopError::new(format!(
                "the constraints are singular at their solution: rows {picked:?} over variables {cols:?} of {rows:?}"
            ))
        };
        let product = |y: &[Vec<ScalarOf<E>>], k: usize, column: &dyn Fn(usize) -> ScalarOf<E>| {
            (0..m).fold(ScalarOf::<E>::ZERO, |sum, j| {
                sum.add(y[k][j].mul(column(j)))
            })
        };
        // Which blocks a pivot moves: only those are stepped.
        let moved: Vec<bool> = (0..x.len())
            .map(|k| {
                let column = self.columns[k];
                let dimension = x[k].dimension();
                cols.iter()
                    .any(|&c| (column..column + dimension).contains(&c))
            })
            .collect();

        // Newton, on the determined variables. Every iterate is only a seed
        // for the next, so it is sharp (a free choice, and without it the
        // width of each step compounds); the last pass takes no step: it is
        // the one the box is proven around. A step that could be zero
        // throughout is rounding: no step after it does better, and the
        // polish is done.
        let mut polished = 0;
        let mut xt = x;
        let mut made = at;
        let (rows, y) = loop {
            let rows = match made.take() {
                Some(rows) => rows,
                None => self.rows_over(&xt, &zero, which).await?,
            };
            let y = inverse(&square(&rows)).ok_or_else(|| singular(&rows))?;
            if polished == POLISH_STEPS {
                break (rows, y);
            }
            polished += 1;
            let mut step = zero.clone();
            for (k, &c) in cols.iter().enumerate() {
                step[c] = product(&y, k, &|j| rows[picked[j]].value).neg();
            }
            if step.iter().all(|d| d.could_be_equal(ScalarOf::<E>::ZERO)) {
                break (rows, y);
            }
            for (k, value) in xt.iter_mut().enumerate() {
                if moved[k] {
                    let column = self.columns[k];
                    *value = value.retract(&step[column..column + value.dimension()])?;
                }
            }
        };
        let yf: Vec<ScalarOf<E>> = (0..m)
            .map(|k| product(&y, k, &|j| rows[picked[j]].value))
            .collect();
        // Half-widths of the candidate box: at least what the Newton step
        // from `x̃` still reaches.
        let mut radius: Vec<ScalarOf<E>> = yf.iter().map(|v| v.abs().upper()).collect();
        for _ in 0..ENCLOSE_ATTEMPTS {
            let candidate: Vec<ScalarOf<E>> = (0..m)
                .map(|k| {
                    let r = ScalarOf::<E>::TWO.mul(radius[k]);
                    r.neg().union(r)
                })
                .collect();
            let mut boxed = zero.clone();
            for (k, &c) in cols.iter().enumerate() {
                boxed[c] = candidate[k];
            }
            let over = self.rows_over(&xt, &boxed, which).await?;
            // `I - Y J(Δ)`.
            let slack: Vec<Vec<ScalarOf<E>>> = (0..m)
                .map(|k| {
                    (0..m)
                        .map(|l| {
                            let yj = product(&y, k, &|j| over[picked[j]].slope[cols[l]]);
                            let identity = match k == l {
                                true => ScalarOf::<E>::ONE,
                                false => ScalarOf::<E>::ZERO,
                            };
                            identity.sub(yj)
                        })
                        .collect()
                })
                .collect();
            let krawczyk: Vec<ScalarOf<E>> = (0..m)
                .map(|k| {
                    let spread = (0..m).fold(ScalarOf::<E>::ZERO, |sum, l| {
                        sum.add(slack[k][l].mul(candidate[l]))
                    });
                    yf[k].neg().add(spread)
                })
                .collect();
            if (0..m).all(|k| krawczyk[k].is_subset_of(candidate[k])) {
                // `K(Δ) ⊆ Δ` alone gives a zero of `Y f` in `Δ`, which is
                // one of `f` only if `Y` is regular. `‖I - Y J(Δ)‖∞ < 1`
                // proves that for `Y` and every matrix of `J(Δ)`, and with
                // it that the zero is the only one in `Δ`: two of them
                // differ by `(I - Y J)` times that difference.
                let norm = slack
                    .iter()
                    .map(|row| {
                        row.iter()
                            .fold(ScalarOf::<E>::ZERO, |sum, e| sum.add(e.abs()))
                    })
                    .fold(ScalarOf::<E>::ZERO, |most, row| most.max(row));
                if !norm.definitely_less(ScalarOf::<E>::ONE) {
                    return Err(GeopError::new(format!(
                        "the constraints are not proven independent over the box around the solution: ‖I - Y J(Δ)‖∞ is {norm:?}, not below one, for rows {picked:?} over variables {cols:?}"
                    ))
                    .into());
                }
                for (k, &c) in cols.iter().enumerate() {
                    boxed[c] = krawczyk[k].intersect(candidate[k]);
                }
                return Ok((xt, boxed));
            }
            for k in 0..m {
                let reach = krawczyk[k].abs().upper();
                if reach.could_be_greater(radius[k]) {
                    radius[k] = reach;
                }
                if !radius[k].is_finite() {
                    return Err(singular(&rows).into());
                }
            }
        }
        Err(GeopError::new(format!(
            "could not enclose the solution: no box around it passed the Krawczyk test in {ENCLOSE_ATTEMPTS} attempts"
        ))
        .into())
    }

    /// The rows of the constraints `which` over the box of steps `steps`
    /// from `center` — the proven solution of the independent rows `pivots`
    /// — if what the proof is of is a solution of them all, as far as the
    /// box resolves: every limit `inactive` holds over it; every other row
    /// encloses zero there — else [`Rejected::Off`], those that do not —
    /// and adds nothing to the independent ones' rank over it.
    ///
    /// The rank is decided over this box, around the solution, and not only
    /// over the box the rows were chosen over, around where the solve
    /// ended: a constraint the others imply leaves the rank as it is
    /// wherever they all hold, and so over a box holding such a point; one
    /// they do not — a line tangent to a circle a point is on — is
    /// independent a hair off its singular point, where the free variables
    /// were left. Its value there is too small to tell from zero, its
    /// derivative is not.
    async fn verify(
        &self,
        center: &[E::Value],
        steps: &[ScalarOf<E>],
        which: &[usize],
        inactive: &[usize],
        pivots: &[(usize, usize)],
    ) -> Result<Vec<Row<ScalarOf<E>>>, Rejected<ScalarOf<E>>> {
        let limits = match inactive.is_empty() {
            true => Vec::new(),
            false => self.rows_over(center, steps, inactive).await?,
        };
        for row in limits {
            if row.value.could_be_greater(ScalarOf::<E>::ZERO) {
                return Err(GeopError::new(format!(
                    "the limit {} is not proven to hold over the box around the solution: it is {:?} there",
                    row.constraint, row.value
                ))
                .into());
            }
        }
        let picked: Vec<usize> = pivots.iter().map(|&(_, r)| r).collect();
        let rows = self.rows_over(center, steps, which).await?;
        let off: Vec<usize> = (0..rows.len())
            .filter(|i| !picked.contains(i) && !rows[*i].value.could_be_equal(ScalarOf::<E>::ZERO))
            .collect();
        if !off.is_empty() {
            return Err(Rejected::Off(off));
        }
        let rank = eliminate(rows.iter().map(|r| r.slope.clone()).collect(), self.n);
        let independent: Vec<usize> = (rank.pivots.iter())
            .map(|&(_, r)| rows[r].constraint)
            .filter(|c| !picked.iter().any(|&p| rows[p].constraint == *c))
            .collect();
        if rank.pivots.len() > picked.len() {
            return Err(GeopError::new(format!(
                "the constraints are singular at their solution: {independent:?} vanish over the box around the solution of the others, but are independent of them there — {} rows of rank {}",
                picked.len(),
                rank.pivots.len()
            ))
            .into());
        }
        Ok(rows)
    }

    /// Why the rows `off` of the constraints `which` — the rows chosen
    /// over the box of steps of half-width `radius` around `x`, where the
    /// solve ended, `selection` — do not vanish where the independent ones
    /// do: a conflict where one is proven within the box of half-width
    /// `radius`, or the tolerance if wider, around `x`, and otherwise not
    /// proven.
    ///
    /// The solution of the independent rows that left them off was found
    /// with the free variables held as they are, so it proves nothing about
    /// other values of those. What does: a combination `g` of the rows
    /// vanishes wherever they all do, and over the box `B`, by the mean
    /// value theorem, `g` is within `g(x) + ∇g(B) B`. Where that is
    /// definitely not zero, no point of `B` is a solution. The combination
    /// is the one the elimination that chose the rows reduced each of
    /// `off` by, sharpened: any combination serves, and that one makes
    /// `∇g` as small as the rows allow.
    async fn conflict(
        &self,
        x: &[E::Value],
        radius: ScalarOf<E>,
        which: &[usize],
        selection: &[Row<ScalarOf<E>>],
        off: &[usize],
    ) -> EncloseError<ScalarOf<E>> {
        let n = self.n;
        let proven = async || -> GeopResult<Option<EncloseError<ScalarOf<E>>>> {
            let within = radius.max(tolerance(self.system.scale)?);
            let at = (self.rows_over(x, &vec![ScalarOf::<E>::ZERO; n], which)).await?;
            let around = vec![within.neg().union(within); n];
            let over = self.rows_over(x, &around, which).await?;
            let reduced = eliminate(augmented(selection.iter().map(|r| r.slope.clone())), n);
            let (mut conflicting, mut with) = (Vec::new(), Vec::new());
            for (k, &o) in reduced.origin.iter().enumerate() {
                if !off.contains(&o) {
                    continue;
                }
                // Zero where it could be: a free choice, which keeps the
                // rows the elimination never combined out of `g`.
                let combination: Vec<ScalarOf<E>> = (0..selection.len())
                    .map(|j| reduced.rows[k][n + j])
                    .map(|a| match a.could_be_equal(ScalarOf::<E>::ZERO) {
                        true => ScalarOf::<E>::ZERO,
                        false => a.midpoint(),
                    })
                    .collect();
                let mut g = ScalarOf::<E>::ZERO;
                let mut gradient = vec![ScalarOf::<E>::ZERO; n];
                for (j, a) in combination.iter().enumerate().filter(|(_, a)| !a.is_zero()) {
                    g = g.add(a.mul(at[j].value));
                    for (slot, s) in gradient.iter_mut().zip(&over[j].slope) {
                        *slot = slot.add(a.mul(*s));
                    }
                }
                let reach = (gradient.iter()).fold(g, |sum, d| sum.add(d.mul(around[0])));
                if reach.could_be_equal(ScalarOf::<E>::ZERO) {
                    continue;
                }
                conflicting.push((selection[o].constraint, reach));
                for (j, a) in combination.iter().enumerate() {
                    let c = selection[j].constraint;
                    if j != o && !a.is_zero() && !off.contains(&j) && !with.contains(&c) {
                        with.push(c);
                    }
                }
            }
            with.sort_unstable();
            Ok((!conflicting.is_empty()).then_some(EncloseError::Conflict {
                conflicting,
                with,
                within,
            }))
        };
        let named: Vec<usize> = off.iter().map(|&o| selection[o].constraint).collect();
        match proven().await {
            Ok(Some(conflict)) => conflict,
            Ok(None) => EncloseError::Unproven(GeopError::new(format!(
                "the constraints {named:?} do not vanish over the box around the solution of the others, but no conflict is proven near where the solve ended: a singular configuration"
            ))),
            Err(e) => EncloseError::Unproven(e.with_context(format!(
                "looking for a conflict of the constraints {named:?}"
            ))),
        }
    }
}
