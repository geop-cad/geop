//! Solving constraints: the one engine behind sketches, assemblies and the
//! parameters of programs.
//!
//! A system has **parameters** — numbers, and poses of rigid bodies
//! ([`Param`]) — and **residuals** ([`Residual`]): functions of a few
//! parameters that are zero exactly when what they stand for holds. Solving
//! moves the parameters that are free until every residual vanishes,
//! changing them as little as it can.
//!
//! - **Residuals are lengths.** Each is in units of length — a
//!   dimensionless one is multiplied by the system's characteristic size —
//!   so none dominates another by its choice of units, and one relative
//!   tolerance decides when a residual holds.
//! - **Residuals are constraints.** A solve minimizes what it is asked to
//!   prefer — pulls, and staying put — among the configurations where every
//!   residual holds, by constrained Levenberg–Marquardt
//!   ([`crate::solvers::least_squares`]): the residuals hold exactly, and no
//!   preference can buy itself a little violation of them. What the
//!   residuals leave free and nothing pulls stays exactly where it is.
//! - **Honest enclosures throughout.** Every residual is computed as a
//!   [`crate::dual::Dual`] over the system's scalar — any
//!   [`crate::scalars::Scalar`] — so its gradient is exact and the
//!   `sqrt`, `sin` and `PI` it needs are enclosed rather than rounded; the
//!   minimizer takes a step only where it definitely helps.
//! - **Increments.** The variables are increments from where the
//!   parameters are when a solve starts: a number's added to it, a pose's —
//!   a translation and a turn about its body's center — composed with it
//!   (see [`Pose::moved`]). A solve that changes nothing leaves everything exactly
//!   where it was, and a pose has no singular configurations to cross.
//! - **Pulls.** What a drag pulls where — a number towards a value, a body
//!   towards a pose, a point of a body towards a point ([`Pull`]) — is a
//!   preference: what is pulled follows exactly as far as the residuals
//!   allow, and they win wherever they disagree.
//!
//! `geop-ops` builds systems of rigid bodies and the constraints between them
//! on this (its `mates`); `geop-core-sketch` builds a sketch's.

use crate::{
    dual::Dual,
    geop_error::{GeopError, GeopResult},
    primitives::{Pose, Quaternion},
    scalars::{Ring, Scalar},
    solvers::least_squares::{self, Evaluation, Stop},
    vector::Vector3,
};

mod linalg;

pub use linalg::rank;
use linalg::{eliminate, inverse, is_zero};

/// A residual holds once it is within this fraction of the system's size:
/// one part in `RELATIVE_TOLERANCE`.
pub const RELATIVE_TOLERANCE: i64 = 1_000_000_000;

/// How strongly a [`Pull`] pulls, as a ratio: the unit the other
/// preferences of a solve are weighed in. Against the residuals it has no
/// strength at all — they hold exactly (see [`solve`]).
///
/// The preferences are ranked a hundred to one, [`PULL`] over
/// [`TURN_RESISTANCE`] over [`DAMPING`]: far enough apart that each yields
/// to the one above (to a ten-thousandth, as they enter squared), and no
/// further — the minimizer resolves the weakest only as well as the
/// strongest lets it, and weights ten orders of magnitude apart in the sum
/// leave the weakest unresolved.
const PULL: (i64, i64) = (1, 1);

/// How strongly a body pulled by a point would rather not turn: a point can
/// be pulled anywhere along more than one turn, and a body dragged with
/// another hanging on it could tip over to spare that one from moving.
/// Weak against the pull — a crank still turns about its axis, lagging the
/// pointer by a ten-thousandth of its turn — and strong against
/// [`DAMPING`]: the others give way before the dragged body turns.
const TURN_RESISTANCE: (i64, i64) = (1, 100);

/// How strongly every free parameter not pulled is held where it is: the
/// damping that makes a solve prefer, among the many configurations an
/// under-constrained system allows, the one nearest where things are. A
/// parameter the residuals leave free stays put; one they need moves as far
/// as they need, and no further.
const DAMPING: (i64, i64) = (1, 10_000);

/// The most steps a solve takes: how hard it tries, never what its answer
/// means.
const MAX_ITERATIONS: usize = 200;

/// Newton steps polishing a solution before it is enclosed: each squares the
/// error, so a handful take [`RELATIVE_TOLERANCE`] to rounding.
const POLISH_STEPS: usize = 6;

/// How many candidate boxes [`enclose`] tries, each twice as wide as
/// the last one needed: how hard it tries, never what a verified box means.
const ENCLOSE_ATTEMPTS: usize = 24;

/// Variables per pose: a translation and a turn.
const POSE_VARS: usize = 6;

fn ratio<S: Scalar>((num, den): (i64, i64)) -> S {
    S::from_ratio(num, den).expect("a weight's denominator is not zero")
}

/// Something a solve may change.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Param<S: Scalar> {
    Scalar(S),
    /// A rigid body's pose, and the point it turns about while solved — in
    /// its own frame; its middle, ideally.
    Pose {
        pose: Pose<S>,
        center: Vector3<S>,
    },
}

impl<S: Scalar> Param<S> {
    fn vars(&self) -> usize {
        match self {
            Param::Scalar(_) => 1,
            Param::Pose { .. } => POSE_VARS,
        }
    }
}

/// A parameter's value while a residual is evaluated.
#[derive(Clone, Debug)]
pub enum Value<T: Scalar> {
    Scalar(T),
    Pose(Pose<T>),
}

impl<T: Scalar> Value<T> {
    /// The number it is; fails for a pose.
    pub fn scalar(&self) -> GeopResult<T> {
        match self {
            Value::Scalar(v) => Ok(*v),
            Value::Pose(_) => Err(GeopError::new("a pose is no number")),
        }
    }

    /// The body it places; fails for a number.
    pub fn pose(&self) -> GeopResult<&Pose<T>> {
        match self {
            Value::Pose(placed) => Ok(placed),
            Value::Scalar(_) => Err(GeopError::new("a number is no pose")),
        }
    }
}

/// Something that holds when its residuals vanish.
pub trait Residual<S: Scalar, const N: usize> {
    /// The parameters it depends on, by index in the system: what
    /// [`Residual::eval`] is handed the values of, in this order. Their
    /// variables together are at most `N`.
    fn params(&self) -> &[usize];

    /// Its residuals at `values`, each a length — appended to `out`. `Err`
    /// where one is undecidable (a direction of zero length), which makes
    /// `values` infeasible.
    fn eval(&self, values: &[Value<Dual<S, N>>], out: &mut Vec<Dual<S, N>>) -> GeopResult<()>;
}

/// What a solve pulls a parameter towards (see the crate docs).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Pull<S: Scalar> {
    /// A number towards `target`.
    Scalar { param: usize, target: S },
    /// A body towards `target`.
    Pose { param: usize, target: Pose<S> },
    /// The point `local` of a body — in its own frame — towards the point
    /// `target`: a drag.
    Point {
        param: usize,
        local: Vector3<S>,
        target: Vector3<S>,
    },
}

impl<S: Scalar> Pull<S> {
    fn param(&self) -> usize {
        match *self {
            Pull::Scalar { param, .. } | Pull::Pose { param, .. } | Pull::Point { param, .. } => {
                param
            }
        }
    }
}

/// One minimization of a solve: why it stopped, after how many steps, and
/// the largest residual it left — an upper bound.
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(bound = "S: Scalar")]
pub struct Phase<S: Scalar> {
    pub stop: Stop,
    pub iterations: usize,
    /// Steps proposed and turned down on the way (see
    /// [`least_squares::Outcome::turned_down`]).
    pub turned_down: usize,
    #[serde(with = "crate::scalars::as_f64")]
    pub max_residual: S,
}

/// Where a solve put the parameters, and which residuals hold there.
pub struct Solution<S: Scalar> {
    pub params: Vec<Param<S>>,
    pub report: Report<S>,
}

/// Which residuals hold.
#[derive(Clone, Debug, PartialEq)]
pub struct Report<S: Scalar> {
    /// Every residual holds, to [`RELATIVE_TOLERANCE`] of the size.
    pub converged: bool,
    /// The largest remaining residual, in units of length: an upper bound.
    pub max_residual: S,
    pub iterations: usize,
    /// Each minimization of the solve.
    pub phases: Vec<Phase<S>>,
    /// The residuals that do not hold, by index.
    pub failed: Vec<usize>,
}

/// What a solve may do with a parameter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mobility {
    /// Nothing: it stays exactly as it is.
    Fixed,
    /// Change it as far as the residuals need, and no further: it is held
    /// where it is, by [`DAMPING`].
    Held,
    /// Change it as the residuals have it, and hold it nowhere: it follows
    /// the others — a joint's coordinate, measured from its bodies. Held as
    /// well, it would pull against them: a hand on a turned forearm kept
    /// only part of its turn in the world, the rest given to its joint's
    /// coordinate, which the damping held as stiffly as the hand.
    Follows,
}

impl Mobility {
    /// Whether a solve may change it.
    pub fn free(self) -> bool {
        self != Mobility::Fixed
    }
}

/// The arguments every function here is given — parameters, what a solve
/// may do with each, the residuals between them, and their size — as one
/// borrowed view, which the questions are asked of.
/// Which variable each slot of a residual's gradient is: `(slot,
/// variable)`.
type Seeded = Vec<(usize, usize)>;

/// A residual group's values, and its seeded slots.
type Group<S, const N: usize> = (Vec<Dual<S, N>>, Seeded);

/// Where each parameter's variables start, if it is free.
fn variable_offsets<S: Scalar>(params: &[Param<S>], mobility: &[Mobility]) -> Vec<Option<usize>> {
    let mut n = 0;
    params
        .iter()
        .zip(mobility)
        .map(|(p, &free)| {
            free.free().then(|| {
                n += p.vars();
                n - p.vars()
            })
        })
        .collect()
}

/// How many variables there are.
fn variable_count<S: Scalar>(params: &[Param<S>], mobility: &[Mobility]) -> usize {
    params
        .iter()
        .zip(mobility)
        .filter(|(_, free)| free.free())
        .map(|(p, _)| p.vars())
        .sum()
}

/// Parameter `param` at the increments `x`, its own variables — if it
/// is free — seeded from slot 0 on.
fn parameter_value<S: Scalar>(
    params: &[Param<S>],
    scale: S,
    param: usize,
    x: &[S],
    offset: Option<usize>,
) -> GeopResult<Value<Dual<S, POSE_VARS>>> {
    let var = |k: usize| match offset {
        Some(o) => Dual::var(x[o + k], k),
        None => Dual::cst(S::ZERO),
    };
    Ok(match &params[param] {
        Param::Scalar(v) => Value::Scalar(Dual::cst(*v).add(var(0))),
        Param::Pose { pose, center } => Value::Pose(pose.map(Dual::cst).moved(
            &pose.apply(center).map(Dual::cst),
            Vector3::from_array([0, 1, 2].map(var)),
            turn_variables(scale, x, offset)?,
        )?),
    })
}

/// The turn variables of the pose parameter at `offset` (`w` of
/// [`Pose::moved`]), over `M` gradient slots from 3 on. They are
/// lengths too — how far a point the system's size from the center
/// turns — so that a step of the minimizer is as long for a turn as for
/// a move.
fn turn_variables<S: Scalar, const M: usize>(
    scale: S,
    x: &[S],
    offset: Option<usize>,
) -> GeopResult<Vector3<Dual<S, M>>> {
    let per_length = Dual::cst(S::ONE.div(scale)?);
    let var = |k: usize| match offset {
        Some(o) => Dual::var(x[o + k], k),
        None => Dual::cst(S::ZERO),
    };
    Ok(Vector3::from_array([3, 4, 5].map(var)).prod_scalar(per_length))
}

/// Every parameter a residual or one of `pulls` depends on, at the
/// increments `x` (see [`parameter_value`]) — each computed once, however
/// many depend on it; those nothing depends on are not computed at all.
fn parameter_values<S: Scalar, const N: usize>(
    params: &[Param<S>],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
    x: &[S],
    pulls: &[(Pull<S>, S)],
    offsets: &[Option<usize>],
) -> Vec<Option<GeopResult<Value<Dual<S, POSE_VARS>>>>> {
    let mut locals: Vec<_> = params.iter().map(|_| None).collect();
    let needed = residuals.iter().flat_map(|r| r.params().iter().copied());
    for p in needed.chain(pulls.iter().map(|(pull, _)| pull.param())) {
        if locals[p].is_none() {
            locals[p] = Some(parameter_value(params, scale, p, x, offsets[p]));
        }
    }
    locals
}

/// The values of `params` among `locals`, their variables seeded one
/// after the other, and which variable of `x` each slot is.
fn seeded_values<S: Scalar, const N: usize>(
    params: &[Param<S>],
    which: &[usize],
    locals: &[Option<GeopResult<Value<Dual<S, POSE_VARS>>>>],
    offsets: &[Option<usize>],
) -> GeopResult<(Vec<Value<Dual<S, N>>>, Seeded)> {
    let mut seeded = Vec::new();
    let mut values = Vec::new();
    for &p in which {
        let slot = seeded.len();
        let vars = if offsets[p].is_some() {
            params[p].vars()
        } else {
            0
        };
        if let Some(o) = offsets[p] {
            seeded.extend((0..vars).map(|k| (slot + k, o + k)));
        }
        if seeded.len() > N {
            return Err(GeopError::new(format!(
                "a residual depends on {} variables, more than the {N} supported",
                seeded.len()
            )));
        }
        let embed = |d: Dual<S, POSE_VARS>| d.embed::<N>(slot, vars);
        values.push(
            match locals[p]
                .as_ref()
                .expect("computed for every parameter something depends on")
                .as_ref()
                .map_err(|e| GeopError::new(format!("{e}")))?
            {
                Value::Scalar(v) => Value::Scalar(embed(*v)),
                Value::Pose(placed) => Value::Pose(placed.map(embed)),
            },
        );
    }
    Ok((values, seeded))
}

/// Every residual group at `x` — one per residual, then one per pull,
/// each pull weighted by its weight.
fn residual_groups<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
    x: &[S],
    pulls: &[(Pull<S>, S)],
) -> Vec<GeopResult<Group<S, N>>> {
    let offsets = variable_offsets(params, mobility);
    let locals = parameter_values(params, residuals, scale, x, pulls, &offsets);
    let mut groups = Vec::new();
    for r in residuals {
        groups.push((|| {
            let (values, seeded) = seeded_values(params, r.params(), &locals, &offsets)?;
            let mut out = Vec::new();
            r.eval(&values, &mut out)?;
            Ok((out, seeded))
        })());
    }
    let l = Dual::cst(scale);
    for (p, weight) in pulls {
        let pull = Dual::cst(*weight);
        groups.push((|| {
            let (values, seeded) = seeded_values(params, &[p.param()], &locals, &offsets)?;
            let mut out = Vec::new();
            match p {
                Pull::Scalar { target, .. } => {
                    out.push(values[0].scalar()?.sub(Dual::cst(*target)).mul(pull));
                }
                Pull::Pose { target, .. } => {
                    let placed = values[0].pose()?;
                    let at = placed.position().sub(&target.position().map(Dual::cst));
                    out.extend([0, 1, 2].map(|k| at[k].mul(pull)));
                    // How far the body is turned from the target: its
                    // rotation quaternion's difference from the target's
                    // — of the two quaternions of the target's rotation,
                    // the nearer one (either, where they are equally
                    // near). It grows the whole way to a half turn, so it
                    // resists turning a body over.
                    let r = placed.rotation();
                    let mut t: Quaternion<S> = target.rotation();
                    if r.map(|c| c.v).dot(&t).definitely_less(S::ZERO) {
                        t = t.scale(S::ONE.neg());
                    }
                    let scaled = l.mul(pull);
                    out.extend(
                        r.components()
                            .iter()
                            .zip(t.components())
                            .map(|(c, t)| c.sub(Dual::cst(t)).mul(scaled)),
                    );
                }
                Pull::Point { local, target, .. } => {
                    let at = values[0]
                        .pose()?
                        .apply(&local.map(Dual::cst))
                        .sub(&target.map(Dual::cst));
                    out.extend([0, 1, 2].map(|k| at[k].mul(pull)));
                    let resist = Dual::cst(ratio(TURN_RESISTANCE)).mul(l);
                    let w: Vector3<Dual<S, N>> = turn_variables(scale, x, offsets[p.param()])?;
                    out.extend([0, 1, 2].map(|k| w[k].mul(resist)));
                }
            }
            Ok((out, seeded))
        })());
    }
    groups
}

/// Every residual's Jacobian, row by row over every variable, where the
/// parameters are now. A residual undecidable there contributes no rows
/// — [`report`] lists it as failed.
fn jacobian<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
) -> Vec<Vec<S>> {
    let n = variable_count(params, mobility);
    let mut rows = Vec::new();
    for (rs, seeded) in residual_groups(params, mobility, residuals, scale, &vec![S::ZERO; n], &[])
        .into_iter()
        .flatten()
    {
        for r in rs {
            let mut row = vec![S::ZERO; n];
            for &(slot, i) in &seeded {
                row[i] = row[i].add(r.d[slot]);
            }
            rows.push(row);
        }
    }
    rows
}

/// Every residual over the box `x` — each free scalar parameter's whole
/// value, by variable — and its gradient with respect to every variable:
/// an enclosure of both, for every point of the box. Fails where a
/// residual is undecidable somewhere in it.
fn rows_over<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    x: &[S],
) -> GeopResult<Vec<Row<S>>> {
    let offsets = variable_offsets(params, mobility);
    let mut out = Vec::new();
    for (residual, r) in residuals.iter().enumerate() {
        let mut seeded = Vec::new();
        let mut values = Vec::new();
        for &p in r.params() {
            values.push(match (&params[p], offsets[p]) {
                (Param::Scalar(_), Some(o)) => {
                    let slot = seeded.len();
                    seeded.push((slot, o));
                    Value::Scalar(Dual::var(x[o], slot))
                }
                (Param::Scalar(v), None) => Value::Scalar(Dual::cst(*v)),
                (Param::Pose { pose, .. }, None) => Value::Pose(pose.map(Dual::cst)),
                (Param::Pose { .. }, Some(_)) => {
                    return Err(GeopError::new(
                        "enclosing the solution of free poses is not supported",
                    ));
                }
            });
        }
        let mut rs = Vec::new();
        r.eval(&values, &mut rs)?;
        for r in &rs {
            let mut slope = vec![S::ZERO; x.len()];
            for &(slot, i) in &seeded {
                slope[i] = slope[i].add(r.d[slot]);
            }
            out.push(Row {
                residual,
                value: r.v,
                slope,
            });
        }
    }
    Ok(out)
}

/// The Krawczyk proof of [`enclose`] for the independent rows
/// `pivots` — `(variable, row)` pairs — from `x`: an enclosure of their
/// solution, every variable no pivot determines as it is in `x`.
fn prove<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    x: Vec<S>,
    pivots: &[(usize, usize)],
) -> GeopResult<Vec<S>> {
    if pivots.is_empty() {
        return Ok(x);
    }
    let (cols, picked): (Vec<usize>, Vec<usize>) = pivots.iter().copied().unzip();
    let m = cols.len();
    let square = |rows: &[Row<S>]| -> Vec<Vec<S>> {
        picked
            .iter()
            .map(|&r| cols.iter().map(|&c| rows[r].slope[c]).collect())
            .collect()
    };
    let singular = |rows: &[Row<S>]| {
        let rows: Vec<(S, &Vec<S>)> = rows.iter().map(|r| (r.value, &r.slope)).collect();
        GeopError::new(format!(
            "the residuals are singular at their solution: rows {picked:?} over variables {cols:?} of {rows:?}"
        ))
    };
    let product = |y: &[Vec<S>], k: usize, column: &dyn Fn(usize) -> S| {
        (0..m).fold(S::ZERO, |sum, j| sum.add(y[k][j].mul(column(j))))
    };

    // Newton, on the determined variables. Every iterate is only a seed
    // for the next, so it is sharpened (a free choice, and without it the
    // width of each step compounds); the last pass takes no step: it is
    // the one the box is proven around, and its center is the last
    // iterate.
    let mut polished = 0;
    let mut xt = x;
    let (rows, y) = loop {
        let rows = rows_over(params, mobility, residuals, &xt)?;
        let y = inverse(&square(&rows)).ok_or_else(|| singular(&rows))?;
        if polished == POLISH_STEPS {
            break (rows, y);
        }
        polished += 1;
        for (k, &c) in cols.iter().enumerate() {
            let step = product(&y, k, &|j| rows[picked[j]].value);
            xt[c] = xt[c].sub(step).sharpen();
        }
    };
    let yf: Vec<S> = (0..m)
        .map(|k| product(&y, k, &|j| rows[picked[j]].value))
        .collect();
    let center: Vec<S> = cols.iter().map(|&c| xt[c]).collect();
    // Half-widths of the candidate box: at least what the Newton step
    // from `x̃` still reaches.
    let mut radius: Vec<S> = yf.iter().map(|v| v.abs().upper()).collect();
    for _ in 0..ENCLOSE_ATTEMPTS {
        let candidate: Vec<S> = (0..m)
            .map(|k| {
                let r = S::TWO.mul(radius[k]);
                center[k].sub(r).union(center[k].add(r))
            })
            .collect();
        let mut boxed = xt.clone();
        for (k, &c) in cols.iter().enumerate() {
            boxed[c] = candidate[k];
        }
        let over = rows_over(params, mobility, residuals, &boxed)?;
        let offset: Vec<S> = (0..m).map(|k| candidate[k].sub(center[k])).collect();
        // `I - Y J(X)`.
        let slack: Vec<Vec<S>> = (0..m)
            .map(|k| {
                (0..m)
                    .map(|l| {
                        let yj = product(&y, k, &|j| over[picked[j]].slope[cols[l]]);
                        let identity = if k == l { S::ONE } else { S::ZERO };
                        identity.sub(yj)
                    })
                    .collect()
            })
            .collect();
        let krawczyk: Vec<S> = (0..m)
            .map(|k| {
                let spread = (0..m).fold(S::ZERO, |sum, l| sum.add(slack[k][l].mul(offset[l])));
                center[k].sub(yf[k]).add(spread)
            })
            .collect();
        if (0..m).all(|k| krawczyk[k].is_subset_of(candidate[k])) {
            // `K(X) ⊆ X` alone gives a zero of `Y f` in `X`, which is one
            // of `f` only if `Y` is regular. `‖I - Y J(X)‖∞ < 1` proves
            // that for `Y` and every matrix of `J(X)`, and with it that
            // the zero is the only one in `X`: two of them differ by
            // `(I - Y J)` times that difference.
            let norm = slack
                .iter()
                .map(|row| row.iter().fold(S::ZERO, |sum, e| sum.add(e.abs())))
                .fold(S::ZERO, |most, row| most.max(row));
            if !norm.definitely_less(S::ONE) {
                return Err(GeopError::new(format!(
                    "the residuals are not proven independent over the box around the solution: ‖I - Y J(X)‖∞ is {norm:?}, not below one, for rows {picked:?} over variables {cols:?}"
                )));
            }
            for (k, &c) in cols.iter().enumerate() {
                boxed[c] = krawczyk[k].intersect(candidate[k]);
            }
            return Ok(boxed);
        }
        for k in 0..m {
            let reach = krawczyk[k].sub(center[k]).abs().upper();
            if reach.could_be_greater(radius[k]) {
                radius[k] = reach;
            }
            if !radius[k].is_finite() {
                return Err(singular(&rows));
            }
        }
    }
    Err(GeopError::new(format!(
        "could not enclose the solution: no box around it passed the Krawczyk test in {ENCLOSE_ATTEMPTS} attempts"
    )))
}

/// Whether every row but the independent `pivots` encloses zero over
/// `boxed`, the proven solution of those: else the conflict, with the
/// rows each such row was reduced against when the pivots were chosen
/// over `around` — what it conflicts with.
fn implied<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    boxed: &[S],
    pivots: &[(usize, usize)],
    around: &[S],
) -> Result<(), EncloseError<S>> {
    let picked: Vec<usize> = pivots.iter().map(|&(_, r)| r).collect();
    let rows = rows_over(params, mobility, residuals, boxed)?;
    let conflicting: Vec<usize> = (0..rows.len())
        .filter(|i| !picked.contains(i) && !rows[*i].value.could_be_equal(S::ZERO))
        .collect();
    if conflicting.is_empty() {
        return Ok(());
    }
    // The elimination that chose the pivots again, each row carrying
    // which rows it is a combination of.
    let n = boxed.len();
    let selection = rows_over(params, mobility, residuals, around)?;
    let m = selection.len();
    let augmented = selection
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let mut row = r.slope.clone();
            row.extend((0..m).map(|j| if i == j { S::ONE } else { S::ZERO }));
            row
        })
        .collect();
    let reduced = eliminate(augmented, n);
    let mut with = Vec::new();
    for (k, &o) in reduced.origin.iter().enumerate() {
        if !conflicting.contains(&o) {
            continue;
        }
        for (j, row) in selection.iter().enumerate() {
            if reduced.rows[k][n + j].definitely_not_equal(S::ZERO)
                && !conflicting.contains(&j)
                && !with.contains(&row.residual)
            {
                with.push(row.residual);
            }
        }
    }
    with.sort_unstable();
    Err(EncloseError::Conflict {
        conflicting: conflicting
            .into_iter()
            .map(|i| (rows[i].residual, rows[i].value))
            .collect(),
        with,
    })
}
/// Every residual at `x`, and its Jacobian row by row over every
/// variable: the system's residuals as constraints if `hard`, else in
/// the sum — and in the sum the pulls, each weighted by its weight.
/// `None` where one is undecidable, which makes `x` infeasible.
pub fn evaluate<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
    x: &[S],
    pulls: &[(Pull<S>, S)],
    hard: bool,
) -> Option<Evaluation<S>> {
    let mut evaluation = Evaluation::default();
    for (i, group) in residual_groups(params, mobility, residuals, scale, x, pulls)
        .into_iter()
        .enumerate()
    {
        let (rs, seeded) = group.ok()?;
        let rows = if hard && i < residuals.len() {
            &mut evaluation.constraints
        } else {
            &mut evaluation.sum
        };
        for r in rs {
            let mut row = vec![S::ZERO; x.len()];
            for &(slot, i) in &seeded {
                row[i] = row[i].add(r.d[slot]);
            }
            // An unbounded residual or slope is no number either.
            if !r.v.is_finite() || !row.iter().all(|d| d.is_finite()) {
                return None;
            }
            rows.push(r.v, row);
        }
    }
    Some(evaluation)
}

/// Minimizes the pulls' sum among the points where the residuals hold if
/// `hard` — else the residuals' and the pulls' sum — from the current
/// parameters, and gives every free one where it ended up.
fn minimize<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
    pulls: &[(Pull<S>, S)],
    hard: bool,
) -> GeopResult<(Vec<Param<S>>, Phase<S>)> {
    let n = variable_count(params, mobility);
    if n == 0 {
        let phase = Phase {
            stop: Stop::NoChange { rejected: None },
            iterations: 0,
            turned_down: 0,
            max_residual: report(params, mobility, residuals, scale)?.max_residual,
        };
        return Ok((params.to_vec(), phase));
    }
    let result = least_squares::minimize(
        |x| evaluate(params, mobility, residuals, scale, x, pulls, hard),
        vec![S::ZERO; n],
        least_squares::Options {
            max_iterations: MAX_ITERATIONS,
            max_step: scale,
        },
    )?;
    let mut moved = params.to_vec();
    for (p, offset) in variable_offsets(params, mobility).into_iter().enumerate() {
        let Some(o) = offset else {
            continue;
        };
        // Not moved at all — nothing pulled it — it is left exactly as
        // it was, not recomposed with a zero step.
        if result.x[o..o + params[p].vars()]
            .iter()
            .all(|v| is_zero(*v))
        {
            continue;
        }
        // Where the parameter ended up is the last step's, as wide as that
        // step from the sharp seed made it: nothing is narrowed here (see
        // [`least_squares::minimize`]).
        moved[p] = match (
            parameter_value(params, scale, p, &result.x, Some(o))?,
            &params[p],
        ) {
            (Value::Scalar(v), Param::Scalar(_)) => Param::Scalar(v.v),
            (Value::Pose(pose), &Param::Pose { center, .. }) => Param::Pose {
                pose: pose.map(|c| c.v),
                center,
            },
            _ => unreachable!("a parameter's value is of its kind"),
        };
    }
    let phase = Phase {
        stop: result.stop,
        iterations: result.iterations,
        turned_down: result.turned_down,
        max_residual: report(&moved, mobility, residuals, scale)?.max_residual,
    };
    Ok((moved, phase))
}

/// Every parameter that is [`Mobility::Held`] but `except`, held where
/// it is, by [`DAMPING`].
fn stays<S: Scalar>(
    params: &[Param<S>],
    mobility: &[Mobility],
    except: &[usize],
) -> Vec<(Pull<S>, S)> {
    params
        .iter()
        .zip(mobility)
        .enumerate()
        .filter(|(param, (_, free))| **free == Mobility::Held && !except.contains(param))
        .map(|(param, (p, _))| {
            let stay = match *p {
                Param::Scalar(target) => Pull::Scalar { param, target },
                Param::Pose { pose, .. } => Pull::Pose {
                    param,
                    target: pose,
                },
            };
            (stay, ratio(DAMPING))
        })
        .collect()
}

/// Where the free parameters go so every residual holds, changing them as
/// little as the residuals allow — pulled as `pulls` ask (see the module
/// docs). `params` is left as it is.
///
/// The residuals are constraints, met exactly; the pulls and the
/// damping (see [`DAMPING`]) only choose among the configurations that
/// meet them — the one nearest where the pulls want things and, among
/// those, nearest where things were. So a part a drag does not need to
/// move stays put, and from far off things move as little as they can
/// rather than wherever the minimizer's first steps would throw them:
/// turned upside down, say, into a configuration they cannot get back
/// out of.
///
/// The parameters are given even if the residuals cannot all hold, to
/// where they come closest — the report says which do not.
pub fn solve<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
    pulls: &[Pull<S>],
) -> GeopResult<Solution<S>> {
    let pulled: Vec<usize> = pulls.iter().map(Pull::param).collect();
    let mut sum: Vec<(Pull<S>, S)> = pulls.iter().map(|p| (*p, ratio(PULL))).collect();
    sum.extend(stays(params, mobility, &pulled));
    let (mut solved, phase) = minimize(params, mobility, residuals, scale, &sum, true)?;
    let mut phases = vec![phase];
    if !report(&solved, mobility, residuals, scale)?.converged {
        // The residuals cannot all hold — or the constrained minimization
        // could not tell how to meet them: as near as they come, least
        // squares, still damped, so that what they leave free stays put.
        let stays = stays(&solved, mobility, &[]);
        let (moved, phase) = minimize(&solved, mobility, residuals, scale, &stays, false)?;
        solved = moved;
        phases.push(phase);
    }
    let held = report(&solved, mobility, residuals, scale)?;
    Ok(Solution {
        report: Report {
            iterations: phases.iter().map(|p| p.iterations).sum(),
            phases,
            ..held
        },
        params: solved,
    })
}

/// Which residuals hold at `params`.
pub fn report<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
) -> GeopResult<Report<S>> {
    let x = vec![S::ZERO; variable_count(params, mobility)];
    let tol = scale.div(S::from_i64(RELATIVE_TOLERANCE))?;
    let mut failed = Vec::new();
    let mut max_residual = S::ZERO;
    for (i, group) in residual_groups(params, mobility, residuals, scale, &x, &[])
        .into_iter()
        .enumerate()
    {
        let holds = match group {
            Ok((rs, _)) => rs.iter().all(|r| {
                let size = r.v.abs().upper();
                if size.could_be_greater(max_residual) {
                    max_residual = size;
                }
                r.v.is_finite() && !r.v.abs().definitely_greater(tol)
            }),
            Err(_) => {
                max_residual = S::INFINITY;
                false
            }
        };
        if !holds {
            failed.push(i);
        }
    }
    Ok(Report {
        converged: failed.is_empty(),
        max_residual,
        iterations: 0,
        phases: Vec::new(),
        failed,
    })
}

/// Which variables — in the order of the free parameters' — can move to
/// first order without changing any residual, and how many independent
/// ways there are to move: the degrees of freedom left.
pub fn free_variables<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
) -> (Vec<bool>, usize) {
    linalg::free_variables(
        jacobian(params, mobility, residuals, scale),
        variable_count(params, mobility),
    )
}

/// A basis of the directions the free variables can move in, to first
/// order, without changing any residual, where the parameters are now:
/// each vector over every variable (see [`variables`]).
pub fn null_space<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
) -> Vec<Vec<S>> {
    linalg::null_space(
        jacobian(params, mobility, residuals, scale),
        variable_count(params, mobility),
    )
}

/// The variables of parameter `param` among every variable, if it is
/// free.
pub fn variables<S: Scalar>(
    params: &[Param<S>],
    mobility: &[Mobility],
    param: usize,
) -> Option<std::ops::Range<usize>> {
    let offset = variable_offsets(params, mobility)[param]?;
    Some(offset..offset + params[param].vars())
}

/// An enclosure of the exact solution near where the free parameters —
/// numbers, all of them — are now, which is a solution to the solver's
/// tolerance; by variable.
///
/// The residuals determine some variables and leave the rest free. The
/// free ones are the designer's choice, and stay exactly as they are.
/// The determined ones are first polished by Newton's method on the
/// independent residuals, then enclosed by the Krawczyk test: for a box
/// `X` around the polished `x̃`, with `Y` any approximate inverse of the
/// Jacobian at `x̃`,
///
/// ```text
/// K(X) = x̃ - Y f(x̃) + (I - Y J(X)) (X - x̃)
/// ```
///
/// and `K(X) ⊆ X` together with `‖I - Y J(X)‖∞ < 1` prove that `X` holds
/// exactly one solution of the independent residuals — in interval
/// arithmetic, so the proof is rigorous. `K(X) ⊆ X` gives a zero of `Y f`
/// in `X` (Brouwer); the norm bound makes `Y` and every matrix of `J(X)`
/// regular, so it is a zero of `f`, the independent residuals stay
/// independent over the whole box, and no second zero exists. The box is
/// widened until the first test passes; its width is then how precisely
/// the residuals pin the solution down, and `K(X) ∩ X` is returned.
///
/// Nothing is sharpened but the Newton iterates, each only a seed for the
/// next: `Y`, `f(x̃)` and `J(X)` are enclosures, and so is the box.
///
/// **Which residuals are independent** is decided over a box, not at a
/// point. A redundant residual — a rectangle's fourth right angle, an arc
/// concentric with a center it already has — says what the others do,
/// often only on the solution. Where the parameters are now, a solution
/// only to the solver's tolerance, it looks independent by a pivot about
/// as small as that tolerance; taken as independent, it makes the
/// solution a curve rather than a point, which nothing proves. So the
/// independent rows are chosen by eliminating the Jacobian over a box
/// around where the parameters are — first the point itself, then a box
/// as wide as the solver's tolerance, then twice as wide, and so on: a
/// pivot that could be zero somewhere in the box is no pivot, and once
/// the box reaches the solution a redundant row's pivot straddles zero.
/// Each new choice is put to the Krawczyk test.
///
/// The residuals left out are then checked over the proven box: one that
/// holds wherever the independent ones do encloses zero there. One that
/// does not is a conflict — [`EncloseError::Conflict`], naming it and
/// the residuals it depends on. (Interval arithmetic can prove a
/// conflict, never its absence: a residual enclosing zero agrees with
/// the others to every digit the box resolves.)
///
/// Fails if no choice passes: a singular Jacobian — a tangency the
/// residuals only just meet, say — leaves the solution unproven, and
/// nothing narrower than that is honest.
pub fn enclose<S: Scalar, const N: usize>(
    params: &[Param<S>],
    mobility: &[Mobility],
    residuals: &[&dyn Residual<S, N>],
    scale: S,
) -> Result<Vec<S>, EncloseError<S>> {
    let offsets = variable_offsets(params, mobility);
    let mut x = vec![S::ZERO; variable_count(params, mobility)];
    for (p, offset) in offsets.iter().enumerate() {
        match (&params[p], offset) {
            (Param::Scalar(v), Some(o)) => x[*o] = *v,
            (Param::Pose { .. }, Some(_)) => {
                return Err(GeopError::new(
                    "enclosing the solution of free poses is not supported",
                )
                .into());
            }
            _ => {}
        }
    }
    let n = x.len();
    let tolerance = scale.div(S::from_i64(RELATIVE_TOLERANCE))?;
    let mut radius = S::ZERO;
    let mut tried: Vec<Vec<(usize, usize)>> = Vec::new();
    let mut failure = None;
    for attempt in 0..ENCLOSE_ATTEMPTS {
        if attempt == 1 {
            radius = tolerance;
        } else if attempt > 1 {
            radius = S::TWO.mul(radius);
        }
        let around: Vec<S> = x
            .iter()
            .map(|v| v.sub(radius).union(v.add(radius)))
            .collect();
        let rows = match rows_over(params, mobility, residuals, &around) {
            Ok(rows) => rows,
            // Undecidable somewhere in this box: no wider one helps.
            Err(e) => {
                failure.get_or_insert(e);
                break;
            }
        };
        let mut pivots = eliminate(rows.into_iter().map(|r| r.slope).collect(), n).pivots;
        pivots.sort_unstable();
        if tried.contains(&pivots) {
            continue;
        }
        tried.push(pivots.clone());
        match prove(params, mobility, residuals, x.clone(), &pivots) {
            Ok(boxed) => {
                return implied(params, mobility, residuals, &boxed, &pivots, &around)
                    .map(|()| boxed);
            }
            Err(e) => failure = Some(e),
        }
    }
    let failure = failure.expect("the first attempt evaluates the residuals");
    let independent: Vec<usize> = tried.iter().map(Vec::len).collect();
    Err(failure
        .with_context(format!(
            "enclose: no choice of independent residual rows was proven — tried {independent:?} rows, over {n} variables"
        ))
        .into())
}

/// One row of a residual over a box (see [`rows_over`]).
struct Row<S: Scalar> {
    /// The residual it is one of, by index.
    residual: usize,
    value: S,
    /// Its gradient with respect to every variable.
    slope: Vec<S>,
}

/// Why [`enclose`] proves no enclosure of the solution.
#[derive(Debug)]
pub enum EncloseError<S: Scalar> {
    /// Residuals that say something the others do not: over the proven
    /// solution of the independent residuals, each of these rows — by its
    /// residual's index, with its value there — stays away from zero.
    /// `with` are the residuals they depend on, by index: what they
    /// conflict with.
    Conflict {
        conflicting: Vec<(usize, S)>,
        with: Vec<usize>,
    },
    /// No solution was proven (see [`enclose`]).
    Unproven(GeopError),
}

impl<S: Scalar> From<GeopError> for EncloseError<S> {
    fn from(e: GeopError) -> Self {
        EncloseError::Unproven(e)
    }
}

impl<S: Scalar> EncloseError<S> {
    /// As an error, each residual named by `name`.
    pub fn named(self, name: impl Fn(usize) -> String) -> GeopError {
        match self {
            EncloseError::Conflict { conflicting, with } => {
                let mut residuals: Vec<usize> = conflicting.iter().map(|&(r, _)| r).collect();
                residuals.dedup();
                let names =
                    |rs: &[usize]| rs.iter().map(|&r| name(r)).collect::<Vec<_>>().join(", ");
                let others = if with.is_empty() {
                    "the others".to_string()
                } else {
                    names(&with)
                };
                let off: Vec<S> = conflicting.iter().map(|&(_, v)| v).collect();
                GeopError::new(format!(
                    "the constraints conflict: {} cannot hold where {others} do (off by {off:?} there)",
                    names(&residuals),
                ))
            }
            EncloseError::Unproven(e) => e,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::for_all_scalars;

    /// `a x + b y = c` over the parameters `0` and `1`, a length.
    struct Line<S: Scalar>([S; 3]);

    impl<S: Scalar> Residual<S, 2> for Line<S> {
        fn params(&self) -> &[usize] {
            &[0, 1]
        }

        fn eval(&self, values: &[Value<Dual<S, 2>>], out: &mut Vec<Dual<S, 2>>) -> GeopResult<()> {
            let [a, b, c] = self.0.map(Dual::cst);
            let (x, y) = (values[0].scalar()?, values[1].scalar()?);
            out.push(a.mul(x).add(b.mul(y)).sub(c));
            Ok(())
        }
    }

    fn n<S: Scalar>(v: i64) -> S {
        S::from_i64(v)
    }

    /// `x + y = 5` and `x - y = 1`: `(3, 2)`.
    fn crossing<S: Scalar>() -> [Line<S>; 2] {
        [Line([n(1), n(1), n(5)]), Line([n(1), n(-1), n(1)])]
    }

    fn enclose_contains_the_solution<S: Scalar>() {
        let lines = crossing::<S>();
        let residuals: Vec<&dyn Residual<S, 2>> = lines.iter().map(|l| l as _).collect();
        let params = [Param::Scalar(n::<S>(0)), Param::Scalar(n::<S>(0))];
        let mobility = [Mobility::Held; 2];
        let solved = solve(&params, &mobility, &residuals, n(1), &[]).unwrap();
        assert!(solved.report.converged, "{:?}", solved.report);
        let boxed = enclose(&solved.params.clone(), &mobility, &residuals, n(1)).unwrap();
        for (found, exact) in boxed.iter().zip([3, 2]) {
            assert!(
                n::<S>(exact).is_subset_of(*found),
                "{found:?} does not contain {exact}"
            );
        }
    }

    #[test]
    fn enclose_contains_the_solution_in_every_scalar() {
        for_all_scalars!(enclose_contains_the_solution);
    }

    /// Conflicting residuals have no solution to enclose.
    fn enclose_refuses_a_conflict<S: Scalar>() {
        let lines = [
            Line([n::<S>(1), n(1), n(5)]),
            Line([n(1), n(1), n(6)]),
            Line([n(1), n(-1), n(1)]),
        ];
        let residuals: Vec<&dyn Residual<S, 2>> = lines.iter().map(|l| l as _).collect();
        let params = [Param::Scalar(n::<S>(3)), Param::Scalar(n::<S>(2))];
        assert!(enclose(&params, &[Mobility::Held; 2], &residuals, n(1)).is_err());
    }

    #[test]
    fn enclose_refuses_a_conflict_in_every_scalar() {
        for_all_scalars!(enclose_refuses_a_conflict);
    }
}
