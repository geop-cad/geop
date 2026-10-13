//! Solving a part's state as an editor does, the same for every program:
//! settling it so every constraint holds ([`solve()`]), dragging a placed part
//! ([`Pull::Point`]), setting a value a step offers to set ([`set`]), and
//! asking which constraints hold ([`check`]), how free the placed parts are
//! ([`freedom`]) and which constraints conflict ([`conflicting`]).
//!
//! Nothing here knows what the constraints are. What a solve may move and
//! where it starts from, the steps say in the part itself: each placed part
//! moves by its pose state ([`crate::placed`]), held where it is as far as
//! the constraints allow; a step declares any other value a solve moves, or
//! holds one fixed, as an [`Unknown`]; and the values a user sets directly
//! — a joint's coordinates — as [`Settings`], for an editor to list.
//! [`geop_core_program::solve`] solves, encloses and measures.

use std::collections::{BTreeMap, BTreeSet};

use geop_core_math::{
    dual::Dual,
    geop_error::{GeopError, GeopResult},
    primitives::Pose,
    scalars::{Ring, Scalar},
    solvers::{
        continuation::continuation,
        least_squares::{Rows, Stop},
        system::{Phase, Variable, tolerance},
    },
    vector::Vector3,
};
use geop_core_program::{
    constraint::{Bound, Constraint},
    editing::Pull,
    solve::{self, Mobility, Preference, Problem},
    target::TargetReference,
};
use serde::Serialize;

pub use geop_core_program::solve::{Unknown, unknown, unknowns};

use crate::{
    Design, ParamValue,
    model::pose_parameter,
    operation::{Build, BuildView, Input},
    parameters::Values,
    placed::{Instance, Located, locate, placed_parts},
};

/// What a value a user sets measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Measure {
    /// An angle, in degrees.
    #[serde(rename = "angle")]
    Angle,
    /// A distance.
    #[serde(rename = "distance")]
    Distance,
}

/// A value of the state a user sets directly — a joint's coordinate — and
/// the limits it is kept within.
#[derive(Clone, Debug, PartialEq)]
pub struct Setting {
    pub parameter: String,
    pub measure: Measure,
    pub min: Option<Design>,
    pub max: Option<Design>,
}

/// Values of the state a user sets directly, as a step declares them for an
/// editor to list (see [`settings`]): a target of the part, named after
/// what they are of — the coordinates of a joint, by the joint's name.
#[derive(Clone, Debug, PartialEq)]
pub struct Settings {
    /// What they are of, by name: the joint's mate.
    pub name: String,
    /// What an editor calls that: `fore m1`, the mate `m1` of the step
    /// `fore`.
    pub label: String,
    /// What kind of thing that is: `revolute`.
    pub kind: &'static str,
    /// What it joins, as the entities it is between read.
    pub between: Vec<String>,
    pub values: Vec<Setting>,
}

/// The target declaring the settings of `of`.
pub fn settings_of(of: &str) -> TargetReference<Settings> {
    TargetReference::new(format!("settings:{of}"))
}

/// A value a user sets, as an editor shows it: where the state has it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SettingState {
    pub parameter: String,
    #[serde(rename = "motion")]
    pub measure: Measure,
    /// The state's value, or — not given one — zero.
    pub value: f64,
    pub min: Option<f64>,
    pub max: Option<f64>,
}

/// [`Settings`] as an editor lists them.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SettingsState {
    pub name: String,
    pub label: String,
    pub kind: &'static str,
    pub between: Vec<String>,
    pub values: Vec<SettingState>,
}

/// Every value a user sets in the part `view` shows — and in the parts
/// placed flexibly in it — grouped as the steps declared them, in the order
/// they made them, each where the state has it.
pub fn settings(view: BuildView) -> Vec<SettingsState> {
    (view.targets_of::<Settings>().into_iter())
        .map(|(_, settings)| SettingsState {
            name: settings.name.clone(),
            label: settings.label.clone(),
            kind: settings.kind,
            between: settings.between.clone(),
            values: (settings.values.iter())
                .map(|s| SettingState {
                    parameter: s.parameter.clone(),
                    measure: s.measure,
                    value: match view.retrieve_state(&s.parameter) {
                        ParamValue::Number(v) => v.to_f64(),
                        _ => 0.0,
                    },
                    min: s.min.map(|v| v.to_f64()),
                    max: s.max.map(|v| v.to_f64()),
                })
                .collect(),
        })
        .collect()
}

/// How a solve went: whether every constraint holds, and how hard it was to
/// get there.
#[derive(Clone, Debug, PartialEq)]
pub struct Report {
    pub converged: bool,
    /// The constraints that do not hold, by name.
    pub failed: Vec<String>,
    /// The values the solve stopped at a limit of: those the limits held
    /// at their bounds read.
    pub at_limit: Vec<String>,
    /// How many steps the solve took, and each of its minimizations — none
    /// for a check: why it stopped, after how many steps, and the largest
    /// residual it left.
    pub iterations: usize,
    pub phases: Vec<(Stop, usize, f64)>,
    /// Every value the solve moved — and the others solved with them — as
    /// enclosed: the box its true value is in, about where the solve put
    /// it (see [`solve::Enclosure`]).
    pub enclosed: Values,
    /// The values no true solution near where the solve ended is proven
    /// for, and why: the constraints that conflict there, by name, or what
    /// is singular.
    pub unproven: Vec<(Vec<String>, String)>,
    /// How many times the constraints were made: by the solve, and to
    /// enclose its solution.
    pub evaluations: [usize; 2],
}

/// How free the placed parts are (see [`freedom`]).
#[derive(Clone, Debug, Default, PartialEq, Serialize)]
pub struct Freedom {
    /// Per placed part, by name, how many independent ways it can still
    /// move: 0 for one held fast or fixed, 6 for one nothing holds.
    pub parts: BTreeMap<String, usize>,
    /// How many independent ways the part's state can move as a whole.
    pub total: usize,
}

/// The size of a placed part: what turns a turn into a length, and what a
/// solve's size is at least. An empty part has none.
fn size_of(instance: &Instance) -> GeopResult<Option<Design>> {
    let drawn = instance.component.drawn()?;
    if drawn.vertices.is_empty() && drawn.faces.is_empty() {
        return Ok(None);
    }
    Ok(Some(drawn.extent.size))
}

/// The size of the largest of the placed parts `instances`, at least one.
pub fn largest<'i>(
    instances: impl IntoIterator<Item = GeopResult<&'i Instance>>,
) -> GeopResult<Design> {
    let mut scale = Design::ONE;
    for instance in instances {
        if let Some(size) = size_of(instance?)?
            && size.definitely_greater(scale)
        {
            scale = size;
        }
    }
    Ok(scale)
}

/// The size of the part `view` shows: of the largest part placed in it, at
/// least one. What a constraint holds to
/// [`geop_core_math::solvers::system::RELATIVE_TOLERANCE`] of.
pub fn scale_of(view: BuildView) -> GeopResult<Design> {
    largest(
        placed_parts(view)?
            .iter()
            .map(|placed| Ok(placed.located.instance)),
    )
}

/// The constraints of the part `view` shows, by name.
fn constraints_of<'a>(view: BuildView<'a>) -> BTreeMap<&'a str, &'a Constraint<ParamValue>> {
    view.targets_of::<Constraint<ParamValue>>()
        .into_iter()
        .collect()
}

/// Which constraints hold where the part `view` shows has its state.
pub fn check(view: BuildView) -> GeopResult<Report> {
    let tolerance = tolerance(scale_of(view)?)?;
    let failed: Vec<String> = (constraints_of(view).into_iter())
        .filter(|(_, constraint)| !constraint.holds(tolerance))
        .map(|(name, _)| name.to_string())
        .collect();
    Ok(Report {
        converged: failed.is_empty(),
        failed,
        at_limit: Vec::new(),
        iterations: 0,
        phases: Vec::new(),
        enclosed: Values::new(),
        unproven: Vec::new(),
        evaluations: [0, 0],
    })
}

/// Every value of the state a solve may move in the part `view` shows,
/// and how: the pose of every placed part — only the one that is the state
/// `only`, if given — held where it is as far as the constraints allow;
/// and every unknown the steps declare, but those `held`. What a step holds
/// fixed is not moved.
fn freed(
    view: BuildView,
    only: Option<&str>,
    held: &BTreeSet<String>,
) -> GeopResult<BTreeMap<String, Mobility>> {
    let declared = unknowns(view);
    let mut free = BTreeMap::new();
    for placed in placed_parts(view)? {
        if let Some(parameter) = placed.located.parameter {
            free.insert(parameter, Mobility::Held);
        }
    }
    for (state, unknown) in &declared {
        free.insert(state.clone(), unknown.mobility);
    }
    free.retain(|state, mobility| {
        *mobility != Mobility::Fixed
            && !held.contains(state)
            && (*mobility != Mobility::Held || only.is_none_or(|o| o == state))
    });
    Ok(free)
}

/// Whether `value` is no value of the kind `seed` is: a state a solve has
/// to start from somewhere.
fn missing(value: Option<&ParamValue>, seed: &ParamValue) -> bool {
    !matches!(
        (value, seed),
        (Some(ParamValue::Number(_)), ParamValue::Number(_))
            | (Some(ParamValue::Pose(_)), ParamValue::Pose(_))
            | (Some(ParamValue::Text(_)), ParamValue::Text(_))
    )
}

/// `state` with the pose of every placed part a solve of the values `free`
/// may move where it has none: where the run put it. What the program
/// builds is the same with it as without.
fn posed(view: BuildView, state: &Values, free: &BTreeMap<String, Mobility>) -> GeopResult<Values> {
    let mut seeded = state.clone();
    for placed in placed_parts(view)? {
        let seed = ParamValue::Pose(placed.located.local);
        if let Some(parameter) = placed.located.parameter
            && free.contains_key(&parameter)
            && missing(state.get(&parameter), &seed)
        {
            seeded.insert(parameter, seed);
        }
    }
    Ok(seeded)
}

/// `state` with every unknown a solve of the values `free` may move seeded
/// where it has none, or where the constraint its seed is unless does not
/// hold — a free choice, which the solve makes exact.
fn declared_seeds(
    view: BuildView,
    state: &Values,
    free: &BTreeMap<String, Mobility>,
) -> GeopResult<Values> {
    let mut seeded = state.clone();
    let tolerance = tolerance(scale_of(view)?)?;
    let constraints = constraints_of(view);
    for (name, unknown) in unknowns(view) {
        let Some(seed) = unknown.seed else {
            continue;
        };
        if !free.contains_key(&name) {
            continue;
        }
        let holds = (unknown.unless.as_deref())
            .is_some_and(|c| constraints.get(c).is_some_and(|c| c.holds(tolerance)));
        if missing(state.get(&name), &seed) || (unknown.unless.is_some() && !holds) {
            seeded.insert(name, seed);
        }
    }
    Ok(seeded)
}

/// `state` with what a solve of the values `free` starts from where it has
/// none (see [`posed`] and [`declared_seeds`]).
fn seeded(
    view: BuildView,
    state: &Values,
    free: &BTreeMap<String, Mobility>,
) -> GeopResult<Values> {
    declared_seeds(view, &posed(view, state, free)?, free)
}

/// Numbers that carry their derivatives by one pose's step: six, as many
/// as a pose's variables.
pub type D = Dual<Design, 6>;

/// A point of a placed part pulled towards a point in the world: a drag.
/// The part resists turning — a hundredth as strongly — so that a point
/// pulled alone moves it rather than spins it.
struct PointPull {
    path: String,
    parameter: String,
    local: Vector3<Design>,
    target: Vector3<Design>,
    /// Where the part was, which it resists turning from.
    start: ParamValue,
    scale: Design,
}

/// How strongly a part pulled by a point resists turning, against the pull.
const TURN_RESISTANCE: (i64, i64) = (1, 100);

/// Where the placed part at `path` is in the world, as `view` has the state,
/// moved by a step of its own pose state: its pose is the innermost the
/// placement reads, so its step moves the part in its own frame.
fn stepped(view: BuildView, path: &str) -> GeopResult<Pose<D>> {
    let world = locate(&mut view.clone(), path)?.world;
    let step = Pose::exp_with_derivatives(&Vector3::zero(), &Vector3::zero())?;
    Ok(world.map(D::cst).compose(&step))
}

/// The rows `values` as a constraint by `parameter`'s step.
fn preference(parameter: &str, rows: Rows<Design>) -> Constraint<ParamValue> {
    Constraint {
        residuals: rows.values,
        jacobian: BTreeMap::from([(parameter.to_string(), rows.jacobian)]),
        bound: Bound::Zero,
    }
}

impl Preference<Input, ParamValue> for PointPull {
    fn evaluate(&self, view: BuildView<'_>) -> GeopResult<Constraint<ParamValue>> {
        let at = stepped(view, &self.path)?
            .apply(&self.local.map(D::cst))
            .sub(&self.target.map(D::cst));
        let mut rows = Rows::default();
        for r in at.to_array() {
            rows.push(r.v, r.d.to_vec());
        }
        let resist = Design::from_ratio(TURN_RESISTANCE.0, TURN_RESISTANCE.1)?;
        let stay = view
            .retrieve_state(&self.parameter)
            .stay(&self.start, self.scale)?;
        // Its turn only: the rows after the position's.
        for (value, row) in stay.values.into_iter().zip(stay.jacobian).skip(3) {
            rows.push(
                value.mul(resist),
                row.into_iter().map(|d| d.mul(resist)).collect(),
            );
        }
        Ok(preference(&self.parameter, rows))
    }
}

/// A placed part pulled to a pose in the world as a whole.
struct PosePull {
    path: String,
    parameter: String,
    target: Pose<Design>,
    scale: Design,
}

impl Preference<Input, ParamValue> for PosePull {
    /// How far it is from the pose, as [`Variable::stay`] measures it: of
    /// the pose in the world, whose step is that of the part's pose state.
    fn evaluate(&self, view: BuildView<'_>) -> GeopResult<Constraint<ParamValue>> {
        let world = ParamValue::Pose(locate(&mut view.clone(), &self.path)?.world);
        let rows = world.stay(&ParamValue::Pose(self.target), self.scale)?;
        Ok(preference(&self.parameter, rows))
    }
}

/// The largest turn, in degrees, a held angle is driven by in one solve:
/// a value set half a turn away cannot be reached in one, and a linkage
/// jumping there could fold over into another of its configurations. How
/// hard a solve tries, never what its answer means.
const TURN_STEP: f64 = 30.0;

/// The pose state that puts the placed part `located` at `world`: relative
/// to the part it is placed in.
fn local_at(located: &Located, world: &Pose<Design>) -> Pose<Design> {
    let parent = located.world.compose(&located.local.inverse());
    parent.inverse().compose(world).map(Scalar::sharpen)
}

/// The state the core's solve found, and its report: the limits by the
/// values they read.
fn report(part: &Build, solved: solve::Solved<ParamValue>) -> (Values, Report) {
    let made = part.constraints();
    let mut at_limit: Vec<String> = (solved.report.at_limit.iter())
        .filter_map(|name| made.get(name.as_str()))
        .flat_map(|c| c.jacobian.keys().cloned())
        .collect();
    at_limit.sort();
    at_limit.dedup();
    let report = Report {
        converged: solved.report.converged,
        failed: solved.report.failed.clone(),
        at_limit,
        iterations: solved.report.iterations,
        phases: (solved.report.phases.iter())
            .map(|p: &Phase<Design>| (p.stop, p.iterations, p.max_residual.to_f64()))
            .collect(),
        enclosed: solved.enclosure.state,
        unproven: solved.enclosure.unproven,
        evaluations: solved.report.evaluations,
    };
    (solved.state, report)
}

/// The values of `solved` that `state` does not have, or has otherwise.
fn changes(state: &Values, solved: Values) -> Values {
    (solved.into_iter())
        .filter(|(name, value)| state.get(name) != Some(value))
        .collect()
}

/// Where the state of `part` has to be for every constraint to hold, as
/// little away from where `state` has it as the constraints allow, after
/// `pulls` pulled it — of the placed parts, only the one the step `only`
/// placed moving, if given — every value within its limits and those
/// `pulls` set held there: the values of the state that changed — the
/// poses of the parts it moved, each relative to the part it is placed in,
/// and the unknowns the steps declared — and how the solve went.
///
/// `part` is as the program last ran with `state`; it is left with its
/// constraints at the state found, which the caller runs the program with.
/// A part no solve may move — fixed, or not `only` — goes where its pulls
/// take it, alone: to a pose pulled to, or moved as far as a point pulled
/// goes.
pub fn solve(
    part: &mut Build,
    state: &Values,
    pulls: &[Pull<Design>],
    only: Option<&str>,
) -> GeopResult<(Values, Report)> {
    let set: BTreeMap<String, Design> = (pulls.iter())
        .filter_map(|pull| match pull {
            Pull::Value { parameter, value } => Some((parameter.clone(), *value)),
            Pull::Point { .. } | Pull::Pose { .. } => None,
        })
        .collect();
    let held: BTreeSet<String> = set.keys().cloned().collect();
    let view = part.current();
    let turns: BTreeSet<String> = (view.targets_of::<Settings>().into_iter())
        .flat_map(|(_, s)| s.values.iter())
        .filter(|s| s.measure == Measure::Angle)
        .map(|s| s.parameter.clone())
        .collect();
    let only = only.map(pose_parameter);
    let free = freed(view, only.as_deref(), &held)?;
    // What the run has: a pose seeded where it put its part is no change.
    let had = posed(view, state, &free)?;
    let mut start = declared_seeds(view, &had, &free)?;
    let scale = scale_of(view)?;
    let placed = placed_parts(view)?;
    let mut preferences: Vec<Box<dyn Preference<Input, ParamValue>>> = Vec::new();
    for pull in pulls {
        let parameter = match pull {
            Pull::Point { parameter, .. } | Pull::Pose { parameter, .. } => parameter,
            Pull::Value { .. } => continue,
        };
        let located = (placed.iter())
            .find(|p| p.located.parameter.as_ref() == Some(parameter))
            .ok_or_else(|| GeopError::new(format!("no placed part's pose is {parameter:?}")))?;
        let path = located.name.clone();
        let located = &located.located;
        match (pull, free.contains_key(parameter)) {
            (Pull::Point { local, target, .. }, true) => preferences.push(Box::new(PointPull {
                path,
                parameter: parameter.clone(),
                local: *local,
                target: *target,
                start: ParamValue::Pose(located.local),
                scale,
            })),
            (Pull::Pose { target, .. }, true) => preferences.push(Box::new(PosePull {
                path,
                parameter: parameter.clone(),
                target: *target,
                scale,
            })),
            // Alone: a point pulled moves the part along, unturned, as a
            // pull resisting turning moves a part nothing else holds.
            (Pull::Point { local, target, .. }, false) => {
                let off = target.sub(&located.world.apply(local));
                let world = located
                    .world
                    .with_position(located.world.position().add(&off));
                start.insert(
                    parameter.clone(),
                    ParamValue::Pose(local_at(located, &world)),
                );
            }
            (Pull::Pose { target, .. }, false) => {
                start.insert(
                    parameter.clone(),
                    ParamValue::Pose(local_at(located, target)),
                );
            }
            (Pull::Value { .. }, _) => {}
        }
    }
    fn problem<'p>(
        free: &BTreeMap<String, Mobility>,
        preferences: Vec<&'p dyn Preference<Input, ParamValue>>,
        scale: Design,
    ) -> Problem<'p, Input, ParamValue> {
        Problem {
            free: free.clone(),
            preferences,
            scale,
        }
    }
    // A held angle driven there in steps, the rest following each; the
    // pulls pull in the last.
    let (driven, ranges): (Vec<String>, Vec<(f64, f64)>) = (set.iter())
        .filter(|(parameter, _)| turns.contains(*parameter))
        .filter_map(|(parameter, to)| match start.get(parameter) {
            Some(ParamValue::Number(from)) => {
                Some((parameter.clone(), (from.to_f64(), to.to_f64())))
            }
            _ => None,
        })
        .unzip();
    continuation(&ranges, TURN_STEP, |at| {
        for (parameter, at) in driven.iter().zip(at) {
            start.insert(parameter.clone(), ParamValue::Number(Design::from_f64(*at)));
        }
        start = solve::solve(part, &start, &problem(&free, Vec::new(), scale))?.state;
        Ok::<_, GeopError>(())
    })?;
    for (parameter, value) in &set {
        start.insert(parameter.clone(), ParamValue::Number(*value));
    }
    let preferences = preferences.iter().map(|p| &**p).collect();
    let solved = solve::solve(part, &start, &problem(&free, preferences, scale))?;
    let (solved, report) = report(part, solved);
    Ok((changes(&had, solved), report))
}

/// Solves the state with the values `set` held there — a joint's
/// coordinates typed in: the parts move to them as a mechanism's would —
/// every other value a user sets keeping where it is, if the constraints
/// let it, as along a serial chain; where they do not, round a closed loop,
/// the others give way.
pub fn set(
    part: &mut Build,
    state: &Values,
    set: &BTreeMap<String, Design>,
) -> GeopResult<(Values, Report)> {
    let value = |(parameter, value): (&String, &Design)| Pull::Value {
        parameter: parameter.clone(),
        value: *value,
    };
    let view = part.current();
    let start = seeded(
        view,
        state,
        &freed(view, None, &set.keys().cloned().collect())?,
    )?;
    let mut every: BTreeMap<String, Design> = BTreeMap::new();
    for (_, settings) in view.targets_of::<Settings>() {
        for s in &settings.values {
            if let Some(ParamValue::Number(v)) = start.get(&s.parameter) {
                every.insert(s.parameter.clone(), *v);
            }
        }
    }
    every.extend(set.iter().map(|(p, v)| (p.clone(), *v)));
    let pulls: Vec<Pull<Design>> = every.iter().map(value).collect();
    let (moved, report) = solve(part, state, &pulls, None)?;
    if report.converged {
        return Ok((moved, report));
    }
    let pulls: Vec<Pull<Design>> = set.iter().map(value).collect();
    solve(part, state, &pulls, None)
}

/// The problem of every value a solve may move in `part`, from `state`,
/// but those `held` — and that state, seeded.
fn whole(
    part: &Build,
    state: &Values,
    held: &BTreeSet<String>,
) -> GeopResult<(Values, Problem<'static, Input, ParamValue>)> {
    let view = part.current();
    let free = freed(view, None, held)?;
    let start = seeded(view, state, &free)?;
    let problem = Problem {
        free,
        preferences: Vec::new(),
        scale: scale_of(view)?,
    };
    Ok((start, problem))
}

/// How free every placed part of `part` is where `state` has it, the
/// values `held` held where they are — a joint's coordinates, to ask what
/// moves without its joints: to first order, as the constraints leave it,
/// decided over the box an enclosure of the solution proves (see [`solve::freedom`]). `part` is as the program last
/// ran with `state`.
pub fn freedom(part: &mut Build, state: &Values, held: &BTreeSet<String>) -> GeopResult<Freedom> {
    let (start, problem) = whole(part, state, held)?;
    let freedom = solve::freedom(part, &start, &problem)?;
    let parts = (placed_parts(part.current())?.into_iter())
        .map(|placed| {
            let dof = (placed.located.parameter)
                .and_then(|p| freedom.states.get(&p).copied())
                .unwrap_or(0);
            (placed.name, dof)
        })
        .collect();
    Ok(Freedom {
        parts,
        total: freedom.total,
    })
}

/// A smallest set of the constraints of `part`, as the program last ran
/// with `state`, that cannot all hold together, by name — none where they
/// all hold after a solve (see [`solve::conflicting`]).
pub fn conflicting(part: &mut Build, state: &Values) -> GeopResult<Vec<String>> {
    let (start, problem) = whole(part, state, &BTreeSet::new())?;
    solve::conflicting(part, &start, &problem)
}

/// The values of the state a solve sets: the poses of the placed parts —
/// and of those in the parts placed flexibly — and the unknowns the steps
/// declare. What a program's file keeps of its state.
pub fn solved_parameters(view: BuildView) -> GeopResult<Vec<String>> {
    let mut names: Vec<String> = (placed_parts(view)?.into_iter())
        .filter_map(|b| b.located.parameter)
        .collect();
    names.extend(unknowns(view).into_keys());
    Ok(names)
}
