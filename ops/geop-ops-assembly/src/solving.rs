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
//! [`geop_ops_constraints::solve`] solves, encloses and measures, running the
//! program again at every state it tries ([`Rerun`]).

use std::collections::{BTreeMap, BTreeSet};

use geop_core_editor::editing::Pull;
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
    element::{ElementReference, HasElements, Kind, whole},
    program::Program,
    target::TargetReference,
};
use geop_ops_constraints::{
    Bound, Constraint, Constraints,
    solve::{self, Mobility, Preference, Problem, Run},
    unknown::Unknowns,
};
use geop_ops_model::Design;
use geop_ops_parameters::{PartReader, Runner, StateValue, StateValues, pose_state};
use serde::Serialize;

pub use geop_ops_constraints::unknown::{Unknown, declare_unknowns};

use crate::placed::{Instance, Located, locate, placed_parts};

/// A program and what it is given, on the runner that last ran it: what a
/// solve runs again at every state it tries.
pub struct Rerun<'r> {
    pub runner: &'r mut Runner,
    pub program: &'r Program,
    pub inputs: &'r BTreeMap<String, Design>,
}

impl Rerun<'_> {
    /// The part as the program last ran.
    pub fn reader(&self) -> PartReader<'_> {
        self.runner.part.current()
    }

    /// The program, to run again by a solve.
    fn run(&mut self) -> Run<'_, Design, StateValue> {
        Run {
            runner: &mut *self.runner,
            program: self.program,
            inputs: self.inputs,
        }
    }
}

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
    pub state: String,
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
    /// What it joins, as the elements it is between read.
    pub between: Vec<String>,
    pub values: Vec<Setting>,
}

/// What the target of [`Settings`] is named, before what they are of.
const SETTINGS: &str = "settings:";

/// The target declaring the settings of `of`.
pub fn settings_of(of: &str) -> TargetReference<Settings> {
    TargetReference::new(format!("{SETTINGS}{of}"))
}

/// Read whole.
impl Kind for Settings {
    const NAME: &'static str = "settings";
}

impl HasElements<Settings> for Settings {
    type Element<'a> = &'a Settings;

    fn element(&self, name: &str) -> GeopResult<&Settings> {
        whole(self, name)
    }

    fn names_target(name: &str) -> bool {
        name.starts_with(SETTINGS)
    }
}

/// Every [`Settings`] the part `reader` reads declares, in the order the
/// steps made them.
fn all_settings<'a>(reader: &PartReader<'a>) -> Vec<&'a Settings> {
    (reader.targets_of::<Settings>().into_iter())
        .filter_map(|target| reader.retrieve_element(&ElementReference::whole(target)).ok())
        .collect()
}

/// A value a user sets, as an editor shows it: where the state has it.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct SettingState {
    #[serde(rename = "parameter")]
    pub state: String,
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

/// Every value a user sets in the part `reader` reads — and in the parts
/// placed flexibly in it — grouped as the steps declared them, in the order
/// they made them, each where the state has it.
pub fn settings(reader: &PartReader) -> Vec<SettingsState> {
    (all_settings(reader).into_iter())
        .map(|settings| SettingsState {
            name: settings.name.clone(),
            label: settings.label.clone(),
            kind: settings.kind,
            between: settings.between.clone(),
            values: (settings.values.iter())
                .map(|s| SettingState {
                    state: s.state.clone(),
                    measure: s.measure,
                    value: match reader.retrieve_state(&s.state) {
                        StateValue::Number(v) => v.to_f64(),
                        StateValue::Pose(_) => 0.0,
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
    pub enclosed: StateValues,
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

/// The size of the part `reader` reads: of the largest part placed in it,
/// at least one. What a constraint holds to
/// [`geop_core_math::solvers::system::RELATIVE_TOLERANCE`] of.
pub fn scale_of(reader: &PartReader) -> GeopResult<Design> {
    largest(
        placed_parts(reader)?
            .iter()
            .map(|placed| Ok(placed.located.instance)),
    )
}

/// Which constraints hold where the part `reader` reads has its state.
pub fn check(reader: &PartReader) -> GeopResult<Report> {
    let tolerance = tolerance(scale_of(reader)?)?;
    let failed: Vec<String> = (Constraints::<StateValue>::of(reader)?.iter())
        .filter(|(_, constraint)| !constraint.holds(tolerance))
        .map(|(name, _)| name.to_string())
        .collect();
    Ok(Report {
        converged: failed.is_empty(),
        failed,
        at_limit: Vec::new(),
        iterations: 0,
        phases: Vec::new(),
        enclosed: StateValues::new(),
        unproven: Vec::new(),
        evaluations: [0, 0],
    })
}

/// Every value of the state a solve may move in the part `reader` reads,
/// and how: the pose of every placed part — only the one that is the state
/// `only`, if given — held where it is as far as the constraints allow;
/// and every unknown the steps declare, but those `held`. What a step holds
/// fixed is not moved.
fn freed(
    reader: &PartReader,
    only: Option<&str>,
    held: &BTreeSet<String>,
) -> GeopResult<BTreeMap<String, Mobility>> {
    let declared = Unknowns::<StateValue>::of(reader)?;
    let mut free = BTreeMap::new();
    for placed in placed_parts(reader)? {
        if let Some(state) = placed.located.state {
            free.insert(state, Mobility::Held);
        }
    }
    for unknown in declared.iter() {
        free.insert(unknown.state.clone(), unknown.mobility);
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
fn missing(value: Option<&StateValue>, seed: &StateValue) -> bool {
    !matches!(
        (value, seed),
        (Some(StateValue::Number(_)), StateValue::Number(_))
            | (Some(StateValue::Pose(_)), StateValue::Pose(_))
    )
}

/// `state` with the pose of every placed part a solve of the values `free`
/// may move where it has none: where the run put it. What the program
/// builds is the same with it as without.
fn posed(
    reader: &PartReader,
    state: &StateValues,
    free: &BTreeMap<String, Mobility>,
) -> GeopResult<StateValues> {
    let mut seeded = state.clone();
    for placed in placed_parts(reader)? {
        let seed = StateValue::Pose(placed.located.local);
        if let Some(name) = placed.located.state
            && free.contains_key(&name)
            && missing(state.get(&name), &seed)
        {
            seeded.insert(name, seed);
        }
    }
    Ok(seeded)
}

/// `state` with every unknown a solve of the values `free` may move seeded
/// where it has none, or where the constraint its seed is unless does not
/// hold — a free choice, which the solve makes exact.
fn declared_seeds(
    reader: &PartReader,
    state: &StateValues,
    free: &BTreeMap<String, Mobility>,
) -> GeopResult<StateValues> {
    let mut seeded = state.clone();
    let tolerance = tolerance(scale_of(reader)?)?;
    let constraints = Constraints::<StateValue>::of(reader)?;
    for unknown in Unknowns::<StateValue>::of(reader)?.iter() {
        let Some(seed) = &unknown.seed else {
            continue;
        };
        if !free.contains_key(&unknown.state) {
            continue;
        }
        let holds = (unknown.unless.as_deref())
            .is_some_and(|c| constraints.get(c).is_some_and(|c| c.holds(tolerance)));
        if missing(state.get(&unknown.state), seed) || (unknown.unless.is_some() && !holds) {
            seeded.insert(unknown.state.clone(), seed.clone());
        }
    }
    Ok(seeded)
}

/// `state` with what a solve of the values `free` starts from where it has
/// none (see [`posed`] and [`declared_seeds`]).
fn seeded(
    reader: &PartReader,
    state: &StateValues,
    free: &BTreeMap<String, Mobility>,
) -> GeopResult<StateValues> {
    declared_seeds(reader, &posed(reader, state, free)?, free)
}

/// Numbers that carry their derivatives by one pose's step: six, as many
/// as a pose's variables.
pub type D = Dual<Design, 6>;

/// A point of a placed part pulled towards a point in the world: a drag.
/// The part resists turning — a hundredth as strongly — so that a point
/// pulled alone moves it rather than spins it.
struct PointPull {
    path: String,
    state: String,
    local: Vector3<Design>,
    target: Vector3<Design>,
    /// Where the part was, which it resists turning from.
    start: StateValue,
    scale: Design,
}

/// How strongly a part pulled by a point resists turning, against the pull.
const TURN_RESISTANCE: (i64, i64) = (1, 100);

/// Where the placed part at `path` is in the world, as `reader` has the
/// state, moved by a step of its own pose state: its pose is the innermost
/// the placement reads, so its step moves the part in its own frame.
fn stepped(reader: &PartReader, path: &str) -> GeopResult<Pose<D>> {
    let world = locate(reader, path)?.world;
    let step = Pose::exp_with_derivatives(&Vector3::zero(), &Vector3::zero())?;
    Ok(world.map(D::cst).compose(&step))
}

/// The rows `values` as a constraint by `state`'s step.
fn preference(state: &str, rows: Rows<Design>) -> Constraint<StateValue> {
    Constraint {
        residuals: rows.values,
        jacobian: BTreeMap::from([(state.to_string(), rows.jacobian)]),
        bound: Bound::Zero,
    }
}

impl Preference<Design, StateValue> for PointPull {
    fn evaluate(&self, reader: &PartReader<'_>) -> GeopResult<Constraint<StateValue>> {
        let at = stepped(reader, &self.path)?
            .apply(&self.local.map(D::cst))
            .sub(&self.target.map(D::cst));
        let mut rows = Rows::default();
        for r in at.to_array() {
            rows.push(r.v, r.d.to_vec());
        }
        let resist = Design::from_ratio(TURN_RESISTANCE.0, TURN_RESISTANCE.1)?;
        let stay = (reader.retrieve_state(&self.state)).stay(&self.start, self.scale)?;
        // Its turn only: the rows after the position's.
        for (value, row) in stay.values.into_iter().zip(stay.jacobian).skip(3) {
            rows.push(
                value.mul(resist),
                row.into_iter().map(|d| d.mul(resist)).collect(),
            );
        }
        Ok(preference(&self.state, rows))
    }
}

/// A placed part pulled to a pose in the world as a whole.
struct PosePull {
    path: String,
    state: String,
    target: Pose<Design>,
    scale: Design,
}

impl Preference<Design, StateValue> for PosePull {
    /// How far it is from the pose, as [`Variable::stay`] measures it: of
    /// the pose in the world, whose step is that of the part's pose state.
    fn evaluate(&self, reader: &PartReader<'_>) -> GeopResult<Constraint<StateValue>> {
        let world = StateValue::Pose(locate(reader, &self.path)?.world);
        let rows = world.stay(&StateValue::Pose(self.target), self.scale)?;
        Ok(preference(&self.state, rows))
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

/// The state the solve found, and its report: the limits by the values
/// they read.
fn report(
    reader: &PartReader,
    solved: solve::Solved<StateValue>,
) -> GeopResult<(StateValues, Report)> {
    let made = Constraints::<StateValue>::of(reader)?;
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
    Ok((solved.state, report))
}

/// The values of `solved` that `state` does not have, or has otherwise.
fn changes(state: &StateValues, solved: StateValues) -> StateValues {
    (solved.into_iter())
        .filter(|(name, value)| state.get(name) != Some(value))
        .collect()
}

/// Where the state has to be for every constraint of the program `rerun`
/// runs to hold, as little away from where `state` has it as the
/// constraints allow, after `pulls` pulled it — of the placed parts, only
/// the one the step `only` placed moving, if given — every value within its
/// limits and those `pulls` set held there: the values of the state that
/// changed — the poses of the parts it moved, each relative to the part it
/// is placed in, and the unknowns the steps declared — and how the solve
/// went.
///
/// The program was last run with `state`; it is left run at the state
/// found. A part no solve may move — fixed, or not `only` — goes where its
/// pulls take it, alone: to a pose pulled to, or moved as far as a point
/// pulled goes.
pub fn solve(
    rerun: &mut Rerun,
    state: &StateValues,
    pulls: &[Pull<Design>],
    only: Option<&str>,
) -> GeopResult<(StateValues, Report)> {
    let set: BTreeMap<String, Design> = (pulls.iter())
        .filter_map(|pull| match pull {
            Pull::Value { parameter, value } => Some((parameter.clone(), *value)),
            Pull::Point { .. } | Pull::Pose { .. } => None,
        })
        .collect();
    let held: BTreeSet<String> = set.keys().cloned().collect();
    let only = only.map(pose_state);
    let (free, had, mut start, scale, preferences, turns) = {
        let reader = rerun.reader();
        let turns: BTreeSet<String> = (all_settings(&reader).into_iter())
            .flat_map(|s| s.values.iter())
            .filter(|s| s.measure == Measure::Angle)
            .map(|s| s.state.clone())
            .collect();
        let free = freed(&reader, only.as_deref(), &held)?;
        // What the run has: a pose seeded where it put its part is no change.
        let had = posed(&reader, state, &free)?;
        let mut start = declared_seeds(&reader, &had, &free)?;
        let scale = scale_of(&reader)?;
        let placed = placed_parts(&reader)?;
        let mut preferences: Vec<Box<dyn Preference<Design, StateValue>>> = Vec::new();
        for pull in pulls {
            let name = match pull {
                Pull::Point { parameter, .. } | Pull::Pose { parameter, .. } => parameter,
                Pull::Value { .. } => continue,
            };
            let located = (placed.iter())
                .find(|p| p.located.state.as_ref() == Some(name))
                .ok_or_else(|| GeopError::new(format!("no placed part's pose is {name:?}")))?;
            let path = located.name.clone();
            let located = &located.located;
            match (pull, free.contains_key(name)) {
                (Pull::Point { local, target, .. }, true) => {
                    preferences.push(Box::new(PointPull {
                        path,
                        state: name.clone(),
                        local: *local,
                        target: *target,
                        start: StateValue::Pose(located.local),
                        scale,
                    }))
                }
                (Pull::Pose { target, .. }, true) => preferences.push(Box::new(PosePull {
                    path,
                    state: name.clone(),
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
                    start.insert(name.clone(), StateValue::Pose(local_at(located, &world)));
                }
                (Pull::Pose { target, .. }, false) => {
                    start.insert(name.clone(), StateValue::Pose(local_at(located, target)));
                }
                (Pull::Value { .. }, _) => {}
            }
        }
        (free, had, start, scale, preferences, turns)
    };
    fn problem<'p>(
        free: &BTreeMap<String, Mobility>,
        preferences: Vec<&'p dyn Preference<Design, StateValue>>,
        scale: Design,
    ) -> Problem<'p, Design, StateValue> {
        Problem {
            free: free.clone(),
            preferences,
            scale,
        }
    }
    // A held angle driven there in steps, the rest following each; the
    // pulls pull in the last.
    let (driven, ranges): (Vec<String>, Vec<(f64, f64)>) = (set.iter())
        .filter(|(name, _)| turns.contains(*name))
        .filter_map(|(name, to)| match start.get(name) {
            Some(StateValue::Number(from)) => Some((name.clone(), (from.to_f64(), to.to_f64()))),
            _ => None,
        })
        .unzip();
    continuation(&ranges, TURN_STEP, |at| {
        for (name, at) in driven.iter().zip(at) {
            start.insert(name.clone(), StateValue::Number(Design::from_f64(*at)));
        }
        start = solve::solve(rerun.run(), &start, &problem(&free, Vec::new(), scale))?.state;
        Ok::<_, GeopError>(())
    })?;
    for (name, value) in &set {
        start.insert(name.clone(), StateValue::Number(*value));
    }
    let preferences = preferences.iter().map(|p| &**p).collect();
    let solved = solve::solve(rerun.run(), &start, &problem(&free, preferences, scale))?;
    let (solved, report) = report(&rerun.reader(), solved)?;
    Ok((changes(&had, solved), report))
}

/// Solves the state with the values `set` held there — a joint's
/// coordinates typed in: the parts move to them as a mechanism's would —
/// every other value a user sets keeping where it is, if the constraints
/// let it, as along a serial chain; where they do not, round a closed loop,
/// the others give way.
pub fn set(
    rerun: &mut Rerun,
    state: &StateValues,
    set: &BTreeMap<String, Design>,
) -> GeopResult<(StateValues, Report)> {
    let value = |(name, value): (&String, &Design)| Pull::Value {
        parameter: name.clone(),
        value: *value,
    };
    let mut every: BTreeMap<String, Design> = BTreeMap::new();
    {
        let reader = rerun.reader();
        let start = seeded(
            &reader,
            state,
            &freed(&reader, None, &set.keys().cloned().collect())?,
        )?;
        for settings in all_settings(&reader) {
            for s in &settings.values {
                if let Some(StateValue::Number(v)) = start.get(&s.state) {
                    every.insert(s.state.clone(), *v);
                }
            }
        }
    }
    every.extend(set.iter().map(|(p, v)| (p.clone(), *v)));
    let pulls: Vec<Pull<Design>> = every.iter().map(value).collect();
    let (moved, report) = solve(rerun, state, &pulls, None)?;
    if report.converged {
        return Ok((moved, report));
    }
    let pulls: Vec<Pull<Design>> = set.iter().map(value).collect();
    solve(rerun, state, &pulls, None)
}

/// The problem of every value a solve may move in the program `rerun`
/// runs, from `state`, but those `held` — and that state, seeded.
fn every_value(
    rerun: &Rerun,
    state: &StateValues,
    held: &BTreeSet<String>,
) -> GeopResult<(StateValues, Problem<'static, Design, StateValue>)> {
    let reader = rerun.reader();
    let free = freed(&reader, None, held)?;
    let start = seeded(&reader, state, &free)?;
    let problem = Problem {
        free,
        preferences: Vec::new(),
        scale: scale_of(&reader)?,
    };
    Ok((start, problem))
}

/// How free every placed part of the program `rerun` runs is where `state`
/// has it, the values `held` held where they are — a joint's coordinates,
/// to ask what moves without its joints: to first order, as the
/// constraints leave it, decided over the box an enclosure of the solution
/// proves (see [`solve::freedom`]). The program was last run with `state`.
pub fn freedom(
    rerun: &mut Rerun,
    state: &StateValues,
    held: &BTreeSet<String>,
) -> GeopResult<Freedom> {
    let (start, problem) = every_value(rerun, state, held)?;
    let freedom = solve::freedom(rerun.run(), &start, &problem)?;
    let parts = (placed_parts(&rerun.reader())?.into_iter())
        .map(|placed| {
            let dof = (placed.located.state)
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

/// A smallest set of the constraints of the program `rerun` runs, last run
/// with `state`, that cannot all hold together, by name — none where they
/// all hold after a solve (see [`solve::conflicting`]).
pub fn conflicting(rerun: &mut Rerun, state: &StateValues) -> GeopResult<Vec<String>> {
    let (start, problem) = every_value(rerun, state, &BTreeSet::new())?;
    solve::conflicting(rerun.run(), &start, &problem)
}

/// The values of the state a solve sets: the poses of the placed parts —
/// and of those in the parts placed flexibly — and the unknowns the steps
/// declare. What a program's file keeps of its state.
pub fn solved_states(reader: &PartReader) -> GeopResult<Vec<String>> {
    let mut names: Vec<String> = (placed_parts(reader)?.into_iter())
        .filter_map(|b| b.located.state)
        .collect();
    names.extend(Unknowns::<StateValue>::of(reader)?.iter().map(|u| u.state.clone()));
    Ok(names)
}
