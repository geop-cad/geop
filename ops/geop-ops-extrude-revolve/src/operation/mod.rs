//! Extrude, revolve, sweep and loft as operations of a program (see
//! [`geop_ops::operation`]): a sketch's one area swept into a solid — along
//! its plane's normal, around an axis, along another sketch's curves, or
//! through other sketches' profiles — kept as a new body or combined with
//! one the model has; or its curves swept into a sheet, faces standing on
//! their own. Which of these, each says by an [`Output`].
//!
//! Extrude and revolve say how far they go the same way, by an [`Extents`]:
//! one side, both sides alike, or each side its own way — a length (an
//! angle), up to the next face of the solid they are combined with, or
//! through all of it — and are built the same way by `build_extents`,
//! each sweeping its own way (see `Sweeping`).

mod extrude;
mod loft;
mod revolve;
mod sweep;

pub use extrude::{Extrude, ExtrudeArgs, shape_loops};
pub use loft::{Loft, LoftArgs};
pub use revolve::{Revolve, RevolveArgs};
pub use sweep::{Sweep, SweepArgs};

use std::collections::HashSet;

use geop_core_math::vector::Vector3;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_sketch::{Shape, Sketch};
use geop_core_topology::{SolidId, build::BuiltBody};
use geop_ops::{
    Design, NamedModel, Namer,
    operation::{BuildReader, BuildView, EntityRef, Lookup, Role, Scope, Seen},
    parameters::{Formula, expressions},
    ui::{Dialog, Number, Tone, Value},
};
use geop_ops_booleans::{
    Combine, Extent, Tool,
    boolean::{BooleanOp, Reach, boolean},
    extent::{End, hull},
    remesh::remesh::RemeshParams,
    trim::trim_up_to_next,
};
use serde::{Deserialize, Serialize};

use crate::sweep::SweepLoop;

/// The field `key` of a dialog, labelled the same, picking `sketches` —
/// one, unless `many` — or, before there is any sketch, a hint to draw one.
fn sketch_field<S: Scalar>(
    dialog: &mut Dialog<S>,
    view: &BuildView,
    key: &str,
    sketches: &[String],
    many: bool,
) {
    if Seen::new(*view).sketch_names().is_empty() {
        dialog.text(key, "No sketch yet — add one first.", Tone::Hint);
        return;
    }
    let value = EntityRef::of_names(EntityRef::sketch, sketches);
    dialog.reference(key, key, value, &[Role::Sketch], None, many);
}

/// What a step sweeping a profile makes of it: a solid — kept as a new
/// body, or combined with another solid as [`Combine`] says — or faces
/// standing on their own, swept from its curves: the area's outline, or a
/// chain of curves enclosing nothing. Faces going as far as a solid (see
/// [`Extent`]) go as far as the solid named `up_to` — or, if none, every
/// solid of the model — and leave it as it is.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Output {
    Solid(Combine),
    Faces {
        #[serde(default, skip_serializing_if = "Option::is_none")]
        up_to: Option<String>,
    },
}

impl Default for Output {
    fn default() -> Self {
        Output::Solid(Combine::NewBody)
    }
}

impl From<Combine> for Output {
    fn from(combine: Combine) -> Self {
        Output::Solid(combine)
    }
}

impl Output {
    /// The default for a step after `before`: a solid, joined to the newest
    /// solid if there is one (see [`Combine::new_for`]).
    pub fn new_for<S: Scalar>(before: &NamedModel<S>) -> Self {
        Output::Solid(Combine::new_for(before))
    }

    /// What of `sketch` it sweeps: as faces, what the sketch draws — its
    /// one area, or the one chain of curves enclosing nothing (see
    /// [`Sketch::shape`]); as a solid, its one area.
    pub fn shape<S: Scalar>(&self, sketch: &Sketch<S>) -> GeopResult<Shape> {
        match self {
            Output::Faces { .. } => sketch.shape(),
            Output::Solid(_) => sketch.region().map(Shape::Region),
        }
    }

    /// The solids a step going as far as a solid goes as far as: the one
    /// it is combined with, or the faces' `up_to` — or, for a new body or
    /// faces up to none, every solid `model` has.
    fn stops<S: Scalar>(&self, model: &NamedModel<S>) -> Vec<String> {
        let target = match self {
            Output::Solid(combine) => combine.target(),
            Output::Faces { up_to } => up_to.as_deref(),
        };
        match target {
            Some(target) => vec![target.to_string()],
            None => model.solid_names(),
        }
    }

    /// Its fields in `dialog`: whether faces; for a solid, how to combine
    /// it; for faces going as far as a solid, `reaches`, which solid.
    pub(crate) fn show<S: Scalar>(&self, dialog: &mut Dialog<S>, reaches: bool) {
        let faces = matches!(self, Output::Faces { .. });
        dialog.checkbox("face", "face", faces);
        match self {
            Output::Solid(combine) => combine.show(dialog),
            Output::Faces { up_to } if reaches => {
                Combine::target_field(dialog, "up to solid", up_to.as_slice());
            }
            Output::Faces { .. } => {}
        }
    }

    /// Its field `key` set to `value`, if `key` is one of
    /// [`Output::show`]'s, and whether it was. Faces turned into a solid
    /// join the solid they went up to — or, if none, are combined as
    /// [`Output::new_for`] `before`; a solid turned into faces goes up to
    /// what it was combined with.
    pub(crate) fn set<S: Scalar>(
        &mut self,
        key: &str,
        value: &Value,
        before: &NamedModel<S>,
    ) -> bool {
        match (key, value, &mut *self) {
            ("face", Value::Bool(true), Output::Solid(combine)) => {
                let up_to = combine.target().map(str::to_string);
                *self = Output::Faces { up_to };
                true
            }
            ("face", Value::Bool(false), Output::Faces { up_to }) => {
                *self = match up_to.take() {
                    Some(target) => Output::Solid(Combine::With {
                        op: BooleanOp::Union,
                        target,
                    }),
                    None => Output::new_for(before),
                };
                true
            }
            ("face", Value::Bool(_), _) => true,
            (key, value, Output::Solid(combine)) => combine.set(key, value, before),
            ("combine_target", Value::Entities(picked), Output::Faces { up_to }) => {
                *up_to = EntityRef::names_of(EntityRef::solid, picked).pop();
                true
            }
            _ => false,
        }
    }

    /// Builds what a step sweeps whole, named by `namer`, as the step `step`:
    /// `sweep(model, name)` sweeps a solid named `name`, or faces if none —
    /// the solid then combined as its [`Combine`] says.
    fn build_whole(
        &self,
        model: &mut NamedModel<Design>,
        namer: &Namer,
        step: &str,
        sweep: impl FnOnce(&mut NamedModel<Design>, Option<&str>) -> GeopResult<BuiltBody>,
    ) -> GeopResult<()> {
        match self {
            Output::Faces { .. } => sweep(model, None).map(drop),
            Output::Solid(combine) => {
                let built = sweep(model, Some(&combine.built_name(namer)))?;
                let tool = Tool::whole(built.solid.expect("swept as a solid"));
                combine.apply(model, namer, step, &[tool])?.stopped()
            }
        }
    }
}

/// How far an extrude or revolve goes, from its sketch's plane: `side1`
/// along its direction — the plane's normal, the turn's, or against it if
/// `reversed` — and, if `symmetric`, as far again the other way, half of a
/// length each way; or else `side2`, if any, the other way on its own
/// terms.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Extents {
    pub side1: Extent,
    #[serde(default)]
    pub symmetric: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub side2: Option<Extent>,
    #[serde(default)]
    pub reversed: bool,
}

impl Extents {
    /// One side, `amount` far: a number, or a formula's text.
    pub fn blind(amount: impl Into<Formula>) -> Self {
        Self {
            side1: Extent::blind(amount),
            symmetric: false,
            side2: None,
            reversed: false,
        }
    }

    /// The formulas of its sides' lengths (see
    /// [`geop_core_program::operation::Operation::formulas`]).
    pub fn formulas(&mut self) -> Vec<&mut String> {
        let sides = [Some(&mut self.side1), self.side2.as_mut()];
        expressions(sides.into_iter().flatten().filter_map(Extent::length_mut))
    }

    /// Whether a side goes as far as the solid it is combined with, which
    /// it then needs.
    pub fn reaches_target(&self) -> bool {
        let far = |e: &Extent| matches!(e, Extent::UpToNext | Extent::ThroughAll);
        far(&self.side1) || (!self.symmetric && self.side2.as_ref().is_some_and(far))
    }

    /// What to build for it, see [`Plan`], its lengths' formulas reading
    /// the parameters through `lookup`: `far(sign)` is how far, the way
    /// `sign` says, reaches past the solid it is combined with.
    fn plan<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
        far: impl Fn(f64) -> GeopResult<f64>,
    ) -> GeopResult<Plan> {
        use End::*;
        let ahead = if self.reversed { -1.0 } else { 1.0 };
        let side1 = self.side1.end(lookup)?;
        let side2 = match &self.side2 {
            Some(e) if !self.symmetric => Some(e.end(lookup)?),
            _ => None,
        };
        let side = |sign: f64, end: End| -> GeopResult<Side> {
            Ok(match end {
                Blind(d) => Side {
                    to: sign * d,
                    up_to_next: false,
                },
                ThroughAll | UpToNext => Side {
                    to: sign * far(sign)?,
                    up_to_next: end == UpToNext,
                },
            })
        };
        let sides = match (side1, self.symmetric, side2) {
            (Blind(d), true, _) => return Ok(Plan::Whole(-d / 2.0, d / 2.0)),
            (e, true, _) => vec![side(ahead, e)?, side(-ahead, e)?],
            (e, false, None) => vec![side(ahead, e)?],
            (e, false, Some(e2)) => vec![side(ahead, e)?, side(-ahead, e2)?],
        };
        Ok(if sides.iter().any(|s| s.up_to_next) {
            Plan::Sides(sides)
        } else {
            Plan::Whole(sides.get(1).map_or(0.0, |s| s.to), sides[0].to)
        })
    }

    /// The fields to say how far: how the first side ends and, blind, how
    /// far — typed, as a number or a formula of the part's parameters, or
    /// dragged on its handle — whether it is turned the other way, whether
    /// symmetric, and unless so, the other side the same way.
    /// `amount(length, second)` makes a side's number field (see
    /// [`geop_ops::ui::formula_number`]), keyed `key` for the first side and
    /// `key2` for the second.
    pub(crate) fn show<S: Scalar>(
        &self,
        dialog: &mut Dialog<S>,
        key: &str,
        mut amount: impl FnMut(&Formula, bool) -> Number<S>,
    ) {
        let side1 = Extent::mode(Some(&self.side1));
        dialog.select("end", "end", side1, Extent::choices(false), false);
        if let Extent::Blind(d) = &self.side1 {
            dialog.number(key, amount(d, false));
        }
        (dialog.checkbox("reversed", "reverse direction", self.reversed)).checkbox(
            "symmetric",
            "symmetric",
            self.symmetric,
        );
        if self.symmetric {
            return;
        }
        let second = Extent::mode(self.side2.as_ref());
        dialog.select(
            "second",
            "second side",
            second,
            Extent::choices(true),
            false,
        );
        if let Some(Extent::Blind(d)) = &self.side2 {
            dialog.number(&format!("{key}2"), amount(d, true));
        }
    }

    /// Its field `field` of those [`Extents::show`] shows, keyed `key`, set
    /// to `value` — a side turned blind starts `default` far — and whether
    /// it was one. The first side's length is the caller's to set (see
    /// [`Extents::set_length`]), which an operation may want to react to.
    pub(crate) fn set(&mut self, key: &str, field: &str, value: &Value, default: f64) -> bool {
        match (field, value) {
            ("end", Value::Choice(mode)) => {
                if let Some(extent) = Extent::pick(Some(&self.side1), mode, default) {
                    self.side1 = extent;
                }
            }
            ("reversed", Value::Bool(b)) => self.reversed = *b,
            ("symmetric", Value::Bool(b)) => self.symmetric = *b,
            ("second", Value::Choice(mode)) => {
                self.side2 = Extent::pick(self.side2.as_ref(), mode, default);
            }
            (field, value) if field == format!("{key}2") => {
                if let Some(length) = Formula::from_value(value) {
                    self.side2 = Some(Extent::Blind(length));
                }
            }
            _ => return false,
        }
        true
    }

    /// The first side's length, if `value` sets one: what it was set from,
    /// for an operation that reacts to the change.
    pub(crate) fn set_length(&mut self, value: &Value) -> Option<Extent> {
        let length = Formula::from_value(value)?;
        Some(std::mem::replace(&mut self.side1, Extent::Blind(length)))
    }
}

/// What an [`Extents`] builds: one sweep between two ends, or — when a side
/// goes up to the next face — each side on its own, from the sketch's plane.
#[derive(Clone, Debug, PartialEq)]
enum Plan {
    /// From the first to the second, in the operation's measure: its start
    /// on the sketch's plane, unless it goes both ways.
    Whole(f64, f64),
    /// Each side, the first one first.
    Sides(Vec<Side>),
}

/// A side built on its own: from the sketch's plane to `to`, in the
/// operation's measure, and only up to the next face it meets if
/// `up_to_next` — then `to` is past everything it could meet.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Side {
    to: f64,
    up_to_next: bool,
}

/// The names of the side `k` of a step built side by side, by `namer`: the
/// first side is named like a step built whole, the second within the scope
/// `side2`.
fn side_namer(namer: &Namer, k: usize) -> Namer {
    match k {
        0 => namer.clone(),
        k => namer.scoped(&format!("side{}", k + 1)),
    }
}

/// What an extrude and a revolve each do their own way, for
/// [`build_extents`]: sweep their loops between two stations of their
/// measure — a distance along the sketch plane's normal, an angle around
/// the axis — and say how far along it things lie.
trait Sweeping {
    /// The loops it sweeps.
    fn loops(&self) -> &[SweepLoop<Design>];

    /// Sweeps its loops from `from` to `to`, in its measure, named by
    /// `namer`: into a solid named `name`, or faces standing on their own if
    /// none.
    fn sweep(
        &self,
        model: &mut NamedModel<Design>,
        namer: &Namer,
        name: Option<&str>,
        from: f64,
        to: f64,
    ) -> GeopResult<BuiltBody>;

    /// How far — forwards for `sign = 1`, backwards for `-1` — reaches past
    /// everything of the solids whose convex hull `hull` spans (see
    /// [`hull`]).
    fn past(&self, hull: &[[f64; 3]], sign: f64) -> GeopResult<f64>;

    /// The names of the stations a sweep from the sketch's plane to `to`
    /// starts and ends at.
    fn ends(&self, to: f64) -> GeopResult<(String, String)>;

    /// How far a side reaching `far` past the solids it stops at goes all
    /// the way: through all, or a full turn.
    fn all_the_way(&self, far: f64) -> f64;

    /// How far along a side `p` is — forwards for `sign = 1`, backwards for
    /// `-1` — or none for a point that could lie on the sketch's plane
    /// itself, where every side starts.
    fn measure(&self, sign: f64, p: &Vector3<Design>) -> Option<f64>;
}

/// Builds an extrude or a revolve, the step `step` named by `namer`: what
/// `sweeping` sweeps, as far as `extents` say — its formulas read through
/// `reader` — made into what `output` says. A side going as far as a solid
/// goes as far as the solids [`Output::stops`] names, and one going up to
/// the next face is built on its own (see [`Plan`]) and then trimmed — faces
/// — or combined (see [`combine_sides`]).
fn build_extents(
    model: &mut NamedModel<Design>,
    reader: &mut BuildReader,
    namer: &Namer,
    step: &str,
    (extents, output): (&Extents, &Output),
    sweeping: &impl Sweeping,
) -> GeopResult<()> {
    let stops = output.stops(model);
    let hull = if extents.reaches_target() {
        hull(model, &stops)?
    } else {
        None
    };
    let plan = extents.plan(&mut Scope { model, reader }, |sign| match &hull {
        Some(hull) => sweeping.past(hull, sign),
        None => Err(GeopError::new(
            "up to next and through all go as far as a solid, and the model has none yet",
        )),
    })?;
    match (output, plan) {
        (_, Plan::Whole(from, to)) => output.build_whole(model, namer, step, |model, name| {
            sweeping.sweep(model, namer, name, from, to)
        }),
        (Output::Faces { .. }, Plan::Sides(sides)) => {
            for (k, side) in sides.into_iter().enumerate() {
                let named = side_namer(namer, k);
                let built = sweeping.sweep(model, &named, None, 0.0, side.to)?;
                if side.up_to_next {
                    let (first, last) = sweeping.ends(side.to)?;
                    trim_side(
                        model,
                        step,
                        k,
                        &built,
                        &stops,
                        (&named, sweeping.loops()),
                        (&first, &last),
                    )?;
                }
            }
            Ok(())
        }
        (Output::Solid(combine), Plan::Sides(sides)) => {
            let mut tools = Vec::new();
            for (k, &side) in sides.iter().enumerate() {
                let named = side_namer(namer, k);
                let name = combine.built_name(&named);
                let built = sweeping.sweep(model, &named, Some(&name), 0.0, side.to)?;
                // Built from the sketch's plane, so its cap there is named
                // `start`, and its far one `end`.
                tools.push(Tool {
                    solid: built.solid.expect("swept as a solid"),
                    up_to_next: side
                        .up_to_next
                        .then(|| (named.name(&["start"]), named.name(&["end"]))),
                    scope: (k > 0).then(|| format!("side{}", k + 1)),
                });
            }
            combine_sides(model, namer, step, combine, tools, &sides, &stops, sweeping)
        }
    }
}

/// Trims the sheets the side `k` of the step `step` built — by `namer`,
/// from `loops`, from the station `first` on the sketch's plane to `last`
/// — up to the next face of the solids `targets` (see [`trim_up_to_next`]).
fn trim_side<S: Scalar>(
    model: &mut NamedModel<S>,
    step: &str,
    k: usize,
    built: &BuiltBody,
    targets: &[String],
    (namer, loops): (&Namer, &[SweepLoop<S>]),
    (first, last): (&str, &str),
) -> GeopResult<()> {
    let targets = targets
        .iter()
        .map(|t| model.solid_id(t))
        .collect::<GeopResult<Vec<_>>>()?;
    let at = |station: &str| -> HashSet<String> {
        loops
            .iter()
            .flat_map(|l| &l.profile.curve_names)
            .map(|curve| namer.name(&[curve, station]))
            .collect()
    };
    let (start, end) = (at(first), at(last));
    let combine = side_namer(&Namer::new("combine", step)?, k);
    for &sheet in &built.shells {
        trim_up_to_next(
            model,
            &combine,
            sheet,
            &targets,
            (&start, &end),
            RemeshParams::default(),
        )?;
    }
    Ok(())
}

/// Combines the `tools` the `sides` of a step built, as `combine` says (see
/// [`Combine::apply`]). A side going up to the next face that the solids
/// it stops at, `stops`, do not stop all round — the profile meets them
/// with part of itself, and the rest goes on past — goes another way, but
/// never flat up to where it first meets them: that is a corner or an edge
/// they already have, and a tool ending there touches them along it alone
/// — a join left two solids sharing an edge.
///
/// A cut or an intersection meeting nothing of them before it first meets
/// them ([`first_contact`]) — its profile came from outside — goes on from
/// there up to the next face, as a tool starting there would. Otherwise —
/// a join, which keeps what lies before, a cut from inside, or one not
/// stopped all round from there either — part of the profile, once in,
/// only comes out going all the way, a full turn or through all (see
/// [`Sweeping::all_the_way`]), and so it does: with nothing of them before
/// it met them, going all the way cuts from them exactly what that leaves.
/// A new body has no way to go all the way — it would overlap the solids it
/// is not combined with — and is refused, as is a profile meeting nothing
/// of them at all.
#[allow(clippy::too_many_arguments)]
fn combine_sides(
    model: &mut NamedModel<Design>,
    namer: &Namer,
    step: &str,
    combine: &Combine,
    mut tools: Vec<Tool>,
    sides: &[Side],
    stops: &[String],
    sweeping: &impl Sweeping,
) -> GeopResult<()> {
    let mut trial = model.clone();
    let runs_on = match combine.apply(&mut trial, namer, step, &tools)? {
        Reach::Stopped(()) => {
            *model = trial;
            return Ok(());
        }
        Reach::RunsOn(error) => error,
    };
    let Combine::With { op, .. } = combine else {
        return Err(runs_on);
    };
    let stops = stops
        .iter()
        .map(|s| model.solid_id(s))
        .collect::<GeopResult<Vec<_>>>()?;
    // The side `k`, swept from `from` to `to` along it.
    let rebuild = |model: &mut NamedModel<Design>, k: usize, from: f64, to: f64| {
        let named = side_namer(namer, k);
        let name = combine.built_name(&named);
        let sign = sides[k].to.signum();
        let built = sweeping.sweep(model, &named, Some(&name), from * sign, to * sign)?;
        Ok::<_, GeopError>(built.solid.expect("swept as a solid"))
    };
    for k in 0..tools.len() {
        let Some(caps) = tools[k].up_to_next.take() else {
            continue;
        };
        let solid = tools[k].solid;
        let sign = sides[k].to.signum();
        let measure = |p: &Vector3<Design>| sweeping.measure(sign, p);
        let Some(contact) = first_contact(model, step, solid, &stops, measure)? else {
            return Err(runs_on);
        };
        model.assemble_solid(&[solid.into()], &[], "")?;
        let far = sides[k].to.abs();
        if *op != BooleanOp::Union {
            let mut scratch = model.clone();
            let before = rebuild(&mut scratch, k, 0.0, contact)?;
            let mut from_outside = true;
            for &stop in &stops {
                from_outside &= overlap_with(&scratch, step, before, stop)?.is_none();
            }
            if from_outside {
                // From where it first meets them, up to the next face, as a
                // tool starting there would: ending on the faces it meets.
                let rest = Tool {
                    solid: rebuild(model, k, contact, far)?,
                    up_to_next: Some(caps),
                    scope: tools[k].scope.clone(),
                };
                let mut trial = model.clone();
                let alone = std::slice::from_ref(&rest);
                if let Reach::Stopped(()) = combine.apply(&mut trial, namer, step, alone)? {
                    tools[k] = rest;
                    continue;
                }
                model.assemble_solid(&[rest.solid.into()], &[], "")?;
            }
        }
        tools[k].solid = rebuild(model, k, 0.0, sweeping.all_the_way(far))?;
    }
    combine.apply(model, namer, step, &tools)?.stopped()
}

/// Where `tool` and the solid `stop` overlap, worked out on a copy of
/// `model`: the copy and the overlap in it, or none if they do not.
fn overlap_with<S: Scalar>(
    model: &NamedModel<S>,
    step: &str,
    tool: SolidId,
    stop: SolidId,
) -> GeopResult<Option<(NamedModel<S>, SolidId)>> {
    let mut scratch = model.clone();
    let namer = Namer::new("contact", step)?;
    let overlap = boolean(
        &mut scratch,
        &namer,
        stop,
        tool,
        BooleanOp::Intersection,
        RemeshParams::default(),
    )?;
    Ok(overlap.map(|overlap| (scratch, overlap)))
}

/// How far along a side the profile first meets `stops`: the least of
/// `measure` over the corners of where the side's tool `tool` and each of
/// them overlap, their intersection worked out on a copy of the model. None
/// if it meets none of them.
fn first_contact<S: Scalar>(
    model: &NamedModel<S>,
    step: &str,
    tool: SolidId,
    stops: &[SolidId],
    measure: impl Fn(&Vector3<S>) -> Option<f64>,
) -> GeopResult<Option<f64>> {
    let mut least: Option<f64> = None;
    for &stop in stops {
        let Some((scratch, overlap)) = overlap_with(model, step, tool, stop)? else {
            continue;
        };
        let topology = scratch.topology();
        for vertex in topology.iter_body_vertices(overlap)? {
            if let Some(at) = measure(&topology.get_vertex(vertex)?.point) {
                least = Some(least.map_or(at, |l: f64| l.min(at)));
            }
        }
    }
    Ok(least)
}
