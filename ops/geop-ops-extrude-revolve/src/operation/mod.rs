//! Extrude, revolve, sweep and loft as operations of a program: a sketch's
//! one area swept into a solid — along
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
pub use revolve::{Revolve, RevolveArgs, RevolveAxis};
pub use sweep::{Sweep, SweepArgs};

use std::collections::HashSet;

use geop_core_math::vector::Vector3;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_sketch::{Shape, Sketch};
use geop_core_topology::{SolidId, build::BuiltBody};
use geop_core_editor::{
    edit_form::{Edit, EditForm},
    editing::Number,
    numeric_input::NumericInput,
};
use geop_core_program::part::{Input, Reader, State};
use geop_ops_booleans::{
    Combine, Extent, SolidRef, Tool,
    boolean::{BooleanOp, Reach, boolean},
    extent::{End, hull},
    remesh::remesh::RemeshParams,
    trim::trim_up_to_next,
};
use serde::{Deserialize, Serialize};

use geop_core_editor::one_of;
use geop_core_geometry::nurb_curve::NurbCurve3D;
use geop_core_program::element::ElementReference;
use geop_ops_model::{Chain, Design, NamedModel, Namer, Path, ToolReach, read_model};
use geop_ops_sketch::{PlacedSketch, SketchRef};
use geop_ops_sketch3d::PlacedSketch3d;
use geop_ops_parameters::PartReader;

use crate::sweep::SweepLoop;

one_of! {
    /// Curves a sweep runs along, or a rail or a guide follows: a sketch's
    /// or a 3-D sketch's one chain of curves, or one loop, or an edge of a
    /// face.
    pub enum PathRef -> Chain<NurbCurve3D<Design>> {
        Sketch(ElementReference<PlacedSketch<Design>, Path>),
        Sketch3d(ElementReference<PlacedSketch3d, Path>),
        Edge(ElementReference<NamedModel<Design>, Path>),
    }
}

impl PathRef {
    /// Whether it is the sketch `sketch`, as a path.
    pub fn is_sketch(&self, sketch: &SketchRef) -> bool {
        matches!(self, PathRef::Sketch(path) if path.target() == sketch.target())
    }
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
        up_to: Option<SolidRef>,
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
            Output::Faces { up_to } => up_to.as_ref().map(|solid| solid.name()),
        };
        match target {
            Some(target) => vec![target.to_string()],
            None => model.solid_names(),
        }
    }

    /// Its fields in `f`: whether faces; for a solid, how to combine it;
    /// for faces going as far as a solid, `reaches`, which solid. `set`
    /// puts the output set where the arguments keep it. Faces turned into a
    /// solid join the solid they went up to — or, if none, are combined as
    /// [`Output::new_for`] the part `reader` reads; a solid turned into
    /// faces goes up to what it was combined with.
    pub(crate) fn fields<'a, S: Scalar, I: Input, St: State, A, T>(
        &self,
        f: &mut EditForm<'a, S, A, T>,
        reader: &Reader<'a, I, St>,
        reaches: bool,
        set: impl Fn(&mut A, Output) + Clone + 'a,
    ) {
        let faces = matches!(self, Output::Faces { .. });
        // What the other way round is: faces of a solid, a solid of faces.
        let turned = match self {
            Output::Solid(Combine::With { target, .. }) => Output::Faces {
                up_to: Some(target.clone()),
            },
            Output::Solid(Combine::NewBody) => Output::Faces { up_to: None },
            Output::Faces {
                up_to: Some(target),
            } => Output::Solid(Combine::join(target.name())),
            Output::Faces { up_to: None } => {
                Output::new_for(&*read_model(reader).unwrap_or_default())
            }
        };
        let set_faces = set.clone();
        f.checkbox("face", "face", faces, move |edit, checked| {
            if checked != faces {
                set_faces(edit.args, turned.clone());
            }
        });
        match self {
            Output::Solid(combine) => {
                combine.fields(f, reader, move |args, combine| {
                    set(args, Output::Solid(combine))
                });
            }
            Output::Faces { up_to } if reaches => {
                Combine::target_field(f, reader, "up to solid", up_to.as_ref(), move |edit, up_to| {
                    set(edit.args, Output::Faces { up_to: Some(up_to) })
                });
            }
            Output::Faces { .. } => {}
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
    pub fn blind(amount: impl Into<NumericInput>) -> Self {
        Self {
            side1: Extent::blind(amount),
            symmetric: false,
            side2: None,
            reversed: false,
        }
    }

    /// Whether a side goes as far as the solid it is combined with, which
    /// it then needs.
    pub fn reaches_target(&self) -> bool {
        let far = |e: &Extent| matches!(e, Extent::UpToNext | Extent::ThroughAll);
        far(&self.side1) || (!self.symmetric && self.side2.as_ref().is_some_and(far))
    }

    /// What to build for it, see [`Plan`], its lengths' formulas reading
    /// the parameters through `reader`: `far(sign)` is how far, the way
    /// `sign` says, reaches past the solid it is combined with.
    fn plan(&self, reader: &PartReader, far: impl Fn(f64) -> GeopResult<f64>) -> GeopResult<Plan> {
        use End::*;
        let ahead = if self.reversed { -1.0 } else { 1.0 };
        let side1 = self.side1.end(reader)?;
        let side2 = match &self.side2 {
            Some(e) if !self.symmetric => Some(e.end(reader)?),
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
    /// symmetric, and unless so, the other side the same way, a side turned
    /// blind starting `default` far. `amount(length, second)` makes a
    /// side's number field (see [`NumericInput::number`]), keyed `key` for
    /// the first side and `key2` for the second; `get` finds the extents in
    /// the arguments. The first side's length is `length`'s to set, which an
    /// operation may want to react to.
    pub(crate) fn fields<'a, S: Scalar, A, T>(
        &self,
        f: &mut EditForm<'a, S, A, T>,
        (key, default): (&str, f64),
        mut amount: impl FnMut(&NumericInput, bool) -> Number<S>,
        get: impl Fn(&mut A) -> &mut Extents + Copy + 'a,
        length: impl Fn(Edit<'_, A, T>, NumericInput) + 'a,
    ) {
        let side1 = Extent::mode(Some(&self.side1));
        let choices = Extent::choices(false);
        f.select("end", "end", side1, choices, false, move |edit, mode| {
            let extents = get(edit.args);
            if let Some(extent) = Extent::pick(Some(&extents.side1), mode, default) {
                extents.side1 = extent;
            }
        });
        if let Extent::Blind(d) = &self.side1 {
            f.numeric(key, amount(d, false), length);
        }
        let reversed = self.reversed;
        f.checkbox("reversed", "reverse direction", reversed, move |edit, b| {
            get(edit.args).reversed = b
        })
        .checkbox("symmetric", "symmetric", self.symmetric, move |edit, b| {
            get(edit.args).symmetric = b
        });
        if self.symmetric {
            return;
        }
        let second = Extent::mode(self.side2.as_ref());
        let choices = Extent::choices(true);
        f.select(
            "second",
            "second side",
            second,
            choices,
            false,
            move |edit, mode| {
                let extents = get(edit.args);
                extents.side2 = Extent::pick(extents.side2.as_ref(), mode, default);
            },
        );
        if let Some(Extent::Blind(d)) = &self.side2 {
            f.numeric(&format!("{key}2"), amount(d, true), move |edit, length| {
                get(edit.args).side2 = Some(Extent::Blind(length))
            });
        }
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
    reader: &PartReader,
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
    let plan = extents.plan(reader, |sign| match &hull {
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
                    reach: ToolReach {
                        up_to_next: side
                            .up_to_next
                            .then(|| (named.name(&["start"]), named.name(&["end"]))),
                        scope: (k > 0).then(|| format!("side{}", k + 1)),
                    },
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
        let Some(caps) = tools[k].reach.up_to_next.take() else {
            continue;
        };
        let solid = tools[k].solid;
        let sign = sides[k].to.signum();
        let measure = |p: &Vector3<Design>| sweeping.measure(sign, p);
        let Some(contact) = first_contact(model, step, solid, &stops, measure)? else {
            return Err(runs_on);
        };
        model.delete_bodies(&[solid.into()])?;
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
                    reach: ToolReach {
                        up_to_next: Some(caps),
                        scope: tools[k].reach.scope.clone(),
                    },
                };
                let mut trial = model.clone();
                let alone = std::slice::from_ref(&rest);
                if let Reach::Stopped(()) = combine.apply(&mut trial, namer, step, alone)? {
                    tools[k] = rest;
                    continue;
                }
                model.delete_bodies(&[rest.solid.into()])?;
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
