//! Extrude, revolve, sweep and loft as operations of a program (see
//! [`geop_ops::operation`]): a sketch's one area swept into a solid — along
//! its plane's normal, around an axis, along another sketch's curves, or
//! through other sketches' profiles — kept as a new body or combined with
//! one the part has; or its curves swept into a sheet, faces standing on
//! their own.
//!
//! Both say how far they go the same way, by an [`Extents`]: one side, both
//! sides alike, or each side its own way — a length (an angle), up to the
//! next face of the solid they are combined with, or through all of it.

mod extrude;
mod loft;
mod revolve;
mod sweep;

pub use extrude::{Extrude, ExtrudeArgs, reach_past, shape_loops};
pub use loft::{Loft, LoftArgs};
pub use revolve::{Revolve, RevolveArgs};
pub use sweep::{Sweep, SweepArgs, path_chain};

use std::collections::HashSet;

use geop_core_math::vector::Vector3;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
};
use geop_core_topology::{Body, SolidId, build::BuiltBody};
use geop_ops::{
    Namer, Part,
    operation::{EntityRef, Role},
    ui::{Choice, Form, Number, Tone},
};
use geop_ops_booleans::{
    Combine, Tool,
    boolean::{BooleanOp, NOTHING_STOPS, boolean},
    remesh::remesh::RemeshParams,
    trim::trim_up_to_next,
};
use serde::{Deserialize, Serialize};

use crate::sweep::SweepLoop;

/// The sketch field `key` of a form, labelled the same: a sketch to pick,
/// `set` given its name (none when the field is cleared) — or, before there
/// is any sketch, a hint to draw one.
fn sketch_field<'a, S: Scalar, A: 'a>(
    form: &mut Form<'a, S, A>,
    before: &Part<S>,
    key: &str,
    sketch: &str,
    set: impl Fn(&mut A, String) + 'a,
) {
    if before.sketches().next().is_none() {
        form.text(key, "No sketch yet — add one first.", Tone::Hint);
        return;
    }
    let value = if sketch.is_empty() {
        Vec::new()
    } else {
        vec![EntityRef::Sketch {
            name: sketch.into(),
        }]
    };
    form.reference(
        key,
        key,
        value,
        &[Role::Sketch],
        None,
        false,
        move |edit, picked| {
            let name = match picked.as_slice() {
                [EntityRef::Sketch { name }] => name.clone(),
                _ => String::new(),
            };
            set(edit.args, name);
        },
    );
}

/// The path field `key` of a form, labelled the same: a sketch or a 3-D
/// sketch whose curves something runs along, `set` given its name (none
/// when the field is cleared) — or, before there is any, a hint to draw
/// one.
fn path_field<'a, S: Scalar, A: 'a>(
    form: &mut Form<'a, S, A>,
    before: &Part<S>,
    key: &str,
    path: &str,
    set: impl Fn(&mut A, String) + 'a,
) {
    if before.sketches().next().is_none() && before.sketches3d().next().is_none() {
        form.text(key, "No sketch yet — add one first.", Tone::Hint);
        return;
    }
    let value = if path.is_empty() {
        Vec::new()
    } else if before.sketch3d_id(path).is_ok() {
        vec![EntityRef::Sketch3d { name: path.into() }]
    } else {
        vec![EntityRef::Sketch { name: path.into() }]
    };
    form.reference(
        key,
        key,
        value,
        &[Role::Path],
        None,
        false,
        move |edit, picked| {
            let name = match picked.as_slice() {
                [EntityRef::Sketch { name } | EntityRef::Sketch3d { name }] => name.clone(),
                _ => String::new(),
            };
            set(edit.args, name);
        },
    );
}

/// How far one side of an extrude or revolve goes: a length — an angle, in
/// degrees, for a revolve — or as far as the solid it is combined with: up
/// to its next face, or through all of it. Serialized as `{"blind": 20.0}`,
/// `"up_to_next"` or `"through_all"`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Extent {
    Blind(f64),
    UpToNext,
    ThroughAll,
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
    /// One side, `amount` far.
    pub fn blind(amount: f64) -> Self {
        Self {
            side1: Extent::Blind(amount),
            symmetric: false,
            side2: None,
            reversed: false,
        }
    }

    /// Whether a side goes as far as the solid it is combined with, which
    /// it then needs.
    pub fn reaches_target(&self) -> bool {
        let far = |e: Extent| matches!(e, Extent::UpToNext | Extent::ThroughAll);
        far(self.side1) || (!self.symmetric && self.side2.is_some_and(far))
    }

    /// What to build for it, see [`Plan`]: `far(sign)` is how far, the way
    /// `sign` says, reaches past the solid it is combined with.
    fn plan(&self, far: impl Fn(f64) -> GeopResult<f64>) -> GeopResult<Plan> {
        use Extent::*;
        let ahead = if self.reversed { -1.0 } else { 1.0 };
        let side = |sign: f64, extent: Extent| -> GeopResult<Side> {
            Ok(match extent {
                Blind(d) => Side {
                    to: sign * d,
                    up_to_next: false,
                },
                ThroughAll => Side {
                    to: sign * far(sign)?,
                    up_to_next: false,
                },
                UpToNext => Side {
                    to: sign * far(sign)?,
                    up_to_next: true,
                },
            })
        };
        let sides = match (self.side1, self.symmetric, self.side2) {
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
    /// far — typed, or dragged on its handle — whether it is turned the
    /// other way, whether symmetric, and unless so, the other side the same
    /// way. `amount(value, second)` makes a side's number field, keyed `key`
    /// for the first side and `key2` for the second; a side turned blind
    /// starts `default` far. `set1` sets the first side's length, which an
    /// operation may want to react to.
    pub(crate) fn show<'a, S: Scalar, A: 'a>(
        &self,
        form: &mut Form<'a, S, A>,
        key: &str,
        amount: impl Fn(f64, bool) -> Number<S>,
        default: f64,
        extents: fn(&mut A) -> &mut Extents,
        set1: impl Fn(&mut A, f64) + 'a,
    ) {
        let choices = |none: bool| {
            let mut choices = Vec::new();
            if none {
                choices.push(Choice::new("none", "None"));
            }
            choices.push(Choice::new("blind", "Blind"));
            choices.push(Choice::new("up_to_next", "Up to next"));
            choices.push(Choice::new("through_all", "Through all"));
            choices
        };
        let mode = |extent: Option<Extent>| match extent {
            None => "none",
            Some(Extent::Blind(_)) => "blind",
            Some(Extent::UpToNext) => "up_to_next",
            Some(Extent::ThroughAll) => "through_all",
        };
        let pick = move |old: Option<Extent>, mode: &str| match mode {
            "blind" => Some(Extent::Blind(match old {
                Some(Extent::Blind(d)) => d,
                _ => default,
            })),
            "up_to_next" => Some(Extent::UpToNext),
            "through_all" => Some(Extent::ThroughAll),
            _ => None,
        };
        form.select(
            "end",
            "end",
            mode(Some(self.side1)),
            choices(false),
            false,
            move |args, value| {
                let this = extents(args);
                if let Some(extent) = pick(Some(this.side1), value) {
                    this.side1 = extent;
                }
            },
        );
        if let Extent::Blind(d) = self.side1 {
            form.number(key, amount(d, false), set1);
        }
        form.checkbox(
            "reversed",
            "reverse direction",
            self.reversed,
            move |args, b| extents(args).reversed = b,
        );
        form.checkbox("symmetric", "symmetric", self.symmetric, move |args, b| {
            extents(args).symmetric = b
        });
        if self.symmetric {
            return;
        }
        form.select(
            "second",
            "second side",
            mode(self.side2),
            choices(true),
            false,
            move |args, value| {
                let this = extents(args);
                this.side2 = pick(this.side2, value);
            },
        );
        if let Some(Extent::Blind(d)) = self.side2 {
            form.number(&format!("{key}2"), amount(d, true), move |args, d| {
                extents(args).side2 = Some(Extent::Blind(d))
            });
        }
    }
}

/// The field of a form making faces that go as far as a solid: which solid
/// — the target of `combine`, which a face is otherwise not combined with;
/// none for every solid of the part.
fn face_target_field<'a, S: Scalar, A: 'a>(
    form: &mut Form<'a, S, A>,
    combine: &Combine,
    combine_of: fn(&mut A) -> &mut Combine,
) {
    let value = combine
        .target()
        .map(|name| vec![EntityRef::Solid { name: name.into() }])
        .unwrap_or_default();
    form.reference(
        "combine_target",
        "up to solid",
        value,
        &[Role::Solid],
        None,
        false,
        move |edit, picked| {
            if let [EntityRef::Solid { name }] = picked.as_slice() {
                let this = combine_of(edit.args);
                *this = this.with_target(name.clone());
            }
        },
    );
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

/// The tool the side `k` builds, as [`geop_ops_booleans::Combine::apply`]
/// takes it: `solid`, built from the sketch's plane by `namer` — so its cap
/// there is named `start`, and its far one `end`.
fn side_tool(namer: &Namer, k: usize, solid: SolidId, side: Side) -> Tool {
    Tool {
        solid,
        up_to_next: side
            .up_to_next
            .then(|| (namer.name(&["start"]), namer.name(&["end"]))),
        scope: (k > 0).then(|| format!("side{}", k + 1)),
    }
}

/// Trims the sheets the side `k` of the step `operation_id` built — by
/// `namer`, from `loops`, from the station `first` on the sketch's plane to
/// `last` — up to the next face of the solids `targets` (see
/// [`trim_up_to_next`]).
fn trim_side<S: Scalar>(
    part: &mut Part<S>,
    operation_id: &str,
    k: usize,
    built: &BuiltBody,
    targets: &[String],
    (namer, loops): (&Namer, &[SweepLoop<S>]),
    (first, last): (&str, &str),
) -> GeopResult<()> {
    let targets = targets
        .iter()
        .map(|t| part.solid_id(t))
        .collect::<GeopResult<Vec<_>>>()?;
    let at = |station: &str| -> HashSet<String> {
        loops
            .iter()
            .flat_map(|l| &l.profile.curve_names)
            .map(|curve| namer.name(&[curve, station]))
            .collect()
    };
    let (start, end) = (at(first), at(last));
    let combine = side_namer(&Namer::new("combine", operation_id)?, k);
    for &sheet in &built.shells {
        trim_up_to_next(
            part,
            &combine,
            sheet,
            &targets,
            (&start, &end),
            RemeshParams::default(),
        )?;
    }
    Ok(())
}

/// Combines the `tools` the sides of a step built, as `combine` says (see
/// [`Combine::apply`]). A side going up to the next face that the solids
/// it stops at, `stops`, do not stop all round — the profile meets them
/// with part of itself, and the rest goes on past — goes instead exactly as
/// far as the profile first meets them: as far as the nearest point where it
/// and they overlap ([`first_contact`]). A cut or an intersection meeting
/// nothing of them up to there — its profile came from outside — goes on
/// from there up to the next face, as a tool starting there would; if that
/// is not stopped all round either, part of the profile, once in, only comes
/// out going all the way, `all[k]` — a full turn, or through all — and so
/// it does. Each is a length, rebuilt by `rebuild(part, k, from, to)` for
/// the side `k` as a tool from `from` to `to`; `far[k]` is how far the
/// side's tool up to the next face reaches.
///
/// `measure(k, p)` is how far along the side `k` a point `p` is — its
/// distance or angle from the sketch's plane, the way the side goes — or
/// none for a point that could lie on that plane itself, where every side
/// starts.
#[allow(clippy::too_many_arguments)]
fn combine_sides<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    operation_id: &str,
    combine: &Combine,
    mut tools: Vec<Tool>,
    stops: &[String],
    far: &[f64],
    all: &[f64],
    rebuild: impl Fn(&mut Part<S>, usize, f64, f64) -> GeopResult<SolidId>,
    measure: impl Fn(usize, &Vector3<S>) -> Option<f64>,
) -> GeopResult<()> {
    let mut trial = part.clone();
    let error = match combine.apply(&mut trial, namer, operation_id, &tools) {
        Ok(()) => {
            *part = trial;
            return Ok(());
        }
        Err(error) if error.root_message().starts_with(NOTHING_STOPS) => error,
        Err(error) => return Err(error),
    };
    // Returned, if the profile meets nothing at all.
    let mut error = Some(error);
    let stops = stops
        .iter()
        .map(|s| part.solid_id(s))
        .collect::<GeopResult<Vec<_>>>()?;
    let inside = matches!(
        combine,
        Combine::Difference { .. } | Combine::Intersection { .. }
    );
    for k in 0..tools.len() {
        let Some(caps) = tools[k].up_to_next.take() else {
            continue;
        };
        let solid = tools[k].solid;
        let Some(mut to) = first_contact(part, operation_id, solid, &stops, |p| measure(k, p))?
        else {
            return Err(error.take().expect("returned once"));
        };
        part.assemble_solid(&[solid.into()], &[], "")?;
        if inside {
            let mut scratch = part.clone();
            let before = rebuild(&mut scratch, k, 0.0, to)?;
            if overlap(&scratch, operation_id, before, &stops)?.is_none() {
                // Coming from outside, it cuts or keeps nothing up to where
                // it first meets them: from there on, it goes up to the
                // next face as a tool starting there would — ending on the
                // faces it meets.
                let from = to;
                let rest = Tool {
                    solid: rebuild(part, k, from, far[k])?,
                    up_to_next: Some(caps),
                    scope: tools[k].scope.clone(),
                };
                let mut trial = part.clone();
                match combine.apply(&mut trial, namer, operation_id, std::slice::from_ref(&rest)) {
                    Ok(()) => {
                        tools[k] = rest;
                        continue;
                    }
                    Err(e) if e.root_message().starts_with(NOTHING_STOPS) => {}
                    Err(e) => return Err(e),
                }
                // Not stopped all round from there either: some of the
                // profile, once in, never comes out short of going all the
                // way — round to where it started, for a turn. With nothing
                // of them before it met them, going all the way takes from
                // them exactly what that leaves.
                part.assemble_solid(&[rest.solid.into()], &[], "")?;
                to = all[k];
            }
        }
        tools[k].solid = rebuild(part, k, 0.0, to)?;
    }
    combine.apply(part, namer, operation_id, &tools)
}

/// Where `tool` and the solid `stop` overlap, worked out on a copy of
/// `part`: the copy and the overlap in it, or none if they do not.
fn overlap_with<S: Scalar>(
    part: &Part<S>,
    operation_id: &str,
    tool: SolidId,
    stop: SolidId,
) -> GeopResult<Option<(Part<S>, SolidId)>> {
    let mut scratch = part.clone();
    let namer = Namer::new("contact", operation_id)?;
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

/// Whether `tool` overlaps any of `stops`: the first overlap found, as
/// [`overlap_with`] gives it.
fn overlap<S: Scalar>(
    part: &Part<S>,
    operation_id: &str,
    tool: SolidId,
    stops: &[SolidId],
) -> GeopResult<Option<(Part<S>, SolidId)>> {
    for &stop in stops {
        if let Some(found) = overlap_with(part, operation_id, tool, stop)? {
            return Ok(Some(found));
        }
    }
    Ok(None)
}

/// How far along a side the profile first meets `stops`: the least of
/// `measure` over the corners of where the side's tool `tool` and each of
/// them overlap, their intersection worked out on a copy of the part. None
/// if it meets none of them.
fn first_contact<S: Scalar>(
    part: &Part<S>,
    operation_id: &str,
    tool: SolidId,
    stops: &[SolidId],
    measure: impl Fn(&Vector3<S>) -> Option<f64>,
) -> GeopResult<Option<f64>> {
    let mut least: Option<f64> = None;
    for &stop in stops {
        let Some((scratch, overlap)) = overlap_with(part, operation_id, tool, stop)? else {
            continue;
        };
        let model = scratch.topology();
        for vertex in model.iter_body_vertices(overlap)? {
            if let Some(at) = measure(&model.get_vertex(vertex)?.point) {
                least = Some(least.map_or(at, |l: f64| l.min(at)));
            }
        }
    }
    Ok(least)
}

/// The solids a step going as far as a solid goes as far as: the one it is
/// combined with — or, for a new body or a face picked none, every solid
/// the part has.
fn stops<S: Scalar>(part: &Part<S>, combine: &Combine) -> Vec<String> {
    match combine.target() {
        Some(target) => vec![target.to_string()],
        None => part.solid_names(),
    }
}

/// Why a step that goes as far as a solid has none to go to.
fn no_target() -> GeopError {
    GeopError::new("up to next and through all go as far as a solid, and the part has none yet")
}

/// Points whose convex hull holds the solids named `targets`: the control
/// points of all their faces' surfaces — a NURBS surface lies within the
/// hull of its control points — as plain numbers, for sizing a tool that has
/// to reach past them. None for no solids.
pub fn hull<S: Scalar>(part: &Part<S>, targets: &[String]) -> GeopResult<Option<Vec<[f64; 3]>>> {
    let model = part.topology();
    let mut points = Vec::new();
    for target in targets {
        for face in model.body_faces(Body::Solid(part.solid_id(target)?))? {
            for cp in &model.get_face(face)?.surface.control_points {
                let w = cp[3].to_f64();
                points.push([0, 1, 2].map(|k| cp[k].to_f64() / w));
            }
        }
    }
    Ok((!points.is_empty()).then_some(points))
}
