//! What the operations of this crate share: the bodies they act on, copies
//! of them moved by a [`Motion`] and named after what they copy, combining
//! the result as an extrude would, and the directions and axes they move
//! along.

use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::Body;
use geop_ops::{
    Namer, Part,
    operation::{Aspects, EntityRef, Role},
    parameters::Formula,
    part::State,
    ui::{Choice, Form, Number, Track, Unit},
};
use geop_ops_booleans::{Combine, Tool};
use serde::{Deserialize, Serialize};

/// How far apart the instances of a pattern are: each `Step` from the
/// last, or spread evenly over the `Extent` from the first to the last —
/// a length, or an angle in degrees, plain or a formula of the part's
/// parameters. Serialized as `{"step": 2.0}`, `{"extent": 10.0}` or
/// `{"step": "pitch"}`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Spacing {
    Step(Formula),
    Extent(Formula),
}

impl Spacing {
    /// Each `step` — a number, or a formula's text — from the last.
    pub fn step(step: impl Into<Formula>) -> Self {
        Spacing::Step(step.into())
    }

    /// Spread evenly over `extent` — a number, or a formula's text.
    pub fn extent(extent: impl Into<Formula>) -> Self {
        Spacing::Extent(extent.into())
    }

    /// The number it holds, whichever it is.
    pub fn value(&self) -> &Formula {
        match self {
            Spacing::Step(v) | Spacing::Extent(v) => v,
        }
    }

    /// The number it holds, to change.
    pub fn value_mut(&mut self) -> &mut Formula {
        match self {
            Spacing::Step(v) | Spacing::Extent(v) => v,
        }
    }

    /// The same kind of spacing, holding `value`.
    fn with_value(&self, value: Formula) -> Self {
        match self {
            Spacing::Step(_) => Spacing::Step(value),
            Spacing::Extent(_) => Spacing::Extent(value),
        }
    }

    /// Its fields in `form`: which kind — keyed `{key}_mode` — and the
    /// number, keyed `key`, made by `number(label, value)` (see
    /// [`Number::formula`]). Switching the kind keeps the number.
    pub(crate) fn show<'a, S: Scalar, A: 'a>(
        &self,
        form: &mut Form<'a, S, A>,
        key: &str,
        labels: [&str; 2],
        number: impl Fn(&str, &Formula) -> Number<S>,
        spacing: impl Fn(&mut A) -> &mut Spacing + Copy + 'a,
    ) {
        let mode = match self {
            Spacing::Step(_) => "step",
            Spacing::Extent(_) => "extent",
        };
        form.select(
            &format!("{key}_mode"),
            "spacing",
            mode,
            vec![
                Choice::new("step", labels[0]),
                Choice::new("extent", labels[1]),
            ],
            false,
            move |args, mode| {
                let this = spacing(args);
                let value = this.value().clone();
                *this = match mode {
                    "extent" => Spacing::Extent(value),
                    _ => Spacing::Step(value),
                };
            },
        );
        let label = match self {
            Spacing::Step(_) => labels[0],
            Spacing::Extent(_) => labels[1],
        };
        form.formula(key, number(label, self.value()), move |args, value| {
            let this = spacing(args);
            *this = this.with_value(value);
        });
    }
}

/// The count field of a pattern, keyed `key`, of `count` — dragged by
/// `handle` unless it follows a formula — `set` given what it is set to:
/// a formula as typed, a number rounded to a whole one of at least one,
/// the seed alone.
pub(crate) fn count_field<'a, S: Scalar, A: 'a>(
    form: &mut Form<'a, S, A>,
    key: &str,
    count: &Formula,
    inputs: &State,
    handle: Option<Track<S>>,
    set: impl Fn(&mut A, Formula) + 'a,
) {
    let mut number = Number::formula("count", count, inputs, Unit::Count)
        .range(1.0, 24.0)
        .handle(handle);
    number.step = 1.0;
    form.formula(key, number, move |args, count| {
        let count = match count {
            Formula::Plain(value) if value.is_finite() && value >= 1.0 => value.round(),
            Formula::Plain(_) => 1.0,
            formula => return set(args, formula),
        };
        set(args, Formula::Plain(count))
    });
}

/// The count `formula` comes to, `value`: a whole number of instances, at
/// least one — the seed alone. Anything else is refused, saying so.
pub(crate) fn whole_count(formula: &Formula, value: f64) -> GeopResult<usize> {
    if value.fract() != 0.0 || value < 1.0 {
        return Err(GeopError::new(format!(
            "the count {formula} comes to {value}: a pattern has a whole number of instances, at least one — round a formula with round(), floor() or ceil()"
        )));
    }
    Ok(value as usize)
}

/// The field picking the bodies a step acts on, keyed `bodies`: solids,
/// and sheets by one of their faces.
pub(crate) fn bodies_field<'a, S: Scalar, A: 'a>(
    form: &mut Form<'a, S, A>,
    bodies: &[EntityRef],
    set: fn(&mut A) -> &mut Vec<EntityRef>,
) {
    form.reference(
        "bodies",
        "bodies",
        bodies.to_vec(),
        &[Role::Solid, Role::Sheet],
        None,
        true,
        move |edit, picked| *set(edit.args) = picked,
    );
}

/// The newest solid of `before`, as a body field starts out holding it:
/// none, if there is none.
pub(crate) fn newest_solid<S: Scalar>(before: &Part<S>) -> Vec<EntityRef> {
    before
        .solid_names()
        .pop()
        .map(|name| vec![EntityRef::Solid { name }])
        .unwrap_or_default()
}

/// The bodies `refs` refer to, each once, in the order picked. None picked
/// is refused.
pub(crate) fn seeds<S: Scalar>(part: &Part<S>, refs: &[EntityRef]) -> GeopResult<Vec<Body>> {
    if refs.is_empty() {
        return Err(GeopError::new("pick the bodies first"));
    }
    let mut bodies = Vec::new();
    for entity in refs {
        let body = entity.resolve_body(part)?;
        if !bodies.contains(&body) {
            bodies.push(body);
        }
    }
    Ok(bodies)
}

/// One body a step leaves behind as one of its instances — a seed, or a
/// copy of one — and the scope that tells its combination's names apart
/// from the other instances' (see [`Tool::scope`]).
pub(crate) struct Instance {
    pub body: Body,
    pub scope: String,
}

/// The scope of the `k`-th of `n` seeds' instance labelled `label`: the
/// label itself, if there is one seed.
fn scope(label: &str, k: usize, n: usize) -> String {
    if n == 1 {
        label.to_string()
    } else {
        format!("{label}.{k}")
    }
}

/// The seeds as instances labelled `label`.
pub(crate) fn seed_instances(seeds: &[Body], label: &str) -> Vec<Instance> {
    seeds
        .iter()
        .enumerate()
        .map(|(k, &body)| Instance {
            body,
            scope: scope(label, k, seeds.len()),
        })
        .collect()
}

/// Copies every seed and moves the copy by `motion`, as the instance
/// labelled `label`: the copy of each entity named `X` — the solid
/// included — named `rename(X)`.
pub(crate) fn copy_seeds<S: Scalar>(
    part: &mut Part<S>,
    seeds: &[Body],
    motion: &Motion<S>,
    label: &str,
    rename: impl Fn(&str) -> String,
) -> GeopResult<Vec<Instance>> {
    let mut copies = Vec::with_capacity(seeds.len());
    for (k, &seed) in seeds.iter().enumerate() {
        let built = part.copy_body(seed, &rename)?;
        let body = match built.solid {
            Some(solid) => Body::Solid(solid),
            None => Body::Sheet(built.shells[0]),
        };
        part.transform_body(body, motion)?;
        copies.push(Instance {
            body,
            scope: scope(label, k, seeds.len()),
        });
    }
    Ok(copies)
}

/// A body as an error names it: a solid by its name, a sheet by one of its
/// faces'.
fn describe<S: Scalar>(part: &Part<S>, body: Body) -> String {
    let name = match body {
        Body::Solid(solid) => part.name_of(solid),
        Body::Sheet(_) => part
            .topology()
            .body_faces(body)
            .ok()
            .and_then(|faces| faces.first().and_then(|&f| part.name_of(f))),
    };
    match (body, name) {
        (Body::Solid(_), Some(name)) => format!("solid {name:?}"),
        (Body::Sheet(_), Some(name)) => format!("the face {name:?} standing on its own"),
        (_, None) => body.to_string(),
    }
}

/// Combines the `instances` a step left as `combine` says: kept as new
/// bodies, each as it is; or each joined to, cut from or intersected with
/// the target, one after the other — the target itself, if it is one of
/// them, excepted. Combined, the result is named `namer`'s root, and what
/// each combination creates `combine(S,scope,...)` for the step `S` (see
/// [`Combine::apply`]).
///
/// Only solids are combined: a sheet among the instances is refused.
pub(crate) fn combine_instances<S: Scalar>(
    part: &mut Part<S>,
    namer: &Namer,
    operation_id: &str,
    combine: &Combine,
    instances: &[Instance],
) -> GeopResult<()> {
    let Some(target) = combine.target() else {
        return Ok(());
    };
    let ctx = with_context!("combining with {target:?}");
    let target = part.solid_id(target).with_context(ctx)?;
    let mut tools = Vec::new();
    for instance in instances {
        match instance.body {
            Body::Solid(solid) if solid == target => {}
            Body::Solid(solid) => tools.push(Tool {
                solid,
                up_to_next: None,
                scope: Some(instance.scope.clone()),
            }),
            Body::Sheet(_) => {
                return Err(GeopError::new(format!(
                    "{} is no solid: only solids are joined, cut or intersected — keep the copies as new bodies",
                    describe(part, instance.body)
                )))
                .with_context(ctx);
            }
        }
    }
    if tools.is_empty() {
        part.rename(target, namer.root()).with_context(ctx)?;
        return Ok(());
    }
    combine
        .apply(part, namer, operation_id, &tools)
        .with_context(ctx)
}

/// The unit direction `entity` gives: a line's — a straight edge, an axis,
/// a sketch line — or a plane's normal.
pub(crate) fn direction<S: Scalar>(part: &Part<S>, entity: &EntityRef) -> GeopResult<Vector3<S>> {
    let aspects = Aspects::of(entity, part)?;
    if let Some(line) = aspects.line {
        return Ok(line.direction);
    }
    if let Some(plane) = aspects.plane {
        return Ok(*plane.w());
    }
    Err(GeopError::new(format!(
        "{entity} gives no direction: pick a straight edge, an axis, or a plane to go along its normal"
    )))
}

/// The axis `entity` gives: a line's — a straight edge, an axis, a sketch
/// line — or what a round entity turns around: a circular edge, a
/// cylindrical or conical face, a sketch circle.
pub(crate) fn axis<S: Scalar>(part: &Part<S>, entity: &EntityRef) -> GeopResult<Axis<S>> {
    let aspects = Aspects::of(entity, part)?;
    aspects.line.or(aspects.round).ok_or_else(|| {
        GeopError::new(format!(
            "{entity} gives no axis: pick a straight edge, an axis, or a circular edge or round face to turn around its axis"
        ))
    })
}

/// Where the handles of a step acting on `refs` sit: the middle of the
/// corners of the bodies they refer to. None, if they refer to none.
pub(crate) fn center<S: Scalar>(part: &Part<S>, refs: &[EntityRef]) -> Option<Vector3<S>> {
    let model = part.topology();
    let mut sum = [0.0; 3];
    let mut n = 0usize;
    for body in seeds(part, refs).ok()? {
        for vertex in model.iter_body_vertices(body).ok()? {
            let p = model.get_vertex(vertex).ok()?.point;
            for (k, s) in sum.iter_mut().enumerate() {
                *s += p[k].to_f64();
            }
            n += 1;
        }
    }
    (n > 0).then(|| Vector3::from_array(sum.map(|s| S::from_f64(s / n as f64))))
}

/// A handle at `at`, dragged along `direction`; none for a direction that
/// could be zero, which nothing could be dragged along.
pub(crate) fn track<S: Scalar>(at: Vector3<S>, direction: Vector3<S>) -> Option<Track<S>> {
    let length = direction.norm_sq();
    length
        .definitely_greater(S::ZERO)
        .then_some(Track { at, direction })
}
