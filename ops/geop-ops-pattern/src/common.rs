//! What the operations of this crate share: the bodies they act on, copies
//! of them moved by a [`Motion`] and named after what they copy, combining
//! the result as an extrude would, and the directions they move along.

use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::Body;
use geop_ops::ui::{Choice, Dialog, Form, Number, Track, Unit, Value};
use geop_ops::{
    Design, NamedModel, Namer, ToolReach,
    operation::{
        Build, BuildReader, EntityRef, Lookup, ModelStep, Role, Scope, Seen, build_model,
        resolve_bodies,
    },
    parameters::Formula,
    ui::formula_number,
};
use geop_ops::{Threads, draw_threads, threads};
use geop_ops_booleans::{Combine, Tool, boolean::Reach};
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

    /// Its fields in `dialog`: which kind — keyed `{key}_mode` — and the
    /// number, keyed `key`, made by `number(label, value)` (see
    /// [`formula_number`]).
    pub(crate) fn show<S: Scalar>(
        &self,
        dialog: &mut Dialog<S>,
        key: &str,
        labels: [&str; 2],
        number: impl FnOnce(&str, &Formula) -> Number<S>,
    ) {
        let (mode, label) = match self {
            Spacing::Step(_) => ("step", labels[0]),
            Spacing::Extent(_) => ("extent", labels[1]),
        };
        let modes = vec![
            Choice::new("step", labels[0]),
            Choice::new("extent", labels[1]),
        ];
        (dialog.select(&format!("{key}_mode"), "spacing", mode, modes, false))
            .number(key, number(label, self.value()));
    }

    /// Its field `field` of those [`Spacing::show`] shows, keyed `key`, set
    /// to `value` — switching the kind keeps the number — and whether it was
    /// one.
    pub(crate) fn set(&mut self, key: &str, field: &str, value: &Value) -> bool {
        if field == format!("{key}_mode") {
            if let Value::Choice(mode) = value {
                let number = self.value().clone();
                *self = match mode.as_str() {
                    "extent" => Spacing::Extent(number),
                    _ => Spacing::Step(number),
                };
            }
            return true;
        }
        if field == key {
            if let Some(number) = Formula::from_value(value) {
                *self = self.with_value(number);
            }
            return true;
        }
        false
    }
}

/// The count field of a pattern, keyed `key`, of `count` as `lookup` reads
/// the parameters — dragged by `handle` unless it follows a formula; set by
/// [`count_of`].
pub(crate) fn count_field<S: Scalar>(
    dialog: &mut Dialog<S>,
    key: &str,
    count: &Formula,
    lookup: &mut (impl Lookup<S> + ?Sized),
    handle: Option<Track<S>>,
) {
    let mut number = formula_number("count", count, lookup, Unit::Count)
        .range(1.0, 24.0)
        .handle(handle);
    number.step = 1.0;
    dialog.number(key, number);
}

/// What a count field set to `value` makes the count: a formula as typed,
/// a number rounded to a whole one of at least one, the seed alone.
pub(crate) fn count_of(value: &Value) -> Option<Formula> {
    Some(match Formula::from_value(value)? {
        Formula::Plain(value) if value.is_finite() && value >= 1.0 => Formula::Plain(value.round()),
        Formula::Plain(_) => Formula::Plain(1.0),
        formula => formula,
    })
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
pub(crate) fn bodies_field<S: Scalar>(dialog: &mut Dialog<S>, bodies: &[EntityRef]) {
    let roles = [Role::Solid, Role::Sheet];
    dialog.reference("bodies", "bodies", bodies.to_vec(), &roles, None, true);
}

/// What a pattern repeats: `bodies` — kept as new bodies, or combined as
/// `combine` says — or `features`, each copy combined as its feature was
/// (see [`crate::features`]). Its fields are among those of the step's
/// arguments.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Seeds {
    /// The bodies to copy: solids, or a face of each sheet.
    pub bodies: Vec<EntityRef>,
    /// Or the features to do again (see [`EntityRef::Feature`]).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub features: Vec<EntityRef>,
    /// Keep the copies of bodies as new bodies, or combine them with a
    /// solid.
    #[serde(default)]
    pub combine: Combine,
}

impl Seeds {
    /// The bodies `bodies`, the copies combined as `combine` says.
    pub fn bodies(bodies: Vec<EntityRef>, combine: Combine) -> Self {
        Seeds {
            bodies,
            features: Vec::new(),
            combine,
        }
    }

    /// The features `features`.
    pub fn features(features: Vec<EntityRef>) -> Self {
        Seeds {
            features,
            ..Seeds::default()
        }
    }

    /// Its fields in `form`, around those `own` adds: first the bodies (see
    /// [`bodies_field`]) or the features, keyed `features`, each by a face
    /// it made — a hole by its wall — and last, of bodies, how to combine
    /// them. A step repeats one or the other, so only the one in use waits
    /// for picks (see [`Seeds::set`]).
    pub(crate) fn show<S: Scalar>(&self, form: &mut Form<S>, own: impl FnOnce(&mut Form<S>)) {
        let dialog = &mut form.dialog;
        bodies_field(dialog, &self.bodies);
        let features = self.features.clone();
        dialog.reference(
            "features",
            "features",
            features,
            &[Role::Feature],
            None,
            true,
        );
        // Only the one in use is needed; the other is picked for when asked.
        dialog.optional(if self.features.is_empty() {
            "features"
        } else {
            "bodies"
        });
        own(form);
        if self.features.is_empty() {
            self.combine.show(&mut form.dialog);
        }
    }

    /// Its field `field` of those [`Seeds::show`] shows set to `value`, and
    /// whether it was one: picking bodies empties the features, and the
    /// other way round; a combination defaults to the newest solid of
    /// `before`.
    pub(crate) fn set<S: Scalar>(
        &mut self,
        field: &str,
        value: &Value,
        before: &NamedModel<S>,
    ) -> bool {
        let (this, other, picked) = match (field, value) {
            ("bodies", Value::Entities(p)) => (&mut self.bodies, &mut self.features, p),
            ("features", Value::Entities(p)) => (&mut self.features, &mut self.bodies, p),
            _ => return self.combine.set(field, value, before),
        };
        if !picked.is_empty() {
            other.clear();
        }
        *this = picked.clone();
        true
    }

    /// Copies what it picks to every placement — a label and the motion
    /// from where the seeds are — as the step `operation_id`, whose names
    /// `names` gives (see [`Names::copy`]). Bodies are then kept, or
    /// combined as [`combine_instances`] does, the bodies themselves
    /// labelled `seed_label`; features are done again (see
    /// [`crate::features::repeat_features`]).
    ///
    /// Bodies and features both, or neither, are refused. The features'
    /// cosmetic threads are copied by [`copy_threads`].
    pub(crate) fn repeat<S: Scalar>(
        &self,
        part: &mut NamedModel<S>,
        names: &Names,
        operation_id: &str,
        seed_label: &str,
        placements: &[(String, Motion<S>)],
    ) -> GeopResult<()> {
        match (self.bodies.is_empty(), self.features.is_empty()) {
            (false, false) => Err(GeopError::new(
                "both bodies and features are picked: a step patterns one or the other",
            )),
            (true, true) => Err(GeopError::new("pick the bodies or the features first")),
            (true, false) => crate::features::repeat_features(
                part,
                names,
                operation_id,
                &self.features,
                placements,
            ),
            (false, true) => {
                let seeds = resolve_bodies(&self.bodies, &mut &*part)?;
                let mut instances = seed_instances(&seeds, seed_label);
                for (label, motion) in placements {
                    instances.extend(copy_seeds(part, &seeds, motion, label, |name| {
                        names.copy(label, name)
                    })?);
                }
                combine_instances(part, &names.namer, operation_id, &self.combine, &instances)
            }
        }
    }

    /// Where the handles of a step repeating it sit: the middle of the
    /// corners of the bodies, or of the tools of the features. None, if
    /// it picks none.
    pub(crate) fn center<S: Scalar>(&self, part: &NamedModel<S>) -> Option<Vector3<S>> {
        if self.features.is_empty() {
            center(part, &self.bodies)
        } else {
            crate::features::features_center(part, &self.features)
        }
    }
}

/// How a pattern names what it makes, after the kind of its operation:
/// the copy of each entity named `X` in the instance labelled `i` is named
/// `kind(P,i,X)` for the step `P` — or, not `labelled`, `kind(P,X)`: of a
/// step of one instance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Naming {
    pub kind: &'static str,
    pub labelled: bool,
}

impl Naming {
    /// The names it gives as the step `step`.
    pub(crate) fn of(self, step: &str) -> GeopResult<Names> {
        Ok(Names {
            namer: Namer::new(self.kind, step)?,
            labelled: self.labelled,
        })
    }
}

/// The names a pattern's step gives (see [`Naming`]).
pub(crate) struct Names {
    pub namer: Namer,
    labelled: bool,
}

impl Names {
    /// The name of the copy of the entity named `name` in the instance
    /// labelled `label`.
    pub(crate) fn copy(&self, label: &str, name: &str) -> String {
        match self.labelled {
            true => self.namer.name(&[label, name]),
            false => self.namer.name(&[name]),
        }
    }
}

/// The arguments of a step that repeats bodies or features: a pattern, or
/// a mirror.
pub(crate) trait Pattern: std::fmt::Debug {
    /// How it names what it makes.
    const NAMING: Naming;

    /// What it repeats.
    fn seeds(&self) -> &Seeds;

    /// The label of the seeds themselves among the instances.
    fn seed_label(&self) -> &'static str;

    /// Where its instances go, as `lookup` has the part: each a label and
    /// the motion from the seeds, the seeds themselves left out.
    fn placements<S: Scalar>(
        &self,
        lookup: &mut (impl Lookup<S> + ?Sized),
    ) -> GeopResult<Vec<(String, Motion<S>)>>;
}

/// Builds the pattern `args` as the step `step` into `model`: its
/// [`ModelStep::build`].
pub(crate) fn build_pattern<A: Pattern>(
    model: &mut NamedModel<Design>,
    reader: &mut BuildReader,
    step: &str,
    args: &A,
) -> GeopResult<()> {
    let ctx = with_context!("{}({step}, {args:?})", A::NAMING.kind);
    let names = A::NAMING.of(step).with_context(ctx)?;
    let placements = (args.placements(&mut Scope { model, reader })).with_context(ctx)?;
    (args.seeds())
        .repeat(model, &names, step, args.seed_label(), &placements)
        .with_context(ctx)
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
    part: &mut NamedModel<S>,
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
fn describe<S: Scalar>(part: &NamedModel<S>, body: Body) -> String {
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
    part: &mut NamedModel<S>,
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
                reach: ToolReach {
                    up_to_next: None,
                    scope: Some(instance.scope.clone()),
                },
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
        .and_then(Reach::stopped)
        .with_context(ctx)
}

/// Where the handles of a step acting on `refs` sit: the middle of the
/// corners of the bodies they refer to. None, if they refer to none.
pub(crate) fn center<S: Scalar>(part: &NamedModel<S>, refs: &[EntityRef]) -> Option<Vector3<S>> {
    let model = part.topology();
    let mut corners = Vec::new();
    for body in resolve_bodies(refs, &mut &*part).ok()? {
        for vertex in model.iter_body_vertices(body).ok()? {
            corners.push(model.get_vertex(vertex).ok()?.point);
        }
    }
    middle(corners)
}

/// The middle of `points`, where a handle sits: their average, read off
/// midpoints — a free choice. None, of none.
pub(crate) fn middle<S: Scalar>(
    points: impl IntoIterator<Item = Vector3<S>>,
) -> Option<Vector3<S>> {
    let mut sum = [0.0; 3];
    let mut n = 0usize;
    for p in points {
        for (k, s) in sum.iter_mut().enumerate() {
            *s += p[k].to_f64();
        }
        n += 1;
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

/// What a pattern of features copies of their cosmetic threads: the copies'
/// step and how it names them, the features' steps, and the placements.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ThreadCopies {
    pub step: String,
    pub naming: Naming,
    pub features: Vec<String>,
    pub placements: Vec<(String, Motion<Design>)>,
}

/// The part's cosmetic threads with those each feature of `copies`
/// recorded copied to every placement, named alike.
fn copy_threads(
    mut threads: Threads<Design>,
    _: &mut BuildReader,
    copies: &ThreadCopies,
) -> GeopResult<Threads<Design>> {
    let names = copies.naming.of(&copies.step)?;
    for step in &copies.features {
        let ctx = with_context!("copying the threads of feature {step:?}");
        for name in threads.of_step(step) {
            let thread = threads.get(&name)?.clone();
            for (label, motion) in &copies.placements {
                let mut copy = thread.clone();
                copy.axis = Axis {
                    point: motion.apply(&thread.axis.point),
                    direction: motion.rotate(&thread.axis.direction),
                };
                copy.face = names.copy(label, &thread.face);
                threads
                    .add(names.copy(label, &name), copy)
                    .with_context(ctx)?;
            }
        }
    }
    Ok(threads)
}

/// Runs the pattern `O` of features or bodies as the step `step`: copies
/// of its seeds in the model, at its placements — of the part as it is
/// before the step — and, of features, copies of their cosmetic threads,
/// drawn as the step's annotations.
pub(crate) fn run_pattern<O: ModelStep>(
    part: &mut Build,
    step: &str,
    args: &O::Args,
) -> GeopResult<()>
where
    O::Args: Pattern,
{
    let features = &args.seeds().features;
    let copies = (!features.is_empty()).then(|| {
        let mut seen = Seen::new(part.current());
        Ok::<_, GeopError>(ThreadCopies {
            step: step.to_string(),
            naming: O::Args::NAMING,
            features: crate::features::feature_steps(seen.model(), features)?,
            placements: args.placements(&mut seen)?,
        })
    });
    build_model::<O>(part, step, args)?;
    match copies {
        Some(copies) => {
            part.revise_or_start(&threads(), &copies?, copy_threads)?;
            draw_threads(part, step)
        }
        None => Ok(()),
    }
}
