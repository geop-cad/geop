//! What the operations of this crate share: the bodies they act on, copies
//! of them moved by a [`Motion`] and named after what they copy, combining
//! the result as an extrude would, and the directions they move along.

use geop_core_geometry::shape::Axis;
use geop_core_math::{
    geop_error::{GeopError, GeopResult, WithContext},
    plain,
    primitives::Motion,
    scalars::Scalar,
    vector::Vector3,
    with_context,
};
use geop_core_topology::Body;
use geop_ops::ui::{Edit, EditForm, Track};
use geop_ops::{
    Design, NamedModel, Namer, ToolReach,
    operation::{
        Build, BuildReader, EntityRef, Lookup, PartOperation, Role, Scope, Seen, build_model,
        resolve_bodies,
    },
};
use geop_ops::{Threads, draw_threads, threads};
use geop_ops_booleans::{Combine, Tool, boolean::Reach};
use serde::{Deserialize, Serialize};

/// The field picking the bodies a step acts on, keyed `bodies`: solids,
/// and sheets by one of their faces. `set` gets the bodies picked.
pub(crate) fn bodies_field<'a, A>(
    f: &mut EditForm<'a, Design, A>,
    bodies: &[EntityRef],
    set: impl Fn(Edit<'_, A, ()>, Vec<EntityRef>) + 'a,
) {
    let roles = [Role::Solid, Role::Sheet];
    f.reference("bodies", "bodies", bodies.to_vec(), &roles, None, true, set);
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

    /// Its fields in `f`, around those `own` adds: first the bodies (see
    /// [`bodies_field`]) or the features, keyed `features`, each by a face
    /// it made — a hole by its wall — and last, of bodies, how to combine
    /// them (see [`Combine::fields`], a mode defaulting to the newest solid
    /// of `before`). A step repeats one or the other, so only the one in
    /// use waits for picks, and picking bodies empties the features, and
    /// the other way round. `set` puts the seeds set where the arguments
    /// keep them.
    pub(crate) fn fields<'a, A>(
        &self,
        f: &mut EditForm<'a, Design, A>,
        before: &NamedModel<Design>,
        set: impl Fn(&mut A, Seeds) + Clone + 'a,
        own: impl FnOnce(&mut EditForm<'a, Design, A>),
    ) {
        let (seeds, set_bodies) = (self.clone(), set.clone());
        bodies_field(f, &self.bodies, move |edit, picked| {
            let mut seeds = seeds.clone();
            if !picked.is_empty() {
                seeds.features.clear();
            }
            seeds.bodies = picked;
            set_bodies(edit.args, seeds);
        });
        let features = self.features.clone();
        let (seeds, set_features) = (self.clone(), set.clone());
        let roles = [Role::Feature];
        f.reference(
            "features",
            "features",
            features,
            &roles,
            None,
            true,
            move |edit, picked| {
                let mut seeds = seeds.clone();
                if !picked.is_empty() {
                    seeds.bodies.clear();
                }
                seeds.features = picked;
                set_features(edit.args, seeds);
            },
        );
        // Only the one in use is needed; the other is picked for when asked.
        f.form.dialog.optional(if self.features.is_empty() {
            "features"
        } else {
            "bodies"
        });
        own(f);
        if self.features.is_empty() {
            let seeds = self.clone();
            self.combine.fields(f, before, move |args, combine| {
                set(
                    args,
                    Seeds {
                        combine,
                        ..seeds.clone()
                    },
                )
            });
        }
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
/// [`PartOperation::build`].
pub(crate) fn build_pattern<A: Pattern>(
    model: &mut NamedModel<Design>,
    reader: &mut BuildReader,
    step: &str,
    args: &A,
) -> GeopResult<()> {
    let names = A::NAMING.of(step)?;
    let placements = args.placements(&mut Scope { model, reader })?;
    (args.seeds()).repeat(model, &names, step, args.seed_label(), &placements)
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
        let body = part.copy_body(seed, &rename)?.body()?;
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
    let c = plain::centroid(points.into_iter().map(|p| p.to_f64_array()))?;
    Some(Vector3::from_array(c.map(S::from_f64)))
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
pub(crate) fn run_pattern<O: PartOperation>(
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
            part.revise_target(&threads(), &copies?, copy_threads)?;
            draw_threads(part, step)
        }
        None => Ok(()),
    }
}
