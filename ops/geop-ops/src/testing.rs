//! What the tests of the operations share: checking a model valid, finding
//! its entities by where they are, building a part as a program would from
//! a model given, and building programs, reading their steps' arguments as
//! their types. Only built for tests: with `testing`, or in this crate's
//! own.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::{Pose, Ray},
    scalars::{ScalInF64, Scalar},
    vector::Vector3,
};
use geop_core_program::program::ProgramStep;
use geop_core_topology::{
    Body, Edge, FaceId, Model, SolidId,
    validation::{ValidationParameters, validate, validate_manifold},
};

use geop_ops_rasterize::{
    rasterize,
    stl::{enclosed_volume, stl_triangles},
};

use serde::{Serialize, de::DeserializeOwned};

use crate::{
    Design, NamedModel, ParamValue,
    model::{model, pose_parameter},
    operation::{Build, BuildReader, Input, given},
    parameters::Values,
    placed::{Component, Instance, Placement, instance},
    program::{ProgramFile, Registry, Runner, Workspace},
    ui::{Pointer, Reach},
};

// ── Validity ────────────────────────────────────────────────────────────────

/// Whether `model` is valid — a manifold of closed shells too, if
/// `manifold` — and if not, every validation error's root message, one per
/// line.
fn check(model: &Model<impl Scalar>, manifold: bool) -> Result<(), Vec<GeopError>> {
    let params = ValidationParameters::default();
    // `validate_manifold` runs `validate` first.
    match manifold {
        true => validate_manifold(&params, model),
        false => validate(&params, model),
    }
}

/// The report of `errors`: how many, and each one's root message, a line
/// each.
fn report(errors: &[GeopError]) -> String {
    let all: Vec<&str> = errors.iter().map(|e| e.root_message()).collect();
    format!("{} validation error(s):\n{}", all.len(), all.join("\n"))
}

/// Checks `model` is a valid manifold model, failing with every validation
/// error's root message.
#[track_caller]
pub fn assert_model_valid(model: &Model<impl Scalar>) {
    if let Err(errors) = check(model, true) {
        panic!("{}", report(&errors));
    }
}

/// Checks `model`, which may hold open bodies — sheets, wires — is valid,
/// failing with every validation error's root message.
#[track_caller]
pub fn assert_model_valid_sheets(model: &Model<impl Scalar>) {
    if let Err(errors) = check(model, false) {
        panic!("{}", report(&errors));
    }
}

/// Whether `part` is a valid manifold model — if not, every validation
/// error's root message, one per line, and the names of the entities they
/// mention.
pub fn check_valid(part: &NamedModel<impl Scalar>) -> Result<(), String> {
    check(part.topology(), true).map_err(|errors| named_report(part, &errors))
}

/// [`report`] of `errors` in `part`, with the names of the entities they
/// mention.
fn named_report(part: &NamedModel<impl Scalar>, errors: &[GeopError]) -> String {
    let report = report(errors);
    format!(
        "{report}\nwhere {}",
        part.names_mentioned(&report).join(", ")
    )
}

/// Checks `part` is a valid manifold model with consistent names, failing
/// with the report of [`check_valid`].
#[track_caller]
pub fn assert_valid(part: &NamedModel<impl Scalar>) {
    if let Err(report) = check_valid(part) {
        panic!("{report}");
    }
    part.check_names().unwrap();
}

/// Checks `part`, which may hold open bodies — sheets, wires — is valid with
/// consistent names, failing as [`assert_valid`] does.
#[track_caller]
pub fn assert_valid_sheets(part: &NamedModel<impl Scalar>) {
    if let Err(errors) = check(part.topology(), false) {
        panic!("{}", named_report(part, &errors));
    }
    part.check_names().unwrap();
}

// ── Entities by where they are ──────────────────────────────────────────────

/// The point `p`, as sharp scalars.
fn point<S: Scalar>(p: [f64; 3]) -> Vector3<S> {
    Vector3::from_array(p.map(S::from_f64))
}

/// Whether `part` has a vertex that could be at `p`.
pub fn has_vertex<S: Scalar>(part: &NamedModel<S>, p: [f64; 3]) -> bool {
    let p = point(p);
    (part.topology().vertices.values()).any(|vertex| vertex.point.could_be_equal(&p))
}

/// The name of the vertex of `part` that could be at `p`.
#[track_caller]
pub fn vertex_at<S: Scalar>(part: &NamedModel<S>, p: [f64; 3]) -> String {
    let at = point(p);
    let (&id, _) = (part.topology().vertices.iter())
        .find(|(_, vertex)| vertex.point.could_be_equal(&at))
        .unwrap_or_else(|| panic!("no vertex at {p:?}"));
    part.name_of(id).unwrap().to_string()
}

/// The middle of `edge`, halfway along its curve's domain.
pub fn middle<S: Scalar>(edge: &Edge<S>) -> Vector3<S> {
    (edge.curve.evaluate(edge.curve.mid_parameter().unwrap())).unwrap()
}

/// The names of the edges of `part` that `keep` accepts, sorted.
pub fn edges_where<S: Scalar>(
    part: &NamedModel<S>,
    keep: impl Fn(&Edge<S>) -> bool,
) -> Vec<String> {
    let mut names: Vec<String> = (part.topology().edges.iter())
        .filter(|(_, edge)| keep(edge))
        .map(|(&id, _)| part.name_of(id).unwrap().to_string())
        .collect();
    names.sort();
    names
}

/// The names of the edges of `part` whose [`middle`] `keep` accepts, as
/// plain numbers, sorted.
pub fn edges_with_middle_where<S: Scalar>(
    part: &NamedModel<S>,
    keep: impl Fn([f64; 3]) -> bool,
) -> Vec<String> {
    edges_where(part, |edge| {
        keep(middle(edge).to_array().map(|x| x.to_f64()))
    })
}

/// The name of the edge of `part` whose [`middle`] could be at `p`.
#[track_caller]
pub fn edge_at<S: Scalar>(part: &NamedModel<S>, p: [f64; 3]) -> String {
    let at = point(p);
    let names = edges_where(part, |edge| middle(edge).could_be_equal(&at));
    match names.as_slice() {
        [name] => name.clone(),
        _ => panic!("not one edge with its middle at {p:?}: {names:?}"),
    }
}

/// The planar face of `solid` whose outward normal is `normal`, lying
/// furthest that way.
#[track_caller]
pub fn face_towards<S: Scalar>(part: &NamedModel<S>, solid: SolidId, normal: [f64; 3]) -> FaceId {
    let n = point(normal);
    let model = part.topology();
    (model.body_faces(Body::Solid(solid)).unwrap().into_iter())
        .filter_map(|f| {
            let plane = model.get_face(f).unwrap().surface.as_plane().unwrap()?;
            (plane.normal.could_be_equal(&n)).then(|| (f, plane.point.prod_dot(&n).to_f64()))
        })
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .map(|(f, _)| f)
        .unwrap_or_else(|| panic!("no face towards {normal:?}"))
}

// ── Measures ────────────────────────────────────────────────────────────────

/// The volume `solid` of `part` encloses, from its mesh as drawn at
/// `resolution` (see [`rasterize`]): negative if its faces point in. Its
/// triangles' corners are `f32`, as STL stores them: a test's measuring
/// stick, not a kernel comparison.
pub fn volume<S: Scalar>(part: &NamedModel<S>, solid: SolidId, resolution: usize) -> f64 {
    let faces = part.topology().body_faces(Body::Solid(solid)).unwrap();
    let raster = rasterize(part.topology(), resolution).unwrap();
    enclosed_volume(&stl_triangles(&raster, &faces))
}

/// Checks `a` is within `tol` of `b` in every coordinate.
#[track_caller]
pub fn assert_close(a: [f64; 3], b: [f64; 3], tol: f64) {
    assert!(
        (0..3).all(|k| (a[k] - b[k]).abs() < tol),
        "{a:?} is not within {tol} of {b:?}"
    );
}

// ── Parts built from a model ────────────────────────────────────────────────

/// The instance placed, as the generator of its target.
fn placing(
    _: &mut BuildReader,
    placed: &Instance,
) -> geop_core_math::geop_error::GeopResult<Instance> {
    Ok(placed.clone())
}

/// The part a program builds that is run with `inputs`, gives `bodies` as
/// its model in a step `given`, places each of `placed` in a step of its
/// name, fixed at its pose, and last runs `then`.
pub fn built_with(
    inputs: &BTreeMap<String, Input>,
    bodies: &NamedModel<Design>,
    placed: &[(&str, &Rc<Component>, Pose<Design>)],
    then: impl FnOnce(&mut Build),
) -> Build {
    let state = (placed.iter())
        .map(|(name, _, pose)| (pose_parameter(name), ParamValue::Pose(*pose)))
        .collect();
    let mut part = Build::default();
    part.begin_run(inputs, &state);
    part.begin_step("given").unwrap();
    part.define_target(&model(), bodies, given).unwrap();
    for (name, component, _) in placed {
        part.begin_step(name).unwrap();
        let placed = Instance {
            component: Rc::clone(component),
            placement: Placement::Parameter(pose_parameter(name)),
            fixed: true,
        };
        (part.define_target(&instance(name), &placed, placing)).unwrap();
    }
    then(&mut part);
    part.end_run(Default::default());
    part
}

/// The part a program builds that gives `bodies` as its model and nothing
/// else (see [`built_with`]).
pub fn built(bodies: &NamedModel<Design>) -> Build {
    built_with(&BTreeMap::new(), bodies, &[], |_| {})
}

/// The volume the solid named `solid` of `part` encloses, from its mesh as
/// drawn (see [`volume`]).
#[track_caller]
pub fn solid_volume(part: &NamedModel<Design>, solid: &str) -> f64 {
    volume(part, part.solid_id(solid).unwrap(), 64)
}

/// Checks the solid named `solid` of `part` encloses `expected`, to within
/// `tolerance` (see [`solid_volume`]).
#[track_caller]
pub fn assert_volume(part: &NamedModel<Design>, solid: &str, expected: f64, tolerance: f64) {
    let got = solid_volume(part, solid);
    assert!(
        (got - expected).abs() <= tolerance,
        "{solid} encloses {got}, expected {expected} (± {tolerance})"
    );
}

// ── Programs ────────────────────────────────────────────────────────────────

/// Building a program on its own, in an empty [`Workspace`] whose programs
/// are written in the operations `registry` makes.
pub trait Built {
    /// The runner, holding the part the program builds.
    fn built(&self, registry: fn() -> GeopResult<Registry>) -> GeopResult<Runner>;

    /// The model the program builds, or why a step fails.
    fn build_model(
        &self,
        registry: fn() -> GeopResult<Registry>,
    ) -> GeopResult<NamedModel<Design>> {
        Ok(model_of(&self.built(registry)?))
    }
}

impl Built for ProgramFile {
    fn built(&self, registry: fn() -> GeopResult<Registry>) -> GeopResult<Runner> {
        let library = Workspace::new(BTreeMap::new(), registry).scope("");
        let mut runner = Runner::new(registry()?);
        self.run(&mut runner, &Values::new(), &library)?;
        Ok(runner)
    }
}

/// The model the part `runner` built holds: empty if it built no topology.
pub fn model_of(runner: &Runner) -> NamedModel<Design> {
    let built = runner.part.current().retrieve_target(&model());
    built.cloned().unwrap_or_default()
}

/// The arguments of `step`, as their type `A`.
#[track_caller]
pub fn args<A: DeserializeOwned>(step: &ProgramStep) -> A {
    serde_json::from_value(step.args.clone())
        .unwrap_or_else(|e| panic!("step {:?} holds no such arguments: {e}", step.id))
}

/// Changes the arguments of `step`, as their type `A`, by `edit`.
pub fn edit_args<A: Serialize + DeserializeOwned>(
    step: &mut ProgramStep,
    edit: impl FnOnce(&mut A),
) {
    let mut args: A = args(step);
    edit(&mut args);
    step.args = serde_json::to_value(args).expect("arguments are plain data");
}

/// The step `id` of `program`.
#[track_caller]
pub fn step<'a>(program: &'a mut ProgramFile, id: &str) -> &'a mut ProgramStep {
    let index = program.index_of(id).unwrap_or_else(|e| panic!("{e}"));
    &mut program.program.steps[index]
}

/// The error `result` holds, for a test that expects one.
#[track_caller]
pub fn error<T>(result: GeopResult<T>) -> GeopError {
    match result {
        Ok(_) => panic!("expected an error"),
        Err(e) => e,
    }
}

// ── Pointers ────────────────────────────────────────────────────────────────

/// A pointer from `origin` along `dir`, reaching 0.009 units.
pub fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<ScalInF64> {
    let v = |p: [f64; 3]| Vector3::from_array(p.map(ScalInF64::from_f64));
    Pointer {
        ray: Ray::try_new(v(origin), v(dir)).unwrap(),
        reach: Reach::Tube {
            radius: ScalInF64::from_f64(0.009),
        },
    }
}

/// Looking straight down at `(x, y)`.
pub fn from_above(x: f64, y: f64) -> Pointer<ScalInF64> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}
