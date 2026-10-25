//! What the tests of the operations share about the model: checking it
//! valid and drawn as its trims say, finding its entities by where they
//! are, the model an editor shows, building a part of a model given and
//! steps run on it one by one, and the model a program builds. Only built
//! for tests: with `testing`, or in this crate's own.

// Helpers of tests fail as tests do: by panicking, naming what is wrong.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use geop_core_editor::editor::Editor;
use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    operation::Operation, operation_registry::OperationRegistry, part::Part, program::Program,
    reader::Reader, runner::Runner, values::Design,
};
use geop_core_rasterize::{
    face_triangles_uv, rasterize,
    stl::{enclosed_volume, stl_triangles},
};
use geop_core_topology::{
    Body, Edge, FaceId, Model, SolidId,
    boundary::BoundaryType,
    contains::shell::{PointClassification, shell_contains},
    uv_loop::sample_loop_to_polygon,
    validation::{ValidationParameters, validate, validate_manifold},
};

use crate::{NamedModel, read_model};

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

/// Twice the signed area of a closed polygon.
fn area2(polygon: &[[f64; 2]]) -> f64 {
    (0..polygon.len())
        .map(|i| {
            let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum()
}

/// Checks every face of `part` is drawn exactly where its trims say: the
/// triangles the rasterizer draws it with cover, in `(u, v)`, the area its
/// outer boundary encloses less its holes' — as the rasterizer samples
/// them. A face drawn over a hole it goes around, or leaving a gap, covers
/// more or less. Plain `f64`, like the rasterizer.
#[track_caller]
pub fn assert_draws_its_trims(part: &NamedModel<impl Scalar>) {
    const N: usize = 24;
    let model = part.topology();
    for (&id, face) in &model.faces {
        let enclosed: f64 = face
            .boundaries()
            .filter_map(|b| match b {
                BoundaryType::Loop(anchor) => Some(anchor),
                BoundaryType::Vertex(_) => None,
            })
            .map(|anchor| {
                let polygon: Vec<[f64; 2]> = sample_loop_to_polygon(model, anchor, N)
                    .unwrap()
                    .iter()
                    .map(|p| [p[0].to_f64(), p[1].to_f64()])
                    .collect();
                area2(&polygon) / 2.0
            })
            .sum::<f64>()
            .abs();
        let covered: f64 = face_triangles_uv(model, face, N)
            .unwrap()
            .iter()
            .map(|(a, b, c)| area2(&[a, b, c].map(|p| [p[0].to_f64(), p[1].to_f64()])).abs() / 2.0)
            .sum();
        assert!(
            (covered - enclosed).abs() <= 1e-9 * enclosed.max(1e-9),
            "face {:?} is drawn covering {covered} of its (u, v), where its trims enclose {enclosed}",
            part.name_of(id),
        );
    }
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

/// Where the point `p` lies relative to the solid `solid` of `part`:
/// inside it, outside it, or on its boundary — inside, if inside any of
/// its shells.
pub fn classify(part: &NamedModel<impl Scalar>, solid: &str, p: [f64; 3]) -> PointClassification {
    let model = part.topology();
    let solid = part.solid_id(solid).unwrap();
    let mut result = PointClassification::Outside;
    for &shell in &model.get_solid(solid).unwrap().shells {
        match shell_contains(model, shell, point(p), 2000, Scalar::from_f64(1e-6), 7).unwrap() {
            PointClassification::Outside => {}
            other => result = other,
        }
    }
    result
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

/// A unit cube, every entity named by its kind and its position among
/// those: `v0` … `v7`, `e0` … `e11`, `f0` … `f5`, `s0`.
pub fn unit_cube() -> NamedModel<Design> {
    let topology = geop_core_topology::test_fixtures::test_cube_model::<Design>();
    let mut names = crate::NameRegistry::default();
    let mut name = |id: crate::RefId, kind: &str, k: usize| {
        names
            .insert(id, format!("{kind}{k}"))
            .map_err(|e| e.with_context("unit_cube"))
    };
    for (k, &id) in topology.vertices.keys().enumerate() {
        name(id.into(), "v", k).expect("names are fresh");
    }
    for (k, &id) in topology.edges.keys().enumerate() {
        name(id.into(), "e", k).expect("names are fresh");
    }
    for (k, &id) in topology.faces.keys().enumerate() {
        name(id.into(), "f", k).expect("names are fresh");
    }
    for (k, &id) in topology.solids.keys().enumerate() {
        name(id.into(), "s", k).expect("names are fresh");
    }
    NamedModel {
        topology,
        names,
        features: Vec::new(),
    }
}

// ── What an editor shows ────────────────────────────────────────────────────

/// The model the editor `editor` shows: what the program it edits builds,
/// as far as it runs it.
pub fn model_shown(editor: &Editor) -> NamedModel<Design> {
    read_model(&editor.part()).unwrap().into_owned()
}

// ── Parts of a model given and one step ─────────────────────────────────────

/// The generator of a model given: `model` itself.
fn given(_: &Reader, model: &NamedModel<Design>) -> GeopResult<NamedModel<Design>> {
    Ok(model.clone())
}

/// A part of one step that defines its model as `model`: what the tests of
/// an operation, an exporter or an inspector build on.
pub fn given_part(model: &NamedModel<Design>) -> GeopResult<Part> {
    let mut part = Part::default();
    let runner = Runner::new(OperationRegistry::default());
    runner.run_step(&mut part, GIVEN, |ctx| {
        ctx.produce(&crate::model(), model, given)
    })?;
    Ok(part)
}

/// The id of the step that gives a part its model, in [`given_part`].
pub const GIVEN: &str = "given";

/// Runs `O`'s step `step` with `args` on `part`, after the steps it ran:
/// the next step of a program built one step at a time, on a model given
/// (see [`given_part`]).
pub fn run_step<O: Operation>(part: &mut Part, step: &str, args: &O::Args) -> GeopResult<()> {
    let runner = Runner::new(OperationRegistry::default());
    runner.run_step(part, step, |ctx| O::run(ctx, args))
}

/// What `O`'s step `step` with `args` makes of `model`, as a program of two
/// steps would build it — the model given, then the step — together with
/// what else the step made: the part, after its one run. For an
/// operation's tests, which build the model they act on directly.
pub fn apply<O: Operation>(
    model: NamedModel<Design>,
    step: &str,
    args: &O::Args,
) -> GeopResult<Part> {
    let mut part = given_part(&model)?;
    run_step::<O>(&mut part, step, args)?;
    Ok(part)
}

// ── Parts built by programs ─────────────────────────────────────────────────

/// A runner of programs written in the operations `registry` makes, as
/// the file `part.geop` — of a workspace of no other files.
pub fn runner_of(registry: OperationRegistry) -> Runner {
    let mut runner = Runner::new(registry);
    runner.location.file = "part.geop".into();
    runner
}

/// Building a program on its own, written in the operations `registry`
/// makes.
pub trait Built {
    /// The part the program builds.
    fn built(&self, registry: fn() -> GeopResult<OperationRegistry>) -> GeopResult<Part>;

    /// The model the program builds, or why a step fails.
    fn build_model(
        &self,
        registry: fn() -> GeopResult<OperationRegistry>,
    ) -> GeopResult<NamedModel<Design>> {
        Ok(model_of(&self.built(registry)?))
    }
}

impl Built for Program {
    fn built(&self, registry: fn() -> GeopResult<OperationRegistry>) -> GeopResult<Part> {
        let runner = runner_of(registry()?);
        let mut part = Part::default();
        self.run(&runner, &mut part)?;
        Ok(part)
    }
}

/// The model `part` holds: empty if it built no topology.
pub fn model_of(part: &Part) -> NamedModel<Design> {
    (read_model(&part.current()).map(|model| model.into_owned())).unwrap_or_default()
}

/// The model `O`'s step `step` with `args` leaves of `model` (see
/// [`apply`]): for the tests of an operation that builds on the model.
pub fn apply_to_model<O: Operation>(
    model: NamedModel<Design>,
    step: &str,
    args: &O::Args,
) -> GeopResult<NamedModel<Design>> {
    let part = apply::<O>(model, step, args)?;
    Ok(read_model(&part.current())?.into_owned())
}

/// The error `result` holds, for a test that expects one.
#[track_caller]
pub fn error<T>(result: GeopResult<T>) -> GeopError {
    match result {
        Ok(_) => panic!("expected an error"),
        Err(e) => e,
    }
}

// ── Sweeps ──────────────────────────────────────────────────────────────────

/// What one case of a sweep came to.
pub enum Outcome {
    /// A valid solid.
    Built,
    /// Refused as unsupported, saying why.
    Refused,
    /// Anything else: an error that is not a refusal, an invalid result, a
    /// panic — what the case is listed for.
    Wrong(String),
}

/// Applies a step of `O` with `args` to a copy of `part` and says how it
/// went: refused, where the error's root message is one `is_refusal`
/// accepts — the operation refusing what it does not support, rather than
/// failing at what it does.
pub fn outcome<O: Operation>(
    part: &NamedModel<Design>,
    args: &O::Args,
    is_refusal: fn(&str) -> bool,
) -> Outcome {
    let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        apply_to_model::<O>(part.clone(), "op", args)
    }));
    match run {
        Err(panic) => Outcome::Wrong(format!(
            "panicked: {}",
            panic
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| panic.downcast_ref::<&str>().map(|s| s.to_string()))
                .unwrap_or_default()
        )),
        Ok(Err(e)) if is_refusal(e.root_message()) => Outcome::Refused,
        Ok(Err(e)) => Outcome::Wrong(format!("failed: {e:?}")),
        Ok(Ok(built)) => match check_valid(&built) {
            Ok(()) => Outcome::Built,
            Err(e) => Outcome::Wrong(format!("invalid: {e}")),
        },
    }
}

/// The cases of a sweep, as they come: how many built, how many were
/// refused, and every one that went wrong.
#[derive(Default)]
pub struct Cases {
    built: usize,
    refused: usize,
    wrong: Vec<String>,
}

impl Cases {
    /// Records that the case `case` came to `outcome`.
    pub fn record(&mut self, case: String, outcome: Outcome) {
        match outcome {
            Outcome::Built => self.built += 1,
            Outcome::Refused => self.refused += 1,
            Outcome::Wrong(why) => self.wrong.push(format!("{case}: {why}")),
        }
    }

    /// Checks no case went wrong, listing every one that did.
    #[track_caller]
    pub fn assert_none_wrong(&self) {
        let Cases {
            built,
            refused,
            wrong,
        } = self;
        assert!(
            wrong.is_empty(),
            "{} of {} cases went wrong ({built} built, {refused} refused):\n\n{}",
            wrong.len(),
            wrong.len() + built + refused,
            wrong.join("\n\n")
        );
    }
}
