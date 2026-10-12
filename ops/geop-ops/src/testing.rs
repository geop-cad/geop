//! What the tests of the operations share: checking a model valid, finding
//! its entities by where they are, and building a part as a program would
//! from a model given. Only built for tests: with `testing`, or in this
//! crate's own.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_math::{geop_error::GeopError, primitives::Pose, scalars::Scalar, vector::Vector3};
use geop_core_topology::{
    Body, Edge, FaceId, Model, SolidId,
    validation::{ValidationParameters, validate, validate_manifold},
};

use crate::{
    Design, NamedModel, ParamValue,
    model::{model, pose_parameter},
    operation::{Build, BuildReader, Input, given},
    placed::{Component, Instance, Placement, instance},
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
    let (t0, t1) = edge.curve.domain();
    (edge
        .curve
        .evaluate(S::interpolate(t0, t1, S::from_f64(0.5))))
    .unwrap()
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
