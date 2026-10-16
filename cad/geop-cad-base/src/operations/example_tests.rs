//! Every example the editor offers, whichever crate ships it: drawn in
//! every view, weighed and halved — and the `.geop`-text examples drawn
//! here, which no operation crate owns. Tests of one example live in the
//! crate of what they test.

use std::collections::BTreeMap;

use geop_core_editor::output::Bounded;
use geop_core_geometry::contains::curve::curve_could_contain;
use geop_core_math::{
    geop_error::GeopError,
    scalars::{Ring, ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::{part::Part, program::Program};
use geop_lib_booleans::{RemeshParams, boolean};
use geop_lib_sweep::shapes::cube_solid;
use geop_ops_assembly::solids::placed_solids;
use geop_ops_drawing::{
    DrawingArgs, LineKind, ProjectedView, ViewKind, ViewLine, ViewOptions, compose,
    drawing::drawn_faces, parts_list, project_view, scene::Scene,
};
use geop_ops_inspect::{
    interference::bounding_box,
    mass::{MassSummary, mass_report},
};
use geop_ops_model::{BooleanOp, NamedModel, Namer, RefId};

use crate::registry::testing::Built;

/// Every example program the editor offers, by name, made.
fn examples() -> Vec<(&'static str, Program)> {
    let registry = crate::registry().unwrap();
    (registry.example_names())
        .map(|name| (name, registry.example(name).unwrap()))
        .collect()
}

/// The model of the example program `name`, built.
fn example(name: &str) -> NamedModel<S> {
    let program = crate::registry().unwrap().example(name).unwrap();
    program.build_model().unwrap()
}

/// `error` with every edge and face id it mentions followed by its name.
fn named(part: &NamedModel<S>, error: &impl std::fmt::Display) -> String {
    let mut text = error.to_string();
    for (id, name) in part.names().iter() {
        let tag = match id {
            RefId::Edge(e) => format!("{e:?}"),
            RefId::Face(f) => format!("{f:?}"),
            _ => continue,
        };
        text = text.replace(&tag, &format!("{tag} [{name}]"));
    }
    text
}

/// The view `kind` of the example `name`, or its error with the entities
/// it mentions named.
fn example_view(name: &str, kind: ViewKind) -> Result<ProjectedView<S>, String> {
    let part = &example(name);
    let model = part.topology();
    project_view(
        model,
        &drawn_faces(model),
        &kind.frame().unwrap(),
        &ViewOptions::default(),
    )
    .map_err(|e| named(part, &e))
}

/// The flange turned about `y`, seen from the back along that axis: the
/// waist of its curved side, where the profile runs parallel to the axis,
/// is its silhouette, a circle of radius 13.04 under the 13.14 rim of its
/// top, so hidden. Its quarter faces meet on the paper's axes, and the
/// silhouette's pieces end there.
#[test]
fn cool_flange_from_the_back_draws() {
    let back = example_view("cool_flange", ViewKind::Back).unwrap();
    let silhouettes: Vec<_> = back
        .lines
        .iter()
        .filter(|l| l.kind == LineKind::Silhouette)
        .map(|l| {
            let (a, b) = l.curve.domain();
            let at = |t: S| {
                let p = l.curve.evaluate(t).unwrap();
                (p[0].to_f64(), p[1].to_f64())
            };
            (at(a), at(b), l.visible)
        })
        .collect();
    assert!(!silhouettes.is_empty());
    for &(p, q, visible) in &silhouettes {
        let r = |(x, y): (f64, f64)| (x * x + y * y).sqrt();
        assert!(
            !visible && (r(p) - 13.04).abs() < 0.01 && (r(q) - 13.04).abs() < 0.01,
            "{silhouettes:?}"
        );
    }
}

/// The conebox's double cone turns about the box's top edge `c12`, its
/// apexes on that edge. Seen from the top, across its axis, each cone's
/// silhouette is two generators in the box's top plane, running from the
/// apex to the base circle, about 13.7 long: no face of either cone has a
/// silhouette piece shorter than that, as one sitting at the apex would be.
#[test]
fn conebox_cone_from_the_top_shows_whole_generators() {
    let part = &example("conebox");
    let d = ViewKind::Top.frame::<S>().unwrap().direction.vector();
    for curve in ["c9", "c10"] {
        for quarter in ["q0", "q1", "q2", "q3"] {
            let name = format!("revolve(revolve1,sketch2,{curve},{quarter})");
            let Some(RefId::Face(face)) = part.names().id_of(&name) else {
                panic!("no face {name}");
            };
            let pieces =
                geop_ops_drawing::silhouette::face_silhouettes(part.topology(), face, &d).unwrap();
            for piece in &pieces {
                let (a, b) = piece.curve.domain();
                let (p, q) = (
                    piece.curve.evaluate(a).unwrap(),
                    piece.curve.evaluate(b).unwrap(),
                );
                let length = p.sub(&q).norm().to_f64();
                assert!(length > 13.0, "{name}: a piece from {p:?} to {q:?}");
            }
        }
    }
}

#[test]
fn conebox_from_the_top_draws() {
    example_view("conebox", ViewKind::Top).unwrap();
}

/// Views known not to build, and why.
///
/// `revolved_cone_on_box` from the front and back: the boolean's vertex
/// where the cone's intersection with the box's face `y = 1.18`
/// (`combine(revolve1,extrude(extrude1,sketch1,c4),...)`, edge `a`) meets the
/// box's right face lies at `x = 2.2456168`, 3.1e-5 off that face
/// (`x = 2.2456482`). On the paper, `a` then ends 3e-5 short of where the
/// cone's base circle (`combine(revolve1,revolve(revolve1,sketch2,p3,q0),...)`)
/// ends, running into it tangentially, and the search for where the two
/// cross cannot isolate a near-touch that close. The defect is the vertex's
/// accuracy in the boolean, not the drawing.
const KNOWN_TO_FAIL: [(&str, ViewKind); 2] = [
    ("revolved_cone_on_box", ViewKind::Front),
    ("revolved_cone_on_box", ViewKind::Back),
];

/// Every view of every example builds, and no piece of a line is drawn
/// both visible and hidden.
#[test]
#[ignore = "slow: every view of every example — run with `cargo test -- --ignored`"]
fn every_example_draws() {
    let mut failures = Vec::new();
    for (name, program) in examples() {
        let built = program.built().unwrap();
        // A part with no model of its own is drawn as an empty one.
        let part = &(geop_ops_model::read_model(&built.current()))
            .map(|model| model.into_owned())
            .unwrap_or_default();
        for kind in ViewKind::ALL {
            let model = part.topology();
            let v = match project_view(
                model,
                &drawn_faces(model),
                &kind.frame().unwrap(),
                &ViewOptions::default(),
            ) {
                Ok(v) => v,
                Err(_) if KNOWN_TO_FAIL.contains(&(name, kind)) => continue,
                Err(e) => {
                    failures.push(format!("{name}, {} view: {}", kind.name(), named(part, &e)));
                    continue;
                }
            };
            // A hidden line may cross or touch a visible one, never run
            // along it: no hidden piece has the points a third and two
            // thirds along it both on one visible line.
            for hidden in v.lines.iter().filter(|l| !l.visible) {
                let (a, b) = hidden.curve.domain();
                let at = |f: f64| {
                    let t = a.add(b.sub(a).mul(S::from_f64(f))).sharpen();
                    hidden.curve.evaluate(t).unwrap()
                };
                let (p, q) = (at(1.0 / 3.0), at(2.0 / 3.0));
                for visible in v.lines.iter().filter(|l| l.visible) {
                    let on = |x: &geop_core_math::vector::Vector2<S>| {
                        curve_could_contain(&visible.curve, x, 20_000, S::from_f64(1e-7))
                            .unwrap()
                            .is_some()
                    };
                    assert!(
                        !(on(&p) && on(&q)),
                        "{name}, {} view: the hidden {:?} of edge {:?} runs along the visible \
                         {:?} of edge {:?}",
                        kind.name(),
                        hidden.kind,
                        hidden.edge.and_then(|e| part.name_of(e)),
                        visible.kind,
                        visible.edge.and_then(|e| part.name_of(e)),
                    );
                }
            }
        }
        // The whole default sheet, too, of every example whose views build.
        if !KNOWN_TO_FAIL.iter().any(|(n, _)| *n == name)
            && let Err(e) = compose(&built.current(), &DrawingArgs::default(), "", &[])
        {
            failures.push(format!("{name}, the sheet: {}", named(part, &e)));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}

/// The example workspace `name` built: its first file, as that file's
/// path, and the part.
fn example_assembly(name: &str) -> (&'static str, Part) {
    let files = crate::registry().unwrap().workspace_example(name).unwrap();
    let (path, program) = files[0].clone();
    let files: BTreeMap<String, String> = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    (
        path,
        crate::build(&program, path, crate::workspace(files))
            .unwrap()
            .part,
    )
}

/// Every example assembly draws: its default sheet with the bill of
/// materials builds, with a balloon per line of the bill whose part has
/// faces drawn, and in no view does a hidden piece run along a visible one.
#[test]
#[ignore = "slow: every example assembly drawn — run with `cargo test -- --ignored`"]
fn every_example_assembly_draws() {
    use geop_lib_sheet::Shape;
    use geop_ops_drawing::layer;
    let mut failures = Vec::new();
    let registry = crate::registry().unwrap();
    for name in registry.workspace_example_names() {
        let (path, built) = example_assembly(name);
        let args = DrawingArgs {
            bom: true,
            ..Default::default()
        };
        let parts = parts_list(&built.current(), path, &args).unwrap();
        let scene = Scene::of(&built.current()).unwrap();
        let sheet = match compose(&built.current(), &args, "", &parts) {
            Ok(sheet) => sheet,
            Err(e) => {
                failures.push(format!("{name}: {e}"));
                continue;
            }
        };
        let drawn = parts
            .iter()
            .filter(|line| {
                scene
                    .bodies
                    .iter()
                    .any(|b| line.placements.contains(&b.path))
            })
            .count();
        let balloons = sheet
            .strokes
            .iter()
            .filter(|s| {
                s.layer == layer::DIMENSION
                    && matches!(s.shape, Shape::Circle { radius, .. } if radius == 4.0)
            })
            .count();
        if balloons != drawn {
            failures.push(format!(
                "{name}: {balloons} balloons for {drawn} lines of the bill drawn"
            ));
        }
        for kind in ViewKind::ALL {
            let v = match scene.project(&kind.frame().unwrap(), &ViewOptions::default()) {
                Ok(v) => v,
                Err(e) => {
                    failures.push(format!("{name}, {} view: {e}", kind.name()));
                    continue;
                }
            };
            for hidden in v.lines.iter().filter(|l| !l.visible) {
                let (a, b) = hidden.curve.domain();
                let at = |f: f64| {
                    let t = a.add(b.sub(a).mul(S::from_f64(f))).sharpen();
                    hidden.curve.evaluate(t).unwrap()
                };
                let (p, q) = (at(1.0 / 3.0), at(2.0 / 3.0));
                let on = |line: &ViewLine<S>, x| {
                    curve_could_contain(&line.curve, x, 20_000, S::from_f64(1e-7))
                        .unwrap()
                        .is_some()
                };
                if let Some(visible) = v
                    .lines
                    .iter()
                    .filter(|l| l.visible)
                    .find(|l| on(l, &p) && on(l, &q))
                {
                    failures.push(format!(
                        "{name}, {} view: the hidden {:?} of {} runs along the visible {:?} of {}",
                        kind.name(),
                        hidden.kind,
                        scene.bodies[hidden.body].path,
                        visible.kind,
                        scene.bodies[visible.body].path,
                    ));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}

/// The checks every solid's mass properties pass: converged, a positive
/// volume, a symmetric inertia tensor whose principal moments are positive
/// and satisfy the triangle inequality, as every body's do.
fn check_mass(name: &str, summary: &MassSummary, failures: &mut Vec<String>) {
    let mut fail = |why: String| failures.push(format!("{name}: {why}"));
    if !summary.converged {
        fail("did not converge".into());
    }
    if summary.volume.value - summary.volume.error <= 0.0 {
        fail(format!("volume {:?}", summary.volume));
    }
    for a in 0..3 {
        for b in 0..3 {
            let (x, y) = (summary.inertia[a][b], summary.inertia[b][a]);
            if (x.value - y.value).abs() > x.error + y.error {
                fail(format!("I[{a}][{b}] = {x:?} but I[{b}][{a}] = {y:?}"));
            }
        }
    }
    let [i1, i2, i3] = summary.principal_moments;
    if i1.value - i1.error <= 0.0 {
        fail(format!("a principal moment {i1:?} is not positive"));
    }
    if i1.value + i2.value + i1.error + i2.error < i3.value - i3.error {
        fail(format!("principal moments {i1:?} + {i2:?} < {i3:?}"));
    }
}

/// Every example part: every solid's mass properties pass [`check_mass`],
/// and its volume is that of its two pieces — cut across by a box, by
/// booleans, each piece's volume integrated on its own.
#[test]
#[ignore = "slow: every example's mass properties, and booleans halving each solid — run with `cargo test -- --ignored`"]
fn example_mass_properties_are_consistent() {
    let mut failures = Vec::new();
    for (example, program) in examples() {
        let part = program.built().unwrap();
        let reader = part.current();
        let report = mass_report(&reader).unwrap();
        for (body, placed) in report.bodies.iter().zip(placed_solids(&reader).unwrap()) {
            let name = format!("{example}/{}", body.name);
            let Some(summary) = &body.properties else {
                failures.push(format!("{name}: {:?}", body.error));
                continue;
            };
            check_mass(&name, summary, &mut failures);
            // The part of its box below a plane across x, a little beyond
            // it elsewhere. Not through the centre of mass: of a symmetric
            // part, that is where its circles' seams are, and a cut placed
            // on an existing edge asks the boolean for a degenerate crossing
            // (see `AGENTS.md`) — so a fraction of the width no construction
            // chooses.
            assert!(placed.pose.is_none(), "{name}: an example places no part");
            let faces = placed.model.topology().solid_faces(placed.solid).unwrap();
            let bounds = bounding_box(&placed, &faces).unwrap();
            let lo = |k: usize| bounds[k].lower().to_f64() - 1.0;
            let hi = |k: usize| bounds[k].upper().to_f64() + 1.0;
            let cut = bounds[0].lower().to_f64() + 0.437_281 * bounds[0].width().to_f64();
            let halves = [BooleanOp::Intersection, BooleanOp::Difference].map(|op| {
                let mut scratch = placed.model.clone().into_owned();
                let knife = cube_solid(
                    &mut scratch,
                    "knife",
                    Vector3::from_array([lo(0), lo(1), lo(2)].map(S::from_f64)),
                    Vector3::from_array([cut, hi(1), hi(2)].map(S::from_f64)),
                )?;
                let namer = Namer::new("halve", "half")?;
                let half = boolean(
                    &mut scratch,
                    &namer,
                    placed.solid,
                    knife,
                    op,
                    RemeshParams::default(),
                )?;
                let Some(half) = half else {
                    return Ok(S::ZERO);
                };
                Ok(scratch.topology().mass_properties(half)?.volume)
            });
            match halves {
                [Ok(a), Ok(b)] => {
                    let sum = Bounded::of(a.add(b));
                    let whole = summary.volume;
                    if (sum.value - whole.value).abs() > sum.error + whole.error {
                        failures.push(format!(
                            "{name}: halves {sum:?} do not add up to the whole {whole:?}"
                        ));
                    }
                }
                [a, b] => failures.push(format!(
                    "{name}: halving failed: {:?}",
                    [a.err(), b.err()]
                        .into_iter()
                        .flatten()
                        .map(|e: GeopError| e.to_string())
                        .collect::<Vec<_>>()
                )),
            }
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
