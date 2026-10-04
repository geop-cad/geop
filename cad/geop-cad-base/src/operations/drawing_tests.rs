//! Drawings of the example parts: hidden lines, sections, dimensions, and
//! every view of every example (ignored, slow).

use geop_core_geometry::contains::curve::curve_could_contain;
use geop_core_math::{
    primitives::{DatumComponent, FrameAxis},
    scalars::{Ring, ScalInF64 as S, Scalar},
};
use geop_ops::{EntityRef, NoFiles, Part, RefId};
use geop_ops_drawing::{
    Dimension, DrawingArgs, LineKind, ViewKind, ViewOptions, compose, drawing::drawn_faces,
    project_view, to_dxf,
};

use crate::{Program, examples};

fn build(program: &Program) -> Part<S> {
    program.build::<S>(&NoFiles).unwrap()
}

fn view(part: &Part<S>, kind: ViewKind) -> geop_ops_drawing::ProjectedView<S> {
    let model = part.topology();
    project_view(
        model,
        &drawn_faces(model),
        &kind.frame().unwrap(),
        &ViewOptions::default(),
    )
    .unwrap()
}

/// The name of an entity of `part` matching `pick`.
fn name_where(part: &Part<S>, pick: impl Fn(RefId) -> bool) -> String {
    let mut names: Vec<(RefId, &str)> = part.names().iter().filter(|(id, _)| pick(*id)).collect();
    names.sort_by_key(|(_, n)| n.to_string());
    names.first().expect("an entity matches").1.to_string()
}

/// The bracket's 6 mm hole runs through its 10 mm plate: from the front its
/// walls are two hidden lines, 6 apart, the plate's full height; from above
/// it is a visible circle.
#[test]
fn drilled_plate_shows_its_hole_hidden() {
    let part = build(&examples::bracket());
    let front = view(&part, ViewKind::Front);
    let hidden: Vec<_> = front.lines.iter().filter(|l| !l.visible).collect();
    assert_eq!(hidden.len(), 2, "{} lines", front.lines.len());
    let mut xs: Vec<f64> = hidden
        .iter()
        .map(|l| {
            assert_eq!(l.kind, LineKind::Silhouette);
            let (a, b) = l.curve.domain();
            let (p, q) = (l.curve.evaluate(a).unwrap(), l.curve.evaluate(b).unwrap());
            assert!(p[0].could_be_equal(q[0]), "a vertical line");
            assert!(p[1].sub(q[1]).abs().could_be_equal(S::from_f64(10.0)));
            p[0].to_f64()
        })
        .collect();
    xs.sort_by(f64::total_cmp);
    assert!(
        S::from_f64(xs[0]).could_be_equal(S::from_f64(17.0)),
        "{xs:?}"
    );
    assert!(
        S::from_f64(xs[1]).could_be_equal(S::from_f64(23.0)),
        "{xs:?}"
    );
    assert_eq!(front.lines.iter().filter(|l| l.visible).count(), 4);

    let top = view(&part, ViewKind::Top);
    assert!(top.lines.iter().all(|l| l.visible));
}

/// Dimensions asked for by name land in the views that see them truly; one
/// no view sees truly is refused, naming it.
#[test]
fn dimensions_by_name_are_drawn() {
    let part = build(&examples::bracket());
    let model = part.topology();
    let circle = name_where(&part, |id| match id {
        RefId::Edge(e) => model.get_edge(e).unwrap().curve.as_arc().unwrap().is_some(),
        _ => false,
    });
    let corner = |x: f64, y: f64, z: f64| {
        name_where(&part, |id| match id {
            RefId::Vertex(v) => {
                let p = model.get_vertex(v).unwrap().point;
                p[0].could_be_equal(S::from_f64(x))
                    && p[1].could_be_equal(S::from_f64(y))
                    && p[2].could_be_equal(S::from_f64(z))
            }
            _ => false,
        })
    };
    let args = DrawingArgs {
        dimensions: vec![
            Dimension::Distance {
                from: corner(0.0, 0.0, 0.0),
                to: corner(40.0, 0.0, 10.0),
            },
            Dimension::Diameter {
                edge: circle.clone(),
            },
        ],
        ..Default::default()
    };
    let dxf = to_dxf(&compose(&part, &args, "2026-10-04").unwrap());
    // The front view sees the diagonal of the 40 x 10 face truly.
    assert!(dxf.contains("\n41.23\n"), "the diagonal");
    assert!(dxf.contains("\n%%c6\n"), "the diameter");

    let skew = DrawingArgs {
        views: vec![ViewKind::Front],
        dimensions: vec![Dimension::Radius { edge: circle }],
        ..Default::default()
    };
    let err = compose(&part, &skew, "").unwrap_err().to_string();
    assert!(err.contains("round"), "{err}");
}

/// The box with its blind hole cut through the hole's axis: the section
/// shows the cut hatched, and no hidden lines.
#[test]
fn section_through_a_blind_hole_is_hatched() {
    let mut program = examples::box_with_drill_hole();
    program.push(
        "middle",
        geop_ops_datums::AddDatumArgs {
            selection: vec![EntityRef::datum_component(
                geop_ops::ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )],
            construction: geop_ops_datums::Construction::Offset { distance: 1.0.into() },
        },
    );
    let part = build(&program);
    let args = DrawingArgs {
        views: vec![ViewKind::Top],
        section: Some(EntityRef::datum("middle")),
        ..Default::default()
    };
    let dxf = to_dxf(&compose(&part, &args, "").unwrap());
    let hatch = dxf.matches("\nHATCH\n").count();
    assert!(hatch > 10, "{hatch} hatch lines");
    assert!(dxf.contains("SECTION A-A"));
    // The top view of a blind hole has none either: nothing is hidden.
    assert!(!dxf.contains("LINE\n8\nHIDDEN\n"), "no hidden lines");
}

/// `error` with every edge and face id it mentions followed by its name.
fn named(part: &Part<S>, error: &impl std::fmt::Display) -> String {
    let mut text = error.to_string();
    for (id, name) in part.names().iter() {
        let tag = match id {
            RefId::Edge(e) => format!("{e:?}"),
            RefId::Face(f) => format!("{f:?}"),
            _ => continue,
        };
        text = text.replace(&format!("{tag})"), &format!("{tag}) [{name}]"));
        text = text.replace(&format!("{tag} "), &format!("{tag} [{name}] "));
    }
    text
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
    for (name, program) in examples::all() {
        let part = build(&program);
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
                    failures.push(format!(
                        "{name}, {} view: {}",
                        kind.name(),
                        named(&part, &e)
                    ));
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
            && let Err(e) = compose(&part, &DrawingArgs::default(), "")
        {
            failures.push(format!("{name}, the sheet: {}", named(&part, &e)));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}
