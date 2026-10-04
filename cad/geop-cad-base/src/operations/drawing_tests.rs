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
    let dxf = to_dxf(&compose(&part, &args, "2026-10-04", &[]).unwrap());
    // The front view sees the diagonal of the 40 x 10 face truly.
    assert!(dxf.contains("\n41.23\n"), "the diagonal");
    assert!(dxf.contains("\n%%c6\n"), "the diameter");

    let skew = DrawingArgs {
        views: vec![ViewKind::Front],
        dimensions: vec![Dimension::Radius { edge: circle }],
        ..Default::default()
    };
    let err = compose(&part, &skew, "", &[]).unwrap_err().to_string();
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
            construction: geop_ops_datums::Construction::Offset {
                distance: 1.0.into(),
            },
        },
    );
    let part = build(&program);
    let args = DrawingArgs {
        views: vec![ViewKind::Top],
        section: Some(EntityRef::datum("middle")),
        ..Default::default()
    };
    let dxf = to_dxf(&compose(&part, &args, "", &[]).unwrap());
    let hatch = dxf.matches("\nHATCH\n").count();
    assert!(hatch > 10, "{hatch} hatch lines");
    assert!(dxf.contains("SECTION A-A"));
    // The top view of a blind hole has none either: nothing is hidden.
    assert!(!dxf.contains("LINE\n8\nHIDDEN\n"), "no hidden lines");
}

/// The hole plate's tapped M6 hole, 8 deep into the 10 mm plate, drawn as
/// drafting draws an internal thread: from above, end on, three quarters
/// of a thin circle at the major diameter, labelled `M6x1`; from the front
/// and the right, two hidden lines at the major diameter, 6 apart, as deep
/// as the thread runs. Without hidden lines, those go.
#[test]
fn a_tapped_hole_draws_its_thread() {
    use geop_ops_drawing::sheet::{Layer, Shape};
    let part = build(&examples::hole_plate());
    let args = DrawingArgs {
        scale: Some(2.0),
        ..Default::default()
    };
    let sheet = compose(&part, &args, "", &[]).unwrap();
    let arcs: Vec<f64> = sheet
        .strokes
        .iter()
        .filter_map(|s| match (&s.layer, &s.shape) {
            (
                Layer::Thread,
                Shape::Arc {
                    radius, start, end, ..
                },
            ) => {
                let mut sweep = end - start;
                while sweep <= 0.0 {
                    sweep += std::f64::consts::TAU;
                }
                assert!((sweep.to_degrees() - 270.0).abs() < 1e-9, "{sweep}");
                Some(*radius / 2.0)
            }
            _ => None,
        })
        .collect();
    assert_eq!(arcs.len(), 1, "one view sees the hole end on: {arcs:?}");
    assert!(
        (arcs[0] - 3.0).abs() < 1e-9,
        "at the major radius: {arcs:?}"
    );
    let hidden: Vec<(f64, f64)> = sheet
        .strokes
        .iter()
        .filter_map(|s| match (&s.layer, &s.shape) {
            (Layer::Hidden, Shape::Line(a, b)) if (a[0] - b[0]).abs() < 1e-9 => {
                Some((a[0], (a[1] - b[1]).abs() / 2.0))
            }
            _ => None,
        })
        .filter(|(_, length)| (length - 8.0).abs() < 1e-9)
        .collect();
    // The front and right views: the hole's walls, 5 apart (its tap drill),
    // and the thread's two lines each, 6 apart.
    assert_eq!(hidden.len(), 8, "{hidden:?}");
    let apart = |d: f64| {
        let mut pairs = 0;
        for (i, a) in hidden.iter().enumerate() {
            for b in &hidden[i + 1..] {
                if ((a.0 - b.0).abs() / 2.0 - d).abs() < 1e-9 {
                    pairs += 1;
                }
            }
        }
        pairs
    };
    assert_eq!((apart(5.0), apart(6.0)), (2, 2), "{hidden:?}");
    assert_eq!(
        sheet.labels.iter().filter(|l| l.text == "M6x1").count(),
        1,
        "labelled once"
    );
    assert!(
        !sheet
            .strokes
            .iter()
            .any(|s| s.layer == Layer::Thread && matches!(s.shape, Shape::Line(..))),
        "no view sees the thread from the side"
    );

    let without = DrawingArgs {
        hidden_lines: false,
        ..args
    };
    let sheet = compose(&part, &without, "", &[]).unwrap();
    assert!(!sheet.strokes.iter().any(|s| s.layer == Layer::Hidden));
    assert_eq!(
        sheet
            .strokes
            .iter()
            .filter(|s| s.layer == Layer::Thread)
            .count(),
        1
    );
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
        let part = build(&program());
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
            && let Err(e) = compose(&part, &DrawingArgs::default(), "", &[])
        {
            failures.push(format!("{name}, the sheet: {}", named(&part, &e)));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n\n"));
}

/// The example workspace `name` built: its first file, as that file's
/// path, and the part.
fn example_assembly(name: &str) -> (&'static str, Part<S>) {
    let (_, files) = crate::examples::workspaces()
        .into_iter()
        .find(|(n, _)| *n == name)
        .expect("the example is there");
    let files = files();
    let (path, program) = files[0].clone();
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    let workspace = crate::Workspace::<S>::new(crate::stdlib::WithStandardParts(files));
    (path, program.build(&workspace.scope(path)).unwrap())
}

/// The paper's `y` range a line spans: its heights in the front view.
fn heights(line: &geop_ops_drawing::ViewLine<S>) -> (f64, f64) {
    let (a, b) = line.curve.domain();
    let (p, q) = (
        line.curve.evaluate(a).unwrap(),
        line.curve.evaluate(b).unwrap(),
    );
    let (p, q) = (p[1].to_f64(), q[1].to_f64());
    (p.min(q), p.max(q))
}

/// The bolted plate from the front: the 5 mm plate, the M4x12 screw's
/// head on it, its shank hidden inside the plate and inside the 3.2 mm nut
/// under it and seen below that, down to its end 7 under the plate. With
/// its bill of materials, the sheet balloons the plate, the screw and the
/// nut — items 1 to 3.
#[test]
fn bolted_plate_from_the_front_hides_the_screw_in_the_plate() {
    use geop_ops_drawing::{
        scene::Scene,
        sheet::{Layer, Shape},
    };
    let (path, part) = example_assembly("bolted_plate");
    let scene = Scene::of(&part).unwrap();
    let paths: Vec<&str> = scene.bodies.iter().map(|b| b.path.as_str()).collect();
    assert_eq!(paths, ["plate", "screw", "nut"]);
    let front = scene
        .project(&ViewKind::Front.frame().unwrap(), &ViewOptions::default())
        .unwrap();
    let of = |path: &str, visible: bool| -> Vec<(f64, f64)> {
        front
            .lines
            .iter()
            .filter(|l| scene.bodies[l.body].path == path && l.visible == visible)
            .map(heights)
            .collect()
    };
    let inside = |(lo, hi): (f64, f64), from: f64, to: f64| lo >= from - 1e-9 && hi <= to + 1e-9;
    let screw_hidden = of("screw", false);
    assert!(
        screw_hidden
            .iter()
            .any(|&h| inside(h, -3.2, 5.0) && h.1 - h.0 > 8.1),
        "the shank through the plate and the nut is hidden: {screw_hidden:?}"
    );
    let screw_seen = of("screw", true);
    assert!(
        screw_seen
            .iter()
            .all(|&h| !inside(h, 0.0, 5.0) || h.1 - h.0 < 1e-9),
        "nothing of the screw is seen through the plate: {screw_seen:?}"
    );
    assert!(
        screw_seen.iter().any(|&h| h.0 >= 5.0 - 1e-9),
        "its head is seen on the plate: {screw_seen:?}"
    );
    assert!(
        screw_seen
            .iter()
            .any(|&h| inside(h, -7.0, -3.2) && h.1 - h.0 > 3.0),
        "its end is seen under the nut: {screw_seen:?}"
    );
    let nut_seen = of("nut", true);
    assert!(
        !nut_seen.is_empty() && nut_seen.iter().all(|&h| inside(h, -3.2, 0.0)),
        "the nut is seen under the plate: {nut_seen:?}"
    );

    let args = DrawingArgs {
        views: vec![ViewKind::Front],
        bom: true,
        ..Default::default()
    };
    let parts = crate::inspect::parts_list(&part, path, &args).unwrap();
    let sheet = compose(&part, &args, "", &parts).unwrap();
    let balloons = sheet
        .strokes
        .iter()
        .filter(|s| {
            s.layer == Layer::Dimension
                && matches!(s.shape, Shape::Circle { radius, .. } if radius == 4.0)
        })
        .count();
    assert_eq!(balloons, 3);
    for item in ["1", "2", "3"] {
        assert_eq!(
            sheet
                .labels
                .iter()
                .filter(|l| l.layer == Layer::Dimension && l.text == item)
                .count(),
            1,
            "balloon {item}"
        );
    }
}

/// Every example assembly draws: its default sheet with the bill of
/// materials builds, with a balloon per line of the bill whose part has
/// faces drawn, and in no view does a hidden piece run along a visible one.
#[test]
#[ignore = "slow: every example assembly drawn — run with `cargo test -- --ignored`"]
fn every_example_assembly_draws() {
    use geop_ops_drawing::{
        scene::Scene,
        sheet::{Layer, Shape},
    };
    let mut failures = Vec::new();
    for (name, _) in crate::examples::workspaces() {
        let (path, part) = example_assembly(name);
        let args = DrawingArgs {
            bom: true,
            ..Default::default()
        };
        let parts = crate::inspect::parts_list(&part, path, &args).unwrap();
        let sheet = match compose(&part, &args, "", &parts) {
            Ok(sheet) => sheet,
            Err(e) => {
                failures.push(format!("{name}: {e}"));
                continue;
            }
        };
        let scene = Scene::of(&part).unwrap();
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
                s.layer == Layer::Dimension
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
                let on = |line: &geop_ops_drawing::ViewLine<S>, x| {
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

/// The bolted plate cut through the screw's axis: each part cut in its own
/// frame, the plate, the screw and the nut each hatched, the hatching
/// turning the other way from one part to the next; and the hole of the
/// plate placed in it dimensioned by its name as the assembly names it.
#[test]
fn an_assembly_is_cut_and_dimensioned_through_its_parts() {
    use geop_ops_drawing::sheet::{Layer, Shape};
    let (_, files) = crate::examples::workspaces()
        .into_iter()
        .find(|(n, _)| *n == "bolted_plate")
        .unwrap();
    let files = files();
    let mut program = files[0].1.clone();
    program.push(
        "middle",
        geop_ops_datums::AddDatumArgs {
            selection: vec![EntityRef::datum_component(
                geop_ops::ORIGIN,
                DatumComponent::Plane(FrameAxis::Y),
            )],
            construction: geop_ops_datums::Construction::Offset {
                distance: 20.0.into(),
            },
        },
    );
    let files = files
        .into_iter()
        .map(|(path, program)| (path.to_string(), program.to_json().unwrap()))
        .collect();
    let workspace = crate::Workspace::<S>::new(crate::stdlib::WithStandardParts(files));
    let part = program
        .build(&workspace.scope("bolted_plate.geop"))
        .unwrap();
    let plate = part
        .instance(part.instance_id("plate").unwrap())
        .unwrap()
        .part();
    let model = plate.topology();
    let hole = name_where(plate, |id| match id {
        RefId::Edge(e) => model.get_edge(e).unwrap().curve.as_arc().unwrap().is_some(),
        _ => false,
    });
    let args = DrawingArgs {
        views: vec![ViewKind::Top],
        section: Some(EntityRef::datum("middle")),
        dimensions: vec![Dimension::Diameter {
            edge: format!("plate/{hole}"),
        }],
        ..Default::default()
    };
    let sheet = compose(&part, &args, "", &[]).unwrap();
    let (mut rising, mut falling) = (0, 0);
    for stroke in sheet.strokes.iter().filter(|s| s.layer == Layer::Hatch) {
        if let Shape::Line(a, b) = stroke.shape {
            match (b[0] - a[0]) * (b[1] - a[1]) > 0.0 {
                true => rising += 1,
                false => falling += 1,
            }
        }
    }
    assert!(
        rising > 10 && falling > 10,
        "{rising} and {falling} hatch lines"
    );
    assert!(sheet.labels.iter().any(|l| l.text == "SECTION A-A"));
    assert!(
        sheet.labels.iter().any(|l| l.text == "⌀4.5"),
        "the hole's diameter"
    );
}

/// A plate with `n` by `n` M3x10 screws standing on it, 10 apart, and as
/// many M3 nuts hanging under it, each placed on its own: the program
/// `screwed.geop`, built.
fn screwed_plate(n: usize) -> Part<S> {
    use geop_ops::part::{ParamValue, State, pose_parameter};
    let mut program = Program::new();
    let mut place = |id: String, file: &str, size: Option<&str>, at: [f64; 3], turn: [f64; 3]| {
        let parameters = size
            .map(|row| State::from([("size".to_string(), ParamValue::Text(row.into()))]))
            .unwrap_or_default();
        program.push(
            &id,
            geop_ops_assembly::AddPartArgs {
                file: file.into(),
                parameters,
                ..Default::default()
            },
        );
        program.state.insert(
            pose_parameter(&id),
            ParamValue::Pose(examples::pose(at, turn)),
        );
    };
    place("plate".into(), "plate.geop", None, [0.0; 3], [0.0; 3]);
    for i in 0..n {
        for j in 0..n {
            let (x, y) = (5.0 + 10.0 * i as f64, 5.0 + 10.0 * j as f64);
            place(
                format!("screw{i}_{j}"),
                "std:iso4762_socket_head_cap_screw.geop",
                Some("M3x10"),
                [x, y, 5.0],
                [0.0; 3],
            );
            place(
                format!("nut{i}_{j}"),
                "std:iso4032_hex_nut.geop",
                Some("M3"),
                [x, y, 0.0],
                [180.0, 0.0, 0.0],
            );
        }
    }
    let files = std::collections::BTreeMap::from([(
        "plate.geop".to_string(),
        examples::metric_plate().to_json().unwrap(),
    )]);
    let workspace = crate::Workspace::<S>::new(crate::stdlib::WithStandardParts(files));
    program.build(&workspace.scope("screwed.geop")).unwrap()
}

/// The balloons of `sheet`: the item numbers on the dimension layer next
/// to a circle of a balloon's radius.
fn balloon_items(sheet: &geop_ops_drawing::sheet::Sheet) -> Vec<String> {
    use geop_ops_drawing::sheet::{Layer, Shape};
    let circles: Vec<[f64; 2]> = sheet
        .strokes
        .iter()
        .filter_map(|s| match s.shape {
            Shape::Circle { center, radius } if s.layer == Layer::Dimension && radius == 4.0 => {
                Some(center)
            }
            _ => None,
        })
        .collect();
    let mut items: Vec<String> = sheet
        .labels
        .iter()
        .filter(|l| {
            l.layer == Layer::Dimension
                && circles
                    .iter()
                    .any(|c| (c[0] - l.at[0]).abs() < 1e-9 && (c[1] - l.at[1] - 1.75).abs() < 1e-9)
        })
        .map(|l| l.text.clone())
        .collect();
    items.sort();
    items
}

/// A plate held by four screws and four nuts: three lines of the bill, so
/// three balloons — not one per part placed — and one view of each part
/// serves every copy of it.
#[test]
fn repeated_parts_are_ballooned_once() {
    use geop_ops_drawing::scene::Scene;
    let part = screwed_plate(2);
    let scene = Scene::of(&part).unwrap();
    assert_eq!(scene.bodies.len(), 9);
    assert_eq!(scene.groups().len(), 3, "the plate, the screw and the nut");
    let args = DrawingArgs {
        views: vec![ViewKind::Front, ViewKind::Top],
        bom: true,
        ..Default::default()
    };
    let parts = crate::inspect::parts_list(&part, "screwed.geop", &args).unwrap();
    assert_eq!(
        parts.iter().map(|l| l.quantity).collect::<Vec<_>>(),
        [1, 4, 4]
    );
    let sheet = compose(&part, &args, "", &parts).unwrap();
    assert_eq!(balloon_items(&sheet), ["1", "2", "3"]);
}

/// A hundred screws and a hundred nuts in a plate draw — each part's own
/// lines found once for all its copies.
#[test]
#[ignore = "slow: a hundred screws drawn — run with `cargo test -- --ignored`"]
fn a_hundred_screws_draw() {
    use geop_ops_drawing::scene::Scene;
    let part = screwed_plate(10);
    let scene = Scene::of(&part).unwrap();
    assert_eq!(scene.groups().len(), 3);
    let args = DrawingArgs {
        bom: true,
        sheet: geop_ops_drawing::SheetSize::A2,
        ..Default::default()
    };
    let parts = crate::inspect::parts_list(&part, "screwed.geop", &args).unwrap();
    let sheet = compose(&part, &args, "", &parts).unwrap();
    assert_eq!(balloon_items(&sheet), ["1", "2", "3"]);
}
