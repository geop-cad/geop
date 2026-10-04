use geop_core_math::{scalars::ScalInF64 as S, scalars::Scalar, vector::Vector3};
use geop_core_topology::FaceId;
use geop_ops::Part;
use geop_ops_extrude_revolve::shapes::{cube::cube_solid, cylinder::revolved_cylinder};

use crate::{LineKind, ProjectedView, ViewKind, ViewOptions, project_view};

fn v(x: f64, y: f64, z: f64) -> Vector3<S> {
    Vector3::from_array([S::from_f64(x), S::from_f64(y), S::from_f64(z)])
}

fn all_faces(part: &Part<S>) -> Vec<FaceId> {
    let model = part.topology();
    let mut faces: Vec<FaceId> = model.faces.keys().copied().collect();
    faces.sort_by_key(|f| f.0);
    faces
}

fn view(part: &Part<S>, kind: ViewKind, options: ViewOptions) -> ProjectedView<S> {
    project_view(
        part.topology(),
        &all_faces(part),
        &kind.frame().unwrap(),
        &options,
    )
    .unwrap()
}

fn count(view: &ProjectedView<S>, visible: bool) -> usize {
    view.lines.iter().filter(|l| l.visible == visible).count()
}

/// Each line as `(kind, visible, from, to)` on the paper, for a failure to
/// show.
fn summary(view: &ProjectedView<S>) -> Vec<String> {
    view.lines
        .iter()
        .map(|l| {
            let (a, b) = l.curve.domain();
            let (p, q) = (l.curve.evaluate(a).unwrap(), l.curve.evaluate(b).unwrap());
            format!(
                "{:?} {} ({:.3}, {:.3}) -> ({:.3}, {:.3})",
                l.kind,
                if l.visible { "visible" } else { "hidden" },
                p[0].to_f64(),
                p[1].to_f64(),
                q[0].to_f64(),
                q[1].to_f64()
            )
        })
        .collect()
}

fn assert_extents(view: &ProjectedView<S>, expected: [f64; 4]) {
    let e = view.extents().unwrap();
    for k in 0..4 {
        assert!(
            e[k].could_be_equal(S::from_f64(expected[k])),
            "extents {e:?}, expected {expected:?}"
        );
    }
}

/// A 2 x 1 x 3 box seen from the front is a 2 x 3 rectangle: four visible
/// lines, and the edges behind it, running along them, are not drawn.
#[test]
fn box_front_view_is_a_rectangle() {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "box", v(0.0, 0.0, 0.0), v(2.0, 1.0, 3.0)).unwrap();
    let front = view(&part, ViewKind::Front, ViewOptions::default());
    assert_eq!(count(&front, true), 4, "{:#?}", summary(&front));
    assert_eq!(count(&front, false), 0);
    assert_extents(&front, [0.0, 0.0, 2.0, 3.0]);

    let top = view(&part, ViewKind::Top, ViewOptions::default());
    assert_eq!(count(&top, true), 4);
    assert_extents(&top, [0.0, 0.0, 2.0, 1.0]);
    let right = view(&part, ViewKind::Right, ViewOptions::default());
    assert_eq!(count(&right, true), 4);
    assert_extents(&right, [0.0, 0.0, 1.0, 3.0]);

    // From a corner, three faces show: nine edges are visible, and the three
    // at the far corner hidden — cut where they pass behind visible ones.
    let iso = view(&part, ViewKind::Iso, ViewOptions::default());
    let edges = |visible: bool| {
        let mut edges: Vec<_> = iso
            .lines
            .iter()
            .filter(|l| l.visible == visible)
            .map(|l| l.edge.unwrap())
            .collect();
        edges.sort_by_key(|e| e.0);
        edges.dedup();
        edges
    };
    let (visible, hidden) = (edges(true), edges(false));
    assert_eq!(visible.len(), 9, "{:#?}", summary(&iso));
    assert_eq!(hidden.len(), 3, "{:#?}", summary(&iso));
    assert!(visible.iter().all(|e| !hidden.contains(e)));
}

/// A standing cylinder seen from the side shows its two silhouettes, and
/// its circles edge on as lines; from above, a circle and nothing else.
#[test]
fn cylinder_side_view_has_silhouettes() {
    let mut part = Part::<S>::new();
    revolved_cylinder(&mut part, "c", v(0.0, 0.0, 0.0), S::ONE, S::TWO).unwrap();
    let front = view(&part, ViewKind::Front, ViewOptions::default());
    let silhouettes = front
        .lines
        .iter()
        .filter(|l| l.kind == LineKind::Silhouette && l.visible)
        .count();
    assert_eq!(silhouettes, 2, "{:#?}", summary(&front));
    assert_eq!(count(&front, false), 0, "{:#?}", summary(&front));
    assert_extents(&front, [-1.0, 0.0, 1.0, 2.0]);

    let top = view(&part, ViewKind::Top, ViewOptions::default());
    assert!(
        top.lines
            .iter()
            .all(|l| l.kind == LineKind::Edge && l.visible)
    );
    assert_extents(&top, [-1.0, -1.0, 1.0, 1.0]);
}

/// A shaft of radius 3, threaded M6 from its top 8 down, drawn as drafting
/// draws an external thread: from the front two thin visible lines at the
/// minor diameter, the length of the thread; from above three quarters of
/// a thin circle at it; labelled once.
#[test]
fn a_threaded_shaft_draws_its_thread() {
    use crate::{
        DrawingArgs, compose,
        sheet::{Layer, Shape, Stroke},
    };
    use geop_core_geometry::{nurb_curve::Handedness, shape::Axis};
    use geop_ops::CosmeticThread;

    let mut part = Part::<S>::new();
    revolved_cylinder(
        &mut part,
        "shaft",
        v(0.0, 0.0, 0.0),
        S::from_f64(3.0),
        S::from_f64(10.0),
    )
    .unwrap();
    let minor = 4.917;
    part.add_thread(
        "thread",
        CosmeticThread {
            designation: "M6x1".into(),
            face: "the shaft's side".into(),
            axis: Axis::try_new(v(0.0, 0.0, 10.0), v(0.0, 0.0, -1.0)).unwrap(),
            radius: S::from_f64(3.0),
            major_diameter: 6.0,
            minor_diameter: minor,
            pitch: 1.0,
            length: 8.0,
            internal: false,
            handedness: Handedness::Right,
        },
    )
    .unwrap();
    let args = DrawingArgs {
        views: vec![ViewKind::Front, ViewKind::Top],
        scale: Some(1.0),
        ..Default::default()
    };
    let sheet = compose(&part, &args, "", &[]).unwrap();
    let thread = |s: &&Stroke| s.layer == Layer::Thread;
    let lines: Vec<(f64, f64)> = sheet
        .strokes
        .iter()
        .filter(thread)
        .filter_map(|s| match s.shape {
            Shape::Line(a, b) => Some((a[0], (a[1] - b[1]).abs())),
            _ => None,
        })
        .collect();
    assert_eq!(lines.len(), 2, "{lines:?}");
    assert!(lines.iter().all(|(_, length)| (length - 8.0).abs() < 1e-9));
    assert!(
        ((lines[0].0 - lines[1].0).abs() - minor).abs() < 1e-9,
        "{lines:?}"
    );
    let arcs: Vec<f64> = sheet
        .strokes
        .iter()
        .filter(thread)
        .filter_map(|s| match s.shape {
            Shape::Arc { radius, .. } => Some(radius),
            _ => None,
        })
        .collect();
    assert_eq!(arcs.len(), 1, "{arcs:?}");
    assert!((arcs[0] - minor / 2.0).abs() < 1e-9, "{arcs:?}");
    assert_eq!(sheet.labels.iter().filter(|l| l.text == "M6x1").count(), 1);
}

/// A bill of materials stands on the title block and pushes the views up
/// off it: no view line comes down into its rows. One too long for the
/// paper is refused, saying which paper.
#[test]
fn a_bill_of_materials_stands_on_the_title_block() {
    use crate::{DrawingArgs, PartsListLine, SheetSize, compose, sheet::Shape};
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "block", v(0.0, 0.0, 0.0), v(100.0, 100.0, 100.0)).unwrap();
    let line = |k: usize| PartsListLine {
        item: k.to_string(),
        quantity: 2,
        name: format!("part {k}"),
        designation: "ISO 4762 M4x12".into(),
        material: "Steel".into(),
        ..Default::default()
    };
    let lines: Vec<PartsListLine> = (1..=5).map(line).collect();
    let args = DrawingArgs {
        bom: true,
        sheet: SheetSize::A4,
        ..Default::default()
    };
    let sheet = compose(&part, &args, "", &lines).unwrap();
    for text in ["ITEM", "part 5", "ISO 4762 M4x12"] {
        assert!(sheet.labels.iter().any(|l| l.text == text), "{text}");
    }
    // Margin 10, title block 40, six rows of 6: the bill reaches y = 86.
    let lowest = sheet
        .strokes
        .iter()
        .filter(|s| s.layer == crate::sheet::Layer::Visible)
        .flat_map(|s| match &s.shape {
            Shape::Line(a, b) => vec![a[1], b[1]],
            Shape::Polyline(points) => points.iter().map(|p| p[1]).collect(),
            _ => Vec::new(),
        })
        .fold(f64::INFINITY, f64::min);
    assert!(lowest > 86.0, "a view comes down to {lowest}");

    let many: Vec<PartsListLine> = (1..=40).map(line).collect();
    let error = compose(&part, &args, "", &many).unwrap_err().to_string();
    assert!(
        error.contains("40 lines") && error.contains("A4"),
        "{error}"
    );
}

/// A block `size` on a side, as a component to place.
fn block(size: f64) -> std::sync::Arc<geop_ops::Component<S>> {
    let mut part = Part::<S>::new();
    cube_solid(&mut part, "block", v(0.0, 0.0, 0.0), v(size, size, size)).unwrap();
    std::sync::Arc::new(geop_ops::Component::new(
        "block.geop".into(),
        part,
        Default::default(),
    ))
}

/// Places `component` in `part` as `name`, at `at`, turned `degrees` about
/// `x`, `y` and `z`.
fn place(
    part: &mut Part<S>,
    component: &std::sync::Arc<geop_ops::Component<S>>,
    name: &str,
    at: Vector3<S>,
    degrees: [f64; 3],
) {
    let pose = geop_core_math::primitives::Pose::from_euler(at, degrees.map(S::from_f64)).unwrap();
    let instance = geop_ops::Instance {
        component: component.clone(),
        pose,
        parameter: None,
        fixed: true,
        flexible: false,
    };
    part.add_instance(instance, name).unwrap();
}

/// A hundred blocks of one part in a row, placed alike but for where: one
/// view of the block serves them all, and each draws its square from the
/// front, none hiding another.
#[test]
fn repeated_parts_share_their_view() {
    use crate::scene::Scene;
    let mut part = Part::<S>::new();
    let unit = block(1.0);
    for k in 0..100 {
        place(&mut part, &unit, &format!("b{k}"), v(2.0 * k as f64, 0.0, 0.0), [0.0; 3]);
    }
    let scene = Scene::of(&part).unwrap();
    assert_eq!(scene.bodies.len(), 100);
    assert_eq!(scene.groups().len(), 1, "one view of the block for all");
    let front = scene
        .project(&ViewKind::Front.frame().unwrap(), &ViewOptions::default())
        .unwrap();
    assert_eq!((count(&front, true), count(&front, false)), (400, 0));
    assert_extents(&front, [0.0, 0.0, 199.0, 1.0]);

    // Turned, a block is seen from another side: a view of its own.
    place(&mut part, &unit, "turned", v(0.0, 5.0, 0.0), [0.0, 0.0, 90.0]);
    assert_eq!(Scene::of(&part).unwrap().groups().len(), 2);

    // A column of blocks seen from its end: the one in front hides the
    // others whole, and each of their lines lies on one of its own. Only
    // it is drawn.
    let mut column = Part::<S>::new();
    for k in 0..10 {
        place(&mut column, &unit, &format!("b{k}"), v(0.0, 9.0 - 2.0 * k as f64, 0.0), [0.0; 3]);
    }
    let scene = Scene::of(&column).unwrap();
    let front = scene
        .project(&ViewKind::Front.frame().unwrap(), &ViewOptions::default())
        .unwrap();
    assert_eq!((count(&front, true), count(&front, false)), (4, 0));
    assert!(front.lines.iter().all(|l| scene.bodies[l.body].path == "b9"));
}

/// A block of side 2 in front of another, which stands 1 to its right and
/// 1 higher: from the front, the corner of the one behind that the front
/// one covers is hidden — two dashed sides inside the front square — and
/// its other sides are seen; the front one is seen whole.
#[test]
fn a_part_in_front_hides_one_behind() {
    use crate::scene::Scene;
    let mut part = Part::<S>::new();
    let big = block(2.0);
    place(&mut part, &big, "front", v(0.0, 0.0, 0.0), [0.0; 3]);
    place(&mut part, &big, "back", v(1.0, 5.0, 1.0), [0.0; 3]);
    let scene = Scene::of(&part).unwrap();
    let front = scene
        .project(&ViewKind::Front.frame().unwrap(), &ViewOptions::default())
        .unwrap();
    let of = |body: usize, visible: bool| {
        front
            .lines
            .iter()
            .filter(|l| l.body == body && l.visible == visible)
            .count()
    };
    assert_eq!(of(0, true), 4, "{:#?}", summary(&front));
    assert_eq!(of(0, false), 0, "{:#?}", summary(&front));
    // The block behind: its left and bottom sides cut where they pass
    // behind the front one, the halves inside hidden.
    assert_eq!(of(1, false), 2, "{:#?}", summary(&front));
    assert_eq!(of(1, true), 4, "{:#?}", summary(&front));
    for line in front.lines.iter().filter(|l| !l.visible) {
        let e = line.curve.domain();
        for t in [e.0, e.1] {
            let p = line.curve.evaluate(t).unwrap();
            assert!(
                (0..2).all(|k| p[k].to_f64() >= 1.0 - 1e-9 && p[k].to_f64() <= 2.0 + 1e-9),
                "a hidden line inside the front square: {:#?}",
                summary(&front)
            );
        }
    }
    let without = scene
        .project(
            &ViewKind::Front.frame().unwrap(),
            &ViewOptions {
                hidden_lines: false,
                ..Default::default()
            },
        )
        .unwrap();
    assert_eq!(count(&without, false), 0);
    assert_eq!(count(&without, true), 8);
}

/// An assembly's bill of materials balloons each line once, however often
/// its part is placed, around the view that shows them all, with the items
/// the caller numbered; a line of no part drawn gets none.
#[test]
fn an_assembly_balloons_each_line_once() {
    use crate::{
        DrawingArgs, PartsListLine, compose,
        sheet::{Layer, Shape},
    };
    let mut part = Part::<S>::new();
    let (big, small) = (block(4.0), block(1.0));
    place(&mut part, &big, "base", v(0.0, 0.0, 0.0), [0.0; 3]);
    let mut placements = Vec::new();
    for k in 0..4 {
        let name = format!("peg{k}");
        place(&mut part, &small, &name, v(k as f64, 1.0, 4.0), [0.0; 3]);
        placements.push(name);
    }
    let lines = vec![
        PartsListLine {
            item: "1".into(),
            quantity: 1,
            name: "base".into(),
            placements: vec!["base".into()],
            ..Default::default()
        },
        PartsListLine {
            item: "2".into(),
            quantity: 4,
            name: "peg".into(),
            placements,
            ..Default::default()
        },
        PartsListLine {
            item: "3".into(),
            quantity: 1,
            name: "wire".into(),
            ..Default::default()
        },
    ];
    let args = DrawingArgs {
        bom: true,
        ..Default::default()
    };
    let sheet = compose(&part, &args, "", &lines).unwrap();
    let balloons: Vec<[f64; 2]> = sheet
        .strokes
        .iter()
        .filter_map(|s| match s.shape {
            Shape::Circle { center, radius } if s.layer == Layer::Dimension && radius == 4.0 => {
                Some(center)
            }
            _ => None,
        })
        .collect();
    assert_eq!(balloons.len(), 2, "{balloons:?}");
    let distance = (balloons[0][0] - balloons[1][0]).hypot(balloons[0][1] - balloons[1][1]);
    assert!(distance >= 8.0, "the balloons overlap: {balloons:?}");
    for item in ["1", "2"] {
        let labelled = sheet
            .labels
            .iter()
            .filter(|l| l.layer == Layer::Dimension && l.text == item)
            .count();
        assert_eq!(labelled, 1, "balloon {item}");
    }
}
