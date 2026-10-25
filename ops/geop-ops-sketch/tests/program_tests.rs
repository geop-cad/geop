//! Sketches run as a program builds them: placed on a plane of the part,
//! their dimensions following the part's parameters, what they project of
//! the part kept up to date, and each of their elements what it is to the
//! steps that build on them — and sketches drawn on parts extruded from
//! sketches, and extruded themselves.
//!
//! An integration test: the extrusions come from
//! `geop-ops-extrude-revolve`, which depends on this crate, and a unit test
//! would hold a second copy of its targets that the extrusion cannot read.

use geop_core_program::{
    element::ElementReference,
    numeric::NumericInput,
    operation::Operation,
    operation_registry::OperationRegistry,
    part::{Part, StepContext},
    program::{Program, ProgramStep},
    runner::Runner,
    values::{Design, Setting, Settings},
};

use geop_core_editor::view::{PartView, drawn};
use geop_core_editor::{
    editing::{Pointer, Reach},
    selectable::selected,
};
use geop_core_math::{
    geop_error::GeopResult,
    primitives::{FrameAxis, Ray},
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_sketch::{Constraint, CurveId, PointId};
use geop_ops_datums::origin_plane;
use geop_ops_extrude_revolve::{
    examples::parametric_plate,
    testing::{drilled_box, registry},
};
use geop_ops_model::{
    Curve, Line, NamedModel, Path, Plane, Point, element, model,
    testing::{Built, unit_cube},
};
use geop_ops_parameters::{
    DefineParameters, DefineParametersArgs, NumberParameter, ParameterKind, parameters_of,
    set_parameters,
};
use geop_ops_sketch::{
    AddSketch, AddSketchArgs, CurveKind, PlacedSketch, Sketch, SketchPlane,
    figures::{n, on_face, on_origin},
    placed::sketch_element,
    references::{Projected, Reference, Source},
    sketch, whole_sketch,
};

/// Defines the model as a unit cube: what the sketches are drawn on.
struct Cube;

impl Operation for Cube {
    const KIND: &'static str = "cube";
    type Args = ();

    fn run(ctx: &mut StepContext, _: &()) -> GeopResult<()> {
        ctx.produce(&model(), &(), |_, _| Ok(unit_cube()))
    }
}

/// A runner of sketches, of parameters and of the unit cube.
fn runner() -> Runner {
    let registry = (OperationRegistry::default().with(AddSketch))
        .and_then(|r| r.with(DefineParameters))
        .and_then(|r| r.with(Cube))
        .unwrap();
    Runner::new(registry)
}

/// A closed square of side 1 from the origin: its points, its sides.
fn square(sketch: &mut Sketch) -> (Vec<PointId>, Vec<CurveId>) {
    let points: Vec<_> = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]
        .iter()
        .map(|&[x, y]| sketch.add_point(n(x), n(y)))
        .collect();
    let sides = (0..4)
        .map(|i| sketch.add_line(points[i], points[(i + 1) % 4]))
        .collect();
    (points, sides)
}

/// Runs a program of the sketch step `s` with `args` on the unit cube, its
/// parameters `inputs` — each a number, 1 unless the run sets it — on
/// `part`.
fn run(part: &mut Part, args: &AddSketchArgs, inputs: &[(&str, f64)]) {
    let parameters = DefineParametersArgs {
        values: (inputs.iter())
            .map(|&(name, _)| NumberParameter::new(name, "1", 0.0, 10.0))
            .collect(),
    };
    let program = Program::from_steps(vec![
        ProgramStep::new::<DefineParameters>("parameters", &parameters).unwrap(),
        ProgramStep::new::<Cube>("cube", &()).unwrap(),
        ProgramStep::new::<AddSketch>("s", args).unwrap(),
    ]);
    let inputs: Settings = (inputs.iter())
        .map(|&(name, value)| (name.to_string(), Setting::Number(value)))
        .collect();
    let output = runner().run_with(part, &program, &inputs, None);
    assert!(output.step_errors.is_empty(), "{:?}", output.step_errors);
}

fn origin_xy() -> SketchPlane {
    SketchPlane::Datum(origin_plane(FrameAxis::Z))
}

/// A square on the origin's `xy` plane is a target of the part: whole, a
/// path of four curves named after the sketch, its plane the origin's,
/// its sides lines and its corners points where the plane puts them.
#[test]
fn a_sketch_is_its_elements() {
    let mut drawn_square = Sketch::new();
    let (points, sides) = square(&mut drawn_square);
    let mut part = Part::default();
    run(
        &mut part,
        &AddSketchArgs::on(origin_xy(), drawn_square),
        &[],
    );
    let reader = part.current();

    let placed: &PlacedSketch<Design> = reader.retrieve_element(&whole_sketch("s")).unwrap();
    assert_eq!(placed.name, "s");
    let path = reader
        .retrieve_element(&ElementReference::<_, Path>::whole(sketch("s")))
        .unwrap();
    assert_eq!(path.curves.len(), 4);
    assert!(path.is_closed());
    assert!(path.curve_names.iter().all(|name| name.starts_with("s,c")));
    let plane = reader
        .retrieve_element(&ElementReference::<_, Plane>::whole(sketch("s")))
        .unwrap();
    assert!(plane.w().to_array()[2].could_be_equal(Design::ONE));
    let side = reader
        .retrieve_element(&sketch_element::<Line>("s", sides[0]))
        .unwrap();
    assert!(side.direction.to_array()[1].could_be_equal(Design::ZERO));
    let corner = reader
        .retrieve_element(&sketch_element::<Point>("s", points[2]))
        .unwrap();
    assert!(corner.could_be_equal(&Vector3::from_array([n(1.0), n(1.0), n(0.0)])));
    let curve = reader
        .retrieve_element(&sketch_element::<Curve>("s", sides[1]))
        .unwrap();
    assert_eq!(curve.curve_names, [format!("s,{}", sides[1])]);
    assert!(
        reader
            .retrieve_element(&sketch_element::<Line>("s", points[0]))
            .is_err(),
        "a point is no line"
    );
}

/// A dimension given by a formula follows the parameter it reads, and is
/// saved as the formula.
#[test]
fn a_dimension_follows_its_formula() {
    let mut drawn = Sketch::new();
    let a = drawn.add_point(n(0.0), n(0.0));
    let b = drawn.add_point(n(1.0), n(0.0));
    let line = drawn.add_line(a, b);
    drawn.constrain(Constraint::Fix {
        point: a,
        x: n(0.0),
        y: n(0.0),
    });
    drawn.constrain(Constraint::Horizontal { line });
    let distance = drawn.constrain(Constraint::Distance {
        a,
        b,
        value: n(1.0),
    });
    let mut args = AddSketchArgs::on(origin_xy(), drawn);
    (args.formulas).insert(distance, NumericInput::from("width / 2"));
    let saved = serde_json::to_value(&args).unwrap();
    assert_eq!(
        saved["formulas"][distance.0.to_string()]["formula"],
        "width / 2"
    );
    assert_eq!(saved["plane"], serde_json::json!({"datum": "origin:xy"}));

    let mut part = Part::default();
    let end = |part: &Part| {
        let reader = part.current();
        reader
            .retrieve_element(&sketch_element::<Point>("s", b))
            .unwrap()
            .to_array()[0]
            .to_f64()
    };
    run(&mut part, &args, &[("width", 6.0)]);
    assert!((end(&part) - 3.0).abs() < 1e-9);
    run(&mut part, &args, &[("width", 10.0)]);
    assert!((end(&part) - 5.0).abs() < 1e-9);
}

/// A sketch on a face of the cube lies in its plane, and projects an edge
/// of the cube as a fixed line of construction geometry, named after the
/// edge.
#[test]
fn a_sketch_on_a_face_projects_an_edge() {
    let cube = unit_cube();
    let mut part = Part::default();
    let edge = cube
        .name(cube.topology().edges.keys().next().copied().unwrap())
        .unwrap();
    let mut args = AddSketchArgs::on(
        SketchPlane::Face(ElementReference::new(model(), "f0")),
        Sketch::new(),
    );
    args.ensure_frame();
    args.references.push(Reference::new(Source::Projection {
        entity: Projected::Edge(ElementReference::new(model(), edge)),
    }));
    run(&mut part, &args, &[]);
    let reader = part.current();
    let placed: &PlacedSketch<Design> = reader.retrieve_element(&whole_sketch("s")).unwrap();
    let face = reader
        .retrieve_element(&ElementReference::<NamedModel<Design>, Plane>::new(
            model(),
            "f0",
        ))
        .unwrap();
    assert_eq!(placed.plane, face);
    let projected = placed
        .sketch
        .curves
        .values()
        .filter(|c| c.fixed && c.construction);
    // Its own two axes, and the edge — unless the face's normal runs along
    // it, when it projects to a point.
    assert!(projected.count() >= 2);
}

/// A projection follows what it projects: change the parameter the box's
/// width is a formula of, and the top's edges projected into a sketch
/// built after it move with it — the sketch's ids for them unchanged.
#[test]
fn projections_follow_the_part() {
    let mut program = parametric_plate();
    let mut on_floor = AddSketchArgs::new(Some(on_origin(FrameAxis::Z)));
    // What it projects is put into the sketch when it is built.
    on_floor.references.push(Reference::new(Source::Projection {
        entity: Projected::Face(element("extrude(plate,end)")),
    }));
    // Right after the plate: the hole renames the faces it cuts.
    let mut floor = Program::new();
    floor.push("floor", on_floor);
    let after_plate = program.index_of("plate").unwrap() + 1;
    program.steps.insert(after_plate, floor.steps.remove(0));

    let widest = |program: &Program| {
        let part = program.built(registry).unwrap();
        let reader = part.current();
        let placed: &PlacedSketch<Design> =
            reader.retrieve_element(&whole_sketch("floor")).unwrap();
        let ids: Vec<_> = placed.sketch.points.keys().copied().collect();
        let x = (placed.sketch.points.values())
            .map(|p| p.x.to_f64())
            .fold(f64::MIN, f64::max);
        (x, ids)
    };
    let (before, ids) = widest(&program);
    assert!((before - 4.0).abs() < 1e-9, "{before}");
    let mut parameters = parameters_of(&program);
    let ParameterKind::Number { expression, .. } = &mut parameters.values[0].kind else {
        panic!("width is a number")
    };
    *expression = "5.5".into();
    set_parameters(&mut program, parameters);
    let (after, ids_after) = widest(&program);
    assert!((after - 5.5).abs() < 1e-9, "{after}");
    assert_eq!(ids, ids_after);
}

/// Drawn, a sketch's curve is picked as what a field asks for: a line, for
/// a field of lines, of the sketch's target.
#[test]
fn a_sketch_curve_is_picked_as_a_line() {
    let mut drawn_square = Sketch::new();
    let (_, sides) = square(&mut drawn_square);
    let mut part = Part::default();
    run(
        &mut part,
        &AddSketchArgs::on(origin_xy(), drawn_square),
        &[],
    );
    let reader = part.current();
    let placed: &PlacedSketch<Design> = reader.retrieve_element(&whole_sketch("s")).unwrap();
    let mut view = PartView::new();
    placed.draw(sketch("s").name(), &mut view).unwrap();
    let view = view.finish();
    let v = |p: [f64; 3]| Vector3::from_array(p.map(n));
    let pointer = Pointer {
        ray: Ray::try_new(v([0.5, 0.0, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
        reach: Reach::Tube { radius: n(0.01) },
    };
    let hit = view
        .pick(&pointer, &[drawn::SKETCH_CURVE.to_string()])
        .unwrap();
    let lines: Vec<ElementReference<PlacedSketch<Design>, Line>> =
        selected(&reader, &[hit.element]);
    assert_eq!(lines, [sketch_element::<Line>("s", sides[0])]);
    // Not of the model, though both have lines.
    let edges: Vec<ElementReference<NamedModel<Design>, Line>> =
        selected(&reader, &[lines[0].picked()]);
    assert!(edges.is_empty());
}

/// The top face of the box of [`drilled_box`].
fn box_top() -> Projected {
    Projected::Face(element("extrude(box,end)"))
}

/// `entity` of `part` projected into a new sketch on `plane`: the sketch,
/// and the reference that gave it.
fn projected(part: &Part, plane: &SketchPlane, entity: Projected) -> (Sketch, Reference) {
    let reader = part.current();
    let frame = plane.resolve(&reader).unwrap();
    let mut sketch = Sketch::new();
    let mut reference = Reference::new(Source::Projection { entity });
    reference.update(&mut sketch, &reader, &frame).unwrap();
    sketch.validate().unwrap();
    (sketch, reference)
}

/// What kinds of curve a sketch holds, sorted.
fn kinds(sketch: &Sketch) -> Vec<&'static str> {
    let mut kinds: Vec<_> = sketch
        .curves
        .values()
        .map(|c| match c.kind {
            CurveKind::Line { .. } => "line",
            CurveKind::Arc { .. } => "arc",
            CurveKind::Circle { .. } => "circle",
            CurveKind::Spline { .. } => "spline",
        })
        .collect();
    kinds.sort();
    kinds
}

/// Projecting reads what an edge is from its NURBS: the box's top, seen
/// from above, is four lines around the hole's circle — as lines and a
/// circle, to constrain against, every one of them fixed. Seen from the
/// front, the hole's edge is no circle any more but the spline it
/// projects to, and the top's edges along the view collapse to points.
#[test]
fn projections_read_lines_arcs_and_splines() {
    let part = drilled_box().built(registry).unwrap();
    let top = box_top();
    let (sketch, reference) = projected(&part, &on_origin(FrameAxis::Z), top.clone());
    let round = kinds(&sketch)
        .iter()
        .filter(|k| **k == "arc" || **k == "circle")
        .count();
    assert_eq!(kinds(&sketch).iter().filter(|k| **k == "line").count(), 4);
    assert!(round >= 1, "{:?}", kinds(&sketch));
    assert!(!kinds(&sketch).contains(&"spline"));
    assert!(sketch.points.values().all(|p| p.fixed));
    // Reference geometry: fixed, and construction — never a profile.
    assert!(sketch.curves.values().all(|c| c.fixed && c.construction));
    assert!(sketch.regions().map_or(true, |r| r.is_empty()));
    // The hole's circle: around (1, 1), radius 0.4.
    for c in sketch.curves.values() {
        if let CurveKind::Circle { center, radius } = c.kind {
            let at = sketch.points[&center].xy();
            assert!((at[0].to_f64() - 1.0).abs() < 1e-9 && (at[1].to_f64() - 1.0).abs() < 1e-9);
            assert!((radius.to_f64() - 0.4).abs() < 1e-9);
        }
    }
    // Every curve is keyed by the name of the edge it comes from.
    assert!(reference.curves.keys().all(|k| k.starts_with("extrude(")));

    // Seen from the front the top is edge-on: its edges along the view are
    // points, the others lines, and the hole's circle the line it collapses
    // to — no spline lying flat along one.
    let (front, _) = projected(&part, &on_origin(FrameAxis::Y), top);
    assert!(front.validate().is_ok());
    let kinds = kinds(&front);
    assert!(
        !kinds.is_empty() && kinds.iter().all(|k| *k == "line"),
        "{kinds:?}"
    );
}

/// The box's top, with its hole, projected onto a plane seen from the side
/// — its hole's circle edge-on — and a rectangle drawn on its projected
/// corners: the sketch's regions are found — "could not tell which way it
/// winds" was a projected circle left as a spline lying flat along a line
/// — and only what was drawn bounds one.
#[test]
fn drawing_on_an_edge_on_projection() {
    let part = drilled_box().built(registry).unwrap();
    let (mut sketch, reference) = projected(&part, &on_origin(FrameAxis::Y), box_top());
    // Down from the top's two projected corners, and across.
    let corners: Vec<_> = reference.points.values().copied().collect();
    let xs: Vec<f64> = corners
        .iter()
        .map(|p| sketch.points[p].x.to_f64())
        .collect();
    let (lo, hi) = (
        corners[xs
            .iter()
            .position(|&x| x == xs.iter().cloned().fold(f64::MAX, f64::min))
            .unwrap()],
        corners[xs
            .iter()
            .position(|&x| x == xs.iter().cloned().fold(f64::MIN, f64::max))
            .unwrap()],
    );
    let below = |p: geop_core_sketch::PointId, s: &mut Sketch| {
        let q = s.points[&p];
        s.add_point(q.x, S::from_f64(q.y.to_f64() - 1.0))
    };
    let (lo2, hi2) = (below(lo, &mut sketch), below(hi, &mut sketch));
    for (a, b) in [(lo, hi), (hi, hi2), (hi2, lo2), (lo2, lo)] {
        sketch.add_line(a, b);
    }
    sketch.validate().unwrap();
    let regions = sketch.regions().unwrap();
    assert_eq!(regions.len(), 1);
    assert_eq!(regions[0].outer.edges.len(), 4);
}

/// Sketches made with the sketch tools — patterns, mirror, offset — extrude
/// into valid solids.
mod tools {
    use std::f64::consts::PI;

    use geop_core_sketch::{copies::Step, offset::Corners};
    use geop_ops_extrude_revolve::{Extents, testing::extrusion};
    use geop_ops_model::Combine;
    use geop_ops_sketch::Constraint;

    use geop_ops_model::testing::check_valid;
    use geop_ops_sketch::figures::{circle, rectangle, solved};

    use super::*;

    /// `sketch`, solved, on the origin's `xy` plane, extruded up by
    /// `height`: the part, checked to be valid.
    fn extruded(sketch: Sketch, height: f64) -> geop_ops_model::NamedModel<S> {
        let mut program = Program::new();
        program.push(
            "outline",
            AddSketchArgs::on(on_origin(FrameAxis::Z), solved(sketch)),
        );
        program.push(
            "body",
            extrusion("outline", Extents::blind(height), Combine::NewBody),
        );
        let part = program.build_model(registry).unwrap();
        if let Err(e) = check_valid(&part) {
            panic!("{e}");
        }
        assert_eq!(part.solid_names().len(), 1);
        part
    }

    /// A 4 x 3 plate with a bolt circle of six holes round its middle and a
    /// row of three along its bottom: each a pattern of one hole.
    #[test]
    fn a_plate_with_patterned_holes_extrudes() {
        let mut sketch = Sketch::new();
        let sides = rectangle(&mut sketch, [0.0, 0.0], 4.0, 3.0);
        let middle = sketch.add_point(n(2.0), n(1.7));
        sketch.constrain(Constraint::Fix {
            point: middle,
            x: n(2.0),
            y: n(1.7),
        });
        let bolt = circle(&mut sketch, [2.8, 1.7], 0.15);
        sketch
            .pattern(
                &[bolt],
                &Step::Round {
                    center: middle,
                    angle: n(PI / 3.0),
                },
                6,
            )
            .unwrap();
        let first = circle(&mut sketch, [0.8, 0.4], 0.1);
        sketch
            .pattern(
                &[first],
                &Step::Along {
                    along: sides[0],
                    backwards: false,
                    spacing: n(1.2),
                },
                3,
            )
            .unwrap();
        let part = extruded(sketch, 0.4);
        // The plate's six faces and a wall for each of the nine holes, at
        // least: none of them merged or lost.
        assert!(
            part.topology().faces.len() >= 15,
            "{}",
            part.topology().faces.len()
        );
    }

    /// Half a slot right of the `y` axis, its ends on the axis, mirrored
    /// into the whole slot, and offset outwards into a ring round it.
    #[test]
    fn a_mirrored_slot_offset_into_a_ring_extrudes() {
        let mut sketch = Sketch::new();
        let o = sketch.add_fixed_point(n(0.0), n(0.0));
        let up = sketch.add_fixed_point(n(0.0), n(1.0));
        let axis = sketch.add_line(o, up);
        sketch.set_construction(axis, true);
        let p = [[0.02, 0.0], [1.0, 0.03], [1.0, 1.0], [0.0, 1.02]]
            .map(|q| sketch.add_point(n(q[0]), n(q[1])));
        let bottom = sketch.add_line(p[0], p[1]);
        let round = sketch.add_arc(p[1], p[2], n(PI));
        let top = sketch.add_line(p[2], p[3]);
        for (a, b) in [(bottom, round), (round, top)] {
            sketch.constrain(Constraint::Tangent { a, b });
        }
        for point in [p[0], p[3]] {
            sketch.constrain(Constraint::PointOnCurve { point, curve: axis });
        }
        sketch.constrain(Constraint::Fix {
            point: p[0],
            x: n(0.0),
            y: n(0.0),
        });
        sketch.constrain(Constraint::Horizontal { line: bottom });
        sketch.constrain(Constraint::Horizontal { line: top });
        sketch.constrain(Constraint::Length {
            curve: bottom,
            value: n(1.0),
        });
        sketch.constrain(Constraint::Radius {
            curve: round,
            value: n(0.5),
        });
        let mirrored = sketch.mirror(&[bottom, round, top], axis).unwrap();
        let mut slot = vec![bottom, round, top];
        slot.extend(mirrored);
        let report = sketch.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        let chain = sketch.chain(&slot).unwrap();
        assert!(chain.closed);
        // Outwards: the slot's bottom runs to the right, so to its right.
        let ring = sketch
            .offset(&slot, Design::from_f64(-0.2), Corners::Round)
            .unwrap();
        assert_eq!(ring.curves.len(), 6, "a curve round each of the slot's");
        let part = extruded(sketch, 0.3);
        let z = |k: usize| {
            part.topology()
                .vertices
                .values()
                .map(|v| v.point[k].to_f64())
                .fold(f64::MIN, f64::max)
        };
        assert!(
            (z(0) - 1.7).abs() < 1e-9 && (z(1) - 1.2).abs() < 1e-9,
            "{} {}",
            z(0),
            z(1)
        );
    }

    /// The box's top projected into a sketch on it, and its outline offset
    /// 0.1 in: the offset is the sketch's one region — the projection is
    /// construction — fully constrained by the part, and it follows the
    /// part when the program is built, a pad on the box.
    #[test]
    fn projected_edges_offset_into_a_profile() {
        let mut program = drilled_box();
        let built = program.built(registry).unwrap();
        let plane = on_face("extrude(box,end)");
        let (mut sketch, reference) = projected(&built, &plane, box_top());
        let line = *sketch
            .curves
            .iter()
            .find(|(_, c)| matches!(c.kind, CurveKind::Line { .. }))
            .unwrap()
            .0;
        let outline = sketch.chain_through(line).unwrap();
        assert_eq!(outline.len(), 4);
        let chain = sketch.chain(&outline).unwrap();
        // Inwards: towards the outline's middle, wherever the face's own
        // frame puts it.
        let corners: Vec<[f64; 2]> = outline
            .iter()
            .flat_map(|c| sketch.curves[c].points())
            .map(|p| geop_core_sketch::plain::xy(&sketch, p))
            .collect();
        let middle =
            [0, 1].map(|k| corners.iter().map(|q| q[k]).sum::<f64>() / corners.len() as f64);
        let inside = sketch.chain_side(&chain, middle).unwrap();
        let toward = Design::from_f64(0.1 * inside.signum());
        sketch.offset(&outline, toward, Corners::Round).unwrap();
        let report = sketch.solve().unwrap();
        assert!(report.converged && report.dof == 0, "{report:?}");
        assert_eq!(sketch.regions().unwrap().len(), 1);
        program.push(
            "pad_sketch",
            AddSketchArgs {
                plane: Some(plane),
                sketch,
                references: vec![reference],
                ..Default::default()
            },
        );
        program.push(
            "pad",
            extrusion("pad_sketch", Extents::blind(0.2), Combine::NewBody),
        );
        let part = program.build_model(registry).unwrap();
        if let Err(e) = check_valid(&part) {
            panic!("{e}");
        }
        let tallest = part
            .topology()
            .vertices
            .values()
            .map(|v| v.point[2].to_f64())
            .fold(f64::MIN, f64::max);
        assert!((tallest - 1.2).abs() < 1e-9, "{tallest}");
    }
}

/// A sketch projects another sketch's curve as it projects an edge: a
/// circle on the floor seen from above is that circle, keyed by the sketch
/// and the curve.
#[test]
fn sketch_curves_are_projected() {
    let mut program = Program::new();
    let mut floor = Sketch::new();
    let c = floor.add_point(n(1.0), n(1.0));
    let circle = floor.add_circle(c, n(0.5));
    program.push("floor", AddSketchArgs::on(on_origin(FrameAxis::Z), floor));
    let part = program.built(registry).unwrap();
    let curve = Projected::SketchCurve(sketch_element("floor", circle));
    let (sketch, reference) = projected(&part, &on_origin(FrameAxis::Z), curve);
    assert_eq!(kinds(&sketch), ["circle"]);
    assert!(
        reference
            .curves
            .keys()
            .all(|k| k == &format!("floor,{circle}"))
    );
}

/// A sketch projects a 3-D sketch's curve as it projects an edge — it is
/// one, of the wire the 3-D sketch is built as: a line rising across the
/// view is the line beneath it, keyed by its edge's name.
#[test]
fn sketch3d_curves_are_projected() {
    let mut program = Program::new();
    let mut route = geop_ops_sketch3d::Sketch3d::new();
    let at = |p: [f64; 3]| geop_core_math::vector::Vector3::from_array(p.map(n));
    let (a, b) = (
        route.add_point(at([3.0, 0.0, 1.0])),
        route.add_point(at([4.0, 2.0, 3.0])),
    );
    route.add_line(a, b);
    program.push(
        "route",
        geop_ops_sketch3d::AddSketch3dArgs {
            sketch: route,
            references: Vec::new(),
        },
    );
    let with_3d = || registry()?.with(geop_ops_sketch3d::AddSketch3d);
    let part = program.built(with_3d).unwrap();
    let edge = Projected::Edge(element("sketch3d(route,c2)"));
    let (sketch, reference) = projected(&part, &on_origin(FrameAxis::Z), edge);
    assert_eq!(kinds(&sketch), ["line"]);
    assert_eq!(
        reference.curves.keys().collect::<Vec<_>>(),
        ["sketch3d(route,c2)"]
    );
    let mut ends: Vec<[f64; 2]> = sketch
        .points
        .values()
        .map(|p| [p.x.to_f64(), p.y.to_f64()])
        .collect();
    ends.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let near = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9;
    assert!(
        near(ends[0], [3.0, 0.0]) && near(ends[1], [4.0, 2.0]),
        "{ends:?}"
    );
}
