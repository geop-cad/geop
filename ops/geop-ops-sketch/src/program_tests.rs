//! Sketches run as a program builds them: placed on a plane of the part,
//! their dimensions following the part's parameters, what they project of
//! the part kept up to date, and each of their elements what it is to the
//! steps that build on them.

use std::{collections::BTreeMap, rc::Rc};

use geop_core_editor::{
    editing::{Pointer, Reach},
    numeric_input::NumericInput,
    selectable::selected,
};
use geop_core_math::{
    primitives::{FrameAxis, Ray},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{element::ElementReference, program::Program};
use geop_core_sketch::{Constraint, CurveId, PointId};
use geop_ops_datums::{define_datums, origin_plane};
use geop_ops_model::{Curve, Design, Line, NamedModel, Path, Plane, Point, model, testing::unit_cube};
use geop_ops_parameters::{Registry, Runner};
use geop_ops_view::{PartView, drawn};

use crate::{
    AddSketch, AddSketchArgs, PlacedSketch, Sketch, SketchPlane,
    placed::sketch_element,
    references::{Projected, Reference, Source},
    sketch, whole_sketch,
};

/// A runner of sketches, whose given step defines the datums and a unit
/// cube as the model.
fn runner() -> Runner {
    let mut runner = Runner::new(Registry::default().with(AddSketch).unwrap());
    runner.given = Some(Rc::new(|part| {
        define_datums(part)?;
        part.define_target(&model(), &(), |_, _| Ok(unit_cube()))
    }));
    runner
}

fn n(x: f64) -> Design {
    Design::from_f64(x)
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

/// Runs a program of the sketch step `s` with `args`, and the parameters
/// `inputs`.
fn run(runner: &mut Runner, args: &AddSketchArgs, inputs: &[(&str, f64)]) {
    let program = Program::new(vec![
        geop_core_program::program::ProgramStep::new::<AddSketch>("s", args).unwrap(),
    ]);
    let inputs: BTreeMap<String, Design> =
        (inputs.iter()).map(|&(name, value)| (name.to_string(), n(value))).collect();
    let output = runner.run(&program, &inputs, &BTreeMap::new());
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
    let mut runner = runner();
    run(&mut runner, &AddSketchArgs::on(origin_xy(), drawn_square), &[]);
    let reader = runner.part.current();

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
    assert_eq!(saved["formulas"][distance.0.to_string()]["formula"], "width / 2");
    assert_eq!(saved["plane"], serde_json::json!({"datum": "origin:xy"}));

    let mut runner = runner();
    let end = |runner: &Runner| {
        let reader = runner.part.current();
        reader.retrieve_element(&sketch_element::<Point>("s", b)).unwrap().to_array()[0].to_f64()
    };
    run(&mut runner, &args, &[("width", 6.0)]);
    assert!((end(&runner) - 3.0).abs() < 1e-9);
    run(&mut runner, &args, &[("width", 10.0)]);
    assert!((end(&runner) - 5.0).abs() < 1e-9);
}

/// A sketch on a face of the cube lies in its plane, and projects an edge
/// of the cube as a fixed line of construction geometry, named after the
/// edge.
#[test]
fn a_sketch_on_a_face_projects_an_edge() {
    let cube = unit_cube();
    let mut runner = runner();
    let edge = cube.name(cube.topology().edges.keys().next().copied().unwrap()).unwrap();
    let mut args = AddSketchArgs::on(
        SketchPlane::Face(ElementReference::new(model(), "f0")),
        Sketch::new(),
    );
    args.ensure_frame();
    args.references.push(Reference::new(Source::Projection {
        entity: Projected::Edge(ElementReference::new(model(), edge)),
    }));
    run(&mut runner, &args, &[]);
    let reader = runner.part.current();
    let placed: &PlacedSketch<Design> = reader.retrieve_element(&whole_sketch("s")).unwrap();
    let face = reader
        .retrieve_element(&ElementReference::<NamedModel<Design>, Plane>::new(model(), "f0"))
        .unwrap();
    assert_eq!(placed.plane, face);
    let projected = placed.sketch.curves.values().filter(|c| c.fixed && c.construction);
    // Its own two axes, and the edge — unless the face's normal runs along
    // it, when it projects to a point.
    assert!(projected.count() >= 2);
}

/// Drawn, a sketch's curve is picked as what a field asks for: a line, for
/// a field of lines, of the sketch's target.
#[test]
fn a_sketch_curve_is_picked_as_a_line() {
    let mut drawn_square = Sketch::new();
    let (_, sides) = square(&mut drawn_square);
    let mut runner = runner();
    run(&mut runner, &AddSketchArgs::on(origin_xy(), drawn_square), &[]);
    let reader = runner.part.current();
    let placed: &PlacedSketch<Design> = reader.retrieve_element(&whole_sketch("s")).unwrap();
    let mut view = PartView::new();
    placed.draw(sketch("s").name(), &mut view).unwrap();
    let view = view.finish();
    let v = |p: [f64; 3]| Vector3::from_array(p.map(n));
    let pointer = Pointer {
        ray: Ray::try_new(v([0.5, 0.0, 10.0]), v([0.0, 0.0, -1.0])).unwrap(),
        reach: Reach::Tube { radius: n(0.01) },
    };
    let hit = view.pick(&pointer, &[drawn::SKETCH_CURVE.to_string()]).unwrap();
    let lines: Vec<ElementReference<PlacedSketch<Design>, Line>> =
        selected(&reader, &[hit.element]);
    assert_eq!(lines, [sketch_element::<Line>("s", sides[0])]);
    // Not of the model, though both have lines.
    let edges: Vec<ElementReference<NamedModel<Design>, Line>> =
        selected(&reader, &[lines[0].picked()]);
    assert!(edges.is_empty());
}
