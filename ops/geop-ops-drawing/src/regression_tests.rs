//! Programs from tutorials and bug reports, rebuilt in Rust: drawings of
//! parts whose features were made as a user makes them.

use geop_core_program::{part::Part, program::Program, runner::Runner};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output};
use geop_ops_model::sketch::{
    figures::{disc, on_face},
    whole_sketch,
};
use geop_ops_model::{
    Combine, RefId,
    testing::{middle, model_of},
};
use geop_ops_sketch::AddSketchArgs;

use crate::{Annotation, DrawingArgs, DrawnView, ViewKind, compose, program_tests::registry};

/// `program` built on its own, in the operations of [`registry`].
fn build(program: &Program) -> Part {
    let runner = Runner::new(registry().operations());
    let mut part = Part::default();
    program
        .run(&runner, &mut part)
        .unwrap_or_else(|e| panic!("{e}"));
    part
}

/// A 40 x 40 x 10 plate with a hole of diameter 6 through its middle, made
/// as the Onshape and SolidWorks part tutorials make a hole: a circle
/// sketched on the top face and extruded "through all" as a cut. Then its
/// drawing, the bottom view with the hole's diameter dimensioned.
///
/// The dimension is refused: "the edge combine(hole,...) is not circular:
/// only a circle has a radius". The hole's rims are the cylinder's
/// intersections with the plate's faces, which the boolean traced as
/// fitted curves instead of keeping the tool's circles, so the drawing
/// cannot see a circle to dimension (nor does the DXF write one). The
/// same plate drilled by the hole operation, or cut by a disc that ends
/// flush with the faces, has rims that are arcs and dimensions as "⌀6".
#[test]
#[ignore = "bug: the rims of a hole cut by extruding a circle through all are traced curves, not the cylinder's circles, so a diameter dimension of the hole in a drawing is refused (not circular); the hole operation's rims are arcs"]
fn diameter_of_a_hole_extruded_through_all() {
    let mut program = Program::new();
    program.push(
        "plate",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [40.0, 40.0, 10.0]),
    );
    program.push(
        "hole_sketch",
        AddSketchArgs::on(on_face("block(plate,end)"), disc([20.0, 20.0], 3.0)),
    );
    program.push(
        "hole",
        ExtrudeArgs {
            sketch: whole_sketch("hole_sketch"),
            extent: Extents {
                side1: Extent::ThroughAll,
                reversed: true,
                ..Extents::blind(0.0)
            },
            output: Output::Solid(Combine::cut("block(plate)")),
            draft: 0.0,
        },
    );
    let built = build(&program);
    let part = model_of(&built);
    let model = part.topology();
    let mut rims: Vec<String> = (part.names().iter())
        .filter(|(id, _)| matches!(id, RefId::Edge(_)))
        .map(|(_, name)| name.to_string())
        .filter(|name| {
            let m = middle(model.get_edge(part.edge_id(name).unwrap()).unwrap());
            let (dx, dy) = (m[0].to_f64() - 20.0, m[1].to_f64() - 20.0);
            m[2].to_f64().abs() < 1e-6 && (dx * dx + dy * dy).sqrt() < 4.0
        })
        .collect();
    rims.sort();
    assert!(!rims.is_empty(), "no rim on the top");
    let args = DrawingArgs {
        views: vec![ViewKind::Bottom, ViewKind::Front],
        annotations: vec![Annotation::Radius {
            view: DrawnView::View(ViewKind::Bottom),
            edge: rims[0].clone(),
            diameter: true,
            label: [10.0, 10.0],
        }],
        ..Default::default()
    };
    let sheet = compose(&built.current(), &args, "", &[]).unwrap_or_else(|e| panic!("{e}"));
    let labels: Vec<&str> = sheet.labels.iter().map(|l| l.text.as_str()).collect();
    assert!(labels.contains(&"⌀6"), "{labels:?}");
}
