//! What the tests of programs share that start from sketches: the
//! operations they are written in, drawing outlines, the steps sketching
//! and extruding them, and building a program as an editor does, and
//! reading its steps' arguments as their types. Only built for tests: with
//! `testing`, or in this crate's own.

// Helpers of tests fail as tests do: by panicking, naming what is wrong.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use geop_core_math::{
    geop_error::{GeopError, GeopResult},
    primitives::FrameAxis,
    scalars::Scalar,
};
use geop_core_program::{
    element::ElementReference,
    operation_registry::OperationRegistry,
    part::Part,
    program::{Program, ProgramStep},
    runner::Runner,
    values::Design,
};
use geop_core_sketch::{CurveId, PointId};
use geop_ops_booleans::Combine;
use geop_ops_datums::origin_plane;
use geop_ops_model::{NamedModel, model, read_model};
use geop_ops_parameters::{DefineParameters, PartProperties};
use geop_ops_sketch::{AddSketch, AddSketchArgs, Sketch, SketchPlane, whole_sketch};
use serde::{Serialize, de::DeserializeOwned};

use crate::{Extent, Extents, Extrude, ExtrudeArgs, Loft, Output, Revolve, Sweep};

/// Sketching, and the solids and sheets made from sketches: the operations
/// a test's programs are written in, besides those it tests.
pub fn registry() -> GeopResult<OperationRegistry> {
    (OperationRegistry::default()
        .with(DefineParameters)?
        .with(PartProperties)?
        .with(AddSketch)?
        .with(Extrude)?
        .with(Revolve)?)
    .with(Sweep)?
    .with(Loft)
}

/// A number of a test, as design data.
pub fn n(x: f64) -> Design {
    Design::from_f64(x)
}

/// A closed polygon through `corners`, one line per side: its points and
/// lines.
pub fn polygon(sketch: &mut Sketch, corners: &[[f64; 2]]) -> (Vec<PointId>, Vec<CurveId>) {
    let points: Vec<PointId> = corners
        .iter()
        .map(|c| sketch.add_point(n(c[0]), n(c[1])))
        .collect();
    let lines = (0..points.len())
        .map(|i| sketch.add_line(points[i], points[(i + 1) % points.len()]))
        .collect();
    (points, lines)
}

/// A sketch of nothing but the closed polygon through `corners` (see
/// [`polygon`]).
pub fn outline(corners: &[[f64; 2]]) -> Sketch {
    let mut sketch = Sketch::new();
    polygon(&mut sketch, corners);
    sketch
}

/// The rectangle from `(x0, y0)` to `(x1, y1)`.
pub fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Sketch {
    outline(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
}

/// A sketch of nothing but the circle of `radius` around `center`.
pub fn disc(center: [f64; 2], radius: f64) -> Sketch {
    let mut sketch = Sketch::new();
    let c = sketch.add_point(n(center[0]), n(center[1]));
    sketch.add_circle(c, n(radius));
    sketch
}

/// The plane of the origin normal to `normal`, to sketch on.
pub fn on_origin(normal: FrameAxis) -> SketchPlane {
    SketchPlane::Datum(origin_plane(normal))
}

/// The flat face named `face`, to sketch on.
pub fn on_face(face: &str) -> SketchPlane {
    SketchPlane::Face(ElementReference::new(model(), face))
}

/// The step drawing `sketch` on `plane`.
pub fn sketch(plane: SketchPlane, sketch: Sketch) -> AddSketchArgs {
    AddSketchArgs {
        plane: Some(plane),
        sketch,
        ..Default::default()
    }
}

/// One side only, ending as `side1` says.
pub fn one_way(side1: Extent) -> Extents {
    Extents {
        side1,
        symmetric: false,
        side2: None,
        reversed: false,
    }
}

/// Both ways alike, `length` in all.
pub fn both_ways(length: f64) -> Extents {
    Extents {
        symmetric: true,
        ..Extents::blind(length)
    }
}

/// `first` one way and `second` the other.
pub fn two_ways(first: f64, second: f64) -> Extents {
    Extents {
        side2: Some(Extent::blind(second)),
        ..Extents::blind(first)
    }
}

/// The step extruding the sketch `sketch` as far as `extent` says into a
/// solid, combined as `combine` says.
pub fn extrusion(sketch: &str, extent: Extents, combine: Combine) -> ExtrudeArgs {
    ExtrudeArgs {
        sketch: whole_sketch(sketch),
        extent,
        output: Output::Solid(combine),
    }
}

/// Pushes `drawn` on `on` as `{name}_sketch`, and its extrude as far as
/// `extent` says as `name`, combined as `combine` says.
pub fn extrude_sketch(
    program: &mut Program,
    name: &str,
    on: SketchPlane,
    drawn: Sketch,
    extent: Extents,
    combine: Combine,
) {
    let sketch_name = format!("{name}_sketch");
    program.push(&sketch_name, sketch(on, drawn));
    program.push(name, extrusion(&sketch_name, extent, combine));
}

/// A 2 x 2 x 1 box with a blind hole drilled into its top, in the steps the
/// example `box_with_drill_hole` has: a square sketched on the Z plane
/// (`outline`) and extruded up (`box`), and a circle of radius 0.4 around
/// its middle sketched on the box's end cap (`hole_sketch`) and cut half
/// way back into it (`hole`) — the drilled box is `extrude(hole)`.
pub fn drilled_box() -> Program {
    let mut program = Program::new();
    let outline = sketch(on_origin(FrameAxis::Z), rectangle(0.0, 0.0, 2.0, 2.0));
    program.push("outline", outline);
    let body = extrusion("outline", Extents::blind(1.0), Combine::NewBody);
    program.push("box", body);
    let hole = sketch(on_face("extrude(box,end)"), disc([1.0, 1.0], 0.4));
    program.push("hole_sketch", hole);
    let cut = extrusion(
        "hole_sketch",
        Extents::blind(-0.5),
        Combine::cut("extrude(box)"),
    );
    program.push("hole", cut);
    program
}

// ── Programs ────────────────────────────────────────────────────────────────

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
pub fn step<'a>(program: &'a mut Program, id: &str) -> &'a mut ProgramStep {
    let index = program.index_of(id).unwrap_or_else(|e| panic!("{e}"));
    &mut program.steps[index]
}

/// The error `result` holds, for a test that expects one.
#[track_caller]
pub fn error<T>(result: GeopResult<T>) -> GeopError {
    match result {
        Ok(_) => panic!("expected an error"),
        Err(e) => e,
    }
}
