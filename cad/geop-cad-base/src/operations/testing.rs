//! What the tests of programs share: building one, reading and changing
//! its steps' arguments as their types, drawing the sketches it starts
//! from, editing it as a front end does, and checking what it built.

use std::collections::BTreeMap;

use geop_core_math::geop_error::{GeopError, GeopResult};
use geop_core_math::{
    primitives::Ray,
    scalars::{ScalInF64 as S, Scalar},
    vector::Vector3,
};
use geop_core_program::program::ProgramStep;
use geop_core_topology::{
    Body,
    validation::{ValidationParameters, validate_manifold},
};
use geop_ops::{
    Design, EntityRef, NamedModel,
    model::model,
    ui::{Button, Pointer, Reach, StepEditEvent, Value},
};
use geop_ops_booleans::{Combine, boolean::BooleanOp};
use geop_ops_extrude_revolve::{Extent, Extents, ExtrudeArgs, Output};
use geop_ops_rasterize::{rasterize, stl::stl_triangles};
use geop_ops_sketch::{AddSketchArgs, Sketch};
use serde::{Serialize, de::DeserializeOwned};

pub(crate) use geop_ops::{origin_axis, origin_plane};

use super::{Program, Runner, build, workspace};
use crate::{
    Command, Editor, Push, Update,
    examples::{n, polygon},
};

/// Building a program on its own, in an empty [`workspace`] (see [`build`]).
pub(crate) trait Built {
    /// The runner, holding the part the program builds.
    fn built(&self) -> GeopResult<Runner>;

    /// The model the program builds, or why a step fails.
    fn legacy_build_model(&self) -> GeopResult<NamedModel<Design>>;
}

impl Built for Program {
    fn built(&self) -> GeopResult<Runner> {
        build(self, &workspace(BTreeMap::new()).scope(""))
    }

    fn legacy_build_model(&self) -> GeopResult<NamedModel<Design>> {
        let runner = self.built()?;
        let built = runner.part.current().retrieve_target(&model());
        // A program that builds no topology builds an empty model.
        Ok(built.cloned().unwrap_or_default())
    }
}

/// The arguments of `step`, as their type `A`.
pub(crate) fn args<A: DeserializeOwned>(step: &ProgramStep) -> A {
    serde_json::from_value(step.args.clone())
        .unwrap_or_else(|e| panic!("step {:?} holds no such arguments: {e}", step.id))
}

/// Changes the arguments of `step`, as their type `A`, by `edit`.
pub(crate) fn edit_args<A: Serialize + DeserializeOwned>(
    step: &mut ProgramStep,
    edit: impl FnOnce(&mut A),
) {
    let mut args: A = args(step);
    edit(&mut args);
    step.args = serde_json::to_value(args).expect("arguments are plain data");
}

/// The step `id` of `program`.
pub(crate) fn step<'a>(program: &'a mut Program, id: &str) -> &'a mut ProgramStep {
    let index = program.index_of(id).unwrap_or_else(|e| panic!("{e}"));
    &mut program.program.steps[index]
}

/// The error `result` holds, for a test that expects one.
pub(crate) fn error<T>(result: GeopResult<T>) -> GeopError {
    match result {
        Ok(_) => panic!("expected an error"),
        Err(e) => e,
    }
}

/// The step drawing `sketch` on `plane`.
pub(crate) fn sketch(plane: EntityRef, sketch: Sketch) -> AddSketchArgs {
    AddSketchArgs {
        plane: Some(plane),
        sketch,
        ..Default::default()
    }
}

/// Combined with the solid `target` by cutting it away.
pub(crate) fn cut(target: impl Into<String>) -> Combine {
    Combine::With {
        op: BooleanOp::Difference,
        target: target.into(),
    }
}

/// Combined with the solid `target` by joining it.
pub(crate) fn join(target: impl Into<String>) -> Combine {
    Combine::With {
        op: BooleanOp::Union,
        target: target.into(),
    }
}

/// One side only, ending as `side1` says.
pub(crate) fn one_way(side1: Extent) -> Extents {
    Extents {
        side1,
        symmetric: false,
        side2: None,
        reversed: false,
    }
}

/// Both ways alike, `length` in all.
pub(crate) fn both_ways(length: f64) -> Extents {
    Extents {
        symmetric: true,
        ..Extents::blind(length)
    }
}

/// `first` one way and `second` the other.
pub(crate) fn two_ways(first: f64, second: f64) -> Extents {
    Extents {
        side2: Some(Extent::blind(second)),
        ..Extents::blind(first)
    }
}

/// The step extruding the sketch `sketch` as far as `extent` says into a
/// solid, combined as `combine` says.
pub(crate) fn extrusion(sketch: &str, extent: Extents, combine: Combine) -> ExtrudeArgs {
    ExtrudeArgs {
        sketch: sketch.into(),
        extent,
        output: Output::Solid(combine),
    }
}

/// Pushes `drawn` on `on` as `{name}_sketch`, and its extrude as far as
/// `extent` says as `name`, combined as `combine` says.
pub(crate) fn extrude_sketch(
    program: &mut Program,
    name: &str,
    on: EntityRef,
    drawn: Sketch,
    extent: Extents,
    combine: Combine,
) {
    let sketch_name = format!("{name}_sketch");
    program.push(&sketch_name, sketch(on, drawn));
    program.push(name, extrusion(&sketch_name, extent, combine));
}

/// The rectangle from `(x0, y0)` to `(x1, y1)`.
pub(crate) fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> Sketch {
    outline(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
}

/// Whether `part` has a vertex that could be at `p`.
pub(crate) fn has_vertex(part: &NamedModel<Design>, p: [f64; 3]) -> bool {
    let p = p.map(S::from_f64);
    (part.topology().vertices.values()).any(|v| (0..3).all(|k| v.point[k].could_be_equal(p[k])))
}

/// A sketch of nothing but the closed polygon through `corners` (see
/// [`polygon`]).
pub(crate) fn outline(corners: &[[f64; 2]]) -> Sketch {
    let mut sketch = Sketch::new();
    polygon(&mut sketch, corners);
    sketch
}

/// A sketch of nothing but the circle of `radius` around `center`.
pub(crate) fn disc(center: [f64; 2], radius: f64) -> Sketch {
    let mut sketch = Sketch::new();
    let c = sketch.add_point(n(center[0]), n(center[1]));
    sketch.add_circle(c, n(radius));
    sketch
}

/// What `editor` sends back on `command`, which it has to take without an
/// error.
pub(crate) fn handled(editor: &mut Editor, command: Command) -> Update {
    let update = editor.handle(command);
    assert!(update.error.is_none(), "{:?}", update.error);
    update
}

/// A pointer from `origin` along `dir`, reaching 0.009 units.
pub(crate) fn pointer(origin: [f64; 3], dir: [f64; 3]) -> Pointer<S> {
    let v = |p: [f64; 3]| Vector3::from_array(p.map(S::from_f64));
    Pointer {
        ray: Ray::try_new(v(origin), v(dir)).unwrap(),
        reach: Reach::Tube {
            radius: S::from_f64(0.009),
        },
    }
}

/// Looking straight down at `(x, y)`.
pub(crate) fn from_above(x: f64, y: f64) -> Pointer<S> {
    pointer([x, y, 10.0], [0.0, 0.0, -1.0])
}

/// A click of the primary button with `pointer`.
pub(crate) fn click_at(pointer: Pointer<S>) -> Command {
    Command::Event {
        event: StepEditEvent::Click {
            pointer,
            button: Button::Primary,
            double: false,
            shift: false,
        },
    }
}

/// An editor on `program`, which loads without an error.
pub(crate) fn loaded(program: Program) -> Editor {
    let mut editor = Editor::new();
    handled(
        &mut editor,
        Command::Load {
            program,
            path: None,
        },
    );
    editor
}

/// An editor on the example `name`.
pub(crate) fn example_editor(name: &str) -> Editor {
    let mut editor = Editor::new();
    handled(&mut editor, Command::LoadExample { name: name.into() });
    editor
}

/// The command a front end sends when the open step's dialog field `key`
/// is set to `value`.
pub(crate) fn dialog(key: &str, value: Value) -> Command {
    Command::Event {
        event: StepEditEvent::Dialog {
            key: key.into(),
            value,
        },
    }
}

/// The volume the solid named `solid` of `part` encloses, from its mesh as
/// drawn.
pub(crate) fn volume(part: &NamedModel<Design>, solid: &str) -> f64 {
    let id = part.solid_id(solid).unwrap();
    let faces = part.topology().body_faces(Body::Solid(id)).unwrap();
    let raster = rasterize(part.topology(), 64).unwrap();
    stl_triangles(&raster, &faces)
        .iter()
        .map(|t| {
            let [a, b, c] = t.corners.map(|p| p.map(f64::from));
            (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0]))
                / 6.0
        })
        .sum()
}

/// Checks the solid named `solid` of `part` encloses `expected`, to within
/// `tolerance` (see [`volume`]).
pub(crate) fn assert_volume(part: &NamedModel<Design>, solid: &str, expected: f64, tolerance: f64) {
    let got = volume(part, solid);
    assert!(
        (got - expected).abs() <= tolerance,
        "{solid} encloses {got}, expected {expected} (± {tolerance})"
    );
}

/// Checks `part` is a valid manifold model with consistent names, failing
/// with the report of [`check_valid`].
pub(crate) fn assert_valid(part: &NamedModel<Design>) {
    if let Err(report) = check_valid(part) {
        panic!("{report}");
    }
    part.check_names().unwrap();
}

/// Whether `part` is a valid manifold model — if not, every validation
/// error's root message, one per line, and the names of the entities they
/// mention.
pub(crate) fn check_valid(part: &NamedModel<Design>) -> Result<(), String> {
    // `validate_manifold` runs `validate` first.
    let Err(errors) = validate_manifold(&ValidationParameters::default(), part.topology()) else {
        return Ok(());
    };
    let all = errors
        .iter()
        .map(|e| e.root_message())
        .collect::<Vec<_>>()
        .join("\n");
    Err(format!(
        "{} validation error(s):\n{all}\nwhere {}",
        errors.len(),
        part.names_mentioned(&all).join(", ")
    ))
}
