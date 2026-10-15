//! What the drawing's tests share: parts as a program's run leaves them —
//! a model given, parts placed — and parts to place.

use geop_core_math::{geop_error::GeopResult, primitives::Pose};
use geop_ops_assembly::testing::{Shelf, given_model, place, runner};
use geop_ops_model::{Design, NamedModel, model};
use geop_ops_parameters::{
    BuildPart, PartReader, ProgramFile, Settings, StateValue, StateValues, pose_state,
};

/// A part's model, as the tests build it.
pub(crate) type Part<S> = NamedModel<S>;

/// `value`, as the generator of a target that is given it.
pub(crate) fn given<T: Clone>(_: &PartReader, value: &T) -> GeopResult<T> {
    Ok(value.clone())
}

/// A part to place: the program file's name, and its program, which gives
/// the part its model.
#[derive(Clone)]
pub(crate) struct Placeable {
    pub(crate) file: String,
    pub(crate) program: ProgramFile,
}

/// `bodies` as the part of the program file `file`, to place.
pub(crate) fn placeable(file: &str, bodies: &NamedModel<Design>) -> Placeable {
    let mut program = ProgramFile::new();
    program.program.steps.push(given_model("model", bodies.clone(), None));
    Placeable {
        file: file.into(),
        program,
    }
}

/// The part a program builds that gives `bodies` as its model, and last
/// runs `then`.
pub(crate) fn built_with(
    bodies: &NamedModel<Design>,
    then: impl FnOnce(&mut BuildPart),
) -> BuildPart {
    let mut part = BuildPart::default();
    part.begin_run(&Default::default(), &Default::default());
    part.begin_step();
    part.define_target(&model(), bodies, given).unwrap();
    then(&mut part);
    part.end_run(0);
    part
}

/// The part a program builds that gives `bodies` as its model and nothing
/// else (see [`built_with`]).
pub(crate) fn built(bodies: &NamedModel<Design>) -> BuildPart {
    built_with(bodies, |_| {})
}

/// The part the program of the file `assembly.geop` builds that places
/// each of `placed` in a step of its name, fixed at its pose: the
/// placeables' files beside it.
pub(crate) fn assembled(placed: &[(&str, &Placeable, Pose<Design>)]) -> BuildPart {
    let mut root = ProgramFile::new();
    let mut shelf = Shelf::default();
    for (name, placeable, pose) in placed {
        root.program.steps.push(place(name, &placeable.file, &Settings::new()));
        (root.state).insert(pose_state(name), StateValue::Pose(*pose));
        shelf = shelf.with(&placeable.file, placeable.program.clone());
    }
    let file = "assembly.geop";
    let runner = runner(shelf.with(file, root.clone()), file);
    let mut part = BuildPart::default();
    (root.run(&runner, &mut part, &Settings::new(), &StateValues::new())).unwrap();
    part
}
