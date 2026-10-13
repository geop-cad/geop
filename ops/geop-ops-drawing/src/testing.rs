//! What the drawing's tests share: parts as a program's run leaves them —
//! a model given, parts placed — and parts to place.

use std::{collections::BTreeSet, rc::Rc};

use geop_core_math::{geop_error::GeopResult, primitives::Pose};
use geop_ops_assembly::{Component, Instance, Placement};
use geop_ops_model::{Design, NamedModel, model};
use geop_ops_parameters::{BuildPart, Parameters, PartReader, StateValue, pose_state};
use geop_ops_view::PartView;

/// A part's model, as the tests build it.
pub(crate) type Part<S> = NamedModel<S>;

/// `value`, as the generator of a target that is given it.
pub(crate) fn given<T: Clone>(_: &PartReader, value: &T) -> GeopResult<T> {
    Ok(value.clone())
}

/// The part a program builds that gives `bodies` as its model in its
/// first step, places each of `placed` in a step of its name, fixed at its
/// pose, and last runs `then`.
pub(crate) fn built_with(
    bodies: &NamedModel<Design>,
    placed: &[(&str, &Rc<Component>, Pose<Design>)],
    then: impl FnOnce(&mut BuildPart),
) -> BuildPart {
    let state = (placed.iter())
        .map(|(name, _, pose)| (pose_state(name), StateValue::Pose(*pose)))
        .collect();
    let mut part = BuildPart::default();
    part.begin_run(&Default::default(), &state);
    part.begin_step();
    part.define_target(&model(), bodies, given).unwrap();
    for (name, component, _) in placed {
        part.begin_step();
        let placed = Instance {
            component: (*component).clone(),
            placement: Placement::State(pose_state(name)),
        };
        (part.define_instance(name, &placed, |_, placed, _| Ok(placed.clone()))).unwrap();
    }
    then(&mut part);
    part.end_run(0);
    part
}

/// The part a program builds that gives `bodies` as its model and nothing
/// else (see [`built_with`]).
pub(crate) fn built(bodies: &NamedModel<Design>) -> BuildPart {
    built_with(bodies, &[], |_| {})
}

/// A part placed, as a drawing reads it: drawn from its model, never as
/// the editor draws it.
fn undrawn(_: &PartReader) -> GeopResult<PartView> {
    Ok(PartView::default())
}

/// `bodies` as the part of the program file `file`, to place.
pub(crate) fn component(file: &str, bodies: &NamedModel<Design>) -> Rc<Component> {
    Rc::new(Component::new(
        file.into(),
        BTreeSet::from([file.to_string()]),
        file.into(),
        Parameters::default(),
        built(bodies),
        undrawn,
    ))
}
