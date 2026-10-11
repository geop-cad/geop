//! What the drawing's tests share: parts built as a program would build
//! them, from a model, cosmetic threads and parts placed in it.

use std::{collections::BTreeMap, collections::BTreeSet, rc::Rc};

use geop_core_math::{geop_error::GeopResult, primitives::Pose};
use geop_ops::{
    Design, NamedModel, ParamValue,
    model::{model, pose_parameter},
    operation::{Build, BuildReader},
    parameters::Parameters,
    placed::{Component, Instance, Placement, instance},
};
use geop_ops_hole::{Threads, threads};

/// A part's model, as the tests build it.
pub(crate) type Part<S> = NamedModel<S>;

fn given(_: &mut BuildReader, model: &NamedModel<Design>) -> GeopResult<NamedModel<Design>> {
    Ok(model.clone())
}

fn recorded(_: &mut BuildReader, threads: &Threads<Design>) -> GeopResult<Threads<Design>> {
    Ok(threads.clone())
}

fn placing(_: &mut BuildReader, placed: &Instance) -> GeopResult<Instance> {
    Ok(placed.clone())
}

/// The part of the model `bodies`, its cosmetic threads `threads`, with
/// `placed` placed in it, each at its pose: as a program of a step for
/// each would build it.
pub(crate) fn built_with(
    bodies: &NamedModel<Design>,
    threads_given: Option<&Threads<Design>>,
    placed: &[(String, Rc<Component>, Pose<Design>)],
) -> Build {
    let state = (placed.iter())
        .map(|(name, _, pose)| (pose_parameter(name), ParamValue::Pose(*pose)))
        .collect();
    let mut part = Build::default();
    part.begin_run(&BTreeMap::new(), &state);
    part.begin_step("given").unwrap();
    part.define_target(&model(), bodies, given).unwrap();
    if let Some(given) = threads_given {
        part.define_target(&threads(), given, recorded).unwrap();
    }
    for (name, component, _) in placed {
        part.begin_step(name).unwrap();
        let placed = Instance {
            component: Rc::clone(component),
            placement: Placement::Parameter(pose_parameter(name)),
            fixed: true,
        };
        part.define_target(&instance(name), &placed, placing)
            .unwrap();
    }
    part.end_run(Default::default());
    part
}

/// The part of the model `bodies` alone.
pub(crate) fn built(bodies: &NamedModel<Design>) -> Build {
    built_with(bodies, None, &[])
}

/// `bodies` as the part of the program file `file`, to place.
pub(crate) fn component(file: &str, bodies: &NamedModel<Design>) -> Rc<Component> {
    Rc::new(Component::new(
        file.into(),
        BTreeSet::from([file.to_string()]),
        file.into(),
        Parameters::default(),
        built(bodies),
    ))
}
