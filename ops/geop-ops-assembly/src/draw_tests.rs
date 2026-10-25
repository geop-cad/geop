//! Placed parts as drawn ([`AssemblyDrawer`]): by reference, one instance
//! per placement, never a mesh of their own.

use geop_core_editor::registry::Registry;
use geop_core_math::{
    primitives::{Pose, Quaternion},
    scalars::Scalar,
    vector::Vector3,
};
use geop_core_program::{
    part::Part,
    program::Program,
    values::{Settings, StateValue, pose_state},
};
use geop_lib_sweep::shapes::block_part;

use crate::{
    draw::AssemblyDrawer,
    testing::{Shelf, given_model, place, runner},
};

fn at(x: f64) -> Pose {
    let v = Vector3::from_array([x, 0.0, 0.0].map(Scalar::from_f64));
    Pose::new(v, Quaternion::identity()).unwrap()
}

/// `program`, placing `file` as `name` at `x` along `x`.
fn placing(mut program: Program, name: &str, file: &str, x: f64) -> Program {
    program.steps.push(place(name, file, &Settings::new()));
    (program.state).insert(pose_state(name), StateValue::Pose(at(x)));
    program
}

/// A cube placed twice in an assembly, and once more in a sub-assembly
/// placed in it: four instances, however deep, each where it is — the
/// three cubes of one component, which a viewer is sent once by its key,
/// and the assembly itself drawn with no face of its own.
#[test]
fn a_part_placed_three_times_is_three_instances_of_one_component() {
    let mut cube = Program::new();
    cube.steps.push(given_model(
        "model",
        block_part("c", [0.0; 3], [1.0; 3]),
        None,
    ));
    let sub = placing(Program::new(), "c", "cube.geop", 0.0);
    let top = placing(Program::new(), "a", "cube.geop", 0.0);
    let top = placing(top, "b", "cube.geop", 3.0);
    let top = placing(top, "s", "sub.geop", 6.0);
    let shelf = (Shelf::default().with("cube.geop", cube))
        .with("sub.geop", sub)
        .with("top.geop", top.clone());
    let mut part = Part::default();
    top.run(&runner(shelf, "top.geop"), &mut part).unwrap();

    let registry = Registry::default().drawer(AssemblyDrawer).unwrap();
    let view = registry.draw(&part.current()).unwrap();
    assert!(
        view.faces.is_empty(),
        "the assembly has no model of its own"
    );
    let names: Vec<&str> = view.instances.iter().map(|i| i.name.as_str()).collect();
    assert_eq!(names, ["a", "b", "s", "s/c"]);
    let cubes: Vec<_> = (view.instances.iter()).filter(|i| i.name != "s").collect();
    for cube in &cubes {
        assert_eq!(cube.component, cubes[0].component, "{cube:?}");
        assert_eq!(cube.view.faces.len(), 6);
    }
    let xs: Vec<f64> = cubes
        .iter()
        .map(|c| c.pose.position()[0].to_f64())
        .collect();
    assert_eq!(xs, [0.0, 3.0, 6.0]);
}
