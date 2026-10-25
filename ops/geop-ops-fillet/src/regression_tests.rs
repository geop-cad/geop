//! Programs from tutorials and bug reports, rebuilt in Rust: fillets of
//! shelled boxes, as the beginners' lessons round them.

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{
    element::ElementReference, operation_registry::OperationRegistry, program::Program,
};
use geop_lib_fixtures::PrimitiveArgs;
use geop_ops_model::{
    NamedModel, model, solid,
    testing::{Built, assert_valid, edges_with_middle_where},
};
use geop_ops_shell::{Shell, ShellArgs};

use crate::{Fillet, FilletArgs};

/// The operations these programs are written in: given solids, shelled
/// and rounded.
fn registry() -> GeopResult<OperationRegistry> {
    geop_lib_fixtures::registry()?.with(Shell)?.with(Fillet)
}

fn near(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-6
}

/// The vertices of `part` at height `z`, as points.
fn vertices_at(part: &NamedModel, z: f64) -> Vec<[f64; 3]> {
    let mut points: Vec<[f64; 3]> = (part.topology().vertices.values())
        .map(|v| v.point.to_f64_array())
        .filter(|p| near(p[2], z))
        .collect();
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    points
}

/// SolidWorks Lesson 1 with the shell first: a 100 x 60 x 50 block
/// hollowed from the top with walls and a floor 5 thick, then its four
/// outer vertical edges rounded with radius 10.
///
/// The fillet builds a valid solid of the right volume and face count
/// (15), but splits every contact edge of the blends at `z = 5`: eight new
/// vertices at the height of the floor's inside, on the *outer* walls —
/// `(10, 0, 5)`, `(0, 10, 5)`, ... — where nothing meets them. The edges
/// are named after the floor's inner face, `fillet(round,...,ta) x
/// shell(shell,block(box,start))`: the blend's contact line was crossed
/// with the floor's plane rather than with the floor's face, which ends 5
/// inside the walls. The same box hollowed by cutting a block out instead
/// of shelling — the same solid — is rounded without the splits, so it is
/// something the shell leaves in its floor (its inner faces are offsets)
/// that the fillet's boolean reads as reaching the walls.
#[test]
#[ignore = "bug: rounding the outer vertical edges of a shelled box splits each blend's contact edges at the floor's inside height (z = 5), on the outer walls, where no face meets them; the same box hollowed by a cut is rounded cleanly"]
fn rounding_the_outer_edges_of_a_shelled_box_splits_them_at_the_floor() {
    let mut program = Program::new();
    program.push(
        "box",
        PrimitiveArgs::block([0.0, 0.0, 0.0], [100.0, 60.0, 50.0]),
    );
    program.push(
        "shell",
        ShellArgs {
            solid: solid("block(box)"),
            faces: vec![ElementReference::new(model(), "block(box,end)")],
            thickness: 5.0.into(),
        },
    );
    let part = program.build_model(registry).unwrap();
    let outer = edges_with_middle_where(&part, |c| near(c[2], 25.0));
    assert_eq!(outer.len(), 4, "{outer:?}");
    program.push("round", FilletArgs::constant(outer, 10.0));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let split = edges_with_middle_where(&part, |c| near(c[2], 27.5));
    // Only the four inner corners run from the floor at `z = 5` to the top.
    assert_eq!(split.len(), 4, "{split:?}");
    let floor = vertices_at(&part, 5.0);
    assert_eq!(floor.len(), 4, "{floor:?}");
}
