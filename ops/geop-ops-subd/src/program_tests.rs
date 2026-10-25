//! SubD bodies among the other solids: united with a block like any solid,
//! and the mouse example, as a program, saved and loaded and edited.

use crate::{Cage, Mirror, Subd, SubdArgs, build_subd};
use geop_core_math::geop_error::GeopResult;
use geop_core_program::operation_registry::OperationRegistry;
use geop_lib_booleans::{RemeshParams, boolean};
use geop_lib_sweep::shapes::block;
use geop_ops_model::{BooleanOp, NamedModel, Namer, testing::assert_valid};

/// The operations these tests' programs are written in.
fn registry() -> GeopResult<OperationRegistry> {
    OperationRegistry::default().with(Subd)
}

/// A rounded blob — the limit of a box cage two units a side around the
/// origin — united with a block reaching into it from one corner: one
/// valid solid.
#[test]
fn subd_body_united_with_a_box() {
    let mut part: NamedModel = NamedModel::new();
    let blob = SubdArgs {
        cage: Cage::cuboid([2.0, 2.0, 2.0]),
        mirror: Mirror::None,
    };
    build_subd(&mut part, "blob", &blob).unwrap();
    let blob = part.solid_id("subd(blob)").unwrap();
    let block = block(&mut part, "block", [0.1, 0.15, 0.0], [1.4, 1.35, 1.4]);
    let namer = Namer::new("boolean", "join").unwrap();
    let joined = boolean(
        &mut part,
        &namer,
        blob,
        block,
        BooleanOp::Union,
        RemeshParams::default(),
    )
    .unwrap_or_else(|e| panic!("{e}"));
    assert!(joined.is_some(), "the union is empty");
    assert_valid(&part);
    assert_eq!(part.solid_names().len(), 1, "{:?}", part.solid_names());
}

/// A mouse's sole is flat: the crease around its bottom keeps the
/// bottom face in the cage's bottom plane. Saved and loaded, the example
/// builds what it built.
#[test]
fn subd_mouse_round_trips() {
    use geop_core_math::scalars::Scalar;
    use geop_ops_model::read_model;

    let registry = std::rc::Rc::new(registry().unwrap());
    let program = crate::examples::subd_mouse();
    let built = geop_core_program::testing::assert_round_trips(registry, None, &program);
    let reader = built.current();
    let part = read_model(&reader).unwrap();
    assert_valid(&part);
    assert_eq!(part.solid_names(), ["subd(mouse)"]);
    let sole = part.face_id("subd(mouse,f8)").unwrap();
    let surface = &part.topology().get_face(sole).unwrap().surface;
    let p = surface
        .evaluate(Scalar::from_f64(0.7), Scalar::from_f64(1.2))
        .unwrap();
    assert!(p[2].could_be_equal(Scalar::from_f64(-0.4)), "{p:?}");
}

/// The example `subd_mouse`, edited as a user edits it, builds incrementally
/// what it builds from scratch.
#[test]
fn subd_mouse_builds_incrementally() {
    let registry = std::rc::Rc::new(registry().unwrap());
    let programs = geop_core_program::testing::edit_sequence(&crate::examples::subd_mouse());
    geop_core_program::testing::assert_incremental(registry, None, &programs);
}
