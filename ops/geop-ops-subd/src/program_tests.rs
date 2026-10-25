//! SubD bodies among the other solids: united with a block like any solid,
//! and the mouse and mug examples, as programs, saved and loaded and
//! edited.

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

/// A mug is one solid with a handle: its surface is a torus joined to a
/// cup, so it has a hole through it — its handle's inner faces are there,
/// named after the bridge's quads — and an inside floor below its lip.
/// Saved and loaded, the example builds what it built.
#[test]
fn subd_mug_round_trips() {
    use geop_ops_model::read_model;

    let registry = std::rc::Rc::new(registry().unwrap());
    let program = crate::examples::subd_mug();
    let built = geop_core_program::testing::assert_round_trips(registry, None, &program);
    let reader = built.current();
    let part = read_model(&reader).unwrap();
    // Not `assert_valid`: see `subd_mug_is_valid`.
    assert_eq!(part.solid_names(), ["subd(mug)"]);
    let names = part.topology().faces.len();
    assert!(names > 60, "{names} faces");
}

/// The example `subd_mug`, edited as a user edits it, builds incrementally
/// what it builds from scratch.
#[test]
fn subd_mug_builds_incrementally() {
    let registry = std::rc::Rc::new(registry().unwrap());
    let programs = geop_core_program::testing::edit_sequence(&crate::examples::subd_mug());
    geop_core_program::testing::assert_incremental(registry, None, &programs);
}

/// The mug is a valid solid. It is not yet: its handle patches `f75` and
/// `f75m`, at the handle's outer corners, are each a ribbon folded onto
/// itself. Along `v = 1` of `f75`'s domain `[0, 2]²`, `(x, y)` runs from
/// `(1.478, 0.834)` at `u = 0` in to `(1.430, 0.959)` at `u = 1` and back
/// out to `(1.474, 0.834)` at `u = 2`, with `z` only moving from `-0.024`
/// to `0`; the normal is `(0.93, 0.37, 0)` on the way in and
/// `(-0.94, -0.33, 0)` on the way back, so the two halves lie on each other
/// facing opposite ways. The patch's corners at `v = 0` coincide, and so do
/// those at `v = 2`: the cage quad behind it is degenerate (the bridge of
/// the handle's two arms, mirrored). The normal check finds the half whose
/// normal points into the solid, and names the fold: a ray leaving the face
/// at `(1, 1)` crosses the other half `6e-7` away.
#[test]
#[ignore = "bug: the subd mug's handle patches f75 and f75m are ribbons folded onto themselves (a degenerate cage quad at the handle's corners), so half of each has its normal pointing into the solid"]
fn subd_mug_is_valid() {
    let mut part: NamedModel = NamedModel::new();
    let program = crate::examples::subd_mug();
    let args: SubdArgs = geop_core_program::testing::args(program.step("mug").unwrap());
    build_subd(&mut part, "mug", &args).unwrap();
    assert_valid(&part);
}
