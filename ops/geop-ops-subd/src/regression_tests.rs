//! The SubD examples edited further, as the box modelling tutorials edit
//! them, built and checked.

use geop_core_math::scalars::Scalar;
use geop_ops_model::{NamedModel, testing::assert_valid};

use crate::{Cage, Mirror, SubdArgs, build_subd, examples};

/// The cage and the mirror of the example `example`'s one step.
fn cage_of(example: fn() -> geop_core_program::program::Program) -> (Cage, Mirror) {
    let program = example();
    let args: SubdArgs = geop_core_program::testing::args(&program.steps[0]);
    (args.cage, args.mirror)
}

/// The body `step` builds from `cage`.
fn built(step: &str, cage: Cage, mirror: Mirror) -> NamedModel {
    let mut part = NamedModel::new();
    build_subd(&mut part, step, &SubdArgs { cage, mirror }).unwrap();
    part
}

/// The edges of `cage` both of whose ends lie beyond `x`: on the mug, the
/// handle's.
fn edges_beyond_x(cage: &Cage, x: f64) -> Vec<[u32; 2]> {
    cage.edges()
        .into_iter()
        .filter(|&[a, b]| cage.vertex(a).unwrap().at[0] > x && cage.vertex(b).unwrap().at[0] > x)
        .collect()
}

/// The mug example (half an eight-sided cylinder mirrored in `y = 0`, its
/// handle two faces extruded out and bridged) with a loop cut across one
/// edge of the handle, as a tutorial cuts a loop to shape the handle, then
/// built.
///
/// The body built is invalid: `edge subd(mug,e96-97) runs from ... a chord
/// of 1e-14 — shorter than the 1e-4 the searches can resolve`, `vertex
/// subd(mug,v95) and vertex subd(mug,v98) could overlap`, `edge
/// subd(mug,e95-96) and edge subd(mug,e97-98) are coincident along an
/// arc`. The cut cage itself has two pairs of vertices at one point:
/// `v95`/`v98` at `(1.8, 0, 0)` on the mirror plane and `v96`/`v97` at
/// `(1.446, 1.013, 0)`. Cut at 0.3 of the way instead of half, no two
/// vertices coincide, yet the body is still invalid (8 errors): the ring
/// around the handle crossed some edge twice, so the cut is laid into it
/// twice over. Ruled out: the crease (none is set here); the other handle
/// edge (the same); a loop cut across a body edge of the mug builds a
/// valid body.
#[test]
#[ignore = "bug: a loop cut across the mug's handle builds an invalid body, subd(mug,e96-97) of chord 1e-14 and vertices v95/v98 coincident; the ring around the handle visits an edge twice (not the crease, not the cut's position)"]
fn loop_cut_across_the_mug_handle_builds_a_valid_body() {
    let (mut cage, mirror) = cage_of(examples::subd_mug);
    let handle = edges_beyond_x(&cage, 1.5);
    let [a, b] = handle[0];
    cage.insert_loop(a, b, 0.5).unwrap();
    let coincident: Vec<(u32, u32)> = cage
        .vertices
        .iter()
        .flat_map(|p| {
            cage.vertices
                .iter()
                .filter(move |q| q.id > p.id && p.at == q.at)
                .map(move |q| (p.id, q.id))
        })
        .collect();
    assert!(
        coincident.is_empty(),
        "the cut cage has vertices at one point: {coincident:?}"
    );
    assert_valid(&built("mug", cage, mirror));
}
