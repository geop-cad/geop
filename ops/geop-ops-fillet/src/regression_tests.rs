//! Fillets on bodies the tutorials round: SubD bodies, as modelled and as
//! cut, their rims picked as a user picks them.

use geop_core_math::geop_error::GeopResult;
use geop_core_program::{operation_registry::OperationRegistry, program::Program};
use geop_lib_fixtures::{Primitive, PrimitiveArgs};
use geop_ops_model::{
    NamedModel,
    testing::{Built, apply, assert_valid, model_of},
};
use geop_ops_subd::{Subd, SubdArgs, build_subd, examples};

use crate::{Fillet, FilletArgs};

/// The operations these programs are written in: SubD bodies, given solids
/// to cut them with, and fillets.
fn registry() -> GeopResult<OperationRegistry> {
    OperationRegistry::default()
        .with(Subd)?
        .with(Primitive)?
        .with(Fillet)
}

/// The mouse example's one step, as arguments.
fn mouse() -> SubdArgs {
    geop_core_program::testing::args(&examples::subd_mouse().steps[0])
}

/// The edges bounding exactly one of the faces `faces` of `part`: the rim
/// of the region they make up, without the edges between them.
fn perimeter_of(part: &NamedModel, faces: &[&str]) -> Vec<String> {
    let model = part.topology();
    let mut count: std::collections::BTreeMap<String, usize> = Default::default();
    for f in faces {
        for c in model.iterate_face_coedges(part.face_id(f).unwrap()) {
            if let Ok(e) = model.get_coedge(c).unwrap().edge() {
                *count
                    .entry(part.name_of(e).unwrap().to_string())
                    .or_default() += 1;
            }
        }
    }
    count
        .into_iter()
        .filter(|(_, k)| *k == 1)
        .map(|(n, _)| n)
        .collect()
}

/// The faces of `part` all of whose vertices are at height `z`.
fn faces_at_z(part: &NamedModel, z: f64) -> Vec<String> {
    let model = part.topology();
    let mut names: Vec<String> = model
        .faces
        .keys()
        .filter(|&&f| {
            let mut any = false;
            let flat = model.iterate_face_coedges(f).all(|c| {
                any = true;
                let p = model.coedge_start_vertex(c).unwrap().point;
                (p[2].to_f64() - z).abs() < 1e-9
            });
            any && flat
        })
        .map(|&f| part.name_of(f).unwrap().to_string())
        .collect();
    names.sort();
    names
}

/// The SubD mouse example — a box cage mirrored in `x = 0`, the rim of its
/// flat sole creased sharp — with its sole's rim filleted by 0.05, as a
/// tutorial rounds the bottom edge of a mouse. The rim is the six edges
/// around the two halves of the sole, `subd(mouse,f8)` and `f8m`.
///
/// Fails with `Cannot normalize zero-length vector` at edge
/// `subd(mouse,e1-14)` (its context: `vertex subd(mouse,v1) meets faces
/// f8, f10, f11`). Each rim edge fails on its own too: `e1-2`, `e1-14` and
/// `e14-1m` the same way, `e15-2m`, `e1m-2m` and `e2-15` with
/// `surface_could_contain (clipping): exhausted max_nodes=20000 with 885
/// patches pending`. The zero vector is the sole's normal at its corners
/// `v1` and `v2`: the limit patch's parametrization is singular there,
/// `|du x dv| = 1.5e-14` with `|du| = |dv| = 0.52` (see `geop-ops-subd`'s
/// `regression_tests::sole_of_the_mouse_has_a_normal_at_its_corners`).
/// Ruled out: the radius (0.1 the same), the mirror plane (edge `e1-2`
/// touches it at neither end).
#[test]
#[ignore = "bug: filleting the SubD mouse's creased sole rim fails with `Cannot normalize zero-length vector` at subd(mouse,e1-14); the sole's limit patch is singular at its corners v1 and v2, where the fillet takes its normal (not the radius, not the mirror plane)"]
fn subd_mouse_sole_rim_filleted() {
    let args = mouse();
    let mut model = NamedModel::new();
    build_subd(&mut model, "mouse", &args).unwrap();
    let rim = perimeter_of(&model, &["subd(mouse,f8)", "subd(mouse,f8m)"]);
    assert_eq!(rim.len(), 6, "{rim:?}");
    let part = apply::<Fillet>(model, "round", &FilletArgs::constant(rim, 0.05)).unwrap();
    assert_valid(&model_of(&part));
}
