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

/// The SubD mouse with its top sliced off flat at `z = 0.5` by a block cut
/// away, and the rim of the flat top filleted by 0.05 — the way a tutorial
/// flattens a SubD body for a button and rounds the cut. The rim is the
/// two traced curves where the slice meets the hump, `combine(slice,
/// block(slice,start), subd(mouse,f9), ...)` and its mirror image.
///
/// Fails with `the blend strays inf from the rolling ball with 64 stations
/// per edge (inf with half as many), more than the 5e-8 it may: in span 0,
/// at 0.523 of the span the blend could dip through face FaceId(39)` — an
/// infinite deviation, so an enclosure in the blend's check diverged
/// (`distance` is the midpoint of an interval norm), not a blend that
/// strays. Each of the two rim edges fails alike on its own, and so does a
/// radius of 0.02. Sliced lower, at `z = 0.3`, the rim is refused by name
/// instead (`the third face could run along the edge`).
#[test]
#[ignore = "bug: filleting the rim of a SubD mouse sliced flat at z = 0.5 refuses with `the blend strays inf from the rolling ball`: the deviation check's enclosure diverged to infinity (each rim edge alone, radius 0.02 alike)"]
fn sliced_subd_mouse_rim_filleted() {
    let mut program = Program::new();
    program.push("mouse", mouse());
    program.push(
        "slice",
        PrimitiveArgs::block([-2.0, -2.0, 0.5], [2.0, 2.0, 2.0]).cut("subd(mouse)"),
    );
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
    let top = faces_at_z(&part, 0.5);
    assert_eq!(top.len(), 1, "{top:?}");
    let rim = perimeter_of(&part, &[&top[0]]);
    assert_eq!(rim.len(), 2, "{rim:?}");
    program.push("round", FilletArgs::constant(rim, 0.05));
    let part = program.build_model(registry).unwrap();
    assert_valid(&part);
}
