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
/// The sole's limit patch is singular at its corners `v1` and `v2` (and
/// their mirror images): the crease runs on smoothly through them, and the
/// patch's two sides leave each along one line, `S_u` and `S_v` collinear
/// (see `geop-ops-subd`'s
/// `regression_tests::sole_of_the_mouse_has_a_normal_at_its_corners`). The
/// rim is one closed tangent chain, and its vertices are stations of their
/// own (`turns`), so balls are placed exactly there. What that broke, in
/// turn:
///
/// - `Cannot normalize zero-length vector`, the sole's normal at `v1`:
///   `NurbSurface::normal` now takes the corner's (`corner_normal`).
/// - `surface_could_contain (clipping): exhausted max_nodes=20000 with 885
///   patches pending`, locating `v2m` on `f8m` for its normal: about the
///   corner, a valley of `(u, v)` along the collinear direction maps
///   within the search's `1e-7` of the vertex, its point moving only to
///   second order, some 1500 boxes long. The tangent chain now takes a
///   face's normal at its vertex where its own loop puts it
///   (`Model::normal_at_vertex`).
/// - Still: `fits` asks the sole's curvature at the ball's foot on it at
///   `v1`, `(1.99984, 1.99984)` — a point of that valley — and the first
///   fundamental form there is singular to its width (`E = 0.27221`,
///   `F = -0.27250`, `G = 0.27279`): `normal_curvature` divides by
///   `EG - F^2`, which could be zero. Every differential quantity read
///   through these coordinates breaks down at that corner the same way;
///   what is needed is either the curvature (and the ball's placing) read
///   off the surface without inverting its metric, or the SubD B-rep not
///   putting a patch corner where its boundary runs on smoothly.
///
/// Ruled out: the radius (0.1 the same), the mirror plane (edge `e1-2`
/// touches it at neither end).
#[test]
#[ignore = "the sole's limit patch is singular at its corners v1 and v2 (S_u, S_v collinear where the crease runs on smoothly); its normal and the tangent chain now take that, but `fits` reads the sole's curvature there through its singular metric (EG - F^2 could be zero) — see the doc comment"]
fn subd_mouse_sole_rim_filleted() {
    let args = mouse();
    let mut model = NamedModel::new();
    build_subd(&mut model, "mouse", &args).unwrap();
    let rim = perimeter_of(&model, &["subd(mouse,f8)", "subd(mouse,f8m)"]);
    assert_eq!(rim.len(), 6, "{rim:?}");
    let part = apply::<Fillet>(model.clone(), "round", &FilletArgs::constant(rim, 0.05))
        .unwrap_or_else(|e| {
            let text = format!("{e:?}");
            panic!("{text}\n{:#?}", model.names_mentioned(&text))
        });
    assert_valid(&model_of(&part));
}

/// The SubD mouse with its top sliced off flat at `z = 0.5` by a block cut
/// away, and the rim of the flat top filleted by 0.05 — the way a tutorial
/// flattens a SubD body for a button and rounds the cut. The rim is the
/// two traced curves where the slice meets the hump, `combine(slice,
/// block(slice,start), subd(mouse,f9), ...)` and its mirror image.
///
/// Refused: `the blend strays inf from the rolling ball with 256 stations
/// per edge`, which is no diverged enclosure but the check that the blend
/// leaves its face towards the ball: in span 0 of the rim's chain, at 0.62
/// of it, the blend leaves `subd(mouse,f9)` at `-1.16e-5` rad, where it
/// should at `+1e-6` (`TILT`), at `(0.2811, 0.4333, 0.4979)`. That point is
/// at `(u, v) = (0.9963, 1.6313)` on `f9`, `0.0037` before its inner knot
/// line `u = 1`: a SubD face is four bicubic patches joined at triple
/// knots (see `geop-ops-subd`'s `brep`), and Loop and Schaefer's patches
/// are the limit surface only where every corner is regular, which no
/// corner of a box cage is: there they meet only nearly tangentially —
/// `f9`'s normal turns by `1.2e-6` rad across `u = 1`, more than the blend's
/// tilt (measured `1e-6` either side of it). No spline through the stations
/// follows the ball across that line, however many, and a station on it
/// (P13) would leave the tilt smaller than the crease it rolls over: a
/// crease inside a face, which a ball rolling over a crease between two
/// faces is refused for. Each of the two rim edges fails alike on its own,
/// and so does a radius of 0.02. Sliced lower, at `z = 0.3`, the rim is
/// refused by name instead (`the third face could run along the edge`).
#[test]
#[ignore = "the rim's contact on subd(mouse,f9) crosses the face's inner knot line u = 1, where its bicubic patches meet only nearly tangentially (the normal turns by 1.2e-6 rad, more than the blend's 1e-6 tilt): the blend dips through f9 by 1.16e-5 rad at any number of stations — see the doc comment"]
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
    let rounded = program.build_model(registry).unwrap_or_else(|e| {
        let text = format!("{e:?}");
        panic!("{text}\n{:#?}", part.names_mentioned(&text))
    });
    assert_valid(&rounded);
}
