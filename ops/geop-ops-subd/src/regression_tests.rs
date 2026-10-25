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

/// The mouse example (a box cage mirrored in `x = 0`, the rim of its sole
/// creased sharp) built: the sole is a flat face, `subd(mouse,f8)`, and at
/// its two corners away from the mirror plane, `subd(mouse,v1)` and `v2`,
/// the face's parametrization is singular — `|du x dv|` is `1.5e-14` where
/// `|du|` and `|dv|` are both `0.52`, the two derivatives collinear.
/// (At `v14` on the mirror plane it is not: `|du x dv| = 0.3`.)
///
/// The sole is flat, so the surface is fine; the coordinates are not (see
/// AGENTS.md, "Degenerate parametrizations"). Filleting the sole's rim,
/// as a tutorial rounds a mouse's bottom edge, fails on it: edges `e1-2`,
/// `e1-14` and `e14-1m` with `Cannot normalize zero-length vector`, the
/// others with `surface_could_contain (clipping): exhausted
/// max_nodes=20000 with 885 patches pending` — each edge on its own too
/// (see `geop-ops-fillet`'s `regression_tests`). The corner is where two
/// sharp crease edges meet; a crease vertex with two sharp edges is
/// smoothed along the crease, which is what collapses the corner.
#[test]
#[ignore = "bug: the limit patch of the mouse's sole is singular at its crease corners v1 and v2 (du x dv = 1.5e-14, |du| = |dv| = 0.52), and fillets of the sole's rim fail on it; not at v14 on the mirror plane"]
fn sole_of_the_mouse_has_a_normal_at_its_corners() {
    let (cage, mirror) = cage_of(examples::subd_mouse);
    let part = built("mouse", cage, mirror);
    let model = part.topology();
    let sole = part.face_id("subd(mouse,f8)").unwrap();
    let surface = &model.get_face(sole).unwrap().surface;
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let mut degenerate = Vec::new();
    for c in model.iterate_face_coedges(sole) {
        let vertex = model.coedge_start_vertex(c).unwrap();
        let p = vertex.point;
        // The foot of the vertex, seeded from the nearest domain corner.
        let (_, su, sv) = [u0, u1]
            .iter()
            .flat_map(|&u| [v0, v1].map(|v| (u, v)))
            .map(|(u, v)| {
                let q = surface.evaluate(u, v).unwrap();
                (q.sub(&p).norm().to_f64(), u, v)
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .unwrap();
        let (u, v) = surface.project(p, su, sv, 30).unwrap();
        let (du, dv) = surface.derivatives(u, v).unwrap();
        let area = du.prod_cross(&dv).norm();
        let scale = du.norm().mul(dv.norm());
        if !area.definitely_greater(scale.mul(Scalar::from_f64(1e-6))) {
            degenerate.push((
                part.name_of(model.coedge_start_vertex_id(c).unwrap())
                    .unwrap()
                    .to_string(),
                area.to_f64(),
                du.norm().to_f64(),
                dv.norm().to_f64(),
            ));
        }
    }
    assert!(
        degenerate.is_empty(),
        "the sole's parametrization is singular at (vertex, |du x dv|, |du|, |dv|): {degenerate:?}"
    );
}
