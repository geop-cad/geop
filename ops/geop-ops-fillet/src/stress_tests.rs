//! Fillets and chamfers across boxes, cylinders and spheres, whole and with
//! parts cut off, added or drilled — and bodies with free-form edges, a
//! boss on a sphere and a pipe tee (see
//! `geop_ops_extrude_revolve::testing::bodies`): every edge blended on its
//! own, and every corner's edges together. The shells of the same bodies
//! are swept in `geop-ops-shell`.
//!
//! An operation may refuse what it does not support — saying so — but what
//! it builds has to be a valid solid, and it must not fail otherwise. Each
//! body is one test, so they run side by side; each lists every case that
//! went wrong, not only the first.
//!
//! The bodies that take seconds run with every `cargo test`. The rest —
//! mostly those with circular edges, each blended by a revolved tool and a
//! boolean — take up to a minute each and are `#[ignore]`d: the full set
//! runs with `cargo test -- --ignored`, before changing a blend or the
//! booleans under it.

use geop_core_program::{operation::Operation, program::Program, values::Design};
use geop_core_topology::EdgeId;
use geop_ops_extrude_revolve::testing::{
    bodies::block, bodies::block_corner_cut_off, bodies::block_drilled, bodies::block_pocketed,
    bodies::block_stepped, bodies::block_with_boss, bodies::cylinder, bodies::cylinder_drilled,
    bodies::cylinder_drilled_off_axis, bodies::cylinder_pocketed, bodies::cylinder_with_boss,
    bodies::cylinder_with_flat, bodies::hemisphere, bodies::one_solid, bodies::pipe_tee,
    bodies::sphere, bodies::sphere_drilled, bodies::sphere_with_boss,
    bodies::sphere_with_side_cut_off,
};
use geop_ops_model::{
    NamedModel,
    testing::{Cases, Outcome, outcome},
};

use crate::{Chamfer, ChamferArgs, Fillet, FilletArgs};

/// Whether an error's root message is a blend refusing what it does not
/// support, rather than failing at what it does.
fn is_refusal(message: &str) -> bool {
    [
        "not supported",
        "only ",
        "too large",
        "has to",
        "is not planar",
        "no corner to blend",
        "does not stand square",
        "could run along the edge",
        "could meet tangentially",
        "reach across the axis",
        "across its axis",
        "exactly on another edge",
    ]
    .iter()
    .any(|refusal| message.contains(refusal))
}

/// Every edge of the solid `program` builds blended on its own, by fillet
/// and by chamfer; panics listing every case gone wrong. `size` is the
/// fillet's radius and the chamfer's distance.
fn stress(program: Program, size: f64) {
    let (part, _) = one_solid(&program);
    let model = part.topology();
    let name = |id: EdgeId| part.name_of(id).unwrap().to_string();
    let mut edges: Vec<String> = model.edges.keys().map(|&id| name(id)).collect();
    edges.sort();
    let groups = edges.into_iter().map(|e| (e.clone(), vec![e])).collect();
    blend_cases(&part, groups, size);
}

/// Each of `groups` — what a case is called, and its edges — blended
/// together, by fillet and by chamfer; panics listing every case gone
/// wrong. `size` is the fillet's radius and the chamfer's distance.
fn blend_cases(part: &NamedModel<Design>, groups: Vec<(String, Vec<String>)>, size: f64) {
    let mut cases = Cases::default();
    for (what, edges) in groups {
        let args = FilletArgs::constant(edges.clone(), size);
        cases.record(
            format!("fillet {what}"),
            outcome::<Fillet>(part, &args, is_refusal),
        );
        let args = ChamferArgs::even(edges, size);
        cases.record(
            format!("chamfer {what}"),
            outcome::<Chamfer>(part, &args, is_refusal),
        );
    }
    cases.assert_none_wrong();
}

/// The edges of the solid `program` builds around each of its vertices
/// blended together — the corner rounded where every edge there is
/// filleted — and every edge at once, by fillet and by chamfer; panics
/// listing every case gone wrong. `size` is the fillet's radius and the
/// chamfer's distance.
fn stress_corners(program: Program, size: f64) {
    let (part, _) = one_solid(&program);
    let model = part.topology();
    let name = |id: EdgeId| part.name_of(id).unwrap().to_string();
    let mut groups: Vec<(String, Vec<String>)> = Vec::new();
    for &vertex in model.vertices.keys() {
        let mut edges: Vec<String> = model
            .edges
            .iter()
            .filter(|(_, e)| e.start_vertex == vertex || e.end_vertex == vertex)
            .map(|(&id, _)| name(id))
            .collect();
        edges.sort();
        let at = format!("at vertex {}", part.name_of(vertex).unwrap());
        groups.push((at, edges));
    }
    groups.sort();
    let mut all: Vec<String> = model.edges.keys().map(|&id| name(id)).collect();
    all.sort();
    groups.push(("every edge".to_string(), all));
    blend_cases(&part, groups, size);
}

#[test]
fn stress_block() {
    stress(block(), 0.1);
}

#[test]
fn stress_block_corners() {
    stress_corners(block(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_corner_cut_off_corners() {
    stress_corners(block_corner_cut_off(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_pocketed_corners() {
    stress_corners(block_pocketed(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_stepped_corners() {
    stress_corners(block_stepped(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_with_flat_corners() {
    stress_corners(cylinder_with_flat(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_drilled() {
    stress(block_drilled(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_pocketed() {
    stress(block_pocketed(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_stepped() {
    stress(block_stepped(), 0.1);
}

#[test]
fn stress_block_corner_cut_off() {
    stress(block_corner_cut_off(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_block_with_boss() {
    stress(block_with_boss(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder() {
    stress(cylinder(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_drilled() {
    stress(cylinder_drilled(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_drilled_off_axis() {
    stress(cylinder_drilled_off_axis(), 0.1);
}

#[test]
fn stress_cylinder_with_flat() {
    stress(cylinder_with_flat(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_pocketed() {
    stress(cylinder_pocketed(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_cylinder_with_boss() {
    stress(cylinder_with_boss(), 0.1);
}

#[test]
fn stress_sphere() {
    stress(sphere(), 0.1);
}

#[test]
fn stress_hemisphere() {
    stress(hemisphere(), 0.1);
}

#[test]
fn stress_sphere_drilled() {
    stress(sphere_drilled(), 0.1);
}

#[test]
fn stress_sphere_with_side_cut_off() {
    stress(sphere_with_side_cut_off(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_sphere_with_boss() {
    stress(sphere_with_boss(), 0.1);
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_pipe_tee() {
    stress(pipe_tee(), 0.1);
}

/// Applies `operation` with `args` to the one solid `program` builds, and
/// checks it builds valid, saying why not.
fn assert_builds<O: Operation>(program: Program, args: O::Args) {
    let (part, _) = one_solid(&program);
    if let Outcome::Wrong(why) = outcome::<O>(&part, &args, is_refusal) {
        panic!("{why}");
    }
}

/// The hemisphere's rim — an arc of the circle where the flat meets the
/// sphere — bevelled: the chamfer rolls round the whole circle, its point
/// on the sphere moving across the seams of the sphere's quarters.
#[test]
fn chamfer_hemisphere_rim() {
    let program = hemisphere();
    let args = ChamferArgs::even(vec!["revolve(body,body_sketch,c2,a0)".into()], 0.1);
    assert_builds::<Chamfer>(program, args);
}

/// The cylinder's flat's upright side bevelled: a straight edge between the
/// flat and the cylinder, rolled.
#[test]
fn chamfer_cylinder_flat_side() {
    let program = cylinder_with_flat();
    let args = ChamferArgs::even(vec!["combine(flat,extrude(body,body_sketch,c1),extrude(flat,flat_sketch,c7),combine(flat,extrude(body,body_sketch,c1,end),extrude(flat,flat_sketch,c7,end),0,1),combine(flat,extrude(body,body_sketch,c1,start),extrude(flat,flat_sketch,c7,start),0,1))".into()], 0.1);
    assert_builds::<Chamfer>(program, args);
}
