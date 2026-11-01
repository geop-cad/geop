//! Fillets and chamfers across boxes, cylinders and spheres, whole and with
//! parts cut off, added or drilled — and bodies with free-form edges, a
//! boss on a sphere and a pipe tee (see `geop_lib_fixtures::bodies`):
//! every edge blended on its own, and every corner's edges together. The
//! shells of the same bodies are swept in `geop-ops-shell`.
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

use geop_core_math::scalars::Scalar;
use geop_core_program::{operation::Operation, program::Program};
use geop_core_topology::EdgeId;
use geop_lib_fixtures::bodies::{
    block, block_corner_cut_off, block_drilled, block_pocketed, block_stepped, block_with_boss,
    cylinder, cylinder_drilled, cylinder_drilled_off_axis, cylinder_pocketed, cylinder_with_boss,
    cylinder_with_flat, hemisphere, one_solid, pipe_tee, sphere, sphere_drilled, sphere_with_boss,
    sphere_with_side_cut_off,
};
use geop_ops_model::{
    NamedModel,
    testing::{Built, Cases, Outcome, edge_at, edges_where, middle, outcome},
};

use crate::{Chamfer, ChamferArgs, Fillet, FilletArgs};

/// Whether an error's root message is a blend refusing what it does not
/// support, rather than failing at what it does.
pub(crate) fn is_refusal(message: &str) -> bool {
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
fn blend_cases(part: &NamedModel, groups: Vec<(String, Vec<String>)>, size: f64) {
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

/// A thread that starts on the bottom rim of the cylinder it stands on and
/// runs on above its top; and one edge of each of its chains: the flanks'
/// corners on the wall, and clear of it below and above; the rims crossed by
/// them.
fn thread_chains() -> (NamedModel, Vec<String>) {
    use crate::regression_tests::{square_thread, thread_registry};
    let part = square_thread(1.5, 8.0)
        .build_model(thread_registry)
        .unwrap();
    let model = part.topology();
    let mut edges: Vec<EdgeId> = model.edges.keys().copied().collect();
    edges.sort_by_key(|e| e.0);
    let mut covered: Vec<EdgeId> = Vec::new();
    let mut names = Vec::new();
    for edge in edges {
        if covered.contains(&edge) || model.faces_of_edge(edge).is_none() {
            continue;
        }
        if let Ok(chain) = geop_ops_model::tangent::tangent_chain(&part, edge) {
            covered.extend(chain.edges());
        }
        names.push(part.name_of(edge).unwrap().to_string());
    }
    (part, names)
}

#[test]
#[ignore = "slow: part of the full stress set — run with `cargo test -- --ignored`"]
fn stress_square_thread_fillets() {
    let (part, chains) = thread_chains();
    let mut cases = Cases::default();
    for edge in chains {
        let args = FilletArgs::constant(vec![edge.clone()], 0.1);
        cases.record(
            format!("fillet {edge}"),
            outcome::<Fillet>(&part, &args, is_refusal),
        );
    }
    cases.assert_none_wrong();
}

/// The chamfers of the same chains. Eight of them fail, fast, and none is a
/// refusal saying why: `Scalar::div: divisor interval contains zero` where a
/// chord moves onto the next quarter (`chamfer_crossing`, five of them; its
/// Newton along the seam, by the look of it, from a seed at the chain's own
/// point, which is not yet shown); the blend
/// straying `0.1` from the chord over a span of one link (`sweep(thread,
/// section,p0,route,c4.q5)`, which rounds); the boolean's search exhausted on
/// the outer corner (`p2`); an invalid result at the start cap's edge.
#[test]
#[ignore = "bug: chamfers along a thread fail fast, without saying why, where the same edges round; see the comment"]
fn stress_square_thread_chamfers() {
    let (part, chains) = thread_chains();
    let mut cases = Cases::default();
    for edge in chains {
        let args = ChamferArgs::even(vec![edge.clone()], 0.1);
        cases.record(
            format!("chamfer {edge}"),
            outcome::<Chamfer>(&part, &args, is_refusal),
        );
    }
    cases.assert_none_wrong();
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

/// Applies `operation` with `args(part)` to the one solid `program`
/// builds, and checks it builds valid, saying why not.
fn assert_builds<O: Operation>(program: Program, args: impl Fn(&NamedModel) -> O::Args) {
    let (part, _) = one_solid(&program);
    if let Outcome::Wrong(why) = outcome::<O>(&part, &args(&part), is_refusal) {
        panic!("{why}");
    }
}

/// The hemisphere's rim — an arc of the circle where the flat meets the
/// sphere — bevelled: the chamfer rolls round the whole circle, its point
/// on the sphere moving across the seams of the sphere's quarters.
#[test]
fn chamfer_hemisphere_rim() {
    assert_builds::<Chamfer>(hemisphere(), |part| {
        let rim = edges_where(part, |e| middle(e)[2].could_be_equal(Scalar::ZERO));
        assert_eq!(rim.len(), 4, "{rim:?}");
        ChamferArgs::even(vec![rim[0].clone()], 0.1)
    });
}

/// The cylinder's flat's upright side bevelled: a straight edge between the
/// flat and the cylinder, rolled.
#[test]
fn chamfer_cylinder_flat_side() {
    assert_builds::<Chamfer>(cylinder_with_flat(), |part| {
        ChamferArgs::even(vec![edge_at(part, [0.6, -0.8, 0.75])], 0.1)
    });
}
