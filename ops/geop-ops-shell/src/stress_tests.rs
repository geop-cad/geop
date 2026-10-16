//! Shells across boxes, cylinders and spheres, whole and with parts cut
//! off, added or drilled — and bodies with free-form edges, a boss on a
//! sphere and a pipe tee (see `geop_ops_extrude_revolve::testing::bodies`):
//! the solid shelled closed, and open at each of its faces in turn. The
//! blends of the same bodies are swept in `geop-ops-fillet`.
//!
//! The shell may refuse what it does not support — saying so — but what
//! it builds has to be a valid solid, and it must not fail otherwise. Each
//! body is one test, so they run side by side; each lists every case that
//! went wrong, not only the first.
//!
//! The bodies that take seconds run with every `cargo test`. The rest are
//! `#[ignore]`d, as their blends are: the full set runs with
//! `cargo test -- --ignored`, before changing the shell or the booleans
//! under it.

use geop_core_program::program::Program;
use geop_ops_extrude_revolve::testing::bodies::{
    block, block_corner_cut_off, block_drilled, block_pocketed, block_stepped, block_with_boss,
    cylinder, cylinder_drilled, cylinder_drilled_off_axis, cylinder_pocketed, cylinder_with_boss,
    cylinder_with_flat, hemisphere, one_solid, pipe_tee, sphere, sphere_drilled, sphere_with_boss,
    sphere_with_side_cut_off,
};
use geop_ops_model::{
    element, solid,
    testing::{Cases, outcome},
};

use crate::{Shell, ShellArgs};

/// Whether an error's root message is the shell refusing what it does not
/// support, rather than failing at what it does.
fn is_refusal(message: &str) -> bool {
    [
        "not supported",
        "only ",
        "too thick",
        "has to",
        "cannot end on a smooth surface",
        "shrinks the meridian",
    ]
    .iter()
    .any(|refusal| message.contains(refusal))
}

/// The solid `program` builds shelled closed, and open at each of its
/// faces; panics listing every case gone wrong. `size` is the shell's
/// thickness.
fn stress(program: Program, size: f64) {
    let (part, solid_name) = one_solid(&program);
    let model = part.topology();
    let mut faces: Vec<String> = (model.faces.keys())
        .map(|&id| part.name_of(id).unwrap().to_string())
        .collect();
    faces.sort();
    let mut cases = Cases::default();
    for open in std::iter::once(Vec::new()).chain(faces.into_iter().map(|f| vec![f])) {
        let args = ShellArgs {
            solid: solid(&solid_name),
            faces: open.into_iter().map(element).collect(),
            thickness: size.into(),
        };
        let open: Vec<&str> = args.faces.iter().map(|f| f.name()).collect();
        let case = format!("shell open at {open:?}");
        cases.record(case, outcome::<Shell>(&part, &args, is_refusal));
    }
    cases.assert_none_wrong();
}

#[test]
fn stress_block() {
    stress(block(), 0.1);
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
